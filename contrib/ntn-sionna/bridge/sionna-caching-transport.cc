/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Roadmap §4.2.4)
 */
#include "sionna-caching-transport.h"

#include "ns3/double.h"
#include "ns3/integer.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SionnaCachingTransport");
NS_OBJECT_ENSURE_REGISTERED(SionnaCachingTransport);

TypeId
SionnaCachingTransport::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SionnaCachingTransport")
            .SetParent<SionnaTransport>()
            .SetGroupName("NtnSionna")
            .AddConstructor<SionnaCachingTransport>()
            .AddAttribute("SpatialResolutionM",
                          "Spatial grid edge length in meters.",
                          DoubleValue(100.0),
                          MakeDoubleAccessor(&SionnaCachingTransport::SetSpatialResolutionM,
                                             &SionnaCachingTransport::GetSpatialResolutionM),
                          MakeDoubleChecker<double>(1e-3, 1e7))
            .AddAttribute("TemporalBucketUs",
                          "Time-bucket size in microseconds.",
                          UintegerValue(1000),
                          MakeUintegerAccessor(&SionnaCachingTransport::SetTemporalBucketUs,
                                                &SionnaCachingTransport::GetTemporalBucketUs),
                          MakeUintegerChecker<uint64_t>(1, 60ULL * 60ULL * 1000000ULL))
            .AddAttribute("MaxEntries",
                          "Maximum cached entries.",
                          UintegerValue(4096),
                          MakeUintegerAccessor(&SionnaCachingTransport::SetMaxEntries,
                                                &SionnaCachingTransport::GetMaxEntries),
                          MakeUintegerChecker<uint32_t>(1, 1u << 24));
    return tid;
}

SionnaCachingTransport::SionnaCachingTransport()
    : m_hits(0),
      m_misses(0),
      m_evictions(0)
{
}

SionnaCachingTransport::~SionnaCachingTransport()
{
    Reset();
}

void
SionnaCachingTransport::SetSpatialResolutionM(double res_m)
{
    m_spatialResM = std::max(1e-3, res_m);
}

void
SionnaCachingTransport::SetTemporalBucketUs(uint64_t us)
{
    m_temporalBucketUs = std::max<uint64_t>(1, us);
}

void
SionnaCachingTransport::SetMaxEntries(uint32_t maxEntries)
{
    m_maxEntries = std::max<uint32_t>(1, maxEntries);
}

uint64_t
SionnaCachingTransport::GetEntries() const
{
    std::lock_guard<std::mutex> lock(m_mu);
    return m_cache.size();
}

double
SionnaCachingTransport::GetHitRate() const
{
    const uint64_t h = m_hits.load();
    const uint64_t m = m_misses.load();
    const uint64_t total = h + m;
    return total ? static_cast<double>(h) / total : 0.0;
}

void
SionnaCachingTransport::Reset()
{
    std::lock_guard<std::mutex> lock(m_mu);
    m_cache.clear();
    m_lru.clear();
    m_hits.store(0);
    m_misses.store(0);
    m_evictions.store(0);
}

namespace
{

/// SIONNA-04 helper: fold one value into a running 64-bit digest.
///
/// This is the same xor/shift combine the key hash uses, kept separate so the
/// digest is stable and does not depend on std::hash's implementation-defined
/// behaviour across runs.
inline uint64_t
CfgMix(uint64_t h, uint64_t v)
{
    h ^= v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    return h;
}

inline uint64_t
CfgMixDouble(uint64_t h, double v)
{
    // Quantize before hashing: a spacing of 0.5 must digest identically however
    // it was computed, and bit-identical doubles are not guaranteed across
    // arithmetic paths. 1e-6 is far finer than any physically meaningful
    // difference in wavelengths or metres.
    return CfgMix(h, static_cast<uint64_t>(static_cast<int64_t>(std::llround(v * 1e6))));
}

inline uint64_t
CfgMixString(uint64_t h, const std::string& s)
{
    for (unsigned char c : s)
    {
        h = CfgMix(h, static_cast<uint64_t>(c));
    }
    return CfgMix(h, s.size());
}

inline uint64_t
CfgMixArray(uint64_t h, const std::optional<MimoArrayConfig>& a)
{
    if (!a.has_value())
    {
        return CfgMix(h, 0x4e4f4e45ULL); // "NONE": absent is its own state
    }
    h = CfgMix(h, a->rows);
    h = CfgMix(h, a->cols);
    h = CfgMixDouble(h, a->spacing_lambda);
    h = CfgMixString(h, a->pattern);
    h = CfgMixString(h, a->polarization);
    return h;
}

inline uint64_t
CfgMixRis(uint64_t h, const std::optional<RisConfig>& r)
{
    if (!r.has_value())
    {
        return CfgMix(h, 0x4e4f524953ULL); // "NORIS"
    }
    h = CfgMixDouble(h, r->pos_x);
    h = CfgMixDouble(h, r->pos_y);
    h = CfgMixDouble(h, r->pos_z);
    h = CfgMixDouble(h, r->normal_x);
    h = CfgMixDouble(h, r->normal_y);
    h = CfgMixDouble(h, r->normal_z);
    h = CfgMix(h, r->rows);
    h = CfgMix(h, r->cols);
    h = CfgMixDouble(h, r->spacing_lambda);
    h = CfgMixString(h, r->phase_profile);
    h = CfgMixDouble(h, r->focal_x);
    h = CfgMixDouble(h, r->focal_y);
    h = CfgMixDouble(h, r->focal_z);
    return h;
}

} // namespace

SionnaCachingTransport::Key
SionnaCachingTransport::MakeKey(const Request& req) const
{
    const double inv = 1.0 / m_spatialResM;
    Key k;
    k.txGx = static_cast<int64_t>(std::floor(req.tx_x * inv));
    k.txGy = static_cast<int64_t>(std::floor(req.tx_y * inv));
    k.txGz = static_cast<int64_t>(std::floor(req.tx_z * inv));
    k.rxGx = static_cast<int64_t>(std::floor(req.rx_x * inv));
    k.rxGy = static_cast<int64_t>(std::floor(req.rx_y * inv));
    k.rxGz = static_cast<int64_t>(std::floor(req.rx_z * inv));
    k.freqHz = static_cast<uint64_t>(std::lround(req.freq_hz));
    const uint64_t now_us =
        static_cast<uint64_t>(Simulator::Now().GetMicroSeconds());
    k.tBucket = now_us / m_temporalBucketUs;
    // SIONNA-04: everything else the server reads off the Request. los_only is
    // included because it selects max_depth and switches reflection,
    // diffraction and scattering on or off, so it changes the answer at least
    // as much as the arrays do.
    uint64_t cfg = 0;
    cfg = CfgMix(cfg, req.los_only ? 1u : 0u);
    cfg = CfgMixArray(cfg, req.tx_array);
    cfg = CfgMixArray(cfg, req.rx_array);
    cfg = CfgMixRis(cfg, req.ris);
    k.cfg = cfg;
    return k;
}

void
SionnaCachingTransport::EvictOneLocked() const
{
    if (m_lru.empty())
    {
        return;
    }
    const Key& oldest = m_lru.back();
    m_cache.erase(oldest);
    m_lru.pop_back();
    m_evictions.fetch_add(1);
}

SionnaTransport::Response
SionnaCachingTransport::Query(const Request& req) const
{
    Key k = MakeKey(req);
    {
        std::lock_guard<std::mutex> lock(m_mu);
        auto it = m_cache.find(k);
        if (it != m_cache.end())
        {
            // Hit: move to front of LRU.
            m_lru.splice(m_lru.begin(), m_lru, it->second.second);
            it->second.second = m_lru.begin();
            ++m_queriesSent;
            m_hits.fetch_add(1);
            Response r = it->second.first;
            r.compute_ms = 0.0; // signal cached
            return r;
        }
    }

    // Miss: forward to inner, record stats. Counters on the SionnaTransport
    // base (m_queriesSent, m_timeouts, m_failures, m_lastRttMs) update from
    // the inner transport, so we mirror queriesSent here to stay consistent
    // with the base contract.
    Response r;
    if (m_inner)
    {
        r = m_inner->Query(req);
        m_lastRttMs.store(m_inner->GetLastRttMs());
    }
    m_misses.fetch_add(1);
    ++m_queriesSent;
    if (!r.ok || !std::isfinite(r.path_loss_db))
    {
        ++m_failures;
        return r;
    }
    // Insert into cache.
    {
        std::lock_guard<std::mutex> lock(m_mu);
        m_lru.push_front(k);
        m_cache.emplace(k, CacheEntry{r, m_lru.begin()});
        while (m_cache.size() > m_maxEntries)
        {
            EvictOneLocked();
        }
    }
    return r;
}

} // namespace ns3
