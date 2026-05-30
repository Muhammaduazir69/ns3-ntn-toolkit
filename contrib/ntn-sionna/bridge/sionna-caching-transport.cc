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
