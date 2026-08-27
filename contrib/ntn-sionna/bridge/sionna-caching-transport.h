/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Roadmap §4.2.4)
 *
 * sionna-caching-transport — 4-D LRU cache decorator on top of any
 * SionnaTransport implementation. Keys are
 *
 *   (tx_grid_cell, rx_grid_cell, freq_hz, time_bucket)
 *
 * with spatial quantisation by `SpatialResolutionM` (default 100 m) and
 * temporal quantisation by `TemporalBucketUs` (default 1000 µs = 1 ms).
 * Bounded by `MaxEntries` (default 4096); LRU eviction on overflow.
 *
 * Two grid cells coincide iff they fall in the same quantisation cube and
 * fall in the same time bucket. The cache is purely deterministic — no
 * interpolation or stochastic prefetch — and is safe to use under
 * Simulator::Run() because all access is synchronous (no background
 * thread). The underlying ratio of hits to misses depends on the
 * scenario's spatial locality: a static UE under a sweeping LEO sees
 * ~0 hits, while a digital-twin replay loop with 100 ms cadence sees
 * deep cache reuse.
 */
#ifndef SIONNA_CACHING_TRANSPORT_H
#define SIONNA_CACHING_TRANSPORT_H

#include "sionna-transport.h"

#include <atomic>
#include <cstdint>
#include <list>
#include <mutex>
#include <unordered_map>

namespace ns3
{

/**
 * \ingroup ntn-sionna
 *
 * \brief LRU caching decorator for SionnaTransport.
 *
 * Stores at most `MaxEntries` past responses keyed by quantised
 * (tx, rx, freq, time). On a hit, returns the cached response with
 * `compute_ms` cleared to 0 so callers can distinguish a cached value
 * from a live one. On a miss, forwards the request to the inner
 * transport, and on a successful inner response, inserts the response
 * into the cache and evicts the LRU entry if necessary.
 */
class SionnaCachingTransport : public SionnaTransport
{
  public:
    static TypeId GetTypeId();

    SionnaCachingTransport();
    ~SionnaCachingTransport() override;

    /// Set the underlying transport whose responses are cached. Pass
    /// nullptr to disable forwarding (every miss returns the not-ok
    /// sentinel Response).
    void SetInner(Ptr<SionnaTransport> inner) { m_inner = inner; }
    Ptr<SionnaTransport> GetInner() const { return m_inner; }

    /// Spatial grid size in meters along each axis. Smaller = finer
    /// cache granularity = more misses but more accurate. Default: 100 m.
    void SetSpatialResolutionM(double res_m);
    double GetSpatialResolutionM() const { return m_spatialResM; }

    /// Time-bucket size in microseconds. Two queries fall in the same
    /// bucket if floor(now_us / bucket_us) is the same. Default: 1000 µs.
    void SetTemporalBucketUs(uint64_t us);
    uint64_t GetTemporalBucketUs() const { return m_temporalBucketUs; }

    /// Max entries kept in the cache. LRU eviction on overflow.
    /// Default: 4096.
    void SetMaxEntries(uint32_t maxEntries);
    uint32_t GetMaxEntries() const { return m_maxEntries; }

    // Stats
    uint64_t GetHits() const { return m_hits.load(); }
    uint64_t GetMisses() const { return m_misses.load(); }
    uint64_t GetEvictions() const { return m_evictions.load(); }
    uint64_t GetEntries() const;
    /// Hit rate over all queries seen so far (0 if no queries).
    double GetHitRate() const;
    /// Reset stats and clear the cache.
    void Reset();

    // SionnaTransport overrides — base virtuals are const-qualified.
    Response Query(const Request& req) const override;
    bool IsAvailable() const override
    {
        return m_inner ? m_inner->IsAvailable() : false;
    }
    std::string Name() const override { return "caching"; }

  private:
    struct Key
    {
        int64_t txGx;
        int64_t txGy;
        int64_t txGz;
        int64_t rxGx;
        int64_t rxGy;
        int64_t rxGz;
        uint64_t freqHz;
        uint64_t tBucket;
        /// SIONNA-04: every non-geometric input that changes the server's
        /// answer, folded into one digest.
        ///
        /// The key used to be geometry, frequency and time only. The server
        /// rebuilds the PlanarArray and installs or removes the RIS per query
        /// (sionna-server.py), and honours los_only, so two requests that agree
        /// on position and differ on antenna or surface configuration get
        /// genuinely different answers. Hashing only the first set meant a
        /// single transport shared between a SISO and an 8x8 run, or between
        /// RIS-on and RIS-off, returned whichever was asked first within the
        /// same 100 m cell and 1 ms bucket. That silently turns an A/B
        /// comparison into a constant, which is worse than a slow cache.
        uint64_t cfg;

        bool operator==(const Key& o) const
        {
            return txGx == o.txGx && txGy == o.txGy && txGz == o.txGz &&
                   rxGx == o.rxGx && rxGy == o.rxGy && rxGz == o.rxGz &&
                   freqHz == o.freqHz && tBucket == o.tBucket && cfg == o.cfg;
        }
    };

    struct KeyHash
    {
        size_t operator()(const Key& k) const
        {
            // Hash combine via xor + shift on 64-bit grid coords.
            auto mix = [](size_t s, uint64_t v) {
                return s ^ (std::hash<uint64_t>{}(v) + 0x9e3779b97f4a7c15ULL +
                            (s << 6) + (s >> 2));
            };
            size_t h = 0;
            h = mix(h, static_cast<uint64_t>(k.txGx));
            h = mix(h, static_cast<uint64_t>(k.txGy));
            h = mix(h, static_cast<uint64_t>(k.txGz));
            h = mix(h, static_cast<uint64_t>(k.rxGx));
            h = mix(h, static_cast<uint64_t>(k.rxGy));
            h = mix(h, static_cast<uint64_t>(k.rxGz));
            h = mix(h, k.freqHz);
            h = mix(h, k.tBucket);
            h = mix(h, k.cfg);
            return h;
        }
    };

    using LruList = std::list<Key>;
    using CacheEntry = std::pair<Response, LruList::iterator>;
    using CacheMap = std::unordered_map<Key, CacheEntry, KeyHash>;

    Key MakeKey(const Request& req) const;
    /// Take the LRU lock and evict the back of the list. Caller must hold lock.
    void EvictOneLocked() const;

    Ptr<SionnaTransport> m_inner;
    double m_spatialResM{100.0};
    uint64_t m_temporalBucketUs{1000};
    uint32_t m_maxEntries{4096};

    mutable std::mutex m_mu;
    mutable LruList m_lru;
    mutable CacheMap m_cache;

    mutable std::atomic<uint64_t> m_hits;
    mutable std::atomic<uint64_t> m_misses;
    mutable std::atomic<uint64_t> m_evictions;
};

} // namespace ns3

#endif // SIONNA_CACHING_TRANSPORT_H
