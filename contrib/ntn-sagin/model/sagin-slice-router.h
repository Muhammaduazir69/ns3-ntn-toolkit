/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Roadmap §4.4.7)
 *
 * sagin-slice-router — cross-layer slice-aware routing on top of the
 * MultiLayerRouter (W5). Maps a 5G QoS flow (QFI) to a slice profile
 * (S-NSSAI + KPIs), then biases the Ground→UAV→HAPS→LEO layer choice so
 * that:
 *
 *   - URLLC (low latency budget, allowGeo=false) prefers low layers and
 *     refuses any hop whose cumulative one-way propagation latency would
 *     blow the slice's latencyBudgetMs (LEO ≈ 1.8 ms one-way, GEO ≈ 119 ms).
 *   - eMBB (high throughput, latency-tolerant) is free to climb to HAPS /
 *     LEO for capacity.
 *   - mMTC accepts any layer including GEO-equivalent ranges.
 *
 * The slice constraint is realised through the §4.4.10 scoring callback:
 * `RouteForSlice` installs a slice-derived scorer, runs one Route(), and
 * restores the prior scorer. Feasibility is enforced by handing infeasible
 * candidates a −inf score so the router never selects them; if a whole
 * layer is infeasible the route simply stops at the previous hop.
 */
#ifndef NTN_SAGIN_SLICE_ROUTER_H
#define NTN_SAGIN_SLICE_ROUTER_H

#include "multi-layer-router.h"

#include "ns3/ntn-slice-types.h"
#include "ns3/object.h"

#include <cstdint>
#include <map>
#include <vector>

namespace ns3
{

/**
 * \ingroup ntn-sagin
 *
 * \brief Slice-aware wrapper around MultiLayerRouter (Roadmap §4.4.7).
 */
class SaginSliceRouter : public Object
{
  public:
    static TypeId GetTypeId();

    SaginSliceRouter();
    ~SaginSliceRouter() override;

    void SetRouter(Ptr<MultiLayerRouter> router) { m_router = router; }
    Ptr<MultiLayerRouter> GetRouter() const { return m_router; }

    /// Register a slice profile under its packed S-NSSAI key.
    void AddSlice(const ntnslice::SliceProfile& profile);

    /// Map a 5G QFI (0..63) to a registered slice. The QFI→S-NSSAI map is
    /// configurable via SetQfiSlice; defaults follow the common 5QI bands:
    ///   QFI 1..4  → URLLC   (GBR, delay-critical)
    ///   QFI 5..9  → eMBB    (non-GBR broadband)
    ///   QFI 10..63 → mMTC   (background / IoT)
    void SetQfiSlice(uint8_t qfi, const ntnslice::Snssai& snssai);
    ntnslice::Snssai QfiToSnssai(uint8_t qfi) const;

    /// Look up a registered slice profile by S-NSSAI. Returns a default
    /// eMBB profile if the slice is unknown.
    ntnslice::SliceProfile GetProfile(const ntnslice::Snssai& s) const;

    /// Route honouring the slice profile bound to `snssai`.
    std::vector<SaginHop> RouteForSlice(Ptr<MobilityModel> source,
                                        const ntnslice::Snssai& snssai);

    /// Route honouring the slice mapped from `qfi`.
    std::vector<SaginHop> RouteForQfi(Ptr<MobilityModel> source, uint8_t qfi);

    /// One-way propagation latency (ms) for a given slant range in metres.
    static double OneWayLatencyMs(double rangeM);

    /// Altitude (m) above which a hop is treated as "GEO-equivalent" for
    /// the allowGeo gate. Default 30 000 km is comfortably above any LEO.
    void SetGeoAltitudeThresholdM(double m) { m_geoAltThresholdM = m; }
    double GetGeoAltitudeThresholdM() const { return m_geoAltThresholdM; }

  private:
    /// Build a scorer that enforces the slice's latency + GEO constraints
    /// and otherwise prefers max elevation.
    SaginScoreCallback MakeSliceScorer(
        const ntnslice::SliceProfile& profile) const;

    Ptr<MultiLayerRouter> m_router;
    std::map<uint32_t, ntnslice::SliceProfile> m_slices;
    std::map<uint8_t, ntnslice::Snssai> m_qfiMap;
    double m_geoAltThresholdM{30000000.0};
};

} // namespace ns3

#endif // NTN_SAGIN_SLICE_ROUTER_H
