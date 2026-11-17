/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W6)
 */
#ifndef NTN_SLICE_HELPER_H
#define NTN_SLICE_HELPER_H

#include "ns3/ntn-slice-selector.h"
#include "ns3/ntn-slice-types.h"
#include "ns3/mobility-model.h"
#include "ns3/ptr.h"
#include "ns3/slice-isolation-monitor.h"
#include "ns3/slice-orchestrator-xapp.h"

namespace ns3
{
class NtnRealStackHelper;

namespace ntnslice
{

/**
 * @brief One-call factory for the canonical eMBB / URLLC / mMTC stack.
 *
 *   auto h = NtnSliceHelper::ThreeSliceDefault();
 *   // h.selector + h.orchestrator + h.monitor are pre-wired.
 */
struct NtnSliceStack
{
    Ptr<NtnSliceSelector> selector;
    Ptr<SliceOrchestratorXapp> orchestrator;
    Ptr<SliceIsolationMonitor> monitor;
    SliceProfile embb;
    SliceProfile urllc;
    SliceProfile mmtc;
};

class NtnSliceHelper
{
  public:
    /// Build the three canonical slices with default profiles per TS 22.261.
    static NtnSliceStack ThreeSliceDefault();

    /**
     * \brief Link budget of the shared slice cell (Wave 7C).
     *
     * The slice examples used to declare 70 dBm (or 55 dBm) CONDUCTED at the
     * satellite array input on the nr backend. With the 8x8 UPA gain on top
     * that radiated about 83 to 88 dBm into a TR 38.811 handheld, 6 to 11 dB
     * above the TR 38.821 Table 6.1.1.1-1 Set-1 S-band payload (34 dBW/MHz, 77
     * dBm over 20 MHz). Handheld SINR then sat above 30 dB, TBLER near 0 and
     * no packet was ever lost.
     *
     * \p satPowerDbm < 0 on nr: TR 38.821 Set-1 terminal and payload
     * (NtnRealStackHelper::ApplyTr38821Set1Defaults; handheld 1T2R 0 dBi,
     * NF 7 dB, 23 dBm; 34 dBW/MHz on S band). On mmwave (no TR 38.821 set for
     * the FR2 bridge) a negative value keeps the backend's 55 dBm conducted.
     * Any value >= 0 is the conducted power at the array input, as before.
     * Call after the carrier and bandwidth are set and before Build().
     * \return true when the Set-1 budget was applied.
     */
    static bool ConfigureCellLinkBudget(NtnRealStackHelper& rs, double satPowerDbm, bool mmwave);

    /**
     * \brief Feeder link of the slice cell (Wave 7C).
     *
     * The examples carried a constant 5 ms "feeder + core" backhaul, so the
     * one-way delay did not follow the satellite and the jitter came from slot
     * timing alone. With \p gwDistanceKm > 0 a WGS-84 gateway is placed that
     * far north of (\p subLatDeg, \p subLonDeg) and the backhaul follows the
     * live satellite-to-gateway slant range (plus the folded service link,
     * NtnRealStackHelper::SetFeederGateway). \p gwDistanceKm <= 0 keeps the
     * constant \p backhaulMs. Call after Build().
     * \return the gateway mobility, or null when the constant backhaul is kept.
     */
    static Ptr<MobilityModel> ConfigureFeeder(NtnRealStackHelper& rs,
                                              double subLatDeg,
                                              double subLonDeg,
                                              double gwDistanceKm);

    /// Lower bound of the one-way delay (ms) of the slice cell: altitude / c
    /// for the service link plus altitude / c for a feeder (gateway case) or
    /// the constant backhaul.
    static double OneWayFloorMs(double altitudeKm, bool liveFeeder, double backhaulMs);
};

/**
 * @brief Helper used by routing/scheduling code to enforce GEO mode-skip.
 *
 * Per TS 22.261, URLLC slices have a 5 ms latency budget which a GEO link
 * (~250 ms one-way) cannot meet. When `IsGeoSatellite(...)` is true and the
 * slice is URLLC, the route MUST skip the GEO hop and prefer LEO.
 */
inline bool
ShouldSkipGeo(const SliceProfile& slice)
{
    return slice.snssai.sst == SliceSst::Urllc || !slice.allowGeo;
}

/// Convenience: classify a satellite as GEO-class. Altitudes at or above the
/// conservative 30 000 km boundary are treated as GEO; GEO proper is 35 786 km.
inline bool
IsGeoSatellite(double altitudeKm)
{
    return altitudeKm >= 30000.0;
}

} // namespace ntnslice
} // namespace ns3

#endif // NTN_SLICE_HELPER_H
