/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W6)
 */
#ifndef NTN_SLICE_HELPER_H
#define NTN_SLICE_HELPER_H

#include "ns3/ntn-slice-selector.h"
#include "ns3/ntn-slice-types.h"
#include "ns3/ptr.h"
#include "ns3/slice-isolation-monitor.h"
#include "ns3/slice-orchestrator-xapp.h"

namespace ns3
{
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

/// Convenience: classify a satellite altitude as GEO if above 35 000 km.
inline bool
IsGeoSatellite(double altitudeKm)
{
    return altitudeKm >= 30000.0;
}

} // namespace ntnslice
} // namespace ns3

#endif // NTN_SLICE_HELPER_H
