/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W6)
 *
 * Slice orchestrator xApp.
 *
 * This is the C++ counterpart of `SliceEnv` in W4 (`ns3_ai_ntn.envs.slice`):
 * the observation and action shapes match so an RL policy trained in
 * Gymnasium can drive the C++ allocator without re-encoding the API.
 *
 * Algorithmic role:
 *   - Inputs:   per-slice demand (Mbps), reserved minimum, priority.
 *   - Action:   PRB share for each slice on a single cell.
 *   - Output:   allocated PRBs, served Mbps, satisfaction ratio.
 *
 * The default policy is **min-throughput-first proportional fair**:
 *   1. Reserve enough PRBs to meet each slice's minThroughputMbps.
 *   2. Distribute remaining PRBs proportionally to (priority · demand).
 *
 * RL policies override step 2 by calling @c StepWithShares(shares).
 */
#ifndef NTN_SLICE_ORCHESTRATOR_XAPP_H
#define NTN_SLICE_ORCHESTRATOR_XAPP_H

#include "ntn-slice-types.h"

#include "ns3/object.h"
#include "ns3/traced-callback.h"

#include <map>
#include <vector>

namespace ns3
{
namespace ntnslice
{

struct SliceTick
{
    Snssai snssai;
    double demandMbps{0.0};
    double servedMbps{0.0};
    uint32_t prbAllocated{0};
    double satisfactionRatio{0.0}; ///< served / demand (clamped 0..1)
    bool minThroughputMet{true};
};

class SliceOrchestratorXapp : public Object
{
  public:
    static TypeId GetTypeId();
    SliceOrchestratorXapp();

    /// Add a slice with its profile to the orchestrator.
    void RegisterSlice(const SliceProfile& profile);

    /// Total cell PRB budget (NR FR1 100 MHz / 30 kHz SCS = 273 PRBs by default).
    void SetTotalPrb(uint32_t prb);
    uint32_t GetTotalPrb() const;

    /// Capacity of one PRB (Mbps). MCS-16 average ≈ 0.6 Mbps/PRB at 30 kHz SCS.
    void SetPrbCapacityMbps(double mbpsPerPrb);

    /// Update per-slice demand (Mbps). Slices missing → demand 0.
    void SetDemand(const Snssai& s, double mbps);
    double GetDemand(const Snssai& s) const;

    /// Run one allocation tick using the built-in proportional-fair policy.
    /// Result is the per-slice service vector for this tick.
    std::vector<SliceTick> Step();

    /// Run one allocation tick using externally-supplied PRB shares (RL hook).
    /// `shares` need not sum to 1 — they are renormalised internally.
    std::vector<SliceTick> StepWithShares(const std::map<Snssai, double>& shares);

    /// Aggregate satisfaction ratio across slices for the most recent tick.
    double GetAggregateSatisfaction() const;

    /// Number of slices currently registered.
    std::size_t SliceCount() const;

    typedef TracedCallback<const SliceTick&> SliceTickTrace;

  private:
    std::vector<SliceTick> Allocate(const std::map<Snssai, double>& sharesOpt,
                                    bool useShares);

    std::vector<SliceProfile> m_slices;
    std::map<Snssai, double> m_demand;
    uint32_t m_totalPrb{273};
    double m_mbpsPerPrb{0.6};
    double m_lastAggregateSat{0.0};
    SliceTickTrace m_traceTick;
};

} // namespace ntnslice
} // namespace ns3

#endif // NTN_SLICE_ORCHESTRATOR_XAPP_H
