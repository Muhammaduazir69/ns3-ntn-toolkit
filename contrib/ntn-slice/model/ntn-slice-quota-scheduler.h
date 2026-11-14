/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 */
#ifndef NTN_SLICE_QUOTA_SCHEDULER_H
#define NTN_SLICE_QUOTA_SCHEDULER_H

#include "ns3/nr-mac-scheduler-ofdma-pf.h"
#include "ns3/traced-callback.h"

#include <map>
#include <vector>

namespace ns3
{
namespace ntnslice
{

/**
 * \brief OFDMA proportional-fair NR scheduler that enforces per-slice RBG quotas in every slot.
 *
 * Each UE (by RNTI) belongs to one slice. In every downlink slot, each RBG goes to the UE ranked best by
 * the proportional-fair metric among the UEs with unmet demand whose slice is still below its quota; only
 * when no such UE exists may a slice exceed its quota (work conservation). A slice with demand therefore
 * gets at least floor(quota * RBGs) RBGs per slot, whatever the other slices want, and RBGs no slice under
 * quota can use are not wasted. Quotas can be changed at any time (SetQuotas), which is how an orchestrator
 * closes the loop. The uplink is scheduled by the parent PF scheduler unchanged.
 */
class NtnSliceQuotaScheduler : public NrMacSchedulerOfdmaPF
{
  public:
    static TypeId GetTypeId();
    NtnSliceQuotaScheduler();
    ~NtnSliceQuotaScheduler() override = default;

    /// Assign a UE to a slice (slice indices from 0); unknown RNTIs count as slice 0.
    void SetSliceOfRnti(uint16_t rnti, uint32_t slice);
    /// Set the per-slice quota as shares of the RBGs of a slot; the shares must sum to at most 1.
    void SetQuotas(const std::vector<double>& shares);
    std::vector<double> GetQuotas() const;
    /// RBGs given to each slice since the last call, and the number of slots counted; resets the counters.
    std::vector<uint64_t> TakeRbgCounts(uint64_t* slots);
    uint32_t GetSliceOfRnti(uint16_t rnti) const;
    /// Mean downlink backlog (bytes) per slice over the slots since the last call, and the bytes delivered per
    /// RBG to each slice (the slice's measured capacity per RBG); resets the counters.
    std::vector<double> TakeBacklog();
    std::vector<double> TakeBytesPerRbg();

  protected:
    BeamSymbolMap AssignDLRBG(uint32_t symAvail, const ActiveUeMap& activeDl) const override;

  private:
    std::map<uint16_t, uint32_t> m_sliceOfRnti;
    std::vector<double> m_quota;
    bool m_workConserving{true};
    mutable std::vector<uint64_t> m_rbgCount;
    mutable uint64_t m_slots{0};
    mutable std::vector<double> m_backlogSum;   ///< summed buffer requests per slice
    mutable uint64_t m_backlogSlots{0};
    mutable std::vector<double> m_tbBytes;      ///< TB bytes assigned per slice
    TracedCallback<uint32_t, uint32_t, uint32_t> m_sliceAlloc; ///< slice, RBGs this slot, RBGs in the slot
};

} // namespace ntnslice
} // namespace ns3

#endif
