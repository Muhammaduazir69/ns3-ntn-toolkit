/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 */
#ifndef NTN_SLICE_QUOTA_SCHEDULER_H
#define NTN_SLICE_QUOTA_SCHEDULER_H

#include "ns3/nr-mac-scheduler-ofdma-pf.h"
#include "ns3/traced-callback.h"

#include <map>
#include <string>
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
    /**
     * Check a quota vector: every share finite and in [0, 1], and the shares summing to at most 1 (the TS 28.541
     * RRMPolicyRatio rule that dedicated ratios add up to at most 100 %). An empty vector is valid (no slicing).
     * \param shares the per-slice shares of the RBGs of a slot
     * \param why if not null, receives the reason a vector is refused
     * \return true if the vector may be applied
     */
    static bool ValidateQuotas(const std::vector<double>& shares, std::string* why = nullptr);
    /// Set the per-slice quota as shares of the RBGs of a slot; aborts on a vector ValidateQuotas refuses.
    void SetQuotas(const std::vector<double>& shares);
    /// As SetQuotas, but an invalid vector is refused (false) and the quotas in force stay unchanged.
    bool TrySetQuotas(const std::vector<double>& shares);
    std::vector<double> GetQuotas() const;
    /// RBGs given to each slice since the last call, and the number of slots counted; resets the counters.
    /// It does not reset the window of TakeBytesPerRbg, so the two may be called in either order.
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
    mutable std::vector<uint64_t> m_tbRbgCount; ///< RBGs per slice in the TakeBytesPerRbg window
    TracedCallback<uint32_t, uint32_t, uint32_t> m_sliceAlloc; ///< slice, RBGs this slot, RBGs in the slot
};

} // namespace ntnslice
} // namespace ns3

#endif
