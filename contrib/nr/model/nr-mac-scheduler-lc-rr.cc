// Copyright (c) 2023 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-mac-scheduler-lc-rr.h"

#include "ns3/log.h"

#include <algorithm>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrMacSchedulerLcRR");
NS_OBJECT_ENSURE_REGISTERED(NrMacSchedulerLcRR);

NrMacSchedulerLcRR::NrMacSchedulerLcRR()
    : NrMacSchedulerLcAlgorithm()
{
    NS_LOG_FUNCTION(this);
}

NrMacSchedulerLcRR::~NrMacSchedulerLcRR()
{
}

TypeId
NrMacSchedulerLcRR::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrMacSchedulerLcRR")
                            .SetParent<NrMacSchedulerLcAlgorithm>()
                            .AddConstructor<NrMacSchedulerLcRR>();
    return tid;
}

std::vector<NrMacSchedulerLcAlgorithm::Assignation>
NrMacSchedulerLcRR::DoAssignBytesToDlLC(const std::unordered_map<uint8_t, LCGPtr>& ueLCG,
                                        uint32_t tbs,
                                        [[maybe_unused]] Time slotPeriod) const
{
    return AssignBytesToLC(ueLCG, tbs, true);
}

std::vector<NrMacSchedulerLcAlgorithm::Assignation>
NrMacSchedulerLcRR::DoAssignBytesToUlLC(const std::unordered_map<uint8_t, LCGPtr>& ueLCG,
                                        uint32_t tbs) const
{
    return AssignBytesToLC(ueLCG, tbs, false);
}

std::vector<NrMacSchedulerLcAlgorithm::Assignation>
NrMacSchedulerLcRR::AssignBytesToLC(const std::unordered_map<uint8_t, LCGPtr>& ueLCG,
                                    uint32_t tbs,
                                    bool isDl) const
{
    NS_LOG_FUNCTION(this);

    std::vector<NrMacSchedulerLcAlgorithm::Assignation> ret;

    NS_LOG_INFO("To distribute: " << tbs << " bytes over " << ueLCG.size() << " LCG");

    auto activeLc = RetrieveActiveLcs(ueLCG, isDl);

    // Drop control LCs (SRB0/SRB1) from the data distribution: SRBs are
    // already handled by AssignControlBytes() and partial leftovers must not
    // be split further. In particular, slicing SRB1 (RLC AM) into a sub-PDU
    // smaller than the RLC AM header (4 bytes) makes the receiving RLC layer
    // assert.
    for (auto it = activeLc.begin(); it != activeLc.end();)
    {
        if (it->first.second == 0 || it->first.second == 1)
        {
            it = activeLc.erase(it);
        }
        else
        {
            ++it;
        }
    }

    if (activeLc.empty())
    {
        return ret;
    }

    // Sub-PDU floor: 3 bytes for the MAC subheader plus 7 bytes for the
    // smallest useful RLC AM data PDU (2-byte RLC AM header + at least one
    // byte of payload and SDU overhead -- NrRlcAm requires at least 7 bytes
    // of TxOpportunity for a new data PDU; less than that and it asserts).
    // Anything below this floor would force the receiving RLC layer to drop
    // a stub. Bail out of the round-robin loop instead of splitting too thin.
    constexpr uint32_t kMinSubPduBytes = 3 + 7;

    while (tbs >= kMinSubPduBytes)
    {
        // Count how many logical channels still need bytes
        auto numActive = std::count_if(activeLc.begin(), activeLc.end(), [](const auto& lc) {
            return lc.second.first > 0;
        });

        // Calculate bytes to assign per LC this round, but cap the number of
        // LCs served so each gets at least the minimum useful sub-PDU.
        auto numServable =
            std::min<std::size_t>(static_cast<std::size_t>(numActive), tbs / kMinSubPduBytes);
        if (numServable == 0)
        {
            break;
        }
        uint32_t assignBlockSize = tbs / static_cast<uint32_t>(numServable);

        // Cap block size to the smallest remaining buffer (but keep it at
        // least the minimum sub-PDU size so any served LC stays valid).
        for (const auto& [lcId, lcData] : activeLc)
        {
            if (lcData.first > 0)
            {
                assignBlockSize = std::min(assignBlockSize, lcData.first);
            }
        }

        // Ensure at least kMinSubPduBytes bytes per assignment when many LCs compete
        assignBlockSize = std::max(assignBlockSize, kMinSubPduBytes);

        // Distribute bytes to each LC
        for (auto& [lcId, lcData] : activeLc)
        {
            auto& unallocatedBytes = lcData.first;
            auto& allocatedBytes = lcData.second;

            // Skip satisfied LCs while others remain active
            if (unallocatedBytes == 0 && numActive > 0)
            {
                continue;
            }

            if (tbs < kMinSubPduBytes)
            {
                break;
            }

            const uint32_t toAssign = std::min(assignBlockSize, tbs);
            allocatedBytes += toAssign;
            tbs -= toAssign;
            unallocatedBytes = (unallocatedBytes >= toAssign) ? unallocatedBytes - toAssign : 0u;

            if (tbs == 0)
            {
                break;
            }
        }
    }

    for (auto& activeLcEntry : activeLc)
    {
        const auto lcg = activeLcEntry.first.first;
        const auto lcId = activeLcEntry.first.second;
        const auto amountPerLC = activeLcEntry.second.second;
        if (amountPerLC == 0)
        {
            continue;
        }
        ret.emplace_back(lcg, lcId, amountPerLC);
        NS_LOG_INFO("Assigned to LCID " << static_cast<uint32_t>(lcId) << " inside LCG "
                                        << static_cast<uint32_t>(lcg) << " an amount of "
                                        << amountPerLC << " B");
    }
    return ret;
}

}; // namespace ns3
