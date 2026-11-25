// Copyright (c) 2023 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-mac-scheduler-lc-alg.h"

#include "nr-phy-mac-common.h"

#include "ns3/log.h"

#include <algorithm>
#include <numeric>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrMacSchedulerLcAlgorithm");
NS_OBJECT_ENSURE_REGISTERED(NrMacSchedulerLcAlgorithm);

NrMacSchedulerLcAlgorithm::NrMacSchedulerLcAlgorithm()
    : Object()
{
    NS_LOG_FUNCTION(this);
}

NrMacSchedulerLcAlgorithm::~NrMacSchedulerLcAlgorithm()
{
}

TypeId
NrMacSchedulerLcAlgorithm::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrMacSchedulerLcAlgorithm").SetParent<Object>();
    return tid;
}

std::map<std::pair<Lcg, LcId>, std::pair<UnassignedBytes, AssignedBytes>>
NrMacSchedulerLcAlgorithm::RetrieveActiveLcs(const std::unordered_map<uint8_t, LCGPtr>& ueLCG,
                                             bool isDl)
{
    NS_LOG_FUNCTION_NOARGS();
    std::map<std::pair<Lcg, LcId>, std::pair<UnassignedBytes, AssignedBytes>> activeLc;
    for (const auto& lcg : ueLCG)
    {
        std::vector<uint8_t> lcs = lcg.second->GetLCId();
        for (const auto& lcId : lcs)
        {
            if (lcg.second->GetTotalSizeOfLC(lcId) > 0)
            {
                activeLc[{lcg.first, lcId}] = {
                    lcg.second->GetTotalSizeOfLCPlusOverheads(lcId, isDl),
                    0};
            }
        }
    }
    return activeLc;
}

std::vector<NrMacSchedulerLcAlgorithm::Assignation>
NrMacSchedulerLcAlgorithm::AssignControlBytes(const std::unordered_map<uint8_t, LCGPtr>& ueLCG,
                                              uint32_t tbs,
                                              bool isDl)
{
    std::vector<NrMacSchedulerLcAlgorithm::Assignation> ret;
    auto activeLc = RetrieveActiveLcs(ueLCG, isDl);

    if (activeLc.empty())
    {
        return ret;
    }

    // Minimum sub-PDU size that can carry a control LC: 3 bytes of MAC
    // subheader plus the smallest RLC PDU header. SRB0 uses RLC TM (no RLC
    // header) but SRB1 uses RLC AM, which rejects a TxOpportunity smaller
    // than 4 bytes; allocating fewer bytes than that crashes the receiving
    // RLC layer instead of harmlessly deferring the transmission. Use the
    // RLC AM minimum here so the same code path is safe for both SRBs.
    constexpr uint32_t kMinControlSubPduBytes = 3 /* MAC subheader */ + 4 /* RLC AM hdr */;

    for (auto& [lcId, lcData] : activeLc)
    {
        if ((lcId.second == 0 || lcId.second == 1) && (lcData.first > 0))
        {
            auto& unallocatedBytes = lcData.first;
            auto& allocatedBytes = lcData.second;
            if (unallocatedBytes <= tbs)
            {
                ret.emplace_back(lcId.first, lcId.second, unallocatedBytes);
                tbs -= unallocatedBytes;
                allocatedBytes = unallocatedBytes;
                unallocatedBytes = 0;
            }
            else if (tbs >= kMinControlSubPduBytes)
            {
                ret.emplace_back(lcId.first, lcId.second, tbs);
                allocatedBytes = tbs;
                unallocatedBytes -= tbs;
                tbs = 0;
                NS_LOG_WARN("TBS size " << tbs
                                        << " bytes is not sufficient for control channel LCID "
                                        << +lcId.second << " (" << unallocatedBytes << " bytes)");
            }
            else
            {
                // Remaining TBS is below the smallest sub-PDU that carries an
                // RLC AM header. Leave the LC pending for the next slot so we
                // don't hand RLC AM a stub it must assert on.
                NS_LOG_WARN("Remaining TBS " << tbs << " bytes is below the minimum sub-PDU size ("
                                             << kMinControlSubPduBytes << ") for control LCID "
                                             << +lcId.second << "; deferring " << unallocatedBytes
                                             << " bytes");
            }
        }
    }
    return ret;
}

std::vector<NrMacSchedulerLcAlgorithm::Assignation>
NrMacSchedulerLcAlgorithm::AssignBytesToLC(const std::unordered_map<uint8_t, LCGPtr>& ueLCG,
                                           uint32_t tbs,
                                           Time slotPeriod,
                                           bool isDl) const
{
    // Work on a ueLCG copy
    std::unordered_map<uint8_t, LCGPtr> ueLCGCopy;
    for (const auto& [key, val] : ueLCG)
    {
        NrMacSchedulerLCG lcg = *val;
        ueLCGCopy[key] = std::make_unique<NrMacSchedulerLCG>(lcg);
    }

    // We first handle allocation of control logic channels. They ALWAYS have priority over data.
    auto ret = AssignControlBytes(ueLCGCopy, tbs, isDl);

    // If there was some control assignment, we need to clear it up on our ueLCG copy,
    // then pass to data
    if (!ret.empty())
    {
        // Reduce assigned control bytes from TBS
        for (const auto& assignation : ret)
        {
            tbs -= assignation.m_bytes;
            ueLCGCopy.at(assignation.m_lcg)
                ->AssignedData(assignation.m_lcId, assignation.m_bytes, isDl ? "DL" : "UL");
        }
    }

    // Allocate data for data channels using custom policies (RR, QoS, etc)
    auto retData = isDl ? DoAssignBytesToDlLC(ueLCGCopy, tbs, slotPeriod)
                        : DoAssignBytesToUlLC(ueLCGCopy, tbs);

    // Merge control and data assignments
    if (!retData.empty())
    {
        for (auto& assignment : retData)
        {
            if (assignment.m_bytes != 0)
            {
                ret.emplace_back(assignment.m_lcg, assignment.m_lcId, assignment.m_bytes);
            }
        }
    }

    // Safety net: remove any entries with fewer than MAC_SUBHEADER_SIZE bytes
    // (Use manual filtering since Assignation is not copyable)
    std::vector<Assignation> filteredRet;
    filteredRet.reserve(ret.size());
    for (auto& a : ret)
    {
        if (a.m_bytes >= MAC_SUBHEADER_SIZE)
        {
            filteredRet.push_back(std::move(a));
        }
    }
    ret = std::move(filteredRet);

    return ret;
}

std::vector<NrMacSchedulerLcAlgorithm::Assignation>
NrMacSchedulerLcAlgorithm::AssignBytesToDlLC(const std::unordered_map<uint8_t, LCGPtr>& ueLCG,
                                             uint32_t tbs,
                                             Time slotPeriod) const
{
    return AssignBytesToLC(ueLCG, tbs, slotPeriod, true);
}

std::vector<NrMacSchedulerLcAlgorithm::Assignation>
NrMacSchedulerLcAlgorithm::AssignBytesToUlLC(const std::unordered_map<uint8_t, LCGPtr>& ueLCG,
                                             uint32_t tbs) const
{
    return AssignBytesToLC(ueLCG, tbs, Time(0), false);
}

} // namespace ns3
