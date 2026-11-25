// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#define NS_LOG_APPEND_CONTEXT                                                                      \
    do                                                                                             \
    {                                                                                              \
        std::clog << " [ CellId " << GetCellId() << ", bwpId " << GetBwpId() << "] ";              \
    } while (false);
#include "nr-mac-scheduler-harq-rr.h"

#include "nr-fh-control.h"

#include "ns3/boolean.h"
#include "ns3/log.h"

#include <algorithm>
#include <iterator>
#include <set>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrMacSchedulerHarqRr");
NS_OBJECT_ENSURE_REGISTERED(NrMacSchedulerHarqRr);

TypeId
NrMacSchedulerHarqRr::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrMacSchedulerHarqRr")
            .SetParent<Object>()
            .AddAttribute("ConsolidateHarqRetx",
                          "Consolidate HARQ DCI through reshaping to improve resource utilization",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrMacSchedulerHarqRr::m_consolidateHarqRetx),
                          MakeBooleanChecker());
    return tid;
}

NrMacSchedulerHarqRr::NrMacSchedulerHarqRr()
{
}

void
NrMacSchedulerHarqRr::InstallGetBwpIdFn(const std::function<uint16_t()>& fn)
{
    m_getBwpId = fn;
}

void
NrMacSchedulerHarqRr::InstallGetCellIdFn(const std::function<uint16_t()>& fn)
{
    m_getCellId = fn;
}

void
NrMacSchedulerHarqRr::InstallGetBwInRBG(const std::function<uint16_t()>& fn)
{
    m_getBwInRbg = fn;
}

void
NrMacSchedulerHarqRr::InstallGetFhControlMethodFn(const std::function<uint8_t()>& fn)
{
    m_getFhControlMethod = fn;
}

void
NrMacSchedulerHarqRr::InstallDoesFhAllocationFitFn(
    const std::function<
        bool(uint16_t bwpId, uint32_t mcs, uint32_t nRegs, uint8_t dlRank, uint8_t numSym)>& fn)
{
    m_getDoesAllocationFit = fn;
}

void
NrMacSchedulerHarqRr::InstallReshapeAllocation(
    const std::function<
        const std::vector<DciInfoElementTdma>(const std::vector<DciInfoElementTdma>& dcis,
                                              uint8_t& startingSymbol,
                                              uint8_t& numSymbols,
                                              std::vector<bool>& bitmask,
                                              const bool isDl)>& fn)
{
    m_getReshapeAllocation = fn;
}

std::vector<BeamId>
NrMacSchedulerHarqRr::GetBeamOrderRR(const NrMacSchedulerNs3::ActiveHarqMap& activeHarqMap,
                                     BeamQueue* queue) const
{
    for (const auto& el : activeHarqMap)
    {
        if (queue->knownBeams.insert(el.first).second)
        {
            queue->order.push_back(el.first);
        }
    }

    // The queue keeps every beam ever seen, so skip the ones without retransmissions
    std::vector<BeamId> ret;
    ret.reserve(activeHarqMap.size());
    for (const auto& beam : queue->order)
    {
        if (activeHarqMap.find(beam) != activeHarqMap.end())
        {
            ret.push_back(beam);
        }
    }

    if (!ret.empty())
    {
        auto first = std::find(queue->order.begin(), queue->order.end(), ret.front());
        std::rotate(queue->order.begin(), std::next(first), queue->order.end());
    }
    return ret;
}

std::ostream&
operator<<(std::ostream& os, const std::vector<bool>& h)
{
    for (auto hb : h)
    {
        os << (int)hb;
    }
    return os;
}

/**
 * @brief Schedule DL HARQ in RR fashion
 * @param startingPoint starting point of the first retransmission.
 * @param symAvail Available symbols
 * @param activeDlHarq Map of the active HARQ processes
 * @param ueMap Map of the UEs
 * @param dlHarqToRetransmit HARQ feedbacks that could not be transmitted (to fill)
 * @param dlHarqFeedback all the HARQ feedbacks
 * @param slotAlloc Slot allocation info
 * @return the VarTtiSlotAlloc ID to use next
 *
 * The retransmissions of a beam are placed on the same symbols, with the RBGs and
 * symbol count of their first transmission. A retransmission whose RBGs are taken by
 * another one of the beam, or whose symbols do not fit, is reshaped into the fewest
 * symbols left after every beam that hold its resources, keeping its MCS and TBS. With
 * ConsolidateHarqRetx, every retransmission is reshaped into symbols of its own.
 */
uint8_t
NrMacSchedulerHarqRr::ScheduleDlHarq(
    NrMacSchedulerNs3::PointInFTPlane* startingPoint,
    uint8_t symAvail,
    const Ns3Sched::ActiveHarqMap& activeDlHarq,
    const std::unordered_map<uint16_t, std::shared_ptr<NrMacSchedulerUeInfo>>& ueMap,
    std::vector<DlHarqInfo>* dlHarqToRetransmit,
    const std::vector<DlHarqInfo>& dlHarqFeedback,
    SlotAllocInfo* slotAlloc) const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(startingPoint->m_rbg == 0);
    const auto preexistingDciNum = slotAlloc->m_varTtiAllocInfo.size();
    auto currStartingSymbol = startingPoint->m_sym;

    const bool isDl = true;
    auto dlBitmask = m_getDlBitmask();
    // Retransmissions that do not fit with their shape, to reshape into the symbols left
    std::vector<Ns3Sched::HarqVectorIterator> toReshape;
    std::vector<uint16_t> retransmittingUes;

    // Place the retransmission of a HARQ process with the shape of reshapedDci.
    // rbgAssigned is the RBGs times symbols of the first transmission.
    auto placeRetx =
        [&](HarqProcess& harqProcess, const DciInfoElementTdma& reshapedDci, uint32_t rbgAssigned) {
            NS_ASSERT(reshapedDci.m_format == DciInfoElementTdma::DL);
            auto dci = std::make_shared<DciInfoElementTdma>(reshapedDci.m_rnti,
                                                            reshapedDci.m_format,
                                                            reshapedDci.m_symStart,
                                                            reshapedDci.m_numSym,
                                                            reshapedDci.m_mcs,
                                                            reshapedDci.m_rank,
                                                            reshapedDci.m_precMats,
                                                            reshapedDci.m_tbSize,
                                                            0,
                                                            reshapedDci.m_rv + 1,
                                                            DciInfoElementTdma::DATA,
                                                            reshapedDci.m_bwpIndex,
                                                            reshapedDci.m_tpc);
            dci->m_rbgBitmask = reshapedDci.m_rbgBitmask;
            dci->m_harqProcess = reshapedDci.m_harqProcess;
            harqProcess.m_dciElement = dci;
            harqProcess.m_txAttempts++;
            retransmittingUes.push_back(dci->m_rnti);

            VarTtiAllocInfo slotInfo(dci);
            NS_LOG_DEBUG("UE" << dci->m_rnti << " gets DL symbols " << +dci->m_symStart << "-"
                              << dci->m_symStart + dci->m_numSym - 1 << " tbs " << dci->m_tbSize
                              << " harqId " << +dci->m_harqProcess << " rv " << +dci->m_rv
                              << " RETX on RBGs" << dci->m_rbgBitmask);
            for (const auto& rlcPdu : harqProcess.m_rlcPduInfo)
            {
                slotInfo.m_rlcPduInfo.push_back(rlcPdu);
            }
            slotAlloc->m_varTtiAllocInfo.push_back(slotInfo);
            ueMap.find(dci->m_rnti)->second->m_dlMRBRetx = dci->m_numSym * rbgAssigned;
        };

    NS_LOG_INFO("We have " << activeDlHarq.size() << " beams with data to RETX");
    for (const auto beamId : GetBeamOrderRR(activeDlHarq, &m_dlBeams))
    {
        const auto& beam = *activeDlHarq.find(beamId);
        const auto preexistingDciNumToBeam = slotAlloc->m_varTtiAllocInfo.size();
        auto beamStartingSymbol = currStartingSymbol;
        std::vector<uint16_t> allocatedUe;
        NS_LOG_INFO(" Try to assign HARQ resource for Beam sector: "
                    << static_cast<uint32_t>(beam.first.GetSector())
                    << " Beam theta:  " << static_cast<uint32_t>(beam.first.GetElevation())
                    << " # HARQ to Retx=" << beam.second.size());

        for (auto it : beam.second)
        {
            HarqProcess& harqProcess = it->second;
            NS_ASSERT_MSG(harqProcess.m_status == HarqProcess::RECEIVED_FEEDBACK,
                          "Process " << static_cast<uint32_t>(it->first)
                                     << " is not in RECEIVED_FEEDBACK status");

            harqProcess.m_status = HarqProcess::WAITING_FEEDBACK;
            harqProcess.m_timer = 0;

            auto& dciInfoReTx = harqProcess.m_dciElement;

            uint32_t rbgAssigned =
                std::count(dciInfoReTx->m_rbgBitmask.begin(), dciInfoReTx->m_rbgBitmask.end(), 1) *
                dciInfoReTx->m_numSym;
            uint32_t rbgAvail = (GetBandwidthInRbg() - startingPoint->m_rbg) * symAvail;

            NS_LOG_INFO("Evaluating space to retransmit HARQ PID="
                        << static_cast<uint32_t>(dciInfoReTx->m_harqProcess) << " for UE="
                        << static_cast<uint32_t>(dciInfoReTx->m_rnti) << " SYM assigned previously="
                        << static_cast<uint32_t>(dciInfoReTx->m_numSym)
                        << " RBG assigned previously=" << static_cast<uint32_t>(rbgAssigned)
                        << " SYM avail=" << static_cast<uint32_t>(symAvail)
                        << " RBG avail for this beam=" << rbgAvail);

            if (std::find(allocatedUe.begin(), allocatedUe.end(), dciInfoReTx->m_rnti) !=
                allocatedUe.end())
            {
                NS_LOG_INFO("UE " << dciInfoReTx->m_rnti
                                  << " already has an HARQ allocated, buffer this HARQ process"
                                  << static_cast<uint32_t>(dciInfoReTx->m_harqProcess));
                BufferHARQFeedback(dlHarqFeedback,
                                   dlHarqToRetransmit,
                                   dciInfoReTx->m_rnti,
                                   dciInfoReTx->m_harqProcess);
                continue;
            }
            else if (rbgAvail < rbgAssigned)
            {
                NS_LOG_INFO("No resource for this retx, we have to buffer it");
                BufferHARQFeedback(dlHarqFeedback,
                                   dlHarqToRetransmit,
                                   dciInfoReTx->m_rnti,
                                   dciInfoReTx->m_harqProcess);
                continue;
            }
            if (GetFromSchedFhControlMethod() == NrFhControl::FhControlMethod::Postponing ||
                GetFromSchedFhControlMethod() == NrFhControl::FhControlMethod::OptimizeMcs ||
                GetFromSchedFhControlMethod() == NrFhControl::FhControlMethod::OptimizeRBs)
            {
                if (GetDoesFhAllocationFit(GetBwpId(),
                                           dciInfoReTx->m_mcs,
                                           rbgAssigned,
                                           dciInfoReTx->m_rank,
                                           dciInfoReTx->m_numSym) == 0)
                {
                    NS_LOG_INFO("No FH resources for this retx, we have to buffer it");
                    BufferHARQFeedback(dlHarqFeedback,
                                       dlHarqToRetransmit,
                                       dciInfoReTx->m_rnti,
                                       dciInfoReTx->m_harqProcess);
                    continue;
                }
            }

            // Pass copies, not to have to commit to any changes
            uint8_t symAvailBackup = symAvail;
            auto dlBitmaskBackup = dlBitmask;
            auto currStartingSymbolBackup = currStartingSymbol;
            std::vector<DciInfoElementTdma> reshapedDcis;
            if (m_consolidateHarqRetx)
            {
                reshapedDcis = m_getReshapeAllocation({*harqProcess.m_dciElement},
                                                      currStartingSymbolBackup,
                                                      symAvailBackup,
                                                      dlBitmaskBackup,
                                                      isDl);
            }
            else
            {
                // If not reshaping, we just change at most the starting symbol.
                // Note: do NOT debit symAvail here. In this path every
                // retransmission of a beam is placed at the same starting
                // symbol, so the slot symbol budget must be debited exactly
                // once per beam, by the beam's symbol span, at the end of the
                // beam loop below (see "symAvail -= symbolsUsedForBeam").
                // First, check if there are collisions.
                bool collision = false;
                for (std::size_t i = 0; i < dlBitmaskBackup.size(); i++)
                {
                    if (harqProcess.m_dciElement->m_rbgBitmask.at(i))
                    {
                        if (!dlBitmaskBackup.at(i))
                        {
                            collision = true;
                            break;
                        }
                        dlBitmaskBackup.at(i) = false;
                    }
                }
                if (!collision)
                {
                    reshapedDcis.emplace_back(currStartingSymbolBackup,
                                              harqProcess.m_dciElement->m_numSym,
                                              harqProcess.m_dciElement->m_rbgBitmask,
                                              *harqProcess.m_dciElement);
                }
            }

            // Without consolidation, a retransmission that keeps its shape is placed on the
            // symbols of the beam. When its RBGs are taken, or its symbols do not fit, it is
            // reshaped into the symbols left after every beam.
            if (!m_consolidateHarqRetx &&
                (reshapedDcis.empty() || symAvail < reshapedDcis.front().m_numSym))
            {
                NS_LOG_INFO("This HARQ allocation does not fit with its shape, "
                            "reshape it into the symbols left");
                toReshape.push_back(it);
                continue;
            }

            // If allocation reshaping did not work, buffer DCI
            if (reshapedDcis.empty())
            {
                NS_LOG_INFO(
                    (m_consolidateHarqRetx
                         ? "This HARQ allocation does not fit in the remaining resources, "
                         : "This HARQ allocation collides with a previously allocated HARQ, ")
                    << "we have to buffer it");
                BufferHARQFeedback(dlHarqFeedback,
                                   dlHarqToRetransmit,
                                   dciInfoReTx->m_rnti,
                                   dciInfoReTx->m_harqProcess);
                continue;
            }

            // This code currently only passes one DCI at a time for reshaping, while the
            // function actually supports multiple. So here we only consume the first element.
            harqProcess.m_dciElement = std::make_shared<DciInfoElementTdma>(reshapedDcis.front());
            dciInfoReTx = harqProcess.m_dciElement;
            auto numSymbols = dciInfoReTx->m_numSym;
            if (symAvail < numSymbols)
            {
                NS_LOG_INFO("No symbols available for this HARQ allocation, we have to buffer it");
                BufferHARQFeedback(dlHarqFeedback,
                                   dlHarqToRetransmit,
                                   dciInfoReTx->m_rnti,
                                   dciInfoReTx->m_harqProcess);
                continue;
            }

            // Commit changes made to number of symbols, RBG bitmask and starting symbol
            // during reshaping
            symAvail = symAvailBackup;
            dlBitmask = dlBitmaskBackup;
            currStartingSymbol = currStartingSymbolBackup;
            allocatedUe.push_back(dciInfoReTx->m_rnti);
            placeRetx(harqProcess, *dciInfoReTx, rbgAssigned);
        }
        // If there are still symbols left for the next beam, reset RBG mask
        if (symAvail > 0)
        {
            dlBitmask = m_getDlBitmask();
            // Advance symbol for OFDMA to prevent overlapping allocations with different beams
            if (beamStartingSymbol == currStartingSymbol)
            {
                auto symbolsUsedForBeam = nr::CountUsedSymbolsFromVarAllocTtiRange(
                    startingPoint->m_sym,
                    slotAlloc->m_varTtiAllocInfo.begin() + preexistingDciNumToBeam,
                    slotAlloc->m_varTtiAllocInfo.end());
                // symAvail is a uint8_t: guard against underflow. Every
                // retransmission placed in this beam was checked against symAvail
                // above, so the beam span can never exceed it. If this fires, the
                // symbol budget is being debited more than once for the same
                // symbols.
                NS_ASSERT_MSG(symAvail >= symbolsUsedForBeam,
                              "HARQ beam span (" << +symbolsUsedForBeam
                                                 << ") exceeds remaining symbols (" << +symAvail
                                                 << "); symbol budget was double-counted");
                currStartingSymbol += symbolsUsedForBeam;
                symAvail -= symbolsUsedForBeam;
            }
        }
    }
    // Each reshaped retransmission gets symbols of its own, over the DL band
    for (auto it : toReshape)
    {
        HarqProcess& harqProcess = it->second;
        const auto& dciInfoReTx = harqProcess.m_dciElement;
        const uint32_t rbgAssigned =
            std::count(dciInfoReTx->m_rbgBitmask.begin(), dciInfoReTx->m_rbgBitmask.end(), 1) *
            dciInfoReTx->m_numSym;
        std::vector<DciInfoElementTdma> reshapedDcis;
        if (std::find(retransmittingUes.begin(), retransmittingUes.end(), dciInfoReTx->m_rnti) ==
                retransmittingUes.end() &&
            symAvail > 0)
        {
            auto bitmask = m_getDlBitmask();
            reshapedDcis =
                m_getReshapeAllocation({*dciInfoReTx}, currStartingSymbol, symAvail, bitmask, isDl);
        }
        if (reshapedDcis.empty())
        {
            NS_LOG_INFO("No resources left to reshape this HARQ allocation, we have to buffer it");
            BufferHARQFeedback(dlHarqFeedback,
                               dlHarqToRetransmit,
                               dciInfoReTx->m_rnti,
                               dciInfoReTx->m_harqProcess);
            continue;
        }
        NS_LOG_INFO("Reshaped the retransmission from " << +dciInfoReTx->m_numSym << " to "
                                                        << +reshapedDcis.front().m_numSym
                                                        << " symbols");
        placeRetx(harqProcess, reshapedDcis.front(), rbgAssigned);
    }

    NS_ASSERT(startingPoint->m_rbg == 0);

    auto usedSymbols = nr::CountUsedSymbolsFromVarAllocTtiRange(
        startingPoint->m_sym,
        slotAlloc->m_varTtiAllocInfo.begin() + preexistingDciNum,
        slotAlloc->m_varTtiAllocInfo.end());
    return usedSymbols;
}

/**
 * @brief Schedule the UL HARQ
 * @param startingPoint starting point of the first retransmission.
 * It should be set to the next available starting point
 * @param symAvail Available symbols
 * @param activeUlHarq Map of the active HARQ processes, sorted per beam
 * @param ueMap Map of the UEs
 * @param ulHarqToRetransmit HARQ feedbacks that could not be transmitted (to fill)
 * @param ulHarqFeedback all the HARQ feedbacks
 * @param slotAlloc Slot allocation info
 * @return the VarTtiSlotAlloc ID to use next
 *
 * The feedbacks are visited beam by beam in round-robin order, and in the
 * order of activeUlHarq within each beam.
 *
 * For each NACKed process a DCI is built, with the exact same specification
 * (RBGs, symbol count, MCS and TBS) as the first transmission. A retransmission
 * shares the symbols of one already placed in this slot when both have the same
 * symbol count, the UEs share a beam, and their RBGs do not overlap; otherwise it
 * gets its own symbols. When its symbols do not fit in the ones left, it is
 * reshaped into the fewest of them that hold its resources, keeping its MCS and
 * TBS. If even that does not fit, the feedback is buffered for the next slot.
 */
uint8_t
NrMacSchedulerHarqRr::ScheduleUlHarq(
    NrMacSchedulerNs3::PointInFTPlane* startingPoint,
    uint8_t symAvail,
    const NrMacSchedulerNs3::ActiveHarqMap& activeUlHarq,
    const std::unordered_map<uint16_t, std::shared_ptr<NrMacSchedulerUeInfo>>& ueMap,
    std::vector<UlHarqInfo>* ulHarqToRetransmit,
    const std::vector<UlHarqInfo>& ulHarqFeedback,
    SlotAllocInfo* slotAlloc) const
{
    NS_LOG_FUNCTION(this);
    uint8_t symUsed = 0;
    NS_ASSERT(startingPoint->m_rbg == 0);

    /// Symbols already given to UL retransmissions in this slot
    struct RetxSpan
    {
        uint8_t symStart;       ///< First symbol
        uint8_t numSym;         ///< Number of symbols
        BeamId beamId;          ///< Beam of the UEs retransmitting in this span
        std::vector<bool> rbgs; ///< RBGs already used in this span
        std::set<uint16_t> ues; ///< UEs retransmitting in this span
    };

    std::vector<RetxSpan> spans;

    NS_LOG_INFO("Scheduling UL HARQ starting from sym "
                << +startingPoint->m_sym << " and RBG " << startingPoint->m_rbg
                << ". Available symbols: " << symAvail
                << " number of feedback: " << ulHarqFeedback.size());

    std::vector<UlHarqInfo> orderedFeedback;
    orderedFeedback.reserve(ulHarqFeedback.size());
    for (const auto& beamId : GetBeamOrderRR(activeUlHarq, &m_ulBeams))
    {
        for (const auto& it : activeUlHarq.at(beamId))
        {
            auto feedback =
                std::find_if(ulHarqFeedback.begin(), ulHarqFeedback.end(), [&](const auto& info) {
                    return info.m_harqProcessId == it->first &&
                           info.m_rnti == it->second.m_dciElement->m_rnti;
                });
            NS_ASSERT_MSG(feedback != ulHarqFeedback.end(), "No feedback for an active UL HARQ");
            orderedFeedback.push_back(*feedback);
        }
    }
    NS_ASSERT(orderedFeedback.size() == ulHarqFeedback.size());

    // ntn: every NACK is visited. The loop used to stop as soon as the symbols ran out, and the
    // feedbacks after that point were neither scheduled nor put back in ulHarqToRetransmit: their
    // processes stayed RECEIVED_FEEDBACK until ResetExpiredHARQ erased them, so a TB the gNB knew
    // it had to retransmit was silently dropped (measured on a weak NTN uplink: ~1 in 3 failed
    // first transmissions, 102 processes per run). The process state now changes only when the
    // retransmission is actually placed.
    for (std::size_t i = 0; i < orderedFeedback.size(); i++)
    {
        const UlHarqInfo& harqInfo = orderedFeedback.at(i);
        uint8_t harqId = harqInfo.m_harqProcessId;
        uint16_t rnti = harqInfo.m_rnti;

        NS_ABORT_IF(harqInfo.IsReceivedOk());

        // retx correspondent block: retrieve the UL-DCI
        HarqProcess& harqProcess = ueMap.find(rnti)->second->m_ulHarq.Find(harqId)->second;
        NS_ASSERT(harqProcess.m_status == HarqProcess::RECEIVED_FEEDBACK);

        auto& dciInfoReTx = harqProcess.m_dciElement;

        NS_LOG_INFO("Feedback is for UE " << rnti << " process " << +harqId
                                          << " sym: " << +dciInfoReTx->m_numSym);

        const auto& beamId = ueMap.find(rnti)->second->m_beamId;
        uint8_t numSym = dciInfoReTx->m_numSym;
        std::vector<bool> rbgs = dciInfoReTx->m_rbgBitmask;
        auto span = std::find_if(spans.begin(), spans.end(), [&](const RetxSpan& sp) {
            // A UE transmits a single PUSCH at a time
            if (sp.numSym != numSym || sp.beamId != beamId || sp.rbgs.size() != rbgs.size() ||
                sp.ues.count(rnti) > 0)
            {
                return false;
            }
            for (std::size_t rbg = 0; rbg < rbgs.size(); ++rbg)
            {
                if (rbgs[rbg] && sp.rbgs[rbg])
                {
                    return false;
                }
            }
            return true;
        });

        if (span == spans.end() && symAvail > 0 && symAvail < numSym)
        {
            // The symbols left are taken from the end of the slot backwards
            uint8_t reshapeStart = startingPoint->m_sym - symAvail;
            uint8_t reshapeSym = symAvail;
            auto bitmask = m_getUlBitmask();
            const bool isDl = false;
            const auto reshaped =
                m_getReshapeAllocation({*dciInfoReTx}, reshapeStart, reshapeSym, bitmask, isDl);
            if (!reshaped.empty())
            {
                NS_LOG_INFO("Reshaped the retransmission from "
                            << +numSym << " to " << +reshaped.front().m_numSym << " symbols");
                numSym = reshaped.front().m_numSym;
                rbgs = reshaped.front().m_rbgBitmask;
            }
        }

        if (span == spans.end() && symAvail >= numSym)
        {
            symAvail -= numSym;
            symUsed += numSym;
            startingPoint->m_sym -= numSym;
            slotAlloc->m_numSymAlloc += numSym;
            spans.push_back(
                {startingPoint->m_sym, numSym, beamId, std::vector<bool>(rbgs.size(), false), {}});
            span = std::prev(spans.end());
        }

        if (span != spans.end())
        {
            for (std::size_t rbg = 0; rbg < rbgs.size(); ++rbg)
            {
                span->rbgs[rbg] = span->rbgs[rbg] || rbgs[rbg];
            }
            span->ues.insert(rnti);
            harqProcess.m_status = HarqProcess::WAITING_FEEDBACK;
            harqProcess.m_timer = 0;

            NS_ASSERT(dciInfoReTx->m_format == DciInfoElementTdma::UL);

            const uint8_t nextAttempts = harqProcess.m_txAttempts + 1;
            auto dci = std::make_shared<DciInfoElementTdma>(dciInfoReTx->m_rnti,
                                                            dciInfoReTx->m_format,
                                                            span->symStart,
                                                            numSym,
                                                            dciInfoReTx->m_mcs,
                                                            dciInfoReTx->m_rank,
                                                            dciInfoReTx->m_precMats,
                                                            dciInfoReTx->m_tbSize,
                                                            0,
                                                            dciInfoReTx->m_rv + 1,
                                                            DciInfoElementTdma::DATA,
                                                            dciInfoReTx->m_bwpIndex,
                                                            dciInfoReTx->m_tpc);
            dci->m_rbgBitmask = rbgs;
            dci->m_harqProcess = harqId;
            harqProcess.m_dciElement = dci;
            harqProcess.m_txAttempts = nextAttempts;
            dciInfoReTx = harqProcess.m_dciElement;

            VarTtiAllocInfo slotInfo(dciInfoReTx);
            NS_LOG_DEBUG(
                "UE" << dciInfoReTx->m_rnti << " gets UL symbols "
                     << static_cast<uint32_t>(dciInfoReTx->m_symStart) << "-"
                     << static_cast<uint32_t>(dciInfoReTx->m_symStart + dciInfoReTx->m_numSym - 1)
                     << " tbs " << dciInfoReTx->m_tbSize << " harqId "
                     << static_cast<uint32_t>(dciInfoReTx->m_harqProcess) << " rv "
                     << static_cast<uint32_t>(dciInfoReTx->m_rv) << " RETX");
            slotAlloc->m_varTtiAllocInfo.push_front(slotInfo);

            ueMap.find(rnti)->second->m_ulMRBRetx = dciInfoReTx->m_numSym * GetBandwidthInRbg();
        }
        else
        {
            ulHarqToRetransmit->push_back(orderedFeedback.at(i));
        }
    }

    // Retransmissions sharing a span were added after the spans that followed it
    std::stable_sort(slotAlloc->m_varTtiAllocInfo.begin(), slotAlloc->m_varTtiAllocInfo.end());

    NS_ASSERT(startingPoint->m_rbg == 0);

    return symUsed;
}

/**
 * @brief Sort Dl Harq retx based on their symbol requirement
 * @param activeDlHarq map of the active retx
 */
void
NrMacSchedulerHarqRr::SortDlHarq(NrMacSchedulerNs3::ActiveHarqMap* activeDlHarq) const
{
    NS_LOG_FUNCTION(this);

    // Order based on required sym
    static struct
    {
        bool operator()(const NrMacSchedulerNs3::HarqVectorIterator& a,
                        const NrMacSchedulerNs3::HarqVectorIterator& b) const
        {
            return a->second.m_dciElement->m_numSym > b->second.m_dciElement->m_numSym;
        }
    } CompareNumSym;

    for (auto& it : *activeDlHarq)
    {
        std::stable_sort(it.second.begin(), it.second.end(), CompareNumSym);
    }
}

/**
 * @brief Sort Ul Harq retx based on their symbol requirement
 * @param activeUlHarq map of the active retx
 *
 */
void
NrMacSchedulerHarqRr::SortUlHarq(NrMacSchedulerNs3::ActiveHarqMap* activeUlHarq) const
{
    NS_LOG_FUNCTION(this);
    SortDlHarq(activeUlHarq); // We reuse DL sort
}

/**
 * @brief Find the specified HARQ process and buffer it into a vector
 * @param dlHarqFeedback HARQ not retransmitted list
 * @param dlHarqToRetransmit HARQ buffer list (to retransmit)
 * @param rnti RNTI to find
 * @param harqProcess process ID to find
 */
void
NrMacSchedulerHarqRr::BufferHARQFeedback(const std::vector<DlHarqInfo>& dlHarqFeedback,
                                         std::vector<DlHarqInfo>* dlHarqToRetransmit,
                                         uint16_t rnti,
                                         uint8_t harqProcess) const
{
    NS_LOG_FUNCTION(this);

    bool found = false;
    for (const auto& feedback : dlHarqFeedback)
    {
        if (feedback.m_rnti == rnti && feedback.m_harqProcessId == harqProcess)
        {
            dlHarqToRetransmit->push_back(feedback);
            found = true;
            break;
        }
    }
    NS_ASSERT(found);
}

uint16_t
NrMacSchedulerHarqRr::GetBwpId() const
{
    return m_getBwpId();
}

uint16_t
NrMacSchedulerHarqRr::GetCellId() const
{
    return m_getCellId();
}

uint16_t
NrMacSchedulerHarqRr::GetBandwidthInRbg() const
{
    return m_getBwInRbg();
}

uint8_t
NrMacSchedulerHarqRr::GetFromSchedFhControlMethod() const
{
    return m_getFhControlMethod();
}

bool
NrMacSchedulerHarqRr::GetDoesFhAllocationFit(uint16_t bwpId,
                                             uint32_t mcs,
                                             uint32_t nRegs,
                                             uint8_t dlRank,
                                             uint8_t numSym) const
{
    return m_getDoesAllocationFit(bwpId, mcs, nRegs, dlRank, numSym);
}

void
NrMacSchedulerHarqRr::InstallGetDlBitmask(const std::function<std::vector<bool>()>& fn)
{
    m_getDlBitmask = fn;
}

void
NrMacSchedulerHarqRr::InstallGetUlBitmask(const std::function<std::vector<bool>()>& fn)
{
    m_getUlBitmask = fn;
}

} // namespace ns3
