// Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
// Copyright (c) 2013 Budiarto Herman
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Original work authors (from lte-enb-rrc.cc):
//   Nicola Baldo <nbaldo@cttc.es>
//   Marco Miozzo <mmiozzo@cttc.es>
//   Manuel Requena <manuel.requena@cttc.es>
//
// Converted to handover algorithm interface by:
//   Budiarto Herman <budiarto.herman@magister.fi>

#include "nr-a2-a4-rsrp-handover-algorithm.h"

#include "ns3/log.h"
#include "ns3/uinteger.h"

#include <algorithm>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrA2A4RsrpHandoverAlgorithm");

NS_OBJECT_ENSURE_REGISTERED(NrA2A4RsrpHandoverAlgorithm);

///////////////////////////////////////////
// Handover Management SAP forwarder
///////////////////////////////////////////

NrA2A4RsrpHandoverAlgorithm::NrA2A4RsrpHandoverAlgorithm()
    : m_servingCellThreshold(46),
      m_neighbourCellOffset(1),
      m_handoverManagementSapUser(nullptr)
{
    NS_LOG_FUNCTION(this);
    m_handoverManagementSapProvider =
        new MemberNrHandoverManagementSapProvider<NrA2A4RsrpHandoverAlgorithm>(this);
}

NrA2A4RsrpHandoverAlgorithm::~NrA2A4RsrpHandoverAlgorithm()
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrA2A4RsrpHandoverAlgorithm::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrA2A4RsrpHandoverAlgorithm")
            .SetParent<NrHandoverAlgorithm>()
            .SetGroupName("Nr")
            .AddConstructor<NrA2A4RsrpHandoverAlgorithm>()
            .AddAttribute(
                "ServingCellThreshold",
                "If the RSRP of the serving cell is worse than this "
                "threshold, neighbour cells are consider for handover. "
                "Expressed in quantized range of [0..127] as per Table 10.1.6.1-1 "
                "of 3GPP TS 38.133.",
                UintegerValue(46),
                MakeUintegerAccessor(&NrA2A4RsrpHandoverAlgorithm::m_servingCellThreshold),
                MakeUintegerChecker<uint8_t>(0, 127))
            .AddAttribute("NeighbourCellOffset",
                          "Minimum offset between the serving and the best neighbour "
                          "cell to trigger the handover. Expressed in quantized "
                          "range of [0..127] as per Table 10.1.6.1-1 of 3GPP TS 38.133.",
                          UintegerValue(1),
                          MakeUintegerAccessor(&NrA2A4RsrpHandoverAlgorithm::m_neighbourCellOffset),
                          MakeUintegerChecker<uint8_t>());
    return tid;
}

void
NrA2A4RsrpHandoverAlgorithm::SetNrHandoverManagementSapUser(NrHandoverManagementSapUser* s)
{
    NS_LOG_FUNCTION(this << s);
    m_handoverManagementSapUser = s;
}

NrHandoverManagementSapProvider*
NrA2A4RsrpHandoverAlgorithm::GetNrHandoverManagementSapProvider()
{
    NS_LOG_FUNCTION(this);
    return m_handoverManagementSapProvider;
}

void
NrA2A4RsrpHandoverAlgorithm::DoInitialize()
{
    NS_LOG_FUNCTION(this);

    NS_LOG_LOGIC(this << " requesting Event A2 measurements"
                      << " (threshold=" << (uint16_t)m_servingCellThreshold << ")");
    NrRrcSap::ReportConfigEutra reportConfigA2;
    reportConfigA2.eventId = NrRrcSap::ReportConfigEutra::EVENT_A2;
    reportConfigA2.threshold1.choice =
        NrRrcSap::ThresholdEutra::THRESHOLD_RSRP; // todo: implement RSRQ
    reportConfigA2.threshold1.range = m_servingCellThreshold;
    reportConfigA2.triggerQuantity = NrRrcSap::ReportConfigEutra::RSRP; // todo: implement RSRQ
    reportConfigA2.reportInterval = NrRrcSap::ReportConfigEutra::MS240;
    m_a2MeasIds = m_handoverManagementSapUser->AddUeMeasReportConfigForHandover(reportConfigA2);

    NS_LOG_LOGIC(this << " requesting Event A4 measurements"
                      << " (threshold=0)");
    NrRrcSap::ReportConfigEutra reportConfigA4;
    reportConfigA4.eventId = NrRrcSap::ReportConfigEutra::EVENT_A4;
    reportConfigA4.threshold1.choice =
        NrRrcSap::ThresholdEutra::THRESHOLD_RSRP; // todo: implement RSRQ
    reportConfigA4.threshold1.range = 0;          // intentionally very low threshold
    reportConfigA4.triggerQuantity = NrRrcSap::ReportConfigEutra::RSRP; // todo: implement RSRQ
    reportConfigA4.reportInterval = NrRrcSap::ReportConfigEutra::MS480;
    m_a4MeasIds = m_handoverManagementSapUser->AddUeMeasReportConfigForHandover(reportConfigA4);

    NrHandoverAlgorithm::DoInitialize();
}

void
NrA2A4RsrpHandoverAlgorithm::DoDispose()
{
    NS_LOG_FUNCTION(this);
    delete m_handoverManagementSapProvider;
}

void
NrA2A4RsrpHandoverAlgorithm::DoReportUeMeas(uint16_t rnti, NrRrcSap::MeasResults measResults)
{
    NS_LOG_FUNCTION(this << rnti << (uint16_t)measResults.measId);

    if (std::find(begin(m_a2MeasIds), end(m_a2MeasIds), measResults.measId) !=
        std::end(m_a2MeasIds))
    {
        NS_ASSERT_MSG(measResults.measResultPCell.rsrpResult <= m_servingCellThreshold,
                      "Invalid UE measurement report");
        EvaluateHandover(rnti, measResults.measResultPCell.rsrpResult);
    }
    else if (std::find(begin(m_a4MeasIds), end(m_a4MeasIds), measResults.measId) !=
             std::end(m_a4MeasIds))
    {
        if (measResults.haveMeasResultNeighCells && !measResults.measResultListEutra.empty())
        {
            for (auto it = measResults.measResultListEutra.begin();
                 it != measResults.measResultListEutra.end();
                 ++it)
            {
                NS_ASSERT_MSG(it->haveRsrpResult == true,
                              "RSRQ measurement is missing from cellId " << it->physCellId);
                UpdateNeighbourMeasurements(rnti, it->physCellId, it->rsrpResult);
            }
        }
        else
        {
            NS_LOG_WARN(
                this << " Event A4 received without measurement results from neighbouring cells");
        }
    }
    else
    {
        NS_LOG_WARN("Ignoring measId " << (uint16_t)measResults.measId);
    }

} // end of DoReportUeMeas

void
NrA2A4RsrpHandoverAlgorithm::EvaluateHandover(uint16_t rnti, uint8_t servingCellRsrp)
{
    NS_LOG_FUNCTION(this << rnti << (uint16_t)servingCellRsrp);

    auto it1 = m_neighbourCellMeasures.find(rnti);

    if (it1 == m_neighbourCellMeasures.end())
    {
        NS_LOG_WARN("Skipping handover evaluation for RNTI "
                    << rnti << " because neighbour cells information is not found");
    }
    else
    {
        // Find the best neighbour cell (gNB)
        NS_LOG_LOGIC("Number of neighbour cells = " << it1->second.size());
        uint16_t bestNeighbourCellId = 0;
        uint8_t bestNeighbourRsrp = 0;
        for (auto it2 = it1->second.begin(); it2 != it1->second.end(); ++it2)
        {
            if ((it2->second->m_rsrp > bestNeighbourRsrp) && IsValidNeighbour(it2->first))
            {
                bestNeighbourCellId = it2->first;
                bestNeighbourRsrp = it2->second->m_rsrp;
            }
        }

        // Trigger Handover, if needed
        if (bestNeighbourCellId > 0)
        {
            NS_LOG_LOGIC("Best neighbour cellId " << bestNeighbourCellId);

            if ((bestNeighbourRsrp - servingCellRsrp) >= m_neighbourCellOffset)
            {
                NS_LOG_LOGIC("Trigger Handover to cellId " << bestNeighbourCellId);
                NS_LOG_LOGIC("target cell RSRP " << (uint16_t)bestNeighbourRsrp);
                NS_LOG_LOGIC("serving cell RSRP " << (uint16_t)servingCellRsrp);

                // Inform gNB RRC about handover
                m_handoverManagementSapUser->TriggerHandover(rnti, bestNeighbourCellId);
            }
        }

    } // end of else of if (it1 == m_neighbourCellMeasures.end ())

} // end of EvaluateMeasurementReport

bool
NrA2A4RsrpHandoverAlgorithm::IsValidNeighbour(uint16_t cellId)
{
    NS_LOG_FUNCTION(this << cellId);

    /**
     * @todo In the future, this function can be expanded to validate whether the
     *       neighbour cell is a valid target cell, e.g., taking into account the
     *       NRT in ANR and whether it is a CSG cell with closed access.
     */

    return true;
}

void
NrA2A4RsrpHandoverAlgorithm::UpdateNeighbourMeasurements(uint16_t rnti,
                                                         uint16_t cellId,
                                                         uint8_t rsrp)
{
    NS_LOG_FUNCTION(this << rnti << cellId << (uint16_t)rsrp);
    auto it1 = m_neighbourCellMeasures.find(rnti);

    if (it1 == m_neighbourCellMeasures.end())
    {
        // insert a new UE entry
        MeasurementRow_t row;
        auto ret = m_neighbourCellMeasures.insert(std::pair<uint16_t, MeasurementRow_t>(rnti, row));
        NS_ASSERT(ret.second);
        it1 = ret.first;
    }

    NS_ASSERT(it1 != m_neighbourCellMeasures.end());
    Ptr<UeMeasure> neighbourCellMeasures;
    auto it2 = it1->second.find(cellId);

    if (it2 != it1->second.end())
    {
        neighbourCellMeasures = it2->second;
        neighbourCellMeasures->m_cellId = cellId;
        neighbourCellMeasures->m_rsrp = rsrp;
    }
    else
    {
        // insert a new cell entry
        neighbourCellMeasures = Create<UeMeasure>();
        neighbourCellMeasures->m_cellId = cellId;
        neighbourCellMeasures->m_rsrp = rsrp;
        it1->second[cellId] = neighbourCellMeasures;
    }

} // end of UpdateNeighbourMeasurements

} // end of namespace ns3
