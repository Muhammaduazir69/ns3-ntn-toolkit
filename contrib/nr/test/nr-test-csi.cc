// Copyright (c) 2024 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-csi.cc
 *
 * @brief Test suite (nr-test-csi) for CSI feedback processing and sub-band aware scheduling. It
 * creates a victim gNB/UE pair plus an interfering pair whose transmissions cover the whole band
 * or only its lower or upper half and toggle on and off during the run. A state machine driven
 * by CSI feedback, scheduling and PHY traces asserts that sub-band CQIs degrade on the
 * interfered part of the band, that the scheduler steers resource block allocations away from
 * interfered RBGs, that the error rate stays bounded once the link adapts, and that the CQI
 * recovers after the interferer stops. Cases sweep the CSI feedback sources (CSI-RS/CSI-IM,
 * PDSCH MIMO, SISO), 3GPP 2-bit sub-band CQI clamping, and the MCS derivation policy used by the
 * scheduler.
 */

#include "nr-test-csi.h"

#include "ns3/application-container.h"
#include "ns3/beam-manager.h"
#include "ns3/boolean.h"
#include "ns3/config.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/double.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ideal-beamforming-algorithm.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/nr-amc.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-gnb-rrc.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-interference.h"
#include "ns3/nr-json.hpp"
#include "ns3/nr-mac-scheduler-ns3.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/nr-spectrum-signal-parameters.h"
#include "ns3/nr-spectrum-value-helper.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/nr-ue-rrc.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <map>

nlohmann::json outputJson;

namespace ns3
{

NrCsiTestCase::NrCsiTestCase(NrCsiTestCase::NrCsiTestCaseParams params)
    : TestCase("NR CSI feedback test case"),
      m_params(params)
{
    // Check for valid interference pattern
    std::string pattern;
    int interferingNodes = m_params.m_interfPattern.size();
    switch (interferingNodes)
    {
    case 0:
        pattern = "no interference";
        break;
    case 1:
        if (m_params.m_interfPattern.at(0).front() == m_params.m_interfPattern.at(0).back())
        {
            pattern = "wideband interference";
        }
        else
        {
            pattern = m_params.m_interfPattern.at(0).front() == 0 ? "high half-band interference"
                                                                  : "low half-band interference";
        }
        break;
    default:
        NS_ABORT_MSG("Invalid number of interferers");
    }
    NS_ASSERT_MSG(interferingNodes < 3, "Unsupported number of orthogonal interferers");

    std::string csiFeedback;
    std::string mcsSource;
    switch (m_params.m_csiFeedbackFlags)
    {
    case CQI_PDSCH_MIMO | CQI_CSI_RS | CQI_CSI_IM:
        csiFeedback = "PDSCH MIMO + CSI-RS + CSI-IM";
        break;
    case CQI_CSI_RS | CQI_CSI_IM:
        csiFeedback = "CSI-RS + CSI-IM";
        break;
    case CQI_PDSCH_MIMO:
        csiFeedback = "PDSCH MIMO";
        break;
    case CQI_PDSCH_SISO:
        csiFeedback = "PDSCH SISO";
        break;
    default:
        NS_ABORT_MSG("enum to string not implemented");
    }
    switch (m_params.m_mcsCsiSource)
    {
    case NrMacSchedulerUeInfo::McsCsiSource::AVG_MCS:
        mcsSource = "Average allocated RBG MCS";
        break;
    case NrMacSchedulerUeInfo::McsCsiSource::AVG_SPEC_EFF:
        mcsSource = "Average allocated RBG spectral efficiency";
        break;
    case NrMacSchedulerUeInfo::McsCsiSource::AVG_SINR:
        mcsSource = "Average allocated RBG SINR";
        break;
    case NrMacSchedulerUeInfo::McsCsiSource::WIDEBAND_MCS:
        mcsSource = "Wideband MCS";
        break;
    default:
        NS_ABORT_MSG("enum to string not implemented");
    }
    // Save test description
    std::stringstream ss;
    ss << "InterfNodes=" << std::to_string(interferingNodes)
       << ", distInterferers=" << m_params.m_interfDistance
       << ", distUeGnb=" << m_params.m_ueGnbDistance << ", pattern=" << pattern
       << ", 3gpp sub-band CQI clamping=" << m_params.m_subbandCqiClamping << "\n"
       << ", MIMO feedback=" << m_params.m_enableCsiFeedback
       << ", CSI feedback source=" << csiFeedback << ", MCS computation based on=" << mcsSource;

    m_description = ss.str();

    // Create entries to hold measurements of each test case
    outputJson[m_description] = nlohmann::json{
        {"rxTb", std::vector<nlohmann::json>{}},
        {"csiFb", nlohmann::json{}},
        {"appState", nlohmann::json{}},
        {"ueThr", std::vector<nlohmann::json>{}},
    };
}

NrCsiTestCase::~NrCsiTestCase()
{
}

double
NrCsiTestCase::GetSlidingWindowErrorRate()
{
    double errorRate = std::transform_reduce(
        m_tbErrorSlidingWindow.begin(),
        m_tbErrorSlidingWindow.end(),
        0,
        [](auto a, auto b) { return a + b; },
        [](auto a) { return (int)a.second; });
    errorRate /= m_tbErrorSlidingWindow.size();
    return errorRate;
}

void
NrCsiTestCase::UeReception(RxPacketTraceParams params)
{
    nlohmann::json entry;
    entry["ts"] = Simulator::Now().GetNanoSeconds();
    entry["mcs"] = params.m_mcs;
    entry["rank"] = params.m_rank;
    entry["corrupted"] = params.m_corrupt;
    entry["assignedRbgs"] = params.m_rbAssignedNum;
    entry["cellid"] = params.m_cellId;
    outputJson[m_description]["rxTb"].push_back(entry);

    if (params.m_cellId == 1)
    {
        m_tbErrorSlidingWindow.emplace_back(Simulator::Now(), params.m_corrupt);
        if (m_tbErrorSlidingWindow.size() > 10)
        {
            m_tbErrorSlidingWindow.pop_front();
        }
        m_errorRateHistory.push_back(GetSlidingWindowErrorRate());
        if (m_errorRateHistory.size() > 10)
        {
            m_errorRateHistory.pop_front();
        }
        if (m_params.m_enableCsiFeedback & CQI_PDSCH_MIMO ||
            m_params.m_mcsCsiSource == NrMacSchedulerUeInfo::McsCsiSource::WIDEBAND_MCS ||
            m_csiSlidingWindow.empty())
        {
            return;
        }
        auto interferer = GetInterfererState();
        if (interferer.type == InterferenceType::LOWBAND_INTERFERENCE ||
            interferer.type == InterferenceType::HIGHBAND_INTERFERENCE)
        {
            switch (m_machineState)
            {
            case WAITING_FOR_INTERFERER_TO_STOP:
                // If interference is sub-band, we should have around half of RBGs allocated
                if (m_csiSlidingWindow.front().second.rank != params.m_rank)
                {
                    NS_TEST_ASSERT_MSG_EQ_TOL(params.m_rbAssignedNum,
                                              106 / 2,
                                              106 * 0.25,
                                              "Expected about half of the RBGs allocated under "
                                              "half-bandwidth interference");
                }
                break;
            case WAITING_FOR_INTERFERER_TO_START:
                // If no sub-band interference, we should have almost all RBGs allocated
                if (m_csiSlidingWindow.front().second.rank != params.m_rank)
                {
                    NS_TEST_ASSERT_MSG_EQ_TOL(
                        params.m_rbAssignedNum,
                        106,
                        106 * 0.1,
                        "Expected almost all RBGs allocated under no interference");
                }
                break;
            default:
                break;
            }
        }
    }
}

void
NrCsiTestCase::CsiFeedbackReceived(uint16_t cellId,
                                   uint16_t bwpId,
                                   const std::shared_ptr<NrMacSchedulerUeInfo>& ueInfo)
{
    std::stringstream ss;
    ss << ueInfo;
    std::string ueInfoStr = ss.str();
    if (!outputJson[m_description]["csiFb"].contains(ueInfoStr))
    {
        outputJson[m_description]["csiFb"][ueInfoStr] = std::vector<nlohmann::json>();
    }
    nlohmann::json entry;
    entry["ts"] = Simulator::Now().GetNanoSeconds();

    // Create fake entry
    if (ueInfo->m_dlSbMcsInfo.empty())
    {
        ueInfo->m_dlCqi.m_sbCqi = std::vector<uint8_t>(7, ueInfo->m_dlCqi.m_wbCqi);
    }
    entry["sbCqi"] = ueInfo->m_dlCqi.m_sbCqi;
    outputJson[m_description]["csiFb"][ueInfoStr].push_back(entry);

    // If we created a fake sub-band CQI entry based on wideband CQI for plotting purposes, we can
    // stop
    if (ueInfo->m_dlSbMcsInfo.empty() || (m_params.m_csiFeedbackFlags & CQI_PDSCH_MIMO) ||
        m_params.m_mcsCsiSource == NrMacSchedulerUeInfo::McsCsiSource::WIDEBAND_MCS)
    {
        return;
    }

    if (cellId == 1)
    {
        if (!ueInfo->m_dlCqi.m_sbCqi.empty())
        {
            m_csiSlidingWindow.push_back({Simulator::Now(),
                                          {ueInfo->m_dlCqi.m_wbCqi,
                                           ueInfo->m_dlCqi.m_sbCqi.front(),
                                           ueInfo->m_dlCqi.m_sbCqi.back(),
                                           ueInfo->m_dlRank}});
        }
        else
        {
            m_csiSlidingWindow.push_back({Simulator::Now(), {ueInfo->m_dlCqi.m_wbCqi, 255, 255}});
        }
        if (m_csiSlidingWindow.size() > 10)
        {
            m_csiSlidingWindow.pop_front();
        }
        const auto oldCsi = m_csiSlidingWindow.front().second;
        const auto newCsi = m_csiSlidingWindow.back().second;
        InterfererState interferer = GetInterfererState();
        bool interfered =
            !interferer.timestampAndState.empty() && interferer.timestampAndState.back().second;

        if (interfered && !m_errorRateHistory.empty())
        {
            if (m_machineState == WAITING_FOR_UP_TO_DATE_INTERFERED_CQI)
            {
                // While we are being interfered, error rates are supposed to be high or at least
                // increasing over time
                NS_TEST_ASSERT_MSG_GT_OR_EQ(
                    m_errorRateHistory.back(),
                    m_errorRateHistory.front(),
                    "Before receiving an up-to-date CQI, the error rate should continue going up");
                // Check if CSI was updated after interference began
                bool csiUpdated = newCsi.rank < oldCsi.rank;
                csiUpdated |= (interferer.type == InterferenceType::HIGHBAND_INTERFERENCE) &
                              (newCsi.highbandCqi < oldCsi.widebandCqi);
                csiUpdated |= (interferer.type == InterferenceType::LOWBAND_INTERFERENCE) &
                              (newCsi.lowbandCqi < oldCsi.widebandCqi);
                if (csiUpdated)
                {
                    // Determine if link adaptation actually happened according to the interference
                    // type This below assume only one interference is active at a time
                    switch (interferer.type)
                    {
                    case InterferenceType::HIGHBAND_INTERFERENCE:
                        NS_TEST_ASSERT_MSG_EQ((newCsi.highbandCqi < newCsi.lowbandCqi),
                                              true,
                                              "High band interferer isn't causing a significant "
                                              "CQI difference between high band and low band");
                        break;
                    case InterferenceType::LOWBAND_INTERFERENCE:
                        NS_TEST_ASSERT_MSG_EQ((newCsi.lowbandCqi < newCsi.highbandCqi),
                                              true,
                                              "Low band interferer isn't causing a significant CQI "
                                              "difference between high band and low band");
                        break;
                    case InterferenceType::WIDEBAND_INTERFERENCE:
                        NS_TEST_ASSERT_MSG_EQ(((newCsi.rank < oldCsi.rank) ||
                                               (newCsi.rank == oldCsi.rank &&
                                                (newCsi.lowbandCqi < oldCsi.lowbandCqi &&
                                                 newCsi.highbandCqi < oldCsi.highbandCqi))),
                                              true,
                                              "Wideband interferer is not causing the same "
                                              "interference on high and low bands");
                        break;
                    case InterferenceType::NO_INTERFERENCE:
                        NS_TEST_ASSERT_MSG_EQ((newCsi.lowbandCqi == newCsi.highbandCqi),
                                              true,
                                              "No interference case, both low and high bands "
                                              "should be equal or similar");
                        break;
                    default:
                        NS_ABORT_MSG("Unknown interference type");
                    }
                    StateMachineStep(LINK_ADAPTED_TO_INTERFERENCE);
                }
            }
            else if (m_machineState == WAITING_FOR_INTERFERER_TO_STOP)
            {
                if (oldCsi < newCsi)
                {
                    NS_TEST_ASSERT_MSG_LT_OR_EQ(m_errorRateHistory.back(),
                                                m_errorRateHistory.front(),
                                                "After receiving an up-to-date CQI, the error rate "
                                                "should start going down");
                }
            }
            else
            {
                NS_ABORT_MSG("Unexpected state");
            }
        }

        if (!interfered && (oldCsi != newCsi) &&
            m_machineState == WAITING_FOR_UP_TO_DATE_NON_INTERFERED_CQI)
        {
            StateMachineStep(LINK_ADAPTED_TO_NO_INTERFERENCE);
        }
    }
}

void
NrCsiTestCase::LogApplicationStateTrampoline(NrCsiTestCase* testCase,
                                             Ptr<OnOffApplication> app,
                                             bool beforeState,
                                             bool afterState)
{
    testCase->LogApplicationState(app, beforeState, afterState);
}

void
NrCsiTestCase::LogApplicationState(Ptr<OnOffApplication> app, bool beforeState, bool afterState)
{
    std::stringstream ss;
    ss << app;
    std::string appStr = ss.str();
    if (!outputJson[m_description]["appState"].contains(appStr))
    {
        outputJson[m_description]["appState"][appStr] = std::vector<nlohmann::json>();
    }
    nlohmann::json entry;
    entry["ts"] = Simulator::Now().GetNanoSeconds();
    entry["state"] = afterState;
    outputJson[m_description]["appState"][appStr].push_back(entry);

    m_interferers[m_interfAppToString[app]].timestampAndState.emplace_back(Simulator::Now(),
                                                                           afterState);
    if (!m_interfAppToString.at(app).empty())
    {
        StateMachineStep(afterState ? WAITING_FOR_UP_TO_DATE_INTERFERED_CQI
                                    : WAITING_FOR_UP_TO_DATE_NON_INTERFERED_CQI);
    }
}

void
NrCsiTestCase::LogThroughputUe0(Ptr<FlowMonitor> monitor, Time udpAppStartTime)
{
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
    if (!stats.empty())
    {
        double flowDuration = (Simulator::Now() - udpAppStartTime).GetSeconds();

        nlohmann::json entry;
        entry["ts"] = Simulator::Now().GetNanoSeconds();
        entry["thr"] = stats.begin()->second.rxBytes * 8.0 / flowDuration / 1000 / 1000;
        outputJson[m_description]["ueThr"].push_back(entry);
    }
    Simulator::Schedule(MilliSeconds(100),
                        &NrCsiTestCase::LogThroughputUe0,
                        this,
                        monitor,
                        udpAppStartTime);
}

void
NrCsiTestCase::StateMachineStep(MachineState nextState)
{
    // Ignore state machine if csi feedback is disabled
    if (!m_params.m_enableCsiFeedback || (m_params.m_csiFeedbackFlags & CQI_PDSCH_MIMO) ||
        m_params.m_mcsCsiSource == NrMacSchedulerUeInfo::McsCsiSource::WIDEBAND_MCS)
    {
        return;
    }

    // Waiting for transition
    if (nextState == m_machineState)
    {
        return;
    }

    switch (nextState)
    {
    case WAITING_FOR_INTERFERER_TO_START:
        break;
    case WAITING_FOR_UP_TO_DATE_INTERFERED_CQI:
        NS_TEST_ASSERT_MSG_EQ(m_machineState,
                              WAITING_FOR_INTERFERER_TO_START,
                              "Invalid machine state transition from " << m_machineState << " to "
                                                                       << nextState);
        break;
    case LINK_ADAPTED_TO_INTERFERENCE:
        NS_TEST_ASSERT_MSG_NE((m_machineState & WAITING_FOR_UP_TO_DATE_INTERFERED_CQI),
                              0,
                              "Invalid machine state transition from " << m_machineState << " to "
                                                                       << nextState);
        nextState = WAITING_FOR_INTERFERER_TO_STOP; // We skip straight to the next state
        break;
    case WAITING_FOR_INTERFERER_TO_STOP:
        break;
    case WAITING_FOR_UP_TO_DATE_NON_INTERFERED_CQI:
        NS_TEST_ASSERT_MSG_NE((m_machineState & WAITING_FOR_INTERFERER_TO_STOP),
                              0,
                              "Invalid machine state transition from " << m_machineState << " to "
                                                                       << nextState);
        break;
    case LINK_ADAPTED_TO_NO_INTERFERENCE:
        NS_TEST_ASSERT_MSG_NE((m_machineState & WAITING_FOR_UP_TO_DATE_NON_INTERFERED_CQI),
                              0,
                              "Invalid machine state transition from " << m_machineState << " to "
                                                                       << nextState);
        nextState = WAITING_FOR_INTERFERER_TO_START; // We skip straight to the next state
        break;
    default:
        NS_TEST_EXPECT_MSG_EQ(0, 1, "Unexpected state");
    }
    // Move from the old state to the next state
    m_machineState = nextState;
}

void
NrCsiTestCase::DoRun()
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(2);
    Config::SetDefault("ns3::NrGnbPhy::PowerAllocationType",
                       EnumValue<NrSpectrumValueHelper::PowerAllocationType>(
                           NrSpectrumValueHelper::UNIFORM_POWER_ALLOCATION_BW));
    Config::SetDefault("ns3::NrMacSchedulerNs3::McsCsiSource",
                       EnumValue<NrMacSchedulerUeInfo::McsCsiSource>(m_params.m_mcsCsiSource));
    Config::SetDefault("ns3::NrPmSearch::RankLimit", UintegerValue(4));
    Config::SetDefault("ns3::NrPmSearch::SubbandSize", UintegerValue(16));
    Config::SetDefault("ns3::NrPmSearch::SubbandCqiClamping",
                       BooleanValue(m_params.m_subbandCqiClamping));
    Config::SetDefault("ns3::NrPmSearchFull::CodebookType",
                       TypeIdValue(TypeId::LookupByName("ns3::NrCbTypeOneSp")));

    NrHelper::AntennaParams apUe;
    NrHelper::AntennaParams apGnb;
    apUe.antennaElem = "ns3::ThreeGppAntennaModel";
    apUe.nAntCols = 2;
    apUe.nAntRows = 2;
    apUe.nHorizPorts = 2;
    apUe.nVertPorts = 1;
    apUe.isDualPolarized = true;
    apGnb.antennaElem = "ns3::ThreeGppAntennaModel";
    apGnb.nAntCols = 4;
    apGnb.nAntRows = 2;
    apGnb.nHorizPorts = 2;
    apGnb.nVertPorts = 1;
    apGnb.isDualPolarized = true;

    // The polarization slant angle in degrees in case of x-polarized
    double polSlantAngleGnb = 0.0;
    double polSlantAngleUe = 0.0;
    // The bearing angles in degrees
    double bearingAngleGnb = 180.0;
    double bearingAngleUe = 0.0;

    // Traffic parameters
    Time udpAppStartTime = MilliSeconds(400);

    // Other simulation scenario parameters
    Time simTime = MilliSeconds(3000);
    uint16_t numerology = 0;
    double centralFrequency = 3.5e9;
    double bandwidth = 20e6;
    double txPowerGnb = 30; // dBm
    double txPowerUe = 20;  // dBm
    uint16_t updatePeriodMs = 0;
    std::string errorModel = "ns3::NrEesmIrT2";
    std::string scheduler = "ns3::NrMacSchedulerOfdmaRR";
    std::string beamformingMethod = "ns3::DirectPathBeamforming";

    // convert angle values into radians
    apUe.bearingAngle = bearingAngleUe * (M_PI / 180);
    apUe.polSlantAngle = polSlantAngleUe * (M_PI / 180);
    apGnb.bearingAngle = bearingAngleGnb * (M_PI / 180);
    apGnb.polSlantAngle = polSlantAngleGnb * (M_PI / 180);

    NS_ABORT_IF(centralFrequency < 0.5e9 && centralFrequency > 100e9);

    int interferingNodes = m_params.m_interfPattern.size();
    uint16_t pairsToCreate = 1 + interferingNodes;

    NodeContainer gnbContainer;
    gnbContainer.Create(pairsToCreate);
    NodeContainer ueContainer;
    ueContainer.Create(pairsToCreate);

    /**
     * We configure the mobility model to ConstantPositionMobilityModel.
     * The default topology is the following:
     *                                   UE2 gNB2
     *         UE0 gNB0                  UE1 gNB1
     *          |---|   m_ueGnbDistance   |---|
     *              |-------------------------| m_interfDistance
     */
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();

    // the positions each pair of gNB and UE
    for (int i = 0; i < interferingNodes + 1; i++)
    {
        positionAlloc->Add(Vector((i == 0 ? 0 : 1) * m_params.m_interfDistance, i, 10.0));
        positionAlloc->Add(
            Vector((i == 0 ? 0 : 1) * m_params.m_interfDistance - m_params.m_ueGnbDistance,
                   i,
                   1.5));
    }
    mobility.SetPositionAllocator(positionAlloc);

    // install mobility of the second pair of gNB and UE
    for (int i = 0; i < interferingNodes + 1; i++)
    {
        mobility.Install(gnbContainer.Get(i));
        mobility.Install(ueContainer.Get(i));
    }

    /**
     * Create the NR helpers that will be used to create and setup NR devices, spectrum, ...
     */
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);
    /**
     * Prepare spectrum. Prepare one operational band, containing
     * one component carrier, and a single bandwidth part
     * centered at the frequency specified by the input parameters.
     *
     *
     * The configured spectrum division is:
     * ------------Band--------------
     * ------------CC1----------------
     * ------------BWP1---------------
     */

    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, numCcPerBand);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    // Settings that strongly affect the CSI feedback
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(m_params.m_rlcBufferSize));
    if (m_params.m_enableCsiFeedback)
    {
        Config::SetDefault(
            "ns3::NrUePhy::AlphaCovMat",
            DoubleValue(m_params.m_interfCovMatAlpha)); // Control the averaging weight for temporal
                                                        // interference covariance matrix
    }
    nrHelper->SetAttribute("CsiFeedbackFlags", UintegerValue(m_params.m_csiFeedbackFlags));

    /**
     * Configure NrHelper, prepare most of the parameters that will be used in the simulation.
     */
    nrHelper->SetDlErrorModel(errorModel);
    nrHelper->SetUlErrorModel(errorModel);
    nrHelper->SetGnbDlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrHelper->SetGnbUlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName(scheduler));
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(TypeId::LookupByName(beamformingMethod)));
    // Core latency
    nrEpcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    /**
     * Configure gNb antenna
     */
    nrHelper->SetupGnbAntennas(apGnb);
    /**
     * Configure UE antenna
     */
    nrHelper->SetupUeAntennas(apUe);
    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(numerology));
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(txPowerGnb));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(txPowerUe));

    uint32_t bwpId = 0;
    // gNb routing between flow type and bandwidth part
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpId));
    // UE routing between flow type and bandwidth part
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpId));
    /**
     * Initialize channel and pathloss, plus other things inside band.
     */

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    // Set and configure the channel to the current band
    channelHelper->ConfigureFactories("UMi", "LOS", "ThreeGpp");
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(updatePeriodMs)));
    // channelHelper->SetChannelConditionModelAttribute("UpdatePeriod",
    // TimeValue(MilliSeconds(updatePeriodMs)));
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    channelHelper->AssignChannelsToBands({band});
    auto allBwps = CcBwpCreator::GetAllBwps({band});

    /**
     * Finally, create the gNB and the UE device.
     */
    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbContainer, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueContainer, allBwps);

    /**
     * Fix the random stream throughout the nr, propagation, and spectrum
     * modules classes. This configuration is extremely important for the
     * reproducibility of the results.
     */
    nrHelper->AssignStreams({.assignEpc = true, .gnbDevs = gnbNetDev, .ueDevs = ueNetDev});

    // Hookup transport block reception trace at measuring UE0
    Ptr<NrSpectrumPhy> ue0SpectrumPhy =
        DynamicCast<NrUeNetDevice>(ueNetDev.Get(0))->GetPhy(0)->GetSpectrumPhy();
    ue0SpectrumPhy->TraceConnectWithoutContext("RxPacketTraceUe",
                                               MakeCallback(&NrCsiTestCase::UeReception, this));
    Simulator::ScheduleDestroy([ue0SpectrumPhy, testCase = this]() {
        ue0SpectrumPhy->TraceDisconnectWithoutContext(
            "RxPacketTraceUe",
            MakeCallback(&NrCsiTestCase::UeReception, testCase));
    });
    if (ueContainer.GetN() > 1)
    {
        Ptr<NrSpectrumPhy> ue1SpectrumPhy =
            DynamicCast<NrUeNetDevice>(ueNetDev.Get(1))->GetPhy(0)->GetSpectrumPhy();
        ue1SpectrumPhy->TraceConnectWithoutContext("RxPacketTraceUe",
                                                   MakeCallback(&NrCsiTestCase::UeReception, this));
        Simulator::ScheduleDestroy([ue1SpectrumPhy, testCase = this]() {
            ue1SpectrumPhy->TraceDisconnectWithoutContext(
                "RxPacketTraceUe",
                MakeCallback(&NrCsiTestCase::UeReception, testCase));
        });
    }

    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::NrGnbNetDevice/BandwidthPartMap/"
                                  "*/MacScheduler/$ns3::NrMacSchedulerNs3/CsiFeedbackReceived",
                                  MakeCallback(&NrCsiTestCase::CsiFeedbackReceived, this));
    Simulator::ScheduleDestroy([testCase = this]() {
        Config::DisconnectWithoutContext(
            "/NodeList/*/DeviceList/*/$ns3::NrGnbNetDevice/BandwidthPartMap/"
            "*/MacScheduler/$ns3::NrMacSchedulerNs3/CsiFeedbackReceived",
            MakeCallback(&NrCsiTestCase::CsiFeedbackReceived, testCase));
    });

    // create the Internet and install the IP stack on the UEs
    // get SGW/PGW and create a single RemoteHost
    auto [remoteHost, remoteHostIpv4Address] =
        nrEpcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0.000));

    InternetStackHelper internet;
    Ipv4InterfaceContainer ueIpIface;
    internet.Install(ueContainer);
    nrHelper->AssignStreams({.ueNodes = ueContainer, .ueNodeStream = 1000});
    ueIpIface = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));

    for (int i = 0; i < interferingNodes + 1; i++)
    {
        // attach each UE to its gNB according to desired scenario
        nrHelper->AttachToGnb(ueNetDev.Get(i), gnbNetDev.Get(i));

        if (i > 0)
        {
            // Set notched mask for interfering gNB to restrict its interference to specific
            // sub-bands
            Ptr<NrMacSchedulerNs3> schedulerBwp1 =
                DynamicCast<NrMacSchedulerNs3>(NrHelper::GetScheduler(gnbNetDev.Get(i), 0));
            schedulerBwp1->SetDlNotchedRbgMask(m_params.m_interfPattern.at(i - 1));
            schedulerBwp1->SetUlNotchedRbgMask(m_params.m_interfPattern.at(i - 1));
            // Increase TxPower of interferers, since they are farther away
            auto gnb = DynamicCast<NrGnbNetDevice>(gnbNetDev.Get(i));
            gnb->GetPhy(0)->SetTxPower(txPowerGnb + 20);
        }
    }

    /**
     * Install DL traffic part.
     */
    uint16_t dlPort = 1234;
    ApplicationContainer serverApps;
    // The sink will always listen to the specified ports
    UdpServerHelper dlPacketSink(dlPort);
    // The server, that is the application which is listening, is installed in the UE
    serverApps.Add(dlPacketSink.Install(ueContainer));
    /**
     * Configure attributes for the CBR traffic generator, using user-provided
     * parameters
     */
    ObjectFactory dlClient;
    dlClient.SetTypeId(OnOffApplication::GetTypeId());
    dlClient.Set("PacketSize", UintegerValue(1000));
    dlClient.Set("DataRate", DataRateValue(DataRate("400Mbps")));

    /**
     * Configure attributes for ON_OFF applications, used by interferers
     */
    // The QoS flow that will carry the traffic
    NrQosFlow flow(NrQosFlow::NGBR_LOW_LAT_EMBB);

    // The filter for the traffic
    Ptr<NrQosRule> dlRule = Create<NrQosRule>();
    NrQosRule::PacketFilter dlPktFilter;
    dlPktFilter.localPortStart = dlPort;
    dlPktFilter.localPortEnd = dlPort;
    dlRule->Add(dlPktFilter);

    /**
     * Let's install the applications!
     */
    ApplicationContainer clientApps;

    for (uint32_t i = 0; i < ueContainer.GetN(); ++i)
    {
        Ptr<Node> ue = ueContainer.Get(i);
        Ptr<NetDevice> ueDevice = ueNetDev.Get(i);
        Address ueAddress = ueIpIface.GetAddress(i);

        // The client, who is transmitting, is installed in the remote host,
        // with destination address set to the address of the UE
        switch (i)
        {
        // Cause overlapping and non-overlapping interference temporally
        // Interf1 = ___---___---___---___---
        // Interf2 = ____----____----____----
        case 1: // First interferer pair
            dlClient.Set("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.240]"));
            dlClient.Set("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.240]"));
            break;
        case 2: // Second interferer pair
            dlClient.Set("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.320]"));
            dlClient.Set("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.320]"));
            break;
        default: // Measuring UE transmits all the time
            dlClient.Set("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            dlClient.Set("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
            break;
        }

        dlClient.Set("Remote",
                     AddressValue(InetSocketAddress(Ipv4Address::ConvertFrom(ueAddress), dlPort)));
        Ptr<OnOffApplication> app = dlClient.Create<OnOffApplication>();
        app->AssignStreams(7000 + i * 10);
        app->TraceConnectWithoutContext(
            "OnOffState",
            MakeBoundCallback(&NrCsiTestCase::LogApplicationStateTrampoline, this, app));
        Simulator::ScheduleDestroy([app, testCase = this]() {
            app->TraceDisconnectWithoutContext(
                "OnOffState",
                MakeBoundCallback(&NrCsiTestCase::LogApplicationStateTrampoline, testCase, app));
        });
        remoteHost->AddApplication(app);
        clientApps.Add(app);
        clientApps.Get(i)->SetStartTime(udpAppStartTime +
                                        (i == 0 ? NanoSeconds(0) : MilliSeconds(100)));

        // Activate a dedicated bearer for the traffic
        nrHelper->ActivateDedicatedQosFlow(ueDevice, flow, dlRule);
        if (i > 0)
        {
            std::stringstream ss;
            ss << app;
            m_interfAppToString[app] = ss.str();
            bool front = m_params.m_interfPattern.at(i - 1).front();
            bool back = m_params.m_interfPattern.at(i - 1).back();
            InterferenceType type = InterferenceType::NO_INTERFERENCE;
            if (front && back)
            {
                type = InterferenceType::WIDEBAND_INTERFERENCE;
            }
            if (front && !back)
            {
                type = InterferenceType::LOWBAND_INTERFERENCE;
            }
            if (!front && back)
            {
                type = InterferenceType::HIGHBAND_INTERFERENCE;
            }
            m_interferers[m_interfAppToString[app]] = {};
            m_interferers[m_interfAppToString[app]].type = type;
            m_interferers[m_interfAppToString[app]].timestampAndState.emplace_back(Simulator::Now(),
                                                                                   false);
        }
    }

    // start UDP server and client apps
    serverApps.Start(udpAppStartTime);
    serverApps.Stop(simTime);
    clientApps.Stop(simTime);

    FlowMonitorHelper flowmonHelper;
    NodeContainer endpointNodes;
    endpointNodes.Add(remoteHost);
    endpointNodes.Add(ueContainer);

    Ptr<ns3::FlowMonitor> monitor = flowmonHelper.Install(endpointNodes);
    monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(20));

    // Schedule change to RLC buffers in interfered nodes after RRC connection is properly setup
    if (gnbContainer.GetN() > 1)
    {
        Simulator::Schedule(MilliSeconds(300), [ueNode = gnbContainer.GetN()]() {
            NS_ASSERT(Config::SetFailSafe(
                "/NodeList/0/DeviceList/*/$ns3::NrGnbNetDevice/NrGnbRrc/UeMap/*/"
                "DataRadioBearerMap/*/NrRlc/$ns3::NrRlcUm/MaxTxBufferSize",
                UintegerValue(999999999))); // Unbounded RLC buffer for gNB0
            NS_ASSERT(Config::SetFailSafe(
                "/NodeList/" + std::to_string(ueNode) +
                    "/DeviceList/*/$ns3::NrUeNetDevice/NrUeRrc/DataRadioBearerMap/"
                    "*/NrRlc/$ns3::NrRlcUm/MaxTxBufferSize",
                UintegerValue(999999999))); // Unbounded RLC buffer for UE0
        });
    }
    Simulator::Schedule(udpAppStartTime,
                        &NrCsiTestCase::LogThroughputUe0,
                        this,
                        monitor,
                        Time(udpAppStartTime));

    Simulator::Stop(simTime);
    Simulator::Run();

    // Print per-flow statistics
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    for (auto i = stats.begin(); i != stats.end(); ++i)
    {
        auto rxPackets = i->second.rxPackets;

        // We are observing only the first UE at this time
        if (i == stats.begin())
        {
            NS_TEST_EXPECT_MSG_NE(rxPackets, 0, "Expected received packets");
        }
    }

    // Update output file for each additional test case
    std::ofstream ofs;
    ofs.open(GetParent()->GetName() + "-output.json", std::ofstream::out);
    ofs << outputJson.dump(2);
    ofs.close();

    Simulator::Destroy();
}

/**
 * @ingroup test
 *
 * Test CSI-RS, CSI-IM and PDSCH based CSI feedback when a UE loses its
 * connection. After a radio link failure, the UE goes back to the idle cell
 * search without an RNTI, while the gNB keeps it registered for CSI-RS. The
 * gNB used to transmit CSI-RS towards RNTI 0, which any UE without an RNTI
 * accepted, possibly without having processed the DL control of the slot,
 * scheduling the CSI-IM check with a negative delay and aborting the
 * simulation. The test checks that the gNB transmits CSI-RS, but never
 * towards RNTI 0, and that the UE loses its connection as intended.
 */
class NrCsiRsAfterRadioLinkFailureTestCase : public TestCase
{
  public:
    NrCsiRsAfterRadioLinkFailureTestCase()
        : TestCase("CSI-RS, CSI-IM and PDSCH feedback with a UE losing its connection")
    {
    }

  private:
    void DoRun() override;
};

void
NrCsiRsAfterRadioLinkFailureTestCase::DoRun()
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
    Config::SetDefault("ns3::NrPmSearch::RankLimit", UintegerValue(4));
    Config::SetDefault("ns3::NrPmSearch::SubbandSize", UintegerValue(16));
    Config::SetDefault("ns3::NrPmSearchFull::CodebookType",
                       TypeIdValue(TypeId::LookupByName("ns3::NrCbTypeOneSp")));
    // Fast radio link failure detection
    Config::SetDefault("ns3::NrUeRrc::N310", UintegerValue(1));
    Config::SetDefault("ns3::NrUeRrc::N311", UintegerValue(1));
    Config::SetDefault("ns3::NrUeRrc::T310", TimeValue(MilliSeconds(100)));
    Config::SetDefault("ns3::NrUePhy::NumQoutEvalSf", UintegerValue(10));
    Config::SetDefault("ns3::NrUePhy::NumQinEvalSf", UintegerValue(10));

    NodeContainer gnbContainer(1);
    NodeContainer ueContainer(2);
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbContainer);
    mobility.Install(ueContainer);
    gnbContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 10));
    for (uint32_t i = 0; i < ueContainer.GetN(); i++)
    {
        ueContainer.Get(i)->GetObject<MobilityModel>()->SetPosition(Vector(-20, 5.0 * i, 1.5));
    }

    auto nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    auto idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(true));
    nrHelper->SetAttribute("CsiFeedbackFlags",
                           UintegerValue(CQI_PDSCH_MIMO | CQI_CSI_RS | CQI_CSI_IM));
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));

    NrHelper::AntennaParams apUe;
    apUe.antennaElem = "ns3::ThreeGppAntennaModel";
    apUe.nAntCols = 2;
    apUe.nAntRows = 2;
    apUe.nHorizPorts = 2;
    apUe.nVertPorts = 1;
    apUe.isDualPolarized = true;
    NrHelper::AntennaParams apGnb = apUe;
    apGnb.nAntCols = 4;
    apGnb.bearingAngle = M_PI;
    nrHelper->SetupGnbAntennas(apGnb);
    nrHelper->SetupUeAntennas(apUe);
    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(0));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 20e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    auto channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "LOS", "ThreeGpp");
    channelHelper->AssignChannelsToBands({band});
    auto allBwps = CcBwpCreator::GetAllBwps({band});

    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbContainer, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueContainer, allBwps);
    nrHelper->AssignStreams({.assignEpc = true, .gnbDevs = gnbNetDev, .ueDevs = ueNetDev});

    InternetStackHelper internet;
    internet.Install(ueContainer);
    nrEpcHelper->AssignUeIpv4Address(ueNetDev);
    for (uint32_t i = 0; i < ueNetDev.GetN(); i++)
    {
        nrHelper->AttachToGnb(ueNetDev.Get(i), gnbNetDev.Get(0));
    }

    // The second UE jumps out of coverage once connected, so it detects a radio
    // link failure and returns to the idle cell search, still registered for
    // CSI-RS at the gNB
    bool backToCellSearch = false;
    auto ueRrc = DynamicCast<NrUeNetDevice>(ueNetDev.Get(1))->GetRrc();
    ueRrc->TraceConnectWithoutContext(
        "StateTransition",
        MakeCallback(
            +[](bool* idle, uint64_t, uint16_t, uint16_t, NrUeRrc::State, NrUeRrc::State state) {
                *idle |= state == NrUeRrc::IDLE_CELL_SEARCH;
            })
            .Bind(&backToCellSearch));
    // It then returns into coverage, finds the cell again and waits for its
    // system information without an RNTI
    auto ueMobility = ueContainer.Get(1)->GetObject<MobilityModel>();
    Simulator::Schedule(MilliSeconds(200),
                        [ueMobility]() { ueMobility->SetPosition(Vector(15000, 0, 1.5)); });
    Simulator::Schedule(MilliSeconds(400),
                        [ueMobility]() { ueMobility->SetPosition(Vector(-20, 5, 1.5)); });

    // Count the CSI-RS transmissions, and those towards RNTI 0
    std::pair<uint32_t, uint32_t> csiRsCount{0, 0};
    auto channel = DynamicCast<NrGnbNetDevice>(gnbNetDev.Get(0))
                       ->GetPhy(0)
                       ->GetSpectrumPhy()
                       ->GetSpectrumChannel();
    channel->TraceConnectWithoutContext(
        "TxSigParams",
        MakeCallback(+[](std::pair<uint32_t, uint32_t>* count,
                         Ptr<SpectrumSignalParameters> params) {
            if (auto csiRs = DynamicCast<NrSpectrumSignalParametersCsiRs>(params))
            {
                count->first++;
                count->second += csiRs->rnti == 0 ? 1 : 0;
            }
        }).Bind(&csiRsCount));

    Simulator::Stop(MilliSeconds(800));
    Simulator::Run();

    NS_TEST_EXPECT_MSG_GT(csiRsCount.first, 0, "The gNB should transmit CSI-RS");
    NS_TEST_EXPECT_MSG_EQ(csiRsCount.second,
                          0,
                          "The gNB should not transmit CSI-RS towards a UE without an RNTI");

    NS_TEST_EXPECT_MSG_EQ(backToCellSearch,
                          true,
                          "The second UE should lose its connection and search for a cell");
    NS_TEST_EXPECT_MSG_EQ(DynamicCast<NrUeNetDevice>(ueNetDev.Get(0))->GetRrc()->GetState(),
                          NrUeRrc::CONNECTED_NORMALLY,
                          "The first UE should stay connected");
    Simulator::Destroy();
}

/**
 * @ingroup test
 *
 * Test that the CSI-RS and the beams follow a UE that connects to another cell. The
 * helper used to register a UE only at the gNB of AttachToGnb(). After a handover or
 * a radio link failure, the new gNB of the UE then transmitted no CSI-RS towards it,
 * so the UE kept computing its CSI with the last CSI-RS of its previous cell, and the
 * beamforming kept steering the UE towards its previous gNB. The test moves a UE to a
 * second gNB, either by an X2 handover or by a radio link failure followed by a new
 * connection, and checks that only the target gNB transmits CSI-RS towards it, that
 * the UE processes them (including the CSI-IM that they trigger), and that the UE
 * beam points to the target gNB.
 */
class NrCsiRsAfterCellChangeTestCase : public TestCase
{
  public:
    /**
     * Constructor.
     *
     * @param handover Whether the UE changes cell by an X2 handover, or by a radio
     *        link failure and a new connection.
     */
    NrCsiRsAfterCellChangeTestCase(bool handover)
        : TestCase(std::string("CSI-RS, CSI-IM and beamforming follow a UE that ") +
                   (handover ? "is handed over" : "reconnects to another cell after an RLF")),
          m_handover(handover)
    {
    }

  private:
    void DoRun() override;

    bool m_handover; ///< Whether the UE changes cell by an X2 handover
};

void
NrCsiRsAfterCellChangeTestCase::DoRun()
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
    Config::SetDefault("ns3::NrPmSearch::RankLimit", UintegerValue(4));
    Config::SetDefault("ns3::NrPmSearch::SubbandSize", UintegerValue(16));
    Config::SetDefault("ns3::NrPmSearchFull::CodebookType",
                       TypeIdValue(TypeId::LookupByName("ns3::NrCbTypeOneSp")));
    // Fast radio link failure detection for the connection-loss variant. The handover
    // variant disables it instead (long T310): the commanded handover sends the UE to a
    // cell whose DL ctrl is weaker than the source's at the UE position (both gNB panels
    // face +x, so the UE sits in the back lobe of the target), and radio link monitoring
    // would tear the new connection down before the CSI-RS and beam checks run. Radio
    // link monitoring is not the subject of this test case.
    Config::SetDefault("ns3::NrUeRrc::N310", UintegerValue(1));
    Config::SetDefault("ns3::NrUeRrc::N311", UintegerValue(1));
    Config::SetDefault("ns3::NrUeRrc::T310",
                       m_handover ? TimeValue(Seconds(2)) : TimeValue(MilliSeconds(100)));
    Config::SetDefault("ns3::NrUePhy::NumQoutEvalSf", UintegerValue(10));
    Config::SetDefault("ns3::NrUePhy::NumQinEvalSf", UintegerValue(10));
    const Time cellChangeTime = MilliSeconds(300);
    // Margin for the UE to connect to the target cell before the checks
    const Time afterCellChange = cellChangeTime + MilliSeconds(m_handover ? 100 : 500);
    const Time stopTime = afterCellChange + MilliSeconds(400);

    NodeContainer gnbContainer(2);
    NodeContainer ueContainer(1);
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbContainer);
    mobility.Install(ueContainer);
    auto ueMobility = ueContainer.Get(0)->GetObject<MobilityModel>();
    gnbContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 10));
    if (m_handover)
    {
        gnbContainer.Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(60, 0, 10));
        // Closer to the target gNB, so that the handover does not depend on the shadowing
        ueMobility->SetPosition(Vector(45, 5, 1.5));
    }
    else
    {
        // The UE jumps out of the coverage of the first gNB, next to the second one
        gnbContainer.Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(15000, 0, 10));
        ueMobility->SetPosition(Vector(20, 5, 1.5));
        Simulator::Schedule(cellChangeTime,
                            [ueMobility]() { ueMobility->SetPosition(Vector(14980, 5, 1.5)); });
    }

    auto nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    auto idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(true));
    nrHelper->SetAttribute("CsiFeedbackFlags",
                           UintegerValue(CQI_PDSCH_MIMO | CQI_CSI_RS | CQI_CSI_IM));
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));

    NrHelper::AntennaParams apUe;
    apUe.antennaElem = "ns3::ThreeGppAntennaModel";
    apUe.nAntCols = 2;
    apUe.nAntRows = 2;
    apUe.nHorizPorts = 2;
    apUe.nVertPorts = 1;
    apUe.isDualPolarized = true;
    NrHelper::AntennaParams apGnb = apUe;
    apGnb.nAntCols = 4;
    nrHelper->SetupGnbAntennas(apGnb);
    nrHelper->SetupUeAntennas(apUe);
    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(0));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 20e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    auto channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "LOS", "ThreeGpp");
    channelHelper->AssignChannelsToBands({band});
    auto allBwps = CcBwpCreator::GetAllBwps({band});

    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbContainer, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueContainer, allBwps);
    nrHelper->AssignStreams({.assignEpc = true, .gnbDevs = gnbNetDev, .ueDevs = ueNetDev});

    InternetStackHelper internet;
    internet.Install(ueContainer);
    nrEpcHelper->AssignUeIpv4Address(ueNetDev);
    nrHelper->AttachToGnb(ueNetDev.Get(0), gnbNetDev.Get(0));
    if (m_handover)
    {
        nrHelper->AddX2Interface(gnbContainer);
        nrHelper->HandoverRequest(cellChangeTime,
                                  ueNetDev.Get(0),
                                  gnbNetDev.Get(0),
                                  gnbNetDev.Get(1));
    }

    const auto sourceCellId = DynamicCast<NrGnbNetDevice>(gnbNetDev.Get(0))->GetCellId();
    const auto targetCellId = DynamicCast<NrGnbNetDevice>(gnbNetDev.Get(1))->GetCellId();
    auto uePhy = DynamicCast<NrUeNetDevice>(ueNetDev.Get(0))->GetPhy(0);

    // CSI-RS transmissions per cell after the cell change; the UE is the only one
    std::map<uint16_t, uint32_t> csiRsTx;
    auto channel = uePhy->GetSpectrumPhy()->GetSpectrumChannel();
    channel->TraceConnectWithoutContext(
        "TxSigParams",
        MakeCallback(+[](std::map<uint16_t, uint32_t>* count,
                         Time after,
                         Ptr<SpectrumSignalParameters> params) {
            auto csiRs = DynamicCast<NrSpectrumSignalParametersCsiRs>(params);
            if (csiRs && Simulator::Now() > after)
            {
                (*count)[csiRs->cellId]++;
            }
        }).Bind(&csiRsTx, afterCellChange));

    // CSI-RS processed by the UE after the cell change
    uint32_t csiRsRx = 0;
    auto csiRsProcessor = Create<NrMimoChunkProcessor>();
    csiRsProcessor->AddCallback(
        MakeCallback(+[](uint32_t* count, Time after, const std::vector<MimoSignalChunk>&) {
            if (Simulator::Now() > after)
            {
                (*count)++;
            }
        }).Bind(&csiRsRx, afterCellChange));
    uePhy->GetSpectrumPhy()->AddCsiRsMimoChunkProcessor(csiRsProcessor);

    // CSI-IM measured by the UE after the cell change
    uint32_t csiImRx = 0;
    auto csiImProcessor = Create<NrMimoChunkProcessor>();
    csiImProcessor->AddCallback(
        MakeCallback(+[](uint32_t* count, Time after, const std::vector<MimoSignalChunk>&) {
            if (Simulator::Now() > after)
            {
                (*count)++;
            }
        }).Bind(&csiImRx, afterCellChange));
    uePhy->GetSpectrumPhy()->AddCsiImMimoChunkProcessor(csiImProcessor);

    Simulator::Stop(stopTime);
    Simulator::Run();

    NS_TEST_ASSERT_MSG_EQ(uePhy->GetCellId(), targetCellId, "The UE should change cell");
    NS_TEST_ASSERT_MSG_EQ(DynamicCast<NrUeNetDevice>(ueNetDev.Get(0))->GetRrc()->GetState(),
                          NrUeRrc::CONNECTED_NORMALLY,
                          "The UE should be connected to the target cell");
    NS_TEST_EXPECT_MSG_EQ(csiRsTx[sourceCellId],
                          0,
                          "The source gNB should stop the CSI-RS towards the UE");
    NS_TEST_EXPECT_MSG_GT(csiRsTx[targetCellId],
                          0,
                          "The target gNB should transmit CSI-RS towards the UE");
    NS_TEST_EXPECT_MSG_GT(csiRsRx, 0, "The UE should process the CSI-RS of the target gNB");
    NS_TEST_EXPECT_MSG_GT(csiImRx, 0, "The UE should measure CSI-IM in the target cell");

    auto ueBeamManager = uePhy->GetSpectrumPhy()->GetBeamManager();
    const auto towardsTarget = ueBeamManager->GetBeamformingVector(gnbNetDev.Get(1));
    const auto towardsSource = ueBeamManager->GetBeamformingVector(gnbNetDev.Get(0));
    NS_TEST_EXPECT_MSG_EQ((towardsTarget != towardsSource),
                          true,
                          "The UE should have a beam towards each gNB");
    NS_TEST_EXPECT_MSG_EQ((ueBeamManager->GetCurrentBeamformingVector() == towardsTarget),
                          true,
                          "The UE beam should point to the target gNB");
    Simulator::Destroy();
}

/**
 * @ingroup test
 *
 * Test the order of the CSI-RS transmissions towards the UEs sharing a CSI-RS
 * offset. The gNB used to keep these UEs in a set ordered by pointer, so the
 * order of their CSI-RS within the slot depended on the memory layout. The test
 * makes the device allocation order the reverse of the IMSI order, puts all UEs
 * on the same offset, and checks that every slot with CSI-RS transmits it
 * towards all UEs, in increasing IMSI order.
 */
class NrCsiRsOrderTestCase : public TestCase
{
  public:
    NrCsiRsOrderTestCase()
        : TestCase("CSI-RS towards the UEs of the same offset is ordered by IMSI")
    {
    }

  private:
    void DoRun() override;
};

void
NrCsiRsOrderTestCase::DoRun()
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
    const uint32_t numUes = 6;

    NodeContainer gnbContainer(1);
    NodeContainer ueContainer(numUes);
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbContainer);
    mobility.Install(ueContainer);
    gnbContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 10));
    for (uint32_t i = 0; i < ueContainer.GetN(); i++)
    {
        ueContainer.Get(i)->GetObject<MobilityModel>()->SetPosition(Vector(20, 5.0 * i, 1.5));
    }

    // The IMSI is the node ID, so installing the devices from the last node
    // allocates them in the reverse IMSI order
    NodeContainer reversedUes;
    for (uint32_t i = numUes; i > 0; i--)
    {
        reversedUes.Add(ueContainer.Get(i - 1));
    }

    auto nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    auto idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(true));
    nrHelper->SetAttribute("CsiFeedbackFlags", UintegerValue(CQI_PDSCH_MIMO | CQI_CSI_RS));
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));
    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(0));
    // A single DL slot per CSI-RS period, so all UEs share the same offset
    nrHelper->SetGnbPhyAttribute("Pattern", StringValue("DL|UL|UL|UL|UL|"));
    nrHelper->SetGnbPhyAttribute("CsiRsPeriodicity", UintegerValue(5));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 20e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    auto channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "LOS", "ThreeGpp");
    channelHelper->AssignChannelsToBands({band});
    auto allBwps = CcBwpCreator::GetAllBwps({band});

    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbContainer, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(reversedUes, allBwps);
    nrHelper->AssignStreams({.assignEpc = true, .gnbDevs = gnbNetDev, .ueDevs = ueNetDev});

    InternetStackHelper internet;
    internet.Install(ueContainer);
    nrEpcHelper->AssignUeIpv4Address(ueNetDev);
    for (uint32_t i = 0; i < ueNetDev.GetN(); i++)
    {
        nrHelper->AttachToGnb(ueNetDev.Get(i), gnbNetDev.Get(0));
    }

    // Record the RNTI of each CSI-RS, grouped by slot
    std::map<int64_t, std::vector<uint16_t>> csiRsRntisPerSlot;
    auto channel = DynamicCast<NrGnbNetDevice>(gnbNetDev.Get(0))
                       ->GetPhy(0)
                       ->GetSpectrumPhy()
                       ->GetSpectrumChannel();
    channel->TraceConnectWithoutContext(
        "TxSigParams",
        MakeCallback(+[](std::map<int64_t, std::vector<uint16_t>>* rntis,
                         Ptr<SpectrumSignalParameters> params) {
            if (auto csiRs = DynamicCast<NrSpectrumSignalParametersCsiRs>(params))
            {
                (*rntis)[Simulator::Now().GetMilliSeconds()].push_back(csiRs->rnti);
            }
        }).Bind(&csiRsRntisPerSlot));

    Simulator::Stop(MilliSeconds(300));
    Simulator::Run();

    std::map<uint16_t, uint64_t> rntiToImsi;
    for (uint32_t i = 0; i < ueNetDev.GetN(); i++)
    {
        auto ueDev = DynamicCast<NrUeNetDevice>(ueNetDev.Get(i));
        rntiToImsi[ueDev->GetPhy(0)->GetRnti()] = ueDev->GetImsi();
    }
    NS_TEST_ASSERT_MSG_EQ(rntiToImsi.size(), numUes, "Every UE should have its own RNTI");

    uint32_t fullSlots = 0;
    for (const auto& [slot, rntis] : csiRsRntisPerSlot)
    {
        std::vector<uint64_t> imsis;
        for (auto rnti : rntis)
        {
            NS_TEST_ASSERT_MSG_EQ(rntiToImsi.contains(rnti), true, "CSI-RS towards unknown RNTI");
            imsis.push_back(rntiToImsi.at(rnti));
        }
        NS_TEST_EXPECT_MSG_EQ(std::is_sorted(imsis.begin(), imsis.end()),
                              true,
                              "CSI-RS at " << slot << " ms is not in increasing IMSI order");
        fullSlots += imsis.size() == numUes ? 1 : 0;
    }
    NS_TEST_EXPECT_MSG_GT(fullSlots, 0, "No slot transmitted CSI-RS towards all UEs");
    Simulator::Destroy();
}

/**
 * @ingroup test
 *
 * Test CSI-RS and CSI-IM feedback with a propagation delay. The gNB transmits
 * the CSI-RS at the end of the DL control, so with a propagation delay the UE
 * receives it after the DL control of the slot ended, on the UE timeline. The
 * UE checks whether to measure CSI-IM at the end of the DL control, which used
 * to be scheduled with a negative delay in this case, aborting the simulation.
 * The test checks that the UE processes the CSI-RS and stays connected.
 */
class NrCsiRsPropagationDelayTestCase : public TestCase
{
  public:
    NrCsiRsPropagationDelayTestCase()
        : TestCase("CSI-RS and CSI-IM feedback with a propagation delay")
    {
    }

  private:
    void DoRun() override;
};

void
NrCsiRsPropagationDelayTestCase::DoRun()
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
    Config::SetDefault("ns3::NrPmSearch::RankLimit", UintegerValue(4));
    Config::SetDefault("ns3::NrPmSearch::SubbandSize", UintegerValue(16));
    Config::SetDefault("ns3::NrPmSearchFull::CodebookType",
                       TypeIdValue(TypeId::LookupByName("ns3::NrCbTypeOneSp")));

    NodeContainer gnbContainer(1);
    NodeContainer ueContainer(1);
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbContainer);
    mobility.Install(ueContainer);
    gnbContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 10));
    ueContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(20, 5, 1.5));

    auto nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    auto idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(true));
    nrHelper->SetAttribute("CsiFeedbackFlags", UintegerValue(CQI_CSI_RS | CQI_CSI_IM));
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));

    NrHelper::AntennaParams apUe;
    apUe.antennaElem = "ns3::ThreeGppAntennaModel";
    apUe.nAntCols = 2;
    apUe.nAntRows = 2;
    apUe.nHorizPorts = 2;
    apUe.nVertPorts = 1;
    apUe.isDualPolarized = true;
    NrHelper::AntennaParams apGnb = apUe;
    apGnb.nAntCols = 4;
    nrHelper->SetupGnbAntennas(apGnb);
    nrHelper->SetupUeAntennas(apUe);
    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(0));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 20e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    auto channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "LOS", "ThreeGpp");
    channelHelper->AssignChannelsToBands({band});
    auto allBwps = CcBwpCreator::GetAllBwps({band});

    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbContainer, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueContainer, allBwps);
    nrHelper->AssignStreams({.assignEpc = true, .gnbDevs = gnbNetDev, .ueDevs = ueNetDev});

    InternetStackHelper internet;
    internet.Install(ueContainer);
    nrEpcHelper->AssignUeIpv4Address(ueNetDev);
    nrHelper->AttachToGnb(ueNetDev.Get(0), gnbNetDev.Get(0));

    auto uePhy = DynamicCast<NrUeNetDevice>(ueNetDev.Get(0))->GetPhy(0);
    uePhy->GetSpectrumPhy()->GetSpectrumChannel()->SetPropagationDelayModel(
        CreateObject<ConstantSpeedPropagationDelayModel>());

    uint32_t csiRsRx = 0;
    auto csiRsProcessor = Create<NrMimoChunkProcessor>();
    csiRsProcessor->AddCallback(
        MakeCallback(+[](uint32_t* count, const std::vector<MimoSignalChunk>&) {
            (*count)++;
        }).Bind(&csiRsRx));
    uePhy->GetSpectrumPhy()->AddCsiRsMimoChunkProcessor(csiRsProcessor);

    Simulator::Stop(MilliSeconds(400));
    Simulator::Run();

    NS_TEST_ASSERT_MSG_EQ(DynamicCast<NrUeNetDevice>(ueNetDev.Get(0))->GetRrc()->GetState(),
                          NrUeRrc::CONNECTED_NORMALLY,
                          "The UE should be connected");
    NS_TEST_EXPECT_MSG_GT(csiRsRx, 0, "The UE should process the CSI-RS");
    Simulator::Destroy();
}

namespace
{
using Duration = TestCase::Duration;

// Interference patterns
const std::vector<bool> wbInterf(106, true);
const std::vector<bool> hbInterf = [] {
    std::vector<bool> pattern(106, true);
    std::fill(pattern.begin(), pattern.begin() + 16 * 4, false);
    return pattern;
}();
const std::vector<bool> lbInterf = [] {
    std::vector<bool> pattern(106, true);
    std::fill(pattern.rbegin(), pattern.rbegin() + 16 * 4, false);
    return pattern;
}();

std::vector<NrCsiTestCase::NrCsiTestCaseParams>
GetCsiRsImParams()
{
    // clang-format off
    using MCS = NrMacSchedulerUeInfo::McsCsiSource;
    return {
        ////  |---D1---|
        //// UE0 GNB0 UE1 GNB1
        //// |-D2-|
        ////
        //// Interference patterns
        /// Wide band  ------------------
        /// High band  ________----------
        /// Low band   ----------________
        //                                                     RLC buffer size
        //                 Interference moving average weight     |
        //                                                  |     |     MIMO feedback
        //                           Sub-band CQI clamping  |     |     |
        //                      D1   D2  Interference    V  V     V     v    CSI feedback       MCS CSI source
        //
        // Test CSI-RS plus CSI-IM feedback under no interference, or wideband/half-bandwidth interference
        {      Duration::QUICK, 200, 20,         {}, false, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::AVG_MCS},
        {      Duration::QUICK, 200, 20, {wbInterf}, false, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::AVG_MCS},
        {  Duration::EXTENSIVE, 200, 20, {hbInterf}, false, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::AVG_MCS},
        {      Duration::QUICK, 200, 20, {hbInterf}, false, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::AVG_SPEC_EFF},
        {      Duration::QUICK, 200, 20, {hbInterf}, false, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::AVG_SINR},
        {  Duration::EXTENSIVE, 200, 20, {lbInterf}, false, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::AVG_MCS},
        {  Duration::EXTENSIVE, 200, 20, {lbInterf}, false, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::AVG_SPEC_EFF},
        {  Duration::EXTENSIVE, 200, 20, {lbInterf}, false, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::AVG_SINR},

        // Test with 3GPP 2-bit clamping (sub-band CQI must be within wideband CQI [-1,+2] range)
        {  Duration::EXTENSIVE, 200, 20,         {},  true, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::AVG_MCS},
        {  Duration::EXTENSIVE, 200, 20, {wbInterf},  true, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::AVG_MCS},
        {  Duration::EXTENSIVE, 200, 20, {hbInterf},  true, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::AVG_MCS},
        {      Duration::QUICK, 200, 20, {lbInterf},  true, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::AVG_MCS},
    };
    // clang-format on
}

std::vector<NrCsiTestCase::NrCsiTestCaseParams>
GetPdschParams()
{
    // clang-format off
    using MCS = NrMacSchedulerUeInfo::McsCsiSource;
    return {
        // Test with PDSCH MIMO
        {  Duration::EXTENSIVE, 200, 20, {wbInterf},  true, 1, 70000, true, CQI_PDSCH_MIMO, MCS::AVG_MCS},
        {      Duration::QUICK, 200, 20, {hbInterf},  true, 1, 70000, true, CQI_PDSCH_MIMO, MCS::AVG_MCS},

        // Test with PDSCH MIMO, CSI-RS and CSI-IM
        {      Duration::QUICK, 200, 20, {hbInterf},  true, 1, 70000, true, CQI_PDSCH_MIMO | CQI_CSI_RS | CQI_CSI_IM, MCS::AVG_MCS},

        // Test without MIMO
        {  Duration::EXTENSIVE, 200, 20, {wbInterf},  true, 1, 70000, false, CQI_PDSCH_SISO, MCS::AVG_MCS},
        {  Duration::EXTENSIVE, 200, 20, {hbInterf},  true, 1, 70000, false, CQI_PDSCH_SISO, MCS::AVG_MCS},
        {      Duration::QUICK, 200, 20, {lbInterf},  true, 1, 70000, false, CQI_PDSCH_SISO, MCS::AVG_MCS},
    };
    // clang-format on
}

std::vector<NrCsiTestCase::NrCsiTestCaseParams>
GetWidebandParams()
{
    // clang-format off
    using MCS = NrMacSchedulerUeInfo::McsCsiSource;
    return {
        // Test legacy scheduling with wideband CQI/MCS without clamping
        {  Duration::EXTENSIVE, 200, 20,         {}, false, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::WIDEBAND_MCS},
        {  Duration::EXTENSIVE, 200, 20, {wbInterf}, false, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::WIDEBAND_MCS},
        {  Duration::EXTENSIVE, 200, 20, {hbInterf}, false, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::WIDEBAND_MCS},
        {  Duration::EXTENSIVE, 200, 20, {lbInterf}, false, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::WIDEBAND_MCS},

        // Test legacy scheduling with wideband CQI/MCS with clamping
        {      Duration::QUICK, 200, 20,         {},  true, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::WIDEBAND_MCS},
        {  Duration::EXTENSIVE, 200, 20, {wbInterf},  true, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::WIDEBAND_MCS},
        {  Duration::EXTENSIVE, 200, 20, {hbInterf},  true, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::WIDEBAND_MCS},
        {      Duration::QUICK, 200, 20, {lbInterf},  true, 1, 70000, true, CQI_CSI_RS | CQI_CSI_IM, MCS::WIDEBAND_MCS},
    };
    // clang-format on
}

} // namespace

NrCsiTestSuite::NrCsiTestSuite(const std::string& name,
                               const std::vector<NrCsiTestCase::NrCsiTestCaseParams>& params)
    : TestSuite(name)
{
    for (const auto& param : params)
    {
        AddTestCase(new NrCsiTestCase(param), param.m_duration);
    }
}

NrCsiMobilityTestSuite::NrCsiMobilityTestSuite()
    : TestSuite("nr-test-csi-mobility")
{
    AddTestCase(new NrCsiRsAfterRadioLinkFailureTestCase(), Duration::QUICK);
    AddTestCase(new NrCsiRsAfterCellChangeTestCase(true), Duration::QUICK);
    AddTestCase(new NrCsiRsAfterCellChangeTestCase(false), Duration::QUICK);
    AddTestCase(new NrCsiRsPropagationDelayTestCase(), Duration::QUICK);
    AddTestCase(new NrCsiRsOrderTestCase(), Duration::QUICK);
}

// Allocate an instance of each TestSuite
static NrCsiTestSuite g_nrCsiRsImTestSuite("nr-test-csi-rs-im", GetCsiRsImParams());
static NrCsiTestSuite g_nrCsiPdschTestSuite("nr-test-csi-pdsch", GetPdschParams());
static NrCsiTestSuite g_nrCsiWidebandTestSuite("nr-test-csi-wideband", GetWidebandParams());
static NrCsiMobilityTestSuite g_nrCsiMobilityTestSuite;

} // namespace ns3
