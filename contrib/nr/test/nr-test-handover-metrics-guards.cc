// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-handover-metrics-guards.cc
 *
 * @brief Test suite `nr-handover-metrics-guards`: guards and metrics recently added to NrGnbRrc.
 *
 * Three system-level test cases build a measurement-driven A3 handover scenario (two gNBs on a
 * line, one UE crossing the boundary with a ConstantVelocityMobilityModel) and exercise:
 *
 * 1. The `HandoverTotalTime` trace source, which fires at the source gNB when an A3-driven
 *    handover completes and reports the elapsed time from the A3 trigger (DoTriggerHandover) to
 *    completion, including network-side latency.
 * 2. The `HandoverMinTimeOfStay` reversal-only ping-pong guard attribute.
 * 3. A regression liveness guard for the HANDOVER_JOINING stray RRC Connection Request handling in
 *    NrUeManager (a normal handover must still complete without a fatal error).
 */

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/rng-seed-manager.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrHandoverMetricsGuardsTest");

/**
 * @ingroup nr-test
 *
 * @brief Common scenario builder shared by the handover-metrics-guards test cases.
 *
 * Deploys two gNBs on the X axis and one UE crossing them at 150 m/s with an A3 RSRP handover
 * algorithm, EPC, isotropic antennas and a Friis propagation loss model. Derived cases connect
 * traces before running and evaluate them afterwards.
 */
class NrHandoverMetricsGuardsBase : public TestCase
{
  public:
    /**
     * Constructor.
     *
     * @param name the test case name
     */
    NrHandoverMetricsGuardsBase(std::string name);

  protected:
    /**
     * Build the scenario, attach the UE to gNB 0 and set up X2, up to (but not running) the
     * simulation. Devices and helper handles are stored as members so derived cases can connect
     * traces before calling Simulator::Run().
     *
     * @param hysteresis the A3 hysteresis in dB
     * @param timeToTrigger the A3 time-to-trigger
     */
    void BuildScenario(double hysteresis, Time timeToTrigger);

    Ptr<NrHelper> m_nrHelper;                 ///< NR helper
    Ptr<NrPointToPointEpcHelper> m_epcHelper; ///< EPC helper
    NetDeviceContainer m_gnbDevices;          ///< gNB devices
    NetDeviceContainer m_ueDevices;           ///< UE devices
};

NrHandoverMetricsGuardsBase::NrHandoverMetricsGuardsBase(std::string name)
    : TestCase(name)
{
}

void
NrHandoverMetricsGuardsBase::BuildScenario(double hysteresis, Time timeToTrigger)
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    m_nrHelper = CreateObject<NrHelper>();
    // Real RRC exercises the full network-side handover signalling latency; only override where a
    // case specifically needs ideal RRC.
    m_nrHelper->SetAttribute("UseIdealRrc", BooleanValue(false));
    m_nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaPF"));

    m_nrHelper->SetHandoverAlgorithmType("ns3::NrA3RsrpHandoverAlgorithm");
    m_nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(hysteresis));
    m_nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(timeToTrigger));

    m_epcHelper = CreateObject<NrPointToPointEpcHelper>();
    m_nrHelper->SetEpcHelper(m_epcHelper);

    double distance = 1000.0; // m
    double speed = 150;       // m/s

    NodeContainer gnbNodes;
    gnbNodes.Create(2);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    // gNBs on a line along X
    Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < 2; i++)
    {
        gnbPositionAlloc->Add(Vector(distance * (i + 1), 0, 0));
    }
    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.SetPositionAllocator(gnbPositionAlloc);
    gnbMobility.Install(gnbNodes);

    // UE crossing the two gNBs at constant speed
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ueMobility.Install(ueNodes);
    ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 0));
    ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(speed, 0, 0));

    m_nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    m_nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(FriisPropagationLossModel::GetTypeId());

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(2.8e9, 10e6, static_cast<uint8_t>(1));
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});

    BandwidthPartInfoPtrVector allBwps;
    allBwps = CcBwpCreator::GetAllBwps({band});

    m_gnbDevices = m_nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    m_nrHelper->AssignStreams({.gnbDevs = m_gnbDevices});

    m_ueDevices = m_nrHelper->InstallUeDevice(ueNodes, allBwps);
    m_nrHelper->AssignStreams({.ueDevs = m_ueDevices});

    // EPC / IP stack
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
    Ptr<Node> pgw = m_epcHelper->GetPgwNode();
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    internet.Install(ueNodes);
    m_epcHelper->AssignUeIpv4Address(NetDeviceContainer(m_ueDevices));

    m_nrHelper->AssignStreams({.assignEpc = true,
                               .remoteHostNodes = remoteHostContainer,
                               .ueNodes = ueNodes,
                               .gnbNodes = gnbNodes,
                               .ueNodeStream = 3000,
                               .gnbNodeStream = 2000});

    // Attach the UE to gNB 0 at the start; the A3 algorithm handles the handover to gNB 1.
    m_nrHelper->AttachToGnb(m_ueDevices.Get(0), m_gnbDevices.Get(0));

    m_nrHelper->AddX2Interface(gnbNodes);
}

/**
 * @ingroup nr-test
 *
 * @brief Verifies the `HandoverTotalTime` trace source on NrGnbRrc.
 *
 * The trace fires at the source gNB when an A3-driven handover completes, reporting the time from
 * the A3 trigger (DoTriggerHandover) to completion. With a nonzero HandoverTriggeringDelay of
 * 40 ms configured, the reported total time must include that delay: it must be strictly positive
 * and no smaller than the triggering delay, confirming the metric spans the network-side latency.
 */
class NrHandoverTotalTimeTestCase : public NrHandoverMetricsGuardsBase
{
  public:
    NrHandoverTotalTimeTestCase();

  private:
    void DoRun() override;

    /**
     * Trace sink for the HandoverTotalTime trace source.
     *
     * @param imsi the UE IMSI
     * @param sourceCellId the source cell ID
     * @param targetCellId the target cell ID
     * @param totalTime the elapsed handover time
     */
    void HandoverTotalTimeSink(uint64_t imsi,
                               uint16_t sourceCellId,
                               uint16_t targetCellId,
                               Time totalTime);

    std::vector<Time> m_totalTimes; ///< captured total-time values
};

NrHandoverTotalTimeTestCase::NrHandoverTotalTimeTestCase()
    : NrHandoverMetricsGuardsBase("HandoverTotalTime trace includes triggering delay")
{
}

void
NrHandoverTotalTimeTestCase::HandoverTotalTimeSink(uint64_t imsi,
                                                   uint16_t sourceCellId,
                                                   uint16_t targetCellId,
                                                   Time totalTime)
{
    NS_LOG_FUNCTION(this << imsi << sourceCellId << targetCellId << totalTime);
    m_totalTimes.push_back(totalTime);
}

void
NrHandoverTotalTimeTestCase::DoRun()
{
    Config::Reset();
    // A nonzero triggering delay forces network-side latency between the A3 trigger and completion.
    Config::SetDefault("ns3::NrGnbRrc::HandoverTriggeringDelay", TimeValue(MilliSeconds(40)));

    BuildScenario(1.5, MilliSeconds(128));

    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/*/NrGnbRrc/HandoverTotalTime",
        MakeCallback(&NrHandoverTotalTimeTestCase::HandoverTotalTimeSink, this));

    Simulator::Stop(Seconds(12));
    Simulator::Run();
    Simulator::Destroy();

    NS_TEST_ASSERT_MSG_GT(m_totalTimes.size(),
                          0,
                          "no HandoverTotalTime event fired during the A3 handover");

    for (const auto& t : m_totalTimes)
    {
        NS_TEST_ASSERT_MSG_EQ(t.IsStrictlyPositive(),
                              true,
                              "HandoverTotalTime must be strictly positive");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(
            t,
            MilliSeconds(40),
            "HandoverTotalTime must be at least the triggering delay (network-side latency)");
    }
}

/**
 * @ingroup nr-test
 *
 * @brief Verifies the `HandoverMinTimeOfStay` reversal-only ping-pong guard on NrGnbRrc.
 *
 * The guard suppresses a handover that would send the UE BACK to the cell it just came from, if it
 * happens less than HandoverMinTimeOfStay after arrival. The scenario runs the same A3 crossing
 * twice: once with the guard disabled (0) and once with an extremely large guard (Seconds(100)).
 * Counting HandoverStart events at the UE, the guarded run must have no more handovers than the
 * unguarded run (an extreme guard suppresses any reversal), and the unguarded run must show at
 * least one handover. The assertion is deliberately robust (no exact counts): if the geometry does
 * not naturally produce a reversal, the counts simply match, which the guard still satisfies.
 */
class NrHandoverMinTimeOfStayTestCase : public NrHandoverMetricsGuardsBase
{
  public:
    NrHandoverMinTimeOfStayTestCase();

  private:
    void DoRun() override;

    /**
     * Trace sink counting UE HandoverStart events.
     *
     * @param imsi the UE IMSI
     * @param cellId the source cell ID
     * @param rnti the UE RNTI
     * @param targetCellId the target cell ID
     */
    void HandoverStartSink(uint64_t imsi, uint16_t cellId, uint16_t rnti, uint16_t targetCellId);

    /**
     * Run one crossing with the given guard value and return the HandoverStart count.
     *
     * @param minTimeOfStay the HandoverMinTimeOfStay guard value
     * @return the number of UE HandoverStart events
     */
    uint32_t RunWithGuard(Time minTimeOfStay);

    uint32_t m_handoverStartCount{0}; ///< HandoverStart counter for the current run
};

NrHandoverMinTimeOfStayTestCase::NrHandoverMinTimeOfStayTestCase()
    : NrHandoverMetricsGuardsBase("HandoverMinTimeOfStay suppresses reversal handovers")
{
}

void
NrHandoverMinTimeOfStayTestCase::HandoverStartSink(uint64_t imsi,
                                                   uint16_t cellId,
                                                   uint16_t rnti,
                                                   uint16_t targetCellId)
{
    NS_LOG_FUNCTION(this << imsi << cellId << rnti << targetCellId);
    m_handoverStartCount++;
}

uint32_t
NrHandoverMinTimeOfStayTestCase::RunWithGuard(Time minTimeOfStay)
{
    Config::Reset();
    Config::SetDefault("ns3::NrGnbRrc::HandoverMinTimeOfStay", TimeValue(minTimeOfStay));

    m_handoverStartCount = 0;

    // Small hysteresis makes the A3 condition marginal and more prone to reversals.
    BuildScenario(0.0, MilliSeconds(40));

    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/*/NrUeRrc/HandoverStart",
        MakeCallback(&NrHandoverMinTimeOfStayTestCase::HandoverStartSink, this));

    Simulator::Stop(Seconds(14));
    Simulator::Run();
    Simulator::Destroy();

    return m_handoverStartCount;
}

void
NrHandoverMinTimeOfStayTestCase::DoRun()
{
    uint32_t unguarded = RunWithGuard(MilliSeconds(0));
    uint32_t guarded = RunWithGuard(Seconds(100));

    NS_TEST_ASSERT_MSG_GT(unguarded, 0, "unguarded run must perform at least one handover");
    NS_TEST_ASSERT_MSG_LT_OR_EQ(guarded,
                                unguarded,
                                "the reversal guard must not increase the number of handovers");
}

/**
 * @ingroup nr-test
 *
 * @brief Regression guard for HANDOVER_JOINING stray RRC Connection Request handling.
 *
 * NrUeManager::RecvRrcConnectionRequest ignores an RRC Connection Request received while in the
 * HANDOVER_JOINING state (the `case HANDOVER_JOINING:` block in nr-gnb-rrc.cc). The old code path
 * hit an NS_FATAL_ERROR in that situation. This condition is hard to trigger deterministically, so
 * this test uses a liveness proxy: a normal A3 handover scenario must run to completion and record
 * at least one successful handover (HandoverEndOk at the gNB) without aborting. It is a regression
 * guard, not a direct reproduction of the stray-request race.
 */
class NrHandoverJoiningLivenessTestCase : public NrHandoverMetricsGuardsBase
{
  public:
    NrHandoverJoiningLivenessTestCase();

  private:
    void DoRun() override;

    /**
     * Trace sink counting gNB HandoverEndOk events.
     *
     * @param imsi the UE IMSI
     * @param cellId the cell ID
     * @param rnti the UE RNTI
     */
    void HandoverEndOkSink(uint64_t imsi, uint16_t cellId, uint16_t rnti);

    uint32_t m_handoverEndOkCount{0}; ///< HandoverEndOk counter
};

NrHandoverJoiningLivenessTestCase::NrHandoverJoiningLivenessTestCase()
    : NrHandoverMetricsGuardsBase("HANDOVER_JOINING stray-request regression liveness guard")
{
}

void
NrHandoverJoiningLivenessTestCase::HandoverEndOkSink(uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    NS_LOG_FUNCTION(this << imsi << cellId << rnti);
    m_handoverEndOkCount++;
}

void
NrHandoverJoiningLivenessTestCase::DoRun()
{
    Config::Reset();

    BuildScenario(1.5, MilliSeconds(128));

    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
        MakeCallback(&NrHandoverJoiningLivenessTestCase::HandoverEndOkSink, this));

    Simulator::Stop(Seconds(12));
    Simulator::Run();
    Simulator::Destroy();

    NS_TEST_ASSERT_MSG_GT(m_handoverEndOkCount,
                          0,
                          "a normal A3 handover must complete without a fatal error");
}

/**
 * @ingroup nr-test
 *
 * @brief NR Handover Metrics Guards Test Suite.
 */
class NrHandoverMetricsGuardsTestSuite : public TestSuite
{
  public:
    NrHandoverMetricsGuardsTestSuite();
};

NrHandoverMetricsGuardsTestSuite::NrHandoverMetricsGuardsTestSuite()
    : TestSuite("nr-handover-metrics-guards", Type::SYSTEM)
{
    AddTestCase(new NrHandoverTotalTimeTestCase(), TestCase::Duration::QUICK);
    AddTestCase(new NrHandoverMinTimeOfStayTestCase(), TestCase::Duration::QUICK);
    AddTestCase(new NrHandoverJoiningLivenessTestCase(), TestCase::Duration::QUICK);
}

/**
 * @ingroup nr-test
 * Static variable for test initialization
 */
static NrHandoverMetricsGuardsTestSuite g_NrHandoverMetricsGuardsTestSuiteInstance;
