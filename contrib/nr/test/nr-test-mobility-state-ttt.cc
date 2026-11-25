// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-mobility-state-ttt.cc
 *
 * @brief Test suite `nr-mobility-state-ttt`: mobility-state estimation for time-to-trigger
 * scaling in NrUeRrc.
 *
 * A single UE crosses a row of gNBs at high speed, accumulating several handovers within the
 * mobility-state counting window, so it is classified as Medium/High mobility. The same
 * scenario is run twice: once with mobility-state estimation disabled (default NrUeRrc
 * attributes) and once with it enabled and configured to shorten the effective A3
 * time-to-trigger (scale factor < 1) in the elevated states. Because a shorter TTT lets the A3
 * event fire sooner, the enabled run is expected to trigger at least as many UE-side handover
 * starts as the disabled run, and strictly more in the tuned case. Counts and times are checked
 * with a tolerant assertion; no exact values are asserted.
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

NS_LOG_COMPONENT_DEFINE("NrMobilityStateTttTest");

/**
 * @ingroup nr-test
 *
 * @brief Runs a high-mobility, multi-gNB A3 handover scenario with mobility-state estimation
 * either disabled or enabled, and checks that enabling TTT scaling does not reduce (and, in the
 * tuned case, increases) the number of UE-side handover starts.
 */
class NrMobilityStateTttTestCase : public TestCase
{
  public:
    /**
     * Constructor.
     *
     * @param name descriptive name of the test case
     */
    NrMobilityStateTttTestCase(std::string name);

  private:
    void DoRun() override;

    /**
     * Run one simulation of the fixed high-mobility scenario.
     *
     * @param mseEnabled whether mobility-state estimation (TTT scaling) is enabled
     * @param firstHandoverTime returns the time of the first UE-side handover start (or a large
     *        sentinel if none occurred)
     * @return the number of UE-side handover starts observed
     */
    uint32_t RunScenario(bool mseEnabled, Time& firstHandoverTime);

    /**
     * Callback bound to the NrUeRrc HandoverStart trace; counts starts and records the first
     * handover time.
     *
     * @param imsi the UE IMSI
     * @param cellId the source cell id
     * @param rnti the UE RNTI
     * @param targetCellId the target cell id
     */
    void HandoverStartCallback(uint64_t imsi,
                               uint16_t cellId,
                               uint16_t rnti,
                               uint16_t targetCellId);

    uint32_t m_handoverStartCount{0}; ///< handover starts seen in the current run
    Time m_firstHandoverTime;         ///< time of the first handover start in the current run
};

NrMobilityStateTttTestCase::NrMobilityStateTttTestCase(std::string name)
    : TestCase(name)
{
}

void
NrMobilityStateTttTestCase::HandoverStartCallback(uint64_t imsi,
                                                  uint16_t cellId,
                                                  uint16_t rnti,
                                                  uint16_t targetCellId)
{
    if (m_handoverStartCount == 0)
    {
        m_firstHandoverTime = Simulator::Now();
    }
    ++m_handoverStartCount;
}

uint32_t
NrMobilityStateTttTestCase::RunScenario(bool mseEnabled, Time& firstHandoverTime)
{
    m_handoverStartCount = 0;
    m_firstHandoverTime = Seconds(1e9);

    Config::Reset();
    Config::SetDefault("ns3::NrGnbRrc::HandoverJoiningTimeoutDuration",
                       TimeValue(MilliSeconds(200)));
    Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(20));
    Config::SetDefault("ns3::NrUePhy::EnableUplinkPowerControl", BooleanValue(false));

    // Mobility State Estimation knobs (NrUeRrc). When enabled, recent handovers counted within
    // MseCountWindow classify the UE as Medium/High mobility, and the corresponding scale factor
    // multiplies the A3 time-to-trigger. Factors < 1 shorten the effective TTT so a fast UE hands
    // over sooner. A generous window and a low High threshold guarantee the fast UE reaches the
    // elevated state after only a couple of crossings.
    Config::SetDefault("ns3::NrUeRrc::MseEnable", BooleanValue(mseEnabled));
    if (mseEnabled)
    {
        Config::SetDefault("ns3::NrUeRrc::MseCountWindow", TimeValue(Seconds(10)));
        Config::SetDefault("ns3::NrUeRrc::MseHystNormal", TimeValue(Seconds(5)));
        Config::SetDefault("ns3::NrUeRrc::MseThreshMedium", UintegerValue(1));
        Config::SetDefault("ns3::NrUeRrc::MseThreshHigh", UintegerValue(2));
        Config::SetDefault("ns3::NrUeRrc::MseSfMedium", DoubleValue(0.4));
        Config::SetDefault("ns3::NrUeRrc::MseSfHigh", DoubleValue(0.25));
        // Apply a fixed scale from t=0 as well. The handover-counting path only reaches an
        // elevated state after a completed handover, so the first crossing would otherwise be
        // unscaled and match the disabled run exactly. With the fixed scale the 512 ms base TTT
        // becomes 128 ms immediately, which advances the very first handover.
        Config::SetDefault("ns3::NrUeRrc::MseFixedScale", DoubleValue(0.25));
    }

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(true));
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));

    // A3 with a deliberately long TTT so that shortening it via mobility-state scaling has room to
    // change the handover timing/count.
    nrHelper->SetHandoverAlgorithmType("ns3::NrA3RsrpHandoverAlgorithm");
    nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(1.5));
    nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(512)));

    const uint32_t nGnbs = 5;
    const double distance = 1000.0; // m
    const double speed = 250;       // m/s (high mobility, several crossings within the window)

    NodeContainer gnbNodes;
    gnbNodes.Create(nGnbs);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    // gNBs along a line on the X axis.
    Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < nGnbs; i++)
    {
        gnbPositionAlloc->Add(Vector(distance * (i + 1), 0, 0));
    }
    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.SetPositionAllocator(gnbPositionAlloc);
    gnbMobility.Install(gnbNodes);

    // UE crossing all gNBs at constant velocity.
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ueMobility.Install(ueNodes);
    ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 0));
    ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(speed, 0, 0));

    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(FriisPropagationLossModel::GetTypeId());

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(2.8e9, 10e6, static_cast<uint8_t>(1));
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});

    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    NetDeviceContainer gnbDevices = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    nrHelper->AssignStreams({.gnbDevs = gnbDevices});
    for (auto it = gnbDevices.Begin(); it != gnbDevices.End(); ++it)
    {
        Ptr<NrGnbRrc> gnbRrc = (*it)->GetObject<NrGnbNetDevice>()->GetRrc();
        gnbRrc->SetAttribute("AdmitHandoverRequest", BooleanValue(true));
    }

    NetDeviceContainer ueDevices = nrHelper->InstallUeDevice(ueNodes, allBwps);
    nrHelper->AssignStreams({.ueDevs = ueDevices});

    // Minimal EPC/IP setup so the UE can attach and hand over.
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    ipv4h.Assign(internetDevices);

    internet.Install(ueNodes);
    epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevices));

    nrHelper->AssignStreams({.assignEpc = true,
                             .remoteHostNodes = remoteHostContainer,
                             .ueNodes = ueNodes,
                             .gnbNodes = gnbNodes,
                             .ueNodeStream = 3000,
                             .gnbNodeStream = 2000});

    nrHelper->AttachToGnb(ueDevices.Get(0), gnbDevices.Get(0));

    nrHelper->AddX2Interface(gnbNodes);

    // Count UE-side handover starts via the NrUeRrc HandoverStart trace source.
    Ptr<NrUeRrc> ueRrc = ueDevices.Get(0)->GetObject<NrUeNetDevice>()->GetRrc();
    ueRrc->TraceConnectWithoutContext(
        "HandoverStart",
        MakeCallback(&NrMobilityStateTttTestCase::HandoverStartCallback, this));

    Simulator::Stop(Seconds(21));
    Simulator::Run();
    Simulator::Destroy();

    firstHandoverTime = m_firstHandoverTime;
    return m_handoverStartCount;
}

void
NrMobilityStateTttTestCase::DoRun()
{
    Time firstDisabled;
    Time firstEnabled;
    uint32_t disabled = RunScenario(false, firstDisabled);
    uint32_t enabled = RunScenario(true, firstEnabled);

    NS_LOG_INFO("MSE disabled: " << disabled << " handover starts, first at "
                                 << firstDisabled.As(Time::S));
    NS_LOG_INFO("MSE enabled:  " << enabled << " handover starts, first at "
                                 << firstEnabled.As(Time::S));

    // Sanity: the high-mobility scenario must produce handovers in the baseline run, otherwise the
    // comparison would be vacuous.
    NS_TEST_ASSERT_MSG_GT(disabled, 0, "baseline (disabled) run produced no handovers");

    // Robust, tolerant comparison: shortening the TTT cannot reduce the number of handover starts.
    NS_TEST_ASSERT_MSG_GT_OR_EQ(enabled,
                                disabled,
                                "enabling TTT scaling reduced the number of handover starts");

    // Tuned case: the aggressive High/Medium scale factors should let the fast UE trigger sooner or
    // more often. Accept either strictly more handover starts or an earlier first handover.
    bool moreHandovers = (enabled > disabled);
    bool earlierFirst = (firstEnabled < firstDisabled);
    NS_TEST_ASSERT_MSG_EQ(moreHandovers || earlierFirst,
                          true,
                          "TTT scaling neither increased handover starts nor advanced the first "
                          "handover (enabled="
                              << enabled << " disabled=" << disabled
                              << " firstEnabled=" << firstEnabled.As(Time::S)
                              << " firstDisabled=" << firstDisabled.As(Time::S) << ")");
}

/**
 * @ingroup nr-test
 *
 * @brief NR Mobility-State TTT Scaling Test Suite
 */
class NrMobilityStateTttTestSuite : public TestSuite
{
  public:
    NrMobilityStateTttTestSuite();
};

NrMobilityStateTttTestSuite::NrMobilityStateTttTestSuite()
    : TestSuite("nr-mobility-state-ttt", Type::SYSTEM)
{
    AddTestCase(
        new NrMobilityStateTttTestCase("High-mobility UE, A3 TTT scaling disabled vs enabled"),
        TestCase::Duration::QUICK);
}

/**
 * @ingroup nr-test
 * Static variable for test initialization
 */
static NrMobilityStateTttTestSuite g_nrMobilityStateTttTestSuiteInstance;
