// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-tr36839-handover-failure.cc
 *
 * @brief Test suite `nr-tr36839-handover-failure`: exercises the NrUeRrc
 * "Tr36839HandoverFailure" attribute, which models the TR 36.839 (5.3.2) too-late
 * handover: a handover command that arrives while the source radio link is already
 * below Qout (modelled as a running T310) fails the handover instead of rescuing
 * the link.
 *
 * The scenario makes the T310-vs-command ordering exact rather than a tuning race:
 * - The source gNB sits at the origin and the target 15 km away. At 0.4 s the UE is
 *   teleported next to the target, so the source link collapses and T310 starts --
 *   the same device the nr-radio-link-failure suite uses.
 * - At 1.2 s, while that T310 is still pending (it runs for 2 s), the handover
 *   command is delivered by a manual NrHelper::HandoverRequest. The feature lives in
 *   the UE RRC reconfiguration handler, so it is agnostic to how the handover was
 *   triggered, and a scheduled request pins the delivery instant.
 * - Ideal RRC is used so the command is delivered by direct function call. This is
 *   essential: over a real-RRC link this badly degraded, the command would never be
 *   delivered and the branch under test could never execute.
 *
 * With the attribute enabled the UE declares radio link failure; with it disabled
 * (legacy behaviour) the late command rescues the link and the handover completes.
 * The test asserts that enabling the feature never yields a better outcome than
 * disabling it (>= as many RLF events, <= as many HandoverEndOk successes), and that
 * the enabled run actually reaches the below-Qout failure branch (at least one RLF)
 * while the disabled run completes at least one handover.
 */

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/udp-client-server-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrTr36839HandoverFailureTest");

/**
 * @ingroup nr-test
 *
 * @brief Result of a single scenario run: the counts of radio link failures and
 * successful handovers observed at the RRC trace sinks.
 */
struct HandoverOutcome
{
    uint32_t rlfCount{0};           ///< number of RadioLinkFailure trace events
    uint32_t handoverEndOkCount{0}; ///< number of gNB HandoverEndOk trace events
};

/**
 * @ingroup nr-test
 *
 * @brief Test the NrUeRrc "Tr36839HandoverFailure" attribute in an A3-driven
 * handover scenario where the handover command tends to arrive while T310 is
 * running on the source link.
 */
class NrTr36839HandoverFailureTestCase : public TestCase
{
  public:
    /**
     * Constructor.
     *
     * @param tr36839Enabled value of the NrUeRrc "Tr36839HandoverFailure"
     * attribute to use for the "feature" run of this case
     */
    NrTr36839HandoverFailureTestCase(bool tr36839Enabled);

  private:
    /**
     * Build the test case name string.
     *
     * @param tr36839Enabled whether the TR 36.839 handover failure feature is enabled
     * @return the name string
     */
    static std::string BuildNameString(bool tr36839Enabled);

    void DoRun() override;

    /**
     * Run the crossing-UE scenario once with a given value of the
     * "Tr36839HandoverFailure" attribute and report the observed outcome.
     *
     * @param tr36839Enabled value of the NrUeRrc "Tr36839HandoverFailure" attribute
     * @return the observed handover outcome (RLF and HandoverEndOk counts)
     */
    HandoverOutcome RunScenario(bool tr36839Enabled);

    /**
     * Trace sink for the NrUeRrc "RadioLinkFailure" trace source.
     *
     * @param context trace context string
     * @param imsi the IMSI of the UE
     * @param cellId the cell ID
     * @param rnti the RNTI of the UE
     */
    void RadioLinkFailureCallback(std::string context,
                                  uint64_t imsi,
                                  uint16_t cellId,
                                  uint16_t rnti);

    /**
     * Trace sink for the NrGnbRrc "HandoverEndOk" trace source.
     *
     * @param context trace context string
     * @param imsi the IMSI of the UE
     * @param cellId the cell ID
     * @param rnti the RNTI of the UE
     */
    void HandoverEndOkCallback(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti);

    bool m_tr36839Enabled;     ///< feature value for this case
    HandoverOutcome m_outcome; ///< outcome accumulator for the current run
};

std::string
NrTr36839HandoverFailureTestCase::BuildNameString(bool tr36839Enabled)
{
    std::ostringstream oss;
    oss << "Tr36839HandoverFailure=" << (tr36839Enabled ? "true" : "false");
    return oss.str();
}

NrTr36839HandoverFailureTestCase::NrTr36839HandoverFailureTestCase(bool tr36839Enabled)
    : TestCase(BuildNameString(tr36839Enabled)),
      m_tr36839Enabled(tr36839Enabled)
{
}

void
NrTr36839HandoverFailureTestCase::RadioLinkFailureCallback(std::string context,
                                                           uint64_t imsi,
                                                           uint16_t cellId,
                                                           uint16_t rnti)
{
    NS_LOG_FUNCTION(this << imsi << cellId << rnti);
    ++m_outcome.rlfCount;
}

void
NrTr36839HandoverFailureTestCase::HandoverEndOkCallback(std::string context,
                                                        uint64_t imsi,
                                                        uint16_t cellId,
                                                        uint16_t rnti)
{
    NS_LOG_FUNCTION(this << imsi << cellId << rnti);
    ++m_outcome.handoverEndOkCount;
}

HandoverOutcome
NrTr36839HandoverFailureTestCase::RunScenario(bool tr36839Enabled)
{
    m_outcome = HandoverOutcome();

    Config::Reset();

    // Keep the scenario deterministic across the two runs of this case.
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
    RngSeedManager::ResetNextStreamIndex();

    // Disable uplink power control so the source link degrades purely with distance.
    Config::SetDefault("ns3::NrUePhy::EnableUplinkPowerControl", BooleanValue(false));
    Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(20));

    // Radio link failure detection, using the same shortened timers as the
    // nr-radio-link-failure test suite: one out-of-sync indication starts T310, which then
    // runs for a full second.
    Config::SetDefault("ns3::NrUeRrc::N310", UintegerValue(1));
    Config::SetDefault("ns3::NrUeRrc::N311", UintegerValue(1));
    Config::SetDefault("ns3::NrUeRrc::T310", TimeValue(Seconds(2)));

    // The feature under test. The companion graded threshold keeps its 0 ms default, so any
    // pending T310 at command time qualifies as "source below Qout" (the binary form).
    Config::SetDefault("ns3::NrUeRrc::Tr36839HandoverFailure", BooleanValue(tr36839Enabled));

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    // Ideal RRC delivers the handover command by direct function call rather than over the
    // air. That is what makes this test deterministic: the command still reaches the UE while
    // the source link is dead, which is exactly the "too-late handover" the feature models. On
    // a real-RRC link that bad, the command would simply never be delivered and the branch
    // under test could never execute.
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(true));
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));

    // No measurement-driven handover: the handover is issued manually at a chosen instant so
    // the T310-vs-command ordering is exact instead of a tuning race. The feature lives in the
    // UE RRC reconfiguration handler, so it is agnostic to how the handover was triggered.
    nrHelper->SetHandoverAlgorithmType("ns3::NrNoOpHandoverAlgorithm");

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    // The source gNB sits at the origin and the target 15 km away, next to the position the UE
    // jumps to. The same "teleport far away" device the nr-radio-link-failure suite uses to
    // drive the source link below Qout without relying on marginal cell-edge SINR.
    const Vector sourceGnbPos(0, 0, 0);
    const Vector targetGnbPos(15000, 0, 0);
    const Vector ueStartPos(100, 0, 1.5);
    const Vector ueJumpPos(14990, 0, 1.5);

    NodeContainer gnbNodes;
    gnbNodes.Create(2);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator>();
    gnbPositionAlloc->Add(sourceGnbPos);
    gnbPositionAlloc->Add(targetGnbPos);
    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.SetPositionAllocator(gnbPositionAlloc);
    gnbMobility.Install(gnbNodes);

    // The UE starts served by the source gNB and is teleported next to the target later on.
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    ueMobility.Install(ueNodes);
    ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(ueStartPos);

    // Isotropic antennas and Friis propagation for a clean distance-driven link.
    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(FriisPropagationLossModel::GetTypeId());

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(2.8e9, 10e6, static_cast<uint8_t>(1));
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});

    BandwidthPartInfoPtrVector allBwps;
    allBwps = CcBwpCreator::GetAllBwps({band});

    NetDeviceContainer gnbDevices = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    nrHelper->AssignStreams({.gnbDevs = gnbDevices});

    NetDeviceContainer ueDevices = nrHelper->InstallUeDevice(ueNodes, allBwps);
    nrHelper->AssignStreams({.ueDevs = ueDevices});

    // Internet and a single remote host.
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
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces =
        epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevices));

    nrHelper->AssignStreams({.assignEpc = true,
                             .remoteHostNodes = remoteHostContainer,
                             .ueNodes = ueNodes,
                             .gnbNodes = gnbNodes,
                             .ueNodeStream = 3000,
                             .gnbNodeStream = 2000});

    // Attach to the source gNB; the handover to the target is issued manually below.
    nrHelper->AttachToGnb(ueDevices.Get(0), gnbDevices.Get(0));

    // A dedicated DL bearer keeps traffic flowing so out-of-sync indications
    // accumulate as the source link degrades.
    uint16_t dlPort = 10001;
    UdpClientHelper dlClientHelper(ueIpIfaces.GetAddress(0), dlPort);
    dlClientHelper.SetAttribute("Interval", TimeValue(MilliSeconds(10)));
    dlClientHelper.SetAttribute("MaxPackets", UintegerValue(1000000));
    dlClientHelper.SetAttribute("PacketSize", UintegerValue(100));
    ApplicationContainer clientApps = dlClientHelper.Install(remoteHost);

    PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(Ipv4Address::GetAny(), dlPort));
    ApplicationContainer serverApps = dlPacketSinkHelper.Install(ueNodes.Get(0));

    Ptr<NrQosRule> rule = Create<NrQosRule>();
    NrQosRule::PacketFilter dlpf;
    dlpf.localPortStart = dlPort;
    dlpf.localPortEnd = dlPort;
    rule->Add(dlpf);
    NrQosFlow flow(NrQosFlow::NGBR_VIDEO_TCP_DEFAULT);
    nrHelper->ActivateDedicatedQosFlow(ueDevices.Get(0), flow, rule);

    serverApps.Start(Seconds(0.1));
    clientApps.Start(Seconds(0.1));

    nrHelper->AddX2Interface(gnbNodes);

    // Drive the exact ordering the feature keys off. At 0.4 s the UE is teleported 15 km from
    // the source, so out-of-sync indications accumulate and T310 starts. At 0.6 s -- while that
    // T310 is still pending, well before its 1 s expiry -- the handover command is delivered.
    Simulator::Schedule(Seconds(0.4),
                        &MobilityModel::SetPosition,
                        ueNodes.Get(0)->GetObject<MobilityModel>(),
                        ueJumpPos);
    nrHelper->HandoverRequest(Seconds(1.2), ueDevices.Get(0), gnbDevices.Get(0), gnbDevices.Get(1));

    // Connect the RLF (UE) and HandoverEndOk (gNB) trace sinks.
    Config::Connect(
        "/NodeList/*/DeviceList/*/NrUeRrc/RadioLinkFailure",
        MakeCallback(&NrTr36839HandoverFailureTestCase::RadioLinkFailureCallback, this));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                    MakeCallback(&NrTr36839HandoverFailureTestCase::HandoverEndOkCallback, this));

    // Long enough to cover the teleport (0.4 s), the handover command (0.6 s) and the full T310
    // window that follows, including the reconnection the UE attempts afterwards.
    Simulator::Stop(Seconds(4));
    Simulator::Run();

    HandoverOutcome outcome = m_outcome;
    Simulator::Destroy();
    return outcome;
}

void
NrTr36839HandoverFailureTestCase::DoRun()
{
    NS_LOG_FUNCTION(this << GetName());

    // Run the identical scenario twice: once with the feature disabled (legacy
    // behaviour, the reference) and once with the value under test for this case.
    HandoverOutcome disabled = RunScenario(false);
    HandoverOutcome feature = RunScenario(m_tr36839Enabled);

    if (!m_tr36839Enabled)
    {
        // Both runs share the disabled configuration: they must be identical.
        NS_TEST_ASSERT_MSG_EQ(feature.rlfCount,
                              disabled.rlfCount,
                              "disabled runs should be deterministic and identical");
        NS_TEST_ASSERT_MSG_EQ(feature.handoverEndOkCount,
                              disabled.handoverEndOkCount,
                              "disabled runs should be deterministic and identical");
        // Legacy behaviour: the late command rescues the link, so the handover
        // completes in the tuned scenario.
        NS_TEST_ASSERT_MSG_GT_OR_EQ(disabled.handoverEndOkCount,
                                    1,
                                    "legacy behaviour should complete at least one handover");
        return;
    }

    // Robust cross-run assertion: enabling the feature must never yield a better
    // handover outcome than disabling it in the identical scenario.
    NS_TEST_ASSERT_MSG_GT_OR_EQ(feature.rlfCount,
                                disabled.rlfCount,
                                "enabling Tr36839HandoverFailure must not reduce RLF events");
    NS_TEST_ASSERT_MSG_LT_OR_EQ(feature.handoverEndOkCount,
                                disabled.handoverEndOkCount,
                                "enabling Tr36839HandoverFailure must not increase successful "
                                "handovers");

    // The scenario is tuned to exercise the below-Qout branch: with the feature
    // enabled at least one RLF must occur, while the disabled reference completes
    // at least one handover. This demonstrates the feature changed behaviour.
    NS_TEST_ASSERT_MSG_GT_OR_EQ(feature.rlfCount,
                                1,
                                "with the feature enabled and the source below Qout at command "
                                "time, at least one RLF is expected");
    NS_TEST_ASSERT_MSG_GT_OR_EQ(disabled.handoverEndOkCount,
                                1,
                                "with the feature disabled the handover should succeed in the "
                                "identical scenario");
}

/**
 * @ingroup nr-test
 *
 * @brief NR TR 36.839 handover failure test suite.
 */
class NrTr36839HandoverFailureTestSuite : public TestSuite
{
  public:
    NrTr36839HandoverFailureTestSuite();
};

NrTr36839HandoverFailureTestSuite::NrTr36839HandoverFailureTestSuite()
    : TestSuite("nr-tr36839-handover-failure", Type::SYSTEM)
{
    AddTestCase(new NrTr36839HandoverFailureTestCase(false), TestCase::Duration::QUICK);
    AddTestCase(new NrTr36839HandoverFailureTestCase(true), TestCase::Duration::QUICK);
}

/**
 * @ingroup nr-test
 * Static variable for test initialization.
 */
static NrTr36839HandoverFailureTestSuite g_nrTr36839HandoverFailureTestSuiteInstance;
