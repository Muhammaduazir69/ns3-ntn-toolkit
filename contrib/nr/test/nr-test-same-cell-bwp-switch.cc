// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-same-cell-bwp-switch.cc
 *
 * @brief Test suite `nr-same-cell-bwp-switch`: intra-cell primary bandwidth-part switching. A
 * single gNB carries two BWPs on different carriers (2.8 and 3.5 GHz) that share one cellId, and
 * a stationary UE configured with both BWPs camps on BWP A while a sustained downlink UDP flow
 * runs. At 1 s the gNB's BWP-A transmit power is dropped so the UE's measured RSRP on BWP B
 * overtakes BWP A. The test asserts that the UE switches its primary DL BWP to B while the
 * serving cellId stays unchanged (no handover), that the gNB tracks the UE's new primary BWP, and
 * that downlink data keeps flowing after the switch; both the ideal and the real RRC protocol are
 * exercised.
 */

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/point-to-point-module.h"
#include "ns3/udp-client-server-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrSameCellBwpSwitchTest");

/**
 * @ingroup nr-test
 *
 * @brief Same-cell primary-BWP switch test (intra-cell, not a handover).
 *
 * A SINGLE gNB (a single cellId) is configured with TWO bandwidth parts, A and
 * B, on different carrier frequencies (2.8 / 3.5 GHz). In ns-3 NR the two BWPs
 * of one gNB share the SAME cellId and are differentiated by ARFCN alone. A
 * single UE is configured with both BWPs and camps on BWP A, where a sustained
 * downlink flow runs.
 *
 * Mid-simulation the gNB's BWP-A PHY transmit power is dropped, so the UE's
 * per-carrier RSRP for BWP B overtakes BWP A. The UE's RSRP-driven same-cell
 * policy then switches its primary serving BWP A->B WITHOUT changing cell
 * (no handover), notifies the gNB, and the gNB re-points the UE's downlink
 * scheduling to BWP B so data keeps flowing.
 *
 * The test asserts:
 *  - both BWPs share one cellId;
 *  - the UE initially camps on BWP A and is connected to the single cell;
 *  - after the power drop the UE primary DL BWP becomes B;
 *  - the cellId is UNCHANGED (intra-cell, not a handover);
 *  - the gNB learned the UE's new primary BWP (gNB-side tracking);
 *  - downlink data continues to be delivered after the switch.
 *
 * No genie/ideal RSRP measurement is used: the gNB transmits DL control on both
 * BWPs every slot, so the UE's BWP-B PHY measures RSRP_B from the real signal.
 */
class NrSameCellBwpSwitchTestCase : public TestCase
{
  public:
    /**
     * @brief Constructor.
     * @param useIdealRrc whether to use the ideal RRC protocol
     */
    NrSameCellBwpSwitchTestCase(bool useIdealRrc);

  private:
    /**
     * @brief Build the test case name.
     * @param useIdealRrc whether to use the ideal RRC protocol
     * @return the name string
     */
    static std::string BuildNameString(bool useIdealRrc);

    void DoRun() override;

    /**
     * @brief Sample the UE primary BWP / cell once RSRP_B should beat RSRP_A.
     *
     * Captured at this instant so the post-switch assertions can compare against
     * the steady state after the gNB BWP-A power has been dropped.
     *
     * @param ueDevice  the UE net device
     * @param gnbDevice the gNB net device
     * @param bwpB      the BWP/CC index of BWP B (expected new primary)
     */
    void SampleAfterDrop(Ptr<NetDevice> ueDevice, Ptr<NetDevice> gnbDevice, uint16_t bwpB);

    bool m_useIdealRrc;             ///< whether to use the ideal RRC protocol
    Ptr<PacketSink> m_dlSink;       ///< downlink packet sink on the UE
    uint32_t m_rxBeforeDrop{0};     ///< DL bytes delivered before the power drop
    uint32_t m_rxAfterSwitch{0};    ///< DL bytes delivered right after the switch sample
    bool m_sampled{false};          ///< whether SampleAfterDrop already ran
    uint16_t m_uePrimaryAfter{0};   ///< UE primary DL BWP sampled after the drop
    uint16_t m_ueCellAfter{0};      ///< UE serving cellId sampled after the drop
    uint8_t m_gnbPrimaryAfter{255}; ///< gNB-tracked primary BWP sampled after the drop
};

std::string
NrSameCellBwpSwitchTestCase::BuildNameString(bool useIdealRrc)
{
    std::ostringstream oss;
    oss << "Same-cell BWP switch" << (useIdealRrc ? ", ideal RRC" : ", real RRC");
    return oss.str();
}

NrSameCellBwpSwitchTestCase::NrSameCellBwpSwitchTestCase(bool useIdealRrc)
    : TestCase(BuildNameString(useIdealRrc)),
      m_useIdealRrc(useIdealRrc)
{
}

void
NrSameCellBwpSwitchTestCase::SampleAfterDrop(Ptr<NetDevice> ueDevice,
                                             Ptr<NetDevice> gnbDevice,
                                             uint16_t bwpB)
{
    auto nrUeDevice = ueDevice->GetObject<NrUeNetDevice>();
    auto ueRrc = nrUeDevice->GetRrc();
    auto nrGnbDevice = gnbDevice->GetObject<NrGnbNetDevice>();

    m_uePrimaryAfter = ueRrc->GetPrimaryDlIndex();
    m_ueCellAfter = ueRrc->GetCellId();
    m_rxAfterSwitch = m_dlSink ? m_dlSink->GetTotalRx() : 0;

    uint16_t rnti = ueRrc->GetRnti();
    if (nrGnbDevice->GetRrc()->HasUeManager(rnti))
    {
        m_gnbPrimaryAfter = nrGnbDevice->GetRrc()->GetUeManager(rnti)->GetPrimaryBwp();
    }
    m_sampled = true;
    NS_LOG_INFO("Sample after drop: UE primary DL BWP "
                << m_uePrimaryAfter << " cell " << m_ueCellAfter << " gNB-tracked primary "
                << +m_gnbPrimaryAfter << " (expected BWP B " << bwpB << ")");
}

void
NrSameCellBwpSwitchTestCase::DoRun()
{
    uint32_t previousSeed = RngSeedManager::GetSeed();
    uint64_t previousRun = RngSeedManager::GetRun();
    Config::Reset();
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(40));
    Config::SetDefault("ns3::NrUePhy::TxPower", DoubleValue(23));
    Config::SetDefault("ns3::NrUePhy::EnableUplinkPowerControl", BooleanValue(false));

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(m_useIdealRrc));

    // Single gNB, single UE, both stationary. The UE sits where both BWPs are
    // initially receivable; the switch is induced by changing TX power, not by
    // moving the UE.
    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator>();
    gnbPositionAlloc->Add(Vector(0, 0, 0));
    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.SetPositionAllocator(gnbPositionAlloc);
    gnbMobility.Install(gnbNodes);

    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    ueMobility.Install(ueNodes);
    ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(80, 0, 0));

    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(FriisPropagationLossModel::GetTypeId());

    CcBwpCreator ccBwpCreator;

    // BWP A on 2.8 GHz and BWP B on 3.5 GHz, BOTH belonging to the SAME gNB.
    // A 5 MHz channel is used (rather than the usual 20 MHz) because a UE served
    // by two BWPs of one cell is configured with the second BWP as a secondary
    // component carrier, whose RRC SCell DL-bandwidth field is serialized as an
    // LTE-style RB-count enum constrained to [11,100]; 5 MHz maps to 50 and 20
    // MHz (200) would trip that pre-existing carrier-aggregation serialization
    // limit (unrelated to BWP switching).
    CcBwpCreator::SimpleOperationBandConf bandConfA(2.8e9, 5e6, static_cast<uint8_t>(1));
    OperationBandInfo bandA = ccBwpCreator.CreateOperationBandContiguousCc(bandConfA);
    channelHelper->AssignChannelsToBands({bandA});
    BandwidthPartInfoPtrVector bwpsA = CcBwpCreator::GetAllBwps({bandA});

    CcBwpCreator::SimpleOperationBandConf bandConfB(3.5e9, 5e6, static_cast<uint8_t>(1));
    OperationBandInfo bandB = ccBwpCreator.CreateOperationBandContiguousCc(bandConfB);
    channelHelper->AssignChannelsToBands({bandB});
    BandwidthPartInfoPtrVector bwpsB = CcBwpCreator::GetAllBwps({bandB});

    // Install ONE gNB device carrying BOTH BWPs (one cellId, two carriers).
    NetDeviceContainer gnbDevices =
        nrHelper->InstallGnbDevice(gnbNodes.Get(0), {bwpsA.front(), bwpsB.front()});

    // The UE is configured with both BWPs so it can listen/measure on both.
    NetDeviceContainer ueDevices =
        nrHelper->InstallUeDevice(ueNodes.Get(0), {bwpsA.front(), bwpsB.front()});

    NrHelper::GetGnbPhy(gnbDevices.Get(0), 0)->SetAttribute("Numerology", UintegerValue(0));
    NrHelper::GetGnbPhy(gnbDevices.Get(0), 1)->SetAttribute("Numerology", UintegerValue(0));

    nrHelper->AssignStreams({.gnbDevs = gnbDevices, .ueDevs = ueDevices});

    Ptr<NrGnbNetDevice> gnb = gnbDevices.Get(0)->GetObject<NrGnbNetDevice>();
    Ptr<NrUeNetDevice> ue = ueDevices.Get(0)->GetObject<NrUeNetDevice>();

    // Resolve the BWP indices for carrier A and carrier B on the UE.
    uint32_t arfcnA = gnb->GetBwpArfcn(0);
    uint32_t arfcnB = gnb->GetBwpArfcn(1);
    uint16_t ueBwpA = ue->GetArfcnBwpId(arfcnA);
    uint16_t ueBwpB = ue->GetArfcnBwpId(arfcnB);

    // Both BWPs must share a single cellId (intra-cell pre-condition).
    NS_TEST_ASSERT_MSG_EQ(gnb->GetCellId(0),
                          gnb->GetCellId(1),
                          "The two BWPs of one gNB must share a single cellId");
    NS_TEST_ASSERT_MSG_NE(arfcnA, arfcnB, "The two BWPs must be on different ARFCNs");

    // Internet / remote host.
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

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces =
        epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevices));

    // Attach the UE to the single gNB. With BWP A the stronger carrier at the
    // start, the UE camps on BWP A.
    nrHelper->AttachToGnb(ueDevices.Get(0), gnbDevices.Get(0));

    // Sustained downlink UDP flow.
    uint16_t dlPort = 10000;
    UdpClientHelper dlClientHelper(ueIpIfaces.GetAddress(0), dlPort);
    dlClientHelper.SetAttribute("Interval", TimeValue(MilliSeconds(5)));
    dlClientHelper.SetAttribute("MaxPackets", UintegerValue(1000000));
    dlClientHelper.SetAttribute("PacketSize", UintegerValue(200));
    ApplicationContainer clientApps = dlClientHelper.Install(remoteHost);

    PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(Ipv4Address::GetAny(), dlPort));
    ApplicationContainer sinkApps = dlPacketSinkHelper.Install(ueNodes.Get(0));
    Ptr<PacketSink> dlSink = sinkApps.Get(0)->GetObject<PacketSink>();
    m_dlSink = dlSink;

    NrQosFlow flow(NrQosFlow::NGBR_VIDEO_TCP_DEFAULT);
    Ptr<NrQosRule> rule = Create<NrQosRule>();
    NrQosRule::PacketFilter dlpf;
    dlpf.localPortStart = dlPort;
    dlpf.localPortEnd = dlPort;
    rule->Add(dlpf);
    nrHelper->ActivateDedicatedQosFlow(ueDevices.Get(0), flow, rule);

    clientApps.Start(MilliSeconds(50));
    sinkApps.Start(MilliSeconds(40));

    // Verify the UE initially camps on BWP A and is connected to the single cell.
    Simulator::Schedule(MilliSeconds(200), [&]() {
        Ptr<NrUeRrc> ueRrc = ue->GetRrc();
        NS_TEST_ASSERT_MSG_EQ(ueRrc->GetState(),
                              NrUeRrc::CONNECTED_NORMALLY,
                              "UE not connected at start");
        NS_TEST_ASSERT_MSG_EQ(ueRrc->GetPrimaryDlIndex(),
                              ueBwpA,
                              "UE did not initially camp on BWP A");
        NS_TEST_ASSERT_MSG_EQ(ueRrc->GetCellId(),
                              gnb->GetCellId(),
                              "UE not connected to the single cell");
    });

    // Record DL bytes delivered while still on BWP A (data plane up before drop).
    Simulator::Schedule(MilliSeconds(900), [&]() { m_rxBeforeDrop = dlSink->GetTotalRx(); });

    // Drop BWP-A PHY TX power well below BWP B so RSRP_B overtakes RSRP_A and the
    // RSRP-driven policy switches the primary BWP A->B.
    Simulator::Schedule(Seconds(1.0), [&]() {
        NrHelper::GetGnbPhy(gnbDevices.Get(0), 0)->SetTxPower(1.0);
        NS_LOG_INFO("Dropped gNB BWP-A TX power; expecting UE to switch A->B");
    });

    // Sample steady state after RSRP_B has overtaken and the switch settled.
    Simulator::Schedule(Seconds(2.5),
                        &NrSameCellBwpSwitchTestCase::SampleAfterDrop,
                        this,
                        ueDevices.Get(0),
                        gnbDevices.Get(0),
                        ueBwpB);

    Simulator::Stop(Seconds(3.0));
    Simulator::Run();

    // Pre-conditions: data plane was up before the drop.
    NS_TEST_ASSERT_MSG_GT(m_rxBeforeDrop, 0, "No downlink data delivered before the power drop");
    NS_TEST_ASSERT_MSG_EQ(m_sampled, true, "Post-drop sample did not run");

    // The UE switched its primary DL BWP A->B.
    NS_TEST_ASSERT_MSG_EQ(m_uePrimaryAfter,
                          ueBwpB,
                          "UE did not switch its primary DL BWP to B after RSRP_B > RSRP_A");

    // Intra-cell: the cellId is unchanged (this is NOT a handover).
    NS_TEST_ASSERT_MSG_EQ(m_ueCellAfter,
                          gnb->GetCellId(),
                          "Cell changed: the switch must be intra-cell, not a handover");

    // The gNB learned the UE's new primary BWP (gNB-side tracking).
    NS_TEST_ASSERT_MSG_EQ(+m_gnbPrimaryAfter,
                          +static_cast<uint8_t>(ueBwpB),
                          "gNB did not track the UE's primary BWP switch to B");

    // Downlink data continuity: the flow keeps delivering bytes after the switch.
    uint32_t rxAtEnd = dlSink->GetTotalRx();
    NS_TEST_ASSERT_MSG_GT(rxAtEnd,
                          m_rxAfterSwitch,
                          "No downlink data delivered after the same-cell BWP switch (DL did not "
                          "follow the UE to BWP B)");
    NS_TEST_ASSERT_MSG_GT_OR_EQ(rxAtEnd - m_rxAfterSwitch,
                                2000,
                                "Insufficient sustained downlink traffic after the same-cell BWP "
                                "switch");

    Simulator::Destroy();

    Config::Reset();
    RngSeedManager::SetSeed(previousSeed);
    RngSeedManager::SetRun(previousRun);
}

/**
 * @ingroup nr-test
 *
 * @brief NR same-cell BWP switch test suite.
 *
 * Validates an intra-cell primary-BWP switch driven by per-carrier RSRP on a
 * single gNB with two BWPs (one cellId, two carriers), including the gNB-side
 * primary-BWP tracking and downlink data continuity after the switch.
 */
class NrSameCellBwpSwitchTestSuite : public TestSuite
{
  public:
    NrSameCellBwpSwitchTestSuite();
};

NrSameCellBwpSwitchTestSuite::NrSameCellBwpSwitchTestSuite()
    : TestSuite("nr-same-cell-bwp-switch", Type::SYSTEM)
{
    AddTestCase(new NrSameCellBwpSwitchTestCase(true), Duration::QUICK);
    AddTestCase(new NrSameCellBwpSwitchTestCase(false), Duration::QUICK);
}

/**
 * @ingroup nr-test
 * Static variable for test initialization.
 */
static NrSameCellBwpSwitchTestSuite g_nrSameCellBwpSwitchTestSuiteInstance;
