// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-inter-freq-handover.cc
 *
 * @brief Test suite `nr-inter-freq-handover`: connected-mode handover between two gNBs operating
 * on different carrier frequencies (2.8 and 3.5 GHz), optionally with different numerologies. A
 * single UE with one bandwidth part per carrier starts on gNB0 and moves towards gNB1 while a
 * downlink UDP flow runs; each gNB registers the other's ARFCN as a neighbour measurement
 * frequency so the A3-RSRP algorithm can trigger an X2 handover to the cross-frequency cell. The
 * test asserts that at least one handover completes onto the other-ARFCN cell and that downlink
 * data flowed before the handover and keeps flowing (sustained) on the target cell afterwards.
 * All four combinations of ideal/real RRC and same/inter numerology are exercised.
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

NS_LOG_COMPONENT_DEFINE("NrInterFreqHandoverTest");

/**
 * @ingroup nr-test
 *
 * @brief Connected-mode inter-frequency (and inter-numerology) handover test.
 *
 * Two gNBs operate on different carrier frequencies (ARFCNs) and, optionally,
 * different numerologies. A single UE, equipped with one bandwidth part per
 * carrier frequency, starts attached to the first gNB and moves towards the
 * second. The A3-RSRP handover algorithm is expected to detect the stronger
 * neighbour on the other frequency, trigger an X2 handover, and the UE is
 * expected to re-establish on the target cell with data still flowing.
 *
 * This exercises the inter-frequency measurement support: the gNBs register
 * each other's carrier frequency as a neighbour measurement frequency, so the
 * UE measures and reports cross-ARFCN neighbours that can trigger handover.
 */
class NrInterFreqHandoverTestCase : public TestCase
{
  public:
    /**
     * @brief Constructor.
     * @param useIdealRrc whether to use the ideal RRC protocol
     * @param interNumerology whether the two cells use different numerologies
     */
    NrInterFreqHandoverTestCase(bool useIdealRrc, bool interNumerology);

  private:
    /**
     * @brief Build the test case name.
     * @param useIdealRrc whether to use the ideal RRC protocol
     * @param interNumerology whether the two cells use different numerologies
     * @return the name string
     */
    static std::string BuildNameString(bool useIdealRrc, bool interNumerology);

    void DoRun() override;

    /**
     * @brief Record a successful handover reported by a gNB RRC.
     * @param context trace context
     * @param imsi the UE IMSI
     * @param cellId the cell the UE handed over to
     * @param rnti the UE RNTI on the target cell
     */
    void HandoverEndOkGnb(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti);

    /**
     * @brief Assert that the UE is connected to the expected gNB.
     * @param ueDevice the UE device
     * @param gnbDevice the expected serving gNB device
     */
    void CheckConnected(Ptr<NetDevice> ueDevice, Ptr<NetDevice> gnbDevice);

    bool m_useIdealRrc;            ///< whether to use the ideal RRC protocol
    bool m_interNumerology;        ///< whether the two cells use different numerologies
    uint16_t m_handoverCount;      ///< number of successful handovers observed
    uint16_t m_handoverTargetCell; ///< cell ID of the last handover target
    Ptr<PacketSink> m_dlSink;      ///< downlink packet sink on the UE
    uint32_t m_rxAtHandover;       ///< downlink bytes received at handover completion
    Time m_handoverTime;           ///< time the handover completed
};

std::string
NrInterFreqHandoverTestCase::BuildNameString(bool useIdealRrc, bool interNumerology)
{
    std::ostringstream oss;
    oss << "Inter-frequency handover"
        << (interNumerology ? " (inter-numerology)" : " (same numerology)")
        << (useIdealRrc ? ", ideal RRC" : ", real RRC");
    return oss.str();
}

NrInterFreqHandoverTestCase::NrInterFreqHandoverTestCase(bool useIdealRrc, bool interNumerology)
    : TestCase(BuildNameString(useIdealRrc, interNumerology)),
      m_useIdealRrc(useIdealRrc),
      m_interNumerology(interNumerology),
      m_handoverCount(0),
      m_handoverTargetCell(0),
      m_dlSink(nullptr),
      m_rxAtHandover(0),
      m_handoverTime(Seconds(0))
{
}

void
NrInterFreqHandoverTestCase::HandoverEndOkGnb(std::string context,
                                              uint64_t imsi,
                                              uint16_t cellId,
                                              uint16_t rnti)
{
    NS_LOG_INFO("HandoverEndOk: IMSI " << imsi << " now on cellId " << cellId << " rnti " << rnti);
    m_handoverCount++;
    m_handoverTargetCell = cellId;
    m_handoverTime = Simulator::Now();
    // Snapshot the downlink bytes delivered right at handover completion so we
    // can later assert that traffic kept flowing on the target cell afterwards.
    if (m_dlSink)
    {
        m_rxAtHandover = m_dlSink->GetTotalRx();
    }
}

void
NrInterFreqHandoverTestCase::DoRun()
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
    nrHelper->SetHandoverAlgorithmType("ns3::NrA3RsrpHandoverAlgorithm");
    nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(1.5));
    nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(64)));

    // Topology: two gNBs on the X axis, UE moving from gNB0 towards gNB1.
    NodeContainer gnbNodes;
    gnbNodes.Create(2);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    double gnbDistance = 1000.0;
    Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator>();
    gnbPositionAlloc->Add(Vector(0, 0, 0));
    gnbPositionAlloc->Add(Vector(gnbDistance, 0, 0));
    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.SetPositionAllocator(gnbPositionAlloc);
    gnbMobility.Install(gnbNodes);

    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ueMobility.Install(ueNodes);
    // Start near gNB0 and move towards gNB1 at a moderate speed, ending the
    // simulation close to (but not past) gNB1 so the UE settles on the target
    // cell after the handover completes.
    ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(100, 0, 0));
    ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(700, 0, 0));

    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(FriisPropagationLossModel::GetTypeId());

    CcBwpCreator ccBwpCreator;

    // First operation band / carrier frequency for gNB0.
    CcBwpCreator::SimpleOperationBandConf bandConf0(2.8e9, 20e6, static_cast<uint8_t>(1));
    OperationBandInfo band0 = ccBwpCreator.CreateOperationBandContiguousCc(bandConf0);
    channelHelper->AssignChannelsToBands({band0});
    BandwidthPartInfoPtrVector bwps0 = CcBwpCreator::GetAllBwps({band0});

    // Second operation band / carrier frequency for gNB1.
    CcBwpCreator::SimpleOperationBandConf bandConf1(3.5e9, 20e6, static_cast<uint8_t>(1));
    OperationBandInfo band1 = ccBwpCreator.CreateOperationBandContiguousCc(bandConf1);
    channelHelper->AssignChannelsToBands({band1});
    BandwidthPartInfoPtrVector bwps1 = CcBwpCreator::GetAllBwps({band1});

    // Each gNB has a single BWP on its own carrier frequency.
    NetDeviceContainer gnbDevices;
    gnbDevices.Add(nrHelper->InstallGnbDevice(gnbNodes.Get(0), bwps0));
    gnbDevices.Add(nrHelper->InstallGnbDevice(gnbNodes.Get(1), bwps1));

    // The UE is configured with both carrier frequencies so it can measure and
    // re-tune to either of them.
    NetDeviceContainer ueDevices =
        nrHelper->InstallUeDevice(ueNodes.Get(0), {bwps0.front(), bwps1.front()});

    // Configure numerologies. gNB0/UE-BWP0 use numerology 0, gNB1/UE-BWP1 use
    // numerology 1 when testing inter-numerology handover.
    // The UE picks up each cell's numerology from the broadcast MIB, so only
    // the gNB PHY numerology needs to be configured here.
    uint16_t numGnb1 = m_interNumerology ? 1 : 0;
    NrHelper::GetGnbPhy(gnbDevices.Get(0), 0)->SetAttribute("Numerology", UintegerValue(0));
    NrHelper::GetGnbPhy(gnbDevices.Get(1), 0)->SetAttribute("Numerology", UintegerValue(numGnb1));

    nrHelper->AssignStreams({.gnbDevs = gnbDevices, .ueDevs = ueDevices});

    Ptr<NrGnbNetDevice> gnb0 = gnbDevices.Get(0)->GetObject<NrGnbNetDevice>();
    Ptr<NrGnbNetDevice> gnb1 = gnbDevices.Get(1)->GetObject<NrGnbNetDevice>();

    // Register each gNB's frequency as a neighbour measurement frequency on the
    // other gNB, enabling inter-frequency measurement reporting and handover.
    // This must happen before the cell is configured (i.e. before attach).
    uint32_t arfcn0 = gnb0->GetBwpArfcn(0);
    uint32_t arfcn1 = gnb1->GetBwpArfcn(0);
    uint8_t bw0 = gnb0->GetBwpDlBandwidth(0);
    uint8_t bw1 = gnb1->GetBwpDlBandwidth(0);
    gnb0->GetRrc()->AddNeighbourMeasFrequency(arfcn1, bw1);
    gnb1->GetRrc()->AddNeighbourMeasFrequency(arfcn0, bw0);

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

    // Attach the UE to gNB0 at the start.
    nrHelper->AttachToGnb(ueDevices.Get(0), gnbDevices.Get(0));

    // Downlink UDP flow to verify data continuity after handover.
    uint16_t dlPort = 10000;
    UdpClientHelper dlClientHelper(ueIpIfaces.GetAddress(0), dlPort);
    dlClientHelper.SetAttribute("Interval", TimeValue(MilliSeconds(10)));
    dlClientHelper.SetAttribute("MaxPackets", UintegerValue(1000000));
    dlClientHelper.SetAttribute("PacketSize", UintegerValue(100));
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

    nrHelper->AddX2Interface(gnbNodes);

    // Connect to the gNB HandoverEndOk trace to detect successful handovers.
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::NrGnbNetDevice/NrGnbRrc/HandoverEndOk",
                    MakeCallback(&NrInterFreqHandoverTestCase::HandoverEndOkGnb, this));

    // Initially connected to gNB0 (on the first carrier frequency).
    Simulator::Schedule(MilliSeconds(120),
                        &NrInterFreqHandoverTestCase::CheckConnected,
                        this,
                        ueDevices.Get(0),
                        gnbDevices.Get(0));

    // Record the downlink bytes received while still served by gNB0, to confirm
    // the data plane is up before the handover.
    uint32_t rxOnSource = 0;
    Simulator::Schedule(MilliSeconds(800), [&]() { rxOnSource = dlSink->GetTotalRx(); });

    Simulator::Stop(Seconds(5));

    Simulator::Run();

    // The UE measured the neighbour gNB on a different carrier frequency, the
    // A3 event was triggered for that cross-ARFCN neighbour, and an X2 handover
    // onto it completed successfully.
    NS_TEST_ASSERT_MSG_GT_OR_EQ(m_handoverCount,
                                1,
                                "No inter-frequency handover was triggered/completed");
    NS_TEST_ASSERT_MSG_EQ(m_handoverTargetCell,
                          gnb1->GetCellId(),
                          "Handover target is not the inter-frequency neighbour cell");
    NS_TEST_ASSERT_MSG_NE(gnb0->GetBwpArfcn(0),
                          gnb1->GetBwpArfcn(0),
                          "Source and target cells must be on different ARFCNs for this test");

    // Downlink data must have been flowing on the source cell before the
    // inter-frequency handover (proving the end-to-end data plane).
    NS_TEST_ASSERT_MSG_GT(rxOnSource,
                          0,
                          "No downlink data received on the source cell before handover");

    // Data continuity: the downlink flow must keep delivering packets on the
    // target cell for a sustained period after the inter-frequency (inter-BWP)
    // handover completes, not merely complete the control-plane handover.
    //
    // This is asserted for every variant: same- and inter-numerology, under
    // both the ideal and the real RRC protocol. The inter-numerology case
    // requires that the target cell's broadcast PHY configuration (numerology,
    // TDD pattern, control symbols) is carried in the handover command and that
    // the UE does not let a neighbour cell's overheard MIB reset the re-tuned
    // serving BWP, so DL data reception (and hence the DL-CQI feedback the
    // target gNB needs to schedule downlink) recovers on the new numerology.
    uint32_t rxAtEnd = dlSink->GetTotalRx();
    NS_TEST_ASSERT_MSG_GT(rxAtEnd,
                          m_rxAtHandover,
                          "No downlink data delivered on the target cell after the "
                          "inter-frequency handover (data plane did not survive the re-tune)");
    // Require a meaningful amount of post-handover traffic (several packets
    // of 100 bytes each at one packet per 10 ms), demonstrating sustained
    // delivery.
    NS_TEST_ASSERT_MSG_GT_OR_EQ(rxAtEnd - m_rxAtHandover,
                                500,
                                "Insufficient sustained downlink traffic on the target cell "
                                "after the inter-frequency handover");

    Simulator::Destroy();

    Config::Reset();
    RngSeedManager::SetSeed(previousSeed);
    RngSeedManager::SetRun(previousRun);
}

void
NrInterFreqHandoverTestCase::CheckConnected(Ptr<NetDevice> ueDevice, Ptr<NetDevice> gnbDevice)
{
    Ptr<NrUeNetDevice> nrUeDevice = ueDevice->GetObject<NrUeNetDevice>();
    Ptr<NrUeRrc> ueRrc = nrUeDevice->GetRrc();
    NS_TEST_ASSERT_MSG_EQ(ueRrc->GetState(), NrUeRrc::CONNECTED_NORMALLY, "Wrong NrUeRrc state!");

    Ptr<NrGnbNetDevice> nrGnbDevice = gnbDevice->GetObject<NrGnbNetDevice>();
    NS_TEST_ASSERT_MSG_EQ(ueRrc->GetCellId(),
                          nrGnbDevice->GetCellId(),
                          "IMSI " << ueRrc->GetImsi() << " connected to CellId "
                                  << ueRrc->GetCellId() << " instead of expected "
                                  << nrGnbDevice->GetCellId());

    Ptr<NrGnbRrc> gnbRrc = nrGnbDevice->GetRrc();
    uint16_t rnti = ueRrc->GetRnti();
    Ptr<NrUeManager> nrUeManager = gnbRrc->GetUeManager(rnti);
    NS_TEST_ASSERT_MSG_NE(nrUeManager, nullptr, "RNTI " << rnti << " not found in gNB");
    NS_TEST_ASSERT_MSG_EQ(nrUeManager->GetState(),
                          NrUeManager::CONNECTED_NORMALLY,
                          "Wrong NrUeManager state!");

    // The serving DL ARFCN at the UE must match the serving gNB's DL ARFCN.
    uint16_t ueDlBwp = ueRrc->GetPrimaryDlIndex();
    uint32_t ueDlArfcn = nrUeDevice->GetBwpArfcn(ueDlBwp);
    uint16_t gnbDlBwp = nrGnbDevice->GetArfcnBwpId(ueDlArfcn);
    uint32_t gnbDlArfcn = nrGnbDevice->GetBwpArfcn(gnbDlBwp);
    NS_TEST_ASSERT_MSG_EQ(ueDlArfcn, gnbDlArfcn, "inconsistent DlArfcn after (re)connection");
}

/**
 * @ingroup nr-test
 *
 * @brief NR inter-frequency handover test suite.
 *
 * Validates connected-mode handover between two cells operating on different
 * carrier frequencies (ARFCNs), including the inter-numerology case, driven by
 * the A3-RSRP handover algorithm.
 */
class NrInterFreqHandoverTestSuite : public TestSuite
{
  public:
    NrInterFreqHandoverTestSuite();
};

NrInterFreqHandoverTestSuite::NrInterFreqHandoverTestSuite()
    : TestSuite("nr-inter-freq-handover", Type::SYSTEM)
{
    // useIdealRrc, interNumerology.
    // Each case drives a measurement-triggered inter-frequency handover and
    // verifies the control-plane handover, the pre-handover data plane, and
    // sustained end-to-end downlink data continuity on the target cell after
    // the inter-BWP re-tune. All four combinations are exercised:
    // same-numerology and inter-numerology, each under both the ideal and the
    // real RRC protocol.
    AddTestCase(new NrInterFreqHandoverTestCase(true, false), Duration::QUICK);
    AddTestCase(new NrInterFreqHandoverTestCase(false, false), Duration::QUICK);
    AddTestCase(new NrInterFreqHandoverTestCase(true, true), Duration::QUICK);
    AddTestCase(new NrInterFreqHandoverTestCase(false, true), Duration::QUICK);
}

/**
 * @ingroup nr-test
 * Static variable for test initialization.
 */
static NrInterFreqHandoverTestSuite g_nrInterFreqHandoverTestSuiteInstance;
