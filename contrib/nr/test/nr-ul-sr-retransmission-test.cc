// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup tests
 * @file nr-ul-sr-retransmission-test.cc
 * @brief Test suite `nr-ul-sr-retransmission`: the UE MAC must keep retransmitting the
 * scheduling request while it is backlogged, even when the bootstrap BSR transport blocks
 * are never decoded by the gNB.
 *
 * A single UE with a very low transmit power sends a continuous stream of UL packets. The
 * UL data SINR is so low that every UL data TB is corrupt, so the gNB never learns the UE
 * buffer status from a BSR; UL control messages (SR) are delivered without an error model,
 * so they always get through. While a BSR is pending and no UL-SCH resources are available,
 * the UE must keep transmitting the scheduling request on its periodic SR occasions
 * (3GPP TS 38.321, clauses 5.4.4 and 5.4.5). Before the fix, the SR re-arm condition in
 * NrUeMac::DoTransmitBufferStatusReport was keyed to the last received UL DCI (HARQ process
 * 0, or rv 3): once the few bootstrap grants were spent without a decoded BSR, the stale
 * DCI never satisfied the condition, no SR was ever sent again, and the uplink stalled
 * permanently. The test asserts that SRs keep arriving at the gNB and that the gNB keeps
 * issuing UL grants after the bootstrap window.
 */

#include "ns3/applications-module.h"
#include "ns3/config.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/test.h"

namespace ns3
{

/**
 * @ingroup nr-test
 * @brief System test: SR retransmission keeps the UL scheduling alive when BSR TBs are lost
 */
class NrUlSrRetransmissionTestCase : public TestCase
{
  public:
    NrUlSrRetransmissionTestCase();
    ~NrUlSrRetransmissionTestCase() override;

  private:
    void DoRun() override;

    /**
     * @brief Trace UL grants issued by the gNB MAC
     * @param data scheduling callback info
     */
    void GrantTrace(NrSchedulingCallbackInfo data);

    /**
     * @brief Trace control messages received by the gNB MAC
     * @param imsi UE IMSI
     * @param sfn slot indication
     * @param cellId cell ID
     * @param rnti UE RNTI
     * @param ccId component carrier ID
     * @param msg the received control message
     */
    void CtrlTrace(uint64_t imsi,
                   SfnSf sfn,
                   uint16_t cellId,
                   uint16_t rnti,
                   uint8_t ccId,
                   Ptr<const NrControlMessage> msg);

    uint32_t m_grantsTotal{0}; ///< UL grants issued during the whole simulation
    uint32_t m_grantsLate{0};  ///< UL grants issued after the bootstrap window
    uint32_t m_srTotal{0};     ///< SRs received by the gNB during the whole simulation
    uint32_t m_srLate{0};      ///< SRs received by the gNB after the bootstrap window
    Time m_bootstrapEnd;       ///< end of the bootstrap window
};

NrUlSrRetransmissionTestCase::NrUlSrRetransmissionTestCase()
    : TestCase("UE retransmits SR while backlogged when bootstrap BSRs are lost")
{
}

NrUlSrRetransmissionTestCase::~NrUlSrRetransmissionTestCase()
{
}

void
NrUlSrRetransmissionTestCase::GrantTrace(NrSchedulingCallbackInfo data)
{
    m_grantsTotal++;
    if (Simulator::Now() > m_bootstrapEnd)
    {
        m_grantsLate++;
    }
}

void
NrUlSrRetransmissionTestCase::CtrlTrace(uint64_t imsi,
                                        SfnSf sfn,
                                        uint16_t cellId,
                                        uint16_t rnti,
                                        uint8_t ccId,
                                        Ptr<const NrControlMessage> msg)
{
    if (msg->GetMessageType() == NrControlMessage::SR)
    {
        m_srTotal++;
        if (Simulator::Now() > m_bootstrapEnd)
        {
            m_srLate++;
        }
    }
}

void
NrUlSrRetransmissionTestCase::DoRun()
{
    Time simTime = Seconds(3);
    m_bootstrapEnd = MilliSeconds(1500);

    NodeContainer gNbNode;
    NodeContainer ueNode;
    gNbNode.Create(1);
    ueNode.Create(1);

    Ptr<ConstantPositionMobilityModel> mobility = CreateObject<ConstantPositionMobilityModel>();
    gNbNode.Get(0)->AggregateObject(mobility);
    mobility->SetPosition(Vector(0, 0, 10));

    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    ueMobility.Install(ueNode);
    ueNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(100, 0, 1.5));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(28e9, 50e6, 1);
    OperationBandInfo band0 = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(QuasiOmniDirectPathBeamforming::GetTypeId()));
    nrHelper->SetEpcHelper(nrEpcHelper);

    std::string errorModel = "ns3::NrEesmIrT2";
    nrHelper->SetUlErrorModel(errorModel);
    nrHelper->SetDlErrorModel(errorModel);

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("RMa", "Default");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    channelHelper->AssignChannelsToBands({band0});

    auto allBwps = CcBwpCreator::GetAllBwps({band0});
    NetDeviceContainer gnbDevice = nrHelper->InstallGnbDevice(gNbNode, allBwps);
    NrHelper::GetGnbPhy(gnbDevice.Get(0), 0)->SetAttribute("TxPower", DoubleValue(35));
    NetDeviceContainer ueDevice = nrHelper->InstallUeDevice(ueNode, allBwps);

    // Very low UE power: every UL data TB is corrupt, so the gNB never decodes a BSR.
    // UL control messages are delivered without an error model, so SRs still get through.
    NrHelper::GetUePhy(ueDevice.Get(0), 0)->SetAttribute("TxPower", DoubleValue(-30));

    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    InternetStackHelper internet;
    internet.Install(remoteHostContainer);
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.000)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    internet.Install(ueNode);

    nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevice));

    nrHelper->AttachToClosestGnb(ueDevice, gnbDevice);

    // Continuous UL traffic keeps the UE backlogged for the whole simulation
    uint16_t ulPort = 20000;
    UdpServerHelper ulPacket(ulPort);
    ApplicationContainer serverApps = ulPacket.Install(remoteHost);

    UdpClientHelper ulClient(remoteHostAddr, ulPort);
    ulClient.SetAttribute("MaxPackets", UintegerValue(1000));
    ulClient.SetAttribute("Interval", TimeValue(MilliSeconds(20)));
    ulClient.SetAttribute("PacketSize", UintegerValue(1250));
    ApplicationContainer clientApps = ulClient.Install(ueNode.Get(0));

    serverApps.Start(MilliSeconds(500));
    clientApps.Start(MilliSeconds(500));
    serverApps.Stop(simTime);
    clientApps.Stop(simTime);

    NrHelper::GetGnbMac(gnbDevice.Get(0), 0)
        ->TraceConnectWithoutContext("UlScheduling",
                                     MakeCallback(&NrUlSrRetransmissionTestCase::GrantTrace, this));
    NrHelper::GetGnbMac(gnbDevice.Get(0), 0)
        ->TraceConnectWithoutContext("GnbMacRxedCtrlMsgsTrace",
                                     MakeCallback(&NrUlSrRetransmissionTestCase::CtrlTrace, this));

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
    nrHelper->AssignStreams(gnbDevice, 1);
    nrHelper->AssignStreams(ueDevice, 1000);

    Simulator::Stop(simTime);
    Simulator::Run();

    NS_TEST_ASSERT_MSG_GT(m_srTotal,
                          0,
                          "The gNB should receive at least the bootstrap SR from the UE");
    NS_TEST_ASSERT_MSG_GT(
        m_srLate,
        0,
        "The UE stopped sending SRs after the bootstrap window: without a decoded BSR the "
        "uplink stalls permanently, so the SR must be retransmitted while the UE is backlogged");
    NS_TEST_ASSERT_MSG_GT(
        m_grantsLate,
        0,
        "The gNB stopped issuing UL grants after the bootstrap window: it should keep "
        "granting in response to the retransmitted SRs");

    Simulator::Destroy();
}

/**
 * @ingroup nr-test
 * @brief Test suite for the SR retransmission regression
 */
class NrUlSrRetransmissionTestSuite : public TestSuite
{
  public:
    NrUlSrRetransmissionTestSuite();
};

NrUlSrRetransmissionTestSuite::NrUlSrRetransmissionTestSuite()
    : TestSuite("nr-ul-sr-retransmission", Type::SYSTEM)
{
    AddTestCase(new NrUlSrRetransmissionTestCase(), Duration::QUICK);
}

static NrUlSrRetransmissionTestSuite g_nrUlSrRetransmissionTestSuite; //!< Nr test suite

} // namespace ns3
