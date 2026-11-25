//
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//

/**
 * @ingroup test
 * @file nr-test-stray-rrc-request.cc
 *
 * @brief Regression/liveness guard for the fix "ignore stray RrcConnectionRequest
 * from reconnecting RLF'd UE".
 *
 * When an RLF'd UE re-selects a cell and re-attaches before the serving/target gNB
 * has released its context, a fresh RRC Connection Request can reach NrUeManager in
 * an already-active state (e.g. CONNECTION_RECONFIGURATION or HANDOVER_PATH_SWITCH).
 * Before the fix, the default branch of NrUeManager::RecvRrcConnectionRequest fired
 * NS_FATAL_ERROR("method unexpected in state ..."); the fix instead ignores the
 * stray request and lets the state's own timeout / RLF cleanup release the context
 * so the UE re-attaches cleanly.
 *
 * Directly injecting a stray request is impractical, so this test reproduces the
 * condition it guards: a UE with aggressive RLF timers (N310=1, short T310) and RRC
 * reestablishment enabled is torn away from its serving cell, declares RLF, and then
 * reconnects (to a nearby second gNB / after teleporting back). The test asserts that
 * (a) the simulation runs to completion without the fatal error, (b) at least one RLF
 * actually occurred (so the reconnection path was exercised), and (c) the UE
 * reconnects afterwards (ConnectionEstablished on the UE fires again and the UE ends
 * in CONNECTED_NORMALLY).
 */

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/udp-client-server-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrStrayRrcRequestTest");

/**
 * @ingroup nr-test
 *
 * @brief Test case reproducing the RLF-then-reconnect path guarded by the
 * stray-RrcConnectionRequest fix.
 */
class NrStrayRrcRequestTestCase : public TestCase
{
  public:
    /**
     * @brief Constructor.
     *
     * @param isIdealRrc Whether to use the ideal RRC protocol.
     */
    NrStrayRrcRequestTestCase(bool isIdealRrc);
    ~NrStrayRrcRequestTestCase() override;

  private:
    void DoRun() override;

    /**
     * @brief Teleport the UE to the given position.
     *
     * @param position Target position.
     */
    void JumpAway(Vector position);

    /**
     * @brief Trace sink for UE ConnectionEstablished.
     *
     * @param context Trace context.
     * @param imsi UE IMSI.
     * @param cellId Serving cell ID.
     * @param rnti UE RNTI.
     */
    void ConnectionEstablishedUeCallback(std::string context,
                                         uint64_t imsi,
                                         uint16_t cellId,
                                         uint16_t rnti);

    /**
     * @brief Trace sink for UE RadioLinkFailure.
     *
     * @param context Trace context.
     * @param imsi UE IMSI.
     * @param cellId Serving cell ID at failure.
     * @param rnti UE RNTI.
     */
    void RadioLinkFailureCallback(std::string context,
                                  uint64_t imsi,
                                  uint16_t cellId,
                                  uint16_t rnti);

    bool m_isIdealRrc;               ///< Whether to use the ideal RRC protocol
    Ptr<MobilityModel> m_ueMobility; ///< UE mobility model
    uint32_t m_rlfCount;             ///< Number of RLF trace events
    uint32_t m_ueConnEstCount;       ///< Number of UE ConnectionEstablished events
    bool m_rlfSeen;                  ///< True once at least one RLF has been observed
    uint32_t m_ueConnEstAfterRlf;    ///< UE ConnectionEstablished events observed after first RLF
};

NrStrayRrcRequestTestCase::NrStrayRrcRequestTestCase(bool isIdealRrc)
    : TestCase(std::string("Stray RrcConnectionRequest, RRC ") + (isIdealRrc ? "Ideal" : "Real")),
      m_isIdealRrc(isIdealRrc),
      m_rlfCount(0),
      m_ueConnEstCount(0),
      m_rlfSeen(false),
      m_ueConnEstAfterRlf(0)
{
    NS_LOG_FUNCTION(this << GetName());
}

NrStrayRrcRequestTestCase::~NrStrayRrcRequestTestCase()
{
    NS_LOG_FUNCTION(this << GetName());
}

void
NrStrayRrcRequestTestCase::DoRun()
{
    NS_LOG_FUNCTION(this << GetName());

    Config::Reset();
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
    RngSeedManager::ResetNextStreamIndex();

    double gnbTxPower = 40;

    Config::SetDefault("ns3::NrHelper::UseIdealRrc", BooleanValue(m_isIdealRrc));
    Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(gnbTxPower));
    Config::SetDefault("ns3::NrUePhy::TxPower", DoubleValue(23));
    Config::SetDefault("ns3::ThreeGppPropagationLossModel::ShadowingEnabled", BooleanValue(false));

    // Aggressive RLF detection so the UE declares RLF quickly once torn away, and
    // make sure reestablishment (reconnection) is enabled so it re-attaches.
    Config::SetDefault("ns3::NrUeRrc::N310", UintegerValue(1));
    Config::SetDefault("ns3::NrUeRrc::N311", UintegerValue(1));
    Config::SetDefault("ns3::NrUeRrc::T310", TimeValue(Seconds(1)));
    Config::SetDefault("ns3::NrUeRrc::UseRrcReestablishment", BooleanValue(true));

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(nrEpcHelper);

    auto bandwidthAndBWPPair = nrHelper->CreateBandwidthParts({{1.93e9, 10e6, 1}}, "UMa", "LOS");

    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));

    // Internet / remote host
    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // Two gNBs: gNB 0 serves the UE initially; gNB 1 sits near the position the UE
    // teleports to, so the RLF'd UE re-selects and reconnects there.
    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    gnbNodes.Create(2);
    ueNodes.Create(1);

    Ptr<ListPositionAllocator> positionAllocGnb = CreateObject<ListPositionAllocator>();
    positionAllocGnb->Add(Vector(0, 0, 10));
    positionAllocGnb->Add(Vector(15020, 0, 10));

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAllocGnb);
    mobility.Install(gnbNodes);

    Ptr<ListPositionAllocator> positionAllocUe = CreateObject<ListPositionAllocator>();
    positionAllocUe->Add(Vector(10, 0, 1.5));
    mobility.SetPositionAllocator(positionAllocUe);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(ueNodes);
    m_ueMobility = ueNodes.Get(0)->GetObject<MobilityModel>();

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, bandwidthAndBWPPair.second);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, bandwidthAndBWPPair.second);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces =
        nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    // A single DL/UL bearer to keep the link active.
    uint16_t dlPort = 10001;
    uint16_t ulPort = 20001;
    DataRateValue dataRateValue = DataRate("18.6Mbps");
    uint64_t bitRate = dataRateValue.Get().GetBitRate();
    uint32_t packetSize = 1024;
    Time udpInterval = Seconds(static_cast<double>(packetSize * 8) / bitRate);

    UdpClientHelper dlClientHelper(ueIpIfaces.GetAddress(0), dlPort);
    dlClientHelper.SetAttribute("Interval", TimeValue(udpInterval));
    dlClientHelper.SetAttribute("MaxPackets", UintegerValue(UINT32_MAX));
    ApplicationContainer dlClientApps = dlClientHelper.Install(remoteHost);

    PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(Ipv4Address::GetAny(), dlPort));
    ApplicationContainer dlServerApps = dlPacketSinkHelper.Install(ueNodes.Get(0));

    UdpClientHelper ulClientHelper(remoteHostAddr, ulPort);
    ulClientHelper.SetAttribute("Interval", TimeValue(udpInterval));
    ulClientHelper.SetAttribute("MaxPackets", UintegerValue(UINT32_MAX));
    ApplicationContainer ulClientApps = ulClientHelper.Install(ueNodes.Get(0));

    PacketSinkHelper ulPacketSinkHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(Ipv4Address::GetAny(), ulPort));
    ApplicationContainer ulServerApps = ulPacketSinkHelper.Install(remoteHost);

    Ptr<NrQosRule> rule = Create<NrQosRule>();
    NrQosRule::PacketFilter dlpf;
    dlpf.localPortStart = dlPort;
    dlpf.localPortEnd = dlPort;
    rule->Add(dlpf);
    NrQosRule::PacketFilter ulpf;
    ulpf.remotePortStart = ulPort;
    ulpf.remotePortEnd = ulPort;
    rule->Add(ulpf);
    NrQosFlow flow(NrQosFlow::NGBR_IMS);
    nrHelper->ActivateDedicatedQosFlow(ueDevs.Get(0), flow, rule);

    dlServerApps.Start(Seconds(0.27));
    dlClientApps.Start(Seconds(0.27));
    ulServerApps.Start(Seconds(0.27));
    ulClientApps.Start(Seconds(0.27));

    // Tear the UE away from gNB 0 towards gNB 1: it loses the link, declares RLF and
    // re-selects / reconnects to gNB 1. This is the reconnecting-RLF'd-UE path that
    // can deliver a stray RrcConnectionRequest to an already-active UeManager state.
    Simulator::Schedule(Seconds(0.4),
                        &NrStrayRrcRequestTestCase::JumpAway,
                        this,
                        Vector(15000.0, 0.0, 1.5));

    nrHelper->AssignStreams({.assignEpc = true,
                             .remoteHostNodes = remoteHostContainer,
                             .ueNodes = ueNodes,
                             .gnbNodes = gnbNodes,
                             .gnbDevs = gnbDevs,
                             .ueDevs = ueDevs,
                             .remoteHostStream = 4000,
                             .ueNodeStream = 3000,
                             .gnbNodeStream = 2000});

    Config::Connect(
        "/NodeList/*/DeviceList/*/NrUeRrc/ConnectionEstablished",
        MakeCallback(&NrStrayRrcRequestTestCase::ConnectionEstablishedUeCallback, this));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/RadioLinkFailure",
                    MakeCallback(&NrStrayRrcRequestTestCase::RadioLinkFailureCallback, this));

    Simulator::Stop(Seconds(4));
    Simulator::Run();

    // (a) At least one RLF must have occurred, otherwise the scenario did not exercise
    // the reconnecting-RLF'd-UE path that the fix guards.
    NS_TEST_ASSERT_MSG_GT_OR_EQ(m_rlfCount,
                                1u,
                                "No RLF occurred; scenario does not exercise the fix");

    // (b) The UE must recover: a ConnectionEstablished after the first RLF and a final
    // CONNECTED_NORMALLY state. Reaching this point at all already proves the fatal
    // error that the fix removes was not triggered.
    NS_TEST_ASSERT_MSG_GT_OR_EQ(m_ueConnEstAfterRlf, 1u, "UE did not reconnect after RLF");

    Ptr<NrUeRrc> ueRrc = DynamicCast<NrUeNetDevice>(ueDevs.Get(0))->GetRrc();
    NS_TEST_ASSERT_MSG_EQ(ueRrc->GetState(),
                          NrUeRrc::CONNECTED_NORMALLY,
                          "UE did not return to CONNECTED_NORMALLY after RLF and reconnection");

    Simulator::Destroy();
}

void
NrStrayRrcRequestTestCase::JumpAway(Vector position)
{
    NS_LOG_FUNCTION(this << position);
    m_ueMobility->SetPosition(position);
}

void
NrStrayRrcRequestTestCase::ConnectionEstablishedUeCallback(std::string context,
                                                           uint64_t imsi,
                                                           uint16_t cellId,
                                                           uint16_t rnti)
{
    NS_LOG_FUNCTION(this << imsi << cellId << rnti);
    ++m_ueConnEstCount;
    if (m_rlfSeen)
    {
        ++m_ueConnEstAfterRlf;
    }
}

void
NrStrayRrcRequestTestCase::RadioLinkFailureCallback(std::string context,
                                                    uint64_t imsi,
                                                    uint16_t cellId,
                                                    uint16_t rnti)
{
    NS_LOG_FUNCTION(this << imsi << cellId << rnti);
    ++m_rlfCount;
    m_rlfSeen = true;
}

/**
 * @ingroup nr-test
 *
 * @brief Test suite for the stray-RrcConnectionRequest fix.
 */
class NrStrayRrcRequestTestSuite : public TestSuite
{
  public:
    NrStrayRrcRequestTestSuite();
};

NrStrayRrcRequestTestSuite::NrStrayRrcRequestTestSuite()
    : TestSuite("nr-stray-rrc-request", Type::SYSTEM)
{
    AddTestCase(new NrStrayRrcRequestTestCase(true), Duration::QUICK);
    AddTestCase(new NrStrayRrcRequestTestCase(false), Duration::QUICK);
}

/// Static instance to register the suite.
static NrStrayRrcRequestTestSuite g_nrStrayRrcRequestTestSuite;
