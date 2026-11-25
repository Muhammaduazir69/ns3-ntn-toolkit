// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-fdd-ul-routing.cc
 *
 * @brief Test suite (nr-fdd-ul-routing) verifying that uplink traffic of a QoS flow the
 * network pins to a DL-only FDD carrier is served by the paired UL carrier. The gNB
 * advertises its 5QI-to-BWP mapping in dedicated RRC configuration to drive its downlink
 * scheduling; the UE must not route its own uplink transmissions of that flow to the
 * DL-only BWP (which has no UL slots), but to the UL carrier the BWP is paired with.
 */

#include "ns3/boolean.h"
#include "ns3/bwp-manager-algorithm.h"
#include "ns3/bwp-manager-ue.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nstime.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/pointer.h"
#include "ns3/position-allocator.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/udp-echo-helper.h"
#include "ns3/uinteger.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrFddUlRoutingTest");

/**
 * @ingroup nr-test
 *
 * @brief UL traffic of a 5QI mapped to the DL-only FDD carrier must flow on the
 * paired UL carrier, with the mapping learned from dedicated RRC configuration.
 */
class NrFddUlRoutingTestCase : public TestCase
{
  public:
    /**
     * Constructor
     * @param name the name of the test case, to be displayed in the test result
     */
    NrFddUlRoutingTestCase(std::string name)
        : TestCase(name)
    {
    }

  private:
    void DoRun() override;
};

void
NrFddUlRoutingTestCase::DoRun()
{
    auto nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(nrEpcHelper);

    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    Config::SetDefault("ns3::LogDistancePropagationLossModel::Exponent", DoubleValue(3.5));
    Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue(35));

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(LogDistancePropagationLossModel::GetTypeId());

    // One DL band and one UL band, one BWP each: a pure FDD cell
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf dlBandConf(2.8e9, 40e6, static_cast<uint8_t>(1));
    CcBwpCreator::SimpleOperationBandConf ulBandConf(2.9e9, 40e6, static_cast<uint8_t>(1));
    OperationBandInfo dlBand = ccBwpCreator.CreateOperationBandContiguousCc(dlBandConf);
    OperationBandInfo ulBand = ccBwpCreator.CreateOperationBandContiguousCc(ulBandConf);
    channelHelper->AssignChannelsToBands({dlBand, ulBand});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({dlBand, ulBand});

    // The gNB pins the QoS flow to BWP 0, the DL-only carrier, for its downlink
    // scheduling. Nothing is configured on the UE side: the UE must derive from
    // the advertised mapping and the carrier pairing that its uplink traffic of
    // this flow goes out on BWP 1, the UL-only carrier.
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT", UintegerValue(0));

    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    auto ueNode = CreateObject<Node>();

    auto posAlloc = CreateObject<ListPositionAllocator>();
    posAlloc->Add(Vector(0, 0, 0));
    posAlloc->Add(Vector(100, 0, 0));
    MobilityHelper mobilityHelper;
    mobilityHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityHelper.SetPositionAllocator(posAlloc);
    mobilityHelper.Install(gnbNodes);
    mobilityHelper.Install(ueNode);

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    auto ueDev = nrHelper->InstallUeDevice(ueNode, allBwps).Get(0);

    NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)
        ->SetAttribute("Pattern", StringValue("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|"));
    NrHelper::GetGnbPhy(gnbDevs.Get(0), 1)
        ->SetAttribute("Pattern", StringValue("UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|"));

    InternetStackHelper inetStackHelper;
    inetStackHelper.Install(ueNode);
    Ipv4InterfaceContainer ueIfs = nrEpcHelper->AssignUeIpv4Address(ueDev);

    auto [remoteHost, remoteHostAddr] =
        nrEpcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0.000));

    nrHelper->AttachToGnb(ueDev, gnbDevs.Get(0));

    // Downlink and uplink UDP traffic on the default bearer, whose 5QI is
    // NGBR_VIDEO_TCP_DEFAULT, i.e. the one the gNB pinned to the DL-only BWP
    const uint16_t dlPort = 2000;
    const uint16_t ulPort = 2001;
    const uint32_t numPkts = 200;

    PacketSinkHelper dlSinkHelper("ns3::UdpSocketFactory",
                                  InetSocketAddress(Ipv4Address::GetAny(), dlPort));
    ApplicationContainer dlSinkApps = dlSinkHelper.Install(ueNode);
    dlSinkApps.Start(Seconds(0.4));
    auto dlSink = dlSinkApps.Get(0)->GetObject<PacketSink>();

    UdpEchoClientHelper dlClient(ueIfs.GetAddress(0), dlPort);
    dlClient.SetAttribute("MaxPackets", UintegerValue(numPkts));
    dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(5)));
    dlClient.SetAttribute("PacketSize", UintegerValue(500));
    ApplicationContainer dlClientApps = dlClient.Install(remoteHost);
    dlClientApps.Start(Seconds(0.4));
    dlClientApps.Stop(Seconds(1.4));

    PacketSinkHelper ulSinkHelper("ns3::UdpSocketFactory",
                                  InetSocketAddress(Ipv4Address::GetAny(), ulPort));
    ApplicationContainer ulSinkApps = ulSinkHelper.Install(remoteHost);
    ulSinkApps.Start(Seconds(0.4));
    auto ulSink = ulSinkApps.Get(0)->GetObject<PacketSink>();

    UdpEchoClientHelper ulClient(remoteHostAddr, ulPort);
    ulClient.SetAttribute("MaxPackets", UintegerValue(numPkts));
    ulClient.SetAttribute("Interval", TimeValue(MilliSeconds(5)));
    ulClient.SetAttribute("PacketSize", UintegerValue(500));
    ApplicationContainer ulClientApps = ulClient.Install(ueNode);
    ulClientApps.Start(Seconds(0.4));
    ulClientApps.Stop(Seconds(1.4));

    Simulator::Stop(Seconds(2));
    Simulator::Run();

    // The UE's BWP manager must map the flow's 5QI to the UL-only BWP (index 1),
    // not to the DL-only BWP the gNB advertised for downlink scheduling
    Ptr<BwpManagerUe> bwpManagerUe = NrHelper::GetBwpManagerUe(ueDev);
    PointerValue algoPtr;
    bwpManagerUe->GetAttribute("BwpManagerAlgorithm", algoPtr);
    auto algo = DynamicCast<BwpManagerAlgorithmStatic>(algoPtr.GetObject());
    NS_TEST_ASSERT_MSG_EQ(algo == nullptr,
                          false,
                          "UE BWP manager algorithm is not BwpManagerAlgorithmStatic");
    NS_TEST_ASSERT_MSG_EQ(+algo->GetBwpForQosFlow(NrQosFlow::NGBR_VIDEO_TCP_DEFAULT),
                          1,
                          "UE did not map the 5QI to the paired UL BWP for its uplink");

    NS_TEST_ASSERT_MSG_GT(dlSink->GetTotalRx(), 0, "no downlink packet reached the UE");
    NS_TEST_ASSERT_MSG_GT(ulSink->GetTotalRx(),
                          0,
                          "no uplink packet reached the remote host: UL traffic of a flow "
                          "pinned to a DL-only BWP was not routed to the paired UL carrier");

    Simulator::Destroy();
}

/**
 * @ingroup nr-test
 *
 * @brief A UE output link explicitly installed by the scenario takes precedence
 * over the pairing derived from the gNB's dedicated RRC configuration.
 */
class NrFddUlRoutingManualOverrideTestCase : public TestCase
{
  public:
    /**
     * Constructor
     * @param name the name of the test case, to be displayed in the test result
     */
    NrFddUlRoutingManualOverrideTestCase(std::string name)
        : TestCase(name)
    {
    }

  private:
    void DoRun() override;
};

void
NrFddUlRoutingManualOverrideTestCase::DoRun()
{
    auto nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(nrEpcHelper);

    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    Config::SetDefault("ns3::LogDistancePropagationLossModel::Exponent", DoubleValue(3.5));
    Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue(35));

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(LogDistancePropagationLossModel::GetTypeId());

    // Three BWPs: BWP 0 is a TDD carrier (the primary), BWP 1 a DL-only carrier
    // and BWP 2 an UL-only carrier, forming an FDD pair
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf tddBandConf(2.8e9, 40e6, static_cast<uint8_t>(1));
    CcBwpCreator::SimpleOperationBandConf fddBandConf(3.0e9, 40e6, static_cast<uint8_t>(1));
    fddBandConf.m_numBwp = 2; // the FDD band holds the DL-only and the UL-only BWP
    OperationBandInfo tddBand = ccBwpCreator.CreateOperationBandContiguousCc(tddBandConf);
    OperationBandInfo fddBand = ccBwpCreator.CreateOperationBandContiguousCc(fddBandConf);
    channelHelper->AssignChannelsToBands({tddBand, fddBand});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({tddBand, fddBand});

    // The gNB pins the QoS flow to BWP 1, the DL-only carrier, for its downlink
    // scheduling
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT", UintegerValue(1));

    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    auto ueNode = CreateObject<Node>();

    auto posAlloc = CreateObject<ListPositionAllocator>();
    posAlloc->Add(Vector(0, 0, 0));
    posAlloc->Add(Vector(100, 0, 0));
    MobilityHelper mobilityHelper;
    mobilityHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityHelper.SetPositionAllocator(posAlloc);
    mobilityHelper.Install(gnbNodes);
    mobilityHelper.Install(ueNode);

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    auto ueDev = nrHelper->InstallUeDevice(ueNode, allBwps).Get(0);

    NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)
        ->SetAttribute("Pattern", StringValue("F|F|F|F|F|F|F|F|F|F|"));
    NrHelper::GetGnbPhy(gnbDevs.Get(0), 1)
        ->SetAttribute("Pattern", StringValue("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|"));
    NrHelper::GetGnbPhy(gnbDevs.Get(0), 2)
        ->SetAttribute("Pattern", StringValue("UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|"));

    // Explicit scenario configuration: the DL-only BWP's uplink goes out through
    // the TDD BWP, NOT through the FDD-UL BWP the gNB advertises as its pair.
    // Being explicit, the advertised pairing must not overwrite it.
    NrHelper::GetBwpManagerUe(ueDev)->SetOutputLink(1, 0);

    InternetStackHelper inetStackHelper;
    inetStackHelper.Install(ueNode);
    Ipv4InterfaceContainer ueIfs = nrEpcHelper->AssignUeIpv4Address(ueDev);

    auto [remoteHost, remoteHostAddr] =
        nrEpcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0.000));

    nrHelper->AttachToGnb(ueDev, gnbDevs.Get(0));

    const uint16_t dlPort = 2000;
    const uint16_t ulPort = 2001;
    const uint32_t numPkts = 200;

    PacketSinkHelper dlSinkHelper("ns3::UdpSocketFactory",
                                  InetSocketAddress(Ipv4Address::GetAny(), dlPort));
    ApplicationContainer dlSinkApps = dlSinkHelper.Install(ueNode);
    dlSinkApps.Start(Seconds(0.4));
    auto dlSink = dlSinkApps.Get(0)->GetObject<PacketSink>();

    UdpEchoClientHelper dlClient(ueIfs.GetAddress(0), dlPort);
    dlClient.SetAttribute("MaxPackets", UintegerValue(numPkts));
    dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(5)));
    dlClient.SetAttribute("PacketSize", UintegerValue(500));
    ApplicationContainer dlClientApps = dlClient.Install(remoteHost);
    dlClientApps.Start(Seconds(0.4));
    dlClientApps.Stop(Seconds(1.4));

    PacketSinkHelper ulSinkHelper("ns3::UdpSocketFactory",
                                  InetSocketAddress(Ipv4Address::GetAny(), ulPort));
    ApplicationContainer ulSinkApps = ulSinkHelper.Install(remoteHost);
    ulSinkApps.Start(Seconds(0.4));
    auto ulSink = ulSinkApps.Get(0)->GetObject<PacketSink>();

    UdpEchoClientHelper ulClient(remoteHostAddr, ulPort);
    ulClient.SetAttribute("MaxPackets", UintegerValue(numPkts));
    ulClient.SetAttribute("Interval", TimeValue(MilliSeconds(5)));
    ulClient.SetAttribute("PacketSize", UintegerValue(500));
    ApplicationContainer ulClientApps = ulClient.Install(ueNode);
    ulClientApps.Start(Seconds(0.4));
    ulClientApps.Stop(Seconds(1.4));

    Simulator::Stop(Seconds(2));
    Simulator::Run();

    // The explicit link survived the RRC-driven configuration
    Ptr<BwpManagerUe> bwpManagerUe = NrHelper::GetBwpManagerUe(ueDev);
    NS_TEST_ASSERT_MSG_EQ(bwpManagerUe->GetOutputLink(1),
                          0,
                          "explicit UE output link was overwritten by the advertised pairing");

    // The 5QI mapping still follows the advertised pairing: the flow is pinned
    // to the DL-only BWP 1, whose advertised UL pair is BWP 2
    PointerValue algoPtr;
    bwpManagerUe->GetAttribute("BwpManagerAlgorithm", algoPtr);
    auto algo = DynamicCast<BwpManagerAlgorithmStatic>(algoPtr.GetObject());
    NS_TEST_ASSERT_MSG_EQ(algo == nullptr,
                          false,
                          "UE BWP manager algorithm is not BwpManagerAlgorithmStatic");
    NS_TEST_ASSERT_MSG_EQ(+algo->GetBwpForQosFlow(NrQosFlow::NGBR_VIDEO_TCP_DEFAULT),
                          2,
                          "UE did not map the 5QI to the advertised UL pair");

    NS_TEST_ASSERT_MSG_GT(dlSink->GetTotalRx(), 0, "no downlink packet reached the UE");
    NS_TEST_ASSERT_MSG_GT(ulSink->GetTotalRx(), 0, "no uplink packet reached the remote host");

    Simulator::Destroy();
}

/**
 * @ingroup nr-test
 * @brief Test suite nr-fdd-ul-routing
 */
class NrFddUlRoutingTestSuite : public TestSuite
{
  public:
    NrFddUlRoutingTestSuite()
        : TestSuite("nr-fdd-ul-routing", Type::SYSTEM)
    {
        AddTestCase(new NrFddUlRoutingTestCase(
                        "UL flow on a DL-only-mapped 5QI is served by the paired UL carrier"),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrFddUlRoutingManualOverrideTestCase(
                        "Manual UE output link overrides the RRC-advertised pairing"),
                    TestCase::Duration::QUICK);
    }
};

/// Static variable for test initialization
static NrFddUlRoutingTestSuite g_nrFddUlRoutingTestSuite;
