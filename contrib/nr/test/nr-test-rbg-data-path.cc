// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-rbg-data-path.cc
 *
 * @brief Test suite (nr-rbg-data-path) verifying that DL and UL data flows work with
 * NumRbPerRbg > 1. The DCI carries an RBG bitmask sized by the number of RBGs in the
 * BWP (RBs / NumRbPerRbg), which the UE must compare against its own RBG count and not
 * against its plain RB count; with the wrong comparison every transport block is
 * silently discarded and no data flows.
 */

#include "ns3/boolean.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nstime.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/position-allocator.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/udp-echo-helper.h"
#include "ns3/uinteger.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrRbgDataPathTest");

/**
 * @ingroup nr-test
 *
 * @brief DL and UL data flow with NumRbPerRbg > 1.
 */
class NrRbgDataPathTestCase : public TestCase
{
  public:
    /**
     * Constructor
     * @param name the name of the test case, to be displayed in the test result
     * @param numRbPerRbg number of RBs per RBG to configure
     */
    NrRbgDataPathTestCase(std::string name, uint32_t numRbPerRbg)
        : TestCase(name),
          m_numRbPerRbg(numRbPerRbg)
    {
    }

  private:
    void DoRun() override;

    uint32_t m_numRbPerRbg; ///< Number of RBs per RBG
};

void
NrRbgDataPathTestCase::DoRun()
{
    auto nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(nrEpcHelper);

    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetAttribute("NumRbPerRbg", UintegerValue(m_numRbPerRbg));

    Config::SetDefault("ns3::LogDistancePropagationLossModel::Exponent", DoubleValue(3.5));
    Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue(35));

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(LogDistancePropagationLossModel::GetTypeId());

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(2.8e9, 40e6, static_cast<uint8_t>(1));
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

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

    NS_TEST_ASSERT_MSG_GT(dlSink->GetTotalRx(),
                          0,
                          "no downlink packet reached the UE with NumRbPerRbg > 1");
    NS_TEST_ASSERT_MSG_GT(ulSink->GetTotalRx(),
                          0,
                          "no uplink packet reached the remote host with NumRbPerRbg > 1");

    Simulator::Destroy();
}

/**
 * @ingroup nr-test
 * @brief Test suite nr-rbg-data-path
 */
class NrRbgDataPathTestSuite : public TestSuite
{
  public:
    NrRbgDataPathTestSuite()
        : TestSuite("nr-rbg-data-path", Type::SYSTEM)
    {
        AddTestCase(new NrRbgDataPathTestCase("DL and UL data flow with NumRbPerRbg=2", 2),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrRbgDataPathTestCase("DL and UL data flow with NumRbPerRbg=4", 4),
                    TestCase::Duration::QUICK);
    }
};

/// Static variable for test initialization
static NrRbgDataPathTestSuite g_nrRbgDataPathTestSuite;
