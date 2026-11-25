// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-system-test-scheduler-many-ues.cc
 *
 * @brief System test that checks that a cell with many simultaneously active
 * UEs does not starve. When the number of active UEs is greater than or equal
 * to the available resources in a slot, each UE receives an allocation whose
 * TBS at the initial MCS is below the minimum required to create a DCI. If the
 * scheduler does not reap and redistribute those allocations, no data is ever
 * transmitted, no CSI feedback is generated, the MCS never improves, and the
 * cell deadlocks. Unlike the nr-system-test-schedulers-* suites, this test
 * does not fix the MCS: it relies on the default adaptive MCS starting at 0,
 * which is the condition that triggers the starvation. One gNB and 100 UEs
 * exchange light CBR traffic in downlink and uplink, and the test asserts that
 * every UE sends and receives at least one packet in each direction. A notched
 * variant of the TDMA case blanks half of the RBGs, so that the reaping of
 * sub-minimum-TBS allocations runs with fewer assignable RBGs per symbol than
 * the full bandwidth.
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/config.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/pointer.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/uinteger.h"

namespace ns3
{

/**
 * @ingroup test
 * @brief Test that a scheduler serves all UEs of a heavily loaded cell in both
 * directions, starting from the default MCS.
 */
class NrSystemTestSchedulerManyUes : public TestCase
{
  public:
    /**
     * @brief Constructor.
     *
     * @param name The unique test configuration name.
     * @param ueNum The number of UEs attached to the gNB.
     * @param schedulerType The TypeId name of the scheduler under test.
     * @param notching True to notch (blank) part of the RBGs, so that the
     * scheduler assigns fewer RBGs per symbol than the full bandwidth.
     */
    NrSystemTestSchedulerManyUes(const std::string& name,
                                 uint32_t ueNum,
                                 const std::string& schedulerType,
                                 bool notching = false);

  private:
    void DoRun() override;

    uint32_t m_ueNum;            //!< Number of UEs attached to the gNB
    std::string m_schedulerType; //!< TypeId name of the scheduler under test
    bool m_notching;             //!< True to notch part of the RBGs
};

NrSystemTestSchedulerManyUes::NrSystemTestSchedulerManyUes(const std::string& name,
                                                           uint32_t ueNum,
                                                           const std::string& schedulerType,
                                                           bool notching)
    : TestCase(name),
      m_ueNum(ueNum),
      m_schedulerType(schedulerType),
      m_notching(notching)
{
}

void
NrSystemTestSchedulerManyUes::DoRun()
{
    const Time simTime = MilliSeconds(1300);
    const Time udpAppStartTime = MilliSeconds(400);
    const Time udpAppStopTime = MilliSeconds(1200);
    const uint32_t packetSize = 100;
    const Time udpInterval = MilliSeconds(100);
    const double centralFrequency = 2e9;
    const double bandwidth = 10e6;
    const uint32_t numerology = 0;

    // Disable channel updates to speed up the simulation
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));

    NodeContainer gNbNodes;
    NodeContainer ueNodes;
    gNbNodes.Create(1);
    ueNodes.Create(m_ueNum);

    Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator>();
    gnbPositionAlloc->Add(Vector(0.0, 0.0, 10.0));
    for (uint32_t i = 0; i < m_ueNum; ++i)
    {
        uePositionAlloc->Add(Vector(1.0 + 0.1 * i, 10.0 + 0.1 * i, 1.5));
    }
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(gnbPositionAlloc);
    mobility.Install(gNbNodes);
    mobility.SetPositionAllocator(uePositionAlloc);
    mobility.Install(ueNodes);

    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "LOS");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(numerology));

    // Do not fix the MCS: the starvation this test guards against can only be
    // triggered when the UEs start from the default MCS 0
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName(m_schedulerType));

    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});
    allBwps = CcBwpCreator::GetAllBwps({band});

    NetDeviceContainer gNbNetDevs = nrHelper->InstallGnbDevice(gNbNodes, allBwps);
    NetDeviceContainer ueNetDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
    nrHelper->AssignStreams({.gnbDevs = gNbNetDevs, .ueDevs = ueNetDevs});

    if (m_notching)
    {
        // Notch the lower half of the RBGs in both directions, so that the
        // scheduler (and in particular the reaping of sub-minimum-TBS
        // allocations) operates with fewer assignable RBGs per symbol than
        // the full bandwidth. 10 MHz with numerology 0 yields 53 RBs.
        const uint32_t rbNum = 53;
        std::vector<bool> notchedMask(rbNum, true);
        for (uint32_t i = 0; i < rbNum / 2; ++i)
        {
            notchedMask[i] = false;
        }
        auto scheduler =
            DynamicCast<NrMacSchedulerNs3>(NrHelper::GetScheduler(gNbNetDevs.Get(0), 0));
        NS_TEST_ASSERT_MSG_NE(scheduler, nullptr, "Could not retrieve the gNB scheduler");
        scheduler->SetDlNotchedRbgMask(notchedMask);
        scheduler->SetUlNotchedRbgMask(notchedMask);
    }

    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.0)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface = nrEpcHelper->AssignUeIpv4Address(ueNetDevs);

    nrHelper->AttachToClosestGnb(ueNetDevs, gNbNetDevs);

    // Per-UE DL flow (remote host -> UE) and UL flow (UE -> remote host).
    // All the clients start at the same time, so all the UEs are active
    // simultaneously, which is the condition that triggers the starvation.
    const uint16_t dlPort = 1234;
    const uint16_t ulPortBase = 2000;
    ApplicationContainer serverAppsDl;
    ApplicationContainer serverAppsUl;
    ApplicationContainer clientApps;

    for (uint32_t i = 0; i < m_ueNum; ++i)
    {
        UdpServerHelper dlServer(dlPort);
        serverAppsDl.Add(dlServer.Install(ueNodes.Get(i)));

        UdpClientHelper dlClient(ueIpIface.GetAddress(i), dlPort);
        dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        dlClient.SetAttribute("PacketSize", UintegerValue(packetSize));
        dlClient.SetAttribute("Interval", TimeValue(udpInterval));
        clientApps.Add(dlClient.Install(remoteHost));

        const uint16_t ulPort = ulPortBase + i;
        UdpServerHelper ulServer(ulPort);
        serverAppsUl.Add(ulServer.Install(remoteHost));

        UdpClientHelper ulClient(remoteHostAddr, ulPort);
        ulClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        ulClient.SetAttribute("PacketSize", UintegerValue(packetSize));
        ulClient.SetAttribute("Interval", TimeValue(udpInterval));
        clientApps.Add(ulClient.Install(ueNodes.Get(i)));
    }

    serverAppsDl.Start(udpAppStartTime);
    serverAppsUl.Start(udpAppStartTime);
    clientApps.Start(udpAppStartTime);
    serverAppsDl.Stop(udpAppStopTime);
    serverAppsUl.Stop(udpAppStopTime);
    clientApps.Stop(udpAppStopTime);

    Simulator::Stop(simTime);
    Simulator::Run();

    for (uint32_t i = 0; i < m_ueNum; ++i)
    {
        Ptr<UdpServer> dlServerApp = serverAppsDl.Get(i)->GetObject<UdpServer>();
        NS_TEST_ASSERT_MSG_GT(dlServerApp->GetReceived(),
                              0,
                              "UE " << i << " did not receive any DL packet");
        Ptr<UdpServer> ulServerApp = serverAppsUl.Get(i)->GetObject<UdpServer>();
        NS_TEST_ASSERT_MSG_GT(ulServerApp->GetReceived(),
                              0,
                              "UE " << i << " did not deliver any UL packet");
    }

    Simulator::Destroy();
}

/**
 * @ingroup test
 * @brief The test suite checking that a cell with many active UEs does not
 * starve under the TDMA and OFDMA round robin schedulers.
 */
class NrSystemTestSchedulerManyUesSuite : public TestSuite
{
  public:
    NrSystemTestSchedulerManyUesSuite();
};

NrSystemTestSchedulerManyUesSuite::NrSystemTestSchedulerManyUesSuite()
    : TestSuite("nr-system-test-scheduler-many-ues", Type::SYSTEM)
{
    const uint32_t ueNum = 100;
    AddTestCase(new NrSystemTestSchedulerManyUes("DL_UL, Tdma RR, 100 UEs, 1 beam, Num 0",
                                                 ueNum,
                                                 "ns3::NrMacSchedulerTdmaRR"),
                Duration::QUICK);
    AddTestCase(new NrSystemTestSchedulerManyUes("DL_UL, Ofdma RR, 100 UEs, 1 beam, Num 0",
                                                 ueNum,
                                                 "ns3::NrMacSchedulerOfdmaRR"),
                Duration::QUICK);
    AddTestCase(new NrSystemTestSchedulerManyUes("DL_UL, Tdma RR, 100 UEs, 1 beam, Num 0, notched",
                                                 ueNum,
                                                 "ns3::NrMacSchedulerTdmaRR",
                                                 true),
                Duration::QUICK);
}

static NrSystemTestSchedulerManyUesSuite g_nrSystemTestSchedulerManyUesSuite; //!< Suite instance

} // namespace ns3
