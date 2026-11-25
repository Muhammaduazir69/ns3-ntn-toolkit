// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-core-roaming.cc
 *
 * @brief Test suite (nr-core-roaming) for roaming between two core networks of the simulation.
 * A UE, subscriber of a home network, attaches through a gNB of its home network, moves to the
 * coverage of a gNB of a visited network, roaming partner of the home network, and comes back.
 * Each move causes a radio link failure, after which the UE selects the gNB of the other network
 * and attaches there. The test checks the attachments (roaming or not) in the MMEs of both
 * networks, the deletion of the session in the network the UE leaves (Cancel Location), the
 * IPv4 and IPv6 addresses of the UE in each network, and the remote hosts (IPv4 and IPv6) that
 * echo the UDP packets of the UE: with local breakout, the UE gets addresses of the visited
 * network, whose remote hosts only answer while the UE is there; with home-routed roaming, the UE
 * keeps its home addresses, and the remote hosts of its home network answer in both networks,
 * through the S8 link.
 */

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/udp-echo-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrCoreRoamingTest");

/**
 * @ingroup test
 * @brief Roaming of a UE between its home network and a visited network, and back.
 */
class NrCoreRoamingTestCase : public TestCase
{
  public:
    /**
     * Constructor.
     * @param homeRouted whether the roaming is home-routed, rather than with local breakout
     */
    NrCoreRoamingTestCase(bool homeRouted)
        : TestCase(std::string("Roaming between two core networks, and back, ") +
                   (homeRouted ? "home-routed" : "with local breakout")),
          m_homeRouted(homeRouted)
    {
    }

  private:
    void DoRun() override;

    /**
     * Record an attachment.
     * @param network the network of the MME
     * @param imsi the IMSI of the UE
     * @param cellId the cell ID of the gNB
     * @param roaming whether the UE is roaming
     */
    void Attach(std::string network, uint64_t imsi, uint16_t cellId, bool roaming)
    {
        m_attachments.push_back(network + "/" + std::to_string(cellId) +
                                (roaming ? "/roaming" : "/home"));
    }

    /**
     * Record the deletion of a session.
     * @param network the network of the MME
     * @param imsi the IMSI of the UE
     */
    void CancelLocation(std::string network, uint64_t imsi)
    {
        m_cancellations.push_back(network);
    }

    /**
     * Record the address of the UE.
     * @param key the key of the record
     * @param ue the UE node
     */
    void RecordAddress(std::string key, Ptr<Node> ue)
    {
        auto ipv4 = ue->GetObject<Ipv4>();
        m_addresses[key] = ipv4->GetAddress(1, 0).GetLocal();
        auto ipv6 = ue->GetObject<Ipv6>();
        const int32_t interface6 = ipv6->GetInterfaceForDevice(ue->GetDevice(0));
        for (uint32_t i = 0; i < ipv6->GetNAddresses(interface6); i++)
        {
            if (ipv6->GetAddress(interface6, i).GetScope() == Ipv6InterfaceAddress::GLOBAL)
            {
                m_addresses6[key].push_back(ipv6->GetAddress(interface6, i).GetAddress());
            }
        }
    }

    /**
     * Count an echo, in the period of its reception.
     * @param host the remote host that echoed the packet
     * @param packet the echoed packet
     */
    void Echo(std::string host, Ptr<const Packet> packet)
    {
        const double t = Simulator::Now().GetSeconds();
        m_echoes[host + " host while " + (t < 3 ? "home" : (t < 9 ? "visited" : "home again"))]++;
    }

    bool m_homeRouted;                              ///< whether the roaming is home-routed
    std::vector<std::string> m_attachments;         ///< attachments: network/cell/roaming
    std::vector<std::string> m_cancellations;       ///< networks that deleted the session
    std::map<std::string, Ipv4Address> m_addresses; ///< addresses of the UE, by period
    /// global IPv6 addresses of the UE, by period
    std::map<std::string, std::vector<Ipv6Address>> m_addresses6;
    std::map<std::string, uint32_t> m_echoes; ///< echoes received, by host and period
};

void
NrCoreRoamingTestCase::DoRun()
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    Config::SetDefault("ns3::NrHelper::UseIdealRrc", BooleanValue(true));
    Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(40));
    Config::SetDefault("ns3::ThreeGppPropagationLossModel::ShadowingEnabled", BooleanValue(false));
    Config::SetDefault("ns3::NrUeRrc::N310", UintegerValue(1));
    Config::SetDefault("ns3::NrUeRrc::N311", UintegerValue(1));
    Config::SetDefault("ns3::NrUeRrc::T310", TimeValue(Seconds(1)));

    // the home network and the visited network, roaming partners, each with its remote host
    auto home = CreateObject<NrPointToPointEpcHelper>();
    auto visited = CreateObjectWithAttributes<NrPointToPointEpcHelper>("UeIpv4Network",
                                                                       Ipv4AddressValue("8.0.0.0"));
    // a delay between the networks, so that the session deleted in the visited network when the
    // UE is back home is deleted in the home PGW after the UE created its new session there
    home->SetAttribute("S8LinkDelay", TimeValue(MilliSeconds(10)));
    home->AddRoamingPartner(visited, m_homeRouted);
    auto [homeHost, homeHostAddress] = home->SetupRemoteHost("100Gb/s", 1500, MilliSeconds(10));
    auto [visitedHost, visitedHostAddress] =
        visited->SetupRemoteHost("100Gb/s", 1500, MilliSeconds(10));
    auto [homeHost6, homeHostAddress6] = home->SetupRemoteHost6("100Gb/s", 1500, MilliSeconds(10));
    auto [visitedHost6, visitedHostAddress6] =
        visited->SetupRemoteHost6("100Gb/s", 1500, MilliSeconds(10));
    home->GetMmeApplication()->TraceConnect("Attach",
                                            "home",
                                            MakeCallback(&NrCoreRoamingTestCase::Attach, this));
    visited->GetMmeApplication()->TraceConnect("Attach",
                                               "visited",
                                               MakeCallback(&NrCoreRoamingTestCase::Attach, this));
    home->GetMmeApplication()->TraceConnect(
        "CancelLocation",
        "home",
        MakeCallback(&NrCoreRoamingTestCase::CancelLocation, this));
    visited->GetMmeApplication()->TraceConnect(
        "CancelLocation",
        "visited",
        MakeCallback(&NrCoreRoamingTestCase::CancelLocation, this));

    NodeContainer gnbNodes(2);
    NodeContainer ueNodes(1);
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    auto positions = CreateObject<ListPositionAllocator>();
    positions->Add(Vector(0, 0, 10));
    positions->Add(Vector(15020, 0, 10));
    positions->Add(Vector(10, 0, 1.5));
    mobility.SetPositionAllocator(positions);
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
    auto bandwidthParts = nrHelper->CreateBandwidthParts({{1.93e9, 10e6, 1}}, "UMa", "LOS");
    nrHelper->SetEpcHelper(home);
    auto homeGnb = nrHelper->InstallGnbDevice(gnbNodes.Get(0), bandwidthParts.second);
    nrHelper->SetEpcHelper(visited);
    auto visitedGnb = nrHelper->InstallGnbDevice(gnbNodes.Get(1), bandwidthParts.second);
    nrHelper->SetEpcHelper(home);
    auto ueDevices = nrHelper->InstallUeDevice(ueNodes, bandwidthParts.second);
    const auto homeCell = DynamicCast<NrGnbNetDevice>(homeGnb.Get(0))->GetCellId();
    const auto visitedCell = DynamicCast<NrGnbNetDevice>(visitedGnb.Get(0))->GetCellId();

    InternetStackHelper internet;
    internet.Install(ueNodes);
    home->AssignUeIpv4Address(ueDevices);
    Ipv4StaticRoutingHelper routing;
    routing.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>())
        ->SetDefaultRoute(home->GetUeDefaultGatewayAddress(), 1);
    // dual stack: an IPv6 address as well, with its default route
    home->AssignUeIpv6Address(ueDevices);
    nrHelper->AttachToGnb(ueDevices.Get(0), homeGnb.Get(0));

    // the UE sends packets to both remote hosts, each one reachable from its network only
    UdpEchoServerHelper echoServer(9);
    for (const auto& host : {homeHost, visitedHost, homeHost6, visitedHost6})
    {
        echoServer.Install(host).Start(Seconds(0.5));
    }
    for (const auto& [host, address] : {std::pair{"home", Address(homeHostAddress)},
                                        std::pair{"visited", Address(visitedHostAddress)},
                                        std::pair{"home6", Address(homeHostAddress6)},
                                        std::pair{"visited6", Address(visitedHostAddress6)}})
    {
        UdpEchoClientHelper echoClient(address, 9);
        echoClient.SetAttribute("Interval", TimeValue(MilliSeconds(100)));
        echoClient.SetAttribute("MaxPackets", UintegerValue(1000));
        auto client = echoClient.Install(ueNodes.Get(0));
        client.Start(Seconds(1));
        client.Get(0)->TraceConnect("Rx", host, MakeCallback(&NrCoreRoamingTestCase::Echo, this));
    }

    // to the visited network, and back home
    auto ueMobility = ueNodes.Get(0)->GetObject<MobilityModel>();
    Simulator::Schedule(Seconds(3), [ueMobility]() { ueMobility->SetPosition({15000, 0, 1.5}); });
    Simulator::Schedule(Seconds(9), [ueMobility]() { ueMobility->SetPosition({10, 0, 1.5}); });
    Simulator::Schedule(MilliSeconds(2900),
                        &NrCoreRoamingTestCase::RecordAddress,
                        this,
                        "home",
                        ueNodes.Get(0));
    Simulator::Schedule(MilliSeconds(8900),
                        &NrCoreRoamingTestCase::RecordAddress,
                        this,
                        "visited",
                        ueNodes.Get(0));
    Simulator::Schedule(MilliSeconds(14900),
                        &NrCoreRoamingTestCase::RecordAddress,
                        this,
                        "home again",
                        ueNodes.Get(0));

    Simulator::Stop(Seconds(15));
    Simulator::Run();

    const std::vector<std::string> expectedAttachments{
        "home/" + std::to_string(homeCell) + "/home",
        "visited/" + std::to_string(visitedCell) + "/roaming",
        "home/" + std::to_string(homeCell) + "/home"};
    NS_TEST_EXPECT_MSG_EQ(m_attachments.size(), expectedAttachments.size(), "Wrong attachments");
    for (size_t i = 0; i < std::min(m_attachments.size(), expectedAttachments.size()); i++)
    {
        NS_TEST_EXPECT_MSG_EQ(m_attachments[i], expectedAttachments[i], "Wrong attachment");
    }
    // the home network deletes its session when the UE roams, the visited network when it is back
    const std::vector<std::string> expectedCancellations{"home", "visited"};
    NS_TEST_EXPECT_MSG_EQ((m_cancellations == expectedCancellations),
                          true,
                          "Wrong deletions of the session");
    NS_TEST_EXPECT_MSG_EQ(m_addresses["home"], Ipv4Address("7.0.0.2"), "Wrong home address");
    NS_TEST_EXPECT_MSG_EQ(m_addresses["visited"],
                          Ipv4Address(m_homeRouted ? "7.0.0.2" : "8.0.0.2"),
                          "Wrong address in the visited network");
    NS_TEST_EXPECT_MSG_EQ(m_addresses["home again"],
                          Ipv4Address("7.0.0.2"),
                          "Home address not restored");
    // IPv6: the prefix of the network of the session, with the interface identifier of the UE
    for (const auto& period : {"home", "visited", "home again"})
    {
        NS_TEST_EXPECT_MSG_EQ(m_addresses6[period].size(),
                              1,
                              "Not a single global IPv6 address while " << period);
        m_addresses6[period].resize(1);
    }
    const Ipv6Prefix prefix(64);
    const Ipv6Address homePrefix = m_addresses6["home"][0].CombinePrefix(prefix);
    const Ipv6Address visitedPrefix = m_addresses6["visited"][0].CombinePrefix(prefix);
    NS_TEST_EXPECT_MSG_EQ(homePrefix, Ipv6Address("7777:f00d::"), "Wrong home IPv6 prefix");
    NS_TEST_EXPECT_MSG_EQ(visitedPrefix,
                          Ipv6Address(m_homeRouted ? "7777:f00d::" : "7777:f00d:0:1::"),
                          "Wrong IPv6 prefix in the visited network");
    NS_TEST_EXPECT_MSG_EQ(m_addresses6["home again"][0],
                          m_addresses6["home"][0],
                          "Home IPv6 address not restored");
    // the network of the session: the visited network with local breakout, the home network
    // with home-routed roaming
    const std::string roamingHost = m_homeRouted ? "home" : "visited";
    const std::string otherHost = m_homeRouted ? "visited" : "home";
    for (const std::string family : {"", "6"})
    {
        NS_TEST_EXPECT_MSG_GT(m_echoes["home" + family + " host while home"],
                              10,
                              "No traffic at home, IPv" << (family.empty() ? 4 : 6));
        NS_TEST_EXPECT_MSG_GT(m_echoes[roamingHost + family + " host while visited"],
                              10,
                              "No traffic while roaming, IPv" << (family.empty() ? 4 : 6));
        NS_TEST_EXPECT_MSG_GT(m_echoes["home" + family + " host while home again"],
                              10,
                              "No traffic back home, IPv" << (family.empty() ? 4 : 6));
        NS_TEST_EXPECT_MSG_EQ(m_echoes["visited" + family + " host while home"] +
                                  m_echoes["visited" + family + " host while home again"] +
                                  m_echoes[otherHost + family + " host while visited"],
                              0,
                              "Traffic through a network that does not have the session, IPv"
                                  << (family.empty() ? 4 : 6));
    }

    Simulator::Destroy();
}

/**
 * @ingroup test
 * @brief Roaming of a dual-stack UE from its home network to a visited network with local
 * breakout, then to a visited network with home-routed roaming: in the latter, the UE gets its
 * home IPv4 and IPv6 addresses back, and reaches the remote hosts of its home network.
 */
class NrMixedRoamingTestCase : public TestCase
{
  public:
    NrMixedRoamingTestCase()
        : TestCase("Roaming with local breakout, then home-routed")
    {
    }

  private:
    void DoRun() override;

    /**
     * Count an echo, in the period of its reception.
     * @param host the remote host that echoed the packet
     * @param packet the echoed packet
     */
    void Echo(std::string host, Ptr<const Packet> packet)
    {
        const double t = Simulator::Now().GetSeconds();
        m_echoes[host + (t < 3 ? " at home" : (t < 9 ? " in local breakout" : " home-routed"))]++;
    }

    std::map<std::string, uint32_t> m_echoes; ///< echoes received, by host and period
};

void
NrMixedRoamingTestCase::DoRun()
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
    Config::SetDefault("ns3::NrHelper::UseIdealRrc", BooleanValue(true));
    Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(40));
    Config::SetDefault("ns3::ThreeGppPropagationLossModel::ShadowingEnabled", BooleanValue(false));
    Config::SetDefault("ns3::NrUeRrc::N310", UintegerValue(1));
    Config::SetDefault("ns3::NrUeRrc::N311", UintegerValue(1));
    Config::SetDefault("ns3::NrUeRrc::T310", TimeValue(Seconds(1)));

    auto home = CreateObject<NrPointToPointEpcHelper>();
    auto localBreakout =
        CreateObjectWithAttributes<NrPointToPointEpcHelper>("UeIpv4Network",
                                                            Ipv4AddressValue("8.0.0.0"));
    auto homeRouted =
        CreateObjectWithAttributes<NrPointToPointEpcHelper>("UeIpv4Network",
                                                            Ipv4AddressValue("9.0.0.0"));
    home->AddRoamingPartner(localBreakout, false);
    home->AddRoamingPartner(homeRouted, true);
    auto [homeHost, homeHostAddress] = home->SetupRemoteHost("100Gb/s", 1500, MilliSeconds(10));
    auto [homeHost6, homeHostAddress6] = home->SetupRemoteHost6("100Gb/s", 1500, MilliSeconds(10));

    NodeContainer gnbNodes(3);
    NodeContainer ueNodes(1);
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    auto positions = CreateObject<ListPositionAllocator>();
    positions->Add(Vector(0, 0, 10));
    positions->Add(Vector(15020, 0, 10));
    positions->Add(Vector(30040, 0, 10));
    positions->Add(Vector(10, 0, 1.5));
    mobility.SetPositionAllocator(positions);
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
    auto bandwidthParts = nrHelper->CreateBandwidthParts({{1.93e9, 10e6, 1}}, "UMa", "LOS");
    NetDeviceContainer gnbDevices;
    for (const auto& [i, helper] :
         {std::pair{0, home}, std::pair{1, localBreakout}, std::pair{2, homeRouted}})
    {
        nrHelper->SetEpcHelper(helper);
        gnbDevices.Add(nrHelper->InstallGnbDevice(gnbNodes.Get(i), bandwidthParts.second));
    }
    nrHelper->SetEpcHelper(home);
    auto ueDevices = nrHelper->InstallUeDevice(ueNodes, bandwidthParts.second);
    InternetStackHelper internet;
    internet.Install(ueNodes);
    home->AssignUeIpv4Address(ueDevices);
    Ipv4StaticRoutingHelper()
        .GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>())
        ->SetDefaultRoute(home->GetUeDefaultGatewayAddress(), 1);
    home->AssignUeIpv6Address(ueDevices);
    nrHelper->AttachToGnb(ueDevices.Get(0), gnbDevices.Get(0));

    UdpEchoServerHelper echoServer(9);
    echoServer.Install(homeHost).Start(Seconds(0.5));
    echoServer.Install(homeHost6).Start(Seconds(0.5));
    for (const auto& [host, address] : {std::pair{"home host", Address(homeHostAddress)},
                                        std::pair{"home host 6", Address(homeHostAddress6)}})
    {
        UdpEchoClientHelper echoClient(address, 9);
        echoClient.SetAttribute("Interval", TimeValue(MilliSeconds(100)));
        echoClient.SetAttribute("MaxPackets", UintegerValue(1000));
        auto client = echoClient.Install(ueNodes.Get(0));
        client.Start(Seconds(1));
        client.Get(0)->TraceConnect("Rx", host, MakeCallback(&NrMixedRoamingTestCase::Echo, this));
    }

    auto ue = ueNodes.Get(0);
    auto ueMobility = ue->GetObject<MobilityModel>();
    Simulator::Schedule(Seconds(3), [ueMobility]() { ueMobility->SetPosition({15000, 0, 1.5}); });
    Simulator::Schedule(Seconds(9), [ueMobility]() { ueMobility->SetPosition({30020, 0, 1.5}); });
    // the addresses: at home, in local breakout and home-routed
    std::vector<std::pair<Ipv4Address, Ipv6Address>> addresses;
    for (const auto& t : {MilliSeconds(2900), MilliSeconds(8900), MilliSeconds(14900)})
    {
        Simulator::Schedule(t, [ue, &addresses]() {
            auto ipv6 = ue->GetObject<Ipv6>();
            const int32_t interface6 = ipv6->GetInterfaceForDevice(ue->GetDevice(0));
            Ipv6Address global;
            for (uint32_t i = 0; i < ipv6->GetNAddresses(interface6); i++)
            {
                if (ipv6->GetAddress(interface6, i).GetScope() == Ipv6InterfaceAddress::GLOBAL)
                {
                    global = ipv6->GetAddress(interface6, i).GetAddress();
                }
            }
            addresses.emplace_back(ue->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal(), global);
        });
    }
    Simulator::Stop(Seconds(15));
    Simulator::Run();

    NS_TEST_EXPECT_MSG_EQ(addresses.size(), 3, "Missing addresses");
    addresses.resize(3);
    NS_TEST_EXPECT_MSG_EQ(addresses[1].first,
                          Ipv4Address("8.0.0.2"),
                          "Wrong address in local breakout");
    NS_TEST_EXPECT_MSG_EQ(addresses[1].second.CombinePrefix(Ipv6Prefix(64)),
                          Ipv6Address("7777:f00d:0:1::"),
                          "Wrong IPv6 prefix in local breakout");
    NS_TEST_EXPECT_MSG_EQ(addresses[2].first,
                          addresses[0].first,
                          "Home address not restored with home-routed roaming");
    NS_TEST_EXPECT_MSG_EQ(addresses[2].second,
                          addresses[0].second,
                          "Home IPv6 address not restored with home-routed roaming");
    for (const std::string host : {"home host", "home host 6"})
    {
        NS_TEST_EXPECT_MSG_GT(m_echoes[host + " at home"], 10, "No traffic at home");
        NS_TEST_EXPECT_MSG_EQ(m_echoes[host + " in local breakout"],
                              0,
                              "Home host reached in local breakout");
        NS_TEST_EXPECT_MSG_GT(m_echoes[host + " home-routed"],
                              10,
                              "No traffic with home-routed roaming, " << host);
    }
    Simulator::Destroy();
}

/**
 * @ingroup test
 * @brief A subscriber served by a network outside of the simulation (e.g., an external 5G core
 * network roaming with the home network through a SEPP): its session in the home network is
 * deleted, and when it attaches again in the home network, the external network is asked to
 * release it.
 */
class NrExternalServingNetworkTestCase : public TestCase
{
  public:
    NrExternalServingNetworkTestCase()
        : TestCase("Subscriber served by an external network")
    {
    }

  private:
    void DoRun() override;
};

void
NrExternalServingNetworkTestCase::DoRun()
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
    Config::SetDefault("ns3::NrHelper::UseIdealRrc", BooleanValue(true));

    auto epcHelper = CreateObject<NrPointToPointEpcHelper>();
    NodeContainer gnbNodes(1);
    NodeContainer ueNodes(1);
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    auto positions = CreateObject<ListPositionAllocator>();
    positions->Add(Vector(0, 0, 10));
    positions->Add(Vector(10, 0, 1.5));
    mobility.SetPositionAllocator(positions);
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    auto bandwidthParts = nrHelper->CreateBandwidthParts({{1.93e9, 10e6, 1}}, "UMa", "LOS");
    auto gnbDevices = nrHelper->InstallGnbDevice(gnbNodes, bandwidthParts.second);
    auto ueDevices = nrHelper->InstallUeDevice(ueNodes, bandwidthParts.second);
    InternetStackHelper internet;
    internet.Install(ueNodes);
    epcHelper->AssignUeIpv4Address(ueDevices);
    nrHelper->AttachToGnb(ueDevices.Get(0), gnbDevices.Get(0));

    auto mme = epcHelper->GetMmeApplication();
    auto ue = DynamicCast<NrUeNetDevice>(ueDevices.Get(0));
    auto gnb = DynamicCast<NrGnbNetDevice>(gnbDevices.Get(0));
    const uint64_t imsi = ue->GetImsi();
    std::vector<std::string> events;
    mme->TraceConnectWithoutContext("CancelLocation", Callback<void, uint64_t>([&events](uint64_t) {
                                        events.emplace_back("cancel");
                                    }));
    mme->TraceConnectWithoutContext(
        "Attach",
        Callback<void, uint64_t, uint16_t, bool>(
            [&events](uint64_t, uint16_t, bool) { events.emplace_back("attach"); }));
    mme->SetExternalCancelLocationCallback(
        Callback<void, uint64_t>([&events](uint64_t) { events.emplace_back("external cancel"); }));
    bool known = false;
    // the external network serves the UE, which releases its session at home
    Simulator::Schedule(Seconds(2),
                        [mme, imsi, &known]() { known = mme->ExternalUpdateLocation(imsi); });
    // the UE attaches at home again
    Simulator::Schedule(MilliSeconds(2500), [ue, gnb]() {
        ue->GetNas()->Connect(gnb->GetCellId(), gnb->GetBwpArfcn(0));
    });
    Simulator::Stop(Seconds(4));
    Simulator::Run();

    NS_TEST_EXPECT_MSG_EQ(known, true, "Subscriber not known");
    NS_TEST_EXPECT_MSG_EQ(mme->ExternalUpdateLocation(imsi + 100), false, "Unknown UE accepted");
    // the external network is released before the UE attaches at home again
    const std::vector<std::string> expected{"attach", "cancel", "external cancel", "attach"};
    NS_TEST_EXPECT_MSG_EQ(events.size(), expected.size(), "Wrong events");
    for (size_t i = 0; i < std::min(events.size(), expected.size()); i++)
    {
        NS_TEST_EXPECT_MSG_EQ(events[i], expected[i], "Wrong event");
    }
    NS_TEST_EXPECT_MSG_EQ(ue->GetRrc()->GetState(),
                          NrUeRrc::CONNECTED_NORMALLY,
                          "UE not connected at home again");
    Simulator::Destroy();
}

/**
 * @ingroup test
 * @brief Release of the context of a UE at its gNB, when another network serves it (S1-AP UE
 * Context Release Command): a command for another UE with the same RNTI is ignored, and the
 * Cancel Location of the UE releases its RRC connection.
 */
class NrUeContextReleaseTestCase : public TestCase
{
  public:
    NrUeContextReleaseTestCase()
        : TestCase("Release of the UE context at the gNB")
    {
    }

  private:
    void DoRun() override;

    /**
     * Record the release of a UE context at the gNB.
     * @param context the context of the trace
     * @param imsi the IMSI of the UE
     * @param cellId the cell ID
     * @param rnti the RNTI of the UE
     */
    void Release(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
    {
        m_releases.push_back(Simulator::Now());
    }

    std::vector<Time> m_releases; ///< times of the releases of the UE context
};

void
NrUeContextReleaseTestCase::DoRun()
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
    Config::SetDefault("ns3::NrHelper::UseIdealRrc", BooleanValue(true));

    auto epcHelper = CreateObject<NrPointToPointEpcHelper>();
    NodeContainer gnbNodes(1);
    NodeContainer ueNodes(1);
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    auto positions = CreateObject<ListPositionAllocator>();
    positions->Add(Vector(0, 0, 10));
    positions->Add(Vector(10, 0, 1.5));
    mobility.SetPositionAllocator(positions);
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    auto bandwidthParts = nrHelper->CreateBandwidthParts({{1.93e9, 10e6, 1}}, "UMa", "LOS");
    auto gnbDevices = nrHelper->InstallGnbDevice(gnbNodes, bandwidthParts.second);
    auto ueDevices = nrHelper->InstallUeDevice(ueNodes, bandwidthParts.second);
    InternetStackHelper internet;
    internet.Install(ueNodes);
    epcHelper->AssignUeIpv4Address(ueDevices);
    nrHelper->AttachToGnb(ueDevices.Get(0), gnbDevices.Get(0));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/NotifyConnectionRelease",
                    MakeCallback(&NrUeContextReleaseTestCase::Release, this));

    auto ue = DynamicCast<NrUeNetDevice>(ueDevices.Get(0));
    auto gnbApp = gnbNodes.Get(0)->GetApplication(0)->GetObject<NrEpcGnbApplication>();
    const uint64_t imsi = ue->GetImsi();
    // the same RNTI, but another UE: ignored
    Simulator::Schedule(MilliSeconds(1500), [ue, gnbApp, imsi]() {
        gnbApp->GetS1apSapGnb()->UeContextReleaseCommand(imsi + 100, ue->GetRrc()->GetRnti());
    });
    // another network serves the UE
    Simulator::Schedule(Seconds(2), [epcHelper, imsi]() {
        epcHelper->GetMmeApplication()->CancelLocation(imsi);
    });
    NrUeRrc::State stateBefore{};
    NrUeRrc::State stateAfter{};
    Simulator::Schedule(MilliSeconds(1900),
                        [ue, &stateBefore]() { stateBefore = ue->GetRrc()->GetState(); });
    Simulator::Schedule(MilliSeconds(2100),
                        [ue, &stateAfter]() { stateAfter = ue->GetRrc()->GetState(); });
    Simulator::Stop(Seconds(3));
    Simulator::Run();

    NS_TEST_EXPECT_MSG_EQ(stateBefore,
                          NrUeRrc::CONNECTED_NORMALLY,
                          "UE not connected before the release");
    NS_TEST_EXPECT_MSG_EQ(m_releases.size(), 1, "Wrong number of releases of the UE context");
    NS_TEST_EXPECT_MSG_EQ((m_releases.empty() ? Time() : m_releases[0]),
                          Seconds(2),
                          "UE context not released on Cancel Location");
    NS_TEST_EXPECT_MSG_NE(stateAfter,
                          NrUeRrc::CONNECTED_NORMALLY,
                          "RRC connection of the UE not released");
    Simulator::Destroy();
}

/**
 * @ingroup test
 * @brief Test suite for the roaming between core networks.
 */
class NrCoreRoamingTestSuite : public TestSuite
{
  public:
    NrCoreRoamingTestSuite()
        : TestSuite("nr-core-roaming", Type::SYSTEM)
    {
        AddTestCase(new NrCoreRoamingTestCase(false), Duration::QUICK);
        AddTestCase(new NrCoreRoamingTestCase(true), Duration::QUICK);
        AddTestCase(new NrMixedRoamingTestCase, Duration::QUICK);
        AddTestCase(new NrUeContextReleaseTestCase, Duration::QUICK);
        AddTestCase(new NrExternalServingNetworkTestCase, Duration::QUICK);
    }
};

static NrCoreRoamingTestSuite g_nrCoreRoamingTestSuite; //!< the test suite
