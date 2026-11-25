// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Manuel Requena <manuel.requena@cttc.es>
//         (based on the original point-to-point-epc-helper.cc)

#include "nr-no-backhaul-epc-helper.h"

#include "ns3/boolean.h"
#include "ns3/iana-ieee802-numbers.h"
#include "ns3/icmpv6-l4-protocol.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-generator.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv6-routing-table-entry.h"
#include "ns3/ipv6-static-routing-helper.h"
#include "ns3/log.h"
#include "ns3/node-list.h"
#include "ns3/nr-epc-gnb-application.h"
#include "ns3/nr-epc-mme-application.h"
#include "ns3/nr-epc-pgw-application.h"
#include "ns3/nr-epc-sgw-application.h"
#include "ns3/nr-epc-ue-nas.h"
#include "ns3/nr-epc-x2.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-gnb-rrc.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/packet-socket-address.h"
#include "ns3/point-to-point-helper.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrNoBackhaulEpcHelper");

NS_OBJECT_ENSURE_REGISTERED(NrNoBackhaulEpcHelper);

namespace
{

/**
 * @param base an IPv6 prefix of /64 or shorter
 * @param coreIndex the index of a core network
 * @return the prefix with the index of the core network in its 7th and 8th octets
 */
Ipv6Address
CoreIpv6Prefix(const char* base, uint32_t coreIndex)
{
    uint8_t bytes[16];
    Ipv6Address(base).GetBytes(bytes);
    bytes[6] = static_cast<uint8_t>(coreIndex >> 8);
    bytes[7] = static_cast<uint8_t>(coreIndex);
    return Ipv6Address(bytes);
}

} // namespace

NrNoBackhaulEpcHelper::NrNoBackhaulEpcHelper()
    : m_gtpuUdpPort(2152), // fixed by the standard
      m_s11LinkDataRate(DataRate("10Gb/s")),
      m_s11LinkDelay(Seconds(0)),
      m_s11LinkMtu(3000),
      m_gtpcUdpPort(2123), // fixed by the standard
      m_s5LinkDataRate(DataRate("10Gb/s")),
      m_s5LinkDelay(Seconds(0)),
      m_s5LinkMtu(3000)
{
    NS_LOG_FUNCTION(this);
}

void
NrNoBackhaulEpcHelper::NotifyConstructionCompleted()
{
    NrRanEpcHelper::NotifyConstructionCompleted();

    int retval;

    // since we use point-to-point links for links between the core network nodes,
    // we use a /30 subnet which can hold exactly two addresses
    // (remember that net broadcast and null address are not valid)
    m_s11Ipv4AddressHelper.SetBase(GetCoreSubnet(13), "255.255.255.252");
    m_s5Ipv4AddressHelper.SetBase(GetCoreSubnet(14), "255.255.255.252");

    // the first address of the UE network is the one of the TUN device of the PGW
    NS_ABORT_MSG_IF(
        Ipv4AddressGenerator::IsAddressAllocated(Ipv4Address(m_ueIpv4Network.Get() + 1)),
        "Another core network uses the UE network " << m_ueIpv4Network
                                                    << ": set the UeIpv4Network attribute");
    m_uePgwAddressHelper.SetBase(m_ueIpv4Network, m_ueIpv4Mask);

    // we use a /64 IPv6 net all UEs
    m_uePgwAddressHelper6.SetBase(CoreIpv6Prefix("7777:f00d::", GetCoreIndex()), Ipv6Prefix(64));

    // Create PGW, SGW and MME nodes
    m_pgw = CreateObject<Node>();
    m_sgw = CreateObject<Node>();
    m_mme = CreateObject<Node>();
    InternetStackHelper internet;
    internet.Install(m_pgw);
    internet.Install(m_sgw);
    internet.Install(m_mme);

    // The Tun device resides in different 64 bit subnet.
    // We must create an unique route to tun device for all the packets destined
    // to all 64 bit IPv6 prefixes of UEs, based by the unique 48 bit network prefix of this EPC
    // network
    Ipv6StaticRoutingHelper ipv6RoutingHelper;
    Ptr<Ipv6StaticRouting> pgwStaticRouting =
        ipv6RoutingHelper.GetStaticRouting(m_pgw->GetObject<Ipv6>());
    pgwStaticRouting->AddNetworkRouteTo(CoreIpv6Prefix("7777:f00d::", GetCoreIndex()),
                                        Ipv6Prefix(64),
                                        Ipv6Address("::"),
                                        1,
                                        0);

    // create TUN device implementing tunneling of user data over GTP-U/UDP/IP in the PGW
    m_tunDevice = CreateObject<VirtualNetDevice>();

    // allow jumbo packets
    m_tunDevice->SetAttribute("Mtu", UintegerValue(30000));

    // yes we need this
    m_tunDevice->SetAddress(Mac48Address::Allocate());

    m_pgw->AddDevice(m_tunDevice);
    NetDeviceContainer tunDeviceContainer;
    tunDeviceContainer.Add(m_tunDevice);
    // the TUN device is on the same subnet as the UEs, so when a packet
    // addressed to an UE arrives at the internet to the WAN interface of
    // the PGW it will be forwarded to the TUN device.
    Ipv4InterfaceContainer tunDeviceIpv4IfContainer = AssignUeIpv4Address(tunDeviceContainer);

    // the TUN device for IPv6 address is on the different subnet as the
    // UEs, it will forward the UE packets as we have inserted the route
    // for all UEs at the time of assigning UE addresses
    Ipv6InterfaceContainer tunDeviceIpv6IfContainer = AssignUeIpv6Address(tunDeviceContainer);

    // Set Forwarding of the IPv6 interface
    tunDeviceIpv6IfContainer.SetForwarding(0, true);
    tunDeviceIpv6IfContainer.SetDefaultRouteInAllNodes(0);

    // Create S5 link between PGW and SGW
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(m_s5LinkDataRate));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(m_s5LinkMtu));
    p2ph.SetChannelAttribute("Delay", TimeValue(m_s5LinkDelay));
    NetDeviceContainer pgwSgwDevices = p2ph.Install(m_pgw, m_sgw);
    NS_LOG_LOGIC("IPv4 ifaces of the PGW after installing p2p dev: "
                 << m_pgw->GetObject<Ipv4>()->GetNInterfaces());
    NS_LOG_LOGIC("IPv4 ifaces of the SGW after installing p2p dev: "
                 << m_sgw->GetObject<Ipv4>()->GetNInterfaces());
    Ptr<NetDevice> pgwDev = pgwSgwDevices.Get(0);
    Ptr<NetDevice> sgwDev = pgwSgwDevices.Get(1);
    m_s5Ipv4AddressHelper.NewNetwork();
    Ipv4InterfaceContainer pgwSgwIpIfaces = m_s5Ipv4AddressHelper.Assign(pgwSgwDevices);
    NS_LOG_LOGIC("IPv4 ifaces of the PGW after assigning Ipv4 addr to S5 dev: "
                 << m_pgw->GetObject<Ipv4>()->GetNInterfaces());
    NS_LOG_LOGIC("IPv4 ifaces of the SGW after assigning Ipv4 addr to S5 dev: "
                 << m_sgw->GetObject<Ipv4>()->GetNInterfaces());

    Ipv4Address pgwS5Address = pgwSgwIpIfaces.GetAddress(0);
    Ipv4Address sgwS5Address = pgwSgwIpIfaces.GetAddress(1);
    m_pgwS5Address = pgwS5Address;
    m_sgwS5Address = sgwS5Address;

    // Create S5-U socket in the PGW
    Ptr<Socket> pgwS5uSocket =
        Socket::CreateSocket(m_pgw, TypeId::LookupByName("ns3::UdpSocketFactory"));
    retval = pgwS5uSocket->Bind(InetSocketAddress(pgwS5Address, m_gtpuUdpPort));
    NS_ASSERT(retval == 0);

    // Create S5-C socket in the PGW
    Ptr<Socket> pgwS5cSocket =
        Socket::CreateSocket(m_pgw, TypeId::LookupByName("ns3::UdpSocketFactory"));
    retval = pgwS5cSocket->Bind(InetSocketAddress(pgwS5Address, m_gtpcUdpPort));
    NS_ASSERT(retval == 0);

    // Create NrEpcPgwApplication
    m_pgwApp =
        CreateObject<NrEpcPgwApplication>(m_tunDevice, pgwS5Address, pgwS5uSocket, pgwS5cSocket);
    m_pgw->AddApplication(m_pgwApp);

    // Connect NrEpcPgwApplication and virtual net device for tunneling
    m_tunDevice->SetSendCallback(MakeCallback(&NrEpcPgwApplication::RecvFromTunDevice, m_pgwApp));

    // Create S5-U socket in the SGW
    Ptr<Socket> sgwS5uSocket =
        Socket::CreateSocket(m_sgw, TypeId::LookupByName("ns3::UdpSocketFactory"));
    retval = sgwS5uSocket->Bind(InetSocketAddress(sgwS5Address, m_gtpuUdpPort));
    NS_ASSERT(retval == 0);

    // Create S5-C socket in the SGW
    Ptr<Socket> sgwS5cSocket =
        Socket::CreateSocket(m_sgw, TypeId::LookupByName("ns3::UdpSocketFactory"));
    retval = sgwS5cSocket->Bind(InetSocketAddress(sgwS5Address, m_gtpcUdpPort));
    NS_ASSERT(retval == 0);

    // Create S1-U socket in the SGW
    Ptr<Socket> sgwS1uSocket =
        Socket::CreateSocket(m_sgw, TypeId::LookupByName("ns3::UdpSocketFactory"));
    retval = sgwS1uSocket->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_gtpuUdpPort));
    NS_ASSERT(retval == 0);

    // Create NrEpcSgwApplication
    m_sgwApp =
        CreateObject<NrEpcSgwApplication>(sgwS1uSocket, sgwS5Address, sgwS5uSocket, sgwS5cSocket);
    m_sgw->AddApplication(m_sgwApp);
    m_sgwApp->AddPgw(pgwS5Address);
    m_pgwApp->AddSgw(sgwS5Address);
    m_pgwApp->SetUnstructuredSessionRequestCallback(
        MakeCallback(&NrNoBackhaulEpcHelper::CreateUnstructuredSessionDevice, this));

    // Create S11 link between MME and SGW
    PointToPointHelper s11P2ph;
    s11P2ph.SetDeviceAttribute("DataRate", DataRateValue(m_s11LinkDataRate));
    s11P2ph.SetDeviceAttribute("Mtu", UintegerValue(m_s11LinkMtu));
    s11P2ph.SetChannelAttribute("Delay", TimeValue(m_s11LinkDelay));
    NetDeviceContainer mmeSgwDevices = s11P2ph.Install(m_mme, m_sgw);
    NS_LOG_LOGIC("MME's IPv4 ifaces after installing p2p dev: "
                 << m_mme->GetObject<Ipv4>()->GetNInterfaces());
    NS_LOG_LOGIC("SGW's IPv4 ifaces after installing p2p dev: "
                 << m_sgw->GetObject<Ipv4>()->GetNInterfaces());
    Ptr<NetDevice> mmeDev = mmeSgwDevices.Get(0);
    Ptr<NetDevice> sgwS11Dev = mmeSgwDevices.Get(1);
    m_s11Ipv4AddressHelper.NewNetwork();
    Ipv4InterfaceContainer mmeSgwIpIfaces = m_s11Ipv4AddressHelper.Assign(mmeSgwDevices);
    NS_LOG_LOGIC("MME's IPv4 ifaces after assigning Ipv4 addr to S11 dev: "
                 << m_mme->GetObject<Ipv4>()->GetNInterfaces());
    NS_LOG_LOGIC("SGW's IPv4 ifaces after assigning Ipv4 addr to S11 dev: "
                 << m_sgw->GetObject<Ipv4>()->GetNInterfaces());

    Ipv4Address mmeS11Address = mmeSgwIpIfaces.GetAddress(0);
    Ipv4Address sgwS11Address = mmeSgwIpIfaces.GetAddress(1);

    // Create S11 socket in the MME
    Ptr<Socket> mmeS11Socket =
        Socket::CreateSocket(m_mme, TypeId::LookupByName("ns3::UdpSocketFactory"));
    retval = mmeS11Socket->Bind(InetSocketAddress(mmeS11Address, m_gtpcUdpPort));
    NS_ASSERT(retval == 0);

    // Create S11 socket in the SGW
    Ptr<Socket> sgwS11Socket =
        Socket::CreateSocket(m_sgw, TypeId::LookupByName("ns3::UdpSocketFactory"));
    retval = sgwS11Socket->Bind(InetSocketAddress(sgwS11Address, m_gtpcUdpPort));
    NS_ASSERT(retval == 0);

    // Create MME Application and connect with SGW via S11 interface
    m_mmeApp = CreateObject<NrEpcMmeApplication>();
    m_mme->AddApplication(m_mmeApp);
    m_mmeApp->SetUeAttachCallback(MakeCallback(&NrNoBackhaulEpcHelper::UeAttached, this));
    m_mmeApp->AddSgw(sgwS11Address, mmeS11Address, mmeS11Socket);
    m_sgwApp->AddMme(mmeS11Address, sgwS11Socket);
}

NrNoBackhaulEpcHelper::~NrNoBackhaulEpcHelper()
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrNoBackhaulEpcHelper::GetTypeId()
{
    NS_LOG_FUNCTION_NOARGS();
    static TypeId tid =
        TypeId("ns3::NrNoBackhaulEpcHelper")
            .SetParent<NrRanEpcHelper>()
            .SetGroupName("Nr")
            .AddConstructor<NrNoBackhaulEpcHelper>()
            .AddAttribute("UeIpv4Network",
                          "The IPv4 network of the UEs, from which the UE addresses are assigned",
                          Ipv4AddressValue("7.0.0.0"),
                          MakeIpv4AddressAccessor(&NrNoBackhaulEpcHelper::m_ueIpv4Network),
                          MakeIpv4AddressChecker())
            .AddAttribute("S8LinkDataRate",
                          "The data rate of the S8 links created by AddRoamingPartner() for "
                          "home-routed roaming",
                          DataRateValue(DataRate("10Gb/s")),
                          MakeDataRateAccessor(&NrNoBackhaulEpcHelper::m_s8LinkDataRate),
                          MakeDataRateChecker())
            .AddAttribute("S8LinkDelay",
                          "The delay of the S8 links created by AddRoamingPartner() for "
                          "home-routed roaming",
                          TimeValue(Seconds(0)),
                          MakeTimeAccessor(&NrNoBackhaulEpcHelper::m_s8LinkDelay),
                          MakeTimeChecker())
            .AddAttribute("S8LinkMtu",
                          "The MTU of the S8 links created by AddRoamingPartner() for "
                          "home-routed roaming",
                          UintegerValue(2000),
                          MakeUintegerAccessor(&NrNoBackhaulEpcHelper::m_s8LinkMtu),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("UeIpv4Mask",
                          "The mask of the IPv4 network of the UEs",
                          Ipv4MaskValue("255.0.0.0"),
                          MakeIpv4MaskAccessor(&NrNoBackhaulEpcHelper::m_ueIpv4Mask),
                          MakeIpv4MaskChecker())
            .AddAttribute("S5LinkDataRate",
                          "The data rate to be used for the next S5 link to be created",
                          DataRateValue(DataRate("10Gb/s")),
                          MakeDataRateAccessor(&NrNoBackhaulEpcHelper::m_s5LinkDataRate),
                          MakeDataRateChecker())
            .AddAttribute("S5LinkDelay",
                          "The delay to be used for the next S5 link to be created",
                          TimeValue(Seconds(0)),
                          MakeTimeAccessor(&NrNoBackhaulEpcHelper::m_s5LinkDelay),
                          MakeTimeChecker())
            .AddAttribute("S5LinkMtu",
                          "The MTU of the next S5 link to be created",
                          UintegerValue(2000),
                          MakeUintegerAccessor(&NrNoBackhaulEpcHelper::m_s5LinkMtu),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("S11LinkDataRate",
                          "The data rate to be used for the next S11 link to be created",
                          DataRateValue(DataRate("10Gb/s")),
                          MakeDataRateAccessor(&NrNoBackhaulEpcHelper::m_s11LinkDataRate),
                          MakeDataRateChecker())
            .AddAttribute("S11LinkDelay",
                          "The delay to be used for the next S11 link to be created",
                          TimeValue(Seconds(0)),
                          MakeTimeAccessor(&NrNoBackhaulEpcHelper::m_s11LinkDelay),
                          MakeTimeChecker())
            .AddAttribute("S11LinkMtu",
                          "The MTU of the next S11 link to be created.",
                          UintegerValue(2000),
                          MakeUintegerAccessor(&NrNoBackhaulEpcHelper::m_s11LinkMtu),
                          MakeUintegerChecker<uint16_t>());
    return tid;
}

void
NrNoBackhaulEpcHelper::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_tunDevice->SetSendCallback(
        MakeNullCallback<bool, Ptr<Packet>, const Address&, const Address&, uint16_t>());
    m_tunDevice = nullptr;
    m_sgwApp = nullptr;
    m_sgw->Dispose();
    m_pgwApp = nullptr;
    m_pgw->Dispose();
    m_mmeApp = nullptr;
    m_mme->Dispose();
}

void
NrNoBackhaulEpcHelper::AddRoamingPartner(Ptr<NrNoBackhaulEpcHelper> partner, bool homeRouted)
{
    NS_LOG_FUNCTION(this << partner << homeRouted);
    m_mmeApp->AddRoamingPartner(partner->m_mmeApp);
    partner->m_mmeApp->AddRoamingPartner(m_mmeApp);
    if (homeRouted)
    {
        AddS8Link(this, partner);
        AddS8Link(partner, this);
    }
}

void
NrNoBackhaulEpcHelper::AddS8Link(Ptr<NrNoBackhaulEpcHelper> visited,
                                 Ptr<NrNoBackhaulEpcHelper> home)
{
    NS_LOG_FUNCTION(this << visited << home);
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(m_s8LinkDataRate));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(m_s8LinkMtu));
    p2ph.SetChannelAttribute("Delay", TimeValue(m_s8LinkDelay));
    NetDeviceContainer devices = p2ph.Install(visited->m_sgw, home->m_pgw);
    // 15.<visited index>.<home index>.0/30
    Ipv4AddressHelper addresses(
        Ipv4Address(15u << 24 | visited->GetCoreIndex() << 16 | home->GetCoreIndex() << 8),
        "255.255.255.252");
    addresses.Assign(devices);

    // The SGW and the PGW use their S5 addresses on S8 too: the S8 link carries the traffic
    // between them
    Ipv4StaticRoutingHelper routingHelper;
    auto sgwIpv4 = visited->m_sgw->GetObject<Ipv4>();
    auto pgwIpv4 = home->m_pgw->GetObject<Ipv4>();
    routingHelper.GetStaticRouting(sgwIpv4)->AddHostRouteTo(
        home->m_pgwS5Address,
        sgwIpv4->GetInterfaceForDevice(devices.Get(0)));
    routingHelper.GetStaticRouting(pgwIpv4)->AddHostRouteTo(
        visited->m_sgwS5Address,
        pgwIpv4->GetInterfaceForDevice(devices.Get(1)));
    visited->m_homeRoutedPartners[PeekPointer(home->m_mmeApp)] = PeekPointer(home);
}

Ptr<NrEpcMmeApplication>
NrNoBackhaulEpcHelper::GetMmeApplication() const
{
    return m_mmeApp;
}

Ipv4Address
NrNoBackhaulEpcHelper::UeAttached(uint64_t imsi, NrEpcMmeApplication* home)
{
    NS_LOG_FUNCTION(this << imsi << home);
    Ptr<NrUeNetDevice> ueDevice;
    for (auto node = NodeList::Begin(); node != NodeList::End() && !ueDevice; ++node)
    {
        for (uint32_t i = 0; i < (*node)->GetNDevices() && !ueDevice; i++)
        {
            auto device = DynamicCast<NrUeNetDevice>((*node)->GetDevice(i));
            ueDevice = device && device->GetImsi() == imsi ? device : nullptr;
        }
    }
    // the IPv4 and IPv6 interfaces of the NR device of the UE, if it has addresses of the family
    Ptr<Ipv4> ueIpv4 = ueDevice ? ueDevice->GetNode()->GetObject<Ipv4>() : nullptr;
    const int32_t interface = ueIpv4 ? ueIpv4->GetInterfaceForDevice(ueDevice) : -1;
    Ptr<Ipv6> ueIpv6 = ueDevice ? ueDevice->GetNode()->GetObject<Ipv6>() : nullptr;
    const int32_t interface6 = ueIpv6 ? ueIpv6->GetInterfaceForDevice(ueDevice) : -1;

    // Home-routed roaming: the session is in the PGW of the home network, through the S8 link,
    // and the UE keeps its home addresses
    auto homeRouted = home ? m_homeRoutedPartners.find(home) : m_homeRoutedPartners.end();
    if (homeRouted != m_homeRoutedPartners.end())
    {
        NrNoBackhaulEpcHelper* homeHelper = homeRouted->second;
        if (interface >= 0)
        {
            SetUeAddress(ueIpv4,
                         interface,
                         Ipv4InterfaceAddress(homeHelper->m_pgwApp->GetUeAddress(imsi),
                                              homeHelper->m_ueIpv4Mask),
                         homeHelper->GetUeDefaultGatewayAddress());
        }
        const Ipv6Address address6 = homeHelper->m_pgwApp->GetUeAddress6(imsi);
        if (interface6 >= 0 && address6 != Ipv6Address::GetAny())
        {
            SetUeAddress6(ueIpv6,
                          interface6,
                          Ipv6InterfaceAddress(address6, Ipv6Prefix(64)),
                          homeHelper->GetUeDefaultGatewayAddress6());
        }
        return homeHelper->m_pgwS5Address;
    }

    // a subscriber, or a UE roaming with local breakout: the session is in the PGW of this
    // network, with the addresses of the UE in this network, assigned on its first attachment
    // if it has none
    if (interface < 0 && interface6 < 0)
    {
        // a UE with no IP stack, whose sessions are unstructured, roaming with local breakout:
        // its sessions are in the PGW of this network
        if (ueDevice && home && !m_pgwApp->HasUe(imsi))
        {
            m_pgwApp->AddUe(imsi);
        }
        // otherwise, not an NR UE device with an address (e.g., in the S1-U tests): its address
        // is set by its owner
        return Ipv4Address();
    }
    if (!m_pgwApp->HasUe(imsi))
    {
        m_pgwApp->AddUe(imsi);
    }
    if (interface >= 0)
    {
        Ipv4Address address = m_pgwApp->GetUeAddress(imsi);
        if (address == Ipv4Address())
        {
            address = m_uePgwAddressHelper.NewAddress();
            m_pgwApp->SetUeAddress(imsi, address);
        }
        SetUeAddress(ueIpv4,
                     interface,
                     Ipv4InterfaceAddress(address, m_ueIpv4Mask),
                     GetUeDefaultGatewayAddress());
    }
    if (interface6 >= 0)
    {
        Ipv6Address address6 = m_pgwApp->GetUeAddress6(imsi);
        if (address6 == Ipv6Address::GetAny())
        {
            // the prefix of this network, with the interface identifier of the device
            address6 = m_uePgwAddressHelper6.NewAddress(ueDevice->GetAddress());
            m_pgwApp->SetUeAddress6(imsi, address6);
        }
        SetUeAddress6(ueIpv6,
                      interface6,
                      Ipv6InterfaceAddress(address6, Ipv6Prefix(64)),
                      GetUeDefaultGatewayAddress6());
    }
    return Ipv4Address();
}

void
NrNoBackhaulEpcHelper::SetUeAddress(Ptr<Ipv4> ueIpv4,
                                    int32_t interface,
                                    Ipv4InterfaceAddress address,
                                    Ipv4Address gateway)
{
    // the UE address in the network of the session, which the UE gets with its PDU session
    if (ueIpv4->GetNAddresses(interface) == 1 &&
        ueIpv4->GetAddress(interface, 0).GetLocal() == address.GetLocal())
    {
        return;
    }
    NS_LOG_INFO("UE address " << address.GetLocal());
    while (ueIpv4->GetNAddresses(interface) > 0)
    {
        ueIpv4->RemoveAddress(interface, 0);
    }
    ueIpv4->AddAddress(interface, address);
    Ptr<Ipv4StaticRouting> routing = Ipv4StaticRoutingHelper().GetStaticRouting(ueIpv4);
    for (uint32_t route = routing->GetNRoutes(); route > 0; route--)
    {
        if (routing->GetRoute(route - 1).IsDefault())
        {
            routing->RemoveRoute(route - 1);
        }
    }
    routing->SetDefaultRoute(gateway, interface);
}

void
NrNoBackhaulEpcHelper::SetUeAddress6(Ptr<Ipv6> ueIpv6,
                                     int32_t interface,
                                     Ipv6InterfaceAddress address,
                                     Ipv6Address gateway)
{
    // the global addresses of the UE are replaced, its link-local address is kept
    bool present = false;
    std::vector<Ipv6Address> others;
    for (uint32_t i = 0; i < ueIpv6->GetNAddresses(interface); i++)
    {
        const auto current = ueIpv6->GetAddress(interface, i);
        if (current.GetScope() == Ipv6InterfaceAddress::GLOBAL)
        {
            if (current.GetAddress() == address.GetAddress())
            {
                present = true;
            }
            else
            {
                others.push_back(current.GetAddress());
            }
        }
    }
    if (present && others.empty())
    {
        return;
    }
    NS_LOG_INFO("UE address " << address.GetAddress());
    for (const auto& other : others)
    {
        ueIpv6->RemoveAddress(interface, other);
    }
    if (!present)
    {
        ueIpv6->AddAddress(interface, address);
    }
    Ptr<Ipv6StaticRouting> routing = Ipv6StaticRoutingHelper().GetStaticRouting(ueIpv6);
    for (uint32_t route = routing->GetNRoutes(); route > 0; route--)
    {
        if (routing->GetRoute(route - 1).IsDefault())
        {
            routing->RemoveRoute(route - 1);
        }
    }
    routing->SetDefaultRoute(gateway, interface);
}

void
NrNoBackhaulEpcHelper::AddUe(Ptr<NetDevice> ueDevice, uint64_t imsi)
{
    NS_LOG_FUNCTION(this << imsi << ueDevice);

    m_mmeApp->AddUe(imsi);
    m_pgwApp->AddUe(imsi);
}

uint8_t
NrNoBackhaulEpcHelper::ActivateQosFlow(Ptr<NetDevice> ueDevice,
                                       uint64_t imsi,
                                       Ptr<NrQosRule> rule,
                                       NrQosFlow flow)
{
    NS_LOG_FUNCTION(this << ueDevice << imsi);

    // we now retrieve the IPv4/IPv6 address of the UE and notify it to the SGW;
    // we couldn't do it before since address assignment is triggered by
    // the user simulation program, rather than done by the EPC
    Ptr<Node> ueNode = ueDevice->GetNode();
    Ptr<Ipv4> ueIpv4 = ueNode->GetObject<Ipv4>();
    Ptr<Ipv6> ueIpv6 = ueNode->GetObject<Ipv6>();
    NS_ASSERT_MSG(ueIpv4 || ueIpv6,
                  "UEs need to have IPv4/IPv6 installed before QoS flows can be activated");

    if (ueIpv4)
    {
        int32_t interface = ueIpv4->GetInterfaceForDevice(ueDevice);
        if (interface >= 0 && ueIpv4->GetNAddresses(interface) == 1)
        {
            Ipv4Address ueAddr = ueIpv4->GetAddress(interface, 0).GetLocal();
            NS_LOG_LOGIC(" UE IPv4 address: " << ueAddr);
            m_pgwApp->SetUeAddress(imsi, ueAddr);
        }
    }
    if (ueIpv6)
    {
        int32_t interface6 = ueIpv6->GetInterfaceForDevice(ueDevice);
        if (interface6 >= 0 && ueIpv6->GetNAddresses(interface6) == 2)
        {
            Ipv6Address ueAddr6 = ueIpv6->GetAddress(interface6, 1).GetAddress();
            NS_LOG_LOGIC(" UE IPv6 address: " << ueAddr6);
            m_pgwApp->SetUeAddress6(imsi, ueAddr6);
        }
    }
    uint8_t qosFlowId = m_mmeApp->AddFlow(imsi, rule, flow);
    rule->SetQfi(qosFlowId);
    DoActivateQosFlowForUe(ueDevice, rule, flow);

    return qosFlowId;
}

uint8_t
NrNoBackhaulEpcHelper::ActivateUnstructuredQosFlow(Ptr<NetDevice> ueDevice,
                                                   uint64_t imsi,
                                                   uint16_t protocolNumber,
                                                   NrQosFlow flow)
{
    NS_LOG_FUNCTION(this << ueDevice << imsi << protocolNumber);

    // The session carries unstructured payload, so the UE has no address to notify to the
    // core: the session itself identifies the UE, and it is reached through a device
    // of its own on the PGW rather than by routing on an address.
    Ptr<NrQosRule> rule = NrQosRule::Unstructured(protocolNumber);
    uint8_t qosFlowId = m_mmeApp->AddFlow(imsi, rule, flow);
    rule->SetQfi(qosFlowId);

    CreateUnstructuredSessionDevice(imsi, qosFlowId, protocolNumber);

    DoActivateQosFlowForUe(ueDevice, rule, flow);

    return qosFlowId;
}

void
NrNoBackhaulEpcHelper::CreateUnstructuredSessionDevice(uint64_t imsi,
                                                       uint8_t qfi,
                                                       uint16_t protocolNumber)
{
    NS_LOG_FUNCTION(this << imsi << +qfi << protocolNumber);
    auto& sessionDevice = m_unstructuredSessionDevices[{imsi, qfi}];
    if (!sessionDevice)
    {
        sessionDevice = CreateObject<VirtualNetDevice>();
        // allow jumbo packets
        sessionDevice->SetAttribute("Mtu", UintegerValue(30000));
        sessionDevice->SetAddress(Mac48Address::Allocate());
        m_pgw->AddDevice(sessionDevice);
    }
    m_pgwApp->AddUnstructuredSession(imsi, qfi, protocolNumber, sessionDevice);
}

Ptr<VirtualNetDevice>
NrNoBackhaulEpcHelper::GetUnstructuredSessionDevice(uint64_t imsi, uint8_t qfi) const
{
    NS_LOG_FUNCTION(this << imsi << qfi);
    auto it = m_unstructuredSessionDevices.find({imsi, qfi});
    if (it == m_unstructuredSessionDevices.end())
    {
        return nullptr;
    }
    return it->second;
}

void
NrNoBackhaulEpcHelper::DoActivateQosFlowForUe(const Ptr<NetDevice>& ueDevice,
                                              const Ptr<NrQosRule>& rule,
                                              const NrQosFlow& flow) const
{
    NS_LOG_FUNCTION(this);
    Ptr<NrUeNetDevice> ueNrDevice = DynamicCast<NrUeNetDevice>(ueDevice);
    if (!ueNrDevice)
    {
        // You may wonder why this is not an assert. Well, take a look in epc-test-s1u-downlink
        // and -uplink: we are using CSMA to simulate UEs.
        NS_LOG_WARN("Unable to find NrUeNetDevice while activating the QoS flow");
    }
    else
    {
        // Schedule with context so that logging statements have Node ID
        Simulator::ScheduleWithContext(ueNrDevice->GetNode()->GetId(),
                                       Time(),
                                       &NrEpcUeNas::ActivateQosFlow,
                                       ueNrDevice->GetNas(),
                                       flow,
                                       rule);
    }
}

Ptr<Node>
NrNoBackhaulEpcHelper::GetPgwNode() const
{
    return m_pgw;
}

Ipv4InterfaceContainer
NrNoBackhaulEpcHelper::AssignUeIpv4Address(NetDeviceContainer ueDevices)
{
    auto ipv4ifaces = m_uePgwAddressHelper.Assign(ueDevices);
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    for (size_t i = 0; i < ueDevices.GetN(); i++)
    {
        if (!DynamicCast<NrUeNetDevice>(ueDevices.Get(i)))
        {
            continue;
        }
        auto ueNode = ueDevices.Get(i)->GetNode();
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(GetUeDefaultGatewayAddress(), 1);
    }
    return ipv4ifaces;
}

Ipv6InterfaceContainer
NrNoBackhaulEpcHelper::AssignUeIpv6Address(NetDeviceContainer ueDevices)
{
    for (auto iter = ueDevices.Begin(); iter != ueDevices.End(); iter++)
    {
        Ptr<Icmpv6L4Protocol> icmpv6 = (*iter)->GetNode()->GetObject<Icmpv6L4Protocol>();
        icmpv6->SetAttribute("DAD", BooleanValue(false));
    }
    auto ipv6ifaces = m_uePgwAddressHelper6.Assign(ueDevices);
    Ipv6StaticRoutingHelper ipv6RoutingHelper;
    for (size_t i = 0; i < ueDevices.GetN(); i++)
    {
        if (!DynamicCast<NrUeNetDevice>(ueDevices.Get(i)))
        {
            continue;
        }
        auto ueNode = ueDevices.Get(i)->GetNode();
        Ptr<Ipv6StaticRouting> ueStaticRouting =
            ipv6RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv6>());
        ueStaticRouting->SetDefaultRoute(GetUeDefaultGatewayAddress6(), 1);
    }
    return ipv6ifaces;
}

Ipv4Address
NrNoBackhaulEpcHelper::GetUeDefaultGatewayAddress()
{
    // return the address of the tun device
    return m_pgw->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
}

Ipv6Address
NrNoBackhaulEpcHelper::GetUeDefaultGatewayAddress6()
{
    // return the address of the tun device
    return m_pgw->GetObject<Ipv6>()->GetAddress(1, 1).GetAddress();
}

Ptr<Node>
NrNoBackhaulEpcHelper::GetSgwNode() const
{
    return m_sgw;
}

void
NrNoBackhaulEpcHelper::ConnectGnbToCore(Ptr<NrEpcGnbApplication> gnbApp,
                                        uint16_t cellId,
                                        Ipv4Address gnbAddress,
                                        Ipv4Address sgwAddress)
{
    NS_LOG_FUNCTION(this << gnbApp << cellId << gnbAddress << sgwAddress);

    NS_LOG_INFO("Connect S1-AP interface");
    NS_LOG_DEBUG("Adding MME and SGW for cell ID " << cellId);
    m_mmeApp->AddGnb(cellId, gnbAddress, gnbApp->GetS1apSapGnb());
    m_sgwApp->AddGnb(cellId, gnbAddress, sgwAddress);
    gnbApp->SetS1apSapMme(m_mmeApp->GetS1apSapMme());
}

int64_t
NrNoBackhaulEpcHelper::AssignStreams(int64_t stream)
{
    int64_t currentStream = stream;
    NS_ABORT_MSG_UNLESS(m_pgw && m_sgw && m_mme, "Running AssignStreams on empty node pointers");
    InternetStackHelper internet;
    NodeContainer nc;
    nc.Add(m_pgw);
    nc.Add(m_sgw);
    nc.Add(m_mme);
    currentStream += internet.AssignStreams(nc, currentStream);
    return (currentStream - stream);
}

std::pair<Ptr<Node>, Ipv4Address>
NrNoBackhaulEpcHelper::SetupRemoteHost(std::optional<std::string> dataRate,
                                       std::optional<uint16_t> mtu,
                                       std::optional<Time> delay)
{
    // create the internet and install the IP stack on the UEs
    // get SGW/PGW and create a single RemoteHost
    Ptr<Node> pgw = GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // connect a remoteHost to pgw. Setup routing too
    PointToPointHelper p2ph;
    if (dataRate.has_value())
    {
        p2ph.SetDeviceAttribute("DataRate", StringValue(dataRate.value()));
    }
    if (mtu.has_value())
    {
        p2ph.SetDeviceAttribute("Mtu", UintegerValue(mtu.value()));
    }
    if (delay.has_value())
    {
        p2ph.SetChannelAttribute("Delay", TimeValue(delay.value()));
    }
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    // Setup IPv4 addresses and routing from remoteHost to the UEs through PGW
    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    // 1.0.0.0/8 for the first core network, 1.<index>.0.0/16 for the others
    if (GetCoreIndex() == 0)
    {
        ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    }
    else
    {
        ipv4h.SetBase(GetCoreSubnet(1), "255.255.0.0");
    }
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting4 =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting4->AddNetworkRouteTo(m_ueIpv4Network, m_ueIpv4Mask, 1);
    return std::make_pair(remoteHost, internetIpIfaces.GetAddress(1, 0));
}

std::pair<Ptr<Node>, Ipv6Address>
NrNoBackhaulEpcHelper::SetupRemoteHost6(std::optional<std::string> dataRate,
                                        std::optional<uint16_t> mtu,
                                        std::optional<Time> delay)
{
    // create the internet and install the IP stack on the UEs
    // get SGW/PGW and create a single RemoteHost
    Ptr<Node> pgw = GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // connect a remoteHost to pgw. Setup routing too
    PointToPointHelper p2ph;
    if (dataRate.has_value())
    {
        p2ph.SetDeviceAttribute("DataRate", StringValue(dataRate.value()));
    }
    if (mtu.has_value())
    {
        p2ph.SetDeviceAttribute("Mtu", UintegerValue(mtu.value()));
    }
    if (delay.has_value())
    {
        p2ph.SetChannelAttribute("Delay", TimeValue(delay.value()));
    }
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    // Setup IPv6 addresses and routing from remoteHost to the UEs through PGW
    Ipv6AddressHelper ipv6h;
    ipv6h.SetBase(CoreIpv6Prefix("6001:db80::", GetCoreIndex()), Ipv6Prefix(64));
    Ipv6InterfaceContainer internetIpIfaces = ipv6h.Assign(internetDevices);
    internetIpIfaces.SetForwarding(0, true);
    internetIpIfaces.SetDefaultRouteInAllNodes(0);

    Ipv6StaticRoutingHelper ipv6RoutingHelper;
    Ptr<Ipv6StaticRouting> remoteHostStaticRouting =
        ipv6RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv6>());
    remoteHostStaticRouting->AddNetworkRouteTo(CoreIpv6Prefix("7777:f00d::", GetCoreIndex()),
                                               Ipv6Prefix(64),
                                               internetIpIfaces.GetAddress(0, 1),
                                               1,
                                               0);
    return std::make_pair(remoteHost, internetIpIfaces.GetAddress(1, 1));
}

} // namespace ns3
