// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-net-device.h"

#include "ns3/channel.h"
#include "ns3/error-model.h"
#include "ns3/iana-ieee802-numbers.h"
#include "ns3/ipv4-header.h"
#include "ns3/ipv6-header.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/pointer.h"
#include "ns3/uinteger.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrNetDevice");

NS_OBJECT_ENSURE_REGISTERED(NrNetDevice);

TypeId
NrNetDevice::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrNetDevice")
            .SetParent<NetDevice>()
            .AddAttribute("Mtu",
                          "The MAC-level Maximum Transmission Unit",
                          UintegerValue(30000),
                          MakeUintegerAccessor(&NrNetDevice::SetMtu, &NrNetDevice::GetMtu),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("ReceiveErrorModel",
                          "An optional packet error model to simulate packet loss",
                          PointerValue(),
                          MakePointerAccessor(&NrNetDevice::m_receiveErrorModel),
                          MakePointerChecker<ErrorModel>())
            .AddTraceSource("Tx",
                            "A packet has been transmitted with the Address as the recipient",
                            MakeTraceSourceAccessor(&NrNetDevice::m_txTrace),
                            "ns3::Packet::AddressTracedCallback")
            .AddTraceSource("Rx",
                            "A packet has been received",
                            MakeTraceSourceAccessor(&NrNetDevice::m_rxTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("Drop",
                            "A packet has been dropped by the receive error model",
                            MakeTraceSourceAccessor(&NrNetDevice::m_dropTrace),
                            "ns3::Packet::TracedCallback");

    return tid;
}

NrNetDevice::NrNetDevice()
{
    NS_LOG_FUNCTION(this);
}

NrNetDevice::~NrNetDevice()
{
    NS_LOG_FUNCTION(this);
}

void
NrNetDevice::DoDispose()
{
    m_node = nullptr;
    NetDevice::DoDispose();
}

void
NrNetDevice::SetIfIndex(const uint32_t index)
{
    m_ifIndex = index;
}

uint32_t
NrNetDevice::GetIfIndex() const
{
    return m_ifIndex;
}

Ptr<Channel>
NrNetDevice::GetChannel() const
{
    return nullptr;
}

void
NrNetDevice::SetAddress(Address address)
{
    NS_LOG_FUNCTION(this << address);
    m_macAddress = Mac48Address::ConvertFrom(address);
}

Address
NrNetDevice::GetAddress() const
{
    NS_LOG_FUNCTION(this);
    return m_macAddress;
}

bool
NrNetDevice::SetMtu(const uint16_t mtu)
{
    m_mtu = mtu;
    return true;
}

uint16_t
NrNetDevice::GetMtu() const
{
    return m_mtu;
}

bool
NrNetDevice::IsLinkUp() const
{
    return m_linkUp;
}

void
NrNetDevice::AddLinkChangeCallback([[maybe_unused]] Callback<void> callback)
{
}

bool
NrNetDevice::IsBroadcast() const
{
    return false;
}

Address
NrNetDevice::GetBroadcast() const
{
    return Mac48Address::GetBroadcast();
}

bool
NrNetDevice::IsMulticast() const
{
    return false;
}

Address
NrNetDevice::GetMulticast([[maybe_unused]] Ipv4Address multicastGroup) const
{
    return Mac48Address("01:00:5e:00:00:00");
}

bool
NrNetDevice::IsBridge() const
{
    return false;
}

bool
NrNetDevice::IsPointToPoint() const
{
    return false;
}

bool
NrNetDevice::SendFrom([[maybe_unused]] Ptr<Packet> packet,
                      [[maybe_unused]] const Address& source,
                      [[maybe_unused]] const Address& dest,
                      [[maybe_unused]] uint16_t protocolNumber)
{
    NS_FATAL_ERROR("Send from not supported");
    return false;
}

Ptr<Node>
NrNetDevice::GetNode() const
{
    return m_node;
}

void
NrNetDevice::SetNode(Ptr<Node> node)
{
    m_node = node;
}

bool
NrNetDevice::NeedsArp() const
{
    return false;
}

Address
NrNetDevice::GetMulticast([[maybe_unused]] Ipv6Address addr) const
{
    return Address();
}

void
NrNetDevice::SetReceiveCallback(ReceiveCallback cb)
{
    NS_LOG_FUNCTION(this);
    m_rxCallback = cb;
}

void
NrNetDevice::SetPromiscReceiveCallback([[maybe_unused]] PromiscReceiveCallback cb)
{
}

bool
NrNetDevice::SupportsSendFrom() const
{
    return false;
}

void
NrNetDevice::Receive(Ptr<Packet> p)
{
    NS_LOG_FUNCTION(this << p);

    Ipv4Header ipv4Header;
    Ipv6Header ipv6Header;

    if (m_receiveErrorModel && m_receiveErrorModel->IsCorrupt(p))
    {
        NS_LOG_INFO("Dropping " << p->GetSize() << " bytes on " << m_macAddress);
        m_dropTrace(p);
        return;
    }
    // Deserialize reads the whole header before the packet can be rejected for
    // carrying something else, so a buffer shorter than the header is not one to
    // hand to it: what is too short to be a header is not one.
    if (p->GetSize() >= ipv4Header.GetSerializedSize() && p->PeekHeader(ipv4Header) != 0)
    {
        NS_LOG_INFO("Received " << p->GetSize() << " bytes on " << m_macAddress
                                << ". IPv4 packet from " << ipv4Header.GetSource() << " to "
                                << ipv4Header.GetDestination());
        ForwardUp(p, iana::ieee802numbers::IPV4);
    }
    else if (p->GetSize() >= ipv6Header.GetSerializedSize() && p->PeekHeader(ipv6Header) != 0)
    {
        NS_LOG_INFO("Received " << p->GetSize() << " bytes on " << m_macAddress
                                << ". IPv6 packet from " << ipv6Header.GetSource() << " to "
                                << ipv6Header.GetDestination());
        ForwardUp(p, iana::ieee802numbers::IPV6);
    }
    else
    {
        ForwardUnclassifiedUp(p);
    }
}

void
NrNetDevice::ForwardUp(Ptr<Packet> p, uint16_t protocolNumber)
{
    NS_LOG_FUNCTION(this << p << protocolNumber);
    m_rxTrace(p);
    m_rxCallback(this, p, protocolNumber, Address());
}

void
NrNetDevice::ForwardUnclassifiedUp(Ptr<Packet> p)
{
    NS_LOG_FUNCTION(this << p);
    // The packet handed up here is neither IPv4 nor IPv6. In a correct stack this
    // never happens, but a incorrectly reassembled RLC-UM SDU (e.g. a segment concatenated
    // across an undetected discontinuity during fast-channel/handover loss) can
    // surface a non-IP buffer. A real UE's IP layer simply discards a packet with an
    // unrecognized version field; aborting the whole simulation over one corrupt
    // data-plane SDU is too strict, so drop and trace it instead.
    NS_LOG_WARN("Dropping " << p->GetSize() << " bytes with unrecognized IP version on "
                            << m_macAddress << " (malformed or incorrectly reassembled SDU)");
    m_dropTrace(p);
}

bool
NrNetDevice::Send(Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber)
{
    NS_LOG_FUNCTION(this << packet << dest << protocolNumber);
    bool ret = DoSend(packet, dest, protocolNumber);
    m_txTrace(packet, dest);
    return ret;
}

} // namespace ns3
