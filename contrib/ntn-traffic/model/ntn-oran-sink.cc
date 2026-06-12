// SPDX-License-Identifier: GPL-2.0-only
#include "ntn-oran-sink.h"

#include "ns3/address-utils.h"
#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"
#include "ns3/udp-socket-factory.h"

#include <cmath>
#include <cstring>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnOranSink");
NS_OBJECT_ENSURE_REGISTERED(NtnOranSink);

namespace
{
void
WriteDouble(uint8_t* buf, double v)
{
    uint64_t bits;
    std::memcpy(&bits, &v, 8);
    for (int i = 0; i < 8; ++i)
    {
        buf[i] = (bits >> (56 - 8 * i)) & 0xFF;
    }
}

double
ReadDouble(const uint8_t* buf)
{
    uint64_t bits = 0;
    for (int i = 0; i < 8; ++i)
    {
        bits = (bits << 8) | buf[i];
    }
    double v;
    std::memcpy(&v, &bits, 8);
    return v;
}
} // namespace

void
NtnCncTelemetry::WriteTo(uint8_t* buf) const
{
    const double vals[9] = {posX, posY, posZ, velX, velY, velZ, rollDeg, pitchDeg, yawDeg};
    for (int i = 0; i < 9; ++i)
    {
        WriteDouble(buf + 8 * i, vals[i]);
    }
    const uint16_t batPermille =
        static_cast<uint16_t>(std::max(0.0, std::min(1.0, batteryFraction)) * 1000.0);
    buf[72] = (batPermille >> 8) & 0xFF;
    buf[73] = batPermille & 0xFF;
    for (int i = 0; i < 8; ++i)
    {
        buf[74 + i] = (uptimeNs >> (56 - 8 * i)) & 0xFF;
    }
}

NtnCncTelemetry
NtnCncTelemetry::ReadFrom(const uint8_t* buf)
{
    NtnCncTelemetry t;
    double* vals[9] = {&t.posX, &t.posY, &t.posZ, &t.velX, &t.velY,
                       &t.velZ, &t.rollDeg, &t.pitchDeg, &t.yawDeg};
    for (int i = 0; i < 9; ++i)
    {
        *vals[i] = ReadDouble(buf + 8 * i);
    }
    const uint16_t batPermille = (static_cast<uint16_t>(buf[72]) << 8) | buf[73];
    t.batteryFraction = batPermille / 1000.0;
    uint64_t up = 0;
    for (int i = 0; i < 8; ++i)
    {
        up = (up << 8) | buf[74 + i];
    }
    t.uptimeNs = up;
    return t;
}

TypeId
NtnOranSink::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnOranSink")
            .SetParent<Application>()
            .SetGroupName("NtnTraffic")
            .AddConstructor<NtnOranSink>()
            .AddAttribute("Local",
                          "Local listen address (InetSocketAddress).",
                          AddressValue(),
                          MakeAddressAccessor(&NtnOranSink::m_local),
                          MakeAddressChecker())
            .AddTraceSource("Rx",
                            "A packet has been received.",
                            MakeTraceSourceAccessor(&NtnOranSink::m_rxTrace),
                            "ns3::Packet::AddressTracedCallback");
    return tid;
}

NtnOranSink::NtnOranSink() = default;
NtnOranSink::~NtnOranSink() = default;

void
NtnOranSink::DoDispose()
{
    m_socket = nullptr;
    Application::DoDispose();
}

void
NtnOranSink::StartApplication()
{
    if (!m_socket)
    {
        m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        if (m_socket->Bind(m_local) == -1)
        {
            NS_FATAL_ERROR("NtnOranSink failed to bind " << m_local);
        }
    }
    m_socket->SetRecvCallback(MakeCallback(&NtnOranSink::HandleRead, this));
}

void
NtnOranSink::StopApplication()
{
    if (m_socket)
    {
        m_socket->Close();
        m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
    }
}

void
NtnOranSink::HandleRead(Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom(from)))
    {
        if (packet->GetSize() == 0)
        {
            break;
        }
        m_totalRxBytes += packet->GetSize();
        ++m_totalRxPackets;
        m_rxTrace(packet, from);

        if (packet->GetSize() < NtnOranPayloadHeader::SERIALIZED_SIZE)
        {
            continue; // not an ORAN-NTN payload (foreign traffic)
        }
        NtnOranPayloadHeader hdr;
        Ptr<Packet> copy = packet->Copy();
        copy->RemoveHeader(hdr);
        if (hdr.GetVersion() != NtnOranPayloadHeader::NTN_ORAN_PAYLOAD_VERSION)
        {
            ++m_versionErrors;
            // Warn once per flow key (rate-limited, not per packet) so a
            // version mismatch cannot silently drain a flow undiagnosed.
            FlowKey badKey{hdr.GetSrcId(), hdr.GetFiveQi(), hdr.GetSst(), hdr.GetSd()};
            if (m_versionWarnedFlows.insert(badKey).second)
            {
                NS_LOG_WARN("NtnOranSink: discarding packet with payload-header version "
                            << +hdr.GetVersion() << " (expected "
                            << +NtnOranPayloadHeader::NTN_ORAN_PAYLOAD_VERSION << ") from "
                            << from << " srcId=" << hdr.GetSrcId()
                            << "; flow excluded from KPIs (total version errors: "
                            << m_versionErrors << ")");
            }
            continue;
        }

        const Time now = Simulator::Now();
        FlowKey key{hdr.GetSrcId(), hdr.GetFiveQi(), hdr.GetSst(), hdr.GetSd()};
        FlowStats& fs = m_flows[key];
        if (fs.rxPackets == 0)
        {
            fs.fiveQi = hdr.GetFiveQi();
            fs.sst = hdr.GetSst();
            fs.sd = hdr.GetSd();
            fs.srcId = hdr.GetSrcId();
            fs.dstId = hdr.GetDstId();
            fs.payloadType = hdr.GetPayloadType();
            fs.firstRx = now;
        }
        ++fs.rxPackets;
        fs.rxBytes += packet->GetSize();
        fs.lastRx = now;
        if (hdr.GetSeq() > fs.highestSeq)
        {
            fs.highestSeq = hdr.GetSeq();
        }
        else if (fs.rxPackets > 1)
        {
            ++fs.reordered;
        }

        // One-way delay from the in-band TX timestamp (same simulated clock).
        const double delayMs =
            (now.GetNanoSeconds() - static_cast<int64_t>(hdr.GetTxTimestampNs())) / 1e6;
        fs.sumDelayMs += delayMs;
        fs.maxDelayMs = std::max(fs.maxDelayMs, delayMs);
        // RFC 3550 interarrival jitter: J += (|D| - J) / 16, with D the
        // change in transit time between consecutive packets.
        if (fs.rxPackets > 1)
        {
            const double d = std::abs(delayMs - fs.lastTransitMs);
            fs.jitterMs += (d - fs.jitterMs) / 16.0;
        }
        fs.lastTransitMs = delayMs;

        if (hdr.GetPayloadType() == NtnOranPayloadHeader::CNC_TELEMETRY &&
            copy->GetSize() >= NtnCncTelemetry::SERIALIZED_SIZE)
        {
            uint8_t buf[NtnCncTelemetry::SERIALIZED_SIZE];
            copy->CopyData(buf, NtnCncTelemetry::SERIALIZED_SIZE);
            m_telemetry[hdr.GetSrcId()] = NtnCncTelemetry::ReadFrom(buf);
        }
    }
}

double
NtnOranSink::GetMeanDelayMs() const
{
    double sum = 0;
    uint64_t n = 0;
    for (const auto& [key, fs] : m_flows)
    {
        sum += fs.sumDelayMs;
        n += fs.rxPackets;
    }
    return n ? sum / n : 0.0;
}

double
NtnOranSink::GetMeanJitterMs() const
{
    double sum = 0;
    uint64_t n = 0;
    for (const auto& [key, fs] : m_flows)
    {
        if (fs.rxPackets > 1)
        {
            sum += fs.jitterMs;
            ++n;
        }
    }
    return n ? sum / n : 0.0;
}

double
NtnOranSink::GetLossRatio() const
{
    uint64_t lost = 0;
    uint64_t expected = 0;
    for (const auto& [key, fs] : m_flows)
    {
        lost += fs.LostPackets();
        expected += static_cast<uint64_t>(fs.highestSeq) + 1;
    }
    return expected ? static_cast<double>(lost) / expected : 0.0;
}

bool
NtnOranSink::GetLatestTelemetry(uint16_t srcId, NtnCncTelemetry& out) const
{
    auto it = m_telemetry.find(srcId);
    if (it == m_telemetry.end())
    {
        return false;
    }
    out = it->second;
    return true;
}

} // namespace ns3
