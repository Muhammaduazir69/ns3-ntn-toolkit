/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-influx-sink.h"

#include <ns3/log.h>
#include <ns3/simulator.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnInfluxSink");

namespace ntnobs
{

namespace
{

// Escape rules per https://docs.influxdata.com/influxdb/v2/reference/syntax/line-protocol/
std::string
EscapeKey(const std::string& s)
{
    std::string out;
    out.reserve(s.size());
    for (char c : s)
    {
        if (c == ',' || c == ' ' || c == '=')
        {
            out.push_back('\\');
        }
        out.push_back(c);
    }
    return out;
}

std::string
EscapeMeasurement(const std::string& s)
{
    std::string out;
    out.reserve(s.size());
    for (char c : s)
    {
        if (c == ',' || c == ' ')
        {
            out.push_back('\\');
        }
        out.push_back(c);
    }
    return out;
}

std::string
EscapeStringValue(const std::string& s)
{
    std::string out;
    out.reserve(s.size() + 2);
    out.push_back('"');
    for (char c : s)
    {
        if (c == '"' || c == '\\')
        {
            out.push_back('\\');
        }
        out.push_back(c);
    }
    out.push_back('"');
    return out;
}

std::string
FormatFloat(double v)
{
    std::ostringstream ss;
    ss << std::setprecision(15) << v;
    return ss.str();
}

} // namespace

std::string
EncodeLineProtocol(const std::vector<Point>& pts, bool useSimulationTime)
{
    std::string out;
    out.reserve(pts.size() * 96);
    for (const auto& p : pts)
    {
        out += EscapeMeasurement(p.measurement);
        for (const auto& [k, v] : p.tags)
        {
            out.push_back(',');
            out += EscapeKey(k);
            out.push_back('=');
            out += EscapeKey(v);
        }
        out.push_back(' ');
        bool first = true;
        for (const auto& [k, v] : p.fieldsFloat)
        {
            if (!first)
                out.push_back(',');
            out += EscapeKey(k);
            out.push_back('=');
            out += FormatFloat(v);
            first = false;
        }
        for (const auto& [k, v] : p.fieldsInt)
        {
            if (!first)
                out.push_back(',');
            out += EscapeKey(k);
            out.push_back('=');
            out += std::to_string(v);
            out.push_back('i');
            first = false;
        }
        for (const auto& [k, v] : p.fieldsString)
        {
            if (!first)
                out.push_back(',');
            out += EscapeKey(k);
            out.push_back('=');
            out += EscapeStringValue(v);
            first = false;
        }
        long long ts_ns =
            useSimulationTime ? p.timestamp.GetNanoSeconds() : p.timestamp.GetNanoSeconds();
        out.push_back(' ');
        out += std::to_string(ts_ns);
        out.push_back('\n');
    }
    return out;
}

NS_OBJECT_ENSURE_REGISTERED(NtnInfluxSink);

TypeId
NtnInfluxSink::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntnobs::NtnInfluxSink")
                            .SetParent<Object>()
                            .SetGroupName("NtnObservability")
                            .AddConstructor<NtnInfluxSink>();
    return tid;
}

NtnInfluxSink::NtnInfluxSink() = default;
NtnInfluxSink::~NtnInfluxSink() = default;

void
NtnInfluxSink::DoDispose()
{
    Stop();
    if (m_udpSocketFd >= 0)
    {
        ::close(m_udpSocketFd);
        m_udpSocketFd = -1;
    }
    Object::DoDispose();
}

void
NtnInfluxSink::SetTransport(Transport t)
{
    m_transport = t;
}

void
NtnInfluxSink::SetUdpEndpoint(const std::string& host, uint16_t port)
{
    m_udpHost = host;
    m_udpPort = port;
}

void
NtnInfluxSink::SetFilePath(const std::string& path)
{
    m_filePath = path;
}

void
NtnInfluxSink::SetFlushPeriod(Time period)
{
    NS_ASSERT_MSG(period > Time(0), "flush period must be positive");
    m_flushPeriod = period;
}

void
NtnInfluxSink::SetUseSimulationTime(bool yes)
{
    m_useSimulationTime = yes;
}

void
NtnInfluxSink::Push(const Point& p)
{
    Point copy = p;
    if (copy.timestamp == Time(0) && m_useSimulationTime)
    {
        copy.timestamp = Simulator::Now();
    }
    m_buffer.push_back(std::move(copy));
    m_emitted++;
}

void
NtnInfluxSink::Flush()
{
    if (m_buffer.empty())
    {
        return;
    }
    const std::string payload = EncodeLineProtocol(m_buffer, m_useSimulationTime);
    if (m_transport == Transport::Udp)
    {
        SendUdp(payload);
    }
    else
    {
        AppendToFile(payload);
    }
    m_buffer.clear();
}

void
NtnInfluxSink::Start()
{
    if (m_running)
    {
        return;
    }
    m_running = true;
    m_event = Simulator::Schedule(m_flushPeriod, &NtnInfluxSink::TickFlush, this);
}

void
NtnInfluxSink::Stop()
{
    if (!m_running)
    {
        return;
    }
    Flush();
    m_running = false;
    if (m_event.IsPending())
    {
        Simulator::Cancel(m_event);
    }
}

uint64_t
NtnInfluxSink::GetEmittedCount() const
{
    return m_emitted;
}

void
NtnInfluxSink::TickFlush()
{
    if (!m_running)
    {
        return;
    }
    Flush();
    m_event = Simulator::Schedule(m_flushPeriod, &NtnInfluxSink::TickFlush, this);
}

void
NtnInfluxSink::SendUdp(const std::string& payload)
{
    if (m_udpSocketFd < 0)
    {
        m_udpSocketFd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, IPPROTO_UDP);
        if (m_udpSocketFd < 0)
        {
            NS_LOG_WARN("UDP socket() failed: " << std::strerror(errno));
            return;
        }
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(m_udpPort);
    if (::inet_pton(AF_INET, m_udpHost.c_str(), &addr.sin_addr) != 1)
    {
        NS_LOG_WARN("UDP target host invalid: " << m_udpHost);
        return;
    }
    // InfluxDB UDP listener defaults to 65 535-byte payload; fragment if larger.
    constexpr size_t kMaxDatagram = 60 * 1024;
    size_t off = 0;
    while (off < payload.size())
    {
        size_t end = off;
        while (end < payload.size() && (end - off) < kMaxDatagram)
        {
            const auto next = payload.find('\n', end);
            if (next == std::string::npos)
            {
                end = payload.size();
                break;
            }
            if ((next + 1 - off) > kMaxDatagram)
            {
                break;
            }
            end = next + 1;
        }
        if (end == off)
        {
            // Cannot make progress without truncating a single line.
            NS_LOG_WARN("dropping oversized line at offset " << off);
            const auto next = payload.find('\n', off);
            if (next == std::string::npos)
                break;
            off = next + 1;
            continue;
        }
        const ssize_t n = ::sendto(m_udpSocketFd,
                                   payload.data() + off,
                                   end - off,
                                   0,
                                   reinterpret_cast<sockaddr*>(&addr),
                                   sizeof(addr));
        if (n < 0)
        {
            NS_LOG_WARN("UDP sendto failed: " << std::strerror(errno));
            break;
        }
        off = end;
    }
}

void
NtnInfluxSink::AppendToFile(const std::string& payload)
{
    if (m_filePath.empty())
    {
        return;
    }
    std::ofstream f(m_filePath, std::ios::app);
    if (!f)
    {
        NS_LOG_WARN("could not open " << m_filePath << " for append");
        return;
    }
    f.write(payload.data(), static_cast<std::streamsize>(payload.size()));
}

} // namespace ntnobs
} // namespace ns3
