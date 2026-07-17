/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-influx-sink.h"

#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
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
    // The sim-time vs wall-time choice is applied earlier, in NtnInfluxSink::Push(),
    // which adds the base epoch onto each Point's sim-time. By the time a Point
    // reaches here its timestamp is already final, so we always emit it as-is.
    // The parameter is kept for source/ABI compatibility of this public helper.
    (void)useSimulationTime;
    std::string out;
    out.reserve(pts.size() * 96);
    for (const auto& p : pts)
    {
        // Line protocol requires at least one field, and forbids non-finite
        // float values (an out-of-range float rejects the whole line). Build
        // the field section first so a Point with no emittable field is skipped
        // entirely rather than producing an invalid fieldless line.
        std::string fields;
        bool first = true;
        for (const auto& [k, v] : p.fieldsFloat)
        {
            if (!std::isfinite(v))
            {
                continue; // NaN / +-Inf is invalid line protocol — skip the field
            }
            if (!first)
                fields.push_back(',');
            fields += EscapeKey(k);
            fields.push_back('=');
            fields += FormatFloat(v);
            first = false;
        }
        for (const auto& [k, v] : p.fieldsInt)
        {
            if (!first)
                fields.push_back(',');
            fields += EscapeKey(k);
            fields.push_back('=');
            fields += std::to_string(v);
            fields.push_back('i');
            first = false;
        }
        for (const auto& [k, v] : p.fieldsString)
        {
            if (!first)
                fields.push_back(',');
            fields += EscapeKey(k);
            fields.push_back('=');
            fields += EscapeStringValue(v);
            first = false;
        }
        if (fields.empty())
        {
            continue; // no valid field -> no legal line
        }
        out += EscapeMeasurement(p.measurement);
        for (const auto& [k, v] : p.tags)
        {
            out.push_back(',');
            out += EscapeKey(k);
            out.push_back('=');
            out += EscapeKey(v);
        }
        out.push_back(' ');
        out += fields;
        long long ts_ns = p.timestamp.GetNanoSeconds();
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
    static TypeId tid =
        TypeId("ns3::ntnobs::NtnInfluxSink")
            .SetParent<Object>()
            .SetGroupName("NtnObservability")
            .AddConstructor<NtnInfluxSink>()
            .AddAttribute("MaxBufferPoints",
                          "Maximum number of Points buffered between flushes; when "
                          "exceeded the OLDEST points are dropped (and counted via "
                          "GetDroppedPoints) instead of exhausting RAM on long runs.",
                          UintegerValue(1000000),
                          MakeUintegerAccessor(&NtnInfluxSink::m_maxBufferPoints),
                          MakeUintegerChecker<uint32_t>(1));
    return tid;
}

NtnInfluxSink::NtnInfluxSink()
{
    // Default the base epoch to the wall-clock time at construction (nanoseconds
    // since the Unix epoch), matching ntn-digital-twin's convention. This makes
    // emitted points land inside a retention-bounded InfluxDB bucket and inside
    // Grafana's `now()-1h` window, instead of at epoch 1970 (pure sim time).
    const auto wall = std::chrono::system_clock::now().time_since_epoch();
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(wall).count();
    m_baseEpoch = NanoSeconds(static_cast<int64_t>(ns));
}

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
NtnInfluxSink::SetBaseEpoch(Time epoch)
{
    m_baseEpoch = epoch;
}

Time
NtnInfluxSink::GetBaseEpoch() const
{
    return m_baseEpoch;
}

void
NtnInfluxSink::Push(const Point& p)
{
    Point copy = p;
    // Interpret the incoming timestamp as SIM time (explicit `now`, or, when it
    // is unset and auto-stamping is on, Simulator::Now()). Add the base epoch so
    // the wire timestamp is an absolute Unix time (base + simTime).
    Time simT = copy.timestamp;
    if (simT == Time(0) && m_useSimulationTime)
    {
        simT = Simulator::Now();
    }
    copy.timestamp = m_baseEpoch + simT;

    // Drop non-finite float fields (invalid line protocol); if that leaves the
    // point with no field at all, drop the whole point and count it — a
    // fieldless line is also invalid and would be rejected by InfluxDB.
    for (auto it = copy.fieldsFloat.begin(); it != copy.fieldsFloat.end();)
    {
        if (!std::isfinite(it->second))
        {
            it = copy.fieldsFloat.erase(it);
        }
        else
        {
            ++it;
        }
    }
    if (copy.fieldsFloat.empty() && copy.fieldsInt.empty() && copy.fieldsString.empty())
    {
        m_droppedPoints++;
        if (!m_dropWarned)
        {
            m_dropWarned = true;
            NS_LOG_WARN("NtnInfluxSink dropping a Point with no valid (finite) field; "
                        "see GetDroppedPoints()");
        }
        return;
    }

    if (m_buffer.size() >= m_maxBufferPoints)
    {
        // Bounded buffer (audit issue 14): drop the oldest point so a long
        // run without flushes degrades gracefully instead of exhausting RAM.
        m_buffer.erase(m_buffer.begin());
        m_droppedPoints++;
        if (!m_dropWarned)
        {
            m_dropWarned = true;
            NS_LOG_WARN("NtnInfluxSink buffer reached MaxBufferPoints="
                        << m_maxBufferPoints
                        << "; dropping oldest points (check the flush period / "
                           "transport, see GetDroppedPoints())");
        }
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

uint64_t
NtnInfluxSink::GetDroppedPoints() const
{
    return m_droppedPoints;
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
