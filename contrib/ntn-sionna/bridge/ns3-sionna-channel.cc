/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W9)
 */
#include "ns3-sionna-channel.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnSionnaChannel");
NS_OBJECT_ENSURE_REGISTERED(NtnSionnaChannel);

TypeId
NtnSionnaChannel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnSionnaChannel")
            .SetParent<PropagationLossModel>()
            .SetGroupName("NtnSionna")
            .AddConstructor<NtnSionnaChannel>()
            .AddAttribute("ServerHost",
                          "Host of the sionna-server.py UDP endpoint",
                          StringValue("127.0.0.1"),
                          MakeStringAccessor(&NtnSionnaChannel::m_host),
                          MakeStringChecker())
            .AddAttribute("ServerPort",
                          "UDP port the sionna-server.py is listening on",
                          UintegerValue(8765),
                          MakeUintegerAccessor(&NtnSionnaChannel::m_port),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("FrequencyHz",
                          "Carrier frequency the server should ray-trace at",
                          DoubleValue(2.0e9),
                          MakeDoubleAccessor(&NtnSionnaChannel::m_freqHz),
                          MakeDoubleChecker<double>(1e6, 1e12))
            .AddAttribute("TimeoutMs",
                          "Per-query UDP wait before falling back to FSPL",
                          UintegerValue(50),
                          MakeUintegerAccessor(&NtnSionnaChannel::m_timeoutMs),
                          MakeUintegerChecker<uint32_t>(1, 60'000));
    return tid;
}

NtnSionnaChannel::NtnSionnaChannel()
    : m_host("127.0.0.1"),
      m_port(8765),
      m_freqHz(2.0e9),
      m_timeoutMs(50),
      m_sock(-1),
      m_seq(0),
      m_queriesSent(0),
      m_timeouts(0),
      m_fallbacks(0),
      m_lastRttMs(0.0)
{
}

NtnSionnaChannel::~NtnSionnaChannel()
{
    if (m_sock >= 0)
    {
        close(m_sock);
        m_sock = -1;
    }
}

void
NtnSionnaChannel::SetServer(const std::string& host, uint16_t port)
{
    m_host = host;
    m_port = port;
    if (m_sock >= 0)
    {
        close(m_sock);
        m_sock = -1;
    }
}

void
NtnSionnaChannel::SetFrequencyHz(double freqHz)
{
    m_freqHz = freqHz;
}

void
NtnSionnaChannel::SetTimeoutMs(uint32_t timeoutMs)
{
    m_timeoutMs = timeoutMs;
    if (m_sock >= 0)
    {
        close(m_sock);
        m_sock = -1;
    }
}

double
NtnSionnaChannel::FreeSpacePathLossDb(double distM, double freqHz)
{
    double d = std::max(distM, 1e-3);
    double fGhz = freqHz / 1e9;
    return 20.0 * std::log10(d) + 20.0 * std::log10(fGhz) + 32.45;
}

bool
NtnSionnaChannel::EnsureSocket() const
{
    if (m_sock >= 0)
    {
        return true;
    }
    int s = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0)
    {
        NS_LOG_WARN("socket() failed: " << std::strerror(errno));
        return false;
    }
    struct timeval tv;
    tv.tv_sec = m_timeoutMs / 1000;
    tv.tv_usec = (m_timeoutMs % 1000) * 1000;
    if (::setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
    {
        NS_LOG_WARN("SO_RCVTIMEO failed: " << std::strerror(errno));
        close(s);
        return false;
    }
    m_sock = s;
    return true;
}

double
NtnSionnaChannel::QuerySionnaDb(double txX, double txY, double txZ,
                                 double rxX, double rxY, double rxZ) const
{
    if (!EnsureSocket())
    {
        return std::numeric_limits<double>::infinity();
    }

    struct sockaddr_in dst {};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(m_port);
    if (::inet_pton(AF_INET, m_host.c_str(), &dst.sin_addr) != 1)
    {
        NS_LOG_WARN("inet_pton failed for host=" << m_host);
        return std::numeric_limits<double>::infinity();
    }

    uint64_t id = ++m_seq;
    char buf[512];
    int n = std::snprintf(buf, sizeof(buf),
                          "{\"tx\":[%.6f,%.6f,%.6f],\"rx\":[%.6f,%.6f,%.6f],"
                          "\"freq_hz\":%.6e,\"id\":%llu}",
                          txX, txY, txZ, rxX, rxY, rxZ, m_freqHz,
                          static_cast<unsigned long long>(id));
    if (n <= 0 || static_cast<size_t>(n) >= sizeof(buf))
    {
        NS_LOG_WARN("request truncated");
        return std::numeric_limits<double>::infinity();
    }

    auto t0 = std::chrono::steady_clock::now();
    ssize_t sent = ::sendto(m_sock, buf, n, 0,
                            reinterpret_cast<struct sockaddr*>(&dst), sizeof(dst));
    if (sent != n)
    {
        NS_LOG_WARN("sendto failed: " << std::strerror(errno));
        return std::numeric_limits<double>::infinity();
    }
    ++m_queriesSent;

    char rsp[1024];
    ssize_t got = ::recv(m_sock, rsp, sizeof(rsp) - 1, 0);
    auto t1 = std::chrono::steady_clock::now();
    m_lastRttMs.store(std::chrono::duration<double, std::milli>(t1 - t0).count());

    if (got <= 0)
    {
        ++m_timeouts;
        NS_LOG_INFO("UDP recv timed out (" << m_timeoutMs << " ms)");
        return std::numeric_limits<double>::infinity();
    }
    rsp[got] = '\0';

    // Cheap JSON scan — request/response is tightly schema'd by the server, so
    // a real JSON lib would be overkill. Look for the "path_loss_db" key.
    const char* k = std::strstr(rsp, "\"path_loss_db\"");
    if (!k)
    {
        NS_LOG_WARN("malformed response: " << rsp);
        return std::numeric_limits<double>::infinity();
    }
    const char* colon = std::strchr(k, ':');
    if (!colon)
    {
        return std::numeric_limits<double>::infinity();
    }
    return std::atof(colon + 1);
}

double
NtnSionnaChannel::DoCalcRxPower(double txPowerDbm,
                                Ptr<MobilityModel> a,
                                Ptr<MobilityModel> b) const
{
    Vector pa = a->GetPosition();
    Vector pb = b->GetPosition();
    double pl = QuerySionnaDb(pa.x, pa.y, pa.z, pb.x, pb.y, pb.z);
    if (!std::isfinite(pl))
    {
        ++m_fallbacks;
        double d = std::sqrt((pa.x - pb.x) * (pa.x - pb.x) +
                             (pa.y - pb.y) * (pa.y - pb.y) +
                             (pa.z - pb.z) * (pa.z - pb.z));
        pl = FreeSpacePathLossDb(d, m_freqHz);
    }
    return txPowerDbm - pl;
}

int64_t
NtnSionnaChannel::DoAssignStreams(int64_t /*stream*/)
{
    // Deterministic / no internal RNG — Sionna RT itself is the only randomness
    // source, and that's owned by the server process.
    return 0;
}

} // namespace ns3
