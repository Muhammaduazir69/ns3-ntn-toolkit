/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "sionna-udp-transport.h"

#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SionnaUdpTransport");

TypeId
SionnaUdpTransport::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SionnaUdpTransport")
            .SetParent<SionnaTransport>()
            .SetGroupName("NtnSionna")
            .AddConstructor<SionnaUdpTransport>()
            .AddAttribute("ServerHost",
                          "Host of the sionna-server.py UDP endpoint",
                          StringValue("127.0.0.1"),
                          MakeStringAccessor(&SionnaUdpTransport::m_host),
                          MakeStringChecker())
            .AddAttribute("ServerPort",
                          "UDP port the sionna-server.py listens on",
                          UintegerValue(8765),
                          MakeUintegerAccessor(&SionnaUdpTransport::m_port),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("TimeoutMs",
                          "Per-query UDP wait before failing",
                          UintegerValue(50),
                          MakeUintegerAccessor(&SionnaUdpTransport::m_timeoutMs),
                          MakeUintegerChecker<uint32_t>(1, 60'000));
    return tid;
}

SionnaUdpTransport::SionnaUdpTransport()
    : m_host("127.0.0.1"),
      m_port(8765),
      m_timeoutMs(50),
      m_sock(-1)
{
}

SionnaUdpTransport::~SionnaUdpTransport()
{
    if (m_sock >= 0)
    {
        close(m_sock);
        m_sock = -1;
    }
}

void
SionnaUdpTransport::SetServer(const std::string& host, uint16_t port)
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
SionnaUdpTransport::SetTimeoutMs(uint32_t timeoutMs)
{
    m_timeoutMs = timeoutMs;
    if (m_sock >= 0)
    {
        close(m_sock);
        m_sock = -1;
    }
}

bool
SionnaUdpTransport::EnsureSocket() const
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

SionnaTransport::Response
SionnaUdpTransport::Query(const Request& req) const
{
    Response rsp{std::numeric_limits<double>::infinity(), 0, 0.0, false};
    if (!EnsureSocket())
    {
        ++m_failures;
        return rsp;
    }

    struct sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(m_port);
    if (::inet_pton(AF_INET, m_host.c_str(), &dst.sin_addr) != 1)
    {
        NS_LOG_WARN("inet_pton failed for host=" << m_host);
        ++m_failures;
        return rsp;
    }

    char buf[512];
    int n = std::snprintf(buf, sizeof(buf),
                          "{\"tx\":[%.6f,%.6f,%.6f],\"rx\":[%.6f,%.6f,%.6f],"
                          "\"freq_hz\":%.6e,\"id\":%llu}",
                          req.tx_x, req.tx_y, req.tx_z, req.rx_x, req.rx_y,
                          req.rx_z, req.freq_hz,
                          static_cast<unsigned long long>(req.request_id));
    if (n <= 0 || static_cast<size_t>(n) >= sizeof(buf))
    {
        NS_LOG_WARN("request truncated");
        ++m_failures;
        return rsp;
    }

    auto t0 = std::chrono::steady_clock::now();
    ssize_t sent = ::sendto(m_sock, buf, n, 0,
                            reinterpret_cast<struct sockaddr*>(&dst), sizeof(dst));
    if (sent != n)
    {
        NS_LOG_WARN("sendto failed: " << std::strerror(errno));
        ++m_failures;
        return rsp;
    }
    ++m_queriesSent;

    char rspBuf[2048];
    ssize_t got = ::recv(m_sock, rspBuf, sizeof(rspBuf) - 1, 0);
    auto t1 = std::chrono::steady_clock::now();
    m_lastRttMs.store(std::chrono::duration<double, std::milli>(t1 - t0).count());

    if (got <= 0)
    {
        ++m_timeouts;
        return rsp;
    }
    rspBuf[got] = '\0';

    // Cheap targeted JSON scan — server response is tightly schema'd.
    const char* k = std::strstr(rspBuf, "\"path_loss_db\"");
    if (!k)
    {
        ++m_failures;
        return rsp;
    }
    const char* colon = std::strchr(k, ':');
    if (!colon)
    {
        ++m_failures;
        return rsp;
    }
    rsp.path_loss_db = std::atof(colon + 1);
    rsp.ok = std::isfinite(rsp.path_loss_db);

    // Optional n_paths / compute_ms fields. Best-effort parse; missing is OK.
    if (const char* p = std::strstr(rspBuf, "\"n_paths\""))
    {
        if (const char* c = std::strchr(p, ':'))
        {
            rsp.n_paths = static_cast<uint32_t>(std::atoi(c + 1));
        }
    }
    if (const char* p = std::strstr(rspBuf, "\"compute_ms\""))
    {
        if (const char* c = std::strchr(p, ':'))
        {
            rsp.compute_ms = std::atof(c + 1);
        }
    }
    return rsp;
}

} // namespace ns3
