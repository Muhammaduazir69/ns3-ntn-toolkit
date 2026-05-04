/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W9)
 */
#include "ns3/constant-position-mobility-model.h"
#include "ns3/test.h"

#include "ns3/ns3-sionna-channel.h"

#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace ns3
{
namespace
{

/// One-shot UDP echo that responds with a hand-crafted Sionna-style JSON.
/// Used by the loopback test so we don't need the Python server in CI.
class MockSionnaServer
{
  public:
    MockSionnaServer(uint16_t port, double pathLossDb)
        : m_port(port),
          m_pathLossDb(pathLossDb),
          m_running(false),
          m_sock(-1),
          m_received(0)
    {
    }

    bool Start()
    {
        m_sock = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (m_sock < 0)
        {
            return false;
        }
        int reuse = 1;
        ::setsockopt(m_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        struct sockaddr_in addr {};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(m_port);
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (::bind(m_sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
        {
            ::close(m_sock);
            m_sock = -1;
            return false;
        }
        m_running = true;
        m_thread = std::thread([this] { Loop(); });
        return true;
    }

    void Stop()
    {
        m_running = false;
        if (m_sock >= 0)
        {
            ::shutdown(m_sock, SHUT_RDWR);
            ::close(m_sock);
            m_sock = -1;
        }
        if (m_thread.joinable())
        {
            m_thread.join();
        }
    }

    uint64_t GetReceived() const { return m_received.load(); }

  private:
    void Loop()
    {
        char buf[2048];
        while (m_running.load())
        {
            struct sockaddr_in peer {};
            socklen_t plen = sizeof(peer);
            ssize_t n = ::recvfrom(m_sock, buf, sizeof(buf), 0,
                                   reinterpret_cast<struct sockaddr*>(&peer), &plen);
            if (n <= 0)
            {
                break; // socket closed
            }
            ++m_received;
            char rsp[256];
            int rlen = std::snprintf(rsp, sizeof(rsp),
                                     "{\"id\":1,\"path_loss_db\":%.4f,"
                                     "\"n_paths\":1,\"compute_ms\":1.0}",
                                     m_pathLossDb);
            ::sendto(m_sock, rsp, rlen, 0,
                     reinterpret_cast<struct sockaddr*>(&peer), plen);
        }
    }

    uint16_t m_port;
    double m_pathLossDb;
    std::atomic<bool> m_running;
    int m_sock;
    std::atomic<uint64_t> m_received;
    std::thread m_thread;
};

/// FSPL closed form must be exact at known geometry.
/// 1 km @ 2 GHz → 98.47 dB; 1413 m @ 2 GHz → 101.47 dB (matches Sionna RT
/// in an empty scene to 0.00 dB; verified during W9 bring-up).
class FreeSpaceSpotCheckTest : public TestCase
{
  public:
    FreeSpaceSpotCheckTest()
        : TestCase("FSPL closed form is exact at known reference geometry")
    {
    }

  private:
    void DoRun() override
    {
        double pl1 = NtnSionnaChannel::FreeSpacePathLossDb(1000.0, 2.0e9);
        NS_TEST_ASSERT_MSG_EQ_TOL(pl1, 98.4706, 0.01,
                                  "FSPL @ 1 km, 2 GHz mismatch");

        double pl2 = NtnSionnaChannel::FreeSpacePathLossDb(1413.0, 2.0e9);
        NS_TEST_ASSERT_MSG_EQ_TOL(pl2, 101.47, 0.01,
                                  "FSPL @ 1413 m, 2 GHz mismatch (Sionna spot)");
    }
};

/// Channel falls back to FSPL when no server is available.
/// The fallback must equal the closed-form value to keep matched-scenario
/// CI runs aligned with the live Sionna reference.
class FallbackPathLossTest : public TestCase
{
  public:
    FallbackPathLossTest()
        : TestCase("Channel falls back to FSPL when no server responds")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<NtnSionnaChannel> ch = CreateObject<NtnSionnaChannel>();
        // Pick a port that should not have a server bound.
        ch->SetServer("127.0.0.1", 9);
        ch->SetTimeoutMs(20);
        ch->SetFrequencyHz(2.0e9);

        Ptr<ConstantPositionMobilityModel> a = CreateObject<ConstantPositionMobilityModel>();
        Ptr<ConstantPositionMobilityModel> b = CreateObject<ConstantPositionMobilityModel>();
        a->SetPosition(Vector(0, 0, 0));
        b->SetPosition(Vector(1000, 0, 0));

        double rx = ch->CalcRxPower(30.0 /* dBm */, a, b);
        double pl = 30.0 - rx;
        double expect = NtnSionnaChannel::FreeSpacePathLossDb(1000.0, 2.0e9);
        NS_TEST_ASSERT_MSG_EQ_TOL(pl, expect, 0.01,
                                  "Fallback path loss does not match FSPL");
        NS_TEST_ASSERT_MSG_GT(ch->GetTimeouts(), 0u,
                              "Expected at least one timeout against unused port");
        NS_TEST_ASSERT_MSG_GT(ch->GetFallbacks(), 0u,
                              "Expected at least one fallback");
    }
};

/// Loopback test against a mock UDP server. Verifies the JSON wire encoding
/// and that RTT meets the W9 <50 ms gate. The Sionna-vs-TR-38811 ±3 dB
/// gate is exercised by the example, not in unit tests, since it needs the
/// real ray tracer.
class LoopbackRttGateTest : public TestCase
{
  public:
    LoopbackRttGateTest()
        : TestCase("Mock-server loopback RTT under 50 ms gate")
    {
    }

  private:
    void DoRun() override
    {
        const uint16_t port = 38765;
        const double mockPlDb = 123.45;
        MockSionnaServer mock(port, mockPlDb);
        bool started = mock.Start();
        NS_TEST_ASSERT_MSG_EQ(started, true,
                              "Mock UDP server failed to bind");

        Ptr<NtnSionnaChannel> ch = CreateObject<NtnSionnaChannel>();
        ch->SetServer("127.0.0.1", port);
        ch->SetTimeoutMs(500);
        ch->SetFrequencyHz(2.0e9);

        Ptr<ConstantPositionMobilityModel> a = CreateObject<ConstantPositionMobilityModel>();
        Ptr<ConstantPositionMobilityModel> b = CreateObject<ConstantPositionMobilityModel>();
        a->SetPosition(Vector(0, 0, 1000.0));
        b->SetPosition(Vector(1000, 0, 1.5));

        // Warm up — first call may include socket setup / kernel routing.
        ch->CalcRxPower(30.0, a, b);

        const int N = 20;
        double maxRtt = 0.0;
        for (int i = 0; i < N; ++i)
        {
            double rx = ch->CalcRxPower(30.0, a, b);
            double pl = 30.0 - rx;
            NS_TEST_ASSERT_MSG_EQ_TOL(pl, mockPlDb, 0.01,
                                      "Mock path loss not echoed back exactly");
            if (ch->GetLastRttMs() > maxRtt)
            {
                maxRtt = ch->GetLastRttMs();
            }
        }
        mock.Stop();

        NS_TEST_ASSERT_MSG_LT(maxRtt, 50.0,
                              "Loopback RTT exceeded 50 ms gate "
                                  "(actual " << maxRtt << " ms)");
        NS_TEST_ASSERT_MSG_EQ(ch->GetTimeouts(), 0u,
                              "Loopback should never time out against mock");
        NS_TEST_ASSERT_MSG_EQ(ch->GetFallbacks(), 0u,
                              "Loopback should not need to fall back");
    }
};

class NtnSionnaTestSuite : public TestSuite
{
  public:
    NtnSionnaTestSuite()
        : TestSuite("ntn-sionna", Type::UNIT)
    {
        AddTestCase(new FreeSpaceSpotCheckTest, Duration::QUICK);
        AddTestCase(new FallbackPathLossTest, Duration::QUICK);
        AddTestCase(new LoopbackRttGateTest, Duration::QUICK);
    }
};

static NtnSionnaTestSuite g_ntnSionnaTestSuite;

} // namespace
} // namespace ns3
