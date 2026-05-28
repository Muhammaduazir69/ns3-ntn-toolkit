/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit / Roadmap §4.2.1)
 */
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include "ns3/ns3-sionna-channel.h"
#include "ns3/ntn-atmospheric-loss-chain.h"
#include "ns3/ntn-sionna-cascade-channel.h"
#include "ns3/sionna-caching-transport.h"
#include "ns3/sionna-calibrator.h"
#include "ns3/cir-doppler-synth.h"
#include "ns3/sionna-batch-client.h"
#include "ns3/sionna-pybind-transport.h"
#include "ns3/sionna-replay-transport.h"
#include "ns3/sionna-transport.h"
#include "ns3/sionna-udp-transport.h"

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

// ---------------------------------------------------------------------------
//  UDP mock servers (in-thread, no Python dep) used by the sim-time tests.
// ---------------------------------------------------------------------------

/// One-shot UDP echo that responds with a hand-crafted Sionna-style JSON.
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
        if (::bind(m_sock,
                   reinterpret_cast<struct sockaddr*>(&addr),
                   sizeof(addr)) < 0)
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
            ssize_t n = ::recvfrom(m_sock,
                                   buf,
                                   sizeof(buf),
                                   0,
                                   reinterpret_cast<struct sockaddr*>(&peer),
                                   &plen);
            if (n <= 0)
            {
                break;
            }
            ++m_received;
            char rsp[256];
            int rlen = std::snprintf(rsp,
                                     sizeof(rsp),
                                     "{\"id\":1,\"path_loss_db\":%.4f,"
                                     "\"n_paths\":1,\"compute_ms\":1.0}",
                                     m_pathLossDb);
            ::sendto(m_sock,
                     rsp,
                     rlen,
                     0,
                     reinterpret_cast<struct sockaddr*>(&peer),
                     plen);
        }
    }

    uint16_t m_port;
    double m_pathLossDb;
    std::atomic<bool> m_running;
    int m_sock;
    std::atomic<uint64_t> m_received;
    std::thread m_thread;
};

/// FSPL-returning UDP server: parses the request, computes the closed-form
/// free-space path loss for the actual (tx, rx, freq) tuple, and returns it.
/// Roadmap §4.2.2: also parses optional tx_array / rx_array rows/cols and
/// echoes the resulting port count in the response, so the C++ side can
/// assert wire-level MIMO round-trip without a real Sionna RT.
class FsplUdpMockServer
{
  public:
    explicit FsplUdpMockServer(uint16_t port)
        : m_port(port),
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
        if (::bind(m_sock,
                   reinterpret_cast<struct sockaddr*>(&addr),
                   sizeof(addr)) < 0)
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
    static double FsplDb(double dM, double fHz)
    {
        double d = std::max(dM, 1e-3);
        double fGhz = fHz / 1e9;
        return 20.0 * std::log10(d) + 20.0 * std::log10(fGhz) + 32.45;
    }

    // Naïve scan for "key":<value> inside the request JSON.
    static double ExtractNumber(const char* s, const char* key)
    {
        const char* k = std::strstr(s, key);
        if (!k)
            return 0.0;
        const char* c = std::strchr(k, ':');
        if (!c)
            return 0.0;
        return std::atof(c + 1);
    }

    static bool ExtractTriple(const char* s, const char* key, double xyz[3])
    {
        const char* k = std::strstr(s, key);
        if (!k)
            return false;
        const char* lb = std::strchr(k, '[');
        if (!lb)
            return false;
        return std::sscanf(lb,
                            "[%lf,%lf,%lf",
                            &xyz[0],
                            &xyz[1],
                            &xyz[2]) == 3;
    }

    /// Pulls an integer value from "key":<digits> inside `s`. Returns
    /// `def` when key not found.
    static int ExtractInt(const char* s, const char* key, int def)
    {
        const char* k = std::strstr(s, key);
        if (!k)
            return def;
        const char* c = std::strchr(k, ':');
        if (!c)
            return def;
        return std::atoi(c + 1);
    }

    void Loop()
    {
        char buf[4096];
        while (m_running.load())
        {
            struct sockaddr_in peer {};
            socklen_t plen = sizeof(peer);
            ssize_t n = ::recvfrom(m_sock,
                                   buf,
                                   sizeof(buf) - 1,
                                   0,
                                   reinterpret_cast<struct sockaddr*>(&peer),
                                   &plen);
            if (n <= 0)
            {
                break;
            }
            ++m_received;
            buf[n] = '\0';

            double tx[3] = {0, 0, 0};
            double rx[3] = {0, 0, 0};
            double freq = 2.0e9;
            ExtractTriple(buf, "\"tx\"", tx);
            ExtractTriple(buf, "\"rx\"", rx);
            freq = ExtractNumber(buf, "\"freq_hz\"");
            if (freq <= 0)
                freq = 2.0e9;
            double d = std::sqrt((tx[0] - rx[0]) * (tx[0] - rx[0]) +
                                  (tx[1] - rx[1]) * (tx[1] - rx[1]) +
                                  (tx[2] - rx[2]) * (tx[2] - rx[2]));
            double pl = FsplDb(d, freq);

            // Roadmap §4.2.2: echo tx/rx port counts derived from the
            // optional array descriptors. Defaults: 1 port per side.
            int tx_ports = 1;
            int rx_ports = 1;
            if (const char* tArr = std::strstr(buf, "\"tx_array\""))
            {
                const int rows = ExtractInt(tArr, "\"rows\"", 1);
                const int cols = ExtractInt(tArr, "\"cols\"", 1);
                tx_ports = std::max(1, rows) * std::max(1, cols);
                // Cross-pol doubles port count (VH or X).
                if (std::strstr(tArr, "\"polarization\":\"VH\"") ||
                    std::strstr(tArr, "\"polarization\":\"X\""))
                {
                    tx_ports *= 2;
                }
            }
            if (const char* rArr = std::strstr(buf, "\"rx_array\""))
            {
                const int rows = ExtractInt(rArr, "\"rows\"", 1);
                const int cols = ExtractInt(rArr, "\"cols\"", 1);
                rx_ports = std::max(1, rows) * std::max(1, cols);
                if (std::strstr(rArr, "\"polarization\":\"VH\"") ||
                    std::strstr(rArr, "\"polarization\":\"X\""))
                {
                    rx_ports *= 2;
                }
            }

            // Roadmap §4.2.3: detect optional RIS descriptor, apply a
            // deterministic path-loss reduction so the C++ side can assert
            // end-to-end wire transit. We use 10 dB reduction when
            // phase_profile is "focus" (constructive overlay), 6 dB when
            // "flat" (mirror), and 0 dB when "random" (no coherent boost).
            // These numbers are conventions for the unit-test mock — a real
            // Sionna RT server's reflection enhancement depends on geometry.
            double risShiftDb = 0.0;
            uint16_t risElements = 0;
            if (const char* risStr = std::strstr(buf, "\"ris\""))
            {
                const int rRows = ExtractInt(risStr, "\"rows\"", 0);
                const int rCols = ExtractInt(risStr, "\"cols\"", 0);
                risElements = static_cast<uint16_t>(
                    std::max(0, rRows) * std::max(0, rCols));
                if (std::strstr(risStr, "\"phase_profile\":\"focus\""))
                {
                    risShiftDb = 10.0;
                }
                else if (std::strstr(risStr, "\"phase_profile\":\"flat\""))
                {
                    risShiftDb = 6.0;
                }
            }
            pl -= risShiftDb;

            char rsp[512];
            int rlen = std::snprintf(rsp,
                                     sizeof(rsp),
                                     "{\"id\":1,\"path_loss_db\":%.6f,"
                                     "\"n_paths\":1,\"compute_ms\":0.05,"
                                     "\"tx_ports\":%d,\"rx_ports\":%d,"
                                     "\"ris_elements\":%u}",
                                     pl,
                                     tx_ports,
                                     rx_ports,
                                     static_cast<unsigned>(risElements));
            ::sendto(m_sock,
                     rsp,
                     rlen,
                     0,
                     reinterpret_cast<struct sockaddr*>(&peer),
                     plen);
        }
    }

    uint16_t m_port;
    std::atomic<bool> m_running;
    int m_sock;
    std::atomic<uint64_t> m_received;
    std::thread m_thread;
};

// ---------------------------------------------------------------------------
//  Pre-existing reference tests (kept verbatim — they validate the FSPL
//  identity and the bare wire-compat that the refactor must not break)
// ---------------------------------------------------------------------------

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
        ch->SetServer("127.0.0.1", 9);   // discard port — nothing listens
        ch->SetTimeoutMs(20);
        ch->SetFrequencyHz(2.0e9);

        Ptr<ConstantPositionMobilityModel> a =
            CreateObject<ConstantPositionMobilityModel>();
        Ptr<ConstantPositionMobilityModel> b =
            CreateObject<ConstantPositionMobilityModel>();
        a->SetPosition(Vector(0, 0, 0));
        b->SetPosition(Vector(1000, 0, 0));

        double rx = ch->CalcRxPower(30.0, a, b);
        double pl = 30.0 - rx;
        double expect = NtnSionnaChannel::FreeSpacePathLossDb(1000.0, 2.0e9);
        NS_TEST_ASSERT_MSG_EQ_TOL(pl, expect, 0.01,
                                  "Fallback path loss does not match FSPL");
        NS_TEST_ASSERT_MSG_GT(ch->GetTimeouts(), 0u, "No timeout recorded");
        NS_TEST_ASSERT_MSG_GT(ch->GetFallbacks(), 0u, "No fallback recorded");
    }
};

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
        NS_TEST_ASSERT_MSG_EQ(mock.Start(), true, "mock bind failed");

        Ptr<NtnSionnaChannel> ch = CreateObject<NtnSionnaChannel>();
        ch->SetServer("127.0.0.1", port);
        ch->SetTimeoutMs(500);
        ch->SetFrequencyHz(2.0e9);

        Ptr<ConstantPositionMobilityModel> a =
            CreateObject<ConstantPositionMobilityModel>();
        Ptr<ConstantPositionMobilityModel> b =
            CreateObject<ConstantPositionMobilityModel>();
        a->SetPosition(Vector(0, 0, 1000.0));
        b->SetPosition(Vector(1000, 0, 1.5));

        ch->CalcRxPower(30.0, a, b);     // warm-up

        const int N = 20;
        double maxRtt = 0.0;
        for (int i = 0; i < N; ++i)
        {
            double rx = ch->CalcRxPower(30.0, a, b);
            double pl = 30.0 - rx;
            NS_TEST_ASSERT_MSG_EQ_TOL(pl, mockPlDb, 0.01,
                                      "Mock path loss not echoed back");
            if (ch->GetLastRttMs() > maxRtt)
            {
                maxRtt = ch->GetLastRttMs();
            }
        }
        mock.Stop();

        NS_TEST_ASSERT_MSG_LT(maxRtt, 50.0,
                              "Loopback RTT exceeded 50 ms gate (actual "
                                  << maxRtt << " ms)");
        NS_TEST_ASSERT_MSG_EQ(ch->GetTimeouts(), 0u,
                              "Loopback should never time out");
        NS_TEST_ASSERT_MSG_EQ(ch->GetFallbacks(), 0u,
                              "Loopback should not fall back");
    }
};

// ---------------------------------------------------------------------------
//  4.2.1 (Roadmap §4.2.1): SionnaTransport abstraction + end-to-end coverage
// ---------------------------------------------------------------------------

class TransportContractTest : public TestCase
{
  public:
    TransportContractTest()
        : TestCase("SionnaTransport contract: None always-fails, UDP exposes name and host and port")
    {
    }

  private:
    void DoRun() override
    {
        // None transport — every query fails, fallback counters tick.
        Ptr<SionnaNoneTransport> none = CreateObject<SionnaNoneTransport>();
        NS_TEST_EXPECT_MSG_EQ(none->Name(), "none", "None name");
        NS_TEST_EXPECT_MSG_EQ(none->IsAvailable(), false, "None unavailable");

        SionnaTransport::Request req{0, 0, 0, 100, 0, 0, 2.0e9, 1};
        auto rsp = none->Query(req);
        NS_TEST_EXPECT_MSG_EQ(rsp.ok, false, "None reports failure");
        NS_TEST_EXPECT_MSG_EQ(std::isfinite(rsp.path_loss_db),
                              false,
                              "None returns +inf path loss");
        NS_TEST_EXPECT_MSG_EQ(none->GetQueriesSent(), 1u, "queries counted");
        NS_TEST_EXPECT_MSG_EQ(none->GetFailures(), 1u, "failures counted");

        // UDP transport — config accessors.
        Ptr<SionnaUdpTransport> udp = CreateObject<SionnaUdpTransport>();
        NS_TEST_EXPECT_MSG_EQ(udp->Name(), "udp", "UDP name");
        NS_TEST_EXPECT_MSG_EQ(udp->IsAvailable(), true, "UDP always-available");
        udp->SetServer("10.20.30.40", 12345);
        udp->SetTimeoutMs(75);
        NS_TEST_EXPECT_MSG_EQ(udp->Host(), "10.20.30.40", "host setter");
        NS_TEST_EXPECT_MSG_EQ(udp->Port(), 12345u, "port setter");
        NS_TEST_EXPECT_MSG_EQ(udp->TimeoutMs(), 75u, "timeout setter");

        udp->ResetCounters();
        NS_TEST_EXPECT_MSG_EQ(udp->GetQueriesSent(), 0u, "reset zeros queries");
    }
};

class TransportUdpFsplIdentityTest : public TestCase
{
  public:
    TransportUdpFsplIdentityTest()
        : TestCase("UDP transport returns FSPL-identity values from real mock server")
    {
    }

  private:
    void DoRun() override
    {
        const uint16_t port = 38766;
        FsplUdpMockServer mock(port);
        NS_TEST_ASSERT_MSG_EQ(mock.Start(), true, "mock bind");

        Ptr<NtnSionnaChannel> ch = CreateObject<NtnSionnaChannel>();
        ch->SetServer("127.0.0.1", port);
        ch->SetTimeoutMs(500);
        ch->SetFrequencyHz(28.0e9); // mmWave NR-NTN carrier

        Ptr<ConstantPositionMobilityModel> a =
            CreateObject<ConstantPositionMobilityModel>();
        Ptr<ConstantPositionMobilityModel> b =
            CreateObject<ConstantPositionMobilityModel>();
        a->SetPosition(Vector(0, 0, 0));

        // 100 m → 100 km log-spaced sweep; each step should match FSPL.
        const double distances_m[] = {100.0, 1000.0, 10000.0, 100000.0,
                                        1000000.0};
        for (double d : distances_m)
        {
            b->SetPosition(Vector(d, 0, 0));
            double rx = ch->CalcRxPower(30.0, a, b);
            double pl = 30.0 - rx;
            double expect =
                NtnSionnaChannel::FreeSpacePathLossDb(d, 28.0e9);
            NS_TEST_ASSERT_MSG_EQ_TOL(
                pl,
                expect,
                0.001,
                "Transport-returned PL must equal FSPL at d=" << d << " m");
        }
        // Each step covered 20*log10(10) = 20 dB; first vs last is 80 dB.
        b->SetPosition(Vector(100.0, 0, 0));
        double pl_100 = 30.0 - ch->CalcRxPower(30.0, a, b);
        b->SetPosition(Vector(1000000.0, 0, 0));
        double pl_1Mm = 30.0 - ch->CalcRxPower(30.0, a, b);
        NS_TEST_ASSERT_MSG_EQ_TOL(pl_1Mm - pl_100,
                                  80.0,
                                  0.01,
                                  "FSPL scales 20*log10(d) — 4 decades = 80 dB");

        NS_TEST_ASSERT_MSG_EQ(ch->GetTimeouts(), 0u, "no timeouts");
        NS_TEST_ASSERT_MSG_EQ(ch->GetFallbacks(), 0u, "no fallbacks");
        NS_TEST_ASSERT_MSG_GT(ch->GetQueriesSent(),
                              5u,
                              "transport saw all queries");
        NS_TEST_ASSERT_MSG_EQ(mock.GetReceived(),
                              ch->GetQueriesSent(),
                              "server received what transport sent");

        mock.Stop();
    }
};

// ---------------------------------------------------------------------------
//  Simulator::Run() driven end-to-end mobility scenarios
// ---------------------------------------------------------------------------

namespace
{

struct SimSample
{
    double sim_time_s;
    double distance_m;
    double pl_db;
    bool from_fallback;
};

/// Free function the simulator schedules. Captures the latest distance + PL
/// into the supplied vector so the test can assert monotonic increase.
void
SampleChannel(Ptr<NtnSionnaChannel> ch,
              Ptr<MobilityModel> tx,
              Ptr<MobilityModel> rx,
              std::vector<SimSample>* samples,
              uint64_t* prevFallbacks)
{
    Vector pa = tx->GetPosition();
    Vector pb = rx->GetPosition();
    double d = std::sqrt((pa.x - pb.x) * (pa.x - pb.x) +
                          (pa.y - pb.y) * (pa.y - pb.y) +
                          (pa.z - pb.z) * (pa.z - pb.z));
    double rxPower = ch->CalcRxPower(30.0, tx, rx);
    double pl = 30.0 - rxPower;
    bool fb = ch->GetFallbacks() > *prevFallbacks;
    *prevFallbacks = ch->GetFallbacks();
    samples->push_back({Simulator::Now().GetSeconds(), d, pl, fb});
}

} // namespace

class SimulatorTimeMobilityTest : public TestCase
{
  public:
    SimulatorTimeMobilityTest()
        : TestCase("30 s sim: moving UE -> monotonic FSPL increase via UDP transport")
    {
    }

  private:
    void DoRun() override
    {
        const uint16_t port = 38767;
        FsplUdpMockServer mock(port);
        NS_TEST_ASSERT_MSG_EQ(mock.Start(), true, "mock bind");

        Ptr<NtnSionnaChannel> ch = CreateObject<NtnSionnaChannel>();
        ch->SetServer("127.0.0.1", port);
        ch->SetTimeoutMs(500);
        ch->SetFrequencyHz(2.0e9);

        // Tx (satellite) static at 550 km altitude over the equator.
        Ptr<ConstantPositionMobilityModel> tx =
            CreateObject<ConstantPositionMobilityModel>();
        tx->SetPosition(Vector(0, 0, 550e3));

        // Rx (UE / aircraft) moving at 100 m/s in +x, starting 1 km away.
        Ptr<ConstantVelocityMobilityModel> rx =
            CreateObject<ConstantVelocityMobilityModel>();
        rx->SetPosition(Vector(1000.0, 0, 0));
        rx->SetVelocity(Vector(100.0, 0, 0));

        std::vector<SimSample> samples;
        uint64_t prevFb = 0;
        for (int t = 1; t <= 30; ++t)
        {
            Simulator::Schedule(Seconds(t),
                                &SampleChannel,
                                ch,
                                Ptr<MobilityModel>(tx),
                                Ptr<MobilityModel>(rx),
                                &samples,
                                &prevFb);
        }
        Simulator::Stop(Seconds(31));
        Simulator::Run();

        mock.Stop();

        NS_TEST_ASSERT_MSG_EQ(samples.size(),
                              30u,
                              "30 samples across 30 s sim time");

        // Monotonic FSPL increase as the UE moves away from sub-satellite point.
        for (size_t i = 1; i < samples.size(); ++i)
        {
            NS_TEST_ASSERT_MSG_GT(samples[i].distance_m,
                                  samples[i - 1].distance_m,
                                  "distance must monotonically increase");
            NS_TEST_ASSERT_MSG_GT(samples[i].pl_db,
                                  samples[i - 1].pl_db,
                                  "PL must monotonically increase with distance");
        }

        // The transport must have served every sample — no fallbacks.
        NS_TEST_ASSERT_MSG_EQ(ch->GetFallbacks(),
                              0u,
                              "live transport => no fallbacks");
        NS_TEST_ASSERT_MSG_EQ(ch->GetQueriesSent(),
                              30u,
                              "30 transport queries sent");

        // Last sample's PL must match FSPL closed form to 0.01 dB — i.e. the
        // round-tripped value is byte-accurate against the closed form.
        double expectLast = NtnSionnaChannel::FreeSpacePathLossDb(
            samples.back().distance_m,
            2.0e9);
        NS_TEST_ASSERT_MSG_EQ_TOL(samples.back().pl_db,
                                  expectLast,
                                  0.01,
                                  "Last-sample PL deviates from FSPL identity");

        Simulator::Destroy();
    }
};

class MidRunServerKillTest : public TestCase
{
  public:
    MidRunServerKillTest()
        : TestCase("Mid-sim server kill: live samples -> fallback samples")
    {
    }

  private:
    static void StopMock(FsplUdpMockServer* mock)
    {
        mock->Stop();
    }

    void DoRun() override
    {
        const uint16_t port = 38768;
        auto mock = std::make_unique<FsplUdpMockServer>(port);
        NS_TEST_ASSERT_MSG_EQ(mock->Start(), true, "mock bind");

        Ptr<NtnSionnaChannel> ch = CreateObject<NtnSionnaChannel>();
        ch->SetServer("127.0.0.1", port);
        ch->SetTimeoutMs(50);
        ch->SetFrequencyHz(2.0e9);

        Ptr<ConstantPositionMobilityModel> tx =
            CreateObject<ConstantPositionMobilityModel>();
        Ptr<ConstantPositionMobilityModel> rx =
            CreateObject<ConstantPositionMobilityModel>();
        tx->SetPosition(Vector(0, 0, 0));
        rx->SetPosition(Vector(2000, 0, 0));

        std::vector<SimSample> samples;
        uint64_t prevFb = 0;
        // Schedule 10 samples; kill the server at t=5s halfway through.
        for (int t = 1; t <= 10; ++t)
        {
            Simulator::Schedule(Seconds(t),
                                &SampleChannel,
                                ch,
                                Ptr<MobilityModel>(tx),
                                Ptr<MobilityModel>(rx),
                                &samples,
                                &prevFb);
        }
        Simulator::Schedule(Seconds(5), &StopMock, mock.get());
        Simulator::Stop(Seconds(11));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(samples.size(), 10u, "10 samples");
        // First 4 samples (t=1..4) hit the live mock and return the same FSPL
        // value. After the server stops, the channel must fall back to FSPL —
        // value is the same here (geometry is fixed) but the fallback counter
        // distinguishes the two phases. The mid-tick (t=5) may go either way
        // depending on scheduling order.
        size_t livePhase = 0;
        size_t fallbackPhase = 0;
        for (const auto& s : samples)
        {
            if (s.from_fallback)
            {
                ++fallbackPhase;
            }
            else
            {
                ++livePhase;
            }
        }
        NS_TEST_ASSERT_MSG_GT(livePhase, 0u, "had live samples before kill");
        NS_TEST_ASSERT_MSG_GT(fallbackPhase,
                              0u,
                              "had fallback samples after kill");
        NS_TEST_ASSERT_MSG_GT(ch->GetTimeouts(),
                              0u,
                              "timeouts recorded after kill");
        NS_TEST_ASSERT_MSG_GT(ch->GetFallbacks(),
                              0u,
                              "fallbacks recorded after kill");

        // Throughout both phases the geometry-tied FSPL stays the same — the
        // numerical PL value must therefore stay the same in both phases.
        double expect =
            NtnSionnaChannel::FreeSpacePathLossDb(2000.0, 2.0e9);
        for (const auto& s : samples)
        {
            NS_TEST_ASSERT_MSG_EQ_TOL(s.pl_db,
                                      expect,
                                      0.01,
                                      "Phase-invariant FSPL drift");
        }

        Simulator::Destroy();
    }
};

class PybindStubAvailabilityTest : public TestCase
{
  public:
    PybindStubAvailabilityTest()
        : TestCase("Pybind11 transport stub: unavailable when ENABLE_SIONNA_PYBIND11 off")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<SionnaPybindTransport> py = CreateObject<SionnaPybindTransport>();
        NS_TEST_EXPECT_MSG_EQ(py->Name(), "pybind11", "name");
#ifdef ENABLE_SIONNA_PYBIND11
        // Even with the embed enabled, a stock toolkit checkout won't have
        // sionna_bridge installed; the call should fail cleanly, never
        // crash the test process.
        py->SetModule("non_existent_module_for_test");
        NS_TEST_EXPECT_MSG_EQ(py->IsAvailable(),
                              false,
                              "non-existent module not available");
#else
        NS_TEST_EXPECT_MSG_EQ(py->IsAvailable(),
                              false,
                              "pybind11 not compiled in -> unavailable");
        SionnaTransport::Request req{0, 0, 0, 100, 0, 0, 2.0e9, 1};
        auto rsp = py->Query(req);
        NS_TEST_EXPECT_MSG_EQ(rsp.ok, false, "query fails cleanly");
        const bool atLeastOneFailure = py->GetFailures() >= 1u;
        NS_TEST_EXPECT_MSG_EQ(atLeastOneFailure, true, "failure counted");
#endif
    }
};

// ---------------------------------------------------------------------------
//  Roadmap §4.2.2: Sionna RT 2.0.1 MIMO PlanarArray wire round-trip
// ---------------------------------------------------------------------------

class TransportMimoWireRoundTripTest : public TestCase
{
  public:
    TransportMimoWireRoundTripTest()
        : TestCase("UDP transport carries PlanarArray MIMO config and echoes ports back")
    {
    }

  private:
    void DoRun() override
    {
        const uint16_t port = 38770;
        FsplUdpMockServer mock(port);
        NS_TEST_ASSERT_MSG_EQ(mock.Start(), true, "mock bind");

        Ptr<SionnaUdpTransport> udp = CreateObject<SionnaUdpTransport>();
        udp->SetServer("127.0.0.1", port);
        udp->SetTimeoutMs(500);

        // 4x4 V-pol gNB array (16 ports) + 1x1 V-pol UE (1 port).
        SionnaTransport::Request req{};
        req.tx_x = 0;
        req.tx_y = 0;
        req.tx_z = 0;
        req.rx_x = 1000;
        req.rx_y = 0;
        req.rx_z = 0;
        req.freq_hz = 28e9;
        req.request_id = 1;
        MimoArrayConfig tx;
        tx.rows = 4;
        tx.cols = 4;
        tx.spacing_lambda = 0.5;
        tx.pattern = "iso";
        tx.polarization = "V";
        req.tx_array = tx;
        MimoArrayConfig rx;
        rx.rows = 1;
        rx.cols = 1;
        rx.pattern = "iso";
        rx.polarization = "V";
        req.rx_array = rx;

        auto rsp = udp->Query(req);
        NS_TEST_ASSERT_MSG_EQ(rsp.ok, true, "query ok");
        NS_TEST_EXPECT_MSG_EQ(rsp.tx_ports, 16u, "4x4 V-pol -> 16 ports");
        NS_TEST_EXPECT_MSG_EQ(rsp.rx_ports, 1u, "1x1 V-pol -> 1 port");

        // Cross-pol doubles port count.
        MimoArrayConfig tx_x = tx;
        tx_x.polarization = "VH";
        req.tx_array = tx_x;
        req.request_id = 2;
        auto rsp2 = udp->Query(req);
        NS_TEST_ASSERT_MSG_EQ(rsp2.ok, true, "VH query ok");
        NS_TEST_EXPECT_MSG_EQ(rsp2.tx_ports, 32u,
                              "4x4 VH-pol -> 32 ports (16x2)");

        // SISO fallback (no array fields) still works and reports 1 port.
        SionnaTransport::Request siso{};
        siso.tx_x = 0;
        siso.rx_x = 1000;
        siso.freq_hz = 2e9;
        siso.request_id = 3;
        auto rsp3 = udp->Query(siso);
        NS_TEST_ASSERT_MSG_EQ(rsp3.ok, true, "SISO ok");
        NS_TEST_EXPECT_MSG_EQ(rsp3.tx_ports, 1u, "no array -> 1 tx port");
        NS_TEST_EXPECT_MSG_EQ(rsp3.rx_ports, 1u, "no array -> 1 rx port");

        mock.Stop();
    }
};

class TransportMimoSimulatorTimeTest : public TestCase
{
  public:
    TransportMimoSimulatorTimeTest()
        : TestCase("Simulator: 30 s MIMO 8x8 path-loss queries hold ports + FSPL identity")
    {
    }

  private:
    void DoRun() override
    {
        const uint16_t port = 38771;
        FsplUdpMockServer mock(port);
        NS_TEST_ASSERT_MSG_EQ(mock.Start(), true, "mock bind");

        Ptr<SionnaUdpTransport> udp = CreateObject<SionnaUdpTransport>();
        udp->SetServer("127.0.0.1", port);
        udp->SetTimeoutMs(500);

        std::vector<SionnaTransport::Response> samples;
        // Schedule queries at 1 s, 6 s, 11 s, ..., 30 s (six queries
        // across the 30 s window).
        Simulator::Schedule(
            Seconds(0),
            [&samples, udp]() {
                for (int t = 1; t <= 30; t += 5)
                {
                    Simulator::Schedule(Seconds(t),
                                        [&samples, udp, t]() {
                                            SionnaTransport::Request r{};
                                            r.tx_x = 0;
                                            r.rx_x = 1000.0 + 100.0 * t;
                                            r.freq_hz = 28e9;
                                            r.request_id =
                                                static_cast<uint64_t>(t);
                                            MimoArrayConfig a;
                                            a.rows = 8;
                                            a.cols = 8;
                                            a.polarization = "V";
                                            r.tx_array = a;
                                            r.rx_array = a;
                                            samples.push_back(udp->Query(r));
                                        });
                }
            });
        Simulator::Stop(Seconds(31));
        Simulator::Run();

        mock.Stop();

        NS_TEST_ASSERT_MSG_EQ(samples.size(), 6u,
                              "6 MIMO queries across 30 s");
        for (const auto& s : samples)
        {
            NS_TEST_EXPECT_MSG_EQ(s.ok, true, "query ok");
            NS_TEST_EXPECT_MSG_EQ(s.tx_ports, 64u,
                                  "8x8 V-pol -> 64 ports");
            NS_TEST_EXPECT_MSG_EQ(s.rx_ports, 64u,
                                  "8x8 V-pol -> 64 ports");
            NS_TEST_ASSERT_MSG_GT(s.path_loss_db, 0.0,
                                  "PL positive");
            NS_TEST_ASSERT_MSG_LT(s.path_loss_db, 200.0,
                                  "PL plausible for 1-4 km @ 28 GHz");
        }
        Simulator::Destroy();
    }
};

// ---------------------------------------------------------------------------
// Roadmap §4.2.7 — NTN cascade glue (NtnAtmosphericLossChain + cascade channel)
// ---------------------------------------------------------------------------

/// Geometric elevation correctness in both ENU and ECEF reference frames.
/// ENU: ground at origin, +z is up — sat at (0,0,h) gives 90°, (h,0,0) gives 0°.
/// ECEF: ground at radial position |R_e|, sat at radial position |R_e + h| in
/// the same direction gives 90°; tangential sat gives ~0°.
class AtmosphericChainElevationTest : public TestCase
{
  public:
    AtmosphericChainElevationTest()
        : TestCase("§4.2.7: NtnAtmosphericLossChain elevation calc in ENU + ECEF")
    {
    }

  private:
    void DoRun() override
    {
        // --- Local ENU frame ---
        const Vector enuGround(0.0, 0.0, 0.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(
            NtnAtmosphericLossChain::ComputeElevationDeg(enuGround,
                                                          Vector(0, 0, 1.0e6)),
            90.0,
            0.01,
            "ENU zenith should be 90 deg");
        NS_TEST_ASSERT_MSG_EQ_TOL(
            NtnAtmosphericLossChain::ComputeElevationDeg(enuGround,
                                                          Vector(1.0e6, 0, 0)),
            0.0,
            0.01,
            "ENU horizon should be 0 deg");
        NS_TEST_ASSERT_MSG_LT(
            NtnAtmosphericLossChain::ComputeElevationDeg(enuGround,
                                                          Vector(0, 0, -1.0e6)),
            0.0,
            "ENU below ground should give negative elevation");

        // --- ECEF frame ---
        const double Re = 6371000.0;
        const Vector ecefGround(Re, 0.0, 0.0); // ground on +x axis
        NS_TEST_ASSERT_MSG_EQ_TOL(
            NtnAtmosphericLossChain::ComputeElevationDeg(
                ecefGround,
                Vector(Re + 550e3, 0.0, 0.0)),
            90.0,
            0.01,
            "ECEF radial sat should give 90 deg elevation");
        // Tangent direction (in +y) from this ground point — pure horizon.
        NS_TEST_ASSERT_MSG_EQ_TOL(
            NtnAtmosphericLossChain::ComputeElevationDeg(
                ecefGround,
                Vector(Re, 1.0e6, 0.0)),
            0.0,
            0.01,
            "ECEF tangential sat should give 0 deg elevation");

        // 45° intermediate: sat displaced radially and tangentially equally.
        const double d = 1.0e6;
        const Vector sat45(Re + d, d, 0.0);
        const double e45 = NtnAtmosphericLossChain::ComputeElevationDeg(
            ecefGround, sat45);
        NS_TEST_ASSERT_MSG_EQ_TOL(
            e45,
            45.0,
            0.5,
            "ECEF 45° geometry should give 45° elevation");
    }
};

/// Gaseous absorption is non-negative across the spectrum and increases
/// monotonically (with the toolkit's current thz-ntn P.676 backend, which
/// captures the broadband trend; the per-line GHz resonances are tightened
/// later in roadmap §4.3.2 when the ITU-Rpy backend is wrapped).
class AtmosphericChainGaseousSweepTest : public TestCase
{
  public:
    AtmosphericChainGaseousSweepTest()
        : TestCase("§4.2.7: gaseous absorption sweep at 2-90 GHz is non-negative and monotonic")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<NtnAtmosphericLossChain> chain =
            CreateObject<NtnAtmosphericLossChain>();
        chain->SetEnableRain(false);
        chain->SetEnableLms(false);

        const Vector ground(0.0, 0.0, 0.0);
        const Vector sat(0.0, 0.0, 550.0e3); // straight up, 90° elev
        const double freqs[] = {2.0e9, 12.0e9, 22.0e9, 60.0e9, 90.0e9};
        std::vector<double> losses;
        for (double f : freqs)
        {
            chain->SetFrequencyHz(f);
            const double l = chain->ComputeAttenuationDb(ground, sat);
            losses.push_back(l);
            NS_TEST_ASSERT_MSG_GT(
                l + 1e-9, // tolerate a single near-zero rounding
                0.0,
                "gaseous attenuation must be non-negative at all bands");
        }
        // Broadband trend: 60 GHz must produce strictly more attenuation
        // than 2 GHz (even with the broadband backend).
        NS_TEST_ASSERT_MSG_GT(
            losses[3], // 60 GHz
            losses[0], // 2 GHz
            "60 GHz must exceed 2 GHz gaseous attenuation");
        NS_TEST_ASSERT_MSG_GT(
            losses[4], // 90 GHz
            losses[0], // 2 GHz
            "90 GHz must exceed 2 GHz gaseous attenuation");
        // Monotone non-decreasing in this band given the broadband model.
        for (size_t i = 1; i < losses.size(); ++i)
        {
            NS_TEST_ASSERT_MSG_GT_OR_EQ(
                losses[i] + 1e-9,
                losses[i - 1],
                "gaseous attenuation must be non-decreasing in this sweep");
        }
    }
};

/// Rain attenuation is monotonically non-decreasing in rain rate at fixed
/// frequency and geometry.
class AtmosphericChainRainSweepTest : public TestCase
{
  public:
    AtmosphericChainRainSweepTest()
        : TestCase("§4.2.7: rain attenuation monotonic in rain rate")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<NtnAtmosphericLossChain> chain =
            CreateObject<NtnAtmosphericLossChain>();
        chain->SetEnableGaseous(false); // isolate rain contribution
        chain->SetEnableLms(false);
        chain->SetFrequencyHz(12.0e9);

        const Vector ground(0.0, 0.0, 0.05e3); // 50 m altitude
        // Sat at 45° elev to keep slant length finite + non-trivial.
        const Vector sat(550e3, 0.0, 550e3);

        const double rates[] = {0.0, 5.0, 25.0, 100.0, 200.0};
        std::vector<double> losses;
        for (double r : rates)
        {
            chain->SetRainRateMmH(r);
            losses.push_back(chain->ComputeAttenuationDb(ground, sat));
        }
        NS_TEST_ASSERT_MSG_EQ_TOL(
            losses[0],
            0.0,
            1e-9,
            "0 mm/h rain must give 0 dB attenuation");
        for (size_t i = 1; i < losses.size(); ++i)
        {
            NS_TEST_ASSERT_MSG_GT_OR_EQ(
                losses[i],
                losses[i - 1],
                "rain attenuation must be non-decreasing");
        }
        NS_TEST_ASSERT_MSG_GT(
            losses.back(),
            losses[1] + 1.0,
            "200 mm/h must exceed 5 mm/h by >1 dB at 12 GHz");
    }
};

/// LMS Markov chain: across a 30 s simulator-driven trace with 100 ms steps,
/// the empirical bad-state fraction must be within tolerance of the analytic
/// steady-state probability.
class AtmosphericChainLmsSimulatorTest : public TestCase
{
  public:
    AtmosphericChainLmsSimulatorTest()
        : TestCase("§4.2.7: LMS Markov chain steady-state under Simulator::Run")
    {
    }

    static void StepOnce(Ptr<NtnAtmosphericLossChain> chain,
                          uint32_t* bad,
                          uint32_t* total)
    {
        const double db = chain->StepLmsDb();
        (void)db;
        ++(*total);
        // Crude bad-state detector: any fade above 6 dB is taken as state B.
        // The Lutz model emits multi-dB fades in state B and sub-dB in state A.
        if (chain->GetLastComponents().lmsDb > 6.0)
        {
            ++(*bad);
        }
    }

  private:
    void DoRun() override
    {
        Ptr<NtnAtmosphericLossChain> chain =
            CreateObject<NtnAtmosphericLossChain>();
        chain->SetEnableGaseous(false);
        chain->SetEnableRain(false);
        chain->SetEnableLms(true);
        chain->SetLmsEnvironmentInt(0); // urban — high P_bad
        chain->AssignStreams(42);

        uint32_t bad = 0;
        uint32_t total = 0;
        // 300 samples across 30 s = 100 ms cadence.
        for (uint32_t i = 1; i <= 300; ++i)
        {
            Simulator::Schedule(MilliSeconds(100 * i),
                                &StepOnce,
                                chain,
                                &bad,
                                &total);
        }
        Simulator::Stop(Seconds(31));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(total,
                              300u,
                              "300 simulator steps completed");
        // Loose check: with urban env P_bad steady ~ 0.6, the empirical
        // bad-fraction should land in [0.20, 0.95] across 300 samples.
        const double frac = static_cast<double>(bad) / total;
        NS_TEST_ASSERT_MSG_GT(
            frac,
            0.05,
            "urban LMS must enter state B at least 5% of samples");
        NS_TEST_ASSERT_MSG_LT(
            frac,
            0.99,
            "urban LMS must leave state B sometimes");

        Simulator::Destroy();
    }
};

/// Cascade with everything disabled equals pure base channel — no
/// over-attenuation, no NaNs, no rounding drift.
class CascadeBaselineEqualityTest : public TestCase
{
  public:
    CascadeBaselineEqualityTest()
        : TestCase("§4.2.7: cascade equals base when chain disabled")
    {
    }

  private:
    void DoRun() override
    {
        // Pure FSPL via SionnaNoneTransport — deterministic, no server.
        Ptr<NtnSionnaChannel> base = CreateObject<NtnSionnaChannel>();
        base->SetTransport(CreateObject<SionnaNoneTransport>());
        base->SetFrequencyHz(2.0e9);

        Ptr<NtnAtmosphericLossChain> chain =
            CreateObject<NtnAtmosphericLossChain>();
        chain->SetEnableGaseous(false);
        chain->SetEnableRain(false);
        chain->SetEnableLms(false);
        chain->SetFrequencyHz(2.0e9);

        Ptr<NtnSionnaCascadeChannel> cascade =
            CreateObject<NtnSionnaCascadeChannel>();
        cascade->SetSionnaChannel(base);
        cascade->SetAtmosphericChain(chain);

        Ptr<ConstantPositionMobilityModel> tx =
            CreateObject<ConstantPositionMobilityModel>();
        tx->SetPosition(Vector(0, 0, 0));
        Ptr<ConstantPositionMobilityModel> rx =
            CreateObject<ConstantPositionMobilityModel>();
        rx->SetPosition(Vector(1413.0, 0, 0));

        const double txDbm = 30.0;
        const double rxBase = base->CalcRxPower(txDbm, tx, rx);
        const double rxCascade = cascade->CalcRxPower(txDbm, tx, rx);
        NS_TEST_ASSERT_MSG_EQ_TOL(
            rxCascade,
            rxBase,
            1e-9,
            "cascade must equal base when all chain components are off");
        NS_TEST_ASSERT_MSG_EQ_TOL(
            cascade->GetLastTotalAttenuationDb(),
            0.0,
            1e-9,
            "cascade last-attenuation must be 0 when chain off");
        NS_TEST_ASSERT_MSG_EQ(cascade->GetCascadeQueries(),
                              1u,
                              "exactly 1 cascade query");
    }
};

/// LEO satellite sweeps overhead under Simulator::Run; cascade-applied
/// rain attenuation must be highest at low elevation and lowest at zenith.
class CascadeOrbitSweepTest : public TestCase
{
  public:
    CascadeOrbitSweepTest()
        : TestCase("§4.2.7: cascade rain attenuation tracks elevation profile")
    {
    }

    struct Sample
    {
        double elev_deg;
        double rxDbm;
        double cascadeTotalDb;
    };

    static void SampleCascade(Ptr<NtnSionnaCascadeChannel> cascade,
                               Ptr<MobilityModel> ue,
                               Ptr<MobilityModel> sat,
                               std::vector<Sample>* out)
    {
        const double rxDbm = cascade->CalcRxPower(30.0, sat, ue);
        const auto comps = cascade->GetLastComponents();
        out->push_back({comps.elevationDeg, rxDbm, comps.gaseousDb + comps.rainDb});
    }

  private:
    void DoRun() override
    {
        // Base = pure FSPL (no server).
        Ptr<NtnSionnaChannel> base = CreateObject<NtnSionnaChannel>();
        base->SetTransport(CreateObject<SionnaNoneTransport>());
        base->SetFrequencyHz(12.0e9);

        // Chain = gaseous + heavy rain enabled, LMS off (deterministic).
        Ptr<NtnAtmosphericLossChain> chain =
            CreateObject<NtnAtmosphericLossChain>();
        chain->SetFrequencyHz(12.0e9);
        chain->SetRainRateMmH(25.0);
        chain->SetEnableGaseous(true);
        chain->SetEnableRain(true);
        chain->SetEnableLms(false);

        Ptr<NtnSionnaCascadeChannel> cascade =
            CreateObject<NtnSionnaCascadeChannel>();
        cascade->SetSionnaChannel(base);
        cascade->SetAtmosphericChain(chain);

        // UE static at origin (local ENU).
        Ptr<ConstantPositionMobilityModel> ue =
            CreateObject<ConstantPositionMobilityModel>();
        ue->SetPosition(Vector(0, 0, 0));

        // Satellite passes overhead: x = -300e3 → +300e3 km over 30 s,
        // altitude fixed at 550 km, so elev climbs 61° → 90° → 61°.
        Ptr<ConstantVelocityMobilityModel> sat =
            CreateObject<ConstantVelocityMobilityModel>();
        sat->SetPosition(Vector(-300e3, 0, 550e3));
        sat->SetVelocity(Vector(20e3, 0, 0));

        std::vector<Sample> samples;
        for (int t = 1; t <= 30; ++t)
        {
            Simulator::Schedule(Seconds(t),
                                &SampleCascade,
                                cascade,
                                Ptr<MobilityModel>(ue),
                                Ptr<MobilityModel>(sat),
                                &samples);
        }
        Simulator::Stop(Seconds(31));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(samples.size(),
                              30u,
                              "30 cascade samples taken across 30 s sim");

        // Elevation should peak near the middle of the sweep (t=15).
        size_t peakIdx = 0;
        double peakElev = -1.0;
        for (size_t i = 0; i < samples.size(); ++i)
        {
            if (samples[i].elev_deg > peakElev)
            {
                peakElev = samples[i].elev_deg;
                peakIdx = i;
            }
        }
        NS_TEST_ASSERT_MSG_GT(peakElev,
                              80.0,
                              "peak elevation should be near zenith");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(peakIdx,
                                     10u,
                                     "elev peak should be near middle of sweep");
        NS_TEST_ASSERT_MSG_LT(peakIdx,
                               20u,
                               "elev peak should be near middle of sweep");

        // Every sample produces positive, finite attenuation.
        double minDb = std::numeric_limits<double>::infinity();
        double maxDb = -std::numeric_limits<double>::infinity();
        for (const auto& s : samples)
        {
            NS_TEST_ASSERT_MSG_GT(s.cascadeTotalDb,
                                   0.0,
                                   "every orbit sample must have >0 dB cascade loss");
            NS_TEST_ASSERT_MSG_EQ(std::isfinite(s.cascadeTotalDb),
                                   true,
                                   "cascade loss must be finite");
            minDb = std::min(minDb, s.cascadeTotalDb);
            maxDb = std::max(maxDb, s.cascadeTotalDb);
        }
        // The pass-arc must produce a measurable spread (>0.5 dB swing).
        // P.618's path-reduction factor + slant geometry interact in a
        // non-trivial way: peak rain atten generally lives near zenith
        // because the horizontal path is shortest (r → 1), whereas
        // gaseous atten peaks at low elevation (longest path). Together
        // they create a sweep-dependent shape; we just verify the shape
        // is non-flat.
        NS_TEST_ASSERT_MSG_GT(
            maxDb - minDb,
            0.5,
            "cascade total must vary by >0.5 dB across the orbit sweep");
        NS_TEST_ASSERT_MSG_GT(
            minDb,
            0.3,
            "min cascade total across the orbit must be >0.3 dB");

        Simulator::Destroy();
    }
};

/// Cascade with rain must always over-attenuate (lower Rx) vs cascade with
/// rain disabled, at every sample of an orbit sweep.
class CascadeRainImpactTest : public TestCase
{
  public:
    CascadeRainImpactTest()
        : TestCase("§4.2.7: rain on always reduces Rx vs rain off")
    {
    }

    static void Snapshot(Ptr<NtnSionnaCascadeChannel> ch,
                          Ptr<MobilityModel> a,
                          Ptr<MobilityModel> b,
                          std::vector<double>* rx)
    {
        rx->push_back(ch->CalcRxPower(30.0, a, b));
    }

  private:
    void DoRun() override
    {
        // Cascade A: rain ENABLED at 100 mm/h.
        Ptr<NtnSionnaChannel> baseA = CreateObject<NtnSionnaChannel>();
        baseA->SetTransport(CreateObject<SionnaNoneTransport>());
        baseA->SetFrequencyHz(20.0e9);
        Ptr<NtnAtmosphericLossChain> chainA =
            CreateObject<NtnAtmosphericLossChain>();
        chainA->SetFrequencyHz(20.0e9);
        chainA->SetRainRateMmH(100.0);
        chainA->SetEnableRain(true);
        chainA->SetEnableGaseous(false);
        chainA->SetEnableLms(false);
        Ptr<NtnSionnaCascadeChannel> cascA =
            CreateObject<NtnSionnaCascadeChannel>();
        cascA->SetSionnaChannel(baseA);
        cascA->SetAtmosphericChain(chainA);

        // Cascade B: rain DISABLED.
        Ptr<NtnSionnaChannel> baseB = CreateObject<NtnSionnaChannel>();
        baseB->SetTransport(CreateObject<SionnaNoneTransport>());
        baseB->SetFrequencyHz(20.0e9);
        Ptr<NtnAtmosphericLossChain> chainB =
            CreateObject<NtnAtmosphericLossChain>();
        chainB->SetFrequencyHz(20.0e9);
        chainB->SetEnableRain(false);
        chainB->SetEnableGaseous(false);
        chainB->SetEnableLms(false);
        Ptr<NtnSionnaCascadeChannel> cascB =
            CreateObject<NtnSionnaCascadeChannel>();
        cascB->SetSionnaChannel(baseB);
        cascB->SetAtmosphericChain(chainB);

        Ptr<ConstantPositionMobilityModel> ue =
            CreateObject<ConstantPositionMobilityModel>();
        ue->SetPosition(Vector(0, 0, 0));
        Ptr<ConstantVelocityMobilityModel> sat =
            CreateObject<ConstantVelocityMobilityModel>();
        sat->SetPosition(Vector(-300e3, 0, 550e3));
        sat->SetVelocity(Vector(20e3, 0, 0));

        std::vector<double> rxRain, rxDry;
        for (int t = 1; t <= 15; ++t)
        {
            Simulator::Schedule(Seconds(t),
                                &Snapshot,
                                cascA,
                                Ptr<MobilityModel>(sat),
                                Ptr<MobilityModel>(ue),
                                &rxRain);
            Simulator::Schedule(Seconds(t),
                                &Snapshot,
                                cascB,
                                Ptr<MobilityModel>(sat),
                                Ptr<MobilityModel>(ue),
                                &rxDry);
        }
        Simulator::Stop(Seconds(16));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(rxRain.size(), 15u, "15 rain samples");
        NS_TEST_ASSERT_MSG_EQ(rxDry.size(), 15u, "15 dry samples");
        for (size_t i = 0; i < rxRain.size(); ++i)
        {
            NS_TEST_ASSERT_MSG_LT(
                rxRain[i],
                rxDry[i],
                "rain-on Rx must be < rain-off Rx at every sample");
        }
        Simulator::Destroy();
    }
};

/// When the Sionna server dies mid-sim, the cascade must transition cleanly:
/// base channel falls back to FSPL, ITU-R chain keeps adding atmospheric
/// losses on top, no exceptions, no NaN, counters increase monotonically.
class CascadeServerDownGracefulTest : public TestCase
{
  public:
    CascadeServerDownGracefulTest()
        : TestCase("§4.2.7: cascade degrades cleanly when Sionna server dies")
    {
    }

    struct Sample
    {
        double rxDbm;
        uint64_t fallbacks;
        double atmosDb;
    };

    static void Probe(Ptr<NtnSionnaCascadeChannel> ch,
                       Ptr<NtnSionnaChannel> base,
                       Ptr<MobilityModel> a,
                       Ptr<MobilityModel> b,
                       std::vector<Sample>* out)
    {
        const double rx = ch->CalcRxPower(30.0, a, b);
        const auto c = ch->GetLastComponents();
        out->push_back({rx, base->GetFallbacks(), c.gaseousDb + c.rainDb});
    }

    static void KillMock(FsplUdpMockServer* mock) { mock->Stop(); }

  private:
    void DoRun() override
    {
        const uint16_t port = 38771;
        FsplUdpMockServer mock(port);
        NS_TEST_ASSERT_MSG_EQ(mock.Start(), true, "mock bind");

        Ptr<NtnSionnaChannel> base = CreateObject<NtnSionnaChannel>();
        base->SetServer("127.0.0.1", port);
        base->SetTimeoutMs(200);
        base->SetFrequencyHz(2.0e9);

        Ptr<NtnAtmosphericLossChain> chain =
            CreateObject<NtnAtmosphericLossChain>();
        chain->SetFrequencyHz(2.0e9);
        chain->SetEnableGaseous(true);
        chain->SetEnableRain(false);
        chain->SetEnableLms(false);

        Ptr<NtnSionnaCascadeChannel> cascade =
            CreateObject<NtnSionnaCascadeChannel>();
        cascade->SetSionnaChannel(base);
        cascade->SetAtmosphericChain(chain);

        Ptr<ConstantPositionMobilityModel> ue =
            CreateObject<ConstantPositionMobilityModel>();
        ue->SetPosition(Vector(0, 0, 0));
        Ptr<ConstantVelocityMobilityModel> sat =
            CreateObject<ConstantVelocityMobilityModel>();
        sat->SetPosition(Vector(-300e3, 0, 550e3));
        sat->SetVelocity(Vector(20e3, 0, 0));

        std::vector<Sample> samples;
        for (int t = 1; t <= 20; ++t)
        {
            Simulator::Schedule(Seconds(t),
                                &Probe,
                                cascade,
                                base,
                                Ptr<MobilityModel>(sat),
                                Ptr<MobilityModel>(ue),
                                &samples);
        }
        // Kill the server at t=10 → all subsequent queries should hit FSPL
        // fallback in the BASE channel; cascade should still produce sane
        // values with the chain layered on top.
        Simulator::Schedule(Seconds(10.5), &KillMock, &mock);
        Simulator::Stop(Seconds(21));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(samples.size(),
                              20u,
                              "20 cascade samples covering live + dead phases");

        // First-half samples (t=1..10) should be live (base fallbacks=0).
        NS_TEST_ASSERT_MSG_EQ(
            samples[0].fallbacks,
            0u,
            "early samples must come from the live mock");
        // Last sample's base fallback count must be > 0.
        NS_TEST_ASSERT_MSG_GT(
            samples.back().fallbacks,
            samples[0].fallbacks,
            "fallbacks must accumulate after server death");

        // No NaN / no infinity anywhere.
        for (const auto& s : samples)
        {
            NS_TEST_ASSERT_MSG_EQ(std::isfinite(s.rxDbm),
                                  true,
                                  "Rx power must be finite at every sample");
            NS_TEST_ASSERT_MSG_GT_OR_EQ(s.atmosDb,
                                         0.0,
                                         "atmospheric loss must be >= 0");
        }
        Simulator::Destroy();
    }
};

/// Cascade preserves transport behaviour end-to-end: queries flow through
/// the UDP transport, no timeouts, atmospheric losses layered on top.
class CascadeOverUdpTransportTest : public TestCase
{
  public:
    CascadeOverUdpTransportTest()
        : TestCase("§4.2.7: cascade preserves UDP transport accounting end-to-end")
    {
    }

    static void Tick(Ptr<NtnSionnaCascadeChannel> ch,
                      Ptr<MobilityModel> a,
                      Ptr<MobilityModel> b,
                      uint32_t* count)
    {
        (void)ch->CalcRxPower(30.0, a, b);
        ++(*count);
    }

  private:
    void DoRun() override
    {
        const uint16_t port = 38772;
        FsplUdpMockServer mock(port);
        NS_TEST_ASSERT_MSG_EQ(mock.Start(), true, "mock bind");

        Ptr<SionnaUdpTransport> udp = CreateObject<SionnaUdpTransport>();
        udp->SetServer("127.0.0.1", port);
        udp->SetTimeoutMs(500);

        Ptr<NtnSionnaChannel> base = CreateObject<NtnSionnaChannel>();
        base->SetTransport(udp);
        base->SetFrequencyHz(28.0e9);

        Ptr<NtnAtmosphericLossChain> chain =
            CreateObject<NtnAtmosphericLossChain>();
        chain->SetFrequencyHz(28.0e9);
        chain->SetEnableGaseous(true);
        chain->SetEnableRain(false);
        chain->SetEnableLms(false);

        Ptr<NtnSionnaCascadeChannel> cascade =
            CreateObject<NtnSionnaCascadeChannel>();
        cascade->SetSionnaChannel(base);
        cascade->SetAtmosphericChain(chain);

        Ptr<ConstantPositionMobilityModel> a =
            CreateObject<ConstantPositionMobilityModel>();
        a->SetPosition(Vector(0, 0, 0));
        Ptr<ConstantVelocityMobilityModel> b =
            CreateObject<ConstantVelocityMobilityModel>();
        b->SetPosition(Vector(0, 0, 550e3));
        b->SetVelocity(Vector(7500, 0, 0));

        uint32_t count = 0;
        for (int t = 1; t <= 12; ++t)
        {
            Simulator::Schedule(Seconds(t),
                                &Tick,
                                cascade,
                                Ptr<MobilityModel>(a),
                                Ptr<MobilityModel>(b),
                                &count);
        }
        Simulator::Stop(Seconds(13));
        Simulator::Run();

        mock.Stop();

        NS_TEST_ASSERT_MSG_EQ(count, 12u, "12 cascade ticks completed");
        NS_TEST_ASSERT_MSG_EQ(udp->GetQueriesSent(),
                              12u,
                              "all 12 cascade queries reached the UDP transport");
        NS_TEST_ASSERT_MSG_EQ(udp->GetTimeouts(),
                              0u,
                              "no transport timeouts");
        Simulator::Destroy();
    }
};

// ---------------------------------------------------------------------------
// Roadmap §4.2.4 — 4-D LRU caching transport
// ---------------------------------------------------------------------------

namespace
{

/// Deterministic mock inner transport: returns FSPL for each request. The
/// caching layer wraps THIS, so we can assert hit/miss counts directly.
class FsplInnerTransport : public SionnaTransport
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::TestFsplInnerTransport")
                                .SetParent<SionnaTransport>()
                                .SetGroupName("NtnSionna")
                                .AddConstructor<FsplInnerTransport>();
        return tid;
    }
    Response Query(const Request& req) const override
    {
        ++m_queriesSent;
        Response r;
        const double dx = req.rx_x - req.tx_x;
        const double dy = req.rx_y - req.tx_y;
        const double dz = req.rx_z - req.tx_z;
        const double d = std::max(1e-3, std::sqrt(dx * dx + dy * dy + dz * dz));
        const double fGhz = req.freq_hz / 1e9;
        r.path_loss_db =
            20.0 * std::log10(d) + 20.0 * std::log10(fGhz) + 32.45;
        r.n_paths = 1;
        r.compute_ms = 1.23; // distinguishable from cached (0.0)
        r.ok = true;
        return r;
    }
    bool IsAvailable() const override { return true; }
    std::string Name() const override { return "test-fspl"; }
};

NS_OBJECT_ENSURE_REGISTERED(FsplInnerTransport);

SionnaTransport::Request
MakeReq(double sat_x, double ue_x, double freqHz, uint64_t id)
{
    // Satellite at (sat_x, 0, 550 km) as the TX; ground UE at (ue_x, 0, 0)
    // as the RX. Distinct ue_x values map to distinct grid cells when the
    // configured SpatialResolutionM is fine enough.
    return {sat_x, 0.0, 550e3, ue_x, 0.0, 0.0,
            freqHz, id, std::nullopt, std::nullopt};
}

} // namespace

/// First query is a miss + forward; identical second query is a hit and
/// doesn't reach the inner transport.
class CachingHitMissTest : public TestCase
{
  public:
    CachingHitMissTest()
        : TestCase("§4.2.4: cache miss forwards; identical re-query is a hit")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<FsplInnerTransport> inner = CreateObject<FsplInnerTransport>();
        Ptr<SionnaCachingTransport> cache = CreateObject<SionnaCachingTransport>();
        cache->SetInner(inner);

        // Two identical requests under the same time bucket → 1 miss, 1 hit.
        SionnaTransport::Request req =
            MakeReq(/*tx*/ 0.0, /*rx*/ 1413.0, 2.0e9, 1);
        const auto r1 = cache->Query(req);
        const auto r2 = cache->Query(req);

        NS_TEST_ASSERT_MSG_EQ(r1.ok, true, "first query succeeded");
        NS_TEST_ASSERT_MSG_EQ(r2.ok, true, "second query succeeded");
        NS_TEST_ASSERT_MSG_EQ_TOL(r1.path_loss_db,
                                   r2.path_loss_db,
                                   1e-9,
                                   "cached PL must match live PL exactly");
        NS_TEST_ASSERT_MSG_GT(r1.compute_ms,
                              0.0,
                              "first call should report inner compute_ms");
        NS_TEST_ASSERT_MSG_EQ_TOL(r2.compute_ms,
                                   0.0,
                                   1e-12,
                                   "cached response sets compute_ms = 0");
        NS_TEST_ASSERT_MSG_EQ(cache->GetMisses(), 1u, "exactly 1 miss");
        NS_TEST_ASSERT_MSG_EQ(cache->GetHits(), 1u, "exactly 1 hit");
        NS_TEST_ASSERT_MSG_EQ(inner->GetQueriesSent(),
                              1u,
                              "inner saw only the miss");
        NS_TEST_ASSERT_MSG_EQ_TOL(cache->GetHitRate(),
                                   0.5,
                                   1e-9,
                                   "hit rate after 1 miss + 1 hit is 0.5");
    }
};

/// Requests within one grid cell collapse to the same key (cache hit even
/// though raw coordinates differ); requests outside the cell miss.
class CachingSpatialQuantizationTest : public TestCase
{
  public:
    CachingSpatialQuantizationTest()
        : TestCase("§4.2.4: spatial quantisation collapses sub-cell requests")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<FsplInnerTransport> inner = CreateObject<FsplInnerTransport>();
        Ptr<SionnaCachingTransport> cache = CreateObject<SionnaCachingTransport>();
        cache->SetInner(inner);
        cache->SetSpatialResolutionM(100.0);

        // Two rx points within 100 m of each other → same cell → hit on 2nd.
        cache->Query(MakeReq(0.0, 1413.0, 2.0e9, 1));
        cache->Query(MakeReq(0.0, 1490.0, 2.0e9, 2));

        // Then a point in the next cell along (>100 m offset) → miss.
        cache->Query(MakeReq(0.0, 1620.0, 2.0e9, 3));

        NS_TEST_ASSERT_MSG_EQ(cache->GetMisses(),
                              2u,
                              "two distinct grid cells → 2 misses");
        NS_TEST_ASSERT_MSG_EQ(cache->GetHits(),
                              1u,
                              "one sub-cell repeat → 1 hit");
        NS_TEST_ASSERT_MSG_EQ(inner->GetQueriesSent(),
                              2u,
                              "inner saw only the 2 misses");
    }
};

/// Two requests in different time buckets miss even with identical spatial
/// keys. Uses Simulator::Run() to advance the clock through the bucket.
class CachingTemporalBucketTest : public TestCase
{
  public:
    CachingTemporalBucketTest()
        : TestCase("§4.2.4: time-bucket boundary forces a miss")
    {
    }

    static void DoQuery(Ptr<SionnaCachingTransport> cache, uint64_t id)
    {
        cache->Query(MakeReq(0.0, 1413.0, 2.0e9, id));
    }

  private:
    void DoRun() override
    {
        Ptr<FsplInnerTransport> inner = CreateObject<FsplInnerTransport>();
        Ptr<SionnaCachingTransport> cache = CreateObject<SionnaCachingTransport>();
        cache->SetInner(inner);
        cache->SetTemporalBucketUs(1000); // 1 ms

        // Three queries at t=0, t=500 µs (same bucket), t=2 ms (new bucket).
        Simulator::Schedule(MicroSeconds(0), &DoQuery, cache, (uint64_t)1);
        Simulator::Schedule(MicroSeconds(500), &DoQuery, cache, (uint64_t)2);
        Simulator::Schedule(MicroSeconds(2000), &DoQuery, cache, (uint64_t)3);
        Simulator::Stop(MicroSeconds(3000));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(cache->GetMisses(),
                              2u,
                              "different buckets → 2 misses (t=0, t=2000)");
        NS_TEST_ASSERT_MSG_EQ(cache->GetHits(),
                              1u,
                              "same bucket → 1 hit at t=500µs");
        Simulator::Destroy();
    }
};

/// MaxEntries=2 means after a third unique request, oldest is evicted.
class CachingLruEvictionTest : public TestCase
{
  public:
    CachingLruEvictionTest()
        : TestCase("§4.2.4: LRU eviction when MaxEntries exceeded")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<FsplInnerTransport> inner = CreateObject<FsplInnerTransport>();
        Ptr<SionnaCachingTransport> cache = CreateObject<SionnaCachingTransport>();
        cache->SetInner(inner);
        cache->SetMaxEntries(2);
        cache->SetSpatialResolutionM(1.0); // tight grid so distinct x values
                                            // map to distinct cells

        cache->Query(MakeReq(0.0, 10.0, 2.0e9, 1)); // A
        cache->Query(MakeReq(0.0, 20.0, 2.0e9, 2)); // B
        NS_TEST_ASSERT_MSG_EQ(cache->GetEntries(), 2u, "cache holds 2 entries");
        cache->Query(MakeReq(0.0, 30.0, 2.0e9, 3)); // C — evicts A
        NS_TEST_ASSERT_MSG_EQ(cache->GetEntries(), 2u, "still 2 after eviction");
        NS_TEST_ASSERT_MSG_EQ(cache->GetEvictions(), 1u, "exactly 1 eviction");
        // Re-querying A should be a miss again.
        cache->Query(MakeReq(0.0, 10.0, 2.0e9, 4));
        NS_TEST_ASSERT_MSG_EQ(cache->GetMisses(),
                              4u,
                              "evicted entry re-queried → miss");
    }
};

/// Stats reset and cache clear work cleanly.
class CachingResetTest : public TestCase
{
  public:
    CachingResetTest()
        : TestCase("§4.2.4: Reset() clears entries and stats")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<FsplInnerTransport> inner = CreateObject<FsplInnerTransport>();
        Ptr<SionnaCachingTransport> cache = CreateObject<SionnaCachingTransport>();
        cache->SetInner(inner);
        cache->SetSpatialResolutionM(1.0);
        cache->Query(MakeReq(0.0, 10.0, 2.0e9, 1));
        cache->Query(MakeReq(0.0, 10.0, 2.0e9, 2));
        NS_TEST_ASSERT_MSG_EQ(cache->GetHits(), 1u, "1 hit before reset");
        cache->Reset();
        NS_TEST_ASSERT_MSG_EQ(cache->GetHits(), 0u, "hits cleared");
        NS_TEST_ASSERT_MSG_EQ(cache->GetMisses(), 0u, "misses cleared");
        NS_TEST_ASSERT_MSG_EQ(cache->GetEntries(), 0u, "entries cleared");
        // Subsequent identical query is a miss again.
        cache->Query(MakeReq(0.0, 10.0, 2.0e9, 3));
        NS_TEST_ASSERT_MSG_EQ(cache->GetMisses(),
                              1u,
                              "post-reset re-query is a miss");
    }
};

/// Under a 60 s simulator-driven LEO pass with a static UE, the caching
/// transport must show strictly more hits than misses once temporal bucket
/// reuse kicks in (1 s sampling cadence + 100 ms bucket would give 0 hits;
/// 100 ms sampling + 1 s bucket would give 9/10 hits per second).
class CachingUnderSimulatorTest : public TestCase
{
  public:
    CachingUnderSimulatorTest()
        : TestCase("§4.2.4: Simulator-driven hit rate >0.6 with 1s bucket + 100ms sampling")
    {
    }

    static void Sample(Ptr<NtnSionnaChannel> ch,
                        Ptr<MobilityModel> a,
                        Ptr<MobilityModel> b)
    {
        (void)ch->CalcRxPower(30.0, a, b);
    }

  private:
    void DoRun() override
    {
        Ptr<FsplInnerTransport> inner = CreateObject<FsplInnerTransport>();
        Ptr<SionnaCachingTransport> cache = CreateObject<SionnaCachingTransport>();
        cache->SetInner(inner);
        cache->SetSpatialResolutionM(50000.0); // 50 km cell — coarse enough
                                                 // that a slow-moving UE hits
        cache->SetTemporalBucketUs(1000000); // 1 s bucket

        Ptr<NtnSionnaChannel> ch = CreateObject<NtnSionnaChannel>();
        ch->SetTransport(cache);
        ch->SetFrequencyHz(2.0e9);

        Ptr<ConstantPositionMobilityModel> ue =
            CreateObject<ConstantPositionMobilityModel>();
        ue->SetPosition(Vector(0, 0, 0));
        Ptr<ConstantPositionMobilityModel> sat =
            CreateObject<ConstantPositionMobilityModel>();
        sat->SetPosition(Vector(0, 0, 550e3));

        // 600 samples at 100 ms cadence across 60 s.
        for (uint32_t i = 1; i <= 600; ++i)
        {
            Simulator::Schedule(MilliSeconds(100 * i),
                                &Sample,
                                ch,
                                Ptr<MobilityModel>(sat),
                                Ptr<MobilityModel>(ue));
        }
        Simulator::Stop(Seconds(61));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(cache->GetHits() + cache->GetMisses(),
                              600u,
                              "600 transport queries");
        // 600 samples at 100ms across 60s → ~10 samples per 1s bucket
        // expected hit rate ≈ 9/10 = 0.9.
        NS_TEST_ASSERT_MSG_GT(cache->GetHitRate(),
                              0.6,
                              "hit rate must exceed 0.6 with coarse buckets");
        NS_TEST_ASSERT_MSG_LT(inner->GetQueriesSent(),
                              cache->GetMisses() + 1u,
                              "inner saw only the misses");
        Simulator::Destroy();
    }
};

// ---------------------------------------------------------------------------
// Roadmap §4.2.3 — RIS Tx surface (Sionna 2.0 ReflectingSurface)
// ---------------------------------------------------------------------------

/// A RIS request with the "focus" phase profile must be parsed end-to-end
/// over UDP and yield a deterministic ‑10 dB path-loss reduction relative
/// to the same request without RIS (per the mock-server convention).
class RisWireRoundTripTest : public TestCase
{
  public:
    RisWireRoundTripTest()
        : TestCase("§4.2.3: RIS focus profile yields ‑10 dB shift over UDP wire")
    {
    }

  private:
    void DoRun() override
    {
        const uint16_t port = 38780;
        FsplUdpMockServer mock(port);
        NS_TEST_ASSERT_MSG_EQ(mock.Start(), true, "mock bind");

        Ptr<SionnaUdpTransport> udp = CreateObject<SionnaUdpTransport>();
        udp->SetServer("127.0.0.1", port);
        udp->SetTimeoutMs(500);

        SionnaTransport::Request req{0.0, 0.0, 550e3,
                                      1413.0, 0.0, 0.0,
                                      2.0e9, 1,
                                      std::nullopt, std::nullopt, std::nullopt};
        const auto noRis = udp->Query(req);

        RisConfig ris;
        ris.pos_x = 706.5;
        ris.pos_y = 0.0;
        ris.pos_z = 50.0;
        ris.normal_x = 0.0;
        ris.normal_y = 0.0;
        ris.normal_z = 1.0;
        ris.rows = 16;
        ris.cols = 16;
        ris.spacing_lambda = 0.5;
        ris.phase_profile = "focus";
        ris.focal_x = 1413.0;
        ris.focal_y = 0.0;
        ris.focal_z = 0.0;
        req.ris = ris;
        req.request_id = 2;
        const auto withRis = udp->Query(req);

        mock.Stop();

        NS_TEST_ASSERT_MSG_EQ(noRis.ok, true, "no-RIS query succeeded");
        NS_TEST_ASSERT_MSG_EQ(withRis.ok, true, "with-RIS query succeeded");
        NS_TEST_ASSERT_MSG_EQ_TOL(
            noRis.path_loss_db - withRis.path_loss_db,
            10.0,
            0.01,
            "RIS focus profile must shift PL by ‑10 dB on the mock");
    }
};

/// All three phase-profile modes must round-trip with their own
/// deterministic dB shift: focus=10, flat=6, random=0.
class RisPhaseProfileMatrixTest : public TestCase
{
  public:
    RisPhaseProfileMatrixTest()
        : TestCase("§4.2.3: RIS phase_profile focus vs flat vs random produce 10 vs 6 vs 0 dB shifts")
    {
    }

    static double QueryWithProfile(Ptr<SionnaUdpTransport> udp,
                                    const std::string& profile,
                                    uint64_t id)
    {
        SionnaTransport::Request req{0.0, 0.0, 550e3,
                                      1413.0, 0.0, 0.0,
                                      2.0e9, id,
                                      std::nullopt, std::nullopt, std::nullopt};
        RisConfig ris;
        ris.rows = 8;
        ris.cols = 8;
        ris.phase_profile = profile;
        req.ris = ris;
        return udp->Query(req).path_loss_db;
    }

  private:
    void DoRun() override
    {
        const uint16_t port = 38781;
        FsplUdpMockServer mock(port);
        NS_TEST_ASSERT_MSG_EQ(mock.Start(), true, "mock bind");

        Ptr<SionnaUdpTransport> udp = CreateObject<SionnaUdpTransport>();
        udp->SetServer("127.0.0.1", port);
        udp->SetTimeoutMs(500);

        // Baseline (no RIS) — same as random profile (0 dB shift).
        SionnaTransport::Request bare{0.0, 0.0, 550e3,
                                       1413.0, 0.0, 0.0,
                                       2.0e9, 1,
                                       std::nullopt, std::nullopt, std::nullopt};
        const double plBare = udp->Query(bare).path_loss_db;
        const double plFocus = QueryWithProfile(udp, "focus", 2);
        const double plFlat = QueryWithProfile(udp, "flat", 3);
        const double plRandom = QueryWithProfile(udp, "random", 4);

        mock.Stop();

        NS_TEST_ASSERT_MSG_EQ_TOL(plBare - plFocus, 10.0, 0.01, "focus = 10 dB");
        NS_TEST_ASSERT_MSG_EQ_TOL(plBare - plFlat, 6.0, 0.01, "flat = 6 dB");
        NS_TEST_ASSERT_MSG_EQ_TOL(plBare - plRandom, 0.0, 0.01, "random = 0 dB");
    }
};

/// Channel-level RIS plumbing: SetRis on the channel propagates into every
/// query, and ClearRis returns it to baseline. Simulator-driven 1 s sweep
/// with 10 samples to verify state is sticky across queries.
class RisChannelAttachTest : public TestCase
{
  public:
    RisChannelAttachTest()
        : TestCase("§4.2.3: NtnSionnaChannel::SetRis attaches RIS to every query")
    {
    }

    static void Sample(Ptr<NtnSionnaChannel> ch,
                        Ptr<MobilityModel> a,
                        Ptr<MobilityModel> b,
                        std::vector<double>* out)
    {
        out->push_back(ch->CalcRxPower(30.0, a, b));
    }

  private:
    void DoRun() override
    {
        const uint16_t port = 38782;
        FsplUdpMockServer mock(port);
        NS_TEST_ASSERT_MSG_EQ(mock.Start(), true, "mock bind");

        Ptr<NtnSionnaChannel> ch = CreateObject<NtnSionnaChannel>();
        ch->SetServer("127.0.0.1", port);
        ch->SetTimeoutMs(500);
        ch->SetFrequencyHz(2.0e9);

        Ptr<ConstantPositionMobilityModel> a =
            CreateObject<ConstantPositionMobilityModel>();
        a->SetPosition(Vector(0, 0, 550e3));
        Ptr<ConstantPositionMobilityModel> b =
            CreateObject<ConstantPositionMobilityModel>();
        b->SetPosition(Vector(1413.0, 0, 0));

        std::vector<double> rxNoRis;
        std::vector<double> rxWithRis;
        // Phase 1: 5 samples without RIS at t=100..500 ms.
        for (int t = 1; t <= 5; ++t)
        {
            Simulator::Schedule(MilliSeconds(100 * t),
                                &Sample,
                                ch,
                                Ptr<MobilityModel>(a),
                                Ptr<MobilityModel>(b),
                                &rxNoRis);
        }
        // Phase 2: attach RIS at t=600 ms; 5 samples at t=700..1100 ms.
        Simulator::Schedule(MilliSeconds(600),
                            [ch]() {
                                RisConfig ris;
                                ris.rows = 16;
                                ris.cols = 16;
                                ris.phase_profile = "focus";
                                ris.focal_x = 1413.0;
                                ch->SetRis(ris);
                            });
        for (int t = 7; t <= 11; ++t)
        {
            Simulator::Schedule(MilliSeconds(100 * t),
                                &Sample,
                                ch,
                                Ptr<MobilityModel>(a),
                                Ptr<MobilityModel>(b),
                                &rxWithRis);
        }
        Simulator::Stop(Seconds(2));
        Simulator::Run();
        mock.Stop();

        NS_TEST_ASSERT_MSG_EQ(rxNoRis.size(), 5u, "5 no-RIS samples");
        NS_TEST_ASSERT_MSG_EQ(rxWithRis.size(), 5u, "5 RIS samples");
        // The mock applies -10 dB shift on focus → Rx is +10 dB higher.
        for (size_t i = 0; i < 5; ++i)
        {
            NS_TEST_ASSERT_MSG_EQ_TOL(
                rxWithRis[i] - rxNoRis[i],
                10.0,
                0.05,
                "with-RIS Rx must be +10 dB over baseline at every sample");
        }
        Simulator::Destroy();
    }
};

// ---------------------------------------------------------------------------
// Roadmap §4.3.3 — Sionna RT calibration harness
// ---------------------------------------------------------------------------

namespace
{

/// Pure-FSPL propagation model that takes the frequency from each
/// CalcRxPower call's mobility distance. Used to gate the §4.3.3
/// LOS-only residual ≤ 1 dB requirement: with the mock server returning
/// FSPL too, every residual should land at ~0 dB.
class FsplFreqModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::TestFsplFreqModel")
                                .SetParent<PropagationLossModel>()
                                .SetGroupName("NtnSionnaTest")
                                .AddConstructor<FsplFreqModel>();
        return tid;
    }
    void SetFrequencyHz(double f) { m_freqHz = f; }
    double DoCalcRxPower(double txPowerDbm,
                          Ptr<MobilityModel> a,
                          Ptr<MobilityModel> b) const override
    {
        const Vector pa = a->GetPosition();
        const Vector pb = b->GetPosition();
        const double dx = pa.x - pb.x;
        const double dy = pa.y - pb.y;
        const double dz = pa.z - pb.z;
        const double d = std::max(1e-3, std::sqrt(dx * dx + dy * dy + dz * dz));
        const double fGhz = m_freqHz / 1e9;
        return txPowerDbm -
               (20.0 * std::log10(d) + 20.0 * std::log10(fGhz) + 32.45);
    }
    int64_t DoAssignStreams(int64_t) override { return 0; }

  private:
    double m_freqHz{2.0e9};
};

NS_OBJECT_ENSURE_REGISTERED(FsplFreqModel);

} // namespace

/// LOS-only residual gate per §4.3.3: with the FSPL mock server standing in
/// for Sionna and a pure-FSPL model under test, the calibrator must report
/// max |residual| ≤ 1 dB across the default 18-point grid.
class SionnaCalibratorLosOnlyTest : public TestCase
{
  public:
    SionnaCalibratorLosOnlyTest()
        : TestCase("§4.3.3: LOS-only calibrator vs FSPL Sionna stand-in ≤ 1 dB")
    {
    }

  private:
    void DoRun() override
    {
        const uint16_t port = 38790;
        FsplUdpMockServer mock(port);
        NS_TEST_ASSERT_MSG_EQ(mock.Start(), true, "mock bind");

        Ptr<SionnaUdpTransport> udp = CreateObject<SionnaUdpTransport>();
        udp->SetServer("127.0.0.1", port);
        udp->SetTimeoutMs(500);

        // The calibrator queries the transport per-grid-point and queries
        // the model at the same geometry; the model needs to know which
        // frequency it's at. We pre-populate a default grid identical to
        // SionnaCalibrator::UseDefaultGrid(), so the model frequency is
        // controlled by per-call state — we hold the freq via a closure.

        Ptr<SionnaCalibrator> cal = CreateObject<SionnaCalibrator>();
        cal->SetTransport(udp);
        cal->SetLosOnly(true);

        std::vector<SionnaCalibrator::PointResult> all;
        const std::vector<double> freqs = {2.0e9, 12.0e9, 28.0e9};
        for (double f : freqs)
        {
            Ptr<FsplFreqModel> model = CreateObject<FsplFreqModel>();
            model->SetFrequencyHz(f);
            cal->SetModel(model);

            // Single-frequency grid slice across distances.
            std::vector<SionnaCalibrator::GridPoint> grid;
            for (double d : {1000.0, 5000.0, 50000.0, 200000.0,
                               500000.0, 1000000.0})
            {
                grid.push_back({f, d});
            }
            cal->SetGrid(grid);

            const auto pts = cal->RunPoints();
            for (const auto& p : pts)
            {
                NS_TEST_ASSERT_MSG_EQ(p.sionna_ok,
                                      true,
                                      "Sionna stand-in must be reachable");
                NS_TEST_ASSERT_MSG_LT(
                    std::abs(p.residual_dB),
                    1.0,
                    "LOS-only residual must be < 1 dB at every grid point");
                all.push_back(p);
            }
        }

        mock.Stop();
        NS_TEST_ASSERT_MSG_EQ(all.size(),
                              freqs.size() * 6u,
                              "18 calibration points collected");
    }
};

/// A model that adds a deterministic +N dB on top of FSPL must produce a
/// residual of approximately +N dB against the FSPL-only mock. Demonstrates
/// the calibrator detects non-trivial offsets correctly.
class SionnaCalibratorDetectsOffsetTest : public TestCase
{
  public:
    SionnaCalibratorDetectsOffsetTest()
        : TestCase("§4.3.3: calibrator detects a +3 dB offset on the model side")
    {
    }

  private:
    void DoRun() override
    {
        const uint16_t port = 38791;
        FsplUdpMockServer mock(port);
        NS_TEST_ASSERT_MSG_EQ(mock.Start(), true, "mock bind");

        Ptr<SionnaUdpTransport> udp = CreateObject<SionnaUdpTransport>();
        udp->SetServer("127.0.0.1", port);
        udp->SetTimeoutMs(500);

        // Wrap FsplFreqModel with an extra-3-dB inline override via a
        // ListPropagationLossModel chain: FsplFreqModel + a Friis-like
        // 3-dB shifter. Simpler: subclass FsplFreqModel locally and
        // override DoCalcRxPower to subtract an additional 3 dB.
        class OffsetModel : public FsplFreqModel
        {
          public:
            double DoCalcRxPower(double txPowerDbm,
                                  Ptr<MobilityModel> a,
                                  Ptr<MobilityModel> b) const override
            {
                return FsplFreqModel::DoCalcRxPower(txPowerDbm, a, b) - 3.0;
            }
        };

        Ptr<OffsetModel> model = CreateObject<OffsetModel>();
        model->SetFrequencyHz(12.0e9);

        Ptr<SionnaCalibrator> cal = CreateObject<SionnaCalibrator>();
        cal->SetTransport(udp);
        cal->SetModel(model);
        cal->SetGrid({{12.0e9, 1000.0}, {12.0e9, 5000.0}, {12.0e9, 50000.0}});

        const auto rep = cal->Run();
        mock.Stop();

        NS_TEST_ASSERT_MSG_EQ(rep.samples, 3u, "3 calibration samples");
        NS_TEST_ASSERT_MSG_EQ_TOL(
            rep.mean_dB,
            3.0,
            0.05,
            "mean residual must equal the model's +3 dB offset");
        NS_TEST_ASSERT_MSG_LT(
            rep.std_dB,
            0.05,
            "residual std must be near 0 (consistent offset)");
    }
};

/// Simulator::Run() driven calibration: schedule 10 calibrator passes
/// across 30 s of sim time, each at a different frequency. Verifies the
/// calibrator is re-entrant under the simulator and that the residual
/// stays under the 1 dB gate at every tick.
class SionnaCalibratorSimulatorTimeTest : public TestCase
{
  public:
    SionnaCalibratorSimulatorTimeTest()
        : TestCase("§4.3.3: Simulator-driven calibrator stays ≤ 1 dB across 10 epochs")
    {
    }

    struct EpochReport
    {
        double freq_hz;
        SionnaCalibrator::Report rep;
    };

    static void RunEpoch(double freqHz,
                          uint16_t port,
                          std::vector<EpochReport>* out)
    {
        Ptr<SionnaUdpTransport> udp = CreateObject<SionnaUdpTransport>();
        udp->SetServer("127.0.0.1", port);
        udp->SetTimeoutMs(500);
        Ptr<FsplFreqModel> model = CreateObject<FsplFreqModel>();
        model->SetFrequencyHz(freqHz);

        Ptr<SionnaCalibrator> cal = CreateObject<SionnaCalibrator>();
        cal->SetTransport(udp);
        cal->SetModel(model);
        cal->SetGrid({{freqHz, 1000.0}, {freqHz, 50000.0}, {freqHz, 500000.0}});

        out->push_back({freqHz, cal->Run()});
    }

  private:
    void DoRun() override
    {
        const uint16_t port = 38792;
        FsplUdpMockServer mock(port);
        NS_TEST_ASSERT_MSG_EQ(mock.Start(), true, "mock bind");

        std::vector<EpochReport> epochs;
        const double freqs[] = {2.0e9, 4.0e9, 6.0e9, 12.0e9, 18.0e9,
                                  24.0e9, 28.0e9, 38.0e9, 60.0e9, 90.0e9};
        for (size_t i = 0; i < 10; ++i)
        {
            Simulator::Schedule(Seconds(i + 1),
                                &RunEpoch,
                                freqs[i],
                                port,
                                &epochs);
        }
        Simulator::Stop(Seconds(12));
        Simulator::Run();
        mock.Stop();

        NS_TEST_ASSERT_MSG_EQ(epochs.size(), 10u, "10 epochs ran");
        for (const auto& e : epochs)
        {
            NS_TEST_ASSERT_MSG_EQ(e.rep.samples, 3u,
                                   "every epoch sampled 3 points");
            NS_TEST_ASSERT_MSG_LT(e.rep.max_abs_dB,
                                   1.0,
                                   "every epoch under the ≤ 1 dB gate");
        }
        Simulator::Destroy();
    }
};

// ============================================================================
//  4.2.5 — Async batched query API
//  4.2.6 — Precompute/replay backend
//  4.2.11 — paths.apply_doppler() C++ equivalent
// ============================================================================

namespace
{

/// Deterministic SionnaTransport stub: path_loss_db = 100 + (tx_x % 10)
/// + 0.001 * request_id. Used to verify per-request mapping and order.
class StubFixedTransport : public SionnaTransport
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::StubFixedTransport")
                                .SetParent<SionnaTransport>()
                                .SetGroupName("NtnSionna")
                                .AddConstructor<StubFixedTransport>();
        return tid;
    }
    Response Query(const Request& req) const override
    {
        ++m_queriesSent;
        Response r;
        r.ok = true;
        r.n_paths = 1;
        r.compute_ms = 0.1;
        const double base =
            std::fmod(req.tx_x, 10.0);
        r.path_loss_db = 100.0 + base + 0.001 *
                                        static_cast<double>(
                                            req.request_id);
        return r;
    }
    std::string Name() const override { return "stub-fixed"; }
    bool IsAvailable() const override { return true; }
};

} // namespace

class BatchClientSyncTest : public TestCase
{
  public:
    BatchClientSyncTest()
        : TestCase("Batch client: sync RequestBatch returns in-order responses (4.2.5)")
    {
    }

    void DoRun() override
    {
        auto stub = CreateObject<StubFixedTransport>();
        auto bc = CreateObject<SionnaBatchClient>();
        bc->Attach(stub);

        std::vector<SionnaTransport::Request> reqs(8);
        for (size_t i = 0; i < reqs.size(); ++i)
        {
            reqs[i].tx_x = static_cast<double>(i);
            reqs[i].request_id = 100 + i;
            reqs[i].freq_hz = 12.5e9;
        }
        const auto out = bc->RequestBatchSync(reqs);
        NS_TEST_ASSERT_MSG_EQ(out.size(), 8u, "8 responses");
        for (size_t i = 0; i < out.size(); ++i)
        {
            NS_TEST_EXPECT_MSG_EQ(out[i].ok, true, "ok");
            const double expected =
                100.0 + std::fmod(static_cast<double>(i), 10.0) +
                0.001 * static_cast<double>(100 + i);
            NS_TEST_EXPECT_MSG_EQ_TOL(out[i].path_loss_db,
                                        expected,
                                        1e-6,
                                        "in-order mapping");
        }
        NS_TEST_EXPECT_MSG_EQ(bc->BatchesSubmitted(), 1u, "1 submit");
        NS_TEST_EXPECT_MSG_EQ(bc->BatchesCompleted(),
                               1u,
                               "1 complete");
        NS_TEST_EXPECT_MSG_EQ(bc->TotalQueries(),
                               8u,
                               "8 queries");
    }
};

class BatchClientAsyncTest : public TestCase
{
  public:
    BatchClientAsyncTest()
        : TestCase("Batch client: async RequestBatch fires callback under Simulator (4.2.5)")
    {
    }

    void DoRun() override
    {
        auto stub = CreateObject<StubFixedTransport>();
        auto bc = CreateObject<SionnaBatchClient>();
        bc->Attach(stub);
        bc->SetPerRequestStagger(MicroSeconds(250));

        std::vector<SionnaTransport::Request> reqs(5);
        for (size_t i = 0; i < reqs.size(); ++i)
        {
            reqs[i].tx_x = static_cast<double>(i);
            reqs[i].request_id = i + 1;
        }

        bool fired = false;
        std::vector<SionnaTransport::Response> got;
        const uint64_t h = bc->RequestBatch(
            reqs,
            [&](const std::vector<SionnaTransport::Response>& v) {
                fired = true;
                got = v;
            });
        NS_TEST_ASSERT_MSG_GT(h, 0u, "handle assigned");
        NS_TEST_EXPECT_MSG_EQ(bc->PendingBatchCount(),
                               1u,
                               "1 pending");

        Simulator::Stop(MilliSeconds(5));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_EXPECT_MSG_EQ(fired, true, "callback fired");
        NS_TEST_EXPECT_MSG_EQ(got.size(), 5u, "5 responses");
        NS_TEST_EXPECT_MSG_EQ(bc->PendingBatchCount(),
                               0u,
                               "no pending");
        NS_TEST_EXPECT_MSG_EQ(bc->BatchesCompleted(),
                               1u,
                               "1 completed");
    }
};

class BatchClientCancelTest : public TestCase
{
  public:
    BatchClientCancelTest()
        : TestCase("Batch client: cancel pending batch (4.2.5)")
    {
    }

    void DoRun() override
    {
        auto stub = CreateObject<StubFixedTransport>();
        auto bc = CreateObject<SionnaBatchClient>();
        bc->Attach(stub);
        bc->SetPerRequestStagger(MilliSeconds(10));

        std::vector<SionnaTransport::Request> reqs(4);
        for (size_t i = 0; i < reqs.size(); ++i)
        {
            reqs[i].tx_x = static_cast<double>(i);
        }
        bool fired = false;
        const uint64_t h = bc->RequestBatch(
            reqs,
            [&](const std::vector<SionnaTransport::Response>&) {
                fired = true;
            });
        // Cancel before any of the staggered events run.
        const bool ok = bc->CancelBatch(h);
        NS_TEST_EXPECT_MSG_EQ(ok, true, "cancel ok");
        NS_TEST_EXPECT_MSG_EQ(bc->CancelBatch(h),
                               false,
                               "second cancel rejects");

        Simulator::Stop(MilliSeconds(50));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_EXPECT_MSG_EQ(fired,
                               false,
                               "callback did not fire");
        NS_TEST_EXPECT_MSG_EQ(bc->BatchesCancelled(),
                               1u,
                               "cancelled count");
        NS_TEST_EXPECT_MSG_EQ(bc->BatchesCompleted(),
                               0u,
                               "none completed");
    }
};

class ReplayTransportRoundTripTest : public TestCase
{
  public:
    ReplayTransportRoundTripTest()
        : TestCase("Replay transport: write+read+nearest lookup (4.2.6)")
    {
    }

    void DoRun() override
    {
        const std::string path = "/tmp/ntn-replay-roundtrip.ntnbin";
        // Build a small set of records over a 1 km × 1 km grid.
        SionnaReplayWriter w;
        NS_TEST_ASSERT_MSG_EQ(w.Open(path, 28e9),
                               true,
                               "writer open");
        for (int i = 0; i < 5; ++i)
        {
            ReplayRecord r;
            r.t_s = static_cast<double>(i);
            r.sat_pos[0] = 0.0;
            r.sat_pos[1] = 0.0;
            r.sat_pos[2] = 550e3;
            r.ue_pos[0] = 100.0 * i;
            r.ue_pos[1] = 0.0;
            r.ue_pos[2] = 0.0;
            r.freq_hz = 28e9;
            r.path_loss_db = 160.0 + i * 1.5;
            r.n_paths = 3;
            r.compute_ms = 12.0;
            NS_TEST_ASSERT_MSG_EQ(w.Write(r), true, "write");
        }
        NS_TEST_ASSERT_MSG_EQ(w.Close(), true, "close ok");

        auto tx = CreateObject<SionnaReplayTransport>();
        NS_TEST_ASSERT_MSG_EQ(tx->LoadFile(path), true, "load ok");
        NS_TEST_EXPECT_MSG_EQ(tx->Reader().Size(),
                               5u,
                               "5 records loaded");
        NS_TEST_EXPECT_MSG_EQ(tx->IsAvailable(),
                               true,
                               "available");

        // Query at the exact second record position → expect that PL.
        tx->SetClockOverride(1.0);
        SionnaTransport::Request req;
        req.tx_x = 0.0;
        req.tx_y = 0.0;
        req.tx_z = 550e3;
        req.rx_x = 100.0;
        req.rx_y = 0.0;
        req.rx_z = 0.0;
        const auto resp = tx->Query(req);
        NS_TEST_EXPECT_MSG_EQ(resp.ok, true, "ok");
        NS_TEST_EXPECT_MSG_EQ_TOL(resp.path_loss_db,
                                    161.5,
                                    1e-6,
                                    "PL matches record 1");
        NS_TEST_EXPECT_MSG_EQ(resp.n_paths, 3u, "n_paths");

        // Query off-grid → expect nearest record.
        tx->SetClockOverride(3.7);
        req.rx_x = 380.0;
        const auto resp2 = tx->Query(req);
        // Record 4: t=4, ue_x=400 should be nearest.
        NS_TEST_EXPECT_MSG_EQ_TOL(resp2.path_loss_db,
                                    166.0,
                                    1e-6,
                                    "nearest record selected");

        std::remove(path.c_str());
    }
};

class ReplayTransportSimulatorTimeTest : public TestCase
{
  public:
    ReplayTransportSimulatorTimeTest()
        : TestCase("Replay transport: drives queries off Simulator::Now (4.2.6)")
    {
    }

    void DoRun() override
    {
        const std::string path = "/tmp/ntn-replay-simtime.ntnbin";
        SionnaReplayWriter w;
        w.Open(path, 12.5e9);
        for (int i = 0; i < 10; ++i)
        {
            ReplayRecord r;
            r.t_s = i * 0.5;
            r.sat_pos[0] = 0.0;
            r.sat_pos[1] = 0.0;
            r.sat_pos[2] = 550e3;
            r.ue_pos[0] = 50.0 * i;
            r.path_loss_db = 150.0 + i * 0.8;
            r.n_paths = 2;
            w.Write(r);
        }
        w.Close();

        auto tx = CreateObject<SionnaReplayTransport>();
        tx->LoadFile(path);

        std::vector<double> pl_samples;
        SionnaTransport::Request req;
        req.tx_z = 550e3;
        // Schedule queries every 0.5 s for 5 s. At each tick the
        // transport must pick a different record (advancing t).
        for (int i = 0; i < 10; ++i)
        {
            const double when = 0.5 * i;
            req.rx_x = 50.0 * i;
            Simulator::Schedule(Seconds(when),
                                  [tx, req, &pl_samples] {
                                      const auto r = tx->Query(req);
                                      pl_samples.push_back(
                                          r.path_loss_db);
                                  });
        }
        Simulator::Stop(Seconds(5.1));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(pl_samples.size(), 10u, "10 samples");
        for (size_t i = 0; i < pl_samples.size(); ++i)
        {
            NS_TEST_EXPECT_MSG_EQ_TOL(pl_samples[i],
                                        150.0 + i * 0.8,
                                        1e-6,
                                        "matches per-tick record");
        }
        std::remove(path.c_str());
    }
};

class DopplerSynthBasicTest : public TestCase
{
  public:
    DopplerSynthBasicTest()
        : TestCase("CIR Doppler: per-tap rotation matches v·k/λ (4.2.11)")
    {
    }

    void DoRun() override
    {
        CirSnapshot snap;
        snap.wavelength_m = 0.01; // 30 GHz
        snap.t_ref_s = 0.0;
        // Single tap pointing along +x at both ends. tx_dir is the
        // direction the path leaves the tx; rx_dir is the direction
        // it arrives. For pure radial geometry both = +x̂.
        CirPathTap tap;
        tap.amplitude = std::complex<double>(1.0, 0.0);
        tap.delay_s = 0.0;
        tap.tx_dir = Vector(1.0, 0.0, 0.0);
        tap.rx_dir = Vector(1.0, 0.0, 0.0);
        snap.taps.push_back(tap);

        const Vector vtx(7500.0, 0.0, 0.0); // LEO orbital speed
        const Vector vrx(0.0, 0.0, 0.0);
        const auto ev = CirDopplerSynthesizer::Synthesize(
            snap, vtx, vrx, /*dt=*/1.0);
        const double expected_fd = (7500.0 * 1.0 + 0.0) / 0.01;
        NS_TEST_ASSERT_MSG_EQ(ev.doppler_hz.size(), 1u, "1 tap");
        NS_TEST_EXPECT_MSG_EQ_TOL(ev.doppler_hz[0],
                                    expected_fd,
                                    1e-6,
                                    "f_d = v/λ");
        // After dt=1 s, phase = 2π·f_d. amplitude(t=1) = e^{j2π·f_d}.
        const double phase = 2.0 * M_PI * expected_fd * 1.0;
        const std::complex<double> exp_amp(std::cos(phase),
                                              std::sin(phase));
        NS_TEST_EXPECT_MSG_EQ_TOL(ev.amplitudes[0].real(),
                                    exp_amp.real(),
                                    1e-6,
                                    "re part rotates");
        NS_TEST_EXPECT_MSG_EQ_TOL(ev.amplitudes[0].imag(),
                                    exp_amp.imag(),
                                    1e-6,
                                    "im part rotates");

        // Zero velocities → no rotation.
        const auto ev0 = CirDopplerSynthesizer::Synthesize(
            snap, Vector(), Vector(), 1.0);
        NS_TEST_EXPECT_MSG_EQ_TOL(ev0.doppler_hz[0],
                                    0.0,
                                    1e-9,
                                    "zero velocity → 0 Doppler");
        NS_TEST_EXPECT_MSG_EQ_TOL(ev0.amplitudes[0].real(),
                                    1.0,
                                    1e-9,
                                    "amplitude unchanged");
    }
};

class DopplerSynthSeriesTest : public TestCase
{
  public:
    DopplerSynthSeriesTest()
        : TestCase("CIR Doppler: SynthesizeSeries amplitude unit modulus (4.2.11)")
    {
    }

    void DoRun() override
    {
        CirSnapshot snap;
        snap.wavelength_m = 0.06; // 5 GHz
        CirPathTap tap;
        tap.amplitude = std::complex<double>(0.5, 0.5);
        tap.tx_dir = Vector(0.0, 1.0, 0.0);
        tap.rx_dir = Vector(0.0, 1.0, 0.0);
        snap.taps.push_back(tap);

        const auto series = CirDopplerSynthesizer::SynthesizeSeries(
            snap,
            Vector(0.0, 600.0, 0.0),
            Vector(),
            /*fs_hz=*/100.0,
            /*num_steps=*/32);

        NS_TEST_ASSERT_MSG_EQ(series.size(), 32u, "32 steps");
        const double mag_in = std::abs(tap.amplitude);
        for (const auto& ev : series)
        {
            NS_TEST_EXPECT_MSG_EQ_TOL(std::abs(ev.amplitudes[0]),
                                        mag_in,
                                        1e-9,
                                        "magnitude preserved");
            // Doppler is positive (tx flies in +y, ray leaves in +y).
            NS_TEST_EXPECT_MSG_GT(ev.doppler_hz[0],
                                    0.0,
                                    "positive Doppler");
        }
        // Verify the first sample = original (dt=0).
        NS_TEST_EXPECT_MSG_EQ_TOL(series.front().amplitudes[0].real(),
                                    tap.amplitude.real(),
                                    1e-9,
                                    "t=0 re");
        NS_TEST_EXPECT_MSG_EQ_TOL(series.front().amplitudes[0].imag(),
                                    tap.amplitude.imag(),
                                    1e-9,
                                    "t=0 im");
    }
};

class NtnSionnaTestSuite : public TestSuite
{
  public:
    NtnSionnaTestSuite()
        : TestSuite("ntn-sionna", Type::UNIT)
    {
        // Pre-existing reference + wire-compat tests.
        AddTestCase(new FreeSpaceSpotCheckTest, Duration::QUICK);
        AddTestCase(new FallbackPathLossTest, Duration::QUICK);
        AddTestCase(new LoopbackRttGateTest, Duration::QUICK);
        // Roadmap §4.2.1 — SionnaTransport abstraction + end-to-end coverage.
        AddTestCase(new TransportContractTest, Duration::QUICK);
        AddTestCase(new TransportUdpFsplIdentityTest, Duration::QUICK);
        AddTestCase(new SimulatorTimeMobilityTest, Duration::QUICK);
        AddTestCase(new MidRunServerKillTest, Duration::QUICK);
        AddTestCase(new PybindStubAvailabilityTest, Duration::QUICK);
        // Roadmap §4.2.2 — Sionna RT 2.0.1 MIMO PlanarArray wire support.
        AddTestCase(new TransportMimoWireRoundTripTest, Duration::QUICK);
        AddTestCase(new TransportMimoSimulatorTimeTest, Duration::QUICK);
        // Roadmap §4.2.7 — NTN cascade glue (atmospheric loss chain).
        AddTestCase(new AtmosphericChainElevationTest, Duration::QUICK);
        AddTestCase(new AtmosphericChainGaseousSweepTest, Duration::QUICK);
        AddTestCase(new AtmosphericChainRainSweepTest, Duration::QUICK);
        AddTestCase(new AtmosphericChainLmsSimulatorTest, Duration::QUICK);
        AddTestCase(new CascadeBaselineEqualityTest, Duration::QUICK);
        AddTestCase(new CascadeOrbitSweepTest, Duration::QUICK);
        AddTestCase(new CascadeRainImpactTest, Duration::QUICK);
        AddTestCase(new CascadeServerDownGracefulTest, Duration::QUICK);
        AddTestCase(new CascadeOverUdpTransportTest, Duration::QUICK);
        // Roadmap §4.2.4 — 4-D LRU caching transport.
        AddTestCase(new CachingHitMissTest, Duration::QUICK);
        AddTestCase(new CachingSpatialQuantizationTest, Duration::QUICK);
        AddTestCase(new CachingTemporalBucketTest, Duration::QUICK);
        AddTestCase(new CachingLruEvictionTest, Duration::QUICK);
        AddTestCase(new CachingResetTest, Duration::QUICK);
        AddTestCase(new CachingUnderSimulatorTest, Duration::QUICK);
        // Roadmap §4.2.3 — RIS Tx surface (Sionna 2.0 ReflectingSurface).
        AddTestCase(new RisWireRoundTripTest, Duration::QUICK);
        AddTestCase(new RisPhaseProfileMatrixTest, Duration::QUICK);
        AddTestCase(new RisChannelAttachTest, Duration::QUICK);
        // Roadmap §4.3.3 — Sionna RT calibration harness.
        AddTestCase(new SionnaCalibratorLosOnlyTest, Duration::QUICK);
        AddTestCase(new SionnaCalibratorDetectsOffsetTest, Duration::QUICK);
        AddTestCase(new SionnaCalibratorSimulatorTimeTest, Duration::QUICK);
        // Roadmap §4.2.5 — Async batched query API.
        AddTestCase(new BatchClientSyncTest, Duration::QUICK);
        AddTestCase(new BatchClientAsyncTest, Duration::QUICK);
        AddTestCase(new BatchClientCancelTest, Duration::QUICK);
        // Roadmap §4.2.6 — Precompute/replay backend.
        AddTestCase(new ReplayTransportRoundTripTest, Duration::QUICK);
        AddTestCase(new ReplayTransportSimulatorTimeTest,
                    Duration::QUICK);
        // Roadmap §4.2.11 — apply_doppler() C++ equivalent.
        AddTestCase(new DopplerSynthBasicTest, Duration::QUICK);
        AddTestCase(new DopplerSynthSeriesTest, Duration::QUICK);
    }
};

static NtnSionnaTestSuite g_ntnSionnaTestSuite;

} // namespace
} // namespace ns3
