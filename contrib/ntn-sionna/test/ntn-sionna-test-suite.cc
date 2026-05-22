/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit / Roadmap §4.2.1)
 */
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include "ns3/ns3-sionna-channel.h"
#include "ns3/sionna-pybind-transport.h"
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
/// This is what makes the C++ tests truly end-to-end — request goes out on a
/// real socket, gets parsed, response comes back, matched against the same
/// FSPL identity the toolkit's fall-back uses.
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

    void Loop()
    {
        char buf[2048];
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

            char rsp[256];
            int rlen = std::snprintf(rsp,
                                     sizeof(rsp),
                                     "{\"id\":1,\"path_loss_db\":%.6f,"
                                     "\"n_paths\":1,\"compute_ms\":0.05}",
                                     pl);
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
    }
};

static NtnSionnaTestSuite g_ntnSionnaTestSuite;

} // namespace
} // namespace ns3
