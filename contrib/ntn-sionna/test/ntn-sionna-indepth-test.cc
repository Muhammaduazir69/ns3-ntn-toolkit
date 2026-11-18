// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// In-depth ntn-sionna validation (18 Nov 2026 test campaign).
//
// No Python server is needed: the ray-traced answers come from the replay
// transport (a recorded Sionna trace) or from a scripted transport defined
// here, so every case is deterministic and runs inside Simulator::Run().
// Packet-level cases put the Sionna channel on a real P2P link through the
// thz-ntn error model and measure delivery with FlowMonitor. The atmosphere
// cases check ITU-R relations:
//   - P.676 Annex 2: slant gaseous attenuation A(el) = A_zenith / sin(el)
//     above 10 deg; the 22.235 GHz water-vapour line and the 60 GHz oxygen
//     complex;
//   - P.838-3: k_H > k_V, so horizontal polarization fades more than vertical,
//     circular in between;
//   - P.618: elevation is measured from the WGS-84 local horizontal.
// FSPL references follow TR 38.811 eq. 6.6-2.

#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ns3-sionna-channel.h"
#include "ns3/ntn-atmospheric-loss-chain.h"
#include "ns3/ntn-atmospheric-propagation-loss-model.h"
#include "ns3/ntn-geocentric-mobility-adapter.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/pointer.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/sionna-caching-transport.h"
#include "ns3/sionna-replay-transport.h"
#include "ns3/sionna-transport.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/thz-ntn-link-error-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <vector>

using namespace ns3;

namespace
{

/// The channel's own free-space fallback; checked against TR 38.811
/// eq. 6.6-2 (32.45 + 20 log10 f[GHz] + 20 log10 d[m]) in SioReplayWrongBandTest.
double
Fspl(double d, double f)
{
    return NtnSionnaChannel::FreeSpacePathLossDb(d, f);
}

/// Write a replay trace: constant geometry, the path loss given per time.
std::string
WriteTrace(const std::string& path, double freqHz, double dt, double span,
           const std::function<double(double)>& pl)
{
    SionnaReplayWriter w;
    w.Open(path, static_cast<uint64_t>(freqHz));
    for (double t = 0.0; t <= span + 1e-9; t += dt)
    {
        ReplayRecord r;
        r.t_s = t;
        r.sat_pos[2] = 550e3;
        r.freq_hz = freqHz;
        r.path_loss_db = pl(t);
        r.n_paths = 3;
        w.Write(r);
    }
    w.Close();
    return path;
}

/// A P2P space-to-ground link whose receive error model evaluates the given
/// propagation chain per packet (thz-ntn), carrying a 100 packet/s UDP flow.
struct PacketLink
{
    Ptr<thzntn::ThzNtnLinkErrorModel> em;
    Ptr<FlowMonitor> fm;
    FlowMonitorHelper fmh;
    std::vector<double> rxTimes;

    static void OnRx(PacketLink* l, Ptr<const Packet>, const Address&)
    {
        l->rxTimes.push_back(Simulator::Now().GetSeconds());
    }

    void Build(Ptr<PropagationLossModel> chain, double stopS)
    {
        NodeContainer n;
        n.Create(2);
        Ptr<ConstantPositionMobilityModel> sat = CreateObject<ConstantPositionMobilityModel>();
        sat->SetPosition(Vector(0, 0, 550e3));
        Ptr<ConstantPositionMobilityModel> gnd = CreateObject<ConstantPositionMobilityModel>();
        gnd->SetPosition(Vector(0, 0, 0));
        n.Get(0)->AggregateObject(sat);
        n.Get(1)->AggregateObject(gnd);
        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("100Mbps"));
        p2p.SetChannelAttribute("Delay", TimeValue(Seconds(550e3 / 299792458.0)));
        NetDeviceContainer d = p2p.Install(n);
        em = CreateObject<thzntn::ThzNtnLinkErrorModel>();
        em->SetEndpoints(sat, gnd);
        em->SetPropagationChain(chain);
        em->SetTxPowerDbm(30.0);
        em->SetAntennaGainsDb(40.0, 45.0);
        em->SetBandwidthHz(1e9);
        em->SetNoiseFigureDb(8.0);
        em->SetDecodeSnrDb(0.0);
        d.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(em));
        InternetStackHelper inet;
        inet.Install(n);
        Ipv4AddressHelper ip;
        ip.SetBase("10.66.0.0", "255.255.255.252");
        Ipv4InterfaceContainer ifs = ip.Assign(d);
        OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(ifs.GetAddress(1), 9));
        onoff.SetConstantRate(DataRate("400kbps"), 500);
        ApplicationContainer src = onoff.Install(n.Get(0));
        src.Start(Seconds(0.0));
        src.Stop(Seconds(stopS));
        PacketSinkHelper ps("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 9));
        ApplicationContainer snk = ps.Install(n.Get(1));
        snk.Get(0)->TraceConnectWithoutContext("Rx", MakeBoundCallback(&OnRx, this));
        fm = fmh.InstallAll();
    }

    void Totals(uint64_t& tx, uint64_t& rx) const
    {
        tx = 0;
        rx = 0;
        for (const auto& [id, st] : fm->GetFlowStats())
        {
            tx += st.txPackets;
            rx += st.rxPackets;
        }
    }
};

/// A scripted ray tracer: every 3rd query fails (timeout), every 5th answers
/// with a negative path loss (a broken reply), the rest answer FSPL + 12 dB.
class ScriptedTransport : public SionnaTransport
{
  public:
    Response Query(const Request& req) const override
    {
        const uint64_t k = ++m_queriesSent;
        Response r{};
        if (k % 3 == 0)
        {
            ++m_timeouts;
            r.ok = false;
            r.path_loss_db = std::numeric_limits<double>::infinity();
            return r;
        }
        r.ok = true;
        r.n_paths = 4;
        if (k % 5 == 0)
        {
            r.path_loss_db = -20.0;
            return r;
        }
        const double d = std::sqrt(std::pow(req.tx_x - req.rx_x, 2) + std::pow(req.tx_y - req.rx_y, 2) +
                                   std::pow(req.tx_z - req.rx_z, 2));
        r.path_loss_db = Fspl(d, req.freq_hz) + 12.0;
        return r;
    }

    std::string Name() const override
    {
        return "scripted";
    }

    bool IsAvailable() const override
    {
        return true;
    }
};

} // namespace

// ---------------------------------------------------------------------------
/// A recorded Sionna trace drives real packets: the trace holds 150 dB except
/// a 1 s, 200 dB blockage. On the thz-ntn link (30 dBm, 40 + 45 dBi, 1 GHz,
/// NF 8 dB: SNR 41 dB clear, -9 dB blocked) exactly the packets that arrive
/// inside the blockage are lost, and every evaluation is ray traced.
class SioReplayFadeOnPacketLinkTest : public TestCase
{
  public:
    SioReplayFadeOnPacketLinkTest()
        : TestCase("A replayed Sionna blockage removes exactly its packets from a real link")
    {
    }

  private:
    void DoRun() override
    {
        const std::string path = WriteTrace(CreateTempDirFilename("fade.trace"), 28e9, 0.05, 5.0,
                                            [](double t) { return (t >= 2.0 && t < 3.0) ? 200.0 : 150.0; });
        Ptr<SionnaReplayTransport> rp = CreateObject<SionnaReplayTransport>();
        NS_TEST_ASSERT_MSG_EQ(rp->LoadFile(path), true, "trace loads");
        Ptr<NtnSionnaChannel> ch = CreateObject<NtnSionnaChannel>();
        ch->SetTransport(rp);
        ch->SetFrequencyHz(28e9);
        PacketLink link;
        link.Build(ch, 5.0);
        Simulator::Stop(Seconds(5.5));
        Simulator::Run();
        uint64_t tx;
        uint64_t rx;
        link.Totals(tx, rx);
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ_TOL(double(tx), 500.0, 1.0, "100 packets/s for 5 s");
        uint32_t inFade = 0;
        for (double t : link.rxTimes)
        {
            // nearest record: arrivals within 25 ms of the edges may go either way
            if (t > 2.03 && t < 2.97)
            {
                ++inFade;
            }
        }
        NS_TEST_ASSERT_MSG_EQ(inFade, 0u, "nothing is delivered inside the blockage");
        NS_TEST_ASSERT_MSG_EQ_TOL(double(tx - rx), 100.0, 6.0, "the blockage costs one second of packets");
        NS_TEST_ASSERT_MSG_EQ(link.em->GetPacketsCorrupted(), tx - rx, "losses are SNR decode failures");
        NS_TEST_ASSERT_MSG_EQ(ch->GetFallbacks(), 0u, "every evaluation answered by the trace");
        NS_TEST_ASSERT_MSG_EQ(ch->GetRayTraced(), rp->GetQueriesSent(), "ray-traced count is exact");
        NS_TEST_ASSERT_MSG_EQ(ch->AllQueriesRayTraced(), true, "provenance: fully ray traced");
        NS_TEST_ASSERT_MSG_GT(link.em->GetLastSnrDb(), 40.0, "clear-sky SNR is 41 dB");
    }
};

// ---------------------------------------------------------------------------
/// Failure path: a trace recorded at 2 GHz must not answer a 28 GHz link. The
/// channel falls back to TR 38.811 FSPL at 28 GHz for every query and says
/// so; the same trace at its own carrier is used as recorded.
class SioReplayWrongBandTest : public TestCase
{
  public:
    SioReplayWrongBandTest()
        : TestCase("A replay trace from another band is refused and the channel falls back to FSPL")
    {
    }

  private:
    void DoRun() override
    {
        const std::string path =
            WriteTrace(CreateTempDirFilename("sband.trace"), 2e9, 0.1, 2.0, [](double) { return 160.0; });
        Ptr<ConstantPositionMobilityModel> a = CreateObject<ConstantPositionMobilityModel>();
        a->SetPosition(Vector(0, 0, 550e3));
        Ptr<ConstantPositionMobilityModel> b = CreateObject<ConstantPositionMobilityModel>();
        for (double f : {28e9, 2e9})
        {
            Ptr<SionnaReplayTransport> rp = CreateObject<SionnaReplayTransport>();
            rp->LoadFile(path);
            Ptr<NtnSionnaChannel> ch = CreateObject<NtnSionnaChannel>();
            ch->SetTransport(rp);
            ch->SetFrequencyHz(f);
            std::vector<double> loss;
            for (int k = 0; k < 20; ++k)
            {
                Simulator::Schedule(MilliSeconds(100 * k), [&loss, ch, a, b]() {
                    loss.push_back(30.0 - ch->CalcRxPower(30.0, a, b));
                });
            }
            Simulator::Run();
            Simulator::Destroy();
            if (f == 28e9)
            {
                NS_TEST_ASSERT_MSG_EQ(ch->GetFallbacks(), 20u, "every query fell back");
                NS_TEST_ASSERT_MSG_EQ(rp->GetFailures(), 20u, "the transport refused them");
                NS_TEST_ASSERT_MSG_EQ(ch->AllQueriesRayTraced(), false, "provenance says so");
                NS_TEST_ASSERT_MSG_EQ_TOL(Fspl(550e3, 28e9),
                                          32.45 + 20.0 * std::log10(28.0) + 20.0 * std::log10(550e3),
                                          0.01, "fallback is TR 38.811 eq. 6.6-2");
                for (double l : loss)
                {
                    NS_TEST_ASSERT_MSG_EQ_TOL(l, Fspl(550e3, 28e9), 1e-9, "FSPL at 28 GHz, 550 km");
                }
            }
            else
            {
                NS_TEST_ASSERT_MSG_EQ(ch->GetFallbacks(), 0u, "matching band: no fallback");
                NS_TEST_ASSERT_MSG_EQ_TOL(loss.back(), 160.0, 1e-9, "the recorded value");
            }
        }
    }
};

// ---------------------------------------------------------------------------
/// Accounting under a misbehaving tracer while a UE drives away: timeouts and
/// negative path losses (an amplifying "channel") both fall back to FSPL,
/// every other answer is used as given, and the counters add up exactly.
class SioScriptedTransportAccountingTest : public TestCase
{
  public:
    SioScriptedTransportAccountingTest()
        : TestCase("Timeouts and negative path losses fall back to FSPL; counters add up exactly")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<ScriptedTransport> tr = CreateObject<ScriptedTransport>();
        Ptr<NtnSionnaChannel> ch = CreateObject<NtnSionnaChannel>();
        ch->SetTransport(tr);
        ch->SetFrequencyHz(3.5e9);
        Ptr<ConstantPositionMobilityModel> gnb = CreateObject<ConstantPositionMobilityModel>();
        gnb->SetPosition(Vector(0, 0, 25));
        Ptr<ConstantVelocityMobilityModel> ue = CreateObject<ConstantVelocityMobilityModel>();
        ue->SetPosition(Vector(50, 0, 1.5));
        ue->SetVelocity(Vector(20, 0, 0));
        uint32_t bad = 0;
        uint32_t good = 0;
        uint32_t wrong = 0;
        for (int k = 1; k <= 300; ++k)
        {
            Simulator::Schedule(MilliSeconds(10 * k), [&, k]() {
                const double l = 30.0 - ch->CalcRxPower(30.0, gnb, ue);
                const double f = Fspl(CalculateDistance(gnb->GetPosition(), ue->GetPosition()), 3.5e9);
                const bool fails = (k % 3 == 0) || (k % 5 == 0);
                bad += fails;
                good += !fails;
                wrong += std::abs(l - (fails ? f : f + 12.0)) > 1e-9;
            });
        }
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_EQ(wrong, 0u, "each answer is either the tracer's or FSPL, never a gain");
        NS_TEST_ASSERT_MSG_EQ(ch->GetFallbacks(), bad, "fallbacks = timeouts + negative replies");
        NS_TEST_ASSERT_MSG_EQ(ch->GetRayTraced(), good, "ray traced = usable replies");
        NS_TEST_ASSERT_MSG_EQ(bad, 140u, "100 timeouts + 40 negative replies in 300");
        NS_TEST_ASSERT_MSG_EQ(tr->GetTimeouts(), 100u, "transport-side timeout count");
    }
};

// ---------------------------------------------------------------------------
/// Cross-module (ntn-constellation): the atmospheric chain over a live SGP4
/// pass seen from 45 deg N. The chain's elevation equals the propagator's
/// WGS-84 elevation, and gaseous attenuation follows the P.676 Annex 2
/// cosecant law A(el) sin(el) = A_zenith above 10 deg elevation.
class SioCosecantLawSgp4PassTest : public TestCase
{
  public:
    SioCosecantLawSgp4PassTest()
        : TestCase("Gaseous slant attenuation over an SGP4 pass follows P.676 cosecant scaling")
    {
    }

  private:
    void DoRun() override
    {
        ntncon::WalkerConfig w;
        w.total_sats = 10;
        w.num_planes = 1;
        w.altitude_km = 550.0;
        w.inclination_deg = 53.0;
        w.epoch_unix_s = 1767225600.0;
        auto el = ntncon::WalkerConstellation::BuildDelta(w);
        Ptr<ntncon::Sgp4MobilityModel> sat = CreateObject<ntncon::Sgp4MobilityModel>();
        sat->SetElementsSgp4(el[0]);
        // find when the satellite is near 45 N and put the station under it
        double lat;
        double lon;
        double alt;
        double tCross = -1;
        for (int t = 0; t < 6000 && tCross < 0; t += 5)
        {
            Simulator::Schedule(Seconds(t), [&, t]() {
                if (tCross >= 0)
                {
                    return;
                }
                sat->GetGeodetic(lat, lon, alt);
                if (lat > 44.5 && lat < 46.0)
                {
                    tCross = t;
                }
            });
        }
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_GT_OR_EQ(tCross, 0.0, "the 53 deg orbit reaches 45 N");
        const double gsLat = lat;
        const double gsLon = lon + 2.0; // a little off-track: not exactly zenith
        const Vector gs = ntncon::NtnGeocentricMobilityAdapter::GeodeticToEcef(Vector(gsLat, gsLon, 0.0));

        Ptr<NtnAtmosphericLossChain> chain = CreateObject<NtnAtmosphericLossChain>();
        chain->SetFrequencyHz(20e9);
        chain->SetEnableRain(false);
        chain->SetGroundAltKm(0.0);
        double zenithA = 0;
        Simulator::Schedule(Seconds(0), [&]() {
            zenithA = chain->ComputeAttenuationDb(gs, gs * (1.0 + 600e3 / gs.GetLength()));
        });
        struct S
        {
            double el;
            double elRef;
            double a;
        };

        std::vector<S> samples;
        for (int t = std::max(0, static_cast<int>(tCross) - 400); t <= static_cast<int>(tCross) + 400; t += 10)
        {
            Simulator::Schedule(Seconds(t), [&]() {
                const double a = chain->ComputeAttenuationDb(gs, sat->GetPosition());
                samples.push_back({chain->GetLastComponents().elevationDeg,
                                   sat->GetElevationDeg(gsLat, gsLon), a});
            });
        }
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_GT(zenithA, 0.1, "20 GHz zenith gas loss is a few tenths of a dB");
        NS_TEST_ASSERT_MSG_LT(zenithA, 1.0, "and below 1 dB at sea level");
        uint32_t used = 0;
        for (const auto& s : samples)
        {
            NS_TEST_ASSERT_MSG_EQ_TOL(s.el, s.elRef, 0.02, "WGS-84 elevation, not geocentric");
            if (s.el > 10.0)
            {
                ++used;
                NS_TEST_ASSERT_MSG_EQ_TOL(s.a * std::sin(s.el * M_PI / 180.0), zenithA, 0.05 * zenithA,
                                          "P.676 cosecant law at el = " << s.el);
            }
            else if (s.el <= 0.0)
            {
                NS_TEST_ASSERT_MSG_GT_OR_EQ(s.a, 100.0, "below the horizon the link is blocked");
            }
        }
        NS_TEST_ASSERT_MSG_GT(used, 10u, "the pass gives samples above 10 deg");
    }
};

// ---------------------------------------------------------------------------
/// P.676 spectral shape, evaluated during a run: the 22.235 GHz water-vapour
/// line stands above 18 and 26 GHz, the 60 GHz oxygen complex is more than
/// 100x the 30 GHz window, and a 2 km high station sees less water vapour.
class SioGasSpectralShapeTest : public TestCase
{
  public:
    SioGasSpectralShapeTest()
        : TestCase("P.676 zenith attenuation shows the 22 GHz H2O line and the 60 GHz O2 complex")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<NtnAtmosphericLossChain> c = CreateObject<NtnAtmosphericLossChain>();
        c->SetEnableRain(false);
        std::map<double, double> a;
        double high22 = 0;
        const Vector g(0, 0, 0);
        const Vector s(0, 0, 600e3);
        int k = 0;
        for (double f : {18e9, 22.235e9, 26e9, 30e9, 60e9})
        {
            Simulator::Schedule(Seconds(++k), [&a, c, f, g, s]() {
                c->SetFrequencyHz(f);
                c->SetGroundAltKm(0.0);
                a[f] = c->ComputeAttenuationDb(g, s);
            });
        }
        Simulator::Schedule(Seconds(10), [&high22, c, g, s]() {
            c->SetFrequencyHz(22.235e9);
            c->SetGroundAltKm(2.0);
            high22 = c->ComputeAttenuationDb(g, s);
        });
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_GT(a[22.235e9], a[18e9], "H2O line above 18 GHz");
        NS_TEST_ASSERT_MSG_GT(a[22.235e9], a[26e9], "H2O line above 26 GHz");
        NS_TEST_ASSERT_MSG_GT(a[60e9], 100.0 * a[30e9], "O2 complex dwarfs the 30 GHz window");
        NS_TEST_ASSERT_MSG_LT(high22, 0.7 * a[22.235e9], "2 km of water vapour removed (scale height 2 km)");
    }
};

// ---------------------------------------------------------------------------
/// P.838-3 / P.618 rain on a packet link (thz-ntn error model behind Friis +
/// the atmospheric excess): 20 GHz, 30 deg elevation geometry. Clear sky
/// delivers everything; at 50 mm/h horizontal polarization fades more than
/// circular, circular more than vertical (k_H > k_V); a 150 mm/h storm with
/// a link designed for 20 dB margin drops the link (failure path).
class SioRainPolarizationPacketTest : public TestCase
{
  public:
    SioRainPolarizationPacketTest()
        : TestCase("Rain fade by polarization (P.838-3) and a storm outage on a real packet link")
    {
    }

  private:
    void DoRun() override
    {
        // 30 deg elevation, 1000 km slant in the local frame
        const Vector gnd(0, 0, 0);
        const Vector sat(1000e3 * std::cos(M_PI / 6), 0, 1000e3 * std::sin(M_PI / 6));
        double aH = 0;
        double aV = 0;
        double aC = 0;
        Ptr<NtnAtmosphericLossChain> c = CreateObject<NtnAtmosphericLossChain>();
        c->SetFrequencyHz(20e9);
        c->SetEnableGaseous(false);
        c->SetRainRateMmH(50.0);
        Simulator::Schedule(Seconds(1), [&]() {
            c->SetPolarizationInt(0);
            aH = c->ComputeAttenuationDb(gnd, sat);
            c->SetPolarizationInt(1);
            aV = c->ComputeAttenuationDb(gnd, sat);
            c->SetPolarizationInt(2);
            aC = c->ComputeAttenuationDb(gnd, sat);
        });
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_GT(aV, 3.0, "50 mm/h at 20 GHz is a multi-dB fade");
        NS_TEST_ASSERT_MSG_GT(aH, aV, "k_H > k_V: horizontal fades more");
        NS_TEST_ASSERT_MSG_GT(aC, aV, "circular above vertical");
        NS_TEST_ASSERT_MSG_LT(aC, aH, "circular below horizontal");

        for (double rain : {0.0, 150.0})
        {
            Ptr<NtnAtmosphericLossChain> ch = CreateObject<NtnAtmosphericLossChain>();
            ch->SetFrequencyHz(20e9);
            ch->SetRainRateMmH(rain);
            Ptr<NtnAtmosphericPropagationLossModel> atm = CreateObject<NtnAtmosphericPropagationLossModel>();
            atm->SetChain(ch);
            Ptr<FriisPropagationLossModel> friis = CreateObject<FriisPropagationLossModel>();
            friis->SetFrequency(20e9);
            friis->SetNext(atm);
            // Friis 550 km zenith at 20 GHz: 173.3 dB; designed so clear sky
            // has a 20 dB margin: 30 + 40 + 45 - 173.3 + 76 = 17.7 dB SNR
            PacketLink link;
            link.Build(friis, 2.0);
            Simulator::Stop(Seconds(2.5));
            Simulator::Run();
            uint64_t tx;
            uint64_t rx;
            link.Totals(tx, rx);
            const double snr = link.em->GetLastSnrDb();
            Simulator::Destroy();
            if (rain == 0.0)
            {
                NS_TEST_ASSERT_MSG_EQ(rx, tx, "clear sky delivers every packet");
                NS_TEST_ASSERT_MSG_GT(snr, 15.0, "clear-sky SNR near 17 dB");
            }
            else
            {
                NS_TEST_ASSERT_MSG_EQ(rx, 0u, "a 150 mm/h storm at zenith on 20 GHz drops the link");
                NS_TEST_ASSERT_MSG_LT(snr, 0.0, "SNR below the decode threshold");
            }
        }
    }
};

// ---------------------------------------------------------------------------
/// P.681 land-mobile-satellite shadowing inside a run: the same stream gives
/// the same fade series, a different stream a different one, and both stay
/// physical (non-negative, bounded) with time in both Markov states.
class SioLmsDeterminismTest : public TestCase
{
  public:
    SioLmsDeterminismTest()
        : TestCase("P.681 LMS fades are reproducible per stream and stay physically bounded")
    {
    }

  private:
    std::vector<double> Series(int64_t stream)
    {
        Ptr<NtnAtmosphericLossChain> c = CreateObject<NtnAtmosphericLossChain>();
        c->SetEnableGaseous(false);
        c->SetEnableLms(true);
        c->SetLmsEnvironmentInt(0); // urban: frequent shadowing
        c->AssignStreams(stream);
        std::vector<double> v;
        for (int k = 0; k < 600; ++k)
        {
            Simulator::Schedule(MilliSeconds(100 * k), [&v, c]() { v.push_back(c->StepLmsDb()); });
        }
        Simulator::Run();
        Simulator::Destroy();
        return v;
    }

    void DoRun() override
    {
        auto a = Series(11);
        auto b = Series(11);
        auto c = Series(12);
        NS_TEST_ASSERT_MSG_EQ((a == b), true, "same stream, same series");
        NS_TEST_ASSERT_MSG_EQ((a == c), false, "another stream, another series");
        for (const auto* s : {&a, &c})
        {
            uint32_t faded = 0;
            for (double x : *s)
            {
                NS_TEST_ASSERT_MSG_EQ(std::isfinite(x), true, "finite fade");
                NS_TEST_ASSERT_MSG_GT_OR_EQ(x, -10.0, "no large gain from shadowing");
                NS_TEST_ASSERT_MSG_LT_OR_EQ(x, 60.0, "bounded fade (the P.681 chain caps at 60 dB)");
                faded += x > 3.0;
            }
            NS_TEST_ASSERT_MSG_GT(faded, 0u, "urban LMS visits the shadowed state");
            NS_TEST_ASSERT_MSG_LT(faded, s->size(), "and the line-of-sight state");
        }
    }
};

// ---------------------------------------------------------------------------
/// The caching transport over a recorded trace inside a run: with a 1 s time
/// bucket and 100 ms sampling, 9 of 10 queries hit and return the cached
/// answer; a one-entry cache shared by two alternating UEs never hits
/// (thrashing failure path) yet still answers correctly.
class SioCachingOverReplayTest : public TestCase
{
  public:
    SioCachingOverReplayTest()
        : TestCase("Caching transport over a replay trace: exact hit rate and correct answers")
    {
    }

  private:
    void DoRun() override
    {
        const std::string path = WriteTrace(CreateTempDirFilename("cache.trace"), 12e9, 0.1, 20.0,
                                            [](double t) { return 160.0 + t; });
        for (uint32_t maxEntries : {1024u, 1u})
        {
            Ptr<SionnaReplayTransport> rp = CreateObject<SionnaReplayTransport>();
            rp->LoadFile(path);
            Ptr<SionnaCachingTransport> cache = CreateObject<SionnaCachingTransport>();
            cache->SetInner(rp);
            cache->SetTemporalBucketUs(1000000);
            cache->SetSpatialResolutionM(10.0);
            cache->SetMaxEntries(maxEntries);
            Ptr<NtnSionnaChannel> ch = CreateObject<NtnSionnaChannel>();
            ch->SetTransport(cache);
            ch->SetFrequencyHz(12e9);
            Ptr<ConstantPositionMobilityModel> sat = CreateObject<ConstantPositionMobilityModel>();
            sat->SetPosition(Vector(0, 0, 550e3));
            Ptr<ConstantPositionMobilityModel> ue1 = CreateObject<ConstantPositionMobilityModel>();
            Ptr<ConstantPositionMobilityModel> ue2 = CreateObject<ConstantPositionMobilityModel>();
            ue2->SetPosition(Vector(500, 0, 0));
            uint32_t bad = 0;
            for (int k = 0; k < 100; ++k)
            {
                Simulator::Schedule(MilliSeconds(100 * k + 1), [&, k]() {
                    Ptr<MobilityModel> ue = (maxEntries == 1 && k % 2) ? Ptr<MobilityModel>(ue2) : Ptr<MobilityModel>(ue1);
                    const double l = 30.0 - ch->CalcRxPower(30.0, sat, ue);
                    // the cached answer is the one traced at the bucket start
                    const double bucket = std::floor(Simulator::Now().GetSeconds());
                    bad += std::abs(l - (160.0 + bucket)) > 0.051;
                });
            }
            Simulator::Run();
            Simulator::Destroy();
            if (maxEntries > 1)
            {
                NS_TEST_ASSERT_MSG_EQ(cache->GetMisses(), 10u, "one miss per 1 s bucket");
                NS_TEST_ASSERT_MSG_EQ(cache->GetHits(), 90u, "nine hits per bucket");
                NS_TEST_ASSERT_MSG_EQ(rp->GetQueriesSent(), 10u, "the tracer is asked 10 times only");
                NS_TEST_ASSERT_MSG_EQ(bad, 0u, "every answer is the bucket's traced value");
            }
            else
            {
                NS_TEST_ASSERT_MSG_EQ(cache->GetHits(), 0u, "two UEs thrash a one-entry cache");
                NS_TEST_ASSERT_MSG_EQ(rp->GetQueriesSent(), 100u, "every query reaches the tracer");
                NS_TEST_ASSERT_MSG_GT_OR_EQ(cache->GetEvictions(), 99u, "and evicts each time");
            }
        }
    }
};

class NtnSionnaInDepthTestSuite : public TestSuite
{
  public:
    NtnSionnaInDepthTestSuite()
        : TestSuite("ntn-sionna-indepth", Type::UNIT)
    {
        AddTestCase(new SioReplayFadeOnPacketLinkTest, Duration::QUICK);
        AddTestCase(new SioReplayWrongBandTest, Duration::QUICK);
        AddTestCase(new SioScriptedTransportAccountingTest, Duration::QUICK);
        AddTestCase(new SioCosecantLawSgp4PassTest, Duration::QUICK);
        AddTestCase(new SioGasSpectralShapeTest, Duration::QUICK);
        AddTestCase(new SioRainPolarizationPacketTest, Duration::QUICK);
        AddTestCase(new SioLmsDeterminismTest, Duration::QUICK);
        AddTestCase(new SioCachingOverReplayTest, Duration::QUICK);
    }
};

static NtnSionnaInDepthTestSuite g_ntnSionnaInDepthTestSuite;
