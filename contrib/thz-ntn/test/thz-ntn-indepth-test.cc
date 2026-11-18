// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// In-depth thz-ntn validation (18 Nov 2026 test campaign).
//
// References asserted here:
//   - ETSI EN 302 307-1 Table 13: DVB-S2 spectral efficiency and ideal Es/N0
//     per MODCOD; ACM picks the most efficient MODCOD that closes;
//   - pointing loss L = 12 (theta / theta_3dB)^2 dB (ITU-R S.1528 parabolic
//     main lobe) and the Gaussian-beam law 10 log10(e) 4 ln2 = 12.04;
//   - Rayleigh-distributed radial pointing error, E[theta] = sigma sqrt(pi/2);
//   - PA distortion that shrinks with input back-off;
//   - hardware-limited SNR = SNR / (1 + SNR kappa^2) (Schenk 2008);
//   - TS 38.321 clause 5.17 beam-failure detection on consecutive low-SINR
//     instances, then recovery.
// The cross-module case flies the THz packet link over an SGP4 pass from
// ntn-constellation in the Earth-fixed frame, measured with FlowMonitor.

#include "ns3/applications-module.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/double.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ntn-geocentric-mobility-adapter.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/test.h"
#include "ns3/thz-ntn-beam-tracking.h"
#include "ns3/thz-ntn-hardware-impairments.h"
#include "ns3/thz-ntn-isl-link.h"
#include "ns3/thz-ntn-link-error-model.h"
#include "ns3/thz-ntn-mac.h"
#include "ns3/thz-ntn-molecular-absorption.h"
#include "ns3/thz-ntn-packet-link.h"
#include "ns3/thz-ntn-pointing-error.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <map>
#include <vector>

using namespace ns3;
using namespace ns3::thzntn;

// ---------------------------------------------------------------------------
/// DVB-S2 MODCOD table and ACM selection. Spectral efficiencies match ETSI
/// EN 302 307-1 Table 13 (normal frames, no pilots) to 1e-3; required Es/N0
/// rises with efficiency and sits within 1.5 dB of the table's ideal value;
/// ACM picks the most efficient MODCOD whose requirement plus margin is met,
/// and refuses an Es/N0 below the lowest MODCOD (failure path).
class ThzDvbS2TableAcmTest : public TestCase
{
  public:
    ThzDvbS2TableAcmTest()
        : TestCase("DVB-S2 MODCOD table (EN 302 307-1 Table 13) and ACM selection")
    {
    }

  private:
    void DoRun() override
    {
        struct Row
        {
            SatEnums::SatModcod_t m;
            double eff;
            double esno;
        };

        const std::vector<Row> table = {
            {SatEnums::SAT_MODCOD_QPSK_1_TO_2, 0.988858, 1.00},
            {SatEnums::SAT_MODCOD_QPSK_3_TO_4, 1.487473, 4.03},
            {SatEnums::SAT_MODCOD_8PSK_3_TO_4, 2.228124, 7.91},
            {SatEnums::SAT_MODCOD_16APSK_3_TO_4, 2.966728, 10.21},
            {SatEnums::SAT_MODCOD_32APSK_3_TO_4, 3.703295, 12.73},
        };
        double lastReq = -1e9;
        for (const auto& r : table)
        {
            NS_TEST_ASSERT_MSG_EQ_TOL(ThzNtnPacketLink::SpectralEfficiency(r.m), r.eff, 2e-3,
                                      "Table 13 spectral efficiency");
            const double req = ThzNtnLinkErrorModel::DvbS2RequiredEsNoDb(r.m, 1e-4);
            NS_TEST_ASSERT_MSG_EQ_TOL(req, r.esno, 1.5, "required Es/N0 near the ideal value");
            NS_TEST_ASSERT_MSG_GT(req, lastReq, "requirement grows with efficiency");
            lastReq = req;
            // BLER falls through the waterfall around the requirement
            NS_TEST_ASSERT_MSG_GT(ThzNtnLinkErrorModel::DvbS2Bler(r.m, req - 1.0), 1e-3,
                                  "1 dB below: in the waterfall");
            NS_TEST_ASSERT_MSG_LT(ThzNtnLinkErrorModel::DvbS2Bler(r.m, req + 1.0), 1e-4,
                                  "1 dB above: quasi error free");
        }

        Ptr<ThzNtnPacketLink> link = CreateObject<ThzNtnPacketLink>();
        ThzNtnPacketLink::Config c;
        c.acmMarginDb = 1.0;
        link->SetConfig(c);
        double lastEff = 0;
        for (double esno = -1.0; esno <= 20.0; esno += 0.5)
        {
            SatEnums::SatModcod_t m;
            const bool ok = link->SelectModcod(esno, m);
            if (!ok)
            {
                continue;
            }
            const double eff = ThzNtnPacketLink::SpectralEfficiency(m);
            NS_TEST_ASSERT_MSG_GT_OR_EQ(eff, lastEff, "ACM never steps down as Es/N0 rises");
            NS_TEST_ASSERT_MSG_LT_OR_EQ(ThzNtnLinkErrorModel::DvbS2RequiredEsNoDb(m, c.blerTarget) + 1.0,
                                        esno + 1e-9, "the chosen MODCOD closes with margin");
            for (auto o : ThzNtnPacketLink::DvbS2ModcodSet())
            {
                if (ThzNtnPacketLink::SpectralEfficiency(o) > eff + 1e-9)
                {
                    NS_TEST_ASSERT_MSG_GT(ThzNtnLinkErrorModel::DvbS2RequiredEsNoDb(o, c.blerTarget) + 1.0,
                                          esno, "no more efficient MODCOD would close");
                }
            }
            lastEff = eff;
        }
        SatEnums::SatModcod_t m;
        NS_TEST_ASSERT_MSG_EQ(link->SelectModcod(-10.0, m), false, "-10 dB closes no MODCOD");
    }
};

// ---------------------------------------------------------------------------
/// Cross-module: the THz packet link (140 GHz, DVB-S2 ACM) from an SGP4
/// satellite (ntn-constellation) to a WGS-84 ground station, both in the
/// Earth-fixed frame, from culmination down the pass. Delay floor = altitude
/// over c, ACM adapts as the elevation falls, the ACM-sized BLER keeps PDR
/// high, and the same stream reproduces the run exactly.
class ThzPacketLinkSgp4PassTest : public TestCase
{
  public:
    ThzPacketLinkSgp4PassTest()
        : TestCase("THz packet link over an Earth-fixed SGP4 pass: delay floor, ACM, PDR, determinism")
    {
    }

  private:
    struct Out
    {
        uint64_t tx{0};
        uint64_t rx{0};
        double minDelayMs{0};
        double minElev{0};
        double maxElev{0};
        uint64_t changes{0};
        double alt{0};
        double fmDelayMs{0};
    };

    Out RunOnce(int64_t stream)
    {
        ntncon::WalkerConfig w;
        w.total_sats = 10;
        w.num_planes = 1;
        w.altitude_km = 550.0;
        w.epoch_unix_s = 1767225600.0;
        auto el = ntncon::WalkerConstellation::BuildDelta(w);
        Ptr<ntncon::Sgp4MobilityModel> sat = CreateObject<ntncon::Sgp4MobilityModel>();
        sat->SetElementsSgp4(el[0]);
        double lat;
        double lon;
        double alt;
        sat->GetGeodetic(lat, lon, alt);
        Ptr<ConstantPositionMobilityModel> gnd = CreateObject<ConstantPositionMobilityModel>();
        gnd->SetPosition(ntncon::NtnGeocentricMobilityAdapter::GeodeticToEcef(Vector(lat, lon, 0.0)));
        NodeContainer n;
        n.Create(2);
        n.Get(0)->AggregateObject(sat);
        n.Get(1)->AggregateObject(gnd);
        Ptr<ThzNtnPacketLink> link = CreateObject<ThzNtnPacketLink>();
        ThzNtnPacketLink::Config c;
        c.freqHz = 140e9;
        c.txPowerDbm = 30.0;
        c.spaceGainDbi = 50.0;
        c.groundGainDbi = 50.0;
        c.bandwidthHz = 400e6;
        link->SetConfig(c);
        NetDeviceContainer d = link->Install(n.Get(0), n.Get(1), sat, gnd);
        link->AssignStreams(stream);
        InternetStackHelper inet;
        inet.Install(n);
        Ipv4AddressHelper ip;
        ip.SetBase("10.140.0.0", "255.255.255.252");
        Ipv4InterfaceContainer ifs = ip.Assign(d);
        OnOffHelper src("ns3::UdpSocketFactory", InetSocketAddress(ifs.GetAddress(1), 9));
        src.SetConstantRate(DataRate("4Mbps"), 1000);
        ApplicationContainer sa = src.Install(n.Get(0));
        sa.Start(Seconds(0.2));
        sa.Stop(Seconds(150.0));
        PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 9));
        sink.Install(n.Get(1));
        FlowMonitorHelper fmh;
        Ptr<FlowMonitor> fm = fmh.InstallAll();
        Simulator::Stop(Seconds(151.0));
        Simulator::Run();
        Out o;
        double delaySum = 0;
        for (const auto& [id, st] : fm->GetFlowStats())
        {
            o.tx += st.txPackets;
            o.rx += st.rxPackets;
            delaySum += st.delaySum.GetSeconds();
        }
        o.fmDelayMs = o.rx ? delaySum / o.rx * 1e3 : 0;
        o.minDelayMs = link->GetMinDelayMs();
        o.minElev = link->GetMinElevationDeg();
        o.maxElev = link->GetMaxElevationDeg();
        o.changes = link->GetModcodChanges();
        o.alt = alt;
        Simulator::Destroy();
        return o;
    }

    void DoRun() override
    {
        const Out a = RunOnce(5);
        NS_TEST_ASSERT_MSG_GT(a.tx, 70000u, "4 Mb/s offered for 150 s");
        NS_TEST_ASSERT_MSG_EQ_TOL(a.minDelayMs, a.alt / 299792458.0 * 1e3, 0.01,
                                  "delay floor = altitude / c at culmination");
        NS_TEST_ASSERT_MSG_GT(a.fmDelayMs, a.minDelayMs, "measured mean delay above the floor");
        NS_TEST_ASSERT_MSG_GT(a.maxElev, 85.0, "starts near zenith in the ECEF frame");
        NS_TEST_ASSERT_MSG_LT(a.minElev, 45.0, "descends the pass");
        NS_TEST_ASSERT_MSG_GT(a.changes, 0u, "ACM follows the falling elevation");
        NS_TEST_ASSERT_MSG_GT(double(a.rx) / a.tx, 0.98, "ACM sized for BLER 1e-4 keeps PDR high");
        const Out b = RunOnce(5);
        NS_TEST_ASSERT_MSG_EQ(b.rx, a.rx, "same stream, same deliveries");
        NS_TEST_ASSERT_MSG_EQ(b.changes, a.changes, "same stream, same MODCOD history");
    }
};

// ---------------------------------------------------------------------------
/// Pointing: the parabolic main-lobe law and the Gaussian-beam law at known
/// offsets, and the vibration jitter sampled every 10 ms inside a run is
/// Rayleigh with E[theta] = sigma sqrt(pi/2), reproducible per stream.
class ThzPointingStatisticsTest : public TestCase
{
  public:
    ThzPointingStatisticsTest()
        : TestCase("Pointing loss laws and Rayleigh jitter statistics under a running clock")
    {
    }

  private:
    std::vector<double> Sample(int64_t stream, double sigma)
    {
        Ptr<ThzNtnPointingError> p = CreateObject<ThzNtnPointingError>();
        p->SetVibrationRms_deg(sigma);
        p->AssignStreams(stream);
        std::vector<double> v;
        for (int k = 0; k < 6000; ++k)
        {
            Simulator::Schedule(MilliSeconds(10 * k), [&v, p]() { v.push_back(p->GetPointingErrorSample_deg()); });
        }
        Simulator::Run();
        Simulator::Destroy();
        return v;
    }

    void DoRun() override
    {
        Ptr<ThzNtnPointingError> p = CreateObject<ThzNtnPointingError>();
        NS_TEST_ASSERT_MSG_EQ_TOL(p->ComputePointingLoss_dB(0.05, 0.1), 3.0, 1e-9,
                                  "half the 3 dB beamwidth costs 3 dB (12 x 0.25)");
        NS_TEST_ASSERT_MSG_EQ_TOL(p->ComputePointingLoss_dB(0.1, 0.1), 12.0, 1e-9, "12 dB at theta_3dB");
        NS_TEST_ASSERT_MSG_EQ_TOL(p->ComputeGaussianPointingLoss_dB(0.05, 0.1),
                                  10.0 * std::log10(std::exp(1.0)) * 4.0 * std::log(2.0) * 0.25, 0.01,
                                  "Gaussian beam: 3.01 dB at half the beamwidth");
        NS_TEST_ASSERT_MSG_EQ(p->ComputePointingLoss_dB(0.05, 0.0), 0.0, "invalid beamwidth: no loss");

        const double sigma = 0.01;
        auto a = Sample(3, sigma);
        auto b = Sample(3, sigma);
        auto c = Sample(4, sigma);
        NS_TEST_ASSERT_MSG_EQ((a == b), true, "same stream, same jitter");
        NS_TEST_ASSERT_MSG_EQ((a == c), false, "another stream, other jitter");
        double sum = 0;
        double sum2 = 0;
        for (double x : a)
        {
            NS_TEST_ASSERT_MSG_GT_OR_EQ(x, 0.0, "a radial error is non-negative");
            sum += x;
            sum2 += x * x;
        }
        const double mean = sum / a.size();
        NS_TEST_ASSERT_MSG_EQ_TOL(mean, sigma * std::sqrt(M_PI / 2.0), 0.03 * sigma * std::sqrt(M_PI / 2.0),
                                  "Rayleigh mean sigma sqrt(pi/2)");
        NS_TEST_ASSERT_MSG_EQ_TOL(sum2 / a.size(), 2.0 * sigma * sigma, 0.06 * 2.0 * sigma * sigma,
                                  "Rayleigh second moment 2 sigma^2");
    }
};

// ---------------------------------------------------------------------------
/// Hardware impairments: PA distortion against input back-off, the
/// hardware-limited SNR ceiling 1 / kappa^2, and capacity that saturates
/// instead of growing without bound. Evaluated along a running SNR ramp.
class ThzHardwareCeilingTest : public TestCase
{
  public:
    ThzHardwareCeilingTest()
        : TestCase("PA back-off and the SNR / capacity ceiling set by EVM")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<ThzNtnHardwareImpairments> h = CreateObject<ThzNtnHardwareImpairments>();
        // PA distortion falls as the input back-off grows (Saleh/Rapp AM/AM
        // compression is weaker further from saturation)
        double lastEvm = 1e9;
        for (double ibo : {0.0, 3.0, 6.0, 10.0})
        {
            const double e = h->ComputePaNonlinearityEvm(ibo);
            NS_TEST_ASSERT_MSG_GT_OR_EQ(e, 0.0, "EVM is non-negative");
            NS_TEST_ASSERT_MSG_LT(e, lastEvm, "more back-off, less PA distortion");
            lastEvm = e;
        }

        const double kappa = h->ComputeEvmTotal();
        NS_TEST_ASSERT_MSG_GT(kappa, 0.0, "some EVM");
        NS_TEST_ASSERT_MSG_LT(kappa, 0.5, "a plausible EVM");
        const double ceiling = -20.0 * std::log10(kappa);
        std::vector<double> eff;
        std::vector<double> cap;
        for (int k = 0; k <= 12; ++k)
        {
            Simulator::Schedule(Seconds(k), [&, k]() {
                const double snr = -10.0 + 5.0 * k;
                eff.push_back(h->ComputeEffectiveSnr_dB(snr));
                cap.push_back(h->ComputeHardwareLimitedCapacity(std::pow(10.0, snr / 10.0)));
            });
        }
        Simulator::Run();
        Simulator::Destroy();
        for (size_t i = 0; i < eff.size(); ++i)
        {
            const double snr = -10.0 + 5.0 * i;
            const double s = std::pow(10.0, snr / 10.0);
            NS_TEST_ASSERT_MSG_EQ_TOL(eff[i], 10.0 * std::log10(s / (1.0 + s * kappa * kappa)), 1e-9,
                                      "SNR / (1 + SNR kappa^2)");
            NS_TEST_ASSERT_MSG_LT(eff[i], std::min(snr, ceiling) + 1e-9, "below both limits");
            if (i > 0)
            {
                NS_TEST_ASSERT_MSG_GT_OR_EQ(cap[i], cap[i - 1], "capacity non-decreasing in SNR");
            }
        }
        NS_TEST_ASSERT_MSG_EQ_TOL(eff.back(), ceiling, 0.5, "50 dB ideal SNR is pinned near 1 / kappa^2");
        NS_TEST_ASSERT_MSG_LT(cap.back(), std::log2(1.0 + 1.0 / (kappa * kappa)) + 1e-6,
                              "capacity bounded by log2(1 + 1 / kappa^2)");
    }
};

// ---------------------------------------------------------------------------
/// EKF beam tracking of a satellite sweeping 0.6 deg/s in elevation and
/// 0.3 deg/s in azimuth, with 0.05 deg measurement noise at 10 Hz: the
/// one-step prediction error stays well below the raw measurement noise.
/// Then a blockage drives the SINR below the failure threshold: beam failure
/// is declared after exactly ConsecutiveFailures instances (TS 38.321 5.17),
/// and recovery returns to tracking on the first good measurement.
class ThzBeamTrackingEkfTest : public TestCase
{
  public:
    ThzBeamTrackingEkfTest()
        : TestCase("EKF beam tracking error, beam-failure declaration and recovery (TS 38.321 5.17)")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<ThzNtnBeamTracking> bt = CreateObject<ThzNtnBeamTracking>();
        bt->SetAttribute("TrackingMode", StringValue("EKF"));
        bt->SetAttribute("MeasurementNoise", DoubleValue(0.05));
        bt->SetAttribute("BeamFailureThreshold_dB", DoubleValue(0.0));
        bt->SetAttribute("ConsecutiveFailures", UintegerValue(4));
        Ptr<NormalRandomVariable> noise = CreateObject<NormalRandomVariable>();
        noise->SetAttribute("Variance", DoubleValue(0.05 * 0.05));
        noise->SetStream(77);
        auto truth = [](double t) { return std::make_pair(30.0 + 0.6 * t, 100.0 + 0.3 * t); };
        double se = 0;
        uint32_t ne = 0;
        std::vector<ThzBeamTrackingState> states;
        for (int k = 0; k <= 400; ++k)
        {
            Simulator::Schedule(MilliSeconds(100 * k), [&, k]() {
                const double t = Simulator::Now().GetSeconds();
                if (k > 50 && k <= 300)
                {
                    // score the prediction made before this measurement
                    const auto p = bt->PredictBeamDirection(t);
                    const auto tr = truth(t);
                    se += std::pow(p.first - tr.first, 2) + std::pow(p.second - tr.second, 2);
                    ++ne;
                }
                const bool blocked = k > 300 && k <= 306;
                const auto tr = truth(t);
                bt->UpdateMeasurement(tr.first + noise->GetValue(), tr.second + noise->GetValue(),
                                      blocked ? -10.0 : 20.0, t);
                if (k == 304)
                {
                    states.push_back(bt->GetTrackingState()); // after 4 low instances
                    bt->TriggerBeamRecovery();
                    states.push_back(bt->GetTrackingState());
                }
                if (k == 303)
                {
                    states.push_back(bt->GetTrackingState()); // after 3 low instances
                }
                if (k == 307)
                {
                    states.push_back(bt->GetTrackingState()); // first good after blockage
                }
            });
        }
        Simulator::Run();
        Simulator::Destroy();
        const double rms = std::sqrt(se / ne);
        // raw 2-D measurement error RMS would be 0.05 sqrt(2) = 0.071 deg
        NS_TEST_ASSERT_MSG_LT(rms, 0.05, "EKF prediction beats the raw measurement noise");
        NS_TEST_ASSERT_MSG_EQ(states.size(), 4u, "four state probes");
        NS_TEST_ASSERT_MSG_EQ((states[0] == ThzBeamTrackingState::TRACKING), true,
                              "3 low instances: not yet a failure");
        NS_TEST_ASSERT_MSG_EQ((states[1] == ThzBeamTrackingState::BEAM_FAILURE), true,
                              "4th consecutive low instance declares beam failure");
        NS_TEST_ASSERT_MSG_EQ((states[2] == ThzBeamTrackingState::RECOVERY), true, "recovery triggered");
        NS_TEST_ASSERT_MSG_EQ((states[3] == ThzBeamTrackingState::TRACKING), true,
                              "first good measurement after recovery returns to tracking");
        NS_TEST_ASSERT_MSG_EQ(bt->GetTrackingMetrics().beamFailureCount, 1u, "one failure counted");
    }
};

// ---------------------------------------------------------------------------
/// Absorption-aware sub-band selection on a 270-330 GHz grid during a run:
/// with the link geometry of a 30 deg slant path, the sub-band holding the
/// 325.15 GHz water-vapour line is excluded at a 60 dB threshold while part
/// of the 270-300 GHz window (about 10 dB at zenith, twice that at 30 deg)
/// stays usable; every returned sub-band respects the
/// threshold, and a 0 dB threshold leaves nothing (failure path).
class ThzMacAbsorptionAwareTest : public TestCase
{
  public:
    ThzMacAbsorptionAwareTest()
        : TestCase("MAC excludes sub-bands on the 325 GHz water line and keeps the window")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<ThzNtnMac> mac = CreateObject<ThzNtnMac>();
        mac->SetAttribute("MaxAbsorptionThreshold_dB", DoubleValue(60.0)); // DAMC filter
        mac->ConfigureResourceGrid(60e9, 1e9, 300e9);
        mac->SetMolecularAbsorptionModel(CreateObject<ThzNtnMolecularAbsorption>());
        Ptr<ConstantPositionMobilityModel> gnd = CreateObject<ConstantPositionMobilityModel>();
        Ptr<ConstantPositionMobilityModel> sat = CreateObject<ConstantPositionMobilityModel>();
        sat->SetPosition(Vector(500e3 * std::cos(M_PI / 6), 0, 500e3 * std::sin(M_PI / 6)));
        std::vector<ThzNtnSubBand> avail;
        std::vector<ThzNtnSubBand> none;
        Simulator::Schedule(Seconds(1), [&]() {
            mac->UpdateSubBandAvailability(gnd, sat);
            avail = mac->GetAvailableSubBands(60.0);
            none = mac->GetAvailableSubBands(0.0);
        });
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_GT(avail.size(), 5u, "part of the 270-300 GHz window is usable");
        NS_TEST_ASSERT_MSG_LT(avail.size(), 60u, "something is excluded");
        bool lineKept = false;
        bool windowKept = false;
        for (const auto& s : avail)
        {
            NS_TEST_ASSERT_MSG_LT_OR_EQ(s.absorptionLoss_dB, 60.0, "threshold respected");
            lineKept |= std::abs(s.centerFreqHz - 325.15e9) < 0.5 * s.bandwidthHz;
            windowKept |= std::abs(s.centerFreqHz - 285e9) < 0.6 * s.bandwidthHz;
        }
        NS_TEST_ASSERT_MSG_EQ(lineKept, false, "the 325.15 GHz H2O line sub-band is dropped");
        NS_TEST_ASSERT_MSG_EQ(windowKept, true, "285 GHz stays");
        NS_TEST_ASSERT_MSG_EQ(none.size(), 0u, "a slant path through the air is never lossless");
    }
};

// ---------------------------------------------------------------------------
/// Cross-module: a THz ISL ring over a 12-satellite SGP4 plane
/// (ntn-constellation). Every neighbour pair (chord 2 a sin 15 deg = 3586 km)
/// is established; the opposite satellite (13 856 km) and an unknown id are
/// refused, and so is a duplicate. Links stay ESTABLISHING for LinkSetupTime
/// (50 ms) and carry no route until UP; then the minimum-delay route 0 -> 3
/// is 3 hops of light time, it survives a topology refresh a minute later,
/// and tearing one ring link reroutes the long way round (9 hops).
class ThzIslRingSgp4Test : public TestCase
{
  public:
    ThzIslRingSgp4Test()
        : TestCase("THz ISL ring over an SGP4 plane: setup latency, light-time routes, teardown reroute")
    {
    }

  private:
    void DoRun() override
    {
        ntncon::WalkerConfig w;
        w.total_sats = 12;
        w.num_planes = 1;
        w.altitude_km = 550.0;
        w.epoch_unix_s = 1767225600.0;
        auto el = ntncon::WalkerConstellation::BuildDelta(w);
        std::map<uint32_t, Ptr<MobilityModel>> sats;
        for (uint32_t i = 0; i < 12; ++i)
        {
            Ptr<ntncon::Sgp4MobilityModel> m = CreateObject<ntncon::Sgp4MobilityModel>();
            m->SetElementsSgp4(el[i]);
            sats[i] = m;
        }
        Ptr<ThzNtnIslLink> isl = CreateObject<ThzNtnIslLink>();
        const double chordMs = 2.0 * (6378137.0 + 550e3) * std::sin(M_PI / 12.0) / 299792458.0 * 1e3;
        uint32_t established = 0;
        bool far = true;
        bool unknown = true;
        bool dup = true;
        ThzNtnIslRoute early;
        ThzNtnIslRoute r3;
        ThzNtnIslRoute r3late;
        ThzNtnIslRoute rTorn;
        size_t earlyActive = 99;
        size_t lateActive = 0;
        Simulator::Schedule(Seconds(0), [&]() {
            isl->UpdateTopology(sats);
            for (uint32_t i = 0; i < 12; ++i)
            {
                established += isl->EstablishLink(i, (i + 1) % 12);
            }
            far = isl->EstablishLink(0, 6);
            unknown = isl->EstablishLink(0, 99);
            dup = isl->EstablishLink(1, 0);
        });
        Simulator::Schedule(MilliSeconds(10), [&]() {
            earlyActive = isl->GetActiveLinkIds().size();
            early = isl->ComputeRoute(0, 3, ThzNtnRoutingAlgorithm::MINIMUM_DELAY);
        });
        Simulator::Schedule(MilliSeconds(100), [&]() {
            r3 = isl->ComputeRoute(0, 3, ThzNtnRoutingAlgorithm::MINIMUM_DELAY);
        });
        Simulator::Schedule(Seconds(60), [&]() {
            isl->UpdateTopology(sats);
            r3late = isl->ComputeRoute(0, 3, ThzNtnRoutingAlgorithm::MINIMUM_DELAY);
            const auto ids = isl->GetActiveLinkIds();
            lateActive = ids.size();
            if (ids.size() == 12)
            {
                isl->TeardownLink(ids[2]); // the (2, 3) link
            }
            rTorn = isl->ComputeRoute(0, 3, ThzNtnRoutingAlgorithm::MINIMUM_DELAY);
        });
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(established, 12u, "every ring neighbour within 5000 km");
        NS_TEST_ASSERT_MSG_EQ(far, false, "the opposite satellite is beyond MaxLinkDistance");
        NS_TEST_ASSERT_MSG_EQ(unknown, false, "unknown satellite refused");
        NS_TEST_ASSERT_MSG_EQ(dup, false, "an existing link is not duplicated");
        NS_TEST_ASSERT_MSG_EQ(earlyActive, 0u, "10 ms after the request no link is active yet");
        NS_TEST_ASSERT_MSG_EQ(lateActive, 12u, "after the 50 ms setup all 12 are active");
        NS_TEST_ASSERT_MSG_EQ(early.numHops, 0u, "no route over links that are not up");
        NS_TEST_ASSERT_MSG_EQ(r3.numHops, 3u, "0 -> 3 is three hops");
        NS_TEST_ASSERT_MSG_EQ_TOL(r3.totalDelay_ms, 3.0 * chordMs, 0.02 * chordMs, "three chords of light time");
        NS_TEST_ASSERT_MSG_EQ_TOL(r3late.totalDelay_ms, r3.totalDelay_ms, 0.01 * r3.totalDelay_ms,
                                  "in-plane spacing holds a minute later");
        NS_TEST_ASSERT_MSG_EQ(rTorn.numHops, 9u, "with (2, 3) torn down the route goes the long way");
        NS_TEST_ASSERT_MSG_EQ_TOL(rTorn.totalDelay_ms, 9.0 * chordMs, 0.02 * 9.0 * chordMs, "nine chords");
    }
};

class ThzNtnInDepthTestSuite : public TestSuite
{
  public:
    ThzNtnInDepthTestSuite()
        : TestSuite("thz-ntn-indepth", Type::UNIT)
    {
        AddTestCase(new ThzDvbS2TableAcmTest, Duration::QUICK);
        AddTestCase(new ThzPacketLinkSgp4PassTest, Duration::QUICK);
        AddTestCase(new ThzPointingStatisticsTest, Duration::QUICK);
        AddTestCase(new ThzHardwareCeilingTest, Duration::QUICK);
        AddTestCase(new ThzBeamTrackingEkfTest, Duration::QUICK);
        AddTestCase(new ThzMacAbsorptionAwareTest, Duration::QUICK);
        AddTestCase(new ThzIslRingSgp4Test, Duration::QUICK);
    }
};

static ThzNtnInDepthTestSuite g_thzNtnInDepthTestSuite;
