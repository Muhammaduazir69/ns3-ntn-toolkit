/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// In-depth cases for NtnRealStackHelper (Nov 2026 test campaign), focused on
// the Wave 9-11b behavior: TS 38.101-5 band edges and channel placement for
// paired FDD, frequency reuse over three live cells with the sub-band and
// power invariants of TR 38.821 6.1.1.1, a reuse-color misconfiguration as a
// failure path, the Shannon bound and overload on the measured plane, a link
// outage with recovery, the SSB beam grid along a moving pass, and
// determinism of the whole stack.

#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/double.h"
#include "ns3/node-container.h"
#include "ns3/nr-gnb-mac.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/nr-epc-helper.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-static-extra-loss-model.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <cmath>
#include <iostream>
#include <set>
#include <vector>

using namespace ns3;

namespace
{

/// `n` satellites at 600 km in a row along the equator `spacingKm` apart, and
/// one UE at each satellite's nadir.
void
Row(NodeContainer& sats, NodeContainer& ues, uint32_t n, double spacingKm)
{
    sats.Create(n);
    ues.Create(n);
    for (uint32_t i = 0; i < n; ++i)
    {
        const double lon = i * spacingKm / 111.195;
        auto sm = CreateObject<ConstantPositionMobilityModel>();
        sm->SetPosition(NtnRealStackHelper::GeodeticToEcef(0.0, lon, 600e3));
        sats.Get(i)->AggregateObject(sm);
        auto um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(NtnRealStackHelper::GeodeticToEcef(0.0, lon, 0.0));
        ues.Get(i)->AggregateObject(um);
    }
}

void
NrSet1(NtnRealStackHelper& rs, double simS, const std::string& dir)
{
    rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
    rs.SetNumerology(1);
    rs.SetSimTime(Seconds(simS));
    rs.SetOutputDir(dir);
    rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
}

} // namespace

/// TS 38.101-5 Table 5.2-1 / 5.3.5-1: n255 (DL 1525-1559, UL 1626.5-1660.5
/// MHz, 5/10 MHz) and n256 (DL 2170-2200, UL 1980-2010 MHz, 5..20 MHz). The
/// paired uplink keeps the band's duplex spacing up to the block edges; a
/// carrier just outside a block has no pair; a channel must fit wholly inside
/// its block to be conformant. On a live cell, Auto duplex on n256 builds paired
/// FDD with the uplink 190 MHz below, and a carrier outside any NTN band falls
/// back to TDD (failure path, no abort).
class FddBandEdgesTest : public TestCase
{
  public:
    FddBandEdgesTest()
        : TestCase("TS 38.101-5 n255/n256: duplex pairing at band edges, channel in block, Auto FDD")
    {
    }

  private:
    void DoRun() override
    {
        using H = NtnRealStackHelper;
        NS_TEST_ASSERT_MSG_EQ_TOL(H::PairedUlCarrierHz(2170.0e6), 1980.0e6, 1.0, "n256 lower edge");
        NS_TEST_ASSERT_MSG_EQ_TOL(H::PairedUlCarrierHz(2200.0e6), 2010.0e6, 1.0, "n256 upper edge");
        NS_TEST_ASSERT_MSG_EQ_TOL(H::PairedUlCarrierHz(1525.0e6), 1626.5e6, 1.0, "n255 lower edge");
        NS_TEST_ASSERT_MSG_EQ_TOL(H::PairedUlCarrierHz(1559.0e6), 1660.5e6, 1.0, "n255 upper edge");
        NS_TEST_ASSERT_MSG_EQ(H::PairedUlCarrierHz(2169.9e6), 0.0, "just below n256 DL: no pair");
        NS_TEST_ASSERT_MSG_EQ(H::PairedUlCarrierHz(2200.1e6), 0.0, "just above n256 DL: no pair");
        NS_TEST_ASSERT_MSG_EQ(H::PairedUlCarrierHz(1995.0e6), 0.0, "an UL carrier has no DL pair");
        NS_TEST_ASSERT_MSG_EQ(std::string(H::ClassifyNtnFr1(1995e6, 10e6).name), "n256", "n256 UL");
        NS_TEST_ASSERT_MSG_EQ(H::ClassifyNtnFr1(1995e6, 10e6).carrierInUplink, true, "UL block");
        NS_TEST_ASSERT_MSG_EQ(H::ClassifyNtnFr1(2185e6, 20e6).conformant, true, "centered 20 MHz");
        NS_TEST_ASSERT_MSG_EQ(H::ClassifyNtnFr1(2175e6, 20e6).conformant, false,
                              "2165..2185 MHz leaves the n256 block (TS 38.101-5 5.4)");
        NS_TEST_ASSERT_MSG_EQ(H::ClassifyNtnFr1(2175e6, 20e6).channelInBlock, false, "flagged");
        NS_TEST_ASSERT_MSG_EQ(H::ClassifyNtnFr1(2195e6, 10e6).conformant, true, "2190..2200 fits");
        NS_TEST_ASSERT_MSG_EQ(H::ClassifyNtnFr1(1554e6, 10e6).conformant, true, "1549..1559 fits");
        NS_TEST_ASSERT_MSG_EQ(H::ClassifyNtnFr1(1555e6, 10e6).conformant, false, "1560 is out");
        NS_TEST_ASSERT_MSG_EQ(H::ClassifyNtnFr1(1540e6, 20e6).conformant, false,
                              "20 MHz is not an n255 bandwidth");
        NS_TEST_ASSERT_MSG_EQ(std::string(H::ClassifyNtnFr1(28e9, 100e6).name), "none", "FR2");

        for (int mode = 0; mode < 2; ++mode)
        {
            NodeContainer sats;
            NodeContainer ues;
            Row(sats, ues, 1, 0.0);
            NtnRealStackHelper rs;
            NrSet1(rs, 1.0, CreateTempDirFilename("fdd"));
            if (mode == 1)
            {
                rs.SetCarrierFrequencyHz(3.5e9); // a terrestrial n78 carrier
                rs.SetBandwidthHz(20e6);
            }
            rs.Build(sats, ues);
            rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::UrllcPings, Seconds(0.3), Seconds(0.95));
            Simulator::Stop(Seconds(1.0));
            Simulator::Run();
            const uint64_t rx = rs.GetUeRxPackets(0);
            if (mode == 0)
            {
                NS_TEST_ASSERT_MSG_EQ(rs.IsFddActive(), true, "Auto on n256 builds paired FDD");
                NS_TEST_ASSERT_MSG_EQ_TOL(rs.GetUlCarrierFrequencyHz(),
                                          rs.GetCarrierFrequencyHz() - 190e6, 1.0,
                                          "uplink 190 MHz below the downlink");
            }
            else
            {
                NS_TEST_ASSERT_MSG_EQ(rs.IsFddActive(), false, "no NTN band: Auto falls back to TDD");
                NS_TEST_ASSERT_MSG_EQ(rs.GetUlCarrierFrequencyHz(), rs.GetCarrierFrequencyHz(),
                                      "TDD shares the carrier");
            }
            NS_TEST_ASSERT_MSG_GT(rx, 40u, "packets flow in either duplex mode");
            Simulator::Destroy();
        }
    }
};

/// Frequency reuse over THREE live cells (TR 38.821 6.1.1.1 FRF 3). Under the
/// Set-1 budget, Auto picks FRF 3; the three sub-bands are disjoint, of equal
/// RBG count and inside the carrier; each cell radiates the carrier power times
/// its RB share; and with all three cells loaded at once every UE is
/// noise-limited (SINR > 12 dB), at least 4 dB better than the same three cells
/// at FRF 1 (satellites 120 km apart). Failure path: two adjacent cells forced onto the same color get
/// their co-channel interference back.
class FrequencyReuseThreeCellsTest : public TestCase
{
  public:
    FrequencyReuseThreeCellsTest()
        : TestCase("TR 38.821 FRF 3 on three loaded cells: disjoint sub-bands, power share, SINR gain")
    {
    }

  private:
    struct Out
    {
        uint8_t frf{0};
        std::vector<uint8_t> color;
        std::vector<std::pair<uint32_t, uint32_t>> rb;
        std::vector<double> txDbm;
        double carrierDbm{0.0};
        uint32_t rbTotal{0};
        uint32_t rbPerRbg{1};
        std::vector<double> sinrDb;
        std::vector<uint64_t> rx;
    };

    // mode 0: Auto (FRF 3), 1: FRF 1, 2: FRF 3 with cells 0 and 1 on color 0
    static Out Run(int mode, const std::string& dir)
    {
        NodeContainer sats;
        NodeContainer ues;
        Row(sats, ues, 3, 120.0);
        NtnRealStackHelper rs;
        NrSet1(rs, 2.5, dir);
        if (mode == 1)
        {
            rs.SetFrequencyReuse(1);
        }
        if (mode == 2)
        {
            rs.SetFrequencyReuse(3);
            rs.SetCellReuseColor(0, 0);
            rs.SetCellReuseColor(1, 0);
            rs.SetCellReuseColor(2, 2);
        }
        rs.SetEmbbRate(DataRate("20Mb/s"));
        rs.Build(sats, ues);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming, Seconds(0.5), Seconds(2.45));
        Out o;
        o.frf = rs.GetFrequencyReuseApplied();
        o.carrierDbm = rs.GetBwpConductedPowerDbm(0);
        for (uint32_t i = 0; i < 3; ++i)
        {
            o.color.push_back(rs.GetCellReuseColor(i));
            o.rb.push_back(rs.GetCellRbRange(i));
            DoubleValue tx;
            NrHelper::GetGnbPhy(rs.GetEnbDevices().Get(i), 0)->GetAttribute("TxPower", tx);
            o.txDbm.push_back(tx.Get());
        }
        o.rbTotal = NrHelper::GetGnbPhy(rs.GetEnbDevices().Get(0), 0)->GetRbNum();
        o.rbPerRbg = NrHelper::GetGnbMac(rs.GetEnbDevices().Get(0), 0)->GetNumRbPerRbg();
        Simulator::Schedule(Seconds(1.0), [&rs]() {
            for (uint32_t u = 0; u < 3; ++u)
            {
                rs.TakeUeSinrWindowMeanDb(u);
            }
        });
        Simulator::Schedule(Seconds(2.4), [&rs, &o]() {
            for (uint32_t u = 0; u < 3; ++u)
            {
                o.sinrDb.push_back(rs.TakeUeSinrWindowMeanDb(u));
            }
        });
        Simulator::Stop(Seconds(2.5));
        Simulator::Run();
        for (uint32_t u = 0; u < 3; ++u)
        {
            o.rx.push_back(rs.GetUeRxPackets(u));
        }
        std::cout << "  [FRF mode " << mode << "] frf=" << +o.frf << " SINR dB:";
        for (double x : o.sinrDb)
        {
            std::cout << " " << x;
        }
        std::cout << "\n";
        Simulator::Destroy();
        return o;
    }

    void DoRun() override
    {
        const Out r = Run(0, CreateTempDirFilename("frf3"));
        NS_TEST_ASSERT_MSG_EQ(+r.frf, 3, "Auto: FRF 3 under Set-1 with three cells");
        NS_TEST_ASSERT_MSG_EQ(std::set<uint8_t>(r.color.begin(), r.color.end()).size(), 3u,
                              "three distinct colors");
        for (uint32_t i = 0; i < 3; ++i)
        {
            NS_TEST_ASSERT_MSG_GT(r.rb[i].second, 0u, "a sub-band per cell");
            NS_TEST_ASSERT_MSG_LT(r.rb[i].first + r.rb[i].second, r.rbTotal + 1, "inside the carrier");
            NS_TEST_ASSERT_MSG_LT(std::abs(static_cast<int>(r.rb[i].second) -
                                           static_cast<int>(r.rb[0].second)),
                                  static_cast<int>(r.rbPerRbg) + 1,
                                  "sub-bands equal to within one RBG");
            for (uint32_t j = i + 1; j < 3; ++j)
            {
                NS_TEST_ASSERT_MSG_EQ((r.rb[i].first + r.rb[i].second <= r.rb[j].first) ||
                                          (r.rb[j].first + r.rb[j].second <= r.rb[i].first),
                                      true, "cells " << i << " and " << j << " share no RB");
            }
            NS_TEST_ASSERT_MSG_EQ_TOL(r.txDbm[i],
                                      r.carrierDbm + 10.0 * std::log10(1.0 * r.rb[i].second / r.rbTotal),
                                      1e-6, "cell power = carrier power x RB share");
            NS_TEST_ASSERT_MSG_GT(r.sinrDb[i], 12.0, "UE " << i << " noise-limited under FRF 3");
            NS_TEST_ASSERT_MSG_GT(r.rx[i], 500u, "UE " << i << " served concurrently");
        }
        const Out one = Run(1, CreateTempDirFilename("frf1"));
        NS_TEST_ASSERT_MSG_EQ(+one.frf, 1, "FRF 1 when asked");
        for (uint32_t i = 0; i < 3; ++i)
        {
            // Measured: 5.1 to 7.1 dB of reuse gain with satellites 120 km apart.
            NS_TEST_ASSERT_MSG_GT(r.sinrDb[i], one.sinrDb[i] + 4.0,
                                  "UE " << i << ": FRF 3 beats FRF 1 by > 4 dB");
            NS_TEST_ASSERT_MSG_LT(one.sinrDb[i], 15.0, "UE " << i << ": FRF 1 is interference-limited");
        }
        const Out bad = Run(2, CreateTempDirFilename("frfbad"));
        NS_TEST_ASSERT_MSG_EQ(bad.rb[0].first, bad.rb[1].first, "cells 0 and 1 share a sub-band");
        NS_TEST_ASSERT_MSG_LT(bad.sinrDb[0], r.sinrDb[0] - 3.0,
                              "failure path: same color brings the co-channel interference back");
        NS_TEST_ASSERT_MSG_GT(bad.sinrDb[2], 12.0, "the correctly colored cell is unaffected");
    }
};

/// The measured plane obeys the Shannon bound and degrades gracefully under
/// overload. One noise-limited zenith cell, 20 MHz, eMBB offered at 200 Mb/s:
/// the measured application throughput is at most B log2(1 + SINR) of the
/// measured SINR (single layer), less than half the offered load is delivered
/// while the delivered packets show queueing delay, and the throughput still
/// reaches at least a third of the Shannon figure.
class ShannonBoundUnderOverloadTest : public TestCase
{
  public:
    ShannonBoundUnderOverloadTest()
        : TestCase("Measured throughput <= B log2(1+SINR); a 200 Mb/s overload queues the excess")
    {
    }

  private:
    void DoRun() override
    {
        NodeContainer sats;
        NodeContainer ues;
        Row(sats, ues, 1, 0.0);
        NtnRealStackHelper rs;
        NrSet1(rs, 2.0, CreateTempDirFilename("shannon"));
        rs.SetEmbbRate(DataRate("200Mb/s"));
        rs.Build(sats, ues);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming, Seconds(0.5), Seconds(1.95));
        Simulator::Stop(Seconds(2.0));
        Simulator::Run();
        rs.Collect();
        const double sinrDb = rs.GetMeanDlSinrDb();
        const double bw = rs.GetBandwidthHz();
        const double shannonMbps = bw * std::log2(1.0 + std::pow(10.0, sinrDb / 10.0)) / 1e6;
        const double thr = rs.GetRxThroughputMbps();
        NS_TEST_ASSERT_MSG_EQ(std::isfinite(sinrDb), true, "SINR measured");
        NS_TEST_ASSERT_MSG_GT(thr, 0.0, "traffic delivered");
        NS_TEST_ASSERT_MSG_LT(thr, shannonMbps, "below the Shannon capacity at the measured SINR");
        NS_TEST_ASSERT_MSG_GT(thr, shannonMbps / 3.0, "and a realistic fraction of it");
        // The excess is not delivered: it queues behind the bottleneck (RLC)
        // and is still queued when the run ends. GetAppLossRatio() counts
        // sequence gaps among received packets only, so a tail backlog is not
        // in it; the delivered share of the offered load is.
        NS_TEST_ASSERT_MSG_LT(thr / 200.0, 0.5, "less than half of the offered 200 Mb/s delivered");
        NS_TEST_ASSERT_MSG_GT(rs.GetMeanDelayMs(), 50.0,
                              "and what is delivered waited in the queue (OWD floor is 2 ms)");
        Simulator::Destroy();
    }
};

/// Link outage on the live NR stack (failure path): an 80 dB excess loss from
/// 1.5 s to 2.5 s. Application packets stop arriving during it and resume
/// after it; the DL SINR drops by tens of dB while it lasts.
class LinkOutageRecoveryTest : public TestCase
{
  public:
    LinkOutageRecoveryTest()
        : TestCase("Failure path: an 80 dB outage stops delivery, the stack recovers after it")
    {
    }

  private:
    void DoRun() override
    {
        NodeContainer sats;
        NodeContainer ues;
        Row(sats, ues, 1, 0.0);
        NtnRealStackHelper rs;
        NrSet1(rs, 4.0, CreateTempDirFilename("outage"));
        rs.Build(sats, ues);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::UrllcPings, Seconds(0.5), Seconds(3.95));
        auto loss = CreateObject<NtnStaticExtraLossModel>();
        loss->SetLossDb(0.0);
        rs.AddExtraPropagationLoss(loss);
        uint64_t a = 0;
        uint64_t b = 0;
        uint64_t c = 0;
        Simulator::Schedule(Seconds(1.5), [&]() {
            a = rs.GetUeRxPackets(0);
            loss->SetLossDb(80.0);
        });
        Simulator::Schedule(Seconds(1.7), [&]() { b = rs.GetUeRxPackets(0); });
        Simulator::Schedule(Seconds(2.5), [&]() {
            c = rs.GetUeRxPackets(0);
            loss->SetLossDb(0.0);
        });
        Simulator::Stop(Seconds(4.0));
        Simulator::Run();
        const uint64_t d = rs.GetUeRxPackets(0);
        NS_TEST_ASSERT_MSG_GT(a, 80u, "URLLC flows before the outage (10 ms cadence)");
        NS_TEST_ASSERT_MSG_LT(c - b, 3u, "nothing gets through 80 dB of excess loss");
        NS_TEST_ASSERT_MSG_GT(d - c, 60u, "delivery resumes after the outage");
        Simulator::Destroy();
    }
};

/// Wave 11 SSB beam grid on a moving satellite. The satellite flies at
/// 7.56 km/s across a fixed UE for 6 s (45 km of track). The UE's best SSB
/// index changes as the beams sweep past (the grid is fixed in the satellite
/// body frame), and with small-scale fading off the serving SS-RSRP from the
/// best beam never falls 3 dB below its running maximum (adjacent beams cross
/// at their 3 dB contours; TS 38.213 4.1 sweep, TS 38.331 5.5.3.3 best-beam
/// cell quality).
class SsbGridAlongPassTest : public TestCase
{
  public:
    SsbGridAlongPassTest()
        : TestCase("Wave 11: best SSB beam tracks a moving satellite; ripple below the 3 dB contour")
    {
    }

  private:
    void DoRun() override
    {
        const double re = 6371.0e3;
        const double phi = 120.0e3 / re;
        NodeContainer sats;
        sats.Create(1);
        auto sm = CreateObject<ConstantVelocityMobilityModel>();
        sm->SetPosition(Vector(re + 600.0e3, -22.0e3, 0.0));
        sm->SetVelocity(Vector(0.0, 7560.0, 0.0));
        sats.Get(0)->AggregateObject(sm);
        NodeContainer ues;
        ues.Create(1);
        auto um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(Vector(re * std::cos(phi), 0.0, re * std::sin(phi)));
        ues.Get(0)->AggregateObject(um);
        NtnRealStackHelper rs;
        NrSet1(rs, 6.5, CreateTempDirFilename("ssbpass"));
        // Small-scale fading off: with it on, a common fade of 6 to 8 dB moves
        // every beam's SS-RSRP together (measured), which says nothing about
        // the grid. The grid's own ripple is the quantity under test.
        rs.SetSmallScaleFading(false);
        rs.SetPeriodicMeasurementReports(true, MilliSeconds(240));
        rs.SetSsbCellBeam(true, nullptr);
        rs.Build(sats, ues);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::UrllcPings, Seconds(0.5), Seconds(6.4));
        Ptr<NrUePhy> phy = DynamicCast<NrUeNetDevice>(rs.GetUeDevices().Get(0))->GetPhy(0);
        std::set<uint8_t> beams;
        double maxRsrp = -1e9;
        double worstDrop = 0.0;
        uint32_t samples = 0;
        for (int k = 4; k <= 64; ++k)
        {
            Simulator::Schedule(MilliSeconds(100 * k), [&]() {
                const uint8_t best = phy->GetBestSsbIndex(phy->GetCellId());
                const double rsrp = rs.GetUeServingRsrpDbm(0);
                if (best != 255 && std::isfinite(rsrp))
                {
                    beams.insert(best);
                    maxRsrp = std::max(maxRsrp, rsrp);
                    worstDrop = std::max(worstDrop, maxRsrp - rsrp);
                    ++samples;
                }
            });
        }
        Simulator::Stop(Seconds(6.5));
        Simulator::Run();
        NS_TEST_ASSERT_MSG_GT(rs.GetSsbBeamGridSize(), 19u, "a beam grid is in use");
        NS_TEST_ASSERT_MSG_GT(samples, 40u, "best-beam samples along the pass");
        NS_TEST_ASSERT_MSG_GT(beams.size(), 1u, "the best SSB beam changes as the satellite moves");
        // Before nr patch 21 periods that missed the best beam in the sweep
        // reported the cell 6 to 8 dB low about once a second.
        NS_TEST_ASSERT_MSG_LT(worstDrop, 3.0,
                              "cell quality stays within the 3 dB beam-crossing contour");
        NS_TEST_ASSERT_MSG_GT(rs.GetUeRxPackets(0), 400u, "the UE is served throughout");
        Simulator::Destroy();
    }
};

/// Determinism of the whole measured stack: the same RngRun reproduces the
/// packet counts and the mean SINR exactly; another run stays within physical
/// bounds (same order of delivered packets, SINR within 2 dB).
class RealStackDeterminismTest : public TestCase
{
  public:
    RealStackDeterminismTest()
        : TestCase("Determinism: same RngRun identical packets and SINR, other run within bounds")
    {
    }

  private:
    struct Out
    {
        uint64_t rx;
        uint64_t bytes;
        double sinr;
    };

    Out Run(uint32_t run)
    {
        RngSeedManager::SetSeed(1);
        RngSeedManager::SetRun(run);
        RngSeedManager::ResetNextStreamIndex();
        NodeContainer sats;
        NodeContainer ues;
        Row(sats, ues, 1, 0.0);
        NtnRealStackHelper rs;
        NrSet1(rs, 2.0, CreateTempDirFilename("det"));
        rs.SetEmbbRate(DataRate("10Mb/s"));
        rs.Build(sats, ues);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming, Seconds(0.5), Seconds(1.95));
        Simulator::Stop(Seconds(2.0));
        Simulator::Run();
        rs.Collect();
        Out o{rs.GetUeRxPackets(0), rs.GetUeRxBytes(0), rs.GetMeanDlSinrDb()};
        Simulator::Destroy();
        return o;
    }

    void DoRun() override
    {
        const Out a = Run(1);
        const Out b = Run(1);
        const Out c = Run(9);
        RngSeedManager::SetRun(1);
        NS_TEST_ASSERT_MSG_EQ(a.rx, b.rx, "identical packets");
        NS_TEST_ASSERT_MSG_EQ(a.bytes, b.bytes, "identical bytes");
        NS_TEST_ASSERT_MSG_EQ(a.sinr, b.sinr, "identical SINR");
        NS_TEST_ASSERT_MSG_EQ_TOL(1.0 * c.rx, 1.0 * a.rx, 0.1 * a.rx, "another run: packets within 10 %");
        NS_TEST_ASSERT_MSG_EQ_TOL(c.sinr, a.sinr, 2.0, "and SINR within 2 dB");
    }
};

/// Wave 9 paired FDD against TDD on the measured plane (TS 38.101-5 n256
/// paired spectrum vs one shared carrier). A saturating downlink with a
/// concurrent 128 kb/s uplink per UE: under FDD the downlink keeps its whole
/// carrier whatever the uplink does (DL throughput with and without the uplink
/// within 10 %), the uplink packets are delivered on the paired carrier, and
/// the downlink throughput is at least that of the shared TDD carrier.
class FddVersusTddMeasuredTest : public TestCase
{
  public:
    FddVersusTddMeasuredTest()
        : TestCase("Wave 9: FDD downlink unaffected by uplink load; at least the TDD throughput")
    {
    }

  private:
    struct Out
    {
        double dlMbps{0.0};
        bool fdd{false};
        uint64_t ulRx{0};
    };

    Out Run(bool fdd, bool uplink)
    {
        RngSeedManager::ResetNextStreamIndex();
        NodeContainer sats;
        NodeContainer ues;
        Row(sats, ues, 1, 0.0);
        NtnRealStackHelper rs;
        NrSet1(rs, 2.0, CreateTempDirFilename("duplex"));
        rs.SetDuplexMode(fdd ? NtnRealStackHelper::DuplexMode::Fdd : NtnRealStackHelper::DuplexMode::Tdd);
        rs.SetUplink(uplink);
        rs.SetEmbbRate(DataRate("150Mb/s"));
        rs.Build(sats, ues);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming, Seconds(0.5), Seconds(1.95));
        Simulator::Stop(Seconds(2.0));
        Simulator::Run();
        rs.Collect();
        Out o;
        o.dlMbps = rs.GetRxThroughputMbps();
        o.fdd = rs.IsFddActive();
        for (uint32_t a = 0; a < rs.GetRemoteHost()->GetNApplications(); ++a)
        {
            auto sink = DynamicCast<NtnOranSink>(rs.GetRemoteHost()->GetApplication(a));
            if (sink)
            {
                o.ulRx += sink->GetTotalRx();
            }
        }
        Simulator::Destroy();
        return o;
    }

    void DoRun() override
    {
        const Out fddDl = Run(true, false);
        const Out fddUl = Run(true, true);
        const Out tddUl = Run(false, true);
        std::cout << "  [duplex] FDD DL-only " << fddDl.dlMbps << " Mb/s, FDD +UL " << fddUl.dlMbps
                  << " Mb/s (UL bytes " << fddUl.ulRx << "), TDD +UL " << tddUl.dlMbps << " Mb/s\n";
        NS_TEST_ASSERT_MSG_EQ(fddDl.fdd && fddUl.fdd, true, "paired FDD built");
        NS_TEST_ASSERT_MSG_EQ(tddUl.fdd, false, "TDD when asked");
        NS_TEST_ASSERT_MSG_GT(fddDl.dlMbps, 1.0, "the downlink carries traffic");
        NS_TEST_ASSERT_MSG_EQ_TOL(fddUl.dlMbps, fddDl.dlMbps, 0.1 * fddDl.dlMbps,
                                  "FDD: the uplink does not take downlink capacity");
        // 256 B every 16 ms for 1.45 s is 90 packets; the first ones wait for the
        // UE's first grant. Measured: 63 delivered (16128 bytes).
        NS_TEST_ASSERT_MSG_GT(fddUl.ulRx, 40u * 256u, "uplink delivered on the paired carrier");
        NS_TEST_ASSERT_MSG_GT(fddUl.dlMbps, 0.95 * tddUl.dlMbps,
                              "FDD downlink at least the shared-carrier TDD downlink");
    }
};

class NtnRealStackDepthTestSuite : public TestSuite
{
  public:
    NtnRealStackDepthTestSuite()
        : TestSuite("ntn-real-stack-depth", Type::UNIT)
    {
        AddTestCase(new FddBandEdgesTest, TestCase::Duration::QUICK);
        AddTestCase(new FrequencyReuseThreeCellsTest, TestCase::Duration::QUICK);
        AddTestCase(new ShannonBoundUnderOverloadTest, TestCase::Duration::QUICK);
        AddTestCase(new LinkOutageRecoveryTest, TestCase::Duration::QUICK);
        AddTestCase(new SsbGridAlongPassTest, TestCase::Duration::QUICK);
        AddTestCase(new RealStackDeterminismTest, TestCase::Duration::QUICK);
        AddTestCase(new FddVersusTddMeasuredTest, TestCase::Duration::QUICK);
    }
};

static NtnRealStackDepthTestSuite g_ntnRealStackDepthTestSuite;
