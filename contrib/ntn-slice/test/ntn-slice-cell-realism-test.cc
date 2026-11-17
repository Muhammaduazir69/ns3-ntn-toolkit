/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * Wave 7C: link budget and feeder of the shared slice cell, end to end.
 *
 * The slice examples declared 70 dBm CONDUCTED at the satellite array input
 * (about 83 to 88 dBm radiated with the UPA gain) into TR 38.811 handheld UEs,
 * and a constant 5 ms backhaul. Measured handheld SINR sat above 30 dB, no
 * packet was ever lost over a pass, and the one-way delay did not follow the
 * satellite. These tests run the same cell (SGP4 satellite, TR 38.811 UEs,
 * 5G-LENA through NtnRealStackHelper, MixedBouquet traffic) under
 * Simulator::Run with the fixed configuration and with the old one.
 */
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-slice-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/test.h"
#include "ns3/walker-constellation.h"

#include <cmath>

using namespace ns3;
using namespace ns3::ntnslice;

namespace
{

/// TR 38.821 Table 6.1.1.1-1 Set-1 S band: 34 dBW/MHz over 20 MHz = 77.01 dBm.
constexpr double kSet1EirpDbm = 34.0 + 13.0103 + 30.0;
constexpr double kAltKm = 550.0;

struct CellRun
{
    bool set1Applied{false};
    bool liveFeeder{false};
    double effEirpDbm{0.0};
    double sinrDb{0.0};
    uint64_t rxPkts{0};
    double backhaulEarlyMs{0.0};
    double backhaulLateMs{0.0};
};

double
BackhaulMs(const NtnRealStackHelper& rs)
{
    TimeValue d;
    rs.GetBackhaulChannel()->GetAttribute("Delay", d);
    return d.Get().GetSeconds() * 1e3;
}

/// One 4 s run of the slice cell. \p satPowerDbm and \p gwKm go through the
/// same NtnSliceHelper calls the examples make.
CellRun
RunCell(double satPowerDbm, double gwKm)
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
    Config::SetDefault("ns3::NrHelper::UseIdealRrc", BooleanValue(true));
    NodeContainer sat;
    sat.Create(1);
    NodeContainer ues;
    ues.Create(3);
    ns3::ntncon::WalkerConfig w;
    w.num_planes = 1;
    w.total_sats = 80;
    w.altitude_km = kAltKm;
    w.inclination_deg = 53.0;
    w.epoch_unix_s = 1735689600.0;
    auto mob = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    mob->SetElementsSgp4(ns3::ntncon::WalkerConstellation::BuildDelta(w)[0]);
    sat.Get(0)->AggregateObject(mob);
    double lat;
    double lon;
    double alt;
    mob->GetGeodetic(lat, lon, alt);
    NtnTr38811MobilityHelper um(1);
    um.Install(ues, NtnMobilityScenarios::MixedContinental(), lat - 0.03, lat + 0.03, lon - 0.03,
               lon + 0.03);

    CellRun r;
    NtnRealStackHelper rs;
    rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
    rs.SetNumerology(1);
    rs.SetSimTime(Seconds(4));
    r.set1Applied = NtnSliceHelper::ConfigureCellLinkBudget(rs, satPowerDbm, false);
    rs.SetBackhaulDelay(MilliSeconds(5));
    rs.Build(sat, ues);
    r.liveFeeder = NtnSliceHelper::ConfigureFeeder(rs, lat, lon, gwKm) != nullptr;
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet, Seconds(1.0),
                      Seconds(3.9));
    Simulator::Schedule(Seconds(1.05), [&r, &rs]() { r.backhaulEarlyMs = BackhaulMs(rs); });
    Simulator::Schedule(Seconds(3.95), [&r, &rs]() { r.backhaulLateMs = BackhaulMs(rs); });
    Simulator::Stop(Seconds(4));
    Simulator::Run();
    rs.Collect();
    r.effEirpDbm = rs.GetEffectiveEirpDbm();
    r.sinrDb = rs.GetMeanDlSinrDb();
    for (uint32_t u = 0; u < ues.GetN(); ++u)
    {
        r.rxPkts += rs.GetUeRxPackets(u);
    }
    Simulator::Destroy();
    return r;
}

const CellRun&
FixedRun()
{
    static const CellRun r = RunCell(-1.0, 500.0);
    return r;
}

const CellRun&
LegacyRun()
{
    static const CellRun r = RunCell(70.0, 0.0);
    return r;
}

/// The default slice cell radiates TR 38.821 Set-1 and a handheld sees a
/// realistic SINR; the old 70 dBm conducted default is detectably too hot.
class SliceCellSet1LinkBudgetTest : public TestCase
{
  public:
    SliceCellSet1LinkBudgetTest()
        : TestCase("Wave 7C: slice cell radiates TR 38.821 Set-1 into a handheld (Simulator::Run)")
    {
    }

  private:
    void DoRun() override
    {
        const CellRun& fx = FixedRun();
        NS_TEST_ASSERT_MSG_EQ(fx.set1Applied, true, "-1 must select the Set-1 budget on nr");
        NS_TEST_ASSERT_MSG_EQ_TOL(fx.effEirpDbm, kSet1EirpDbm, 1.0,
                                  "effective EIRP must be the Set-1 S-band payload");
        NS_TEST_ASSERT_MSG_GT(fx.rxPkts, 100u, "the Set-1 cell must still carry traffic");
        NS_TEST_ASSERT_MSG_LT(fx.sinrDb, 25.0,
                              "a TR 38.811 handheld under a Set-1 payload must not see more than "
                              "25 dB of SINR");
        NS_TEST_ASSERT_MSG_GT(fx.sinrDb, 0.0, "the near-zenith Set-1 link must close");

        // Failure path: the old explicit 70 dBm conducted budget.
        const CellRun& lg = LegacyRun();
        NS_TEST_ASSERT_MSG_EQ(lg.set1Applied, false, "an explicit power is conducted, not Set-1");
        NS_TEST_ASSERT_MSG_GT(lg.effEirpDbm, kSet1EirpDbm + 3.0,
                              "70 dBm conducted plus the array gain overshoots Set-1");
        NS_TEST_ASSERT_MSG_GT(lg.sinrDb - fx.sinrDb, 3.0,
                              "the hot budget is what lifted the handheld SINR");
    }
};

/// With a gateway the backhaul follows the satellite-gateway range; without
/// one it stays the 5 ms constant plus the folded service leg.
class SliceCellFeederTracksGeometryTest : public TestCase
{
  public:
    SliceCellFeederTracksGeometryTest()
        : TestCase("Wave 7C: slice cell feeder delay follows the SGP4 geometry (Simulator::Run)")
    {
    }

  private:
    void DoRun() override
    {
        const double legMs = kAltKm * 1e3 / 299792458.0 * 1e3;
        const CellRun& fx = FixedRun();
        NS_TEST_ASSERT_MSG_EQ(fx.liveFeeder, true, "gwDistanceKm > 0 must wire a gateway");
        NS_TEST_ASSERT_MSG_GT(fx.backhaulEarlyMs, 2.0 * legMs - 0.01,
                              "feeder plus folded service link cannot be below 2 h / c");
        NS_TEST_ASSERT_MSG_LT(fx.backhaulEarlyMs, 20.0, "a 500 km gateway is not a GEO hop");
        NS_TEST_ASSERT_MSG_GT(std::abs(fx.backhaulLateMs - fx.backhaulEarlyMs), 1e-4,
                              "the delay must move with the satellite over 3 s");
        NS_TEST_ASSERT_MSG_EQ_TOL(NtnSliceHelper::OneWayFloorMs(kAltKm, true, 5.0), 2.0 * legMs,
                                  1e-9, "live-feeder floor is two altitude legs");

        // Failure path: gwDistanceKm <= 0 keeps the constant backhaul.
        const CellRun& lg = LegacyRun();
        NS_TEST_ASSERT_MSG_EQ(lg.liveFeeder, false, "gwDistanceKm <= 0 must not wire a gateway");
        // The channel carries the 5 ms constant plus the folded service link;
        // only the service leg moves, so the feeder geometry is absent.
        NS_TEST_ASSERT_MSG_GT(lg.backhaulEarlyMs, 5.0, "constant backhaul kept");
        NS_TEST_ASSERT_MSG_LT(lg.backhaulEarlyMs, 5.0 + legMs + 0.5,
                              "constant backhaul plus one folded service leg");
        NS_TEST_ASSERT_MSG_LT(std::abs(lg.backhaulLateMs - lg.backhaulEarlyMs),
                              std::abs(fx.backhaulLateMs - fx.backhaulEarlyMs),
                              "without a gateway the delay moves less: no feeder geometry");
        NS_TEST_ASSERT_MSG_EQ_TOL(NtnSliceHelper::OneWayFloorMs(kAltKm, false, 5.0), legMs + 5.0,
                                  1e-9, "constant-backhaul floor");
    }
};

class NtnSliceCellRealismTestSuite : public TestSuite
{
  public:
    NtnSliceCellRealismTestSuite()
        : TestSuite("ntn-slice-cell-realism", Type::SYSTEM)
    {
        AddTestCase(new SliceCellSet1LinkBudgetTest, TestCase::Duration::QUICK);
        AddTestCase(new SliceCellFeederTracksGeometryTest, TestCase::Duration::QUICK);
    }
};

static NtnSliceCellRealismTestSuite g_ntnSliceCellRealismTestSuite;

} // namespace
