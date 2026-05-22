/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W5)
 */
#include "ns3/a2g-channel-tr36777.h"
#include "ns3/aeronautical-scenario.h"
#include "ns3/box.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/double.h"
#include "ns3/haps-mobility-model.h"
#include "ns3/multi-layer-router.h"
#include "ns3/ais-maritime-trace.h"
#include "ns3/ais-mobility-model.h"
#include "ns3/opensky-adsb-trace.h"
#include "ns3/opensky-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/uav-mobility-models.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <vector>

using namespace ns3;

namespace
{

class HapsAltitudeStableTest : public TestCase
{
  public:
    HapsAltitudeStableTest()
        : TestCase("HAPS holds altitude within ±50 m for 1 hour")
    {
    }

    void DoRun() override
    {
        Ptr<HapsMobilityModel> haps = CreateObject<HapsMobilityModel>();
        haps->SetAttribute("Altitude", DoubleValue(20000.0));
        haps->SetAttribute("MaxVerticalDeviation", DoubleValue(50.0));
        haps->SetCenter(Vector{0, 0, 0});

        double minAlt = 1e12, maxAlt = -1e12;
        for (double t = 0; t <= 3600.0; t += 60.0)
        {
            Simulator::Schedule(Seconds(t), [&minAlt, &maxAlt, haps]() {
                double z = haps->GetPosition().z;
                if (z < minAlt) minAlt = z;
                if (z > maxAlt) maxAlt = z;
            });
        }
        Simulator::Stop(Seconds(3601));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_GT_OR_EQ(minAlt, 19950.0, "HAPS dipped below 19 950 m");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(maxAlt, 20050.0, "HAPS exceeded 20 050 m");
    }
};

class UavPatrolReturnsToStartTest : public TestCase
{
  public:
    UavPatrolReturnsToStartTest()
        : TestCase("Patrol UAV returns to start point each cycle")
    {
    }

    void DoRun() override
    {
        Ptr<UavPatrolMobilityModel> uav = CreateObject<UavPatrolMobilityModel>();
        uav->SetAttribute("Speed", DoubleValue(25.0));
        Vector a{-500, 0, 100};
        Vector b{500, 0, 100};
        uav->SetEndpoints(a, b);

        Vector finalPos;
        // 1000 m at 25 m/s + 1 s pause + 1000 m + 1 s pause = ~82 s per round-trip
        Simulator::Schedule(Seconds(82.5), [&finalPos, uav]() {
            finalPos = uav->GetPosition();
        });
        Simulator::Stop(Seconds(83));
        Simulator::Run();
        Simulator::Destroy();

        // After one full round-trip we should be near @c a again (within speed*tickStep).
        Vector d{finalPos.x - a.x, finalPos.y - a.y, finalPos.z - a.z};
        double dist = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        NS_TEST_ASSERT_MSG_LT(dist, 50.0,
                              "UAV not within 50 m of start after round-trip "
                              "(d=" << dist << " m)");
    }
};

class A2gPathLossSpotCheckTest : public TestCase
{
  public:
    A2gPathLossSpotCheckTest()
        : TestCase("TR 36.777 PL spot-checks within ±2 dB of spec")
    {
    }

    void DoRun() override
    {
        // Spot check: RMa-AV LOS, h_UT = 50 m, d3D = 1 km, fc = 2 GHz.
        // Slope: max(23.9 - 1.8*log10(50), 20) = max(23.9 - 3.06, 20) ≈ 20.84
        // PL_LOS = 28 + 20.84*log10(1000) + 20*log10(2)
        //        = 28 + 62.52 + 6.02 = 96.54 dB
        double pl = A2gChannelTr36777::PathLossDb(
            A2gScenario::RMa_AV, A2gLink::LOS, 1000.0, 2.0, 50.0);
        NS_TEST_ASSERT_MSG_LT(std::abs(pl - 96.54), 2.0,
                              "PL_LOS RMa-AV out of spec (got " << pl << ")");

        // UMa-AV LOS, h_UT = 100 m, d3D = 500 m, fc = 2 GHz.
        // PL_LOS = 28 + 22*log10(500) + 20*log10(2) = 28 + 59.42 + 6.02 = 93.44 dB
        pl = A2gChannelTr36777::PathLossDb(
            A2gScenario::UMa_AV, A2gLink::LOS, 500.0, 2.0, 100.0);
        NS_TEST_ASSERT_MSG_LT(std::abs(pl - 93.44), 2.0,
                              "PL_LOS UMa-AV out of spec (got " << pl << ")");

        // UMi-AV LOS, h_UT = 50 m, d3D = 200 m, fc = 2 GHz.
        // PL_LOS = 30.9 + 22.25*log10(200) + 20*log10(2)
        //        = 30.9 + 51.20 + 6.02 = 88.12 dB
        pl = A2gChannelTr36777::PathLossDb(
            A2gScenario::UMi_AV, A2gLink::LOS, 200.0, 2.0, 50.0);
        NS_TEST_ASSERT_MSG_LT(std::abs(pl - 88.12), 2.0,
                              "PL_LOS UMi-AV out of spec (got " << pl << ")");

        // NLOS PL must be greater than LOS PL at same geometry.
        double plLos = A2gChannelTr36777::PathLossDb(
            A2gScenario::UMa_AV, A2gLink::LOS, 200.0, 2.0, 50.0);
        double plNlos = A2gChannelTr36777::PathLossDb(
            A2gScenario::UMa_AV, A2gLink::NLOS, 200.0, 2.0, 50.0);
        NS_TEST_ASSERT_MSG_GT(plNlos, plLos,
                              "NLOS PL must exceed LOS PL");
    }
};

class A2gLosProbabilityMonotonicTest : public TestCase
{
  public:
    A2gLosProbabilityMonotonicTest()
        : TestCase("LOS probability rises with UAV altitude")
    {
    }

    void DoRun() override
    {
        // At a fixed d2D, LOS prob must rise (or stay) with altitude.
        double prev = -1.0;
        for (double h : {1.5, 10.0, 25.0, 50.0, 100.0, 200.0})
        {
            double p = A2gChannelTr36777::LosProbability(
                A2gScenario::UMa_AV, /*d2dM=*/200.0, h);
            NS_TEST_ASSERT_MSG_GT_OR_EQ(p, prev,
                                        "LOS prob non-monotonic in altitude");
            NS_TEST_ASSERT_MSG_LT_OR_EQ(p, 1.0001, "LOS prob > 1");
            prev = p;
        }
    }
};

class MultiLayerRouterConvergesTest : public TestCase
{
  public:
    MultiLayerRouterConvergesTest()
        : TestCase("Router emits a 4-layer path in <5 s wallclock")
    {
    }

    void DoRun() override
    {
        // 50 nodes per layer — exceeds realistic SAGIN sizing.
        Ptr<MultiLayerRouter> router = CreateObject<MultiLayerRouter>();
        for (int i = 0; i < 50; ++i)
        {
            Ptr<ConstantPositionMobilityModel> uav = CreateObject<ConstantPositionMobilityModel>();
            uav->SetPosition(Vector{100.0 * i, 0, 150.0});
            router->AddNode(SaginLayer::Uav, uav);

            Ptr<ConstantPositionMobilityModel> haps = CreateObject<ConstantPositionMobilityModel>();
            haps->SetPosition(Vector{1000.0 * i, 0, 20000.0});
            router->AddNode(SaginLayer::Haps, haps);

            Ptr<ConstantPositionMobilityModel> leo = CreateObject<ConstantPositionMobilityModel>();
            leo->SetPosition(Vector{1.0e6 * i, 0, 550000.0});
            router->AddNode(SaginLayer::Leo, leo);
        }

        Ptr<ConstantPositionMobilityModel> source = CreateObject<ConstantPositionMobilityModel>();
        source->SetPosition(Vector{0, 0, 0});

        auto t0 = std::chrono::steady_clock::now();
        auto path = router->Route(source);
        auto dt = std::chrono::steady_clock::now() - t0;
        double secs = std::chrono::duration<double>(dt).count();

        NS_TEST_ASSERT_MSG_EQ(path.size(), 4u,
                              "expected 4-layer path, got " << path.size());
        NS_TEST_ASSERT_MSG_LT(secs, 5.0,
                              "route() took " << secs << " s, > 5 s gate");
    }
};

class AeronauticalReachesArrivalTest : public TestCase
{
  public:
    AeronauticalReachesArrivalTest()
        : TestCase("Aircraft reaches arrival point within expected ETA")
    {
    }

    void DoRun() override
    {
        Ptr<AeronauticalMobilityModel> ac = CreateObject<AeronauticalMobilityModel>();
        Vector dep{0, 0, 0};
        Vector arr{250000, 0, 0};   // 250 km
        ac->SetFlightPlan(dep, arr, 11000.0, 250.0);
        // ETA = 250 km / 250 m/s = 1000 s

        Vector finalPos;
        Simulator::Schedule(Seconds(1010), [&finalPos, ac]() {
            finalPos = ac->GetPosition();
        });
        Simulator::Stop(Seconds(1020));
        Simulator::Run();
        Simulator::Destroy();

        Vector d{finalPos.x - arr.x, finalPos.y - arr.y, finalPos.z - 11000.0};
        double dist = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        NS_TEST_ASSERT_MSG_LT(dist, 100.0,
                              "Aircraft not at arrival after ETA (d=" << dist << " m)");
    }
};

// ============================================================================
// Roadmap §4.4.1: OpenSky ADS-B trace importer + replay mobility model
// ============================================================================

class OpenSkyImporterParseTest : public TestCase
{
  public:
    OpenSkyImporterParseTest()
        : TestCase("OpenSky importer parses CSV header + rows for multiple aircraft")
    {
    }

  private:
    void DoRun() override
    {
        sagin::OpenSkyAdsbImporter imp;
        const std::string candidates[] = {
            "contrib/ntn-sagin/data/opensky-sample-trace.csv",
            "/home/uzair/6g_ntn_ns3/ns-3-dev/contrib/ntn-sagin/data/"
            "opensky-sample-trace.csv",
        };
        std::map<std::string, sagin::OpenSkyAdsbTrace> traces;
        for (const auto& p : candidates)
        {
            traces = imp.LoadCsv(p);
            if (!traces.empty())
                break;
        }
        NS_TEST_ASSERT_MSG_GT(traces.size(), 0u,
                              "bundled OpenSky sample not loaded");
        // Sample dump has 3 icao24 codes.
        NS_TEST_ASSERT_MSG_EQ(traces.size(), 3u, "3 aircraft expected");
        NS_TEST_ASSERT_MSG_EQ(traces.count("4ca7b7"), 1u, "DLH123 present");
        NS_TEST_ASSERT_MSG_EQ(traces.count("abc123"), 1u, "BAW456 present");
        NS_TEST_ASSERT_MSG_EQ(traces.count("ground42"), 1u, "ground aircraft");

        const auto& dlh = traces.at("4ca7b7");
        NS_TEST_ASSERT_MSG_EQ(dlh.samples.size(), 7u, "DLH123 has 7 samples");
        // First sample must match the CSV (lat=52, lon=9, alt=1500).
        NS_TEST_ASSERT_MSG_EQ_TOL(dlh.samples.front().lat_deg, 52.0, 1e-9,
                                  "first lat");
        NS_TEST_ASSERT_MSG_EQ_TOL(dlh.samples.front().lon_deg, 9.0, 1e-9,
                                  "first lon");
        NS_TEST_ASSERT_MSG_EQ_TOL(dlh.samples.front().alt_m, 1500.0, 1e-6,
                                  "first alt (baro)");
        // Last sample: lon should have moved ~ 0.162° east.
        NS_TEST_ASSERT_MSG_GT(dlh.samples.back().lon_deg,
                              dlh.samples.front().lon_deg,
                              "longitude advances east");
        NS_TEST_ASSERT_MSG_GT(dlh.samples.back().alt_m,
                              dlh.samples.front().alt_m,
                              "aircraft climbed during trace");

        // Ground aircraft: onground=true => alt clamped to 0 m.
        const auto& gr = traces.at("ground42");
        for (const auto& s : gr.samples)
        {
            NS_TEST_ASSERT_MSG_EQ(s.alt_m, 0.0,
                                  "onground => alt clamped to 0");
        }
    }
};

class OpenSkyImporterMalformedTest : public TestCase
{
  public:
    OpenSkyImporterMalformedTest()
        : TestCase("OpenSky importer skips malformed rows and counts them")
    {
    }

  private:
    void DoRun() override
    {
        const std::string path = "/tmp/opensky-test-bad.csv";
        std::ofstream f(path);
        f << "time,icao24,lat,lon,velocity,heading,vertrate,callsign,"
             "onground,alert,spi,squawk,baroaltitude,geoaltitude,"
             "lastposupdate,lastcontact\n";
        f << "1572912000,abc,52.0,9.0,200,90,0,X,false,false,false,0,1000,1000,0,0\n";
        f << ",,,bad,row,here\n"; // malformed
        f << "1572912010,abc,52.001,9.0,200,90,0,X,false,false,false,0,1000,1000,0,0\n";
        f << "1572912020,,no_icao,,,,,,,,,,,,\n"; // empty icao
        f << "1572912030,def,not_a_number,9.0,200,90,0,X,false,false,false,0,1000,1000,0,0\n";
        f.close();

        sagin::OpenSkyAdsbImporter imp;
        auto traces = imp.LoadCsv(path);
        NS_TEST_ASSERT_MSG_EQ(traces.size(), 1u,
                              "only the 'abc' aircraft is valid");
        NS_TEST_ASSERT_MSG_EQ(traces.at("abc").samples.size(), 2u,
                              "2 valid samples for abc");
        NS_TEST_ASSERT_MSG_GT(imp.LastRowsSkipped(), 0u, "skip counter > 0");
        std::remove(path.c_str());
    }
};

class OpenSkyTraceInterpolationTest : public TestCase
{
  public:
    OpenSkyTraceInterpolationTest()
        : TestCase("OpenSky trace InterpolateAt linearly interpolates lat lon alt")
    {
    }

  private:
    void DoRun() override
    {
        sagin::OpenSkyAdsbTrace tr;
        tr.icao24 = "tst";
        tr.samples.push_back({0.0, 52.0, 9.0, 1000.0, 100.0, 90.0});
        tr.samples.push_back({10.0, 52.01, 9.01, 1500.0, 200.0, 90.0});

        // Midpoint should give average values.
        auto mid = tr.InterpolateAt(5.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(mid.lat_deg, 52.005, 1e-9, "lat mid");
        NS_TEST_ASSERT_MSG_EQ_TOL(mid.lon_deg, 9.005, 1e-9, "lon mid");
        NS_TEST_ASSERT_MSG_EQ_TOL(mid.alt_m, 1250.0, 1e-9, "alt mid");
        NS_TEST_ASSERT_MSG_EQ_TOL(mid.velocity_mps, 150.0, 1e-9, "vel mid");

        // Below range -> first sample.
        auto before = tr.InterpolateAt(-5.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(before.lat_deg, 52.0, 1e-9, "clamp low");
        // Above range -> last sample.
        auto after = tr.InterpolateAt(20.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(after.lat_deg, 52.01, 1e-9, "clamp high");

        // Heading interpolation wraps around 0/360.
        tr.samples.clear();
        tr.samples.push_back({0.0, 0.0, 0.0, 0.0, 100.0, 350.0});
        tr.samples.push_back({10.0, 0.0, 0.0, 0.0, 100.0, 10.0});
        // Midpoint heading should be 0 (or 360 - either is fine).
        auto h = tr.InterpolateAt(5.0);
        const bool headingNearZero =
            (h.heading_deg < 5.0) || (h.heading_deg > 355.0);
        NS_TEST_ASSERT_MSG_EQ(headingNearZero,
                              true,
                              "heading interpolates around 0/360");
    }
};

namespace
{

struct TraceSample
{
    double t_s;
    double east_m;
    double north_m;
    double up_m;
    double speed_mps;
};

void
SampleOpenSkyModel(Ptr<sagin::OpenSkyMobilityModel> mob,
                    std::vector<TraceSample>* out)
{
    Vector p = mob->GetPosition();
    Vector v = mob->GetVelocity();
    const double sp = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    out->push_back(
        {Simulator::Now().GetSeconds(), p.x, p.y, p.z, sp});
}

} // namespace

class OpenSkySimulatorTimeReplayTest : public TestCase
{
  public:
    OpenSkySimulatorTimeReplayTest()
        : TestCase("Simulator: 60 s OpenSky replay matches CSV trajectory in ENU")
    {
    }

  private:
    void DoRun() override
    {
        sagin::OpenSkyAdsbImporter imp;
        const std::string candidates[] = {
            "contrib/ntn-sagin/data/opensky-sample-trace.csv",
            "/home/uzair/6g_ntn_ns3/ns-3-dev/contrib/ntn-sagin/data/"
            "opensky-sample-trace.csv",
        };
        std::map<std::string, sagin::OpenSkyAdsbTrace> traces;
        for (const auto& p : candidates)
        {
            traces = imp.LoadCsv(p);
            if (!traces.empty())
                break;
        }
        NS_TEST_ASSERT_MSG_GT(traces.size(), 0u, "sample trace loaded");
        const auto& dlh = traces.at("4ca7b7");

        Ptr<sagin::OpenSkyMobilityModel> mob =
            CreateObject<sagin::OpenSkyMobilityModel>();
        mob->SetTrace(dlh);
        // Reference at the trace start (lat=52, lon=9, alt=1500).
        mob->SetReference(52.0, 9.0, 1500.0);
        mob->SetTraceTimeOffsetSeconds(dlh.TStart());

        std::vector<TraceSample> samples;
        for (int t = 0; t <= 60; t += 5)
        {
            Simulator::Schedule(Seconds(t), &SampleOpenSkyModel, mob,
                                &samples);
        }
        Simulator::Stop(Seconds(61));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(samples.size(), 13u, "13 samples over 60 s");

        // At t=0, position must be exactly at the reference origin.
        NS_TEST_ASSERT_MSG_EQ_TOL(samples.front().east_m, 0.0, 1.0,
                                  "t=0 east≈0");
        NS_TEST_ASSERT_MSG_EQ_TOL(samples.front().north_m, 0.0, 1.0,
                                  "t=0 north≈0");
        NS_TEST_ASSERT_MSG_EQ_TOL(samples.front().up_m, 0.0, 1.0,
                                  "t=0 up≈0");

        // Monotonic east (aircraft heading 90°, longitude increases).
        for (size_t i = 1; i < samples.size(); ++i)
        {
            NS_TEST_ASSERT_MSG_GT(samples[i].east_m,
                                  samples[i - 1].east_m,
                                  "east increases as aircraft flies E");
        }

        // Vertical climb during the first 30 s.
        NS_TEST_ASSERT_MSG_GT(samples[6].up_m, samples[0].up_m,
                              "altitude climbed by t=30 s");

        // Speed in the trace is 180..250 m/s. ENU velocity magnitude must
        // land in that range.
        for (const auto& s : samples)
        {
            NS_TEST_ASSERT_MSG_GT(s.speed_mps, 100.0,
                                  "speed > 100 m/s during cruise");
            NS_TEST_ASSERT_MSG_LT(s.speed_mps, 400.0,
                                  "speed < 400 m/s during cruise");
        }

        // Past TEnd, position clamps to the last sample.
        Simulator::Destroy();
    }
};

// ============================================================================
// Roadmap §4.4.2: AIS maritime trace importer + replay mobility model
// ============================================================================

class AisImporterParseTest : public TestCase
{
  public:
    AisImporterParseTest()
        : TestCase("AIS Danish importer parses CSV header + rows for multiple MMSIs")
    {
    }

  private:
    void DoRun() override
    {
        sagin::AisDanishImporter imp;
        const std::string candidates[] = {
            "contrib/ntn-sagin/data/ais-sample-trace.csv",
            "/home/uzair/6g_ntn_ns3/ns-3-dev/contrib/ntn-sagin/data/"
            "ais-sample-trace.csv",
        };
        std::map<uint32_t, sagin::AisMaritimeTrace> traces;
        for (const auto& p : candidates)
        {
            traces = imp.LoadCsv(p);
            if (!traces.empty())
                break;
        }
        NS_TEST_ASSERT_MSG_GT(traces.size(), 0u,
                              "bundled AIS sample not loaded");
        // 4 distinct MMSIs in the bundled sample.
        NS_TEST_ASSERT_MSG_EQ(traces.size(), 4u, "4 vessels expected");
        NS_TEST_ASSERT_MSG_EQ(traces.count(219015785u), 1u, "DLK cargo");
        NS_TEST_ASSERT_MSG_EQ(traces.count(257891234u), 1u, "OSL pleasure");
        NS_TEST_ASSERT_MSG_EQ(traces.count(538001234u), 1u, "PIRAEUS tanker");

        const auto& cargo = traces.at(219015785u);
        NS_TEST_ASSERT_MSG_EQ(cargo.samples.size(), 5u, "5 cargo samples");
        NS_TEST_ASSERT_MSG_EQ(cargo.ship_type, "Cargo", "ship type");
        // First sample at lat=55, lon=12, SOG=12, COG=90.
        const auto& s0 = cargo.samples.front();
        NS_TEST_ASSERT_MSG_EQ_TOL(s0.lat_deg, 55.0, 1e-9, "lat");
        NS_TEST_ASSERT_MSG_EQ_TOL(s0.lon_deg, 12.0, 1e-9, "lon");
        NS_TEST_ASSERT_MSG_EQ_TOL(s0.sog_knots, 12.0, 1e-9, "SOG");
        NS_TEST_ASSERT_MSG_EQ_TOL(s0.cog_deg, 90.0, 1e-9, "COG");
        // Heading 511 (not available) sentinel preserved on the base station.
        const auto& base = traces.at(2190001u);
        NS_TEST_ASSERT_MSG_EQ_TOL(base.samples.front().heading_deg, 511.0, 1e-9,
                                  "heading=511 sentinel preserved");

        // Last sample of the cargo vessel has moved east.
        NS_TEST_ASSERT_MSG_GT(cargo.samples.back().lon_deg,
                              cargo.samples.front().lon_deg,
                              "cargo moves east during trace");
    }
};

class AisImporterMalformedTest : public TestCase
{
  public:
    AisImporterMalformedTest()
        : TestCase("AIS importer skips malformed rows and counts them")
    {
    }

  private:
    void DoRun() override
    {
        const std::string path = "/tmp/ais-test-bad.csv";
        std::ofstream f(path);
        f << "Timestamp,Type of mobile,MMSI,Latitude,Longitude,Navigational "
             "status,ROT,SOG,COG,Heading,IMO,Callsign,Name,Ship type,Cargo "
             "type,Width,Length,Type of position fixing device,Draught,"
             "Destination,ETA,Data source type,A,B,C,D\n";
        f << "07/10/2019 00:00:00,Class A,123456789,55.0,12.0,Under way using "
             "engine,0,10,90,90,,,SHIP,Cargo,,,,GPS,0,,,AIS,,,,\n";
        f << ",,,bad,row,here\n";
        f << "07/10/2019 00:00:30,Class A,123456789,55.001,12.0,Under way using "
             "engine,0,10,90,90,,,SHIP,Cargo,,,,GPS,0,,,AIS,,,,\n";
        f << "07/10/2019 00:00:30,Class A,0,55.001,12.0,Under way using "
             "engine,0,10,90,90,,,SHIP,Cargo,,,,GPS,0,,,AIS,,,,\n"; // mmsi=0
        f << "garbage,Class A,234,not_a_lat,12.0,,,,,,,,,,,,,,,,,,,,,\n";
        f.close();

        sagin::AisDanishImporter imp;
        auto traces = imp.LoadCsv(path);
        NS_TEST_ASSERT_MSG_EQ(traces.size(), 1u,
                              "only MMSI 123456789 is valid");
        NS_TEST_ASSERT_MSG_EQ(traces.at(123456789u).samples.size(), 2u,
                              "2 valid samples");
        NS_TEST_ASSERT_MSG_GT(imp.LastRowsSkipped(), 0u, "skip counter > 0");
        std::remove(path.c_str());
    }
};

class AisTraceInterpolationTest : public TestCase
{
  public:
    AisTraceInterpolationTest()
        : TestCase("AIS trace InterpolateAt linearly interpolates lat lon sog cog")
    {
    }

  private:
    void DoRun() override
    {
        sagin::AisMaritimeTrace tr;
        tr.mmsi = 1234;
        tr.samples.push_back({0.0, 55.0, 12.0, 10.0, 90.0, 90.0,
                              "Cargo", "TST"});
        tr.samples.push_back({10.0, 55.001, 12.001, 12.0, 100.0, 100.0,
                              "Cargo", "TST"});

        auto mid = tr.InterpolateAt(5.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(mid.lat_deg, 55.0005, 1e-9, "lat mid");
        NS_TEST_ASSERT_MSG_EQ_TOL(mid.lon_deg, 12.0005, 1e-9, "lon mid");
        NS_TEST_ASSERT_MSG_EQ_TOL(mid.sog_knots, 11.0, 1e-9, "SOG mid");
        NS_TEST_ASSERT_MSG_EQ_TOL(mid.cog_deg, 95.0, 1e-9, "COG mid");

        // Below range -> first sample.
        auto before = tr.InterpolateAt(-5.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(before.lat_deg, 55.0, 1e-9, "clamp low");
        // Above range -> last sample.
        auto after = tr.InterpolateAt(20.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(after.lat_deg, 55.001, 1e-9, "clamp high");
    }
};

namespace
{

struct AisSimSample
{
    double t_s;
    double east_m;
    double north_m;
    double speed_mps;
};

void
SampleAisModel(Ptr<sagin::AisMobilityModel> mob,
                std::vector<AisSimSample>* out)
{
    Vector p = mob->GetPosition();
    Vector v = mob->GetVelocity();
    const double sp = std::sqrt(v.x * v.x + v.y * v.y);
    out->push_back({Simulator::Now().GetSeconds(), p.x, p.y, sp});
}

} // namespace

class AisSimulatorTimeReplayTest : public TestCase
{
  public:
    AisSimulatorTimeReplayTest()
        : TestCase("Simulator: 120 s AIS replay matches CSV trajectory at hull speed")
    {
    }

  private:
    void DoRun() override
    {
        sagin::AisDanishImporter imp;
        const std::string candidates[] = {
            "contrib/ntn-sagin/data/ais-sample-trace.csv",
            "/home/uzair/6g_ntn_ns3/ns-3-dev/contrib/ntn-sagin/data/"
            "ais-sample-trace.csv",
        };
        std::map<uint32_t, sagin::AisMaritimeTrace> traces;
        for (const auto& p : candidates)
        {
            traces = imp.LoadCsv(p);
            if (!traces.empty())
                break;
        }
        NS_TEST_ASSERT_MSG_GT(traces.size(), 0u, "sample loaded");
        const auto& cargo = traces.at(219015785u);

        Ptr<sagin::AisMobilityModel> mob =
            CreateObject<sagin::AisMobilityModel>();
        mob->SetTrace(cargo);
        mob->SetReference(55.0, 12.0);
        mob->SetTraceTimeOffsetSeconds(cargo.TStart());

        std::vector<AisSimSample> samples;
        for (int t = 0; t <= 120; t += 10)
        {
            Simulator::Schedule(Seconds(t), &SampleAisModel, mob, &samples);
        }
        Simulator::Stop(Seconds(121));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(samples.size(), 13u, "13 samples over 120 s");

        // t=0 at reference origin.
        NS_TEST_ASSERT_MSG_EQ_TOL(samples.front().east_m, 0.0, 1.0, "t=0 east");
        NS_TEST_ASSERT_MSG_EQ_TOL(samples.front().north_m, 0.0, 1.0, "t=0 north");

        // Hull speed at 12 knots = 6.17 m/s.
        const double expectedMps = 12.0 * 0.5144;
        for (const auto& s : samples)
        {
            NS_TEST_ASSERT_MSG_EQ_TOL(
                s.speed_mps,
                expectedMps,
                0.5,
                "speed should be ~12 knots = 6.17 m/s");
        }

        // Past the last sample (t > 120s of trace) position clamps to
        // the last sample's east. With the trace ending at t=120s and the
        // cargo at lon=12.0068 the east displacement is ~430 m.
        const auto& last = samples.back();
        NS_TEST_ASSERT_MSG_GT(last.east_m, 100.0,
                              "vessel travelled > 100 m east");
        NS_TEST_ASSERT_MSG_LT(std::abs(last.north_m), 5.0,
                              "vessel kept ~constant latitude");

        Simulator::Destroy();
    }
};

class NtnSaginTestSuite : public TestSuite
{
  public:
    NtnSaginTestSuite()
        : TestSuite("ntn-sagin", Type::UNIT)
    {
        AddTestCase(new HapsAltitudeStableTest, TestCase::Duration::QUICK);
        AddTestCase(new UavPatrolReturnsToStartTest, TestCase::Duration::QUICK);
        AddTestCase(new A2gPathLossSpotCheckTest, TestCase::Duration::QUICK);
        AddTestCase(new A2gLosProbabilityMonotonicTest, TestCase::Duration::QUICK);
        AddTestCase(new MultiLayerRouterConvergesTest, TestCase::Duration::QUICK);
        AddTestCase(new AeronauticalReachesArrivalTest, TestCase::Duration::QUICK);
        // Roadmap §4.4.1 — OpenSky ADS-B trace importer + replay mobility.
        AddTestCase(new OpenSkyImporterParseTest, TestCase::Duration::QUICK);
        AddTestCase(new OpenSkyImporterMalformedTest, TestCase::Duration::QUICK);
        AddTestCase(new OpenSkyTraceInterpolationTest, TestCase::Duration::QUICK);
        AddTestCase(new OpenSkySimulatorTimeReplayTest, TestCase::Duration::QUICK);
        // Roadmap §4.4.2 — AIS maritime trace importer + replay mobility.
        AddTestCase(new AisImporterParseTest, TestCase::Duration::QUICK);
        AddTestCase(new AisImporterMalformedTest, TestCase::Duration::QUICK);
        AddTestCase(new AisTraceInterpolationTest, TestCase::Duration::QUICK);
        AddTestCase(new AisSimulatorTimeReplayTest, TestCase::Duration::QUICK);
    }
};

static NtnSaginTestSuite g_ntnSaginTestSuite;

} // namespace
