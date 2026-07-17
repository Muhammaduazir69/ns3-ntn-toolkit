/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/ntn-influx-sink.h"
#include "ns3/ntn-metric-schema.h"
#include "ns3/ntn-netsimulyzer-exporter.h"
#include "ns3/ntn-observability-helper.h"
#include "ns3/ntn-repro-manifest.h"
#include "ns3/ntn-scene-recorder.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/uinteger.h"

#include <cstdio>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

using namespace ns3;
using namespace ns3::ntnobs;

namespace
{

std::string
ReadFile(const std::string& p)
{
    std::ifstream f(p);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

} // namespace

class LineProtocolBasicEncodeTest : public TestCase
{
  public:
    LineProtocolBasicEncodeTest()
        : TestCase("Line protocol encodes measurement, tags, fields, timestamp")
    {
    }

  private:
    void DoRun() override
    {
        Point p;
        p.measurement = measurement::kRadio;
        p.tags[tag::kCellId] = "C42";
        p.tags[tag::kUeImsi] = "100001";
        p.fieldsFloat[field::kRsrpDbm] = -97.5;
        p.fieldsFloat[field::kSinrDb] = 12.3;
        p.timestamp = NanoSeconds(1234567890LL);
        const std::string out = EncodeLineProtocol({p}, true);

        // Expected: ntn_radio,cell_id=C42,ue_imsi=100001 rsrp_dbm=-97.5,sinr_db=12.3 1234567890
        NS_TEST_EXPECT_MSG_NE(out.find("ntn_radio,"), std::string::npos, "measurement");
        NS_TEST_EXPECT_MSG_NE(out.find("cell_id=C42"), std::string::npos, "tag cell_id");
        NS_TEST_EXPECT_MSG_NE(out.find("ue_imsi=100001"), std::string::npos, "tag ue_imsi");
        NS_TEST_EXPECT_MSG_NE(out.find("rsrp_dbm=-97.5"), std::string::npos, "field rsrp");
        NS_TEST_EXPECT_MSG_NE(out.find("sinr_db=12.3"), std::string::npos, "field sinr");
        NS_TEST_EXPECT_MSG_NE(out.find("1234567890"), std::string::npos, "timestamp ns");
        NS_TEST_EXPECT_MSG_EQ(out.back(), '\n', "line must end with newline");
    }
};

class LineProtocolEscapeTest : public TestCase
{
  public:
    LineProtocolEscapeTest()
        : TestCase("Line protocol escapes commas spaces and equals in tags")
    {
    }

  private:
    void DoRun() override
    {
        Point p;
        p.measurement = "ntn radio with space";
        p.tags["key with space"] = "value,with,comma";
        p.tags["k=eq"] = "v=eq";
        p.fieldsInt["i"] = 7;
        p.timestamp = NanoSeconds(1);
        const std::string out = EncodeLineProtocol({p}, true);
        NS_TEST_EXPECT_MSG_NE(out.find(R"(ntn\ radio\ with\ space)"), std::string::npos,
                              "measurement space escape");
        NS_TEST_EXPECT_MSG_NE(out.find(R"(key\ with\ space=value\,with\,comma)"),
                              std::string::npos,
                              "tag space + comma escape");
        NS_TEST_EXPECT_MSG_NE(out.find(R"(k\=eq=v\=eq)"), std::string::npos, "tag = escape");
        NS_TEST_EXPECT_MSG_NE(out.find("i=7i"), std::string::npos, "int field suffix");
    }
};

class InfluxFileSinkRoundTripTest : public TestCase
{
  public:
    InfluxFileSinkRoundTripTest()
        : TestCase("File-mode sink writes points and Stop() flushes the buffer")
    {
    }

  private:
    void DoRun() override
    {
        const std::string path = "/tmp/ntn-observability-test.lp";
        std::remove(path.c_str());

        NtnObservabilityHelper helper;
        helper.SetRunId("unit-test");
        helper.SetInfluxFile(path);
        helper.SetFlushPeriod(MilliSeconds(50));
        Ptr<NtnInfluxSink> sink = helper.InstallInfluxSink();

        sink->Start();

        Simulator::Schedule(MilliSeconds(10), [sink]() {
            Point p;
            p.measurement = measurement::kRadio;
            p.tags[tag::kRunId] = "unit-test";
            p.fieldsFloat[field::kRsrpDbm] = -100.0;
            sink->Push(p);
        });
        Simulator::Schedule(MilliSeconds(120), [sink]() {
            Point p;
            p.measurement = measurement::kRadio;
            p.tags[tag::kRunId] = "unit-test";
            p.fieldsFloat[field::kRsrpDbm] = -90.5;
            sink->Push(p);
        });
        Simulator::Stop(MilliSeconds(200));
        Simulator::Run();
        sink->Stop();

        const std::string body = ReadFile(path);
        NS_TEST_EXPECT_MSG_GT(body.size(), 50u, "file body too small");
        NS_TEST_EXPECT_MSG_EQ(sink->GetEmittedCount(), 2u, "emitted count");
        // Two newline-terminated lines
        const auto count = std::count(body.begin(), body.end(), '\n');
        NS_TEST_EXPECT_MSG_EQ(count, 2, "two lines expected");
        NS_TEST_EXPECT_MSG_NE(body.find("rsrp_dbm=-100"), std::string::npos, "first point");
        NS_TEST_EXPECT_MSG_NE(body.find("rsrp_dbm=-90.5"), std::string::npos, "second point");

        Simulator::Destroy();
    }
};

class InfluxSinkBoundedBufferTest : public TestCase
{
  public:
    InfluxSinkBoundedBufferTest()
        : TestCase("MaxBufferPoints drops the OLDEST points and counts them")
    {
    }

  private:
    void DoRun() override
    {
        const std::string path = "/tmp/ntn-observability-bounded-buffer-test.lp";
        std::remove(path.c_str());

        Ptr<NtnInfluxSink> sink = CreateObject<NtnInfluxSink>();
        sink->SetAttribute("MaxBufferPoints", UintegerValue(10));
        sink->SetTransport(NtnInfluxSink::Transport::File);
        sink->SetFilePath(path);

        // 15 pushes into a 10-point buffer with no flush in between: the 5
        // OLDEST points (seq 0..4) must be dropped and counted.
        for (int i = 0; i < 15; ++i)
        {
            Point p;
            p.measurement = measurement::kRadio;
            p.fieldsInt["seq"] = i;
            p.timestamp = NanoSeconds(i + 1);
            sink->Push(p);
        }
        NS_TEST_EXPECT_MSG_EQ(sink->GetEmittedCount(), 15u, "all pushes counted");
        NS_TEST_EXPECT_MSG_EQ(sink->GetDroppedPoints(), 5u, "five points dropped");

        sink->Flush();
        const std::string body = ReadFile(path);
        const auto lines = std::count(body.begin(), body.end(), '\n');
        NS_TEST_EXPECT_MSG_EQ(lines, 10, "buffer was capped at MaxBufferPoints");
        for (int i = 0; i < 5; ++i)
        {
            NS_TEST_EXPECT_MSG_EQ(body.find("seq=" + std::to_string(i) + "i"),
                                  std::string::npos,
                                  "oldest point seq=" << i << " was dropped");
        }
        for (int i = 5; i < 15; ++i)
        {
            NS_TEST_EXPECT_MSG_NE(body.find("seq=" + std::to_string(i) + "i"),
                                  std::string::npos,
                                  "newest point seq=" << i << " survived");
        }

        Simulator::Destroy();
    }
};

class NetSimulyzerJsonShapeTest : public TestCase
{
  public:
    NetSimulyzerJsonShapeTest()
        : TestCase("NetSimulyzer JSON has configuration nodes series events sections")
    {
    }

  private:
    void DoRun() override
    {
        const std::string path = "/tmp/ntn-observability-test.json";
        std::remove(path.c_str());

        NtnObservabilityHelper helper;
        helper.SetNetSimulyzerOutput(path);
        Ptr<NtnNetSimulyzerExporter> exp = helper.InstallNetSimulyzerExporter();
        exp->AddNode(1, "satellite", Vector{0, 0, 550e3}, 1.0);
        exp->AddNode(2, "ue", Vector{0, 0, 0}, 0.5);
        const uint32_t rsrp = exp->AddSeries("RSRP", "time (s)", "RSRP", "dBm");
        const uint32_t sinr = exp->AddSeries("SINR", "time (s)", "SINR", "dB");

        exp->Start();
        exp->NodeMove(1, 0.5, Vector{1, 2, 3});
        exp->LogMessage(2, 1.0, "hello \"world\"");
        exp->SampleSeries(rsrp, 1.5, -95.5);
        exp->SampleSeries(sinr, 1.5, 8.0);
        exp->Stop();

        const std::string body = ReadFile(path);
        // Top-level structural checks
        NS_TEST_EXPECT_MSG_NE(body.find("\"configuration\""), std::string::npos, "configuration");
        NS_TEST_EXPECT_MSG_NE(body.find("\"nodes\""), std::string::npos, "nodes");
        NS_TEST_EXPECT_MSG_NE(body.find("\"series\""), std::string::npos, "series");
        NS_TEST_EXPECT_MSG_NE(body.find("\"events\""), std::string::npos, "events");
        NS_TEST_EXPECT_MSG_NE(body.find("\"NodeMove\""), std::string::npos, "NodeMove event");
        NS_TEST_EXPECT_MSG_NE(body.find("\"LogMessage\""), std::string::npos, "LogMessage");
        NS_TEST_EXPECT_MSG_NE(body.find("\"SeriesSample\""), std::string::npos,
                              "SeriesSample");
        // Escape: the log message must appear with escaped quotes
        NS_TEST_EXPECT_MSG_NE(body.find(R"(hello \"world\")"), std::string::npos,
                              "string escape");
        // Document is balanced
        NS_TEST_EXPECT_MSG_EQ(body.front(), '{', "starts with {");
        NS_TEST_EXPECT_MSG_EQ(body.back(), '}', "ends with }");
    }
};

class MetricSchemaStableTest : public TestCase
{
  public:
    MetricSchemaStableTest()
        : TestCase("Canonical KPI names are unchanged (downstream dashboards depend on them)")
    {
    }

  private:
    void DoRun() override
    {
        // If you must rename one of these, also update every committed
        // dashboard JSON or downstream consumers will silently break.
        NS_TEST_EXPECT_MSG_EQ(std::string(measurement::kRadio), "ntn_radio", "measurement radio");
        NS_TEST_EXPECT_MSG_EQ(std::string(measurement::kHandover),
                              "ntn_handover",
                              "measurement handover");
        NS_TEST_EXPECT_MSG_EQ(std::string(measurement::kTa),
                              "ntn_timing_advance",
                              "measurement TA");
        NS_TEST_EXPECT_MSG_EQ(std::string(measurement::kIsl), "ntn_isl", "measurement ISL");
        NS_TEST_EXPECT_MSG_EQ(std::string(field::kRsrpDbm), "rsrp_dbm", "field rsrp");
        NS_TEST_EXPECT_MSG_EQ(std::string(field::kTaTotalUs), "ta_total_us", "field ta_total");
        NS_TEST_EXPECT_MSG_EQ(std::string(tag::kCellId), "cell_id", "tag cell_id");
    }
};

class ReproManifestRoundTripTest : public TestCase
{
  public:
    ReproManifestRoundTripTest()
        : TestCase("Reproducibility manifest round-trips through JSON file")
    {
    }

  private:
    void DoRun() override
    {
        const std::string path = "/tmp/ntn-repro-manifest-test.json";
        std::remove(path.c_str());

        const char* argv[] = {"./ns3", "run", "oran-ntn-full-scenario",
                              "--duration=600"};
        ntnobs::NtnReproManifest m;
        m.SetToolkitGitSha("deadbeef")
            .SetNs3Version("ns-3.43")
            .SetScenarioName("oran-ntn-full-scenario")
            .SetScenarioDuration(600.0)
            .SetRng(1, 7)
            .SetTleEpoch("2026-05-22T00:00:00Z")
            .AddNoradId(25544)
            .AddNoradId(25545)
            .SetConstellation("Walker-Star", 6, 11, 550.0, 53.0)
            .SetSionna("2.0.1",
                       "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
                       "aaaaaaa")
            .SetHitranRelease("HITRAN-2024")
            .SetItuRpyVersion("0.4.0")
            .AddServiceModel("KPM", "v3.00")
            .AddServiceModel("RC", "v1.03")
            .SetCliArgv(4, const_cast<char**>(argv))
            .AddExtra("scenario_note", "Q3 2026 sprint baseline");

        NS_TEST_ASSERT_MSG_EQ(m.WriteJson(path), true, "manifest write");

        ntnobs::NtnReproManifest loaded;
        NS_TEST_ASSERT_MSG_EQ(ntnobs::NtnReproManifest::LoadJson(path, loaded),
                              true,
                              "manifest load");

        NS_TEST_EXPECT_MSG_EQ(loaded.GetToolkitGitSha(), "deadbeef", "git sha");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetScenarioName(),
                              "oran-ntn-full-scenario",
                              "scenario name");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetScenarioDuration(),
                              600.0,
                              "scenario duration");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetRngSeed(), 1u, "rng seed");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetRngRun(), 7u, "rng run");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetTleEpoch(),
                              "2026-05-22T00:00:00Z",
                              "tle epoch");
        NS_TEST_ASSERT_MSG_EQ(loaded.GetNoradIds().size(), 2u, "norad size");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetNoradIds()[0], 25544u, "norad[0]");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetNoradIds()[1], 25545u, "norad[1]");
        NS_TEST_EXPECT_MSG_EQ(loaded.HasConstellation(), true, "has const");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetConstellationName(),
                              "Walker-Star",
                              "const name");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetPlanes(), 6u, "planes");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetSatsPerPlane(), 11u, "sats per plane");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetAltitudeKm(), 550.0, "alt km");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetInclinationDeg(), 53.0, "inc deg");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetSionnaRtVersion(), "2.0.1", "sionna rt");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetHitranRelease(),
                              "HITRAN-2024",
                              "hitran");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetItuRpyVersion(), "0.4.0", "itu-rpy");
        NS_TEST_ASSERT_MSG_EQ(loaded.GetServiceModels().size(),
                              2u,
                              "service models size");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetServiceModels()[0].first, "KPM", "sm[0]");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetServiceModels()[1].second,
                              "v1.03",
                              "sm[1].version");
        NS_TEST_ASSERT_MSG_EQ(loaded.GetCliArgv().size(), 4u, "argv size");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetCliArgv()[2],
                              "oran-ntn-full-scenario",
                              "argv[2]");
        NS_TEST_ASSERT_MSG_EQ(loaded.GetExtras().size(), 1u, "extras size");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetExtras()[0].first,
                              "scenario_note",
                              "extras key");
    }
};

class ReproManifestSchemaHeaderTest : public TestCase
{
  public:
    ReproManifestSchemaHeaderTest()
        : TestCase("Manifest JSON declares schema name + version")
    {
    }

  private:
    void DoRun() override
    {
        ntnobs::NtnReproManifest m;
        const std::string body = m.ToJson();
        NS_TEST_EXPECT_MSG_NE(
            body.find("\"schema\": \"ns3-ntn-toolkit/manifest\""),
            std::string::npos,
            "schema name present");
        NS_TEST_EXPECT_MSG_NE(body.find("\"schema_version\": 1"),
                              std::string::npos,
                              "schema version present");
        NS_TEST_EXPECT_MSG_EQ(body.front(), '{', "starts with {");
        // body ends with closing-brace + newline
        const auto trimmed = body.find_last_not_of(" \t\r\n");
        NS_TEST_ASSERT_MSG_NE(trimmed, std::string::npos, "non-blank body");
        NS_TEST_EXPECT_MSG_EQ(body[trimmed], '}', "ends with }");
    }
};

class ReproManifestUnknownKeyTest : public TestCase
{
  public:
    ReproManifestUnknownKeyTest()
        : TestCase("Manifest parser ignores unknown top-level keys")
    {
    }

  private:
    void DoRun() override
    {
        const std::string body =
            R"({
              "schema": "ns3-ntn-toolkit/manifest",
              "schema_version": 1,
              "scenario": { "name": "x", "duration_s": 10, "rng_seed": 2, "rng_run": 3 },
              "future_field": { "nested": [1, 2, "three"], "x": null, "y": true },
              "extras": { "k": "v" }
            })";
        ntnobs::NtnReproManifest m;
        NS_TEST_ASSERT_MSG_EQ(ntnobs::NtnReproManifest::ParseJson(body, m),
                              true,
                              "parse OK with unknown key");
        NS_TEST_EXPECT_MSG_EQ(m.GetScenarioName(), "x", "scenario name");
        NS_TEST_EXPECT_MSG_EQ(m.GetRngSeed(), 2u, "rng seed");
        NS_TEST_EXPECT_MSG_EQ(m.GetRngRun(), 3u, "rng run");
        NS_TEST_ASSERT_MSG_EQ(m.GetExtras().size(), 1u, "extras parsed");
        NS_TEST_EXPECT_MSG_EQ(m.GetExtras()[0].second, "v", "extras value");
    }
};

class ReproManifestMalformedRejectedTest : public TestCase
{
  public:
    ReproManifestMalformedRejectedTest()
        : TestCase("Manifest parser rejects malformed JSON")
    {
    }

  private:
    void DoRun() override
    {
        ntnobs::NtnReproManifest m;
        NS_TEST_EXPECT_MSG_EQ(
            ntnobs::NtnReproManifest::ParseJson("{ this is not json }", m),
            false,
            "garbage rejected");
        NS_TEST_EXPECT_MSG_EQ(
            ntnobs::NtnReproManifest::ParseJson("{\"schema\": ", m),
            false,
            "truncated rejected");
        NS_TEST_EXPECT_MSG_EQ(
            ntnobs::NtnReproManifest::ParseJson("not even an object", m),
            false,
            "non-object rejected");
    }
};

class ReproManifestEscapeRoundTripTest : public TestCase
{
  public:
    ReproManifestEscapeRoundTripTest()
        : TestCase("Manifest round-trip preserves quotes, backslashes, "
                   "newlines in strings")
    {
    }

  private:
    void DoRun() override
    {
        const std::string tricky =
            "value with \"quotes\" and \\backslash\\ and \n newline";
        ntnobs::NtnReproManifest m;
        m.SetScenarioName(tricky).AddExtra("k", tricky);
        const std::string body = m.ToJson();

        ntnobs::NtnReproManifest loaded;
        NS_TEST_ASSERT_MSG_EQ(ntnobs::NtnReproManifest::ParseJson(body, loaded),
                              true,
                              "parse tricky strings");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetScenarioName(),
                              tricky,
                              "scenario name preserved");
        NS_TEST_ASSERT_MSG_EQ(loaded.GetExtras().size(), 1u, "extras size");
        NS_TEST_EXPECT_MSG_EQ(loaded.GetExtras()[0].second,
                              tricky,
                              "extras value preserved");
    }
};

/**
 * \brief End-to-end: NtnSceneRecorder taps a moving node + a fixed ground node
 *        under Simulator::Run(), fans positions + KPI samples to NetSimulyzer
 *        JSON and CZML, and the files come out non-trivial and well-formed.
 */
class SceneRecorderEndToEndTest : public TestCase
{
  public:
    SceneRecorderEndToEndTest()
        : TestCase("NtnSceneRecorder - real Run() fans positions+KPIs to NetSimulyzer+CZML")
    {
    }

    void DoRun() override
    {
        const std::string nsPath = "/tmp/ntn-scene-test.json";
        const std::string czmlPath = "/tmp/ntn-scene-test.czml";
        std::remove(nsPath.c_str());
        std::remove(czmlPath.c_str());

        // A "satellite" moving in ECEF (ConstantVelocity stands in for an orbit
        // here) and a fixed ground terminal, both in the global ECEF frame.
        Ptr<Node> sat = CreateObject<Node>();
        Ptr<ConstantVelocityMobilityModel> satMob =
            CreateObject<ConstantVelocityMobilityModel>();
        satMob->SetPosition(Vector(7.0e6, 0.0, 0.0));
        satMob->SetVelocity(Vector(0.0, 7500.0, 0.0)); // ~LEO speed
        sat->AggregateObject(satMob);

        Ptr<Node> gs = CreateObject<Node>();
        Ptr<ConstantPositionMobilityModel> gsMob =
            CreateObject<ConstantPositionMobilityModel>();
        gsMob->SetPosition(Vector(6.371e6, 0.0, 0.0));
        gs->AggregateObject(gsMob);

        Ptr<ntnobs::NtnSceneRecorder> rec = CreateObject<ntnobs::NtnSceneRecorder>();
        rec->SetFrame(ntnobs::NtnSceneRecorder::EcefGlobal);
        const uint32_t satId = rec->TrackNode(sat, ntnobs::NtnSceneRecorder::Sat, "sat-0");
        rec->TrackNode(gs, ntnobs::NtnSceneRecorder::Gateway, "gs-0");
        const uint32_t sinrSeries =
            rec->TrackKpiSeries(satId, ntnobs::NtnSceneRecorder::Sinr, "SINR-dl");
        rec->TrackBeam(satId, gs->GetId());
        rec->SetSampleInterval(Seconds(1.0));
        rec->EnableNetSimulyzer(nsPath, 1e-6);
        rec->EnableCzml(czmlPath);
        rec->Start();

        // Feed a few measured-KPI samples and a handover event during the run.
        for (int s = 1; s <= 5; ++s)
        {
            Simulator::Schedule(Seconds(s), [rec, sinrSeries, s]() {
                rec->RecordKpi(sinrSeries, 12.0 + s); // dB
            });
        }
        Simulator::Schedule(Seconds(3.0),
                            [rec, satId, gs]() { rec->OnHandover(gs->GetId(), satId, satId); });
        // Toggle the tracked beam down at t=2 and back up at t=4 so the CZML
        // polyline must carry two gated availability intervals (Fix 4).
        Simulator::Schedule(Seconds(2.0),
                            [rec, satId, gs]() { rec->OnLinkChange(satId, gs->GetId(), false); });
        Simulator::Schedule(Seconds(4.0),
                            [rec, satId, gs]() { rec->OnLinkChange(satId, gs->GetId(), true); });

        Simulator::Stop(Seconds(6.0));
        Simulator::Run();
        rec->Stop();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_GT(rec->GetEventCount(), 10u, "expected many scene events");

        const std::string js = ReadFile(nsPath);
        NS_TEST_ASSERT_MSG_NE(js, "", "NetSimulyzer JSON should be written");
        // Native (toolkit) scene schema, NOT the official usnistgov
        // NetSimulyzer schema — the label is deliberately distinct so the file
        // is not mistaken for one the NetSimulyzer app can open.
        const bool hasSchema = js.find("\"schema\":\"ntn-observability-native-1.0\"") != std::string::npos;
        const bool hasNodeMove = js.find("\"NodeMove\"") != std::string::npos;
        const bool hasEarth = js.find("\"earth\"") != std::string::npos;
        const bool hasHandover = js.find("handover") != std::string::npos;
        NS_TEST_ASSERT_MSG_EQ(hasSchema, true, "native scene schema tag present");
        NS_TEST_ASSERT_MSG_EQ(hasNodeMove, true, "NodeMove events present");
        NS_TEST_ASSERT_MSG_EQ(hasEarth, true, "Earth sphere node present");
        NS_TEST_ASSERT_MSG_EQ(hasHandover, true, "handover log event present");

        const std::string cz = ReadFile(czmlPath);
        NS_TEST_ASSERT_MSG_NE(cz, "", "CZML should be written");
        const bool hasDoc = cz.find("\"id\":\"document\"") != std::string::npos;
        const bool hasFixed = cz.find("\"referenceFrame\":\"FIXED\"") != std::string::npos;
        const bool hasSatPacket = cz.find("node-" + std::to_string(satId)) != std::string::npos;
        NS_TEST_ASSERT_MSG_EQ(hasDoc, true, "CZML document packet present");
        NS_TEST_ASSERT_MSG_EQ(hasFixed, true, "CZML FIXED-frame Cartesian positions present");
        NS_TEST_ASSERT_MSG_EQ(hasSatPacket, true, "CZML satellite packet present");
        // The handover scheduled at t=3 must produce an animated arc packet.
        const bool hasArc =
            cz.find("\"ho-0\"") != std::string::npos && cz.find("polyline") != std::string::npos &&
            cz.find("availability") != std::string::npos;
        NS_TEST_ASSERT_MSG_EQ(hasArc, true, "CZML handover arc packet present");

        // Fix 4: the tracked beam must be rendered as a CZML polyline whose
        // endpoints REFERENCE the two node positions (so the link tracks the
        // moving satellite), gated by the OnLinkChange transitions.
        const std::string linkId = "\"link-" + std::to_string(satId) + "-" +
                                   std::to_string(gs->GetId()) + "\"";
        NS_TEST_ASSERT_MSG_NE(cz.find(linkId), std::string::npos, "CZML link polyline present");
        const std::string refA = "node-" + std::to_string(satId) + "#position";
        const std::string refB = "node-" + std::to_string(gs->GetId()) + "#position";
        NS_TEST_ASSERT_MSG_NE(cz.find(refA), std::string::npos, "link references sat position");
        NS_TEST_ASSERT_MSG_NE(cz.find(refB), std::string::npos, "link references ground position");
        NS_TEST_ASSERT_MSG_NE(cz.find("\"references\""), std::string::npos,
                              "link uses positions.references");

        // Fix 4: the measured KPI series must be emitted as a CZML custom
        // property on its node packet (previously accumulated but never written).
        NS_TEST_ASSERT_MSG_NE(cz.find("\"properties\""), std::string::npos,
                              "CZML node properties present");
        NS_TEST_ASSERT_MSG_NE(cz.find("\"SINR-dl\""), std::string::npos,
                              "CZML KPI property SINR-dl present");

        std::remove(nsPath.c_str());
        std::remove(czmlPath.c_str());
    }
};

class NtnObservabilityTestSuite : public TestSuite
{
  public:
    NtnObservabilityTestSuite()
        : TestSuite("ntn-observability", Type::UNIT)
    {
        AddTestCase(new LineProtocolBasicEncodeTest, TestCase::Duration::QUICK);
        AddTestCase(new LineProtocolEscapeTest, TestCase::Duration::QUICK);
        AddTestCase(new InfluxFileSinkRoundTripTest, TestCase::Duration::QUICK);
        AddTestCase(new InfluxSinkBoundedBufferTest, TestCase::Duration::QUICK);
        AddTestCase(new NetSimulyzerJsonShapeTest, TestCase::Duration::QUICK);
        AddTestCase(new MetricSchemaStableTest, TestCase::Duration::QUICK);
        AddTestCase(new ReproManifestRoundTripTest, TestCase::Duration::QUICK);
        AddTestCase(new ReproManifestSchemaHeaderTest, TestCase::Duration::QUICK);
        AddTestCase(new ReproManifestUnknownKeyTest, TestCase::Duration::QUICK);
        AddTestCase(new ReproManifestMalformedRejectedTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new ReproManifestEscapeRoundTripTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new SceneRecorderEndToEndTest, TestCase::Duration::QUICK);
    }
};

static NtnObservabilityTestSuite g_ntnObservabilityTestSuite;
