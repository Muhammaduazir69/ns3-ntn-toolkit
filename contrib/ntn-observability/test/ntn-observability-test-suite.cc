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
        // OBS-13: the wire names adopted from the traffic example are pinned to
        // their literals for the same reason as the seven above. The
        // schema-membership checks below build their sets FROM the constants,
        // so renaming a constant renames both sides and those checks would not
        // notice; only a literal pin catches a rename that breaks a consumer.
        NS_TEST_EXPECT_MSG_EQ(std::string(measurement::kDownlink), "ntn_downlink",
                              "measurement downlink");
        NS_TEST_EXPECT_MSG_EQ(std::string(field::kGoodputMbps), "goodput_mbps", "field goodput");
        NS_TEST_EXPECT_MSG_EQ(std::string(field::kTbler), "tbler", "field tbler");
        NS_TEST_EXPECT_MSG_EQ(std::string(field::kElevationDeg), "elevation_deg",
                              "field elevation");
        NS_TEST_EXPECT_MSG_EQ(std::string(field::kSlantRangeKm), "slant_range_km",
                              "field slant range");
        NS_TEST_EXPECT_MSG_EQ(std::string(field::kRxBytesTotal), "rx_bytes_total",
                              "field rx bytes");

        // OBS-13: the checks above assert that seven constants equal their own
        // string literals. They pass whatever any exporter actually emits, and
        // they passed unchanged while this module's own traffic example emitted
        // a measurement ("ntn_downlink") and five field names that appeared
        // nowhere in the schema and in none of the four shipped dashboards.
        //
        // A schema is only a schema if something checks emitted points against
        // it. These do.
        const std::set<std::string> knownMeasurements = {
            measurement::kRadio,    measurement::kHandover, measurement::kIsl,
            measurement::kTa,       measurement::kSib,      measurement::kDrx,
            measurement::kSatPosition, measurement::kUeReport, measurement::kSlice,
            measurement::kBeam,     measurement::kDownlink,
        };
        const std::set<std::string> knownFields = {
            field::kRsrpDbm,    field::kSinrDb,     field::kBler,
            field::kMcs,        field::kTaTotalUs,  field::kTaCommonUs,
            field::kTaDriftUsPerS, field::kSatXM,   field::kSatYM,
            field::kSatZM,      field::kSatVxMps,   field::kSatVyMps,
            field::kSatVzMps,   field::kLatDeg,     field::kLonDeg,
            field::kAltM,       field::kThroughputMbps, field::kLatencyMs,
            field::kBroadcastSeq, field::kReportSeq, field::kGoodputMbps,
            field::kTbler,      field::kElevationDeg, field::kSlantRangeKm,
            field::kRxBytesTotal,
        };

        // The measurement the traffic example emits must be one the schema
        // knows. Before OBS-13 this was "ntn_downlink" as a bare literal and
        // the schema had never heard of it.
        NS_TEST_ASSERT_MSG_EQ(knownMeasurements.count(measurement::kDownlink), 1u,
                              "ntn_downlink must be part of the schema, not invented at the "
                              "call site");

        // Build the point the example builds and check every name against the
        // schema. This is the check that would have caught the original defect.
        Point p;
        p.measurement = measurement::kDownlink;
        p.tags[tag::kLink] = "leo-gnd";
        p.tags[tag::kBand] = "Ku";
        p.tags[tag::kProvenance] = "phy-trace";
        p.fieldsFloat[field::kGoodputMbps] = 4.0;
        p.fieldsFloat[field::kSinrDb] = 12.0;
        p.fieldsFloat[field::kTbler] = 0.01;
        p.fieldsFloat[field::kElevationDeg] = 40.0;
        p.fieldsFloat[field::kSlantRangeKm] = 900.0;
        p.fieldsInt[field::kRxBytesTotal] = 1000;

        NS_TEST_ASSERT_MSG_EQ(knownMeasurements.count(p.measurement), 1u,
                              "emitted measurement '" << p.measurement << "' is off-schema");
        for (const auto& [k, v] : p.fieldsFloat)
        {
            (void)v;
            NS_TEST_ASSERT_MSG_EQ(knownFields.count(k), 1u,
                                  "emitted float field '" << k << "' is off-schema");
        }
        for (const auto& [k, v] : p.fieldsInt)
        {
            (void)v;
            NS_TEST_ASSERT_MSG_EQ(knownFields.count(k), 1u,
                                  "emitted int field '" << k << "' is off-schema");
        }

        // And the check must be capable of rejecting: an invented name has to
        // fail, or the set above is decoration.
        NS_TEST_ASSERT_MSG_EQ(knownFields.count("goodput_megabits"), 0u,
                              "a name that is not in the schema must not be accepted");
        NS_TEST_ASSERT_MSG_EQ(knownMeasurements.count("ntn_uplink"), 0u,
                              "likewise for measurements");
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
/// OBS-05: the serving edge must follow the handover.
///
/// TrackBeam() declared the serving edge once during setup and nothing ever
/// changed it: OnHandover recorded the event and left m_beams untouched, so the
/// CZML polyline and the live link frame both pointed at the original satellite
/// for the whole run. A viewer watching a handover study saw the very
/// association the study is about never move. OnLinkChange, which builds the
/// availability intervals the CZML writer needs, had zero production callers.
///
/// This hands a UE off from one satellite to another mid-run and asserts the
/// CZML carries two distinct polylines whose availability does not overlap.
class SceneRecorderServingEdgeFollowsHandoverTest : public TestCase
{
  public:
    SceneRecorderServingEdgeFollowsHandoverTest()
        : TestCase("OBS-05 - the serving edge moves with the handover")
    {
    }

    void DoRun() override
    {
        const std::string czmlPath = "/tmp/ntn-scene-obs05.czml";
        std::remove(czmlPath.c_str());

        Ptr<Node> satA = CreateObject<Node>();
        auto aMob = CreateObject<ConstantVelocityMobilityModel>();
        aMob->SetPosition(Vector(7.0e6, 0.0, 0.0));
        aMob->SetVelocity(Vector(0.0, 7500.0, 0.0));
        satA->AggregateObject(aMob);

        Ptr<Node> satB = CreateObject<Node>();
        auto bMob = CreateObject<ConstantVelocityMobilityModel>();
        bMob->SetPosition(Vector(7.0e6, -2.0e6, 0.0));
        bMob->SetVelocity(Vector(0.0, 7500.0, 0.0));
        satB->AggregateObject(bMob);

        Ptr<Node> ue = CreateObject<Node>();
        auto uMob = CreateObject<ConstantPositionMobilityModel>();
        uMob->SetPosition(Vector(6.371e6, 0.0, 0.0));
        ue->AggregateObject(uMob);

        Ptr<ntnobs::NtnSceneRecorder> rec = CreateObject<ntnobs::NtnSceneRecorder>();
        rec->SetFrame(ntnobs::NtnSceneRecorder::EcefGlobal);
        const uint32_t aId = rec->TrackNode(satA, ntnobs::NtnSceneRecorder::Sat, "sat-A");
        const uint32_t bId = rec->TrackNode(satB, ntnobs::NtnSceneRecorder::Sat, "sat-B");
        const uint32_t uId = rec->TrackNode(ue, ntnobs::NtnSceneRecorder::Gateway, "ue-0");
        // Only the FIRST serving edge is declared, exactly as a scenario does.
        rec->TrackBeam(aId, uId);
        rec->SetSampleInterval(Seconds(1.0));
        rec->EnableCzml(czmlPath);
        rec->Start();

        // One handover, halfway through the run.
        Simulator::Schedule(Seconds(5.0),
                            [rec, uId, aId, bId]() { rec->OnHandover(uId, aId, bId); });

        Simulator::Stop(Seconds(10.0));
        Simulator::Run();
        rec->Stop();
        Simulator::Destroy();

        const std::string czml = ReadFile(czmlPath);
        NS_TEST_ASSERT_MSG_NE(czml, "", "CZML should be written");

        // Two link packets must exist: the edge the UE started on and the one it
        // moved to. Before this fix the second was never created, because
        // OnHandover did not touch the beam list.
        size_t links = 0;
        for (size_t pos = czml.find("\"link-"); pos != std::string::npos;
             pos = czml.find("\"link-", pos + 1))
        {
            ++links;
        }
        NS_TEST_ASSERT_MSG_GT(links, 1u,
                              "a handover must produce a SECOND serving polyline; one link packet "
                              "means the edge declared at setup is still the only one and the "
                              "association never moved");

        // And the two must be gated: an availability interval has to appear, or
        // both polylines are drawn for the whole run and the viewer sees the UE
        // served by two satellites at once.
        const bool gated = (czml.find("availability") != std::string::npos);
        NS_TEST_ASSERT_MSG_EQ(gated, true,
                              "the polylines must carry availability intervals, otherwise both "
                              "are shown for the entire run");
    }
};

/// OBS-10: a label must not be able to break the file it is written into.
///
/// Two output paths took caller-supplied strings and emitted them without
/// adequate escaping.
///
/// The Influx sink escaped ',', ' ' and '=' for keys and '"' and '\\' for
/// values, and neither handled a newline. In line protocol a newline
/// TERMINATES a point and the format offers no escape for it, so a value
/// carrying one does not produce a mis-rendered point - it produces a SECOND
/// point whose entire content is caller-supplied. The value reaches the sink
/// straight from the command line: ntn-observability-demo takes --runId and
/// writes it into a tag.
///
/// The CZML writer was worse: node labels and KPI series names went into JSON
/// string literals with no escaping whatever, so a single double quote produced
/// a document no parser accepts.
class ObservabilityLabelInjectionTest : public TestCase
{
  public:
    ObservabilityLabelInjectionTest()
        : TestCase("OBS-10 - hostile labels cannot break line protocol or CZML")
    {
    }

    void DoRun() override
    {
        // ---- Influx line protocol ----
        {
            Point p;
            p.measurement = "radio";
            // A run id that tries to close the point and start another one.
            p.tags["run_id"] = "abc\ninjected,evil=1 value=99i";
            p.fieldsFloat["sinr_db"] = 12.5;
            p.timestamp = NanoSeconds(1000);
            const std::string body = EncodeLineProtocol({p}, true);

            // Exactly one point: count non-empty lines.
            size_t lines = 0;
            std::istringstream is(body);
            std::string line;
            while (std::getline(is, line))
            {
                if (!line.empty())
                {
                    ++lines;
                }
            }
            NS_TEST_ASSERT_MSG_EQ(lines, 1u,
                                  "a newline inside a tag value must not be able to start a "
                                  "second point; line protocol has no escape for it, so it has "
                                  "to be removed rather than escaped");
            const bool rawNewlineInside =
                (body.find("\ninjected") != std::string::npos);
            NS_TEST_ASSERT_MSG_EQ(rawNewlineInside, false,
                                  "the injected payload must not survive as its own record");
        }

        // ---- CZML ----
        {
            const std::string czmlPath = "/tmp/ntn-obs10.czml";
            std::remove(czmlPath.c_str());

            Ptr<Node> sat = CreateObject<Node>();
            auto sm = CreateObject<ConstantVelocityMobilityModel>();
            sm->SetPosition(Vector(7.0e6, 0.0, 0.0));
            sm->SetVelocity(Vector(0.0, 7500.0, 0.0));
            sat->AggregateObject(sm);

            Ptr<ntnobs::NtnSceneRecorder> rec = CreateObject<ntnobs::NtnSceneRecorder>();
            rec->SetFrame(ntnobs::NtnSceneRecorder::EcefGlobal);
            // A label that closes the JSON string and injects a key.
            rec->TrackNode(sat, ntnobs::NtnSceneRecorder::Sat, "sat\",\"evil\":\"x");
            rec->SetSampleInterval(Seconds(1.0));
            rec->EnableCzml(czmlPath);
            rec->Start();
            Simulator::Stop(Seconds(3.0));
            Simulator::Run();
            rec->Stop();
            Simulator::Destroy();

            const std::string czml = ReadFile(czmlPath);
            NS_TEST_ASSERT_MSG_NE(czml, "", "CZML should be written");
            // The injected key must appear only as escaped text inside the
            // label, never as a JSON key of its own.
            const bool injectedKey = (czml.find("\"evil\":") != std::string::npos);
            NS_TEST_ASSERT_MSG_EQ(injectedKey, false,
                                  "a quote inside a node label must be escaped, not allowed to "
                                  "terminate the JSON string and inject a sibling key");
            const bool escaped = (czml.find("\\\"") != std::string::npos);
            NS_TEST_ASSERT_MSG_EQ(escaped, true,
                                  "the quote must still be PRESENT, escaped: silently dropping "
                                  "the character would lose the label rather than encode it");
        }
    }
};

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
        AddTestCase(new ObservabilityLabelInjectionTest, TestCase::Duration::QUICK);
/// OBS-06: links must reach the NetSimulyzer sink, not only the CZML one.
///
/// NtnSceneRecorder kept its beams in m_beams and forwarded them to live stdout
/// and WriteCzml(). Start() never told NtnNetSimulyzerExporter about them, so
/// its JSON described nodes moving with nothing between them, while a commit
/// message claimed links landed there too. A reader could see the TEXT
/// "link 3<->7 up" in the message stream and had no link object to raise.
///
/// The handover case is the one that matters: the link a handover CREATES did
/// not exist when the header was written, so a header-only declaration would
/// drop precisely the link anyone opens the file to watch.
/// OBS-15: the LocalEnu-to-ECEF conversion must be checked, not assumed.
///
/// Roughly forty examples reach the recorder through the LocalEnu path. The only
/// test of the recorder set EcefGlobal, which makes ToEcef the identity for the
/// whole test, so the conversion had zero coverage. Every assertion in that test
/// is a substring search on the output files, so a numerically wrong position
/// would have passed all of them.
/// OBS-14: the export surfaces anchor time differently, and none of it was
/// tested.
///
/// The CZML scene uses a scenario epoch (2026-01-01), NtnInfluxSink adds a
/// wall-clock base epoch taken at construction, and the Python backend stamps
/// datetime.now(). Those differences are deliberate: a globe must animate at the
/// ephemeris date, and a live Grafana "last 1 hour" query cannot find points
/// stamped in 1970. What was missing is that no test exercised the base epoch at
/// all - the round-trip test never set or asserted a timestamp, the encode test
/// bypassed Push() where the epoch is added, and the docker leg queried a
/// two-hour window and asserted only a count.
class ObservabilityTimeAnchorTest : public TestCase
{
  public:
    ObservabilityTimeAnchorTest()
        : TestCase("OBS-14: the base epoch is applied to emitted points and is reported")
    {
    }

  private:
    static long long StampOf(const std::string& lp)
    {
        // Line protocol: "<meas>,<tags> <fields> <ns>"
        const auto sp = lp.find_last_of(' ');
        NS_ABORT_MSG_IF(sp == std::string::npos, "no timestamp in '" << lp << "'");
        return std::stoll(lp.substr(sp + 1));
    }

    void DoRun() override
    {
        // ---- A pure sim-time sink puts points at epoch 1970 ---------------
        Ptr<NtnInfluxSink> simSink = CreateObject<NtnInfluxSink>();
        simSink->SetBaseEpoch(Time(0));
        NS_TEST_ASSERT_MSG_EQ(simSink->GetBaseEpoch(), Time(0), "base epoch must be settable");

        Point p;
        p.measurement = measurement::kRadio;
        p.tags[tag::kCellId] = "1";
        p.fieldsFloat[field::kSinrDb] = 10.0;
        p.timestamp = Seconds(5.0);

        const std::string simLp = EncodeLineProtocol({simSink->StampForTest(p)}, false);
        NS_TEST_ASSERT_MSG_EQ_TOL(static_cast<double>(StampOf(simLp)), 5.0e9, 1.0,
                                  "with a zero base epoch a point at sim t=5 s must be stamped "
                                  "5e9 ns, i.e. 1970 plus five seconds");

        // ---- A wall-clock base epoch shifts it, by exactly that much ------
        const Time base = Seconds(1767225600.0); // the scene epoch, for comparability
        Ptr<NtnInfluxSink> wallSink = CreateObject<NtnInfluxSink>();
        wallSink->SetBaseEpoch(base);
        const std::string wallLp = EncodeLineProtocol({wallSink->StampForTest(p)}, false);
        const long long shifted = StampOf(wallLp);
        NS_TEST_ASSERT_MSG_EQ_TOL(static_cast<double>(shifted),
                                  base.GetNanoSeconds() + 5.0e9, 1.0,
                                  "the base epoch must be ADDED to sim time; this was the "
                                  "behaviour no test exercised");
        NS_TEST_ASSERT_MSG_GT(shifted, StampOf(simLp),
                              "and it must actually move the stamp");

        // ---- The default is wall-clock, not zero and not the scene epoch ---
        Ptr<NtnInfluxSink> defSink = CreateObject<NtnInfluxSink>();
        NS_TEST_ASSERT_MSG_GT(defSink->GetBaseEpoch().GetSeconds(), 1.6e9,
                              "the default base epoch must be a real wall-clock time, or a live "
                              "dashboard query cannot find the points");

        // ---- Each surface must be able to say which anchor it used --------
        const std::string note = simSink->TimeAnchorNote();
        const bool namesSimTime = (note.find("PURE SIM TIME") != std::string::npos);
        NS_TEST_ASSERT_MSG_EQ(namesSimTime, true,
                              "a zero-epoch sink must say its points are in sim time (got: "
                                  << note << ")");
        const std::string wnote = wallSink->TimeAnchorNote();
        const bool warnsOfDivergence = (wnote.find("not on one clock") != std::string::npos);
        NS_TEST_ASSERT_MSG_EQ(warnsOfDivergence, true,
                              "and an offset sink must say the surfaces do not share a clock, "
                              "which is the fact that made cross-referencing guesswork");

        // ---- The scene epoch is a named constant, not a bare literal ------
        Ptr<ntnobs::NtnSceneRecorder> rec = CreateObject<ntnobs::NtnSceneRecorder>();
        NS_TEST_ASSERT_MSG_EQ_TOL(rec->GetSceneEpochUnix(),
                                  ntnobs::NtnSceneRecorder::kDefaultSceneEpochUnix, 1e-9,
                                  "the recorder must start at the declared scene epoch");
        NS_TEST_ASSERT_MSG_EQ_TOL(ntnobs::NtnSceneRecorder::kDefaultSceneEpochUnix,
                                  1767225600.0, 1e-9,
                                  "which is 2026-01-01T00:00:00Z; the CZML in every committed "
                                  "scene depends on this value");
        rec->SetSceneEpochUnix(1.0e9);
        NS_TEST_ASSERT_MSG_EQ_TOL(rec->GetSceneEpochUnix(), 1.0e9, 1e-9,
                                  "and a scenario aligning the surfaces must be able to move it");
    }
};

class SceneRecorderEnuToEcefTest : public TestCase
{
  public:
    SceneRecorderEnuToEcefTest()
        : TestCase("OBS-15: the LocalEnu-to-ECEF conversion is correct at known geometries")
    {
    }

  private:
    static double Norm(const Vector& v)
    {
        return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    }

    void DoRun() override
    {
        Ptr<ntnobs::NtnSceneRecorder> rec = CreateObject<ntnobs::NtnSceneRecorder>();
        rec->SetFrame(ntnobs::NtnSceneRecorder::LocalEnu);

        // ---- Reference at (0 N, 0 E): the axes map cleanly ----------------
        // ECEF origin is (a, 0, 0). East is +y, North is +z, Up is +x.
        rec->SetEnuReference(0.0, 0.0, 0.0);
        const Vector o = rec->ToEcefForTest(Vector(0, 0, 0));
        NS_TEST_ASSERT_MSG_EQ_TOL(o.x, 6378137.0, 1.0,
                                  "the ENU origin must land on the WGS-84 equatorial radius");
        NS_TEST_ASSERT_MSG_EQ_TOL(o.y, 0.0, 1e-6, "and on the Greenwich meridian");
        NS_TEST_ASSERT_MSG_EQ_TOL(o.z, 0.0, 1e-6, "and on the equator");

        {
            const Vector east = rec->ToEcefForTest(Vector(1000.0, 0, 0));
            NS_TEST_ASSERT_MSG_EQ_TOL(east.y - o.y, 1000.0, 1e-6,
                                      "at (0,0) local EAST is ECEF +y");
            NS_TEST_ASSERT_MSG_EQ_TOL(east.x - o.x, 0.0, 1e-6, "and does not move x");
            NS_TEST_ASSERT_MSG_EQ_TOL(east.z - o.z, 0.0, 1e-6, "or z");
        }
        {
            const Vector north = rec->ToEcefForTest(Vector(0, 1000.0, 0));
            NS_TEST_ASSERT_MSG_EQ_TOL(north.z - o.z, 1000.0, 1e-6,
                                      "at (0,0) local NORTH is ECEF +z");
        }
        {
            const Vector up = rec->ToEcefForTest(Vector(0, 0, 1000.0));
            NS_TEST_ASSERT_MSG_EQ_TOL(up.x - o.x, 1000.0, 1e-6,
                                      "at (0,0) local UP is ECEF +x");
        }

        // ---- Reference at (0 N, 90 E): the axes rotate with it ------------
        // A rotation error that happens to be harmless at (0,0) shows here.
        rec->SetEnuReference(0.0, 90.0, 0.0);
        const Vector o90 = rec->ToEcefForTest(Vector(0, 0, 0));
        NS_TEST_ASSERT_MSG_EQ_TOL(o90.x, 0.0, 1.0, "origin at 90 E lies on +y");
        NS_TEST_ASSERT_MSG_EQ_TOL(o90.y, 6378137.0, 1.0, "at the equatorial radius");
        {
            const Vector east = rec->ToEcefForTest(Vector(1000.0, 0, 0));
            NS_TEST_ASSERT_MSG_EQ_TOL(east.x - o90.x, -1000.0, 1e-6,
                                      "at (0,90) local EAST is ECEF -x");
            const Vector up = rec->ToEcefForTest(Vector(0, 0, 1000.0));
            NS_TEST_ASSERT_MSG_EQ_TOL(up.y - o90.y, 1000.0, 1e-6,
                                      "and local UP is ECEF +y");
        }

        // ---- The basis must be orthonormal and right-handed ---------------
        // Sign errors in one axis survive the axis-by-axis checks above at some
        // reference points; East x North = Up does not.
        rec->SetEnuReference(48.0, 11.0, 0.0);
        const Vector base = rec->ToEcefForTest(Vector(0, 0, 0));
        auto axis = [&](double e, double n, double u) {
            const Vector p = rec->ToEcefForTest(Vector(e, n, u));
            return Vector(p.x - base.x, p.y - base.y, p.z - base.z);
        };
        const Vector E = axis(1.0, 0, 0);
        const Vector N = axis(0, 1.0, 0);
        const Vector U = axis(0, 0, 1.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(Norm(E), 1.0, 1e-9, "East must be a unit vector");
        NS_TEST_ASSERT_MSG_EQ_TOL(Norm(N), 1.0, 1e-9, "North must be a unit vector");
        NS_TEST_ASSERT_MSG_EQ_TOL(Norm(U), 1.0, 1e-9, "Up must be a unit vector");
        NS_TEST_ASSERT_MSG_EQ_TOL(E.x * N.x + E.y * N.y + E.z * N.z, 0.0, 1e-9,
                                  "East and North must be orthogonal");
        NS_TEST_ASSERT_MSG_EQ_TOL(E.x * U.x + E.y * U.y + E.z * U.z, 0.0, 1e-9,
                                  "East and Up must be orthogonal");
        // East x North must equal Up (right-handed ENU).
        const Vector cross(E.y * N.z - E.z * N.y, E.z * N.x - E.x * N.z, E.x * N.y - E.y * N.x);
        NS_TEST_ASSERT_MSG_EQ_TOL(cross.x, U.x, 1e-9, "East x North must be Up (x)");
        NS_TEST_ASSERT_MSG_EQ_TOL(cross.y, U.y, 1e-9, "East x North must be Up (y)");
        NS_TEST_ASSERT_MSG_EQ_TOL(cross.z, U.z, 1e-9, "East x North must be Up (z)");

        // The Up axis at a non-polar latitude must point away from geocentre.
        NS_TEST_ASSERT_MSG_GT(U.x * base.x + U.y * base.y + U.z * base.z, 0.0,
                              "Up must point outward, not inward");

        // ---- EcefGlobal must remain the identity --------------------------
        rec->SetFrame(ntnobs::NtnSceneRecorder::EcefGlobal);
        const Vector same = rec->ToEcefForTest(Vector(1.0, 2.0, 3.0));
        NS_TEST_ASSERT_MSG_EQ_TOL(same.x, 1.0, 1e-12, "EcefGlobal is a pass-through");
        NS_TEST_ASSERT_MSG_EQ_TOL(same.y, 2.0, 1e-12, "EcefGlobal is a pass-through");
        NS_TEST_ASSERT_MSG_EQ_TOL(same.z, 3.0, 1e-12, "EcefGlobal is a pass-through");
    }
};

class SceneRecorderLinksReachNetSimulyzerTest : public TestCase
{
  public:
    SceneRecorderLinksReachNetSimulyzerTest()
        : TestCase("OBS-06 - declared beams and handover links reach the NetSimulyzer sink")
    {
    }

    void DoRun() override
    {
        const std::string jsonPath = "/tmp/ntn-scene-obs06.scene.json";
        std::remove(jsonPath.c_str());

        Ptr<Node> satA = CreateObject<Node>();
        auto aMob = CreateObject<ConstantVelocityMobilityModel>();
        aMob->SetPosition(Vector(7.0e6, 0.0, 0.0));
        aMob->SetVelocity(Vector(0.0, 7500.0, 0.0));
        satA->AggregateObject(aMob);

        Ptr<Node> satB = CreateObject<Node>();
        auto bMob = CreateObject<ConstantVelocityMobilityModel>();
        bMob->SetPosition(Vector(7.0e6, -2.0e6, 0.0));
        bMob->SetVelocity(Vector(0.0, 7500.0, 0.0));
        satB->AggregateObject(bMob);

        Ptr<Node> ue = CreateObject<Node>();
        auto uMob = CreateObject<ConstantPositionMobilityModel>();
        uMob->SetPosition(Vector(6.371e6, 0.0, 0.0));
        ue->AggregateObject(uMob);

        Ptr<ntnobs::NtnSceneRecorder> rec = CreateObject<ntnobs::NtnSceneRecorder>();
        rec->SetFrame(ntnobs::NtnSceneRecorder::EcefGlobal);
        const uint32_t aId = rec->TrackNode(satA, ntnobs::NtnSceneRecorder::Sat, "sat-A");
        const uint32_t bId = rec->TrackNode(satB, ntnobs::NtnSceneRecorder::Sat, "sat-B");
        const uint32_t uId = rec->TrackNode(ue, ntnobs::NtnSceneRecorder::Gateway, "ue-0");
        rec->TrackBeam(aId, uId); // only the first edge, as a scenario does
        rec->SetSampleInterval(Seconds(1.0));
        rec->EnableNetSimulyzer(jsonPath);
        rec->Start();

        Simulator::Schedule(Seconds(5.0),
                            [rec, uId, aId, bId]() { rec->OnHandover(uId, aId, bId); });
        Simulator::Stop(Seconds(10.0));
        Simulator::Run();
        rec->Stop();
        Simulator::Destroy();

        std::ifstream in(jsonPath);
        NS_TEST_ASSERT_MSG_EQ(in.good(), true, "the NetSimulyzer sink must be written");
        std::stringstream buf;
        buf << in.rdbuf();
        const std::string json = buf.str();

        // 1. The pre-declared beam must appear as a LINK, not only as prose.
        const bool hasLinksArray = (json.find("\"links\":[") != std::string::npos);
        NS_TEST_ASSERT_MSG_EQ(hasLinksArray, true,
                              "the scene JSON must carry a links array");
        const bool declared = (json.find("\"links\":[]") == std::string::npos);
        NS_TEST_ASSERT_MSG_EQ(declared, true,
                              "the beam declared before Start() must be in that array; an empty "
                              "array is the defect this case exists for");

        // 2. The handover must LOWER the outgoing link and RAISE an incoming
        //    one, and the incoming link must have been ADDED mid-run, because
        //    it did not exist when the header was written.
        const bool hasAdd = (json.find("\"type\":\"LinkAdd\"") != std::string::npos);
        const bool hasChange = (json.find("\"type\":\"LinkChange\"") != std::string::npos);
        const bool hasDown = (json.find("\"up\":false") != std::string::npos);
        const bool hasUp = (json.find("\"up\":true") != std::string::npos);
        NS_TEST_ASSERT_MSG_EQ(hasAdd, true,
                              "the handover creates a link that did not exist at Start(); it "
                              "must be added, not dropped");
        NS_TEST_ASSERT_MSG_EQ(hasChange, true, "and the transitions must be emitted");
        NS_TEST_ASSERT_MSG_EQ(hasDown, true,
                              "the outgoing edge must go down at the handover");
        NS_TEST_ASSERT_MSG_EQ(hasUp, true, "and the incoming edge must come up");

        // 3. A log line saying "link a<->b up" is NOT a link. The defect was
        //    that the prose existed and the object did not, so a check that
        //    accepted the prose would have passed on the broken code. The
        //    assertions above deliberately key on the link OBJECTS only.
    }
};

        AddTestCase(new ObservabilityTimeAnchorTest, TestCase::Duration::QUICK);
        AddTestCase(new SceneRecorderEnuToEcefTest, TestCase::Duration::QUICK);
        AddTestCase(new SceneRecorderLinksReachNetSimulyzerTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new SceneRecorderServingEdgeFollowsHandoverTest,
                    TestCase::Duration::QUICK);
    }
};

static NtnObservabilityTestSuite g_ntnObservabilityTestSuite;
