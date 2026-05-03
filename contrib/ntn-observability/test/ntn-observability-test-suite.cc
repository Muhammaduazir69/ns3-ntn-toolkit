/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/log.h"
#include "ns3/ntn-influx-sink.h"
#include "ns3/ntn-metric-schema.h"
#include "ns3/ntn-netsimulyzer-exporter.h"
#include "ns3/ntn-observability-helper.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

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

class NtnObservabilityTestSuite : public TestSuite
{
  public:
    NtnObservabilityTestSuite()
        : TestSuite("ntn-observability", Type::UNIT)
    {
        AddTestCase(new LineProtocolBasicEncodeTest, TestCase::Duration::QUICK);
        AddTestCase(new LineProtocolEscapeTest, TestCase::Duration::QUICK);
        AddTestCase(new InfluxFileSinkRoundTripTest, TestCase::Duration::QUICK);
        AddTestCase(new NetSimulyzerJsonShapeTest, TestCase::Duration::QUICK);
        AddTestCase(new MetricSchemaStableTest, TestCase::Duration::QUICK);
    }
};

static NtnObservabilityTestSuite g_ntnObservabilityTestSuite;
