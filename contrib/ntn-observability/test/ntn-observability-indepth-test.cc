// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// In-depth ntn-observability validation (18 Nov 2026 test campaign).
//
// Every case runs Simulator::Run() with the sink, exporter or recorder taking
// part in the simulation, and checks the emitted artifact against its format:
//   - InfluxDB line protocol (docs.influxdata.com, "Line protocol" reference:
//     escaping rules, the i suffix for integers, nanosecond timestamps, no
//     empty measurement, tag key/value or field key),
//   - RFC 8259 JSON for the NetSimulyzer and CZML documents (no NaN literal),
//   - CZML packet structure (AGI/Cesium CZML guide: document packet first,
//     ISO 8601 availability intervals, FIXED-frame cartesian samples),
//   - WGS-84 geometry for SGP4 satellites tracked from ntn-constellation.
// Packet-level KPIs come from FlowMonitor on a real IPv4/P2P data plane.

#include "ns3/applications-module.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/node-container.h"
#include "ns3/ntn-influx-sink.h"
#include "ns3/ntn-netsimulyzer-exporter.h"
#include "ns3/ntn-observability-helper.h"
#include "ns3/ntn-scene-helper.h"
#include "ns3/ntn-scene-recorder.h"
#include "ns3/point-to-point-module.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;
using namespace ns3::ntnobs;

namespace
{

std::string
Slurp(const std::string& path)
{
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::vector<std::string>
Lines(const std::string& path)
{
    std::vector<std::string> out;
    std::ifstream f(path);
    std::string l;
    while (std::getline(f, l))
    {
        if (!l.empty())
        {
            out.push_back(l);
        }
    }
    return out;
}

/// Minimal RFC 8259 syntax checker (no NaN/Infinity, strict strings and
/// numbers). Returns true when the whole text is exactly one JSON value.
class JsonCheck
{
  public:
    explicit JsonCheck(const std::string& s)
        : m_s(s)
    {
    }

    bool Valid()
    {
        Ws();
        if (!Value())
        {
            return false;
        }
        Ws();
        return m_i == m_s.size();
    }

  private:
    void Ws()
    {
        while (m_i < m_s.size() &&
               (m_s[m_i] == ' ' || m_s[m_i] == '\n' || m_s[m_i] == '\r' || m_s[m_i] == '\t'))
        {
            ++m_i;
        }
    }

    bool Lit(const char* w)
    {
        const size_t n = std::char_traits<char>::length(w);
        if (m_s.compare(m_i, n, w) == 0)
        {
            m_i += n;
            return true;
        }
        return false;
    }

    bool Str()
    {
        if (m_s[m_i] != '"')
        {
            return false;
        }
        ++m_i;
        while (m_i < m_s.size())
        {
            const unsigned char c = m_s[m_i];
            if (c == '"')
            {
                ++m_i;
                return true;
            }
            if (c < 0x20)
            {
                return false;
            }
            if (c == '\\')
            {
                ++m_i;
                if (m_i >= m_s.size())
                {
                    return false;
                }
                const char e = m_s[m_i];
                if (e == 'u')
                {
                    for (int k = 0; k < 4; ++k)
                    {
                        ++m_i;
                        if (m_i >= m_s.size() || !std::isxdigit(static_cast<unsigned char>(m_s[m_i])))
                        {
                            return false;
                        }
                    }
                }
                else if (std::string("\"\\/bfnrt").find(e) == std::string::npos)
                {
                    return false;
                }
            }
            ++m_i;
        }
        return false;
    }

    bool Num()
    {
        const size_t st = m_i;
        if (m_s[m_i] == '-')
        {
            ++m_i;
        }
        if (m_i >= m_s.size() || !std::isdigit(static_cast<unsigned char>(m_s[m_i])))
        {
            return false;
        }
        if (m_s[m_i] == '0')
        {
            ++m_i;
        }
        else
        {
            while (m_i < m_s.size() && std::isdigit(static_cast<unsigned char>(m_s[m_i])))
            {
                ++m_i;
            }
        }
        if (m_i < m_s.size() && m_s[m_i] == '.')
        {
            ++m_i;
            if (m_i >= m_s.size() || !std::isdigit(static_cast<unsigned char>(m_s[m_i])))
            {
                return false;
            }
            while (m_i < m_s.size() && std::isdigit(static_cast<unsigned char>(m_s[m_i])))
            {
                ++m_i;
            }
        }
        if (m_i < m_s.size() && (m_s[m_i] == 'e' || m_s[m_i] == 'E'))
        {
            ++m_i;
            if (m_i < m_s.size() && (m_s[m_i] == '+' || m_s[m_i] == '-'))
            {
                ++m_i;
            }
            if (m_i >= m_s.size() || !std::isdigit(static_cast<unsigned char>(m_s[m_i])))
            {
                return false;
            }
            while (m_i < m_s.size() && std::isdigit(static_cast<unsigned char>(m_s[m_i])))
            {
                ++m_i;
            }
        }
        return m_i > st;
    }

    bool Value()
    {
        if (m_i >= m_s.size())
        {
            return false;
        }
        const char c = m_s[m_i];
        if (c == '{' || c == '[')
        {
            const char close = c == '{' ? '}' : ']';
            ++m_i;
            Ws();
            if (m_i < m_s.size() && m_s[m_i] == close)
            {
                ++m_i;
                return true;
            }
            while (true)
            {
                Ws();
                if (c == '{')
                {
                    if (m_i >= m_s.size() || !Str())
                    {
                        return false;
                    }
                    Ws();
                    if (m_i >= m_s.size() || m_s[m_i] != ':')
                    {
                        return false;
                    }
                    ++m_i;
                    Ws();
                }
                if (!Value())
                {
                    return false;
                }
                Ws();
                if (m_i >= m_s.size())
                {
                    return false;
                }
                if (m_s[m_i] == ',')
                {
                    ++m_i;
                    continue;
                }
                if (m_s[m_i] == close)
                {
                    ++m_i;
                    return true;
                }
                return false;
            }
        }
        if (c == '"')
        {
            return Str();
        }
        if (Lit("true") || Lit("false") || Lit("null"))
        {
            return true;
        }
        return Num();
    }

    const std::string& m_s;
    size_t m_i{0};
};

bool
IsJson(const std::string& s)
{
    return !s.empty() && JsonCheck(s).Valid();
}

/// The numbers of the first "cartesian":[...] array after the packet whose id
/// is \p id in a CZML document.
std::vector<double>
CzmlCartesian(const std::string& doc, const std::string& id)
{
    std::vector<double> v;
    const size_t p = doc.find("\"id\":\"" + id + "\"");
    if (p == std::string::npos)
    {
        return v;
    }
    const size_t c = doc.find("\"cartesian\":[", p);
    if (c == std::string::npos)
    {
        return v;
    }
    const size_t e = doc.find(']', c);
    std::stringstream ss(doc.substr(c + 13, e - c - 13));
    std::string tok;
    while (std::getline(ss, tok, ','))
    {
        v.push_back(std::stod(tok));
    }
    return v;
}

size_t
Count(const std::string& hay, const std::string& needle)
{
    size_t n = 0;
    for (size_t p = hay.find(needle); p != std::string::npos; p = hay.find(needle, p + 1))
    {
        ++n;
    }
    return n;
}

/// The value of field \p key (as written) in a line-protocol line.
std::string
LpField(const std::string& line, const std::string& key)
{
    const size_t sp = line.find(' ');
    const size_t p = line.find(key + "=", sp);
    if (p == std::string::npos)
    {
        return "";
    }
    const size_t st = p + key.size() + 1;
    size_t e = line.find_first_of(", ", st);
    return line.substr(st, e - st);
}

} // namespace

// ---------------------------------------------------------------------------
/// Line protocol conformance on points pushed during a run: escaping of the
/// measurement, tag keys/values and string fields; the i suffix; the
/// nanosecond timestamp = base epoch + simulation time; and the elements the
/// grammar forbids (empty measurement, empty tag value, control characters).
class ObsLineProtocolConformanceTest : public TestCase
{
  public:
    ObsLineProtocolConformanceTest()
        : TestCase("Influx sink output conforms to the InfluxDB line-protocol grammar during a run")
    {
    }

  private:
    void DoRun() override
    {
        const std::string path = CreateTempDirFilename("lp-conformance.lp");
        Ptr<NtnInfluxSink> sink = CreateObject<NtnInfluxSink>();
        sink->SetTransport(NtnInfluxSink::Transport::File);
        sink->SetFilePath(path);
        sink->SetBaseEpoch(Seconds(1767225600)); // 2026-01-01T00:00:00Z
        sink->SetFlushPeriod(MilliSeconds(100));
        sink->Start();

        Simulator::Schedule(MilliSeconds(1500), [sink]() {
            Point p;
            p.measurement = "ntn kpi,x";
            p.tags["cell"] = "a b";
            p.tags["beam=id"] = "7";
            p.tags["empty"] = ""; // illegal in line protocol: dropped
            p.fieldsFloat["sinr_db"] = 12.5;
            p.fieldsInt["rx_bytes"] = 42;
            p.fieldsString["note"] = "he said \"hi\"\\";
            sink->Push(p);
        });
        Simulator::Schedule(Seconds(2), [sink]() {
            Point p;
            p.measurement = "kpi\nforged v=1"; // a newline must not split the line
            p.fieldsFloat["v"] = 1.0;
            sink->Push(p);
            Point empty;
            empty.fieldsFloat["v"] = 1.0; // no measurement: not a legal line
            sink->Push(empty);
            Point nan;
            nan.measurement = "kpi";
            nan.fieldsFloat["bad"] = std::nan("");
            nan.fieldsFloat["good"] = 3.0;
            sink->Push(nan);
        });
        Simulator::Schedule(Seconds(3), [sink]() { sink->Stop(); });
        Simulator::Run();
        Simulator::Destroy();

        const auto lines = Lines(path);
        NS_TEST_ASSERT_MSG_EQ(lines.size(), 3u, "three legal lines, one per legal point");
        NS_TEST_ASSERT_MSG_EQ(sink->GetDroppedPoints(), 1u, "the empty-measurement point is dropped");
        if (lines.size() == 3)
        {
            NS_TEST_ASSERT_MSG_EQ(lines[0],
                                  "ntn\\ kpi\\,x,beam\\=id=7,cell=a\\ b sinr_db=12.5,rx_bytes=42i,"
                                  "note=\"he said \\\"hi\\\"\\\\\" 1767225601500000000",
                                  "escaping, i suffix and ns timestamp (epoch + 1.5 s)");
            NS_TEST_ASSERT_MSG_EQ(lines[1], "kpi\\ forged\\ v=1 v=1 1767225602000000000",
                                  "control characters in a measurement are neutralized");
            NS_TEST_ASSERT_MSG_EQ(lines[2], "kpi good=3 1767225602000000000",
                                  "a NaN field is removed, the finite field stays");
        }
    }
};

// ---------------------------------------------------------------------------
/// Flush cadence under a running simulation: with a 250 ms period, the file
/// holds exactly the points pushed before each flush instant, nothing later,
/// and two identical runs write identical bytes (determinism).
class ObsFlushCadenceTest : public TestCase
{
  public:
    ObsFlushCadenceTest()
        : TestCase("Influx sink flushes on its period and writes byte-identical files per run")
    {
    }

  private:
    std::string RunOnce(const std::string& path, std::vector<size_t>& seen)
    {
        Ptr<NtnInfluxSink> sink = CreateObject<NtnInfluxSink>();
        sink->SetTransport(NtnInfluxSink::Transport::File);
        sink->SetFilePath(path);
        sink->SetFlushPeriod(MilliSeconds(250));
        sink->Start();
        // a point every 10 ms from t = 5 ms
        for (int k = 0; k < 200; ++k)
        {
            Simulator::Schedule(MilliSeconds(5 + 10 * k), [sink, k]() {
                Point p;
                p.measurement = "tick";
                p.fieldsInt["k"] = k;
                sink->Push(p);
            });
        }
        for (int f = 1; f <= 7; ++f)
        {
            Simulator::Schedule(MilliSeconds(250 * f + 1),
                                [path, &seen]() { seen.push_back(Lines(path).size()); });
        }
        Simulator::Schedule(Seconds(2.0), [sink]() { sink->Stop(); });
        Simulator::Run();
        Simulator::Destroy();
        return Slurp(path);
    }

    void DoRun() override
    {
        std::vector<size_t> seenA;
        std::vector<size_t> seenB;
        const std::string a = RunOnce(CreateTempDirFilename("cad-a.lp"), seenA);
        const std::string b = RunOnce(CreateTempDirFilename("cad-b.lp"), seenB);
        NS_TEST_ASSERT_MSG_EQ(seenA.size(), 7u, "seven flush checkpoints");
        for (size_t f = 0; f < seenA.size(); ++f)
        {
            // points at 5, 15, ... ms strictly before 250 (f+1) ms
            const size_t expect = 25 * (f + 1);
            NS_TEST_ASSERT_MSG_EQ(seenA[f], expect, "flush " << f + 1 << " holds exactly its points");
        }
        NS_TEST_ASSERT_MSG_EQ(Count(a, "\n"), 200u, "Stop() flushes the remainder");
        NS_TEST_ASSERT_MSG_EQ(a, b, "same scenario, same bytes");
    }
};

// ---------------------------------------------------------------------------
/// Failure path: the file cannot be opened. The sink keeps running, and every
/// point it could not write is counted as dropped instead of vanishing.
class ObsUnwritablePathTest : public TestCase
{
  public:
    ObsUnwritablePathTest()
        : TestCase("Influx sink counts points lost to an unwritable file as dropped")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<NtnInfluxSink> sink = CreateObject<NtnInfluxSink>();
        sink->SetTransport(NtnInfluxSink::Transport::File);
        sink->SetFilePath("/nonexistent-dir/ntn/obs.lp");
        sink->SetFlushPeriod(MilliSeconds(100));
        sink->Start();
        for (int k = 0; k < 50; ++k)
        {
            Simulator::Schedule(MilliSeconds(20 * k + 1), [sink]() {
                Point p;
                p.measurement = "m";
                p.fieldsFloat["v"] = 1.0;
                sink->Push(p);
            });
        }
        Simulator::Schedule(Seconds(1.5), [sink]() { sink->Stop(); });
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_EQ(sink->GetEmittedCount(), 50u, "all points were accepted");
        NS_TEST_ASSERT_MSG_EQ(sink->GetDroppedPoints(), 50u, "and all of them are reported lost");
    }
};

// ---------------------------------------------------------------------------
/// Cross-module, packet level: FlowMonitor on a UDP flow over a 10 Mb/s,
/// 5 ms P2P link feeds the sink every 500 ms through the helper. The line
/// protocol must carry the measured counters exactly, and the throughput and
/// delay read back from the file must match the offered load and the link
/// (5 ms + 1030 B (IP + PPP) x 8 / 10 Mb/s = 5.824 ms). Every line carries the run tag.
class ObsFlowMonitorToInfluxTest : public TestCase
{
  public:
    ObsFlowMonitorToInfluxTest()
        : TestCase("FlowMonitor KPIs reach line protocol exactly, tagged with the helper run id")
    {
    }

  private:
    void DoRun() override
    {
        const std::string path = CreateTempDirFilename("flow.lp");
        NodeContainer n;
        n.Create(2);
        InternetStackHelper inet;
        inet.Install(n);
        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
        p2p.SetChannelAttribute("Delay", StringValue("5ms"));
        NetDeviceContainer d = p2p.Install(n);
        Ipv4AddressHelper ip;
        ip.SetBase("10.9.0.0", "255.255.255.0");
        Ipv4InterfaceContainer ifs = ip.Assign(d);
        OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(ifs.GetAddress(1), 5000));
        onoff.SetConstantRate(DataRate("2Mbps"), 1000);
        ApplicationContainer src = onoff.Install(n.Get(0));
        src.Start(Seconds(0.0));
        src.Stop(Seconds(4.0));
        PacketSinkHelper ps("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 5000));
        ApplicationContainer snk = ps.Install(n.Get(1));

        FlowMonitorHelper fmh;
        Ptr<FlowMonitor> fm = fmh.InstallAll();

        NtnObservabilityHelper h;
        h.SetRunId("exp-42");
        h.SetInfluxFile(path);
        h.SetFlushPeriod(Seconds(1));
        Ptr<NtnInfluxSink> sink = h.InstallInfluxSink();
        sink->Start();
        for (int k = 1; k <= 9; ++k)
        {
            Simulator::Schedule(MilliSeconds(500 * k), [fm, sink]() {
                fm->CheckForLostPackets();
                for (const auto& [id, st] : fm->GetFlowStats())
                {
                    Point p;
                    p.measurement = "flow";
                    p.tags["flow"] = std::to_string(id);
                    p.fieldsInt["rx_bytes"] = static_cast<long long>(st.rxBytes);
                    p.fieldsInt["rx_packets"] = static_cast<long long>(st.rxPackets);
                    if (st.rxPackets)
                    {
                        p.fieldsFloat["delay_ms"] =
                            st.delaySum.GetSeconds() * 1e3 / static_cast<double>(st.rxPackets);
                    }
                    sink->Push(p);
                }
            });
        }
        Point own;
        own.measurement = "note";
        own.tags["run"] = "manual";
        own.fieldsInt["x"] = 1;
        Simulator::Schedule(Seconds(4.6), [sink, own]() { sink->Push(own); });
        Simulator::Schedule(Seconds(4.8), [sink]() { sink->Stop(); });
        Simulator::Stop(Seconds(5));
        Simulator::Run();
        uint64_t rxBytes = 0;
        uint64_t rxPackets = 0;
        for (const auto& [id, st] : fm->GetFlowStats())
        {
            rxBytes += st.rxBytes;
            rxPackets += st.rxPackets;
        }
        const uint64_t appRx = DynamicCast<PacketSink>(snk.Get(0))->GetTotalRx();
        Simulator::Destroy();

        const auto lines = Lines(path);
        NS_TEST_ASSERT_MSG_EQ(lines.size(), 10u, "nine samples of one flow plus the manual note");
        std::vector<double> bytes;
        for (const auto& l : lines)
        {
            if (l.rfind("flow,", 0) == 0)
            {
                NS_TEST_ASSERT_MSG_NE(l.find(",run=exp-42"), std::string::npos,
                                      "the helper's run id tags every point");
                std::string b = LpField(l, "rx_bytes");
                b.pop_back(); // the i suffix
                bytes.push_back(std::stod(b));
            }
            else
            {
                NS_TEST_ASSERT_MSG_NE(l.find("run=manual"), std::string::npos,
                                      "an explicit run tag is not overwritten");
            }
        }
        NS_TEST_ASSERT_MSG_EQ(bytes.size(), 9u, "nine flow samples");
        NS_TEST_ASSERT_MSG_EQ(uint64_t(bytes.back()), rxBytes,
                              "the last sample equals FlowMonitor's final count exactly");
        NS_TEST_ASSERT_MSG_EQ(appRx, rxPackets * 1000, "app counter agrees (1000 B payloads)");
        // throughput between 1.0 s and 3.5 s read back from the file:
        // 2 Mb/s of payload is 2.056 Mb/s at the IP layer (1028 B / 1000 B).
        const double mbps = (bytes[6] - bytes[1]) * 8.0 / 2.5 / 1e6;
        NS_TEST_ASSERT_MSG_EQ_TOL(mbps, 2.056, 0.03, "throughput rebuilt from line protocol");
        const double delay = std::stod(LpField(lines[8], "delay_ms"));
        NS_TEST_ASSERT_MSG_EQ_TOL(delay, 5.824, 0.01, "mean one-way delay: 5 ms + serialization");
    }
};

// ---------------------------------------------------------------------------
/// The NetSimulyzer document written during a run is RFC 8259 JSON even when
/// a series is fed NaN/Inf and a log message carries control characters, and
/// its event count equals the calls made.
class ObsNetSimulyzerJsonTest : public TestCase
{
  public:
    ObsNetSimulyzerJsonTest()
        : TestCase("NetSimulyzer export stays valid JSON with NaN samples and hostile messages")
    {
    }

  private:
    void DoRun() override
    {
        const std::string path = CreateTempDirFilename("ns.json");
        Ptr<NtnNetSimulyzerExporter> e = CreateObject<NtnNetSimulyzerExporter>();
        e->SetOutputPath(path);
        e->AddNode(1, "sat", Vector(7, 0, 0));
        e->AddNode(2, "ue", Vector(6.4, 0, 0));
        const uint32_t s = e->AddSeries("SINR", "time (s)", "SINR", "dB");
        e->Start();
        const uint32_t link = e->AddLink(1, 2);
        for (int k = 0; k < 10; ++k)
        {
            Simulator::Schedule(Seconds(k), [e, k, s]() {
                const double t = Simulator::Now().GetSeconds();
                e->NodeMove(1, t, Vector(7.0, 0.1 * k, 0));
                e->SampleSeries(s, t, k == 3 ? std::nan("") : (k == 4 ? INFINITY : 10.0 + k));
            });
        }
        Simulator::Schedule(Seconds(5), [e, link]() {
            e->LinkChange(link, 5.0, false);
            e->LogMessage(2, 5.0, "rlf\n\"cause\"\t\x01");
        });
        Simulator::Schedule(Seconds(10), [e]() { e->Stop(); });
        Simulator::Run();
        Simulator::Destroy();
        const std::string doc = Slurp(path);
        NS_TEST_ASSERT_MSG_EQ(IsJson(doc), true, "the document parses as JSON");
        NS_TEST_ASSERT_MSG_EQ(e->GetEventCount(), 1u + 20u + 2u, "LinkAdd + 2x10 + 2 events");
        NS_TEST_ASSERT_MSG_EQ(Count(doc, "\"value\":null"), 2u, "NaN and Inf become null");
        NS_TEST_ASSERT_MSG_EQ(Count(doc, "\"type\":\"NodeMove\""), 10u, "one move per second");
    }
};

// ---------------------------------------------------------------------------
/// Failure path: the NetSimulyzer output cannot be opened. The recorder keeps
/// running, the CZML (a separate surface) is still written and valid, and the
/// exporter reports no events instead of streaming into a closed file.
class ObsUnopenableExportTest : public TestCase
{
  public:
    ObsUnopenableExportTest()
        : TestCase("An unopenable NetSimulyzer path disables that export only")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<NtnNetSimulyzerExporter> e = CreateObject<NtnNetSimulyzerExporter>();
        e->SetOutputPath("/nonexistent-dir/ns.json");
        e->Start();
        Simulator::Schedule(Seconds(1), [e]() { e->NodeMove(1, 1.0, Vector(0, 0, 0)); });
        const std::string czml = CreateTempDirFilename("ok.czml");
        Ptr<NtnSceneRecorder> rec = CreateObject<NtnSceneRecorder>();
        Ptr<ConstantPositionMobilityModel> m = CreateObject<ConstantPositionMobilityModel>();
        m->SetPosition(Vector(6378137.0, 0, 0));
        rec->TrackMobility(5, m, NtnSceneRecorder::Ue);
        rec->EnableNetSimulyzer("/nonexistent-dir/scene.json");
        rec->EnableCzml(czml);
        rec->Start();
        Simulator::Schedule(Seconds(3.5), [rec, e]() {
            rec->Stop();
            e->Stop();
        });
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_EQ(e->GetEventCount(), 0u, "no events into a file that never opened");
        const std::string doc = Slurp(czml);
        NS_TEST_ASSERT_MSG_EQ(IsJson(doc), true, "the CZML is still written and valid");
        NS_TEST_ASSERT_MSG_EQ(CzmlCartesian(doc, "node-5").size(), 16u, "4 samples x (t,x,y,z)");
    }
};

// ---------------------------------------------------------------------------
/// Cross-module: SGP4 satellites from ntn-constellation tracked through
/// NtnSceneHelper. Every CZML cartesian sample must equal the propagator's
/// ECEF position at that instant (1 m), lie at LEO altitude above WGS-84, and
/// move at an Earth-fixed orbital speed consistent with a 550 km circular
/// orbit (v = sqrt(mu / r) = 7.59 km/s inertial, minus up to 0.5 km/s of
/// Earth rotation in the fixed frame).
class ObsSceneSgp4CzmlTest : public TestCase
{
  public:
    ObsSceneSgp4CzmlTest()
        : TestCase("Scene recorder CZML reproduces SGP4 ECEF positions and LEO kinematics")
    {
    }

  private:
    void DoRun() override
    {
        ntncon::WalkerConfig w;
        w.num_planes = 2;
        w.total_sats = 4;
        w.altitude_km = 550.0;
        w.epoch_unix_s = 1767225600.0;
        auto el = ntncon::WalkerConstellation::BuildDelta(w);
        NodeContainer sats;
        sats.Create(2);
        std::vector<Ptr<ntncon::Sgp4MobilityModel>> mobs;
        for (uint32_t i = 0; i < 2; ++i)
        {
            Ptr<ntncon::Sgp4MobilityModel> m = CreateObject<ntncon::Sgp4MobilityModel>();
            m->SetElementsSgp4(el[i * 2]);
            sats.Get(i)->AggregateObject(m);
            mobs.push_back(m);
        }
        NodeContainer ues;
        ues.Create(1);
        Ptr<ConstantPositionMobilityModel> um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(Vector(6378137.0, 0, 0)); // on the equator at 0 deg E
        ues.Get(0)->AggregateObject(um);

        const std::string czml = CreateTempDirFilename("sgp4.czml");
        NtnSceneHelper sh;
        sh.SetCzml(czml);
        sh.SetSampleInterval(Seconds(10));
        Ptr<NtnSceneRecorder> rec = sh.Build(sats, ues);
        NS_TEST_ASSERT_MSG_NE(rec, nullptr, "a CZML path enables the recorder");

        std::vector<Vector> truth;
        for (int t = 0; t <= 600; t += 10)
        {
            Simulator::Schedule(Seconds(t), [&truth, mobs]() { truth.push_back(mobs[0]->GetPosition()); });
        }
        Simulator::Schedule(MilliSeconds(600500), [rec]() { rec->Stop(); });
        Simulator::Run();
        Simulator::Destroy();

        const std::string doc = Slurp(czml);
        NS_TEST_ASSERT_MSG_EQ(IsJson(doc), true, "CZML parses as JSON");
        NS_TEST_ASSERT_MSG_EQ(doc.find("\"id\":\"document\""), 2u, "the document packet comes first");
        NS_TEST_ASSERT_MSG_NE(doc.find("\"interval\":\"2026-01-01T00:00:00Z/2026-01-01T00:10:00Z\""),
                              std::string::npos, "clock interval spans the 600 s run");
        const auto c = CzmlCartesian(doc, "node-" + std::to_string(sats.Get(0)->GetId()));
        NS_TEST_ASSERT_MSG_EQ(c.size(), 61u * 4, "61 samples of (t, x, y, z)");
        NS_TEST_ASSERT_MSG_EQ(truth.size(), 61u, "61 propagator probes");
        if (c.size() != 61u * 4 || truth.size() != 61u)
        {
            return;
        }
        for (size_t i = 0; i < 61; ++i)
        {
            const Vector p(c[4 * i + 1], c[4 * i + 2], c[4 * i + 3]);
            NS_TEST_ASSERT_MSG_EQ_TOL(c[4 * i], 10.0 * i, 1e-9, "sample time offset");
            NS_TEST_ASSERT_MSG_EQ_TOL(CalculateDistance(p, truth[i]), 0.0, 1.0,
                                      "CZML sample equals the SGP4 ECEF position");
            const double r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            NS_TEST_ASSERT_MSG_EQ_TOL(r - 6371e3, 550e3, 30e3, "LEO altitude");
            if (i > 0)
            {
                const Vector q(c[4 * i - 3], c[4 * i - 2], c[4 * i - 1]);
                const double v = CalculateDistance(p, q) / 10.0;
                NS_TEST_ASSERT_MSG_GT(v, 7000.0, "Earth-fixed speed lower bound");
                NS_TEST_ASSERT_MSG_LT(v, 7700.0, "Earth-fixed speed upper bound");
            }
        }
        NS_TEST_ASSERT_MSG_EQ(Count(doc, "\"path\":"), 2u, "satellites draw an orbit trail");
    }
};

// ---------------------------------------------------------------------------
/// Handover and link events at sub-second instants: CZML availability keeps
/// the milliseconds (ISO 8601), a NaN KPI is not written, and the
/// NetSimulyzer link events follow the serving edge.
class ObsCzmlSubSecondAvailabilityTest : public TestCase
{
  public:
    ObsCzmlSubSecondAvailabilityTest()
        : TestCase("CZML availability keeps sub-second handover times; NaN KPIs are skipped")
    {
    }

  private:
    void DoRun() override
    {
        const std::string czml = CreateTempDirFilename("ho.czml");
        const std::string nsj = CreateTempDirFilename("ho.json");
        Ptr<NtnSceneRecorder> rec = CreateObject<NtnSceneRecorder>();
        auto mk = [](double x) {
            Ptr<ConstantPositionMobilityModel> m = CreateObject<ConstantPositionMobilityModel>();
            m->SetPosition(Vector(x, 0, 0));
            return m;
        };
        rec->TrackMobility(1, mk(6378137.0 + 550e3), NtnSceneRecorder::Sat);
        rec->TrackMobility(2, mk(6378137.0 + 600e3), NtnSceneRecorder::Sat);
        rec->TrackMobility(3, mk(6378137.0), NtnSceneRecorder::Ue);
        rec->TrackBeam(1, 3);
        const uint32_t sinr = rec->TrackKpiSeries(3, NtnSceneRecorder::Sinr);
        rec->EnableCzml(czml);
        rec->EnableNetSimulyzer(nsj);
        rec->Start();
        Simulator::Schedule(MilliSeconds(1000), [rec, sinr]() { rec->RecordKpi(sinr, 8.0); });
        Simulator::Schedule(MilliSeconds(2000), [rec, sinr]() { rec->RecordKpi(sinr, std::nan("")); });
        Simulator::Schedule(MilliSeconds(2500), [rec]() { rec->OnHandover(3, 1, 2); });
        Simulator::Schedule(MilliSeconds(4250), [rec]() { rec->OnLinkChange(2, 3, false); });
        Simulator::Schedule(Seconds(6), [rec]() { rec->Stop(); });
        Simulator::Run();
        Simulator::Destroy();

        const std::string doc = Slurp(czml);
        NS_TEST_ASSERT_MSG_EQ(IsJson(doc), true, "CZML is valid JSON");
        NS_TEST_ASSERT_MSG_NE(
            doc.find("\"availability\":[\"2026-01-01T00:00:00Z/2026-01-01T00:00:02.500Z\"]"),
            std::string::npos, "old serving edge ends at the 2.5 s handover");
        NS_TEST_ASSERT_MSG_NE(
            doc.find("\"availability\":[\"2026-01-01T00:00:02.500Z/2026-01-01T00:00:04.250Z\"]"),
            std::string::npos, "new edge is up from 2.5 s until the 4.25 s link loss");
        NS_TEST_ASSERT_MSG_EQ(doc.find("nan"), std::string::npos, "no NaN literal in the scene");
        NS_TEST_ASSERT_MSG_NE(doc.find("\"number\":[1,8]"), std::string::npos,
                              "only the finite KPI sample is written");
        const std::string ns = Slurp(nsj);
        NS_TEST_ASSERT_MSG_EQ(IsJson(ns), true, "NetSimulyzer document is valid JSON");
        NS_TEST_ASSERT_MSG_EQ(Count(ns, "\"type\":\"LinkChange\""), 3u,
                              "old edge down, new edge up, new edge down");
    }
};

// ---------------------------------------------------------------------------
/// Determinism of the whole observability surface: the same scenario run
/// twice writes byte-identical CZML, NetSimulyzer and line-protocol files.
class ObsSceneDeterminismTest : public TestCase
{
  public:
    ObsSceneDeterminismTest()
        : TestCase("Scene, NetSimulyzer and line-protocol outputs are byte-identical across runs")
    {
    }

  private:
    void RunOnce(const std::string& tag, std::string& czml, std::string& ns, std::string& lp)
    {
        ntncon::WalkerConfig w;
        w.num_planes = 1;
        w.total_sats = 3;
        w.altitude_km = 600.0;
        w.epoch_unix_s = 1767225600.0;
        auto el = ntncon::WalkerConstellation::BuildDelta(w);
        Ptr<NtnSceneRecorder> rec = CreateObject<NtnSceneRecorder>();
        for (uint32_t i = 0; i < 3; ++i)
        {
            Ptr<ntncon::Sgp4MobilityModel> m = CreateObject<ntncon::Sgp4MobilityModel>();
            m->SetElementsSgp4(el[i]);
            rec->TrackMobility(i, m, NtnSceneRecorder::Sat);
        }
        const std::string cz = CreateTempDirFilename("det-" + tag + ".czml");
        const std::string nj = CreateTempDirFilename("det-" + tag + ".json");
        const std::string lpf = CreateTempDirFilename("det-" + tag + ".lp");
        rec->EnableCzml(cz);
        rec->EnableNetSimulyzer(nj);
        rec->SetSampleInterval(Seconds(5));
        Ptr<NtnInfluxSink> sink = CreateObject<NtnInfluxSink>();
        sink->SetTransport(NtnInfluxSink::Transport::File);
        sink->SetFilePath(lpf);
        rec->Start();
        sink->Start();
        for (int t = 1; t < 60; t += 7)
        {
            Simulator::Schedule(Seconds(t), [sink]() {
                Point p;
                p.measurement = "orbit";
                p.fieldsFloat["t"] = Simulator::Now().GetSeconds();
                sink->Push(p);
            });
        }
        Simulator::Schedule(Seconds(60), [rec, sink]() {
            rec->Stop();
            sink->Stop();
        });
        Simulator::Run();
        Simulator::Destroy();
        czml = Slurp(cz);
        ns = Slurp(nj);
        lp = Slurp(lpf);
    }

    void DoRun() override
    {
        std::string c1;
        std::string n1;
        std::string l1;
        std::string c2;
        std::string n2;
        std::string l2;
        RunOnce("a", c1, n1, l1);
        RunOnce("b", c2, n2, l2);
        NS_TEST_ASSERT_MSG_GT(c1.size(), 1000u, "CZML written");
        NS_TEST_ASSERT_MSG_EQ(c1, c2, "CZML identical");
        NS_TEST_ASSERT_MSG_EQ(n1, n2, "NetSimulyzer identical");
        NS_TEST_ASSERT_MSG_EQ(l1, l2, "line protocol identical");
        NS_TEST_ASSERT_MSG_EQ(IsJson(n1), true, "NetSimulyzer valid");
    }
};

class NtnObservabilityInDepthTestSuite : public TestSuite
{
  public:
    NtnObservabilityInDepthTestSuite()
        : TestSuite("ntn-observability-indepth", Type::UNIT)
    {
        AddTestCase(new ObsLineProtocolConformanceTest, Duration::QUICK);
        AddTestCase(new ObsFlushCadenceTest, Duration::QUICK);
        AddTestCase(new ObsUnwritablePathTest, Duration::QUICK);
        AddTestCase(new ObsFlowMonitorToInfluxTest, Duration::QUICK);
        AddTestCase(new ObsNetSimulyzerJsonTest, Duration::QUICK);
        AddTestCase(new ObsUnopenableExportTest, Duration::QUICK);
        AddTestCase(new ObsSceneSgp4CzmlTest, Duration::QUICK);
        AddTestCase(new ObsCzmlSubSecondAvailabilityTest, Duration::QUICK);
        AddTestCase(new ObsSceneDeterminismTest, Duration::QUICK);
    }
};

static NtnObservabilityInDepthTestSuite g_ntnObservabilityInDepthTestSuite;
