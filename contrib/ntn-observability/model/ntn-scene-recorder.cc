/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "ntn-scene-recorder.h"

#include <ns3/abort.h>
#include <ns3/log.h>
#include <ns3/mobility-model.h>
#include <ns3/simulator.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnSceneRecorder");

namespace ntnobs
{

namespace
{
// WGS84.
constexpr double kA = 6378137.0;
constexpr double kE2 = 6.69437999014e-3;
constexpr double kEarthRadius = 6371000.0;

Vector
GeodeticToEcefLocal(double latDeg, double lonDeg, double altM)
{
    const double phi = latDeg * M_PI / 180.0;
    const double lam = lonDeg * M_PI / 180.0;
    const double sphi = std::sin(phi);
    const double cphi = std::cos(phi);
    const double slam = std::sin(lam);
    const double clam = std::cos(lam);
    const double n = kA / std::sqrt(1.0 - kE2 * sphi * sphi);
    return Vector((n + altM) * cphi * clam,
                  (n + altM) * cphi * slam,
                  (n * (1.0 - kE2) + altM) * sphi);
}

std::string
FmtD(double v)
{
    std::ostringstream ss;
    ss << std::setprecision(12) << v;
    return ss.str();
}

// Civil date from a day count since 1970-01-01 (Howard Hinnant's algorithm),
// used to render a CZML ISO epoch without needing wall-clock time.
void
CivilFromDays(long z, int& y, unsigned& m, unsigned& d)
{
    z += 719468;
    const long era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = static_cast<unsigned>(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const long yy = static_cast<long>(yoe) + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    d = doy - (153 * mp + 2) / 5 + 1;
    m = mp < 10 ? mp + 3 : mp - 9;
    y = static_cast<int>(yy + (m <= 2));
}

// ISO-8601 UTC string for an absolute Unix time (seconds since 1970-01-01).
std::string
IsoFromUnix(double unixSec)
{
    long total = static_cast<long>(std::floor(unixSec));
    long days = total / 86400;
    long rem = total % 86400;
    if (rem < 0)
    {
        rem += 86400;
        days -= 1;
    }
    int y;
    unsigned mo;
    unsigned d;
    CivilFromDays(days, y, mo, d);
    const int hh = static_cast<int>(rem / 3600);
    const int mm = static_cast<int>((rem % 3600) / 60);
    const int sscnd = static_cast<int>(rem % 60);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d-%02u-%02uT%02d:%02d:%02dZ", y, mo, d, hh, mm, sscnd);
    return std::string(buf);
}

} // namespace

NS_OBJECT_ENSURE_REGISTERED(NtnSceneRecorder);

TypeId
NtnSceneRecorder::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntnobs::NtnSceneRecorder")
                            .SetParent<Object>()
                            .SetGroupName("NtnObservability")
                            .AddConstructor<NtnSceneRecorder>();
    return tid;
}

NtnSceneRecorder::NtnSceneRecorder() = default;

NtnSceneRecorder::~NtnSceneRecorder() = default;

void
NtnSceneRecorder::DoDispose()
{
    if (m_running)
    {
        Stop();
    }
    m_exporter = nullptr;
    Object::DoDispose();
}

void
NtnSceneRecorder::SetEnuReference(double lat0Deg, double lon0Deg, double alt0M)
{
    m_lat0 = lat0Deg;
    m_lon0 = lon0Deg;
    m_alt0 = alt0M;
    m_enuOriginEcef = GeodeticToEcefLocal(lat0Deg, lon0Deg, alt0M);
    const double phi = lat0Deg * M_PI / 180.0;
    const double lam = lon0Deg * M_PI / 180.0;
    const double sphi = std::sin(phi);
    const double cphi = std::cos(phi);
    const double slam = std::sin(lam);
    const double clam = std::cos(lam);
    m_enuEast = Vector(-slam, clam, 0.0);
    m_enuNorth = Vector(-sphi * clam, -sphi * slam, cphi);
    m_enuUp = Vector(cphi * clam, cphi * slam, sphi);
}

Vector
NtnSceneRecorder::ToEcef(const Vector& p) const
{
    if (m_frame == EcefGlobal)
    {
        return p;
    }
    // LocalEnu: ECEF = origin + e*East + n*North + u*Up.
    return Vector(m_enuOriginEcef.x + p.x * m_enuEast.x + p.y * m_enuNorth.x + p.z * m_enuUp.x,
                  m_enuOriginEcef.y + p.x * m_enuEast.y + p.y * m_enuNorth.y + p.z * m_enuUp.y,
                  m_enuOriginEcef.z + p.x * m_enuEast.z + p.y * m_enuNorth.z + p.z * m_enuUp.z);
}

std::string
NtnSceneRecorder::ModelName(NodeKind kind)
{
    switch (kind)
    {
    case Sat: return "sat";
    case Ue: return "ue";
    case Gateway: return "gateway";
    case Uav: return "uav";
    case Haps: return "haps";
    case Ship: return "ship";
    case Aircraft: return "aircraft";
    case Ris: return "ris";
    case Generic:
    default: return "sphere";
    }
}

uint32_t
NtnSceneRecorder::TrackNode(Ptr<Node> node, NodeKind kind, const std::string& label)
{
    Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
    NS_ASSERT_MSG(mob, "TrackNode: node " << node->GetId() << " has no MobilityModel");
    return TrackMobility(node->GetId(), mob, kind, label);
}

uint32_t
NtnSceneRecorder::TrackMobility(uint32_t id,
                                Ptr<MobilityModel> mob,
                                NodeKind kind,
                                const std::string& label)
{
    NS_ASSERT_MSG(!m_running, "TrackMobility must be called before Start()");
    NS_ABORT_MSG_IF(m_tracks.count(id) != 0,
                    "TrackMobility: id " << id << " is already tracked; "
                    "re-tracking would silently overwrite the existing track");
    const std::string lbl = label.empty() ? (ModelName(kind) + "-" + std::to_string(id)) : label;
    m_nodes.push_back({id, mob, kind, lbl});
    m_tracks[id] = Track{id, kind, lbl, {}};
    return id;
}

void
NtnSceneRecorder::TrackBeam(uint32_t fromId, uint32_t toId)
{
    m_beams.push_back({fromId, toId});
}

uint32_t
NtnSceneRecorder::TrackKpiSeries(uint32_t nodeId, KpiKind kind, const std::string& name)
{
    NS_ASSERT_MSG(!m_running, "TrackKpiSeries must be called before Start()");
    const uint32_t seriesId = static_cast<uint32_t>(m_kpis.size()) + 1;
    std::string nm = name;
    if (nm.empty())
    {
        const char* base = (kind == Sinr      ? "SINR"
                            : kind == Throughput ? "Throughput"
                            : kind == Tbler      ? "TBLER"
                                                 : "Delay");
        nm = std::string(base) + "-" + std::to_string(nodeId);
    }
    m_kpis.push_back(KpiSeries{seriesId, 0, nodeId, kind, nm, {}});
    return seriesId;
}

void
NtnSceneRecorder::EnableNetSimulyzer(const std::string& path, double sceneScale)
{
    m_netSimEnabled = true;
    m_netSimPath = path;
    m_sceneScale = sceneScale;
}

void
NtnSceneRecorder::EnableCzml(const std::string& path)
{
    m_czmlEnabled = true;
    m_czmlPath = path;
}

void
NtnSceneRecorder::Start()
{
    NS_ASSERT_MSG(!m_running, "Start called twice");
    m_running = true;
    m_t0Sec = Simulator::Now().GetSeconds();
    m_tLastSec = m_t0Sec;

    if (m_netSimEnabled)
    {
        m_exporter = CreateObject<NtnNetSimulyzerExporter>();
        m_exporter->SetOutputPath(m_netSimPath);
        // Earth sphere at the origin (NetSimulyzer has no globe).
        m_exporter->AddNode(UINT32_MAX, "earth", Vector(0, 0, 0), kEarthRadius * m_sceneScale);
        for (auto& n : m_nodes)
        {
            const Vector ecef = ToEcef(n.mob->GetPosition());
            m_exporter->AddNode(n.id,
                                ModelName(n.kind),
                                Vector(ecef.x * m_sceneScale,
                                       ecef.y * m_sceneScale,
                                       ecef.z * m_sceneScale),
                                1.0);
        }
        for (auto& k : m_kpis)
        {
            const char* unit = (k.kind == Sinr      ? "dB"
                               : k.kind == Throughput ? "Mbps"
                               : k.kind == Delay      ? "ms"
                                                      : "");
            k.exporterIdx = m_exporter->AddSeries(k.name, "time (s)", k.name, unit);
        }
        m_exporter->Start();
    }

    Poll(); // first sample at t0, reschedules itself
}

void
NtnSceneRecorder::Poll()
{
    if (!m_running)
    {
        return;
    }
    const double t = Simulator::Now().GetSeconds();
    std::ostringstream live;
    if (m_liveStdout)
    {
        live << R"(##NTNSCENE## {"t":)" << FmtD(t) << R"(,"nodes":[)";
    }
    bool first = true;
    for (auto& n : m_nodes)
    {
        const Vector ecef = ToEcef(n.mob->GetPosition());
        m_tracks[n.id].samples.emplace_back(t, ecef);
        if (m_exporter)
        {
            m_exporter->NodeMove(
                n.id,
                t,
                Vector(ecef.x * m_sceneScale, ecef.y * m_sceneScale, ecef.z * m_sceneScale));
        }
        if (m_liveStdout)
        {
            if (!first)
            {
                live << ",";
            }
            first = false;
            live << R"({"id":)" << n.id << R"(,"k":")" << ModelName(n.kind) << R"(","p":[)"
                 << FmtD(ecef.x) << "," << FmtD(ecef.y) << "," << FmtD(ecef.z) << "]}";
        }
        ++m_eventCount;
    }
    if (m_liveStdout)
    {
        live << "]}";
        std::cout << live.str() << std::endl; // flush each frame for live consumers
        // Emit the COMMUNICATION link layer alongside the node frame: the
        // serving/beam/ISL edges declared via TrackBeam (previously collected but
        // never surfaced). Downstream (live_scene.py) turns these into polylines
        // that reference the two node positions, so the real network is shown
        // instead of a client-side nearest-satellite guess.
        if (!m_beams.empty())
        {
            std::ostringstream lk;
            lk << R"(##NTNSCENE_LINK## {"t":)" << FmtD(t) << R"(,"edges":[)";
            bool lf = true;
            for (const auto& b : m_beams)
            {
                if (!lf)
                {
                    lk << ",";
                }
                lf = false;
                lk << "[" << b.fromId << "," << b.toId << R"(,"serving"])";
            }
            lk << "]}";
            std::cout << lk.str() << std::endl;
        }
    }
    m_tLastSec = t;
    Simulator::Schedule(m_sampleDt, &NtnSceneRecorder::Poll, this);
}

void
NtnSceneRecorder::RecordKpi(uint32_t seriesId, double value)
{
    if (seriesId == 0 || seriesId > m_kpis.size())
    {
        return;
    }
    auto& k = m_kpis[seriesId - 1];
    const double t = Simulator::Now().GetSeconds();
    k.samples.emplace_back(t, value);
    if (m_exporter && k.exporterIdx != 0)
    {
        m_exporter->SampleSeries(k.exporterIdx, t, value);
    }
    ++m_eventCount;
}

void
NtnSceneRecorder::OnHandover(uint32_t ueNodeId, uint32_t fromId, uint32_t toId)
{
    const double t = Simulator::Now().GetSeconds();
    std::ostringstream ss;
    ss << "handover ue=" << ueNodeId << " " << fromId << "->" << toId;
    if (m_exporter)
    {
        m_exporter->LogMessage(ueNodeId, t, ss.str());
    }
    m_handovers.push_back({t, ueNodeId, fromId, toId});
    if (m_liveStdout)
    {
        std::cout << R"(##NTNSCENE_HO## {"t":)" << FmtD(t) << R"(,"ue":)" << ueNodeId
                  << R"(,"from":)" << fromId << R"(,"to":)" << toId << "}" << std::endl;
    }
    ++m_eventCount;
}

bool
NtnSceneRecorder::PositionAt(uint32_t id, double t, Vector& out) const
{
    auto it = m_tracks.find(id);
    if (it == m_tracks.end() || it->second.samples.empty())
    {
        return false;
    }
    const auto& s = it->second.samples;
    // Nearest sample by time (samples are appended in time order).
    size_t best = 0;
    double bestDt = std::abs(s[0].first - t);
    for (size_t i = 1; i < s.size(); ++i)
    {
        const double d = std::abs(s[i].first - t);
        if (d < bestDt)
        {
            bestDt = d;
            best = i;
        }
    }
    out = s[best].second;
    return true;
}

void
NtnSceneRecorder::OnLinkChange(uint32_t aId, uint32_t bId, bool up)
{
    const double t = Simulator::Now().GetSeconds();
    std::ostringstream ss;
    ss << "link " << aId << "<->" << bId << (up ? " up" : " down");
    if (m_exporter)
    {
        m_exporter->LogMessage(aId, t, ss.str());
    }
    // Record the transition so WriteCzml() can gate the corresponding polyline's
    // `availability` interval(s).
    m_linkEvents.push_back({t, aId, bId, up});
    if (m_liveStdout)
    {
        std::cout << R"(##NTNSCENE_LINKCHG## {"t":)" << FmtD(t) << R"(,"a":)" << aId << R"(,"b":)"
                  << bId << R"(,"up":)" << (up ? "true" : "false") << "}" << std::endl;
    }
    ++m_eventCount;
}

void
NtnSceneRecorder::Stop()
{
    if (!m_running)
    {
        return;
    }
    m_running = false;
    if (m_exporter)
    {
        m_exporter->Stop();
    }
    if (m_czmlEnabled)
    {
        WriteCzml();
    }
}

void
NtnSceneRecorder::WriteCzml()
{
    std::ofstream out(m_czmlPath, std::ios::out | std::ios::trunc);
    if (!out)
    {
        NS_LOG_WARN("could not open CZML output " << m_czmlPath);
        return;
    }
    const double span = std::max(1.0, m_tLastSec - m_t0Sec);
    const std::string epoch = IsoFromUnix(m_sceneEpochUnix);
    const std::string epochEnd = IsoFromUnix(m_sceneEpochUnix + span);

    out << "[";
    // Document packet with a looping clock.
    out << R"({"id":"document","name":"ntn-scene","version":"1.0",)"
        << R"("clock":{"interval":")" << epoch << "/" << epochEnd << R"(",)"
        << R"("currentTime":")" << epoch << R"(","multiplier":60,)"
        << R"("range":"LOOP_STOP","step":"SYSTEM_CLOCK_MULTIPLIER"}})";

    for (auto& kv : m_tracks)
    {
        const Track& tr = kv.second;
        if (tr.samples.empty())
        {
            continue;
        }
        const bool isSat = (tr.kind == Sat);
        out << ",";
        out << R"({"id":"node-)" << tr.id << R"(","name":")" << tr.label << R"(",)";
        out << R"("point":{"pixelSize":)" << (isSat ? "8" : "6")
            << R"(,"color":{"rgba":[)"
            << (isSat ? "255,210,80,255" : "80,200,255,255") << "]}}";
        // Path trail for satellites.
        if (isSat)
        {
            out << R"(,"path":{"leadTime":0,"trailTime":600,"width":1,)"
                << R"("material":{"solidColor":{"color":{"rgba":[255,210,80,120]}}}})";
        }
        // Measured KPI series that belong to this node, emitted as time-tagged
        // CZML custom `properties` (previously accumulated in KpiSeries::samples
        // but never written out). Cesium exposes these for entity inspection /
        // property-driven styling; consumers read node-<id>#properties.<name>.
        bool wroteProps = false;
        for (const auto& k : m_kpis)
        {
            if (k.nodeId != tr.id || k.samples.empty())
            {
                continue;
            }
            out << (wroteProps ? "," : R"(,"properties":{)");
            wroteProps = true;
            out << R"(")" << k.name << R"(":{"epoch":")" << epoch << R"(","number":[)";
            for (size_t i = 0; i < k.samples.size(); ++i)
            {
                if (i > 0)
                {
                    out << ",";
                }
                out << FmtD(k.samples[i].first - m_t0Sec) << "," << FmtD(k.samples[i].second);
            }
            out << "]}";
        }
        if (wroteProps)
        {
            out << "}";
        }
        out << R"(,"position":{"referenceFrame":"FIXED","epoch":")" << epoch
            << R"(","cartesian":[)";
        for (size_t i = 0; i < tr.samples.size(); ++i)
        {
            if (i > 0)
            {
                out << ",";
            }
            const double dt = tr.samples[i].first - m_t0Sec;
            const Vector& e = tr.samples[i].second;
            out << FmtD(dt) << "," << FmtD(e.x) << "," << FmtD(e.y) << "," << FmtD(e.z);
        }
        out << "]}}";
    }

    // Communication links: one CZML polyline per declared beam/serving edge, its
    // endpoints REFERENCING the two node position properties so the link tracks
    // the moving satellites. `availability` is gated by OnLinkChange up/down
    // transitions (an edge with no transitions is up for the whole run). This is
    // what makes the offline globe show the network instead of bare nodes.
    for (const auto& b : m_beams)
    {
        auto ita = m_tracks.find(b.fromId);
        auto itb = m_tracks.find(b.toId);
        if (ita == m_tracks.end() || itb == m_tracks.end() || ita->second.samples.empty() ||
            itb->second.samples.empty())
        {
            continue; // cannot reference a node that has no position samples
        }
        // Build up-intervals (in sim seconds relative to t0) from the transitions
        // that touch this edge. A declared beam starts UP.
        std::vector<std::pair<double, double>> intervals;
        double curStart = 0.0;
        bool open = true;
        for (const auto& ev : m_linkEvents)
        {
            const bool sameEdge = (ev.aId == b.fromId && ev.bId == b.toId) ||
                                  (ev.aId == b.toId && ev.bId == b.fromId);
            if (!sameEdge)
            {
                continue;
            }
            const double off = ev.t - m_t0Sec;
            if (ev.up && !open)
            {
                curStart = off;
                open = true;
            }
            else if (!ev.up && open)
            {
                intervals.emplace_back(curStart, off);
                open = false;
            }
        }
        if (open)
        {
            intervals.emplace_back(curStart, span);
        }
        if (intervals.empty())
        {
            continue; // link never up -> nothing to render
        }
        out << ",";
        out << R"({"id":"link-)" << b.fromId << "-" << b.toId << R"(","name":"link )" << b.fromId
            << "<->" << b.toId << R"(",)";
        out << R"("availability":[)";
        for (size_t i = 0; i < intervals.size(); ++i)
        {
            if (i > 0)
            {
                out << ",";
            }
            out << R"(")" << IsoFromUnix(m_sceneEpochUnix + intervals[i].first) << "/"
                << IsoFromUnix(m_sceneEpochUnix + intervals[i].second) << R"(")";
        }
        out << "]";
        out << R"(,"polyline":{"width":1,"followSurface":false,)"
            << R"("material":{"solidColor":{"color":{"rgba":[80,255,120,180]}}},)"
            << R"("positions":{"references":["node-)" << b.fromId << R"(#position","node-)"
            << b.toId << R"(#position"]}}})";
    }

    // Handover arcs: a brief red polyline flash from the UE to the target node
    // at the handover time. Rendered via CZML `availability` so it appears only
    // around the event on the shared clock.
    for (size_t k = 0; k < m_handovers.size(); ++k)
    {
        const HandoverArc& h = m_handovers[k];
        Vector uePos;
        Vector toPos;
        if (!PositionAt(h.ueId, h.t, uePos) || !PositionAt(h.toId, h.t, toPos))
        {
            continue;
        }
        const double off = h.t - m_t0Sec;
        const std::string a0 = IsoFromUnix(m_sceneEpochUnix + std::max(0.0, off - 0.5));
        const std::string a1 = IsoFromUnix(m_sceneEpochUnix + off + 2.0);
        out << ",";
        out << R"({"id":"ho-)" << k << R"(","name":"handover ue )" << h.ueId << " " << h.fromId
            << "->" << h.toId << R"(","availability":")" << a0 << "/" << a1 << R"(",)"
            << R"("polyline":{"width":3,"followSurface":false,)"
            << R"("material":{"solidColor":{"color":{"rgba":[255,80,80,255]}}},)"
            << R"("positions":{"referenceFrame":"FIXED","cartesian":[)"
            << FmtD(uePos.x) << "," << FmtD(uePos.y) << "," << FmtD(uePos.z) << ","
            << FmtD(toPos.x) << "," << FmtD(toPos.y) << "," << FmtD(toPos.z) << "]}}}";
    }
    out << "]";
    out.close();
}

uint64_t
NtnSceneRecorder::GetEventCount() const
{
    return m_eventCount;
}

} // namespace ntnobs
} // namespace ns3
