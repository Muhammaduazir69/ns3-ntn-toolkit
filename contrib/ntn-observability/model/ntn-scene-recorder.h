/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * NtnSceneRecorder — the single scene-trace spine for the toolkit's 3D / live
 * visualization (NetSimulyzer + Cesium globe + InfluxDB).
 *
 * The realization behind it: NetSimulyzer 3D, the live Cesium globe and
 * "per-example realistic mobility" are not three projects but ONE capability
 * with three consumers. Every node already owns a real MobilityModel (positions)
 * and, since NtnRealStackHelper, real measured KPIs. What was missing is a
 * single tap that reads positions + KPIs + discrete events over time and fans
 * them out. Build that once and ~60 examples light up at once, instead of
 * hand-wiring each export.
 *
 *            real MobilityModel positions  ┐
 *            (sat / UE / GW / UAV / HAPS)  ├─► NtnSceneRecorder ─► NetSimulyzer JSON
 *   measured KPIs + events (SINR, thrpt,   │     (one tap,      ─► CZML (Cesium globe)
 *   handover, link up/down) via RecordKpi  ┘   many sinks)      ─► InfluxDB (planned)
 *
 * Frames. Positions read from a node's MobilityModel are in one of two frames,
 * declared per example:
 *   - EcefGlobal: GetPosition() already returns ECEF metres (ntn-cho / rrc /
 *     fapi / slice / traffic and the oran real-stack examples).
 *   - LocalEnu:   GetPosition() returns local-ENU metres about a geodetic
 *     origin (the ~40 examples that wrap the satellite in
 *     NtnEnuProjectionMobilityModel). Set the origin with SetEnuReference().
 * The recorder converts everything to ECEF internally, then emits:
 *   - CZML  as Cartesian positions in the FIXED (ECEF) reference frame — Cesium
 *     consumes this directly, no geodetic conversion needed.
 *   - NetSimulyzer as uniformly-scaled ECEF (scene metres), with an Earth-sphere
 *     node at the origin, since NetSimulyzer is local-Cartesian only (no globe).
 *
 * KPIs and events are pushed in by the caller from its existing trace sinks
 * (RecordKpi / OnHandover / OnLinkChange), so the recorder needs no dependency
 * on mmwave / ntn-traffic.
 */

#ifndef NTN_SCENE_RECORDER_H
#define NTN_SCENE_RECORDER_H

#include "ntn-netsimulyzer-exporter.h"

#include <ns3/node.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/ptr.h>
#include <ns3/vector.h>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ns3
{

class MobilityModel;

namespace ntnobs
{

/**
 * \ingroup ntn-observability
 * \brief One tap on real mobility + measured KPIs, fanned out to NetSimulyzer
 *        JSON, Cesium CZML, and (planned) InfluxDB.
 */
class NtnSceneRecorder : public Object
{
  public:
    /// Semantic kind of a tracked node (drives the 3D model + CZML styling).
    enum NodeKind
    {
        Sat,
        Ue,
        Gateway,
        Uav,
        Haps,
        Ship,
        Aircraft,
        Ris,
        Generic
    };

    /// Frame in which the tracked MobilityModels report positions.
    enum SceneFrame
    {
        EcefGlobal,
        LocalEnu
    };

    /// KPI kind for a per-node value series (label + unit defaults).
    enum KpiKind
    {
        Sinr,
        Throughput,
        Tbler,
        Delay
    };

    static TypeId GetTypeId();
    NtnSceneRecorder();
    ~NtnSceneRecorder() override;

    // ---- frame -----------------------------------------------------------
    /// Declare the frame of the tracked positions (default EcefGlobal).
    void SetFrame(SceneFrame frame) { m_frame = frame; }
    /// For LocalEnu: the geodetic origin of the scenario's ENU frame.
    void SetEnuReference(double lat0Deg, double lon0Deg, double alt0M);

    // ---- registration (before Start) ------------------------------------
    /// Track a node's position over time. Returns the scene node id (== ns-3
    /// node id). \p label is shown on the timeline/legend.
    uint32_t TrackNode(Ptr<Node> node, NodeKind kind, const std::string& label = "");
    /// Track a node by an explicit MobilityModel (e.g. a candidate satellite
    /// that is not the node's aggregated model).
    uint32_t TrackMobility(uint32_t id,
                           Ptr<MobilityModel> mob,
                           NodeKind kind,
                           const std::string& label = "");
    /// Declare a beam/serving link from one tracked node to another. Each
    /// declared edge is rendered as a CZML polyline whose endpoints REFERENCE
    /// the two tracked node positions (so the link tracks the moving satellites),
    /// and — on the live stdout stream — as a `##NTNSCENE_LINK##` edge. Its CZML
    /// `availability` is gated by OnLinkChange up/down transitions; an edge with
    /// no OnLinkChange events is shown for the whole run.
    void TrackBeam(uint32_t fromId, uint32_t toId);
    /// Declare a per-node KPI series; returns a series id used by RecordKpi().
    uint32_t TrackKpiSeries(uint32_t nodeId, KpiKind kind, const std::string& name = "");

    // ---- sinks (before Start) -------------------------------------------
    /// Write a NetSimulyzer JSON trace. \p sceneScale maps ECEF metres to scene
    /// units (default 1e-6 so a ~7000 km ECEF fits a unit scene); an Earth
    /// sphere is emitted at the origin.
    void EnableNetSimulyzer(const std::string& path, double sceneScale = 1e-6);
    /// Write a Cesium CZML document (Cartesian FIXED-frame samples).
    void EnableCzml(const std::string& path);
    /// Emit one compact JSON scene frame per sample tick to stdout, prefixed
    /// with "##NTNSCENE##", so a host (the control-center backend) can stream
    /// the globe live DURING the run instead of only after it. Positions are
    /// real ECEF metres. Handovers are emitted as "##NTNSCENE_HO##" lines.
    void EnableLiveStdout(bool on = true) { m_liveStdout = on; }

    /// Set the scenario epoch (Unix seconds) used as t=0 for the CZML clock and
    /// FIXED-frame position samples. Default is 2026-01-01T00:00:00Z. Pass the
    /// scenario's TLE epoch (e.g. WalkerConfig::epoch_unix_s) so Cesium's sun /
    /// lighting matches the orbital epoch instead of being a year off.
    /// OBS-14: the CZML scene epoch, 2026-01-01T00:00:00Z.
    ///
    /// Named rather than a bare literal so a scenario aligning the scene with
    /// another export surface has something to refer to. The three surfaces
    /// anchor time differently on purpose (see NtnInfluxSink::TimeAnchorNote);
    /// what was missing is that none of it was written down or tested.
    static constexpr double kDefaultSceneEpochUnix = 1767225600.0;
    void SetSceneEpochUnix(double unixSeconds) { m_sceneEpochUnix = unixSeconds; }
    double GetSceneEpochUnix() const { return m_sceneEpochUnix; }

    // ---- lifecycle -------------------------------------------------------
    /// Position sampling interval (default 1 s).
    void SetSampleInterval(Time dt) { m_sampleDt = dt; }
    /// Begin recording (registers nodes/series with the sinks, schedules polls).
    void Start();
    /// Stop recording and finalize the sink files.
    void Stop();

    // ---- runtime pushes (between Start and Stop) -------------------------
    /// Sample a KPI value at Simulator::Now() for a series from TrackKpiSeries.
    void RecordKpi(uint32_t seriesId, double value);
    /// Note a handover (UE moves from one serving node to another).
    void OnHandover(uint32_t ueNodeId, uint32_t fromId, uint32_t toId);
    /// Note a link state change between two tracked nodes.
    /// OBS-05: move a UE's serving edge from \p fromId to \p toId.
    ///
    /// Retires the outgoing edge and opens the incoming one through the same
    /// OnLinkChange transitions the CZML writer already uses for availability,
    /// declaring the new edge if the UE has not been served by that satellite
    /// before. Called automatically by OnHandover, so a scenario that already
    /// reports handovers gets a serving edge that follows them without any
    /// further wiring.
    ///
    /// The serving edge used to be declared once by TrackBeam() at setup and
    /// never touched, so the polyline pointed at the original satellite for the
    /// whole run and a handover study's own association never moved on screen.
    void SetServingEdge(uint32_t ueNodeId, uint32_t fromId, uint32_t toId);

    void OnLinkChange(uint32_t aId, uint32_t bId, bool up);

    /// Number of scene events written (positions + KPI samples + log events).
    uint64_t GetEventCount() const;

  protected:
    void DoDispose() override;

  private:
    struct TrackedNode
    {
        uint32_t id;
        Ptr<MobilityModel> mob;
        NodeKind kind;
        std::string label;
    };
    struct Beam
    {
        uint32_t fromId;
        uint32_t toId;
        /// OBS-06: index of this beam in the NetSimulyzer exporter, or 0 if it
        /// was declared after Start() and so has no exporter link. Beams reached
        /// only live stdout and the CZML sink before this.
        uint32_t exporterIdx{0};
    };
    struct KpiSeries
    {
        uint32_t seriesId;    //!< recorder-assigned id (returned to caller)
        uint32_t exporterIdx; //!< NetSimulyzer exporter series index (0 if none)
        uint32_t nodeId;
        KpiKind kind;
        std::string name;
        // accumulated (t, value) samples, emitted to CZML as a time-tagged
        // custom property on the owning node packet.
        std::vector<std::pair<double, double>> samples;
    };
    // A link-state transition (from OnLinkChange), used to build CZML polyline
    // availability intervals for the declared beams.
    struct LinkEvent
    {
        double t;
        uint32_t aId;
        uint32_t bId;
        bool up;
    };
    // Per-node accumulated position samples (sim seconds, ECEF metres).
    struct Track
    {
        uint32_t id;
        NodeKind kind;
        std::string label;
        std::vector<std::pair<double, Vector>> samples; // (t, ecef)
    };
    // A handover, rendered in CZML as a brief polyline arc flash.
    struct HandoverArc
    {
        double t;
        uint32_t ueId;
        uint32_t fromId;
        uint32_t toId;
    };
    /// ECEF position of a tracked node at (or nearest) sim time t.
    bool PositionAt(uint32_t id, double t, Vector& out) const;

    /// Convert a tracked position to ECEF metres given the configured frame.
    Vector ToEcef(const Vector& p) const;

  public:
    /// OBS-15: test seam for the LocalEnu-to-ECEF conversion.
    ///
    /// Roughly forty examples reach the scene recorder through the LocalEnu
    /// path, and the only test of the recorder set EcefGlobal, which makes
    /// ToEcef the identity for the whole test. The conversion every one of
    /// those examples depends on therefore had zero coverage, and a numerically
    /// wrong position would have passed every assertion, all of which are
    /// substring searches on the output files.
    Vector ToEcefForTest(const Vector& p) const { return ToEcef(p); }

  private:
    /// Periodic poll: read every tracked node and emit a position sample.
    void Poll();
    /// Map a NodeKind to a 3D model name (matches our custom .obj asset set).
    static std::string ModelName(NodeKind kind);
    /// Write the CZML document from accumulated tracks (on Stop).
    void WriteCzml();

    SceneFrame m_frame{EcefGlobal};
    double m_lat0{0.0};
    double m_lon0{0.0};
    double m_alt0{0.0};
    Vector m_enuOriginEcef{0.0, 0.0, 0.0};
    Vector m_enuEast{1, 0, 0};
    Vector m_enuNorth{0, 1, 0};
    Vector m_enuUp{0, 0, 1};

    std::vector<TrackedNode> m_nodes;
    std::vector<Beam> m_beams; //!< serving/beam edges; CZML polylines + live ##NTNSCENE_LINK##
    std::vector<KpiSeries> m_kpis;
    std::vector<HandoverArc> m_handovers;
    std::vector<LinkEvent> m_linkEvents; //!< OnLinkChange transitions -> CZML availability
    std::map<uint32_t, Track> m_tracks;  //!< id -> accumulated position samples

    bool m_netSimEnabled{false};
    std::string m_netSimPath;
    double m_sceneScale{1e-6};
    Ptr<NtnNetSimulyzerExporter> m_exporter;

    bool m_czmlEnabled{false};
    std::string m_czmlPath;
    bool m_liveStdout{false};

    double m_sceneEpochUnix{kDefaultSceneEpochUnix}; //!< CZML t=0
    Time m_sampleDt{Seconds(1.0)};
    bool m_running{false};
    double m_t0Sec{0.0};
    double m_tLastSec{0.0};
    uint64_t m_eventCount{0};
};

} // namespace ntnobs
} // namespace ns3

#endif // NTN_SCENE_RECORDER_H
