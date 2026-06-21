/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * NtnSceneHelper — one-call wiring of an NtnSceneRecorder from a scenario's
 * node containers, so any example gains a NetSimulyzer + Cesium 3D trace in a
 * few lines:
 *
 *   NtnSceneHelper sh;
 *   sh.SetNetSimulyzer(netSimPath);   // either/both optional; empty = off
 *   sh.SetCzml(czmlPath);
 *   sh.SetEnuFrame(lat0, lon0, 0.0);  // omit for the ECEF-global frame
 *   Ptr<ntnobs::NtnSceneRecorder> scene = sh.Build(satNodes, ueNodes, gwNodes);
 *   ... Simulator::Run() ...
 *   if (scene) scene->Stop();
 *
 * Build() tracks every node (satellites, UEs, gateways), enables the requested
 * sinks, and Starts the recorder. It returns nullptr when no sink path was set,
 * so the gated `if (scene)` teardown is always safe. Examples that also want
 * measured-KPI series or handover events keep the returned recorder and call
 * TrackKpiSeries()/RecordKpi()/OnHandover() directly (those must precede the
 * Start() that Build does, so add series via the recorder BEFORE Build, or use
 * the recorder API without this helper).
 */

#ifndef NTN_SCENE_HELPER_H
#define NTN_SCENE_HELPER_H

#include "ns3/ntn-scene-recorder.h"

#include <ns3/node-container.h>
#include <ns3/nstime.h>
#include <ns3/ptr.h>

#include <string>

namespace ns3
{
namespace ntnobs
{

/**
 * \ingroup ntn-observability
 * \brief Wires an NtnSceneRecorder from a scenario's node containers.
 */
class NtnSceneHelper
{
  public:
    NtnSceneHelper() = default;

    /// NetSimulyzer JSON output path (empty disables this sink).
    void SetNetSimulyzer(const std::string& path) { m_netSim = path; }
    /// Cesium CZML output path (empty disables this sink).
    void SetCzml(const std::string& path) { m_czml = path; }
    /// Use a local-ENU frame about (lat0,lon0,alt0). Default is ECEF-global.
    void SetEnuFrame(double lat0Deg, double lon0Deg, double alt0M);
    /// Position sampling interval (default 1 s).
    void SetSampleInterval(Time dt) { m_dt = dt; }
    /// Uniform ECEF->scene scale for NetSimulyzer (default 1e-6).
    void SetSceneScale(double s) { m_sceneScale = s; }
    /// Stream live scene frames to stdout (for mid-run globe streaming).
    void SetLiveStdout(bool on) { m_live = on; }

    /// True if at least one sink path is set.
    bool Enabled() const { return !m_netSim.empty() || !m_czml.empty(); }

    /**
     * \brief Track every node in the containers, enable the configured sinks and
     *        Start the recorder.
     * \return the started recorder, or nullptr if no sink path was configured.
     */
    Ptr<NtnSceneRecorder> Build(NodeContainer sats,
                                NodeContainer ues,
                                NodeContainer gateways = NodeContainer());

  private:
    std::string m_netSim;
    std::string m_czml;
    bool m_enu{false};
    double m_lat0{0.0};
    double m_lon0{0.0};
    double m_alt0{0.0};
    Time m_dt{Seconds(1.0)};
    double m_sceneScale{1e-6};
    bool m_live{false};
};

} // namespace ntnobs
} // namespace ns3

#endif // NTN_SCENE_HELPER_H
