/*
 * Copyright (c) 2026 Muhammad Uzair
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ntn-dataset@1 KPI series for examples with their own data plane (point-to-point satellite links,
 * routed constellations, Wi-Fi or LTE demos), which do not run on NtnRealStackHelper and so have no
 * radio counters. Every period it differences the example's FlowMonitor statistics and writes
 *
 *   <outDir>/ue_kpi.csv    one row per destination node per period that delivered or lost packets
 *                          (idle intervals write no row):
 *       time_s, ue (destination node id), flows, ip_rx_thp_mbps, rx_pkts, lost_pkts, loss_ratio,
 *       owd_ms, jitter_ms, sat_node, elevation_deg, slant_range_km, rtt_geom_ms
 *   <outDir>/cell_kpi.csv  the same packets grouped by that node's highest satellite:
 *       time_s, cell_id (satellite node id, 0 = no satellite geometry), active_ues,
 *       ip_rx_thp_mbps, rx_pkts, lost_pkts, loss_ratio, owd_ms
 *
 * Throughput counts IP bytes (FlowMonitor sees IP packets), delay and jitter are FlowMonitor's
 * per-packet measurements. A packet counts as lost when an IPv4 probe reports it dropped (no
 * route, TTL expiry, queue overflow), at once, or when FlowMonitor's MaxPerHopDelay (default 10 s)
 * passes without delivery, so silent losses in the last ~10 s of a run are not yet counted. All
 * of these are measured. The geometry columns are filled only for scenes in Earth-fixed coordinates (terminal
 * on the Earth's surface, satellites above 160 km); local Cartesian scenes leave them empty.
 */
#ifndef NTN_FLOW_KPI_SERIES_H
#define NTN_FLOW_KPI_SERIES_H

#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/ipv4.h"
#include "ns3/mobility-model.h"
#include "ns3/node-list.h"
#include "ns3/node.h"
#include "ns3/simulator.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>

namespace ns3
{
namespace ntnobs
{

/// State of one KPI series (kept alive by the scheduled sampling events).
struct FlowKpiSeriesState
{
    Ptr<FlowMonitor> monitor;
    Ptr<Ipv4FlowClassifier> classifier;
    Time period;
    std::shared_ptr<std::ofstream> ue;
    std::shared_ptr<std::ofstream> cell;
    std::map<FlowId, FlowMonitor::FlowStats> prev;
    std::map<uint32_t, uint32_t> addrToNode; ///< IPv4 address (host order) -> node id
};

namespace detail
{

/// Geodetic elevation (deg) of `sat` seen from `g` (both ECEF, m), on the WGS84 normal.
inline double
ElevationDeg(const Vector& g, const Vector& sat)
{
    const double a = 6378137.0;
    const double e2 = 6.69437999014e-3;
    const double b = a * std::sqrt(1 - e2);
    const double pxy = std::sqrt(g.x * g.x + g.y * g.y);
    const double th = std::atan2(g.z * a, pxy * b);
    const double lat = std::atan2(g.z + (e2 * a * a / (b * b) - e2) * b * std::pow(std::sin(th), 3),
                                  pxy - e2 * a * std::pow(std::cos(th), 3));
    const double lon = std::atan2(g.y, g.x);
    const Vector up(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
    const Vector d(sat.x - g.x, sat.y - g.y, sat.z - g.z);
    const double r = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    return r > 0 ? std::asin((d.x * up.x + d.y * up.y + d.z * up.z) / r) * 180.0 / M_PI : 0.0;
}

inline std::string
Field(double v, int prec)
{
    std::ostringstream f;
    if (std::isfinite(v))
    {
        f << std::fixed << std::setprecision(prec) << v;
    }
    return f.str();
}

/// Highest satellite (node id, elevation deg, slant range m) above the node, or id 0 when the node
/// is not on the Earth's surface in ECEF or no satellite is above the horizon.
inline void
HighestSatellite(uint32_t nodeId, uint32_t& satId, double& elevDeg, double& rangeM)
{
    satId = 0;
    elevDeg = std::nan("");
    rangeM = std::nan("");
    Ptr<Node> n = NodeList::GetNode(nodeId);
    Ptr<MobilityModel> m = n ? n->GetObject<MobilityModel>() : nullptr;
    if (!m)
    {
        return;
    }
    const Vector g = m->GetPosition();
    const double rg = std::sqrt(g.x * g.x + g.y * g.y + g.z * g.z);
    if (rg < 6.30e6 || rg > 6.40e6 + 15e3)
    {
        return; // local frame, or not a ground terminal
    }
    for (uint32_t i = 0; i < NodeList::GetNNodes(); ++i)
    {
        Ptr<MobilityModel> sm = NodeList::GetNode(i)->GetObject<MobilityModel>();
        if (!sm || i == nodeId)
        {
            continue;
        }
        const Vector s = sm->GetPosition();
        const double rs = std::sqrt(s.x * s.x + s.y * s.y + s.z * s.z);
        if (rs < 6.371e6 + 160e3 || rs > 5.0e7)
        {
            continue;
        }
        const double el = ElevationDeg(g, s);
        if (el > 0.0 && (satId == 0 || el > elevDeg))
        {
            satId = i;
            elevDeg = el;
            rangeM = CalculateDistance(g, s);
        }
    }
}

inline void
MapAddresses(FlowKpiSeriesState& st)
{
    st.addrToNode.clear();
    for (uint32_t i = 0; i < NodeList::GetNNodes(); ++i)
    {
        Ptr<Ipv4> ip = NodeList::GetNode(i)->GetObject<Ipv4>();
        if (!ip)
        {
            continue;
        }
        for (uint32_t k = 0; k < ip->GetNInterfaces(); ++k)
        {
            for (uint32_t j = 0; j < ip->GetNAddresses(k); ++j)
            {
                st.addrToNode[ip->GetAddress(k, j).GetLocal().Get()] = i;
            }
        }
    }
}

inline void
Sample(std::shared_ptr<FlowKpiSeriesState> st)
{
    constexpr double kC = 299792458.0;
    const double dt = st->period.GetSeconds();
    st->monitor->CheckForLostPackets();
    const auto stats = st->monitor->GetFlowStats();
    if (st->addrToNode.empty())
    {
        MapAddresses(*st);
    }
    struct Agg
    {
        uint32_t flows{0};
        uint64_t bytes{0};
        uint64_t rx{0};
        uint64_t lost{0};
        double delayS{0.0};
        double jitterS{0.0};
        uint64_t jitterN{0};
    };
    std::map<uint32_t, Agg> perNode;
    for (const auto& [id, s] : stats)
    {
        const Ipv4FlowClassifier::FiveTuple t = st->classifier->FindFlow(id);
        auto it = st->addrToNode.find(t.destinationAddress.Get());
        if (it == st->addrToNode.end())
        {
            MapAddresses(*st); // addresses assigned after the first sample
            it = st->addrToNode.find(t.destinationAddress.Get());
            if (it == st->addrToNode.end())
            {
                continue;
            }
        }
        const FlowMonitor::FlowStats p = st->prev.count(id) ? st->prev[id] : FlowMonitor::FlowStats{};
        Agg& a = perNode[it->second];
        a.flows += 1;
        a.bytes += s.rxBytes - p.rxBytes;
        a.rx += s.rxPackets - p.rxPackets;
        a.lost += s.lostPackets - p.lostPackets;
        a.delayS += (s.delaySum - p.delaySum).GetSeconds();
        const uint64_t jn = (s.rxPackets > 1 ? s.rxPackets - 1 : 0) - (p.rxPackets > 1 ? p.rxPackets - 1 : 0);
        a.jitterS += (s.jitterSum - p.jitterSum).GetSeconds();
        a.jitterN += jn;
        st->prev[id] = s;
    }
    std::map<uint32_t, Agg> perSat;
    std::map<uint32_t, uint32_t> perSatUes;
    const double now = Simulator::Now().GetSeconds();
    for (const auto& [node, a] : perNode)
    {
        if (a.rx == 0 && a.lost == 0)
        {
            continue; // idle interval: no row (store-and-forward runs span days)
        }
        uint32_t sat = 0;
        double el = 0;
        double rangeM = 0;
        HighestSatellite(node, sat, el, rangeM);
        const double nan = std::nan("");
        const double lossRatio = (a.rx + a.lost) ? double(a.lost) / (a.rx + a.lost) : nan;
        *st->ue << Field(now, 3) << "," << node << "," << a.flows << ","
                << Field(8.0 * a.bytes / dt / 1e6, 4) << "," << a.rx << "," << a.lost << ","
                << Field(lossRatio, 5) << "," << Field(a.rx ? 1e3 * a.delayS / a.rx : nan, 3) << ","
                << Field(a.jitterN ? 1e3 * a.jitterS / a.jitterN : nan, 3) << ","
                << (sat ? std::to_string(sat) : std::string()) << "," << Field(el, 3) << ","
                << Field(rangeM / 1e3, 3) << "," << Field(2.0 * rangeM / kC * 1e3, 4) << "\n";
        Agg& c = perSat[sat];
        c.bytes += a.bytes;
        c.rx += a.rx;
        c.lost += a.lost;
        c.delayS += a.delayS;
        perSatUes[sat] += (a.rx > 0) ? 1 : 0;
    }
    for (const auto& [sat, c] : perSat)
    {
        const double nan = std::nan("");
        *st->cell << Field(now, 3) << "," << sat << "," << perSatUes[sat] << ","
                  << Field(8.0 * c.bytes / dt / 1e6, 4) << "," << c.rx << "," << c.lost << ","
                  << Field((c.rx + c.lost) ? double(c.lost) / (c.rx + c.lost) : nan, 5) << ","
                  << Field(c.rx ? 1e3 * c.delayS / c.rx : nan, 3) << "\n";
    }
    st->ue->flush();
    st->cell->flush();
    Simulator::Schedule(st->period, &Sample, st);
}

} // namespace detail

/**
 * Start the per-terminal and per-satellite KPI series of a custom data plane. `flowmon` must
 * already be installed on the nodes (InstallAll or Install); call before Simulator::Run(). The
 * helper object must outlive the run, as for any FlowMonitor use.
 */
inline void
StartFlowKpiSeries(const std::string& outDirIn, FlowMonitorHelper& flowmon, Time period = Seconds(1))
{
    auto st = std::make_shared<FlowKpiSeriesState>();
    st->monitor = flowmon.GetMonitor();
    st->classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    st->period = period;
    if (!st->monitor || !st->classifier || period <= Seconds(0))
    {
        return;
    }
    const std::string outDir = outDirIn.empty() ? std::string(".") : outDirIn;
    std::error_code ec;
    std::filesystem::create_directories(outDir, ec);
    st->ue = std::make_shared<std::ofstream>(outDir + "/ue_kpi.csv");
    st->cell = std::make_shared<std::ofstream>(outDir + "/cell_kpi.csv");
    *st->ue << "time_s,ue,flows,ip_rx_thp_mbps,rx_pkts,lost_pkts,loss_ratio,owd_ms,jitter_ms,"
               "sat_node,elevation_deg,slant_range_km,rtt_geom_ms\n";
    *st->cell << "time_s,cell_id,active_ues,ip_rx_thp_mbps,rx_pkts,lost_pkts,loss_ratio,owd_ms\n";
    Simulator::Schedule(period, &detail::Sample, st);
}

} // namespace ntnobs
} // namespace ns3

#endif // NTN_FLOW_KPI_SERIES_H
