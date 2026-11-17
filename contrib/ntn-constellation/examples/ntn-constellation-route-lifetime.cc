/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ntn-constellation-route-lifetime: ground-to-ground routing over a Walker
 * shell with a +Grid inter-satellite topology, under five route recomputation
 * policies, on a real IPv4 data plane.
 *
 * Every +Grid inter-satellite link and every ground link that can come into
 * view is a point-to-point IPv4 link whose propagation delay follows the live
 * range and whose receive error model drops everything while the contact is
 * down. A UDP flow runs from ground station A to B; host routes along the
 * selected path are installed in Ipv4StaticRouting whenever the policy
 * recomputes. Packets sent on a path whose link has dropped are lost, so the
 * cost of a stale route is measured, not assumed.
 *
 * Policies
 *   snap      shortest-delay path recomputed every --period seconds (snapshot
 *             routing, as in Hypatia with a 100 ms period)
 *   ondemand  shortest-delay path kept until one of its links drops, then
 *             recomputed after --detect + --setup seconds (on-demand routing)
 *   asr       every --window seconds, shortest path over the links that stay
 *             up for the whole window (merged snapshots, ASR-LSA style)
 *   life      longest-lived path within (1 + --eps) of the shortest delay,
 *             switched --guard seconds before its predicted expiry
 * Added for the route-break study (Nov 2026):
 *   asrk      merged snapshot that keeps a route that still lasts the window
 *   track     Algorithm 1 on residual times estimated from a 30 s elevation track
 *   alpr      average-latency persistent routing of Bhattacharjee et al. (eq. 13)
 *   isasr     stability and activeness slotted routing of Bhattacharjee et al. (eq. 14-15)
 *
 * Usage:
 *   ./ns3 run "ntn-constellation-route-lifetime --policy=life --eps=0.05"
 */

#include "ns3/applications-module.h"
#include "ns3/contact-graph-router.h"
#include "ns3/contact-graph-scheduler.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <sys/resource.h>
#include <time.h>

#include <algorithm>
#include <fstream>
#include <cmath>
#include <iostream>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <vector>

using namespace ns3;
using ns3::ntncon::ContactEvent;
using ns3::ntncon::ContactGraphRouter;
using ns3::ntncon::ContactGraphScheduler;
using ns3::ntncon::Sgp4MobilityModel;
using ns3::ntncon::WalkerConfig;
using ns3::ntncon::WalkerConstellation;

NS_LOG_COMPONENT_DEFINE("NtnConstellationRouteLifetime");

namespace
{
constexpr double kC = 299792458.0;

struct Link
{
    Ptr<PointToPointChannel> ch;
    Ptr<RateErrorModel> emA;
    Ptr<RateErrorModel> emB;
    uint32_t ifA;  // interface index at the low node
    uint32_t ifB;  // interface index at the high node
    Ipv4Address ipA;
    Ipv4Address ipB;
};

using Edge = std::pair<uint32_t, uint32_t>;

Edge
Canon(uint32_t a, uint32_t b)
{
    return a < b ? Edge{a, b} : Edge{b, a};
}

struct State
{
    uint32_t n{0};
    uint32_t gsA{0};
    uint32_t gsB{0};
    NodeContainer nodes;
    std::map<Edge, Link> links;
    Ptr<ContactGraphScheduler> sched;
    Ptr<ContactGraphScheduler> predSched; // predictor; differs from sched only with --predSigmaKm
    Ptr<ContactGraphRouter> router;
    Ipv4Address dst;
    std::vector<uint32_t> path;
    std::set<Edge> pathEdges;
    std::map<Edge, double> endCache; // absolute predicted contact end (s)
    std::map<Edge, double> satCache; // link with no end within the horizon: time of that prediction
    double satRecheck{900.0};
    std::string policy;
    double period{0.1}; // snap: Hypatia 100 ms forwarding-state step (Wave 6B; was 1 s)
    double detect{0.05};
    double trigger{25.0};                         // elev policy: switch when an end satellite sinks below this (deg)
    std::vector<Ptr<Sgp4MobilityModel>> satMob;   // satellite mobilities, by node index
    double gsLat[2]{0, 0}, gsLon[2]{0, 0};        // ground stations A and B
    double setup{0.1};
    double window{30.0};
    double eps{0.05};
    double guard{1.0};
    double horizon{1800.0};
    double tAcq{0.0};   // acquisition time of a new ground link, the same for every policy
    bool hold{false};   // life: also switch when the held route's stretch exceeds eps
    EventId pending;
    EventId pendingInstall;
    std::vector<uint32_t> pendingPath;
    uint64_t forced{0};
    std::ofstream reach;
    uint64_t changes{0};
    uint64_t pathBreaks{0};
    uint64_t computations{0};
    uint64_t dijkstra{0};
    uint64_t predictions{0};
    double brokenS{0.0};
    double stretchSum{0.0};
    uint64_t stretchN{0};
    std::vector<double> stretch;
    std::vector<double> delays;
    uint64_t rx{0};
    // ---- route-break study additions
    bool noData{false};
    bool bbm{false};                   // one terminal per gateway: the old ground link stops when acquisition starts
    double fallbackEps{0.10};          // asr, asrk: stretch bound of the fallback route
    double keepEps{1e9};               // asrk: keep a route only while its stretch is within this
    double trackArc{30.0};             // track: arc of the elevation track (s)
    double gammaIsasr{1.0};            // isasr: gamma of eq. 14 (dimensionless)
    bool proactive{true};              // alpr, isasr: start the switch g + tAcq before the known end
    bool logEvents{false};
    uint64_t ingressChanges{0}, egressChanges{0}, islOnlyChanges{0}, acquisitions{0};
    uint64_t breaksIn{0}, breaksOut{0}, breaksIsl{0};
    uint64_t holdSwitches{0};
    double ageSum{0.0}, lastInstall{-1.0};
    uint64_t ageN{0};
    double resInSum{0.0}, resOutSum{0.0};
    uint64_t resN{0};
    double searchCpu{0.0};
    uint64_t searchN{0};
    std::set<Edge> blocked;
    std::map<Edge, double> trackCache;
    uint64_t trackEvals{0};
    std::set<Edge> isasrZeroAct;
    bool isasrIslAct{false};
    double elevNoiseDeg{0.0};
    uint32_t trackSamples{3};
    double floorGrid{5.0};
    Ptr<NormalRandomVariable> noise;
    std::map<std::pair<uint32_t, uint32_t>, double> elevReading; // one reading per link per decision
    uint64_t trackBad{0};
    uint64_t fallbackUsed{0};
    uint64_t mergedDecisions{0};
    uint64_t noAdmissible{0};
    double routeLenSum{0.0};
    uint64_t routeLenN{0};
    double trackErrAbsSum{0.0};
    uint64_t trackErrN{0};
    double dropDeg{23.0};
} g;

double
ThreadCpu()
{
    timespec ts;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return ts.tv_sec + 1e-9 * ts.tv_nsec;
}

void Log(const std::string& what)
{
    if (g.logEvents)
    {
        std::cerr << Simulator::Now().GetSeconds() << " " << what << "\n";
    }
}

std::string PathStr(const std::vector<uint32_t>& p)
{
    std::ostringstream o;
    for (auto v : p) o << v << " ";
    return o.str();
}

double
Residual(uint32_t a, uint32_t b)
{
    const Edge e = Canon(a, b);
    const double now = Simulator::Now().GetSeconds();
    auto sat = g.satCache.find(e);
    if (sat != g.satCache.end() && now - sat->second < g.satRecheck)
    {
        return g.horizon; // no end within the horizon at the last look; it does not age
    }
    auto it = g.endCache.find(e);
    if (it != g.endCache.end() && it->second > now)
    {
        return it->second - now;
    }
    const bool isl = e.second < g.n;
    ++g.predictions;
    const double r =
        g.predSched->PredictContactEndS(e.first, e.second, isl, g.horizon, isl ? 30.0 : 5.0);
    if (r >= g.horizon)
    {
        g.satCache[e] = now;
        g.endCache.erase(e);
    }
    else
    {
        g.satCache.erase(e);
        g.endCache[e] = now + r;
    }
    return r;
}

void
RemoveHostRoutes(uint32_t v)
{
    Ipv4StaticRoutingHelper srh;
    auto sr = srh.GetStaticRouting(g.nodes.Get(v)->GetObject<Ipv4>());
    for (uint32_t k = sr->GetNRoutes(); k-- > 0;)
    {
        if (sr->GetRoute(k).GetDest() == g.dst)
        {
            sr->RemoveRoute(k);
        }
    }
}

void Unblock();

void
CountChange(const std::vector<uint32_t>& p)
{
    const double now = Simulator::Now().GetSeconds();
    if (!g.path.empty() && p != g.path)
    {
        const bool in = p[1] != g.path[1];
        const bool out = p[p.size() - 2] != g.path[g.path.size() - 2];
        g.ingressChanges += in;
        g.egressChanges += out;
        g.islOnlyChanges += (!in && !out);
        g.acquisitions += (in || out);
        if (g.lastInstall >= 0)
        {
            g.ageSum += now - g.lastInstall;
            ++g.ageN;
        }
    }
    g.lastInstall = now;
    if (p.size() >= 3)
    {
        const double h = g.horizon;
        g.resInSum += std::min(h, g.predSched->PredictContactEndS(p[1], g.gsA, false, h, 5.0));
        g.resOutSum += std::min(h, g.predSched->PredictContactEndS(p[p.size() - 2], g.gsB, false, h, 5.0));
        ++g.resN;
    }
}

void
InstallPath(const std::vector<uint32_t>& p)
{
    Log("install " + PathStr(p));
    CountChange(p);
    if (g.noData)
    {
        if (!g.path.empty() && p != g.path)
        {
            ++g.changes;
        }
        g.path = p;
        g.pathEdges.clear();
        for (size_t k = 0; k + 1 < p.size(); ++k)
        {
            g.pathEdges.insert(Canon(p[k], p[k + 1]));
        }
        return;
    }
    Ipv4StaticRoutingHelper srh;
    // Nodes that stay on the path get their next hop replaced now. Nodes that
    // leave it keep their old entry for a drain time longer than any path
    // delay, so packets already in flight on the old path still arrive (make
    // before break); a node that rejoins a later path in the meantime is
    // simply overwritten.
    const std::set<uint32_t> next(p.begin(), p.end());
    for (uint32_t v : g.path)
    {
        if (next.count(v))
        {
            RemoveHostRoutes(v);
        }
        else
        {
            Simulator::Schedule(MilliSeconds(200), [v]() {
                if (std::find(g.path.begin(), g.path.end(), v) == g.path.end())
                {
                    RemoveHostRoutes(v);
                }
            });
        }
    }
    for (size_t k = 0; k + 1 < p.size(); ++k)
    {
        const uint32_t a = p[k];
        const uint32_t b = p[k + 1];
        const Link& l = g.links.at(Canon(a, b));
        const bool aLow = a < b;
        RemoveHostRoutes(a);
        auto sr = srh.GetStaticRouting(g.nodes.Get(a)->GetObject<Ipv4>());
        sr->AddHostRouteTo(g.dst, aLow ? l.ipB : l.ipA, aLow ? l.ifA : l.ifB);
        const double w = g.router->EdgeWeight(a, b);
        if (!std::isnan(w))
        {
            l.ch->SetAttribute("Delay", TimeValue(Seconds(w / kC)));
        }
    }
    if (!g.path.empty() && p != g.path)
    {
        ++g.changes;
    }
    g.path = p;
    g.pathEdges.clear();
    for (size_t k = 0; k + 1 < p.size(); ++k)
    {
        g.pathEdges.insert(Canon(p[k], p[k + 1]));
    }
    Unblock();
}

bool
PathUp(const std::vector<uint32_t>& p)
{
    if (p.empty())
    {
        return false;
    }
    for (size_t k = 0; k + 1 < p.size(); ++k)
    {
        if (!g.router->HasEdge(p[k], p[k + 1]))
        {
            return false;
        }
    }
    return true;
}

void Schedule(double dt);

/// Move the flow to p. A ground link that p uses and the current route does not must first be
/// acquired, which takes tAcq for every policy: if the current route still works, traffic stays on it
/// meanwhile (make before break); if it has already broken, traffic is lost until the new link is up.
void
Switch(const std::vector<uint32_t>& p)
{
    if (p == g.path)
    {
        return;
    }
    if (g.pendingInstall.IsPending() && PathUp(g.pendingPath))
    {
        return; // an acquisition is under way and its route is still good
    }
    const bool oldUp = PathUp(g.path);
    bool newGround = false;
    for (size_t k = 0; k + 1 < p.size(); ++k)
    {
        const Edge e = Canon(p[k], p[k + 1]);
        newGround |= (e.second >= g.n && !g.pathEdges.count(e));
    }
    if (!g.path.empty() && !oldUp)
    {
        ++g.forced;
    }
    g.pendingInstall.Cancel();
    if (g.tAcq > 0 && newGround && !g.path.empty())
    {
        if (g.bbm && !g.noData)
        {
            // One terminal per gateway: the terminal that must slew stops serving its old link now.
            const Edge oldIn = Canon(g.path[1], g.gsA);
            const Edge oldOut = Canon(g.path[g.path.size() - 2], g.gsB);
            for (const Edge& oe : {oldIn, oldOut})
            {
                const Edge ne = oe.second == g.gsA ? Canon(p[1], g.gsA) : Canon(p[p.size() - 2], g.gsB);
                if (oe != ne && g.links.count(oe))
                {
                    g.links.at(oe).emA->SetRate(1.0);
                    g.links.at(oe).emB->SetRate(1.0);
                    g.blocked.insert(oe);
                }
            }
        }
        g.pendingPath = p;
        g.pendingInstall = Simulator::Schedule(Seconds(g.tAcq), [p]() {
            if (PathUp(p))
            {
                InstallPath(p);
            }
            else if (g.policy != "snap" && g.policy != "asr")
            {
                Schedule(0.0); // the route broke while being acquired: compute again (clocked policies retry anyway)
            }
        });
        return;
    }
    InstallPath(p);
}

void Recompute();

/// Elevation (deg) of satellite s seen from ground station gs (WGS84, geodetic up), offset s from now.
double
ElevDeg(uint32_t s, uint32_t gs, double offset = 0.0)
{
    const int k = gs == g.gsA ? 0 : 1;
    const double la = g.gsLat[k] * M_PI / 180, lo = g.gsLon[k] * M_PI / 180;
    const double a = 6378137.0, e2 = 6.69437999014e-3;
    const double nn = a / std::sqrt(1 - e2 * std::sin(la) * std::sin(la));
    const Vector gp(nn * std::cos(la) * std::cos(lo), nn * std::cos(la) * std::sin(lo), nn * (1 - e2) * std::sin(la));
    Vector sp, sv;
    if (offset == 0.0)
    {
        sp = g.satMob[s]->GetEcefPosition();
    }
    else
    {
        g.satMob[s]->GetEcefStateAt(Seconds(offset), sp, sv);
    }
    const Vector d(sp.x - gp.x, sp.y - gp.y, sp.z - gp.z);
    const double r = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    const double up = d.x * std::cos(la) * std::cos(lo) + d.y * std::cos(la) * std::sin(lo) + d.z * std::sin(la);
    return std::asin(up / r) * 180 / M_PI;
}

/// Elevation as a gateway would measure it now.
double
MeasElev(uint32_t s, uint32_t gs, double offset = 0.0)
{
    const double t = ElevDeg(s, gs, offset);
    return g.elevNoiseDeg > 0 ? t + g.noise->GetValue(0.0, g.elevNoiseDeg * g.elevNoiseDeg) : t;
}

/// The reading of one link within one decision.
double
ReadElev(uint32_t s, uint32_t gs)
{
    if (g.elevNoiseDeg <= 0)
    {
        return ElevDeg(s, gs);
    }
    auto key = std::make_pair(s, gs);
    auto it = g.elevReading.find(key);
    if (it != g.elevReading.end())
    {
        return it->second;
    }
    const double v = MeasElev(s, gs);
    g.elevReading[key] = v;
    return v;
}

/// Central angle (rad) from gateway to sub-satellite point at elevation e (deg), for shell radius r.
double
CentralAngle(double eDeg, double r)
{
    const double e = eDeg * M_PI / 180;
    return std::acos(6371000.0 * std::cos(e) / r) - e;
}

/// Residual time of a ground link estimated from three elevation samples over the last trackArc seconds, with no
/// orbit: psi^2 of a pass is close to a parabola in time (the satellite crosses the visibility cap along a chord at
/// constant speed), so a parabola through psi^2 at the three samples gives the time at which psi reaches the drop
/// elevation. Inter-satellite links are taken not to end (the tracker cannot see them).
double
TrackResidual(uint32_t a, uint32_t b)
{
    const Edge e = Canon(a, b);
    if (e.second < g.n)
    {
        return g.horizon;
    }
    auto it = g.trackCache.find(e);
    if (it != g.trackCache.end())
    {
        return it->second;
    }
    ++g.trackEvals;
    const uint32_t s = e.first, gs = e.second;
    // Least-squares parabola y(tau) = c0 + c1 tau + c2 tau^2 through trackSamples readings of psi^2 over the arc;
    // with three samples it passes through them.
    Vector sp = g.satMob[s]->GetEcefPosition();
    const double r = std::sqrt(sp.x * sp.x + sp.y * sp.y + sp.z * sp.z);
    const uint32_t N = std::max<uint32_t>(3, g.trackSamples);
    double S[5] = {0, 0, 0, 0, 0}, Y[3] = {0, 0, 0};
    for (uint32_t k = 0; k < N; ++k)
    {
        const double tau = -g.trackArc * k / (N - 1);
        const double p = CentralAngle(MeasElev(s, gs, tau), r);
        const double y = p * p;
        double tk = 1.0;
        for (int q = 0; q < 5; ++q)
        {
            S[q] += tk;
            if (q < 3)
            {
                Y[q] += tk * y;
            }
            tk *= tau;
        }
    }
    // normal equations, Cramer
    const double D = S[0] * (S[2] * S[4] - S[3] * S[3]) - S[1] * (S[1] * S[4] - S[3] * S[2]) + S[2] * (S[1] * S[3] - S[2] * S[2]);
    const double c0 = (Y[0] * (S[2] * S[4] - S[3] * S[3]) - S[1] * (Y[1] * S[4] - S[3] * Y[2]) + S[2] * (Y[1] * S[3] - S[2] * Y[2])) / D;
    const double B = (S[0] * (Y[1] * S[4] - S[3] * Y[2]) - Y[0] * (S[1] * S[4] - S[3] * S[2]) + S[2] * (S[1] * Y[2] - Y[1] * S[2])) / D;
    const double C = (S[0] * (S[2] * Y[2] - Y[1] * S[3]) - S[1] * (S[1] * Y[2] - Y[1] * S[2]) + Y[0] * (S[1] * S[3] - S[2] * S[2])) / D;
    const double pd = CentralAngle(g.dropDeg, r);
    const double A = c0 - pd * pd; // A + B t + C t^2 = 0 at the drop
    double res = 0.0;
    if (A < 0 && C > 0)
    {
        res = (-B + std::sqrt(std::max(0.0, B * B - 4 * A * C))) / (2 * C);
    }
    else if (A < 0)
    {
        ++g.trackBad; // the arc does not fix a pass: a falling reading extrapolates linearly, a rising one is taken as lasting
        res = B > 0 ? -A / B : g.horizon;
    }
    res = std::min(res, g.horizon);
    g.trackCache[e] = res;
    return res;
}

/// One-way delay (m of path length) of route p at the given offset from now, from the ephemeris.
double
RouteLengthAt(const std::vector<uint32_t>& p, double offset)
{
    auto posOf = [&](uint32_t v) {
        Vector x, u;
        if (v < g.n)
        {
            if (offset == 0.0)
            {
                x = g.satMob[v]->GetEcefPosition();
            }
            else
            {
                g.satMob[v]->GetEcefStateAt(Seconds(offset), x, u);
            }
            return x;
        }
        const int k = v == g.gsA ? 0 : 1;
        const double la = g.gsLat[k] * M_PI / 180, lo = g.gsLon[k] * M_PI / 180;
        const double a = 6378137.0, e2 = 6.69437999014e-3;
        const double nn = a / std::sqrt(1 - e2 * std::sin(la) * std::sin(la));
        return Vector(nn * std::cos(la) * std::cos(lo), nn * std::cos(la) * std::sin(lo), nn * (1 - e2) * std::sin(la));
    };
    double L = 0;
    Vector prev = posOf(p[0]);
    for (size_t k = 1; k < p.size(); ++k)
    {
        const Vector c = posOf(p[k]);
        const double dx = c.x - prev.x, dy = c.y - prev.y, dz = c.z - prev.z;
        L += std::sqrt(dx * dx + dy * dy + dz * dz);
        prev = c;
    }
    return L;
}

/// Elevation-trigger policy (no ephemeris): every dt, if the satellite at either end of the route has sunk
/// below the trigger, move to the shortest route over ground links above the trigger; the old link stays up
/// until it sinks below the drop elevation, so the switch is make before break when the trigger is above it.
void
ElevCheck(double dt)
{
    if (g.path.size() >= 3)
    {
        const uint32_t sA = g.path[1], sB = g.path[g.path.size() - 2];
        if (MeasElev(sA, g.gsA) < g.trigger || MeasElev(sB, g.gsB) < g.trigger)
        {
            Recompute();
        }
    }
    Simulator::Schedule(Seconds(dt), &ElevCheck, dt);
}

void
Schedule(double dt)
{
    g.pending.Cancel();
    g.pending = Simulator::Schedule(Seconds(std::max(dt, 1e-3)), &Recompute);
}

void
Recompute()
{
    ++g.computations;
    g.elevReading.clear();
    std::vector<uint32_t> p;
    auto lifeOf = [](const std::vector<uint32_t>& q) {
        double l = g.horizon;
        for (size_t k = 0; k + 1 < q.size(); ++k)
        {
            l = std::min(l, Residual(q[k], q[k + 1]));
        }
        return l;
    };
    if (g.policy == "track")
    {
        // Algorithm 1 fed by the elevation track, switched by the elevation trigger (ElevCheck)
        g.trackCache.clear();
        const double t0 = ThreadCpu();
        auto lp = g.router->LifetimeAwarePath(g.gsA, g.gsB, g.eps, &TrackResidual, g.guard + 2 * g.tAcq);
        g.searchCpu += ThreadCpu() - t0;
        ++g.searchN;
        g.dijkstra += lp.dijkstra_runs;
        p = lp.path;
    }
    else if (g.policy == "alpr")
    {
        // Bhattacharjee et al., Algorithm 2 and eq. (13) with eta_s = tAcq and 1 s slots: edge-disjoint shortest routes,
        // each scored by (eta_s + sum of delays over the slots it exists) / slots; the lowest is kept until it ends.
        std::set<Edge> removed;
        std::vector<std::vector<uint32_t>> cand;
        const size_t nr = std::min(g.router->Neighbours(g.gsA).size(), g.router->Neighbours(g.gsB).size());
        for (size_t k = 0; k < nr; ++k)
        {
            auto wp = g.router->ShortestPathWeightedFiltered(g.gsA, g.gsB, [&](uint32_t a, uint32_t b) {
                return !removed.count(Canon(a, b));
            });
            ++g.dijkstra;
            if (wp.path.empty())
            {
                break;
            }
            for (size_t q = 0; q + 1 < wp.path.size(); ++q)
            {
                removed.insert(Canon(wp.path[q], wp.path[q + 1]));
            }
            cand.push_back(wp.path);
        }
        double best = std::numeric_limits<double>::infinity();
        double bestL = 0;
        for (const auto& c : cand)
        {
            const double l = lifeOf(c);
            if (l <= g.guard + 2 * g.tAcq && cand.size() > 1 && c != cand.back())
            {
                continue; // cannot be switched to in time
            }
            double sum = 0;
            for (int q = 0; q < 5; ++q)
            {
                sum += RouteLengthAt(c, l * (q + 0.5) / 5) / kC;
            }
            const double score = (g.tAcq + (sum / 5) * std::max(1.0, std::floor(l))) / std::max(1.0, std::floor(l));
            if (score < best)
            {
                best = score;
                p = c;
                bestL = l;
            }
        }
        if (!p.empty() && g.proactive)
        {
            const double wait = bestL - g.guard - g.tAcq;
            Schedule(wait > 1e-3 ? wait : 0.5);
        }
    }
    else if (g.policy == "isasr")
    {
        // Bhattacharjee et al., Algorithm 3 and eq. (14)-(15) with eta_s = tAcq, 1 s slots, gamma = gammaIsasr, on the
        // +Grid topology: an edge of the active route has no activeness cost; an inter-satellite link that outlives the
        // horizon has no stability cost and, being permanent, no activeness cost either. Until the active route's break
        // point its edges keep cost_act = 0; at the break point (here g + tAcq before it) all edges carry eta_s.
        const double etaS = std::max(g.tAcq, 1e-3);
        const double lActive = PathUp(g.path) ? lifeOf(g.path) : 0.0;
        const bool atBreak = !g.path.empty() && lActive <= g.guard + g.tAcq + 1.0; // one slot of margin
        if (atBreak)
        {
            for (const Edge& pe : g.pathEdges)
            {
                g.isasrZeroAct.erase(pe);
            }
        }
        const std::set<Edge>& active = g.isasrZeroAct;
        auto wp = g.router->ShortestPathCost(g.gsA, g.gsB, [&](uint32_t a, uint32_t b, double w) {
            const Edge e = Canon(a, b);
            const double res = Residual(a, b);
            if (e.second < g.n && res >= g.horizon)
            {
                // permanent +Grid link: l = N in eq. (15), so no stability cost. With isasrIslAct its activeness cost follows
                // Algorithm 3 (eta_s until it serves an active route); without it, as in our variant, it costs nothing.
                return g.isasrIslAct && !active.count(e) ? w + g.gammaIsasr * kC * etaS : w;
            }
            if (res <= g.guard + 2 * g.tAcq && !active.count(e))
            {
                return std::numeric_limits<double>::infinity();
            }
            const double costSt = etaS / std::max(1.0, std::floor(res));
            const double costAct = active.count(e) ? 0.0 : etaS;
            return w + g.gammaIsasr * kC * (costSt + costAct); // weights are in meters of path; costs converted by c
        });
        ++g.dijkstra;
        p = wp.path;
        if (!p.empty() && lifeOf(p) > g.guard + g.tAcq + 1.0)
        {
            for (size_t q = 0; q + 1 < p.size(); ++q)
            {
                g.isasrZeroAct.insert(Canon(p[q], p[q + 1]));
            }
        }
        Schedule(1.0);
    }
    else if (g.policy == "life")
    {
        const double t0 = ThreadCpu();
        auto lp = g.router->LifetimeAwarePath(g.gsA, g.gsB, g.eps, &Residual, g.guard + 2 * g.tAcq);
        g.searchCpu += ThreadCpu() - t0;
        ++g.searchN;
        g.dijkstra += lp.dijkstra_runs;
        if (lp.lifetime_s <= g.guard + 2 * g.tAcq)
        {
            ++g.noAdmissible;
        }
        p = lp.path;
        if (!p.empty())
        {
            // when no route outlives the guard the search returns the dying shortest route; look again in half a
            // second rather than polling
            const double wait = lp.lifetime_s - g.guard - g.tAcq;
            Schedule(wait > 1e-3 ? wait : 0.5);
        }
    }
    else if (g.policy == "mrv")
    {
        // Longest remaining visibility at both ends, the classical handover rule, then the shortest
        // route between the two satellites it picks.
        auto pick = [](uint32_t gs) {
            uint32_t best = UINT32_MAX;
            double bestR = -1;
            for (uint32_t s : g.router->Neighbours(gs))
            {
                const double r = Residual(s, gs);
                if (s < g.n && r > bestR)
                {
                    bestR = r;
                    best = s;
                }
            }
            return std::make_pair(best, bestR);
        };
        const auto in = pick(g.gsA);
        const auto out = pick(g.gsB);
        if (in.first != UINT32_MAX && out.first != UINT32_MAX)
        {
            auto wp = g.router->ShortestPathWeightedFiltered(g.gsA, g.gsB, [&](uint32_t a, uint32_t b) {
                if (a == g.gsA || b == g.gsA)
                {
                    return std::min(a, b) == in.first;
                }
                if (a == g.gsB || b == g.gsB)
                {
                    return std::min(a, b) == out.first;
                }
                return true;
            });
            ++g.dijkstra;
            p = wp.path;
            if (!p.empty())
            {
                Schedule(std::min(in.second, out.second) - g.guard - g.tAcq);
            }
        }
    }
    else if (g.policy == "elevb")
    {
        // measurement only, with the delay budget: among routes within (1+eps) of the shortest delay over links
        // above the trigger, the one whose end satellites are highest (largest common elevation floor)
        auto base = g.router->ShortestPathWeightedFiltered(g.gsA, g.gsB, [](uint32_t a, uint32_t b) {
            if (a >= g.n && b >= g.n) return false;
            if (a < g.n && b < g.n) return true;
            return ReadElev(std::min(a, b), std::max(a, b)) >= g.trigger;
        });
        ++g.dijkstra;
        p = base.path;
        if (!base.path.empty())
        {
            for (double floorDeg = 85.0; floorDeg > g.trigger; floorDeg -= g.floorGrid)
            {
                auto wp = g.router->ShortestPathWeightedFiltered(g.gsA, g.gsB, [floorDeg](uint32_t a, uint32_t b) {
                    if (a >= g.n && b >= g.n) return false;
                    if (a < g.n && b < g.n) return true;
                    return ReadElev(std::min(a, b), std::max(a, b)) >= floorDeg;
                });
                ++g.dijkstra;
                if (!wp.path.empty() && wp.total_weight <= (1.0 + g.eps) * base.total_weight)
                {
                    p = wp.path;
                    break;
                }
            }
        }
        else
        {
            p = g.router->ShortestPathWeighted(g.gsA, g.gsB).path;
            ++g.dijkstra;
        }
    }
    else if (g.policy == "elev")
    {
        auto wp = g.router->ShortestPathWeightedFiltered(g.gsA, g.gsB, [](uint32_t a, uint32_t b) {
            if (a >= g.n && b >= g.n)
            {
                return false;
            }
            if (a < g.n && b < g.n)
            {
                return true; // inter-satellite link
            }
            const uint32_t s = std::min(a, b), gs = std::max(a, b);
            return ReadElev(s, gs) >= g.trigger;
        });
        ++g.dijkstra;
        if (wp.path.empty())
        {
            wp = g.router->ShortestPathWeighted(g.gsA, g.gsB);
            ++g.dijkstra;
        }
        p = wp.path;
    }
    else if (g.policy == "asr" || g.policy == "asrk")
    {
        if (g.policy == "asrk" && PathUp(g.path) && lifeOf(g.path) >= g.window + g.tAcq)
        {
            // keep a route that still lasts the window, while its stretch is within the keep bound
            double w = 0;
            for (size_t k = 0; k + 1 < g.path.size(); ++k)
            {
                w += g.router->EdgeWeight(g.path[k], g.path[k + 1]);
            }
            const double best = g.router->ShortestPathWeighted(g.gsA, g.gsB).total_weight;
            ++g.dijkstra;
            if (w <= (1.0 + g.keepEps) * best)
            {
                Schedule(g.window);
                return;
            }
        }
        auto wp = g.router->ShortestPathWeightedFiltered(g.gsA, g.gsB, [](uint32_t a, uint32_t b) {
            return Residual(a, b) >= g.window + g.tAcq;
        });
        ++g.dijkstra;
        ++g.mergedDecisions;
        if (wp.path.empty())
        {
            ++g.fallbackUsed;
            // nothing lasts the window: fall back to the longest-lived route within fallbackEps, repair on a break (OnDown)
            auto lp = g.router->LifetimeAwarePath(g.gsA, g.gsB, g.fallbackEps, &Residual, g.guard + 2 * g.tAcq);
            g.dijkstra += lp.dijkstra_runs;
            wp.path = lp.path;
        }
        p = wp.path;
        Schedule(g.window);
    }
    else
    {
        p = g.router->ShortestPathWeighted(g.gsA, g.gsB).path;
        ++g.dijkstra;
        if (g.policy == "snap")
        {
            Schedule(g.period);
        }
    }
    if (p.empty())
    {
        Schedule(0.5); // no route now: retry shortly
        return;
    }
    Log("decide " + PathStr(p));
    if (g.path.empty())
    {
        InstallPath(p); // the first route is in place before traffic starts
        return;
    }
    Switch(p);
}

void
OnUp(ContactEvent ev)
{
    const Edge e = Canon(ev.node_a, ev.node_b);
    g.endCache.erase(e);
    g.satCache.erase(e);
    auto it = g.links.find(e);
    if (it != g.links.end() && !g.blocked.count(e))
    {
        it->second.emA->SetRate(0.0);
        it->second.emB->SetRate(0.0);
    }
}

/// The acquisition has finished: terminals that were blocked serve their links again if those are still up.
void
Unblock()
{
    for (const Edge& e : g.blocked)
    {
        auto it = g.links.find(e);
        if (it != g.links.end() && g.router->HasEdge(e.first, e.second))
        {
            it->second.emA->SetRate(0.0);
            it->second.emB->SetRate(0.0);
        }
    }
    g.blocked.clear();
}

void
OnUpdate(ContactEvent ev)
{
    // Only links that carry the flow need their delay kept current; a link
    // joining the path gets its delay when the path is installed.
    const Edge e = Canon(ev.node_a, ev.node_b);
    if (!g.pathEdges.count(e))
    {
        return;
    }
    auto it = g.links.find(e);
    if (it != g.links.end())
    {
        it->second.ch->SetAttribute("Delay", TimeValue(Seconds(ev.range_m / kC)));
    }
}

void
OnDown(ContactEvent ev)
{
    const Edge e = Canon(ev.node_a, ev.node_b);
    g.endCache.erase(e);
    g.satCache.erase(e);
    auto it = g.links.find(e);
    if (it != g.links.end())
    {
        it->second.emA->SetRate(1.0);
        it->second.emB->SetRate(1.0);
    }
    if (!g.pathEdges.count(e))
    {
        return;
    }
    ++g.pathBreaks;
    (e.second == g.gsA ? g.breaksIn : e.second == g.gsB ? g.breaksOut : g.breaksIsl)++;
    Log("break " + std::to_string(e.first) + "-" + std::to_string(e.second));
    // A link of the active path dropped. Every policy except snap and asr
    // reacts; life only gets here if its prediction missed.
    if (g.policy == "ondemand" || g.policy == "elev" || g.policy == "elevb" || g.policy == "asr" ||
        g.policy == "asrk" || g.policy == "track" || g.policy == "alpr")
    {
        Schedule(g.detect); // the new ground link is then acquired in Switch, as for every policy
    }
    else if (g.policy == "life" || g.policy == "mrv")
    {
        Schedule(0.0);
    }
}

void
ProbePath(double dt)
{
    // Path health and delay stretch, sampled every dt.
    bool ok = !g.path.empty();
    double w = 0.0;
    for (size_t k = 0; ok && k + 1 < g.path.size(); ++k)
    {
        const double x = g.router->EdgeWeight(g.path[k], g.path[k + 1]);
        ok = !std::isnan(x);
        w += ok ? x : 0.0;
    }
    if (!ok)
    {
        g.brokenS += dt;
    }
    else
    {
        const double best = g.router->ShortestPathWeighted(g.gsA, g.gsB).total_weight;
        if (std::isfinite(best) && best > 0)
        {
            g.stretch.push_back(w / best - 1.0);
            g.routeLenSum += w;
            ++g.routeLenN;
            if (g.hold)
            {
                ++g.dijkstra; // the hold check needs this shortest path
                if (w / best - 1.0 > g.eps && !g.pendingInstall.IsPending())
                {
                    ++g.holdSwitches;
                    Schedule(0.0);
                }
            }
        }
    }
    Simulator::Schedule(Seconds(dt), &ProbePath, dt);
}

void
DumpReach(double dt)
{
    // The longest admissible lifetime available now, for the offline optimum of (P1).
    auto lp = g.router->LifetimeAwarePath(g.gsA, g.gsB, g.eps, &Residual, g.guard + 2 * g.tAcq);
    const double best = g.router->ShortestPathWeighted(g.gsA, g.gsB).total_weight;
    g.reach << Simulator::Now().GetSeconds() << "," << (lp.path.empty() ? 0.0 : lp.lifetime_s) << ","
            << lp.total_weight << "," << best << "\n";
    Simulator::Schedule(Seconds(dt), &DumpReach, dt);
}

void
OnRx(Ptr<const Packet> pkt, const Address&)
{
    Ptr<Packet> c = pkt->Copy();
    SeqTsHeader h;
    c->RemoveHeader(h);
    g.delays.push_back((Simulator::Now() - h.GetTs()).GetSeconds());
    ++g.rx;
}

double
Pct(std::vector<double> v, double q)
{
    if (v.empty())
    {
        return NAN;
    }
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, static_cast<size_t>(q * v.size()))];
}

} // namespace

int
main(int argc, char* argv[])
{
    uint32_t planes = 72;
    uint32_t perPlane = 22;
    uint32_t phasing = 39;
    double inc = 53.0;
    double altKm = 550.0;
    bool star = false;
    double latA = 51.5;
    double lonA = -0.13;
    double latB = 40.7;
    double lonB = -74.0;
    double minElev = 25.0;
    double duration = 600.0;
    // Wave 6B: routing state at <= 100 ms (Hypatia, IMC 2020: a 1 s step
    // misses path changes for 6 % of pairs, 100 ms for 0.4 %). The sampler
    // ticks every 100 ms, ISL ranges (hence link delays) refresh every 100 ms,
    // and contact transitions are event-driven: bisected to --transitionResMs
    // and fired at their own instant instead of on the tick that saw them.
    // --tick=0.5 --islRefresh=1 --transitionResMs=0 reproduces the pre-Wave-6B
    // configuration.
    double tick = 0.1;
    double transitionResMs = 1.0;
    double rate = 20.0;
    double startOffset = 0.0;
    double islCapKm = 5000.0;
    g.policy = "life";

    CommandLine cmd(__FILE__);
    cmd.AddValue("planes", "Walker planes P", planes);
    cmd.AddValue("perPlane", "satellites per plane S", perPlane);
    cmd.AddValue("phasing", "Walker phasing F", phasing);
    cmd.AddValue("inc", "inclination (deg)", inc);
    cmd.AddValue("altKm", "altitude (km)", altKm);
    cmd.AddValue("star", "Walker star (planes over 180 deg) instead of delta", star);
    cmd.AddValue("latA", "ground station A latitude", latA);
    cmd.AddValue("lonA", "ground station A longitude", lonA);
    cmd.AddValue("latB", "ground station B latitude", latB);
    cmd.AddValue("lonB", "ground station B longitude", lonB);
    cmd.AddValue("minElev", "ground-link elevation mask (deg)", minElev);
    cmd.AddValue("trigger", "elev policy: switch when an end satellite sinks below this elevation (deg)", g.trigger);
    cmd.AddValue("duration", "simulated time (s)", duration);
    cmd.AddValue("tick", "contact sampling interval (s)", tick);
    cmd.AddValue("transitionResMs",
                 "contact transitions bisected to this resolution and fired at their instant (ms); "
                 "0 = on the sampling grid (pre-Wave-6B)",
                 transitionResMs);
    cmd.AddValue("rate", "UDP packets per second", rate);
    cmd.AddValue("startOffset", "seconds after the element epoch at which the run starts", startOffset);
    cmd.AddValue("islCapKm", "ISL range cap (km)", islCapKm);
    cmd.AddValue("policy", "snap | ondemand | asr | life", g.policy);
    cmd.AddValue("period", "snap: recomputation period (s)", g.period);
    cmd.AddValue("detect", "ondemand: failure detection delay (s)", g.detect);
    cmd.AddValue("setup", "ondemand: route setup delay (s)", g.setup);
    cmd.AddValue("window", "asr: merged-snapshot window (s)", g.window);
    cmd.AddValue("eps", "life: delay stretch bound", g.eps);
    cmd.AddValue("guard", "life: switch this long before predicted expiry (s)", g.guard);
    cmd.AddValue("tAcq", "acquisition time of a new ground link, every policy (s)", g.tAcq);
    cmd.AddValue("hold", "life: also switch when the held route's stretch exceeds eps", g.hold);
    double predSigmaKm = 0.0;
    cmd.AddValue("predSigmaKm", "along-track error (1 sigma, km) of the ephemeris the predictor uses", predSigmaKm);
    double crossLat = 0.0;
    cmd.AddValue("crossLatDeg", "cross-plane ISLs off above this latitude (deg), 0 = never", crossLat);
    std::string reachOut;
    cmd.AddValue("reachOut", "write the longest admissible lifetime every second to this CSV", reachOut);
    cmd.AddValue("noData", "control plane only: no IP stack, no packets (same decisions, loss not measured)", g.noData);
    cmd.AddValue("bbm", "one terminal per gateway: old ground link stops when acquisition starts", g.bbm);
    cmd.AddValue("fallbackEps", "asr, asrk: stretch bound of the longest-lived fallback route", g.fallbackEps);
    cmd.AddValue("keepEps", "asrk: keep a route only while its stretch is within this", g.keepEps);
    cmd.AddValue("trackArc", "track: arc of the elevation track (s)", g.trackArc);
    cmd.AddValue("gamma", "isasr: gamma of eq. (14), dimensionless", g.gammaIsasr);
    cmd.AddValue("isasrIslAct", "isasr: activeness cost on inter-satellite links too, as in Algorithm 3", g.isasrIslAct);
    cmd.AddValue("proactive", "alpr: start the switch before the known end of the route", g.proactive);
    cmd.AddValue("elevNoiseDeg", "standard deviation of measured elevation per reading (deg)", g.elevNoiseDeg);
    cmd.AddValue("trackSamples", "track: readings over the arc, fitted by least squares", g.trackSamples);
    cmd.AddValue("floorGrid", "elevb: grid of the elevation floor (deg)", g.floorGrid);
    cmd.AddValue("logEvents", "print decisions, installs and breaks to stderr", g.logEvents);
    double islRefresh = 0.1;
    cmd.AddValue("islRefresh", "refresh interval of inter-satellite ranges (s), 0 = every tick", islRefresh);
    std::string tleFile, edgeFile;
    double epochUnix = 1792800000.0;
    cmd.AddValue("tle", "real shell: TLE file (3-line sets), propagated with Vallado SGP4", tleFile);
    cmd.AddValue("edges", "real shell: +Grid candidates, lines 'a b cross'", edgeFile);
    cmd.AddValue("epochUnix", "Unix time of simulation start before startOffset", epochUnix);
    cmd.Parse(argc, argv);

    // ---- shell
    WalkerConfig wc;
    wc.num_planes = planes;
    wc.total_sats = planes * perPlane;
    wc.phasing_f = phasing;
    wc.inclination_deg = inc;
    wc.altitude_km = altKm;
    wc.epoch_unix_s = 1792800000.0; // 2026-10-24
    auto els = star ? WalkerConstellation::BuildStar(wc) : WalkerConstellation::BuildDelta(wc);
    std::vector<ns3::ntncon::TleRecord> tles;
    if (!tleFile.empty())
    {
        std::ifstream tf(tleFile);
        std::stringstream body;
        body << tf.rdbuf();
        ns3::ntncon::ParseTleStream(body.str(), tles);
        els.assign(tles.size(), els[0]);
    }
    g.n = els.size();
    g.gsA = g.n;
    g.gsB = g.n + 1;
    std::vector<Ptr<Sgp4MobilityModel>> sats;
    g.sched = CreateObject<ContactGraphScheduler>();
    g.sched->SetSamplingInterval(Seconds(tick));
    g.sched->SetTransitionResolution(Seconds(transitionResMs * 1e-3));
    g.sched->SetMinElevationDeg(minElev);
    g.sched->SetMaxIslRangeM(islCapKm * 1e3);
    for (uint32_t k = 0; k < g.n; ++k)
    {
        auto m = CreateObject<Sgp4MobilityModel>();
        if (tles.empty())
        {
            m->SetElements(els[k]);
            m->SetSimulationEpochUnix(wc.epoch_unix_s + startOffset);
        }
        else
        {
            m->SetTle(tles[k]);
            m->SetUseVallado(true);
            NS_ABORT_MSG_IF(!m->IsUsingSgp4(), "SGP4 init failed for satellite " << k);
            m->SetSimulationEpochUnix(epochUnix + startOffset);
        }
        sats.push_back(m);
        g.sched->RegisterSatellite(k, m);
    }
    g.satMob = sats;
    g.gsLat[0] = latA; g.gsLon[0] = lonA; g.gsLat[1] = latB; g.gsLon[1] = lonB;
    g.sched->RegisterGroundStation(g.gsA, latA, lonA);
    g.sched->RegisterGroundStation(g.gsB, latB, lonB);

    // +Grid candidates: ring neighbour in the plane, same slot in the next
    // plane (a delta shell closes the seam with the phasing offset; a star
    // shell has no link across its counter-rotating seam).
    std::vector<Edge> isl;
    std::set<Edge> cross;
    if (!edgeFile.empty())
    {
        std::ifstream ef(edgeFile);
        uint32_t a, b;
        int isCross;
        while (ef >> a >> b >> isCross)
        {
            isl.push_back(Canon(a, b));
            if (isCross)
            {
                cross.insert(isl.back());
            }
        }
    }
    else
    for (uint32_t p = 0; p < planes; ++p)
    {
        for (uint32_t s = 0; s < perPlane; ++s)
        {
            const uint32_t a = p * perPlane + s;
            isl.push_back(Canon(a, p * perPlane + (s + 1) % perPlane));
            if (p + 1 < planes)
            {
                isl.push_back(Canon(a, (p + 1) * perPlane + s));
                cross.insert(isl.back());
            }
            else if (!star)
            {
                isl.push_back(Canon(a, (s + phasing) % perPlane));
                cross.insert(isl.back());
            }
        }
    }
    // The predictor: the same orbits, or orbits with an along-track error of predSigmaKm (1 sigma)
    // per satellite, as an operator's element sets would carry.
    g.predSched = g.sched;
    if (predSigmaKm > 0)
    {
        g.predSched = CreateObject<ContactGraphScheduler>();
        g.predSched->SetMinElevationDeg(minElev);
        g.predSched->SetMaxIslRangeM(islCapKm * 1e3);
        Ptr<NormalRandomVariable> err = CreateObject<NormalRandomVariable>();
        for (uint32_t k = 0; k < g.n; ++k)
        {
            auto el = els[k];
            el.mean_anomaly_rad += err->GetValue(0.0, 1.0) * predSigmaKm * 1e3 / el.semi_major_axis_m;
            auto m = CreateObject<Sgp4MobilityModel>();
            m->SetElements(el);
            m->SetSimulationEpochUnix(wc.epoch_unix_s + startOffset);
            g.predSched->RegisterSatellite(k, m);
        }
        g.predSched->RegisterGroundStation(g.gsA, latA, lonA);
        g.predSched->RegisterGroundStation(g.gsB, latB, lonB);
    }
    g.sched->SetCrossPlaneLatCutoffDeg(crossLat);
    g.predSched->SetCrossPlaneLatCutoffDeg(crossLat);
    for (const auto& e : isl)
    {
        g.sched->RegisterIslCandidate(e.first, e.second, cross.count(e) > 0);
        if (g.predSched != g.sched)
        {
            g.predSched->RegisterIslCandidate(e.first, e.second, cross.count(e) > 0);
        }
    }

    // Ground links that can come up during the run: scan ahead.
    std::vector<Edge> gsl;
    for (uint32_t k = 0; k < g.n; ++k)
    {
        for (uint32_t gs : {g.gsA, g.gsB})
        {
            const double lat = gs == g.gsA ? latA : latB;
            const double lon = gs == g.gsA ? lonA : lonB;
            for (double t = 0; t <= duration + 30; t += 10.0)
            {
                Vector p;
                Vector v;
                sats[k]->GetEcefStateAt(Seconds(t), p, v);
                const double r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                const double la = lat * M_PI / 180;
                const double lo = lon * M_PI / 180;
                const double cosc =
                    (p.x * std::cos(la) * std::cos(lo) + p.y * std::cos(la) * std::sin(lo) +
                     p.z * std::sin(la)) / r;
                // Within the cap of a (minElev - 5) deg mask, with margin.
                const double e = (minElev - 5) * M_PI / 180;
                const double psi = std::acos(6378137.0 * std::cos(e) / r) - e;
                if (cosc >= std::cos(psi))
                {
                    gsl.emplace_back(k, gs);
                    break;
                }
            }
        }
    }

    g.sched->SetGslCandidates(gsl);
    g.noise = CreateObject<NormalRandomVariable>();
    // Inter-satellite ranges move slowly and no +Grid link drops under the range cap, so they are refreshed every
    // islRefresh seconds; with a latitude cutoff the links do drop, so those runs sample on every tick.
    g.sched->SetIslRefreshEvery(crossLat > 0 || islRefresh <= 0 ? 1 : static_cast<uint32_t>(std::lround(islRefresh / tick)));
    g.dropDeg = minElev - 2.0;
    g.horizon = 1800.0;

    // ---- data plane
    InternetStackHelper stack;
    Ipv4StaticRoutingHelper srh;
    if (!g.noData)
    {
    g.nodes.Create(g.n + 2);
    stack.SetRoutingHelper(srh);
    stack.Install(g.nodes);
    }
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("100Mbps"));
    p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(5)));
    p2p.SetQueue("ns3::DropTailQueue<Packet>", "MaxSize", StringValue("200p"));
    Ipv4AddressHelper addr("10.0.0.0", "255.255.255.252");
    auto mk = [&](const Edge& e) {
        NetDeviceContainer d = p2p.Install(g.nodes.Get(e.first), g.nodes.Get(e.second));
        Ipv4InterfaceContainer ic = addr.Assign(d);
        addr.NewNetwork();
        Link l;
        l.ch = DynamicCast<PointToPointChannel>(d.Get(0)->GetChannel());
        l.emA = CreateObject<RateErrorModel>();
        l.emB = CreateObject<RateErrorModel>();
        for (auto em : {l.emA, l.emB})
        {
            em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
            em->SetRate(1.0); // down until the scheduler says up
        }
        d.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(l.emA));
        d.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(l.emB));
        l.ifA = ic.Get(0).second;
        l.ifB = ic.Get(1).second;
        l.ipA = ic.GetAddress(0);
        l.ipB = ic.GetAddress(1);
        g.links[e] = l;
    };
    if (!g.noData)
    {
    for (const auto& e : isl)
    {
        mk(e);
    }
    for (const auto& e : gsl)
    {
        mk(e);
    }
    }
    // Destination address: B's address on its first ground link.
    for (const auto& [e, l] : g.links)
    {
        if (e.second == g.gsB)
        {
            g.dst = l.ipB;
            break;
        }
    }

    g.router = CreateObject<ContactGraphRouter>();
    g.router->Attach(g.sched);
    g.sched->m_contactUp.ConnectWithoutContext(MakeCallback(&OnUp));
    g.sched->m_contactDown.ConnectWithoutContext(MakeCallback(&OnDown));
    g.sched->m_contactUpdate.ConnectWithoutContext(MakeCallback(&OnUpdate));
    g.sched->Start();

    // ---- traffic
    const uint16_t port = 9000;
    const double t0 = 5.0;
    const uint64_t nPk = static_cast<uint64_t>((duration - t0) * rate);
    if (!g.noData)
    {
    PacketSinkHelper sinkH("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
    auto sinkApp = sinkH.Install(g.nodes.Get(g.gsB));
    sinkApp.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&OnRx));
    sinkApp.Start(Seconds(0));
    UdpClientHelper cli(g.dst, port);
    cli.SetAttribute("MaxPackets", UintegerValue(nPk));
    cli.SetAttribute("Interval", TimeValue(Seconds(1.0 / rate)));
    cli.SetAttribute("PacketSize", UintegerValue(200));
    auto cliApp = cli.Install(g.nodes.Get(g.gsA));
    cliApp.Start(Seconds(t0));
    cliApp.Stop(Seconds(duration));
    }

    // Snapshot routing starts at a random phase of its clock, so its recomputations are not locked to
    // the contact sampler's ticks; the other policies start right after the first contacts are known.
    Ptr<UniformRandomVariable> phase = CreateObject<UniformRandomVariable>();
    Simulator::Schedule(Seconds(2 * tick + (g.policy == "snap" ? phase->GetValue(0, g.period) : 1e-3)),
                        &Recompute);
    if (g.policy == "elev" || g.policy == "elevb" || g.policy == "track")
    {
        Simulator::Schedule(Seconds(2 * tick + tick), &ElevCheck, tick);
    }
    // Path health is sampled on the routing-state step (was a fixed 0.5 s).
    Simulator::Schedule(Seconds(t0), &ProbePath, tick);
    // Count only from the start of traffic, so the rates per hour are over the measured span.
    Simulator::Schedule(Seconds(t0), []() {
        g.changes = g.pathBreaks = g.computations = g.dijkstra = g.predictions = g.forced = 0;
        g.ingressChanges = g.egressChanges = g.islOnlyChanges = g.acquisitions = 0;
        g.breaksIn = g.breaksOut = g.breaksIsl = g.holdSwitches = g.ageN = g.resN = g.searchN = g.noAdmissible = g.trackBad = g.fallbackUsed = g.mergedDecisions = 0;
        g.ageSum = g.resInSum = g.resOutSum = g.searchCpu = 0.0;
    });
    if (!reachOut.empty())
    {
        g.reach.open(reachOut);
        g.reach << "t,lifetime_s,weight_m,shortest_m\n";
        Simulator::Schedule(Seconds(t0), &DumpReach, 1.0);
    }
    Simulator::Stop(Seconds(duration + 1.0));
    Simulator::Run();

    const double hours = (duration - t0) / 3600.0;
    std::ostringstream js;
    js.precision(6);
    rusage ru;
    getrusage(RUSAGE_SELF, &ru);
    auto shareOver = [](double x) {
        return g.stretch.empty() ? NAN
                                 : static_cast<double>(std::count_if(g.stretch.begin(), g.stretch.end(),
                                                                     [x](double v) { return v > x; })) /
                                       g.stretch.size();
    };
    js << "{\"ver\":7,\"bbm\":" << g.bbm << ",\"no_data\":" << g.noData
       << ",\"ingress_per_h\":" << g.ingressChanges / hours << ",\"egress_per_h\":" << g.egressChanges / hours
       << ",\"isl_only_per_h\":" << g.islOnlyChanges / hours << ",\"acq_per_h\":" << g.acquisitions / hours
       << ",\"breaks_in_per_h\":" << g.breaksIn / hours << ",\"breaks_out_per_h\":" << g.breaksOut / hours
       << ",\"breaks_isl_per_h\":" << g.breaksIsl / hours << ",\"hold_per_h\":" << g.holdSwitches / hours
       << ",\"age_mean_s\":" << (g.ageN ? g.ageSum / g.ageN : NAN)
       << ",\"res_in_mean_s\":" << (g.resN ? g.resInSum / g.resN : NAN)
       << ",\"res_out_mean_s\":" << (g.resN ? g.resOutSum / g.resN : NAN)
       << ",\"search_cpu_ms\":" << (g.searchN ? 1e3 * g.searchCpu / g.searchN : NAN)
       << ",\"stretch_p50\":" << Pct(g.stretch, 0.5) << ",\"stretch_p95\":" << Pct(g.stretch, 0.95)
       << ",\"stretch_max\":" << Pct(g.stretch, 1.0) << ",\"over5\":" << shareOver(0.05)
       << ",\"over10\":" << shareOver(0.10) << ",\"cpu_s\":" << ru.ru_utime.tv_sec + 1e-6 * ru.ru_utime.tv_usec
       << ",\"no_admissible\":" << g.noAdmissible << ",\"track_bad\":" << g.trackBad
       << ",\"fallback_share\":" << (g.mergedDecisions ? static_cast<double>(g.fallbackUsed) / g.mergedDecisions : NAN)
       << ",\"route_ms_mean\":" << (g.routeLenN ? 1e3 * g.routeLenSum / g.routeLenN / kC : NAN)
       << ",\"propagator\":\"" << (sats[0]->IsUsingSgp4() ? "sgp4" : "kepler-j2") << "\""
       << ",\"tick_s\":" << tick << ",\"isl_refresh_s\":" << islRefresh
       << ",\"transition_res_ms\":" << transitionResMs
       << ",\"event_driven_transitions\":" << g.sched->EventDrivenTransitions();
    js << ",\"policy\":\"" << g.policy << "\",\"eps\":" << g.eps << ",\"period\":" << g.period
       << ",\"tacq\":" << g.tAcq << ",\"guard\":" << g.guard << ",\"hold\":" << g.hold
       << ",\"pred_sigma_km\":" << predSigmaKm << ",\"cross_lat\":" << crossLat
       << ",\"window\":" << g.window << ",\"start\":" << startOffset << ",\"sats\":" << g.n
       << ",\"gsl_links\":" << gsl.size() << ",\"isl_links\":" << isl.size()
       << ",\"isl_down_events\":" << g.sched->IslEventsDown()
       << ",\"gsl_down_events\":" << g.sched->GslEventsDown()
       << ",\"changes_per_h\":" << g.changes / hours
       << ",\"breaks_per_h\":" << g.pathBreaks / hours
       << ",\"forced_per_h\":" << g.forced / hours
       << ",\"computations_per_h\":" << g.computations / hours
       << ",\"dijkstra_per_h\":" << g.dijkstra / hours
       << ",\"predictions_per_h\":" << g.predictions / hours << ",\"sent\":" << nPk
       << ",\"received\":" << g.rx << ",\"loss\":" << 1.0 - static_cast<double>(g.rx) / nPk
       << ",\"broken_share\":" << g.brokenS / (duration - t0)
       << ",\"stretch_mean\":"
       << (g.stretch.empty() ? NAN
                             : std::accumulate(g.stretch.begin(), g.stretch.end(), 0.0) /
                                   g.stretch.size())
       << ",\"stretch_p99\":" << Pct(g.stretch, 0.99) << ",\"delay_ms_mean\":"
       << (g.delays.empty() ? NAN
                            : 1e3 * std::accumulate(g.delays.begin(), g.delays.end(), 0.0) /
                                  g.delays.size())
       << ",\"delay_ms_p99\":" << 1e3 * Pct(g.delays, 0.99) << "}";
    std::cout << js.str() << std::endl;
    Simulator::Destroy();
    return 0;
}
