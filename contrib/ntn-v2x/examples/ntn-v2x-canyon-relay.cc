/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-v2x-canyon-relay: vehicles in an urban street canyon report status messages to a server through
 * LEO satellites. A vehicle whose line of sight to every usable satellite is blocked by the facades can
 * name a neighbor as relay in the message it broadcasts over NR sidelink Mode 2; the neighbor forwards
 * it over its own satellite uplink.
 *
 * Geometry. The street runs along the local x axis at azimuth --streetAzDeg from north, at
 * (--lat, --lon). Its walls are at y = 0 and y = --width; each side is a row of buildings of length
 * --bldLen whose heights are drawn once per run from a lognormal law (median --h50, log-sd --hSigma).
 * Satellites fly their own element sets through SGP4 (--tle). A vehicle at (x, y) with antenna height
 * 1.5 m sees a satellite at elevation e and street-relative azimuth a if the ray clears the facade it
 * crosses: the ray reaches the wall on the side of sin(a) after s = d_perp / |sin a| and clears it iff
 * H < 1.5 + s tan(e). No index parity, no random draw per packet: line of sight is geometry.
 *
 * Data plane. Every vehicle has a point-to-point IPv4 link to the server standing for its satellite
 * path; the link's delay follows the slant range of the serving satellite plus a feeder hop to a
 * gateway at --gwLat/--gwLon, and its receive error model drops everything while the vehicle has no
 * usable satellite under the policy. The sidelink is the module's NR Mode-2 MAC (sensing-based
 * semi-persistent scheduling, half duplex, collisions), one transmission per reservation period.
 *
 * Policies (--policy)
 *   direct    uplink only to the highest-elevation satellite above the mask (the serving cell)
 *   multisat  uplink to any satellite above the mask that is in line of sight (parallel links)
 *   relay     multisat, and when blocked name the nearest neighbor that announced an uplink in its last
 *             decoded status broadcast (neighbor table from broadcasts, entries aged out after 1 s)
 *   blind     multisat, and when blocked name the nearest neighbor in the table within --maxRelayM, with no
 *             knowledge of its line of sight
 *   blindfar  as blind, but prefer the nearest neighbor whose ray crossings all lie at least one building
 *             length from every crossing of the source's rays on the same wall (Corollary of the paper)
 *   blindlb   ablation: as blind, nearest neighbor at least one building length away along the street
 *   random    ablation: a random neighbor of the table within range
 *   randpersist ablation: a random neighbor drawn once and kept while it stays in range and heard
 *   blindbest among neighbors with crossings apart from the source's, the one whose lane gives the lowest
 *             probability of being blocked toward every satellite under the street's height law
 *   rand2, rand3  k distinct neighbors drawn afresh for every message
 *   roundrobin the next neighbor of the table in cyclic order
 *   hybrid    fresh draw among apart neighbors in the best-ranked lane (random when there is none)
 *   anyfwd    no relay is named; every neighbor within --maxRelayM that decodes the blocked message and has an uplink forwards it
 *   anyp      as anyfwd, but each such neighbor forwards with probability min(1, kappa / (1 + uplinks it knows within range))
 *
 * Added in Nov 2026: shared per-satellite uplink (--ulKbps, --grantRtt, --ulMaxQueueS), relay link setup
 * latency (--relaySetupMs), mobility from a trace (--mobTrace), facades from a file or with cross-street gaps
 * (--facadeFile, --gapEveryM, --gapLenM), urban sidelink channel (--slUrban), aligned reselection (--alignResel).
 *
 * Output: one CSV row per delivered message (src, seq, gen_s, rx_s, via), and a summary line.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-nr-sidelink.h"
#include "ns3/point-to-point-module.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/orbital-elements.h"

#include <algorithm>
#include <array>
#include <deque>
#include <cmath>
#include <fstream>
#include <map>
#include <cstdio>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnV2xCanyonRelay");

namespace
{
constexpr double kC = 299792458.0;
constexpr double kAnt = 1.5;
constexpr uint32_t kNoRelay = 0xFFFFFFFF;
constexpr uint32_t kAny = 0xFFFFFFFE;
constexpr uint32_t kList = 0xFFFFFFFD; // the broadcast names a list of neighbors (g_named)
constexpr uint16_t kPort = 7000;

struct Veh
{
    double x0, y, v;
    Ptr<ConstantPositionMobilityModel> mob;
    Ptr<NtnSlUeMac> mac;
    Ptr<Socket> sock;
    Ptr<PointToPointChannel> chan;
    Ptr<RateErrorModel> em;
    bool up{false};          // uplink usable now under the policy
    bool advertised{false};  // unused (kept for the summary line); state now comes from decoded broadcasts
    struct Nb
    {
        bool up;
        double x, y, t;
    };
    std::map<uint32_t, Nb> nb; // neighbors heard on the sidelink: uplink flag, position, lane, reception time
    double slant{0};
    int sat{-1};          // satellite carrying the uplink
    double hoUntil{0};    // uplink interrupted until this time after a satellite change
    uint32_t seq{0};
    uint64_t gen{0}, sentDirect{0}, relayed{0}, relayAsked{0};
    uint32_t persist{0xFFFFFFFF}; // randpersist: the neighbor drawn once and kept while it stays usable
    uint32_t rrLast{0};           // roundrobin: the neighbor named last
    uint32_t linked{0xFFFFFFFF};  // relay setup: the neighbor the source has a link with
    uint64_t blockedGen{0};       // messages generated while the uplink was down
    int dir{1};                   // +1 or -1 along the street
    std::vector<float> traceS;    // cumulative distance driven per trace sample (--mobTrace)
};

std::vector<Veh> g_v;
std::vector<Ptr<ntncon::Sgp4MobilityModel>> g_sats;
std::vector<double> g_hLeft, g_hRight;
std::string g_policy = "relay";
double g_width = 20, g_len = 1200, g_bld = 30, g_mask = 20, g_maxRelay = 300;
double g_hoS = 0.0; // uplink interruption at each satellite change (s)
double g_ulKbps = 0;       // capacity of one satellite's uplink left for this street (kb/s); 0 = no limit
double g_grantRtt = 0;     // round trips from a scheduling request to the grant
double g_ulMaxQ = 1.0;     // largest wait for an uplink opportunity (s); later messages are dropped
double g_setupS = 0;       // latency of a link to a relay the source did not name last (s)
double g_nbAge = 1.0;      // neighbor table entries age out after this time (s)
double g_kappa = 1.0;      // anyp: expected number of forwards
double g_traceDt = 1.0;
uint32_t g_relayK = 1;     // relayk, relayrand, randk: how many neighbors a message names
bool g_ulPrio = false;     // uplink discipline: a vehicle's own messages before forwards, instead of first come first served
double g_vMin = 6.0, g_vMax = 14.0; // constant-speed range (m/s)
std::map<uint32_t, std::vector<uint32_t>> g_named; // per source, the neighbors named by its current message
struct UlItem
{
    uint32_t i;
    Ptr<Packet> p;
    uint16_t port;
    double t0;
};
std::map<int, std::array<std::deque<UlItem>, 2>> g_ulQ; // per satellite: own messages, forwards
std::map<int, bool> g_ulBusy;
bool g_useTrace = false;
double g_gapEvery = 0, g_gapLen = 0;
std::vector<float> g_rasterL, g_rasterR; // facades from a file, per meter along the street
std::vector<double> g_satBusy;           // per satellite: the time its uplink is next free
uint64_t g_ulDropQ = 0, g_ulDropLos = 0, g_ulSent = 0, g_ulBytes = 0;
double g_ulWaitSum = 0;
double g_h50 = 18, g_hSig = 0.5; // facade height law known to the vehicles (blindbest)
uint64_t g_farFallback = 0;      // blindfar or blindbest requests that found no apart neighbor
double g_lat = 40.75, g_lon = -73.99, g_az = 29, g_gwLat = 40.0, g_gwLon = -75.0;
Vector g_origin, g_east, g_north, g_up, g_gw;
std::vector<std::pair<double, double>> g_visEA; // (elevation, street-relative azimuth) of satellites above the mask
Ipv4Address g_srvAddr;
std::ofstream g_out;
std::ofstream g_trace; // optional per-tick link states of the first vehicles (--traceOut)

Vector
Ecef(double latDeg, double lonDeg, double h = 0)
{
    const double a = 6378137.0, e2 = 6.69437999014e-3;
    const double la = latDeg * M_PI / 180, lo = lonDeg * M_PI / 180;
    const double n = a / std::sqrt(1 - e2 * std::sin(la) * std::sin(la));
    return Vector((n + h) * std::cos(la) * std::cos(lo), (n + h) * std::cos(la) * std::sin(lo),
                  (n * (1 - e2) + h) * std::sin(la));
}

double
Dot(const Vector& a, const Vector& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

double
PosX(const Veh& v)
{
    // wrapped into [0, len): fmod keeps the sign, so vehicles driving in -x would otherwise get negative
    // positions and wrong neighbor distances and sidelink geometry
    double x0 = v.x0 + v.v * Simulator::Now().GetSeconds();
    if (g_useTrace)
    {
        const double u = Simulator::Now().GetSeconds() / g_traceDt;
        const size_t k = std::min(static_cast<size_t>(u), v.traceS.size() - 2);
        const double f = std::min(1.0, u - k);
        x0 = v.x0 + v.dir * (v.traceS[k] + f * (v.traceS[k + 1] - v.traceS[k]));
    }
    double x = std::fmod(x0, g_len);
    return x < 0 ? x + g_len : x;
}

/// Facade height on the side the ray reaches, at along-street position xc.
double
Facade(double xc, bool right)
{
    const auto& h = right ? g_hRight : g_hLeft;
    double m = std::fmod(xc, g_len);
    if (m < 0)
    {
        m += g_len;
    }
    if (!g_rasterL.empty())
    {
        const auto& ra = right ? g_rasterR : g_rasterL;
        return ra[std::min(static_cast<size_t>(m), ra.size() - 1)];
    }
    if (g_gapEvery > 0 && std::fmod(m, g_gapEvery) < g_gapLen)
    {
        return 0.0; // a cross street opens the sky
    }
    return h[static_cast<size_t>(m / g_bld) % h.size()];
}

bool
Los(double x, double y, double e, double a)
{
    const double sa = std::sin(a);
    if (std::abs(sa) < 1e-6)
    {
        return true;
    }
    const bool right = sa > 0;
    const double s = (right ? g_width - y : y) / std::abs(sa);
    return Facade(x + s * std::cos(a), right) < kAnt + s * std::tan(e);
}

struct SatView
{
    double e, a, range;
    int id;
};

/// Elevation, street-relative azimuth and range of every satellite above the mask.
std::vector<SatView>
Visible()
{
    std::vector<SatView> out;
    for (size_t si = 0; si < g_sats.size(); ++si)
    {
        auto& s = g_sats[si];
        const Vector p = s->GetEcefPosition();
        const Vector d(p.x - g_origin.x, p.y - g_origin.y, p.z - g_origin.z);
        const double r = std::sqrt(Dot(d, d));
        const double e = std::asin(Dot(d, g_up) / r);
        if (e < g_mask * M_PI / 180)
        {
            continue;
        }
        const double az = std::atan2(Dot(d, g_east), Dot(d, g_north));
        out.push_back({e, az - g_az * M_PI / 180, r, static_cast<int>(si)});
    }
    return out;
}

void
Tick(double dt)
{
    const auto vis = Visible();
    g_visEA.clear();
    for (const auto& s : vis)
    {
        g_visEA.emplace_back(s.e, s.a);
    }
    // the serving cell: highest elevation
    int best = -1;
    for (size_t k = 0; k < vis.size(); ++k)
    {
        if (best < 0 || vis[k].e > vis[best].e)
        {
            best = static_cast<int>(k);
        }
    }
    for (auto& v : g_v)
    {
        const double x = PosX(v);
        v.mob->SetPosition(Vector(x, v.y, kAnt));
        bool up = false;
        double slant = 0;
        int sat = -1;
        if (g_policy == "direct")
        {
            if (best >= 0 && Los(x, v.y, vis[best].e, vis[best].a))
            {
                up = true;
                slant = vis[best].range;
                sat = vis[best].id;
            }
        }
        else
        {
            double be = -1;
            for (const auto& s : vis)
            {
                if (s.e > be && Los(x, v.y, s.e, s.a))
                {
                    be = s.e;
                    slant = s.range;
                    up = true;
                    sat = s.id;
                }
            }
        }
        // a change of the uplink satellite costs an interruption (handover, synchronization, random access)
        const double now = Simulator::Now().GetSeconds();
        if (up && sat != v.sat)
        {
            if (v.sat >= 0)
            {
                v.hoUntil = now + g_hoS;
            }
            v.sat = sat;
        }
        up = up && now >= v.hoUntil;
        v.up = up;
        v.em->SetRate(up ? 0.0 : 1.0);
        if (g_trace.is_open() && &v - g_v.data() < 8)
        {
            g_trace << Simulator::Now().GetSeconds() << "," << (&v - g_v.data()) << "," << x << "," << v.y << ","
                    << (best >= 0 ? vis[best].e : -1) << "," << (best >= 0 ? vis[best].a : 0) << "," << up << "\n";
        }
        if (up)
        {
            // slant to the satellite plus its feeder hop to the gateway (taken at the same slant)
            v.chan->SetAttribute("Delay", TimeValue(Seconds(2 * slant / kC)));
            v.slant = slant;
        }
    }
    Simulator::Schedule(Seconds(dt), &Tick, dt);
}


/// Along-street crossing offset of the ray toward satellite (e, a) from lane y: s cos a, s = d_perp / |sin a|.
double
CrossOffset(double y, double a)
{
    const double sa = std::sin(a);
    if (std::abs(sa) < 1e-6)
    {
        return 0.0;
    }
    const double d = sa > 0 ? g_width - y : y;
    return d / std::abs(sa) * std::cos(a);
}

/// Relay choice from what the source has decoded on the sidelink in the last second: each neighbor's
/// position, lane and uplink flag, carried in its status broadcast. Distances are along the line, as the
/// sidelink channel sees them. relay: nearest neighbor that announced an uplink; blind: nearest neighbor;
/// blindfar: nearest neighbor whose rays cross a different building than the source's for every satellite
/// in view, |dx + c(y2) - c(y1)| >= bldLen (Corollary), falling back to the nearest.
uint32_t
ChooseRelay(uint32_t src)
{
    const double now = Simulator::Now().GetSeconds();
    const double xs = PosX(g_v[src]);
    const double ys = g_v[src].y;
    uint32_t pick = kNoRelay, pickFar = kNoRelay, pickLb = kNoRelay, pickBest = kNoRelay;
    double bestD = 1e18, bestFar = 1e18, bestLb = 1e18, bestQ = 2.0, bestQD = 1e18, worstD = -1.0;
    uint32_t pickFarthest = kNoRelay;
    std::vector<uint32_t> cand;
    std::vector<std::pair<double, uint32_t>> byD;
    std::vector<std::pair<uint32_t, double>> apartQ; // hybrid: apart neighbors and the score of their lane
    const bool needApart = g_policy == "blindfar" || g_policy == "blindbest" || g_policy == "hybrid";
    for (const auto& [j, n] : g_v[src].nb)
    {
        if (now - n.t > g_nbAge)
        {
            continue; // not heard for a second: stale
        }
        const double dxs = n.x - xs; // signed, along the line
        const double d = std::hypot(dxs, n.y - ys);
        if (d > g_maxRelay)
        {
            continue;
        }
        if ((g_policy == "relay" || g_policy == "relayk" || g_policy == "relayrand" || g_policy == "relayfar" || g_policy == "relaylb") && !n.up)
        {
            continue;
        }
        if (d < bestD)
        {
            bestD = d;
            pick = j;
        }
        if (d > worstD)
        {
            worstD = d;
            pickFarthest = j;
        }
        // every crossing of the neighbor's rays must lie at least one building length from every crossing of
        // the source's rays on the same wall, over all pairs of satellites in view
        bool apart = needApart;
        for (const auto& [e1, a1] : g_visEA)
        {
            if (!needApart)
            {
                break;
            }
            for (const auto& [e2, a2] : g_visEA)
            {
                if ((std::sin(a1) > 0) != (std::sin(a2) > 0))
                {
                    continue; // different walls
                }
                apart &= std::abs(dxs + CrossOffset(n.y, a1) - CrossOffset(ys, a2)) >= g_bld;
            }
        }
        // model-ranked: among neighbors whose crossings are apart from the source's, the one least likely to be
        // blocked toward every satellite, q = prod_k (1 - F_H(z_k(y2))) with the street's height law (Lemma 1)
        if (apart)
        {
            double q = 1.0;
            for (const auto& [e, a] : g_visEA)
            {
                const double sa = std::abs(std::sin(a));
                if (sa < 1e-6)
                {
                    q = 0.0;
                    break;
                }
                const double dp = std::sin(a) > 0 ? g_width - n.y : n.y;
                const double z = kAnt + dp / sa * std::tan(e);
                const double F = 0.5 * std::erfc(-std::log(z / g_h50) / (g_hSig * std::sqrt(2.0)));
                q *= 1.0 - F;
            }
            apartQ.emplace_back(j, q);
            if (q < bestQ - 1e-9 || (std::abs(q - bestQ) <= 1e-9 && d < bestQD))
            {
                bestQ = q;
                bestQD = d;
                pickBest = j;
            }
        }
        if (std::abs(dxs) >= g_bld && d < bestLb)
        {
            bestLb = d;
            pickLb = j;
        }
        cand.push_back(j);
        byD.emplace_back(d, j);
        if (apart && d < bestFar)
        {
            bestFar = d;
            pickFar = j;
        }
    }
    if (g_policy == "relayfar" && pickFarthest != kNoRelay)
    {
        return pickFarthest + 1;
    }
    if (g_policy == "relaylb")
    {
        return pickLb != kNoRelay ? pickLb + 1 : (pick == kNoRelay ? kNoRelay : pick + 1);
    }
    if (g_policy == "relayk" && !cand.empty())
    {
        std::sort(byD.begin(), byD.end());
        auto& nm = g_named[src];
        nm.clear();
        for (uint32_t m = 0; m < g_relayK && m < byD.size(); ++m)
        {
            nm.push_back(byD[m].second);
        }
        if (g_relayK <= 3)
        {
            uint32_t tagv = 0;
            for (uint32_t m = 0; m < nm.size(); ++m)
            {
                tagv |= (nm[m] + 1) << (10 * m);
            }
            nm.clear();
            return tagv;
        }
        return kList;
    }
    if ((g_policy == "relayrand" || g_policy == "randk") && !cand.empty())
    {
        static Ptr<UniformRandomVariable> ul = CreateObject<UniformRandomVariable>();
        auto& nm = g_named[src];
        nm.clear();
        for (uint32_t m = 0; m < g_relayK && !cand.empty(); ++m)
        {
            const uint32_t idx = ul->GetInteger(0, cand.size() - 1);
            nm.push_back(cand[idx]);
            cand.erase(cand.begin() + idx);
        }
        if (g_relayK <= 3)
        {
            uint32_t tagv = 0;
            for (uint32_t m = 0; m < nm.size(); ++m)
            {
                tagv |= (nm[m] + 1) << (10 * m);
            }
            nm.clear();
            return tagv;
        }
        return kList;
    }
    if ((g_policy == "rand2" || g_policy == "rand3") && !cand.empty())
    {
        static Ptr<UniformRandomVariable> uk = CreateObject<UniformRandomVariable>();
        const uint32_t k = g_policy == "rand2" ? 2 : 3;
        uint32_t tagv = 0;
        for (uint32_t m = 0; m < k && !cand.empty(); ++m)
        {
            const uint32_t idx = uk->GetInteger(0, cand.size() - 1);
            tagv |= (cand[idx] + 1) << (10 * m);
            cand.erase(cand.begin() + idx);
        }
        return tagv;
    }
    if (g_policy == "roundrobin" && !cand.empty())
    {
        auto& v = g_v[src];
        auto it = std::upper_bound(cand.begin(), cand.end(), v.rrLast); // the table is ordered by id
        v.rrLast = it == cand.end() ? cand.front() : *it;
        return v.rrLast + 1;
    }
    if (g_policy == "hybrid" && !cand.empty())
    {
        static Ptr<UniformRandomVariable> uh = CreateObject<UniformRandomVariable>();
        std::vector<uint32_t> best;
        for (const auto& [j, q] : apartQ)
        {
            if (q <= bestQ + 1e-9)
            {
                best.push_back(j);
            }
        }
        if (best.empty())
        {
            ++g_farFallback;
            return cand[uh->GetInteger(0, cand.size() - 1)] + 1;
        }
        return best[uh->GetInteger(0, best.size() - 1)] + 1;
    }
    if (g_policy == "blindfar")
    {
        if (pickFar != kNoRelay)
        {
            return pickFar + 1;
        }
        ++g_farFallback; // no apart neighbor in range: fall back to the nearest
    }
    if (g_policy == "randpersist" && !cand.empty())
    {
        auto& v = g_v[src];
        if (std::find(cand.begin(), cand.end(), v.persist) == cand.end())
        {
            static Ptr<UniformRandomVariable> u2 = CreateObject<UniformRandomVariable>();
            v.persist = cand[u2->GetInteger(0, cand.size() - 1)]; // draw once, keep while in range and heard
        }
        return v.persist + 1;
    }
    if (g_policy == "blindbest")
    {
        if (pickBest != kNoRelay)
        {
            return pickBest + 1;
        }
        ++g_farFallback; // no apart neighbor in range: fall back to the nearest
    }
    if (g_policy == "blindlb" && pickLb != kNoRelay)
    {
        return pickLb + 1; // ablation: nearest neighbor at least one building length away
    }
    if (g_policy == "random" && !cand.empty())
    {
        static Ptr<UniformRandomVariable> u = CreateObject<UniformRandomVariable>();
        return cand[u->GetInteger(0, cand.size() - 1)] + 1; // ablation: any neighbor in range
    }
    return pick == kNoRelay ? kNoRelay : pick + 1;
}

/// One uplink message of vehicle i: a scheduling request and grant take grantRtt round trips to the satellite's
/// gateway, then the message waits its turn on the satellite's shared uplink. A vehicle that has lost its uplink by
/// the time of its turn loses the message.
void
UlFinish(uint32_t i, Ptr<Packet> p, uint16_t port, double t0)
{
    Veh& v = g_v[i];
    if (!v.up)
    {
        ++g_ulDropLos;
        return;
    }
    g_ulWaitSum += Simulator::Now().GetSeconds() - t0;
    ++g_ulSent;
    g_ulBytes += p->GetSize();
    v.sock->SendTo(p, 0, InetSocketAddress(g_srvAddr, port));
}

void UlServe(int sat);

void
UlDone(uint32_t i, Ptr<Packet> p, uint16_t port, double t0, int sat)
{
    UlFinish(i, p, port, t0);
    UlServe(sat);
}

/// Strict priority, not preemptive: own messages first, each class in arrival order; a message that has waited too long is dropped.
void
UlServe(int sat)
{
    auto& q = g_ulQ[sat];
    const double now = Simulator::Now().GetSeconds();
    for (auto& cls : q)
    {
        while (!cls.empty())
        {
            UlItem it = cls.front();
            cls.pop_front();
            if (now - it.t0 > g_ulMaxQ)
            {
                ++g_ulDropQ;
                continue;
            }
            g_ulBusy[sat] = true;
            const double tx = (it.p->GetSize() + 28) * 8.0 / (g_ulKbps * 1e3);
            Simulator::Schedule(Seconds(tx), &UlDone, it.i, it.p, it.port, it.t0, sat);
            return;
        }
    }
    g_ulBusy[sat] = false;
}

void
UlEnqueue(uint32_t i, Ptr<Packet> p, uint16_t port, double t0)
{
    const int sat = g_v[i].sat;
    g_ulQ[sat][port == kPort ? 0 : 1].push_back({i, p, port, t0});
    if (!g_ulBusy[sat])
    {
        UlServe(sat);
    }
}

void
UlSend(uint32_t i, Ptr<Packet> p, uint16_t port)
{
    Veh& v = g_v[i];
    if (g_ulKbps <= 0 && g_grantRtt <= 0)
    {
        ++g_ulSent;
        g_ulBytes += p->GetSize();
        v.sock->SendTo(p, 0, InetSocketAddress(g_srvAddr, port));
        return;
    }
    const double now = Simulator::Now().GetSeconds();
    const double rtt = 2.0 * 2.0 * v.slant / kC; // service and feeder link, both ways
    double t = now + g_grantRtt * rtt;
    if (g_ulPrio && g_ulKbps > 0)
    {
        Simulator::Schedule(Seconds(t - now), &UlEnqueue, i, p, port, now);
        return;
    }
    if (g_ulKbps > 0)
    {
        double& busy = g_satBusy[v.sat];
        t = std::max(t, busy);
        if (t - now > g_ulMaxQ)
        {
            ++g_ulDropQ;
            return;
        }
        t += (p->GetSize() + 28) * 8.0 / (g_ulKbps * 1e3);
        busy = t;
    }
    Simulator::Schedule(Seconds(t - now), &UlFinish, i, p, port, now);
}

Ptr<Packet>
MakeTx(uint32_t i)
{
    Veh& v = g_v[i];
    const uint32_t seq = v.seq++;
    ++v.gen;
    // the status message: source and sequence, generation time
    SeqTsHeader msg;
    msg.SetSeq((i << 20) | (seq & 0xFFFFF));
    Ptr<Packet> body = Create<Packet>(150);
    body->AddHeader(msg);
    if (v.up)
    {
        UlSend(i, body->Copy(), kPort);
        ++v.sentDirect;
    }
    else
    {
        ++v.blockedGen;
    }
    uint32_t relay = kNoRelay;
    const bool canRelay = g_policy == "relay" || g_policy == "blind" || g_policy == "blindfar" ||
                          g_policy == "blindlb" || g_policy == "random" || g_policy == "blindbest" ||
                          g_policy == "randpersist" || g_policy == "rand2" || g_policy == "rand3" ||
                          g_policy == "roundrobin" || g_policy == "hybrid" || g_policy == "relayk" ||
                          g_policy == "relayrand" || g_policy == "randk" ||
                          g_policy == "relayfar" || g_policy == "relaylb";
    if (!v.up && canRelay)
    {
        relay = ChooseRelay(i);
        v.relayAsked += relay != kNoRelay;
    }
    else if (!v.up && (g_policy == "anyfwd" || g_policy == "anyp" || g_policy == "anypn"))
    {
        relay = kAny;
        ++v.relayAsked;
    }
    SeqTsHeader tag; // second header: the relay this message names, or none
    tag.SetSeq(relay); // kNoRelay, kAny, one id, or up to three ids: id1 + 1 + ((id2 + 1) << 10) + ((id3 + 1) << 20)
    Ptr<Packet> sl = body->Copy();
    sl->AddHeader(tag);
    // outermost header: what neighbors learn from this broadcast, the sender's uplink flag, lane index and
    // position along the street in decimeters (1 + 2 + 14 bits)
    SeqTsHeader info;
    const uint32_t xdm = static_cast<uint32_t>(std::lround(PosX(v) * 10.0)) & 0x3FFF;
    info.SetSeq((v.up ? 1u : 0u) | ((i % 4) << 1) | (xdm << 3));
    sl->AddHeader(info);
    return sl;
}

double
lanesOf(uint32_t s)
{
    static const double lanes[] = {4.0, 7.5, 12.5, 16.0};
    return lanes[(s >> 1) & 3u] * g_width / 20.0;
}

void
SlRx(uint32_t self, uint32_t from, Ptr<Packet> p)
{
    Ptr<Packet> c = p->Copy();
    SeqTsHeader info;
    c->RemoveHeader(info);
    {
        static const double lanes[] = {4.0, 7.5, 12.5, 16.0};
        const uint32_t s = info.GetSeq();
        g_v[self].nb[from] = {(s & 1u) != 0, ((s >> 3) & 0x3FFF) / 10.0, lanes[(s >> 1) & 3u] * g_width / 20.0,
                              Simulator::Now().GetSeconds()};
    }
    const double fromX = ((info.GetSeq() >> 3) & 0x3FFF) / 10.0;
    const double fromY = lanesOf(info.GetSeq());
    SeqTsHeader tag;
    c->RemoveHeader(tag);
    const uint32_t tg = tag.GetSeq();
    bool named = false;
    if (tg == kAny)
    {
        const double dme = std::hypot(fromX - PosX(g_v[self]), fromY - g_v[self].y);
        if (dme > g_maxRelay)
        {
            return;
        }
        named = true;
        if (g_policy == "anyp" || g_policy == "anypn")
        {
            const double now = Simulator::Now().GetSeconds();
            uint32_t ups = 0;
            for (const auto& [j, n] : g_v[self].nb)
            {
                ups += (n.up || g_policy == "anypn") && now - n.t <= g_nbAge && std::abs(n.x - PosX(g_v[self])) <= g_maxRelay;
            }
            static Ptr<UniformRandomVariable> up = CreateObject<UniformRandomVariable>();
            named = up->GetValue() < std::min(1.0, g_kappa / (1.0 + ups));
        }
    }
    else if (tg == kList)
    {
        const auto& nm = g_named[from];
        named = std::find(nm.begin(), nm.end(), self) != nm.end();
    }
    else if (tg != kNoRelay)
    {
        for (int k = 0; k < 3; ++k)
        {
            const uint32_t id = (tg >> (10 * k)) & 0x3FF;
            named |= id != 0 && id - 1 == self;
        }
    }
    if (!named || !g_v[self].up)
    {
        return;
    }
    // forward the source's message unchanged; its header keeps the source's generation time
    if (g_setupS > 0 && tg != kAny && g_v[from].linked != self)
    {
        g_v[from].linked = self;
        Simulator::Schedule(Seconds(g_setupS), [self, c]() {
            if (g_v[self].up)
            {
                UlSend(self, c, kPort + 1);
                ++g_v[self].relayed;
            }
        });
        return;
    }
    UlSend(self, c, kPort + 1);
    ++g_v[self].relayed;
}

void
ServerRx(uint16_t via, Ptr<Socket> s)
{
    Ptr<Packet> p;
    while ((p = s->Recv()))
    {
        SeqTsHeader h;
        p->RemoveHeader(h);
        g_out << (h.GetSeq() >> 20) << "," << (h.GetSeq() & 0xFFFFF) << "," << h.GetTs().GetSeconds() << ","
              << Simulator::Now().GetSeconds() << "," << via << "\n";
    }
}
} // namespace

int
main(int argc, char* argv[])
{
    std::string tle = "contrib/ntn-v2x/data/starlink_dtc_20261002.tle";
    std::string out = "canyon_relay.csv";
    uint32_t nVeh = 60;
    double simS = 300, h50 = 18, hSigma = 0.5, discMs = 1000, epoch = 1792800000.0, tick = 0.1;
    CommandLine cmd(__FILE__);
    cmd.AddValue("tle", "satellite element sets", tle);
    cmd.AddValue("out", "output CSV", out);
    cmd.AddValue("nVeh", "vehicles on the street", nVeh);
    cmd.AddValue("simSeconds", "simulated time (s)", simS);
    cmd.AddValue("policy", "direct | multisat | relay | blind | blindfar | blindlb | random", g_policy);
    cmd.AddValue("width", "street width wall to wall (m)", g_width);
    cmd.AddValue("length", "street length simulated, wrapping (m)", g_len);
    cmd.AddValue("bldLen", "building length along the street (m)", g_bld);
    cmd.AddValue("h50", "median facade height (m)", h50);
    cmd.AddValue("hSigma", "log-sd of facade heights", hSigma);
    cmd.AddValue("mask", "minimum satellite elevation (deg)", g_mask);
    cmd.AddValue("maxRelayM", "largest sidelink distance for a relay (m)", g_maxRelay);
    std::string traceOut;
    cmd.AddValue("traceOut", "optional CSV of facade heights and per-tick link states of vehicles 0-7", traceOut);
    cmd.AddValue("discMs", "unused (neighbor state now comes from the status broadcasts)", discMs);
    cmd.AddValue("hoMs", "uplink interruption at each change of uplink satellite (ms)", g_hoS);
    uint32_t periodMs = 100;
    cmd.AddValue("periodMs", "status message period = sidelink reservation period (ms)", periodMs);
    cmd.AddValue("ulKbps", "capacity of a satellite's uplink left for this street (kb/s); 0 = no limit", g_ulKbps);
    cmd.AddValue("grantRtt", "round trips from scheduling request to grant", g_grantRtt);
    cmd.AddValue("ulMaxQueueS", "largest wait for an uplink opportunity (s)", g_ulMaxQ);
    double setupMs = 0;
    cmd.AddValue("relaySetupMs", "latency of a link to a relay the source did not name last (ms)", setupMs);
    cmd.AddValue("nbAgeS", "neighbor entries age out after this time (s)", g_nbAge);
    cmd.AddValue("kappa", "anyp: expected number of forwards", g_kappa);
    cmd.AddValue("relayK", "relayk, relayrand, randk: neighbors named per message", g_relayK);
    cmd.AddValue("ulPrio", "own uplink messages before forwards", g_ulPrio);
    cmd.AddValue("vMin", "constant speed range, low (m/s)", g_vMin);
    cmd.AddValue("vMax", "constant speed range, high (m/s)", g_vMax);
    std::string mobTrace, facadeFile;
    cmd.AddValue("mobTrace", "CSV: first line dt, then one line per vehicle: x0,dir,s(0),s(dt),...", mobTrace);
    cmd.AddValue("facadeFile", "CSV: side (0 left, 1 right),x0,x1,height", facadeFile);
    cmd.AddValue("gapEveryM", "a cross street every this many meters (0 = none)", g_gapEvery);
    cmd.AddValue("gapLenM", "width of a cross street (m)", g_gapLen);
    bool slUrban = false, alignResel = false;
    double slNlosP = 0.0;
    cmd.AddValue("slNlosP", "stress: share of vehicle pairs given the NLOS law", slNlosP);
    cmd.AddValue("slUrban", "TR 37.885 urban sidelink channel", slUrban);
    cmd.AddValue("alignResel", "message arrivals on a fixed period with a random phase", alignResel);
    cmd.AddValue("lat", "street latitude", g_lat);
    cmd.AddValue("lon", "street longitude", g_lon);
    cmd.AddValue("streetAzDeg", "street azimuth from north (deg)", g_az);
    cmd.AddValue("gwLat", "gateway latitude", g_gwLat);
    cmd.AddValue("gwLon", "gateway longitude", g_gwLon);
    cmd.AddValue("epochUnix", "Unix time at simulation start", epoch);
    cmd.Parse(argc, argv);
    g_hoS /= 1000.0; // given in ms
    g_h50 = h50;
    g_hSig = hSigma;
    g_setupS = setupMs / 1000.0;
    if (!facadeFile.empty())
    {
        std::ifstream f(facadeFile);
        NS_ABORT_MSG_UNLESS(f.good(), "cannot open " << facadeFile);
        const size_t nm = static_cast<size_t>(std::ceil(g_len));
        g_rasterL.assign(nm, 0.0f);
        g_rasterR.assign(nm, 0.0f);
        std::string line;
        while (std::getline(f, line))
        {
            int side;
            double a, b, hh;
            if (std::sscanf(line.c_str(), "%d,%lf,%lf,%lf", &side, &a, &b, &hh) != 4)
            {
                continue;
            }
            auto& ra = side ? g_rasterR : g_rasterL;
            for (size_t m = static_cast<size_t>(std::max(0.0, a)); m < nm && m < b; ++m)
            {
                ra[m] = static_cast<float>(hh);
            }
        }
    }

    Ptr<UniformRandomVariable> u = CreateObject<UniformRandomVariable>();
    Ptr<NormalRandomVariable> nrm = CreateObject<NormalRandomVariable>();
    const size_t nb = static_cast<size_t>(std::ceil(g_len / g_bld));
    for (size_t k = 0; k < nb; ++k)
    {
        g_hLeft.push_back(h50 * std::exp(hSigma * nrm->GetValue()));
        g_hRight.push_back(h50 * std::exp(hSigma * nrm->GetValue()));
    }
    g_origin = Ecef(g_lat, g_lon);
    const double la = g_lat * M_PI / 180, lo = g_lon * M_PI / 180;
    g_up = Vector(std::cos(la) * std::cos(lo), std::cos(la) * std::sin(lo), std::sin(la));
    g_east = Vector(-std::sin(lo), std::cos(lo), 0);
    g_north = Vector(-std::sin(la) * std::cos(lo), -std::sin(la) * std::sin(lo), std::cos(la));

    {
        std::ifstream f(tle);
        NS_ABORT_MSG_UNLESS(f.good(), "cannot open " << tle);
        std::stringstream ss;
        ss << f.rdbuf();
        std::vector<ntncon::TleRecord> recs;
        ntncon::ParseTleStream(ss.str(), recs);
        for (const auto& r : recs)
        {
            Ptr<ntncon::Sgp4MobilityModel> m = CreateObject<ntncon::Sgp4MobilityModel>();
            if (!m->SetTle(r))
            {
                continue;
            }
            m->SetSimulationEpochUnix(epoch);
            m->SetCacheGranularity(MilliSeconds(100));
            g_sats.push_back(m);
        }
    }
    g_satBusy.assign(g_sats.size(), 0.0);

    NodeContainer vn, srv;
    vn.Create(nVeh);
    srv.Create(1);
    InternetStackHelper inet;
    inet.Install(vn);
    inet.Install(srv);
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("2Mbps"));
    Ipv4AddressHelper ip("10.0.0.0", "255.255.255.252");
    Ptr<NtnSlChannel> ch = CreateObject<NtnSlChannel>();
    ch->SetUrbanChannel(slUrban, slNlosP);
    const double lanes[] = {4.0, 7.5, 12.5, 16.0};
    g_v.resize(nVeh);
    for (uint32_t i = 0; i < nVeh; ++i)
    {
        Veh& v = g_v[i];
        v.x0 = u->GetValue(0, g_len);
        v.y = lanes[i % 4] * g_width / 20.0;
        v.v = u->GetValue(g_vMin, g_vMax) * (v.y < g_width / 2 ? 1 : -1);
        v.mob = CreateObject<ConstantPositionMobilityModel>();
        v.mob->SetPosition(Vector(v.x0, v.y, kAnt));
        vn.Get(i)->AggregateObject(v.mob);
        NetDeviceContainer nd = p2p.Install(vn.Get(i), srv.Get(0));
        v.em = CreateObject<RateErrorModel>();
        v.em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
        v.em->SetRate(1.0);
        nd.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(v.em));
        v.chan = DynamicCast<PointToPointChannel>(nd.Get(0)->GetChannel());
        Ipv4InterfaceContainer ic = ip.Assign(nd);
        ip.NewNetwork();
        if (i == 0)
        {
            g_srvAddr = ic.GetAddress(1);
        }
        // every vehicle reaches the server's first address over its own link
        Ipv4StaticRoutingHelper srh;
        srh.GetStaticRouting(vn.Get(i)->GetObject<Ipv4>())->AddHostRouteTo(g_srvAddr, ic.GetAddress(1), 1);
        v.sock = Socket::CreateSocket(vn.Get(i), UdpSocketFactory::GetTypeId());
        v.sock->Bind();
        v.mac = CreateObject<NtnSlUeMac>();
        v.mac->SetUeId(i);
        v.mac->SetMobility(v.mob);
        v.mac->SetReservationPeriod(periodMs);
        v.mac->SetAlignReselection(alignResel);
        v.mac->SetTxPacketCallback(MakeCallback(&MakeTx));
        v.mac->SetRxCallback(MakeBoundCallback(&SlRx, i));
        ch->AddUe(v.mac);
    }
    if (!mobTrace.empty())
    {
        std::ifstream f(mobTrace);
        NS_ABORT_MSG_UNLESS(f.good(), "cannot open " << mobTrace);
        std::string line;
        std::getline(f, line);
        g_traceDt = std::stod(line);
        for (uint32_t i = 0; i < nVeh; ++i)
        {
            NS_ABORT_MSG_UNLESS(std::getline(f, line), "trace has too few vehicles");
            std::stringstream ls(line);
            std::string tok;
            std::getline(ls, tok, ',');
            g_v[i].x0 = std::stod(tok);
            std::getline(ls, tok, ',');
            g_v[i].dir = std::stoi(tok);
            while (std::getline(ls, tok, ','))
            {
                g_v[i].traceS.push_back(std::stof(tok));
            }
        }
        g_useTrace = true;
    }
    for (uint16_t via = 0; via < 2; ++via)
    {
        Ptr<Socket> s = Socket::CreateSocket(srv.Get(0), UdpSocketFactory::GetTypeId());
        s->Bind(InetSocketAddress(Ipv4Address::GetAny(), kPort + via));
        s->SetRecvCallback(MakeBoundCallback(&ServerRx, via));
    }
    if (!traceOut.empty())
    {
        g_trace.open(traceOut);
        for (size_t k = 0; k < g_hLeft.size(); ++k)
        {
            g_trace << "H," << k << "," << g_hLeft[k] << "," << g_hRight[k] << "\n";
        }
        for (uint32_t i = 0; i < 8 && i < g_v.size(); ++i)
        {
            g_trace << "V," << i << "," << g_v[i].x0 << "," << g_v[i].y << "," << g_v[i].v << "\n";
        }
    }
    g_out.open(out);
    g_out << "src,seq,gen_s,rx_s,via\n";
    Simulator::Schedule(Seconds(0), &Tick, tick);
    ch->Start(Seconds(1.0), Seconds(simS));
    Simulator::Stop(Seconds(simS + 1));
    Simulator::Run();
    uint64_t gen = 0, direct = 0, relayed = 0, asked = 0, blockedGen = 0;
    for (const auto& v : g_v)
    {
        blockedGen += v.blockedGen;
    }
    for (const auto& v : g_v)
    {
        gen += v.gen;
        direct += v.sentDirect;
        relayed += v.relayed;
        asked += v.relayAsked;
    }
    std::cout << "sats=" << g_sats.size() << " vehicles=" << nVeh << " policy=" << g_policy
              << " generated=" << gen << " sent_direct=" << direct << " relay_asked=" << asked << " far_fallback=" << g_farFallback
              << " relayed=" << relayed << " blocked_gen=" << blockedGen << " ul_sent=" << g_ulSent
              << " ul_drop_q=" << g_ulDropQ << " ul_drop_los=" << g_ulDropLos << " ul_bytes=" << g_ulBytes
              << " ul_wait_ms=" << (g_ulSent ? 1e3 * g_ulWaitSum / g_ulSent : 0.0);
    for (double r : {25.0, 50.0, 100.0, 150.0, 200.0, 300.0})
    {
        std::cout << " prr_" << static_cast<int>(r) << "=" << ch->GetPrrWithinRange(r);
        std::cout << " prrnohd_" << static_cast<int>(r) << "=" << ch->GetPrrWithinRangeExcludingHalfDuplex(r);
    }
    std::cout << " out=" << out << "\n";
    g_out.close();
    Simulator::Destroy();
    return 0;
}
