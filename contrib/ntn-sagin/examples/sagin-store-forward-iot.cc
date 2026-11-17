/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Store-and-forward IoT over a sparse LEO fleet (IoT-J study, Section VI-B).
//
// Real orbits: every satellite flies its own TLE through Vallado SGP4 (ntn-constellation), anchored to
// one common epoch. Ground: ocean-buoy devices and gateway sites read from CSV files (lat, lon).
// Data plane: UDP over IPv4 over point-to-point links, one per (device, satellite) and per
// (satellite, gateway) pair, and one per (gateway, server). Every link is gated by visibility: its
// receive error model drops everything while the pair is below the elevation mask, and its channel
// delay follows the slant range. Custody: each device and each satellite runs a SaginCustodyQueue
// (ntn-sagin) behind a UDP relay; the satellite queue's contact is up while it sees any gateway.
// Policies (--policy):
//   bp   bent pipe: a device sends only to a satellite that sees a gateway at that moment, and the
//        satellite forwards immediately (no custody on board)
//   sf1  store and forward to the visible satellite with the highest elevation
//   sfe  store and forward to the satellite with the earliest predicted delivery (contact plan from
//        the ephemeris: next device contact, then that satellite's next gateway contact)
// --keepNewest makes every custody queue keep only the newest update of each device.
// The server writes one line per received update: device, sequence, generation time, reception time.
//
// Wave 2 M3 options (the defaults reproduce the IoT-J output byte for byte):
//   --linkModel=satlink  every device-satellite and satellite-gateway hop gets a ntn-constellation
//        NtnSatLinkErrorModel per direction: per-packet Es/No from the LIVE slant range (FSPL,
//        P.676 gas, P.618 scintillation) through the DVB-S2 QPSK 1/2 waterfall (EN 302 307-1
//        Table 13 QEF threshold 1.0 dB), instead of the 0/1 elevation gate. The geometric gate
//        stays only for line of sight (elevation >= 0). Declared budget, TR 38.821 Table
//        6.1.1.1-1/-2 Set-1 LEO-600: device = handheld-class IoT terminal 23 dBm, 0 dBi, G/T
//        -31.6 dB/K (NF 7 dB), S band 2 GHz; satellite S-band G/T 1.1 dB/K, EIRP density 34 dBW/MHz;
//        feeder Ka 20/30 GHz: satellite EIRP density 4 dBW/MHz, G/T 13 dB/K; gateway = Set-1
//        VSAT (G/T 15.9 dB/K, 33 dBm + 43.2 dBi). The noise bandwidth is the symbol rate the hop's
//        P2P DataRate needs at QPSK 1/2 (0.988 b/symbol), so the radio carries the rate the link
//        claims. The orbits are the ORBCOMM TLEs; ORBCOMM's own VHF air interface is NOT modeled.
//   --contacts=cgr  contact windows come from ntn-constellation ContactGraphScheduler (one per
//        mask, satellites and RegisterGroundStation sites on the same SGP4 models) and a
//        ContactGraphRouter attached to the gateway scheduler picks the satellite's gateway,
//        instead of the example's own elevation test. --policy=sfe still plans with the
//        example's ephemeris plan (the scheduler predicts contact ends, not future starts).
//   --dtn=bpv7  every update is a BPv7 bundle (RFC 9171 primary block, CBOR, sagin-bundle-
//        protocol.h) with lifetime = --lifetimeH from its creation timestamp (RFC 9171 4.2.2),
//        and custody transfer per RFC 5050 5.10 as an extension: a node keeps a forwarded
//        bundle until the next hop's custody signal (sent back over the same radio hop, so it
//        can be lost too), retransmits after --custodyTimeoutS, refuses on depleted storage
//        (reason 4) and answers duplicates with "redundant reception" (reason 3). The server
//        delivers each bundle once (RFC 9171 5.7) and counts duplicates. Not implemented: CRCs,
//        previous-node and hop-count blocks, fragmentation, BPSec, status reports on the wire
//        (counted only), LTP/TCPCL convergence layers.
//
// Wave 6B options:
//   --bpv7Expiry=bundle (default with --dtn=bpv7)  RFC 9171 5.5 expiry from each bundle's own
//        primary block: deleted on arrival if expired, never forwarded once expired, deleted from
//        storage AT its deadline by a timer, and also checked at the gateway and the server.
//        --bpv7Expiry=queue reproduces the earlier queue-level lifetime.
//   --clockless=1  bundles carry creation time 0 and a Bundle Age block (RFC 9171 4.2.7, 4.4.2);
//        every custodian adds its dwell time to the age when it forwards.
//   --policy=sabr  CCSDS 734.3-B-1 SABR contact graph routing (sagin-sabr-router.h) on every
//        device and satellite, over a contact plan derived from the SGP4 ephemeris at the tick
//        resolution (device->satellite, satellite->gateway, gateway->server contacts). A bundle
//        is sent only when its best candidate route's first contact is open; routes whose
//        projected arrival is after the bundle's expiration are never used (3.2.6.9 a, e).
//        Requires --dtn=bpv7.

#include "ns3/applications-module.h"
#include "ns3/contact-graph-router.h"
#include "ns3/contact-graph-scheduler.h"
#include "ns3/ntn-sat-link-error-model.h"
#include "ns3/sagin-bundle-protocol.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/orbital-elements.h"
#include "ns3/point-to-point-module.h"
#include "ns3/sagin-custody-queue.h"
#include "ns3/sagin-sabr-router.h"
#include "ns3/sgp4-mobility-model.h"

#include <cmath>
#include <iomanip>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

using namespace ns3;
using ns3::ntnsagin::SaginBundleHeader;
using ns3::ntnsagin::SaginBundleTrailer;
using ns3::ntnsagin::SaginCustodyQueue;
using ns3::ntnsagin::SaginCustodySignalHeader;

NS_LOG_COMPONENT_DEFINE("SaginStoreForwardIot");

namespace
{
constexpr double kC = 299792458.0;
constexpr double kA = 6378137.0;
constexpr double kF = 1.0 / 298.257223563;
constexpr uint16_t kPort = 9000;
constexpr uint16_t kSigPort = 9002; // custody signals (--dtn=bpv7)

struct Site
{
    std::string name;
    Vector ecef;
    double latDeg{0.0};
    double lonDeg{0.0};
};

/// --linkModel=satlink: line-of-sight gate (elevation >= 0) in front of the graded
/// NtnSatLinkErrorModel, with per-hop counters.
class GatedSatLinkErrorModel : public ErrorModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("GatedSatLinkErrorModel").SetParent<ErrorModel>();
        return tid;
    }
    bool los{false};
    Ptr<ntncon::NtnSatLinkErrorModel> inner;
    uint64_t* rx{nullptr};
    uint64_t* lost{nullptr};
    double* esnoSum{nullptr};

  private:
    bool DoCorrupt(Ptr<Packet> p) override
    {
        ++*rx;
        bool bad = true;
        if (los)
        {
            bad = inner->IsCorrupt(p);
            *esnoSum += inner->LastEsNoDb();
        }
        *lost += bad ? 1 : 0;
        return bad;
    }
    void DoReset() override
    {
    }
};

struct Link
{
    Ptr<RateErrorModel> emA, emB;
    Ptr<GatedSatLinkErrorModel> slA, slB; // --linkModel=satlink only
    Ptr<PointToPointChannel> chan;
    Ipv4Address addrA, addrB; // A = ground (device or gateway) side, B = satellite side
    bool up{false};
};

Vector
Geodetic(double latDeg, double lonDeg)
{
    const double lat = latDeg * M_PI / 180.0;
    const double lon = lonDeg * M_PI / 180.0;
    const double e2 = kF * (2 - kF);
    const double n = kA / std::sqrt(1 - e2 * std::sin(lat) * std::sin(lat));
    return Vector(n * std::cos(lat) * std::cos(lon), n * std::cos(lat) * std::sin(lon),
                  n * (1 - e2) * std::sin(lat));
}

double
SinElevation(const Vector& g, const Vector& s)
{
    const double gn = g.GetLength();
    const Vector l = s - g;
    return (g.x * l.x + g.y * l.y + g.z * l.z) / (gn * l.GetLength());
}

std::vector<Site>
ReadSites(const std::string& path, uint32_t maxN)
{
    std::vector<Site> out;
    std::ifstream f(path);
    NS_ABORT_MSG_UNLESS(f.good(), "cannot open " << path);
    std::string line;
    std::getline(f, line); // header: name,lat,lon
    while (std::getline(f, line) && out.size() < maxN)
    {
        std::stringstream ss(line);
        std::string name, la, lo;
        std::getline(ss, name, ',');
        std::getline(ss, la, ',');
        std::getline(ss, lo, ',');
        out.push_back({name, Geodetic(std::stod(la), std::stod(lo)), std::stod(la), std::stod(lo)});
    }
    return out;
}

// ---------------------------------------------------------------- global scenario state
std::vector<Ptr<ntncon::Sgp4MobilityModel>> g_sats;
std::vector<Site> g_dev, g_gw;
std::vector<std::vector<Link>> g_ds; // [device][sat]
std::vector<std::vector<Link>> g_sg; // [sat][gateway]
std::vector<Ptr<SaginCustodyQueue>> g_devQ, g_satQ;
std::vector<Ptr<Socket>> g_devSock, g_satSock, g_gwSock;
std::vector<Ipv4Address> g_gwServerAddr;
std::vector<std::vector<double>> g_sinDs; // current sin(elevation) device-sat
std::vector<std::vector<bool>> g_visSg;   // current visibility sat-gateway
std::string g_policy = "sf1";
// Per-contact budgets (packets), 0 = unlimited: how many updates a device can upload while it stays in
// view of the fleet, and how many a satellite can deliver during one gateway contact. They stand for
// the share of a narrowband uplink or feeder pass one device or satellite gets.
uint32_t g_upBudget = 0;
uint32_t g_feedBudget = 0;
std::vector<uint32_t> g_devUsed;
std::vector<uint32_t> g_satUsed;
std::vector<bool> g_devWasUp;
std::vector<bool> g_satWasUp;
uint64_t g_generated = 0;
double g_sinDevMask, g_sinGwMask;
double g_tick = 10.0;
// contact plan for earliest-delivery selection: per (dev,sat) and per sat, sorted visible ticks
std::vector<std::vector<std::vector<uint32_t>>> g_planDs;
std::vector<std::vector<uint32_t>> g_planSg;
uint32_t g_planTicks = 0;
std::ofstream g_out;
double g_genStop = 0.0;

// ---- Wave 2 M3 options ----
bool g_satlink = false;
bool g_cgr = false;
bool g_bpv7 = false;
std::vector<std::vector<bool>> g_cgrDs; // contact state device-sat from the scheduler
std::vector<std::vector<bool>> g_cgrSg; // contact state sat-gateway from the scheduler
Ptr<ntncon::ContactGraphRouter> g_router;
constexpr uint32_t kGwIdBase = 1001;
constexpr uint32_t kDevIdBase = 2001;
uint64_t g_routerGwPicks = 0;
// per-hop radio counters: 0 device->sat, 1 sat->device (signals), 2 sat->gw, 3 gw->sat (signals)
uint64_t g_hopRx[4] = {0, 0, 0, 0};
uint64_t g_hopLost[4] = {0, 0, 0, 0};
double g_hopEsno[4] = {0, 0, 0, 0};
// BPv7 bookkeeping
double g_epochUnix = 0.0;
uint64_t g_lifetimeMs = 0;
std::vector<std::set<uint64_t>> g_satSeen; // bundles each satellite took custody of
std::set<uint64_t> g_srvSeen;
uint64_t g_delivered = 0;
uint64_t g_dupAtServer = 0;
uint64_t g_sigSentOk = 0;
uint64_t g_sigSentRefused = 0;
uint64_t g_sigSentRedundant = 0;
uint64_t g_sigRecv = 0;
std::vector<Ptr<Socket>> g_devSigSock, g_satSigSock;
// ---- Wave 6B ----
bool g_bundleExpiry = false; // RFC 9171 5.5 per-bundle expiry
bool g_clockless = false;    // creation time 0 + Bundle Age block
bool g_sabr = false;
constexpr uint32_t kServerId = 9000;
std::vector<Ptr<ntnsagin::SaginSabrRouter>> g_devSabr, g_satSabr;
uint64_t g_sabrFwd = 0, g_sabrHold = 0, g_sabrNoRoute = 0;
uint64_t g_expiredAtGw = 0, g_expiredAtSrv = 0;
SaginCustodyQueue::ExpiryFunction g_expiryFn;

bool
DevSatVisible(uint32_t d, uint32_t k)
{
    return g_cgr ? g_cgrDs[d][k] : g_sinDs[d][k] >= g_sinDevMask;
}

uint64_t
BundleIdOf(Ptr<const Packet> p)
{
    SaginBundleHeader h;
    p->PeekHeader(h);
    return h.BundleId();
}

uint64_t
BundleSourceKey(Ptr<const Packet> p)
{
    SaginBundleHeader h;
    p->PeekHeader(h);
    return h.srcNode;
}

Time
BundleCreation(Ptr<const Packet> p)
{
    SaginBundleHeader h;
    p->PeekHeader(h);
    return MilliSeconds(static_cast<int64_t>(h.creationMs) -
                        static_cast<int64_t>((g_epochUnix - ntnsagin::kDtnEpochUnixS) * 1000.0));
}

void
SendCustodySignal(Ptr<Socket> sock, const SaginBundleHeader& b, Address to, bool ok, uint8_t reason)
{
    SaginCustodySignalHeader sig;
    sig.succeeded = ok;
    sig.reason = reason;
    sig.srcNode = b.srcNode;
    sig.srcService = b.srcService;
    sig.creationMs = b.creationMs;
    sig.creationSeq = b.creationSeq;
    Ptr<Packet> p = Create<Packet>();
    p->AddHeader(sig);
    const Ipv4Address ip = InetSocketAddress::ConvertFrom(to).GetIpv4();
    sock->SendTo(p, 0, InetSocketAddress(ip, kSigPort));
    if (!ok)
    {
        ++g_sigSentRefused;
    }
    else if (reason == ntnsagin::kCustodyReasonRedundant)
    {
        ++g_sigSentRedundant;
    }
    else
    {
        ++g_sigSentOk;
    }
}

/// RFC 9171 5.5 deadline (simulation seconds) of a bundle that arrived at \p arrival.
double
BundleDeadlineS(Ptr<const Packet> p, Time arrival)
{
    const auto e = g_expiryFn(p, arrival);
    return e.wellFormed ? e.deadline.GetSeconds() : -1.0;
}

void
SetLink(Link& l, bool up, double rangeM, bool los = false)
{
    if (g_satlink)
    {
        // Routing still uses the masks (l.up); the radio passes everything in line of sight
        // through the graded error model.
        l.up = up;
        l.slA->los = los;
        l.slB->los = los;
        l.chan->SetAttribute("Delay", TimeValue(Seconds(rangeM / kC)));
        return;
    }
    l.up = up;
    const double rate = up ? 0.0 : 1.0;
    l.emA->SetRate(rate);
    l.emB->SetRate(rate);
    l.chan->SetAttribute("Delay", TimeValue(Seconds(rangeM / kC)));
}

/// First plan tick >= t at which the list contains a visible tick (lists are sorted).
uint32_t
NextVisible(const std::vector<uint32_t>& v, uint32_t t)
{
    auto it = std::lower_bound(v.begin(), v.end(), t);
    return it == v.end() ? UINT32_MAX : *it;
}

int
ChooseSatellite(uint32_t d)
{
    const uint32_t S = g_sats.size();
    if (g_policy == "sfe")
    {
        const uint32_t now = static_cast<uint32_t>(Simulator::Now().GetSeconds() / g_tick);
        uint32_t best = UINT32_MAX;
        int bestK = -1;
        for (uint32_t k = 0; k < S; ++k)
        {
            if (!DevSatVisible(d, k))
            {
                continue; // only satellites visible now can take the packet now
            }
            const uint32_t del = NextVisible(g_planSg[k], now);
            if (del < best)
            {
                best = del;
                bestK = k;
            }
        }
        if (bestK < 0)
        {
            return -1;
        }
        // hold the packet if a satellite that is not yet visible would deliver earlier
        for (uint32_t k = 0; k < S; ++k)
        {
            const uint32_t up = NextVisible(g_planDs[d][k], now);
            if (up == UINT32_MAX || up == now)
            {
                continue;
            }
            const uint32_t del = NextVisible(g_planSg[k], up);
            if (del < best)
            {
                return -1;
            }
        }
        return bestK;
    }
    int bestK = -1;
    double bestE = -2.0;
    for (uint32_t k = 0; k < S; ++k)
    {
        if (!DevSatVisible(d, k))
        {
            continue;
        }
        if (g_policy == "bp")
        {
            bool gw = false;
            for (uint32_t g = 0; g < g_gw.size(); ++g)
            {
                gw = gw || g_visSg[k][g];
            }
            if (!gw)
            {
                continue;
            }
        }
        if (g_sinDs[d][k] > bestE)
        {
            bestE = g_sinDs[d][k];
            bestK = k;
        }
    }
    return bestK;
}

bool
DeviceForward(uint32_t d, Ptr<Packet> p)
{
    if (g_sabr)
    {
        // CCSDS 734.3-B-1: the best candidate route decides the entry satellite; the bundle
        // goes now only if that route's first contact is open, otherwise it stays in custody.
        const double now = Simulator::Now().GetSeconds();
        const double evc = p->GetSize() * 8.0;
        auto dec = g_devSabr[d]->Route(kServerId, now, evc, BundleDeadlineS(p, Simulator::Now()),
                                       {}, true, false);
        if (!dec.found)
        {
            ++g_sabrNoRoute;
            return false;
        }
        const uint32_t k = dec.entryNode - 1;
        const auto& c0 = g_devSabr[d]->GetContactPlan()[dec.route.contacts.front()];
        if (now < c0.startS || !DevSatVisible(d, k) ||
            (g_upBudget > 0 && g_devUsed[d] >= g_upBudget))
        {
            ++g_sabrHold;
            return false;
        }
        const bool ok =
            g_devSock[d]->SendTo(p->Copy(), 0, InetSocketAddress(g_ds[d][k].addrB, kPort)) >= 0;
        if (ok)
        {
            g_devSabr[d]->Reserve(dec.route, evc);
            ++g_sabrFwd;
            g_devUsed[d] += 1;
        }
        return ok;
    }
    const int k = ChooseSatellite(d);
    if (k < 0 || (g_upBudget > 0 && g_devUsed[d] >= g_upBudget))
    {
        return false;
    }
    const bool ok = g_devSock[d]->SendTo(p->Copy(), 0, InetSocketAddress(g_ds[d][k].addrB, kPort)) >= 0;
    g_devUsed[d] += ok ? 1 : 0;
    return ok;
}

bool
SatelliteForward(uint32_t k, Ptr<Packet> p)
{
    if (g_feedBudget > 0 && g_satUsed[k] >= g_feedBudget)
    {
        return false;
    }
    if (g_sabr)
    {
        const double now = Simulator::Now().GetSeconds();
        const double evc = p->GetSize() * 8.0;
        auto dec = g_satSabr[k]->Route(kServerId, now, evc, BundleDeadlineS(p, Simulator::Now()),
                                       {}, true, false);
        if (!dec.found)
        {
            ++g_sabrNoRoute;
            return false;
        }
        const uint32_t g = dec.entryNode - kGwIdBase;
        const auto& c0 = g_satSabr[k]->GetContactPlan()[dec.route.contacts.front()];
        if (g >= g_gw.size() || now < c0.startS || !g_visSg[k][g])
        {
            ++g_sabrHold;
            return false;
        }
        const bool ok =
            g_satSock[k]->SendTo(p->Copy(), 0, InetSocketAddress(g_sg[k][g].addrA, kPort)) >= 0;
        if (ok)
        {
            g_satSabr[k]->Reserve(dec.route, evc);
            ++g_sabrFwd;
            g_satUsed[k] += 1;
        }
        return ok;
    }
    if (g_cgr)
    {
        // ContactGraphRouter: the gateways adjacent to this satellite in the live contact graph
        // (lowest gateway index first, the same tie rule as the elevation mode).
        const auto nb = g_router->Neighbours(k + 1);
        for (uint32_t id : nb)
        {
            if (id < kGwIdBase || id >= kGwIdBase + g_gw.size())
            {
                continue;
            }
            const uint32_t g = id - kGwIdBase;
            ++g_routerGwPicks;
            const bool ok = g_satSock[k]->SendTo(p->Copy(), 0, InetSocketAddress(g_sg[k][g].addrA, kPort)) >= 0;
            g_satUsed[k] += ok ? 1 : 0;
            return ok;
        }
        return false;
    }
    for (uint32_t g = 0; g < g_gw.size(); ++g)
    {
        if (g_visSg[k][g])
        {
            const bool ok = g_satSock[k]->SendTo(p->Copy(), 0, InetSocketAddress(g_sg[k][g].addrA, kPort)) >= 0;
            g_satUsed[k] += ok ? 1 : 0;
            return ok;
        }
    }
    return false;
}

void
SatelliteReceive(uint32_t k, Ptr<Socket> s)
{
    Ptr<Packet> p;
    Address from;
    while ((p = s->RecvFrom(from)))
    {
        if (!g_bpv7)
        {
            g_satQ[k]->Offer(p);
            continue;
        }
        // RFC 5050 5.10: take custody (or refuse) and tell the previous custodian.
        SaginBundleHeader b;
        p->PeekHeader(b);
        const uint64_t id = b.BundleId();
        if (g_satSeen[k].count(id))
        {
            SendCustodySignal(g_satSigSock[k], b, from, true, ntnsagin::kCustodyReasonRedundant);
            continue;
        }
        const uint64_t expiredBefore = g_satQ[k]->GetExpiredOnArrival() + g_satQ[k]->GetMalformed();
        if (g_satQ[k]->Offer(p))
        {
            g_satSeen[k].insert(id);
            SendCustodySignal(g_satSigSock[k], b, from, true, ntnsagin::kCustodyReasonNone);
        }
        else if (g_satQ[k]->GetExpiredOnArrival() + g_satQ[k]->GetMalformed() > expiredBefore)
        {
            // RFC 9171 5.5: deleted on arrival as expired. The upstream copy has the same
            // absolute deadline, so its own expiry timer has already deleted it.
        }
        else
        {
            SendCustodySignal(g_satSigSock[k], b, from, false, ntnsagin::kCustodyReasonDepletedStorage);
        }
    }
}

void
GatewayReceive(uint32_t g, Ptr<Socket> s)
{
    Ptr<Packet> p;
    Address from;
    while ((p = s->RecvFrom(from)))
    {
        if (g_bpv7 && g_bundleExpiry && Simulator::Now().GetSeconds() >= BundleDeadlineS(p, Simulator::Now()))
        {
            ++g_expiredAtGw; // RFC 9171 5.5: expired, deleted, not handed on
            continue;
        }
        if (g_bpv7)
        {
            // The gateway is the last custodian before the server's wired link: it accepts
            // custody (signal back over the feeder) and hands the bundle on.
            SaginBundleHeader b;
            p->PeekHeader(b);
            SendCustodySignal(g_gwSock[g], b, from, true, ntnsagin::kCustodyReasonNone);
        }
        g_gwSock[g]->SendTo(p, 0, InetSocketAddress(g_gwServerAddr[g], kPort + 1));
    }
}

void
CustodySignalReceive(Ptr<SaginCustodyQueue> q, Ptr<Socket> s)
{
    Ptr<Packet> p;
    while ((p = s->Recv()))
    {
        SaginCustodySignalHeader sig;
        p->RemoveHeader(sig);
        ++g_sigRecv;
        if (sig.succeeded)
        {
            q->AcceptCustody(sig.BundleId());
        }
        else
        {
            q->RefuseCustody(sig.BundleId());
        }
    }
}

void
ServerReceive(Ptr<Socket> s)
{
    Ptr<Packet> p;
    while ((p = s->Recv()))
    {
        if (g_bpv7 && g_bundleExpiry && Simulator::Now().GetSeconds() >= BundleDeadlineS(p, Simulator::Now()))
        {
            ++g_expiredAtSrv; // RFC 9171 5.5: expired before delivery
            continue;
        }
        if (g_bpv7)
        {
            SaginBundleHeader b;
            p->RemoveHeader(b);
            SaginBundleTrailer t;
            p->RemoveTrailer(t);
            if (!g_srvSeen.insert(b.BundleId()).second)
            {
                ++g_dupAtServer; // RFC 9171 5.7: deliver once
                continue;
            }
        }
        ++g_delivered;
        SeqTsHeader h;
        p->RemoveHeader(h);
        const uint32_t seq = h.GetSeq();
        g_out << (seq >> 20) << "," << (seq & 0xFFFFF) << "," << h.GetTs().GetSeconds() << ","
              << Simulator::Now().GetSeconds() << "\n";
    }
}

Time
GenTime(Ptr<const Packet> p)
{
    SeqTsHeader h;
    p->PeekHeader(h);
    return h.GetTs();
}

uint64_t
SourceKey(Ptr<const Packet> p)
{
    SeqTsHeader h;
    p->PeekHeader(h);
    return h.GetSeq() >> 20;
}

void
Generate(uint32_t d, uint32_t seq, Time period)
{
    Ptr<Packet> p = Create<Packet>(40); // status report payload
    SeqTsHeader h;                      // timestamp = generation time
    h.SetSeq((d << 20) | (seq & 0xFFFFF));
    p->AddHeader(h);
    if (g_bpv7)
    {
        SaginBundleHeader b;
        b.srcNode = d + 1; // ipn:<device+1>.1
        b.srcService = 1;
        b.destNode = 9000; // ipn:9000.1 = the server
        b.destService = 1;
        b.reportNode = d + 1;
        b.reportService = 0;
        b.creationMs = static_cast<uint64_t>((g_epochUnix - ntnsagin::kDtnEpochUnixS) * 1000.0) +
                       static_cast<uint64_t>(Simulator::Now().GetMilliSeconds());
        if (g_clockless)
        {
            // RFC 9171 4.2.7: no accurate clock, so creation time 0 and a Bundle Age block.
            b.creationMs = 0;
            b.hasAgeBlock = true;
            b.ageMs = 0;
        }
        b.creationSeq = seq;
        b.lifetimeMs = g_lifetimeMs;
        b.payloadLength = p->GetSize();
        p->AddHeader(b);
        p->AddTrailer(SaginBundleTrailer());
    }
    ++g_generated;
    g_devQ[d]->Offer(p);
    if (Simulator::Now() + period < Seconds(g_genStop))
    {
        Simulator::Schedule(period, &Generate, d, seq + 1, period);
    }
}

void
Tick()
{
    const uint32_t S = g_sats.size();
    std::vector<Vector> pos(S);
    for (uint32_t k = 0; k < S; ++k)
    {
        pos[k] = g_sats[k]->GetPosition();
    }
    if (g_bpv7)
    {
        for (auto& q : g_devQ)
        {
            q->CheckCustodyTimeouts();
        }
        for (auto& q : g_satQ)
        {
            q->CheckCustodyTimeouts();
        }
    }
    for (uint32_t k = 0; k < S; ++k)
    {
        bool any = false;
        for (uint32_t g = 0; g < g_gw.size(); ++g)
        {
            const double se = SinElevation(g_gw[g].ecef, pos[k]);
            const bool v = g_cgr ? g_cgrSg[k][g] : se >= g_sinGwMask;
            g_visSg[k][g] = v;
            any = any || v;
            SetLink(g_sg[k][g], v, CalculateDistance(g_gw[g].ecef, pos[k]), se >= 0.0);
        }
        if (any && !g_satWasUp[k])
        {
            g_satUsed[k] = 0; // a new gateway contact starts with a fresh budget
        }
        g_satWasUp[k] = any;
        g_satQ[k]->SetContactUp(any);
    }
    for (uint32_t d = 0; d < g_dev.size(); ++d)
    {
        bool any = false;
        for (uint32_t k = 0; k < S; ++k)
        {
            const double se = SinElevation(g_dev[d].ecef, pos[k]);
            g_sinDs[d][k] = se;
            const bool v = DevSatVisible(d, k);
            any = any || v;
            SetLink(g_ds[d][k], v, CalculateDistance(g_dev[d].ecef, pos[k]), se >= 0.0);
        }
        if (any && !g_devWasUp[d])
        {
            g_devUsed[d] = 0; // a new contact with the fleet starts with a fresh budget
        }
        g_devWasUp[d] = any;
        // re-offer the backlog: the device's contact is up whenever some satellite is usable
        g_devQ[d]->SetContactUp(false);
        g_devQ[d]->SetContactUp(any && ChooseSatellite(d) >= 0);
    }
    Simulator::Schedule(Seconds(g_tick), &Tick);
}

void
BuildPlan(double horizonS)
{
    const uint32_t S = g_sats.size();
    g_planTicks = static_cast<uint32_t>(horizonS / g_tick) + 1;
    g_planDs.assign(g_dev.size(), std::vector<std::vector<uint32_t>>(S));
    g_planSg.assign(S, {});
    for (uint32_t t = 0; t < g_planTicks; ++t)
    {
        for (uint32_t k = 0; k < S; ++k)
        {
            Vector p, v;
            g_sats[k]->GetEcefStateAt(Seconds(t * g_tick), p, v);
            bool gw = false;
            for (const auto& g : g_gw)
            {
                gw = gw || SinElevation(g.ecef, p) >= g_sinGwMask;
            }
            if (gw)
            {
                g_planSg[k].push_back(t);
            }
            for (uint32_t d = 0; d < g_dev.size(); ++d)
            {
                if (SinElevation(g_dev[d].ecef, p) >= g_sinDevMask)
                {
                    g_planDs[d][k].push_back(t);
                }
            }
        }
    }
}
/// Wave 6B: the SABR contact plan from the SGP4 ephemeris at the tick resolution.
std::vector<ntnsagin::SabrContact>
BuildSabrPlan(double horizonS)
{
    const uint32_t S = g_sats.size();
    const uint32_t D = g_dev.size();
    const uint32_t G = g_gw.size();
    const uint32_t T = static_cast<uint32_t>(horizonS / g_tick) + 1;
    std::vector<ntnsagin::SabrContact> plan;
    // open run start per (d,k) and (k,g); -1 = no run
    std::vector<std::vector<int64_t>> runDs(D, std::vector<int64_t>(S, -1));
    std::vector<std::vector<int64_t>> runSg(S, std::vector<int64_t>(G, -1));
    auto close = [&](uint32_t from, uint32_t to, int64_t t0, uint32_t t1, const Vector& a,
                     uint32_t sat) {
        ntnsagin::SabrContact c;
        c.from = from;
        c.to = to;
        c.startS = t0 * g_tick;
        c.endS = t1 * g_tick;
        c.rateBps = 1e6; // the access P2P DataRate
        Vector p, v;
        g_sats[sat]->GetEcefStateAt(Seconds(0.5 * (c.startS + c.endS)), p, v);
        c.owltS = CalculateDistance(a, p) / kC; // range at mid-contact
        plan.push_back(c);
    };
    for (uint32_t t = 0; t <= T; ++t)
    {
        for (uint32_t k = 0; k < S; ++k)
        {
            Vector p, v;
            g_sats[k]->GetEcefStateAt(Seconds(t * g_tick), p, v);
            for (uint32_t d = 0; d < D; ++d)
            {
                const bool vis = t < T && SinElevation(g_dev[d].ecef, p) >= g_sinDevMask;
                if (vis && runDs[d][k] < 0)
                {
                    runDs[d][k] = t;
                }
                else if (!vis && runDs[d][k] >= 0)
                {
                    close(kDevIdBase + d, k + 1, runDs[d][k], t, g_dev[d].ecef, k);
                    runDs[d][k] = -1;
                }
            }
            for (uint32_t g = 0; g < G; ++g)
            {
                const bool vis = t < T && SinElevation(g_gw[g].ecef, p) >= g_sinGwMask;
                if (vis && runSg[k][g] < 0)
                {
                    runSg[k][g] = t;
                }
                else if (!vis && runSg[k][g] >= 0)
                {
                    close(k + 1, kGwIdBase + g, runSg[k][g], t, g_gw[g].ecef, k);
                    runSg[k][g] = -1;
                }
            }
        }
    }
    for (uint32_t g = 0; g < G; ++g)
    {
        // the wired gateway -> server link is always up (100 Mb/s, 5 ms)
        plan.push_back({kGwIdBase + g, kServerId, 0.0, horizonS + g_tick, 100e6, 0.005});
    }
    return plan;
}
} // namespace

int
main(int argc, char* argv[])
{
    std::string tleFile = "contrib/ntn-sagin/data/orbcomm.tle";
    std::string devFile = "contrib/ntn-sagin/data/buoys.csv";
    std::string gwFile = "contrib/ntn-sagin/data/gateways_k5.csv";
    uint32_t nDev = 40;
    double days = 2.0;
    double periodMin = 15.0;
    double devMaskDeg = 20.0;
    double gwMaskDeg = 10.0;
    double lifetimeH = 0.0;
    uint64_t satBufferBytes = 1 << 30;
    bool keepNewest = false;
    double epochUnix = 1793318400.0; // 2026-10-30 00:00 UTC
    std::string out = "sf_iot.csv";

    CommandLine cmd(__FILE__);
    cmd.AddValue("tle", "TLE file of the fleet", tleFile);
    cmd.AddValue("devices", "device CSV (name,lat,lon)", devFile);
    cmd.AddValue("gateways", "gateway CSV (name,lat,lon)", gwFile);
    cmd.AddValue("nDev", "number of devices read from the CSV", nDev);
    cmd.AddValue("days", "simulated days", days);
    cmd.AddValue("periodMin", "status update period (min)", periodMin);
    cmd.AddValue("devMask", "device elevation mask (deg)", devMaskDeg);
    cmd.AddValue("gwMask", "gateway elevation mask (deg)", gwMaskDeg);
    cmd.AddValue("policy", "bp | sf1 | sfe | sabr (CCSDS 734.3-B-1, needs --dtn=bpv7)", g_policy);
    cmd.AddValue("lifetimeH", "custody lifetime per hop (h), 0 = none", lifetimeH);
    cmd.AddValue("satBufferBytes", "on-board custody capacity (bytes)", satBufferBytes);
    cmd.AddValue("keepNewest", "keep only the newest update of each device in custody", keepNewest);
    cmd.AddValue("tick", "visibility update period (s)", g_tick);
    bool lcfs = false;
    cmd.AddValue("lcfs", "drain custody newest first (last come, first served)", lcfs);
    bool lifetimeE2E = false;
    cmd.AddValue("lifetimeE2E", "lifetime counted from generation, not from entry into each hop", lifetimeE2E);
    cmd.AddValue("upBudget", "updates a device may upload per contact with the fleet, 0 = unlimited", g_upBudget);
    cmd.AddValue("feedBudget", "updates a satellite may deliver per gateway contact, 0 = unlimited", g_feedBudget);
    cmd.AddValue("out", "output CSV", out);
    std::string linkModel = "gate";
    std::string contacts = "elevation";
    std::string dtn = "none";
    double custodyTimeoutS = 30.0;
    cmd.AddValue("linkModel", "gate (0/1 elevation gate, IoT-J default) | satlink (NtnSatLinkErrorModel)",
                 linkModel);
    cmd.AddValue("contacts", "elevation (example's own test, default) | cgr (ContactGraphScheduler/Router)",
                 contacts);
    cmd.AddValue("dtn", "none (default) | bpv7 (RFC 9171 bundles + RFC 5050 custody transfer)", dtn);
    cmd.AddValue("custodyTimeoutS", "custody retransmission timer (s), --dtn=bpv7", custodyTimeoutS);
    std::string bpv7Expiry = "bundle";
    cmd.AddValue("bpv7Expiry",
                 "bundle (RFC 9171 5.5 per-bundle expiry, default) | queue (pre-Wave-6B queue lifetime)",
                 bpv7Expiry);
    cmd.AddValue("clockless", "bundles with creation time 0 and a Bundle Age block (RFC 9171 4.2.7)",
                 g_clockless);
    cmd.Parse(argc, argv);
    NS_ABORT_MSG_UNLESS(linkModel == "gate" || linkModel == "satlink", "--linkModel=gate|satlink");
    NS_ABORT_MSG_UNLESS(contacts == "elevation" || contacts == "cgr", "--contacts=elevation|cgr");
    NS_ABORT_MSG_UNLESS(dtn == "none" || dtn == "bpv7", "--dtn=none|bpv7");
    g_satlink = linkModel == "satlink";
    g_cgr = contacts == "cgr";
    g_bpv7 = dtn == "bpv7";
    NS_ABORT_MSG_UNLESS(bpv7Expiry == "bundle" || bpv7Expiry == "queue", "--bpv7Expiry=bundle|queue");
    g_bundleExpiry = g_bpv7 && bpv7Expiry == "bundle";
    g_sabr = g_policy == "sabr";
    NS_ABORT_MSG_IF(g_sabr && !g_bpv7, "--policy=sabr routes bundles: it needs --dtn=bpv7");
    NS_ABORT_MSG_IF(g_clockless && !g_bundleExpiry,
                    "--clockless needs --dtn=bpv7 --bpv7Expiry=bundle (age-block expiry)");
    g_epochUnix = epochUnix;
    g_expiryFn = SaginCustodyQueue::Bpv7PrimaryBlockExpiry(epochUnix);
    g_lifetimeMs = static_cast<uint64_t>((lifetimeH > 0 ? lifetimeH * 3600.0 : days * 86400.0) * 1000.0);
    g_sinDevMask = std::sin(devMaskDeg * M_PI / 180.0);
    g_sinGwMask = std::sin(gwMaskDeg * M_PI / 180.0);

    // fleet
    {
        std::ifstream f(tleFile);
        NS_ABORT_MSG_UNLESS(f.good(), "cannot open " << tleFile);
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
            m->SetSimulationEpochUnix(epochUnix);
            m->SetCacheGranularity(Seconds(1));
            g_sats.push_back(m);
        }
    }
    g_dev = ReadSites(devFile, nDev);
    g_gw = ReadSites(gwFile, 1000);
    const uint32_t S = g_sats.size(), D = g_dev.size(), G = g_gw.size();
    NS_ABORT_MSG_UNLESS(S > 0 && D > 0 && G > 0, "empty scenario");

    NodeContainer devN, satN, gwN, srvN;
    devN.Create(D);
    satN.Create(S);
    gwN.Create(G);
    srvN.Create(1);
    InternetStackHelper inet;
    inet.Install(devN);
    inet.Install(satN);
    inet.Install(gwN);
    inet.Install(srvN);

    // --linkModel=satlink: ground endpoints as ECEF mobility for the live slant range.
    std::vector<Ptr<MobilityModel>> devMob, gwMob;
    const double accessRateBps = 1e6;           // the hop's P2P DataRate below
    const double qpskHalfBitsPerSymbol = 0.988; // DVB-S2 QPSK 1/2 (EN 302 307-1 Table 13)
    const double noiseBwHz = accessRateBps / qpskHalfBitsPerSymbol;
    const double bwMhzDb = 10.0 * std::log10(noiseBwHz / 1e6);
    if (g_satlink)
    {
        for (const auto& site : g_dev)
        {
            Ptr<ConstantPositionMobilityModel> m = CreateObject<ConstantPositionMobilityModel>();
            m->SetPosition(site.ecef);
            devMob.push_back(m);
        }
        for (const auto& site : g_gw)
        {
            Ptr<ConstantPositionMobilityModel> m = CreateObject<ConstantPositionMobilityModel>();
            m->SetPosition(site.ecef);
            gwMob.push_back(m);
        }
        std::cout << "# satlink budget (TR 38.821 Set-1 LEO-600): noise BW " << noiseBwHz / 1e6
                  << " MHz; device EIRP -7 dBW, sat S G/T 1.1 dB/K, sat S EIRP "
                  << 34.0 + bwMhzDb << " dBW, device G/T -31.6 dB/K; feeder sat Ka EIRP "
                  << 4.0 + bwMhzDb << " dBW, gw G/T 15.9 dB/K, gw EIRP 46.2 dBW, sat Ka G/T 13 dB/K\n";
    }
    auto mkSatLink = [&](Ptr<MobilityModel> tx, Ptr<MobilityModel> rx, double eirpDbw, double gtDbK,
                         double carrierHz, double latDeg, int hop) {
        Ptr<ntncon::NtnSatLinkErrorModel> e = CreateObject<ntncon::NtnSatLinkErrorModel>();
        e->SetAttribute("EirpDbw", DoubleValue(eirpDbw));
        e->SetAttribute("RxGtDbK", DoubleValue(gtDbK));
        e->SetAttribute("BandwidthHz", DoubleValue(noiseBwHz));
        e->SetAttribute("CarrierHz", DoubleValue(carrierHz));
        e->SetAttribute("ThresholdEsNoDb", DoubleValue(1.0));
        e->SetAttribute("GroundAltKm", DoubleValue(0.0));
        e->SetAttribute("StationLatitudeDeg", DoubleValue(latDeg));
        e->SetEndpoints(tx, rx);
        Ptr<GatedSatLinkErrorModel> g = CreateObject<GatedSatLinkErrorModel>();
        g->inner = e;
        g->rx = &g_hopRx[hop];
        g->lost = &g_hopLost[hop];
        g->esnoSum = &g_hopEsno[hop];
        return g;
    };

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("1Mbps"));
    p2p.SetChannelAttribute("Delay", StringValue("5ms"));
    Ipv4AddressHelper ip;
    uint32_t subnet = 0;
    auto mkLink = [&](Ptr<Node> a, Ptr<Node> b) {
        NetDeviceContainer nd = p2p.Install(a, b);
        Link l;
        l.emA = CreateObject<RateErrorModel>();
        l.emA->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
        l.emB = CreateObject<RateErrorModel>();
        l.emB->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
        l.emA->SetRate(1.0);
        l.emB->SetRate(1.0);
        nd.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(l.emA));
        nd.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(l.emB));
        l.chan = DynamicCast<PointToPointChannel>(nd.Get(0)->GetChannel());
        // /30 subnets carved from 10.0.0.0/8
        const uint32_t base = (10u << 24) | ((subnet & 0x3FFFFF) << 2);
        ++subnet;
        ip.SetBase(Ipv4Address(base), Ipv4Mask("255.255.255.252"));
        Ipv4InterfaceContainer ic = ip.Assign(nd);
        l.addrA = ic.GetAddress(0);
        l.addrB = ic.GetAddress(1);
        return l;
    };
    g_ds.assign(D, std::vector<Link>(S));
    for (uint32_t d = 0; d < D; ++d)
    {
        for (uint32_t k = 0; k < S; ++k)
        {
            g_ds[d][k] = mkLink(devN.Get(d), satN.Get(k));
            if (g_satlink)
            {
                Link& l = g_ds[d][k];
                // A = device receives (sat -> device, S band 2.185 GHz), B = sat receives (2.0 GHz).
                l.slA = mkSatLink(g_sats[k], devMob[d], 34.0 + bwMhzDb, -31.6, 2.185e9, g_dev[d].latDeg, 1);
                l.slB = mkSatLink(devMob[d], g_sats[k], -7.0, 1.1, 2.0e9, g_dev[d].latDeg, 0);
                DynamicCast<PointToPointNetDevice>(l.chan->GetDevice(0))
                    ->SetAttribute("ReceiveErrorModel", PointerValue(l.slA));
                DynamicCast<PointToPointNetDevice>(l.chan->GetDevice(1))
                    ->SetAttribute("ReceiveErrorModel", PointerValue(l.slB));
            }
        }
    }
    g_sg.assign(S, std::vector<Link>(G));
    for (uint32_t k = 0; k < S; ++k)
    {
        for (uint32_t g = 0; g < G; ++g)
        {
            g_sg[k][g] = mkLink(gwN.Get(g), satN.Get(k));
            if (g_satlink)
            {
                Link& l = g_sg[k][g];
                // A = gateway receives (Ka 20 GHz), B = sat receives (Ka 30 GHz).
                l.slA = mkSatLink(g_sats[k], gwMob[g], 4.0 + bwMhzDb, 15.9, 20e9, g_gw[g].latDeg, 2);
                l.slB = mkSatLink(gwMob[g], g_sats[k], 46.2, 13.0, 30e9, g_gw[g].latDeg, 3);
                DynamicCast<PointToPointNetDevice>(l.chan->GetDevice(0))
                    ->SetAttribute("ReceiveErrorModel", PointerValue(l.slA));
                DynamicCast<PointToPointNetDevice>(l.chan->GetDevice(1))
                    ->SetAttribute("ReceiveErrorModel", PointerValue(l.slB));
            }
        }
    }
    p2p.SetDeviceAttribute("DataRate", StringValue("100Mbps"));
    for (uint32_t g = 0; g < G; ++g)
    {
        NetDeviceContainer nd = p2p.Install(gwN.Get(g), srvN.Get(0));
        const uint32_t base = (10u << 24) | ((subnet & 0x3FFFFF) << 2);
        ++subnet;
        ip.SetBase(Ipv4Address(base), Ipv4Mask("255.255.255.252"));
        Ipv4InterfaceContainer ic = ip.Assign(nd);
        g_gwServerAddr.push_back(ic.GetAddress(1));
    }

    // sockets and custody
    TypeId udp = UdpSocketFactory::GetTypeId();
    for (uint32_t d = 0; d < D; ++d)
    {
        g_devSock.push_back(Socket::CreateSocket(devN.Get(d), udp));
        g_devSock.back()->Bind();
        Ptr<SaginCustodyQueue> q = CreateObject<SaginCustodyQueue>();
        q->SetCapacityBytes(1 << 30);
        q->SetLifetime(lifetimeH > 0 ? Hours(lifetimeH) : Seconds(0));
        q->SetForwardCallback([d](Ptr<Packet> p) { return DeviceForward(d, p); });
        if (keepNewest)
        {
            q->SetReplaceSameSource(g_bpv7 ? &BundleSourceKey : &SourceKey);
        }
        q->SetLastComeFirstServed(lcfs);
        q->SetPerBundleDrain(g_sabr); // SABR decides per bundle (no head-of-line blocking)
        if (lifetimeE2E && !g_bpv7)
        {
            q->SetOriginFunction(&GenTime);
        }
        if (g_bundleExpiry)
        {
            // RFC 9171 5.5: each bundle expires at its own primary-block deadline.
            q->SetBundleExpiry(g_expiryFn);
            q->SetForwardTransform(SaginCustodyQueue::Bpv7AgeUpdate());
        }
        if (g_bpv7)
        {
            // RFC 9171 4.2.2: the lifetime runs from the creation timestamp.
            if (!g_bundleExpiry)
            {
                q->SetOriginFunction(&BundleCreation);
            }
            q->SetCustodyTransfer(true, Seconds(custodyTimeoutS), &BundleIdOf);
            Ptr<Socket> sig = Socket::CreateSocket(devN.Get(d), udp);
            sig->Bind(InetSocketAddress(Ipv4Address::GetAny(), kSigPort));
            sig->SetRecvCallback(MakeBoundCallback(&CustodySignalReceive, q));
            g_devSigSock.push_back(sig);
        }
        g_devQ.push_back(q);
    }
    for (uint32_t k = 0; k < S; ++k)
    {
        Ptr<Socket> rx = Socket::CreateSocket(satN.Get(k), udp);
        rx->Bind(InetSocketAddress(Ipv4Address::GetAny(), kPort));
        rx->SetRecvCallback(MakeBoundCallback(&SatelliteReceive, k));
        g_satSock.push_back(Socket::CreateSocket(satN.Get(k), udp));
        g_satSock.back()->Bind();
        Ptr<SaginCustodyQueue> q = CreateObject<SaginCustodyQueue>();
        q->SetCapacityBytes(satBufferBytes);
        // bent pipe: nothing is held on board
        q->SetLifetime(g_policy == "bp" ? NanoSeconds(1) : (lifetimeH > 0 ? Hours(lifetimeH) : Seconds(0)));
        q->SetForwardCallback([k](Ptr<Packet> p) { return SatelliteForward(k, p); });
        if (keepNewest)
        {
            q->SetReplaceSameSource(g_bpv7 ? &BundleSourceKey : &SourceKey);
        }
        q->SetLastComeFirstServed(lcfs);
        q->SetPerBundleDrain(g_sabr); // SABR decides per bundle (no head-of-line blocking)
        if (lifetimeE2E && g_policy != "bp" && !g_bpv7)
        {
            q->SetOriginFunction(&GenTime);
        }
        if (g_bpv7)
        {
            if (g_policy != "bp" && g_bundleExpiry)
            {
                q->SetBundleExpiry(g_expiryFn);
                q->SetForwardTransform(SaginCustodyQueue::Bpv7AgeUpdate());
            }
            else if (g_policy != "bp")
            {
                q->SetOriginFunction(&BundleCreation);
            }
            q->SetCustodyTransfer(true, Seconds(custodyTimeoutS), &BundleIdOf);
            Ptr<Socket> sig = Socket::CreateSocket(satN.Get(k), udp);
            sig->Bind(InetSocketAddress(Ipv4Address::GetAny(), kSigPort));
            sig->SetRecvCallback(MakeBoundCallback(&CustodySignalReceive, q));
            g_satSigSock.push_back(sig);
        }
        g_satQ.push_back(q);
    }
    for (uint32_t g = 0; g < G; ++g)
    {
        Ptr<Socket> rx = Socket::CreateSocket(gwN.Get(g), udp);
        rx->Bind(InetSocketAddress(Ipv4Address::GetAny(), kPort));
        rx->SetRecvCallback(MakeBoundCallback(&GatewayReceive, g));
        g_gwSock.push_back(Socket::CreateSocket(gwN.Get(g), udp));
        g_gwSock.back()->Bind();
    }
    Ptr<Socket> srv = Socket::CreateSocket(srvN.Get(0), udp);
    srv->Bind(InetSocketAddress(Ipv4Address::GetAny(), kPort + 1));
    srv->SetRecvCallback(MakeCallback(&ServerReceive));

    g_sinDs.assign(D, std::vector<double>(S, -1.0));
    g_devUsed.assign(D, 0);
    g_devWasUp.assign(D, false);
    g_satUsed.assign(S, 0);
    g_satWasUp.assign(S, false);
    g_visSg.assign(S, std::vector<bool>(G, false));
    const double horizon = days * 86400.0;
    if (g_policy == "sfe")
    {
        BuildPlan(horizon + 86400.0);
    }
    if (g_sabr)
    {
        const auto plan = BuildSabrPlan(horizon + 86400.0);
        for (uint32_t d = 0; d < D; ++d)
        {
            Ptr<ntnsagin::SaginSabrRouter> r = CreateObject<ntnsagin::SaginSabrRouter>();
            r->SetLocalNode(kDevIdBase + d);
            r->SetContactPlan(plan);
            g_devSabr.push_back(r);
        }
        for (uint32_t k = 0; k < S; ++k)
        {
            Ptr<ntnsagin::SaginSabrRouter> r = CreateObject<ntnsagin::SaginSabrRouter>();
            r->SetLocalNode(k + 1);
            r->SetContactPlan(plan);
            g_satSabr.push_back(r);
        }
        std::cout << "# sabr: contact plan " << plan.size() << " contacts (SGP4, " << g_tick
                  << " s resolution)\n";
    }
    g_genStop = horizon - 86400.0;
    g_out.open(out);
    g_out << "device,seq,gen_s,rx_s\n";

    Ptr<UniformRandomVariable> u = CreateObject<UniformRandomVariable>();
    const Time period = Seconds(periodMin * 60.0);
    for (uint32_t d = 0; d < D; ++d)
    {
        // updates during the first (days - 1) days; the last day lets custody drain
        Simulator::Schedule(Seconds(u->GetValue(0, period.GetSeconds())), &Generate, d, 0u, period);
    }
    g_satSeen.assign(S, {});
    Ptr<ntncon::ContactGraphScheduler> schedDev;
    Ptr<ntncon::ContactGraphScheduler> schedGw;
    if (g_cgr)
    {
        // Same SGP4 models, same masks, sampled on the tick grid, no hysteresis (the example's
        // single-threshold rule); started before Tick so a tick reads this tick's contacts.
        g_cgrDs.assign(D, std::vector<bool>(S, false));
        g_cgrSg.assign(S, std::vector<bool>(G, false));
        schedDev = CreateObject<ntncon::ContactGraphScheduler>();
        schedGw = CreateObject<ntncon::ContactGraphScheduler>();
        for (auto [sch, mask] : {std::make_pair(schedDev, devMaskDeg), std::make_pair(schedGw, gwMaskDeg)})
        {
            sch->SetSamplingInterval(Seconds(g_tick));
            sch->SetMinElevationDeg(mask);
            sch->SetGateHysteresisDeg(0.0);
            sch->SetMaxIslRangeM(0.0); // ground contacts only
            for (uint32_t k = 0; k < S; ++k)
            {
                sch->RegisterSatellite(k + 1, g_sats[k]);
            }
        }
        for (uint32_t d = 0; d < D; ++d)
        {
            schedDev->RegisterGroundStation(kDevIdBase + d, g_dev[d].latDeg, g_dev[d].lonDeg, 0.0);
        }
        for (uint32_t g = 0; g < G; ++g)
        {
            schedGw->RegisterGroundStation(kGwIdBase + g, g_gw[g].latDeg, g_gw[g].lonDeg, 0.0);
        }
        auto onDev = [](bool up, ntncon::ContactEvent ev) {
            if (ev.is_isl)
            {
                return;
            }
            g_cgrDs[ev.node_b - kDevIdBase][ev.node_a - 1] = up;
        };
        auto onGw = [](bool up, ntncon::ContactEvent ev) {
            if (ev.is_isl)
            {
                return;
            }
            g_cgrSg[ev.node_a - 1][ev.node_b - kGwIdBase] = up;
        };
        schedDev->m_contactUp.ConnectWithoutContext(
            Callback<void, ntncon::ContactEvent>([onDev](ntncon::ContactEvent e) { onDev(true, e); }));
        schedDev->m_contactDown.ConnectWithoutContext(
            Callback<void, ntncon::ContactEvent>([onDev](ntncon::ContactEvent e) { onDev(false, e); }));
        schedGw->m_contactUp.ConnectWithoutContext(
            Callback<void, ntncon::ContactEvent>([onGw](ntncon::ContactEvent e) { onGw(true, e); }));
        schedGw->m_contactDown.ConnectWithoutContext(
            Callback<void, ntncon::ContactEvent>([onGw](ntncon::ContactEvent e) { onGw(false, e); }));
        g_router = CreateObject<ntncon::ContactGraphRouter>();
        g_router->Attach(schedGw);
        schedDev->Start();
        schedGw->Start();
    }
    Simulator::Schedule(Seconds(0), &Tick);
    Simulator::Stop(Seconds(horizon));
    Simulator::Run();

    // Counters per hop: the device queues supersede and expire too, so the satellite-side numbers are
    // reported apart from them.
    uint64_t replSat = 0, replDev = 0, expSat = 0, expDev = 0, dropSat = 0, dropDev = 0;
    for (auto& q : g_satQ)
    {
        replSat += q->GetReplaced();
        expSat += q->GetExpired();
        dropSat += q->GetDroppedForSpace();
    }
    for (auto& q : g_devQ)
    {
        replDev += q->GetReplaced();
        expDev += q->GetExpired();
        dropDev += q->GetDroppedForSpace();
    }
    std::cout << "sats=" << S << " devices=" << D << " gateways=" << G << " policy=" << g_policy
              << " generated=" << g_generated << " replaced_sat=" << replSat << " replaced_dev=" << replDev
              << " expired_sat=" << expSat << " expired_dev=" << expDev << " dropped_sat=" << dropSat
              << " dropped_dev=" << dropDev << " out=" << out << "\n";
    if (g_satlink || g_cgr || g_bpv7)
    {
        std::cout << "# M3 options: linkModel=" << linkModel << " contacts=" << contacts << " dtn=" << dtn
                  << "\n";
        std::cout << "delivered=" << g_delivered << " delivered_ratio=" << std::fixed
                  << std::setprecision(4) << (g_generated ? double(g_delivered) / g_generated : 0.0)
                  << std::defaultfloat << "\n";
    }
    if (g_satlink)
    {
        const char* names[4] = {"dev->sat", "sat->dev", "sat->gw", "gw->sat"};
        for (int h = 0; h < 4; ++h)
        {
            std::cout << "hop " << names[h] << " rx=" << g_hopRx[h] << " lost=" << g_hopLost[h]
                      << " packet_loss=" << (g_hopRx[h] ? double(g_hopLost[h]) / g_hopRx[h] : 0.0)
                      << " mean_esno_db=" << (g_hopRx[h] ? g_hopEsno[h] / g_hopRx[h] : 0.0) << "\n";
        }
    }
    if (g_cgr)
    {
        std::cout << "cgr: dev_gsl_up=" << schedDev->GslEventsUp() << " dev_gsl_down=" << schedDev->GslEventsDown()
                  << " gw_gsl_up=" << schedGw->GslEventsUp() << " gw_gsl_down=" << schedGw->GslEventsDown()
                  << " router_edges_added=" << g_router->EdgesAddedTotal()
                  << " router_edges_removed=" << g_router->EdgesRemovedTotal()
                  << " router_gateway_picks=" << g_routerGwPicks << "\n";
        schedDev->Stop();
        schedGw->Stop();
    }
    if (g_bpv7)
    {
        uint64_t acc = 0, ref = 0, retx = 0, delRep = 0, awaiting = 0;
        for (auto* v : {&g_devQ, &g_satQ})
        {
            for (auto& q : *v)
            {
                acc += q->GetCustodyAccepted();
                ref += q->GetCustodyRefused();
                retx += q->GetCustodyRetransmissions();
                delRep += q->GetDeletionReports();
                awaiting += q->GetAwaitingCustody();
            }
        }
        const uint64_t sent = g_sigSentOk + g_sigSentRedundant + g_sigSentRefused;
        uint64_t expArr = 0, expSto = 0, malformed = 0;
        for (auto* v : {&g_devQ, &g_satQ})
        {
            for (auto& q : *v)
            {
                expArr += q->GetExpiredOnArrival();
                expSto += q->GetExpiredInStorage();
                malformed += q->GetMalformed();
            }
        }
        std::cout << "bpv7_expiry: mode=" << (g_bundleExpiry ? "bundle(RFC9171-5.5)" : "queue")
                  << " clockless=" << g_clockless << " expired_on_arrival=" << expArr
                  << " expired_in_storage=" << expSto << " expired_at_gateway=" << g_expiredAtGw
                  << " expired_at_server=" << g_expiredAtSrv << " malformed=" << malformed << "\n";
        if (g_sabr)
        {
            uint64_t comps = 0;
            for (auto* v : {&g_devSabr, &g_satSabr})
            {
                for (auto& r : *v)
                {
                    comps += r->GetRouteComputations();
                }
            }
            std::cout << "sabr: forwarded=" << g_sabrFwd << " held_for_later_contact=" << g_sabrHold
                      << " no_candidate_route=" << g_sabrNoRoute << " route_computations=" << comps
                      << "\n";
        }
        std::cout << "bpv7: bundles_generated=" << g_generated << " bundles_delivered=" << g_delivered
                  << " duplicates_at_server=" << g_dupAtServer << " bundles_expired=" << (expSat + expDev)
                  << " deletion_status_reports=" << delRep << " custody_signals_sent=" << sent
                  << " (accepted=" << g_sigSentOk + g_sigSentRedundant << " of which redundant="
                  << g_sigSentRedundant << ", refused=" << g_sigSentRefused << ")"
                  << " custody_signals_received=" << g_sigRecv << " custody_released=" << acc
                  << " custody_refusals_handled=" << ref << " custody_retransmissions=" << retx
                  << " still_awaiting_signal=" << awaiting << "\n";
    }
    g_out.close();
    Simulator::Destroy();
    return 0;
}
