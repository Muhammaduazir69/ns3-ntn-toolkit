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

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/orbital-elements.h"
#include "ns3/point-to-point-module.h"
#include "ns3/sagin-custody-queue.h"
#include "ns3/sgp4-mobility-model.h"

#include <cmath>
#include <fstream>
#include <map>
#include <sstream>

using namespace ns3;
using ns3::ntnsagin::SaginCustodyQueue;

NS_LOG_COMPONENT_DEFINE("SaginStoreForwardIot");

namespace
{
constexpr double kC = 299792458.0;
constexpr double kA = 6378137.0;
constexpr double kF = 1.0 / 298.257223563;
constexpr uint16_t kPort = 9000;

struct Site
{
    std::string name;
    Vector ecef;
};

struct Link
{
    Ptr<RateErrorModel> emA, emB;
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
        out.push_back({name, Geodetic(std::stod(la), std::stod(lo))});
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

void
SetLink(Link& l, bool up, double rangeM)
{
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
            if (g_sinDs[d][k] < g_sinDevMask)
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
        if (g_sinDs[d][k] < g_sinDevMask)
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
    while ((p = s->Recv()))
    {
        g_satQ[k]->Offer(p);
    }
}

void
GatewayReceive(uint32_t g, Ptr<Socket> s)
{
    Ptr<Packet> p;
    while ((p = s->Recv()))
    {
        g_gwSock[g]->SendTo(p, 0, InetSocketAddress(g_gwServerAddr[g], kPort + 1));
    }
}

void
ServerReceive(Ptr<Socket> s)
{
    Ptr<Packet> p;
    while ((p = s->Recv()))
    {
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
    for (uint32_t k = 0; k < S; ++k)
    {
        bool any = false;
        for (uint32_t g = 0; g < g_gw.size(); ++g)
        {
            const bool v = SinElevation(g_gw[g].ecef, pos[k]) >= g_sinGwMask;
            g_visSg[k][g] = v;
            any = any || v;
            SetLink(g_sg[k][g], v, CalculateDistance(g_gw[g].ecef, pos[k]));
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
            const bool v = se >= g_sinDevMask;
            any = any || v;
            SetLink(g_ds[d][k], v, CalculateDistance(g_dev[d].ecef, pos[k]));
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
    cmd.AddValue("policy", "bp | sf1 | sfe", g_policy);
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
    cmd.Parse(argc, argv);
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
        }
    }
    g_sg.assign(S, std::vector<Link>(G));
    for (uint32_t k = 0; k < S; ++k)
    {
        for (uint32_t g = 0; g < G; ++g)
        {
            g_sg[k][g] = mkLink(gwN.Get(g), satN.Get(k));
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
            q->SetReplaceSameSource(&SourceKey);
        }
        q->SetLastComeFirstServed(lcfs);
        if (lifetimeE2E)
        {
            q->SetOriginFunction(&GenTime);
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
            q->SetReplaceSameSource(&SourceKey);
        }
        q->SetLastComeFirstServed(lcfs);
        if (lifetimeE2E && g_policy != "bp")
        {
            q->SetOriginFunction(&GenTime);
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
    g_out.close();
    Simulator::Destroy();
    return 0;
}
