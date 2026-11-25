/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W7)
 *
 * ntn-v2x-leo-relay-traffic, a shadowed vehicle (veh0) relays its basic-safety
 * messages to a ground server through a peer vehicle (veh1) and a LEO satellite:
 * veh0 --V2V--> veh1 --uplink--> LEO --feeder--> server. Real UDP BSMs are
 * forwarded THROUGH the relay nodes (real Ipv4 routing); each hop's propagation
 * delay is the real slant range / c.
 *
 * The per-hop contact gates are driven by the V2xLeoRelay engine itself, NOT
 * by an inline geometry test: V2xLeoRelay::EvaluateAll() returns, per vehicle,
 * the route decision (direct vs relay), the chosen peer, the V2V range, and the
 * direct/via-relay SNRs; the example sets the V2V and uplink error-model rates
 * straight off that RelayDecision (V2V up only when the engine selects a relay
 * peer in range; uplink up only when the serving LEO link clears the engine's
 * --minDirectSnr threshold). The relay engine is the single source of the route
 * decision, not a printed side-channel.
 *
 * Delivered BSM rate / latency / jitter / loss are MEASURED end-to-end by
 * NtnOranSink from the in-band NtnOranPayloadHeader (WS1 application suite).
 * The real-PHY V2X baseline (measured mmwave SINR) is ntn-v2x-rural-highway.
 *
 * Vehicles follow an FCD-format CSV trace supplied via --fcdTrace. The shipped
 * trace is a synthetic constant-speed FCD-format CSV fixture (not a SUMO
 * microsimulation; the loader reads a CSV dialect, not SUMO's native
 * fcd-output XML). You may instead supply your own CSV converted from a SUMO
 * fcd-export.
 *
 * Wave 2 M3 (P1-5): `--v2v=pc5 --uplink=nr` replaces both P2P hops with the
 * measured planes. veh0 (remote UE, no Uu: it sits in the shadow) broadcasts
 * its 10 Hz SAE J2735 BSMs on the NR PC5 Mode-2 sidelink (NtnSlChannel:
 * sensing-based SPS, half duplex, collisions, TR 37.885 Highway path loss)
 * among --pc5Background other vehicles that send their own BSMs. veh1 (relay
 * UE) decodes veh0's BSM and forwards it over its own Uu uplink on the
 * measured NR NTN cell (NtnRealStackHelper, nr backend, TR 38.821 Set-1
 * terminal and payload, feeder from a gateway at the origin): the L3
 * UE-to-network relay of TS 23.304 Rel-17. The ground server measures the
 * end-to-end delay (BSM generation to arrival), the PC5 part (generation to
 * relay decode, including the Mode-2 resource wait of up to T2) and the Uu UL
 * one-way delay (relay forward to arrival). The default (p2p/p2p) is
 * unchanged.
 *
 * Quick test:  --simSeconds=60 --bsmHz=10 --fcdTrace=<path/to/fcd.csv>
 *              --simSeconds=30 --v2v=pc5 --uplink=nr
 */
#include "ns3/ntn-dataset-writer.h"
#include "ns3/applications-module.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/ntn-v2x-helper.h"
#include "ns3/sumo-traci-bridge.h"
#include "ns3/error-model.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-v2x-bsm-header.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4.h"
#include "ns3/point-to-point-module.h"
#include "ns3/v2x-leo-relay.h"
#include "ns3/ntn-nr-sidelink.h"
#include "ns3/ntn-oran-payload-header.h"
#include "ns3/ntn-real-stack-helper.h"

#include <cmath>
#include <set>
#include <vector>
#include <functional>
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnV2xLeoRelayTraffic");

namespace
{
constexpr double kC = 299792458.0;
Ptr<ntnv2x::V2xLeoRelay> g_relay;
Ptr<MobilityModel> g_veh0, g_veh1, g_sat;
Ptr<RateErrorModel> g_emV2v, g_emUplink, g_emDirect;
Ptr<PointToPointChannel> g_chV2v, g_chUplink, g_chDirect, g_chFeeder;
// Gateway (feeder-link ground end) at the scenario origin: the transparent
// payload case of TR 38.821 Sec. 4.2 with the gateway inside the same
// footprint, so the feeder hop is a real slant range, not a 10 us stub.
const Vector g_gwPos(0.0, 0.0, 0.0);
std::ofstream g_csv;
Ptr<Ipv4> g_veh0Ipv4;
uint32_t g_ifV2v = 1, g_ifDirect = 2;
Ptr<NtnOranSink> g_sink;
Ptr<MobilityModel> g_veh0Mob;
uint8_t g_bsmCnt = 0;
Ptr<ntnv2x::SumoTraciBridge> g_bridge;
uint64_t g_lastRx = 0;
double g_maxV2vRangeM = 1500.0;
double g_minDirectSnrDb = 6.0;
double g_simTime = 60.0;
bool g_uplinkUp = false;

double
Dist(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double
ElevDeg(const Vector& u, const Vector& s)
{
    const Vector d(s.x - u.x, s.y - u.y, s.z - u.z);
    return std::atan2(d.z, std::max(std::sqrt(d.x * d.x + d.y * d.y), 1e-3)) * 180.0 / M_PI;
}

void
Tick()
{
    if (Simulator::Now().GetSeconds() >= g_simTime)
    {
        return;
    }
    if (g_bridge)
    {
        g_bridge->Step(); // advance both vehicles along the FCD trace
    }
    const double dV2v = Dist(g_veh0->GetPosition(), g_veh1->GetPosition());
    const double dUp = Dist(g_veh1->GetPosition(), g_sat->GetPosition());
    const double elevUp = ElevDeg(g_veh1->GetPosition(), g_sat->GetPosition());

    // Genuine slant-range propagation delays on each hop (geometry, not a
    // fabricated SINR). These are real range/c and stay regardless of gating.
    g_chV2v->SetAttribute("Delay", TimeValue(Seconds(dV2v / kC)));
    g_chUplink->SetAttribute("Delay", TimeValue(Seconds(dUp / kC)));
    // Feeder hop: satellite to gateway slant range. It was a fixed 10 us, so the
    // reported e2e delay carried only one of the two space legs of a bent pipe.
    g_chFeeder->SetAttribute("Delay", TimeValue(Seconds(Dist(g_sat->GetPosition(), g_gwPos) / kC)));

    // The packet-routing decision is owned by the V2xLeoRelay engine, NOT by an
    // inline geometry test: EvaluateAll() returns, per vehicle, whether veh0
    // should go direct or relay through a peer, the chosen peer, the peer's
    // V2V range, and the measured direct/via-relay SNRs. We drive the V2V and
    // uplink error-model gates straight off that RelayDecision.
    auto decisions = g_relay->EvaluateAll();
    const ntnv2x::RelayDecision* d0 = nullptr;
    for (const auto& d : decisions)
    {
        if (d.vehId == "veh0")
        {
            d0 = &d;
        }
    }

    // V2V hop carries traffic only when the relay engine actually selects a
    // relay path: veh0 is NOT direct, a peer was chosen, and that peer is in
    // V2V range. SetRate(0)=link up (no induced loss); SetRate(1)=link down.
    const bool relaySelected = d0 && !d0->directToLeo && !d0->relayPeerId.empty() &&
                               d0->v2vRangeM <= g_maxV2vRangeM;
    g_emV2v->SetRate(relaySelected ? 0.0 : 1.0);

    // GAP V2 FIX: steer veh0's PATH to match the engine's decision. When it
    // chooses direct, bring veh0's direct-to-sat interface UP and the V2V one
    // DOWN (and vice-versa) and recompute routing, so packets follow the chosen
    // egress instead of the fixed V2V chain. This is what makes "direct" deliver
    // instead of dropping every packet.
    const bool directSelected = d0 && d0->directToLeo;
    const double directSlant = Dist(g_veh0->GetPosition(), g_sat->GetPosition());
    g_chDirect->SetAttribute("Delay", TimeValue(Seconds(directSlant / kC)));
    if (g_veh0Ipv4)
    {
        const bool v2vUpNow = g_veh0Ipv4->IsUp(g_ifV2v);
        const bool directUpNow = g_veh0Ipv4->IsUp(g_ifDirect);
        const bool wantV2vUp = !directSelected;
        const bool wantDirectUp = directSelected;
        if (v2vUpNow != wantV2vUp || directUpNow != wantDirectUp)
        {
            wantV2vUp ? g_veh0Ipv4->SetUp(g_ifV2v) : g_veh0Ipv4->SetDown(g_ifV2v);
            wantDirectUp ? g_veh0Ipv4->SetUp(g_ifDirect) : g_veh0Ipv4->SetDown(g_ifDirect);
            Ipv4GlobalRoutingHelper::RecomputeRoutingTables();
        }
        // The direct hop's own link budget still gates delivery: up only when
        // veh0's direct LEO link clears the engine's SNR threshold.
        g_emDirect->SetRate(directSelected ? 0.0 : 1.0);
    }

    // Uplink hop carries traffic when the relay engine's chosen LEO link (the
    // relay peer's link when relaying, or veh0's own link when direct) clears
    // the minimum direct-SNR threshold the engine was configured with.
    double servingSnr = 0.0;
    if (d0)
    {
        servingSnr = d0->directToLeo ? d0->directSnrDb : d0->viaRelaySnrDb;
    }
    const bool uplinkUp = d0 && (servingSnr >= g_minDirectSnrDb);
    g_uplinkUp = uplinkUp;
    g_emUplink->SetRate(uplinkUp ? 0.0 : 1.0);

    const uint64_t tot = g_sink ? g_sink->GetTotalRx() : 0;
    const double bsmRx = double(tot - g_lastRx);
    g_lastRx = tot;
    std::printf("  %6.1f  v2vRange=%7.0fm  upElev=%5.1f  %-5s peer=%-5s  rx_Bps=%6.0f\n",
                Simulator::Now().GetSeconds(), dV2v, elevUp,
                d0 ? (d0->directToLeo ? "DIR" : "RELAY") : "?",
                (d0 && !d0->relayPeerId.empty()) ? d0->relayPeerId.c_str() : "-", bsmRx);
    if (g_csv.is_open())
    {
        g_csv << Simulator::Now().GetSeconds() << "," << dV2v << "," << elevUp << ","
              << (d0 ? (d0->directToLeo ? "direct" : "relay") : "none") << ","
              << ((d0 && !d0->relayPeerId.empty()) ? d0->relayPeerId : "-") << ","
              << (d0 ? d0->directSnrDb : 0.0) << "," << servingSnr << "," << uplinkUp << ","
              << bsmRx << "\n";
    }
    Simulator::Schedule(Seconds(1.0), &Tick);
}

// ===========================================================================
// Wave 2 M3 (P1-5): measured PC5 V2V hop + measured NR Uu uplink at the relay.
// ===========================================================================
struct Pc5NrState
{
    Ptr<Packet> pendingBsm;      //!< newest BSM at veh0 awaiting its PC5 resource
    uint64_t generated{0};
    uint64_t replacedStale{0};   //!< a newer BSM replaced one never sent (J2735 freshness)
    uint64_t pc5Sent{0};
    uint64_t relayDecoded{0};
    uint64_t forwarded{0};
    uint64_t serverRx{0};
    double pc5SumMs{0.0};
    double ulSumMs{0.0};
    double e2eSumMs{0.0};
    double e2eMaxMs{0.0};
    std::vector<double> e2e;
    Ptr<Socket> relaySock;
    Ptr<MobilityModel> veh0;
    uint32_t seq{0};
    uint32_t bsmBytes{300};
    std::set<uint64_t> bsmUids; //!< packets that carry a veh0 BSM (fillers excluded)
};
Pc5NrState g_pn;

void
Pc5GenerateBsm(Time period, Time stop)
{
    if (Simulator::Now() >= stop)
    {
        return;
    }
    // SAE J2735 BSM Part I from veh0's live kinematics, behind the in-band
    // payload header that carries the generation time.
    ntnv2x::NtnV2xBsmHeader bsm;
    const Vector p = g_pn.veh0->GetPosition();
    const Vector v = g_pn.veh0->GetVelocity();
    bsm.SetFromState(static_cast<uint8_t>(g_pn.seq % 128), 0x00000001u,
                     static_cast<uint16_t>(Simulator::Now().GetMilliSeconds() % 60000),
                     p.y / 111320.0, p.x / 111320.0, p.z, std::sqrt(v.x * v.x + v.y * v.y),
                     std::fmod(std::atan2(v.x, v.y) * 180.0 / M_PI + 360.0, 360.0));
    const uint32_t body = std::max<uint32_t>(
        bsm.GetSerializedSize(), g_pn.bsmBytes > NtnOranPayloadHeader::SERIALIZED_SIZE
                                     ? g_pn.bsmBytes - NtnOranPayloadHeader::SERIALIZED_SIZE
                                     : 0);
    Ptr<Packet> pkt = Create<Packet>(body - bsm.GetSerializedSize());
    pkt->AddHeader(bsm);
    NtnOranPayloadHeader h;
    h.SetPayloadType(NtnOranPayloadHeader::URLLC_CMD);
    h.SetSeq(g_pn.seq++);
    h.SetTxTimestampNs(Simulator::Now().GetNanoSeconds());
    h.SetFiveQi(82);
    h.SetSrcId(0);
    h.SetDstId(3);
    pkt->AddHeader(h);
    if (g_pn.pendingBsm)
    {
        ++g_pn.replacedStale;
    }
    g_pn.pendingBsm = pkt;
    ++g_pn.generated;
    Simulator::Schedule(period, &Pc5GenerateBsm, period, stop);
}

Ptr<Packet>
Pc5TxPacket(uint32_t id)
{
    if (id == 0 && g_pn.pendingBsm)
    {
        Ptr<Packet> p = g_pn.pendingBsm;
        g_pn.pendingBsm = nullptr;
        ++g_pn.pc5Sent;
        g_pn.bsmUids.insert(p->GetUid());
        return p;
    }
    // Other vehicles: their own BSM-sized broadcasts (channel load).
    return Create<Packet>(g_pn.bsmBytes);
}

void
Pc5RelayRx(uint32_t from, Ptr<Packet> pkt)
{
    if (from != 0 || g_pn.bsmUids.erase(pkt->GetUid()) == 0)
    {
        return; // only veh0's BSMs (first decode of each) are relayed
    }
    NtnOranPayloadHeader h;
    if (pkt->GetSize() < h.GetSerializedSize())
    {
        return;
    }
    pkt->PeekHeader(h);
    ++g_pn.relayDecoded;
    g_pn.pc5SumMs += (Simulator::Now().GetNanoSeconds() - static_cast<int64_t>(h.GetTxTimestampNs())) / 1e6;
    // L3 relay forwarding: the relay's own Uu PDU session carries it; the
    // forward time rides a SeqTs header in front of the BSM.
    Ptr<Packet> fwd = pkt->Copy();
    SeqTsHeader st;
    st.SetSeq(h.GetSeq());
    fwd->AddHeader(st);
    if (g_pn.relaySock->Send(fwd) >= 0)
    {
        ++g_pn.forwarded;
    }
}

void
Pc5ServerRx(Ptr<Socket> s)
{
    Ptr<Packet> p;
    while ((p = s->Recv()))
    {
        SeqTsHeader st;
        if (p->GetSize() < st.GetSerializedSize())
        {
            continue;
        }
        p->RemoveHeader(st);
        NtnOranPayloadHeader h;
        if (p->GetSize() < h.GetSerializedSize())
        {
            continue;
        }
        p->PeekHeader(h);
        const int64_t now = Simulator::Now().GetNanoSeconds();
        const double e2e = (now - static_cast<int64_t>(h.GetTxTimestampNs())) / 1e6;
        const double ul = (Simulator::Now() - st.GetTs()).GetSeconds() * 1e3;
        ++g_pn.serverRx;
        g_pn.e2eSumMs += e2e;
        g_pn.e2eMaxMs = std::max(g_pn.e2eMaxMs, e2e);
        g_pn.ulSumMs += ul;
        g_pn.e2e.push_back(e2e);
    }
}

int
RunPc5Nr(double simSeconds,
         double bsmHz,
         uint32_t bsmBytes,
         double leoAltKm,
         const std::string& fcdTrace,
         const std::string& outputDir,
         uint32_t nBackground,
         double relayUeTxDbm,
         double relayUeGainDbi)
{
    g_pn.bsmBytes = bsmBytes;
    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer relayUe;
    relayUe.Create(1);

    g_bridge = CreateObject<ntnv2x::SumoTraciBridge>();
    NS_ABORT_MSG_IF(!g_bridge->LoadFcdTrace(fcdTrace), "could not load FCD trace " << fcdTrace);
    Ptr<ConstantVelocityMobilityModel> veh0 = CreateObject<ConstantVelocityMobilityModel>();
    veh0->SetPosition(Vector(0, 0, 1.5));
    g_bridge->RegisterVehicle("veh0", veh0);
    Ptr<ConstantVelocityMobilityModel> veh1 = CreateObject<ConstantVelocityMobilityModel>();
    veh1->SetPosition(Vector(50, 0, 1.5));
    relayUe.Get(0)->AggregateObject(veh1);
    g_bridge->RegisterVehicle("veh1", veh1);
    g_pn.veh0 = veh0;

    ns3::ntncon::WalkerConfig wcfgSat;
    wcfgSat.num_planes = 1;
    wcfgSat.total_sats = 80;
    wcfgSat.altitude_km = leoAltKm;
    wcfgSat.inclination_deg = 53.0;
    wcfgSat.epoch_unix_s = 1735689600.0;
    const auto satElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfgSat);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElementsSgp4(satElements[0]);
    double la, lo, al;
    satSgp4->GetGeodetic(la, lo, al);
    Ptr<NtnEnuProjectionMobilityModel> sat = CreateObject<NtnEnuProjectionMobilityModel>();
    sat->SetSource(satSgp4);
    sat->SetReference(la, lo, 0.0);
    satNodes.Get(0)->AggregateObject(sat);

    // Measured NR NTN cell; the relay UE is its only UE (veh0 is shadowed and
    // has no Uu connection).
    NtnRealStackHelper rs;
    rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
    rs.SetNumerology(1);
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-v2x-leo-relay-traffic-pc5-nr");
    rs.ApplyTr38821Set1Defaults(); // TR 38.821 Table 6.1.1.1-1/-2 Set-1, S band
    if (relayUeTxDbm > 0.0)
    {
        rs.SetUeTxPowerDbm(relayUeTxDbm); // declared vehicle terminal, not Set-1 handheld
    }
    if (relayUeGainDbi != 0.0)
    {
        rs.SetUeRxAntennaGain(relayUeGainDbi); // reciprocal element gain (roof antenna)
    }
    rs.Build(satNodes, relayUe);
    Ptr<ConstantPositionMobilityModel> gw = CreateObject<ConstantPositionMobilityModel>();
    gw->SetPosition(g_gwPos); // feeder from a gateway at the origin (TR 38.821 4.2)
    rs.SetFeederGateway(gw);

    Ptr<Node> host = rs.GetRemoteHost();
    const Ipv4Address hostAddr = host->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
    const uint16_t port = 7601;
    Ptr<Socket> srv = Socket::CreateSocket(host, UdpSocketFactory::GetTypeId());
    srv->Bind(InetSocketAddress(Ipv4Address::GetAny(), port));
    srv->SetRecvCallback(MakeCallback(&Pc5ServerRx));
    g_pn.relaySock = Socket::CreateSocket(relayUe.Get(0), UdpSocketFactory::GetTypeId());
    g_pn.relaySock->Bind();
    g_pn.relaySock->Connect(InetSocketAddress(hostAddr, port));

    // PC5: veh0 (id 0), veh1 (id 1) and background vehicles (TR 37.885
    // highway platoon spacing 2.5 s x speed ~ 50 m) on both sides.
    Ptr<NtnSlChannel> ch = CreateObject<NtnSlChannel>();
    NtnSlResourcePool pool;
    pool.numSubchannels = 5;
    pool.subchannelSizeRb = 10;
    pool.slotDuration = MilliSeconds(1);
    ch->SetResourcePool(pool);
    const uint32_t prsvp = static_cast<uint32_t>(std::lround(1000.0 / bsmHz));
    std::vector<Ptr<MobilityModel>> mobs{veh0, veh1};
    for (uint32_t i = 0; i < nBackground; ++i)
    {
        Ptr<ConstantVelocityMobilityModel> m = CreateObject<ConstantVelocityMobilityModel>();
        const double off = 50.0 * ((i / 2) + 1) * ((i % 2) ? 1.0 : -1.0);
        m->SetPosition(Vector(off, (i % 3) * 4.0, 1.5));
        m->SetVelocity(Vector(20.0 + (i % 3), 0.0, 0.0));
        mobs.push_back(m);
    }
    std::vector<Ptr<NtnSlUeMac>> macs;
    for (uint32_t i = 0; i < mobs.size(); ++i)
    {
        Ptr<NtnSlUeMac> mac = CreateObject<NtnSlUeMac>();
        mac->SetUeId(i);
        mac->SetMobility(mobs[i]);
        mac->SetSelectionWindow(1, prsvp - 1); // T2 <= PDB (TS 38.214 8.1.4)
        mac->SetReservationPeriod(prsvp);
        mac->SetAlignReselection(true);
        mac->SetPacketBytes(bsmBytes);
        mac->AssignStreams(2000 + i);
        mac->SetTxPacketCallback(MakeCallback(&Pc5TxPacket));
        if (i == 1)
        {
            mac->SetRxCallback(MakeCallback(&Pc5RelayRx));
        }
        ch->AddUe(mac);
        macs.push_back(mac);
    }
    ch->Start(Seconds(1.0), Seconds(simSeconds));
    Simulator::Schedule(Seconds(1.0), &Pc5GenerateBsm, Seconds(1.0 / bsmHz), Seconds(simSeconds - 0.5));

    std::function<void()> step;
    step = [&step, simSeconds]() {
        if (Simulator::Now().GetSeconds() >= simSeconds)
        {
            return;
        }
        g_bridge->Step();
        Simulator::Schedule(Seconds(1.0), step);
    };
    Simulator::Schedule(Seconds(0.0), step);

    std::printf("# ntn-v2x-leo-relay-traffic --v2v=pc5 --uplink=nr\n"
                "#   veh0 --PC5 Mode 2--> veh1 (relay UE) --NR Uu UL (measured)--> LEO --feeder--> "
                "server; %u background PC5 vehicles, BSM %.0f Hz x %u B, SPS %u ms\n",
                nBackground, bsmHz, bsmBytes, prsvp);
    Simulator::Stop(Seconds(simSeconds + 1.0));
    // Globe scene of this run's own geometry (ntn-dataset@1, plan L4).
    ns3::ntnobs::StartSceneFromNodeList(outputDir);
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::sort(g_pn.e2e.begin(), g_pn.e2e.end());
    const double p95 = g_pn.e2e.empty() ? 0.0 : g_pn.e2e[std::min<size_t>(g_pn.e2e.size() - 1,
                                                                  static_cast<size_t>(0.95 * g_pn.e2e.size()))];
    const double pc5 = g_pn.relayDecoded ? g_pn.pc5SumMs / g_pn.relayDecoded : 0.0;
    const double ul = g_pn.serverRx ? g_pn.ulSumMs / g_pn.serverRx : 0.0;
    const double e2e = g_pn.serverRx ? g_pn.e2eSumMs / g_pn.serverRx : 0.0;
    std::printf("# === summary (pc5 + nr) ===\n"
                "#   BSMs generated %llu, replaced before a PC5 resource %llu, sent on PC5 %llu\n"
                "#   decoded by the relay %llu (PC5 PDR %.4f), forwarded on Uu %llu, "
                "received at the server %llu\n"
                "#   end-to-end PDR %.4f\n"
                "#   delay: PC5 (generation->relay decode) %.2f ms + Uu UL OWD (relay->server, "
                "measured) %.2f ms = %.2f ms; measured e2e mean %.2f ms, p95 %.2f ms, max %.2f ms\n"
                "#   PC5 channel: tx %llu, PRR %.4f, PRR within 100 m %.4f\n"
                "#   relay UE Uu: uplink only (no DL traffic; the DL-only sim_health throughput "
                "gate does not apply), UE Tx %.1f dBm\n",
                (unsigned long long)g_pn.generated, (unsigned long long)g_pn.replacedStale,
                (unsigned long long)g_pn.pc5Sent, (unsigned long long)g_pn.relayDecoded,
                g_pn.pc5Sent ? double(g_pn.relayDecoded) / g_pn.pc5Sent : 0.0,
                (unsigned long long)g_pn.forwarded, (unsigned long long)g_pn.serverRx,
                g_pn.generated ? double(g_pn.serverRx) / g_pn.generated : 0.0, pc5, ul, pc5 + ul,
                e2e, p95, g_pn.e2eMaxMs, (unsigned long long)ch->GetTxTotal(), ch->GetPrr(),
                ch->GetPrrWithinRange(100.0), relayUeTxDbm > 0.0 ? relayUeTxDbm : 23.0);
    {
        std::error_code ec;
        std::filesystem::create_directories(outputDir, ec);
        std::ofstream f(outputDir + "/leo_relay_pc5_nr_summary.csv");
        f << "metric,value,unit\n"
          << "bsm_generated," << g_pn.generated << ",count\n"
          << "pc5_sent," << g_pn.pc5Sent << ",count\n"
          << "relay_decoded," << g_pn.relayDecoded << ",count\n"
          << "server_rx," << g_pn.serverRx << ",count\n"
          << "e2e_pdr," << (g_pn.generated ? double(g_pn.serverRx) / g_pn.generated : 0.0) << ",ratio\n"
          << "pc5_delay_mean," << pc5 << ",ms\n"
          << "uu_ul_owd_mean," << ul << ",ms\n"
          << "e2e_delay_mean," << e2e << ",ms\n"
          << "e2e_delay_p95," << p95 << ",ms\n";
    }
    g_pn.relaySock = nullptr;
    g_pn.pendingBsm = nullptr;
    Simulator::Destroy();
    return 0;
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 60.0;
    double bsmHz = 10.0;
    uint32_t bsmBytes = 300;
    double leoAltKm = 550.0;
    double satSpeed = 7500.0;
    double relayDriftMps = 25.0;
    double maxV2vRangeM = 1500.0;
    double minDirectSnrDb = 6.0;
    double veh0BlockageDb = 14.0; // veh0 is the shadowed (NLOS) vehicle that relays
    double linkCapacityMbps = 20.0;
    // Default to the shipped synthetic FCD fixture so the example runs out of
    // the box (resolves from the ns-3 root, which is the run cwd); override with
    // --fcdTrace=<path> to replay a real SUMO-exported trace.
    std::string fcdTrace = "contrib/ntn-v2x/traces/leo-relay-fcd.csv";
    std::string outputDir = "ntn-v2x-leo-relay-output";
    std::string v2v = "p2p";
    std::string uplink = "p2p";
    uint32_t pc5Background = 20;
    double relayUeTxDbm = -1.0;   // <= 0: TR 38.821 Set-1 handheld 23 dBm
    double relayUeGainDbi = 0.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("bsmHz", "Basic-safety-message rate (Hz)", bsmHz);
    cmd.AddValue("bsmBytes", "BSM payload size (bytes)", bsmBytes);
    cmd.AddValue("leoAltKm", "LEO altitude (km)", leoAltKm);
    cmd.AddValue("satSpeed",
                 "IGNORED: the SGP4 orbit at --leoAltKm sets the satellite speed",
                 satSpeed);
    cmd.AddValue("relayDriftMps",
                 "IGNORED: vehicle motion comes from the FCD trace (--fcdTrace)",
                 relayDriftMps);
    cmd.AddValue("maxV2vRange", "Max V2V range for relay (m)", maxV2vRangeM);
    cmd.AddValue("minDirectSnr", "Min direct SNR before relaying (dB)", minDirectSnrDb);
    cmd.AddValue("veh0BlockageDb",
                 "NLOS blockage (dB) on veh0's direct LEO link (3GPP-style shadowing) so "
                 "the relay engine selects the V2V->peer->LEO path",
                 veh0BlockageDb);
    cmd.AddValue("linkCapacityMbps", "Per-hop P2P capacity (Mbps)", linkCapacityMbps);
    cmd.AddValue("fcdTrace",
                 "Path to an FCD-format CSV trace (time,vehid,x,y,z,speed) with rows "
                 "for veh0 and veh1. REQUIRED. The shipped trace is a synthetic "
                 "constant-speed CSV fixture; a CSV converted from SUMO fcd-export "
                 "also works (loader reads a CSV dialect, not native fcd-output XML).",
                 fcdTrace);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("v2v", "V2V hop: p2p (default, gated P2P) or pc5 (NR sidelink Mode 2)", v2v);
    cmd.AddValue("uplink", "Relay uplink: p2p (default) or nr (measured NR Uu UL)", uplink);
    cmd.AddValue("pc5Background", "Other PC5 vehicles sending BSMs (pc5 mode)", pc5Background);
    cmd.AddValue("relayUeTxDbm", "Relay UE Tx power (dBm); <= 0 keeps Set-1 23 dBm", relayUeTxDbm);
    cmd.AddValue("relayUeGainDbi", "Relay UE antenna element gain (dBi)", relayUeGainDbi);
    cmd.Parse(argc, argv);
    NS_ABORT_MSG_IF((v2v == "pc5") != (uplink == "nr"),
                    "supported combinations: --v2v=p2p --uplink=p2p (default) or --v2v=pc5 "
                    "--uplink=nr");
    if (v2v == "pc5")
    {
        g_simTime = simSeconds;
        return RunPc5Nr(simSeconds, bsmHz, bsmBytes, leoAltKm, fcdTrace, outputDir, pc5Background,
                        relayUeTxDbm, relayUeGainDbi);
    }
    g_maxV2vRangeM = maxV2vRangeM;
    g_minDirectSnrDb = minDirectSnrDb;
    g_simTime = simSeconds;

    NodeContainer nodes;
    nodes.Create(4); // 0=veh0 1=veh1 2=sat 3=server

    // Vehicles ride an FCD-format CSV trace supplied via --fcdTrace
    // (trace-replay mode), exactly like ntn-v2x-rural-highway, one mobility
    // source for all four v2x examples. The shipped trace is a synthetic
    // constant-speed FCD-format CSV fixture (not a SUMO microsimulation; the
    // loader reads a CSV dialect, not SUMO's native fcd-output XML). A CSV
    // converted from a SUMO fcd-export (`time,vehid,x,y,z,speed` with rows for
    // veh0 and veh1) may be supplied instead.
    if (fcdTrace.empty())
    {
        NS_FATAL_ERROR("--fcdTrace is required: supply an FCD-format CSV trace "
                       "(time,vehid,x,y,z,speed) with rows for veh0 and veh1. "
                       "The shipped trace is a synthetic constant-speed CSV fixture; "
                       "a CSV converted from SUMO fcd-export also works.");
    }
    g_bridge = CreateObject<ntnv2x::SumoTraciBridge>();
    if (!g_bridge->LoadFcdTrace(fcdTrace))
    {
        NS_FATAL_ERROR("could not load FCD trace " << fcdTrace);
    }
    // V2X-6: ConstantVelocity, not ConstantPosition.
    //
    // These were ConstantPositionMobilityModel, which cannot hold a velocity,
    // so GetVelocity() returned (0,0,0) no matter what SUMO reported. Every
    // BSM this example put on the wire therefore carried speed 0 and heading
    // atan2(0,0) = 0 - two of the three SAE J2735 Part I fields the module's
    // own headline claims to populate from real mobility. The bridge now
    // derives velocity from consecutive FCD samples and SUMO's speed column,
    // and needs a model that can carry it.
    Ptr<ConstantVelocityMobilityModel> veh0 = CreateObject<ConstantVelocityMobilityModel>();
    veh0->SetPosition(Vector(0, 0, 1.5));
    nodes.Get(0)->AggregateObject(veh0);
    g_veh0 = veh0;
    g_bridge->RegisterVehicle("veh0", veh0);
    Ptr<ConstantVelocityMobilityModel> veh1 = CreateObject<ConstantVelocityMobilityModel>();
    veh1->SetPosition(Vector(50, 0, 1.5));
    nodes.Get(1)->AggregateObject(veh1);
    g_veh1 = veh1;
    g_bridge->RegisterVehicle("veh1", veh1);
    // Real SGP4 orbit projected into the scenario's local ENU frame: the
    // satellite passes overhead near t=0 and recedes with genuine orbital
    // dynamics (no straight-line placeholder).
    ns3::ntncon::WalkerConfig wcfgSat;
    wcfgSat.num_planes = 1;
    wcfgSat.total_sats = 80;
    wcfgSat.altitude_km = leoAltKm;
    wcfgSat.inclination_deg = 53.0;
    wcfgSat.epoch_unix_s = 1735689600.0;
    const auto satElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfgSat);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElementsSgp4(satElements[0]);
    double satSubLat, satSubLon, satSubAlt;
    satSgp4->GetGeodetic(satSubLat, satSubLon, satSubAlt);
    Ptr<NtnEnuProjectionMobilityModel> sat = CreateObject<NtnEnuProjectionMobilityModel>();
    sat->SetSource(satSgp4);
    sat->SetReference(satSubLat, satSubLon, 0.0);
    nodes.Get(2)->AggregateObject(sat);
    g_sat = sat;

    g_relay = CreateObject<ntnv2x::V2xLeoRelay>();
    g_relay->SetSatellite(sat);
    g_relay->SetMaxV2vRangeM(maxV2vRangeM);
    g_relay->SetMinDirectSnrDb(minDirectSnrDb);
    g_relay->RegisterVehicle("veh0", veh0);
    g_relay->RegisterVehicle("veh1", veh1);
    // veh0 is the shadowed vehicle: a 3GPP-style NLOS blockage drops its direct
    // LEO SNR below the threshold, so EvaluateAll() selects the relay path
    // through veh1 (clear sky), which is exactly the data-plane chain wired
    // below. The route decision is therefore the engine's, not a hard-coded
    // assumption.
    g_relay->SetVehicleBlockageDb("veh0", veh0BlockageDb);

    InternetStackHelper internet;
    internet.Install(nodes);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate",
                           DataRateValue(DataRate(static_cast<uint64_t>(linkCapacityMbps * 1e6))));
    p2p.SetChannelAttribute("Delay", TimeValue(MicroSeconds(10)));
    Ipv4AddressHelper ipv4;

    NetDeviceContainer dV2v = p2p.Install(NodeContainer(nodes.Get(0), nodes.Get(1)));
    g_emV2v = CreateObject<RateErrorModel>();
    g_emV2v->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    g_emV2v->SetRate(0.0);
    dV2v.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(g_emV2v));
    g_chV2v = DynamicCast<PointToPointChannel>(dV2v.Get(0)->GetChannel());
    ipv4.SetBase("10.60.1.0", "255.255.255.0");
    ipv4.Assign(dV2v);

    NetDeviceContainer dUp = p2p.Install(NodeContainer(nodes.Get(1), nodes.Get(2)));
    g_emUplink = CreateObject<RateErrorModel>();
    g_emUplink->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    g_emUplink->SetRate(0.0);
    dUp.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(g_emUplink));
    g_chUplink = DynamicCast<PointToPointChannel>(dUp.Get(0)->GetChannel());
    ipv4.SetBase("10.60.2.0", "255.255.255.0");
    ipv4.Assign(dUp);

    NetDeviceContainer dFeeder = p2p.Install(NodeContainer(nodes.Get(2), nodes.Get(3)));
    g_chFeeder = DynamicCast<PointToPointChannel>(dFeeder.Get(0)->GetChannel());
    ipv4.SetBase("10.60.3.0", "255.255.255.0");
    Ipv4InterfaceContainer iFeeder = ipv4.Assign(dFeeder);

    // GAP V2 FIX (the "direct decision -> PDR 0" bug): give veh0 a DIRECT link
    // to the satellite, so when the relay engine decides direct-to-LEO the
    // packets have a path that exists. Previously the only veh0 egress was the
    // V2V link, so a "direct" decision closed the V2V gate and dropped every
    // packet -- the engine choosing the BETTER link produced TOTAL loss.
    NetDeviceContainer dDirect = p2p.Install(NodeContainer(nodes.Get(0), nodes.Get(2)));
    g_emDirect = CreateObject<RateErrorModel>();
    g_emDirect->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    g_emDirect->SetRate(0.0);
    dDirect.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(g_emDirect));
    g_chDirect = DynamicCast<PointToPointChannel>(dDirect.Get(0)->GetChannel());
    ipv4.SetBase("10.60.4.0", "255.255.255.0");
    ipv4.Assign(dDirect);
    // veh0's egress interfaces: 1 = V2V (to veh1), 2 = direct (to sat). The Tick
    // brings exactly one UP per the engine's decision and recomputes routing, so
    // veh0's packets follow the CHOSEN path -- routing tracks the decision, not
    // a fixed chain.
    g_veh0Ipv4 = nodes.Get(0)->GetObject<Ipv4>();
    g_ifV2v = 1;
    g_ifDirect = 2;

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    const uint16_t port = 7600;
    g_sink = CreateObject<NtnOranSink>();
    g_sink->SetAttribute("Local",
                         AddressValue(InetSocketAddress(Ipv4Address::GetAny(), port)));
    nodes.Get(3)->AddApplication(g_sink);
    g_sink->SetStartTime(Seconds(0.0));
    g_sink->SetStopTime(Seconds(simSeconds));

    // BSM cadence: periodic V2X safety messages (5QI 82, URLLC class), each
    // carrying a REAL SAE J2735 BSM Part I populated from veh0's live mobility
    // (position + speed + heading) rather than opaque padding. The J2735 core
    // fills the packet body; the in-band NtnOranPayloadHeader is still on top,
    // so PDR/delay/jitter stay measured end-to-end.
    Ptr<NtnOranApplication> src = CreateObject<NtnOranApplication>();
    src->SetRemote(InetSocketAddress(iFeeder.GetAddress(1), port));
    src->SetProfile(NtnOranApplication::URLLC_PERIODIC);
    src->SetAttribute("PacketSize", UintegerValue(bsmBytes));
    src->SetAttribute("Period", TimeValue(Seconds(1.0 / bsmHz)));
    src->SetFlowIdentity(/*5qi*/ 82, /*sst*/ 2, /*sd*/ 0x000001, /*src*/ 0, /*dst*/ 3);
    g_veh0Mob = nodes.Get(0)->GetObject<MobilityModel>();
    src->SetPayloadBuilder(
        MakeCallback(+[](Buffer::Iterator it, uint32_t bodyBytes) {
            // Fill the leading J2735 BSM core; the rest stays padding (Part II).
            ntnv2x::NtnV2xBsmHeader bsm;
            const Vector p = g_veh0Mob ? g_veh0Mob->GetPosition() : Vector(0, 0, 0);
            const Vector v = g_veh0Mob ? g_veh0Mob->GetVelocity() : Vector(0, 0, 0);
            // ENU/local metres -> pseudo lat/lon degrees for the BSM fields; the
            // absolute datum is arbitrary for a relay-latency study, the point is
            // that the values move with the vehicle.
            const double latDeg = p.y / 111320.0;
            const double lonDeg = p.x / 111320.0;
            const double speed = std::sqrt(v.x * v.x + v.y * v.y);
            const double heading = std::fmod(std::atan2(v.x, v.y) * 180.0 / M_PI + 360.0, 360.0);
            const uint16_t secMark =
                static_cast<uint16_t>(std::llround(Simulator::Now().GetMilliSeconds()) % 60000);
            bsm.SetFromState(g_bsmCnt++, /*stationId=*/0x00000001u, secMark, latDeg, lonDeg, p.z,
                             speed, heading);
            if (bodyBytes >= bsm.GetSerializedSize())
            {
                bsm.Serialize(it);
            }
        }));
    nodes.Get(0)->AddApplication(src);
    src->SetStartTime(Seconds(1.0));
    src->SetStopTime(Seconds(simSeconds));

    std::printf("# ntn-v2x-leo-relay-traffic (veh0 --V2V--> veh1 --uplink--> LEO --> server)\n"
                "#   sim=%.0fs bsm=%.0fHz x %uB leoAlt=%.0fkm (SGP4) maxV2vRange=%.0fm "
                "minDirectSnr=%.1fdB veh0Blockage=%.1fdB\n",
                simSeconds, bsmHz, bsmBytes, leoAltKm, maxV2vRangeM, minDirectSnrDb,
                veh0BlockageDb);
    (void)satSpeed;
    (void)relayDriftMps;

    // outputDir was parsed and never used; persist the per-tick decisions.
    {
        std::error_code ec;
        std::filesystem::create_directories(outputDir, ec);
        g_csv.open(outputDir + "/ntn-v2x-leo-relay-traffic.csv");
        if (g_csv.is_open())
        {
            g_csv << "time_s,v2v_range_m,uplink_elev_deg,route,peer,veh0_direct_snr_db,"
                     "serving_snr_db,uplink_up,rx_bytes_last_s\n";
        }
    }

    // First tick at t=0, before the first BSM at t=1 s: it used to run at t=2 s,
    // so the first second of BSMs crossed every hop with the 10 us placeholder
    // delay and ungated links.
    Simulator::Schedule(Seconds(0.0), &Tick);
    Simulator::Stop(Seconds(simSeconds + 1));
    // Globe scene of this run's own geometry (ntn-dataset@1, plan L4).
    ns3::ntnobs::StartSceneFromNodeList(outputDir);
    Simulator::Run();

    // KPIs measured from in-band NtnOranPayloadHeader primitives at the sink.
    const uint64_t txP = src->GetTxPackets();
    const uint64_t rxP = g_sink->GetRxPackets();
    std::printf("# === summary ===  BSM txPackets=%lu rxPackets=%lu PDR=%.2f%%  "
                "mean e2e delay=%.2f ms jitter=%.3f ms (real range delays through the relay)\n",
                (unsigned long)txP, (unsigned long)rxP, txP ? 100.0 * rxP / txP : 0.0,
                g_sink->GetMeanDelayMs(), g_sink->GetMeanJitterMs());

    // V2X-6: report what the BSM headers actually carried. Until this fix the
    // answer was speed 0 and heading 0 on every packet, because the vehicles
    // were registered against a mobility model that cannot hold a velocity and
    // the FCD speed column was parsed and discarded.
    {
        const Vector vFinal = g_veh0Mob ? g_veh0Mob->GetVelocity() : Vector(0, 0, 0);
        const double speedFinal = std::sqrt(vFinal.x * vFinal.x + vFinal.y * vFinal.y);
        std::cout << "#   BSM mobility: veh0 final speed=" << speedFinal << " m/s heading="
                  // compass heading (clockwise from north), the J2735 convention
                  // the BSM itself uses, not the math angle from +x
                  << (speedFinal > 0.0
                          ? std::fmod(std::atan2(vFinal.x, vFinal.y) * 180.0 / M_PI + 360.0, 360.0)
                          : 0.0)
                  << " deg | FCD velocity samples applied="
                  << g_bridge->VelocitySamplesApplied()
                  << " dropped=" << g_bridge->VelocitySamplesDropped() << "\n";
    }
    if (g_csv.is_open())
    {
        g_csv.close();
    }
    // ntn-dataset@1: the default (geometric relay) path is not built on the real-stack helper.
    ns3::ntnobs::WriteDatasetRecord(outputDir, "ntn-v2x-leo-relay-traffic", "measured", argc, argv);
    Simulator::Destroy();
    return 0;
}
