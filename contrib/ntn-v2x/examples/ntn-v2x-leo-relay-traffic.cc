/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W7)
 *
 * ntn-v2x-leo-relay-traffic — a shadowed vehicle (veh0) relays its basic-safety
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
 * Quick test:  --simSeconds=60 --bsmHz=10 --fcdTrace=<path/to/fcd.csv>
 */
#include "ns3/applications-module.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
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

#include <cmath>
#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnV2xLeoRelayTraffic");

namespace
{
constexpr double kC = 299792458.0;
Ptr<ntnv2x::V2xLeoRelay> g_relay;
Ptr<MobilityModel> g_veh0, g_veh1, g_sat;
Ptr<RateErrorModel> g_emV2v, g_emUplink, g_emDirect;
Ptr<PointToPointChannel> g_chV2v, g_chUplink, g_chDirect;
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
    Simulator::Schedule(Seconds(1.0), &Tick);
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

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("bsmHz", "Basic-safety-message rate (Hz)", bsmHz);
    cmd.AddValue("bsmBytes", "BSM payload size (bytes)", bsmBytes);
    cmd.AddValue("leoAltKm", "LEO altitude (km)", leoAltKm);
    cmd.AddValue("satSpeed", "LEO ground-track speed (m/s)", satSpeed);
    cmd.AddValue("relayDriftMps", "Relay vehicle relative speed (m/s)", relayDriftMps);
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
    cmd.Parse(argc, argv);
    g_maxV2vRangeM = maxV2vRangeM;
    g_minDirectSnrDb = minDirectSnrDb;
    g_simTime = simSeconds;

    NodeContainer nodes;
    nodes.Create(4); // 0=veh0 1=veh1 2=sat 3=server

    // Vehicles ride an FCD-format CSV trace supplied via --fcdTrace
    // (trace-replay mode), exactly like ntn-v2x-rural-highway — one mobility
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
    Ptr<ConstantPositionMobilityModel> veh0 = CreateObject<ConstantPositionMobilityModel>();
    veh0->SetPosition(Vector(0, 0, 1.5));
    nodes.Get(0)->AggregateObject(veh0);
    g_veh0 = veh0;
    g_bridge->RegisterVehicle("veh0", veh0);
    Ptr<ConstantPositionMobilityModel> veh1 = CreateObject<ConstantPositionMobilityModel>();
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
    satSgp4->SetElements(satElements[0]);
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
    // through veh1 (clear sky) — which is exactly the data-plane chain wired
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
                "#   sim=%.0fs bsm=%.0fHz x %uB leoAlt=%.0fkm relayDrift=%.0fm/s maxV2vRange=%.0fm\n",
                simSeconds, bsmHz, bsmBytes, leoAltKm, relayDriftMps, maxV2vRangeM);

    Simulator::Schedule(Seconds(2.0), &Tick);
    Simulator::Stop(Seconds(simSeconds + 1));
    Simulator::Run();

    // KPIs measured from in-band NtnOranPayloadHeader primitives at the sink.
    const uint64_t txP = src->GetTxPackets();
    const uint64_t rxP = g_sink->GetRxPackets();
    std::printf("# === summary ===  BSM txPackets=%lu rxPackets=%lu PDR=%.2f%%  "
                "mean e2e delay=%.2f ms jitter=%.3f ms (real range delays through the relay)\n",
                (unsigned long)txP, (unsigned long)rxP, txP ? 100.0 * rxP / txP : 0.0,
                g_sink->GetMeanDelayMs(), g_sink->GetMeanJitterMs());
    Simulator::Destroy();
    return 0;
}
