/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ntn-v2x-real-stack, Phase 2 of 2026-06 protocol-fidelity audit
 * (RAN recipe).
 *
 * Audit finding for ntn-v2x: the direct-vs-relay decision (V2xLeoRelay) was
 * driven entirely by V2xLeoDirect::Compute(), a closed-form free-space SNR
 * formula. No packet ever crossed a real radio, so "which vehicle has the best
 * LEO uplink" was a number, not a measurement.
 *
 * Here every vehicle is a UE on a REAL mmwave NR NTN cell (NtnRealStackHelper:
 * SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC), so each vehicle's direct
 * LEO link quality is the MEASURED per-UE DL SINR off the mmwave PHY trace. The
 * V2X-specific NLOS condition (foliage / canyon / blockage) is modelled the way
 * 3GPP does, an additional per-vehicle blockage loss on top of the measured
 * baseline. A vehicle whose measured-minus-blockage direct SINR falls below the
 * uplink threshold relays through the nearest in-range peer that still has a
 * good measured link (the V2V hop is a genuine short LOS free-space link). A
 * mid-run elevation descent collapses the measured baseline so the relay
 * decisions actually flip over the pass.
 *
 * Wave 2 M3 (Nov 2026): blockage is now CAUSED BY GEOMETRY and applied ON
 * THE RADIO. The street is lined with building facades (a seeded random city
 * block pattern) and a truck drives in the next lane; every one is a
 * TR 38.901 clause 7.6.4.2 blockage-model-B screen (NtnScreenBlockageModel,
 * thz-ntn) installed per RB on the NR spectrum channel through
 * NtnRealStackHelper::AddSpectrumChannelLoss. The loss therefore reaches the
 * SSB/PSS (measured RSRP), PDCCH and PDSCH (measured SINR, TBLER) of exactly
 * the vehicles whose line of sight to the SGP4 satellite crosses a facade.
 *
 * Relay selection follows the Rel-17 UE-to-network relay rules on MEASURED
 * quantities: a vehicle whose L3-filtered serving SS-RSRP (periodic
 * measurement reports, TS 38.331 5.5.4) is below threshHighRemote becomes a
 * remote UE (TS 38.304 clause 8.1 / TS 38.331 SL-RemoteUE-Config); a peer
 * whose RSRP is above threshLowRelay can act as relay (SL-RelayUE-Config),
 * and the PC5 link must decode (SL-RSRP over the subchannel noise floor above
 * the PSSCH decode SINR). The thresholds are not free knobs: threshHighRemote
 * is the RSRP at which the per-RE SNR equals --minDirectSnr, i.e.
 * -174 + 10 log10(SCS) + NF + minDirectSnr dBm, and threshLowRelay adds a
 * 3 dB hysteresis.
 *
 * A remote UE is gated off Uu. Its traffic is delivered as an L3 UE-to-network
 * relay (TS 23.304 Rel-17, 5.5.1.3 / TS 38.300 16.12): the network sends it to
 * the relay UE over the relay's own measured Uu bearer, the relay queues it
 * and forwards it over the NR PC5 Mode-2 sidelink (NtnSlChannel: sensing,
 * SPS reservation, half duplex, collisions, TR 37.885 path loss) to the remote
 * UE. Abstracted: PC5-RRC/PC5-S discovery and link setup (assumed done, the
 * relay is chosen from measured RSRP), the relay's IP routing (the relay's
 * application queue stands in for its L3 forwarding), and the Uu RLC/PDCP
 * of the remote UE's PDU session (the relay bearer carries it).
 *
 * The old parity-flag blockage (odd vehicles minus a constant inside the
 * decision only) is kept as --scripted=1 for regression.
 *
 * Usage:
 *   ./ns3 run "ntn-v2x-real-stack --duration=20 --numVehicles=8"
 *   ./ns3 run "ntn-v2x-real-stack --duration=20 --numVehicles=8 --scripted=1"
 */

#include "ns3/core-module.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/v2x-leo-direct.h"
#include "ns3/ntn-nr-sidelink.h"
#include "ns3/ntn-oran-payload-header.h"
#include "ns3/ntn-screen-blockage-model.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/ntn-spectrum-seam-model.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy.h"

#include <deque>
#include <set>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

#include "ns3/ntn-oran-application.h"
using namespace ns3;
using namespace ns3::ntnv2x;

NS_LOG_COMPONENT_DEFINE("NtnV2xRealStack");

namespace
{
NtnRealStackHelper* g_rs = nullptr;
NodeContainer g_vehicles;
// V2X-2: per-vehicle flow handles and decision state, so the relay decision can
// actuate rather than only counting.
enum class Decision : uint8_t
{
    Direct,
    Relay,
    Outage
};
std::vector<ApplicationContainer> g_vehFlows;
std::vector<Decision> g_vehDecision;
std::vector<bool> g_vehOnAir;
uint32_t g_gatedOff = 0;
uint32_t g_gatedOn = 0;
std::vector<double> g_blockageDb; // per-vehicle NLOS blockage (0 = LOS)
double g_simTime = 20.0;
double g_minDirectSnrDb = 6.0;
double g_maxV2vRangeM = 1500.0;
double g_v2vHopLossDb = 3.0; // short LOS sidelink hop penalty
uint64_t g_directDecisions = 0;
uint64_t g_relayDecisions = 0;
uint64_t g_outageDecisions = 0;

double
Dist(const Vector& a, const Vector& b)
{
    double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void
RelayTick()
{
    const double t = Simulator::Now().GetSeconds();
    if (t >= g_simTime)
    {
        return;
    }
    const uint32_t n = g_vehicles.GetN();

    // MEASURED direct LEO SINR per vehicle, minus its NLOS blockage condition.
    std::vector<double> directSinr(n, std::nan(""));
    for (uint32_t v = 0; v < n; ++v)
    {
        double meas = g_rs->GetUeRecentSinrDb(v);
        if (!std::isnan(meas))
        {
            directSinr[v] = meas - g_blockageDb[v];
        }
    }

    uint32_t direct = 0, relay = 0, outage = 0;
    for (uint32_t v = 0; v < n; ++v)
    {
        if (std::isnan(directSinr[v]))
        {
            continue;
        }
        if (directSinr[v] >= g_minDirectSnrDb)
        {
            ++direct; // good measured direct link
            g_vehDecision[v] = Decision::Direct;
            continue;
        }
        // Shadowed: look for the best in-range peer with a good MEASURED link.
        const Vector pv = g_vehicles.Get(v)->GetObject<MobilityModel>()->GetPosition();
        double bestPeerSinr = -1e9;
        for (uint32_t p = 0; p < n; ++p)
        {
            if (p == v || std::isnan(directSinr[p]))
            {
                continue;
            }
            const Vector pp = g_vehicles.Get(p)->GetObject<MobilityModel>()->GetPosition();
            if (Dist(pv, pp) > g_maxV2vRangeM)
            {
                continue;
            }
            double viaSinr = directSinr[p] - g_v2vHopLossDb; // relay bottleneck
            bestPeerSinr = std::max(bestPeerSinr, viaSinr);
        }
        if (bestPeerSinr >= g_minDirectSnrDb)
        {
            ++relay; // relayed through a peer on the MEASURED-best link
            g_vehDecision[v] = Decision::Relay;
        }
        else
        {
            ++outage; // neither direct nor any peer clears the threshold
            g_vehDecision[v] = Decision::Outage;
        }
    }
    g_directDecisions += direct;
    g_relayDecisions += relay;
    g_outageDecisions += outage;

    // V2X-2 FIX (2026-08-25): ACTUATE the decision.
    //
    // This loop used to terminate in ++direct / ++relay / ++outage and nothing
    // else: no flow was gated, rerouted, started or stopped, so the reported
    // split described a calculation rather than anything that happened to a
    // packet. A vehicle the model has just declared to be in outage now
    // genuinely stops transmitting, and one that recovers resumes, so the
    // measured per-UE delivery reflects the decision instead of running
    // independently of it.
    for (uint32_t v = 0; v < n && v < g_vehFlows.size(); ++v)
    {
        const bool onAir = (g_vehDecision[v] != Decision::Outage);
        if (onAir == g_vehOnAir[v])
        {
            continue; // no change, do not churn the applications
        }
        for (uint32_t a = 0; a < g_vehFlows[v].GetN(); ++a)
        {
            Ptr<NtnOranApplication> app =
                DynamicCast<NtnOranApplication>(g_vehFlows[v].Get(a));
            if (app)
            {
                app->SetTransmitEnabled(onAir);
            }
        }
        g_vehOnAir[v] = onAir;
        if (!onAir)
        {
            ++g_gatedOff;
        }
        else
        {
            ++g_gatedOn;
        }
    }

    static double s_lastPrint = -1.0;
    if (t - s_lastPrint >= 4.0)
    {
        std::printf("  %5.1fs  direct=%u relay=%u outage=%u  (measured UE0 SINR=%.1f dB)\n",
                    t, direct, relay, outage, g_rs->GetUeRecentSinrDb(0));
        s_lastPrint = t;
    }
    Simulator::Schedule(MilliSeconds(500), &RelayTick);
}

// ===========================================================================
// Wave 2 M3: geometric blockage on the radio + measured-RSRP relay selection
// + L3 UE-to-network relay over PC5 (TS 23.304 Rel-17).
// ===========================================================================
struct Fwd
{
    uint32_t dest;      //!< remote UE index
    uint32_t relay;     //!< relay UE index
    Time relayRx;       //!< arrival at the relay over Uu
    uint64_t txNs;      //!< network send time (payload header), 0 if unknown
};

struct VehStats
{
    double sumLossDb{0.0};
    uint32_t ticks{0};
    uint32_t blockedTicks{0};
    double sumRsrp{0.0};
    uint32_t nRsrp{0};
    double sumSinrLos{0.0};
    uint32_t nSinrLos{0};
    double sumSinrBlk{0.0};
    uint32_t nSinrBlk{0};
    double sumLossAtSinrBlk{0.0};
    uint64_t lastRxPkts{0};
    uint64_t uuRxWhileGated{0};  //!< Uu rx later than one tick after gating (must be 0)
    uint64_t uuRxDrainAfterGate{0}; //!< Uu rx in the first tick after gating (in flight / RLC drain)
    uint32_t gatedTicks{0};
    double prevLoss1{-1.0};
    double prevLoss2{-1.0};
    uint32_t relayTicks{0};
    uint32_t outageTicks{0};
    uint32_t servedAsRelay{0};
};

struct GeoState
{
    Ptr<NtnScreenBlockageModel> blk;
    Ptr<MobilityModel> satMob;
    double fcHz{2.185e9};
    Ptr<NtnSlChannel> sl;
    std::vector<Ptr<NtnSlUeMac>> macs;
    std::vector<std::deque<Ptr<Packet>>> relayQ;
    std::map<uint64_t, Fwd> fwd;
    std::vector<int> relayOf;
    std::map<std::pair<uint32_t, uint32_t>, ApplicationContainer> proxy;
    std::vector<VehStats> st;
    double threshHighRemoteDbm{-116.0};
    double threshLowRelayDbm{-113.0};
    double pc5MarginDb{3.0};
    uint32_t queueCap{100};
    // relayed-traffic KPIs
    uint64_t enqueued{0};
    uint64_t dropped{0};
    uint64_t delivered{0};
    double e2eSumMs{0.0};
    double e2eMaxMs{0.0};
    double pc5SumMs{0.0};
    uint64_t e2eN{0};
    // RSRP-vs-geometry check: (measured RSRP deficit vs LOS mean) - loss
    double rsrpErrSum{0.0};
    double rsrpErrAbsSum{0.0};
    double rsrpLossSum{0.0};
    uint32_t rsrpErrN{0};
    // SINR rows for the CSV
    std::vector<std::string> rows;
};

std::unique_ptr<GeoState> g_geo;

void
OnRelayUuRx(uint32_t remote, uint32_t relay, Ptr<const Packet> p, const Address&)
{
    GeoState& g = *g_geo;
    if (g.relayQ[relay].size() >= g.queueCap)
    {
        ++g.dropped;
        return;
    }
    Ptr<Packet> c = p->Copy();
    uint64_t txNs = 0;
    NtnOranPayloadHeader h;
    if (c->GetSize() >= h.GetSerializedSize())
    {
        c->PeekHeader(h);
        txNs = h.GetTxTimestampNs();
    }
    g.fwd[c->GetUid()] = Fwd{remote, relay, Simulator::Now(), txNs};
    g.relayQ[relay].push_back(c);
    ++g.enqueued;
}

Ptr<Packet>
SlTxPacket(uint32_t id)
{
    GeoState& g = *g_geo;
    if (!g.relayQ[id].empty())
    {
        Ptr<Packet> p = g.relayQ[id].front();
        g.relayQ[id].pop_front();
        return p;
    }
    // Nothing to forward: the vehicle's periodic status broadcast (190 B,
    // SAE J2735 BSM size) keeps its SPS reservation alive.
    return Create<Packet>(190);
}

void
SlRx(uint32_t self, uint32_t from, Ptr<Packet> pkt)
{
    GeoState& g = *g_geo;
    auto it = g.fwd.find(pkt->GetUid());
    if (it == g.fwd.end() || it->second.dest != self || it->second.relay != from)
    {
        return;
    }
    const Time now = Simulator::Now();
    ++g.delivered;
    const double pc5 = (now - it->second.relayRx).GetSeconds() * 1e3;
    g.pc5SumMs += pc5;
    if (it->second.txNs > 0)
    {
        const double e2e = (now.GetNanoSeconds() - static_cast<int64_t>(it->second.txNs)) / 1e6;
        g.e2eSumMs += e2e;
        g.e2eMaxMs = std::max(g.e2eMaxMs, e2e);
        ++g.e2eN;
    }
    g.fwd.erase(it);
}

void
SetFlow(ApplicationContainer& apps, bool on)
{
    for (uint32_t a = 0; a < apps.GetN(); ++a)
    {
        Ptr<NtnOranApplication> app = DynamicCast<NtnOranApplication>(apps.Get(a));
        if (app)
        {
            app->SetTransmitEnabled(on);
        }
    }
}

void
GeoRelayTick()
{
    const double t = Simulator::Now().GetSeconds();
    if (t >= g_simTime)
    {
        return;
    }
    GeoState& g = *g_geo;
    const uint32_t n = g_vehicles.GetN();

    // Geometry (printed reference) and MEASURED RSRP / SINR per vehicle.
    std::vector<double> loss(n, 0.0);
    std::vector<double> rsrp(n, std::nan(""));
    for (uint32_t v = 0; v < n; ++v)
    {
        Ptr<MobilityModel> vm = g_vehicles.Get(v)->GetObject<MobilityModel>();
        loss[v] = g.blk->LossDb(PeekPointer(g.satMob), PeekPointer(vm), g.fcHz);
        rsrp[v] = g_rs->GetUeServingRsrpDbm(v);
        VehStats& s = g.st[v];
        s.sumLossDb += loss[v];
        ++s.ticks;
        if (loss[v] > 3.0)
        {
            ++s.blockedTicks;
        }
        if (!std::isnan(rsrp[v]))
        {
            s.sumRsrp += rsrp[v];
            ++s.nRsrp;
        }
        const uint64_t rx = g_rs->GetUeRxPackets(v);
        const uint64_t dRx = rx - s.lastRxPkts;
        s.lastRxPkts = rx;
        if (!g_vehOnAir[v])
        {
            // Packets already queued at the gNB RLC or in flight when the
            // source stopped arrive during the first tick; later, nothing may.
            if (s.gatedTicks == 0)
            {
                s.uuRxDrainAfterGate += dRx;
            }
            else
            {
                s.uuRxWhileGated += dRx;
            }
            ++s.gatedTicks;
        }
        else
        {
            s.gatedTicks = 0;
            // Fresh PDSCH samples exist: the MEASURED SINR of this vehicle.
            const double sinr = (dRx > 0) ? g_rs->GetUeRecentSinrDb(v) : std::nan("");
            if (!std::isnan(sinr))
            {
                if (loss[v] < 0.5)
                {
                    s.sumSinrLos += sinr;
                    ++s.nSinrLos;
                }
                else if (loss[v] > 3.0)
                {
                    s.sumSinrBlk += sinr;
                    ++s.nSinrBlk;
                    s.sumLossAtSinrBlk += loss[v];
                }
            }
        }
    }
    // RSRP deficit of blocked vehicles vs the LOS vehicles of the same tick,
    // on ticks where the geometry has been steady for 0.5 s (the reported RSRP
    // is L3 filtered, TS 38.331 5.5.3.2, and reported every 120 ms).
    std::vector<bool> steady(n, false);
    for (uint32_t v = 0; v < n; ++v)
    {
        VehStats& s = g.st[v];
        steady[v] = s.prevLoss1 >= 0.0 && std::abs(loss[v] - s.prevLoss1) < 1.0;
        s.prevLoss2 = s.prevLoss1;
        s.prevLoss1 = loss[v];
    }
    double losRsrp = 0.0;
    uint32_t nLos = 0;
    for (uint32_t v = 0; v < n; ++v)
    {
        if (loss[v] < 0.5 && steady[v] && !std::isnan(rsrp[v]))
        {
            losRsrp += rsrp[v];
            ++nLos;
        }
    }
    if (nLos > 0)
    {
        losRsrp /= nLos;
        for (uint32_t v = 0; v < n; ++v)
        {
            if (loss[v] > 3.0 && steady[v] && !std::isnan(rsrp[v]))
            {
                const double err = (losRsrp - rsrp[v]) - loss[v];
                g.rsrpErrSum += err;
                g.rsrpErrAbsSum += std::abs(err);
                g.rsrpLossSum += loss[v];
                ++g.rsrpErrN;
            }
        }
    }

    // TS 38.304 8.1 / TS 38.331 SL-RemoteUE-Config: a UE below
    // threshHighRemote on its MEASURED Uu RSRP is a remote UE.
    std::vector<bool> remote(n, false);
    for (uint32_t v = 0; v < n; ++v)
    {
        if (!std::isnan(rsrp[v]))
        {
            remote[v] = rsrp[v] < g.threshHighRemoteDbm;
        }
        else
        {
            remote[v] = (g.relayOf[v] >= 0) || (g_vehDecision[v] == Decision::Outage);
        }
    }
    uint32_t direct = 0, relay = 0, outage = 0;
    for (uint32_t v = 0; v < n; ++v)
    {
        if (!remote[v])
        {
            g.relayOf[v] = -1;
            g_vehDecision[v] = Decision::Direct;
            ++direct;
            continue;
        }
        // SL-RelayUE-Config threshLowRelay on the candidate's measured RSRP,
        // and a PC5 link that decodes (SL-RSRP over the subchannel noise).
        Ptr<MobilityModel> vm = g_vehicles.Get(v)->GetObject<MobilityModel>();
        int best = -1;
        double bestRsrp = -1e9;
        for (uint32_t r = 0; r < n; ++r)
        {
            if (r == v || remote[r] || std::isnan(rsrp[r]) || rsrp[r] < g.threshLowRelayDbm ||
                g.proxy.find({v, r}) == g.proxy.end())
            {
                continue;
            }
            Ptr<MobilityModel> rm = g_vehicles.Get(r)->GetObject<MobilityModel>();
            const double slSnr = g.sl->SlRsrpDbm(rm, vm) - g.sl->NoiseFloorDbm();
            if (slSnr < g.sl->GetDecodeSinrDb() + g.pc5MarginDb)
            {
                continue;
            }
            if (rsrp[r] > bestRsrp)
            {
                bestRsrp = rsrp[r];
                best = static_cast<int>(r);
            }
        }
        g.relayOf[v] = best;
        if (best >= 0)
        {
            g_vehDecision[v] = Decision::Relay;
            ++relay;
            ++g.st[v].relayTicks;
            ++g.st[best].servedAsRelay;
        }
        else
        {
            g_vehDecision[v] = Decision::Outage;
            ++outage;
            ++g.st[v].outageTicks;
        }
    }
    g_directDecisions += direct;
    g_relayDecisions += relay;
    g_outageDecisions += outage;

    // Actuate: own Uu flow only when direct; exactly one proxy flow (to the
    // chosen relay) when relayed.
    for (uint32_t v = 0; v < n; ++v)
    {
        const bool onAir = (g_vehDecision[v] == Decision::Direct);
        if (onAir != g_vehOnAir[v])
        {
            SetFlow(g_vehFlows[v], onAir);
            g_vehOnAir[v] = onAir;
            ++(onAir ? g_gatedOn : g_gatedOff);
        }
        for (auto& [key, apps] : g.proxy)
        {
            if (key.first == v)
            {
                SetFlow(apps, g.relayOf[v] == static_cast<int>(key.second));
            }
        }
    }

    for (uint32_t v = 0; v < n; ++v)
    {
        char row[256];
        std::snprintf(row, sizeof(row), "%.1f,%u,%.2f,%.2f,%.2f,%s,%d", t, v, loss[v], rsrp[v],
                      g_rs->GetUeRecentSinrDb(v),
                      g_vehDecision[v] == Decision::Direct
                          ? "direct"
                          : (g_vehDecision[v] == Decision::Relay ? "relay" : "outage"),
                      g.relayOf[v]);
        g.rows.emplace_back(row);
    }

    static double s_lastPrint = -1.0;
    if (t - s_lastPrint >= 2.0)
    {
        std::printf("  %5.1fs  direct=%u relay=%u outage=%u |", t, direct, relay, outage);
        for (uint32_t v = 0; v < n; ++v)
        {
            std::printf(" v%u:%.0fdB/%.0fdBm", v, loss[v], rsrp[v]);
        }
        std::printf("\n");
        s_lastPrint = t;
    }
    Simulator::Schedule(MilliSeconds(500), &GeoRelayTick);
}
} // namespace

int
main(int argc, char* argv[])
{
    double duration = 20.0;
    uint32_t numVehicles = 8;
    double altitudeKm = 550.0;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    double blockageDb = 14.0; // NLOS blockage applied to shadowed vehicles
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    std::string outputDir = "ntn-v2x-real-stack-output";
    bool scripted = false;     // legacy parity-flag blockage inside the decision only
    double streetHalfWidthM = 10.0;
    double bldLenM = 60.0;
    double bldGapM = 20.0;
    double bldMinH = 20.0;
    double bldMaxH = 120.0;
    uint32_t cityStream = 7;
    uint32_t pc5PeriodMs = 5; // half the URLLC packet period, so one remote per relay is below capacity
    std::string blockagePlane = "spectrum";

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("numVehicles", "Number of vehicle-UEs", numVehicles);
    cmd.AddValue("altitude", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm); -1 = backend default", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("blockageDb", "NLOS blockage loss on shadowed vehicles (dB)", blockageDb);
    cmd.AddValue("minDirectSnr", "Minimum direct LEO SINR (dB)", g_minDirectSnrDb);
    cmd.AddValue("maxV2vRange", "Max V2V relay range (m)", g_maxV2vRangeM);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("scripted",
                 "1 = legacy mode: odd vehicles get --blockageDb inside the decision only",
                 scripted);
    cmd.AddValue("streetHalfWidth", "Facade distance from the lane centre (m)", streetHalfWidthM);
    cmd.AddValue("bldLen", "Building block length along the street (m)", bldLenM);
    cmd.AddValue("bldGap", "Cross-street gap between blocks (m)", bldGapM);
    cmd.AddValue("bldMinH", "Lowest facade height (m)", bldMinH);
    cmd.AddValue("bldMaxH", "Highest facade height (m)", bldMaxH);
    cmd.AddValue("cityStream", "RNG stream of the facade-height draw", cityStream);
    cmd.AddValue("blockagePlane",
                 "spectrum: per-RB plugin (AddSpectrumChannelLoss); scalar: one dB per link at the "
                 "carrier (AddExtraPropagationLoss)",
                 blockagePlane);
    cmd.AddValue("pc5PeriodMs", "PC5 SPS reservation period of every vehicle (ms)", pc5PeriodMs);
    cmd.Parse(argc, argv);
    NS_ABORT_MSG_IF(!scripted && radio == "mmwave",
                    "geometric blockage + measured-RSRP relaying need the nr backend "
                    "(per-RB spectrum seam and periodic measurement reports); use --scripted=1 "
                    "for mmwave");
    g_simTime = duration;

    // EIRP default. This used to put 70 dBm (10 kW) of CONDUCTED power on the
    // nr array, about 20 dB above the TR 38.821 Set-1 S-band payload once the
    // array gain is added, which held every vehicle near 30 dB SINR so a 14 dB
    // blockage never pushed one below the 6 dB threshold for most of the run.
    // The nr default is now the TR 38.821 Table 6.1.1.1-1 Set-1 EIRP density
    // (34 dBW/MHz), as ntn-v2x-edge-urllc and ntn-v2x-maritime-ais already use;
    // an explicit --satEirpDbm is still honored as conducted power.
    const bool eirpFromTr38821 = (satEirpDbm < 0.0 && radio != "mmwave");
    if (satEirpDbm < 0.0 && radio == "mmwave")
    {
        satEirpDbm = 55.0;
    }

    std::cout << "\n=== ntn-v2x REAL-STACK (relay decision on MEASURED LEO radio) ===\n"
              << "  " << numVehicles << " vehicle-UEs on a real " << radio << " NR NTN cell\n";
    if (scripted)
    {
        std::cout << "  mode: SCRIPTED (legacy): " << blockageDb
                  << " dB subtracted from odd vehicles' SINR inside the decision only\n";
    }
    else
    {
        std::cout << "  mode: GEOMETRIC: facades + truck as TR 38.901 7.6.4.2 model-B screens "
                     "on the NR spectrum channel (per RB)\n"
                  << "  relay selection on measured RSRP (TS 38.304 8.1), L3 UE-to-network relay "
                     "over PC5 Mode 2 (TS 23.304)\n";
    }
    std::cout << "  duration: " << duration << " s\n\n";

    NodeContainer satNodes;
    satNodes.Create(1);
    g_vehicles.Create(numVehicles);

    // Real SGP4 orbit projected into the scenario's local ENU frame: the
    // satellite passes overhead near t=0 and recedes with genuine orbital
    // dynamics (no straight-line placeholder).
    ns3::ntncon::WalkerConfig wcfgSat;
    wcfgSat.num_planes = 1;
    wcfgSat.total_sats = 80;
    wcfgSat.altitude_km = altitudeKm;
    wcfgSat.inclination_deg = 53.0;
    wcfgSat.epoch_unix_s = 1735689600.0;
    const auto satElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfgSat);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElementsSgp4(satElements[0]);
    double satSubLat, satSubLon, satSubAlt;
    satSgp4->GetGeodetic(satSubLat, satSubLon, satSubAlt);
    Ptr<NtnEnuProjectionMobilityModel> satEnu = CreateObject<NtnEnuProjectionMobilityModel>();
    satEnu->SetSource(satSgp4);
    satEnu->SetReference(satSubLat, satSubLon, 0.0);
    satNodes.Get(0)->AggregateObject(satEnu);
    MobilityHelper mob;
    // Vehicles drive: ConstantVelocity at 27.8 m/s (100 km/h, inside the
    // TR 38.821 Table 6.1.1.1-3 vehicle-mounted UE speed of up to 120 km/h).
    // They were ConstantPosition, i.e. parked cars on a "highway", which the
    // mobility mandate does not allow and which hid Doppler from the radio.
    mob.SetMobilityModel("ns3::ConstantVelocityMobilityModel");

    // Vehicles along a highway, 300 m apart; odd-indexed ones are NLOS (canyon/
    // foliage) and carry the blockage term. Even ones keep line of sight.
    Ptr<ListPositionAllocator> vehPos = CreateObject<ListPositionAllocator>();
    g_blockageDb.assign(numVehicles, 0.0);
    for (uint32_t i = 0; i < numVehicles; ++i)
    {
        vehPos->Add(Vector(300.0 * i, 0.0, 0.0));
        g_blockageDb[i] = (scripted && i % 2 == 1) ? blockageDb : 0.0;
    }
    mob.SetPositionAllocator(vehPos);
    mob.Install(g_vehicles);
    for (uint32_t i = 0; i < numVehicles; ++i)
    {
        g_vehicles.Get(i)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(
            Vector(27.8, 0.0, 0.0));
    }
    Ptr<MobilityModel> satMob = satNodes.Get(0)->GetObject<MobilityModel>();

    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-v2x-real-stack");
    if (eirpFromTr38821)
    {
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
    }
    else
    {
        // NT-02: an explicit value is CONDUCTED power at the array input.
        rs.SetSatConductedPowerDbm(satEirpDbm);
    }
    if (!scripted)
    {
        // TS 38.331 5.5.4 periodical RSRP reports (Wave 1 C2): the measured
        // Uu RSRP that the relay (re)selection rules of TS 38.304 8.1 use.
        rs.SetPeriodicMeasurementReports(true, MilliSeconds(120)); // reportInterval ms120
        // SSB on the cell beam steered at the Earth-fixed cell centre (Wave 1
        // C2, nr patch 09; TR 38.821 7.3), so the measured SS-RSRP carries the
        // payload's beam gain like the PDSCH does and RSRP and SINR describe
        // the same link. With quasi-omni SSB the RSRP sat ~18 dB below the data
        // beam and no vehicle could clear threshHighRemote.
        Ptr<ConstantPositionMobilityModel> cellCentre = CreateObject<ConstantPositionMobilityModel>();
        cellCentre->SetPosition(Vector(150.0 * numVehicles, 0.0, 0.0));
        rs.SetSsbCellBeam(true, cellCentre);
    }
    rs.Build(satNodes, g_vehicles);

    if (!scripted)
    {
        g_geo = std::make_unique<GeoState>();
        GeoState& g = *g_geo;
        g.satMob = satMob;
        g.fcHz = rs.GetCarrierFrequencyHz();
        g.blk = CreateObject<NtnScreenBlockageModel>();
        // Street canyon along +x: facade blocks on both sides with cross
        // streets; heights from one seeded draw (a city realization, not a
        // schedule). Each block is a model-B screen of the block's length.
        Ptr<UniformRandomVariable> hRv = CreateObject<UniformRandomVariable>();
        hRv->SetStream(cityStream);
        hRv->SetAttribute("Min", DoubleValue(bldMinH));
        hRv->SetAttribute("Max", DoubleValue(bldMaxH));
        const double xEnd = 300.0 * numVehicles + 30.0 * duration + 200.0;
        uint32_t nBld = 0;
        for (double side : {-1.0, 1.0})
        {
            for (double x0 = -200.0; x0 < xEnd; x0 += bldLenM + bldGapM)
            {
                Ptr<ConstantPositionMobilityModel> m =
                    CreateObject<ConstantPositionMobilityModel>();
                m->SetPosition(Vector(x0 + 0.5 * bldLenM, side * streetHalfWidthM, 0.0));
                g.blk->AddScreen(m, bldLenM, hRv->GetValue(), "facade");
                ++nBld;
            }
        }
        // A truck (2.5 m wide, 4 m high) in the next lane, driving with the
        // platoon: a moving blocker, effective only at low elevation.
        Ptr<ConstantVelocityMobilityModel> truck = CreateObject<ConstantVelocityMobilityModel>();
        truck->SetPosition(Vector(300.0 + 2.0, 3.5, 0.0));
        truck->SetVelocity(Vector(27.8, 0.0, 0.0));
        g.blk->AddScreen(truck, 2.5, 4.0, "truck");
        if (blockagePlane == "scalar")
        {
            Ptr<NtnScreenBlockageLossModel> blkL = CreateObject<NtnScreenBlockageLossModel>();
            blkL->SetBlockage(g.blk);
            blkL->SetFrequency(g.fcHz);
            rs.AddExtraPropagationLoss(blkL);
        }
        else
        {
            Ptr<NtnScreenBlockageSpectrumModel> blkSm =
                CreateObject<NtnScreenBlockageSpectrumModel>();
            blkSm->SetBlockage(g.blk);
            rs.AddSpectrumChannelLoss(blkSm);
        }

        // Thresholds from the radio's own noise floor (see header).
        const double scsHz = 15e3 * std::pow(2.0, rs.GetNumerology());
        const double nf = rs.GetUeNoiseFigureDb();
        g.threshHighRemoteDbm = -174.0 + 10.0 * std::log10(scsHz) + nf + g_minDirectSnrDb;
        g.threshLowRelayDbm = g.threshHighRemoteDbm + 3.0;
        std::printf("  city: %u facade screens (h %.0f-%.0f m, street half width %.0f m) + 1 truck\n"
                    "  threshHighRemote %.1f dBm, threshLowRelay %.1f dBm (per-RE SNR %.1f dB, "
                    "NF %.1f dB, SCS %.0f kHz)\n",
                    nBld, bldMinH, bldMaxH, streetHalfWidthM, g.threshHighRemoteDbm,
                    g.threshLowRelayDbm, g_minDirectSnrDb, nf, scsHz / 1e3);
    }
    // V2X-2 FIX (2026-08-25): install ONE flow per vehicle and keep the
    // handles, so the relay decision below has something to act on. With the
    // bouquet installed in bulk there were no per-vehicle handles, which is
    // part of why the decision loop could only increment counters.
    for (uint32_t v = 0; v < numVehicles; ++v)
    {
        g_vehDecision.push_back(Decision::Direct);
        g_vehOnAir.push_back(true);
        g_vehFlows.push_back(rs.InstallOranFlow(v,
                                                /*fiveQi=*/79,
                                                /*sst=*/2,
                                                /*sd=*/0x000002,
                                                NtnOranApplication::URLLC_PERIODIC,
                                                Seconds(1.0),
                                                Seconds(duration - 0.5)));
    }
    if (!scripted)
    {
        GeoState& g = *g_geo;
        g.relayOf.assign(numVehicles, -1);
        g.relayQ.resize(numVehicles);
        g.st.resize(numVehicles);
        // Relay bearers: the network's path to a remote UE through each
        // adjacent vehicle (PC5 range), installed off and switched on by the
        // relay selection. The relay's NtnOranSink hands each packet to the
        // relay's PC5 queue (the relay's L3 forwarding).
        for (uint32_t v = 0; v < numVehicles; ++v)
        {
            for (int d : {-1, 1})
            {
                const int r = static_cast<int>(v) + d;
                if (r < 0 || r >= static_cast<int>(numVehicles))
                {
                    continue;
                }
                ApplicationContainer apps =
                    rs.InstallOranFlow(r, 79, 2, 0x000002, NtnOranApplication::URLLC_PERIODIC,
                                       Seconds(1.0), Seconds(duration - 0.5));
                SetFlow(apps, false);
                Ptr<NtnOranSink> sink = DynamicCast<NtnOranSink>(apps.Get(1));
                NS_ABORT_MSG_IF(!sink, "relay bearer without an NtnOranSink");
                sink->TraceConnectWithoutContext(
                    "Rx", MakeBoundCallback(&OnRelayUuRx, v, static_cast<uint32_t>(r)));
                g.proxy[{v, static_cast<uint32_t>(r)}] = apps;
            }
        }
        // NR PC5 Mode-2 sidelink among the vehicles (TS 38.321 5.22, 5.9 GHz
        // TR 37.885 channel). Every vehicle keeps an SPS reservation of
        // pc5PeriodMs; a relay fills it with queued remote-UE packets.
        g.sl = CreateObject<NtnSlChannel>();
        NtnSlResourcePool pool;
        pool.numSubchannels = 5;
        pool.subchannelSizeRb = 10;
        pool.slotDuration = MilliSeconds(1);
        g.sl->SetResourcePool(pool);
        for (uint32_t v = 0; v < numVehicles; ++v)
        {
            Ptr<NtnSlUeMac> mac = CreateObject<NtnSlUeMac>();
            mac->SetUeId(v);
            mac->SetMobility(g_vehicles.Get(v)->GetObject<MobilityModel>());
            mac->SetSelectionWindow(1, std::max<uint32_t>(1, pc5PeriodMs - 1));
            mac->SetReservationPeriod(pc5PeriodMs);
            mac->SetAlignReselection(true);
            mac->SetPacketBytes(190);
            mac->AssignStreams(1000 + v);
            mac->SetTxPacketCallback(MakeCallback(&SlTxPacket));
            mac->SetRxCallback(MakeBoundCallback(&SlRx, v));
            g.sl->AddUe(mac);
            g.macs.push_back(mac);
        }
        g.sl->Start(Seconds(1.0), Seconds(duration));
    }
    // C6 (Wave 1): KPM from the RAN counters (TS 28.552 DRB.RlcSduDelayDl,
    // TB.TotNbrDl / TB.ErrTotNbrDl, RRU.PrbUsedDl) instead of app probes.
    rs.SetKpmFromRanCounters(true);
    rs.EnableAiFlowMonitor("ntn-v2x-real-stack"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;

    if (scripted)
    {
        Simulator::Schedule(Seconds(1.0), &RelayTick);
    }
    else
    {
        Simulator::Schedule(Seconds(1.5), &GeoRelayTick);
    }

    Simulator::Stop(Seconds(duration));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    // Make the audit fix explicit: the OLD path used V2xLeoDirect's closed-form
    // free-space budget for the same UE0 geometry. Show it next to the measured
    // value so the formula-vs-measurement gap (fast fading, interference, AMC,
    // real antenna gain) is visible, that gap is exactly why relay decisions
    // must ride the measured link.
    Vector v0 = g_vehicles.Get(0)->GetObject<MobilityModel>()->GetPosition();
    // Same budget convention as the radio: total EIRP over the cell bandwidth
    // and kTB + 7 dB NF (TR 38.821 Table 6.1.1.1-3). It used to pass the
    // conducted power as if it were EIRP and a -110 dBm floor that sits below
    // thermal noise for any NR bandwidth.
    const double bwHz = rs.GetBandwidthHz();
    const double eirpTotalDbm =
        eirpFromTr38821
            ? NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz + 10.0 * std::log10(bwHz / 1e6) +
                  30.0
            : satEirpDbm;
    const double noiseDbm = -174.0 + 10.0 * std::log10(bwHz) + 7.0;
    V2xLinkBudget formula =
        V2xLeoDirect::ComputeStatic(v0, satMob->GetPosition(), rs.GetCarrierFrequencyHz() / 1e9,
                                    eirpTotalDbm, noiseDbm);

    const uint64_t total = g_directDecisions + g_relayDecisions + g_outageDecisions;
    std::cout << "\n--- V2X Summary (relay logic on MEASURED radio) ---\n"
              << "  UE0 OLD closed-form SINR:    " << formula.snrDb
              << " dB (V2xLeoDirect::ComputeStatic, superseded)\n"
              << "  UE0 MEASURED SINR (mean):    " << rs.GetUeMeanSinrDb(0) << " dB\n"
              << "  measured cell SINR (mean):   " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured DL throughput:      " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  decisions (vehicle-ticks):   " << total << "\n"
              << "    direct to LEO:             " << g_directDecisions << "\n"
              << "    relayed via peer:          " << g_relayDecisions << "\n"
              << "    outage (no link):          " << g_outageDecisions << "\n"
              << "  -> direct/relay decided on MEASURED SINR, not V2xLeoDirect::Compute().\n";

    // V2X-2: evidence that the decision ACTUATED. Gate transitions are the
    // number of times a vehicle was actually taken off the air or put back,
    // and the per-vehicle delivery below is measured from the sink, so a
    // reader can check the outage decisions against packets that stopped
    // rather than taking the counter's word for it.
    uint64_t rxOnAir = 0;
    uint64_t rxGated = 0;
    uint64_t lostOnAir = 0;
    for (uint32_t v = 0; v < g_vehicles.GetN() && v < g_vehOnAir.size(); ++v)
    {
        const uint64_t rx = g_rs->GetUeRxPackets(v);
        const uint64_t lost = g_rs->GetUeLostPackets(v);
        if (g_vehOnAir[v])
        {
            rxOnAir += rx;
            lostOnAir += lost;
        }
        else
        {
            rxGated += rx;
        }
    }
    std::cout << "  actuation (V2X-2):\n"
              << "    gate transitions off/on:   " << g_gatedOff << " / " << g_gatedOn << "\n"
              << "    measured rx, on-air UEs:   " << rxOnAir << " pkts (lost " << lostOnAir
              << ")\n"
              << "    measured rx, gated UEs:    " << rxGated
              << " pkts (UEs gated at the end, cumulative over the run)\n"
              << "  -> an outage decision now stops that vehicle transmitting; before this the\n"
              << "     decision incremented a counter and every flow ran on regardless.\n";

    if (!scripted)
    {
        GeoState& g = *g_geo;
        std::cout << "\n--- Geometric blockage on the radio (TR 38.901 7.6.4.2 model B) ---\n";
        std::printf("  %3s %9s %8s %10s %10s %10s %9s %7s %7s %7s\n", "veh", "meanLoss", "blkTick",
                    "RSRP(dBm)", "SINR_LOS", "SINR_blk", "lossAtBlk", "relay", "outage",
                    "asRelay");
        double losSum = 0.0, blkSum = 0.0, lossBlkSum = 0.0;
        uint32_t losN = 0, blkN = 0;
        uint64_t uuGated = 0;
        uint64_t uuDrain = 0;
        for (uint32_t v = 0; v < numVehicles; ++v)
        {
            const VehStats& s = g.st[v];
            const double sl = s.nSinrLos ? s.sumSinrLos / s.nSinrLos : std::nan("");
            const double sb = s.nSinrBlk ? s.sumSinrBlk / s.nSinrBlk : std::nan("");
            const double lb = s.nSinrBlk ? s.sumLossAtSinrBlk / s.nSinrBlk : std::nan("");
            std::printf("  %3u %9.2f %8u %10.2f %10.2f %10.2f %9.2f %7u %7u %7u\n", v,
                        s.ticks ? s.sumLossDb / s.ticks : 0.0, s.blockedTicks,
                        s.nRsrp ? s.sumRsrp / s.nRsrp : std::nan(""), sl, sb, lb, s.relayTicks,
                        s.outageTicks, s.servedAsRelay);
            losSum += s.sumSinrLos;
            losN += s.nSinrLos;
            blkSum += s.sumSinrBlk;
            blkN += s.nSinrBlk;
            lossBlkSum += s.sumLossAtSinrBlk;
            uuGated += s.uuRxWhileGated;
            uuDrain += s.uuRxDrainAfterGate;
        }
        const double sinrLos = losN ? losSum / losN : std::nan("");
        const double sinrBlk = blkN ? blkSum / blkN : std::nan("");
        const double lossBlk = blkN ? lossBlkSum / blkN : std::nan("");
        std::printf("  measured SINR, LOS ticks:      %.2f dB (%u samples)\n", sinrLos, losN);
        std::printf("  measured SINR, blocked ticks:  %.2f dB (%u samples), geometric loss %.2f dB\n",
                    sinrBlk, blkN, lossBlk);
        std::printf("  SINR deficit vs geometric loss: %.2f dB vs %.2f dB (difference %.2f dB)\n",
                    sinrLos - sinrBlk, lossBlk, (sinrLos - sinrBlk) - lossBlk);
        std::printf("  RSRP deficit vs geometric loss (steady geometry): mean loss %.2f dB, mean error %.2f dB, "
                    "mean |error| %.2f dB over %u blocked vehicle-ticks\n",
                    g.rsrpErrN ? g.rsrpLossSum / g.rsrpErrN : 0.0,
                    g.rsrpErrN ? g.rsrpErrSum / g.rsrpErrN : 0.0,
                    g.rsrpErrN ? g.rsrpErrAbsSum / g.rsrpErrN : 0.0, g.rsrpErrN);
        std::printf("  blockage model evaluations: %llu (blocked %llu, non-local %llu)\n",
                    static_cast<unsigned long long>(g.blk->GetEvaluations()),
                    static_cast<unsigned long long>(g.blk->GetBlockedEvaluations()),
                    static_cast<unsigned long long>(g.blk->GetNonLocalEvaluations()));
        std::cout << "\n--- L3 UE-to-network relay over PC5 (TS 23.304) ---\n";
        std::printf("  Uu rx while gated (remote/outage UEs): %llu pkts (must be 0); in flight at "
                    "gating (first 500 ms): %llu pkts\n",
                    static_cast<unsigned long long>(uuGated),
                    static_cast<unsigned long long>(uuDrain));
        std::printf("  relay bearer pkts at relays:  %llu (dropped at full relay queue %llu)\n",
                    static_cast<unsigned long long>(g.enqueued),
                    static_cast<unsigned long long>(g.dropped));
        std::printf("  delivered to remote UEs over PC5: %llu (PDR %.4f)\n",
                    static_cast<unsigned long long>(g.delivered),
                    g.enqueued ? double(g.delivered) / g.enqueued : 0.0);
        std::printf("  relayed delay: PC5 hop mean %.2f ms, end to end mean %.2f ms (max %.2f) "
                    "= measured Uu OWD to the relay + relay queue + PC5\n",
                    g.delivered ? g.pc5SumMs / g.delivered : 0.0,
                    g.e2eN ? g.e2eSumMs / g.e2eN : 0.0, g.e2eMaxMs);
        std::printf("  PC5 channel: tx %llu, PRR %.4f, mean PC5 latency %.2f ms\n",
                    static_cast<unsigned long long>(g.sl->GetTxTotal()), g.sl->GetPrr(),
                    g.sl->GetMeanDelayMs());
        std::error_code ec;
        std::filesystem::create_directories(outputDir, ec);
        std::ofstream csv(outputDir + "/v2x_geometric_relay.csv");
        csv << "t_s,vehicle,geom_loss_db,rsrp_dbm,sinr_db,decision,relay\n";
        for (const auto& r : g.rows)
        {
            csv << r << "\n";
        }
        g_geo.reset();
    }

    Simulator::Destroy();
    return 0;
}
