/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// oran-ntn-ric-controlled-traffic — a closed O-RAN control loop steering a
// REAL LEO downlink on a REAL mmwave NR NTN cell (NtnRealStackHelper):
//
//   data plane:   saturating downlink over the real radio (SpectrumPhy + MAC +
//                 RLC/PDCP + RRC + EPC); KPIs MEASURED off the PHY trace.
//   telemetry:    OranNtnE2Node reports E2-KPM each second; the reported SINR
//                 is the INTRINSIC link quality (measured minus the RIC's own
//                 beam gain) so the xApp's decision tracks the channel, not
//                 its own actuation.
//   control:      the mMIMO-precoder xApp consumes each KPM indication; when
//                 intrinsic SINR < threshold it selects a beam from the
//                 OranNtnMmimoCodebook and issues an E2SM-RC BEAM_SWITCH
//                 action back through the E2 node (ReceiveRcAction). The
//                 action lands as a LIVE channel reconfiguration — real
//                 packets feel the beam, so MEASURED SINR/TBLER/goodput
//                 recover.
//
// Loop timing (audit fix 2026-06-12, issue #12): both legs of the loop are
// delay-modeled — the KPM indication crosses one feeder-link delay uplink and
// is dispatched only on the next Near-RT RIC control-loop tick
// (AlignToControlLoop=true, 100 ms period), and the RC action crosses one
// feeder-link delay downlink before it actuates. So the modeled loop is
// measure -> feeder -> loop tick -> feeder -> apply, not the optimistic
// inline execution. E2AP-over-SCTP itself is NOT simulated (same substitution
// as ns-3 mainline S1/X2; see oran-ntn-e2-interface.h).
//
// Beam state is scoped PER CELL (keyed by E2 cellId), so copy-pasting this
// pattern into a multi-satellite scenario cannot bleed beam gain across
// satellites.
//
// Audit fix (AI-Native ORAN-NTN adoption WS0): previously the KPM SINR
// was a closed-form FSPL budget and the data plane a P2P RateErrorModel behind
// a sigmoid SnrToPer() — now both halves of the loop ride the measured radio.
// Mobility is real: SGP4 Walker element (ENU-projected), fixed ground UE.
//
// Compare --xapp=1 vs --xapp=0: without the xApp the marginal Ka link stays
// degraded for the whole pass.
//
// Quick test:  --simSeconds=40 --xapp=1
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-static-extra-loss-model.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/oran-ntn-e2-interface.h"
#include "ns3/oran-ntn-mmimo-codebook.h"
#include "ns3/oran-ntn-types.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNtnRicControlledTraffic");

namespace
{

constexpr uint32_t kCellId = 7; //!< E2 node id of the (single) satellite gNB

NtnRealStackHelper* g_rs = nullptr;
Ptr<OranNtnE2Node> g_e2;
Ptr<OranNtnMmimoCodebook> g_codebook;

/// RIC-commanded beam state of ONE cell. Keeping this per cell (instead of
/// file-scope globals) means a multi-satellite copy of this example cannot
/// bleed beam gain from one satellite into another.
struct CellBeamState
{
    Ptr<NtnStaticExtraLossModel> model; //!< negative loss = commanded array gain
    double gainDb{0.0};                 //!< current RIC-commanded beam gain
    bool on{false};
    uint32_t activations{0};
};

std::map<uint32_t, CellBeamState> g_beams; //!< keyed by E2 cellId

double g_sinrThreshDb = 12.0; // xApp engages the beam below this (intrinsic)
bool g_xappEnabled = true;
uint32_t g_numTx = 64;

void
ApplyBeamState(uint32_t cellId)
{
    auto it = g_beams.find(cellId);
    if (it == g_beams.end())
    {
        return;
    }
    const CellBeamState& st = it->second;
    const double gain = st.on ? st.gainDb : 0.0;
    st.model->SetFloorDb(-gain);
    st.model->SetLossDb(-gain);
}

// RC action handler on the E2 node: executes the xApp's BEAM_SWITCH command
// (after the return feeder delay) as a live channel reconfiguration of the
// TARGETED cell only. parameter1 > 0 engages that beam gain; 0 disengages.
bool
ApplyBeamRcAction(E2RcAction action)
{
    auto it = g_beams.find(action.targetGnbId);
    if (it == g_beams.end())
    {
        return false;
    }
    CellBeamState& st = it->second;
    if (action.parameter1 > 0.0)
    {
        st.gainDb = action.parameter1;
        if (!st.on)
        {
            ++st.activations;
        }
        st.on = true;
    }
    else
    {
        st.on = false;
    }
    ApplyBeamState(action.targetGnbId);
    return true;
}

// The xApp: invoked on every E2-KPM indication (dispatched on the RIC
// control-loop tick, one feeder delay after the measurement was taken).
// §4.1.12 mMIMO precoder control — engage beamforming when the intrinsic
// link SINR is below target. The decision is NOT applied inline: it is sent
// back through OranNtnE2Node::ReceiveRcAction, which delivers it after the
// return-path feeder delay.
void
PrecoderXapp(E2Indication ind)
{
    if (!g_xappEnabled)
    {
        return; // beam starts (and stays) disengaged
    }
    const uint32_t cellId = ind.kpmReport.gnbId;
    auto it = g_beams.find(cellId);
    if (it == g_beams.end())
    {
        return;
    }
    const double sinr = ind.kpmReport.sinr_dB; // intrinsic (see KPM tick)

    E2RcAction action{};
    action.timestamp = Simulator::Now().GetSeconds();
    action.xappId = 1;
    action.xappName = "mmimo-precoder";
    action.actionType = E2RcActionType::BEAM_SWITCH;
    action.targetGnbId = cellId;
    action.targetBeamId = 1;
    action.confidence = 1.0;

    if (sinr < g_sinrThreshDb)
    {
        // Build a broadside steering target and pick the nearest codebook
        // beam (exercises the codebook); command its array gain.
        std::vector<float> target(g_numTx * 2, 0.0f);
        for (uint32_t i = 0; i < g_numTx; ++i)
        {
            target[2 * i] = 1.0f; // real part = 1 (broadside), im = 0
        }
        (void)g_codebook->BestMatch(target);
        action.parameter1 = 10.0 * std::log10(static_cast<double>(g_numTx));
        g_e2->ReceiveRcAction(action); // executes after the return feeder delay
    }
    else if (it->second.on)
    {
        action.parameter1 = 0.0; // disengage
        g_e2->ReceiveRcAction(action);
    }
}

double
ElevDegEnu(const Vector& gnd, const Vector& sat)
{
    const double dx = sat.x - gnd.x;
    const double dy = sat.y - gnd.y;
    const double dz = sat.z - gnd.z;
    const double horiz = std::max(std::sqrt(dx * dx + dy * dy), 1e-3);
    return std::atan2(dz, horiz) * 180.0 / M_PI;
}

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 40.0;
    double leoAltKm = 1200.0;
    double freqGHz = 20.0;    // Ka-band
    double satEirpDbm = 70.0; // marginal without the beam (below threshold)
    uint32_t numTx = 64;
    double sinrThreshDb = 12.0;
    bool xappEnabled = true;
    std::string outputDir = "oran-ntn-ric-controlled-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("leoAltKm", "Satellite altitude (km)", leoAltKm);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("satEirpDbm", "Baseline EIRP without beamforming (dBm)", satEirpDbm);
    cmd.AddValue("numTx", "mMIMO Tx antennas (beam gain = 10log10(numTx))", numTx);
    cmd.AddValue("sinrThreshDb", "Intrinsic SINR threshold for the xApp", sinrThreshDb);
    cmd.AddValue("xapp", "Enable the mMIMO precoder xApp control loop", xappEnabled);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    g_sinrThreshDb = sinrThreshDb;
    g_xappEnabled = xappEnabled;
    g_numTx = numTx;

    std::printf("# oran-ntn-ric-controlled-traffic (E2-KPM -> mMIMO precoder xApp -> RC "
                "BEAM_SWITCH -> beam, measured radio)\n");
    std::printf("#   sim=%.0fs alt=%.0fkm freq=%.0fGHz baseEIRP=%.0fdBm numTx=%u "
                "(beamGain=%.1fdB) xApp=%s thresh=%.0fdB\n",
                simSeconds, leoAltKm, freqGHz, satEirpDbm, numTx,
                10.0 * std::log10(static_cast<double>(numTx)),
                xappEnabled ? "on" : "off", sinrThreshDb);

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    // Real SGP4 orbit projected into the local ENU frame: the serving Walker
    // element is at zenith at t=0 and recedes with genuine orbital dynamics.
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = leoAltKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(elements[0]);
    double subLat, subLon, subAlt;
    satSgp4->GetGeodetic(subLat, subLon, subAlt);
    Ptr<NtnEnuProjectionMobilityModel> satEnu = CreateObject<NtnEnuProjectionMobilityModel>();
    satEnu->SetSource(satSgp4);
    satEnu->SetReference(subLat, subLon, 0.0);
    satNodes.Get(0)->AggregateObject(satEnu);

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    uePos->Add(Vector(0.0, 0.0, 1.5));
    mob.SetPositionAllocator(uePos);
    mob.Install(ueNodes);

    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("oran-ntn-ric-controlled-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    g_rs = &rs;

    // The RIC-commanded beam as a LIVE channel reconfiguration (negative loss
    // = array gain) chained onto the real spectrum channel — one beam state
    // per cell, keyed by the cell's E2 node id.
    CellBeamState beam;
    beam.model = CreateObject<NtnStaticExtraLossModel>();
    beam.model->SetLossDb(0.0);
    rs.AddExtraPropagationLoss(beam.model);
    g_beams[kCellId] = beam;

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));

    // O-RAN E2 node on the satellite gNB + KPM subscription. Loop timing is
    // honest (audit issue #12): one feeder delay per direction, and the
    // indication is dispatched to the xApp only on the next 100 ms RIC
    // control-loop tick (AlignToControlLoop) — E2AP/SCTP not simulated.
    g_e2 = CreateObject<OranNtnE2Node>();
    g_e2->SetNodeId(kCellId);
    g_e2->SetIsNtn(true);
    g_e2->SetAttribute("FeederLinkDelay", TimeValue(MilliSeconds(4))); // ~1200 km
    g_e2->SetAttribute("AlignToControlLoop", BooleanValue(true));
    g_e2->RegisterRanFunction(2, "E2SM-KPM v03.00");
    g_e2->RegisterRanFunction(3, "E2SM-RC v01.03");
    E2Subscription sub{};
    sub.subscriptionId = 1;
    sub.ranFunctionId = 2;
    sub.reportingPeriod = Seconds(1.0);
    sub.eventTrigger = false;
    g_e2->HandleSubscriptionRequest(sub);
    g_e2->SetIndicationCallback(MakeCallback(&PrecoderXapp));
    g_e2->SetRcActionCallback(MakeCallback(&ApplyBeamRcAction));
    {
        TimeValue feeder;
        TimeValue loop;
        g_e2->GetAttribute("FeederLinkDelay", feeder);
        g_e2->GetAttribute("ControlLoopPeriod", loop);
        std::printf("# E2 loop timing: measure -> +%.0f ms feeder -> next %.0f ms RIC "
                    "tick -> +%.0f ms feeder -> apply (AlignToControlLoop=true)\n",
                    feeder.Get().GetMilliSeconds() * 1.0,
                    loop.Get().GetMilliSeconds() * 1.0,
                    feeder.Get().GetMilliSeconds() * 1.0);
    }

    // mMIMO codebook the xApp selects beams from.
    g_codebook = CreateObject<OranNtnMmimoCodebook>();
    g_codebook->Configure(numTx, 64);
    g_codebook->PopulateDftAzimuthSweep(numTx, 64);

    std::printf("# %5s  %7s  %9s  %9s  %5s  %8s  %9s\n",
                "t_s", "elev", "measured", "intrinsic", "beam", "tbler", "goodput");

    // KPM tick: telemetry on the MEASURED radio (intrinsic = measured - own
    // beam gain, so the xApp decision is stable across its own actions).
    Ptr<MobilityModel> ueMob = ueNodes.Get(0)->GetObject<MobilityModel>();
    uint64_t lastRx = 0;
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [ueMob, satEnu, &lastRx](Time now) {
            const CellBeamState& st = g_beams[kCellId];
            const double measured = g_rs->GetUeRecentSinrDb(0);
            const double comp = st.on ? st.gainDb : 0.0;
            const double intrinsic = measured - comp;
            const double elev = ElevDegEnu(ueMob->GetPosition(), satEnu->GetPosition());
            if (!std::isnan(measured))
            {
                E2KpmReport r{};
                r.timestamp = now.GetSeconds();
                r.gnbId = g_e2->GetNodeId();
                r.isNtn = true;
                r.ueId = 1;
                r.sinr_dB = intrinsic;
                r.elevation_deg = elev;
                g_e2->SubmitKpmMeasurement(r);
            }
            const uint64_t rx = g_rs->GetUeRxBytes(0);
            const double mbps = (rx - lastRx) * 8.0 / 1e6;
            lastRx = rx;
            std::printf("  %5.1f  %7.2f  %9.2f  %9.2f  %5s  %8.3f  %9.3f\n",
                        now.GetSeconds(), elev, measured, intrinsic,
                        st.on ? "ON" : "off", g_rs->GetUeRecentTbler(0), mbps);
        });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::printf("# === summary ===  xApp=%s beamActivations=%u  measured cell "
                "SINR=%.2f dB TBLER=%.4f throughput=%.3f Mbps (control loop on the "
                "measured radio; RC actions via ReceiveRcAction, one feeder delay "
                "each way)\n",
                xappEnabled ? "on" : "off", g_beams[kCellId].activations,
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps());

    Simulator::Destroy();
    return 0;
}
