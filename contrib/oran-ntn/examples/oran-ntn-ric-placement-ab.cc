/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// oran-ntn-ric-placement-ab — RIC placement as a measured experiment variable
// (AI-Native ORAN-NTN adoption WS3; Deng 2026 Sec. IV-B).
//
// One REAL mmwave NR NTN cell (Ka 20 GHz, SGP4 satellite), healthy at ~18 dB.
// Periodic deep fades (15 dB extra loss, a real PropagationLossModel in the
// channel chain) crush the link every fadePeriod. A beam-restoration xApp
// reacts to 50 ms KPM reports — but BOTH directions of its loop (KPM uplink,
// control downlink) are delayed by the E2 latency of the chosen RIC
// placement, computed from the LIVE slant range (OranNtnRicPlacement):
//
//   --placement=onboard   processing only (paper "Space RIC")
//   --placement=gateway   slant/c + processing (2.1-18 ms band at LEO)
//   --placement=cloud     slant/c + 20 ms backhaul + processing
//
// Two MEASURED outcomes per placement, same control logic:
//   * control-loop reaction time: fade onset -> beam actuation event on the
//     simulator queue (KPM sense + E2 up + decision + E2 down) — the on-board
//     RIC reacts in tens of ms, the cloud RIC ~50 ms later;
//   * PHY recovery time (TBLER back under 0.2): largely placement-INVARIANT,
//     because the real AMC also adapts its MCS to the fade — a genuine
//     lower-layer self-healing effect this experiment surfaces honestly.
//
// Quick test:  --simSeconds=40 --placement=cloud
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-static-extra-loss-model.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/oran-ntn-ric-placement.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNtnRicPlacementAb");

namespace
{

NtnRealStackHelper* g_rs = nullptr;
Ptr<NtnStaticExtraLossModel> g_fade; // real channel impairment
Ptr<NtnStaticExtraLossModel> g_beam; // RIC-commanded compensation (neg. loss)
Ptr<OranNtnRicPlacement> g_placement;
Ptr<MobilityModel> g_satMob;
Ptr<MobilityModel> g_ueMob;
// Where the RIC actually sits. For an on-board RIC the distance is unused; for
// a gateway or cloud RIC it is the ground site; for an aerial RIC it is the
// stratospheric platform, which is closer to the satellite than the ground is.
Ptr<MobilityModel> g_ricSiteMob;

double g_beamGainDb = 12.0;
double g_threshDb = 12.0;
bool g_beamOn = false;
uint32_t g_actuations = 0;

// Fade bookkeeping for the measured recovery-time KPI.
std::vector<double> g_fadeOnsets;
std::vector<double> g_recoveryS;
std::vector<double> g_reactionS; // onset -> beam actuation (control plane)
bool g_inFade = false;
double g_pendingOnset = -1.0;
double g_pendingReactOnset = -1.0;
bool g_sawDegradation = false;

// CVC-05 FIX (2026-08-24). This used to record a reaction sample only on an
// off->on edge of the beam (`on && !g_beamOn`). Once the beam latched on, later
// fade onsets kept overwriting g_pendingReactOnset while no sample was taken,
// so the next time the beam happened to cycle it recorded the interval from an
// unrelated, much older onset. That is what produced a ~1.1 s "reaction" for
// the cloud placement: a latching artifact, not E2 transport. The reaction time
// is now carried with the scheduled action itself, so exactly one sample is
// recorded per fade onset, measured from that onset to the arrival of the first
// control action that responds to it, whatever the beam state already was.
void
ApplyBeam(bool on, double reactOnset)
{
    if (on && reactOnset >= 0.0)
    {
        g_reactionS.push_back(Simulator::Now().GetSeconds() - reactOnset);
    }
    if (on && !g_beamOn)
    {
        ++g_actuations;
    }
    g_beamOn = on;
    const double gain = on ? g_beamGainDb : 0.0;
    g_beam->SetFloorDb(-gain);
    g_beam->SetLossDb(-gain);
}

// The control loop, with BOTH legs riding the placement's E2 latency.
void
RicDecide(double intrinsicSinrDb)
{
    const bool wantBeam = intrinsicSinrDb < g_threshDb;
    const double slantM = g_ricSiteMob->GetDistanceFrom(g_satMob);
    // Claim the pending onset here, at the moment the RIC decides to act on it,
    // so one fade contributes exactly one reaction sample even if a later KPM
    // indication repeats the same decision.
    double reactOnset = -1.0;
    if (wantBeam && g_pendingReactOnset >= 0.0)
    {
        reactOnset = g_pendingReactOnset;
        g_pendingReactOnset = -1.0;
    }
    Simulator::Schedule(g_placement->ComputeE2Delay(slantM), &ApplyBeam, wantBeam, reactOnset);
}

void
KpmTick(Time period)
{
    const double measured = g_rs->GetUeRecentSinrDb(0);
    if (!std::isnan(measured))
    {
        const double intrinsic = measured - (g_beamOn ? g_beamGainDb : 0.0);
        const double slantM = g_ricSiteMob->GetDistanceFrom(g_satMob);
        // KPM indication arrives at the RIC after the E2 uplink delay.
        Simulator::Schedule(g_placement->ComputeE2Delay(slantM), &RicDecide, intrinsic);
    }
    Simulator::Schedule(period, &KpmTick, period);
}

// 10 ms PHY-health sampler: measures recovery time after each fade onset.
void
HealthTick()
{
    const double tbler = g_rs->GetUeRecentTbler(0);
    if (g_pendingOnset >= 0.0 && !std::isnan(tbler))
    {
        if (!g_sawDegradation && tbler > 0.5)
        {
            g_sawDegradation = true; // the fade has reached the PHY
        }
        else if (g_sawDegradation && tbler < 0.2)
        {
            g_recoveryS.push_back(Simulator::Now().GetSeconds() - g_pendingOnset);
            g_pendingOnset = -1.0;
            g_sawDegradation = false;
        }
    }
    Simulator::Schedule(MilliSeconds(10), &HealthTick);
}

void
SetFade(bool on)
{
    g_inFade = on;
    g_fade->SetLossDb(on ? 15.0 : 0.0);
    if (on)
    {
        g_fadeOnsets.push_back(Simulator::Now().GetSeconds());
        g_pendingOnset = Simulator::Now().GetSeconds();
        g_pendingReactOnset = g_pendingOnset;
        g_sawDegradation = false;
    }
}

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 40.0;
    double leoAltKm = 1200.0;
    double freqGHz = 20.0;
    double satEirpDbm = 85.0;
    double fadePeriodS = 8.0;
    double fadeDurS = 4.0;
    std::string placement = "gateway";
    double hapsAltKm = 20.0;
    std::string radio = "nr"; // radio backend: nr (FR1) or mmwave
    std::string outputDir = "oran-ntn-ric-placement-ab-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("placement", "RIC placement: onboard|haps|gateway|cloud", placement);
    cmd.AddValue("hapsAltKm", "Aerial RIC platform altitude (km)", hapsAltKm);
    cmd.AddValue("fadePeriodS", "Fade event period (s)", fadePeriodS);
    cmd.AddValue("fadeDurS", "Fade event duration (s)", fadeDurS);
    cmd.AddValue("satEirpDbm", "Satellite EIRP (dBm)", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    g_placement = CreateObject<OranNtnRicPlacement>();
    if (placement == "onboard")
    {
        g_placement->SetSite(OranNtnRicPlacement::Site::OnBoardSatellite);
    }
    else if (placement == "haps")
    {
        g_placement->SetSite(OranNtnRicPlacement::Site::Haps);
    }
    else if (placement == "cloud")
    {
        g_placement->SetSite(OranNtnRicPlacement::Site::GroundCloud);
    }
    else if (placement == "gateway")
    {
        g_placement->SetSite(OranNtnRicPlacement::Site::GroundGateway);
    }
    else
    {
        NS_ABORT_MSG("unknown --placement=" << placement
                     << " (expected onboard|haps|gateway|cloud)");
    }

    std::printf("# oran-ntn-ric-placement-ab (REAL cell, placement=%s)\n",
                placement.c_str());

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(1);

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
    g_satMob = satEnu;

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    uePos->Add(Vector(0.0, 0.0, 1.5));
    mob.SetPositionAllocator(uePos);
    mob.Install(ueNodes);
    g_ueMob = ueNodes.Get(0)->GetObject<MobilityModel>();

    // The RIC site. An aerial platform is placed directly above the terminal at
    // its cruise altitude, in the same ENU frame the satellite is projected
    // into, so the E2 leg it sees is a real slant range and not a constant.
    if (placement == "haps")
    {
        Ptr<ConstantPositionMobilityModel> hapsMob =
            CreateObject<ConstantPositionMobilityModel>();
        hapsMob->SetPosition(Vector(0.0, 0.0, hapsAltKm * 1000.0));
        g_ricSiteMob = hapsMob;
    }
    else
    {
        g_ricSiteMob = g_ueMob;
    }

    NtnRealStackHelper rs;
    g_rs = &rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("oran-ntn-ric-placement-ab-" + placement);
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));

    g_fade = CreateObject<NtnStaticExtraLossModel>();
    g_fade->SetLossDb(0.0);
    rs.AddExtraPropagationLoss(g_fade);
    g_beam = CreateObject<NtnStaticExtraLossModel>();
    g_beam->SetLossDb(0.0);
    rs.AddExtraPropagationLoss(g_beam);

    // Real fade schedule.
    for (double t = 5.0; t + fadeDurS < simSeconds; t += fadePeriodS)
    {
        Simulator::Schedule(Seconds(t), &SetFade, true);
        Simulator::Schedule(Seconds(t + fadeDurS), &SetFade, false);
    }

    Simulator::Schedule(MilliSeconds(50), &KpmTick, MilliSeconds(50));
    Simulator::Schedule(MilliSeconds(10), &HealthTick);

    rs.RegisterPeriodicCallback(Seconds(5.0), [](Time now) {
        std::printf("  t=%5.1f  sinr=%6.2f dB  tbler=%5.3f  fade=%d beam=%d  "
                    "e2_delay=%.2f ms\n",
                    now.GetSeconds(), g_rs->GetUeRecentSinrDb(0),
                    g_rs->GetUeRecentTbler(0), g_inFade ? 1 : 0, g_beamOn ? 1 : 0,
                    g_placement
                            ->ComputeE2Delay(g_ricSiteMob->GetDistanceFrom(g_satMob))
                            .GetSeconds() *
                        1e3);
    });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    auto mean = [](const std::vector<double>& v) {
        double m = 0;
        for (double x : v)
        {
            m += x;
        }
        return v.empty() ? 0.0 : m / v.size();
    };
    // CVC-05: report n and the spread alongside every mean. A reaction mean
    // quoted without its sample count cannot be sanity-checked by a reader, and
    // that is how the earlier latching artifact went unnoticed.
    auto stdev = [&mean](const std::vector<double>& v) {
        if (v.size() < 2)
        {
            return 0.0;
        }
        const double m = mean(v);
        double s = 0;
        for (double x : v)
        {
            s += (x - m) * (x - m);
        }
        return std::sqrt(s / (v.size() - 1));
    };
    std::printf("# === summary ===  placement=%s fades=%zu  "
                "meanReaction=%.1f ms sd=%.1f ms n=%zu (control loop, E2 both legs)  "
                "meanPhyRecovery=%.1f ms sd=%.1f ms n=%zu (AMC+beam)  actuations=%u  "
                "SINR=%.2f dB thr=%.3f Mbps\n",
                placement.c_str(), g_fadeOnsets.size(), mean(g_reactionS) * 1e3,
                stdev(g_reactionS) * 1e3, g_reactionS.size(), mean(g_recoveryS) * 1e3,
                stdev(g_recoveryS) * 1e3, g_recoveryS.size(), g_actuations,
                rs.GetMeanDlSinrDb(), rs.GetRxThroughputMbps());
    Simulator::Destroy();
    return 0;
}
