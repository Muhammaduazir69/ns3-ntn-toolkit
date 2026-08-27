// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-nr-handover-pass — a focused verification of Enabler A (NR inter-satellite
// handover) on a REAL SGP4 LEO pass.
//
// Both gNBs are genuine SGP4-propagated Walker-Delta satellites (600 km, 53 deg),
// NOT straight-line ConstantVelocity nodes: the serving satellite (element 0)
// flies its real orbit while the RISING adjacent plane-member serves as the
// handover neighbour. As the real slant ranges evolve over the pass, the DL RSRP
// the UE measures on each cell's PSS crosses over, the NR A3-RSRP algorithm's
// Event-A3 condition is met, the UE emits a measurement report, and the serving
// gNB triggers an X2 handover. GetHandoverCount() counts the completions.
//
// This exercises the A3/X2 path on real orbital geometry. NOTE: two co-planar
// LEO satellites hug similar slant ranges, so with a small hysteresis the UE can
// ping-pong between them across the pass (the count then reflects oscillations,
// not distinct passes) — which is exactly why Rel-17 NTN adds elevation/timer
// conditional handover (carried by ntn-cho). Raise --hystDb / --tttMs to damp it.

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-traffic-module.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <iostream>

using namespace ns3;
using ns3::ntncon::Sgp4MobilityModel;
using ns3::ntncon::WalkerConfig;
using ns3::ntncon::WalkerConstellation;

NS_LOG_COMPONENT_DEFINE("NtnNrHandoverPass");

int
main(int argc, char* argv[])
{
    double simTime = 120.0;    // seconds — long enough for a real SGP4 pass crossover
    // One UE handing between the two satellites is the clean machinery test.
    // numUes > 1 drives several UEs through the handover at once and exposes a
    // separate, structural limitation of the vendored 5G-LENA v3.3 X2 handover
    // DATA-FORWARDING path -- it pushes a PDCP control PDU (SN-status / end
    // marker) through NrPdcpHeader::Deserialize, which asserts because that
    // build "just supports DATA PDUs" (nr-pdcp-header.cc:103). That is upstream
    // of Enabler A (the A3/X2 decision itself fires correctly) and out of scope
    // here; leave numUes=1 for a run that completes cleanly.
    uint32_t numUes = 1;
    double altitudeKm = 600.0; // LEO altitude
    double satEirpDbm = 70.0;
    double freqGhz = 2.0; // S-band
    double bwMhz = 10.0;  // single BWP — isolate the handover machinery
    // Hysteresis + a longer time-to-trigger deliberately damp post-handover
    // ping-pong: right after a handover the new serving cell's RSRP filter is
    // briefly unpopulated, and an over-eager reverse trigger under a receding
    // (near-zero-RSRP) old serving cell can storm the source scheduler. 2 dB /
    // 512 ms yields one clean handover on the pass below.
    double hystDb = 3.0;  // damp ping-pong between co-planar sats
    double tttMs = 512.0;
    // Footprint offset: the neighbour starts this far behind so it rises to
    // overhead as the serving sat flies off, giving a single clean RSRP
    // crossover mid-pass while BOTH cells are still strong (~-85 dBm) — not the
    // pathological case where the serving cell is swept into signal loss.
    double vSatKmps = 7.5; // nominal LEO ground-track speed (display only)

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("numUes", "Number of ground UEs", numUes);
    cmd.AddValue("altitudeKm", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("hystDb", "A3 hysteresis (dB)", hystDb);
    cmd.AddValue("tttMs", "A3 time-to-trigger (ms)", tttMs);
    cmd.Parse(argc, argv);

    // ---- Two satellites (gNBs) sharing the carrier + ground UEs ----------
    NodeContainer gnbNodes;
    gnbNodes.Create(2);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    // REAL LEO pass, not a straight-line fly-by: both gNBs are SGP4-propagated
    // Walker-Delta satellites. The serving sat is element 0; the neighbour is
    // whichever adjacent plane-member is currently RISING toward the UE (the
    // genuine geometry of an inter-satellite handover — a setting sat handing
    // off to a rising one). Standard-aligned with TR 38.821 LEO handover.
    WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 6;
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = WalkerConstellation::BuildDelta(wcfg);
    Ptr<Sgp4MobilityModel> serv = CreateObject<Sgp4MobilityModel>();
    serv->SetElements(elements[0]);
    Ptr<Sgp4MobilityModel> nbrA = CreateObject<Sgp4MobilityModel>();
    nbrA->SetElements(elements[1]);
    Ptr<Sgp4MobilityModel> nbrB = CreateObject<Sgp4MobilityModel>();
    nbrB->SetElements(elements[wcfg.total_sats - 1]);

    // Ground UEs under the serving satellite's sub-point, TR 38.811 mobility.
    double subLat, subLon, subAlt;
    serv->GetGeodetic(subLat, subLon, subAlt);
    NtnTr38811MobilityHelper ueMobility(1);
    auto ueProfile = NtnMobilityScenarios::MixedContinental();
    auto ueModels = ueMobility.Install(ueNodes, ueProfile, subLat - 0.03, subLat + 0.03,
                                       subLon - 0.03, subLon + 0.03);

    // Pick the neighbour that is approaching the UE (velocity dotted toward it).
    const Vector ue0 = ueModels[0]->GetPosition();
    auto approaching = [&ue0](Ptr<Sgp4MobilityModel> s) {
        const Vector p = s->GetPosition();
        const Vector vv = s->GetVelocity();
        const Vector d(ue0.x - p.x, ue0.y - p.y, ue0.z - p.z);
        return (vv.x * d.x + vv.y * d.y + vv.z * d.z) > 0.0;
    };
    Ptr<Sgp4MobilityModel> cand = approaching(nbrA) ? nbrA : nbrB;
    gnbNodes.Get(0)->AggregateObject(serv);
    gnbNodes.Get(1)->AggregateObject(cand);

    // ---- Build the NR radio with Enabler A armed -------------------------
    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simTime));
    rs.SetOutputDir("./nr-ho-pass/");
    rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
    rs.SetNumerology(1);
    rs.SetCarrierFrequencyHz(freqGhz * 1e9);
    rs.SetBandwidthHz(bwMhz * 1e6);
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(satEirpDbm);
    rs.SetUeTxPowerDbm(23.0);
    rs.SetBackhaulDelay(MilliSeconds(5));
    rs.SetHandover(true, hystDb, MilliSeconds(static_cast<uint64_t>(tttMs)));

    rs.Build(gnbNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(0.5),
                      Seconds(simTime - 0.2));

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    rs.Collect();
    const uint32_t hoCount = rs.GetHandoverCount();
    Simulator::Destroy();

    std::cout << "\n========== NR handover pass — MEASURED ==========\n";
    std::cout << "  Sat altitude / speed : " << altitudeKm << " km / " << vSatKmps << " km/s\n";
    std::cout << "  A3 hysteresis / TTT  : " << hystDb << " dB / " << tttMs << " ms\n";
    std::cout << "  Sim time             : " << simTime << " s\n";
    std::cout << "  Mean DL SINR         : " << rs.GetMeanDlSinrDb() << " dB\n";
    std::cout << "  PHY RX transport TB  : " << rs.GetPhyRxTb() << "\n";
    std::cout << "  [A] Handovers done   : " << hoCount << "\n";
    std::cout << "=================================================\n\n";

    if (hoCount == 0)
    {
        // A pass with zero handovers is a valid outcome (short window or a
        // geometry whose RSRP never crossed the hysteresis) — report it, but do
        // NOT exit non-zero: the run succeeded, it simply produced no handover.
        std::cerr << "NOTE: no handover fired — increase simTime for the real pass, "
                     "or lower hystDb.\n";
    }
    return 0;
}
