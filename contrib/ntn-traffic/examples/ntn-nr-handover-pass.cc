// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-nr-handover-pass — a focused verification of Enabler A (NR inter-satellite
// handover) on a REALISTIC LEO pass geometry.
//
// Two satellites at 600 km share the same S-band carrier. The serving satellite
// starts overhead the UE and flies away toward the horizon at true LEO ground-
// track speed (~7.5 km/s); the neighbour starts one footprint behind and rises
// to overhead. The DL RSRP the UE measures on each cell's PSS therefore crosses
// over during the pass, the NR A3-RSRP algorithm's Event-A3 entry condition is
// met, the UE emits a measurement report, and the serving gNB triggers an X2
// handover. GetHandoverCount() counts the completions.
//
// This is the machinery test for the A3/X2 path. On a nominal LEO geometry the
// crossover margin is small (both slant ranges hug 600 km), which is exactly why
// Rel-17 NTN prefers elevation/timer conditional handover (carried by ntn-cho);
// here the serving satellite is driven far enough toward the horizon that a
// genuine RSRP-hysteresis crossover occurs, so the A3 path can be exercised
// end to end.

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-traffic-module.h"

#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnNrHandoverPass");

int
main(int argc, char* argv[])
{
    double simTime = 90.0;     // seconds — long enough for a real RSRP crossover
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
    double hystDb = 2.0;
    double tttMs = 512.0;
    // Footprint offset: the neighbour starts this far behind so it rises to
    // overhead as the serving sat flies off, giving a single clean RSRP
    // crossover mid-pass while BOTH cells are still strong (~-85 dBm) — not the
    // pathological case where the serving cell is swept into signal loss.
    double neighbourBehindKm = 600.0;
    double vSatKmps = 7.5; // true LEO ground-track speed

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("numUes", "Number of ground UEs", numUes);
    cmd.AddValue("altitudeKm", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("hystDb", "A3 hysteresis (dB)", hystDb);
    cmd.AddValue("tttMs", "A3 time-to-trigger (ms)", tttMs);
    cmd.AddValue("neighbourBehindKm", "Neighbour start offset behind serving (km)", neighbourBehindKm);
    cmd.Parse(argc, argv);

    const double H = altitudeKm * 1000.0;
    const double v = vSatKmps * 1000.0;

    // ---- Two satellites (gNBs) sharing the carrier + ground UEs ----------
    NodeContainer gnbNodes;
    gnbNodes.Create(2);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    gnbMobility.Install(gnbNodes);
    // Serving sat: overhead at t=0, flying +x toward the horizon.
    gnbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, H));
    gnbNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(v, 0.0, 0.0));
    // Neighbour sat: one footprint behind, flying +x so it rises to overhead.
    gnbNodes.Get(1)->GetObject<MobilityModel>()->SetPosition(
        Vector(-neighbourBehindKm * 1000.0, 0.0, H));
    gnbNodes.Get(1)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(v, 0.0, 0.0));

    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    ueMobility.Install(ueNodes);
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        double x = (static_cast<double>(i) - (numUes - 1) / 2.0) * 500.0;
        ueNodes.Get(i)->GetObject<MobilityModel>()->SetPosition(Vector(x, 0.0, 1.5));
    }

    // ---- Build the NR radio with Enabler A armed -------------------------
    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simTime));
    rs.SetOutputDir("./nr-ho-pass/");
    rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
    rs.SetNumerology(1);
    rs.SetCarrierFrequencyHz(freqGhz * 1e9);
    rs.SetBandwidthHz(bwMhz * 1e6);
    rs.SetSatEirpDbm(satEirpDbm);
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
        std::cerr << "NOTE: no handover fired — increase simTime or neighbourBehindKm, "
                     "or lower hystDb.\n";
        return 2;
    }
    return 0;
}
