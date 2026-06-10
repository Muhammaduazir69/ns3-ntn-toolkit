// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-real-stack-smoke — Phase-0 validation of NtnRealStackHelper.
//
// Stands up a single LEO gNB (satellite) over a small set of ground UEs using a
// REAL mmwave NR air interface (SpectrumPhy + MAC + HARQ + RLC/PDCP + EPC),
// runs UDP traffic over the radio, and reports MEASURED SINR/TBLER/throughput
// from PHY trace sources + FlowMonitor. This is the template every NTN module
// adopts in Phase 1+ (see PROTOCOL_FIDELITY_AUDIT_AND_FIX_2026-06.md).
//
// Run:
//   ./ns3 run "ntn-real-stack-smoke --simTime=10 --numUes=4 --altKm=600"

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"


using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnRealStackSmoke");

int
main(int argc, char* argv[])
{
    double simTime = 10.0;
    uint32_t numUes = 4;
    double altKm = 600.0;
    double satEirpDbm = 55.0; // LEO beam EIRP -> ~15 dB SINR with active error model
    double freqGhz = 2.0;
    double bwMhz = 50.0;
    std::string outputDir = "ntn-real-stack-smoke-out";

    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation time [s]", simTime);
    cmd.AddValue("numUes", "Number of ground UEs", numUes);
    cmd.AddValue("altKm", "Satellite altitude [km]", altKm);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power [dBm]", satEirpDbm);
    cmd.AddValue("freqGhz", "Carrier frequency [GHz]", freqGhz);
    cmd.AddValue("bwMhz", "Bandwidth [MHz]", bwMhz);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    // gNB = one LEO satellite directly overhead; UEs spread on the ground.
    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    // Real NTN mobility: SGP4 Walker serving satellite + TR 38.811 UEs under
    // its t=0 sub-point (UE+sat share the ECEF frame; the pass is genuine).
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = altKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto wElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> servSatMob =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    servSatMob->SetElements(wElements[0]);
    satNodes.Get(0)->AggregateObject(servSatMob);
    double subLat, subLon, subAlt;
    servSatMob->GetGeodetic(subLat, subLon, subAlt);
    NtnTr38811MobilityHelper ueMobility(1);
    auto mobProfile = NtnMobilityScenarios::MixedContinental();
    ueMobility.Install(ueNodes, mobProfile, subLat - 0.03, subLat + 0.03,
                       subLon - 0.03, subLon + 0.03);

    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simTime));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("smoke");
    rs.SetCarrierFrequencyHz(freqGhz * 1e9);
    rs.SetBandwidthHz(bwMhz * 1e6);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0),
                      Seconds(simTime - 0.5));

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();
    Simulator::Destroy();
    return 0;
}
