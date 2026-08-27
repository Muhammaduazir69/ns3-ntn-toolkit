// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-real-stack-smoke — Phase-0 validation of NtnRealStackHelper.
//
// Stands up a single LEO gNB (satellite) over a small set of ground UEs using a
// REAL mmwave NR air interface (SpectrumPhy + MAC + HARQ + RLC/PDCP + EPC),
// runs UDP traffic over the radio, and reports MEASURED SINR/TBLER/throughput
// from PHY trace sources + FlowMonitor. This is the template every NTN module
// adopts in Phase 1+ (see 2026-06 protocol-fidelity audit.md).
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
    bool strictGates = false;
    double simTime = 10.0;
    uint32_t numUes = 4;
    double altKm = 600.0;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    double freqGhz = 2.0;
    double bwMhz = 50.0;
    std::string radio = "nr"; // radio backend: nr (FR1) or mmwave
    std::string outputDir = "ntn-real-stack-smoke-out";
    // NT-07: two TR 38.811 features shipped with no way to reach them from a
    // scenario. SetSatelliteBeam and SetNtnScenario existed on the helper and
    // had zero callers anywhere in the tree, so the 6.4.1 beam pattern never
    // entered any propagation chain and every run in the toolkit silently used
    // the Suburban shadow-fading bins.
    bool satBeam = false;
    double beamwidthDeg = 4.4127; // TR 38.821 Table 6.1.1.1-1 Set-1, LEO-600 S-band
    double beamCenterXKm = 0.0;   // fixed beam centre offset along +x; 0 = track the UEs
    std::string ntnScenario = "suburban";

    CommandLine cmd;
    cmd.AddValue("strictGates",
                 "WF-07: abort the run if any fidelity gate fails, instead of printing FAIL and "
                 "carrying on. SetStrictGates had no callers anywhere, so every gate the helper "
                 "evaluates could only ever warn.",
                 strictGates);
    cmd.AddValue("simTime", "Simulation time [s]", simTime);
    cmd.AddValue("numUes", "Number of ground UEs", numUes);
    cmd.AddValue("altKm", "Satellite altitude [km]", altKm);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power [dBm]; -1 = backend default", satEirpDbm);
    cmd.AddValue("freqGhz", "Carrier frequency [GHz]", freqGhz);
    cmd.AddValue("bwMhz", "Bandwidth [MHz]", bwMhz);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("satBeam",
                 "NT-07: chain the TR 38.811 6.4.1 satellite beam pattern (off-boresight "
                 "roll-off) onto the measured channel. Off by default: turning it on moves "
                 "every measured SINR in the run.",
                 satBeam);
    cmd.AddValue("beamwidthDeg", "3 dB beamwidth for --satBeam [deg]", beamwidthDeg);
    cmd.AddValue("beamCenterXKm",
                 "Fixed beam-centre offset along +x [km] for --satBeam. 0 keeps the beam "
                 "tracking the terminal, which puts theta at 0 and the roll-off at 0 dB - "
                 "that is exactly why a steered-beam calibration reports a constant offset.",
                 beamCenterXKm);
    cmd.AddValue("ntnScenario",
                 "NT-07: TR 38.811 deployment scenario selecting the shadow-fading sigma bins: "
                 "denseurban | urban | suburban | rural. Clutter is 0 dB in LOS per 6.6.2, so "
                 "sigma is what the scenario actually changes.",
                 ntnScenario);
    cmd.Parse(argc, argv);

    // nr's Friis LEO link needs ~70 dBm for a healthy SINR; mmwave keeps 55.
    if (satEirpDbm < 0.0)
    {
        satEirpDbm = (radio == "mmwave") ? 55.0 : 70.0;
    }

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
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simTime));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("smoke");
    rs.SetCarrierFrequencyHz(freqGhz * 1e9);
    rs.SetBandwidthHz(bwMhz * 1e6);
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(satEirpDbm);

    // NT-07: apply the two TR 38.811 options before Build(), which is where the
    // helper assembles the propagation chain.
    {
        uint8_t sc = 2; // Suburban
        if (ntnScenario == "denseurban") { sc = 0; }
        else if (ntnScenario == "urban") { sc = 1; }
        else if (ntnScenario == "suburban") { sc = 2; }
        else if (ntnScenario == "rural") { sc = 3; }
        else { NS_ABORT_MSG("unknown --ntnScenario '" << ntnScenario << "'"); }
        rs.SetNtnScenario(sc);
    }
    if (satBeam)
    {
        Ptr<MobilityModel> beamCenter;
        if (beamCenterXKm != 0.0)
        {
            Ptr<ConstantPositionMobilityModel> c = CreateObject<ConstantPositionMobilityModel>();
            c->SetPosition(Vector(beamCenterXKm * 1e3, 0.0, 0.0));
            beamCenter = c;
        }
        rs.SetSatelliteBeam(beamwidthDeg, beamCenter);
    }

    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0),
                      Seconds(simTime - 0.5));

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    rs.Collect();
    rs.SetStrictGates(strictGates);
    rs.WriteHealthReport();
    Simulator::Destroy();
    return 0;
}
