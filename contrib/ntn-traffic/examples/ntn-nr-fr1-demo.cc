// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-nr-fr1-demo — proves the 5G-LENA (nr) FR1 NTN radio spine
// (NtnNrStackHelper) closes boundary A5(i): a real NR data plane at FR1
// numerology (30 kHz SCS) on an S-band (2.0 GHz) carrier with 20 MHz BW — the
// FR1 regime the FR2-locked mmwave NtnRealStackHelper cannot reach.
//
// Topology: 1 gNB at ~600 km altitude (LEO), a few ground UEs directly below.
// A saturating UDP DL flow runs remote-host -> each UE for ~2 s. The example
// prints the MEASURED mean DL SINR (from the nr RxPacketTraceUe trace), the
// measured DL TBLER (real error model), and the measured DL throughput
// (FlowMonitor) — no closed-form KPIs.

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-traffic-module.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

using namespace ns3;
using ns3::ntncon::Sgp4MobilityModel;
using ns3::ntncon::WalkerConfig;
using ns3::ntncon::WalkerConstellation;

NS_LOG_COMPONENT_DEFINE("NtnNrFr1Demo");

int
main(int argc, char* argv[])
{
    double simTime = 2.0;       // seconds
    uint32_t numUes = 3;        // ground UEs under the satellite
    double altitudeKm = 600.0;  // LEO altitude
    uint16_t numerology = 1;    // 0 = 15 kHz, 1 = 30 kHz FR1
    double satEirpDbm = 70.0;   // satellite EIRP scalar (gNB TxPower)
    double freqGhz = 2.0;       // S-band
    double bwMhz = 20.0;        // NTN-FR1 max
    std::string outputDir = "./";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("numUes", "Number of ground UEs", numUes);
    cmd.AddValue("altitudeKm", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("numerology", "FR1 numerology (0=15kHz, 1=30kHz)", numerology);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB TxPower (dBm)", satEirpDbm);
    cmd.AddValue("freqGhz", "Carrier frequency (GHz)", freqGhz);
    cmd.AddValue("bwMhz", "Channel bandwidth (MHz)", bwMhz);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    // ---- Nodes + mobility -----------------------------------------------
    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    // ---- gNB = a REAL LEO satellite: SGP4-propagated Walker-Delta orbit,
    //      not a static point. It rides a genuine ~600 km / 53-deg orbit at the
    //      true ~7.5 km/s LEO ground-track speed (TR 38.821 LEO regime). ----
    WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 6;
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0; // 2025-01-01T00:00:00Z
    const auto elements = WalkerConstellation::BuildDelta(wcfg);
    Ptr<Sgp4MobilityModel> satMob = CreateObject<Sgp4MobilityModel>();
    satMob->SetElements(elements[0]);
    gnbNodes.Get(0)->AggregateObject(satMob);

    // ---- Ground UEs: 3GPP TR 38.811 mobility classes, placed under the
    //      serving satellite's t=0 sub-satellite point. ----
    double subLat, subLon, subAlt;
    satMob->GetGeodetic(subLat, subLon, subAlt);
    NtnTr38811MobilityHelper ueMobility(1);
    auto ueProfile = NtnMobilityScenarios::MixedContinental();
    ueMobility.Install(ueNodes, ueProfile, subLat - 0.03, subLat + 0.03,
                       subLon - 0.03, subLon + 0.03);

    // ---- Build the FR1 NR spine -----------------------------------------
    NtnNrStackHelper nr;
    nr.SetSimTime(Seconds(simTime));
    nr.SetOutputDir(outputDir);
    nr.SetCarrierFrequencyHz(freqGhz * 1e9);
    nr.SetBandwidthHz(bwMhz * 1e6);
    nr.SetNumerology(numerology);
    nr.SetSatEirpDbm(satEirpDbm);
    nr.SetUeTxPowerDbm(23.0);
    nr.SetBackhaulDelay(MilliSeconds(5));

    nr.Build(gnbNodes, ueNodes);
    nr.InstallTraffic(Seconds(0.4), Seconds(simTime));

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    nr.Collect();
    Simulator::Destroy();

    std::cout << "\n================ NTN nr FR1 spine — MEASURED KPIs ================\n";
    std::cout << "  Carrier            : " << freqGhz << " GHz (S-band)\n";
    std::cout << "  Bandwidth          : " << bwMhz << " MHz\n";
    std::cout << "  Numerology         : " << nr.GetNumerology() << "  (" << nr.GetScsKhz()
              << " kHz SCS, FR1)\n";
    std::cout << "  Satellite altitude : " << altitudeKm << " km\n";
    std::cout << "  gNB EIRP / TxPower : " << satEirpDbm << " dBm\n";
    std::cout << "  UEs                : " << nr.GetNumUes() << "\n";
    std::cout << "  ----------------------------------------------------------------\n";
    std::cout << "  Mean DL SINR       : " << nr.GetMeanDlSinrDb() << " dB   (RxPacketTraceUe)\n";
    std::cout << "  Mean DL TBLER      : " << nr.GetMeanDlTbler() << "      (real error model)\n";
    std::cout << "  PHY RX transport blocks : " << nr.GetPhyRxTb() << "\n";
    std::cout << "  Measured DL throughput  : " << nr.GetRxThroughputMbps()
              << " Mbps   (FlowMonitor)\n";
    std::cout << "==================================================================\n\n";

    if (nr.GetPhyRxTb() == 0)
    {
        std::cerr << "ERROR: no transport blocks decoded — the FR1 link did not carry data.\n";
        return 1;
    }
    return 0;
}
