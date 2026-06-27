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

using namespace ns3;

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

    // gNB (satellite) high above the origin.
    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.Install(gnbNodes);
    gnbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(
        Vector(0.0, 0.0, altitudeKm * 1000.0));

    // Ground UEs spread within a few km of the sub-satellite point.
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    ueMobility.Install(ueNodes);
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        double x = (static_cast<double>(i) - (numUes - 1) / 2.0) * 1000.0; // 1 km spacing
        ueNodes.Get(i)->GetObject<MobilityModel>()->SetPosition(Vector(x, 0.0, 1.5));
    }

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
