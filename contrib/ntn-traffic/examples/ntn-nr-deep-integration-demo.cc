// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-nr-deep-integration-demo — exercises the four NR deep-integration
// enablers added to NtnRealStackHelper (2026-07), each of which turns a piece
// of 5G-LENA machinery that the toolkit previously bypassed into a MEASURED
// quantity:
//
//   Enabler A (multi-gNB handover): two satellites (gNBs) in view; the NR
//     A3-RSRP handover algorithm + X2 move a UE to a real neighbour cell on
//     measured RSRP instead of free-space-scaled candidate SINR.
//   Enabler C (slice/QoS): three slices -> three BWPs; the OfdmaQos scheduler
//     and the gNB BWP manager differentiate 5QI (eMBB/URLLC/mMTC), and each
//     flow gets a dedicated per-5QI EPS bearer. Isolation is emergent, and the
//     per-slice (per-BWP) SINR/TB counts are reported.
//   Enabler D (native NR stats): EnableTraces() writes the native NR
//     PDCP/RLC/MAC/PHY stat files, and the helper reports measured MCS, MIMO
//     rank and PRB utilisation captured from the NR RxPacketTrace. The NTN HARQ
//     process pool is stretched to the slant RTT.
//   Enabler B (real MIMO): a larger gNB/UE UniformPlanarArray + NrPmSearchFull
//     rank/PMI adaptation, so MIMO rank is a measured PHY quantity. (The
//     frequency-selective spectrum seam, AddSpectrumChannelLoss(), is exercised
//     by the thz-ntn / ntn-sionna modules with their real channel models.)
//
// Everything below is measured from the live NR data plane; nothing is
// closed-form.

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

NS_LOG_COMPONENT_DEFINE("NtnNrDeepIntegrationDemo");

int
main(int argc, char* argv[])
{
    double simTime = 10.0;     // seconds
    uint32_t numUes = 6;       // ground UEs (2 per slice for MixedBouquet)
    double altitudeKm = 600.0; // LEO altitude
    double satEirpDbm = 70.0;
    double freqGhz = 2.0;      // S-band
    double bwMhz = 30.0;       // 30 MHz -> 3 x 10 MHz BWPs (one per slice)
    std::string outputDir = "./nr-deep-demo/";
    bool slices = true; // Enabler C on/off (diagnostic: isolate multi-BWP)

    CommandLine cmd(__FILE__);
    cmd.AddValue("slices", "Enable per-slice BWPs (Enabler C)", slices);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("numUes", "Number of ground UEs", numUes);
    cmd.AddValue("altitudeKm", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB TxPower (dBm)", satEirpDbm);
    cmd.AddValue("bwMhz", "Channel bandwidth (MHz)", bwMhz);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    // ---- Two satellites (gNBs) in view + ground UEs ---------------------
    NodeContainer gnbNodes;
    gnbNodes.Create(2); // Enabler A needs >= 2 cells
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    // Two satellites in view: sat 0 overhead moving at a realistic LEO
    // ground-track speed, sat 1 a neighbour cell 20 km to the side. The A3-RSRP
    // handover algorithm + X2 (Enabler A) are installed, armed and fully wired
    // (the helper cross-wires the A3 algorithm to each gNB RRC, which the
    // vendored NrHelper itself omits), so the UE emits real neighbour RSRP
    // reports and the A3 event will trigger an X2 handover once the neighbour
    // RSRP crosses the serving cell by the hysteresis. On THIS demo's geometry
    // -- sat 1 only 20 km to the side over a short sim -- both slant ranges hug
    // 600 km and that crossover is not reached, so no handover fires here; that
    // is expected and is exactly why Rel-17 NTN prefers elevation/time
    // CondEvents (carried by ntn-cho). The A3/X2 machinery firing end to end on
    // a realistic 600 km LEO pass is verified by the dedicated
    // ntn-nr-handover-pass example.
    // Two REAL LEO satellites from a Walker-Delta shell: adjacent members of the
    // same plane, each SGP4-propagated on a genuine ~600 km / 53-deg orbit at
    // ~7.5 km/s (TR 38.821 LEO regime) — not ConstantVelocity straight lines.
    WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 6;
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0; // 2025-01-01T00:00:00Z
    const auto elements = WalkerConstellation::BuildDelta(wcfg);
    Ptr<Sgp4MobilityModel> sat0 = CreateObject<Sgp4MobilityModel>();
    sat0->SetElements(elements[0]);
    Ptr<Sgp4MobilityModel> sat1 = CreateObject<Sgp4MobilityModel>();
    sat1->SetElements(elements[1]);
    gnbNodes.Get(0)->AggregateObject(sat0);
    gnbNodes.Get(1)->AggregateObject(sat1);

    // Ground UEs under the serving satellite's sub-point, TR 38.811 mobility.
    double subLat, subLon, subAlt;
    sat0->GetGeodetic(subLat, subLon, subAlt);
    NtnTr38811MobilityHelper ueMobility(1);
    auto ueProfile = NtnMobilityScenarios::MixedContinental();
    ueMobility.Install(ueNodes, ueProfile, subLat - 0.03, subLat + 0.03,
                       subLon - 0.03, subLon + 0.03);

    // ---- Build the NR radio with all four enablers ----------------------
    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simTime));
    rs.SetOutputDir(outputDir);
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

    // Enabler C: three slices -> three BWPs + OfdmaQos + per-5QI bearers.
    if (slices)
    {
        rs.SetSlices({{"eMBB", 2}, {"URLLC", 82}, {"mMTC", 9}});
    }
    // Enabler A: NR inter-cell handover across the two satellites.
    rs.SetHandover(true, /*hysteresisDb=*/3.0, /*ttt=*/MilliSeconds(256));
    // Enabler D: native NR stat files + NTN-stretched HARQ pool.
    rs.SetNrNativeTraces(true);
    rs.SetNtnHarqProfile(true);
    // Enabler B: real NR MIMO (2 streams) over a larger UE array.
    rs.SetMimo(/*gnbRows=*/8, /*gnbCols=*/8, /*ueRows=*/2, /*ueCols=*/2, /*rankLimit=*/2);

    rs.Build(gnbNodes, ueNodes);
    // MixedBouquet assigns eMBB(5QI2)/URLLC(5QI82)/mMTC(5QI9) round-robin -> each
    // slice's dedicated bearer routes it to its BWP.
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet,
                      Seconds(0.5),
                      Seconds(simTime - 0.2));

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    rs.Collect();
    Simulator::Destroy();

    std::cout << "\n========== NR deep-integration — MEASURED from the live plane ==========\n";
    std::cout << "  gNBs (satellites)   : " << gnbNodes.GetN() << "   UEs: " << rs.GetNumUes() << "\n";
    std::cout << "  Carrier / BW        : " << freqGhz << " GHz / " << bwMhz << " MHz\n";
    std::cout << "  ----------------------------------------------------------------------\n";
    std::cout << "  Mean DL SINR        : " << rs.GetMeanDlSinrDb() << " dB\n";
    std::cout << "  Mean DL TBLER       : " << rs.GetMeanDlTbler() << "\n";
    std::cout << "  PHY RX transport TB : " << rs.GetPhyRxTb() << "\n";
    std::cout << "  Measured DL thrpt   : " << rs.GetRxThroughputMbps() << " Mbps\n";
    std::cout << "  [D] Mean DL MCS     : " << rs.GetMeanDlMcs() << "   (NR RxPacketTrace)\n";
    std::cout << "  [D] Mean MIMO rank  : " << rs.GetMeanDlRank() << "   (NrPmSearchFull)\n";
    std::cout << "  [D] Mean PRB util   : " << rs.GetMeanPrbUtil() << "\n";
    std::cout << "  [A] Handovers done  : " << rs.GetHandoverCount() << "\n";
    std::cout << "  [C] Per-slice (BWP) measured SINR / TB:\n";
    const char* label[3] = {"eMBB ", "URLLC", "mMTC "};
    for (uint8_t b = 0; b < 3; ++b)
    {
        std::cout << "        BWP " << static_cast<int>(b) << " (" << label[b]
                  << "): SINR=" << rs.GetBwpMeanSinrDb(b) << " dB, TB=" << rs.GetBwpRxTb(b) << "\n";
    }
    std::cout << "======================================================================\n\n";

    if (rs.GetPhyRxTb() == 0)
    {
        std::cerr << "ERROR: no transport blocks decoded — the NR link carried no data.\n";
        return 1;
    }
    return 0;
}
