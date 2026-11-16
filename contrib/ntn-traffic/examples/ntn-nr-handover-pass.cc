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

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>

using namespace ns3;

namespace
{

/// Per-UE serving-cell statistics, sampled every second: serving cell, RNTI,
/// cumulative DL TBs (RAN counter), last TB SINR / TBLER and the linear mean
/// SINR of the TBs received in the last second. All are keyed by the serving
/// cell (nr patch 10), so they keep updating after a handover.
void
SampleUeStats(NtnRealStackHelper* rs,
              std::shared_ptr<std::ofstream> csv,
              uint32_t numUes,
              Time period,
              Time stop)
{
    for (uint32_t u = 0; u < numUes; ++u)
    {
        const auto c = rs->GetUeRanCounters(u);
        *csv << std::fixed << std::setprecision(3) << Simulator::Now().GetSeconds() << "," << u
             << "," << rs->GetUeServingCellId(u) << "," << rs->GetUeRnti(u) << "," << c.dlTb
             << "," << c.dlTbErr << "," << std::setprecision(3) << rs->GetUeRecentSinrDb(u) << ","
             << rs->TakeUeSinrWindowMeanDb(u) << "," << std::setprecision(6)
             << rs->GetUeRecentTbler(u) << "\n";
    }
    if (Simulator::Now() + period <= stop)
    {
        Simulator::Schedule(period, &SampleUeStats, rs, csv, numUes, period, stop);
    }
}

} // namespace
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
    double satEirpDbm = -1.0; // sentinel: TR 38.821 Set-1 S-band density on an S-band carrier
    double freqGhz = 2.185; // TS 38.101-5 n256 downlink block (2170-2200 MHz). Was 2.0,
                              // which is n256 UPLINK and never conformant as a DL carrier.
    double bwMhz = 10.0;  // single BWP — isolate the handover machinery
    // Hysteresis + a longer time-to-trigger deliberately damp post-handover
    // ping-pong: right after a handover the new serving cell's RSRP filter is
    // briefly unpopulated, and an over-eager reverse trigger under a receding
    // (near-zero-RSRP) old serving cell can storm the source scheduler. 2 dB /
    // 512 ms yields one clean handover on the pass below. The default was 3 dB,
    // which contradicted this note and, on the real geometry (neighbour 4.5 deg
    // behind), is only reached at ~89 s and peaks at ~3.6 dB by 120 s: no margin
    // for shadowing. At 2 dB the A3 entry is at ~68 s and the RSRP gap keeps
    // growing afterwards, so there is no reverse trigger (no ping-pong).
    double hystDb = 3.0;  // damp ping-pong between co-planar sats (matches the dashboard default)
    double tttMs = 512.0;
    // Footprint offset: the neighbour starts this far behind so it rises to
    // overhead as the serving sat flies off, giving a single clean RSRP
    // crossover mid-pass while BOTH cells are still strong (~-85 dBm at the old
    // 70 dBm conducted default; ~14 dB lower at the Set-1 density over 10 MHz,
    // which shifts both cells equally and leaves the A3 crossover in place), not the
    // pathological case where the serving cell is swept into signal loss.
    double vSatKmps = 7.5; // nominal LEO ground-track speed (display only)

    std::string outputDir = "./nr-ho-pass/";
    // Wave 1 (C2/C4/C5): cell-level SS-RSRP on an Earth-fixed cell beam, the
    // TR 38.821 Set-1 handheld terminal, and a real gateway for the feeder.
    bool cellBeam = true;
    bool set1 = true;
    double gwDistanceKm = 500.0; // gateway north of the UE site; <= 0 keeps the 5 ms constant
    std::string channel = "legacy"; // legacy | ns3ntn (C3)
    bool fading = true;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("numUes", "Number of ground UEs", numUes);
    cmd.AddValue("altitudeKm", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("satEirpDbm",
                 "-1 = TR 38.821 Set-1 S-band EIRP density (34 dBW/MHz); otherwise conducted power at the array input (dBm)",
                 satEirpDbm);
    cmd.AddValue("hystDb", "A3 hysteresis (dB)", hystDb);
    cmd.AddValue("tttMs", "A3 time-to-trigger (ms)", tttMs);
    // The output directory was hardcoded to ./nr-ho-pass/, so the example wrote
    // into whatever directory it was launched from and left an nr-ho-pass/ tree
    // in the repo when the sweep ran from the source root. Every other example
    // takes --outputDir.
    cmd.AddValue("outputDir", "Directory for the run's artifacts", outputDir);
    cmd.AddValue("cellBeam",
                 "SSB/control on an Earth-fixed cell beam (TS 38.215 SS-RSRP); 0 = legacy "
                 "quasi-omni control",
                 cellBeam);
    cmd.AddValue("channel", "legacy or ns3ntn (ns-3 mainline TR 38.811 NTN channel)", channel);
    cmd.AddValue("fading", "Small-scale fading on the spatial channel (0 = LOS plane wave)", fading);
    cmd.AddValue("set1", "TR 38.821 Set-1 handheld terminal and payload (S band)", set1);
    cmd.AddValue("gwDistanceKm",
                 "Gateway ground distance north of the UE site (km); <= 0 = 5 ms constant backhaul",
                 gwDistanceKm);
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
    // 80 per plane puts the in-plane neighbours 4.5 deg (~500 km of ground
    // track) apart at 600 km, so both cells are in view and the A3 crossover
    // falls inside the 120 s default. With 6 per plane (60 deg apart) the
    // "neighbour" was ~7000 km away, below the UE's horizon for the whole run,
    // and no handover could ever fire.
    WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = WalkerConstellation::BuildDelta(wcfg);
    Ptr<Sgp4MobilityModel> serv = CreateObject<Sgp4MobilityModel>();
    serv->SetElementsSgp4(elements[0]);
    Ptr<Sgp4MobilityModel> nbrA = CreateObject<Sgp4MobilityModel>();
    nbrA->SetElementsSgp4(elements[1]);
    Ptr<Sgp4MobilityModel> nbrB = CreateObject<Sgp4MobilityModel>();
    nbrB->SetElementsSgp4(elements[wcfg.total_sats - 1]);

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
    rs.SetOutputDir(outputDir);
    rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
    rs.SetNumerology(1);
    rs.SetCarrierFrequencyHz(freqGhz * 1e9);
    rs.SetBandwidthHz(bwMhz * 1e6);
    // EIRP default (NT-02 calibration). This used to be a fixed 70 dBm of
    // CONDUCTED power, about 88 dBm EIRP once the ~18 dB array gain is added,
    // ~11 dB above the TR 38.821 Table 6.1.1.1-1 Set-1 S-band payload
    // (34 dBW/MHz, 77 dBm over 20 MHz). On an S-band FR1 carrier the default
    // is now that Set-1 density and the helper back-computes the conducted
    // power from the bandwidth and the array gain (again in Build()). An
    // explicit --satEirpDbm keeps meaning conducted power at the array input.
    const bool eirpFromTr38821 = satEirpDbm < 0.0 && freqGhz * 1e9 > 1.0e9 && freqGhz * 1e9 <= 7.125e9;
    if (eirpFromTr38821 && set1)
    {
        rs.ApplyTr38821Set1Defaults();
    }
    else if (eirpFromTr38821)
    {
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
    }
    else
    {
        if (satEirpDbm < 0.0)
        {
            satEirpDbm = 70.0; // legacy default, kept for carriers outside Set-1
        }
        rs.SetSatConductedPowerDbm(satEirpDbm);
    }
    rs.SetUeTxPowerDbm(23.0);
    rs.SetBackhaulDelay(MilliSeconds(5)); // used only when no gateway is wired
    rs.SetHandover(true, hystDb, MilliSeconds(static_cast<uint64_t>(tttMs)));
    NS_ABORT_MSG_IF(channel != "legacy" && channel != "ns3ntn", "unknown --channel " << channel);
    rs.SetChannelModel(channel == "ns3ntn" ? NtnRealStackHelper::ChannelModel::Ns3Tr38811
                                           : NtnRealStackHelper::ChannelModel::Legacy);
    rs.SetSmallScaleFading(fading);
    if (cellBeam)
    {
        // Earth-fixed cell centred on the UE site: both satellites steer their
        // SSB beam at it, so the measured SS-RSRP difference is the path
        // difference, not whichever UE each cell happens to be serving.
        Ptr<ConstantPositionMobilityModel> cellCenter = CreateObject<ConstantPositionMobilityModel>();
        cellCenter->SetPosition(NtnRealStackHelper::GeodeticToEcef(subLat, subLon, 0.0));
        rs.SetSsbCellBeam(true, cellCenter);
    }

    rs.Build(gnbNodes, ueNodes);
    // C2: per-UE L1 RSRP samples and every RRC measurement report.
    rs.EnableMeasurementTrace(outputDir + "/ho_rsrp_trace.csv");
    if (gwDistanceKm > 0.0)
    {
        // C5: feeder from a real gateway (WGS-84), tracking the serving satellite.
        rs.SetFeederGatewayGeodetic(subLat + gwDistanceKm / 111.195, subLon, 0.0);
    }
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(0.5),
                      Seconds(simTime - 0.2));
    auto ueCsv = std::make_shared<std::ofstream>(outputDir + "/ue_serving_stats.csv");
    *ueCsv << "time_s,ue,serving_cell,rnti,dl_tb,dl_tb_err,recent_sinr_db,window_sinr_db,"
              "recent_tbler\n";
    Simulator::Schedule(Seconds(1.0),
                        &SampleUeStats,
                        &rs,
                        ueCsv,
                        numUes,
                        Seconds(1.0),
                        Seconds(simTime - 0.5));

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    rs.Collect();
    // Ordering matters, and this was wrong until 2026-09-01: the report was
    // written BEFORE Collect(), which is what harvests the PHY traces, so
    // phy_rx_tb and rx_throughput_mbps came out zero while the application
    // counters showed traffic flowing. A health record claiming the radio
    // carried nothing, in a run where it did, is worse than no record at all.
    rs.WriteHealthReport();

    const uint32_t hoCount = rs.GetHandoverCount();
    Simulator::Destroy();

    std::cout << "\n========== NR handover pass — MEASURED ==========\n";
    std::cout << "  Sat altitude / speed : " << altitudeKm << " km / " << vSatKmps << " km/s\n";
    std::cout << "  A3 hysteresis / TTT  : " << hystDb << " dB / " << tttMs << " ms\n";
    std::cout << "  Sim time             : " << simTime << " s\n";
    std::cout << "  Mean DL SINR         : " << rs.GetMeanDlSinrDb() << " dB\n";
    std::cout << "  PHY RX transport TB  : " << rs.GetPhyRxTb() << "\n";
    std::cout << "  [A] Handovers done   : " << hoCount << "\n";
    std::cout << "  Ping-pong (<= 5 s)   : " << rs.GetPingPongCount(Seconds(5)) << "\n";
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
