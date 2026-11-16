/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-constellation-sgp4-mobility-traffic — a single LEO satellite from a TLE,
// propagated with the full Vallado SGP4 backend (drag/B* included, enabled via
// SetUseVallado), drives a REAL mmwave NR NTN cell (NtnRealStackHelper:
// SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC) toward a ground terminal,
// while the ContactGraphScheduler tracks GSL up/down events from the live
// Vallado SGP4 geometry.
// The ground station is auto-placed at the satellite's sub-point at mid-run, so
// the run is centred on the zenith of a real pass (elevation climbs to ~90 deg
// and falls back; a full horizon-to-horizon rise and set takes ~10 min, so it
// needs --simSeconds of that order), and the radio KPIs (SINR/TBLER/throughput) are
// MEASURED off the mmwave PHY trace — no closed-form SINR, no P2P star.
//
// Quick test:  --simSeconds=20 --tle=contrib/ntn-rrc/data/iss-zarya.tle

#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"

#include "ns3/contact-graph-scheduler.h"
#include "ns3/sgp4-mobility-model.h"

#include <cmath>
#include <cstdio>
#include <fstream>

using namespace ns3;
using ns3::ntncon::ContactGraphScheduler;
using ns3::ntncon::Sgp4MobilityModel;
using ns3::ntncon::TleRecord;

namespace
{
bool
ReadThreeLineTle(const std::string& path, TleRecord& tle)
{
    std::ifstream f(path);
    if (!f)
    {
        return false;
    }
    if (!std::getline(f, tle.name) || !std::getline(f, tle.line1) ||
        !std::getline(f, tle.line2))
    {
        return false;
    }
    return tle.line1.size() >= 60 && tle.line2.size() >= 60;
}

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 20.0;
    uint32_t numUes = 4;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1, 30 kHz SCS) | "mmwave" (FR2)
    std::string tlePath;
    double gsLatDeg = std::nan("");
    double gsLonDeg = std::nan("");
    double minElevDeg = 10.0;
    std::string outputDir = "ntn-constellation-sgp4-output";

    // Wave 1 (C5): feeder from a real gateway north of the UE site (WGS-84).

    double gwDistanceKm = 500.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("numUes", "Number of ground UEs", numUes);
    cmd.AddValue("satEirpDbm", "-1 = TR 38.821 Set-1 S-band EIRP density (34 dBW/MHz); otherwise conducted power at the array input (dBm)", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (5G-LENA FR1, 30 kHz SCS) | mmwave (FR2)", radio);
    cmd.AddValue("tle", "Path to 3-line TLE (default: contrib/ntn-rrc/data/iss-zarya.tle)", tlePath);
    cmd.AddValue("gsLat", "Ground-station latitude (deg; default = sat sub-point)", gsLatDeg);
    cmd.AddValue("gsLon", "Ground-station longitude (deg; default = sat sub-point)", gsLonDeg);
    cmd.AddValue("minElev", "Min elevation for in-contact (deg)", minElevDeg);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("gwDistanceKm",

                 "Gateway ground distance north of the UE site (km); <= 0 keeps the 5 ms "

                 "constant backhaul",

                 gwDistanceKm);

    cmd.Parse(argc, argv);

    const bool useNr = (radio != "mmwave");
    // EIRP default (NT-02 calibration). The nr default used to be 70 dBm of
    // CONDUCTED power, about 88 dBm EIRP once the ~18 dB array gain is added,
    // ~11 dB above the TR 38.821 Table 6.1.1.1-1 Set-1 S-band payload
    // (34 dBW/MHz, 77 dBm over 20 MHz). On an S-band FR1 carrier the nr
    // default is now that Set-1 density, as ntn-v2x-real-stack does; an
    // explicit --satEirpDbm keeps meaning conducted power, and mmwave keeps
    // its historical 58.0 dBm.
    // The probe mirrors the cell's carrier and bandwidth and returns the
    // equivalent CONDUCTED power from the helper's own formula
    // (density + 10log10(BW_MHz) + 30 - ArrayGainDb()), so every other
    // consumer of satEirpDbm stays consistent with the radio.
    NtnRealStackHelper eirpProbe;
    const bool eirpFromTr38821 = satEirpDbm < 0.0 && useNr &&
                                 eirpProbe.GetCarrierFrequencyHz() > 1.0e9 &&
                                 eirpProbe.GetCarrierFrequencyHz() <= 7.125e9;
    if (eirpFromTr38821)
    {
        eirpProbe.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        satEirpDbm = eirpProbe.GetSatEirpDbm();
    }
    else if (satEirpDbm < 0.0)
    {
        satEirpDbm = useNr ? 70.0 : 58.0;
    }

    if (tlePath.empty())
    {
        for (const std::string& c : {
                 std::string("contrib/ntn-rrc/data/iss-zarya.tle"),
                 std::string("../contrib/ntn-rrc/data/iss-zarya.tle"),
                 std::string("../../contrib/ntn-rrc/data/iss-zarya.tle"),
             })
        {
            std::ifstream p(c);
            if (p)
            {
                tlePath = c;
                break;
            }
        }
    }
    TleRecord tle;
    if (!ReadThreeLineTle(tlePath, tle))
    {
        std::fprintf(stderr, "error: could not read 3-line TLE from %s\n", tlePath.c_str());
        return 2;
    }

    auto sat = CreateObject<Sgp4MobilityModel>();
    if (!sat->SetTle(tle))
    {
        std::fprintf(stderr, "error: TLE parse failed\n");
        return 2;
    }
    // This example propagates a raw two-line element set, so select the full
    // Vallado SGP4 backend (atmospheric drag / B* included) — the accurate
    // propagator for a low-drag ISS-class TLE. If the Vallado init fails the
    // model falls back to the Kepler+J2-secular path automatically.
    sat->SetUseVallado(true);
    // Sub-point at mid-run, not at t=0: placing the station under the t=0
    // sub-point made every run start at zenith and only ever set, so the
    // "rise, zenith, set" the example describes never had its rising half.
    Vector midPos;
    Vector midVel;
    sat->GetEcefStateAt(Seconds(0.5 * simSeconds), midPos, midVel);
    double subLat, subLon, subAlt;
    ntngeo::EcefToGeodetic(midPos, subLat, subLon, subAlt);
    if (std::isnan(gsLatDeg) || std::isnan(gsLonDeg))
    {
        gsLatDeg = subLat;
        gsLonDeg = subLon;
    }

    // ---- nodes: the Vallado SGP4 satellite (real mmwave gNB) + ground UEs ----
    NodeContainer satNodes;
    satNodes.Create(1);
    satNodes.Get(0)->AggregateObject(sat);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    // TR 38.811 class UEs (real MobilityModel) around the ground station.
    NtnTr38811MobilityHelper ueMobility(1);
    auto mobProfile = NtnMobilityScenarios::MixedContinental();
    ueMobility.Install(ueNodes, mobProfile, gsLatDeg - 0.03, gsLatDeg + 0.03,
                       gsLonDeg - 0.03, gsLonDeg + 0.03);

    // ---- GSL contact tracking from the live geometry ----
    auto scheduler = CreateObject<ContactGraphScheduler>();
    scheduler->SetMinElevationDeg(minElevDeg);
    scheduler->RegisterGroundStation(0, gsLatDeg, gsLonDeg);
    scheduler->RegisterSatellite(100, sat);
    scheduler->Start();

    // ---- real NR cell + measured traffic (mmwave FR2 or nr FR1) ----
    NtnRealStackHelper rs;
    rs.SetRadioBackend(useNr ? NtnRealStackHelper::RadioBackend::Nr
                             : NtnRealStackHelper::RadioBackend::Mmwave);
    if (useNr)
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-constellation-sgp4-mobility-traffic");
    if (eirpFromTr38821)
    {
        // NT-02: TR 38.821 Table 6.1.1.1-1 Set-1 S-band EIRP density. The
        // helper back-computes the conducted power from the bandwidth and
        // the array gain, and again in Build().
        // C4/C7: TR 38.821 Set-1 terminal and payload (34 dBW/MHz, handheld 1T2R, NF 7 dB).

        rs.ApplyTr38821Set1Defaults();
    }
    else
    {
        // NT-02: an explicit value (or a default on a carrier outside the
        // Set-1 S-band reference) is CONDUCTED power at the array input.
        rs.SetSatConductedPowerDbm(satEirpDbm);
    }
    rs.Build(satNodes, ueNodes);
    if (gwDistanceKm > 0.0)
    {
        rs.SetFeederGatewayGeodetic(subLat + gwDistanceKm / 111.195, subLon, 0.0);
    }
    // satEirpDbm was resolved before Build() and handed to the rest of the
    // example; make sure the radio landed on the same conducted power.
    NS_ABORT_MSG_IF(eirpFromTr38821 && std::abs(rs.GetSatEirpDbm() - satEirpDbm) > 1e-6,
                    "Set-1 conducted power mismatch: radio "
                        << rs.GetSatEirpDbm() << " dBm vs " << satEirpDbm << " dBm");
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("ntn-constellation-sgp4-mobility-traffic"); // WS2 KPM series (TS 28.552 names)

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    scheduler->Stop();
    rs.Collect();
    rs.WriteHealthReport();
    const uint64_t gslUp = scheduler->GslEventsUp();
    const uint64_t gslDown = scheduler->GslEventsDown();

    std::printf("# === ntn-constellation-sgp4-mobility-traffic summary ===\n"
                "#   TLE: %s   GS(lat,lon)=(%.4f, %.4f)  minElev=%.1f deg\n"
                "#   measured SINR=%.2f dB  measured throughput=%.3f Mbps  GSL up=%lu down=%lu\n",
                tlePath.c_str(), gsLatDeg, gsLonDeg, minElevDeg, rs.GetMeanDlSinrDb(),
                rs.GetRxThroughputMbps(), static_cast<unsigned long>(gslUp),
                static_cast<unsigned long>(gslDown));

    Simulator::Destroy();
    return 0;
}
