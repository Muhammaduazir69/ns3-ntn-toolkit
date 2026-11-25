/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-rrc-from-tle — W1 + W2 integration: read a real TLE (the kind
// ntn-constellation pulls from CelesTrak), drive a SNS3 SatelliteSGP4Mobility
// Model from it, and run a REAL mmwave NR NTN cell (NtnRealStackHelper) under
// the satellite. The UE is auto-placed at the satellite's t=0 sub-point so a
// real overhead pass occurs; NtnTimingAdvance is logged from the live SGP4
// geometry and the DL SINR/TBLER/throughput are MEASURED off the mmwave PHY
// trace. Output CSV: time_s, sat_x/y/z, slant_km, ta_total_us, drift, meas_sinr.

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"

#include <ctime>
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/ntn-rrc-helper.h"
#include "ns3/ntn-timing-advance.h"
#include "ns3/ntn-ue-location-report.h"
#include "ns3/satellite-sgp4-mobility-model.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace ns3;
using namespace ns3::ntnrrc;

namespace
{
NtnRealStackHelper* g_rs = nullptr;

struct TleFile
{
    std::string name, line1, line2;
};

bool
ReadTle(const std::string& path, TleFile& out)
{
    std::ifstream f(path);
    if (!f)
    {
        return false;
    }
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(f, line))
    {
        if (!line.empty())
        {
            lines.push_back(line);
        }
    }
    if (lines.size() < 3)
    {
        return false;
    }
    out.name = lines[0];
    out.line1 = lines[1];
    out.line2 = lines[2];
    return true;
}

void
SampleStep(Ptr<NtnTimingAdvance> ta,
           Ptr<SatSGP4MobilityModel> sat,
           std::ostream* csv,
           Time step,
           Time stopAt)
{
    if (Simulator::Now() > stopAt)
    {
        return;
    }
    const Vector p = sat->GetPosition();
    const double slant = ta->GetSlantRangeMetres() / 1000.0;
    const double taTotalUs = ta->ComputeTotalTa().GetMicroSeconds();
    const double drift = ta->ComputeTaDriftRate(MilliSeconds(10));
    const double sinr = g_rs ? g_rs->GetUeRecentSinrDb(0) : std::nan("");
    *csv << std::fixed << std::setprecision(3) << Simulator::Now().GetSeconds() << ","
         << std::setprecision(2) << p.x << "," << p.y << "," << p.z << "," << std::setprecision(3)
         << slant << "," << static_cast<long long>(taTotalUs) << "," << std::scientific
         << std::setprecision(3) << (drift * 1e6) << "," << std::fixed << std::setprecision(2)
         << (std::isnan(sinr) ? 0.0 : sinr) << "\n";
    Simulator::Schedule(step, &SampleStep, ta, sat, csv, step, stopAt);
}
} // namespace

int
main(int argc, char* argv[])
{
    std::string tlePath;
    std::string startUtc;
    double simTimeSec = 20.0;
    uint32_t numUes = 4;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    std::string radio = "nr"; // radio spine: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    double stepSec = 1.0;
    bool transparent = true;
    std::string outputDir = "ntn-rrc-from-tle-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("tle", "Path to a 3-line TLE file (name, line1, line2)", tlePath);
    cmd.AddValue("start", "Scenario start UTC, ISO format YYYY-MM-DDTHH:MM:SS", startUtc);
    cmd.AddValue("simTime", "Simulation duration (s)", simTimeSec);
    cmd.AddValue("numUes", "Number of UEs on the serving cell", numUes);
    cmd.AddValue("satEirpDbm", "-1 = TR 38.821 Set-1 S-band EIRP density (34 dBW/MHz); otherwise conducted power at the array input (dBm)", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("step", "Sample period (s)", stepSec);
    cmd.AddValue("transparent", "Transparent (true) vs regenerative (false)", transparent);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    const bool useNr = (radio == "nr");
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
        for (const std::string& candidate : {
                 std::string("contrib/ntn-rrc/data/iss-zarya.tle"),
                 std::string("../contrib/ntn-rrc/data/iss-zarya.tle"),
                 std::string("../../contrib/ntn-rrc/data/iss-zarya.tle"),
             })
        {
            std::ifstream probe(candidate);
            if (probe)
            {
                tlePath = candidate;
                break;
            }
        }
    }
    TleFile tle;
    if (tlePath.empty() || !ReadTle(tlePath, tle))
    {
        std::cerr << "error: --tle is required and must be a 3-line file "
                     "(bundled ISS TLE in contrib/ntn-rrc/data/iss-zarya.tle not found)\n";
        return 2;
    }
    if (startUtc.empty())
    {
        startUtc = "2024-01-01 12:00:00";
    }
    std::string startSpace = startUtc;
    for (auto& ch : startSpace)
    {
        if (ch == 'T')
        {
            ch = ' ';
        }
    }

    // ---- Satellite via SGP4 (SNS3) ----
    Ptr<SatSGP4MobilityModel> satMob = CreateObject<SatSGP4MobilityModel>();
    satMob->SetStartDate(startSpace);
    satMob->SetTleInfo(tle.line1 + "\n" + tle.line2);

    // Auto-place the UE at the satellite's t=0 sub-point so the mmwave cell is in
    // view (a real overhead pass), then build nodes/mobility around it.
    const Vector sat0 = satMob->GetPosition();
    // Sub-point on the WGS84 ellipsoid (the module's own authoritative
    // conversion), not a spherical asin(z/|r|) approximation.
    double subLat, subLon, subAlt;
    ns3::ntnrrc::EcefToGeodeticWgs84(sat0, subLat, subLon, subAlt);
    const Vector ueEcef = ns3::ntnrrc::GeodeticWgs84ToEcef(subLat, subLon, 540.0);

    NodeContainer satNodes;
    satNodes.Create(1);
    satNodes.Get(0)->AggregateObject(satMob);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    // TR 38.811 class UEs (real MobilityModel) around the sub-point.
    NtnTr38811MobilityHelper ueMobility(1);
    auto mobProfile = NtnMobilityScenarios::MixedContinental();
    auto ueModels = ueMobility.Install(ueNodes, mobProfile, subLat - 0.03, subLat + 0.03,
                                       subLon - 0.03, subLon + 0.03);
    Ptr<MobilityModel> ueMob = ueModels[0];

    // ---- real mmwave NR cell + measured traffic ----
    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS (nr backend only)
    }
    rs.SetSimTime(Seconds(simTimeSec));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-rrc-from-tle");
    if (eirpFromTr38821)
    {
        // NT-02: TR 38.821 Table 6.1.1.1-1 Set-1 S-band EIRP density. The
        // helper back-computes the conducted power from the bandwidth and
        // the array gain, and again in Build().
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
    }
    else
    {
        // NT-02: an explicit value (or a default on a carrier outside the
        // Set-1 S-band reference) is CONDUCTED power at the array input.
        rs.SetSatConductedPowerDbm(satEirpDbm);
    }
    rs.Build(satNodes, ueNodes);
    // satEirpDbm was resolved before Build() and handed to the rest of the
    // example; make sure the radio landed on the same conducted power.
    NS_ABORT_MSG_IF(eirpFromTr38821 && std::abs(rs.GetSatEirpDbm() - satEirpDbm) > 1e-6,
                    "Set-1 conducted power mismatch: radio "
                        << rs.GetSatEirpDbm() << " dBm vs " << satEirpDbm << " dBm");
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simTimeSec - 0.5));
    rs.EnableAiFlowMonitor("ntn-rrc-from-tle"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;

    // ---- TA pre-comp from the real SGP4 geometry ----
    NtnRrcHelper helper;
    helper.SetPayloadMode(transparent ? PayloadMode::Transparent : PayloadMode::RegenerativeFull);
    helper.SetReferencePosition(ueEcef);
    // RRC-2c: give the transparent payload its feeder leg, as ntn-rrc-real-stack
    // does. Without a gateway NtnTimingAdvance returns a zero feeder range, so
    // --transparent=1 and --transparent=0 printed the same TA and the payload
    // flag was inert. TS 38.213 section 4.2: the common TA the network signals
    // covers the feeder (gateway to satellite) round trip; TR 38.821 Table
    // 4.2-2 lists the transparent RTT as service plus feeder. The gateway is a
    // fixed ground station 300 km from the t=0 sub-point, in view for the pass.
    if (transparent)
    {
        Ptr<ConstantPositionMobilityModel> gwMob = CreateObject<ConstantPositionMobilityModel>();
        gwMob->SetPosition(ns3::ntnrrc::GeodeticWgs84ToEcef(subLat + 2.7, subLon, 0.0));
        helper.SetGatewayMobility(gwMob);
    }
    Ptr<NtnTimingAdvance> ta = helper.InstallTimingAdvance(ueMob, satMob);

    std::filesystem::create_directories(outputDir);
    std::ofstream csv(outputDir + "/ntn-rrc-from-tle.csv");
    csv << "time_s,sat_x_m,sat_y_m,sat_z_m,slant_km,ta_total_us,ta_drift_rate_us_per_s,"
           "measured_sinr_db\n";
    Simulator::ScheduleNow(&SampleStep, ta, satMob, &csv, Seconds(stepSec), Seconds(simTimeSec));

    Simulator::Stop(Seconds(simTimeSec));
    Simulator::Run();
    rs.Collect();
    // ntn-dataset@1: this example drives the SNS3 SGP4 model, which the helper cannot read,
    // so it declares the element set it loaded (epoch and catalog number from line 1).
    {
        const int yy = std::stoi(tle.line1.substr(18, 2));
        const double doy = std::stod(tle.line1.substr(20, 12));
        std::tm tm0{};
        tm0.tm_year = (yy < 57 ? 2000 + yy : 1900 + yy) - 1900;
        tm0.tm_mday = 1;
        const std::time_t t = timegm(&tm0) + static_cast<std::time_t>((doy - 1.0) * 86400.0);
        char iso[32];
        std::strftime(iso, sizeof iso, "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&t));
        rs.SetTleProvenance(iso, {static_cast<uint32_t>(std::stoul(tle.line1.substr(2, 5)))});
    }
    rs.WriteHealthReport();
    csv.close();

    std::cout << "ntn-rrc-from-tle complete (real SGP4 pass of " << tle.name
              << " over its sub-point).\n"
              << "  UE sub-point: lat " << subLat << ", lon " << subLon << "\n"
              << "  measured mean SINR : " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured throughput: " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  final slant range  : " << ta->GetSlantRangeMetres() / 1000.0 << " km\n"
              << "  csv: " << outputDir << "/ntn-rrc-from-tle.csv\n";

    Simulator::Destroy();
    return 0;
}
