/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Roadmap §4.4.9)
 *
 * sagin-flight-leo-e2 — commercial long-haul flight served by a passing LEO
 * satellite over a REAL mmwave NR NTN cell (NtnRealStackHelper), with OAI-style
 * O-RAN E2-KPM indications emitted from the satellite gNB to a Near-RT RIC. Each
 * reporting period the E2KpmReport is filled with the MEASURED DL SINR / TBLER /
 * throughput off the mmwave PHY trace (not a closed-form FSPL->SINR or Shannon
 * bound); the elevation/Doppler/delay geometry remains real. A mid-pass elevation
 * descent drops the measured SINR below the in-service threshold so the handover
 * CSV shows a real RELEASE event.
 *
 * Run:
 *   ./ns3 run "sagin-flight-leo-e2 --simSeconds=30 --reportMs=500"
 */
#include "ns3/command-line.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"

#include "ns3/aeronautical-scenario.h"
#include "ns3/oran-ntn-e2-interface.h"
#include "ns3/oran-ntn-types.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SaginFlightLeoE2");

namespace
{
constexpr double kC = 299792458.0;
constexpr double kFreqHz = 20.0e9; // Ka-band downlink

double
Distance(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double
ElevDeg(const Vector& ue, const Vector& sat)
{
    const Vector d(sat.x - ue.x, sat.y - ue.y, sat.z - ue.z);
    const double horiz = std::sqrt(d.x * d.x + d.y * d.y);
    return std::atan2(d.z, std::max(horiz, 1e-3)) * 180.0 / M_PI;
}

double
DopplerHz(const Vector& uePos, const Vector& satPos, const Vector& satVel, double fHz)
{
    Vector los(satPos.x - uePos.x, satPos.y - uePos.y, satPos.z - uePos.z);
    const double n = std::max(1e-3, Distance(satPos, uePos));
    los.x /= n; los.y /= n; los.z /= n;
    const double vRadial = satVel.x * los.x + satVel.y * los.y + satVel.z * los.z;
    return -(vRadial / kC) * fHz;
}

struct Context
{
    NtnRealStackHelper* rs;
    Ptr<MobilityModel> aircraft;
    Ptr<MobilityModel> sat;
    Ptr<OranNtnE2Node> e2node;
    double minElevDeg;
    bool inService;
    uint32_t handovers;
    std::ofstream* hoLog;
    uint8_t sliceId;
    uint64_t* indicationCount;
    double simTime;
    uint64_t lastRxBytes = 0; //!< for live per-window throughput measurement
};

void
EmitKpm(Context* ctx, Time reportPeriod)
{
    if (Simulator::Now().GetSeconds() >= ctx->simTime)
    {
        return;
    }
    const Vector ue = ctx->aircraft->GetPosition();
    const Vector ueVel = ctx->aircraft->GetVelocity();
    const Vector sat = ctx->sat->GetPosition();
    const Vector satVel = ctx->sat->GetVelocity();
    const double range = Distance(ue, sat);
    const double elev = ElevDeg(ue, sat);

    // MEASURED radio off the real mmwave PHY (UE 0).
    const double measSinr = ctx->rs->GetUeRecentSinrDb(0);
    const double measTbler = ctx->rs->GetUeRecentTbler(0);
    const bool haveMeas = !std::isnan(measSinr);

    // Handover-style event: in-service follows BOTH visibility and a decodable
    // measured link (SINR above the floor while above min elevation).
    const bool nowInService = (elev >= ctx->minElevDeg) && (haveMeas && measSinr > 0.0);
    if (nowInService != ctx->inService)
    {
        ++ctx->handovers;
        if (ctx->hoLog && ctx->hoLog->is_open())
        {
            (*ctx->hoLog) << Simulator::Now().GetSeconds() << ","
                          << (nowInService ? "ACQUIRE" : "RELEASE") << "," << elev << "," << range
                          << "\n";
        }
        ctx->inService = nowInService;
    }

    // E2-KPM report filled with MEASURED radio + real geometry.
    E2KpmReport r{};
    r.timestamp = Simulator::Now().GetSeconds();
    r.gnbId = ctx->e2node->GetNodeId();
    r.isNtn = true;
    r.ueId = 1;
    r.sinr_dB = haveMeas ? measSinr : -100.0;
    r.rsrp_dBm = r.sinr_dB - 95.0;
    const double sinrLin = std::pow(10.0, r.sinr_dB / 10.0);
    r.rsrq_dB = std::max(-19.5, std::min(-3.0, 10.0 * std::log10(sinrLin / (1.0 + sinrLin))));
    r.cqi = static_cast<uint8_t>(std::clamp(r.sinr_dB / 2.0 + 7.0, 0.0, 15.0));
    // LIVE measured throughput over the report window: GetRxThroughputMbps()
    // is an end-of-run aggregate (computed in Collect()) and reads 0 during
    // the run — the regeneration sweep caught this column stuck at zero.
    const uint64_t rxNow = ctx->rs->GetUeRxBytes(0);
    const double winS = reportPeriod.GetSeconds();
    r.throughput_Mbps =
        (winS > 0.0) ? (rxNow - ctx->lastRxBytes) * 8.0 / winS / 1e6 : 0.0;
    ctx->lastRxBytes = rxNow;
    r.latency_ms = (range / kC) * 1000.0;
    r.elevation_deg = elev;
    const Vector relVel(satVel.x - ueVel.x, satVel.y - ueVel.y, satVel.z - ueVel.z);
    r.doppler_Hz = DopplerHz(ue, sat, relVel, kFreqHz);
    r.propagationDelay_ms = (range / kC) * 1000.0;
    r.beamId = 1;
    r.sliceId = ctx->sliceId;
    r.sliceThroughput_Mbps = r.throughput_Mbps;
    r.sliceLatency_ms = r.latency_ms;
    r.sliceReliability = nowInService ? (1.0 - measTbler) : 0.0;

    ctx->e2node->SubmitKpmMeasurement(r);
    ++(*ctx->indicationCount);

    Simulator::Schedule(reportPeriod, &EmitKpm, ctx, reportPeriod);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 30.0;
    uint32_t reportMs = 500;
    double minElevDeg = 10.0;
    double cruiseAltM = 11000.0;
    double cruiseSpeed = 250.0;
    double satAltM = 550000.0;
    double satSpeed = 7500.0;
    double satEirpDbm = 80.0; // Ka-band budget
    uint8_t sliceId = 0;
    std::string outputDir = "sagin-flight-leo-e2-output";
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1) | "mmwave" (FR2)

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("reportMs", "E2-KPM reporting period (ms)", reportMs);
    cmd.AddValue("minElevDeg", "Min elevation for in-service (deg)", minElevDeg);
    cmd.AddValue("cruiseAltM", "Aircraft cruise altitude (m)", cruiseAltM);
    cmd.AddValue("cruiseSpeed", "Aircraft cruise speed (m/s)", cruiseSpeed);
    cmd.AddValue("satAltM", "Satellite altitude (m)", satAltM);
    cmd.AddValue("satSpeed", "Satellite ground-track speed (m/s)", satSpeed);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    // Aircraft = UE (great-circle leg); LEO satellite = mmwave gNB with the E2 node.
    NodeContainer ueNodes;
    ueNodes.Create(1);
    Ptr<AeronauticalMobilityModel> aircraft = CreateObject<AeronauticalMobilityModel>();
    aircraft->SetFlightPlan(Vector{0, 0, cruiseAltM},
                            Vector{simSeconds * cruiseSpeed, 0, cruiseAltM}, cruiseAltM,
                            cruiseSpeed);
    ueNodes.Get(0)->AggregateObject(aircraft);

    NodeContainer satNodes;
    satNodes.Create(1);
    // Real SGP4 orbit projected into the scenario's local ENU frame: the
    // satellite passes overhead near t=0 and recedes with genuine orbital
    // dynamics (no straight-line placeholder).
    ns3::ntncon::WalkerConfig wcfgSat;
    wcfgSat.num_planes = 1;
    wcfgSat.total_sats = 80;
    wcfgSat.altitude_km = satAltM / 1000.0;
    wcfgSat.inclination_deg = 53.0;
    wcfgSat.epoch_unix_s = 1735689600.0;
    const auto satElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfgSat);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(satElements[0]);
    double satSubLat, satSubLon, satSubAlt;
    satSgp4->GetGeodetic(satSubLat, satSubLon, satSubAlt);
    Ptr<NtnEnuProjectionMobilityModel> sat = CreateObject<NtnEnuProjectionMobilityModel>();
    sat->SetSource(satSgp4);
    sat->SetReference(satSubLat, satSubLon, 0.0);
    satNodes.Get(0)->AggregateObject(sat);

    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("sagin-flight-leo-e2");
    rs.SetCarrierFrequencyHz(kFreqHz);
    rs.SetSatEirpDbm(satEirpDbm); // 80 dBm Ka-band budget — healthy for the nr LEO link too
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("sagin-flight-leo-e2"); // WS2 KPM series (TS 28.552 names)

    // E2 node on the satellite gNB + one periodic KPM subscription.
    Ptr<OranNtnE2Node> e2 = CreateObject<OranNtnE2Node>();
    e2->SetNodeId(42);
    e2->SetIsNtn(true);
    e2->SetFeederLinkDelay(MilliSeconds(2));
    e2->RegisterRanFunction(2, "E2SM-KPM v03.00");
    E2Subscription sub{};
    sub.subscriptionId = 1001;
    sub.ricRequestorId = 1;
    sub.ranFunctionId = 2;
    sub.reportingPeriod = MilliSeconds(reportMs);
    sub.maxBufferAge = Seconds(10);
    e2->HandleSubscriptionRequest(sub);

    std::filesystem::create_directories(outputDir);
    std::ofstream kpmOut(outputDir + "/sagin-flight-leo-kpm.csv");
    kpmOut << "t_s,gnbId,ueId,rsrp_dBm,meas_sinr_dB,cqi,elev_deg,doppler_Hz,prop_delay_ms,"
              "meas_throughput_Mbps,sliceId\n";
    uint64_t indicationsDelivered = 0;
    e2->SetIndicationCallback([&kpmOut, &indicationsDelivered](E2Indication ind) {
        const E2KpmReport& r = ind.kpmReport;
        kpmOut << r.timestamp << "," << r.gnbId << "," << r.ueId << "," << r.rsrp_dBm << ","
               << r.sinr_dB << "," << static_cast<unsigned>(r.cqi) << "," << r.elevation_deg << ","
               << r.doppler_Hz << "," << r.propagationDelay_ms << "," << r.throughput_Mbps << ","
               << static_cast<unsigned>(r.sliceId) << "\n";
        ++indicationsDelivered;
    });

    std::ofstream hoOut(outputDir + "/sagin-flight-leo-handover.csv");
    hoOut << "t_s,event,elev_deg,range_m\n";

    Context ctx{&rs,    aircraft, sat,     e2,      minElevDeg,
                false,  0,        &hoOut,  sliceId, &indicationsDelivered, simSeconds};

    Simulator::Schedule(MilliSeconds(reportMs), &EmitKpm, &ctx, MilliSeconds(reportMs));
    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();
    kpmOut.close();
    hoOut.close();

    std::printf("# sagin-flight-leo-e2 (E2-KPM on a real mmwave NR cell)\n"
                "#   sim=%.0f s  reportPeriod=%u ms  measured SINR=%.2f dB  throughput=%.3f Mbps\n"
                "#   KPM indications delivered: %llu  handover-style events: %u\n",
                simSeconds, reportMs, rs.GetMeanDlSinrDb(), rs.GetRxThroughputMbps(),
                (unsigned long long)indicationsDelivered, ctx.handovers);
    Simulator::Destroy();
    return 0;
}
