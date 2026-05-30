/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Roadmap §4.4.9)
 *
 * sagin-flight-leo-e2 — commercial long-haul flight served by a passing LEO
 * satellite, with OAI-style O-RAN E2-KPM indications emitted from the
 * satellite gNB to a Near-RT RIC.
 *
 * Topology:
 *   - Aircraft: AeronauticalMobilityModel, great-circle leg at 11 km / 250 m/s
 *   - Satellite: ConstantVelocityMobilityModel, 550 km altitude, passing
 *     overhead the flight track
 *   - Ground gateway at the origin (feeder link reference)
 *   - OranNtnE2Node on the satellite gNB; one periodic KPM subscription
 *     (ranFunctionId = 2) at a configurable reporting period.
 *
 * Each reporting period the example computes the sat→aircraft geometry,
 * fills an E2KpmReport (RSRP from FSPL, elevation, Doppler, one-way delay,
 * slice ID), submits it to the E2 node, and the node's indication callback
 * appends a row to the KPM CSV. A handover-style event is logged whenever
 * the satellite's elevation (as seen from the aircraft) crosses the
 * configurable minimum-elevation threshold (sat rising into / setting out
 * of service).
 *
 * The default 900 s window captures a full LEO pass (rise through the
 * minimum-elevation horizon, zenith, set) so the handover CSV shows one
 * ACQUIRE + one RELEASE event out of the box.
 *
 * Run:
 *   ./ns3 run "sagin-flight-leo-e2 --simSeconds=900 --reportMs=500 --minElevDeg=10"
 */
#include "ns3/aeronautical-scenario.h"
#include "ns3/command-line.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/mobility-model.h"
#include "ns3/multi-layer-router.h"

#include "ns3/oran-ntn-e2-interface.h"
#include "ns3/oran-ntn-types.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SaginFlightLeoE2");

namespace
{

constexpr double kC = 299792458.0;       // m/s
constexpr double kFreqHz = 20.0e9;        // Ka-band downlink
// Ka-band LEO->aircraft downlink budget (3GPP TR 38.821 LEO-class figures).
constexpr double kBwHz = 30.0e6;          // 30 MHz NTN carrier
constexpr double kNoiseFigureDb = 3.0;    // aircraft Ka receiver
constexpr double kSatAntGainDbi = 30.0;   // LEO satellite beam gain
constexpr double kAcAntGainDbi = 30.0;    // aeronautical VSAT gain
constexpr double kSpectralEff = 0.75;     // practical fraction of Shannon

double
FsplDb(double dM, double fHz)
{
    const double d = std::max(dM, 1.0);
    return 20.0 * std::log10(d) + 20.0 * std::log10(fHz / 1e9) + 32.45;
}

double
Distance(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Elevation of `sat` as seen from `ue`, local-ENU (+z up).
double
ElevDeg(const Vector& ue, const Vector& sat)
{
    const Vector d(sat.x - ue.x, sat.y - ue.y, sat.z - ue.z);
    const double horiz = std::sqrt(d.x * d.x + d.y * d.y);
    return std::atan2(d.z, std::max(horiz, 1e-3)) * 180.0 / M_PI;
}

// Radial velocity component (m/s) of `sat` relative to `ue` → Doppler.
double
DopplerHz(const Vector& uePos, const Vector& satPos, const Vector& satVel,
          double fHz)
{
    Vector los(satPos.x - uePos.x, satPos.y - uePos.y, satPos.z - uePos.z);
    const double n = std::max(1e-3, Distance(satPos, uePos));
    los.x /= n; los.y /= n; los.z /= n;
    const double vRadial = satVel.x * los.x + satVel.y * los.y + satVel.z * los.z;
    return -(vRadial / kC) * fHz;
}

struct Context
{
    Ptr<MobilityModel> aircraft;
    Ptr<MobilityModel> sat;
    Ptr<OranNtnE2Node> e2node;
    uint32_t subId;
    double minElevDeg;
    bool inService;        // current satellite-in-view state
    uint32_t handovers;
    std::ofstream* hoLog;
    double txPowerDbm;
    uint8_t sliceId;
    uint64_t* indicationCount;
};

void
EmitKpm(Context* ctx, Time reportPeriod)
{
    const Vector ue = ctx->aircraft->GetPosition();
    const Vector ueVel = ctx->aircraft->GetVelocity();
    const Vector sat = ctx->sat->GetPosition();
    const Vector satVel = ctx->sat->GetVelocity();
    const double range = Distance(ue, sat);
    const double elev = ElevDeg(ue, sat);
    const double pl = FsplDb(range, kFreqHz);

    // Handover-style event: satellite crosses the min-elevation threshold.
    const bool nowInService = (elev >= ctx->minElevDeg);
    if (nowInService != ctx->inService)
    {
        ++ctx->handovers;
        if (ctx->hoLog && ctx->hoLog->is_open())
        {
            (*ctx->hoLog) << Simulator::Now().GetSeconds() << ","
                          << (nowInService ? "ACQUIRE" : "RELEASE") << ","
                          << elev << "," << range << "\n";
        }
        ctx->inService = nowInService;
    }

    // Build the E2-KPM report.
    E2KpmReport r{};
    r.timestamp = Simulator::Now().GetSeconds();
    r.gnbId = ctx->e2node->GetNodeId();
    r.isNtn = true;
    r.ueId = 1;
    // Link budget: RSRP = EIRP + Rx gain - FSPL. Noise-based SINR over the
    // carrier bandwidth, then Shannon-bounded (CQI-mapped) throughput. This
    // replaces the earlier affine-proxy SINR and binary 0/80 throughput so the
    // KPM trace tracks the real pass geometry.
    r.rsrp_dBm = ctx->txPowerDbm + kSatAntGainDbi + kAcAntGainDbi - pl;
    const double noiseDbm = -174.0 + 10.0 * std::log10(kBwHz) + kNoiseFigureDb;
    r.sinr_dB = r.rsrp_dBm - noiseDbm;
    const double sinrLin = std::pow(10.0, r.sinr_dB / 10.0);
    // RSRQ = S/(S+I+N) per RE, clamped to 3GPP [-19.5,-3] dB.
    r.rsrq_dB = std::max(-19.5, std::min(-3.0, 10.0 * std::log10(sinrLin / (1.0 + sinrLin))));
    r.cqi = static_cast<uint8_t>(std::clamp(r.sinr_dB / 2.0 + 7.0, 0.0, 15.0));
    // Shannon capacity (Mbps) gated by visibility and a practical efficiency.
    r.throughput_Mbps =
        nowInService ? kSpectralEff * (kBwHz / 1e6) * std::log2(1.0 + sinrLin) : 0.0;
    r.latency_ms = (range / kC) * 1000.0;
    r.elevation_deg = elev;
    // Relative radial velocity (satellite minus aircraft) drives the Doppler.
    const Vector relVel(satVel.x - ueVel.x, satVel.y - ueVel.y, satVel.z - ueVel.z);
    r.doppler_Hz = DopplerHz(ue, sat, relVel, kFreqHz);
    r.propagationDelay_ms = (range / kC) * 1000.0;
    r.beamId = 1;
    r.sliceId = ctx->sliceId;
    r.sliceThroughput_Mbps = r.throughput_Mbps;
    r.sliceLatency_ms = r.latency_ms;
    r.sliceReliability = nowInService ? 0.99 : 0.0;

    ctx->e2node->SubmitKpmMeasurement(r);
    ++(*ctx->indicationCount);

    Simulator::Schedule(reportPeriod, &EmitKpm, ctx, reportPeriod);
}

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 900.0; // a full LEO pass (horizon→zenith→horizon)
    uint32_t reportMs = 500;
    double minElevDeg = 10.0;
    double cruiseAltM = 11000.0;
    double cruiseSpeed = 250.0;
    double satAltM = 550000.0;
    double satSpeed = 7500.0;
    uint8_t sliceId = 0; // eMBB
    std::string kpmCsv = "sagin-flight-leo-kpm.csv";
    std::string hoCsv = "sagin-flight-leo-handover.csv";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("reportMs", "E2-KPM reporting period (ms)", reportMs);
    cmd.AddValue("minElevDeg", "Min elevation for in-service (deg)", minElevDeg);
    cmd.AddValue("cruiseAltM", "Aircraft cruise altitude (m)", cruiseAltM);
    cmd.AddValue("cruiseSpeed", "Aircraft cruise speed (m/s)", cruiseSpeed);
    cmd.AddValue("satAltM", "Satellite altitude (m)", satAltM);
    cmd.AddValue("satSpeed", "Satellite ground-track speed (m/s)", satSpeed);
    cmd.AddValue("kpmCsv", "Output KPM CSV path", kpmCsv);
    cmd.AddValue("hoCsv", "Output handover CSV path", hoCsv);
    cmd.Parse(argc, argv);

    // --- Aircraft: long-haul leg in +x ---
    Ptr<AeronauticalMobilityModel> aircraft =
        CreateObject<AeronauticalMobilityModel>();
    aircraft->SetFlightPlan(Vector{0, 0, cruiseAltM},
                            Vector{simSeconds * cruiseSpeed, 0, cruiseAltM},
                            cruiseAltM, cruiseSpeed);

    // --- Satellite: passes overhead, starting behind the flight ---
    Ptr<ConstantVelocityMobilityModel> sat =
        CreateObject<ConstantVelocityMobilityModel>();
    sat->SetPosition(Vector{-0.5 * satSpeed * simSeconds, 0, satAltM});
    sat->SetVelocity(Vector{satSpeed, 0, 0});

    // --- E2 node on the satellite gNB ---
    Ptr<OranNtnE2Node> e2 = CreateObject<OranNtnE2Node>();
    e2->SetNodeId(42);
    e2->SetIsNtn(true);
    e2->SetFeederLinkDelay(MilliSeconds(2));
    e2->RegisterRanFunction(2, "E2SM-KPM v03.00");

    // One periodic KPM subscription from a (virtual) RIC.
    E2Subscription sub{};
    sub.subscriptionId = 1001;
    sub.ricRequestorId = 1;
    sub.ranFunctionId = 2;
    sub.reportingPeriod = MilliSeconds(reportMs);
    sub.eventTrigger = false;
    sub.batchOnVisibility = false;
    sub.maxBufferAge = Seconds(10);
    sub.useIslRelay = false;
    e2->HandleSubscriptionRequest(sub);

    // KPM CSV via the indication callback (the OAI-style E2 path).
    std::ofstream kpmOut(kpmCsv);
    kpmOut << "t_s,gnbId,ueId,rsrp_dBm,sinr_dB,cqi,elev_deg,doppler_Hz,"
              "prop_delay_ms,throughput_Mbps,sliceId,buffered,delivery_delay_ms\n";
    uint64_t indicationsDelivered = 0;
    e2->SetIndicationCallback([&kpmOut, &indicationsDelivered](
                                   E2Indication ind) {
        const E2KpmReport& r = ind.kpmReport;
        kpmOut << r.timestamp << "," << r.gnbId << "," << r.ueId << ","
               << r.rsrp_dBm << "," << r.sinr_dB << ","
               << static_cast<unsigned>(r.cqi) << "," << r.elevation_deg << ","
               << r.doppler_Hz << "," << r.propagationDelay_ms << ","
               << r.throughput_Mbps << ","
               << static_cast<unsigned>(r.sliceId) << ","
               << (ind.isBuffered ? 1 : 0) << ","
               << ind.deliveryDelay.GetMilliSeconds() << "\n";
        ++indicationsDelivered;
    });

    std::ofstream hoOut(hoCsv);
    hoOut << "t_s,event,elev_deg,range_m\n";

    Context ctx{aircraft, sat, e2, sub.subscriptionId, minElevDeg,
                false, 0, &hoOut, 40.0, sliceId, &indicationsDelivered};

    Simulator::Schedule(MilliSeconds(reportMs), &EmitKpm, &ctx,
                        MilliSeconds(reportMs));
    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    Simulator::Destroy();

    kpmOut.close();
    hoOut.close();

    std::printf("# sagin-flight-leo-e2 (Roadmap §4.4.9)\n");
    std::printf("#   sim=%.0f s  reportPeriod=%u ms  minElev=%.1f deg\n",
                simSeconds, reportMs, minElevDeg);
    std::printf("#   KPM indications delivered: %llu  → %s\n",
                (unsigned long long)indicationsDelivered, kpmCsv.c_str());
    std::printf("#   handover-style events: %u  → %s\n",
                ctx.handovers, hoCsv.c_str());
    return 0;
}
