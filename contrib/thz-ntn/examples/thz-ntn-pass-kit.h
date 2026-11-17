/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// thz-ntn-pass-kit.h - the measured packet companion shared by the thz-ntn
// examples that are otherwise analysis tools (Wave 7C).
//
// One flow through the toolkit:
//   ntn-constellation  Walker-delta shell, SGP4 propagation of the serving
//                      satellite (Sgp4MobilityModel, ECEF);
//   ntn-cho            TR 38.811 Section 6.1.1.1 terminal class for the ground
//                      end (static class: a fixed THz terminal with a dish),
//                      placed under the pass so the satellite crosses its sky;
//   thz-ntn            ThzNtnPacketLink: Friis + P.676-13 Annex 1 gas +
//                      P.840-9 cloud + P.838/P.618 rain at the P.834 apparent
//                      elevation, live slant-range delay, DVB-S2 ACM and
//                      BLER decode, sky noise;
//   ntn-traffic        NtnOranApplication (TS 23.501 5QI profile, in-band
//                      5QI/S-NSSAI header) -> NtnOranSink (measured one-way
//                      delay, RFC 3550 jitter, sequence loss), and the
//                      NtnOranAiFlowMonitor KPM series (TS 28.552 names);
//   ns-3 FlowMonitor   IP-level per-flow statistics (XML);
//   ntn-observability  NtnReproManifest (run manifest JSON).
// Every KPI the kit prints is counted from packets.
#ifndef THZ_NTN_PASS_KIT_H
#define THZ_NTN_PASS_KIT_H

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/network-module.h"
#include "ns3/ntn-oran-ai-flow-monitor.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/ntn-realistic-mobility.h"
#include "ns3/ntn-repro-manifest.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/thz-ntn-packet-link.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <string>

namespace ns3
{
namespace thzkit
{

struct PassConfig
{
    std::string runTag{"thz-pass"};
    std::string outputDir{"thz-pass-out"};
    double simSeconds{30.0};
    // Constellation (ntn-constellation Walker delta, satellite 0 is served).
    double altitudeKm{550.0};
    double inclinationDeg{53.0};
    uint32_t planes{72};
    uint32_t satsPerPlane{22};
    double epochUnixS{1735689600.0}; // 2025-01-01T00:00:00Z
    // Ground end: under the sub-satellite point at passCenterS, displaced
    // north by groundOffsetKm (0 = the satellite crosses the zenith).
    double passCenterS{-1.0}; // < 0: the middle of the run
    double groundOffsetKm{150.0};
    double groundAltM{0.0}; // > 0: an elevated relay (HAPS), geocentric fixed
    // Link.
    thzntn::ThzNtnPacketLink::Config link;
    // Traffic (downlink, satellite -> ground).
    NtnOranApplication::Profile profile{NtnOranApplication::EMBB_VIDEO};
    double appRateMbps{100.0};
    uint32_t packetBytes{1400};
    uint8_t fiveQi{0}; // 0 = profile preset
    uint8_t sst{1};
    uint32_t sd{0x000001};
    // Cloud state (P.840-9): "sampled" (one draw per run), "pXX" exceedance
    // percent, "fixed:L" kg/m^2, or "off".
    std::string cloud{"sampled"};
    uint32_t rngStream{1000};
    int argc{0};
    char** argv{nullptr};
};

struct PassResult
{
    uint64_t txPackets{0};
    uint64_t rxPackets{0};
    double pdr{0.0};
    double goodputMbps{0.0};
    double meanOwdMs{0.0};
    double maxOwdMs{0.0};
    double jitterMs{0.0};
    double lossRatio{0.0};
    double minElevDeg{0.0};
    double maxElevDeg{0.0};
    double meanLineRateMbps{0.0};
    double outageS{0.0};
    std::string linkReport;
};

/// Parse the --cloud string into the cloud model.
inline void
ApplyCloud(Ptr<ThzNtnCloudAttenuation> cm, const std::string& spec)
{
    if (spec == "off")
    {
        cm->SetMode(ThzNtnCloudAttenuation::OFF);
    }
    else if (spec.rfind("fixed:", 0) == 0)
    {
        cm->SetMode(ThzNtnCloudAttenuation::FIXED);
        cm->SetAttribute("FixedLwc", DoubleValue(std::stod(spec.substr(6))));
    }
    else if (!spec.empty() && spec[0] == 'p')
    {
        cm->SetMode(ThzNtnCloudAttenuation::EXCEEDANCE);
        cm->SetAttribute("ExceedancePercent", DoubleValue(std::stod(spec.substr(1))));
    }
    else
    {
        cm->SetMode(ThzNtnCloudAttenuation::SAMPLED_PER_RUN);
    }
}

/**
 * Run one measured THz pass. \p beforeRun is called after the link and the
 * traffic are installed and before Simulator::Run(), so an example can couple
 * its own analysis into the link (e.g. SetExtraLossDb from a beam-squint or a
 * RIS computation, or a scheduled change). Simulator::Destroy() is called.
 */
inline PassResult
RunPass(const PassConfig& cfg,
        const std::function<void(Ptr<thzntn::ThzNtnPacketLink>, Ptr<MobilityModel>,
                                 Ptr<MobilityModel>)>& beforeRun = {})
{
    std::filesystem::create_directories(cfg.outputDir);
    const std::string prefix = cfg.outputDir + "/" + cfg.runTag;

    // ---- ntn-constellation: the SGP4 satellite ----
    ntncon::WalkerConfig w;
    w.num_planes = cfg.planes;
    w.total_sats = cfg.planes * cfg.satsPerPlane;
    w.altitude_km = cfg.altitudeKm;
    w.inclination_deg = cfg.inclinationDeg;
    w.epoch_unix_s = cfg.epochUnixS;
    const auto elements = ntncon::WalkerConstellation::BuildDelta(w);
    Ptr<ntncon::Sgp4MobilityModel> sat = CreateObject<ntncon::Sgp4MobilityModel>();
    sat->SetElementsSgp4(elements[0]);

    const double tc = cfg.passCenterS >= 0.0 ? cfg.passCenterS : cfg.simSeconds / 2.0;
    Vector pc;
    Vector vc;
    sat->GetEcefStateAt(Seconds(tc), pc, vc);
    double subLat;
    double subLon;
    double subAlt;
    ntngeo::EcefToGeodetic(pc, subLat, subLon, subAlt);
    const double siteLat = subLat + cfg.groundOffsetKm / 111.195;
    const double siteLon = subLon;

    NodeContainer nodes;
    nodes.Create(2); // 0 = satellite, 1 = ground
    nodes.Get(0)->AggregateObject(sat);
    Ptr<MobilityModel> groundMob;
    std::string groundClass;
    if (cfg.groundAltM > 0.0)
    {
        // ECEF position so every model on the chain (Friis distance included)
        // sees the same frame as the SGP4 satellite. A
        // GeocentricConstantPositionMobilityModel would return topocentric
        // coordinates from GetPosition().
        auto hap = CreateObject<ConstantPositionMobilityModel>();
        hap->SetPosition(ntngeo::GeodeticToEcef(siteLat, siteLon, cfg.groundAltM));
        nodes.Get(1)->AggregateObject(hap);
        groundMob = hap;
        groundClass = "HAPS (fixed, ECEF)";
    }
    else
    {
        // ---- ntn-cho: TR 38.811 static terminal class at the site ----
        MobilityProfile fixedSite;
        fixedSite.pStatic = 1.0;
        fixedSite.pPedestrian = fixedSite.pVehicular = fixedSite.pHst = 0.0;
        fixedSite.pMaritime = fixedSite.pAviation = fixedSite.pIot = 0.0;
        NtnTr38811MobilityHelper ueMob(cfg.rngStream);
        NodeContainer g(nodes.Get(1));
        auto ms = ueMob.Install(g, fixedSite, siteLat - 1e-4, siteLat + 1e-4, siteLon - 1e-4,
                                siteLon + 1e-4);
        groundMob = ms.at(0);
        groundClass = ms.at(0)->GetClassName();
    }

    // ---- thz-ntn: the measured THz link ----
    Ptr<thzntn::ThzNtnPacketLink> link = CreateObject<thzntn::ThzNtnPacketLink>();
    link->SetConfig(cfg.link);
    NetDeviceContainer devs = link->Install(nodes.Get(0), nodes.Get(1), sat, groundMob);
    ApplyCloud(link->GetAtmosphere()->GetCloudModel(), cfg.cloud);
    link->AssignStreams(cfg.rngStream);
    link->EnableTimeSeries(prefix + "_link.csv");

    InternetStackHelper ip;
    ip.Install(nodes);
    Ipv4AddressHelper addr;
    addr.SetBase("10.61.0.0", "255.255.255.0");
    Ipv4InterfaceContainer ifs = addr.Assign(devs);

    // ---- ntn-traffic: 5QI flow with in-band header, measured at the sink ----
    const uint16_t port = 9100;
    Ptr<NtnOranSink> sink = CreateObject<NtnOranSink>();
    sink->SetAttribute("Local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), port)));
    nodes.Get(1)->AddApplication(sink);
    sink->SetStartTime(Seconds(0.0));
    Ptr<NtnOranApplication> app = CreateObject<NtnOranApplication>();
    app->SetRemote(InetSocketAddress(ifs.GetAddress(1), port));
    app->SetProfile(cfg.profile);
    app->SetAttribute("DataRate", DataRateValue(DataRate(static_cast<uint64_t>(cfg.appRateMbps * 1e6))));
    app->SetAttribute("PacketSize", UintegerValue(cfg.packetBytes));
    if (cfg.fiveQi)
    {
        app->SetAttribute("FiveQi", UintegerValue(cfg.fiveQi));
    }
    app->SetAttribute("Sst", UintegerValue(cfg.sst));
    app->SetAttribute("Sd", UintegerValue(cfg.sd));
    app->SetAttribute("SrcId", UintegerValue(1));
    app->SetAttribute("DstId", UintegerValue(2));
    nodes.Get(0)->AddApplication(app);
    app->SetStartTime(Seconds(1.0));
    app->SetStopTime(Seconds(cfg.simSeconds - 0.5));

    Ptr<NtnOranAiFlowMonitor> kpm = CreateObject<NtnOranAiFlowMonitor>();
    kpm->AddSource(app);
    kpm->AddSink(sink);
    kpm->Start();
    FlowMonitorHelper fmh;
    Ptr<FlowMonitor> fm = fmh.InstallAll();

    if (beforeRun)
    {
        beforeRun(link, sat, groundMob);
    }

    std::printf("[packet-companion] %s: SGP4 sat (Walker %u/%u, %.0f km, %.0f deg) -> %s at "
                "(%.3f, %.3f), pass center t=%.1f s, %.1f GHz, B=%.2f GHz, %s %.1f Mb/s\n",
                cfg.runTag.c_str(), w.total_sats, w.num_planes, cfg.altitudeKm,
                cfg.inclinationDeg, groundClass.c_str(), siteLat, siteLon, tc,
                cfg.link.freqHz / 1e9, cfg.link.bandwidthHz / 1e9,
                cfg.profile == NtnOranApplication::EMBB_VIDEO ? "eMBB video (5QI 2)" : "5QI flow",
                cfg.appRateMbps);

    Simulator::Stop(Seconds(cfg.simSeconds));
    Simulator::Run();

    PassResult r;
    fm->CheckForLostPackets();
    auto cls = DynamicCast<Ipv4FlowClassifier>(fmh.GetClassifier());
    for (const auto& [id, st] : fm->GetFlowStats())
    {
        r.txPackets += st.txPackets;
        r.rxPackets += st.rxPackets;
    }
    r.pdr = r.txPackets ? static_cast<double>(r.rxPackets) / r.txPackets : 0.0;
    for (const auto& [key, fs] : sink->GetFlowStats())
    {
        r.goodputMbps = fs.ThroughputMbps();
        r.meanOwdMs = fs.MeanDelayMs();
        r.maxOwdMs = fs.maxDelayMs;
        r.jitterMs = fs.jitterMs;
        r.lossRatio = fs.LossRatio();
    }
    r.minElevDeg = link->GetMinElevationDeg();
    r.maxElevDeg = link->GetMaxElevationDeg();
    r.meanLineRateMbps = link->GetMeanRateMbps();
    r.outageS = link->GetOutageSeconds();
    r.linkReport = link->GetReport();

    fm->SerializeToXmlFile(prefix + "_flowmon.xml", false, false);
    kpm->WriteCsv(prefix + "_kpm_series.csv");
    kpm->WriteInfluxLp(prefix + "_kpm_series.lp");

    ntnobs::NtnReproManifest man;
    man.SetScenarioName(cfg.runTag).SetScenarioDuration(cfg.simSeconds);
    man.SetRng(RngSeedManager::GetSeed(), RngSeedManager::GetRun());
    if (cfg.argc > 0)
    {
        man.SetCliArgv(cfg.argc, cfg.argv);
    }
    man.AddExtra("thz_link", r.linkReport);
    man.AddExtra("cloud", link->GetAtmosphere()->GetCloudModel()->GetRunReport());
    man.WriteJson(prefix + "_manifest.json");

    std::printf("[packet-companion] %s\n", r.linkReport.c_str());
    std::printf("[packet-companion] measured: tx %llu rx %llu PDR %.5f goodput %.2f Mb/s "
                "OWD mean %.3f ms max %.3f ms jitter %.4f ms seq-loss %.5f\n",
                static_cast<unsigned long long>(r.txPackets),
                static_cast<unsigned long long>(r.rxPackets), r.pdr, r.goodputMbps,
                r.meanOwdMs, r.maxOwdMs, r.jitterMs, r.lossRatio);
    std::printf("[packet-companion] outputs: %s_{link.csv,kpm_series.csv,kpm_series.lp,"
                "flowmon.xml,manifest.json}\n",
                prefix.c_str());
    (void)cls;
    Simulator::Destroy();
    return r;
}

} // namespace thzkit
} // namespace ns3

#endif // THZ_NTN_PASS_KIT_H
