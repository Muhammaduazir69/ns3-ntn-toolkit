/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ntn-e2e-full-stack: one measured data plane, one closed O-RAN loop.
 *
 * Composition (Wave 2 M2):
 *   - ntn-constellation: two SGP4 Walker satellites (serving + the in-plane
 *     neighbour approaching the UEs), each a 5G-LENA gNB (ntn-traffic
 *     NtnRealStackHelper, nr backend, TR 38.821 Set-1 S-band terminal and
 *     payload, cell-level SS-RSRP on an Earth-fixed cell beam, feeder from a
 *     gateway --gwDistanceKm north of the UEs).
 *   - ntn-slice: the gNB MAC runs NtnSliceQuotaScheduler; MixedBouquet traffic
 *     marks eMBB 5QI 2, URLLC 5QI 82, mMTC 5QI 9 (TS 23.501 Table 5.7.4-1).
 *   - oran-ntn: Near-RT RIC + xApps. OranNtnRealStackBinding registers the
 *     actuators this radio has (handover -> TriggerHandover over X2, slice
 *     quota -> NtnSliceQuotaScheduler, Tx power -> NrGnbPhy, MCS -> MAC) and
 *     feeds the E2 nodes from measured counters only (TS 28.552 names). Only
 *     xApps whose every action has an actuator are admitted.
 *   - E2: --e2Transport=event (delay events, default) or packet (E2AP as UDP
 *     datagrams over a per-satellite feeder link with live slant delay; E2
 *     Setup and RIC Subscription precede the first indication).
 *   - ntn-observability: control plane and KPM points in line protocol on the
 *     simulation clock (out/ric.lp).
 *
 * The RIC owns mobility: the vendored A3 algorithm is not installed
 * (SetAutonomousHandover(false)), the stack sends periodic 480 ms RRC
 * measurement reports, and a handover happens only if an xApp decides it.
 *
 * --xapps=0 runs the identical radio with no xApp: the difference between the
 * two runs is what the RIC actuated.
 *
 * Usage:
 *   ntn-e2e-full-stack --duration=15 --numUes=9 [--xapps=0] [--e2Transport=packet]
 *                      [--feederOutageAt=8 --feederOutageS=1]
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include "ns3/ntn-influx-sink.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-slice-types.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/oran-ntn-a1-interface.h"
#include "ns3/oran-ntn-e2-interface.h"
#include "ns3/oran-ntn-e2-transport.h"
#include "ns3/oran-ntn-helper.h"
#include "ns3/oran-ntn-near-rt-ric.h"
#include "ns3/oran-ntn-real-stack-binding.h"
#include "ns3/oran-ntn-ric-placement.h"
#include "ns3/oran-ntn-types.h"
#include "ns3/oran-ntn-xapp-base.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/slice-isolation-monitor.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>

using namespace ns3;
using namespace ns3::ntnslice;

NS_LOG_COMPONENT_DEFINE("NtnE2eFullStack");

namespace
{
std::vector<double>
ParseList(const std::string& s)
{
    std::vector<double> v;
    std::stringstream ss(s);
    std::string x;
    while (std::getline(ss, x, ','))
    {
        v.push_back(std::stod(x));
    }
    return v;
}
} // namespace

int
main(int argc, char* argv[])
{
    double duration = 15.0;
    uint32_t numUes = 9;
    double altitudeKm = 600.0;
    bool xapps = true;
    std::string e2Transport = "event";
    double feederOutageAt = -1.0;
    double feederOutageS = 1.0;
    double gwDistanceKm = 500.0;
    double e2RateMbps = 10.0;
    double ueAlongTrack = 0.5; // UE site between the serving (0) and neighbour (1) sub-points
    double a3OffsetDb = 3.0;
    std::string initialQuotas = "0.9,0.05,0.05";
    std::string outputDir = "ntn-e2e-full-stack-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("numUes", "UEs (MixedBouquet: u%3 = 0 mMTC, 1 eMBB, 2 URLLC)", numUes);
    cmd.AddValue("altitude", "Shell altitude (km)", altitudeKm);
    cmd.AddValue("xapps", "Run the admitted xApps (0 = same radio, no RIC control)", xapps);
    cmd.AddValue("e2Transport", "event (delay events) or packet (E2AP datagrams over the feeder)",
                 e2Transport);
    cmd.AddValue("feederOutageAt", "packet transport: feeder outage start on E2 node 1 (s, <0 off)",
                 feederOutageAt);
    cmd.AddValue("feederOutageS", "packet transport: feeder outage duration (s)", feederOutageS);
    cmd.AddValue("e2RateMbps", "packet transport: E2 control bearer rate (Mb/s)", e2RateMbps);
    cmd.AddValue("gwDistanceKm", "Gateway ground distance north of the UE site (km)", gwDistanceKm);
    cmd.AddValue("ueAlongTrack",
                 "UE site as a fraction of the way from the serving to the neighbour sub-point",
                 ueAlongTrack);
    cmd.AddValue("a3OffsetDb", "ho-predict measured-RSRP A3 offset (dB)", a3OffsetDb);
    cmd.AddValue("initialQuotas", "initial MAC slice quotas eMBB,URLLC,mMTC", initialQuotas);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);
    NS_ABORT_MSG_IF(e2Transport != "event" && e2Transport != "packet",
                    "--e2Transport must be event or packet");
    std::filesystem::create_directories(outputDir);

    // ---- Constellation: serving satellite + the approaching in-plane neighbour
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    auto serv = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    serv->SetElementsSgp4(elements[0]);
    auto nbrA = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    nbrA->SetElementsSgp4(elements[1]);
    auto nbrB = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    nbrB->SetElementsSgp4(elements[wcfg.total_sats - 1]);
    double sLat, sLon, sAlt;
    serv->GetGeodetic(sLat, sLon, sAlt);
    const Vector site0 = NtnRealStackHelper::GeodeticToEcef(sLat, sLon, 0.0);
    auto approaching = [&site0](Ptr<ns3::ntncon::Sgp4MobilityModel> s) {
        const Vector p = s->GetPosition();
        const Vector v = s->GetVelocity();
        return (v.x * (site0.x - p.x) + v.y * (site0.y - p.y) + v.z * (site0.z - p.z)) > 0.0;
    };
    Ptr<ns3::ntncon::Sgp4MobilityModel> nbr = approaching(nbrA) ? nbrA : nbrB;
    double nLat, nLon, nAlt;
    nbr->GetGeodetic(nLat, nLon, nAlt);
    const double uLat = sLat + ueAlongTrack * (nLat - sLat);
    const double uLon = sLon + ueAlongTrack * (nLon - sLon);

    NodeContainer satNodes;
    satNodes.Create(2);
    satNodes.Get(0)->AggregateObject(serv);
    satNodes.Get(1)->AggregateObject(nbr);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    NtnTr38811MobilityHelper ueMobility(1);
    auto ueModels = ueMobility.Install(ueNodes, NtnMobilityScenarios::MixedContinental(),
                                       uLat - 0.03, uLat + 0.03, uLon - 0.03, uLon + 0.03);

    // ---- The measured NR stack ----
    NtnRealStackHelper rs;
    rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
    rs.SetNumerology(1);
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-e2e-full-stack");
    rs.ApplyTr38821Set1Defaults(); // S-band 2.185 GHz, 20 MHz, handheld 1T2R, 34 dBW/MHz
    rs.SetSchedulerTypeName("ns3::ntnslice::NtnSliceQuotaScheduler");
    rs.SetHandover(true, 3.0, MilliSeconds(256));
    rs.SetAutonomousHandover(false); // the RIC owns mobility; periodic 480 ms reports
    Ptr<ConstantPositionMobilityModel> cellCenter = CreateObject<ConstantPositionMobilityModel>();
    cellCenter->SetPosition(NtnRealStackHelper::GeodeticToEcef(uLat, uLon, 0.0));
    rs.SetSsbCellBeam(true, cellCenter);
    rs.SetKpmFromRanCounters(true);
    rs.Build(satNodes, ueNodes);
    Ptr<MobilityModel> gw =
        rs.SetFeederGatewayGeodetic(uLat + gwDistanceKm / 111.195, uLon, 0.0);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet, Seconds(1.0),
                      Seconds(duration - 0.5));

    // ---- O-RAN ----
    auto helper = CreateObject<OranNtnHelper>();
    helper->SetOutputDirectory(outputDir);
    auto nonRtRic = helper->CreateNonRtRic();
    auto ric = helper->CreateNearRtRic();
    helper->ConnectA1Interface(ric, nonRtRic);
    auto e2Nodes = helper->CreateSatelliteE2Nodes(satNodes, ric);
    helper->SetShellAltitudeKm(altitudeKm);

    auto binding = CreateObject<OranNtnRealStackBinding>();
    binding->SetInitialQuotas(ParseList(initialQuotas));
    binding->Bind(helper, &rs);
    for (uint32_t u = 0; u < numUes; ++u)
    {
        binding->SetUeSlice(u, (u % 3 == 0) ? 2 : (u % 3 == 1) ? 0 : 1);
    }
    binding->SetGeometry({serv, nbr},
                         std::vector<Ptr<MobilityModel>>(ueModels.begin(), ueModels.end()),
                         rs.GetCarrierFrequencyHz());
    binding->StartKpm(MilliSeconds(100), Seconds(1.0));

    // E2 transport
    Ptr<OranNtnE2Transport> transport;
    std::vector<Ptr<OranNtnRicPlacement>> sites;
    if (e2Transport == "packet")
    {
        transport = CreateObject<OranNtnE2Transport>();
        transport->SetTraceFile(outputDir + "/e2_trace.csv");
        transport->SetBearerRate(DataRate(static_cast<uint64_t>(e2RateMbps * 1e6)));
        transport->Attach(e2Nodes[0], serv, gw);
        transport->Attach(e2Nodes[1], nbr, gw);
        transport->Start(Seconds(0.1));
        if (feederOutageAt >= 0.0)
        {
            transport->ScheduleOutage(1, Seconds(feederOutageAt), Seconds(feederOutageS));
        }
    }
    else
    {
        // Event delivery bound to the same live satellite-gateway slant.
        for (uint32_t s = 0; s < 2; ++s)
        {
            auto site = CreateObject<OranNtnRicPlacement>();
            site->SetSite(OranNtnRicPlacement::Site::GroundGateway);
            site->BindToGeometry(s == 0 ? Ptr<MobilityModel>(serv) : Ptr<MobilityModel>(nbr), gw,
                                 e2Nodes[s], Seconds(1.0));
            sites.push_back(site);
        }
    }

    std::map<std::string, Ptr<OranNtnXappBase>> admitted;
    if (xapps)
    {
        admitted = helper->CreateAdmittedXapps(ric);
        if (auto ho = ric->GetXappByName("ho-predict"))
        {
            ho->SetAttribute("A3OffsetDb", DoubleValue(a3OffsetDb));
        }
        helper->StartAllXapps(ric);
    }

    // ---- Observability on the simulation clock ----
    auto sink = CreateObject<ns3::ntnobs::NtnInfluxSink>();
    sink->SetTransport(ns3::ntnobs::NtnInfluxSink::Transport::File);
    sink->SetFilePath(outputDir + "/ric.lp");
    sink->SetBaseEpoch(Time(0));
    sink->SetFlushPeriod(Seconds(1.0));
    sink->Start();
    helper->SetActionObserver([sink](const E2RcAction& a, bool actuated) {
        ns3::ntnobs::Point p;
        p.measurement = "oran_rc";
        p.tags["xapp"] = a.xappName;
        p.tags["action_type"] = std::to_string(static_cast<uint32_t>(a.actionType));
        p.tags["gnb"] = std::to_string(a.targetGnbId);
        p.fieldsInt["ue"] = a.targetUeId;
        p.fieldsInt["actuated"] = actuated ? 1 : 0;
        p.fieldsFloat["parameter1"] = a.parameter1;
        p.timestamp = Simulator::Now();
        sink->Push(p);
    });
    uint64_t kpmRows = 0;
    binding->SetReportObserver([sink, &kpmRows](const E2KpmReport& r) {
        ++kpmRows;
        ns3::ntnobs::Point p;
        p.measurement = "oran_kpm";
        p.tags["gnb"] = std::to_string(r.gnbId);
        p.tags["ue"] = std::to_string(r.ueId);
        p.tags["slice"] = std::to_string(r.sliceId);
        p.fieldsFloat["L1M.RS-SINR"] = r.sinr_dB;
        p.fieldsFloat["L1M.SS-RSRP"] = r.rsrp_dBm;
        p.fieldsFloat["DRB.UEThpDl"] = r.throughput_Mbps * 1e3;
        p.fieldsFloat["DRB.RlcSduDelayDl"] = r.rlcSduDelayDl_ms * 10.0;
        p.fieldsFloat["RRU.PrbUsedDl"] = r.prbUsedDl;
        p.timestamp = Simulator::Now();
        sink->Push(p);
    });
    rs.RegisterPeriodicCallback(Seconds(1.0), [sink, binding](Time now) {
        const auto q = binding->GetQuotas(0);
        for (size_t s = 0; s < q.size(); ++s)
        {
            ns3::ntnobs::Point p;
            p.measurement = "slice_rrm";
            p.tags["gnb"] = "1";
            p.tags["slice"] = std::to_string(s);
            p.fieldsFloat["quota"] = q[s];
            p.timestamp = now;
            sink->Push(p);
        }
    });

    std::cout << "\n=== ntn-e2e-full-stack: measured NR stack + O-RAN closed loop ===\n"
              << "  radio: nr (TR 38.821 Set-1 S band), 2 SGP4 gNBs, " << numUes
              << " UEs, " << duration << " s, E2 " << e2Transport << ", xApps "
              << (xapps ? "admitted" : "off") << "\n";
    std::cout << "  actuators registered:";
    for (const auto& a : binding->GetRegisteredActuators())
    {
        std::cout << " " << a;
    }
    std::cout << "\n";
    for (const auto& a : helper->GetAdmissionLog())
    {
        std::cout << "  xApp " << std::setw(18) << std::left << a.xappName
                  << (a.admitted ? " ADMITTED" : " refused ") << "  (" << a.reason << ")\n";
    }

    Simulator::Stop(Seconds(duration));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();
    sink->Flush();
    sink->Stop();

    // ---- Slice SLA on the measured OWD distribution (post-hoc replay) ----
    SliceProfile profiles[3] = {DefaultEmbb(1), DefaultUrllc(2), DefaultMmtc(3)};
    SliceIsolationMonitor sliceMon;
    sliceMon.SetWindow(10000000);
    for (auto& p : profiles)
    {
        sliceMon.RegisterSlice(p);
    }
    const uint8_t sliceQi[3] = {2, 82, 9};
    for (uint32_t slice = 0; slice < 3; ++slice)
    {
        for (const auto& [bin, count] : rs.GetSliceDelayHistogram(sliceQi[slice]))
        {
            for (uint64_t p = 0; p < count; ++p)
            {
                sliceMon.RecordPacket(profiles[slice].snssai, static_cast<double>(bin), true);
            }
        }
        const uint64_t lost = rs.GetSliceMeasuredStats(sliceQi[slice]).lostPackets;
        for (uint64_t p = 0; p < lost; ++p)
        {
            sliceMon.RecordPacket(profiles[slice].snssai, 0.0, false);
        }
    }
    auto breaches = sliceMon.EvaluateAll();
    helper->WriteAllMetrics(ric);

    // ---- Summary ----
    std::cout << "\n--- measured data plane ---\n"
              << "  mean DL SINR " << rs.GetMeanDlSinrDb() << " dB, throughput "
              << rs.GetRxThroughputMbps() << " Mb/s, handovers " << rs.GetHandoverCount()
              << " (requested " << rs.GetHandoverRequestedCount() << ", refused "
              << rs.GetHandoverRefusals() << ")\n";
    for (uint32_t s = 0; s < 3; ++s)
    {
        const auto st = rs.GetSliceMeasuredStats(sliceQi[s]);
        std::cout << "  slice 5QI " << std::right << std::setw(2) << +sliceQi[s] << ": thr " << st.thrMbps
                  << " Mb/s, OWD p50/p95/p99 " << st.p50OwdMs << "/" << st.p95OwdMs << "/"
                  << st.p99OwdMs << " ms, loss " << st.lossRatio
                  << (breaches.size() > s && breaches[s].latencyBreach ? " (latency SLA breach)"
                                                                       : "")
                  << "\n";
    }
    const auto q = binding->GetQuotas(0);
    std::cout << "  final MAC quotas gNB1:";
    for (double x : q)
    {
        std::cout << " " << x;
    }
    std::cout << "\n--- O-RAN ---\n"
              << "  measured KPM reports: serving " << binding->GetServingReports()
              << ", neighbour (RRC RSRP) " << binding->GetNeighbourReports() << "\n"
              << "  RC actions routed " << helper->GetTotalRouted() << ", actuated "
              << helper->GetTotalActuated() << "\n";
    for (const auto& [t, c] : helper->GetActuationCounters())
    {
        std::cout << "    action type " << +t << ": routed " << c.routed << ", actuated "
                  << c.actuated << "\n";
    }
    std::cout << "  actuator outcomes: handovers requested " << binding->GetHandoversRequested()
              << ", quota updates " << binding->GetQuotaUpdates() << ", Tx power updates "
              << binding->GetTxPowerUpdates() << "\n";
    if (transport)
    {
        const auto& c = transport->GetCounters();
        std::cout << "--- E2 over the feeder (packets) ---\n"
                  << "  E2 Setup done " << c.setupDone << " (last at "
                  << transport->GetLastSetupDoneTime().GetSeconds() << " s), subscriptions "
                  << c.subscriptionsDone << ", first indication at "
                  << transport->GetFirstIndicationTime().GetSeconds() << " s\n"
                  << "  indications sent " << c.indicationsSent << ", received "
                  << c.indicationsReceived << ", suppressed before subscription "
                  << c.indicationsBeforeSubscription << ", decode failures "
                  << c.decodeFailures << "\n"
                  << "  indication OWD mean " << transport->GetIndicationOwdMeanMs()
                  << " ms, sd " << transport->GetIndicationOwdSdMs()
                  << " ms; mean feeder link delay at send " << transport->GetMeanLinkDelayMs()
                  << " ms; max indication gap " << transport->GetMaxIndicationGapS() << " s\n"
                  << "  controls sent " << c.controlsSent << ", received " << c.controlsReceived
                  << ", acknowledged " << c.controlAcks << ", before setup "
                  << c.controlsBeforeSetup << "; bytes up " << c.bytesUp << ", down "
                  << c.bytesDown << "\n";
    }
    std::cout << "  observability: " << outputDir << "/ric.lp (" << sink->GetEmittedCount()
              << " points, simulation clock), KPM rows observed " << kpmRows << "\n";

    Simulator::Destroy();
    return 0;
}
