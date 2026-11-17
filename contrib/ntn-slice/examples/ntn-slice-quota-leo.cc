/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-slice-quota-leo: three slices (eMBB, URLLC, mMTC) on ONE 5G-LENA NTN cell over a real SGP4 LEO pass,
 * with the slice partition ENFORCED by the MAC scheduler. Policies (--policy):
 *   pf        one BWP, proportional fair, no slicing (the shared baseline)
 *   bwp       one BWP per slice (static partition of the band, NtnRealStackHelper::SetSlices)
 *   static    one BWP, NtnSliceQuotaScheduler with fixed per-slot RBG quotas (--quotas)
 *   reactive  one BWP, quotas recomputed every --ctrlMs from the scheduler's measured per-slice backlog and
 *             bytes per RBG (closed loop): each slice gets the RBG share its backlog needs within the control
 *             period, URLLC first, then mMTC, eMBB takes the rest, with a floor per slice
 * Output: one JSON line with per-slice delivered packets, loss, one-way delay p50/p95/p99, throughput and the
 * per-slice RBG share actually granted. The delay percentiles come from the sinks' 1 ms histogram and are
 * reported at the bin midpoint (x.5 ms), so they resolve to 1 ms, not finer.
 */
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-slice-helper.h"
#include "ns3/ntn-slice-quota-scheduler.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <sstream>

using namespace ns3;
using namespace ns3::ntnslice;

NS_LOG_COMPONENT_DEFINE("NtnSliceQuotaLeo");

namespace
{
// MixedBouquet assigns UE u the profile u % 3: 0 mMTC (5QI 9), 1 eMBB (5QI 2), 2 URLLC (5QI 82).
// Slice indices used by the scheduler: 0 eMBB, 1 URLLC, 2 mMTC.
constexpr uint32_t kSliceOfProfile[3] = {2, 0, 1};
constexpr uint8_t kFiveQi[3] = {2, 82, 9}; // by slice index

struct Ctl
{
    Ptr<NtnSliceQuotaScheduler> sched;
    double ctrlS{0.1};
    std::vector<double> floor{0.1, 0.1, 0.05};
    std::vector<uint64_t> rbg{0, 0, 0};
    uint64_t slots{0};
    std::vector<double> quotaSum{0, 0, 0};
    uint64_t steps{0};
    double rbgPerSlot{0};
    double headroom{1.25};
    std::vector<double> bprW{0, 0, 0}; // bytes per RBG weighted by RBGs, per slice
} g;

void
Reactive()
{
    const std::vector<double> bpr = g.sched->TakeBytesPerRbg();
    uint64_t slots = 0;
    const std::vector<uint64_t> rbg = g.sched->TakeRbgCounts(&slots);
    const std::vector<double> backlog = g.sched->TakeBacklog();
    for (size_t s = 0; s < rbg.size() && s < 3; ++s)
    {
        g.rbg[s] += rbg[s];
        g.bprW[s] += (s < bpr.size() ? bpr[s] : 0.0) * rbg[s];
    }
    g.slots += slots;
    // RBGs a slice needs to send its mean backlog within one control period, as a share of the RBGs of that
    // period; the RBGs per slot are the most ever granted in a period (the cell is saturated by eMBB), and a
    // slice that got no RBGs uses the cell's mean bytes per RBG
    const double tot = std::accumulate(rbg.begin(), rbg.end(), 0.0);
    if (slots > 0)
    {
        g.rbgPerSlot = std::max(g.rbgPerSlot, tot / slots);
    }
    double meanBpr = 0;
    for (size_t s = 0; s < bpr.size() && s < rbg.size(); ++s)
    {
        meanBpr += tot > 0 ? bpr[s] * rbg[s] / tot : 0;
    }
    const double periodRbg = std::max(1.0, g.rbgPerSlot * std::max<uint64_t>(slots, 1));
    std::vector<double> need(3, 0.0);
    for (size_t s = 0; s < 3; ++s)
    {
        const double b = (s < bpr.size() && bpr[s] > 0) ? bpr[s] : std::max(meanBpr, 1.0);
        // RBGs the slice used last period (its served rate) plus those needed to drain its backlog, with headroom:
        // sizing on the backlog alone starves a slice that is being served well, whose backlog is near zero
        const double r = (s < backlog.size() ? backlog[s] / b : 0.0) + (s < rbg.size() ? double(rbg[s]) : 0.0);
        need[s] = std::max(g.floor[s], std::min(1.0, g.headroom * r / periodRbg));
    }
    // every slice first gets its floor; the rest goes to the need above the floor, URLLC first, then mMTC, and
    // eMBB takes what is left
    std::vector<double> q(g.floor.begin(), g.floor.end());
    double left = 1.0 - std::accumulate(q.begin(), q.end(), 0.0);
    for (size_t s : {1u, 2u, 0u})
    {
        const double extra = std::min(std::max(need[s] - q[s], 0.0), left);
        q[s] += extra;
        left -= extra;
    }
    q[0] += left;
    g.sched->SetQuotas(q);
    for (size_t s = 0; s < 3; ++s)
    {
        g.quotaSum[s] += q[s];
    }
    ++g.steps;
    Simulator::Schedule(Seconds(g.ctrlS), &Reactive);
}

void
Count()
{
    const std::vector<double> bpr = g.sched->TakeBytesPerRbg();
    uint64_t slots = 0;
    const std::vector<uint64_t> rbg = g.sched->TakeRbgCounts(&slots);
    g.sched->TakeBacklog();
    for (size_t s = 0; s < rbg.size() && s < 3; ++s)
    {
        g.rbg[s] += rbg[s];
        g.bprW[s] += (s < bpr.size() ? bpr[s] : 0.0) * rbg[s];
    }
    g.slots += slots;
    // The static policy's quota is fixed but real: average it like the reactive
    // one so mean_quota is reported instead of -1.
    const std::vector<double> q = g.sched->GetQuotas();
    for (size_t s = 0; s < 3 && s < q.size(); ++s)
    {
        g.quotaSum[s] += q[s];
    }
    ++g.steps;
    Simulator::Schedule(Seconds(g.ctrlS), &Count);
}

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
    double duration = 30.0;
    uint32_t numUes = 12;
    std::string policy = "reactive";
    std::string quotas = "0.5,0.3,0.2";
    double ctrlMs = 100.0;
    // Wave 7C: -1 = TR 38.821 Set-1 handheld + S-band payload (34 dBW/MHz). The
    // old default, 70 dBm CONDUCTED at the array input, radiated about 88 dBm:
    // handheld SINR above 30 dB, and over 30 s of the pass not one packet lost.
    double satEirpDbm = -1.0;
    double gwDistanceKm = 500.0;
    double startOffset = 0.0;
    std::string outputDir = "ntn-slice-quota-leo-output";
    double embbMbps = 5.0;      // per eMBB UE offered load
    double surgeMbps = 0.0;     // per eMBB UE load during the surge window (0: no surge)
    double surgeStart = 10.0, surgeStop = 20.0;
    uint32_t urllcBytes = 0;    // URLLC packet size (0: profile default)
    double urllcPeriodMs = 0.0; // URLLC period (0: profile default, 10 ms)
    double bwMhz = 20.0;
    double ueOffsetDeg = 0.0; // UE cluster placed this far north of the sub-satellite point (lower elevation)
    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "simulated seconds", duration);
    cmd.AddValue("numUes", "UEs (a third per slice)", numUes);
    cmd.AddValue("policy", "pf | bwp | static | reactive", policy);
    cmd.AddValue("quotas", "static quotas eMBB,URLLC,mMTC", quotas);
    cmd.AddValue("ctrlMs", "control period of the reactive policy (ms)", ctrlMs);
    cmd.AddValue("satEirpDbm",
                 "satellite conducted power (dBm); -1 = TR 38.821 Set-1 (34 dBW/MHz, handheld)",
                 satEirpDbm);
    cmd.AddValue("gwDistanceKm",
                 "gateway ground distance north of the UE site (km); the feeder delay follows the live "
                 "satellite-gateway range. <= 0 keeps the constant 5 ms backhaul",
                 gwDistanceKm);
    cmd.AddValue("startOffset", "seconds into the pass at which the run starts", startOffset);
    cmd.AddValue("outputDir", "output directory", outputDir);
    cmd.AddValue("embbMbps", "offered load per eMBB UE (Mb/s)", embbMbps);
    cmd.AddValue("surgeMbps", "offered load per eMBB UE during the surge (Mb/s, 0 = none)", surgeMbps);
    cmd.AddValue("surgeStart", "surge start (s)", surgeStart);
    cmd.AddValue("surgeStop", "surge end (s)", surgeStop);
    cmd.AddValue("urllcBytes", "URLLC packet size (bytes, 0 = default)", urllcBytes);
    cmd.AddValue("urllcPeriodMs", "URLLC period (ms, 0 = default)", urllcPeriodMs);
    cmd.AddValue("headroom", "reactive policy: quota headroom over the measured need", g.headroom);
    cmd.AddValue("ueOffsetDeg", "UE cluster offset north of the sub-satellite point (deg)", ueOffsetDeg);
    cmd.AddValue("bwMhz", "carrier bandwidth (MHz)", bwMhz);
    cmd.Parse(argc, argv);
    g.ctrlS = ctrlMs / 1000.0;

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = 550.0;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0 + startOffset;
    const auto el = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    auto satMob = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satMob->SetElements(el[0]);
    satNodes.Get(0)->AggregateObject(satMob);
    double subLat, subLon, subAlt;
    satMob->GetGeodetic(subLat, subLon, subAlt);
    NtnTr38811MobilityHelper ueMob(1);
    ueMob.Install(ueNodes, NtnMobilityScenarios::MixedContinental(), subLat + ueOffsetDeg - 0.03,
                  subLat + ueOffsetDeg + 0.03, subLon - 0.03, subLon + 0.03);

    // ideal RRC: many UEs attaching at once otherwise collide in random access (an attach artefact, not the
    // slicing question)
    Config::SetDefault("ns3::NrHelper::UseIdealRrc", BooleanValue(true));
    NtnRealStackHelper rs;
    rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
    rs.SetNumerology(1);
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-slice-quota-leo");
    rs.SetBackhaulDelay(MilliSeconds(5));
    rs.SetBandwidthHz(bwMhz * 1e6);
    NtnSliceHelper::ConfigureCellLinkBudget(rs, satEirpDbm, false);
    if (policy == "bwp")
    {
        rs.SetSlices({{"eMBB", 2}, {"URLLC", 82}, {"mMTC", 9}});
    }
    else if (policy == "pf")
    {
        rs.SetScheduler(NtnRealStackHelper::Scheduler::OfdmaPF);
    }
    else
    {
        rs.SetSchedulerTypeName("ns3::ntnslice::NtnSliceQuotaScheduler");
    }
    rs.Build(satNodes, ueNodes);
    NtnSliceHelper::ConfigureFeeder(rs, subLat + ueOffsetDeg, subLon, gwDistanceKm);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet, Seconds(1.0), Seconds(duration - 0.5));
    // the downlink clients sit on the remote host in UE order, so client k serves UE k (profile k % 3)
    std::vector<Ptr<NtnOranApplication>> embbApps;
    {
        Ptr<Node> rh = rs.GetRemoteHost();
        uint32_t k = 0;
        for (uint32_t i = 0; i < rh->GetNApplications(); ++i)
        {
            auto app = DynamicCast<NtnOranApplication>(rh->GetApplication(i));
            if (!app)
            {
                continue;
            }
            if (k % 3 == 1)
            {
                app->SetAttribute("DataRate", DataRateValue(DataRate(uint64_t(embbMbps * 1e6))));
                embbApps.push_back(app);
            }
            else if (k % 3 == 2)
            {
                if (urllcBytes > 0)
                {
                    app->SetAttribute("PacketSize", UintegerValue(urllcBytes));
                }
                if (urllcPeriodMs > 0)
                {
                    app->SetAttribute("Period", TimeValue(Seconds(urllcPeriodMs / 1000.0)));
                }
            }
            ++k;
        }
    }
    if (surgeMbps > 0)
    {
        auto setRate = [embbApps](double mbps) {
            for (auto& a : embbApps)
            {
                a->SetAttribute("DataRate", DataRateValue(DataRate(uint64_t(mbps * 1e6))));
            }
        };
        Simulator::Schedule(Seconds(surgeStart), [setRate, surgeMbps]() { setRate(surgeMbps); });
        Simulator::Schedule(Seconds(surgeStop), [setRate, embbMbps]() { setRate(embbMbps); });
    }

    if (policy == "static" || policy == "reactive")
    {
        auto gnb = DynamicCast<NrGnbNetDevice>(rs.GetEnbDevices().Get(0));
        g.sched = DynamicCast<NtnSliceQuotaScheduler>(gnb->GetScheduler(0));
        NS_ABORT_MSG_IF(!g.sched, "the gNB does not run NtnSliceQuotaScheduler");
        g.sched->SetQuotas(policy == "static" ? ParseList(quotas) : std::vector<double>{0.6, 0.25, 0.15});
        // RNTIs exist once the UEs have attached
        Simulator::Schedule(Seconds(0.5), [numUes, &rs, policy]() {
            for (uint32_t u = 0; u < numUes; ++u)
            {
                g.sched->SetSliceOfRnti(rs.GetUeRnti(u), kSliceOfProfile[u % 3]);
            }
            Simulator::Schedule(Seconds(0.5), policy == "reactive" ? &Reactive : &Count);
        });
    }

    // elevation of the satellite seen from UE 0 (positions in ECEF), at the start of the traffic and at the end
    auto elevDeg = [satNodes, ueNodes]() {
        const Vector s = satNodes.Get(0)->GetObject<MobilityModel>()->GetPosition();
        const Vector u = ueNodes.Get(0)->GetObject<MobilityModel>()->GetPosition();
        const Vector d(s.x - u.x, s.y - u.y, s.z - u.z);
        const double nu = std::sqrt(u.x * u.x + u.y * u.y + u.z * u.z);
        const double nd = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        return std::asin((d.x * u.x + d.y * u.y + d.z * u.z) / (nu * nd)) * 180.0 / M_PI;
    };
    static double elevStart = 0, elevEnd = 0;
    Simulator::Schedule(Seconds(1.0), [&elevDeg]() { elevStart = elevDeg(); });
    Simulator::Schedule(Seconds(duration) - MilliSeconds(1), [&elevDeg]() { elevEnd = elevDeg(); });
    Simulator::Stop(Seconds(duration));
    Simulator::Run();
    rs.Collect();

    std::ostringstream js;
    js << "{\"policy\":\"" << policy << "\",\"ues\":" << numUes << ",\"duration\":" << duration
       << ",\"startOffset\":" << startOffset << ",\"slices\":[";
    const double totRbg = std::accumulate(g.rbg.begin(), g.rbg.end(), 0.0);
    for (uint32_t s = 0; s < 3; ++s)
    {
        const auto st = rs.GetSliceMeasuredStats(kFiveQi[s]);
        js << (s ? "," : "") << "{\"fiveQi\":" << +kFiveQi[s] << ",\"rx\":" << st.rxPackets
           << ",\"lost\":" << st.lostPackets << ",\"loss\":" << st.lossRatio << ",\"owd_p50\":" << st.p50OwdMs
           << ",\"owd_p95\":" << st.p95OwdMs << ",\"owd_p99\":" << st.p99OwdMs << ",\"thr_mbps\":" << st.thrMbps
           << ",\"rbg_share\":" << (totRbg > 0 ? g.rbg[s] / totRbg : -1)
           << ",\"mean_quota\":" << (g.steps ? g.quotaSum[s] / g.steps : -1)
           << ",\"bytes_per_rbg\":" << (g.rbg[s] > 0 ? g.bprW[s] / g.rbg[s] : -1) << "}";
    }
    js << "],\"elev_start\":" << elevStart << ",\"elev_end\":" << elevEnd << ",\"slots\":" << g.slots << ",\"rbg_per_slot\":" << (g.slots ? totRbg / g.slots : -1) << "}";
    std::cout << js.str() << std::endl;
    Simulator::Destroy();
    return 0;
}
