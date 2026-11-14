/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-slice-pass-leo: three slices on ONE 5G-LENA NTN cell while the satellite RECEDES from the UE cluster, so the
 * bytes a resource block group (RBG) carries fall during the run. Slices: eMBB (saturating, 5QI 9), delay-critical
 * (periodic V2X-type messages, 5QI 79, packet delay budget --deadlineMs), mMTC (sparse readings, 5QI 9).
 * Policies (--policy):
 *   pf        one BWP, proportional fair, no slicing
 *   qos       one BWP, 5G-LENA OFDMA QoS scheduler (5QI priority, 79 before 9), no slicing
 *   bwp       two BWPs of equal width, one for the delay-critical 5QI and one for the rest
 *   static    NtnSliceQuotaScheduler with fixed per-slot RBG quotas (--quotas)
 *   predict   quotas set every --ctrlMs from the satellite ephemeris: the elevation seen from the cluster gives the
 *             bytes per RBG through a calibration table (--bprTable), and the delay-critical quota is
 *             headroom * offered load / (DL slots per second * RBGs per slot * bytes per RBG)
 *   reactive  quotas set every --ctrlMs from measured RBG usage, backlog and bytes per RBG (no orbit input);
 *             warm-started from --initQuotas and holding its quotas through periods without traffic
 * The UE cluster is placed at the sub-satellite point of --ueBehindS seconds before the run starts, so the
 * satellite is already past it and its elevation falls over the run. Per-packet one-way delays are taken from the
 * sinks' Rx trace (send timestamp in the payload header). Output: one JSON line with per-slice totals after
 * --warmupS and per-window (--windowS) series of elevation, delay-critical p99 and deadline misses, throughputs,
 * quota and bytes per RBG.
 */
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-payload-header.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-slice-quota-scheduler.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <numeric>
#include <sstream>

using namespace ns3;
using namespace ns3::ntnslice;

NS_LOG_COMPONENT_DEFINE("NtnSlicePassLeo");

namespace
{
// UE u carries slice u % 3; scheduler slice indices: 0 eMBB, 1 delay-critical, 2 mMTC
constexpr uint32_t kEmbb = 0, kCrit = 1, kMmtc = 2;

struct Rx
{
    double t;     // receive time (s)
    double owdMs; // one-way delay (ms)
    uint32_t bytes;
};

struct State
{
    Ptr<NtnSliceQuotaScheduler> sched;
    std::string policy;
    double ctrlS{0.1};
    double headroom{1.25};
    std::vector<double> floor{0.1, 0.1, 0.05};
    double lambdaBps{0};                // offered delay-critical load, bytes/s
    std::vector<std::pair<double, double>> bprTable; // (elevation deg, bytes per RBG), ascending elevation
    std::function<double()> elev;
    double rbgPerSlot{0};
    // accounting over the whole run after the warm-up (quota policies only)
    std::vector<uint64_t> rbg{0, 0, 0};
    std::vector<double> tbBytes{0, 0, 0};
    uint64_t slots{0};
    double warmup{0};
    // per-control-step samples: time, delay-critical quota, bytes per RBG of that slice
    std::vector<std::tuple<double, double, double>> samples;
    std::vector<Rx> rx[3];
} g;

double
Interp(const std::vector<std::pair<double, double>>& t, double x)
{
    if (t.empty())
    {
        return 0;
    }
    if (x <= t.front().first)
    {
        return t.front().second;
    }
    for (size_t i = 1; i < t.size(); ++i)
    {
        if (x <= t[i].first)
        {
            const double a = (x - t[i - 1].first) / (t[i].first - t[i - 1].first);
            return t[i - 1].second + a * (t[i].second - t[i - 1].second);
        }
    }
    return t.back().second;
}

/// One control step: read the scheduler's counters, then set the quotas of the policy.
void
Step()
{
    const std::vector<double> bpr = g.sched->TakeBytesPerRbg();
    uint64_t slots = 0;
    const std::vector<uint64_t> rbg = g.sched->TakeRbgCounts(&slots);
    const std::vector<double> backlog = g.sched->TakeBacklog();
    const double now = Simulator::Now().GetSeconds();
    const double tot = std::accumulate(rbg.begin(), rbg.end(), 0.0);
    if (now > g.warmup)
    {
        for (size_t s = 0; s < 3 && s < rbg.size(); ++s)
        {
            g.rbg[s] += rbg[s];
            g.tbBytes[s] += (s < bpr.size() ? bpr[s] : 0.0) * rbg[s];
        }
        g.slots += slots;
    }
    if (slots > 0)
    {
        g.rbgPerSlot = std::max(g.rbgPerSlot, tot / slots);
    }
    std::vector<double> q = g.sched->GetQuotas();
    const double periodRbg = g.rbgPerSlot * slots;
    if (tot > 0 && periodRbg > 0 && (g.policy == "reactive" || g.policy == "predict"))
    {
        std::vector<double> need(3, 0.0);
        if (g.policy == "reactive")
        {
            double meanBpr = 0;
            for (size_t s = 0; s < 3 && s < bpr.size(); ++s)
            {
                meanBpr += bpr[s] * rbg[s] / tot;
            }
            for (size_t s = 0; s < 3; ++s)
            {
                const double b = (s < bpr.size() && bpr[s] > 0) ? bpr[s] : std::max(meanBpr, 1.0);
                const double r = (s < backlog.size() ? backlog[s] / b : 0.0) + (s < rbg.size() ? double(rbg[s]) : 0.0);
                need[s] = std::max(g.floor[s], std::min(1.0, g.headroom * r / periodRbg));
            }
        }
        else
        {
            // ephemeris: bytes per RBG of the delay-critical slice from the elevation, its quota from the offered load
            const double b = Interp(g.bprTable, g.elev());
            const double slotsPerS = slots / g.ctrlS;
            need[kCrit] = std::min(1.0, g.headroom * g.lambdaBps / (slotsPerS * g.rbgPerSlot * b));
            need[kMmtc] = g.floor[kMmtc];
            need[kEmbb] = g.floor[kEmbb];
        }
        q.assign(g.floor.begin(), g.floor.end());
        double left = 1.0 - std::accumulate(q.begin(), q.end(), 0.0);
        for (size_t s : {kCrit, kMmtc, kEmbb})
        {
            const double extra = std::min(std::max(need[s] - q[s], 0.0), left);
            q[s] += extra;
            left -= extra;
        }
        q[kEmbb] += left;
        g.sched->SetQuotas(q);
    }
    g.samples.emplace_back(now, q.size() > kCrit ? q[kCrit] : -1, bpr.size() > kCrit ? bpr[kCrit] : -1);
    Simulator::Schedule(Seconds(g.ctrlS), &Step);
}

std::vector<double>
ParseList(const std::string& s)
{
    std::vector<double> v;
    std::stringstream ss(s);
    std::string x;
    while (std::getline(ss, x, ','))
    {
        if (!x.empty())
        {
            v.push_back(std::stod(x));
        }
    }
    return v;
}

double
Quantile(std::vector<double> v, double p)
{
    if (v.empty())
    {
        return -1;
    }
    std::sort(v.begin(), v.end());
    const size_t i = std::min(v.size() - 1, static_cast<size_t>(std::ceil(p * v.size())) - (p > 0 ? 1 : 0));
    return v[i];
}
} // namespace

int
main(int argc, char* argv[])
{
    double duration = 100.0;
    uint32_t numUes = 12;
    std::string quotas = "0.4,0.5,0.1";
    std::string initQuotas = "0.4,0.5,0.1";
    std::string bprTable;
    double ctrlMs = 100.0;
    double satEirpDbm = 55.0;
    double ueBehindS = 20.0;
    double embbMbps = 30.0;
    uint32_t critBytes = 500;
    double critPeriodMs = 1.0;
    double deadlineMs = 50.0;
    double windowS = 5.0;
    double startOffset = 0.0;
    std::string floors = "0.1,0.1,0.05";
    std::string outputDir = "ntn-slice-pass-leo-output";
    g.policy = "reactive";
    g.warmup = 3.0;
    CommandLine cmd(__FILE__);
    uint64_t dropSeed = 0;
    double clusterOffsetDeg = 0.0;
    bool blockErrors = true;
    cmd.AddValue("dropSeed", "seed of the UE drop and classes; 0 uses the run number", dropSeed);
    cmd.AddValue("clusterOffsetDeg", "latitude offset of the UE square from the sub-satellite point (deg)", clusterOffsetDeg);
    cmd.AddValue("blockErrors", "false disables decoding errors, hence retransmissions", blockErrors);
    cmd.AddValue("duration", "simulated seconds", duration);
    cmd.AddValue("numUes", "UEs (a third per slice)", numUes);
    cmd.AddValue("policy", "pf | qos | bwp | static | predict | reactive", g.policy);
    cmd.AddValue("quotas", "static quotas eMBB,critical,mMTC", quotas);
    cmd.AddValue("initQuotas", "quotas of predict/reactive before their first step", initQuotas);
    cmd.AddValue("floors", "quota floors eMBB,critical,mMTC", floors);
    cmd.AddValue("bprTable", "predict: elevation:bytesPerRbg pairs, comma separated", bprTable);
    cmd.AddValue("ctrlMs", "control period (ms)", ctrlMs);
    cmd.AddValue("headroom", "quota headroom over the measured or predicted need", g.headroom);
    cmd.AddValue("satEirpDbm", "satellite conducted power (dBm)", satEirpDbm);
    cmd.AddValue("ueBehindS", "UE cluster at the sub-satellite point this many seconds before the start", ueBehindS);
    cmd.AddValue("embbMbps", "offered load per eMBB UE (Mb/s)", embbMbps);
    cmd.AddValue("critBytes", "delay-critical packet size (bytes)", critBytes);
    cmd.AddValue("critPeriodMs", "delay-critical packet period per UE (ms)", critPeriodMs);
    cmd.AddValue("deadlineMs", "delay-critical packet delay budget D (ms)", deadlineMs);
    cmd.AddValue("warmupS", "seconds excluded from the statistics", g.warmup);
    cmd.AddValue("windowS", "window of the time series (s)", windowS);
    cmd.AddValue("startOffset", "seconds added to the constellation epoch", startOffset);
    cmd.AddValue("outputDir", "output directory", outputDir);
    cmd.Parse(argc, argv);
    g.ctrlS = ctrlMs / 1000.0;
    g.floor = ParseList(floors);
    {
        std::stringstream ss(bprTable);
        std::string x;
        while (std::getline(ss, x, ','))
        {
            const auto c = x.find(':');
            if (c != std::string::npos)
            {
                g.bprTable.emplace_back(std::stod(x.substr(0, c)), std::stod(x.substr(c + 1)));
            }
        }
        std::sort(g.bprTable.begin(), g.bprTable.end());
    }
    NS_ABORT_MSG_IF(g.policy == "predict" && g.bprTable.empty(), "predict needs --bprTable");
    const uint32_t nCrit = (numUes + 0) / 3;
    g.lambdaBps = nCrit * critBytes / (critPeriodMs / 1000.0);

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
    auto satMob = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satMob->SetElements(ns3::ntncon::WalkerConstellation::BuildDelta(wcfg)[0]);
    satNodes.Get(0)->AggregateObject(satMob);
    // the cluster sits where the satellite was ueBehindS seconds earlier, so the satellite recedes from it
    double subLat, subLon, subAlt;
    {
        ns3::ntncon::WalkerConfig w0 = wcfg;
        w0.epoch_unix_s -= ueBehindS;
        auto m0 = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
        m0->SetElements(ns3::ntncon::WalkerConstellation::BuildDelta(w0)[0]);
        m0->GetGeodetic(subLat, subLon, subAlt);
    }
    subLat += clusterOffsetDeg;
    NtnTr38811MobilityHelper ueMob(dropSeed > 0 ? dropSeed : RngSeedManager::GetRun());
    ueMob.Install(ueNodes, NtnMobilityScenarios::MixedContinental(), subLat - 0.03, subLat + 0.03, subLon - 0.03,
                  subLon + 0.03);
    g.elev = [satNodes, ueNodes]() {
        const Vector s = satNodes.Get(0)->GetObject<MobilityModel>()->GetPosition();
        const Vector u = ueNodes.Get(0)->GetObject<MobilityModel>()->GetPosition();
        const Vector d(s.x - u.x, s.y - u.y, s.z - u.z);
        const double nu = std::sqrt(u.x * u.x + u.y * u.y + u.z * u.z);
        const double nd = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        return std::asin((d.x * u.x + d.y * u.y + d.z * u.z) / (nu * nd)) * 180.0 / M_PI;
    };

    Config::SetDefault("ns3::NrHelper::UseIdealRrc", BooleanValue(true));
    Config::SetDefault("ns3::NrSpectrumPhy::DataErrorModelEnabled", BooleanValue(blockErrors));
    NtnRealStackHelper rs;
    rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
    rs.SetNumerology(1);
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-slice-pass-leo");
    rs.SetSatConductedPowerDbm(satEirpDbm);
    rs.SetBackhaulDelay(MilliSeconds(5));
    if (g.policy == "bwp")
    {
        rs.SetSlices({{"critical", 79}, {"rest", 9}});
    }
    else if (g.policy == "pf")
    {
        rs.SetScheduler(NtnRealStackHelper::Scheduler::OfdmaPF);
    }
    else if (g.policy == "qos")
    {
        rs.SetScheduler(NtnRealStackHelper::Scheduler::OfdmaQos);
    }
    else
    {
        rs.SetSchedulerTypeName("ns3::ntnslice::NtnSliceQuotaScheduler");
    }
    rs.Build(satNodes, ueNodes);

    const double t0 = 1.0, t1 = duration - 0.5;
    std::vector<Ptr<NtnOranApplication>> clients;
    for (uint32_t u = 0; u < numUes; ++u)
    {
        const uint32_t s = u % 3;
        ApplicationContainer a;
        if (s == kEmbb)
        {
            a = rs.InstallOranFlow(u, 9, 1, 0x000001, NtnOranApplication::CBR_SATURATING, Seconds(t0), Seconds(t1));
            a.Get(0)->SetAttribute("DataRate", DataRateValue(DataRate(uint64_t(embbMbps * 1e6))));
        }
        else if (s == kCrit)
        {
            a = rs.InstallOranFlow(u, 79, 1, 0x000002, NtnOranApplication::URLLC_PERIODIC, Seconds(t0), Seconds(t1));
            a.Get(0)->SetAttribute("PacketSize", UintegerValue(critBytes));
            a.Get(0)->SetAttribute("Period", TimeValue(Seconds(critPeriodMs / 1000.0)));
        }
        else
        {
            a = rs.InstallOranFlow(u, 9, 1, 0x000003, NtnOranApplication::MMTC_PERIODIC, Seconds(t0), Seconds(t1));
        }
        clients.push_back(DynamicCast<NtnOranApplication>(a.Get(0)));
        a.Get(1)->TraceConnectWithoutContext(
            "Rx",
            MakeBoundCallback(
                +[](uint32_t slice, Ptr<const Packet> p, const Address&) {
                    if (p->GetSize() < NtnOranPayloadHeader::SERIALIZED_SIZE)
                    {
                        return;
                    }
                    NtnOranPayloadHeader h;
                    p->PeekHeader(h);
                    const double now = Simulator::Now().GetSeconds();
                    g.rx[slice].push_back({now, (Simulator::Now().GetNanoSeconds() - double(h.GetTxTimestampNs())) / 1e6,
                                           p->GetSize()});
                },
                s));
    }

    const bool quotaPolicy = g.policy == "static" || g.policy == "reactive" || g.policy == "predict";
    if (quotaPolicy)
    {
        auto gnb = DynamicCast<NrGnbNetDevice>(rs.GetEnbDevices().Get(0));
        g.sched = DynamicCast<NtnSliceQuotaScheduler>(gnb->GetScheduler(0));
        NS_ABORT_MSG_IF(!g.sched, "the gNB does not run NtnSliceQuotaScheduler");
        g.sched->SetQuotas(ParseList(g.policy == "static" ? quotas : initQuotas));
        Simulator::Schedule(Seconds(0.5), [numUes, &rs]() {
            for (uint32_t u = 0; u < numUes; ++u)
            {
                g.sched->SetSliceOfRnti(rs.GetUeRnti(u), u % 3);
            }
            // counters restart when the traffic starts, so the first step reads one full period of traffic
            Simulator::Schedule(Seconds(0.5), []() {
                g.sched->TakeBytesPerRbg();
                g.sched->TakeRbgCounts(nullptr);
                g.sched->TakeBacklog();
            });
            Simulator::Schedule(Seconds(0.5) + Seconds(g.ctrlS), &Step);
        });
    }
    // elevation every window
    std::vector<std::pair<double, double>> elevs;
    for (double t = 0; t <= duration; t += windowS)
    {
        Simulator::Schedule(Seconds(std::min(t, duration - 0.001)), [t, &elevs]() { elevs.emplace_back(t, g.elev()); });
    }
    Simulator::Stop(Seconds(duration));
    Simulator::Run();
    rs.Collect();

    const double D = deadlineMs;
    auto owds = [](const std::vector<Rx>& r, double a, double b) {
        std::vector<double> v;
        for (const auto& x : r)
        {
            if (x.t >= a && x.t < b)
            {
                v.push_back(x.owdMs);
            }
        }
        return v;
    };
    auto bytesIn = [](const std::vector<Rx>& r, double a, double b) {
        double s = 0;
        for (const auto& x : r)
        {
            if (x.t >= a && x.t < b)
            {
                s += x.bytes;
            }
        }
        return s;
    };
    const double a0 = g.warmup, a1 = t1;
    std::ostringstream js;
    js << "{\"policy\":\"" << g.policy << "\",\"ues\":" << numUes << ",\"duration\":" << duration
       << ",\"deadline_ms\":" << D << ",\"lambda_Bps\":" << g.lambdaBps << ",\"slices\":[";
    for (uint32_t s = 0; s < 3; ++s)
    {
        const auto v = owds(g.rx[s], a0, a1);
        const double miss = v.empty() ? -1
                                      : double(std::count_if(v.begin(), v.end(), [D](double x) { return x > D; })) /
                                            v.size();
        uint64_t tx = 0;
        for (uint32_t u = s; u < numUes; u += 3)
        {
            tx += clients[u]->GetTxPackets();
        }
        js << (s ? "," : "") << "{\"rx\":" << g.rx[s].size() << ",\"tx\":" << tx
           << ",\"thr_mbps\":" << bytesIn(g.rx[s], a0, a1) * 8 / (a1 - a0) / 1e6 << ",\"owd_p50\":" << Quantile(v, 0.5)
           << ",\"owd_p99\":" << Quantile(v, 0.99) << ",\"owd_p999\":" << Quantile(v, 0.999)
           << ",\"miss\":" << miss << ",\"rbg\":" << (quotaPolicy ? g.rbg[s] : 0)
           << ",\"bytes_per_rbg\":" << (quotaPolicy && g.rbg[s] ? g.tbBytes[s] / g.rbg[s] : -1) << "}";
    }
    js << "],\"slots\":" << g.slots << ",\"rbg_per_slot\":" << g.rbgPerSlot << ",\"windows\":[";
    bool first = true;
    for (double w = a0; w + windowS <= a1 + 1e-9; w += windowS)
    {
        const auto v = owds(g.rx[kCrit], w, w + windowS);
        const double miss = v.empty() ? -1
                                      : double(std::count_if(v.begin(), v.end(), [D](double x) { return x > D; })) /
                                            v.size();
        double el = -1;
        for (const auto& e : elevs)
        {
            if (std::abs(e.first - (w + windowS / 2)) <= windowS / 2 + 1e-9)
            {
                el = e.second;
                break;
            }
        }
        double qs = 0, bs = 0;
        int nq = 0, nb = 0;
        for (const auto& [t, q, b] : g.samples)
        {
            if (t >= w && t < w + windowS)
            {
                qs += q;
                ++nq;
                if (b > 0)
                {
                    bs += b;
                    ++nb;
                }
            }
        }
        js << (first ? "" : ",") << "{\"t\":" << w << ",\"elev\":" << el << ",\"crit_p99\":" << Quantile(v, 0.99)
           << ",\"crit_miss\":" << miss
           << ",\"crit_mbps\":" << bytesIn(g.rx[kCrit], w, w + windowS) * 8 / windowS / 1e6
           << ",\"embb_mbps\":" << bytesIn(g.rx[kEmbb], w, w + windowS) * 8 / windowS / 1e6
           << ",\"q_crit\":" << (nq ? qs / nq : -1) << ",\"bpr_crit\":" << (nb ? bs / nb : -1) << "}";
        first = false;
    }
    js << "]}";
    std::cout << js.str() << std::endl;
    Simulator::Destroy();
    return 0;
}
