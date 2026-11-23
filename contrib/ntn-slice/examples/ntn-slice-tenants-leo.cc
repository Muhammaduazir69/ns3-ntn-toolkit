/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-slice-tenants-leo: two elastic tenants share ONE 5G-LENA NTN cell while the satellite recedes from the UEs.
 * Tenant A has bought a byte rate R (--contractMbps); tenant B takes what is left; a third slice carries sparse mMTC
 * readings. Both tenants always have data, except that B stops sending in [--bIdleStart, --bIdleStop) to show who may
 * borrow. Policies (--policy):
 *   pf            one BWP, proportional fair, no slicing
 *   qos           5G-LENA OFDMA QoS scheduler, A at 5QI 79 before B at 5QI 9
 *   policed       as qos, with A's traffic limited at the source to its contract, the effect of a maximum bit rate
 *   static        NtnSliceQuotaScheduler with fixed per-slot RBG quotas (--quotas)
 *   contract      quota of A set every --ctrlMs from its measured bytes per RBG:
 *                 headroom * overhead * R / (DL slots per second * RBGs per slot * bytes per RBG), no orbit input
 *   contract_eph  the same, with the bytes per RBG taken from the elevation through a calibration table (--bprTable)
 *   contract_trim quota of A scaled every --ctrlMs by (target / measured transport-block bytes of A)^trimGain, held
 *                 while B leaves its quota unused
 *   limiter       as policed, with A's limit lifted to its offered rate in any period in which B received nothing, the
 *                 best case of a rate limit that follows the neighbor's demand (no signaling delay)
 * Output: one JSON line with per-tenant totals after --warmupS and per-window (--windowS) series of elevation, tenant
 * rates, quota and bytes per RBG.
 */
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-payload-header.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-slice-contract-trim.h"
#include "ns3/ntn-slice-quota-scheduler.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <sstream>

using namespace ns3;
using namespace ns3::ntnslice;

NS_LOG_COMPONENT_DEFINE("NtnSliceTenantsLeo");

namespace
{
// UE u carries slice u % 3; scheduler slice indices: 0 tenant A, 1 tenant B, 2 mMTC
constexpr uint32_t kA = 0, kB = 1, kM = 2;

struct Rx
{
    double t;
    double owdMs;
    uint32_t bytes;
};

struct State
{
    Ptr<NtnSliceQuotaScheduler> sched;
    std::string policy;
    double ctrlS{0.1};
    double headroom{1.05};
    double overhead{1.0};
    double contractBps{0}; // bytes/s
    double trimGain{0.5};
    std::vector<double> floor{0.1, 0.1, 0.05};
    std::vector<std::pair<double, double>> bprTable;
    std::function<double()> elev;
    double rbgPerSlot{0};
    double warmup{3.0};
    std::vector<uint64_t> rbg{0, 0, 0};
    std::vector<double> tbBytes{0, 0, 0};
    uint64_t slots{0};
    std::vector<std::tuple<double, double, double, double>> samples; // t, q_A, bytes/RBG of A, of B
    std::vector<Rx> rx[3];
} g;

double
Interp(const std::vector<std::pair<double, double>>& t, double x)
{
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

void
Step()
{
    const std::vector<double> bpr = g.sched->TakeBytesPerRbg();
    uint64_t slots = 0;
    const std::vector<uint64_t> rbg = g.sched->TakeRbgCounts(&slots);
    g.sched->TakeBacklog();
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
    if (slots > 0 && g.rbgPerSlot > 0 && (g.policy == "contract" || g.policy == "contract_eph"))
    {
        const double measured = bpr.size() > kA ? bpr[kA] : 0.0;
        const double b = g.policy == "contract_eph" ? Interp(g.bprTable, g.elev()) : measured;
        if (b > 0)
        {
            const double slotsPerS = slots / g.ctrlS;
            const double cap = 1.0 - g.floor[kB] - g.floor[kM];
            const double qa = std::min(cap, std::max(g.floor[kA], g.headroom * g.overhead * g.contractBps /
                                                                      (slotsPerS * g.rbgPerSlot * b)));
            q = {qa, 1.0 - qa - g.floor[kM], g.floor[kM]};
            g.sched->SetQuotas(q);
        }
    }
    if (slots > 0 && g.rbgPerSlot > 0 && g.policy == "contract_trim" && bpr.size() > kA && rbg.size() > kA && rbg[kA] > 0 &&
        bpr[kA] > 0)
    {
        // Delivered-rate trim: scale the quota by the ratio of the target to the transport-block bytes A received, so
        // that the leftover RBGs a saturated tenant also wins do not add to its rate.
        const double target = g.headroom * g.overhead * g.contractBps * g.ctrlS;
        const double cap = 1.0 - g.floor[kB] - g.floor[kM];
        // Hold while B leaves its quota unused: A's rate then says nothing about its quota, and correcting on it would
        // wind the quota down to its floor, to be paid for when B returns.
        const bool bBinds = q.size() > kB && rbg.size() > kB &&
                            rbg[kB] >= 0.9 * std::floor(q[kB] * g.rbgPerSlot) * slots;
        if (bBinds)
        {
            const double qa = ntnslice::TrimmedQuota(q[kA], target, bpr[kA] * rbg[kA], g.trimGain, g.floor[kA], cap);
            q = {qa, 1.0 - qa - g.floor[kM], g.floor[kM]};
            g.sched->SetQuotas(q);
        }
    }
    g.samples.emplace_back(now, q.empty() ? -1 : q[kA], bpr.size() > kA ? bpr[kA] : -1, bpr.size() > kB ? bpr[kB] : -1);
    Simulator::Schedule(Seconds(g.ctrlS), &Step);
}

void
LimiterStep(std::vector<Ptr<NtnOranApplication>> aApps, double lowBps, double highBps, size_t seen, bool lifted)
{
    const bool bActive = g.rx[kB].size() > seen;
    if (bActive == lifted)
    {
        lifted = !bActive;
        for (auto& a : aApps)
        {
            a->SetAttribute("DataRate", DataRateValue(DataRate(uint64_t(lifted ? highBps : lowBps))));
        }
    }
    Simulator::Schedule(Seconds(g.ctrlS), &LimiterStep, aApps, lowBps, highBps, g.rx[kB].size(), lifted);
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
    return v[std::min(v.size() - 1, static_cast<size_t>(std::ceil(p * v.size())) - 1)];
}
} // namespace

int
main(int argc, char* argv[])
{
    double duration = 120.0;
    uint32_t numUes = 12;
    std::string quotas = "0.4,0.5,0.1";
    std::string initQuotas = "0.4,0.5,0.1";
    std::string bprTable;
    double ctrlMs = 100.0;
    double satEirpDbm = 55.0;
    double ueBehindS = 40.0;
    double offeredMbps = 30.0;
    double contractMbps = 12.0;
    double bIdleStart = 50.0, bIdleStop = 70.0;
    uint32_t rlcBytes = 200000;
    double windowS = 5.0;
    std::string floors = "0.1,0.1,0.05";
    std::string outputDir = "ntn-slice-tenants-leo-output";
    g.policy = "contract";
    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "simulated seconds", duration);
    cmd.AddValue("numUes", "UEs (a third per slice)", numUes);
    uint64_t dropSeed = 0;
    double clusterOffsetDeg = 0.0;
    bool blockErrors = true;
    cmd.AddValue("dropSeed", "seed of the UE drop and classes; 0 uses the run number", dropSeed);
    cmd.AddValue("clusterOffsetDeg", "latitude offset of the UE square from the sub-satellite point (deg)", clusterOffsetDeg);
    cmd.AddValue("blockErrors", "false disables decoding errors, hence retransmissions", blockErrors);
    cmd.AddValue("policy", "pf | qos | policed | limiter | static | contract | contract_eph | contract_trim", g.policy);
    cmd.AddValue("trimGain", "contract_trim: exponent of the delivered-rate correction", g.trimGain);
    cmd.AddValue("quotas", "static quotas A,B,mMTC", quotas);
    cmd.AddValue("initQuotas", "quotas before the first control step", initQuotas);
    cmd.AddValue("floors", "quota floors A,B,mMTC", floors);
    cmd.AddValue("bprTable", "contract_eph: elevation:bytesPerRbg pairs, comma separated", bprTable);
    cmd.AddValue("ctrlMs", "control period (ms)", ctrlMs);
    cmd.AddValue("headroom", "quota headroom over the contract", g.headroom);
    cmd.AddValue("overhead", "transport-block bytes per application byte of tenant A", g.overhead);
    cmd.AddValue("contractMbps", "contract of tenant A (Mb/s, application bytes)", contractMbps);
    cmd.AddValue("offeredMbps", "offered load per tenant UE (Mb/s)", offeredMbps);
    cmd.AddValue("bIdleStart", "tenant B stops sending (s)", bIdleStart);
    cmd.AddValue("bIdleStop", "tenant B resumes (s)", bIdleStop);
    cmd.AddValue("rlcBytes", "RLC transmission buffer per bearer (bytes)", rlcBytes);
    cmd.AddValue("satEirpDbm", "satellite conducted power (dBm)", satEirpDbm);
    cmd.AddValue("ueBehindS", "UE cluster at the sub-satellite point this many seconds before the start", ueBehindS);
    cmd.AddValue("warmupS", "seconds excluded from the statistics", g.warmup);
    cmd.AddValue("windowS", "window of the time series (s)", windowS);
    cmd.AddValue("outputDir", "output directory", outputDir);
    cmd.Parse(argc, argv);
    g.ctrlS = ctrlMs / 1000.0;
    g.floor = ParseList(floors);
    g.contractBps = contractMbps * 1e6 / 8;
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
    NS_ABORT_MSG_IF(g.policy == "contract_eph" && g.bprTable.empty(), "contract_eph needs --bprTable");

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = 550.0;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    auto satMob = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satMob->SetElements(ns3::ntncon::WalkerConstellation::BuildDelta(wcfg)[0]);
    satNodes.Get(0)->AggregateObject(satMob);
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
    rs.SetRunTag("ntn-slice-tenants-leo");
    rs.SetSatConductedPowerDbm(satEirpDbm);
    rs.SetBackhaulDelay(MilliSeconds(5));
    const bool priority = g.policy == "qos" || g.policy == "policed" || g.policy == "limiter";
    if (priority)
    {
        rs.SetScheduler(NtnRealStackHelper::Scheduler::OfdmaQos);
    }
    else if (g.policy == "pf")
    {
        rs.SetScheduler(NtnRealStackHelper::Scheduler::OfdmaPF);
    }
    else
    {
        rs.SetSchedulerTypeName("ns3::ntnslice::NtnSliceQuotaScheduler");
    }
    rs.Build(satNodes, ueNodes);
    // Build() sets an effectively unlimited RLC buffer; bound it so a tenant's backlog drains soon after it stops
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(rlcBytes));

    const double t0 = 1.0, t1 = duration - 0.5;
    const uint32_t nA = (numUes + 2) / 3;
    std::vector<Ptr<NtnOranApplication>> bApps, aApps;
    for (uint32_t u = 0; u < numUes; ++u)
    {
        const uint32_t s = u % 3;
        ApplicationContainer a;
        if (s == kA)
        {
            a = rs.InstallOranFlow(u, 79, 1, 0x000001, NtnOranApplication::CBR_SATURATING, Seconds(t0), Seconds(t1));
            const bool limited = g.policy == "policed" || g.policy == "limiter";
            const double mbps = limited ? contractMbps / nA : offeredMbps;
            a.Get(0)->SetAttribute("DataRate", DataRateValue(DataRate(uint64_t(mbps * 1e6))));
            aApps.push_back(DynamicCast<NtnOranApplication>(a.Get(0)));
        }
        else if (s == kB)
        {
            a = rs.InstallOranFlow(u, 9, 1, 0x000002, NtnOranApplication::CBR_SATURATING, Seconds(t0), Seconds(t1));
            a.Get(0)->SetAttribute("DataRate", DataRateValue(DataRate(uint64_t(offeredMbps * 1e6))));
            bApps.push_back(DynamicCast<NtnOranApplication>(a.Get(0)));
        }
        else
        {
            a = rs.InstallOranFlow(u, 9, 1, 0x000003, NtnOranApplication::MMTC_PERIODIC, Seconds(t0), Seconds(t1));
        }
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
                    g.rx[slice].push_back({Simulator::Now().GetSeconds(),
                                           (Simulator::Now().GetNanoSeconds() - double(h.GetTxTimestampNs())) / 1e6,
                                           p->GetSize()});
                },
                s));
    }
    if (bIdleStop > bIdleStart)
    {
        Simulator::Schedule(Seconds(bIdleStart), [bApps]() {
            for (auto& a : bApps)
            {
                a->SetTransmitEnabled(false);
            }
        });
        Simulator::Schedule(Seconds(bIdleStop), [bApps]() {
            for (auto& a : bApps)
            {
                a->SetTransmitEnabled(true);
            }
        });
    }

    const bool quotaPolicy = g.policy == "static" || g.policy == "contract" || g.policy == "contract_eph" ||
                             g.policy == "contract_trim";
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
            Simulator::Schedule(Seconds(0.5), []() {
                g.sched->TakeBytesPerRbg();
                g.sched->TakeRbgCounts(nullptr);
                g.sched->TakeBacklog();
            });
            Simulator::Schedule(Seconds(0.5) + Seconds(g.ctrlS), &Step);
        });
    }
    if (g.policy == "limiter")
    {
        Simulator::Schedule(Seconds(t0 + 2.0), &LimiterStep, aApps, contractMbps / nA * 1e6, offeredMbps * 1e6, size_t(0),
                            false);
    }
    std::vector<std::pair<double, double>> elevs;
    for (double t = 0; t < duration; t += windowS / 2)
    {
        Simulator::Schedule(Seconds(t), [t, &elevs]() { elevs.emplace_back(t, g.elev()); });
    }
    Simulator::Stop(Seconds(duration));
    Simulator::Run();
    rs.Collect();
    // ntn-dataset@1: sim_health.csv, ue_kpi/cell_kpi and repro-manifest.json like every
    // other helper-built example (this example used to skip them).
    rs.WriteHealthReport();

    auto bytesIn = [](const std::vector<Rx>& r, double a, double b) {
        double s = 0;
        for (const auto& x : r)
        {
            s += (x.t >= a && x.t < b) ? x.bytes : 0;
        }
        return s;
    };
    const double a0 = g.warmup, a1 = t1;
    std::ostringstream js;
    js << "{\"policy\":\"" << g.policy << "\",\"ues\":" << numUes << ",\"duration\":" << duration
       << ",\"contract_mbps\":" << contractMbps << ",\"slices\":[";
    for (uint32_t s = 0; s < 3; ++s)
    {
        std::vector<double> v;
        for (const auto& x : g.rx[s])
        {
            if (x.t >= a0 && x.t < a1)
            {
                v.push_back(x.owdMs);
            }
        }
        js << (s ? "," : "") << "{\"thr_mbps\":" << bytesIn(g.rx[s], a0, a1) * 8 / (a1 - a0) / 1e6
           << ",\"owd_p50\":" << Quantile(v, 0.5) << ",\"owd_p99\":" << Quantile(v, 0.99)
           << ",\"rbg\":" << (quotaPolicy ? g.rbg[s] : 0)
           << ",\"bytes_per_rbg\":" << (quotaPolicy && g.rbg[s] ? g.tbBytes[s] / g.rbg[s] : -1) << "}";
    }
    js << "],\"slots\":" << g.slots << ",\"rbg_per_slot\":" << g.rbgPerSlot << ",\"windows\":[";
    bool first = true;
    for (double w = a0; w + windowS <= a1 + 1e-9; w += windowS)
    {
        double el = -1;
        for (const auto& e : elevs)
        {
            if (std::abs(e.first - (w + windowS / 2)) <= windowS / 4 + 1e-9)
            {
                el = e.second;
                break;
            }
        }
        double qs = 0, ba = 0, bb = 0;
        int nq = 0, na = 0, nb = 0;
        for (const auto& [t, q, b1, b2] : g.samples)
        {
            if (t >= w && t < w + windowS)
            {
                qs += q;
                ++nq;
                if (b1 > 0)
                {
                    ba += b1;
                    ++na;
                }
                if (b2 > 0)
                {
                    bb += b2;
                    ++nb;
                }
            }
        }
        js << (first ? "" : ",") << "{\"t\":" << w << ",\"elev\":" << el
           << ",\"a_mbps\":" << bytesIn(g.rx[kA], w, w + windowS) * 8 / windowS / 1e6
           << ",\"b_mbps\":" << bytesIn(g.rx[kB], w, w + windowS) * 8 / windowS / 1e6
           << ",\"q_a\":" << (nq ? qs / nq : -1) << ",\"bpr_a\":" << (na ? ba / na : -1)
           << ",\"bpr_b\":" << (nb ? bb / nb : -1) << "}";
        first = false;
    }
    js << "],\"a_fine\":[";
    for (double w = a0; w + 0.5 <= a1 + 1e-9; w += 0.5)
    {
        js << (w > a0 ? "," : "") << bytesIn(g.rx[kA], w, w + 0.5) * 8 / 0.5 / 1e6;
    }
    js << "]}";
    std::cout << js.str() << std::endl;
    Simulator::Destroy();
    return 0;
}
