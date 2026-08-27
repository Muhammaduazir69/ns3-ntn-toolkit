/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 */

#include "oran-ntn-loop-latency-probe.h"

#include <ns3/log.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <numeric>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnLoopLatencyProbe");
NS_OBJECT_ENSURE_REGISTERED(OranNtnLoopLatencyProbe);

TypeId
OranNtnLoopLatencyProbe::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnLoopLatencyProbe")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnLoopLatencyProbe>();
    return tid;
}

void
OranNtnLoopLatencyProbe::DoDispose()
{
    m_samples.clear();
    m_order.clear();
    m_open.clear();
    m_loopSamples.clear();
    Object::DoDispose();
}

const char*
OranNtnLoopLatencyProbe::KindToString(Kind k)
{
    return (k == Kind::CPU) ? "cpu" : "simulated";
}

// ---- CPU stage timing ----

void
OranNtnLoopLatencyProbe::StartStage(const std::string& stage)
{
    // A re-Start on an open stage replaces the open timestamp: the abandoned
    // interval is dropped rather than being attributed to the new one.
    m_open[stage] = std::chrono::steady_clock::now();
}

void
OranNtnLoopLatencyProbe::EndStage(const std::string& stage)
{
    auto it = m_open.find(stage);
    if (it == m_open.end())
    {
        NS_LOG_WARN("EndStage(\"" << stage << "\") without a matching StartStage; ignored");
        return;
    }
    const auto elapsed = std::chrono::steady_clock::now() - it->second;
    m_open.erase(it);
    RecordCpuSample(stage,
                    std::chrono::duration<double, std::micro>(elapsed).count());
}

void
OranNtnLoopLatencyProbe::RecordCpuSample(const std::string& stage, double us)
{
    const Key key{stage, Kind::CPU};
    auto [it, inserted] = m_samples.try_emplace(key);
    if (inserted)
    {
        m_order.push_back(key);
    }
    it->second.push_back(us);
}

OranNtnLoopLatencyProbe::ScopedCpuTimer::ScopedCpuTimer(Ptr<OranNtnLoopLatencyProbe> probe,
                                                        std::string stage)
    : m_probe(probe),
      m_stage(std::move(stage)),
      m_start(std::chrono::steady_clock::now())
{
}

OranNtnLoopLatencyProbe::ScopedCpuTimer::~ScopedCpuTimer()
{
    if (!m_probe)
    {
        return;
    }
    const auto elapsed = std::chrono::steady_clock::now() - m_start;
    m_probe->RecordCpuSample(m_stage,
                             std::chrono::duration<double, std::micro>(elapsed).count());
}

// ---- Simulated stage timing ----

void
OranNtnLoopLatencyProbe::RecordSimulatedStage(const std::string& stage, Time delta)
{
    const Key key{stage, Kind::SIMULATED};
    auto [it, inserted] = m_samples.try_emplace(key);
    if (inserted)
    {
        m_order.push_back(key);
    }
    it->second.push_back(delta.GetNanoSeconds() / 1000.0);
}

void
OranNtnLoopLatencyProbe::RecordLoop(Time measureTime, Time actuationTime)
{
    const Time delta = actuationTime - measureTime;
    RecordSimulatedStage(oranntn::loopstage::kLoopTotal, delta);

    LoopSample s;
    s.iter = static_cast<uint32_t>(m_loopSamples.size());
    s.measure_t_s = measureTime.GetSeconds();
    s.actuation_t_s = actuationTime.GetSeconds();
    s.loop_latency_ms = delta.GetNanoSeconds() / 1e6;
    m_loopSamples.push_back(s);
}

// ---- Instrument self-cost ----

void
OranNtnLoopLatencyProbe::MeasureInstrumentOverhead(uint32_t iterations)
{
    for (uint32_t i = 0; i < iterations; ++i)
    {
        StartStage(oranntn::loopstage::kProbeSelfOverhead);
        EndStage(oranntn::loopstage::kProbeSelfOverhead);
    }
}

// ---- Query ----

bool
OranNtnLoopLatencyProbe::HasStage(const std::string& stage, Kind kind) const
{
    auto it = m_samples.find(Key{stage, kind});
    return it != m_samples.end() && !it->second.empty();
}

uint64_t
OranNtnLoopLatencyProbe::GetStageCount(const std::string& stage, Kind kind) const
{
    auto it = m_samples.find(Key{stage, kind});
    return (it == m_samples.end()) ? 0 : static_cast<uint64_t>(it->second.size());
}

double
OranNtnLoopLatencyProbe::Percentile(const std::vector<double>& sorted, double p)
{
    if (sorted.empty())
    {
        return 0.0;
    }
    // Nearest-rank (no interpolation): every reported percentile is an
    // observed sample.
    const double n = static_cast<double>(sorted.size());
    long idx = static_cast<long>(std::ceil(p * n)) - 1;
    idx = std::max<long>(0, std::min<long>(idx, static_cast<long>(sorted.size()) - 1));
    return sorted[static_cast<size_t>(idx)];
}

bool
OranNtnLoopLatencyProbe::GetStats(const std::string& stage, Kind kind, StageStats& out) const
{
    auto it = m_samples.find(Key{stage, kind});
    if (it == m_samples.end() || it->second.empty())
    {
        return false;
    }

    std::vector<double> v = it->second;
    std::sort(v.begin(), v.end());

    out.count = static_cast<uint64_t>(v.size());
    out.mean_us = std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
    out.p50_us = Percentile(v, 0.50);
    out.p95_us = Percentile(v, 0.95);
    out.p99_us = Percentile(v, 0.99);
    out.min_us = v.front();
    out.max_us = v.back();
    return true;
}

std::vector<std::pair<std::string, OranNtnLoopLatencyProbe::Kind>>
OranNtnLoopLatencyProbe::GetStageKeys() const
{
    std::vector<Key> keys;
    keys.reserve(m_order.size());
    for (const auto& k : m_order)
    {
        auto it = m_samples.find(k);
        if (it != m_samples.end() && !it->second.empty())
        {
            keys.push_back(k);
        }
    }
    return keys;
}

void
OranNtnLoopLatencyProbe::Clear()
{
    m_samples.clear();
    m_order.clear();
    m_open.clear();
    m_loopSamples.clear();
}

// ---- Output ----

namespace
{

void
EnsureParentDir(const std::string& path)
{
    std::error_code ec;
    const std::filesystem::path p(path);
    if (p.has_parent_path() && !p.parent_path().empty())
    {
        std::filesystem::create_directories(p.parent_path(), ec);
    }
}

} // namespace

bool
OranNtnLoopLatencyProbe::WriteCsv(const std::string& path) const
{
    EnsureParentDir(path);
    std::ofstream ofs(path);
    if (!ofs.is_open())
    {
        NS_LOG_WARN("Cannot open " << path << " for writing");
        return false;
    }

    ofs << "stage,kind,count,mean_us,p50_us,p95_us,p99_us,min_us,max_us\n";
    ofs << std::fixed << std::setprecision(3);

    for (const auto& key : GetStageKeys())
    {
        StageStats s;
        if (!GetStats(key.first, key.second, s))
        {
            continue;
        }
        ofs << key.first << ',' << KindToString(key.second) << ',' << s.count << ','
            << s.mean_us << ',' << s.p50_us << ',' << s.p95_us << ',' << s.p99_us << ','
            << s.min_us << ',' << s.max_us << '\n';
    }
    return true;
}

bool
OranNtnLoopLatencyProbe::WriteSamplesCsv(const std::string& path) const
{
    EnsureParentDir(path);
    std::ofstream ofs(path);
    if (!ofs.is_open())
    {
        NS_LOG_WARN("Cannot open " << path << " for writing");
        return false;
    }

    ofs << "iter,measure_t_s,actuation_t_s,loop_latency_ms\n";
    ofs << std::fixed << std::setprecision(6);
    for (const auto& s : m_loopSamples)
    {
        ofs << s.iter << ',' << s.measure_t_s << ',' << s.actuation_t_s << ','
            << s.loop_latency_ms << '\n';
    }
    return true;
}

} // namespace ns3
