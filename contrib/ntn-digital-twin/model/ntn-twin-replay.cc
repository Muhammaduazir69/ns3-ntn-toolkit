/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-twin-replay.h"

#include "ns3/contact-graph-scheduler.h"
#include "ns3/log.h"
#include "ns3/orbital-elements.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>

namespace ns3
{
namespace ntntwin
{

NS_LOG_COMPONENT_DEFINE("NtnTwinReplay");

namespace
{
// 20 log10(4 pi / c) with d in m and f in Hz, as in a3_guard.py.
constexpr double kFsplConstDb = 147.55;
// Boltzmann constant, dBW/K/Hz.
constexpr double kBoltzmannDbw = 228.6;
// The radius a3_guard.altitude_km_from_ecef subtracts.
constexpr double kTwinAltRefM = 6378135.0;
} // namespace

double
TwinSlantRangeM(double elevDeg, double altKm, double earthRadiusM)
{
    const double el = std::max(0.0, std::min(90.0, elevDeg)) * M_PI / 180.0;
    const double R = earthRadiusM;
    const double h = altKm * 1000.0;
    const double rs = R * std::sin(el);
    return -rs + std::sqrt(rs * rs + 2.0 * R * h + h * h);
}

double
TwinSnrDb(const TwinA3Params& p, double elevDeg, double altKm)
{
    if (elevDeg <= 0.0)
    {
        return -999.0;
    }
    const double d = TwinSlantRangeM(elevDeg, altKm);
    const double fsplDb = 20.0 * std::log10(std::max(d, 1.0)) +
                          20.0 * std::log10(std::max(p.frequencyHz, 1.0)) - kFsplConstDb;
    const double cn0DbHz = (p.eirpDbm - 30.0) - fsplDb + p.gtDbPerK + kBoltzmannDbw;
    return cn0DbHz - 10.0 * std::log10(std::max(p.bandwidthHz, 1.0));
}

double
TwinAltitudeKmFromEcef(const Vector& ecef)
{
    return std::max(0.0, (ecef.GetLength() - kTwinAltRefM) / 1000.0);
}

TwinA3Evaluator::TwinA3Evaluator(const TwinA3Params& p)
    : m_p(p)
{
}

bool
TwinA3Evaluator::Step(double tSec, const std::vector<TwinCandidate>& cands, TwinA3Event& ev)
{
    const TwinCandidate* best = nullptr;
    double bestQ = 0.0;
    const TwinCandidate* servingNow = nullptr;
    for (const auto& c : cands)
    {
        if (c.elevDeg < m_p.minElevationDeg || c.elevDeg <= -90.0)
        {
            continue;
        }
        const double q = TwinSnrDb(m_p, c.elevDeg, c.altKm);
        // Python max() keeps the FIRST maximum: strict '>' does the same.
        if (best == nullptr || q > bestQ)
        {
            best = &c;
            bestQ = q;
        }
        if (m_hasServing && c.key == m_serving.key)
        {
            servingNow = &c;
        }
    }
    if (best == nullptr)
    {
        // Nothing in view: the condition cannot hold through an outage.
        m_timing = false;
        return false;
    }
    if (!m_hasServing)
    {
        // Initial acquisition is not a handover.
        m_serving = *best;
        m_hasServing = true;
        return false;
    }
    if (servingNow != nullptr)
    {
        m_serving = *servingNow;
    }
    if (best->key == m_serving.key)
    {
        m_timing = false;
        return false;
    }
    const double margin = bestQ - TwinSnrDb(m_p, m_serving.elevDeg, m_serving.altKm);
    if (margin <= m_p.a3OffsetDb)
    {
        m_timing = false;
        return false;
    }
    if (!m_timing || m_target != best->key)
    {
        m_timing = true;
        m_since = tSec;
        m_target = best->key;
    }
    const double held = tSec - m_since;
    const double served = m_hadHo ? tSec - m_lastHoT : 1e9;
    if (held < m_p.timeToTriggerS || served < m_p.minServiceS)
    {
        return false;
    }
    ev.tSec = tSec;
    ev.keyIn = best->key;
    ev.keyOut = m_serving.key;
    ev.marginDb = margin;
    m_serving = *best;
    m_hadHo = true;
    m_lastHoT = tSec;
    m_timing = false;
    return true;
}

TwinReplay::TwinReplay() = default;
TwinReplay::~TwinReplay() = default;

std::size_t
TwinReplay::SetTles(const std::vector<TwinTle>& tles)
{
    m_sats.clear();
    for (const auto& t : tles)
    {
        ntncon::TleRecord rec;
        rec.name = t.name;
        rec.line1 = t.line1;
        rec.line2 = t.line2;
        Ptr<ntncon::Sgp4MobilityModel> m = CreateObject<ntncon::Sgp4MobilityModel>();
        if (m->SetTle(rec) && m->IsUsingSgp4())
        {
            // Sample exactly on the grid; the default 100 ms cache is harmless
            // here but a replay should not depend on it.
            m->SetCacheGranularity(MilliSeconds(1));
            m_sats.push_back(m);
        }
        else
        {
            NS_LOG_WARN("TLE " << t.name << " did not initialize SGP4; skipped");
        }
    }
    return m_sats.size();
}

void
TwinReplay::SetRunMeta(const TwinRunMeta& meta)
{
    m_meta = meta;
}

bool
TwinReplay::Schedule()
{
    if (m_sats.empty() || !m_meta.valid || m_meta.stepS <= 0.0 || m_meta.horizonS <= 0.0)
    {
        return false;
    }
    // Simulation time zero is the twin's SHARED epoch (the prediction header's
    // epoch_unix), not each element set's own epoch: t_s in the prediction
    // file is defined against it.
    for (auto& s : m_sats)
    {
        if (m_meta.epochUnix > 0.0)
        {
            s->SetSimulationEpochUnix(m_meta.epochUnix + m_epochSkewS);
        }
        else if (m_epochSkewS != 0.0)
        {
            s->SetSimulationEpochUnix(s->SimBaseUnix() + m_epochSkewS);
        }
    }
    m_handovers.clear();
    m_eval = TwinA3Evaluator(m_a3);
    m_initialCell = 0;
    m_initialT = -1.0;
    m_argmaxPrev = -1;
    m_samples = 0;
    m_noCoverage = 0;
    const auto nTicks = static_cast<uint64_t>(std::floor((m_meta.horizonS + 1e-9) / m_meta.stepS)) + 1;
    for (uint64_t k = 0; k < nTicks; ++k)
    {
        Simulator::Schedule(Seconds(k * m_meta.stepS), &TwinReplay::Sample, this);
    }
    return true;
}

double
TwinReplay::ElevationOfCellDeg(uint32_t cell) const
{
    if (cell == 0 || cell > m_sats.size())
    {
        return -90.0;
    }
    return ntncon::ContactGraphScheduler::ElevationFromEcef(m_sats[cell - 1]->GetPosition(),
                                                            m_meta.observerLatDeg,
                                                            m_meta.observerLonDeg,
                                                            m_meta.observerAltM);
}

uint64_t
TwinReplay::GetSgp4Fallbacks() const
{
    uint64_t n = 0;
    for (const auto& s : m_sats)
    {
        n += s->GetSgp4FallbackCount();
    }
    return n;
}

void
TwinReplay::Sample()
{
    ++m_samples;
    const double t = Simulator::Now().GetSeconds();
    std::vector<TwinCandidate> cands;
    cands.reserve(m_sats.size());
    int64_t argmax = -1;
    double argmaxEl = -90.0;
    for (std::size_t i = 0; i < m_sats.size(); ++i)
    {
        const Vector p = m_sats[i]->GetPosition();
        const double el = ntncon::ContactGraphScheduler::ElevationFromEcef(p,
                                                                           m_meta.observerLatDeg,
                                                                           m_meta.observerLonDeg,
                                                                           m_meta.observerAltM);
        cands.push_back({static_cast<uint32_t>(i), el, TwinAltitudeKmFromEcef(p)});
        if (el > argmaxEl)
        {
            argmaxEl = el;
            argmax = static_cast<int64_t>(i);
        }
    }
    const bool covered = argmax >= 0 && argmaxEl > std::max(0.0, m_a3.minElevationDeg);
    if (!covered)
    {
        ++m_noCoverage;
    }

    if (m_policy == TwinReplayPolicy::A3Guard)
    {
        const bool hadServing = m_eval.HasServing();
        TwinA3Event ev;
        if (m_eval.Step(t, cands, ev))
        {
            m_handovers.push_back({t, 0, ev.keyIn + 1, 1.0});
        }
        if (!hadServing && m_eval.HasServing())
        {
            m_initialCell = m_eval.GetServing() + 1;
            m_initialT = t;
        }
        return;
    }

    if (!covered)
    {
        return;
    }
    if (m_argmaxPrev < 0)
    {
        m_initialCell = static_cast<uint32_t>(argmax + 1);
        m_initialT = t;
    }
    else if (argmax != m_argmaxPrev)
    {
        m_handovers.push_back({t, 0, static_cast<uint32_t>(argmax + 1), 1.0});
    }
    m_argmaxPrev = argmax;
}

} // namespace ntntwin
} // namespace ns3
