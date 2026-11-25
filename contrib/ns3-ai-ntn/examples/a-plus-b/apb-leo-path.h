/*
 * Copyright (c) 2026 Muhammad Uzair
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * NTN input for the a-plus-b AI-bridge examples. Instead of random integers, the C++ side sends
 * the two one-way delays of a LEO bent pipe, user terminal to satellite (a) and satellite to
 * feeder gateway (b), in microseconds; the Python agent returns their sum, the end-to-end
 * one-way delay the terminal sees. The satellites are a Walker-delta shell propagated with SGP4
 * (ntn-constellation); the serving satellite is the one with the highest elevation over both
 * ends and is kept until it drops below the minimum elevation at either end. The ground ends and
 * the satellites that serve are device-less nodes, so a globe scene of the pass can be recorded.
 */

#ifndef APB_LEO_PATH_H
#define APB_LEO_PATH_H

#include "ns3/constant-position-mobility-model.h"
#include "ns3/node-container.h"
#include "ns3/ntn-dataset-writer.h"
#include "ns3/ntn-geocentric-mobility-adapter.h"
#include "ns3/nstime.h"
#include "ns3/orbital-elements.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace apbleo
{

using namespace ns3;

/// One step of the pass.
struct Sample
{
    double timeS{0.0};      //!< time since the start of the pass (s)
    int32_t satIndex{-1};   //!< serving satellite (-1: none above the minimum elevation)
    uint32_t upUs{0};       //!< terminal to satellite one-way delay (us)
    uint32_t downUs{0};     //!< satellite to gateway one-way delay (us)
};

/// LEO bent pipe between a terminal (Toulouse) and a feeder gateway (Madrid).
class LeoPath
{
  public:
    /**
     * Build the shell and the ground ends.
     * @param altKm shell altitude (km)
     * @param inclinationDeg shell inclination (deg)
     * @param minElevationDeg minimum elevation at both ends (deg)
     */
    void Build(double altKm = 550.0, double inclinationDeg = 53.0, double minElevationDeg = 25.0)
    {
        m_minElevationDeg = minElevationDeg;
        m_ut = ntncon::NtnGeocentricMobilityAdapter::GeodeticToEcef(Vector(43.6045, 1.4440, 150.0));
        m_gw = ntncon::NtnGeocentricMobilityAdapter::GeodeticToEcef(Vector(40.4168, -3.7038, 650.0));
        ntncon::WalkerConfig w;
        w.num_planes = 72;
        w.total_sats = 72 * 22;
        w.altitude_km = altKm;
        w.inclination_deg = inclinationDeg;
        w.epoch_unix_s = 1735689600.0; // 2025-01-01T00:00:00Z
        for (const auto& el : ntncon::WalkerConstellation::BuildDelta(w))
        {
            auto m = CreateObject<ntncon::Sgp4MobilityModel>();
            m->SetElementsSgp4(el);
            m_sats.push_back(m);
        }
    }

    /**
     * The delays at time t (s); call with non-decreasing t.
     * @param t time since the start of the pass (s)
     * @return the sample at t
     */
    Sample At(double t)
    {
        Sample s;
        s.timeS = t;
        if (m_serving < 0 || PathElevationDeg(SatEcef(m_serving, t)) < m_minElevationDeg)
        {
            m_serving = -1;
            double best = m_minElevationDeg;
            for (uint32_t i = 0; i < m_sats.size(); ++i)
            {
                const double el = PathElevationDeg(SatEcef(i, t));
                if (el >= best)
                {
                    best = el;
                    m_serving = static_cast<int32_t>(i);
                }
            }
            if (m_serving >= 0 && std::find(m_used.begin(), m_used.end(), m_serving) == m_used.end())
            {
                m_used.push_back(m_serving);
            }
            if (m_serving >= 0 && m_last >= 0 && m_serving != m_last)
            {
                ++m_handovers;
            }
        }
        s.satIndex = m_serving;
        if (m_serving >= 0)
        {
            const Vector p = SatEcef(m_serving, t);
            constexpr double c = 299792458.0;
            s.upUs = static_cast<uint32_t>(std::lround(1e6 * CalculateDistance(m_ut, p) / c));
            s.downUs = static_cast<uint32_t>(std::lround(1e6 * CalculateDistance(p, m_gw) / c));
            m_last = m_serving;
        }
        return s;
    }

    /**
     * Sample the whole pass before the simulation starts (t = 0), so the satellites that serve
     * are known when the scene nodes are created.
     * @param steps number of samples
     * @param dtS spacing between samples (s)
     * @return the samples
     */
    std::vector<Sample> Plan(uint32_t steps, double dtS)
    {
        std::vector<Sample> out;
        out.reserve(steps);
        for (uint32_t i = 0; i < steps; ++i)
        {
            out.push_back(At(i * dtS));
        }
        return out;
    }

    /// Handovers between serving satellites so far.
    uint32_t GetHandovers() const
    {
        return m_handovers;
    }

    /**
     * Device-less scene nodes: the two ground ends and every satellite that served. Call once,
     * after the pass, before recording the scene.
     */
    void AddSceneNodes()
    {
        for (const Vector& g : {m_ut, m_gw})
        {
            Ptr<Node> n = CreateObject<Node>();
            Ptr<ConstantPositionMobilityModel> m = CreateObject<ConstantPositionMobilityModel>();
            m->SetPosition(g);
            n->AggregateObject(m);
        }
        std::vector<Ptr<ntncon::Sgp4MobilityModel>> used;
        for (int32_t i : m_used)
        {
            used.push_back(m_sats[i]);
        }
        ntnobs::AddContextSatelliteNodes(used);
    }

  private:
    Vector SatEcef(int32_t idx, double t) const
    {
        Vector p;
        Vector v;
        m_sats[idx]->GetEcefStateAt(Seconds(t), p, v);
        return p;
    }

    static double ElevationDeg(const Vector& g, const Vector& sat)
    {
        const Vector lla = ntncon::NtnGeocentricMobilityAdapter::EcefToGeodetic(g);
        const double lat = lla.x * M_PI / 180.0;
        const double lon = lla.y * M_PI / 180.0;
        const Vector up(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
        const Vector d(sat.x - g.x, sat.y - g.y, sat.z - g.z);
        const double r = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        return std::asin((d.x * up.x + d.y * up.y + d.z * up.z) / r) * 180.0 / M_PI;
    }

    double PathElevationDeg(const Vector& sat) const
    {
        return std::min(ElevationDeg(m_ut, sat), ElevationDeg(m_gw, sat));
    }

    double m_minElevationDeg{25.0};
    Vector m_ut;
    Vector m_gw;
    std::vector<Ptr<ntncon::Sgp4MobilityModel>> m_sats;
    std::vector<int32_t> m_used;
    int32_t m_serving{-1};
    int32_t m_last{-1};
    uint32_t m_handovers{0};
};

/// Checks the agent's answers and summarizes the pass for the dataset record.
class PassStats
{
  public:
    /**
     * Record one exchange.
     * @param s the sample sent to the agent
     * @param answerUs the agent's answer (us)
     */
    void Add(const Sample& s, uint32_t answerUs)
    {
        ++m_exchanges;
        if (s.satIndex < 0)
        {
            ++m_outage;
            return;
        }
        if (answerUs != s.upUs + s.downUs)
        {
            ++m_wrong;
        }
        const double ms = answerUs * 1e-3;
        m_sumMs += ms;
        m_minMs = std::min(m_minMs, ms);
        m_maxMs = std::max(m_maxMs, ms);
        ++m_valid;
    }

    /**
     * Dataset metrics of the pass.
     * @param path the path that produced the samples
     * @param dtS sample spacing (s)
     * @return the metrics
     */
    std::vector<ntnobs::DatasetMetric> Metrics(const LeoPath& path, double dtS) const
    {
        auto f = [](double v) {
            std::ostringstream o;
            o << v;
            return o.str();
        };
        return {
            {"exchanges", std::to_string(m_exchanges), "measured"},
            {"agent_wrong_answers", std::to_string(m_wrong), "measured"},
            {"e2e_one_way_delay_mean_ms", f(m_valid ? m_sumMs / m_valid : 0.0), "geometry"},
            {"e2e_one_way_delay_min_ms", f(m_valid ? m_minMs : 0.0), "geometry"},
            {"e2e_one_way_delay_max_ms", f(m_valid ? m_maxMs : 0.0), "geometry"},
            {"satellite_handovers", std::to_string(path.GetHandovers()), "geometry"},
            {"outage_s", f(m_outage * dtS), "geometry"},
        };
    }

    /// Print a one-line summary.
    void Print(const LeoPath& path) const
    {
        std::cout << "LEO pass: exchanges=" << m_exchanges << " wrong=" << m_wrong
                  << " e2eDelayMs mean=" << (m_valid ? m_sumMs / m_valid : 0.0)
                  << " min=" << (m_valid ? m_minMs : 0.0) << " max=" << (m_valid ? m_maxMs : 0.0)
                  << " handovers=" << path.GetHandovers() << " outageSamples=" << m_outage
                  << std::endl;
    }

    /// Exchanges whose answer was not the sum of the two delays.
    uint32_t GetWrong() const
    {
        return m_wrong;
    }

  private:
    uint32_t m_exchanges{0};
    uint32_t m_valid{0};
    uint32_t m_outage{0};
    uint32_t m_wrong{0};
    double m_sumMs{0.0};
    double m_minMs{1e9};
    double m_maxMs{0.0};
};

} // namespace apbleo

#endif // APB_LEO_PATH_H
