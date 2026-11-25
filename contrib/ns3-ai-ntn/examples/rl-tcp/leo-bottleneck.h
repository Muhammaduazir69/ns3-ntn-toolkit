/*
 * Copyright (c) 2026 Muhammad Uzair
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * LEO bent-pipe bottleneck for the rl-tcp examples (use-msg and use-gym).
 *
 * The dumbbell bottleneck R0--R1 is read as a transparent (bent-pipe) satellite hop: R0 is
 * a user terminal and R1 a feeder gateway, both at real WGS-84 coordinates, and the
 * one-way delay of the link is the up-plus-down slant range of the serving satellite
 * divided by c. The satellites belong to a Walker-delta shell (Starlink-like defaults,
 * 550 km, 53 deg, 72 x 22) built by WalkerConstellation and propagated with Vallado SGP4
 * through Sgp4MobilityModel, or to a real TLE file. The serving satellite is the one
 * that maximizes the lower of its two elevation angles; it is kept until either angle
 * drops below the minimum elevation (sticky selection), then the best visible satellite
 * takes over. The serving schedule is computed at 1 s steps before the run from the
 * propagator (Sgp4MobilityModel::GetEcefStateAt), and during the run the Delay attribute
 * of the PointToPointChannel is refreshed every UpdateInterval from the exact SGP4 state
 * of the serving satellite.
 *
 * Simplifications, stated plainly: no handover interruption time, no tropospheric or
 * ionospheric excess delay, no on-board processing delay, and the data rate of the
 * bottleneck stays the configured constant (the link budget is not modeled here).
 */

#ifndef RL_TCP_LEO_BOTTLENECK_H
#define RL_TCP_LEO_BOTTLENECK_H

#include "ns3/abort.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/ntn-dataset-writer.h"
#include "ns3/ntn-geocentric-mobility-adapter.h"
#include "ns3/nstime.h"
#include "ns3/orbital-elements.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace rltcpleo
{

using namespace ns3;

/// Configuration of the satellite path (all CLI-exposed by the examples).
struct LeoPathConfig
{
    double utLatDeg{43.6045}; //!< user terminal, Toulouse (FR)
    double utLonDeg{1.4440};
    double utAltM{150.0};
    double gwLatDeg{40.4168}; //!< feeder gateway, Madrid (ES), about 570 km away
    double gwLonDeg{-3.7038};
    double gwAltM{650.0};
    double altKm{550.0};          //!< Walker shell altitude
    double inclinationDeg{53.0};  //!< Walker shell inclination
    uint32_t numPlanes{72};       //!< Walker planes
    uint32_t satsPerPlane{22};    //!< satellites per plane
    double minElevationDeg{25.0}; //!< minimum elevation at both ends
    std::string tleFile;          //!< optional real TLE set; replaces the Walker shell
    double epochUnix{1735689600.0}; //!< Walker epoch, 2025-01-01T00:00:00Z
    Time updateInterval{MilliSeconds(100)}; //!< Delay attribute refresh period
};

/// Bent-pipe satellite path that drives the Delay attribute of one PointToPointChannel.
class LeoBentPipe
{
  public:
    /**
     * Build the constellation, place the two ground ends, compute the serving schedule
     * for [0, duration] and start the delay updates. Call before Simulator::Run() and
     * after the Internet stack is installed (it creates device-less satellite nodes).
     * Returns the initial one-way delay, or a negative value if no satellite is visible.
     */
    double Install(const LeoPathConfig& cfg,
                   Ptr<Node> ut,
                   Ptr<Node> gw,
                   Ptr<PointToPointChannel> channel,
                   double duration)
    {
        m_cfg = cfg;
        m_channel = channel;
        m_utEcef = ntncon::NtnGeocentricMobilityAdapter::GeodeticToEcef(
            Vector(cfg.utLatDeg, cfg.utLonDeg, cfg.utAltM));
        m_gwEcef = ntncon::NtnGeocentricMobilityAdapter::GeodeticToEcef(
            Vector(cfg.gwLatDeg, cfg.gwLonDeg, cfg.gwAltM));
        PlaceGround(ut, m_utEcef);
        PlaceGround(gw, m_gwEcef);
        BuildSatellites();
        if (m_sats.empty())
        {
            return -1.0;
        }
        BuildSchedule(duration);

        // Only the satellites that serve during the run go into the scene, as device-less
        // context nodes, so the network is unchanged by the visualization.
        std::vector<Ptr<ntncon::Sgp4MobilityModel>> used;
        for (int32_t idx : m_schedule)
        {
            if (idx >= 0 && std::find(used.begin(), used.end(), m_sats[idx]) == used.end())
            {
                used.push_back(m_sats[idx]);
            }
        }
        ntnobs::AddContextSatelliteNodes(used);

        const double d0 = Update();
        return m_outage0 ? -1.0 : d0;
    }

    /// Propagator actually used ("sgp4-vallado" or "kepler-j2").
    std::string GetPropagatorName() const
    {
        return m_sats.empty() ? std::string("none") : m_sats.front()->GetPropagatorName();
    }

    uint32_t GetNumSatellites() const
    {
        return m_sats.size();
    }

    uint32_t GetHandovers() const
    {
        return m_handovers;
    }

    double GetOutageSeconds() const
    {
        return m_outageS;
    }

    double GetMinDelayMs() const
    {
        return m_minDelayS * 1e3;
    }

    double GetMaxDelayMs() const
    {
        return m_maxDelayS * 1e3;
    }

    /// Time-weighted mean one-way delay of the bottleneck (ms).
    double GetMeanDelayMs() const
    {
        return m_weightS > 0 ? 1e3 * m_delayTimeS / m_weightS : 0.0;
    }

  private:
    static void PlaceGround(Ptr<Node> n, const Vector& ecef)
    {
        Ptr<ConstantPositionMobilityModel> m = n->GetObject<ConstantPositionMobilityModel>();
        if (!m)
        {
            m = CreateObject<ConstantPositionMobilityModel>();
            n->AggregateObject(m);
        }
        m->SetPosition(ecef); // ECEF metres, as the scene recorder expects
    }

    void BuildSatellites()
    {
        if (!m_cfg.tleFile.empty())
        {
            std::ifstream in(m_cfg.tleFile);
            NS_ABORT_MSG_IF(!in, "cannot open TLE file " << m_cfg.tleFile);
            std::stringstream ss;
            ss << in.rdbuf();
            std::vector<ntncon::TleRecord> recs;
            ntncon::ParseTleStream(ss.str(), recs);
            double epoch = 0;
            for (const auto& r : recs)
            {
                auto m = CreateObject<ntncon::Sgp4MobilityModel>();
                if (!m->SetTle(r))
                {
                    continue;
                }
                m->SetUseVallado(true);
                epoch = std::max(epoch, m->GetElementEpochUnix());
                m_sats.push_back(m);
            }
            // One common clock: every satellite starts at the newest element epoch.
            for (auto& m : m_sats)
            {
                m->SetSimulationEpochUnix(epoch);
            }
            return;
        }
        ntncon::WalkerConfig w;
        w.num_planes = m_cfg.numPlanes;
        w.total_sats = m_cfg.numPlanes * m_cfg.satsPerPlane;
        w.altitude_km = m_cfg.altKm;
        w.inclination_deg = m_cfg.inclinationDeg;
        w.epoch_unix_s = m_cfg.epochUnix;
        for (const auto& el : ntncon::WalkerConstellation::BuildDelta(w))
        {
            auto m = CreateObject<ntncon::Sgp4MobilityModel>();
            m->SetElementsSgp4(el);
            m_sats.push_back(m);
        }
    }

    /// Elevation (deg) of `sat` seen from ground point `g` (ECEF, ellipsoid normal).
    static double ElevationDeg(const Vector& g, const Vector& sat)
    {
        const Vector lla = ntncon::NtnGeocentricMobilityAdapter::EcefToGeodetic(g);
        const double lat = lla.x * M_PI / 180.0;
        const double lon = lla.y * M_PI / 180.0;
        const Vector up(std::cos(lat) * std::cos(lon),
                        std::cos(lat) * std::sin(lon),
                        std::sin(lat));
        const Vector d(sat.x - g.x, sat.y - g.y, sat.z - g.z);
        const double r = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        return std::asin((d.x * up.x + d.y * up.y + d.z * up.z) / r) * 180.0 / M_PI;
    }

    double PathElevationDeg(const Vector& sat) const
    {
        return std::min(ElevationDeg(m_utEcef, sat), ElevationDeg(m_gwEcef, sat));
    }

    Vector SatEcefAt(int32_t idx, double t) const
    {
        Vector p;
        Vector v;
        m_sats[idx]->GetEcefStateAt(Seconds(t) - Simulator::Now(), p, v);
        return p;
    }

    void BuildSchedule(double duration)
    {
        const uint32_t steps = static_cast<uint32_t>(std::ceil(duration)) + 1;
        m_schedule.assign(steps, -1);
        int32_t serving = -1;
        for (uint32_t s = 0; s < steps; ++s)
        {
            const double t = s;
            if (serving >= 0 && PathElevationDeg(SatEcefAt(serving, t)) >= m_cfg.minElevationDeg)
            {
                m_schedule[s] = serving;
                continue;
            }
            int32_t best = -1;
            double bestEl = m_cfg.minElevationDeg;
            for (uint32_t i = 0; i < m_sats.size(); ++i)
            {
                const double el = PathElevationDeg(SatEcefAt(i, t));
                if (el >= bestEl)
                {
                    bestEl = el;
                    best = i;
                }
            }
            serving = best;
            m_schedule[s] = serving;
        }
    }

    /// Refresh the channel delay from the serving satellite; returns the one-way delay (s).
    double Update()
    {
        const double now = Simulator::Now().GetSeconds();
        const uint32_t slot =
            std::min<uint32_t>(static_cast<uint32_t>(now), m_schedule.size() - 1);
        const int32_t idx = m_schedule[slot];
        const double dt = m_cfg.updateInterval.GetSeconds();
        double delayS = m_lastDelayS;
        if (idx < 0)
        {
            // No satellite above the minimum elevation at both ends: keep the last delay
            // and count the time as outage (the link itself is not cut).
            m_outageS += dt;
            if (now == 0.0)
            {
                m_outage0 = true;
            }
        }
        else
        {
            Vector p;
            Vector v;
            m_sats[idx]->GetEcefStateAt(Seconds(0), p, v);
            const double c = 299792458.0;
            delayS = (CalculateDistance(m_utEcef, p) + CalculateDistance(p, m_gwEcef)) / c;
            if (m_lastIdx >= 0 && idx != m_lastIdx)
            {
                ++m_handovers;
            }
            m_lastIdx = idx;
            m_minDelayS = std::min(m_minDelayS, delayS);
            m_maxDelayS = std::max(m_maxDelayS, delayS);
            m_delayTimeS += delayS * dt;
            m_weightS += dt;
            m_channel->SetAttribute("Delay", TimeValue(Seconds(delayS)));
        }
        m_lastDelayS = delayS;
        Simulator::Schedule(m_cfg.updateInterval, &LeoBentPipe::UpdateVoid, this);
        return delayS;
    }

    void UpdateVoid()
    {
        Update();
    }

    LeoPathConfig m_cfg;
    Ptr<PointToPointChannel> m_channel;
    Vector m_utEcef;
    Vector m_gwEcef;
    std::vector<Ptr<ntncon::Sgp4MobilityModel>> m_sats;
    std::vector<int32_t> m_schedule;
    int32_t m_lastIdx{-1};
    uint32_t m_handovers{0};
    double m_outageS{0.0};
    bool m_outage0{false};
    double m_lastDelayS{0.0};
    double m_minDelayS{std::numeric_limits<double>::max()};
    double m_maxDelayS{0.0};
    double m_delayTimeS{0.0};
    double m_weightS{0.0};
};

} // namespace rltcpleo

#endif // RL_TCP_LEO_BOTTLENECK_H
