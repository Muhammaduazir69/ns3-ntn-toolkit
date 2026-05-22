/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "walker-constellation.h"

#include "ns3/log.h"

#include <cmath>
#include <sstream>

namespace ns3
{
namespace ntncon
{

NS_LOG_COMPONENT_DEFINE("WalkerConstellation");

namespace
{

constexpr double kPi = M_PI;
constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kDegToRad = kPi / 180.0;

std::vector<KeplerianElements>
BuildCommon(const WalkerConfig& cfg, double raan_span_deg)
{
    std::vector<KeplerianElements> out;
    if (cfg.num_planes == 0 || cfg.total_sats == 0 ||
        cfg.total_sats % cfg.num_planes != 0)
    {
        NS_LOG_WARN("WalkerConstellation: T must be divisible by P "
                    "(T=" << cfg.total_sats << " P=" << cfg.num_planes
                          << ")");
        return out;
    }
    const uint32_t S = cfg.total_sats / cfg.num_planes;
    const double a = (kEarthRadiusM + cfg.altitude_km * 1000.0);
    out.reserve(cfg.total_sats);
    for (uint32_t p = 0; p < cfg.num_planes; ++p)
    {
        const double raan =
            static_cast<double>(p) * raan_span_deg / cfg.num_planes;
        for (uint32_t s = 0; s < S; ++s)
        {
            KeplerianElements el{};
            el.semi_major_axis_m = a;
            el.eccentricity = cfg.eccentricity;
            el.inclination_rad = cfg.inclination_deg * kDegToRad;
            el.raan_rad = raan * kDegToRad;
            el.arg_perigee_rad = cfg.arg_perigee_deg * kDegToRad;
            const double ma_deg =
                static_cast<double>(s) * 360.0 / S +
                static_cast<double>(p) * cfg.phasing_f * 360.0 /
                    cfg.total_sats;
            el.mean_anomaly_rad = std::fmod(ma_deg, 360.0) * kDegToRad;
            if (el.mean_anomaly_rad < 0.0)
                el.mean_anomaly_rad += kTwoPi;
            el.epoch_unix_s = cfg.epoch_unix_s;
            el.norad_id = 99000u + (p * S + s);
            std::ostringstream nm;
            nm << "WALKER-" << p << "-" << s;
            el.name = nm.str();
            out.push_back(el);
        }
    }
    return out;
}

} // namespace

std::vector<KeplerianElements>
WalkerConstellation::BuildDelta(const WalkerConfig& cfg)
{
    return BuildCommon(cfg, 360.0);
}

std::vector<KeplerianElements>
WalkerConstellation::BuildStar(const WalkerConfig& cfg)
{
    return BuildCommon(cfg, 180.0);
}

} // namespace ntncon
} // namespace ns3
