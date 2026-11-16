/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "thz-ntn-rain-cell-field.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnRainCellField");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnRainCellField);

namespace
{
constexpr double kRe = 6371.0; // mean Earth radius (km), plane projection of ECEF links
// P.618-13 eq. (2)/(3) use R_e = 8500 km (effective radius).
constexpr double kReEff = 8500.0;
} // namespace

TypeId
ThzNtnRainCellField::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnRainCellField")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnRainCellField>()
            .AddAttribute("RainHeightKm",
                          "Rain height h_R above MSL (km), ITU-R P.839 / P.618-13 step 1.",
                          DoubleValue(3.5),
                          MakeDoubleAccessor(&ThzNtnRainCellField::m_rainHeightKm),
                          MakeDoubleChecker<double>(0.0, 10.0))
            .AddAttribute("MinRate",
                          "Rain rate below which the field is dry (mm/h).",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&ThzNtnRainCellField::m_minRate),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("MaxStep",
                          "Largest path-integration step (m).",
                          DoubleValue(50.0),
                          MakeDoubleAccessor(&ThzNtnRainCellField::m_maxStepM),
                          MakeDoubleChecker<double>(1.0));
    return tid;
}

ThzNtnRainCellField::ThzNtnRainCellField() = default;
ThzNtnRainCellField::~ThzNtnRainCellField() = default;

uint32_t
ThzNtnRainCellField::AddCell(const Cell& c)
{
    m_cells.push_back(c);
    return static_cast<uint32_t>(m_cells.size() - 1);
}

void
ThzNtnRainCellField::SetReference(double latDeg, double lonDeg)
{
    m_refLatDeg = latDeg;
    m_refLonDeg = lonDeg;
}

void
ThzNtnRainCellField::SetRainHeightFromClimate(itu::Itu618LossModel::ClimateRegion region)
{
    Ptr<itu::Itu618LossModel> m = CreateObject<itu::Itu618LossModel>();
    m->SetClimateRegion(region);
    m_rainHeightKm = m->GetRainHeightKm();
}

double
ThzNtnRainCellField::RainRateAt(double x, double y, Time t) const
{
    double r = 0.0;
    for (const auto& c : m_cells)
    {
        if (t < c.start || t > c.stop)
        {
            continue;
        }
        const double dt = (t - c.start).GetSeconds();
        const double cx = c.x0 + c.vx * dt;
        const double cy = c.y0 + c.vy * dt;
        const double d = std::hypot(x - cx, y - cy);
        r = std::max(r, c.peakRate * std::exp(-d / std::max(c.r0, 1.0)));
    }
    return (r >= m_minRate) ? r : 0.0;
}

void
ThzNtnRainCellField::LowerEndPlane(const ThzNtnLinkGeometry& g, double& x, double& y) const
{
    if (!g.ecef)
    {
        x = g.lowerEnu.x;
        y = g.lowerEnu.y;
        return;
    }
    const double lat0 = m_refLatDeg * M_PI / 180.0;
    x = (g.lowerLonDeg - m_refLonDeg) * M_PI / 180.0 * kRe * 1000.0 * std::cos(lat0);
    y = (g.lowerLatDeg - m_refLatDeg) * M_PI / 180.0 * kRe * 1000.0;
}

double
ThzNtnRainCellField::SlantLengthKm(double elevationDeg, double hsKm, double hrKm)
{
    const double dh = hrKm - hsKm;
    if (dh <= 0.0)
    {
        return 0.0;
    }
    const double th = std::max(elevationDeg, 0.0) * M_PI / 180.0;
    const double s = std::sin(th);
    if (elevationDeg >= 5.0)
    {
        return dh / s; // P.618-13 eq. (2)
    }
    // P.618-13 eq. (3), Earth curvature with the 8500 km effective radius.
    return 2.0 * dh / (std::sqrt(s * s + 2.0 * dh / kReEff) + s);
}

std::vector<double>
ThzNtnRainCellField::SamplePath(const ThzNtnLinkGeometry& g, Time t, double& ds) const
{
    std::vector<double> rates;
    ds = 0.0;
    if (m_cells.empty() || g.elevationDeg <= 0.0)
    {
        return rates;
    }
    const double ls = SlantLengthKm(g.elevationDeg, g.lowerAltKm, m_rainHeightKm);
    if (ls <= 0.0)
    {
        return rates;
    }
    const double lsM = ls * 1000.0;
    const uint32_t n = std::max<uint32_t>(20, static_cast<uint32_t>(std::ceil(lsM / m_maxStepM)));
    const double stepM = lsM / n;
    ds = stepM / 1000.0;
    double x0;
    double y0;
    LowerEndPlane(g, x0, y0);
    rates.reserve(n);
    bool any = false;
    for (uint32_t i = 0; i < n; ++i)
    {
        const double s = (i + 0.5) * stepM;
        const double r = RainRateAt(x0 + g.dirEnu.x * s, y0 + g.dirEnu.y * s, t);
        any = any || (r > 0.0);
        rates.push_back(r);
    }
    if (!any)
    {
        rates.clear();
    }
    return rates;
}

double
ThzNtnRainCellField::IntegrateDb(const std::vector<double>& rates, double dsKm, double freqHz) const
{
    if (rates.empty())
    {
        return 0.0;
    }
    const auto [k, alpha] = itu::Itu838RainModel::GetKAlpha(freqHz, m_pol);
    double a = 0.0;
    for (double r : rates)
    {
        if (r > 0.0)
        {
            a += k * std::pow(r, alpha) * dsKm;
        }
    }
    return a;
}

double
ThzNtnRainCellField::PathAttenuationDb(const ThzNtnLinkGeometry& g, double freqHz) const
{
    double ds = 0.0;
    const auto rates = SamplePath(g, Simulator::Now(), ds);
    return IntegrateDb(rates, ds, freqHz);
}

} // namespace ns3
