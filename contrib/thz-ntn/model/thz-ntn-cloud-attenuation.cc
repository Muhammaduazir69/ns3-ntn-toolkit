/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
#include "thz-ntn-cloud-attenuation.h"

#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/log.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnCloudAttenuation");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnCloudAttenuation);

namespace itu
{
namespace p840v9
{
extern const int16_t kMl[181][361];
extern const int16_t kSl[181][361];
extern const int16_t kPl[181][361];
} // namespace p840v9

namespace
{
constexpr double kDeg = M_PI / 180.0;

/// Inverse of the standard normal CCDF, Q^-1(q), 0 < q < 1 (Acklam's rational
/// approximation with one Halley refinement step, |error| < 1e-12).
double
QInv(double q)
{
    const double p = 1.0 - q; // Phi^-1(p) = Q^-1(1 - p)
    static const double a[] = {-3.969683028665376e+01, 2.209460984245205e+02,
                               -2.759285104469687e+02, 1.383577518672690e+02,
                               -3.066479806614716e+01, 2.506628277459239e+00};
    static const double b[] = {-5.447609879822406e+01, 1.615858368580409e+02,
                               -1.556989798598866e+02, 6.680131188771972e+01,
                               -1.328068155288572e+01};
    static const double c[] = {-7.784894002430293e-03, -3.223964580411365e-01,
                               -2.400758277161838e+00, -2.549732539343734e+00,
                               4.374664141464968e+00, 2.938163982698783e+00};
    static const double d[] = {7.784695709041462e-03, 3.224671290700398e-01,
                               2.445134137142996e+00, 3.754408661907416e+00};
    const double plow = 0.02425;
    double x;
    if (p < plow)
    {
        const double r = std::sqrt(-2.0 * std::log(p));
        x = (((((c[0] * r + c[1]) * r + c[2]) * r + c[3]) * r + c[4]) * r + c[5]) /
            ((((d[0] * r + d[1]) * r + d[2]) * r + d[3]) * r + 1.0);
    }
    else if (p <= 1.0 - plow)
    {
        const double r0 = p - 0.5;
        const double r = r0 * r0;
        x = (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * r0 /
            (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0);
    }
    else
    {
        const double r = std::sqrt(-2.0 * std::log(1.0 - p));
        x = -(((((c[0] * r + c[1]) * r + c[2]) * r + c[3]) * r + c[4]) * r + c[5]) /
            ((((d[0] * r + d[1]) * r + d[2]) * r + d[3]) * r + 1.0);
    }
    // Halley step on Phi(x) - p.
    const double e = 0.5 * std::erfc(-x / std::sqrt(2.0)) - p;
    const double u = e * std::sqrt(2.0 * M_PI) * std::exp(x * x / 2.0);
    x = x - u / (1.0 + x * u / 2.0);
    return x;
}

/// Refractivity n(h) of P.834-9 eq. (8), h in km.
double
P834RefractiveIndex(double hKm)
{
    return 1.0 + 315e-6 * std::exp(-0.1361 * hKm);
}

/// Eq. (9): tau(h, theta), theta apparent (deg).
double
P834TauApparent(double thetaDeg, double h)
{
    const double t = thetaDeg;
    return 1.0 / (1.314 + 0.6437 * t + 0.02869 * t * t +
                  h * (0.2305 + 0.09428 * t + 0.01096 * t * t) + 0.008583 * h * h);
}

double
ClampHeight(double hKm)
{
    return std::min(std::max(hKm, 0.0), P834::kMaxStationHeightKm);
}
} // namespace

double
P834::CorrectionFromFreeSpaceDeg(double theta0, double hKm)
{
    const double h = ClampHeight(hKm);
    const double t = std::max(theta0, MinimumElevationDeg(h) - P834TauApparent(MinimumElevationDeg(h), h));
    const double den = 1.728 + 0.5411 * t + 0.03723 * t * t +
                       h * (0.1815 + 0.06272 * t + 0.01138 * t * t) +
                       h * h * (0.01727 + 0.008288 * t);
    return den > 0.0 ? 1.0 / den : 0.0;
}

double
P834::ApparentElevationDeg(double theta0, double hKm)
{
    return theta0 + CorrectionFromFreeSpaceDeg(theta0, hKm);
}

double
P834::FreeSpaceElevationDeg(double theta, double hKm)
{
    const double h = ClampHeight(hKm);
    const double t = std::max(theta, MinimumElevationDeg(h));
    return theta - P834TauApparent(t, h);
}

double
P834::MinimumElevationDeg(double hKm)
{
    const double h = ClampHeight(hKm);
    const double r = 6370.0;
    const double arg = r / (r + h) * P834RefractiveIndex(0.0) / P834RefractiveIndex(h);
    return -std::acos(std::min(1.0, arg)) / kDeg;
}

bool
P834::IsVisible(double theta0, double hKm)
{
    const double h = ClampHeight(hKm);
    const double tm = MinimumElevationDeg(h);
    return theta0 >= tm - P834TauApparent(tm, h);
}

double
P840LiquidWaterKl(double f, double temperatureK)
{
    // P.840-9 eqs. (2)-(11).
    const double theta = 300.0 / temperatureK;
    const double e0 = 77.66 + 103.3 * (theta - 1.0);
    const double e1 = 0.0671 * e0;
    const double e2 = 3.52;
    const double fp = 20.20 - 146.0 * (theta - 1.0) + 316.0 * (theta - 1.0) * (theta - 1.0);
    const double fs = 39.8 * fp;
    const double ep = (e0 - e1) / (1.0 + (f / fp) * (f / fp)) +
                      (e1 - e2) / (1.0 + (f / fs) * (f / fs)) + e2;
    const double epp = f * (e0 - e1) / (fp * (1.0 + (f / fp) * (f / fp))) +
                       f * (e1 - e2) / (fs * (1.0 + (f / fs) * (f / fs)));
    const double eta = (2.0 + ep) / epp;
    return 0.819 * f / (epp * (1.0 + eta * eta));
}

double
P840CloudMassAbsorptionCoefficient(double f)
{
    // P.840-9 eq. (13): K_L(f) = K_l(f, 273.75 K) times a fitted correction.
    const double kl = P840LiquidWaterKl(f, 273.75);
    const double A1 = 0.1522, A2 = 11.51, A3 = -10.4912;
    const double f1 = -23.9589, f2 = 219.2096;
    const double s1 = 3.2991e3, s2 = 2.7595e6;
    const double corr = A1 * std::exp(-(f - f1) * (f - f1) / s1) +
                        A2 * std::exp(-(f - f2) * (f - f2) / s2) + A3;
    return std::max(0.0, kl * corr);
}

double
P840LwcStatistics::LwcExceededKgM2(double pPercent) const
{
    if (!cloudy || pPercent >= pLPercent || pPercent <= 0.0)
    {
        return 0.0;
    }
    return std::exp(mL + sigmaL * QInv(pPercent / pLPercent));
}

P840LwcStatistics
P840SiteStatistics(double latDeg, double lonDeg)
{
    P840LwcStatistics s;
    s.latDeg = latDeg;
    s.lonDeg = lonDeg;
    double lat = std::min(90.0, std::max(-90.0, latDeg));
    double lon = std::fmod(lonDeg, 360.0);
    if (lon < 0.0)
    {
        lon += 360.0;
    }
    const double fi = lat + 90.0;
    const double fj = lon;
    const int i0 = std::min(179, static_cast<int>(std::floor(fi)));
    const int j0 = std::min(359, static_cast<int>(std::floor(fj)));
    const double di = fi - i0;
    const double dj = fj - j0;
    const double w[4] = {(1 - di) * (1 - dj), (1 - di) * dj, di * (1 - dj), di * dj};
    const int ii[4] = {i0, i0, i0 + 1, i0 + 1};
    const int jj[4] = {j0, j0 + 1, j0, j0 + 1};
    double pl = 0.0;
    double ml = 0.0;
    double sl = 0.0;
    double wfit = 0.0;
    for (int k = 0; k < 4; ++k)
    {
        pl += w[k] * p840v9::kPl[ii[k]][jj[k]] / 100.0;
        if (p840v9::kMl[ii[k]][jj[k]] != -32768 && w[k] > 0.0)
        {
            ml += w[k] * p840v9::kMl[ii[k]][jj[k]] / 1000.0;
            sl += w[k] * p840v9::kSl[ii[k]][jj[k]] / 1000.0;
            wfit += w[k];
        }
    }
    s.pLPercent = pl;
    // Nodes without a log-normal fit carry P_L <= 0.02 %: no cloud there. The
    // fitted nodes are renormalized so m_L and sigma_L are not pulled to 0.
    if (wfit > 0.0 && pl > 0.02)
    {
        s.mL = ml / wfit;
        s.sigmaL = sl / wfit;
        s.cloudy = true;
    }
    return s;
}

} // namespace itu

TypeId
ThzNtnCloudAttenuation::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnCloudAttenuation")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnCloudAttenuation>()
            .AddAttribute("Mode",
                          "Cloud liquid water state of the run: Off, SampledPerRun "
                          "(one exceedance probability drawn per run from the site's "
                          "P.840-9 statistics), Exceedance (ExceedancePercent) or Fixed "
                          "(FixedLwc).",
                          EnumValue(ThzNtnCloudAttenuation::SAMPLED_PER_RUN),
                          MakeEnumAccessor<ThzNtnCloudAttenuation::Mode>(
                              &ThzNtnCloudAttenuation::m_mode),
                          MakeEnumChecker(ThzNtnCloudAttenuation::OFF,
                                          "Off",
                                          ThzNtnCloudAttenuation::SAMPLED_PER_RUN,
                                          "SampledPerRun",
                                          ThzNtnCloudAttenuation::EXCEEDANCE,
                                          "Exceedance",
                                          ThzNtnCloudAttenuation::FIXED,
                                          "Fixed"))
            .AddAttribute("ExceedancePercent",
                          "Exceedance probability p (%) of L in Exceedance mode.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&ThzNtnCloudAttenuation::m_exceedancePercent),
                          MakeDoubleChecker<double>(1e-6, 100.0))
            .AddAttribute("FixedLwc",
                          "Integrated liquid water content L (kg/m^2) in Fixed mode.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&ThzNtnCloudAttenuation::m_fixedLwc),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("SiteLatitude",
                          "Geodetic latitude (deg) of the ground site for flat-frame "
                          "links; NaN = none (ECEF links use the ground end).",
                          DoubleValue(std::numeric_limits<double>::quiet_NaN()),
                          MakeDoubleAccessor(&ThzNtnCloudAttenuation::m_siteLat),
                          MakeDoubleChecker<double>())
            .AddAttribute("SiteLongitude",
                          "Geodetic longitude (deg) of the ground site for flat-frame links.",
                          DoubleValue(std::numeric_limits<double>::quiet_NaN()),
                          MakeDoubleAccessor(&ThzNtnCloudAttenuation::m_siteLon),
                          MakeDoubleChecker<double>());
    return tid;
}

ThzNtnCloudAttenuation::ThzNtnCloudAttenuation()
    : m_siteLat(std::numeric_limits<double>::quiet_NaN()),
      m_siteLon(std::numeric_limits<double>::quiet_NaN())
{
    m_uv = CreateObject<UniformRandomVariable>();
}

ThzNtnCloudAttenuation::~ThzNtnCloudAttenuation() = default;

void
ThzNtnCloudAttenuation::SetDefaultSite(double latDeg, double lonDeg)
{
    m_siteLat = latDeg;
    m_siteLon = lonDeg;
}

bool
ThzNtnCloudAttenuation::HasDefaultSite() const
{
    return !std::isnan(m_siteLat) && !std::isnan(m_siteLon);
}

int64_t
ThzNtnCloudAttenuation::AssignStreams(int64_t stream)
{
    m_uv->SetStream(stream);
    return 1;
}

void
ThzNtnCloudAttenuation::EnsureDrawn()
{
    if (m_drawn)
    {
        return;
    }
    m_drawn = true;
    switch (m_mode)
    {
    case SAMPLED_PER_RUN: {
        // p uniform on (0, 100) %: L(p) then follows the site's long-term
        // distribution, including the 1 - P_L share of cloud-free time.
        double u = m_uv->GetValue();
        u = std::min(std::max(u, 1e-9), 1.0 - 1e-9);
        m_runP = 100.0 * u;
        break;
    }
    case EXCEEDANCE:
        m_runP = m_exceedancePercent;
        break;
    default:
        m_runP = 100.0;
        break;
    }
}

double
ThzNtnCloudAttenuation::GetRunExceedancePercent()
{
    EnsureDrawn();
    return m_runP;
}

itu::P840LwcStatistics
ThzNtnCloudAttenuation::GetSiteStatistics(double latDeg, double lonDeg)
{
    const auto key = std::make_pair(static_cast<int64_t>(std::llround(latDeg * 100.0)),
                                    static_cast<int64_t>(std::llround(lonDeg * 100.0)));
    auto it = m_siteCache.find(key);
    if (it != m_siteCache.end())
    {
        return it->second;
    }
    if (m_siteCache.size() > 4096)
    {
        m_siteCache.clear();
    }
    auto s = itu::P840SiteStatistics(latDeg, lonDeg);
    m_siteCache.emplace(key, s);
    return s;
}

double
ThzNtnCloudAttenuation::GetRunLwcKgM2(double latDeg, double lonDeg)
{
    EnsureDrawn();
    if (m_mode == OFF)
    {
        return 0.0;
    }
    if (m_mode == FIXED)
    {
        return m_fixedLwc;
    }
    if (std::isnan(latDeg) || std::isnan(lonDeg))
    {
        return 0.0;
    }
    return GetSiteStatistics(latDeg, lonDeg).LwcExceededKgM2(m_runP);
}

double
ThzNtnCloudAttenuation::ComputeAttenuationDb(double freqHz,
                                             double elevationDeg,
                                             double latDeg,
                                             double lonDeg)
{
    if (m_mode == OFF)
    {
        m_lastDb = 0.0;
        return 0.0;
    }
    EnsureDrawn();
    if (std::isnan(latDeg) || std::isnan(lonDeg))
    {
        latDeg = m_siteLat;
        lonDeg = m_siteLon;
    }
    if (m_mode != FIXED && (std::isnan(latDeg) || std::isnan(lonDeg)))
    {
        ++m_noSite;
        m_lastDb = 0.0;
        m_lastL = 0.0;
        return 0.0;
    }
    if (m_mode != FIXED)
    {
        m_lastSite = GetSiteStatistics(latDeg, lonDeg);
        m_haveSite = true;
    }
    const double fGHz = freqHz / 1e9;
    if (fGHz > itu::kP840MaxValidGHz)
    {
        ++m_extrapF;
    }
    double el = elevationDeg;
    if (el < 5.0)
    {
        // P.840-9 Section 3 states the 1/sin(theta) path for 5 to 90 deg.
        ++m_lowElev;
        el = 5.0;
    }
    if (fGHz != m_lastFreqGHz)
    {
        m_lastKl = itu::P840CloudMassAbsorptionCoefficient(fGHz);
        m_lastFreqGHz = fGHz;
    }
    m_lastL = (m_mode == FIXED) ? m_fixedLwc : m_lastSite.LwcExceededKgM2(m_runP);
    m_lastDb = m_lastKl * m_lastL / std::sin(el * M_PI / 180.0);
    if (m_evals == 0)
    {
        m_minDb = m_maxDb = m_lastDb;
    }
    m_minDb = std::min(m_minDb, m_lastDb);
    m_maxDb = std::max(m_maxDb, m_lastDb);
    m_sumDb += m_lastDb;
    ++m_evals;
    return m_lastDb;
}

std::string
ThzNtnCloudAttenuation::GetRunReport() const
{
    static const char* names[] = {"Off", "SampledPerRun", "Exceedance", "Fixed"};
    char buf[640];
    if (m_mode == OFF)
    {
        std::snprintf(buf, sizeof(buf), "cloud P.840-9: off");
        return buf;
    }
    if (m_mode == FIXED)
    {
        std::snprintf(buf,
                      sizeof(buf),
                      "cloud P.840-9: mode=Fixed L=%.4f kg/m2 K_L(%.1f GHz)=%.3f dB/(kg/m2) "
                      "A_C min/mean/max=%.3f/%.3f/%.3f dB evals=%llu low-elev=%llu f>200GHz=%llu",
                      m_fixedLwc, m_lastFreqGHz, m_lastKl, GetMinAttenuationDb(),
                      GetMeanAttenuationDb(), GetMaxAttenuationDb(),
                      static_cast<unsigned long long>(m_evals),
                      static_cast<unsigned long long>(m_lowElev),
                      static_cast<unsigned long long>(m_extrapF));
        return buf;
    }
    if (!m_haveSite && m_noSite == 0)
    {
        std::snprintf(buf,
                      sizeof(buf),
                      "cloud P.840-9: mode=%s, not evaluated (no Earth-space path with its "
                      "ground end below 10 km)",
                      names[m_mode]);
        return buf;
    }
    if (!m_haveSite)
    {
        std::snprintf(buf,
                      sizeof(buf),
                      "cloud P.840-9: mode=%s p=%.3f%% no geodetic site (flat frame and no "
                      "SiteLatitude/SiteLongitude), term not applied on %llu evaluations",
                      names[m_mode], m_runP, static_cast<unsigned long long>(m_noSite));
        return buf;
    }
    std::snprintf(buf,
                  sizeof(buf),
                  "cloud P.840-9: mode=%s site=(%.3f,%.3f) m_L=%.3f sigma_L=%.3f P_L=%.2f%% "
                  "p=%.3f%% L=%.4f kg/m2 K_L(%.1f GHz)=%.3f dB/(kg/m2) A_C min/mean/max="
                  "%.3f/%.3f/%.3f dB evals=%llu low-elev=%llu f>200GHz=%llu no-site=%llu",
                  names[m_mode], m_lastSite.latDeg, m_lastSite.lonDeg, m_lastSite.mL,
                  m_lastSite.sigmaL, m_lastSite.pLPercent, m_runP, m_lastL, m_lastFreqGHz,
                  m_lastKl, GetMinAttenuationDb(), GetMeanAttenuationDb(), GetMaxAttenuationDb(),
                  static_cast<unsigned long long>(m_evals),
                  static_cast<unsigned long long>(m_lowElev),
                  static_cast<unsigned long long>(m_extrapF),
                  static_cast<unsigned long long>(m_noSite));
    return buf;
}

} // namespace ns3
