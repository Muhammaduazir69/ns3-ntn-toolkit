/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ITU-R P.840-9 (08/2023) cloud attenuation on Earth-space paths, and the
// ITU-R P.834-9 (12/2017) conversion between free-space and apparent
// elevation, for the thz-ntn packet path.
#ifndef THZ_NTN_CLOUD_ATTENUATION_H
#define THZ_NTN_CLOUD_ATTENUATION_H

#include "ns3/object.h"
#include "ns3/random-variable-stream.h"

#include <cstdint>
#include <map>
#include <string>
#include <utility>

namespace ns3
{
namespace itu
{

/**
 * \ingroup thz-ntn
 * \brief ITU-R P.834-9 Section 1 tropospheric refraction of the elevation angle
 *        for a station at height h (0 to 3 km) seeing a space station.
 *
 * - eq. (14): tau_s(h, theta0) = 1 / [1.728 + 0.5411 theta0 + 0.03723 theta0^2
 *             + h (0.1815 + 0.06272 theta0 + 0.01138 theta0^2)
 *             + h^2 (0.01727 + 0.008288 theta0)]  (deg), theta = theta0 + tau_s;
 * - eq. (9):  tau(h, theta) = 1 / [1.314 + 0.6437 theta + 0.02869 theta^2
 *             + h (0.2305 + 0.09428 theta + 0.01096 theta^2) + 0.008583 h^2] (deg),
 *             the inverse direction, theta0 = theta - tau;
 * - eq. (10): theta_m = -acos( r/(r+h) n(0)/n(h) ), r = 6370 km, with the
 *             reference refractivity n(h) = 1 + 315e-6 exp(-0.1361 h) of eq. (8);
 * - eq. (11): the space station is visible when theta0 >= theta_m - tau(h, theta_m).
 *
 * The fits are stated for theta0 <= 10 deg; above 10 deg the correction keeps
 * falling smoothly (0.09 deg at 10 deg, 0.003 deg at the zenith) and is used as
 * is. Station heights above 3 km are evaluated at 3 km and counted by the caller.
 */
struct P834
{
    static constexpr double kMaxStationHeightKm = 3.0;
    /// Apparent (refracted) elevation (deg) from the free-space elevation (deg).
    static double ApparentElevationDeg(double freeSpaceElevDeg, double stationHeightKm);
    /// Free-space elevation (deg) from the apparent elevation (deg).
    static double FreeSpaceElevationDeg(double apparentElevDeg, double stationHeightKm);
    /// Refraction correction tau_s (deg) of eq. (14).
    static double CorrectionFromFreeSpaceDeg(double freeSpaceElevDeg, double stationHeightKm);
    /// Minimum apparent elevation theta_m (deg) of eq. (10), negative.
    static double MinimumElevationDeg(double stationHeightKm);
    /// Eq. (11): whether a space station at this free-space elevation is visible.
    static bool IsVisible(double freeSpaceElevDeg, double stationHeightKm);
};

/**
 * \brief ITU-R P.840-9 cloud liquid mass absorption coefficient K_L(f)
 *        ((dB/km)/(g/m^3), equivalently dB per kg/m^2 of integrated liquid):
 *        the double-Debye K_l of eqs. (2)-(11) at T = 273.75 K times the
 *        frequency correction of eq. (13) (A1, A2, A3, f1, f2, sigma1, sigma2).
 *        Stated valid up to 200 GHz; above, it is evaluated and flagged.
 */
double P840CloudMassAbsorptionCoefficient(double freqGHz);
/// P.840-9 double-Debye specific attenuation coefficient K_l(f, T) of liquid
/// water ((dB/km)/(g/m^3)), eqs. (2)-(11), T in kelvin.
double P840LiquidWaterKl(double freqGHz, double temperatureK);
/// Upper frequency (GHz) of the P.840-9 statement of validity.
constexpr double kP840MaxValidGHz = 200.0;

/// P.840-9 Annex 1 log-normal statistics of the integrated cloud liquid water
/// content L (kg/m^2) at a site: P(L > l) = P_L Q((ln l - m_L) / sigma_L).
struct P840LwcStatistics
{
    double latDeg{0.0};
    double lonDeg{0.0};
    double mL{0.0};     //!< mean of ln L
    double sigmaL{0.0}; //!< standard deviation of ln L
    double pLPercent{0.0}; //!< probability of cloud (%), P_L
    bool cloudy{false}; //!< false where the map has no log-normal fit (P_L <= 0.02 %)
    /// L (kg/m^2) exceeded for p percent of an average year (0 when p >= P_L).
    double LwcExceededKgM2(double pPercent) const;
};

/// Bilinear interpolation of the P.840-9 m_L, sigma_L, P_L maps at a site
/// (1 degree nodes of the 0.25 degree Recommendation maps).
P840LwcStatistics P840SiteStatistics(double latDeg, double lonDeg);

} // namespace itu

/**
 * \ingroup thz-ntn
 * \brief Cloud attenuation of an Earth-space path per ITU-R P.840-9:
 *        A_C = K_L(f) L / sin(theta), 5 deg <= theta <= 90 deg, where L is the
 *        integrated liquid water content drawn from the site's P.840-9
 *        log-normal statistics.
 *
 * The cloud state is quasi-static for a run: on the first evaluation one
 * exceedance probability p (%) is drawn uniformly on (0, 100) from an ns-3
 * random stream (Mode SampledPerRun, the default), or taken from the
 * ExceedancePercent attribute (Mode Exceedance), and every site on the path
 * then uses L(p) of its own statistics. Mode Fixed applies FixedLwc; Off
 * disables the term. Below 5 deg the path factor is held at 1/sin(5 deg) and
 * the evaluation is counted; above 200 GHz the coefficient is extrapolated and
 * the evaluation is counted. GetRunReport() gives the per-run statistics.
 */
class ThzNtnCloudAttenuation : public Object
{
  public:
    enum Mode : uint8_t
    {
        OFF = 0,
        SAMPLED_PER_RUN = 1,
        EXCEEDANCE = 2,
        FIXED = 3,
    };

    static TypeId GetTypeId();
    ThzNtnCloudAttenuation();
    ~ThzNtnCloudAttenuation() override;

    void SetMode(Mode m) { m_mode = m; }
    Mode GetMode() const { return m_mode; }
    /// Site used when the caller has no geodetic position (flat ENU frames).
    void SetDefaultSite(double latDeg, double lonDeg);
    bool HasDefaultSite() const;

    /// Cloud attenuation (dB) at a site; NaN lat/lon uses the default site and
    /// returns 0 (counted as no-site) when there is none.
    double ComputeAttenuationDb(double freqHz, double elevationDeg, double latDeg, double lonDeg);

    /// The exceedance probability (%) of this run (draws it if not yet drawn).
    double GetRunExceedancePercent();
    /// L (kg/m^2) of this run at a site.
    double GetRunLwcKgM2(double latDeg, double lonDeg);
    itu::P840LwcStatistics GetSiteStatistics(double latDeg, double lonDeg);

    double GetLastAttenuationDb() const { return m_lastDb; }
    double GetLastLwcKgM2() const { return m_lastL; }
    uint64_t GetEvaluations() const { return m_evals; }
    uint64_t GetLowElevationCount() const { return m_lowElev; }
    uint64_t GetExtrapolatedFrequencyCount() const { return m_extrapF; }
    uint64_t GetNoSiteCount() const { return m_noSite; }
    double GetMinAttenuationDb() const { return m_evals ? m_minDb : 0.0; }
    double GetMaxAttenuationDb() const { return m_evals ? m_maxDb : 0.0; }
    double GetMeanAttenuationDb() const { return m_evals ? m_sumDb / m_evals : 0.0; }
    /// One-line per-run report: mode, site statistics, p, L, K_L, A_C range,
    /// counters.
    std::string GetRunReport() const;

    int64_t AssignStreams(int64_t stream);

  private:
    void EnsureDrawn();

    Mode m_mode{SAMPLED_PER_RUN};
    double m_exceedancePercent{1.0};
    double m_fixedLwc{0.0};
    double m_siteLat;
    double m_siteLon;
    Ptr<UniformRandomVariable> m_uv;
    bool m_drawn{false};
    double m_runP{100.0};
    std::map<std::pair<int64_t, int64_t>, itu::P840LwcStatistics> m_siteCache;
    itu::P840LwcStatistics m_lastSite;
    bool m_haveSite{false};
    double m_lastFreqGHz{0.0};
    double m_lastKl{0.0};
    double m_lastDb{0.0};
    double m_lastL{0.0};
    uint64_t m_evals{0};
    uint64_t m_lowElev{0};
    uint64_t m_extrapF{0};
    uint64_t m_noSite{0};
    double m_minDb{0.0};
    double m_maxDb{0.0};
    double m_sumDb{0.0};
};

} // namespace ns3

#endif // THZ_NTN_CLOUD_ATTENUATION_H
