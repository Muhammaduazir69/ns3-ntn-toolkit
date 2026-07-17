/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef THZ_NTN_ITU_RECOMMENDATIONS_H
#define THZ_NTN_ITU_RECOMMENDATIONS_H

// ITU-R recommendation models for the NTN propagation stack (Roadmap §4.3.2).
//
// Four classes mirror the ITU-Rpy public surface so a Python user familiar
// with the ITU-Rpy API maps directly onto the C++ names:
//
//   Itu838RainModel       P.838-3   rain specific attenuation coefficients
//                                    k and alpha as functions of frequency
//                                    + polarization
//   Itu618LossModel       P.618-13  total slant-path rain attenuation
//                                    (uses Itu838 internally)
//   Itu676AbsorptionModel P.676-13  oxygen + water-vapour gaseous
//                                    attenuation (wraps the toolkit's own
//                                    Van Vleck–Weisskopf implementation
//                                    under the canonical ITU-R name)
//   Itu681LmsModel        P.681-11  Land Mobile Satellite Lutz 2-state
//                                    Markov shadowing model
//
// Each class has a native C++ backend by default; future work (the
// pybind11 path the roadmap calls out) can route through `ITU-Rpy` when
// `ENABLE_ITURPY` is set at build time.
//
// \warning Experimental: all four classes are validated against ITU-R
// reference values by the unit test suite (test/thz-ntn-test-suite.cc) but
// are not yet exercised by any example; the example channel cascade uses
// the module's own ThzNtnWeatherAttenuation / ThzNtnMolecularAbsorption /
// ThzNtnScintillation implementations.
//
// References:
//   ITU-R P.618-13  "Propagation data and prediction methods for the
//                    design of Earth-space telecommunication systems"
//   ITU-R P.676-13  "Attenuation by atmospheric gases and related effects"
//   ITU-R P.838-3   "Specific attenuation model for rain for use in
//                    prediction methods"
//   ITU-R P.681-11  "Propagation data required for the design systems in
//                    the land mobile-satellite service"
//   inigodelportillo/ITU-Rpy 0.4.0  https://github.com/inigodelportillo/ITU-Rpy

#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/random-variable-stream.h>

#include <cstdint>
#include <string>

namespace ns3
{
namespace itu
{

enum class Polarization : uint8_t
{
    horizontal = 0,
    vertical = 1,
    circular = 2, //!< the mean of horizontal and vertical
};

/**
 * \brief ITU-R P.838-3 rain k and alpha coefficients.
 *
 * Pure helper, no ns-3 attribute surface. Used internally by Itu618LossModel
 * and exposed publicly so tests can audit the canonical reference values.
 *
 * Validity range: 1..1000 GHz (extrapolation at the edges is the same
 * power-law form the ITU-R Annex 1 uses).
 */
class Itu838RainModel
{
  public:
    /// Returns (k, alpha) at the given frequency + polarization.
    /// Frequency in Hz. Reference values match ITU-R P.838-3 Annex 1
    /// Tables 1 and 2 (k_h, alpha_h, k_v, alpha_v).
    static std::pair<double, double> GetKAlpha(double freqHz,
                                                 Polarization pol);

    /// Specific rain attenuation in dB/km given (rain rate, frequency,
    /// polarization). The closed form is
    ///     gamma_r = k * R^alpha
    /// where R is in mm/h.
    static double SpecificAttenuationDbKm(double rainRate_mm_h,
                                            double freqHz,
                                            Polarization pol);
};

/**
 * \brief ITU-R P.618-13 slant-path rain attenuation.
 *
 * Computes total rain attenuation along an Earth-space slant path at a
 * given elevation and a given exceedance probability. The model uses
 * P.838 for specific attenuation, P.839 for rain height, and P.618's
 * effective path length factor for the slant geometry.
 *
 * The v2.1 baseline implements the closed-form path-length reduction
 * with rain-height climate-region table (TR 38.821 calibration scenarios:
 * tropical, midlat-summer/winter, subarctic).
 */
class Itu618LossModel : public Object
{
  public:
    enum class ClimateRegion : uint8_t
    {
        tropical = 0,        //!< rain height ~5.0 km
        midlat_summer = 1,   //!< rain height ~3.5 km
        midlat_winter = 2,   //!< rain height ~2.0 km
        subarctic = 3,       //!< rain height ~1.5 km
    };

    static TypeId GetTypeId();
    Itu618LossModel() = default;
    ~Itu618LossModel() override = default;

    void SetClimateRegion(ClimateRegion region) { m_region = region; }
    ClimateRegion GetClimateRegion() const { return m_region; }

    /// Station latitude in degrees (used by the P.618-13 step-8 vertical
    /// adjustment factor v_0.01). Default 45 deg (mid-latitude).
    void SetStationLatitudeDeg(double latDeg) { m_latDeg = latDeg; }
    double GetStationLatitudeDeg() const { return m_latDeg; }

    /// Rain height in km above MSL for the configured climate region.
    double GetRainHeightKm() const;

    /// Total slant-path rain attenuation in dB.
    ///
    /// \param freqHz         carrier frequency (Hz)
    /// \param elevationDeg   link elevation (deg)
    /// \param rainRate_mm_h  point rainfall rate at 0.01% probability
    /// \param groundAlt_km   ground station altitude (km MSL)
    /// \param pol            polarization
    double SlantPathRainAttenuationDb(double freqHz,
                                       double elevationDeg,
                                       double rainRate_mm_h,
                                       double groundAlt_km,
                                       Polarization pol) const;

    /// Horizontal path-reduction factor r_0.01 per ITU-R P.618-13
    /// §2.2.1.1 step 6.
    ///
    /// NOTE: the reduction factor is a function of the SPECIFIC ATTENUATION
    /// gamma_R (dB/km), not the rain rate. Passing the rain rate here (as an
    /// earlier revision did) is dimensionally wrong and mis-scales the
    /// factor. Compute gamma_R = k R^alpha (P.838-3) and pass it in.
    ///
    /// \param horizProjLenKm horizontal projection L_G of the slant path (km)
    /// \param gammaR_dBkm    specific rain attenuation gamma_R (dB/km)
    /// \param freqHz         carrier frequency (Hz)
    static double EffectivePathLengthFactor(double horizProjLenKm,
                                              double gammaR_dBkm,
                                              double freqHz);

  private:
    ClimateRegion m_region{ClimateRegion::midlat_summer};
    double m_latDeg{45.0};   //!< station latitude (deg), for v_0.01
};

/**
 * \brief ITU-R P.676-13 gaseous (O2 + H2O) attenuation.
 *
 * The v2.1 baseline implementation reuses the toolkit's existing Van
 * Vleck–Weisskopf model (thz-ntn-molecular-absorption.cc) under the
 * canonical ITU-R name so reviewers can grep for `Itu676` in the code.
 * Returns specific attenuation in dB/km at a single (freq, altitude).
 *
 * Future work: swap the in-process model for the ITU-Rpy backend when
 * ENABLE_ITURPY is set.
 */
class Itu676AbsorptionModel : public Object
{
  public:
    static TypeId GetTypeId();
    Itu676AbsorptionModel();
    ~Itu676AbsorptionModel() override = default;

    /// Specific gaseous attenuation in dB/km at given (freq, altitude).
    double SpecificAttenuationDbKm(double freqHz, double altKm) const;

    /// Total zenith gaseous attenuation in dB integrated 0..100 km along
    /// a slant path at `elevationDeg`. Same units as ITU-Rpy's
    /// `gaseous_attenuation_slant_path`.
    double SlantPathAttenuationDb(double freqHz, double elevationDeg) const;
};

/**
 * \brief ITU-R P.681-11 Land Mobile Satellite Lutz 2-state Markov model.
 *
 * State A (good): Rician fading, no shadowing — carrier amplitude follows
 *                 the Rice distribution with K-factor `m_rice_K_dB`.
 * State B (bad):  shadowed multipath — amplitude follows Loo
 *                 (log-normal direct + Rayleigh multipath).
 *
 * Per-environment defaults (urban / suburban / rural / open) are taken
 * from P.681-11 Table 1. The transition probabilities are tuned per
 * environment so the steady-state shadowing time-share matches the
 * recommendation.
 *
 * Stateful: maintains the current Markov state across StepDb() calls.
 * Reseed via `AssignStreams(int64_t)`.
 */
class Itu681LmsModel : public Object
{
  public:
    enum class Environment : uint8_t
    {
        urban = 0,
        suburban = 1,
        rural = 2,
        open = 3,
    };

    static TypeId GetTypeId();
    Itu681LmsModel();
    ~Itu681LmsModel() override = default;

    void SetEnvironment(Environment env);
    Environment GetEnvironment() const { return m_env; }

    /// Set the mobile terminal ground speed in m/s. The P.681 Lutz state
    /// transitions are governed by distance travelled, so the dwell time in
    /// each state scales as (characteristic distance / speed), independent
    /// of how often StepDb() is polled.
    void SetSpeedMps(double speedMps) { m_speedMps = speedMps; }
    double GetSpeedMps() const { return m_speedMps; }

    /// Advance the Lutz Markov chain over an explicit travelled distance
    /// (metres) and return a fade-depth sample (dB). Transition probability
    /// over distance dx is 1 - exp(-dx / D_state), with D_good / D_bad the
    /// mean state run-lengths (Lutz). This is the distance-parametrised core.
    double StepByDistance(double distanceM);

    /// One channel poll: advances the chain by speed * elapsed-sim-time
    /// since the previous call (so transitions are per-distance, not
    /// per-call), then returns the instantaneous fade depth in dB
    /// (positive = attenuation).
    double StepDb();

    /// Steady-state shadowing probability (fraction of time in state B).
    double GetBadStateProbability() const { return m_pBadSteady; }

    /// Current Markov state (true = state B / bad / shadowed).
    bool IsShadowed() const { return m_inBad; }

    /// Seed the internal RNGs deterministically (NS-3 stream convention).
    int64_t AssignStreams(int64_t stream);

  private:
    void ApplyEnvironment();

    Environment m_env{Environment::suburban};

    // Lutz Markov chain parameters for the current environment. Transitions
    // are governed by distance travelled: the mean run-length in each state
    // is a characteristic distance (metres), giving a steady-state bad-state
    // probability m_pBadSteady = D_bad / (D_good + D_bad).
    double m_dGood_m{30.0};   //!< mean good-state run length, metres
    double m_dBad_m{10.0};    //!< mean bad-state run length, metres
    double m_pBadSteady{0.0};

    // Mobility: ground speed (m/s) and the sim-time of the previous poll,
    // used to convert elapsed time into travelled distance.
    double m_speedMps{13.9};      //!< default ~50 km/h vehicular
    Time m_lastPollTime{Seconds(0)};

    // State-A (good) parameters.
    double m_riceK_dB{10.0};
    // State-B (bad) parameters.
    double m_looMu_dB{-10.0};      //!< log-normal mean direct, dB
    double m_looSigma_dB{3.0};     //!< log-normal std, dB
    double m_looMultipathDb{6.0};  //!< Rayleigh mean fade, dB

    bool m_inBad{false};
    Ptr<UniformRandomVariable> m_uniform;
    Ptr<NormalRandomVariable> m_normal;
    Ptr<ExponentialRandomVariable> m_exp;
};

} // namespace itu
} // namespace ns3

#endif // THZ_NTN_ITU_RECOMMENDATIONS_H
