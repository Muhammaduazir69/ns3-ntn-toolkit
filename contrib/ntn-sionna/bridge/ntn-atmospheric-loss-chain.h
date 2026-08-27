/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Roadmap §4.2.7)
 *
 * ntn-atmospheric-loss-chain — composite atmospheric-loss chain that layers
 * ITU-R P.676 gaseous absorption, P.618 + P.838 rain attenuation, and
 * P.681 land-mobile-satellite Markov shadowing on top of an arbitrary base
 * propagation model. Composes the four `ns3::itu::*` classes shipped by the
 * thz-ntn module so users get a single attribute surface for the cascade.
 *
 * The chain works in any frame (ECEF or local ENU); elevation is computed
 * relative to the ground-node's local up vector, choosing radial-up when
 * the ground position lies on or near the Earth's surface (|pos| close to
 * R_earth) and Cartesian-z up for local scenarios.
 */
#ifndef NTN_ATMOSPHERIC_LOSS_CHAIN_H
#define NTN_ATMOSPHERIC_LOSS_CHAIN_H

#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/vector.h"

#include <cstdint>

namespace ns3
{

namespace itu
{
class Itu618LossModel;
class Itu676AbsorptionModel;
class Itu681LmsModel;
} // namespace itu

/**
 * \ingroup ntn-sionna
 *
 * \brief Composite ITU-R atmospheric attenuation chain.
 *
 * Computes the additional slant-path attenuation that should be subtracted
 * from a base Sionna RT (or any other) path-loss prediction. All four
 * components can be enabled or disabled independently:
 *
 *   - gaseous_dB   from P.676 (oxygen + water-vapour)
 *   - rain_dB      from P.618 with P.838 specific attenuation
 *   - lms_dB       from P.681 Lutz 2-state Markov chain (stateful)
 *
 * The chain is stateful only in the LMS Markov state. Per-call attenuation
 * is purely a function of geometry and configured rain rate otherwise.
 *
 * Usage:
 * \code
 *   Ptr<NtnAtmosphericLossChain> chain = CreateObject<NtnAtmosphericLossChain>();
 *   chain->SetFrequencyHz(12.0e9);
 *   chain->SetAttribute("RainRate", DoubleValue(25.0));
 *   double extraLossDb = chain->ComputeAttenuationDb(uePos, satPos)
 *                       + chain->StepLmsDb();
 * \endcode
 */
class NtnAtmosphericLossChain : public Object
{
  public:
    /// Per-component breakdown of the last ComputeAttenuationDb() call.
    /// Useful for observability and test assertions.
    struct Components
    {
        double gaseousDb{0.0};
        double rainDb{0.0};
        double lmsDb{0.0};        //!< Last value returned by StepLmsDb()
        double elevationDeg{0.0}; //!< Geometric elevation at last query
    };

    static TypeId GetTypeId();
    NtnAtmosphericLossChain();
    ~NtnAtmosphericLossChain() override;

    /// Carrier frequency used by P.676 and P.838. Default: 12 GHz (Ku).
    void SetFrequencyHz(double freqHz);
    double GetFrequencyHz() const { return m_freqHz; }

    /// Rain rate in mm/h at 0.01% probability. 0 disables rain. Default: 0.
    void SetRainRateMmH(double rateMmH);
    double GetRainRateMmH() const { return m_rainRate; }

    /// Ground-station altitude in km MSL. Default: 0.05 km.
    void SetGroundAltKm(double altKm);
    double GetGroundAltKm() const { return m_groundAltKm; }

    /// Master switches for individual chain components.
    void SetEnableGaseous(bool enable) { m_enableGaseous = enable; }
    bool GetEnableGaseous() const { return m_enableGaseous; }
    void SetEnableRain(bool enable) { m_enableRain = enable; }
    bool GetEnableRain() const { return m_enableRain; }
    void SetEnableLms(bool enable) { m_enableLms = enable; }
    bool GetEnableLms() const { return m_enableLms; }

    /// Compute total gaseous + rain attenuation (no LMS) for the slant
    /// path from `groundPos` to `satPos`. Caches per-component values in
    /// `GetLastComponents()`. Does NOT advance the LMS Markov chain — call
    /// StepLmsDb() separately. Returns 0.0 when no enabled component
    /// contributes.
    double ComputeAttenuationDb(const Vector& groundPos,
                                const Vector& satPos) const;

    /// Advance the LMS Markov chain by one step and return the fade
    /// depth in dB. Returns 0.0 when LMS is disabled or no model exists.
    /// The result is also stored in `GetLastComponents().lmsDb`.
    double StepLmsDb() const;

    /// Geometric elevation angle in degrees of (sat - ground) measured
    /// from the local up vector at `groundPos`. If |groundPos| is within
    /// 10% of the Earth's mean radius the up direction is taken as the
    /// radial unit (ECEF); otherwise +z is assumed (local ENU).
    static double ComputeElevationDeg(const Vector& groundPos,
                                      const Vector& satPos);

    /// Slant-path length in km (3-D distance between the two points).
    static double SlantPathLengthKm(const Vector& groundPos,
                                    const Vector& satPos);

    Components GetLastComponents() const { return m_last; }

    /// Seed the LMS Markov chain RNGs. Returns number of streams consumed.
    int64_t AssignStreams(int64_t stream);

    /// Configure climate region for rain height (forwarded to Itu618).
    /// Encoded as int because the enum is in a separate namespace; valid
    /// values: 0 tropical, 1 midlat_summer, 2 midlat_winter, 3 subarctic.
    void SetClimateRegionInt(int region);
    int GetClimateRegionInt() const { return m_climateRegionInt; }

    /// Polarization (0 horizontal, 1 vertical, 2 circular).
    void SetPolarizationInt(int pol);
    int GetPolarizationInt() const { return m_polInt; }

    /// LMS environment (0 urban, 1 suburban, 2 rural, 3 open).
    void SetLmsEnvironmentInt(int env);
    int GetLmsEnvironmentInt() const { return m_lmsEnvInt; }

    /**
     * \brief THZ-08: terminal speed for the ITU-R P.681-11 LMS model, m/s.
     *
     * Itu681LmsModel::SetSpeedMps existed and was called from exactly one place
     * in the tree: a unit test. This chain, the model's only production
     * consumer, set the ENVIRONMENT and nothing else, and exposed no speed
     * attribute, so every shipped run used the 13.9 m/s default (50 km/h)
     * regardless of what the scenario was actually modelling: a stationary
     * VSAT, a walking handheld and a 900 km/h aircraft all got the same
     * fade-duration statistics.
     */
    void SetTerminalSpeedMps(double v);
    double GetTerminalSpeedMps() const { return m_lmsSpeedMps; }

  private:
    /// Lazily create the underlying ITU-R model objects on first use so
    /// the chain can be constructed cheaply and parameterised before any
    /// computation.
    void EnsureModels() const;
    void ApplyConfig() const;

    double m_freqHz{12.0e9};
    double m_rainRate{0.0};
    double m_groundAltKm{0.05};
    bool m_enableGaseous{true};
    bool m_enableRain{true};
    bool m_enableLms{false};

    int m_climateRegionInt{1}; // midlat_summer
    int m_polInt{1};            // vertical
    int m_lmsEnvInt{1};         // suburban
    /// THZ-08: 13.9 m/s is the P.681 default and stays the default here, so no
    /// existing result moves; what changes is that a scenario can now set it.
    double m_lmsSpeedMps{13.9};

    mutable Ptr<itu::Itu618LossModel> m_rain;
    mutable Ptr<itu::Itu676AbsorptionModel> m_gas;
    mutable Ptr<itu::Itu681LmsModel> m_lms;

    mutable Components m_last{};
    mutable bool m_modelsBuilt{false};
};

} // namespace ns3

#endif // NTN_ATMOSPHERIC_LOSS_CHAIN_H
