/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN Free-Space Loss Extension for Satellite Module Integration
 *
 * Extends the satellite module's SatFreeSpaceLoss class to inject THz-specific
 * atmospheric effects (molecular absorption, weather attenuation, scintillation,
 * pointing error) into the satellite channel pipeline.  This is the primary
 * integration point: the satellite module calls SatFreeSpaceLoss::GetFsl()
 * during signal propagation, so overriding that method allows THz losses to
 * be applied transparently without modifying the satellite module itself.
 *
 * For inter-satellite links (both nodes > 100 km altitude) only pointing
 * error is applied; all atmospheric effects are skipped.
 *
 * References:
 *   [1] J. M. Jornet and I. F. Akyildiz, "Channel modeling and capacity
 *       analysis for electromagnetic wireless nanonetworks in the THz band,"
 *       IEEE Trans. Wireless Commun., vol. 10, no. 10, 2011.
 *   [2] 3GPP TR 38.811, "Study on NR to support NTN," v15.4.0, 2020.
 *   [3] ITU-R P.619-5, "Propagation data for interference evaluation"
 */

#ifndef THZ_NTN_FREE_SPACE_LOSS_H
#define THZ_NTN_FREE_SPACE_LOSS_H

#include <ns3/traced-callback.h>

#include <ns3/satellite-free-space-loss.h>

namespace ns3
{

// Forward declarations
class ThzNtnMolecularAbsorption;
class ThzNtnWeatherAttenuation;
class ThzNtnScintillation;
class ThzNtnPointingError;

/**
 * \ingroup thz-ntn
 * \brief Extended free-space loss model adding THz atmospheric effects to the
 *        satellite module signal pipeline.
 *
 * This class inherits from SatFreeSpaceLoss so it can be used as a drop-in
 * replacement wherever the satellite module computes free-space loss.  The
 * overridden GetFsl() and GetFsldB() methods first compute the standard
 * FSPL via the parent class, then add THz-specific losses from the
 * configured sub-models.
 *
 * Usage:
 * \code
 *   auto thzFsl = CreateObject<ThzNtnFreeSpaceLoss>();
 *   thzFsl->SetMolecularAbsorptionModel(absorptionModel);
 *   thzFsl->SetWeatherModel(weatherModel);
 *   thzFsl->EnableMolecularAbsorption(true);
 *   thzFsl->EnableWeatherEffects(true);
 *   // Replace default FSL in satellite helper
 *   satHelper->SetFreeSpaceLoss(thzFsl);
 * \endcode
 */
class ThzNtnFreeSpaceLoss : public SatFreeSpaceLoss
{
  public:
    /**
     * \brief Get the TypeId for this class.
     * \return the ns-3 TypeId
     */
    static TypeId GetTypeId();

    ThzNtnFreeSpaceLoss();
    ~ThzNtnFreeSpaceLoss() override;

    // ---- Overridden satellite module FSPL methods ----

    /**
     * \brief Compute free-space loss as a linear ratio, adding THz effects.
     *
     * Overrides SatFreeSpaceLoss::GetFsl().  The return value is the total
     * loss (FSPL + THz atmospheric effects) expressed as a linear power
     * ratio, matching the parent class convention.
     *
     * \param a mobility model of node a
     * \param b mobility model of node b
     * \param frequencyHz carrier frequency in Hz
     * \return total loss as a linear ratio (>= 1)
     */
    double GetFsl(Ptr<MobilityModel> a,
                  Ptr<MobilityModel> b,
                  double frequencyHz) const override;

    /**
     * \brief Compute free-space loss in dB, adding THz effects.
     *
     * Overrides SatFreeSpaceLoss::GetFsldB().
     *
     * \param a mobility model of node a
     * \param b mobility model of node b
     * \param frequencyHz carrier frequency in Hz
     * \return total loss in dB
     */
    double GetFsldB(Ptr<MobilityModel> a,
                    Ptr<MobilityModel> b,
                    double frequencyHz) const override;

    // ---- Sub-model setters ----

    /**
     * \brief Set the molecular absorption sub-model.
     * \param model pointer to ThzNtnMolecularAbsorption
     */
    void SetMolecularAbsorptionModel(Ptr<ThzNtnMolecularAbsorption> model);

    /**
     * \brief Set the weather attenuation sub-model.
     * \param model pointer to ThzNtnWeatherAttenuation
     */
    void SetWeatherModel(Ptr<ThzNtnWeatherAttenuation> model);

    /**
     * \brief Set the scintillation sub-model.
     * \param model pointer to ThzNtnScintillation
     */
    void SetScintillationModel(Ptr<ThzNtnScintillation> model);

    /**
     * \brief Set the pointing error sub-model.
     * \param model pointer to ThzNtnPointingError
     */
    void SetPointingErrorModel(Ptr<ThzNtnPointingError> model);

    // ---- Enable/disable THz-specific effects ----

    /**
     * \brief Enable or disable molecular absorption.
     * \param enable true to enable
     */
    void EnableMolecularAbsorption(bool enable);

    /**
     * \brief Enable or disable weather effects.
     * \param enable true to enable
     */
    void EnableWeatherEffects(bool enable);

    /**
     * \brief Enable or disable tropospheric scintillation.
     * \param enable true to enable
     */
    void EnableScintillation(bool enable);

    /**
     * \brief Enable or disable pointing error modelling.
     * \param enable true to enable
     */
    void EnablePointingError(bool enable);

    // ---- Cached result accessors for KPM reporting ----

    /**
     * \brief Get the base FSPL component from the last computation.
     * \return base FSPL in dB
     */
    double GetLastBaseFspl_dB() const;

    /**
     * \brief Get the molecular absorption component from the last computation.
     * \return molecular absorption loss in dB
     */
    double GetLastMolecularAbsorption_dB() const;

    /**
     * \brief Get the weather loss component from the last computation.
     * \return weather loss in dB
     */
    double GetLastWeatherLoss_dB() const;

    /**
     * \brief Get the scintillation loss component from the last computation.
     * \return scintillation loss in dB
     */
    double GetLastScintillationLoss_dB() const;

    /**
     * \brief Get the pointing error loss component from the last computation.
     * \return pointing loss in dB
     */
    double GetLastPointingLoss_dB() const;

    // ---- Trace sources ----

    /**
     * \brief Traced callback for per-component loss breakdown.
     *
     * Signature: (baseFspl_dB, absorption_dB, weather_dB,
     *             scintillation_dB, pointing_dB)
     */
    TracedCallback<double, double, double, double, double> m_thzLossBreakdown;

  private:
    /**
     * \brief Compute the elevation angle from two node positions.
     *
     * Determines which node is the ground station and which is the
     * satellite from their altitudes, then computes the elevation angle
     * at the ground node with Earth-curvature correction.
     *
     * \param ground mobility model of the ground node
     * \param sat mobility model of the satellite node
     * \return elevation angle in degrees
     */
    double ComputeElevationAngle(Ptr<MobilityModel> ground,
                                 Ptr<MobilityModel> sat) const;

    /**
     * \brief Compute the altitude of a node in km above sea level.
     *
     * Attempts to cast to SatMobilityModel and use GeoCoordinate; falls
     * back to the Z-component of the Cartesian position vector.
     *
     * \param node mobility model of the node
     * \return altitude in km
     */
    double ComputeAltitude(Ptr<MobilityModel> node) const;

    /**
     * \brief Determine whether a node is in space (altitude > 100 km).
     * \param node mobility model of the node
     * \return true if the node altitude exceeds 100 km
     */
    bool IsSpaceNode(Ptr<MobilityModel> node) const;

    // ---- Sub-models ----

    Ptr<ThzNtnMolecularAbsorption> m_absorptionModel;  ///< molecular absorption model
    Ptr<ThzNtnWeatherAttenuation> m_weatherModel;       ///< weather attenuation model
    Ptr<ThzNtnScintillation> m_scintillationModel;      ///< scintillation model
    Ptr<ThzNtnPointingError> m_pointingErrorModel;      ///< pointing error model

    // ---- Enable flags ----

    bool m_enableAbsorption;    ///< enable molecular absorption
    bool m_enableWeather;       ///< enable weather effects
    bool m_enableScintillation; ///< enable scintillation
    bool m_enablePointing;      ///< enable pointing error

    // ---- Satellite beam parameters ----

    double m_beamwidth3dB_deg;     ///< half-power beamwidth in degrees
    double m_satVelocity_km_s;     ///< satellite ground-track velocity in km/s

    // ---- Cached results from last computation (mutable for const methods) ----

    mutable double m_lastBaseFspl_dB;       ///< cached base FSPL
    mutable double m_lastAbsorption_dB;     ///< cached molecular absorption
    mutable double m_lastWeather_dB;        ///< cached weather loss
    mutable double m_lastScintillation_dB;  ///< cached scintillation loss
    mutable double m_lastPointing_dB;       ///< cached pointing loss

    // ---- Constants ----

    static constexpr double EARTH_RADIUS_KM = 6371.0;
    static constexpr double ISL_ALTITUDE_THRESHOLD_KM = 100.0;
    static constexpr double MIN_ELEVATION_DEG = 5.0;
};

} // namespace ns3

#endif /* THZ_NTN_FREE_SPACE_LOSS_H */
