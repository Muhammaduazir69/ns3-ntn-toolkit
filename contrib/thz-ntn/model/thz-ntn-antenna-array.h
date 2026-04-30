/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Ultra-Massive MIMO Antenna Array Model for THz-NTN
 *
 * Integrated with SatAntennaGainPattern for satellite gain delegation
 * and MobilityModel for position-based beam direction computation.
 *
 * All gains are computed from actual array geometry and wavelength
 * (c/freq), never hardcoded. Beam directions come from actual
 * MobilityModel 3D positions.
 *
 * References:
 *   [1] I. F. Akyildiz and J. M. Jornet, "Realizing ultra-massive MIMO
 *       communication in the (0.06-10) THz band," Nano Commun. Netw., 2014.
 *   [2] C. A. Balanis, "Antenna Theory: Analysis and Design," 4th ed., 2016.
 *   [3] 3GPP TR 38.811 v15.4.0, NTN antenna modelling guidelines.
 */

#ifndef THZ_NTN_ANTENNA_ARRAY_H
#define THZ_NTN_ANTENNA_ARRAY_H

#include <ns3/mobility-model.h>
#include <ns3/object.h>
#include <ns3/ptr.h>

#include <cstdint>
#include <string>

namespace ns3
{

// Forward declarations — satellite module
class SatAntennaGainPattern;
class SatMobilityModel;

/**
 * \ingroup thz-ntn
 * \brief Array geometry type.
 */
enum class ThzArrayType : uint8_t
{
    UPA,        ///< Uniform Planar Array
    UCA,        ///< Uniform Circular Array
    CASSEGRAIN  ///< Cassegrain reflector antenna
};

/**
 * \ingroup thz-ntn
 * \brief Per-element radiation pattern model.
 */
enum class ThzElementPattern : uint8_t
{
    ISOTROPIC, ///< Omnidirectional (0 dBi)
    PATCH,     ///< Half-space patch with cos rolloff
    COSINE     ///< cos^q(theta) model
};

/**
 * \ingroup thz-ntn
 * \brief Configuration descriptor for a THz antenna array.
 *
 * All derived quantities (element spacing, wavelength, etc.) are
 * computed from the operating frequency, never hardcoded.
 */
struct ThzArrayConfig
{
    ThzArrayType    arrayType{ThzArrayType::UPA};
    uint32_t        numElementsX{32};       ///< columns (UPA)
    uint32_t        numElementsY{32};       ///< rows    (UPA)
    uint32_t        totalElements{1024};    ///< convenience: X*Y
    double          elementSpacing_m{0.0};  ///< inter-element, default lambda/2
    double          freqHz{300e9};          ///< operating frequency
    ThzElementPattern elementPattern{ThzElementPattern::COSINE};
    double          maxElementGain_dBi{5.0};///< peak element gain
};

/**
 * \ingroup thz-ntn
 * \brief Ultra-Massive MIMO antenna array for THz-NTN links.
 *
 * Provides array factor computation, element pattern evaluation, mutual
 * coupling estimation, Cassegrain reflector gain, and near-field distance
 * calculation. Integrates with SatAntennaGainPattern for satellite gain
 * delegation and MobilityModel for position-based beam direction.
 *
 * All gains are derived from actual array geometry and wavelength.
 */
class ThzNtnAntennaArray : public Object
{
  public:
    static TypeId GetTypeId();

    ThzNtnAntennaArray();
    ~ThzNtnAntennaArray() override;

    // ---- Configuration ----

    /**
     * \brief Configure the array geometry and operating frequency.
     * \param numX   number of elements along x (columns)
     * \param numY   number of elements along y (rows)
     * \param freqHz operating frequency in Hz
     * \param type   array type (UPA, UCA, CASSEGRAIN)
     */
    void Configure(uint32_t numX, uint32_t numY, double freqHz, ThzArrayType type);

    // ---- SatAntennaGainPattern integration ----

    /**
     * \brief Set the satellite antenna gain pattern for delegation.
     *
     * When set, GetGainFromPosition() delegates to SatAntennaGainPattern.
     * Otherwise, the own array factor computation is used.
     * \param gainPattern pointer to SatAntennaGainPattern
     */
    void SetSatAntennaGainPattern(Ptr<SatAntennaGainPattern> gainPattern);

    /**
     * \brief Get gain toward a target using actual MobilityModel positions.
     *
     * If a SatAntennaGainPattern is set, delegates to it.
     * Otherwise uses own array factor computation.
     * \param satellite satellite MobilityModel
     * \param target target (UE) MobilityModel
     * \return gain in dBi
     */
    double GetGainFromPosition(Ptr<MobilityModel> satellite,
                               Ptr<MobilityModel> target) const;

    // ---- MobilityModel-based array gain ----

    /**
     * \brief Compute array gain toward a target from actual 3D positions.
     *
     * Computes theta/phi from the 3D position vectors, then evaluates
     * the array factor.
     * \param target target MobilityModel
     * \param self this antenna's MobilityModel
     * \return array gain in dBi
     */
    double ComputeArrayGainToward(Ptr<MobilityModel> target,
                                  Ptr<MobilityModel> self) const;

    // ---- Gain & pattern (angle-based) ----

    /**
     * \brief Compute total array gain including element pattern.
     * \param thetaDeg      observation elevation angle (deg from boresight)
     * \param phiDeg        observation azimuth angle (deg)
     * \param steerThetaDeg steering elevation angle (deg)
     * \param steerPhiDeg   steering azimuth angle (deg)
     * \return array gain in dBi
     */
    double ComputeArrayGain_dBi(double thetaDeg,
                                double phiDeg,
                                double steerThetaDeg,
                                double steerPhiDeg) const;

    /**
     * \brief Theoretical maximum array gain (boresight, no scan loss).
     * \return gain in dBi
     */
    double ComputeMaxGain_dBi() const;

    /**
     * \brief 3 dB beamwidth computed from actual array geometry.
     *
     * theta_3dB = 0.886 * lambda / (N * d), where lambda = c/freq
     * and d = element spacing.
     * \return beamwidth in degrees
     */
    double ComputeBeamwidth3dB_deg() const;

    /**
     * \brief Wavelength from actual operating frequency.
     * \return wavelength in metres
     */
    double GetWavelength() const;

    /**
     * \brief Compute normalised array factor (linear scale, 0-1).
     */
    double ComputeArrayFactor(double thetaDeg,
                              double phiDeg,
                              double steerThetaDeg,
                              double steerPhiDeg) const;

    /**
     * \brief Evaluate element radiation pattern.
     * \param thetaDeg angle from boresight (deg)
     * \return element gain in dBi
     */
    double ComputeElementPattern_dBi(double thetaDeg) const;

    /**
     * \brief Cassegrain reflector antenna gain.
     */
    double ComputeCassegrainGain_dBi(double thetaDeg,
                                     double apertureDiameter_m,
                                     double freqHz) const;

    // ---- Impairments & geometry ----

    /**
     * \brief Mutual coupling loss estimate.
     * \return loss in dB (positive value)
     */
    double ComputeMutualCouplingLoss_dB() const;

    /**
     * \brief Fraunhofer (far-field) distance: d_F = 2*D^2/lambda
     *        where D is total array aperture.
     * \return distance in metres
     */
    double ComputeNearFieldDistance_m() const;

    /**
     * \brief Check whether a given range falls within the near-field region.
     */
    bool IsInNearField(double distance_m) const;

    /**
     * \brief Physical aperture size of the array.
     * \return largest linear dimension in metres
     */
    double ComputePhysicalSize_m() const;

    /**
     * \brief Retrieve the current array configuration.
     */
    const ThzArrayConfig& GetConfig() const;

    // Getter/setter helpers for ns-3 attribute system (member-of-member not supported)
    void SetNumElementsX(uint32_t v)
    {
        m_cfg.numElementsX = v;
        m_cfg.totalElements = m_cfg.numElementsX * m_cfg.numElementsY;
    }
    uint32_t GetNumElementsX() const { return m_cfg.numElementsX; }
    void SetNumElementsY(uint32_t v)
    {
        m_cfg.numElementsY = v;
        m_cfg.totalElements = m_cfg.numElementsX * m_cfg.numElementsY;
    }
    uint32_t GetNumElementsY() const { return m_cfg.numElementsY; }
    void SetFreqHz(double v)
    {
        m_cfg.freqHz = v;
        m_lambda = 299792458.0 / v;
        m_cfg.elementSpacing_m = m_lambda / 2.0;
    }
    double GetFreqHz() const { return m_cfg.freqHz; }
    void SetMaxElementGain(double v) { m_cfg.maxElementGain_dBi = v; }
    double GetMaxElementGain() const { return m_cfg.maxElementGain_dBi; }

  private:
    void RecalculateDerived();

    double ComputeUpaArrayFactor(double thetaRad,
                                 double phiRad,
                                 double steerThetaRad,
                                 double steerPhiRad) const;

    double ComputeUcaArrayFactor(double thetaRad,
                                 double phiRad,
                                 double steerThetaRad,
                                 double steerPhiRad) const;

    /**
     * \brief Compute (theta, phi) angles from self to target positions.
     * \param selfPos self position vector
     * \param targetPos target position vector
     * \param[out] thetaDeg elevation angle (deg)
     * \param[out] phiDeg azimuth angle (deg)
     */
    void ComputeAnglesFromPositions(const Vector& selfPos,
                                    const Vector& targetPos,
                                    double& thetaDeg,
                                    double& phiDeg) const;

    ThzArrayConfig m_cfg;           ///< current configuration
    double         m_lambda;        ///< wavelength (m) = c / freq
    double         m_k;             ///< wavenumber 2*pi/lambda
    double         m_cosExponent;   ///< q exponent for cosine element model
    double         m_apertureDiam;  ///< Cassegrain aperture diameter (m)

    // SatAntennaGainPattern delegation
    Ptr<SatAntennaGainPattern> m_satGainPattern; ///< Satellite gain pattern

    // TypeId backing fields
    std::string m_arrayTypeStr;
    std::string m_elementPatternStr;
};

} // namespace ns3

#endif // THZ_NTN_ANTENNA_ARRAY_H
