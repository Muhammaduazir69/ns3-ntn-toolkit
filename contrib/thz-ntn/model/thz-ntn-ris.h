/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN Reconfigurable Intelligent Surface (RIS) Model
 *
 * Parametric RIS model for THz-band non-terrestrial networks supporting
 * space-borne, aerial (HAP), and ground deployments.  Integrates with
 * MobilityModel for actual 3D positions and SatFreeSpaceLoss for
 * cascaded path loss computation.
 *
 * Features:
 *   - Gain computed from actual wavelength (c/freq), not hardcoded
 *   - RIS gain toward specific targets using MobilityModel positions
 *   - Cascaded path loss via SatFreeSpaceLoss
 *   - Phase profile from actual 3D geometry
 *   - Deployment modes: SPACE_BORNE, AERIAL, GROUND
 *   - Phase shift: discrete (1-4 bit) or continuous
 *
 * References:
 *   [1] E. Basar et al., "Wireless communications through reconfigurable
 *       intelligent surfaces," IEEE Access, 2019.
 *   [2] Q. Wu and R. Zhang, "Intelligent reflecting surface enhanced
 *       wireless network via joint active and passive beamforming,"
 *       IEEE Trans. Wireless Commun., 2019.
 *   [3] C. Huang et al., "Reconfigurable intelligent surfaces for energy
 *       efficiency in wireless communication," IEEE TWC, 2019.
 */

#ifndef THZ_NTN_RIS_H
#define THZ_NTN_RIS_H

#include <ns3/mobility-model.h>
#include <ns3/object.h>
#include <ns3/ptr.h>

#include <cstdint>
#include <vector>

namespace ns3
{

// Forward declarations
class SatFreeSpaceLoss;

/**
 * \brief RIS deployment scenario.
 */
enum class RisDeployment : uint8_t
{
    SPACE_BORNE = 0, ///< Mounted on satellite
    AERIAL,          ///< Mounted on HAP / UAV
    GROUND           ///< Terrestrial deployment
};

/**
 * \brief Phase quantization model for RIS elements.
 */
enum class PhaseModel : uint8_t
{
    CONTINUOUS = 0,    ///< Ideal continuous phase control
    DISCRETE_1BIT,     ///< 1-bit phase (0 or pi)
    DISCRETE_2BIT,     ///< 2-bit phase (4 levels)
    DISCRETE_3BIT,     ///< 3-bit phase (8 levels)
    DISCRETE_4BIT      ///< 4-bit phase (16 levels)
};

/**
 * \ingroup thz-ntn
 * \brief THz-band Reconfigurable Intelligent Surface model.
 *
 * Models a planar RIS panel composed of N = Nx * Ny sub-wavelength
 * reflecting elements.  All element spacing and wavelength are derived
 * from the configured frequency.  Supports MobilityModel-based gain
 * computation and cascaded path loss via SatFreeSpaceLoss.
 */
class ThzNtnRis : public Object
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    ThzNtnRis();
    ~ThzNtnRis() override;

    // ---- Sub-model setters ----

    /**
     * \brief Set the free-space loss model for cascaded path loss.
     * \param fsl pointer to SatFreeSpaceLoss (or ThzNtnFreeSpaceLoss)
     */
    void SetFreeSpaceLossModel(Ptr<SatFreeSpaceLoss> fsl);

    // ---- Configuration ----

    /**
     * \brief Configure the RIS panel geometry and deployment.
     * \param numX number of elements along X axis
     * \param numY number of elements along Y axis
     * \param freqHz operating frequency in Hz
     * \param deploy deployment scenario
     */
    void Configure(uint32_t numX, uint32_t numY, double freqHz, RisDeployment deploy);

    // ---- MobilityModel-based computations ----

    /**
     * \brief Compute RIS gain for a specific TX->RIS->RX link using actual positions.
     *
     * Computes incidence and reflection angles from actual 3D MobilityModel
     * positions, then evaluates the array factor with the current phase profile.
     *
     * \param tx transmitter MobilityModel
     * \param ris RIS panel MobilityModel
     * \param rx receiver MobilityModel
     * \return array gain in dB
     */
    double ComputeRisGainForLink(Ptr<MobilityModel> tx,
                                 Ptr<MobilityModel> ris,
                                 Ptr<MobilityModel> rx) const;

    /**
     * \brief Compute cascaded path loss TX->RIS->RX using actual SatFreeSpaceLoss.
     *
     * Uses the SatFreeSpaceLoss model for each segment (TX->RIS and RIS->RX),
     * then subtracts the RIS gain.
     *
     * \param tx transmitter MobilityModel
     * \param ris RIS panel MobilityModel
     * \param rx receiver MobilityModel
     * \return cascaded path loss in dB
     */
    double ComputeCascadedPathLoss_dB(Ptr<MobilityModel> tx,
                                      Ptr<MobilityModel> ris,
                                      Ptr<MobilityModel> rx) const;

    /**
     * \brief Compute and apply optimal phase profile for actual TX->RIS->RX geometry.
     *
     * Computes incidence and reflection angles from actual 3D positions,
     * then sets the phase profile for coherent reflection.
     *
     * \param tx transmitter MobilityModel
     * \param ris RIS panel MobilityModel
     * \param rx receiver MobilityModel
     */
    void ComputeOptimalPhases(Ptr<MobilityModel> tx,
                              Ptr<MobilityModel> ris,
                              Ptr<MobilityModel> rx);

    // ---- Angle-based computations (fallback / advanced use) ----

    /**
     * \brief Compute RIS array gain for given incidence and reflection angles.
     * \param thetaIn_deg incidence elevation angle (deg)
     * \param phiIn_deg incidence azimuth angle (deg)
     * \param thetaOut_deg reflection elevation angle (deg)
     * \param phiOut_deg reflection azimuth angle (deg)
     * \return array gain in dB
     */
    double ComputeRisGain_dB(double thetaIn_deg,
                             double phiIn_deg,
                             double thetaOut_deg,
                             double phiOut_deg) const;

    /**
     * \brief Compute maximum achievable RIS gain (perfect alignment).
     * \return maximum gain in dB
     */
    double ComputeMaxGain_dB() const;

    /**
     * \brief Set per-element phase profile manually.
     * \param phases_rad vector of phase shifts in radians (size = Nx*Ny)
     */
    void SetPhaseProfile(const std::vector<double>& phases_rad);

    /**
     * \brief Compute and apply optimal phase profile for given angles.
     * \param thetaIn_deg incidence elevation angle (deg)
     * \param phiIn_deg incidence azimuth angle (deg)
     * \param thetaOut_deg desired reflection elevation angle (deg)
     * \param phiOut_deg desired reflection azimuth angle (deg)
     */
    void ComputeOptimalPhases(double thetaIn_deg,
                              double phiIn_deg,
                              double thetaOut_deg,
                              double phiOut_deg);

    /**
     * \brief Compute quantization loss for current phase model.
     * \return quantization loss in dB
     */
    double ComputeQuantizationLoss_dB() const;

    /**
     * \brief Compute SNR gain from RIS.
     * \param perfectCsi true if perfect CSI is available (N^2 scaling)
     * \return SNR gain in dB
     */
    double ComputeSnrGain_dB(bool perfectCsi) const;

    /**
     * \brief Determine whether distance falls within RIS near field.
     * \param distance_m distance in metres
     * \return true if in the near field (Fresnel region)
     */
    bool IsInNearField(double distance_m) const;

    /**
     * \brief Compute coverage angle based on deployment.
     * \return coverage angle in degrees
     */
    double ComputeAerialRisCoverage_deg() const;

    /**
     * \brief Get total element count (Nx * Ny).
     * \return total number of RIS elements
     */
    uint32_t GetTotalElements() const;

    /**
     * \brief Get wavelength from configured frequency.
     * \return wavelength in metres
     */
    double GetWavelength() const;

    /**
     * \brief Get panel dimensions (Nx, Ny) for codebook generation.
     * \param[out] numX elements along X axis
     * \param[out] numY elements along Y axis
     */
    void GetPanelDimensions(uint32_t& numX, uint32_t& numY) const;

  protected:
    void DoDispose() override;

  private:
    uint32_t m_numX;                   ///< Elements along X axis
    uint32_t m_numY;                   ///< Elements along Y axis
    double m_frequency;                ///< Operating frequency (Hz)
    RisDeployment m_deployment;        ///< Deployment type
    PhaseModel m_phaseModel;           ///< Phase quantization model
    double m_reflectionEfficiency;     ///< Reflection efficiency (0-1)
    std::string m_deploymentStr;       ///< Deployment string for TypeId
    std::string m_phaseModelStr;       ///< Phase model string for TypeId
    double m_elementGain_dBi;          ///< Individual element gain (dBi)

    std::vector<double> m_phases_rad;  ///< Per-element phase profile

    Ptr<SatFreeSpaceLoss> m_fsl;       ///< Free-space loss model

    /// Physical constants
    static constexpr double C_LIGHT = 299792458.0;
    static constexpr double PI_VAL = 3.14159265358979323846;
    static constexpr double DEG2RAD = PI_VAL / 180.0;

    /**
     * \brief Compute element spacing (half wavelength).
     * \return spacing in metres
     */
    double GetElementSpacing_m() const;

    /**
     * \brief Compute panel physical aperture.
     * \return aperture side length in metres
     */
    double GetPanelSize_m() const;

    /**
     * \brief Get the number of quantization levels from phase model.
     * \return number of discrete levels (0 if continuous)
     */
    uint32_t GetQuantizationLevels() const;

    /**
     * \brief Quantize a phase to the nearest discrete level.
     * \param phase_rad continuous phase in radians
     * \return quantized phase in radians
     */
    double QuantizePhase(double phase_rad) const;

    /**
     * \brief Compute (theta, phi) angles from RIS position to a target.
     * \param risPos RIS panel position
     * \param targetPos target position
     * \param[out] theta_deg elevation angle (deg)
     * \param[out] phi_deg azimuth angle (deg)
     */
    void ComputeAnglesFromPositions(const Vector& risPos,
                                    const Vector& targetPos,
                                    double& theta_deg,
                                    double& phi_deg) const;

    /**
     * \brief Parse deployment string to enum.
     */
    static RisDeployment ParseDeployment(const std::string& s);

    /**
     * \brief Parse phase model string to enum.
     */
    static PhaseModel ParsePhaseModel(const std::string& s);
};

} // namespace ns3

#endif /* THZ_NTN_RIS_H */
