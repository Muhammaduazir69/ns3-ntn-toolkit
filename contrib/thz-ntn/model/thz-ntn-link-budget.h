/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Complete THz-NTN Link Budget Calculator -- Integrated APIs
 *
 * Computes end-to-end link budgets for terahertz non-terrestrial network
 * links using actual sub-model pointers and MobilityModel positions.
 *
 * Key integration points:
 *   - ThzNtnFreeSpaceLoss: FSPL + atmospheric losses from actual sub-models;
 *     individual components retrieved via GetLast*() cached accessors.
 *   - ThzNtnHardwareImpairments: hardware-limited SNR and capacity from
 *     actual EVM computation.
 *   - MobilityModel: distance computed from actual 3D positions, elevation
 *     from geometry.
 *   - Noise power: N = k*T*B*F from actual bandwidth and noise figure.
 *
 * Preset methods accept MobilityModel pointers so that link geometry
 * (distance, elevation) is derived from actual node positions.
 *
 * Both Shannon capacity and hardware-limited capacity are computed.
 *
 * References:
 *   [1] ITU-R P.619-5, "Propagation data for the evaluation of
 *       interference between stations in space and on the surface
 *       of the Earth"
 *   [2] S. Koenig et al., "Wireless sub-THz communication system
 *       with high data rate," Nature Photonics, vol. 7, 2013.
 *   [3] C. Castro et al., "100 Gb/s real-time transmission over a
 *       THz wireless link," IEEE J. Sel. Areas Commun., 2020.
 */

#ifndef THZ_NTN_LINK_BUDGET_H
#define THZ_NTN_LINK_BUDGET_H

#include <ns3/mobility-model.h>
#include <ns3/object.h>
#include <ns3/ptr.h>

#include <string>

namespace ns3
{

// Forward declarations of component models
class ThzNtnFreeSpaceLoss;
class ThzNtnMolecularAbsorption;
class ThzNtnWeatherAttenuation;
class ThzNtnScintillation;
class ThzNtnPointingError;
class ThzNtnHardwareImpairments;

/**
 * \ingroup thz-ntn
 * \brief Complete link budget calculator for THz non-terrestrial network links
 *
 * This class uses actual sub-model pointers for all propagation and hardware
 * impairment computations.  Distance and elevation are derived from actual
 * MobilityModel positions.  Individual loss components are retrieved from
 * ThzNtnFreeSpaceLoss::GetLast*() cached accessors after GetFsldB() is called.
 */
class ThzNtnLinkBudget : public Object
{
  public:
    /**
     * \brief Get the TypeId for this class.
     * \return the ns-3 TypeId
     */
    static TypeId GetTypeId();

    ThzNtnLinkBudget();
    ~ThzNtnLinkBudget() override;

    /**
     * \brief Link type enumeration.
     */
    enum LinkType
    {
        GROUND_TO_SAT = 0,  ///< Ground station to satellite (uplink)
        SAT_TO_GROUND,      ///< Satellite to ground station (downlink)
        INTER_SATELLITE,    ///< Inter-satellite link (vacuum)
        SAT_TO_HAP,         ///< Satellite to high-altitude platform
        HAP_TO_GROUND       ///< High-altitude platform to ground
    };

    /**
     * \brief Detailed link budget result with every component broken out.
     */
    struct LinkBudgetResult
    {
        // Transmitter
        double txPower_dBm;              ///< transmit power in dBm
        double txAntennaGain_dBi;        ///< transmit antenna gain in dBi
        double eirp_dBm;                 ///< effective isotropic radiated power in dBm

        // Propagation losses (from ThzNtnFreeSpaceLoss sub-models)
        double fspl_dB;                  ///< free-space path loss in dB
        double molecularAbsorption_dB;   ///< molecular absorption loss in dB
        double weatherLoss_dB;           ///< weather-induced loss in dB
        double scintillationLoss_dB;     ///< scintillation fade margin in dB
        double pointingLoss_dB;          ///< pointing error loss in dB
        double totalPathLoss_dB;         ///< total path loss (sum of above) in dB

        // Receiver
        double rxAntennaGain_dBi;        ///< receive antenna gain in dBi
        double rxPower_dBm;              ///< received power in dBm

        // Noise (N = k*T*B*F from actual parameters)
        double noiseFigure_dB;           ///< receiver noise figure in dB
        double thermalNoise_dBm;         ///< thermal noise power in dBm (kTB)
        double noisePower_dBm;           ///< total noise power (thermal + NF) in dBm

        // Performance
        double snr_dB;                   ///< signal-to-noise ratio in dB
        double hardwareLimitedSnr_dB;    ///< SNR after hardware impairment ceiling
        double shannonCapacity_Gbps;     ///< Shannon capacity in Gbps
        double hardwareLimitedCapacity_Gbps; ///< hardware-limited capacity in Gbps
        double linkMargin_dB;            ///< link margin above minimum required SNR in dB

        // Link geometry (from actual MobilityModel positions)
        double distanceM;                ///< 3D distance in metres
        double elevationDeg;             ///< elevation angle in degrees
    };

    // ---- Sub-model setters (actual pointers for computation) ----

    /**
     * \brief Set the ThzNtnFreeSpaceLoss model for FSPL + atmospheric losses.
     *
     * After calling GetFsldB(), individual losses are retrieved via
     * GetLastBaseFspl_dB(), GetLastMolecularAbsorption_dB(), etc.
     *
     * \param fsl pointer to ThzNtnFreeSpaceLoss
     */
    void SetFreeSpaceLossModel(Ptr<ThzNtnFreeSpaceLoss> fsl);

    /**
     * \brief Set the hardware impairments model.
     * \param hw pointer to ThzNtnHardwareImpairments
     */
    void SetHardwareModel(Ptr<ThzNtnHardwareImpairments> hw);

    /**
     * \brief Set the molecular absorption model.
     * \param model pointer to ThzNtnMolecularAbsorption instance
     */
    void SetMolecularAbsorptionModel(Ptr<ThzNtnMolecularAbsorption> model);

    /**
     * \brief Set the weather attenuation model.
     * \param model pointer to ThzNtnWeatherAttenuation instance
     */
    void SetWeatherModel(Ptr<ThzNtnWeatherAttenuation> model);

    /**
     * \brief Set the scintillation model.
     * \param model pointer to ThzNtnScintillation instance
     */
    void SetScintillationModel(Ptr<ThzNtnScintillation> model);

    /**
     * \brief Set the pointing error model.
     * \param model pointer to ThzNtnPointingError instance
     */
    void SetPointingErrorModel(Ptr<ThzNtnPointingError> model);

    // ---- MobilityModel-based link budget computation ----

    /**
     * \brief Compute a full link budget from actual MobilityModel positions.
     *
     * Distance is computed from tx->GetDistanceFrom(rx).  Elevation is
     * derived from the 3D geometry.  All losses are computed from actual
     * sub-models via ThzNtnFreeSpaceLoss::GetFsldB() and its cached
     * GetLast*() accessors.
     *
     * Noise power = k * T * B * F where T=290K.
     *
     * Both Shannon and hardware-limited capacity are computed.
     *
     * \param tx transmitter MobilityModel
     * \param rx receiver MobilityModel
     * \param txPowerDbm transmit power in dBm
     * \param txGainDbi transmit antenna gain in dBi
     * \param rxGainDbi receive antenna gain in dBi
     * \param bandwidthHz channel bandwidth in Hz
     * \param noiseFigureDb receiver noise figure in dB
     * \return detailed LinkBudgetResult
     */
    LinkBudgetResult ComputeLinkBudget(Ptr<MobilityModel> tx,
                                       Ptr<MobilityModel> rx,
                                       double txPowerDbm,
                                       double txGainDbi,
                                       double rxGainDbi,
                                       double bandwidthHz,
                                       double noiseFigureDb) const;

    // ---- Legacy parameter-based computation ----

    /**
     * \brief Compute a full link budget from explicit parameters.
     *
     * Uses internal sub-models when available; falls back to analytical
     * computation otherwise.
     *
     * \param type          link type (ground-sat, ISL, etc.)
     * \param freqHz        carrier frequency in Hz
     * \param distanceM     3-D link distance in metres
     * \param elevationDeg  elevation angle in degrees
     * \param txPower_dBm   transmit power in dBm
     * \param txGain_dBi    transmit antenna gain in dBi
     * \param rxGain_dBi    receive antenna gain in dBi
     * \param bandwidth_Hz  channel bandwidth in Hz
     * \param noiseFigure_dB receiver noise figure in dB
     * \return detailed LinkBudgetResult structure
     */
    LinkBudgetResult ComputeLinkBudget(LinkType type,
                                       double freqHz,
                                       double distanceM,
                                       double elevationDeg,
                                       double txPower_dBm,
                                       double txGain_dBi,
                                       double rxGain_dBi,
                                       double bandwidth_Hz,
                                       double noiseFigure_dB) const;

    // ---- Preset methods with MobilityModel ----

    /**
     * \brief Compute TeraLink-225GHz budget from actual positions.
     *
     * Preset: 225 GHz, 3W TX (34.77 dBm), TX gain 40 dBi,
     *         RX gain 45 dBi, NF 10 dB, BW 10 GHz.
     *
     * \param sat satellite MobilityModel
     * \param ground ground terminal MobilityModel
     * \return detailed LinkBudgetResult
     */
    LinkBudgetResult ComputeTeraLinkBudget(Ptr<MobilityModel> sat,
                                            Ptr<MobilityModel> ground) const;

    /**
     * \brief Compute D-band LEO budget from actual positions.
     *
     * Preset: 140 GHz, 10W TX (40 dBm), TX gain 38 dBi,
     *         RX gain 42 dBi, NF 8 dB, BW 20 GHz.
     *
     * \param sat satellite MobilityModel
     * \param ground ground terminal MobilityModel
     * \return detailed LinkBudgetResult
     */
    LinkBudgetResult ComputeDbandLeoBudget(Ptr<MobilityModel> sat,
                                            Ptr<MobilityModel> ground) const;

    /**
     * \brief Compute TeraLink-1 preset budget (legacy, no MobilityModel).
     * \return detailed LinkBudgetResult
     */
    LinkBudgetResult ComputeTeraLinkBudget() const;

    /**
     * \brief Compute D-band LEO preset budget (legacy, no MobilityModel).
     * \return detailed LinkBudgetResult
     */
    LinkBudgetResult ComputeDbandLeoBudget() const;

    /**
     * \brief Compute ISL budget from actual satellite positions.
     * \param sat1 first satellite MobilityModel
     * \param sat2 second satellite MobilityModel
     * \param freqHz carrier frequency in Hz
     * \return detailed LinkBudgetResult
     */
    LinkBudgetResult ComputeIslBudget(Ptr<MobilityModel> sat1,
                                       Ptr<MobilityModel> sat2,
                                       double freqHz) const;

    /**
     * \brief Compute ISL budget (legacy, explicit parameters).
     * \param freqHz carrier frequency in Hz
     * \param distanceKm inter-satellite distance in km
     * \return detailed LinkBudgetResult
     */
    LinkBudgetResult ComputeIslBudget(double freqHz, double distanceKm) const;

    /**
     * \brief Print a formatted link budget to the ns-3 log.
     * \param result link budget result to print
     */
    void PrintLinkBudget(const LinkBudgetResult& result) const;

  protected:
    void DoDispose() override;

  private:
    /**
     * \brief Compute free-space path loss analytically.
     * FSPL = 20*log10(4*pi*d*f/c) in dB.
     */
    double ComputeFspl(double distanceM, double freqHz) const;

    /**
     * \brief Compute thermal noise power: N = kTB in dBm.
     */
    double ComputeThermalNoise(double bandwidth_Hz) const;

    /**
     * \brief Compute Shannon capacity from SNR and bandwidth.
     */
    double ComputeShannonCapacity(double bandwidth_Hz, double snr_dB) const;

    /**
     * \brief Check if the link type is atmospheric.
     */
    bool IsAtmosphericLink(LinkType type) const;

    /**
     * \brief Compute elevation angle from two positions.
     */
    double ComputeElevationAngle(Ptr<MobilityModel> ground,
                                  Ptr<MobilityModel> sat) const;

    /**
     * \brief Compute altitude from MobilityModel position.
     */
    double ComputeAltitudeKm(Ptr<MobilityModel> node) const;

    // Sub-model pointers (actual computation)
    Ptr<ThzNtnFreeSpaceLoss> m_fslModel;          ///< ThzNtnFreeSpaceLoss
    Ptr<ThzNtnHardwareImpairments> m_hardwareModel; ///< hardware impairments
    Ptr<ThzNtnMolecularAbsorption> m_molecularModel;  ///< molecular absorption
    Ptr<ThzNtnWeatherAttenuation> m_weatherModel;     ///< weather attenuation
    Ptr<ThzNtnScintillation> m_scintillationModel;    ///< scintillation
    Ptr<ThzNtnPointingError> m_pointingModel;         ///< pointing error

    double m_minRequiredSnr_dB; ///< minimum required SNR for link margin
    double m_defaultFreqHz;     ///< default frequency for FSL model
};

} // namespace ns3

#endif // THZ_NTN_LINK_BUDGET_H
