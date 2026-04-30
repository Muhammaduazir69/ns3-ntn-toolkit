/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Satellite THz-NTN PHY Layer Model
 *
 * Satellite-side specialisation of the THz-NTN physical layer.  Bridges
 * to the satellite module's SatPhy pipeline for ModCod selection via
 * SatWaveformConf, Doppler pre-compensation from SatMobilityModel
 * velocity vectors, and regenerative payload processing with actual
 * SatSignalParameters.
 *
 * Integration points:
 *   - SatPhy: holds reference for augmenting satellite PHY operations
 *   - SatWaveformConf: actual DVB-RCS2 waveform configuration for ModCod
 *   - MobilityModel: actual orbital velocity for Doppler pre-compensation
 *   - SatSignalParameters: regenerative payload packet processing
 *   - SatBbFrame: frame construction with computed payload bits
 *
 * References:
 *   [1] ETSI EN 302 307-2, "DVB-S2X," v1.3.1, 2021.
 *   [2] 3GPP TR 38.821, "Solutions for NR to support non-terrestrial
 *       networks," v16.2.0, 2021.
 *   [3] O. Kodheli et al., "Satellite communications in the new space
 *       era: A survey and future challenges," IEEE COMST, vol. 23, 2021.
 */

#ifndef THZ_NTN_PHY_SAT_H
#define THZ_NTN_PHY_SAT_H

#include "thz-ntn-phy.h"

#include <ns3/mobility-model.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/ptr.h>
#include <ns3/traced-callback.h>

#include <string>
#include <vector>

namespace ns3
{

// Forward declarations from satellite module
class SatPhy;
class SatWaveformConf;
class SatSignalParameters;
class SatEnums;

/**
 * \ingroup thz-ntn
 * \brief Satellite payload processing mode
 */
enum PayloadMode
{
    PAYLOAD_TRANSPARENT = 0,  ///< Bent-pipe relay (amplify and forward)
    PAYLOAD_REGENERATIVE      ///< On-board demod/decode/re-encode/re-mod
};

/**
 * \ingroup thz-ntn
 * \brief Satellite platform class
 */
enum SatelliteClass
{
    SAT_CUBESAT = 0,  ///< CubeSat platform (1--3 W)
    SAT_SMALLSAT,     ///< SmallSat platform (5--10 W)
    SAT_FULLSAT       ///< Full-size satellite (20--50 W)
};

/**
 * \ingroup thz-ntn
 * \brief Satellite THz-NTN physical layer model with SatPhy integration
 *
 * Extends the base ThzNtnPhy with satellite-specific functionality
 * that interfaces with the satellite module's actual classes:
 *   - SatPhy for packet transmission/reception
 *   - SatWaveformConf for actual DVB-RCS2 waveform-based ModCod selection
 *   - MobilityModel for Doppler pre-compensation from orbital velocity
 *   - SatSignalParameters for regenerative payload processing
 */
class ThzNtnPhySat : public ThzNtnPhy
{
  public:
    /**
     * \brief Get the ns-3 TypeId for this class
     * \return the TypeId
     */
    static TypeId GetTypeId();

    /** Default constructor */
    ThzNtnPhySat();

    /** Destructor */
    ~ThzNtnPhySat() override;

    /** Dispose */
    void DoDispose() override;

    // ---------------------------------------------------------------
    // SatPhy integration
    // ---------------------------------------------------------------

    /**
     * \brief Set the satellite PHY instance to augment
     * \param satPhy pointer to the satellite module's SatPhy
     */
    void SetSatellitePhy(Ptr<SatPhy> satPhy);

    /**
     * \brief Get the satellite PHY instance
     * \return pointer to SatPhy
     */
    Ptr<SatPhy> GetSatellitePhy() const;

    // ---------------------------------------------------------------
    // SatWaveformConf-based ModCod selection
    // ---------------------------------------------------------------

    /**
     * \brief Set the waveform configuration from the satellite module
     * \param wfConf pointer to SatWaveformConf
     */
    void SetWaveformConf(Ptr<SatWaveformConf> wfConf);

    /**
     * \brief Get the waveform configuration
     * \return pointer to SatWaveformConf
     */
    Ptr<SatWaveformConf> GetWaveformConf() const;

    /**
     * \brief Select the best waveform ID based on measured C/N0
     *
     * Queries the actual SatWaveformConf::GetBestWaveformId() to find the
     * highest-rate waveform whose C/N0 threshold is met.
     *
     * \param cnoDb carrier-to-noise ratio in dB
     * \param symbolRateInBaud symbol rate for threshold calculation
     * \return waveform ID from SatWaveformConf, or default if none suitable
     */
    uint32_t SelectWaveformId(double cnoDb, double symbolRateInBaud) const;

    /**
     * \brief Get spectral efficiency for a waveform ID from SatWaveformConf
     *
     * \param wfId waveform ID from SelectWaveformId()
     * \param carrierBandwidthHz carrier bandwidth for SE calculation
     * \param symbolRateInBaud symbol rate for SE calculation
     * \return spectral efficiency in bps/Hz
     */
    double GetWaveformSpectralEfficiency(uint32_t wfId,
                                          double carrierBandwidthHz,
                                          double symbolRateInBaud) const;

    /**
     * \brief Compute payload bits for a given waveform and symbol rate
     *
     * Uses SatWaveformConf to get payload bytes, then converts to bits.
     *
     * \param wfId waveform ID
     * \return payload size in bits
     */
    uint32_t ComputePayloadBits(uint32_t wfId) const;

    // ---------------------------------------------------------------
    // Doppler pre-compensation from actual MobilityModel
    // ---------------------------------------------------------------

    /**
     * \brief Set the satellite mobility model for Doppler computation
     * \param mobility the satellite's MobilityModel (e.g., SatSgp4MobilityModel)
     */
    void SetSatelliteMobility(Ptr<MobilityModel> mobility);

    /**
     * \brief Get the satellite mobility model
     * \return pointer to the satellite MobilityModel
     */
    Ptr<MobilityModel> GetSatelliteMobility() const;

    /**
     * \brief Compute Doppler pre-compensation from actual orbital velocity
     *
     * Uses the satellite's MobilityModel velocity vector and the ground
     * terminal's MobilityModel position to compute the radial velocity
     * and hence the Doppler shift to pre-compensate.
     *
     * \param groundMobility ground terminal MobilityModel
     * \return Doppler pre-compensation offset in Hz (negated Doppler)
     */
    double ComputeDopplerPreCompensation(Ptr<MobilityModel> groundMobility) const;

    // ---------------------------------------------------------------
    // Regenerative payload processing
    // ---------------------------------------------------------------

    /**
     * \brief Process received signal parameters for regenerative payload
     *
     * When in REGENERATIVE mode:
     *   1. Extract packets from uplink SatSignalParameters
     *   2. Compute uplink SINR from the signal parameters
     *   3. Select THz-optimized waveform for downlink
     *   4. Create new SatSignalParameters for downlink transmission
     *
     * Returns nullptr in TRANSPARENT mode (pass-through).
     *
     * \param rxParams received uplink signal parameters
     * \return new SatSignalParameters for downlink, or nullptr for transparent
     */
    Ptr<SatSignalParameters> ProcessRegenerativePayload(
        Ptr<SatSignalParameters> rxParams) const;

    /**
     * \brief Compute on-board processing delay for regenerative payload
     *
     * Delay depends on satellite class:
     *   - CubeSat: ~500 us (limited FPGA/ASIC)
     *   - SmallSat: ~200 us (moderate processing)
     *   - FullSat: ~50 us (high-performance processor)
     *
     * \return processing delay
     */
    Time ComputeOnBoardProcessingDelay() const;

    // ---------------------------------------------------------------
    // Payload and platform configuration
    // ---------------------------------------------------------------

    void SetPayloadMode(PayloadMode mode);
    PayloadMode GetPayloadMode() const;

    void SetSatelliteClass(SatelliteClass cls);
    SatelliteClass GetSatelliteClass() const;

    void SetEnableDopplerPreComp(bool enable);
    bool GetEnableDopplerPreComp() const;

  private:
    Ptr<SatPhy> m_satPhy;                  ///< Reference to satellite module's SatPhy
    Ptr<SatWaveformConf> m_waveformConf;   ///< Satellite waveform configuration
    Ptr<MobilityModel> m_satMobility;      ///< Satellite mobility model

    PayloadMode m_payloadMode;             ///< Transparent or regenerative
    SatelliteClass m_satelliteClass;       ///< Platform class
    bool m_enableDopplerPreComp;           ///< Enable Doppler pre-compensation

    TracedCallback<uint32_t, double> m_modcodTrace; ///< wfId, spectralEfficiency
    TracedCallback<double> m_preCompTrace;          ///< preCompensation_Hz
};

} // namespace ns3

#endif // THZ_NTN_PHY_SAT_H
