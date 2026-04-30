/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Base THz-NTN PHY Layer Model
 *
 * Provides the foundational physical layer abstraction for terahertz
 * non-terrestrial network links, integrating with the ns-3 mmWave
 * module's SpectrumPhy pipeline (SpectrumValue-based SINR, mmWaveInterference,
 * MmWaveAmc) and the satellite module's SatPhy pipeline (SatSignalParameters,
 * SatWaveformConf).
 *
 * Key integration points:
 *   - SpectrumValue-based PSD creation and SINR computation
 *   - mmWaveInterference for spectrum-based interference accumulation
 *   - MmWaveAmc for MCS/CQI selection from measured SINR
 *   - MobilityModel-derived Doppler computation
 *   - THz-specific noise: kTBF + shot noise + hardware impairments
 *
 * References:
 *   [1] J. M. Jornet and I. F. Akyildiz, "Channel modeling and capacity
 *       analysis for electromagnetic wireless nanonetworks in the THz band,"
 *       IEEE Trans. Wireless Commun., vol. 10, no. 10, 2011.
 *   [2] 3GPP TR 38.811, "Study on New Radio to support non-terrestrial
 *       networks," v15.4.0, 2020.
 *   [3] T. S. Rappaport et al., "Wireless communications and applications
 *       above 100 GHz," IEEE Access, vol. 7, 2019.
 */

#ifndef THZ_NTN_PHY_H
#define THZ_NTN_PHY_H

#include <ns3/mobility-model.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/ptr.h>
#include <ns3/spectrum-model.h>
#include <ns3/spectrum-value.h>
#include <ns3/traced-callback.h>

#include "thz-ntn-hardware-impairments.h"

#include <string>

namespace ns3
{

// Forward declarations from mmwave module
namespace mmwave
{
class mmWaveInterference;
class MmWaveAmc;
class MmWavePhyMacCommon;
} // namespace mmwave

/**
 * \ingroup thz-ntn
 * \brief Waveform types supported by the THz-NTN PHY layer
 */
enum WaveformType
{
    WAVEFORM_OFDM = 0,       ///< Standard OFDM
    WAVEFORM_DFT_S_OFDM,     ///< DFT-spread OFDM (low PAPR)
    WAVEFORM_OTFS,            ///< Orthogonal time frequency space
    WAVEFORM_AFDM,            ///< Affine frequency division multiplexing
    WAVEFORM_SC_FDE,          ///< Single-carrier with FDE
    WAVEFORM_COUNT            ///< Sentinel value for iteration
};

/**
 * \ingroup thz-ntn
 * \brief Link states for the THz-NTN PHY layer
 */
enum ThzNtnLinkState
{
    LINK_IDLE = 0,       ///< No active link
    LINK_SYNCING,        ///< Synchronisation / beam acquisition in progress
    LINK_CONNECTED,      ///< Active data transfer
    LINK_BEAM_FAILURE    ///< Beam lost, recovery needed
};

/**
 * \ingroup thz-ntn
 * \brief Base THz-NTN physical layer model with spectrum-based integration
 *
 * Abstract base class for satellite and ground terminal PHY layers in a
 * THz non-terrestrial network.  Integrates with:
 *   - ns-3 SpectrumValue for PSD-based signal representation
 *   - mmWaveInterference for spectrum-based interference accumulation
 *   - MmWaveAmc for actual MCS/CQI selection from SINR
 *   - MobilityModel for Doppler computation from velocity vectors
 *   - ThzNtnHardwareImpairments for THz-specific noise contributions
 */
class ThzNtnPhy : public Object
{
  public:
    /**
     * \brief Get the ns-3 TypeId for this class
     * \return the TypeId
     */
    static TypeId GetTypeId();

    /** Default constructor */
    ThzNtnPhy();

    /** Destructor */
    ~ThzNtnPhy() override;

    /** Dispose */
    void DoDispose() override;

    // ---------------------------------------------------------------
    // SpectrumModel and PSD creation
    // ---------------------------------------------------------------

    /**
     * \brief Create a THz-band SpectrumModel
     *
     * Creates a SpectrumModel covering the configured bandwidth centered
     * at the configured center frequency, with the specified number of
     * sub-bands.  Each sub-band has frequency-dependent characteristics.
     *
     * \param centerFreqHz center frequency in Hz
     * \param bandwidthHz total bandwidth in Hz
     * \param numSubBands number of sub-bands to divide the bandwidth into
     * \return the created SpectrumModel
     */
    Ptr<SpectrumModel> CreateThzSpectrumModel(double centerFreqHz,
                                               double bandwidthHz,
                                               uint32_t numSubBands) const;

    /**
     * \brief Create transmit power spectral density
     *
     * Creates a SpectrumValue representing the transmit PSD, distributing
     * the configured transmit power uniformly across the spectrum model.
     *
     * \param txPowerDbm transmit power in dBm
     * \return the transmit PSD as a SpectrumValue
     */
    Ptr<SpectrumValue> CreateTxPsd(double txPowerDbm) const;

    /**
     * \brief Create noise power spectral density
     *
     * Creates a SpectrumValue representing the noise PSD including:
     *   - Thermal noise: kTF per sub-band
     *   - Shot noise: 2*q*I_photo per sub-band
     *   - Hardware impairment noise via ThzNtnHardwareImpairments
     *
     * \return the noise PSD as a SpectrumValue
     */
    Ptr<SpectrumValue> CreateNoisePsd() const;

    // ---------------------------------------------------------------
    // Spectrum-based SINR computation
    // ---------------------------------------------------------------

    /**
     * \brief Compute effective SINR from actual SpectrumValue objects
     *
     * Computes per-sub-band SINR from received PSD, noise PSD, and
     * interference PSD, then applies EESM (Exponential Effective SINR
     * Mapping) to produce a single effective SINR value.
     *
     * \param rxPsd received signal PSD
     * \param noisePsd noise PSD
     * \param interferencePsd interference PSD (may be nullptr for no interference)
     * \return effective SINR in dB
     */
    double ComputeSinrFromSpectrum(Ptr<const SpectrumValue> rxPsd,
                                   Ptr<const SpectrumValue> noisePsd,
                                   Ptr<const SpectrumValue> interferencePsd) const;

    // ---------------------------------------------------------------
    // mmWaveInterference integration
    // ---------------------------------------------------------------

    /**
     * \brief Set the mmWaveInterference model for interference accumulation
     * \param interference pointer to mmWaveInterference instance
     */
    void SetInterferenceModel(Ptr<mmwave::mmWaveInterference> interference);

    /**
     * \brief Get the interference model
     * \return pointer to mmWaveInterference instance
     */
    Ptr<mmwave::mmWaveInterference> GetInterferenceModel() const;

    /**
     * \brief Add an interfering signal to the interference model
     * \param interferencePsd PSD of the interfering signal
     * \param duration duration of the interfering signal
     */
    void AddInterference(Ptr<const SpectrumValue> interferencePsd, Time duration);

    // ---------------------------------------------------------------
    // MmWaveAmc integration
    // ---------------------------------------------------------------

    /**
     * \brief Set the AMC model for MCS/CQI selection
     * \param amc pointer to MmWaveAmc instance
     */
    void SetAmcModel(Ptr<mmwave::MmWaveAmc> amc);

    /**
     * \brief Get the AMC model
     * \return pointer to MmWaveAmc instance
     */
    Ptr<mmwave::MmWaveAmc> GetAmcModel() const;

    /**
     * \brief Get MCS index from measured SINR using the AMC model
     *
     * Uses MmWaveAmc's CQI/MCS selection or Shannon-based model
     * to determine the appropriate MCS for the given SINR.
     *
     * \param sinrDb SINR in dB
     * \return MCS index
     */
    uint8_t GetMcsFromSinr(double sinrDb) const;

    /**
     * \brief Get spectral efficiency for a given MCS index
     *
     * Uses the AMC model's transport block size calculation to derive
     * spectral efficiency.
     *
     * \param mcs MCS index
     * \return spectral efficiency in bps/Hz
     */
    double GetSpectralEfficiencyFromMcs(uint8_t mcs) const;

    // ---------------------------------------------------------------
    // Doppler computation from MobilityModel
    // ---------------------------------------------------------------

    /**
     * \brief Compute Doppler shift from actual MobilityModel velocity vectors
     *
     * Obtains the velocity and position vectors from both satellite and
     * ground mobility models, computes the relative radial velocity, and
     * derives the Doppler shift.
     *
     * \param satMobility satellite mobility model
     * \param groundMobility ground terminal mobility model
     * \return Doppler shift in Hz (positive = approaching)
     */
    double ComputeDopplerFromMobility(Ptr<MobilityModel> satMobility,
                                       Ptr<MobilityModel> groundMobility) const;

    /**
     * \brief Select Doppler-aware subcarrier spacing based on computed Doppler
     *
     * Computes actual Doppler from the provided mobility models and selects
     * the smallest SCS from {120kHz, 480kHz, 960kHz, 3.84MHz} that satisfies
     * SCS > 2 * f_D_max.
     *
     * \param satMobility satellite mobility model
     * \param groundMobility ground terminal mobility model
     * \return selected subcarrier spacing in Hz
     */
    uint32_t SelectSubcarrierSpacing_Hz(Ptr<MobilityModel> satMobility,
                                         Ptr<MobilityModel> groundMobility) const;

    // ---------------------------------------------------------------
    // Noise computation
    // ---------------------------------------------------------------

    /**
     * \brief Compute total noise floor in dBm
     *
     * N_floor = 10*log10(k * T * B * F) + shot noise + HW impairment noise
     * where T is the configurable noise temperature, B is bandwidth from
     * SpectrumModel, F is noise figure.
     *
     * \return noise floor in dBm
     */
    double ComputeNoiseFloor_dBm() const;

    // ---------------------------------------------------------------
    // Throughput computation
    // ---------------------------------------------------------------

    /**
     * \brief Compute achievable throughput from spectrum-derived SINR
     *
     * Uses the AMC model to determine MCS, then computes throughput
     * from MCS and bandwidth.
     *
     * \param sinrDb SINR in dB
     * \return throughput in bps
     */
    double ComputeThroughput_bps(double sinrDb) const;

    // ---------------------------------------------------------------
    // Setters and getters
    // ---------------------------------------------------------------

    void SetWaveform(WaveformType wf);
    WaveformType GetWaveform() const;

    void SetCenterFrequency(double freqHz);
    double GetCenterFrequency() const;

    void SetBandwidth(double bwHz);
    double GetBandwidth() const;

    void SetTxPower(double txPowerDbm);
    double GetTxPower() const;

    void SetNoiseFigure(double nfDb);
    double GetNoiseFigure() const;

    void SetNoiseTemperature(double tempK);
    double GetNoiseTemperature() const;

    void SetNumSubBands(uint32_t numSubBands);
    uint32_t GetNumSubBands() const;

    void SetLinkState(ThzNtnLinkState state);
    ThzNtnLinkState GetLinkState() const;

    double GetLastSinr_dB() const;

    /**
     * \brief Set the hardware impairments model
     * \param hwImpairments pointer to ThzNtnHardwareImpairments
     */
    void SetHardwareImpairments(Ptr<ThzNtnHardwareImpairments> hwImpairments);

    /**
     * \brief Get the hardware impairments model
     * \return pointer to ThzNtnHardwareImpairments
     */
    Ptr<ThzNtnHardwareImpairments> GetHardwareImpairments() const;

    /**
     * \brief Set the PhyMacCommon configuration (from mmwave module)
     * \param phyMacCommon pointer to MmWavePhyMacCommon
     */
    void SetPhyMacCommon(Ptr<mmwave::MmWavePhyMacCommon> phyMacCommon);

    /**
     * \brief Get the PhyMacCommon configuration
     * \return pointer to MmWavePhyMacCommon
     */
    Ptr<mmwave::MmWavePhyMacCommon> GetPhyMacCommon() const;

  protected:
    double m_centerFreqHz;               ///< Center frequency in Hz
    double m_bandwidthHz;                ///< Channel bandwidth in Hz
    WaveformType m_waveform;             ///< Active waveform type
    double m_txPowerDbm;                 ///< Transmit power in dBm
    double m_noiseFigureDb;              ///< Receiver noise figure in dB
    double m_noiseTemperatureK;          ///< Receiver noise temperature in K
    uint32_t m_numSubBands;              ///< Number of sub-bands in SpectrumModel
    ThzNtnLinkState m_linkState;         ///< Current link state
    mutable double m_lastSinrDb;         ///< Last computed SINR in dB

    Ptr<SpectrumModel> m_spectrumModel;  ///< Cached THz spectrum model
    Ptr<mmwave::mmWaveInterference> m_interferenceModel; ///< mmWave interference accumulator
    Ptr<mmwave::MmWaveAmc> m_amcModel;   ///< mmWave AMC for MCS selection
    Ptr<mmwave::MmWavePhyMacCommon> m_phyMacCommon; ///< mmWave PHY/MAC config
    Ptr<ThzNtnHardwareImpairments> m_hwImpairments; ///< Hardware impairments model

    // ---------------------------------------------------------------
    // TracedCallbacks
    // ---------------------------------------------------------------

    TracedCallback<double, double, double> m_sinrTrace;     ///< sinr_dB, rxPower_dBm, interference_dBm
    TracedCallback<uint8_t, double> m_mcsTrace;             ///< mcs, spectralEfficiency
    TracedCallback<double, double> m_dopplerTrace;          ///< dopplerHz, scsHz
    TracedCallback<double> m_throughputTrace;                ///< throughput_bps

    // ---------------------------------------------------------------
    // Physical constants
    // ---------------------------------------------------------------
    static constexpr double BOLTZMANN_K = 1.380649e-23;    ///< Boltzmann constant (J/K)
    static constexpr double SPEED_OF_LIGHT = 2.998e8;      ///< Speed of light (m/s)
    static constexpr double ELECTRON_CHARGE = 1.602176634e-19; ///< Elementary charge (C)
    static constexpr double PHOTO_CURRENT_A = 1.0e-6;      ///< Typical photocurrent for THz detector (A)
    static constexpr double OVERHEAD_FRACTION = 0.15;       ///< CP + pilot + guard band overhead
    static constexpr double EESM_BETA = 1.0;               ///< EESM beta parameter (tunable)

    /**
     * \brief Ensure the spectrum model is created and cached
     */
    void EnsureSpectrumModel() const;
};

} // namespace ns3

#endif // THZ_NTN_PHY_H
