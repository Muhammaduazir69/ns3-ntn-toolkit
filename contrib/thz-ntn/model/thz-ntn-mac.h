/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN MAC Protocol — integrated with satellite and mmWave module APIs.
 *
 * Ultra-wideband MAC layer for terahertz non-terrestrial networks.
 * Implements Distance-Aware Multi-Carrier (DAMC) resource allocation
 * using actual ThzNtnMolecularAbsorption per-frequency computations,
 * satellite superframe/carrier structure mapping, actual AMC-based
 * capacity computation, and SatSignalParameters-compatible txInfo.
 *
 * References:
 *   [1] J. M. Jornet and I. F. Akyildiz, "Channel modeling and capacity
 *       analysis for electromagnetic wireless nanonetworks in the THz band,"
 *       IEEE Trans. Wireless Commun., vol. 10, no. 10, 2011.
 *   [2] C. Han and I. F. Akyildiz, "Distance-aware bandwidth-adaptive
 *       resource allocation for wireless systems in the terahertz band,"
 *       IEEE Trans. THz Sci. Technol., vol. 6, no. 4, 2016.
 */

#ifndef THZ_NTN_MAC_H
#define THZ_NTN_MAC_H

#include <ns3/mobility-model.h>
#include <ns3/object.h>
#include <ns3/ptr.h>
#include <ns3/traced-callback.h>

#include <cstdint>
#include <vector>

namespace ns3
{

// Forward declarations — satellite module
class SatSuperframeConf;
class SatWaveformConf;
class SatWaveform;

// Forward declarations — thz-ntn module
class ThzNtnMolecularAbsorption;

/**
 * \brief Access mode for the THz-NTN MAC protocol.
 */
enum class ThzNtnAccessMode : uint8_t
{
    TDMA = 0,            ///< Pure time-division multiple access
    FDMA,                ///< Pure frequency-division multiple access
    HYBRID_TDMA_FDMA,    ///< Hybrid time-frequency division
    OFDMA                ///< Orthogonal frequency-division multiple access
};

/**
 * \brief Describes a single sub-band in the THz resource grid.
 */
struct ThzNtnSubBand
{
    uint32_t index;              ///< Sub-band index in the grid
    double centerFreqHz;         ///< Centre frequency of this sub-band (Hz)
    double bandwidthHz;          ///< Bandwidth of this sub-band (Hz)
    bool isAvailable;            ///< Whether the sub-band is available for allocation
    double absorptionLoss_dB;    ///< Molecular absorption loss for current link (dB)
    uint32_t assignedUeId;       ///< UE ID this sub-band is assigned to (0 = unassigned)
    uint32_t carrierIndex;       ///< Mapped satellite carrier index
};

/**
 * \brief Describes a single MAC frame within a superframe.
 */
struct ThzNtnMacFrame
{
    uint32_t frameIndex;         ///< Frame index within the superframe
    uint32_t numSlots;           ///< Number of time slots in this frame
    double slotDuration_us;      ///< Duration of each slot in microseconds
    double guardInterval_us;     ///< Guard interval between slots in microseconds
    double preambleDuration_us;  ///< Preamble duration per slot in microseconds
    double pilotOverheadRatio;   ///< Fraction of slot used for pilot symbols
};

/**
 * \brief Satellite-compatible txInfo for THz-NTN transmissions.
 *
 * Maps to SatSignalParameters::txInfo_s fields.
 */
struct ThzNtnTxInfo
{
    uint32_t ueId;               ///< Target UE identifier
    uint32_t waveformId;         ///< SatWaveformConf waveform ID
    uint8_t modcod;              ///< Modulation and coding scheme index
    uint8_t sliceId;             ///< Network slice identifier
    std::vector<uint32_t> subBandIndices; ///< Allocated sub-band indices
    uint32_t tbSizeBytes;        ///< Transport block size in bytes
};

/**
 * \ingroup thz-ntn
 * \brief THz-NTN MAC protocol with DAMC resource allocation.
 *
 * This class implements an ultra-wideband MAC protocol for terahertz
 * non-terrestrial network links.  Sub-band filtering uses actual
 * ThzNtnMolecularAbsorption computations at each sub-band centre
 * frequency using real node positions via MobilityModel.  Capacity
 * computation delegates to SatWaveformConf spectral efficiency tables
 * instead of raw Shannon formula.
 */
class ThzNtnMac : public Object
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    ThzNtnMac();
    ~ThzNtnMac() override;

    // ---- Resource grid configuration ----

    /**
     * \brief Configure the ultra-wideband resource grid.
     * \param totalBandwidthHz total system bandwidth in Hz
     * \param subBandWidthHz width of each sub-band in Hz
     * \param centerFreqHz centre frequency of the overall band in Hz
     */
    void ConfigureResourceGrid(double totalBandwidthHz,
                               double subBandWidthHz,
                               double centerFreqHz);

    /**
     * \brief Get sub-bands whose absorption loss is below the threshold.
     * \param maxAbsorptionLoss_dB maximum tolerable absorption loss (dB)
     * \return vector of sub-bands passing the DAMC filter
     */
    std::vector<ThzNtnSubBand> GetAvailableSubBands(double maxAbsorptionLoss_dB) const;

    /**
     * \brief Allocate sub-bands to a UE using distance-aware selection.
     * \param ueId UE identifier
     * \param numSubBands number of sub-bands to allocate
     * \param txMobility transmitter MobilityModel
     * \param rxMobility receiver MobilityModel
     */
    void AllocateSubBands(uint32_t ueId,
                          uint32_t numSubBands,
                          Ptr<MobilityModel> txMobility,
                          Ptr<MobilityModel> rxMobility);

    /**
     * \brief Release all sub-bands allocated to a UE.
     * \param ueId UE identifier
     */
    void ReleaseSubBands(uint32_t ueId);

    /**
     * \brief Get the number of currently active (allocated) sub-bands.
     * \return number of active sub-bands
     */
    uint32_t GetNumActiveSubBands() const;

    // ---- Molecular absorption model (actual per-frequency computation) ----

    /**
     * \brief Set the molecular absorption model for DAMC filtering.
     * \param model pointer to ThzNtnMolecularAbsorption
     */
    void SetMolecularAbsorptionModel(Ptr<ThzNtnMolecularAbsorption> model);

    /**
     * \brief Update sub-band availability using actual molecular absorption
     *        computations at each sub-band centre frequency for the real
     *        link geometry derived from MobilityModel positions.
     * \param txMobility transmitter MobilityModel
     * \param rxMobility receiver MobilityModel
     */
    void UpdateSubBandAvailability(Ptr<MobilityModel> txMobility,
                                   Ptr<MobilityModel> rxMobility);

    // ---- Satellite frame structure integration ----

    /**
     * \brief Set the satellite superframe configuration.
     *
     * Maps THz sub-bands onto the satellite carrier structure.
     * \param sfConf pointer to SatSuperframeConf
     */
    void SetSuperframeConf(Ptr<SatSuperframeConf> sfConf);

    /**
     * \brief Set the satellite waveform configuration for AMC.
     * \param wfConf pointer to SatWaveformConf
     */
    void SetWaveformConf(Ptr<SatWaveformConf> wfConf);

    // ---- Tx info creation (SatSignalParameters-compatible) ----

    /**
     * \brief Create a ThzNtnTxInfo for a UE allocation.
     * \param ueId target UE identifier
     * \param subBandIndices allocated sub-band indices
     * \param modcod modulation and coding index
     * \return ThzNtnTxInfo structure
     */
    ThzNtnTxInfo CreateTxInfo(uint32_t ueId,
                              const std::vector<uint32_t>& subBandIndices,
                              uint8_t modcod) const;

    // ---- Access mode ----

    /**
     * \brief Set the MAC access mode.
     * \param mode the desired access mode
     */
    void SetAccessMode(ThzNtnAccessMode mode);

    // ---- Frame configuration ----

    /**
     * \brief Configure the frame structure with all overhead parameters.
     * \param numSlots number of time slots per frame
     * \param slotDuration_us slot duration in microseconds
     * \param guardInterval_us guard interval in microseconds
     * \param preambleDuration_us preamble duration per slot (us)
     * \param pilotOverheadRatio fraction of slot for pilot symbols [0,1]
     */
    void ConfigureFrame(uint32_t numSlots,
                        double slotDuration_us,
                        double guardInterval_us,
                        double preambleDuration_us,
                        double pilotOverheadRatio);

    // ---- Performance metrics ----

    /**
     * \brief Compute MAC efficiency accounting for guard intervals,
     *        preambles, and pilot symbol overhead.
     * \return efficiency ratio in [0,1]
     */
    double ComputeMacEfficiency() const;

    /**
     * \brief Compute aggregate capacity using actual waveform spectral
     *        efficiencies from SatWaveformConf for each sub-band SINR.
     * \param subBandSinrs_dB per-sub-band SINR values in dB
     * \return aggregate capacity in Gbps
     */
    double ComputeAggregateCapacity_Gbps(const std::vector<double>& subBandSinrs_dB) const;

  protected:
    void DoDispose() override;

  private:
    // ---- Resource grid state ----
    std::vector<ThzNtnSubBand> m_subBands;   ///< All sub-bands in the grid
    double m_totalBandwidthHz;               ///< Total system bandwidth (Hz)
    double m_subBandWidthHz;                 ///< Width of each sub-band (Hz)
    double m_centerFreqHz;                   ///< Centre frequency (Hz)

    // ---- Access mode ----
    ThzNtnAccessMode m_accessMode;           ///< Current access mode
    std::string m_accessModeStr;             ///< Access mode string for TypeId

    // ---- Frame structure (fully parameterised overhead) ----
    ThzNtnMacFrame m_frame;                  ///< Current frame configuration
    uint32_t m_numSlots;                     ///< Number of slots per frame
    double m_slotDuration_us;                ///< Slot duration (us)
    double m_guardInterval_us;               ///< Guard interval (us)
    double m_preambleDuration_us;            ///< Preamble duration per slot (us)
    double m_pilotOverheadRatio;             ///< Pilot overhead ratio [0,1]
    double m_maxAbsorptionThreshold_dB;      ///< DAMC absorption threshold (dB)

    // ---- Models ----
    Ptr<ThzNtnMolecularAbsorption> m_absorptionModel; ///< Absorption model
    Ptr<SatSuperframeConf> m_superframeConf;          ///< Satellite superframe
    Ptr<SatWaveformConf> m_waveformConf;              ///< Satellite waveform AMC

    /**
     * \brief Compute link geometry from two MobilityModel pointers.
     * \param txMobility transmitter
     * \param rxMobility receiver
     * \param[out] distanceM 3D distance in metres
     * \param[out] altTx_km transmitter altitude in km
     * \param[out] altRx_km receiver altitude in km
     * \param[out] elevationDeg elevation angle in degrees
     */
    void ComputeLinkGeometry(Ptr<MobilityModel> txMobility,
                             Ptr<MobilityModel> rxMobility,
                             double& distanceM,
                             double& altTx_km,
                             double& altRx_km,
                             double& elevationDeg) const;

    /**
     * \brief Map sub-bands to satellite carriers from superframe conf.
     */
    void MapSubBandsToCarriers();

    /**
     * \brief Select the best waveform ID for a given C/N0.
     * \param cnoDb C/N0 in dB
     * \return waveform ID from SatWaveformConf, or 0 if none found
     */
    uint32_t SelectWaveformId(double cnoDb) const;

    /**
     * \brief Parse access mode string to enum.
     */
    static ThzNtnAccessMode ParseAccessMode(const std::string& mode);

    // ---- Traces ----
    TracedCallback<uint32_t, uint32_t> m_subBandAllocationTrace;
};

} // namespace ns3

#endif /* THZ_NTN_MAC_H */
