/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz Beam Management for LEO Non-Terrestrial Networks
 *
 * Integrated with MmWaveBeamformingModel for PHY pipeline compatibility,
 * MobilityModel for position-based beam selection, and actual array
 * geometry for gain evaluation at each codebook beam.
 *
 * References:
 *   [1] X. Gao et al., "Wideband beamforming for hybrid massive MIMO THz
 *       communications," IEEE JSAC, vol. 39, no. 6, 2021.
 *   [2] Z. Xiao et al., "Hierarchical codebook design for beamforming
 *       training in millimeter-wave communications," IEEE TWC, 2016.
 *   [3] C. Lin and G. Y. Li, "Terahertz communications: An array-of-
 *       subarrays solution," IEEE Commun. Mag., vol. 54, no. 12, 2016.
 */

#ifndef THZ_NTN_BEAMFORMING_H
#define THZ_NTN_BEAMFORMING_H

#include "thz-ntn-antenna-array.h"

#include <ns3/mobility-model.h>
#include <ns3/object.h>
#include <ns3/ptr.h>

#include <cstdint>
#include <string>
#include <vector>

namespace ns3
{

// Forward declarations — mmWave module
namespace mmwave
{
class MmWaveBeamformingModel;
} // namespace mmwave

// Forward declarations — thz-ntn module
class ThzNtnAntennaArray;

/**
 * \ingroup thz-ntn
 * \brief Beamforming architecture.
 */
enum class ThzBeamformingMode : uint8_t
{
    ANALOG_ONLY, ///< Pure analog (phase-shifter network)
    HYBRID,      ///< Analog + digital baseband precoding
    DIGITAL_ONLY ///< Fully digital (one RF chain per element)
};

/**
 * \ingroup thz-ntn
 * \brief Codebook resolution level for hierarchical search.
 */
enum class ThzCodebookLevel : uint8_t
{
    WIDE        = 0, ///< 4 wide beams covering full angular space
    MEDIUM      = 1, ///< 16 beams — first refinement
    NARROW      = 2, ///< 64 beams — fine search
    ULTRA_NARROW = 3  ///< 256 beams — pencil beams for THz
};

/**
 * \ingroup thz-ntn
 * \brief Single entry in a DFT-based beamforming codebook.
 */
struct ThzCodebookEntry
{
    uint32_t index;          ///< beam index within this level
    double   thetaCenter_deg;///< elevation centre of beam (deg)
    double   phiCenter_deg;  ///< azimuth centre of beam (deg)
    double   beamwidth_deg;  ///< approximate 3 dB beamwidth (deg)
    double   gain_dBi;       ///< peak gain of this beam (dBi)
};

/**
 * \ingroup thz-ntn
 * \brief Snapshot of the currently active beam state.
 */
struct ThzBeamState
{
    uint32_t        beamIndex{0};
    ThzCodebookLevel codebookLevel{ThzCodebookLevel::WIDE};
    double          steerTheta_deg{0.0};
    double          steerPhi_deg{0.0};
    double          gain_dBi{0.0};
    double          lastUpdateTime_s{0.0};
    bool            isTracking{false};
};

/**
 * \ingroup thz-ntn
 * \brief THz beam management engine for LEO-NTN links.
 *
 * Manages codebook generation, hierarchical beam search (with actual
 * array gain evaluation per beam), beam squint computation from actual
 * bandwidth/frequency, and MmWaveBeamformingModel bridge for PHY
 * integration. All beam directions come from actual MobilityModel
 * positions.
 */
class ThzNtnBeamforming : public Object
{
  public:
    static TypeId GetTypeId();

    ThzNtnBeamforming();
    ~ThzNtnBeamforming() override;

    // ---- MmWaveBeamformingModel bridge ----

    /**
     * \brief Set the mmWave beamforming model for PHY pipeline integration.
     * \param bfModel pointer to MmWaveBeamformingModel
     */
    void SetMmWaveBeamformingModel(Ptr<mmwave::MmWaveBeamformingModel> bfModel);

    // ---- Antenna array for gain evaluation ----

    /**
     * \brief Set the antenna array used for gain evaluation during search.
     * \param array pointer to ThzNtnAntennaArray
     */
    void SetAntennaArray(Ptr<ThzNtnAntennaArray> array);

    // ---- Codebook management ----

    /**
     * \brief Generate a DFT-based codebook for a given resolution level.
     */
    void GenerateCodebook(uint32_t numBeams, ThzCodebookLevel level);

    /**
     * \brief Retrieve the codebook for a given level.
     */
    std::vector<ThzCodebookEntry> GetCodebook(ThzCodebookLevel level) const;

    // ---- Beam selection from actual MobilityModel positions ----

    /**
     * \brief Select the best codebook beam toward a target node.
     *
     * Computes theta/phi from actual MobilityModel 3D positions.
     * \param target target MobilityModel
     * \param self this node's MobilityModel
     * \param level codebook level to search
     * \return beam index within the codebook
     */
    uint32_t SelectBeamForTarget(Ptr<MobilityModel> target,
                                 Ptr<MobilityModel> self,
                                 ThzCodebookLevel level) const;

    /**
     * \brief Multi-level hierarchical search with actual gain evaluation.
     *
     * At each level, compute actual array gain for each codebook beam
     * toward the target, selecting the beam with highest gain.
     * \param target target MobilityModel
     * \param self this node's MobilityModel
     * \return final beam state after refinement
     */
    ThzBeamState PerformHierarchicalSearch(Ptr<MobilityModel> target,
                                          Ptr<MobilityModel> self) const;

    // ---- Beam squint from actual bandwidth and frequency ----

    /**
     * \brief Compute beam squint angle at a given frequency.
     */
    double ComputeBeamSquintAngle_deg(double freqHz,
                                      double centerFreqHz,
                                      double steerAngle_deg) const;

    /**
     * \brief Compute beam squint loss from actual m_bandwidthHz and
     *        m_frequency internally.
     * \return worst-case gain loss in dB
     */
    double ComputeBeamSquintLoss_dB() const;

    // ---- Legacy angle-based selection ----

    /**
     * \brief Select the nearest codebook beam to a target direction.
     */
    uint32_t SelectBeam(double targetTheta_deg,
                        double targetPhi_deg,
                        ThzCodebookLevel level) const;

    /**
     * \brief Hierarchical search with angle-based inputs.
     */
    ThzBeamState PerformHierarchicalSearch(double targetTheta_deg,
                                          double targetPhi_deg) const;

    // ---- Mode & switching ----

    void SetBeamformingMode(ThzBeamformingMode mode);
    double ComputeBeamSwitchingDelay_us() const;

    // ---- Tracking interface ----

    void UpdateBeamTracking(double newTheta_deg, double newPhi_deg);
    double ComputeAnalogBeamGain_dBi(double theta_deg, double phi_deg) const;
    uint32_t GetActiveBeamIndex() const;
    const ThzBeamState& GetActiveBeamState() const;

  private:
    void ParseModeString();
    static uint32_t DefaultNumBeams(ThzCodebookLevel level);

    // Codebook storage: one vector per level (indexed 0..3)
    std::vector<ThzCodebookEntry> m_codebook[4];

    ThzBeamState       m_activeBeam;
    ThzBeamformingMode m_mode;

    // mmWave beamforming model bridge
    Ptr<mmwave::MmWaveBeamformingModel> m_mmWaveBfModel;

    // Antenna array for gain evaluation
    Ptr<ThzNtnAntennaArray> m_antennaArray;

    // TypeId attributes
    std::string m_modeStr;
    uint32_t    m_numAnalogBeams;
    uint32_t    m_numDigitalStreams;
    uint32_t    m_phaseShifterBits;
    double      m_beamSwitchDelay_us;
    bool        m_enableSquintComp;
    double      m_frequency;        ///< centre frequency (Hz)
    double      m_bandwidthHz;      ///< signal bandwidth (Hz) for squint
    uint32_t    m_numElements;      ///< total array elements
};

} // namespace ns3

#endif // THZ_NTN_BEAMFORMING_H
