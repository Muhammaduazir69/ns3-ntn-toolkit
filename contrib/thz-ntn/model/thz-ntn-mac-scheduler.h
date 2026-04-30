/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN MAC Scheduler — integrated with satellite AMC, mmWave DCI,
 * and actual Doppler computation from MobilityModel.
 *
 * Scheduling algorithms:
 *   - Round Robin: equal share of resources
 *   - Proportional Fair: balance throughput and fairness using measured history
 *   - Max Throughput: maximise aggregate throughput
 *   - QoS-Aware: URLLC -> low-absorption, eMBB -> wideband, mMTC -> narrowband
 *
 * All MCS selections delegate to actual MmWaveAmc or SatWaveformConf tables.
 * SCS selection computes Doppler from real MobilityModel positions/velocities.
 * TB size uses MmWaveAmc::CalculateTbSize() when available.
 * Scheduling decisions produce DciInfoElementTdma-compatible structures.
 *
 * References:
 *   [1] C. Han and I. F. Akyildiz, "Distance-aware bandwidth-adaptive
 *       resource allocation for wireless systems in the terahertz band,"
 *       IEEE Trans. THz Sci. Technol., vol. 6, no. 4, 2016.
 *   [2] 3GPP TS 38.214, "NR; Physical layer procedures for data,"
 *       v17.4.0, 2023.
 */

#ifndef THZ_NTN_MAC_SCHEDULER_H
#define THZ_NTN_MAC_SCHEDULER_H

#include "thz-ntn-mac.h"

#include <ns3/mobility-model.h>
#include <ns3/object.h>
#include <ns3/ptr.h>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ns3
{

// Forward declarations — mmWave module
namespace mmwave
{
class MmWaveAmc;
struct DciInfoElementTdma;
} // namespace mmwave

// Forward declarations — satellite module
class SatWaveformConf;

/**
 * \brief Scheduling algorithm type.
 */
enum class ThzNtnSchedulerType : uint8_t
{
    ROUND_ROBIN = 0,
    PROPORTIONAL_FAIR,
    MAX_THROUGHPUT,
    QOS_AWARE
};

/**
 * \brief QoS traffic class.
 */
enum class ThzNtnQosClass : uint8_t
{
    EMBB = 0,     ///< Enhanced mobile broadband
    URLLC,        ///< Ultra-reliable low-latency communications
    MMTC          ///< Massive machine-type communications
};

/**
 * \brief Waveform type for THz transmission.
 */
enum class ThzNtnWaveformType : uint8_t
{
    SC_FDMA = 0,  ///< Single-carrier FDMA
    CP_OFDM,      ///< Cyclic-prefix OFDM
    DFT_S_OFDM    ///< DFT-spread OFDM
};

/**
 * \brief Per-UE scheduling context.
 */
struct ThzNtnUeContext
{
    uint32_t ueId;                         ///< UE identifier
    uint16_t rnti;                         ///< Radio Network Temporary Identifier
    ThzNtnQosClass qosClass;               ///< QoS traffic class
    double sinr_dB;                        ///< Current SINR estimate (dB)
    double throughput_Mbps;                ///< Recent average throughput (Mbps)
    double dopplerHz;                      ///< Estimated Doppler shift (Hz)
    std::vector<uint32_t> allocatedSubBands; ///< Currently allocated sub-band indices
    uint8_t mcsIndex;                      ///< Current MCS index
    uint32_t bufferSize_bytes;             ///< Pending data in buffer (bytes)
    Ptr<MobilityModel> mobility;           ///< UE MobilityModel for Doppler computation
};

/**
 * \brief Output of the scheduling decision for a single UE.
 */
struct ThzNtnSchedulingDecision
{
    uint32_t ueId;                         ///< UE identifier
    uint16_t rnti;                         ///< RNTI for DCI compatibility
    std::vector<uint32_t> subBandIndices;  ///< Assigned sub-band indices
    uint8_t mcsIndex;                      ///< Selected MCS index
    ThzNtnWaveformType waveformType;       ///< Waveform selection
    uint32_t scsHz;                        ///< Sub-carrier spacing in Hz
    uint32_t numSymbols;                   ///< Number of OFDM symbols allocated
    uint32_t tbSize_bytes;                 ///< Transport block size (bytes)
    uint8_t harqProcessId;                 ///< HARQ process identifier
};

/**
 * \ingroup thz-ntn
 * \brief THz-aware MAC scheduler for non-terrestrial networks.
 *
 * Integrates with MmWaveAmc for MCS selection, SatWaveformConf for
 * spectral efficiency, and produces DciInfoElementTdma-compatible
 * scheduling decisions. SCS is selected from actual Doppler computed
 * via MobilityModel position/velocity. Proportional Fair metric uses
 * actual measured per-UE throughput history.
 */
class ThzNtnMacScheduler : public Object
{
  public:
    static TypeId GetTypeId();

    ThzNtnMacScheduler();
    ~ThzNtnMacScheduler() override;

    // ---- AMC integration ----

    /**
     * \brief Set the mmWave AMC module for MCS selection.
     * \param amc pointer to MmWaveAmc
     */
    void SetAmc(Ptr<mmwave::MmWaveAmc> amc);

    /**
     * \brief Set the satellite waveform configuration for AMC.
     * \param wfConf pointer to SatWaveformConf
     */
    void SetWaveformConf(Ptr<SatWaveformConf> wfConf);

    // ---- MCS selection using actual AMC ----

    /**
     * \brief Select MCS index delegating to actual AMC tables.
     * \param sinrDb SINR in dB
     * \return MCS index
     */
    uint8_t SelectMcs(double sinrDb) const;

    // ---- Doppler-aware SCS selection from MobilityModel ----

    /**
     * \brief Compute Doppler shift from actual satellite and UE mobility,
     *        then select appropriate SCS.
     * \param satMobility satellite MobilityModel
     * \param ueMobility UE MobilityModel
     * \return SCS in Hz
     */
    uint32_t SelectScs(Ptr<MobilityModel> satMobility,
                       Ptr<MobilityModel> ueMobility) const;

    // ---- Transport block size from actual MCS ----

    /**
     * \brief Compute transport block size using MmWaveAmc::CalculateTbSize
     *        when available, otherwise from waveform spectral efficiency.
     * \param mcs MCS index
     * \param numSubBands number of allocated sub-bands
     * \param numSymbols number of OFDM symbols
     * \return TB size in bytes
     */
    uint32_t ComputeTbSize(uint8_t mcs, uint32_t numSubBands,
                           uint32_t numSymbols) const;

    // ---- DCI creation for mmWave integration ----

    /**
     * \brief Create a DciInfoElementTdma-compatible structure.
     * \param decision scheduling decision
     * \return populated DciInfoElementTdma
     */
    mmwave::DciInfoElementTdma CreateDci(const ThzNtnSchedulingDecision& decision) const;

    // ---- Proportional Fair with actual throughput history ----

    /**
     * \brief Update measured throughput for a UE.
     * \param ueId UE identifier
     * \param throughput_Mbps measured throughput in Mbps
     */
    void UpdateUeThroughput(uint32_t ueId, double throughput_Mbps);

    /**
     * \brief Compute PF metric using actual measured throughput.
     * \param ueId UE identifier
     * \param instantRate instantaneous achievable rate
     * \param avgRate exponentially averaged throughput
     * \return PF metric value
     */
    double ComputePfMetric(uint32_t ueId, double instantRate, double avgRate) const;

    // ---- Main scheduling entry point ----

    /**
     * \brief Schedule resources for all UEs in the current TTI.
     * \param ues vector of UE contexts
     * \param availableSubBands vector of available sub-bands
     * \return vector of scheduling decisions
     */
    std::vector<ThzNtnSchedulingDecision> Schedule(
        const std::vector<ThzNtnUeContext>& ues,
        const std::vector<ThzNtnSubBand>& availableSubBands);

    /**
     * \brief Set the scheduling algorithm.
     * \param type scheduler type
     */
    void SetSchedulerType(ThzNtnSchedulerType type);

    /**
     * \brief Set satellite MobilityModel for Doppler computation.
     * \param satMobility satellite MobilityModel
     */
    void SetSatelliteMobility(Ptr<MobilityModel> satMobility);

  protected:
    void DoDispose() override;

  private:
    ThzNtnSchedulerType m_schedulerType;   ///< Active scheduling algorithm
    std::string m_schedulerTypeStr;        ///< String attribute for TypeId
    uint32_t m_maxRetransmissions;         ///< Maximum HARQ retransmissions
    double m_targetBler;                   ///< Target BLER for MCS selection
    uint8_t m_nextHarqProcessId;           ///< Next HARQ process ID

    // ---- AMC modules ----
    Ptr<mmwave::MmWaveAmc> m_amc;          ///< mmWave AMC
    Ptr<SatWaveformConf> m_waveformConf;   ///< Satellite waveform conf
    Ptr<MobilityModel> m_satMobility;      ///< Satellite MobilityModel

    // ---- Per-UE throughput history for PF ----
    std::map<uint32_t, double> m_avgThroughput; ///< Exponential average per UE
    double m_pfAlpha;                       ///< EMA smoothing factor

    /**
     * \brief Compute Doppler shift from two MobilityModels.
     * \param satMobility satellite
     * \param ueMobility UE
     * \return Doppler shift in Hz
     */
    double ComputeDopplerHz(Ptr<MobilityModel> satMobility,
                            Ptr<MobilityModel> ueMobility) const;

    /**
     * \brief Round-robin scheduling.
     */
    std::vector<ThzNtnSchedulingDecision> ScheduleRoundRobin(
        const std::vector<ThzNtnUeContext>& ues,
        const std::vector<ThzNtnSubBand>& subBands);

    /**
     * \brief Proportional-fair scheduling.
     */
    std::vector<ThzNtnSchedulingDecision> ScheduleProportionalFair(
        const std::vector<ThzNtnUeContext>& ues,
        const std::vector<ThzNtnSubBand>& subBands);

    /**
     * \brief Max-throughput scheduling.
     */
    std::vector<ThzNtnSchedulingDecision> ScheduleMaxThroughput(
        const std::vector<ThzNtnUeContext>& ues,
        const std::vector<ThzNtnSubBand>& subBands);

    /**
     * \brief QoS-aware scheduling.
     */
    std::vector<ThzNtnSchedulingDecision> ScheduleQosAware(
        const std::vector<ThzNtnUeContext>& ues,
        const std::vector<ThzNtnSubBand>& subBands);

    /**
     * \brief Compute scheduler metric for a UE.
     */
    double ComputeSchedulerMetric(const ThzNtnUeContext& ue,
                                  ThzNtnSchedulerType type) const;

    /**
     * \brief Parse scheduler type string to enum.
     */
    static ThzNtnSchedulerType ParseSchedulerType(const std::string& s);
};

} // namespace ns3

#endif /* THZ_NTN_MAC_SCHEDULER_H */
