/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Muhammad Uzair
 *
 * NTN CHO Helper - Top-level helper for setting up NTN-CHO simulation scenarios
 */

#ifndef NTN_CHO_HELPER_H
#define NTN_CHO_HELPER_H

#include "ns3/ntn-cho-algorithm.h"
#include "ns3/ntn-measurement-model.h"
#include "ns3/ntn-orbit-predictor.h"
#include "ns3/ntn-tte-estimator.h"

#include <ns3/node-container.h>
#include <ns3/object.h>

#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace ns3
{

/**
 * \ingroup ntn-cho
 * \brief Top-level helper for NTN CHO simulation scenarios
 *
 * Orchestrates the setup of satellite constellation, NTN channel models,
 * UE devices, and CHO algorithm configuration. Provides KPI collection
 * and trace file output for research analysis.
 */
class NtnChoHelper : public Object
{
  public:
    /**
     * \brief Aggregated KPI results from a simulation run
     */
    struct KpiResults
    {
        // Handover KPIs
        uint32_t totalHandovers = 0;
        uint32_t successfulHandovers = 0;
        uint32_t failedHandovers = 0;
        uint32_t pingPongEvents = 0;
        double hoSuccessRate = 0.0;       //!< %
        double hoFailureRate = 0.0;       //!< %
        double pingPongRate = 0.0;        //!< %

        // Quality KPIs
        double avgTimeOfStay_s = 0.0;     //!< Average time in target cell (s)
        double minTimeOfStay_s = 0.0;
        double maxTimeOfStay_s = 0.0;

        // TTE accuracy
        double avgTteError_s = 0.0;       //!< |TTE_predicted - TTE_actual| avg
        uint32_t totalTteComputations = 0;

        // Candidate statistics
        double avgAdmittedCandidates = 0.0;
        uint32_t totalCandidateEvals = 0;
    };

    static TypeId GetTypeId();
    NtnChoHelper();
    ~NtnChoHelper() override;

    // ========== Configuration ==========

    /**
     * \brief Set the NTN channel scenario
     */
    void SetNtnScenario(NtnMeasurementModel::NtnScenario scenario);

    /**
     * \brief Set the CHO trigger type
     */
    void SetChoTriggerType(NtnChoAlgorithm::TriggerType trigger);

    /**
     * \brief Set carrier frequency
     */
    void SetCarrierFrequency(double freqHz);

    /**
     * \brief Set bandwidth
     */
    void SetBandwidth(double bwHz);

    /**
     * \brief Set the TTE minimum threshold
     */
    void SetTteMinimum(Time minTte);

    /**
     * \brief Set the D1 distance threshold
     */
    void SetD1Threshold(double distance_m);

    /**
     * \brief Set the quality (SINR) threshold
     */
    void SetQualityThreshold(double sinr_dB);

    /**
     * \brief Set satellite transmit power (per-beam EIRP)
     */
    void SetSatelliteTxPower(double txPower_dBm);

    // ========== Scenario Setup ==========

    /**
     * \brief Initialize satellite constellation from existing satellite nodes
     *
     * This connects to an already-created satellite constellation (e.g., from
     * SatHelper::LoadConstellationScenario) and sets up the NTN-CHO framework
     * on top of it.
     *
     * \param satNodes NodeContainer with satellite nodes (must have SatMobilityModel)
     * \param agpContainer Antenna gain pattern container
     */
    void SetupConstellation(NodeContainer satNodes,
                            Ptr<SatAntennaGainPatternContainer> agpContainer);

    /**
     * \brief Create a CHO algorithm instance for a UE
     * \return Configured NtnChoAlgorithm
     */
    Ptr<NtnChoAlgorithm> CreateChoAlgorithm();

    /**
     * \brief Get the orbit predictor instance
     */
    Ptr<NtnOrbitPredictor> GetOrbitPredictor() const;

    /**
     * \brief Get the TTE estimator instance
     */
    Ptr<NtnTteEstimator> GetTteEstimator() const;

    /**
     * \brief Get the measurement model instance
     */
    Ptr<NtnMeasurementModel> GetMeasurementModel() const;

    // ========== KPI Collection ==========

    /**
     * \brief Enable KPI trace collection
     * \param outputDir Directory for trace files
     */
    void EnableTraces(std::string outputDir);

    /**
     * \brief Record a handover event for KPI tracking
     */
    void RecordHandover(uint16_t sourceCellId, uint16_t targetCellId, Time timeOfStay);

    /**
     * \brief Record a handover outcome
     */
    void RecordHandoverOutcome(uint16_t cellId, bool success, std::string reason);

    /**
     * \brief Record a TTE computation for accuracy tracking
     */
    void RecordTteComputation(uint32_t satId, uint32_t beamId,
                              Time predictedTte, Time actualTte);

    /**
     * \brief Get aggregated KPI results
     */
    KpiResults GetKpiResults() const;

    /**
     * \brief Write KPI results to a summary file
     */
    void WriteKpiSummary(std::string filename) const;

    /**
     * \brief Run a single-UE CHO evaluation cycle
     *
     * Convenience method for simple scenarios: scans visible satellites,
     * computes measurements, evaluates CHO conditions, and returns the
     * selected target cell.
     *
     * \param uePosition UE ground position
     * \param ueVelocity UE velocity
     * \param servingCellId Current serving cell
     * \param servingSinr_dB Current serving cell SINR
     * \param algo CHO algorithm to use
     * \return Selected target cell ID (0 if no handover needed)
     */
    uint16_t RunChoEvaluation(GeoCoordinate uePosition,
                              Vector ueVelocity,
                              uint16_t servingCellId,
                              double servingSinr_dB,
                              Ptr<NtnChoAlgorithm> algo);

  protected:
    void DoDispose() override;

  private:
    // Internal callback handlers for trace collection
    void OnHandoverExecuted(uint16_t source, uint16_t target, Time tos);
    void OnHandoverOutcome(uint16_t cellId, bool success, std::string reason);
    void OnTteComputed(uint32_t satId, uint32_t beamId, Time tte, double gain);

    // Core components
    Ptr<NtnOrbitPredictor> m_orbitPredictor;
    Ptr<NtnTteEstimator> m_tteEstimator;
    Ptr<NtnMeasurementModel> m_measurementModel;

    // Configuration
    NtnChoAlgorithm::ChoConfig m_choConfig;
    NtnMeasurementModel::NtnScenario m_scenario;
    double m_carrierFreqHz;
    double m_bandwidthHz;
    double m_satTxPower_dBm;

    // KPI tracking
    KpiResults m_kpis;
    std::vector<Time> m_tosValues;       //!< Time-of-stay samples
    std::vector<double> m_tteErrors;     //!< TTE estimation errors

    // Trace files
    bool m_tracesEnabled;
    std::ofstream m_hoTraceFile;
    std::ofstream m_measurementTraceFile;
    std::ofstream m_tteTraceFile;
    std::string m_outputDir;
};

} // namespace ns3

#endif // NTN_CHO_HELPER_H
