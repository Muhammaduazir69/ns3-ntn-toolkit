/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Muhammad Uzair
 *
 * NTN Conditional Handover Algorithm
 *
 * Implements the 3GPP TS 38.331 CHO state machine with a novel TTE-aware
 * candidate cell selection mechanism for non-terrestrial networks.
 *
 * The algorithm:
 * 1. Network configures CHO with candidate cells and trigger type
 * 2. UE periodically monitors conditions (D1 location, signal quality)
 * 3. When conditions are met, TTE is computed for each candidate
 * 4. NOVEL: Candidate with longest TTE (above minimum) and sufficient
 *    signal quality is selected
 * 5. Handover executes to pre-selected candidate when serving cell degrades
 *
 * Reference: 3GPP TS 38.331 Section 5.3.5.8
 */

#ifndef NTN_CHO_ALGORITHM_H
#define NTN_CHO_ALGORITHM_H

#include "ntn-orbit-predictor.h"
#include "ntn-tte-estimator.h"

#include <ns3/callback.h>
#include <ns3/event-id.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/traced-callback.h>

#include <map>
#include <vector>

namespace ns3
{

/**
 * \ingroup ntn-cho
 * \brief 3GPP Rel-17 Conditional Handover algorithm with TTE-aware candidate selection
 */
class NtnChoAlgorithm : public Object
{
  public:
    /**
     * \brief CHO state machine states per 3GPP TS 38.331
     */
    enum ChoState
    {
        CHO_IDLE,                  //!< No CHO configured
        CHO_PREPARED,              //!< Candidate cells configured, monitoring not started
        CHO_CONDITION_MONITORING,  //!< Actively evaluating trigger conditions
        CHO_EXECUTING,             //!< Handover in progress to selected target
        CHO_COMPLETED              //!< Handover completed successfully
    };

    /**
     * \brief Types of CHO triggers supported
     */
    enum TriggerType
    {
        TRIGGER_EVENT_A3,          //!< Traditional RSRP offset trigger
        TRIGGER_LOCATION_D1,       //!< 3GPP condEventD1: distance to beam center
        TRIGGER_TIME_BASED,        //!< Timer-based beam dwell trigger
        TRIGGER_TTE_AWARE,         //!< NOVEL: TTE + location + quality
        TRIGGER_THZ_BEAM_QUALITY   //!< Handover when THz beam tracking error exceeds threshold
    };

    /**
     * \brief CHO configuration parameters
     */
    struct ChoConfig
    {
        TriggerType triggerType = TRIGGER_TTE_AWARE;
        double d1Threshold_m = 50000.0;       //!< D1 distance threshold (50 km default)
        double qualityThreshold_dB = -3.0;    //!< Min SINR for candidate admission
        Time tteMinimum = Seconds(15.0);      //!< Min acceptable TTE
        Time conditionMonitorPeriod = Seconds(1.0); //!< Condition check interval
        uint8_t maxCandidates = 4;            //!< Max simultaneous prepared candidates
        Time t304Timer = Seconds(2.0);        //!< CHO execution timer
        double gainThreshold_dB = -3.0;       //!< Beam gain threshold for TTE computation
        Time tteEpsilon = Seconds(2.0);       //!< TTE tie-breaking window
        double a3Offset_dB = 3.0;             //!< A3 event offset (for baseline)
        Time a3TimeToTrigger = MilliSeconds(160); //!< A3 TTT (for baseline)
    };

    /**
     * \brief Information tracked for each candidate cell
     */
    struct CandidateInfo
    {
        uint16_t cellId = 0;
        uint32_t satId = 0;
        uint32_t beamId = 0;
        bool d1Met = false;               //!< D1 condition currently satisfied
        double sinr_dB = -100.0;          //!< Latest SINR measurement
        double gain_dB = -100.0;          //!< Latest beam gain
        Time tte = Seconds(0);            //!< Estimated time-to-exit
        Time d1MetSince = Seconds(0);     //!< When D1 was first met
        bool admitted = false;            //!< Passed TTE + quality filter
        Time lastUpdate = Seconds(0);     //!< Last measurement update time
    };

    static TypeId GetTypeId();
    NtnChoAlgorithm();
    ~NtnChoAlgorithm() override;

    /**
     * \brief Configure the CHO algorithm
     */
    void Configure(ChoConfig config);

    /**
     * \brief Get current CHO configuration
     */
    ChoConfig GetConfig() const;

    /**
     * \brief Set the TTE estimator
     */
    void SetTteEstimator(Ptr<NtnTteEstimator> estimator);

    /**
     * \brief Set the orbit predictor
     */
    void SetOrbitPredictor(Ptr<NtnOrbitPredictor> predictor);

    /**
     * \brief Add a candidate cell for CHO preparation
     * \param cellId Logical cell identifier
     * \param satId Satellite identifier
     * \param beamId Beam identifier
     */
    void AddCandidateCell(uint16_t cellId, uint32_t satId, uint32_t beamId);

    /**
     * \brief Remove a candidate cell
     */
    void RemoveCandidateCell(uint16_t cellId);

    /**
     * \brief Clear all candidate cells
     */
    void ClearCandidates();

    /**
     * \brief Update measurement for a candidate cell
     * \param cellId Cell identifier
     * \param sinr_dB Measured SINR
     * \param gain_dB Measured beam gain
     */
    void UpdateMeasurement(uint16_t cellId, double sinr_dB, double gain_dB);

    /**
     * \brief Start condition monitoring
     * \param uePosition Current UE position
     * \param ueVelocity UE velocity vector
     */
    void StartMonitoring(GeoCoordinate uePosition, Vector ueVelocity);

    /**
     * \brief Stop condition monitoring
     */
    void StopMonitoring();

    /**
     * \brief Evaluate all conditions and update candidates
     *
     * Called periodically during CHO_CONDITION_MONITORING state.
     * For each candidate:
     *   1. Check D1 condition (distance to beam center < threshold)
     *   2. Check signal quality (SINR > quality threshold)
     *   3. Compute TTE if both conditions met
     *   4. Admit candidate if TTE >= minimum
     */
    void EvaluateConditions();

    /**
     * \brief Select the best candidate using TTE-aware algorithm (NOVEL)
     *
     * THE CORE NOVEL ALGORITHM:
     * 1. Filter: remove candidates with SINR < Q_threshold
     * 2. Filter: remove candidates with TTE < TTE_minimum
     * 3. Select: candidate with maximum TTE
     * 4. Tie-break: if multiple candidates within epsilon, pick highest SINR
     *
     * \return Cell ID of best candidate (0 if none available)
     */
    uint16_t SelectBestCandidate() const;

    /**
     * \brief Select using baseline A3 algorithm (for comparison)
     * \return Cell ID of best candidate per A3 criterion
     */
    uint16_t SelectBaselineA3(double servingSinr_dB) const;

    /**
     * \brief Select using baseline location-only algorithm (for comparison)
     * \return Cell ID based on D1 condition only (no TTE)
     */
    uint16_t SelectBaselineLocationOnly() const;

    /**
     * \brief Execute handover to a target cell
     * \param targetCellId Target cell identifier
     */
    void ExecuteHandover(uint16_t targetCellId);

    /**
     * \brief Cancel ongoing CHO
     */
    void CancelHandover();

    /**
     * \brief Get current CHO state
     */
    ChoState GetState() const;

    /**
     * \brief Get information about all candidates
     */
    std::map<uint16_t, CandidateInfo> GetCandidates() const;

    /**
     * \brief Get number of admitted candidates
     */
    uint32_t GetNumAdmittedCandidates() const;

    // Callback types
    typedef Callback<void, uint16_t, uint16_t> HandoverExecutionCallback;
    //                    sourceCellId, targetCellId
    typedef Callback<void, uint16_t, double, Time> CandidateAdmittedCallback;
    //                    cellId,  sinr,  tte

    /**
     * \brief Set callback for handover execution
     */
    void SetHandoverExecutionCallback(HandoverExecutionCallback cb);

    /**
     * \brief Set callback for candidate admission
     */
    void SetCandidateAdmittedCallback(CandidateAdmittedCallback cb);

    // Trace sources
    TracedCallback<uint16_t, uint16_t, Time> m_handoverExecutedTrace;
    //              source,  target,  timeOfStay
    TracedCallback<uint16_t, bool, std::string> m_handoverOutcomeTrace;
    //              cellId,  success, reason
    TracedCallback<uint16_t, double, Time, bool> m_candidateEvalTrace;
    //              cellId,  sinr,   tte,   admitted
    TracedCallback<ChoState, ChoState> m_stateTransitionTrace;
    //              oldState, newState

  protected:
    void DoDispose() override;

  private:
    /**
     * \brief Periodic condition evaluation callback
     */
    void DoEvaluateConditions();

    /**
     * \brief Transition CHO state machine
     */
    void TransitionState(ChoState newState);

    /**
     * \brief Check D1 condition for a candidate
     */
    bool CheckD1Condition(const CandidateInfo& cand) const;

    ChoState m_state;                                  //!< Current state
    ChoConfig m_config;                                //!< Configuration
    std::map<uint16_t, CandidateInfo> m_candidates;    //!< cellId -> CandidateInfo
    uint16_t m_servingCellId;                          //!< Current serving cell
    GeoCoordinate m_uePosition;                        //!< Latest UE position
    Vector m_ueVelocity;                               //!< UE velocity

    Ptr<NtnTteEstimator> m_tteEstimator;              //!< TTE computation engine
    Ptr<NtnOrbitPredictor> m_orbitPredictor;           //!< Orbit prediction engine

    EventId m_monitorEvent;                            //!< Periodic monitor event
    EventId m_t304Event;                               //!< T304 timer event
    Time m_lastHoTime;                                 //!< Time of last handover (for ToS)
    uint16_t m_lastSourceCell;                         //!< Last source cell (for ping-pong)

    HandoverExecutionCallback m_hoCallback;
    CandidateAdmittedCallback m_admitCallback;

    /**
     * \brief Evaluate THz beam quality for a candidate
     * \param candidateIdx Cell ID of the candidate
     * \return true if pointing error and THz SNR are within thresholds
     */
    bool EvaluateThzBeamQuality(uint32_t candidateIdx) const;

    /**
     * \brief Compute TTE for a THz narrow beam
     * \param candidateIdx Cell ID of the candidate
     * \return TTE in seconds based on THz beam footprint
     */
    double ComputeThzBeamTte(uint32_t candidateIdx) const;

    /**
     * \brief Prepare multi-band (Ka + THz) candidate sets
     *
     * For each candidate, evaluate both Ka-band and THz quality.
     * THz is used as primary when available, Ka-band as fallback.
     */
    void PrepareMultiBandCandidates();

    double m_thzBeamTrackingThreshold_deg; //!< Max pointing error before HO trigger (default 0.3 deg)
    double m_thzSnrThreshold_dB;           //!< Min THz SNR for candidate admission (default 0 dB)
    bool m_enableMultiBandCho;             //!< Enable Ka+THz dual candidate sets
    double m_thzBeamwidth_deg;             //!< THz beam 3dB beamwidth for TTE calc (default 0.5 deg)

    static constexpr uint16_t INVALID_CELL_ID = 0;
};

} // namespace ns3

#endif // NTN_CHO_ALGORITHM_H
