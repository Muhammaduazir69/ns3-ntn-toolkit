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
 * Standards positioning of the trigger classes (precise):
 *   - CondEvents A4, T1 (time) and D1 (distance) are Rel-17 NORMATIVE
 *     (TS 38.331 §5.5.4); in Rel-17 T1/D1 are configured TOGETHER WITH A4,
 *     not standalone (see ChoConfig::combineWithA4).
 *   - CondEvent D2 (distance with MOVING reference locations derived from
 *     the broadcast ephemeris) is Rel-18 (TS 38.331 §5.5.4.15a).
 *   - The elevation and timing-advance triggers are TR 38.821 §6 STUDIED
 *     mechanisms, not standardized CondEvents.
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
        TRIGGER_THZ_BEAM_QUALITY,  //!< Handover when THz beam tracking error exceeds threshold
        /**
         * NOVEL (3GPP Rel-19 conditional LTM): L1/L2-Triggered Mobility combined
         * with CHO reliability. Candidates are admitted on L1-filtered (moving-
         * average) low-latency measurements crossing serving + hysteresis for N
         * consecutive L1 reports AND passing the TTE stability filter; execution
         * is a MAC-CE-style fast cell switch (ltmSwitchDelay, tens of ms) instead
         * of the full RRC reconfiguration (t304-scale). Refs: 3GPP Rel-19 NR
         * mobility WI (conditional LTM); Ericsson Technology Review, "Reducing
         * handover interruption with L1/L2-Triggered Mobility".
         */
        TRIGGER_LTM_CONDITIONAL,
        /**
         * NOVEL (PCHO — trajectory-prediction CHO for LEO): per-candidate SINR
         * trajectories are forecast over predictionHorizon by a linear-trend
         * predictor over the measurement history (documented stand-in for the
         * GRU predictor of Yang et al., "A Conditional Handover Strategy Based
         * on Trajectory Prediction for High-Speed Terminals in LEO Satellite
         * Networks") and fused with the ephemeris TTE; the handover triggers
         * BEFORE the predicted serving outage, toward the candidate that
         * maximizes the predicted time-of-stay.
         */
        TRIGGER_TRAJECTORY_PREDICTIVE,
        /**
         * 3GPP Rel-17 NTN CondEventT1 (time-based CHO): the ephemeris
         * schedules a handover window — when the SERVING cell's remaining
         * time-of-service (TTE from the orbit predictor) drops inside
         * t1WindowDuration, quality-passing candidates are admitted. The
         * paper's "time-based" trigger class (Deng 2026 Sec. VI).
         */
        TRIGGER_TIME_T1,
        /**
         * Elevation-based NTN trigger: serving elevation (derived from the
         * ephemeris/GNSS slant range at the configured orbit altitude) falls
         * below elevationMinDeg while a candidate is above it plus
         * hysteresis. The paper's "elevation" trigger class.
         */
        TRIGGER_ELEVATION,
        /**
         * Timing-advance-based NTN trigger (Rel-18 discussion): the UE-side
         * TA (2 x slant/c from ephemeris+GNSS) exceeds taServingMax, or a
         * candidate offers at least taAdvantage less TA. The paper's
         * "timing-advance" trigger class.
         */
        TRIGGER_TIMING_ADVANCE,
        /**
         * 3GPP Rel-18 NTN CondEventD2 (TS 38.331 §5.5.4.15a): distance-based
         * CHO with MOVING reference locations derived from the broadcast
         * ephemeris. Entering condition: distance(UE, serving moving ref)
         * - hysteresisLocation > d2Thresh1_m AND distance(UE, candidate
         * moving ref) + hysteresisLocation < d2Thresh2_m. The moving
         * references are the live beam centers from the orbit predictor, so
         * they track the satellites (Earth-moving cells), unlike D1's fixed
         * reference semantics.
         */
        TRIGGER_DISTANCE_D2
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

        // ---- Rel-19 conditional LTM (TRIGGER_LTM_CONDITIONAL) ----
        uint8_t ltmL1FilterK = 4;             //!< L1 moving-average window (reports)
        double ltmHysteresis_dB = 1.0;        //!< L1 SINR hysteresis over serving
        uint8_t ltmConsecutiveReports = 2;    //!< consecutive L1 reports to trigger
        Time ltmSwitchDelay = MilliSeconds(25); //!< MAC-CE cell-switch latency

        // ---- Trajectory-predictive CHO (TRIGGER_TRAJECTORY_PREDICTIVE) ----
        Time predictionHorizon = Seconds(8.0);  //!< SINR forecast horizon
        uint8_t predictionMinSamples = 4;       //!< min history for a forecast
        Time minPredictedTos = Seconds(5.0);    //!< min predicted time-of-stay
        double pchoHysteresis_dB = 1.0;         //!< predicted best-server margin

        // ---- Standardized NTN triggers (TIME_T1 / ELEVATION / TA) ----
        Time t1WindowDuration = Seconds(10.0); //!< CondEventT1 window before serving TTE
        double elevationMinDeg = 10.0;         //!< serving-elevation handover floor
        double elevationHystDeg = 2.0;         //!< candidate must clear floor + hyst
        double orbitAltitudeKm = 550.0;        //!< shell altitude for elevation from slant
        Time taServingMax = MilliSeconds(8);   //!< max acceptable serving TA (2*slant/c)
        Time taAdvantage = MilliSeconds(1);    //!< min TA gain to admit a candidate

        // ---- Rel-18 CondEventD2 (TRIGGER_DISTANCE_D2) ----
        double d2Thresh1_m = 600000.0;  //!< serving moving-ref distance must EXCEED this
        double d2Thresh2_m = 500000.0;  //!< candidate moving-ref distance must be BELOW this
        double d2HysteresisLocation_m = 10000.0; //!< hysteresisLocation (TS 38.331)

        /**
         * Rel-17 combination semantics: TS 38.331 configures the T1/D1 (and
         * Rel-18 D2) CondEvents TOGETHER WITH a measurement event (A4), not
         * standalone. The quality precondition (sinr >= qualityThreshold_dB)
         * already implements the A4 entering condition with Thresh =
         * qualityThreshold_dB; setting combineWithA4 = true additionally
         * enforces the A4 time-to-trigger (a3TimeToTrigger): the candidate
         * must satisfy the quality threshold CONTINUOUSLY for the TTT before
         * a T1/D1/D2 admission may fire.
         */
        bool combineWithA4 = false;

        // ---- RACH-less execution (RCHO; orthogonal to the trigger) ----
        bool rachLess = false;                  //!< skip RACH using ephemeris TA
        /**
         * Fallback NTN RACH duration when no slant range is known for the
         * target. When the target's slant range IS known, the RACH cost is
         * computed slant-dependently as 2*slant/c + rachProcessingDelay
         * instead of this constant (a fixed 80 ms misprices the RACH across
         * a LEO pass where the slant RTT varies by several ms).
         */
        Time rachDuration = MilliSeconds(80);
        Time rachProcessingDelay = MilliSeconds(20); //!< gNB/UE RACH processing on top of slant RTT
        Time choExecutionDelay = MilliSeconds(50); //!< RRC reconfig execution time
    };

    /**
     * \brief Counters/latencies for the novel 6G handover mechanisms.
     */
    struct MechanismStats
    {
        uint32_t ltmSwitches = 0;        //!< LTM fast cell switches executed
        uint32_t pchoTriggers = 0;       //!< trajectory-predicted handovers
        uint32_t rachLessExecutions = 0; //!< handovers executed without RACH
        uint32_t rachExecutions = 0;     //!< handovers paying the full RACH
        double lastInterruptionMs = 0.0; //!< interruption of the last handover
        double totalInterruptionMs = 0.0;//!< cumulative interruption
        double lastPreCompTaUs = 0.0;    //!< last ephemeris-pre-computed TA (us)
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
        Time a4MetSince = Seconds(-1.0);  //!< When the A4 quality condition was first met (-1 = not met)

        // ---- Rel-19 conditional LTM state ----
        double l1Filtered_dB = -100.0;    //!< L1 moving-average SINR
        uint8_t l1AboveCount = 0;         //!< consecutive L1 reports above thresh

        // ---- Trajectory-predictive CHO state ----
        std::vector<std::pair<double, double>> sinrHistory; //!< (t_s, sinr_dB)
        double predictedSinr_dB = -100.0; //!< forecast SINR at +horizon
        Time predictedTos = Seconds(0);   //!< predicted time-of-stay

        // ---- RACH-less execution state ----
        double slantRangeM = 0.0;         //!< ephemeris/GNSS slant range to sat
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
     * \brief Update the MEASURED serving-cell SINR (drives the LTM hysteresis
     *        comparison and the trajectory-predicted serving outage).
     */
    void UpdateServingMeasurement(double sinr_dB);

    /**
     * \brief Set the serving cell (initial attach or after an external HO).
     */
    void SetServingCell(uint16_t cellId);

    /**
     * \brief Feed the live ephemeris/GNSS slant range (m) for a candidate's
     *        satellite. Enables RACH-less execution: TA = 2*slant/c is
     *        pre-compensated (TS 38.821 §6.3.3) so the RACH is skipped.
     */
    void UpdateCandidateSlantRange(uint16_t cellId, double slantRangeM);

    /**
     * \brief Counters/latencies of the novel mechanisms (LTM/PCHO/RACH-less).
     */
    MechanismStats GetMechanismStats() const;

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

    /**
     * \brief Distance (m) from the UE to a cell's MOVING reference location
     * (the live ephemeris-derived beam center), used by CondEventD2.
     * \return distance in meters, or -1 when no orbit predictor / snapshot.
     */
    double DistanceToMovingReference(const CandidateInfo& cand) const;

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

    // ---- Novel 6G mechanism state (LTM / PCHO / RACH-less) ----
    double m_servingSinr_dB{-100.0};   //!< latest MEASURED serving SINR
    std::vector<std::pair<double, double>> m_servingSinrHistory; //!< (t_s, sinr)
    MechanismStats m_mechStats;        //!< novel-mechanism counters

    /**
     * \brief Linear-trend forecast of a SINR history at +horizon seconds
     *        (documented stand-in for the PCHO GRU predictor).
     * \return forecast SINR (dB), or the last sample if history is too short.
     */
    double ForecastSinr(const std::vector<std::pair<double, double>>& history,
                        double horizonS) const;

    /// Evaluate the Rel-19 conditional-LTM admission for one candidate.
    void EvaluateLtmConditional(CandidateInfo& cand);
    /// Standardized NTN trigger classes (TIME_T1 / ELEVATION / TIMING_ADVANCE).
    void EvaluateStandardNtnTrigger(CandidateInfo& cand);
    /// Elevation (deg) from an ephemeris/GNSS slant range at the configured
    /// shell altitude (spherical-Earth relation); NaN if range is invalid.
    double ElevationFromSlantDeg(double slantRangeM) const;

    /// Evaluate the trajectory-predictive (PCHO) admission for one candidate.
    void EvaluateTrajectoryPredictive(CandidateInfo& cand);

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
