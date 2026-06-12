/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Muhammad Uzair
 */

#include "ntn-cho-algorithm.h"

#include <ns3/boolean.h>
#include <ns3/double.h>
#include <ns3/enum.h>
#include <ns3/log.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnChoAlgorithm");
NS_OBJECT_ENSURE_REGISTERED(NtnChoAlgorithm);

TypeId
NtnChoAlgorithm::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnChoAlgorithm")
            .SetParent<Object>()
            .SetGroupName("NtnCho")
            .AddConstructor<NtnChoAlgorithm>()
            .AddTraceSource("HandoverExecuted",
                            "Fired when a CHO handover is executed",
                            MakeTraceSourceAccessor(&NtnChoAlgorithm::m_handoverExecutedTrace),
                            "ns3::NtnChoAlgorithm::HandoverExecutedTracedCallback")
            .AddTraceSource("HandoverOutcome",
                            "Fired with handover success/failure outcome",
                            MakeTraceSourceAccessor(&NtnChoAlgorithm::m_handoverOutcomeTrace),
                            "ns3::NtnChoAlgorithm::HandoverOutcomeTracedCallback")
            .AddTraceSource("CandidateEval",
                            "Fired when a candidate is evaluated",
                            MakeTraceSourceAccessor(&NtnChoAlgorithm::m_candidateEvalTrace),
                            "ns3::NtnChoAlgorithm::CandidateEvalTracedCallback")
            .AddTraceSource("StateTransition",
                            "Fired on CHO state machine transition",
                            MakeTraceSourceAccessor(&NtnChoAlgorithm::m_stateTransitionTrace),
                            "ns3::NtnChoAlgorithm::StateTransitionTracedCallback")
            .AddAttribute("ThzBeamTrackingThreshold",
                          "Maximum THz pointing error before HO trigger (degrees)",
                          DoubleValue(0.3),
                          MakeDoubleAccessor(&NtnChoAlgorithm::m_thzBeamTrackingThreshold_deg),
                          MakeDoubleChecker<double>(0.0, 10.0))
            .AddAttribute("ThzSnrThreshold",
                          "Minimum THz SNR for candidate admission (dB)",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NtnChoAlgorithm::m_thzSnrThreshold_dB),
                          MakeDoubleChecker<double>(-30.0, 50.0))
            .AddAttribute("EnableMultiBandCho",
                          "Enable Ka-band + THz dual candidate sets",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NtnChoAlgorithm::m_enableMultiBandCho),
                          MakeBooleanChecker())
            .AddAttribute("ThzBeamwidth",
                          "THz beam 3dB beamwidth for TTE calculation (degrees)",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&NtnChoAlgorithm::m_thzBeamwidth_deg),
                          MakeDoubleChecker<double>(0.01, 5.0));
    return tid;
}

NtnChoAlgorithm::NtnChoAlgorithm()
    : m_state(CHO_IDLE),
      m_servingCellId(INVALID_CELL_ID),
      m_ueVelocity(Vector(0, 0, 0)),
      m_lastHoTime(Seconds(0)),
      m_lastSourceCell(INVALID_CELL_ID),
      m_thzBeamTrackingThreshold_deg(0.3),
      m_thzSnrThreshold_dB(0.0),
      m_enableMultiBandCho(false),
      m_thzBeamwidth_deg(0.5)
{
    NS_LOG_FUNCTION(this);
}

NtnChoAlgorithm::~NtnChoAlgorithm()
{
    NS_LOG_FUNCTION(this);
}

void
NtnChoAlgorithm::DoDispose()
{
    NS_LOG_FUNCTION(this);
    StopMonitoring();
    m_tteEstimator = nullptr;
    m_orbitPredictor = nullptr;
    m_candidates.clear();
    Object::DoDispose();
}

void
NtnChoAlgorithm::Configure(ChoConfig config)
{
    NS_LOG_FUNCTION(this);
    m_config = config;
    NS_LOG_INFO("CHO configured: trigger=" << config.triggerType
                << " d1=" << config.d1Threshold_m
                << "m qualityTh=" << config.qualityThreshold_dB
                << "dB tteMin=" << config.tteMinimum.GetSeconds() << "s");
}

NtnChoAlgorithm::ChoConfig
NtnChoAlgorithm::GetConfig() const
{
    return m_config;
}

void
NtnChoAlgorithm::SetTteEstimator(Ptr<NtnTteEstimator> estimator)
{
    m_tteEstimator = estimator;
}

void
NtnChoAlgorithm::SetOrbitPredictor(Ptr<NtnOrbitPredictor> predictor)
{
    m_orbitPredictor = predictor;
}

void
NtnChoAlgorithm::AddCandidateCell(uint16_t cellId, uint32_t satId, uint32_t beamId)
{
    NS_LOG_FUNCTION(this << cellId << satId << beamId);

    if (m_candidates.size() >= m_config.maxCandidates)
    {
        NS_LOG_WARN("Maximum candidates (" << (int)m_config.maxCandidates
                    << ") reached. Ignoring cell " << cellId);
        return;
    }

    CandidateInfo cand;
    cand.cellId = cellId;
    cand.satId = satId;
    cand.beamId = beamId;
    m_candidates[cellId] = cand;

    if (m_state == CHO_IDLE)
    {
        TransitionState(CHO_PREPARED);
    }
}

void
NtnChoAlgorithm::RemoveCandidateCell(uint16_t cellId)
{
    m_candidates.erase(cellId);
}

void
NtnChoAlgorithm::ClearCandidates()
{
    m_candidates.clear();
    if (m_state == CHO_PREPARED || m_state == CHO_CONDITION_MONITORING)
    {
        StopMonitoring();
        TransitionState(CHO_IDLE);
    }
}

void
NtnChoAlgorithm::UpdateMeasurement(uint16_t cellId, double sinr_dB, double gain_dB)
{
    auto it = m_candidates.find(cellId);
    if (it != m_candidates.end())
    {
        CandidateInfo& cand = it->second;
        cand.sinr_dB = sinr_dB;
        cand.gain_dB = gain_dB;
        cand.lastUpdate = Simulator::Now();

        // ---- Rel-19 LTM: L1 moving-average filter over the last K reports ----
        const uint8_t k = std::max<uint8_t>(1, m_config.ltmL1FilterK);
        if (cand.l1Filtered_dB <= -99.0)
        {
            cand.l1Filtered_dB = sinr_dB; // first report seeds the filter
        }
        else
        {
            const double alpha = 1.0 / static_cast<double>(k);
            cand.l1Filtered_dB = (1.0 - alpha) * cand.l1Filtered_dB + alpha * sinr_dB;
        }

        // ---- PCHO: bounded SINR history for the trajectory forecast ----
        cand.sinrHistory.emplace_back(Simulator::Now().GetSeconds(), sinr_dB);
        if (cand.sinrHistory.size() > 32)
        {
            cand.sinrHistory.erase(cand.sinrHistory.begin());
        }
    }
    if (cellId == m_servingCellId)
    {
        UpdateServingMeasurement(sinr_dB);
    }
}

void
NtnChoAlgorithm::UpdateServingMeasurement(double sinr_dB)
{
    m_servingSinr_dB = sinr_dB;
    m_servingSinrHistory.emplace_back(Simulator::Now().GetSeconds(), sinr_dB);
    if (m_servingSinrHistory.size() > 32)
    {
        m_servingSinrHistory.erase(m_servingSinrHistory.begin());
    }
}

void
NtnChoAlgorithm::SetServingCell(uint16_t cellId)
{
    m_servingCellId = cellId;
}

void
NtnChoAlgorithm::UpdateCandidateSlantRange(uint16_t cellId, double slantRangeM)
{
    auto it = m_candidates.find(cellId);
    if (it != m_candidates.end())
    {
        it->second.slantRangeM = slantRangeM;
    }
}

NtnChoAlgorithm::MechanismStats
NtnChoAlgorithm::GetMechanismStats() const
{
    return m_mechStats;
}

double
NtnChoAlgorithm::ForecastSinr(const std::vector<std::pair<double, double>>& history,
                              double horizonS) const
{
    // Least-squares linear trend over the history window — the documented
    // stand-in for the PCHO GRU trajectory predictor (Yang et al.). With a
    // LEO pass the SINR trend over a few seconds is locally linear, so the
    // trend forecast captures the approach/recede dynamics the GRU learns.
    const size_t n = history.size();
    if (n < 2)
    {
        return n == 1 ? history.back().second : -100.0;
    }
    double sumT = 0, sumS = 0, sumTT = 0, sumTS = 0;
    for (const auto& [t, s] : history)
    {
        sumT += t;
        sumS += s;
        sumTT += t * t;
        sumTS += t * s;
    }
    const double denom = n * sumTT - sumT * sumT;
    if (std::abs(denom) < 1e-9)
    {
        return history.back().second;
    }
    const double slope = (n * sumTS - sumT * sumS) / denom;
    const double intercept = (sumS - slope * sumT) / n;
    const double tF = Simulator::Now().GetSeconds() + horizonS;
    return intercept + slope * tF;
}

double
NtnChoAlgorithm::ElevationFromSlantDeg(double slantRangeM) const
{
    if (slantRangeM <= 0.0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    // Spherical-Earth relation for a circular shell at the configured
    // altitude: sin(elev) = ((R+h)^2 - R^2 - d^2) / (2 R d).
    constexpr double kRe = 6371e3;
    const double rs = kRe + m_config.orbitAltitudeKm * 1e3;
    const double d = slantRangeM;
    const double s = (rs * rs - kRe * kRe - d * d) / (2.0 * kRe * d);
    if (s < -1.0 || s > 1.0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::asin(s) * 180.0 / M_PI;
}

void
NtnChoAlgorithm::EvaluateStandardNtnTrigger(CandidateInfo& cand)
{
    cand.admitted = false;
    cand.tte = Seconds(0);
    if (cand.sinr_dB < m_config.qualityThreshold_dB)
    {
        // The quality precondition IS the A4 entering condition (Thresh =
        // qualityThreshold_dB); leaving it resets the A4 time-to-trigger.
        cand.a4MetSince = Seconds(-1.0);
        return; // every class keeps the radio-quality precondition
    }
    if (cand.a4MetSince < Seconds(0))
    {
        cand.a4MetSince = Simulator::Now();
    }
    if (m_config.combineWithA4 &&
        Simulator::Now() - cand.a4MetSince < m_config.a3TimeToTrigger)
    {
        // Rel-17 combination semantics: the A4 leg must hold for the TTT
        // before a T1/D1/D2 CondEvent may admit the candidate.
        return;
    }
    constexpr double kC = 299792458.0;

    switch (m_config.triggerType)
    {
    case TRIGGER_TIME_T1: {
        // CondEventT1: the ephemeris-scheduled window opens when the SERVING
        // cell's remaining time-of-service drops inside t1WindowDuration.
        if (!m_tteEstimator)
        {
            return;
        }
        auto servingIt = m_candidates.find(m_servingCellId);
        if (servingIt == m_candidates.end())
        {
            return;
        }
        const auto servingTte = m_tteEstimator->ComputeTte(m_uePosition,
                                                           m_ueVelocity,
                                                           servingIt->second.satId,
                                                           servingIt->second.beamId,
                                                           m_config.gainThreshold_dB);
        cand.admitted = (servingTte.tte > Seconds(0) &&
                         servingTte.tte <= m_config.t1WindowDuration);
        break;
    }
    case TRIGGER_ELEVATION: {
        auto servingIt = m_candidates.find(m_servingCellId);
        if (servingIt == m_candidates.end())
        {
            return;
        }
        const double servingElev = ElevationFromSlantDeg(servingIt->second.slantRangeM);
        const double candElev = ElevationFromSlantDeg(cand.slantRangeM);
        cand.admitted = (!std::isnan(servingElev) && !std::isnan(candElev) &&
                         servingElev < m_config.elevationMinDeg &&
                         candElev >= m_config.elevationMinDeg + m_config.elevationHystDeg);
        break;
    }
    case TRIGGER_TIMING_ADVANCE: {
        auto servingIt = m_candidates.find(m_servingCellId);
        if (servingIt == m_candidates.end() || cand.slantRangeM <= 0.0 ||
            servingIt->second.slantRangeM <= 0.0)
        {
            return;
        }
        const Time taServing = Seconds(2.0 * servingIt->second.slantRangeM / kC);
        const Time taCand = Seconds(2.0 * cand.slantRangeM / kC);
        cand.admitted = (taServing > m_config.taServingMax) ||
                        (taServing - taCand >= m_config.taAdvantage);
        break;
    }
    case TRIGGER_DISTANCE_D2: {
        // Rel-18 CondEventD2 (TS 38.331 §5.5.4.15a): both reference
        // locations MOVE with the satellites (live ephemeris beam centers).
        // Entering condition: Ml1 - Hys > Thresh1 AND Ml2 + Hys < Thresh2.
        auto servingIt = m_candidates.find(m_servingCellId);
        if (servingIt == m_candidates.end())
        {
            return;
        }
        const double dServing = DistanceToMovingReference(servingIt->second);
        const double dCand = DistanceToMovingReference(cand);
        if (dServing < 0.0 || dCand < 0.0)
        {
            return; // no ephemeris for one of the moving references
        }
        const double hys = m_config.d2HysteresisLocation_m;
        cand.admitted = (dServing - hys > m_config.d2Thresh1_m) &&
                        (dCand + hys < m_config.d2Thresh2_m);
        break;
    }
    default:
        break;
    }
}

void
NtnChoAlgorithm::EvaluateLtmConditional(CandidateInfo& cand)
{
    // Rel-19 conditional LTM: the L1-filtered candidate measurement must
    // exceed the serving L1 quality + hysteresis for N consecutive reports
    // (LTM speed), AND the candidate must pass the ephemeris TTE stability
    // filter (CHO reliability) when an estimator is wired.
    const bool above = cand.l1Filtered_dB >= m_servingSinr_dB + m_config.ltmHysteresis_dB;
    cand.l1AboveCount = above ? std::min<uint8_t>(cand.l1AboveCount + 1, 250) : 0;

    bool tteOk = true;
    if (m_tteEstimator)
    {
        auto tteResult = m_tteEstimator->ComputeTte(m_uePosition,
                                                    m_ueVelocity,
                                                    cand.satId,
                                                    cand.beamId,
                                                    m_config.gainThreshold_dB);
        cand.tte = tteResult.tte;
        // Apply the CHO-reliability TTE filter only when the estimator could
        // actually produce an estimate (ephemeris registered for this satId);
        // a zero TTE means "unknown", which must not veto the L1 trigger.
        tteOk = (cand.tte == Seconds(0)) || (cand.tte >= m_config.tteMinimum);
    }
    cand.admitted = (cand.l1AboveCount >= m_config.ltmConsecutiveReports) && tteOk &&
                    (cand.sinr_dB >= m_config.qualityThreshold_dB);
}

void
NtnChoAlgorithm::EvaluateTrajectoryPredictive(CandidateInfo& cand)
{
    // PCHO: forecast both trajectories at +horizon; admit a candidate when
    // the serving cell is PREDICTED to fall below quality within the horizon
    // while the candidate is predicted to stay above it, and the candidate's
    // predicted time-of-stay (capped by the ephemeris TTE) clears the floor.
    const double h = m_config.predictionHorizon.GetSeconds();
    if (cand.sinrHistory.size() < m_config.predictionMinSamples ||
        m_servingSinrHistory.size() < m_config.predictionMinSamples)
    {
        cand.admitted = false;
        return;
    }
    const double servingPredicted = ForecastSinr(m_servingSinrHistory, h);
    cand.predictedSinr_dB = ForecastSinr(cand.sinrHistory, h);

    // Predicted time-of-stay: time until the candidate's forecast trend
    // crosses the quality threshold (clamped to [0, 10*h]), fused with the
    // ephemeris TTE when available.
    double tosS = 10.0 * h;
    const double now = Simulator::Now().GetSeconds();
    const double slope = (cand.predictedSinr_dB - cand.sinr_dB) / std::max(1e-3, h);
    if (slope < -1e-6)
    {
        tosS = std::max(0.0, (cand.sinr_dB - m_config.qualityThreshold_dB) / -slope);
    }
    if (m_tteEstimator)
    {
        auto tteResult = m_tteEstimator->ComputeTte(m_uePosition,
                                                    m_ueVelocity,
                                                    cand.satId,
                                                    cand.beamId,
                                                    m_config.gainThreshold_dB);
        cand.tte = tteResult.tte;
        if (cand.tte > Seconds(0)) // zero = no ephemeris registered (unknown)
        {
            tosS = std::min(tosS, cand.tte.GetSeconds());
        }
    }
    cand.predictedTos = Seconds(tosS);
    (void)now;

    // PCHO fires proactively on EITHER predicted condition (Yang et al.):
    //  (a) predicted serving outage within the horizon, or
    //  (b) predicted best-server change — the candidate trajectory is forecast
    //      to exceed the serving trajectory by the hysteresis margin.
    const bool servingWillDegrade = servingPredicted < m_config.qualityThreshold_dB;
    const bool bestServerChange =
        cand.predictedSinr_dB >= servingPredicted + m_config.pchoHysteresis_dB;
    const bool candidateWillHold = cand.predictedSinr_dB >= m_config.qualityThreshold_dB;
    cand.admitted = (servingWillDegrade || bestServerChange) && candidateWillHold &&
                    (cand.predictedTos >= m_config.minPredictedTos) &&
                    (cand.sinr_dB >= m_config.qualityThreshold_dB);
}

void
NtnChoAlgorithm::StartMonitoring(GeoCoordinate uePosition, Vector ueVelocity)
{
    NS_LOG_FUNCTION(this);
    m_uePosition = uePosition;
    m_ueVelocity = ueVelocity;

    if (m_state == CHO_PREPARED || m_state == CHO_CONDITION_MONITORING)
    {
        TransitionState(CHO_CONDITION_MONITORING);
        DoEvaluateConditions();
    }
}

void
NtnChoAlgorithm::StopMonitoring()
{
    NS_LOG_FUNCTION(this);
    if (m_monitorEvent.IsPending())
    {
        Simulator::Cancel(m_monitorEvent);
    }
}

void
NtnChoAlgorithm::DoEvaluateConditions()
{
    if (m_state != CHO_CONDITION_MONITORING)
    {
        return;
    }

    EvaluateConditions();

    // Schedule next evaluation
    m_monitorEvent = Simulator::Schedule(m_config.conditionMonitorPeriod,
                                         &NtnChoAlgorithm::DoEvaluateConditions,
                                         this);
}

void
NtnChoAlgorithm::EvaluateConditions()
{
    NS_LOG_FUNCTION(this);

    for (auto& [cellId, cand] : m_candidates)
    {
        // The serving cell is tracked for hysteresis/outage prediction but is
        // never admitted as its own handover target.
        if (cellId == m_servingCellId)
        {
            cand.admitted = false;
            continue;
        }

        // ---- Novel 6G triggers: measurement-driven, not D1-gated ----
        if (m_config.triggerType == TRIGGER_LTM_CONDITIONAL)
        {
            EvaluateLtmConditional(cand);
            m_candidateEvalTrace(cellId, cand.sinr_dB, cand.tte, cand.admitted);
            if (cand.admitted && !m_admitCallback.IsNull())
            {
                m_admitCallback(cellId, cand.sinr_dB, cand.tte);
            }
            continue;
        }
        if (m_config.triggerType == TRIGGER_TRAJECTORY_PREDICTIVE)
        {
            EvaluateTrajectoryPredictive(cand);
            m_candidateEvalTrace(cellId, cand.sinr_dB, cand.tte, cand.admitted);
            if (cand.admitted && !m_admitCallback.IsNull())
            {
                m_admitCallback(cellId, cand.sinr_dB, cand.tte);
            }
            continue;
        }
        if (m_config.triggerType == TRIGGER_TIME_T1 ||
            m_config.triggerType == TRIGGER_ELEVATION ||
            m_config.triggerType == TRIGGER_TIMING_ADVANCE ||
            m_config.triggerType == TRIGGER_DISTANCE_D2)
        {
            EvaluateStandardNtnTrigger(cand);
            m_candidateEvalTrace(cellId, cand.sinr_dB, cand.tte, cand.admitted);
            if (cand.admitted && !m_admitCallback.IsNull())
            {
                m_admitCallback(cellId, cand.sinr_dB, cand.tte);
            }
            continue;
        }

        // Step 1: Check D1 condition
        bool d1Now = CheckD1Condition(cand);

        if (d1Now && !cand.d1Met)
        {
            cand.d1Met = true;
            cand.d1MetSince = Simulator::Now();
            NS_LOG_DEBUG("D1 condition MET for cell " << cellId);
        }
        else if (!d1Now)
        {
            cand.d1Met = false;
            cand.admitted = false;
        }

        // Step 2: If D1 met and quality sufficient, compute TTE
        if (cand.d1Met && cand.sinr_dB >= m_config.qualityThreshold_dB)
        {
            if (m_tteEstimator && m_config.triggerType == TRIGGER_TTE_AWARE)
            {
                auto tteResult = m_tteEstimator->ComputeTte(
                    m_uePosition, m_ueVelocity,
                    cand.satId, cand.beamId,
                    m_config.gainThreshold_dB);

                cand.tte = tteResult.tte;

                // Admit if TTE >= minimum
                cand.admitted = (cand.tte >= m_config.tteMinimum);
            }
            else if (m_config.triggerType == TRIGGER_LOCATION_D1)
            {
                // Baseline: admit based on D1 + quality only (no TTE)
                cand.admitted = true;
                cand.tte = Seconds(0); // Unknown
            }
            else if (m_config.triggerType == TRIGGER_EVENT_A3)
            {
                // A3: admit based purely on SINR comparison
                cand.admitted = true;
                cand.tte = Seconds(0);
            }
            else if (m_config.triggerType == TRIGGER_THZ_BEAM_QUALITY)
            {
                // THz beam quality: check pointing error and THz SNR
                bool thzOk = EvaluateThzBeamQuality(cellId);
                if (thzOk)
                {
                    double thzTte = ComputeThzBeamTte(cellId);
                    cand.tte = Seconds(thzTte);
                    cand.admitted = (cand.tte >= m_config.tteMinimum);
                }
                else
                {
                    cand.admitted = false;
                    cand.tte = Seconds(0);
                }
            }
        }
        else if (cand.d1Met && cand.sinr_dB < m_config.qualityThreshold_dB)
        {
            cand.admitted = false;
        }

        // Fire trace
        m_candidateEvalTrace(cellId, cand.sinr_dB, cand.tte, cand.admitted);

        if (cand.admitted && !m_admitCallback.IsNull())
        {
            m_admitCallback(cellId, cand.sinr_dB, cand.tte);
        }
    }
}

uint16_t
NtnChoAlgorithm::SelectBestCandidate() const
{
    NS_LOG_FUNCTION(this);

    // ================================================================
    // THE NOVEL TTE-AWARE CANDIDATE SELECTION ALGORITHM
    //
    // Input:  Set of candidate cells with measured SINR and computed TTE
    // Output: Best candidate cell ID
    //
    // 1. Filter: remove candidates with SINR < Q_threshold
    // 2. Filter: remove candidates with TTE < TTE_minimum
    // 3. Select: candidate with maximum TTE
    // 4. Tie-break: if multiple within epsilon, pick highest SINR
    // ================================================================

    // ---- Novel 6G triggers have their own selection rules ----
    if (m_config.triggerType == TRIGGER_LTM_CONDITIONAL)
    {
        // LTM: fastest-quality cell — max L1-filtered SINR among admitted.
        const CandidateInfo* bestLtm = nullptr;
        for (const auto& [cellId, info] : m_candidates)
        {
            if (info.admitted && (!bestLtm || info.l1Filtered_dB > bestLtm->l1Filtered_dB))
            {
                bestLtm = &info;
            }
        }
        return bestLtm ? bestLtm->cellId : INVALID_CELL_ID;
    }
    if (m_config.triggerType == TRIGGER_TRAJECTORY_PREDICTIVE)
    {
        // PCHO: max predicted time-of-stay; tie-break on predicted SINR.
        const CandidateInfo* bestP = nullptr;
        for (const auto& [cellId, info] : m_candidates)
        {
            if (!info.admitted)
            {
                continue;
            }
            if (!bestP || info.predictedTos > bestP->predictedTos ||
                (info.predictedTos == bestP->predictedTos &&
                 info.predictedSinr_dB > bestP->predictedSinr_dB))
            {
                bestP = &info;
            }
        }
        return bestP ? bestP->cellId : INVALID_CELL_ID;
    }

    if (m_config.triggerType == TRIGGER_TIME_T1 ||
        m_config.triggerType == TRIGGER_ELEVATION ||
        m_config.triggerType == TRIGGER_TIMING_ADVANCE)
    {
        // The admit set already encodes the standardized condition; take the
        // strongest measured candidate.
        const CandidateInfo* bestStd = nullptr;
        for (const auto& [cellId, info] : m_candidates)
        {
            if (info.admitted && (!bestStd || info.sinr_dB > bestStd->sinr_dB))
            {
                bestStd = &info;
            }
        }
        return bestStd ? bestStd->cellId : INVALID_CELL_ID;
    }

    std::vector<const CandidateInfo*> admissible;

    for (const auto& [cellId, info] : m_candidates)
    {
        if (info.admitted &&
            info.d1Met &&
            info.sinr_dB >= m_config.qualityThreshold_dB &&
            info.tte >= m_config.tteMinimum)
        {
            admissible.push_back(&info);
        }
    }

    if (admissible.empty())
    {
        NS_LOG_DEBUG("No admissible candidates for TTE-aware selection");
        return INVALID_CELL_ID;
    }

    // Sort by TTE descending
    std::sort(admissible.begin(), admissible.end(),
              [](const CandidateInfo* a, const CandidateInfo* b) {
                  return a->tte > b->tte;
              });

    // Tie-breaking: within epsilon of the best TTE, pick highest SINR
    const CandidateInfo* best = admissible[0];
    for (size_t i = 1; i < admissible.size(); i++)
    {
        if ((best->tte - admissible[i]->tte) <= m_config.tteEpsilon &&
            admissible[i]->sinr_dB > best->sinr_dB)
        {
            best = admissible[i];
        }
    }

    NS_LOG_INFO("TTE-aware selection: cell " << best->cellId
                << " (TTE=" << best->tte.GetSeconds()
                << "s, SINR=" << best->sinr_dB << " dB)");
    return best->cellId;
}

uint16_t
NtnChoAlgorithm::SelectBaselineA3(double servingSinr_dB) const
{
    NS_LOG_FUNCTION(this << servingSinr_dB);

    uint16_t bestCell = INVALID_CELL_ID;
    double bestSinr = -200.0;

    for (const auto& [cellId, info] : m_candidates)
    {
        // A3: neighbor SINR > serving SINR + offset
        if (info.sinr_dB > (servingSinr_dB + m_config.a3Offset_dB) &&
            info.sinr_dB > bestSinr)
        {
            bestSinr = info.sinr_dB;
            bestCell = cellId;
        }
    }

    return bestCell;
}

uint16_t
NtnChoAlgorithm::SelectBaselineLocationOnly() const
{
    NS_LOG_FUNCTION(this);

    uint16_t bestCell = INVALID_CELL_ID;
    double bestSinr = -200.0;

    for (const auto& [cellId, info] : m_candidates)
    {
        if (info.d1Met && info.sinr_dB >= m_config.qualityThreshold_dB &&
            info.sinr_dB > bestSinr)
        {
            bestSinr = info.sinr_dB;
            bestCell = cellId;
        }
    }

    return bestCell;
}

void
NtnChoAlgorithm::ExecuteHandover(uint16_t targetCellId)
{
    NS_LOG_FUNCTION(this << targetCellId);

    if (targetCellId == INVALID_CELL_ID)
    {
        NS_LOG_WARN("Cannot execute HO to invalid cell");
        m_handoverOutcomeTrace(targetCellId, false, "InvalidTarget");
        return;
    }

    // Record Time-of-Stay from last handover
    Time tos = Seconds(0);
    if (m_lastHoTime > Seconds(0))
    {
        tos = Simulator::Now() - m_lastHoTime;
    }

    uint16_t sourceCell = m_servingCellId;

    // Check for ping-pong
    bool isPingPong = (targetCellId == m_lastSourceCell &&
                       tos < Seconds(5.0)); // 5s ping-pong threshold

    TransitionState(CHO_EXECUTING);

    // ---- Novel 6G execution accounting (LTM fast switch + RACH-less) ----
    // Base execution latency: Rel-19 LTM = MAC-CE cell switch (tens of ms);
    // classic CHO = RRC reconfiguration execution.
    const bool isLtm = (m_config.triggerType == TRIGGER_LTM_CONDITIONAL);
    double interruptionMs = (isLtm ? m_config.ltmSwitchDelay : m_config.choExecutionDelay)
                                .GetMilliSeconds();
    // RACH-less (RCHO): with UE GNSS + satellite ephemeris (SIB19) the target
    // TA is pre-compensated (TS 38.821 §6.3.3), so the RACH is skipped;
    // otherwise the NTN RACH (incl. slant RTT) is paid.
    auto targetIt = m_candidates.find(targetCellId);
    const bool haveSlant = (targetIt != m_candidates.end() && targetIt->second.slantRangeM > 0.0);
    if (m_config.rachLess && haveSlant)
    {
        m_mechStats.lastPreCompTaUs =
            2.0 * targetIt->second.slantRangeM / 299792458.0 * 1e6; // round-trip TA
        ++m_mechStats.rachLessExecutions;
    }
    else
    {
        // Slant-dependent RACH cost: the 4-step RACH pays at least one slant
        // round-trip plus processing; the constant rachDuration misprices it
        // across a LEO pass (slant RTT varies by several ms). Fall back to
        // the constant only when the target's slant range is unknown.
        if (haveSlant)
        {
            const double rachMs =
                2.0 * targetIt->second.slantRangeM / 299792458.0 * 1e3 +
                m_config.rachProcessingDelay.GetMilliSeconds();
            interruptionMs += rachMs;
        }
        else
        {
            interruptionMs += m_config.rachDuration.GetMilliSeconds();
        }
        ++m_mechStats.rachExecutions;
    }
    if (isLtm)
    {
        ++m_mechStats.ltmSwitches;
    }
    if (m_config.triggerType == TRIGGER_TRAJECTORY_PREDICTIVE)
    {
        ++m_mechStats.pchoTriggers;
    }
    m_mechStats.lastInterruptionMs = interruptionMs;
    m_mechStats.totalInterruptionMs += interruptionMs;

    // Fire execution trace
    m_handoverExecutedTrace(sourceCell, targetCellId, tos);

    // Start T304 timer
    m_t304Event = Simulator::Schedule(m_config.t304Timer, [this, targetCellId]() {
        // T304 expired - handover failure
        NS_LOG_WARN("T304 expired for HO to cell " << targetCellId);
        m_handoverOutcomeTrace(targetCellId, false, "T304Expired");
        TransitionState(CHO_IDLE);
    });

    // Execute via callback
    if (!m_hoCallback.IsNull())
    {
        m_hoCallback(sourceCell, targetCellId);
    }

    // Update state
    m_lastSourceCell = sourceCell;
    m_lastHoTime = Simulator::Now();
    m_servingCellId = targetCellId;

    // Cancel T304 (successful execution)
    if (m_t304Event.IsPending())
    {
        Simulator::Cancel(m_t304Event);
    }

    if (isPingPong)
    {
        m_handoverOutcomeTrace(targetCellId, true, "PingPong");
    }
    else
    {
        m_handoverOutcomeTrace(targetCellId, true, "Success");
    }

    TransitionState(CHO_COMPLETED);

    // Reset to monitoring for next CHO cycle
    m_candidates.clear();
    TransitionState(CHO_IDLE);
}

void
NtnChoAlgorithm::CancelHandover()
{
    NS_LOG_FUNCTION(this);
    StopMonitoring();
    if (m_t304Event.IsPending())
    {
        Simulator::Cancel(m_t304Event);
    }
    TransitionState(CHO_IDLE);
}

NtnChoAlgorithm::ChoState
NtnChoAlgorithm::GetState() const
{
    return m_state;
}

std::map<uint16_t, NtnChoAlgorithm::CandidateInfo>
NtnChoAlgorithm::GetCandidates() const
{
    return m_candidates;
}

uint32_t
NtnChoAlgorithm::GetNumAdmittedCandidates() const
{
    uint32_t count = 0;
    for (const auto& [id, info] : m_candidates)
    {
        if (info.admitted)
        {
            count++;
        }
    }
    return count;
}

void
NtnChoAlgorithm::SetHandoverExecutionCallback(HandoverExecutionCallback cb)
{
    m_hoCallback = cb;
}

void
NtnChoAlgorithm::SetCandidateAdmittedCallback(CandidateAdmittedCallback cb)
{
    m_admitCallback = cb;
}

bool
NtnChoAlgorithm::CheckD1Condition(const CandidateInfo& cand) const
{
    if (!m_orbitPredictor)
    {
        return false;
    }

    auto snap = m_orbitPredictor->GetBeamSnapshot(cand.satId, cand.beamId, m_uePosition);

    // D1: UE within d1Threshold of beam center
    Vector ueCart = m_uePosition.ToVector();
    Vector bcCart = snap.beamCenter.ToVector();
    double dx = ueCart.x - bcCart.x;
    double dy = ueCart.y - bcCart.y;
    double dz = ueCart.z - bcCart.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    return (dist <= m_config.d1Threshold_m);
}

double
NtnChoAlgorithm::DistanceToMovingReference(const CandidateInfo& cand) const
{
    if (!m_orbitPredictor)
    {
        return -1.0;
    }
    auto snap = m_orbitPredictor->GetBeamSnapshot(cand.satId, cand.beamId, m_uePosition);
    const Vector ueCart = m_uePosition.ToVector();
    const Vector refCart = snap.beamCenter.ToVector();
    const double dx = ueCart.x - refCart.x;
    const double dy = ueCart.y - refCart.y;
    const double dz = ueCart.z - refCart.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void
NtnChoAlgorithm::TransitionState(ChoState newState)
{
    if (m_state != newState)
    {
        NS_LOG_INFO("CHO state: " << m_state << " -> " << newState);
        ChoState old = m_state;
        m_state = newState;
        m_stateTransitionTrace(old, newState);
    }
}

bool
NtnChoAlgorithm::EvaluateThzBeamQuality(uint32_t candidateIdx) const
{
    NS_LOG_FUNCTION(this << candidateIdx);

    auto it = m_candidates.find(static_cast<uint16_t>(candidateIdx));
    if (it == m_candidates.end())
    {
        return false;
    }

    const CandidateInfo& cand = it->second;

    if (!m_orbitPredictor)
    {
        return false;
    }

    // Get beam snapshot to determine pointing error
    auto snap = m_orbitPredictor->GetBeamSnapshot(cand.satId, cand.beamId, m_uePosition);
    Vector ueCart = m_uePosition.ToVector();
    Vector bcCart = snap.beamCenter.ToVector();
    double dx = ueCart.x - bcCart.x;
    double dy = ueCart.y - bcCart.y;
    double dz = ueCart.z - bcCart.z;
    double offsetDist = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Convert linear offset to angular pointing error (approximate)
    // pointingError_deg ~ atan(offset / altitude) in degrees
    double altitudeM = snap.satellitePosition.GetAltitude();
    double pointingError_deg = std::atan2(offsetDist, altitudeM) * 180.0 / M_PI;

    // Check pointing error threshold
    if (pointingError_deg >= m_thzBeamTrackingThreshold_deg)
    {
        NS_LOG_DEBUG("THz beam quality FAIL for cell " << candidateIdx
                     << ": pointing error=" << pointingError_deg
                     << " deg >= threshold=" << m_thzBeamTrackingThreshold_deg << " deg");
        return false;
    }

    // Check THz SNR threshold
    if (cand.sinr_dB < m_thzSnrThreshold_dB)
    {
        NS_LOG_DEBUG("THz beam quality FAIL for cell " << candidateIdx
                     << ": THz SNR=" << cand.sinr_dB
                     << " dB < threshold=" << m_thzSnrThreshold_dB << " dB");
        return false;
    }

    NS_LOG_DEBUG("THz beam quality OK for cell " << candidateIdx
                 << ": pointing error=" << pointingError_deg
                 << " deg, SNR=" << cand.sinr_dB << " dB");
    return true;
}

double
NtnChoAlgorithm::ComputeThzBeamTte(uint32_t candidateIdx) const
{
    NS_LOG_FUNCTION(this << candidateIdx);

    auto it = m_candidates.find(static_cast<uint16_t>(candidateIdx));
    if (it == m_candidates.end() || !m_orbitPredictor)
    {
        return 0.0;
    }

    const CandidateInfo& cand = it->second;

    // Get satellite snapshot
    auto snap = m_orbitPredictor->GetBeamSnapshot(cand.satId, cand.beamId, m_uePosition);
    double altitudeKm = snap.satellitePosition.GetAltitude() / 1000.0;

    // THz beam coverage radius = altitude * tan(beamwidth/2)
    double halfBeamRad = (m_thzBeamwidth_deg / 2.0) * M_PI / 180.0;
    double coverageRadiusKm = altitudeKm * std::tan(halfBeamRad);

    // Compute current pointing error
    Vector ueCart = m_uePosition.ToVector();
    Vector bcCart = snap.beamCenter.ToVector();
    double dx = ueCart.x - bcCart.x;
    double dy = ueCart.y - bcCart.y;
    double dz = ueCart.z - bcCart.z;
    double offsetDist = std::sqrt(dx * dx + dy * dy + dz * dz);
    double altitudeM = snap.satellitePosition.GetAltitude();
    double pointingError_deg = std::atan2(offsetDist, altitudeM) * 180.0 / M_PI;

    // Effective radius reduced by pointing error
    double effectiveRadiusKm = coverageRadiusKm *
        (1.0 - pointingError_deg / m_thzBeamwidth_deg);
    if (effectiveRadiusKm < 0.0)
    {
        effectiveRadiusKm = 0.0;
    }

    // Ground track velocity (approximate from satellite velocity)
    // v_ground = v_sat * R_earth / (R_earth + altitude)
    static constexpr double R_EARTH_KM = 6371.0;
    double orbitalVelocityKm_s = std::sqrt(398600.4418 / (R_EARTH_KM + altitudeKm));
    double groundTrackVelocityKm_s = orbitalVelocityKm_s * R_EARTH_KM /
        (R_EARTH_KM + altitudeKm);

    if (groundTrackVelocityKm_s <= 0.0)
    {
        return 0.0;
    }

    // TTE = coverage diameter / ground track velocity
    double tteSec = (2.0 * effectiveRadiusKm) / groundTrackVelocityKm_s;

    NS_LOG_DEBUG("THz beam TTE for cell " << candidateIdx
                 << ": altitude=" << altitudeKm << " km"
                 << ", coverage_r=" << coverageRadiusKm << " km"
                 << ", effective_r=" << effectiveRadiusKm << " km"
                 << ", v_ground=" << groundTrackVelocityKm_s << " km/s"
                 << ", TTE=" << tteSec << " s");
    return tteSec;
}

void
NtnChoAlgorithm::PrepareMultiBandCandidates()
{
    NS_LOG_FUNCTION(this);

    if (!m_enableMultiBandCho)
    {
        NS_LOG_DEBUG("Multi-band CHO not enabled");
        return;
    }

    for (auto& [cellId, cand] : m_candidates)
    {
        // Evaluate THz quality
        bool thzGood = EvaluateThzBeamQuality(cellId);

        // Evaluate Ka-band quality (standard D1 + SINR check)
        bool kaGood = (CheckD1Condition(cand) &&
                       cand.sinr_dB >= m_config.qualityThreshold_dB);

        if (thzGood)
        {
            // THz primary: compute THz TTE, Ka-band as fallback
            double thzTte = ComputeThzBeamTte(cellId);
            cand.tte = Seconds(thzTte);
            cand.admitted = (cand.tte >= m_config.tteMinimum);
            NS_LOG_INFO("Cell " << cellId
                        << ": THz PRIMARY (TTE=" << thzTte << "s)"
                        << ", Ka-band FALLBACK=" << (kaGood ? "available" : "unavailable"));
        }
        else if (kaGood)
        {
            // Ka-band only: use standard TTE computation
            if (m_tteEstimator)
            {
                auto tteResult = m_tteEstimator->ComputeTte(
                    m_uePosition, m_ueVelocity,
                    cand.satId, cand.beamId,
                    m_config.gainThreshold_dB);
                cand.tte = tteResult.tte;
                cand.admitted = (cand.tte >= m_config.tteMinimum);
            }
            else
            {
                cand.admitted = true;
                cand.tte = Seconds(0);
            }
            NS_LOG_INFO("Cell " << cellId
                        << ": Ka-band ONLY (TTE=" << cand.tte.GetSeconds() << "s)");
        }
        else
        {
            cand.admitted = false;
            cand.tte = Seconds(0);
            NS_LOG_DEBUG("Cell " << cellId << ": neither THz nor Ka-band available");
        }

        m_candidateEvalTrace(cellId, cand.sinr_dB, cand.tte, cand.admitted);
    }
}

} // namespace ns3
