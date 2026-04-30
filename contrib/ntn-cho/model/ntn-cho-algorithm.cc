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
        it->second.sinr_dB = sinr_dB;
        it->second.gain_dB = gain_dB;
        it->second.lastUpdate = Simulator::Now();
    }
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
