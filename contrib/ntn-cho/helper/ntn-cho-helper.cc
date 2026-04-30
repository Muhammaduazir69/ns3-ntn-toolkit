/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Muhammad Uzair
 */

#include "ntn-cho-helper.h"

#include <ns3/log.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <iomanip>
#include <numeric>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnChoHelper");
NS_OBJECT_ENSURE_REGISTERED(NtnChoHelper);

TypeId
NtnChoHelper::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnChoHelper")
            .SetParent<Object>()
            .SetGroupName("NtnCho")
            .AddConstructor<NtnChoHelper>();
    return tid;
}

NtnChoHelper::NtnChoHelper()
    : m_scenario(NtnMeasurementModel::NTN_SUBURBAN),
      m_carrierFreqHz(2.0e9),
      m_bandwidthHz(30.0e6),
      m_satTxPower_dBm(40.0),
      m_tracesEnabled(false)
{
    NS_LOG_FUNCTION(this);
    // Default CHO config
    m_choConfig.triggerType = NtnChoAlgorithm::TRIGGER_TTE_AWARE;
    m_choConfig.d1Threshold_m = 50000.0;
    m_choConfig.qualityThreshold_dB = -3.0;
    m_choConfig.tteMinimum = Seconds(15.0);
    m_choConfig.conditionMonitorPeriod = Seconds(1.0);
    m_choConfig.maxCandidates = 4;
    m_choConfig.t304Timer = Seconds(2.0);
    m_choConfig.gainThreshold_dB = -3.0;
    m_choConfig.tteEpsilon = Seconds(2.0);
}

NtnChoHelper::~NtnChoHelper()
{
    NS_LOG_FUNCTION(this);
}

void
NtnChoHelper::DoDispose()
{
    if (m_hoTraceFile.is_open()) m_hoTraceFile.close();
    if (m_measurementTraceFile.is_open()) m_measurementTraceFile.close();
    if (m_tteTraceFile.is_open()) m_tteTraceFile.close();
    m_orbitPredictor = nullptr;
    m_tteEstimator = nullptr;
    m_measurementModel = nullptr;
    Object::DoDispose();
}

// ========== Configuration ==========

void NtnChoHelper::SetNtnScenario(NtnMeasurementModel::NtnScenario scenario) { m_scenario = scenario; }
void NtnChoHelper::SetChoTriggerType(NtnChoAlgorithm::TriggerType trigger) { m_choConfig.triggerType = trigger; }
void NtnChoHelper::SetCarrierFrequency(double freqHz) { m_carrierFreqHz = freqHz; }
void NtnChoHelper::SetBandwidth(double bwHz) { m_bandwidthHz = bwHz; }
void NtnChoHelper::SetTteMinimum(Time minTte) { m_choConfig.tteMinimum = minTte; }
void NtnChoHelper::SetD1Threshold(double distance_m) { m_choConfig.d1Threshold_m = distance_m; }
void NtnChoHelper::SetQualityThreshold(double sinr_dB) { m_choConfig.qualityThreshold_dB = sinr_dB; }
void NtnChoHelper::SetSatelliteTxPower(double txPower_dBm) { m_satTxPower_dBm = txPower_dBm; }

// ========== Scenario Setup ==========

void
NtnChoHelper::SetupConstellation(NodeContainer satNodes,
                                 Ptr<SatAntennaGainPatternContainer> agpContainer)
{
    NS_LOG_FUNCTION(this << satNodes.GetN());

    // Create orbit predictor
    m_orbitPredictor = CreateObject<NtnOrbitPredictor>();
    m_orbitPredictor->Initialize(satNodes, agpContainer);

    // Create TTE estimator
    m_tteEstimator = CreateObject<NtnTteEstimator>();
    m_tteEstimator->SetOrbitPredictor(m_orbitPredictor);

    // Create measurement model
    m_measurementModel = CreateObject<NtnMeasurementModel>();
    m_measurementModel->SetOrbitPredictor(m_orbitPredictor);
    m_measurementModel->SetNtnScenario(m_scenario);
    m_measurementModel->SetCarrierFrequency(m_carrierFreqHz);
    m_measurementModel->SetBandwidth(m_bandwidthHz);
    m_measurementModel->SetSatelliteTxPower(m_satTxPower_dBm);

    NS_LOG_INFO("NTN-CHO framework initialized with " << satNodes.GetN()
                << " satellites, " << m_orbitPredictor->GetNumBeamsPerSat() << " beams/sat");
}

Ptr<NtnChoAlgorithm>
NtnChoHelper::CreateChoAlgorithm()
{
    Ptr<NtnChoAlgorithm> algo = CreateObject<NtnChoAlgorithm>();
    algo->Configure(m_choConfig);
    algo->SetTteEstimator(m_tteEstimator);
    algo->SetOrbitPredictor(m_orbitPredictor);
    return algo;
}

Ptr<NtnOrbitPredictor> NtnChoHelper::GetOrbitPredictor() const { return m_orbitPredictor; }
Ptr<NtnTteEstimator> NtnChoHelper::GetTteEstimator() const { return m_tteEstimator; }
Ptr<NtnMeasurementModel> NtnChoHelper::GetMeasurementModel() const { return m_measurementModel; }

// ========== KPI Collection ==========

void
NtnChoHelper::EnableTraces(std::string outputDir)
{
    NS_LOG_FUNCTION(this << outputDir);
    m_tracesEnabled = true;
    m_outputDir = outputDir;

    m_hoTraceFile.open(outputDir + "/ntn-cho-handovers.csv");
    m_hoTraceFile << "Time_s,SourceCell,TargetCell,TimeOfStay_s,Success,Reason" << std::endl;

    m_measurementTraceFile.open(outputDir + "/ntn-cho-measurements.csv");
    m_measurementTraceFile << "Time_s,SatId,BeamId,RSRP_dBm,SINR_dB,Elevation_deg,Range_km"
                           << std::endl;

    m_tteTraceFile.open(outputDir + "/ntn-cho-tte.csv");
    m_tteTraceFile << "Time_s,SatId,BeamId,TTE_s,CurrentGain_dB" << std::endl;
}

void
NtnChoHelper::RecordHandover(uint16_t sourceCellId, uint16_t targetCellId, Time timeOfStay)
{
    m_kpis.totalHandovers++;
    m_tosValues.push_back(timeOfStay);
}

void
NtnChoHelper::RecordHandoverOutcome(uint16_t cellId, bool success, std::string reason)
{
    if (success)
    {
        m_kpis.successfulHandovers++;
        if (reason == "PingPong")
        {
            m_kpis.pingPongEvents++;
        }
    }
    else
    {
        m_kpis.failedHandovers++;
    }

    if (m_tracesEnabled && m_hoTraceFile.is_open())
    {
        m_hoTraceFile << std::fixed << std::setprecision(3)
                      << Simulator::Now().GetSeconds() << ","
                      << cellId << ","
                      << (success ? "1" : "0") << ","
                      << reason << std::endl;
    }
}

void
NtnChoHelper::RecordTteComputation(uint32_t satId, uint32_t beamId,
                                   Time predictedTte, Time actualTte)
{
    m_kpis.totalTteComputations++;
    double error = std::abs(predictedTte.GetSeconds() - actualTte.GetSeconds());
    m_tteErrors.push_back(error);
}

NtnChoHelper::KpiResults
NtnChoHelper::GetKpiResults() const
{
    KpiResults results = m_kpis;

    if (results.totalHandovers > 0)
    {
        results.hoSuccessRate = 100.0 * results.successfulHandovers / results.totalHandovers;
        results.hoFailureRate = 100.0 * results.failedHandovers / results.totalHandovers;
        results.pingPongRate = 100.0 * results.pingPongEvents / results.totalHandovers;
    }

    if (!m_tosValues.empty())
    {
        double sum = 0;
        double minTos = 1e9;
        double maxTos = 0;
        for (const auto& tos : m_tosValues)
        {
            double val = tos.GetSeconds();
            sum += val;
            minTos = std::min(minTos, val);
            maxTos = std::max(maxTos, val);
        }
        results.avgTimeOfStay_s = sum / m_tosValues.size();
        results.minTimeOfStay_s = minTos;
        results.maxTimeOfStay_s = maxTos;
    }

    if (!m_tteErrors.empty())
    {
        results.avgTteError_s = std::accumulate(m_tteErrors.begin(), m_tteErrors.end(), 0.0) /
                                m_tteErrors.size();
    }

    return results;
}

void
NtnChoHelper::WriteKpiSummary(std::string filename) const
{
    KpiResults res = GetKpiResults();
    std::ofstream f(filename);
    f << "=== NTN-CHO KPI Summary ===" << std::endl;
    f << "Trigger Type: " << m_choConfig.triggerType << std::endl;
    f << "D1 Threshold: " << m_choConfig.d1Threshold_m << " m" << std::endl;
    f << "Quality Threshold: " << m_choConfig.qualityThreshold_dB << " dB" << std::endl;
    f << "TTE Minimum: " << m_choConfig.tteMinimum.GetSeconds() << " s" << std::endl;
    f << std::endl;
    f << "Total Handovers:     " << res.totalHandovers << std::endl;
    f << "Successful HOs:      " << res.successfulHandovers << std::endl;
    f << "Failed HOs:          " << res.failedHandovers << std::endl;
    f << "Ping-Pong Events:    " << res.pingPongEvents << std::endl;
    f << "HO Success Rate:     " << std::fixed << std::setprecision(2) << res.hoSuccessRate << " %" << std::endl;
    f << "HO Failure Rate:     " << res.hoFailureRate << " %" << std::endl;
    f << "Ping-Pong Rate:      " << res.pingPongRate << " %" << std::endl;
    f << std::endl;
    f << "Avg Time-of-Stay:    " << res.avgTimeOfStay_s << " s" << std::endl;
    f << "Min Time-of-Stay:    " << res.minTimeOfStay_s << " s" << std::endl;
    f << "Max Time-of-Stay:    " << res.maxTimeOfStay_s << " s" << std::endl;
    f << std::endl;
    f << "TTE Computations:    " << res.totalTteComputations << std::endl;
    f << "Avg TTE Error:       " << res.avgTteError_s << " s" << std::endl;
    f.close();

    NS_LOG_INFO("KPI summary written to " << filename);
}

uint16_t
NtnChoHelper::RunChoEvaluation(GeoCoordinate uePosition,
                               Vector ueVelocity,
                               uint16_t servingCellId,
                               double servingSinr_dB,
                               Ptr<NtnChoAlgorithm> algo)
{
    NS_LOG_FUNCTION(this << servingCellId << servingSinr_dB);

    // Step 1: Scan visible satellites and get measurements
    auto measurements = m_measurementModel->ScanVisibleBeams(uePosition, 10.0);

    // Step 2: Update candidate measurements
    algo->ClearCandidates();
    for (const auto& meas : measurements)
    {
        if (meas.cellId != servingCellId)
        {
            algo->AddCandidateCell(meas.cellId, meas.satId, meas.beamId);
            algo->UpdateMeasurement(meas.cellId, meas.sinr_dB, meas.antennaGain_dB);
        }
    }

    // Step 3: Start monitoring and evaluate
    algo->StartMonitoring(uePosition, ueVelocity);
    algo->EvaluateConditions();

    // Step 4: Select best candidate
    uint16_t target = 0;
    switch (m_choConfig.triggerType)
    {
    case NtnChoAlgorithm::TRIGGER_TTE_AWARE:
        target = algo->SelectBestCandidate();
        break;
    case NtnChoAlgorithm::TRIGGER_EVENT_A3:
        target = algo->SelectBaselineA3(servingSinr_dB);
        break;
    case NtnChoAlgorithm::TRIGGER_LOCATION_D1:
        target = algo->SelectBaselineLocationOnly();
        break;
    default:
        target = algo->SelectBestCandidate();
        break;
    }

    return target;
}

void
NtnChoHelper::OnHandoverExecuted(uint16_t source, uint16_t target, Time tos)
{
    RecordHandover(source, target, tos);
    if (m_tracesEnabled && m_hoTraceFile.is_open())
    {
        m_hoTraceFile << Simulator::Now().GetSeconds() << ","
                      << source << "," << target << ","
                      << tos.GetSeconds() << std::endl;
    }
}

void
NtnChoHelper::OnHandoverOutcome(uint16_t cellId, bool success, std::string reason)
{
    RecordHandoverOutcome(cellId, success, reason);
}

void
NtnChoHelper::OnTteComputed(uint32_t satId, uint32_t beamId, Time tte, double gain)
{
    if (m_tracesEnabled && m_tteTraceFile.is_open())
    {
        m_tteTraceFile << Simulator::Now().GetSeconds() << ","
                       << satId << "," << beamId << ","
                       << tte.GetSeconds() << "," << gain << std::endl;
    }
}

} // namespace ns3
