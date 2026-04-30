/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN RIS Phase Optimization Controller
 */

#include "thz-ntn-ris-controller.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/string.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnRisController");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnRisController);

TypeId
ThzNtnRisController::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnRisController")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnRisController>()
            .AddAttribute("OptimizationMethod",
                          "Phase optimization method: CODEBOOK, RANDOM_SEARCH, "
                          "ALTERNATING_OPT, DRL_BASED",
                          StringValue("CODEBOOK"),
                          MakeStringAccessor(&ThzNtnRisController::m_optimizationMethodStr),
                          MakeStringChecker())
            .AddAttribute("CodebookSize",
                          "Number of codebook entries",
                          UintegerValue(64),
                          MakeUintegerAccessor(&ThzNtnRisController::m_codebookSize),
                          MakeUintegerChecker<uint32_t>(1, 4096))
            .AddAttribute("UpdateInterval",
                          "Phase update interval in ms",
                          DoubleValue(10.0),
                          MakeDoubleAccessor(&ThzNtnRisController::m_updateInterval_ms),
                          MakeDoubleChecker<double>(0.1, 10000.0))
            .AddAttribute("MaxIterations",
                          "Maximum iterations for iterative optimization",
                          UintegerValue(100),
                          MakeUintegerAccessor(&ThzNtnRisController::m_maxIterations),
                          MakeUintegerChecker<uint32_t>(1, 10000));
    return tid;
}

ThzNtnRisController::ThzNtnRisController()
    : m_ris(nullptr),
      m_optimizationMethodStr("CODEBOOK"),
      m_optimizationMethod(OptimizationMethod::CODEBOOK),
      m_codebookSize(64),
      m_updateInterval_ms(10.0),
      m_maxIterations(100),
      m_hasDrlCallback(false)
{
    NS_LOG_FUNCTION(this);
}

ThzNtnRisController::~ThzNtnRisController()
{
    NS_LOG_FUNCTION(this);
}

void
ThzNtnRisController::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_ris = nullptr;
    m_codebook.clear();
    m_drlCallback = RisPhasePredictionCallback();
    m_hasDrlCallback = false;
    Object::DoDispose();
}

// ---------------------------------------------------------------------------
// Angle computation from 3D positions
// ---------------------------------------------------------------------------

void
ThzNtnRisController::ComputeAngles(const Vector& risPos,
                                   const Vector& targetPos,
                                   double& theta_deg,
                                   double& phi_deg) const
{
    double dx = targetPos.x - risPos.x;
    double dy = targetPos.y - risPos.y;
    double dz = targetPos.z - risPos.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist < 1.0)
    {
        theta_deg = 0.0;
        phi_deg = 0.0;
        return;
    }

    theta_deg = std::acos(dz / dist) / DEG2RAD;
    phi_deg = std::atan2(dy, dx) / DEG2RAD;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void
ThzNtnRisController::SetRis(Ptr<ThzNtnRis> ris)
{
    NS_LOG_FUNCTION(this << ris);
    m_ris = ris;
    m_optimizationMethod = ParseOptMethod(m_optimizationMethodStr);
}

void
ThzNtnRisController::GenerateCodebook(uint32_t numEntries)
{
    NS_LOG_FUNCTION(this << numEntries);

    if (m_ris == nullptr)
    {
        NS_LOG_WARN("No RIS attached; cannot generate codebook");
        return;
    }

    m_codebookSize = numEntries;
    m_codebook.clear();
    m_codebook.reserve(numEntries);

    const uint32_t N = m_ris->GetTotalElements();

    // Get actual panel dimensions and wavelength from RIS
    uint32_t numX, numY;
    m_ris->GetPanelDimensions(numX, numY);
    double lambda = m_ris->GetWavelength();
    double d = lambda / 2.0; // Element spacing = lambda/2 from actual frequency

    // DFT-based codebook generation using actual geometry
    uint32_t numPerDim = static_cast<uint32_t>(std::ceil(std::sqrt(numEntries)));
    uint32_t actualEntries = 0;

    for (uint32_t mx = 0; mx < numPerDim && actualEntries < numEntries; ++mx)
    {
        double ux = -1.0 + 2.0 * static_cast<double>(mx) / static_cast<double>(numPerDim);

        for (uint32_t my = 0; my < numPerDim && actualEntries < numEntries; ++my)
        {
            double uy = -1.0 + 2.0 * static_cast<double>(my) / static_cast<double>(numPerDim);

            std::vector<double> phases(N);

            for (uint32_t ix = 0; ix < numX; ++ix)
            {
                for (uint32_t iy = 0; iy < numY; ++iy)
                {
                    uint32_t n = ix * numY + iy;
                    // DFT phase using actual element spacing
                    double posX = ix * d;
                    double posY = iy * d;
                    double k = 2.0 * PI_VAL / lambda;

                    double phase = k * (ux * posX + uy * posY);

                    // Wrap to [-pi, pi]
                    phase = std::fmod(phase, 2.0 * PI_VAL);
                    if (phase > PI_VAL)
                    {
                        phase -= 2.0 * PI_VAL;
                    }
                    if (phase < -PI_VAL)
                    {
                        phase += 2.0 * PI_VAL;
                    }
                    phases[n] = phase;
                }
            }

            m_codebook.push_back(std::move(phases));
            ++actualEntries;
        }
    }

    NS_LOG_INFO("Generated DFT codebook with " << m_codebook.size() << " entries for "
                                                << N << " elements, lambda="
                                                << lambda * 1.0e3 << " mm");
}

void
ThzNtnRisController::SelectCodebookEntry(Ptr<MobilityModel> tx,
                                          Ptr<MobilityModel> ris,
                                          Ptr<MobilityModel> rx)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(tx && ris && rx, "MobilityModel pointers must not be null");

    Vector risPos = ris->GetPosition();
    Vector txPos = tx->GetPosition();
    Vector rxPos = rx->GetPosition();

    double thetaIn, phiIn, thetaOut, phiOut;
    ComputeAngles(risPos, txPos, thetaIn, phiIn);
    ComputeAngles(risPos, rxPos, thetaOut, phiOut);

    SelectCodebookEntry(thetaIn, phiIn, thetaOut, phiOut);
}

void
ThzNtnRisController::SelectCodebookEntry(double thetaIn,
                                          double phiIn,
                                          double thetaOut,
                                          double phiOut)
{
    NS_LOG_FUNCTION(this << thetaIn << phiIn << thetaOut << phiOut);

    if (m_ris == nullptr)
    {
        NS_LOG_WARN("No RIS attached");
        return;
    }

    if (m_codebook.empty())
    {
        NS_LOG_WARN("Codebook is empty; generating default codebook");
        GenerateCodebook(m_codebookSize);
    }

    double bestGain = -std::numeric_limits<double>::infinity();
    size_t bestIdx = 0;

    for (size_t i = 0; i < m_codebook.size(); ++i)
    {
        m_ris->SetPhaseProfile(m_codebook[i]);
        double gain = m_ris->ComputeRisGain_dB(thetaIn, phiIn, thetaOut, phiOut);

        if (gain > bestGain)
        {
            bestGain = gain;
            bestIdx = i;
        }
    }

    m_ris->SetPhaseProfile(m_codebook[bestIdx]);

    NS_LOG_INFO("Selected codebook entry " << bestIdx << " with gain "
                                           << bestGain << " dB");
}

void
ThzNtnRisController::OptimizePhases(Ptr<MobilityModel> tx,
                                     Ptr<MobilityModel> ris,
                                     Ptr<MobilityModel> rx,
                                     OptimizationMethod method)
{
    NS_LOG_FUNCTION(this << static_cast<int>(method));
    NS_ASSERT_MSG(tx && ris && rx, "MobilityModel pointers must not be null");

    Vector risPos = ris->GetPosition();
    Vector txPos = tx->GetPosition();
    Vector rxPos = rx->GetPosition();

    double thetaIn, phiIn, thetaOut, phiOut;
    ComputeAngles(risPos, txPos, thetaIn, phiIn);
    ComputeAngles(risPos, rxPos, thetaOut, phiOut);

    OptimizePhases(thetaIn, phiIn, thetaOut, phiOut, method);
}

void
ThzNtnRisController::OptimizePhases(double thetaIn,
                                     double phiIn,
                                     double thetaOut,
                                     double phiOut,
                                     OptimizationMethod method)
{
    NS_LOG_FUNCTION(this << thetaIn << phiIn << thetaOut << phiOut
                         << static_cast<int>(method));

    if (m_ris == nullptr)
    {
        NS_LOG_WARN("No RIS attached");
        return;
    }

    switch (method)
    {
    case OptimizationMethod::CODEBOOK:
        SelectCodebookEntry(thetaIn, phiIn, thetaOut, phiOut);
        break;
    case OptimizationMethod::RANDOM_SEARCH:
        RandomSearchOptimize(thetaIn, phiIn, thetaOut, phiOut);
        break;
    case OptimizationMethod::ALTERNATING_OPT:
        AlternatingOptimize(thetaIn, phiIn, thetaOut, phiOut);
        break;
    case OptimizationMethod::DRL_BASED:
        DrlOptimize(thetaIn, phiIn, thetaOut, phiOut);
        break;
    }
}

double
ThzNtnRisController::EstimateChannel(const std::vector<double>& pilotMeasurements)
{
    NS_LOG_FUNCTION(this << pilotMeasurements.size());

    if (pilotMeasurements.empty())
    {
        NS_LOG_WARN("Empty pilot measurements");
        return 0.0;
    }

    double sumPower = 0.0;
    for (const auto& p : pilotMeasurements)
    {
        sumPower += p * p;
    }

    double avgPower = sumPower / static_cast<double>(pilotMeasurements.size());
    double channelGain = std::sqrt(avgPower);

    NS_LOG_INFO("Channel estimated from " << pilotMeasurements.size()
                                          << " pilots: gain=" << channelGain);
    return channelGain;
}

void
ThzNtnRisController::CoordinateMultiPanel(const std::vector<Ptr<ThzNtnRis>>& panels)
{
    NS_LOG_FUNCTION(this << panels.size());

    if (panels.size() < 2)
    {
        NS_LOG_WARN("Multi-panel coordination requires at least 2 panels");
        return;
    }

    Ptr<ThzNtnRis> refPanel = panels[0];
    if (refPanel == nullptr)
    {
        NS_LOG_WARN("Reference panel is null");
        return;
    }

    for (size_t p = 1; p < panels.size(); ++p)
    {
        Ptr<ThzNtnRis> panel = panels[p];
        if (panel == nullptr)
        {
            continue;
        }

        uint32_t N = panel->GetTotalElements();

        double panelOffset = 2.0 * PI_VAL * static_cast<double>(p) /
                             static_cast<double>(panels.size());

        std::vector<double> offsetPhases(N, panelOffset);
        panel->SetPhaseProfile(offsetPhases);

        NS_LOG_DEBUG("Panel " << p << ": applied global phase offset "
                              << panelOffset << " rad");
    }

    NS_LOG_INFO("Coordinated " << panels.size() << " RIS panels");
}

void
ThzNtnRisController::SetDrlCallback(RisPhasePredictionCallback cb)
{
    NS_LOG_FUNCTION(this);
    m_drlCallback = cb;
    m_hasDrlCallback = true;
}

// ---------------------------------------------------------------------------
// Private methods
// ---------------------------------------------------------------------------

void
ThzNtnRisController::RandomSearchOptimize(double thetaIn,
                                           double phiIn,
                                           double thetaOut,
                                           double phiOut)
{
    NS_LOG_FUNCTION(this << thetaIn << phiIn << thetaOut << phiOut);

    const uint32_t N = m_ris->GetTotalElements();
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> phaseDist(-PI_VAL, PI_VAL);

    double bestGain = -std::numeric_limits<double>::infinity();
    std::vector<double> bestPhases(N, 0.0);

    // Start with analytical optimal as baseline
    m_ris->ComputeOptimalPhases(thetaIn, phiIn, thetaOut, phiOut);
    bestGain = m_ris->ComputeRisGain_dB(thetaIn, phiIn, thetaOut, phiOut);

    for (uint32_t iter = 0; iter < m_maxIterations; ++iter)
    {
        std::vector<double> candidate(N);
        for (uint32_t n = 0; n < N; ++n)
        {
            candidate[n] = phaseDist(rng);
        }

        m_ris->SetPhaseProfile(candidate);
        double gain = m_ris->ComputeRisGain_dB(thetaIn, phiIn, thetaOut, phiOut);

        if (gain > bestGain)
        {
            bestGain = gain;
            bestPhases = candidate;
        }
    }

    m_ris->SetPhaseProfile(bestPhases);
    NS_LOG_INFO("Random search: best gain " << bestGain << " dB after "
                                            << m_maxIterations << " iterations");
}

void
ThzNtnRisController::AlternatingOptimize(double thetaIn,
                                          double phiIn,
                                          double thetaOut,
                                          double phiOut)
{
    NS_LOG_FUNCTION(this << thetaIn << phiIn << thetaOut << phiOut);

    const uint32_t N = m_ris->GetTotalElements();

    // Initialize with analytical optimal phases
    m_ris->ComputeOptimalPhases(thetaIn, phiIn, thetaOut, phiOut);

    // Get actual wavelength and spacing from RIS
    double lambda = m_ris->GetWavelength();
    double d = lambda / 2.0;
    double k = 2.0 * PI_VAL / lambda;

    double tIn = thetaIn * DEG2RAD;
    double pIn = phiIn * DEG2RAD;
    double tOut = thetaOut * DEG2RAD;
    double pOut = phiOut * DEG2RAD;

    double uxIn = std::sin(tIn) * std::cos(pIn);
    double uyIn = std::sin(tIn) * std::sin(pIn);
    double uxOut = std::sin(tOut) * std::cos(pOut);
    double uyOut = std::sin(tOut) * std::sin(pOut);

    // Get actual panel dimensions
    uint32_t numX, numY;
    m_ris->GetPanelDimensions(numX, numY);

    std::vector<double> currentPhases(N);

    for (uint32_t ix = 0; ix < numX; ++ix)
    {
        for (uint32_t iy = 0; iy < numY; ++iy)
        {
            uint32_t n = ix * numY + iy;
            double posX = ix * d;
            double posY = iy * d;
            double optPhase = k * ((uxOut - uxIn) * posX + (uyOut - uyIn) * posY);
            optPhase = std::fmod(optPhase, 2.0 * PI_VAL);
            if (optPhase > PI_VAL)
            {
                optPhase -= 2.0 * PI_VAL;
            }
            if (optPhase < -PI_VAL)
            {
                optPhase += 2.0 * PI_VAL;
            }
            currentPhases[n] = optPhase;
        }
    }

    double prevGain = -std::numeric_limits<double>::infinity();

    for (uint32_t outerIter = 0; outerIter < m_maxIterations; ++outerIter)
    {
        for (uint32_t n = 0; n < N; ++n)
        {
            double bestPhaseN = currentPhases[n];
            double bestGainN = -std::numeric_limits<double>::infinity();

            static const uint32_t NUM_CANDIDATES = 16;
            for (uint32_t c = 0; c < NUM_CANDIDATES; ++c)
            {
                double candidatePhase = -PI_VAL +
                    2.0 * PI_VAL * static_cast<double>(c) / static_cast<double>(NUM_CANDIDATES);

                currentPhases[n] = candidatePhase;
                m_ris->SetPhaseProfile(currentPhases);
                double gain = m_ris->ComputeRisGain_dB(thetaIn, phiIn, thetaOut, phiOut);

                if (gain > bestGainN)
                {
                    bestGainN = gain;
                    bestPhaseN = candidatePhase;
                }
            }
            currentPhases[n] = bestPhaseN;
        }

        m_ris->SetPhaseProfile(currentPhases);
        double currentGain = m_ris->ComputeRisGain_dB(thetaIn, phiIn, thetaOut, phiOut);

        if (std::abs(currentGain - prevGain) < 0.01)
        {
            NS_LOG_INFO("Alternating opt converged at iteration " << outerIter
                                                                  << " with gain "
                                                                  << currentGain << " dB");
            return;
        }
        prevGain = currentGain;
    }

    NS_LOG_INFO("Alternating opt completed " << m_maxIterations << " iterations, gain "
                                             << prevGain << " dB");
}

void
ThzNtnRisController::DrlOptimize(double thetaIn,
                                  double phiIn,
                                  double thetaOut,
                                  double phiOut)
{
    NS_LOG_FUNCTION(this << thetaIn << phiIn << thetaOut << phiOut);

    if (!m_hasDrlCallback)
    {
        NS_LOG_WARN("No DRL callback set; falling back to analytical optimal");
        m_ris->ComputeOptimalPhases(thetaIn, phiIn, thetaOut, phiOut);
        return;
    }

    std::vector<double> observation = {
        thetaIn / 90.0,
        phiIn / 180.0,
        thetaOut / 90.0,
        phiOut / 180.0
    };

    std::vector<double> predictedPhases = m_drlCallback(observation);

    uint32_t N = m_ris->GetTotalElements();
    if (predictedPhases.size() != N)
    {
        NS_LOG_WARN("DRL predicted " << predictedPhases.size()
                                     << " phases but need " << N
                                     << "; falling back to analytical");
        m_ris->ComputeOptimalPhases(thetaIn, phiIn, thetaOut, phiOut);
        return;
    }

    for (auto& p : predictedPhases)
    {
        p = std::fmod(p, 2.0 * PI_VAL);
        if (p > PI_VAL)
        {
            p -= 2.0 * PI_VAL;
        }
        if (p < -PI_VAL)
        {
            p += 2.0 * PI_VAL;
        }
    }

    m_ris->SetPhaseProfile(predictedPhases);

    double gain = m_ris->ComputeRisGain_dB(thetaIn, phiIn, thetaOut, phiOut);
    NS_LOG_INFO("DRL optimization: gain " << gain << " dB");
}

OptimizationMethod
ThzNtnRisController::ParseOptMethod(const std::string& s)
{
    if (s == "RANDOM_SEARCH")
    {
        return OptimizationMethod::RANDOM_SEARCH;
    }
    if (s == "ALTERNATING_OPT")
    {
        return OptimizationMethod::ALTERNATING_OPT;
    }
    if (s == "DRL_BASED")
    {
        return OptimizationMethod::DRL_BASED;
    }
    return OptimizationMethod::CODEBOOK;
}

} // namespace ns3
