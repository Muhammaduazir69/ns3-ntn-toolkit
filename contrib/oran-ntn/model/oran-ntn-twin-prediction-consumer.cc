/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-twin-prediction-consumer.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <fstream>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnTwinPredictionConsumer");
NS_OBJECT_ENSURE_REGISTERED(OranNtnTwinPredictionConsumer);

TypeId
OranNtnTwinPredictionConsumer::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnTwinPredictionConsumer")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnTwinPredictionConsumer>();
    return tid;
}

OranNtnTwinPredictionConsumer::OranNtnTwinPredictionConsumer()
{
    NS_LOG_FUNCTION(this);
}

OranNtnTwinPredictionConsumer::~OranNtnTwinPredictionConsumer()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnTwinPredictionConsumer::AddPrediction(double tSec,
                                             uint32_t ueId,
                                             uint32_t recommendedGnbId,
                                             double confidence)
{
    TwinPrediction p{tSec, ueId, recommendedGnbId, confidence};
    // Keep sorted by time so Start()/GetRecommendation() can rely on ordering.
    auto it = std::upper_bound(m_predictions.begin(), m_predictions.end(), p,
                               [](const TwinPrediction& a, const TwinPrediction& b) {
                                   return a.tSec < b.tSec;
                               });
    m_predictions.insert(it, p);
}

uint32_t
OranNtnTwinPredictionConsumer::LoadPredictionsFromFile(const std::string& path)
{
    NS_LOG_FUNCTION(this << path);
    std::ifstream in(path);
    if (!in.is_open())
    {
        NS_LOG_WARN("OranNtnTwinPredictionConsumer: cannot open predictions file " << path);
        return 0;
    }

    uint32_t loaded = 0;
    std::string line;
    uint32_t lineNo = 0;
    while (std::getline(in, line))
    {
        ++lineNo;
        // Trim leading whitespace.
        size_t start = line.find_first_not_of(" \t\r\n");
        if (start == std::string::npos)
        {
            continue; // blank
        }
        if (line[start] == '#')
        {
            continue; // comment
        }

        // Parse "t_s,ueId,recommendedGnbId,confidence".
        std::stringstream ss(line);
        std::string tok;
        double tSec = 0.0;
        uint32_t ueId = 0;
        uint32_t gnb = 0;
        double conf = 1.0;
        bool ok = true;
        try
        {
            if (!std::getline(ss, tok, ',')) { ok = false; } else { tSec = std::stod(tok); }
            if (ok && std::getline(ss, tok, ',')) { ueId = static_cast<uint32_t>(std::stoul(tok)); } else { ok = false; }
            if (ok && std::getline(ss, tok, ',')) { gnb = static_cast<uint32_t>(std::stoul(tok)); } else { ok = false; }
            // confidence is optional; default 1.0
            if (ok && std::getline(ss, tok, ',')) { conf = std::stod(tok); }
        }
        catch (const std::exception& e)
        {
            ok = false;
        }

        if (!ok || tSec < 0.0)
        {
            NS_LOG_WARN("OranNtnTwinPredictionConsumer: skipping malformed line " << lineNo
                        << " in " << path << ": '" << line << "'");
            continue;
        }
        AddPrediction(tSec, ueId, gnb, conf);
        ++loaded;
    }
    NS_LOG_INFO("OranNtnTwinPredictionConsumer: loaded " << loaded << " predictions from " << path);
    return loaded;
}

uint32_t
OranNtnTwinPredictionConsumer::GetNumPredictions() const
{
    return static_cast<uint32_t>(m_predictions.size());
}

bool
OranNtnTwinPredictionConsumer::GetRecommendation(uint32_t ueId,
                                                 Time now,
                                                 uint32_t& recommendedGnbId,
                                                 double& confidence) const
{
    // Latest prediction for this UE with tSec <= now (predictions are time-sorted).
    bool found = false;
    double nowS = now.GetSeconds();
    for (const auto& p : m_predictions)
    {
        if (p.tSec > nowS + 1e-9)
        {
            break; // sorted: nothing later can apply
        }
        if (p.ueId == ueId)
        {
            recommendedGnbId = p.recommendedGnbId;
            confidence = p.confidence;
            found = true; // keep advancing to the latest applicable one
        }
    }
    return found;
}

void
OranNtnTwinPredictionConsumer::Start(HandoverSubmitCallback submit, double minConfidence)
{
    NS_LOG_FUNCTION(this << minConfidence);
    NS_ASSERT_MSG(!submit.IsNull(), "OranNtnTwinPredictionConsumer::Start needs a submit callback");
    m_submit = submit;
    m_minConfidence = minConfidence;
    m_started = true;

    double nowS = Simulator::Now().GetSeconds();
    for (uint32_t i = 0; i < m_predictions.size(); ++i)
    {
        const auto& p = m_predictions[i];
        if (p.confidence < m_minConfidence)
        {
            continue;
        }
        // Schedule relative to now; past-due predictions fire immediately (dt=0).
        double dt = std::max(0.0, p.tSec - nowS);
        Simulator::Schedule(Seconds(dt), &OranNtnTwinPredictionConsumer::Fire, this, i);
    }
}

void
OranNtnTwinPredictionConsumer::Fire(uint32_t index)
{
    if (index >= m_predictions.size() || m_submit.IsNull())
    {
        return;
    }
    const auto& p = m_predictions[index];
    NS_LOG_INFO("Twin prediction actuated at t=" << Simulator::Now().GetSeconds() << "s: UE "
                << p.ueId << " -> gNB " << p.recommendedGnbId << " (conf " << p.confidence << ")");
    m_submit(p.ueId, p.recommendedGnbId, p.confidence);
    ++m_actuated;
}

uint32_t
OranNtnTwinPredictionConsumer::GetActuatedCount() const
{
    return m_actuated;
}

} // namespace ns3
