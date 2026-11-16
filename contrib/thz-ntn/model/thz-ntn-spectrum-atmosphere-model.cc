/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "thz-ntn-spectrum-atmosphere-model.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/spectrum-value.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnSpectrumAtmosphereModel");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnSpectrumAtmosphereModel);

TypeId
ThzNtnSpectrumAtmosphereModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnSpectrumAtmosphereModel")
            .SetParent<SpectrumPropagationLossModel>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnSpectrumAtmosphereModel>()
            .AddAttribute("UniformRainRate",
                          "Uniform R_0.01 point rain rate (mm/h) for the P.618 mode; 0 = none.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&ThzNtnSpectrumAtmosphereModel::m_uniformRain),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("CloudLwc",
                          "Cloud/fog liquid water content (g/m^3); 0 = none.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&ThzNtnSpectrumAtmosphereModel::m_lwc),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("ElevationQuantum",
                          "Elevation step (deg) of the gaseous-absorption cache.",
                          DoubleValue(0.02),
                          MakeDoubleAccessor(&ThzNtnSpectrumAtmosphereModel::m_elevQuantumDeg),
                          MakeDoubleChecker<double>(1e-4, 1.0))
            .AddAttribute("RainRefresh",
                          "Largest age of a rain-field path sample per link.",
                          TimeValue(MilliSeconds(10)),
                          MakeTimeAccessor(&ThzNtnSpectrumAtmosphereModel::m_rainRefresh),
                          MakeTimeChecker());
    return tid;
}

ThzNtnSpectrumAtmosphereModel::ThzNtnSpectrumAtmosphereModel()
{
    m_abs = CreateObject<ThzNtnMolecularAbsorption>();
    m_weather = CreateObject<ThzNtnWeatherAttenuation>();
    m_p618 = CreateObject<itu::Itu618LossModel>();
}

ThzNtnSpectrumAtmosphereModel::~ThzNtnSpectrumAtmosphereModel() = default;

void
ThzNtnSpectrumAtmosphereModel::SetAtmosphereProfile(const std::string& profile)
{
    m_abs->SetHumidityProfile(profile);
    m_gasCache.clear();
}

void
ThzNtnSpectrumAtmosphereModel::SetClimateRegion(itu::Itu618LossModel::ClimateRegion r)
{
    m_p618->SetClimateRegion(r);
}

void
ThzNtnSpectrumAtmosphereModel::SetStationLatitudeDeg(double lat)
{
    m_p618->SetStationLatitudeDeg(lat);
}

double
ThzNtnSpectrumAtmosphereModel::GetLastMeanLossDb() const
{
    if (m_lastLoss.empty())
    {
        return 0.0;
    }
    return std::accumulate(m_lastLoss.begin(), m_lastLoss.end(), 0.0) / m_lastLoss.size();
}

double
ThzNtnSpectrumAtmosphereModel::GetLastMinLossDb() const
{
    return m_lastLoss.empty() ? 0.0 : *std::min_element(m_lastLoss.begin(), m_lastLoss.end());
}

double
ThzNtnSpectrumAtmosphereModel::GetLastMaxLossDb() const
{
    return m_lastLoss.empty() ? 0.0 : *std::max_element(m_lastLoss.begin(), m_lastLoss.end());
}

std::vector<double>
ThzNtnSpectrumAtmosphereModel::GasVector(const ThzNtnLinkGeometry& g,
                                         const std::vector<double>& freqsHz) const
{
    std::vector<double> out(freqsHz.size(), 0.0);
    if (!m_gas || freqsHz.empty())
    {
        return out;
    }
    const double eq = std::round(g.elevationDeg / m_elevQuantumDeg);
    const double highKm = std::min(g.upperAltKm, 100.0);
    GasKey key{static_cast<int64_t>(eq),
               static_cast<int64_t>(std::llround(g.lowerAltKm * 1000.0)),
               static_cast<int64_t>(std::llround(highKm)),
               freqsHz.front(),
               freqsHz.size()};
    auto it = m_gasCache.find(key);
    if (it != m_gasCache.end())
    {
        return it->second;
    }
    const double elev = eq * m_elevQuantumDeg;
    for (size_t i = 0; i < freqsHz.size(); ++i)
    {
        out[i] = m_abs->ComputeSlantPathAbsorption(freqsHz[i], elev, g.lowerAltKm, highKm);
    }
    if (m_abs->WasLastElevationClamped())
    {
        ++m_clamped;
    }
    if (m_gasCache.size() > 4096)
    {
        m_gasCache.clear();
    }
    m_gasCache.emplace(key, out);
    return out;
}

std::vector<double>
ThzNtnSpectrumAtmosphereModel::ComputeLossDb(const ThzNtnLinkGeometry& g,
                                             const std::vector<double>& freqsHz,
                                             Time t) const
{
    std::vector<double> loss(freqsHz.size(), 0.0);
    if (g.lowerAltKm >= 100.0 || g.elevationDeg <= 0.0)
    {
        // ISL above the atmosphere, or a path that is not an Earth-space
        // slant (below the horizon: Earth blockage handles it).
        m_lastPathRate = 0.0;
        return loss;
    }
    loss = GasVector(g, freqsHz);
    const size_t mid = freqsHz.size() / 2;
    m_lastGasCentre = freqsHz.empty() ? 0.0 : loss[mid];
    m_lastRainCentre = 0.0;
    if (m_field)
    {
        double ds = 0.0;
        const auto rates = m_field->SamplePath(g, t, ds);
        m_lastPathRate = rates.empty() ? 0.0 : *std::max_element(rates.begin(), rates.end());
        for (size_t i = 0; i < freqsHz.size(); ++i)
        {
            const double a = m_field->IntegrateDb(rates, ds, freqsHz[i]);
            loss[i] += a;
            if (i == mid)
            {
                m_lastRainCentre = a;
            }
        }
    }
    else if (m_uniformRain > 0.0)
    {
        m_lastPathRate = m_uniformRain;
        for (size_t i = 0; i < freqsHz.size(); ++i)
        {
            const double a = m_p618->SlantPathRainAttenuationDb(freqsHz[i],
                                                                g.elevationDeg,
                                                                m_uniformRain,
                                                                g.lowerAltKm,
                                                                m_pol);
            loss[i] += a;
            if (i == mid)
            {
                m_lastRainCentre = a;
            }
        }
    }
    if (m_lwc > 0.0)
    {
        for (size_t i = 0; i < freqsHz.size(); ++i)
        {
            loss[i] += m_weather->ComputeFogAttenuation_dB(freqsHz[i], g.elevationDeg, m_lwc);
        }
    }
    return loss;
}

Ptr<SpectrumValue>
ThzNtnSpectrumAtmosphereModel::DoCalcRxPowerSpectralDensity(
    Ptr<const SpectrumSignalParameters> params,
    Ptr<const MobilityModel> a,
    Ptr<const MobilityModel> b) const
{
    Ptr<SpectrumValue> psd = params->psd->Copy();
    if (!a || !b)
    {
        return psd;
    }
    const ThzNtnLinkGeometry g = ThzNtnLinkGeometry::Compute(PeekPointer(a), PeekPointer(b));
    m_lastElev = g.elevationDeg;
    std::vector<double> freqs;
    freqs.reserve(psd->GetValuesN());
    for (auto it = psd->ConstBandsBegin(); it != psd->ConstBandsEnd(); ++it)
    {
        freqs.push_back(it->fc);
    }
    std::vector<double> loss;
    const Time now = Simulator::Now();
    if (m_field && g.lowerAltKm < 100.0 && g.elevationDeg > 0.0)
    {
        // Rain field: cache per (link, band set) for RainRefresh.
        const auto key = std::make_pair(static_cast<const void*>(PeekPointer(a)),
                                        static_cast<const void*>(PeekPointer(b)));
        auto& e = m_rainCache[key];
        if (e.t < Seconds(0) || now - e.t > m_rainRefresh || e.loss.size() != freqs.size())
        {
            e.loss = ComputeLossDb(g, freqs, now);
            e.pathRate = m_lastPathRate;
            e.t = now;
        }
        else
        {
            // Within RainRefresh the cached vector (gas + rain) is reused; the
            // geometry moves by metres in 10 ms.
            m_lastPathRate = e.pathRate;
        }
        loss = e.loss;
    }
    else
    {
        loss = ComputeLossDb(g, freqs, now);
    }
    if (m_scint && g.lowerAltKm < 100.0 && g.elevationDeg > 0.0 && !freqs.empty())
    {
        // Positive sample = enhancement; loss is its negative (flat over the band).
        // Advance the AR(1) process at most once per simulated instant (several
        // signals share a slot; ThzNtnScintillation would otherwise step its
        // process by its sampling period on every call at the same time).
        if (now != m_lastScintTime)
        {
            m_lastScint =
                m_scint->GetScintillationSample_dB(freqs[freqs.size() / 2], g.elevationDeg);
            m_lastScintTime = now;
        }
        for (auto& l : loss)
        {
            l -= m_lastScint;
        }
    }
    ++m_evals;
    size_t i = 0;
    for (auto v = psd->ValuesBegin(); v != psd->ValuesEnd() && i < loss.size(); ++v, ++i)
    {
        *v *= std::pow(10.0, -loss[i] / 10.0);
    }
    m_lastLoss = std::move(loss);
    return psd;
}

int64_t
ThzNtnSpectrumAtmosphereModel::DoAssignStreams(int64_t /*stream*/)
{
    return 0;
}

} // namespace ns3
