/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Master Composite THz-NTN Channel Model
 *
 * Delegates THz loss computation to ThzNtnFreeSpaceLoss (which extends
 * SatFreeSpaceLoss) so that the same physics models are applied whether
 * the channel is accessed through the PropagationLossModel path (mmWave)
 * or the SatChannel -> SatFreeSpaceLoss::GetFsl() path (satellite module).
 */

#include "thz-ntn-channel-model.h"

#include "thz-ntn-free-space-loss.h"
#include "thz-ntn-hardware-impairments.h"
#include "thz-ntn-molecular-absorption.h"
#include "thz-ntn-pointing-error.h"
#include "thz-ntn-scintillation.h"
#include "thz-ntn-spectrum.h"
#include "thz-ntn-weather-attenuation.h"

#include <ns3/boolean.h>
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/mobility-model.h>
#include <ns3/node.h>
#include <ns3/string.h>

#include <cmath>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnChannelModel");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnChannelModel);

TypeId
ThzNtnChannelModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnChannelModel")
            .SetParent<PropagationLossModel>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnChannelModel>()
            .AddAttribute("CenterFrequency",
                          "Centre frequency in Hz",
                          DoubleValue(225.0e9),
                          MakeDoubleAccessor(&ThzNtnChannelModel::m_centerFreqHz),
                          MakeDoubleChecker<double>(1.0e9, 10.0e12))
            .AddAttribute("BandName",
                          "Operating band name string",
                          StringValue("TeraLink-225GHz"),
                          MakeStringAccessor(&ThzNtnChannelModel::m_bandName),
                          MakeStringChecker())
            .AddAttribute("EnableMolecularAbsorption",
                          "Enable molecular absorption modelling",
                          BooleanValue(true),
                          MakeBooleanAccessor(&ThzNtnChannelModel::m_enableMolecular),
                          MakeBooleanChecker())
            .AddAttribute("EnableWeatherEffects",
                          "Enable weather attenuation modelling",
                          BooleanValue(false),
                          MakeBooleanAccessor(&ThzNtnChannelModel::m_enableWeather),
                          MakeBooleanChecker())
            .AddAttribute("EnableScintillation",
                          "Enable tropospheric scintillation modelling",
                          BooleanValue(false),
                          MakeBooleanAccessor(&ThzNtnChannelModel::m_enableScintillation),
                          MakeBooleanChecker())
            .AddAttribute("EnablePointingError",
                          "Enable beam pointing error modelling",
                          BooleanValue(false),
                          MakeBooleanAccessor(&ThzNtnChannelModel::m_enablePointing),
                          MakeBooleanChecker())
            .AddAttribute("EnableHardwareImpairments",
                          "Enable hardware impairment modelling",
                          BooleanValue(false),
                          MakeBooleanAccessor(&ThzNtnChannelModel::m_enableHardware),
                          MakeBooleanChecker())
            .AddTraceSource("PathLoss",
                            "Total path loss in dB",
                            MakeTraceSourceAccessor(&ThzNtnChannelModel::m_pathLossTrace),
                            "ns3::TracedCallback::DoubleCallback")
            .AddTraceSource("MolecularAbsorption",
                            "Molecular absorption loss in dB",
                            MakeTraceSourceAccessor(
                                &ThzNtnChannelModel::m_molecularAbsorptionTrace),
                            "ns3::TracedCallback::DoubleCallback")
            .AddTraceSource("WeatherLoss",
                            "Weather-induced loss in dB",
                            MakeTraceSourceAccessor(&ThzNtnChannelModel::m_weatherLossTrace),
                            "ns3::TracedCallback::DoubleCallback")
            .AddTraceSource("PointingLoss",
                            "Pointing error loss in dB",
                            MakeTraceSourceAccessor(&ThzNtnChannelModel::m_pointingLossTrace),
                            "ns3::TracedCallback::DoubleCallback")
            .AddTraceSource("TotalRxPower",
                            "Total received power in dBm",
                            MakeTraceSourceAccessor(&ThzNtnChannelModel::m_totalRxPowerTrace),
                            "ns3::TracedCallback::DoubleCallback");
    return tid;
}

ThzNtnChannelModel::ThzNtnChannelModel()
    : m_linkType(AUTO_DETECT),
      m_bandName("TeraLink-225GHz"),
      m_centerFreqHz(225.0e9),
      m_enableMolecular(true),
      m_enableWeather(false),
      m_enableScintillation(false),
      m_enablePointing(false),
      m_enableHardware(false),
      m_thzFsl(nullptr),
      m_molecularModel(nullptr),
      m_weatherModel(nullptr),
      m_scintillationModel(nullptr),
      m_pointingModel(nullptr),
      m_hardwareModel(nullptr),
      m_spectrumModel(nullptr),
      m_lastPathLoss_dB(0.0),
      m_lastMolecularAbsorption_dB(0.0),
      m_lastWeatherLoss_dB(0.0),
      m_lastPointingLoss_dB(0.0),
      m_lastScintillationLoss_dB(0.0)
{
    NS_LOG_FUNCTION(this);

    // Create the ThzNtnFreeSpaceLoss bridge and all default sub-models
    m_thzFsl = CreateObject<ThzNtnFreeSpaceLoss>();

    m_molecularModel = CreateObject<ThzNtnMolecularAbsorption>();
    m_weatherModel = CreateObject<ThzNtnWeatherAttenuation>();
    m_scintillationModel = CreateObject<ThzNtnScintillation>();
    m_pointingModel = CreateObject<ThzNtnPointingError>();

    // Install sub-models into the FSL bridge
    m_thzFsl->SetMolecularAbsorptionModel(m_molecularModel);
    m_thzFsl->SetWeatherModel(m_weatherModel);
    m_thzFsl->SetScintillationModel(m_scintillationModel);
    m_thzFsl->SetPointingErrorModel(m_pointingModel);

    // Sync enable flags
    m_thzFsl->EnableMolecularAbsorption(m_enableMolecular);
    m_thzFsl->EnableWeatherEffects(m_enableWeather);
    m_thzFsl->EnableScintillation(m_enableScintillation);
    m_thzFsl->EnablePointingError(m_enablePointing);
}

ThzNtnChannelModel::~ThzNtnChannelModel()
{
    NS_LOG_FUNCTION(this);
}

void
ThzNtnChannelModel::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_thzFsl = nullptr;
    m_molecularModel = nullptr;
    m_weatherModel = nullptr;
    m_scintillationModel = nullptr;
    m_pointingModel = nullptr;
    m_hardwareModel = nullptr;
    m_spectrumModel = nullptr;
    m_linkInfoMap.clear();
    PropagationLossModel::DoDispose();
}

// ---------------------------------------------------------------------------
// Satellite module integration
// ---------------------------------------------------------------------------

Ptr<ThzNtnFreeSpaceLoss>
ThzNtnChannelModel::GetThzFreeSpaceLoss() const
{
    return m_thzFsl;
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

void
ThzNtnChannelModel::SetLinkType(LinkType type)
{
    NS_LOG_FUNCTION(this << type);
    m_linkType = type;
}

void
ThzNtnChannelModel::SetBand(const std::string& band)
{
    NS_LOG_FUNCTION(this << band);
    m_bandName = band;
    ParseBandName(band);
}

void
ThzNtnChannelModel::EnableMolecularAbsorption(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_enableMolecular = enable;
    if (m_thzFsl)
    {
        m_thzFsl->EnableMolecularAbsorption(enable);
    }
}

void
ThzNtnChannelModel::EnableWeatherEffects(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_enableWeather = enable;
    if (m_thzFsl)
    {
        m_thzFsl->EnableWeatherEffects(enable);
    }
}

void
ThzNtnChannelModel::EnableScintillation(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_enableScintillation = enable;
    if (m_thzFsl)
    {
        m_thzFsl->EnableScintillation(enable);
    }
}

void
ThzNtnChannelModel::EnablePointingError(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_enablePointing = enable;
    if (m_thzFsl)
    {
        m_thzFsl->EnablePointingError(enable);
    }
}

void
ThzNtnChannelModel::EnableHardwareImpairments(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_enableHardware = enable;
}

// ---------------------------------------------------------------------------
// Component model setters
// ---------------------------------------------------------------------------

void
ThzNtnChannelModel::SetMolecularAbsorptionModel(Ptr<ThzNtnMolecularAbsorption> model)
{
    NS_LOG_FUNCTION(this << model);
    m_molecularModel = model;
    if (m_thzFsl)
    {
        m_thzFsl->SetMolecularAbsorptionModel(model);
    }
}

void
ThzNtnChannelModel::SetWeatherModel(Ptr<ThzNtnWeatherAttenuation> model)
{
    NS_LOG_FUNCTION(this << model);
    m_weatherModel = model;
    if (m_thzFsl)
    {
        m_thzFsl->SetWeatherModel(model);
    }
}

void
ThzNtnChannelModel::SetScintillationModel(Ptr<ThzNtnScintillation> model)
{
    NS_LOG_FUNCTION(this << model);
    m_scintillationModel = model;
    if (m_thzFsl)
    {
        m_thzFsl->SetScintillationModel(model);
    }
}

void
ThzNtnChannelModel::SetPointingErrorModel(Ptr<ThzNtnPointingError> model)
{
    NS_LOG_FUNCTION(this << model);
    m_pointingModel = model;
    if (m_thzFsl)
    {
        m_thzFsl->SetPointingErrorModel(model);
    }
}

void
ThzNtnChannelModel::SetHardwareModel(Ptr<ThzNtnHardwareImpairments> model)
{
    NS_LOG_FUNCTION(this << model);
    m_hardwareModel = model;
}

void
ThzNtnChannelModel::SetSpectrumModel(Ptr<ThzNtnSpectrum> model)
{
    NS_LOG_FUNCTION(this << model);
    m_spectrumModel = model;
}

// ---------------------------------------------------------------------------
// Result accessors
// ---------------------------------------------------------------------------

double
ThzNtnChannelModel::GetLastPathLoss_dB() const
{
    return m_lastPathLoss_dB;
}

double
ThzNtnChannelModel::GetLastMolecularAbsorption_dB() const
{
    return m_lastMolecularAbsorption_dB;
}

bool
ThzNtnChannelModel::GetLinkSignalInfo(const std::string& linkKey,
                                       ThzNtnSignalInfo& info) const
{
    auto it = m_linkInfoMap.find(linkKey);
    if (it != m_linkInfoMap.end())
    {
        info = it->second;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Core channel computation
// ---------------------------------------------------------------------------

double
ThzNtnChannelModel::DoCalcRxPower(double txPowerDbm,
                                   Ptr<MobilityModel> a,
                                   Ptr<MobilityModel> b) const
{
    NS_LOG_FUNCTION(this << txPowerDbm);

    // --- 1. Compute 3-D distance from actual mobility model positions ---
    Vector posA = a->GetPosition();
    Vector posB = b->GetPosition();
    double dx = posA.x - posB.x;
    double dy = posA.y - posB.y;
    double dz = posA.z - posB.z;
    double distanceM = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distanceM < 1.0)
    {
        // Nodes co-located; no path loss
        m_lastPathLoss_dB = 0.0;
        m_lastMolecularAbsorption_dB = 0.0;
        m_lastWeatherLoss_dB = 0.0;
        m_lastPointingLoss_dB = 0.0;
        m_lastScintillationLoss_dB = 0.0;
        return txPowerDbm;
    }

    // --- 2. Use ThzNtnFreeSpaceLoss for all THz effects ---
    // GetFsldB calls parent SatFreeSpaceLoss::GetFsl() internally and adds
    // molecular absorption, weather, scintillation, and pointing error.
    double totalLoss_dB = m_thzFsl->GetFsldB(a, b, m_centerFreqHz);

    // --- 3. Rx power = Tx power - total loss ---
    double rxPowerDbm = txPowerDbm - totalLoss_dB;

    // --- 4. Store per-link ThzNtnSignalInfo from cached breakdown ---
    double baseFspl_dB = m_thzFsl->GetLastBaseFspl_dB();
    double absorption_dB = m_thzFsl->GetLastMolecularAbsorption_dB();
    double weather_dB = m_thzFsl->GetLastWeatherLoss_dB();
    double scintillation_dB = m_thzFsl->GetLastScintillationLoss_dB();
    double pointing_dB = m_thzFsl->GetLastPointingLoss_dB();

    // Determine if ISL from altitudes
    double altA_km = posA.z / 1000.0;
    double altB_km = posB.z / 1000.0;
    bool isIsl = (altA_km > ISL_ALTITUDE_THRESHOLD_KM &&
                  altB_km > ISL_ALTITUDE_THRESHOLD_KM);

    // Compute elevation for info struct
    double lower = std::min(altA_km, altB_km);
    double upper = std::max(altA_km, altB_km);
    double dh = std::sqrt(dx * dx + dy * dy);
    double dv = (upper - lower) * 1000.0;
    double elevationDeg = (dh > 1.0) ? std::atan2(dv, dh) * 180.0 / M_PI : 90.0;
    elevationDeg = std::max(0.0, std::min(90.0, elevationDeg));

    // Build and store the signal info
    ThzNtnSignalInfo info;
    info.totalLoss_dB = totalLoss_dB;
    info.baseFspl_dB = baseFspl_dB;
    info.molecularAbsorption_dB = absorption_dB;
    info.weatherLoss_dB = weather_dB;
    info.scintillationLoss_dB = scintillation_dB;
    info.pointingLoss_dB = pointing_dB;
    info.elevationDeg = elevationDeg;
    info.distanceM = distanceM;
    info.frequencyHz = m_centerFreqHz;
    info.isIsl = isIsl;

    std::string linkKey = MakeLinkKey(a, b);
    m_linkInfoMap[linkKey] = info;

    // --- 5. Cache results ---
    m_lastPathLoss_dB = totalLoss_dB;
    m_lastMolecularAbsorption_dB = absorption_dB;
    m_lastWeatherLoss_dB = weather_dB;
    m_lastPointingLoss_dB = pointing_dB;
    m_lastScintillationLoss_dB = scintillation_dB;

    // --- 6. Hardware impairment degradation (post-FSL) ---
    if (m_enableHardware && m_hardwareModel)
    {
        // Hardware impairments reduce effective SNR by adding an equivalent
        // noise contribution.  Modelled as additional power loss.
        // double hwDegradation_dB = m_hardwareModel->ComputeSnrDegradation_dB(rxPowerDbm);
        // rxPowerDbm -= hwDegradation_dB;
        // Placeholder until ThzNtnHardwareImpairments is implemented
    }

    // --- 7. Fire traced callbacks ---
    m_pathLossTrace(totalLoss_dB);
    m_molecularAbsorptionTrace(absorption_dB);
    m_weatherLossTrace(weather_dB);
    m_pointingLossTrace(pointing_dB);
    m_totalRxPowerTrace(rxPowerDbm);

    NS_LOG_INFO("THz-NTN Channel: baseFSPL=" << baseFspl_dB
                << " dB, absorption=" << absorption_dB
                << " dB, weather=" << weather_dB
                << " dB, scintillation=" << scintillation_dB
                << " dB, pointing=" << pointing_dB
                << " dB, total=" << totalLoss_dB
                << " dB, Rx=" << rxPowerDbm << " dBm");

    return rxPowerDbm;
}

int64_t
ThzNtnChannelModel::DoAssignStreams(int64_t stream)
{
    NS_LOG_FUNCTION(this << stream);
    // Sub-models manage their own streams via their AssignStreams methods.
    // Delegate if scintillation model is present (it uses an RNG).
    int64_t consumed = 0;
    if (m_scintillationModel)
    {
        consumed += m_scintillationModel->AssignStreams(stream);
    }
    if (m_pointingModel)
    {
        consumed += m_pointingModel->AssignStreams(stream + consumed);
    }
    return consumed;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

ThzNtnChannelModel::LinkType
ThzNtnChannelModel::DetectLinkType(double altA_km, double altB_km) const
{
    double lower = std::min(altA_km, altB_km);
    double upper = std::max(altA_km, altB_km);

    if (lower > ISL_ALTITUDE_THRESHOLD_KM)
    {
        NS_LOG_DEBUG("Auto-detected ISL: both nodes > 100 km");
        return INTER_SATELLITE;
    }

    if (upper > ISL_ALTITUDE_THRESHOLD_KM && lower <= HAP_ALTITUDE_THRESHOLD_KM)
    {
        NS_LOG_DEBUG("Auto-detected satellite-ground link");
        return SAT_TO_GROUND;
    }

    if (upper > ISL_ALTITUDE_THRESHOLD_KM && lower > HAP_ALTITUDE_THRESHOLD_KM)
    {
        NS_LOG_DEBUG("Auto-detected satellite-to-HAP link");
        return SAT_TO_HAP;
    }

    if (upper > HAP_ALTITUDE_THRESHOLD_KM)
    {
        NS_LOG_DEBUG("Auto-detected HAP-to-ground link");
        return HAP_TO_GROUND;
    }

    NS_LOG_DEBUG("Auto-detect fallback: ground-to-satellite");
    return GROUND_TO_SAT;
}

void
ThzNtnChannelModel::ParseBandName(const std::string& band)
{
    if (band == "D-band-140GHz")
    {
        m_centerFreqHz = 140.0e9;
    }
    else if (band == "TeraLink-225GHz")
    {
        m_centerFreqHz = 225.0e9;
    }
    else if (band == "H-band-300GHz")
    {
        m_centerFreqHz = 300.0e9;
    }
    else if (band == "ISL-300GHz")
    {
        m_centerFreqHz = 300.0e9;
        m_linkType = INTER_SATELLITE;
    }
    else
    {
        NS_LOG_WARN("Unknown band name '" << band << "'; keeping current frequency "
                                          << m_centerFreqHz / 1.0e9 << " GHz");
    }
    NS_LOG_INFO("Band set to '" << band << "' -> " << m_centerFreqHz / 1.0e9 << " GHz");
}

std::string
ThzNtnChannelModel::MakeLinkKey(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
{
    // Use the Object UID as a stable node identifier
    std::ostringstream oss;
    oss << a->GetObject<Node>()->GetId() << "-" << b->GetObject<Node>()->GetId();
    return oss.str();
}

} // namespace ns3
