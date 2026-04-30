/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * NTN Composite Channel Model - Implementation
 *
 * Combines free-space path loss, atmospheric attenuation (ITU-R P.676/P.618/P.531),
 * Loo three-state fading, Markov state transitions, Rician K-factor adaptation
 * per 3GPP TR 38.811, clutter loss, and correlated shadow fading into a unified
 * propagation loss model for LEO NTN links.
 */

#include "oran-ntn-channel-model.h"

#include "oran-ntn-sat-bridge.h"

#include <ns3/boolean.h>
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/mobility-model.h>
#include <ns3/simulator.h>
#include <ns3/string.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnChannelModel");
NS_OBJECT_ENSURE_REGISTERED(OranNtnChannelModel);

// Physical constants
static const double SPEED_OF_LIGHT = 299792458.0;  // m/s
static const double DEG_TO_RAD = M_PI / 180.0;

// Atmospheric model constants (ITU-R P.676)
static const double OXYGEN_ABSORPTION_DB_PER_KM = 0.05;   // at ~20 GHz
static const double WATER_VAPOR_ABSORPTION_DB_PER_KM = 0.02; // at ~20 GHz

// ITU-R P.838 rain attenuation regression coefficients for Ka-band (~20 GHz)
static const double RAIN_K_COEFF = 0.0751;
static const double RAIN_ALPHA_COEFF = 1.099;
static const double RAIN_EFFECTIVE_PATH_KM = 5.0; // effective rain cell path length

// Scintillation model constants (ITU-R P.531)
static const double SCINTILLATION_REF_FREQ_GHZ = 4.0;

// Loo model default parameters (from empirical satellite measurements)
// Format: {mean_direct_dB, std_direct_dB, scattered_power_dB}
struct LooParams
{
    double meanDirect_dB;
    double stdDirect_dB;
    double scatteredPower_dB;
};

/**
 * \brief Get Loo fading parameters based on environment and elevation
 */
static LooParams
GetLooParameters(const std::string& env, double elevationDeg)
{
    LooParams params;

    if (env == "urban" || env == "dense-urban")
    {
        if (elevationDeg > 60.0)
        {
            params = {-0.5, 1.5, -15.0};
        }
        else if (elevationDeg > 30.0)
        {
            params = {-2.0, 3.0, -12.0};
        }
        else
        {
            params = {-4.0, 4.5, -10.0};
        }
    }
    else if (env == "suburban")
    {
        if (elevationDeg > 60.0)
        {
            params = {-0.2, 1.0, -18.0};
        }
        else if (elevationDeg > 30.0)
        {
            params = {-1.0, 2.0, -15.0};
        }
        else
        {
            params = {-2.5, 3.5, -12.0};
        }
    }
    else if (env == "maritime" || env == "aeronautical")
    {
        // Very strong LOS over water/air
        if (elevationDeg > 20.0)
        {
            params = {-0.1, 0.5, -25.0};
        }
        else
        {
            params = {-0.3, 1.0, -20.0};
        }
    }
    else // rural or default
    {
        if (elevationDeg > 60.0)
        {
            params = {-0.1, 0.8, -20.0};
        }
        else if (elevationDeg > 30.0)
        {
            params = {-0.5, 1.5, -17.0};
        }
        else
        {
            params = {-1.5, 2.5, -14.0};
        }
    }

    return params;
}

TypeId
OranNtnChannelModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnChannelModel")
            .SetParent<PropagationLossModel>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnChannelModel>()
            .AddAttribute("Band",
                          "Frequency band: S-band, Ka-band, or L-band",
                          StringValue("S-band"),
                          MakeStringAccessor(&OranNtnChannelModel::m_band),
                          MakeStringChecker())
            .AddAttribute("Environment",
                          "Propagation environment: urban, suburban, rural, "
                          "dense-urban, maritime, aeronautical",
                          StringValue("urban"),
                          MakeStringAccessor(&OranNtnChannelModel::m_environment),
                          MakeStringChecker())
            .AddAttribute("AtmosphericAttenuation",
                          "Enable atmospheric attenuation model (gas, rain, scintillation)",
                          BooleanValue(false),
                          MakeBooleanAccessor(&OranNtnChannelModel::m_atmosphericAttenuation),
                          MakeBooleanChecker())
            .AddAttribute("LooFading",
                          "Enable Loo three-state fading model",
                          BooleanValue(true),
                          MakeBooleanAccessor(&OranNtnChannelModel::m_looFading),
                          MakeBooleanChecker())
            .AddAttribute("MarkovFading",
                          "Enable Markov state transition fading (clear/shadow/blocked)",
                          BooleanValue(true),
                          MakeBooleanAccessor(&OranNtnChannelModel::m_markovFading),
                          MakeBooleanChecker())
            .AddAttribute("CorrelatedShadowFading",
                          "Enable correlated shadow fading across beams",
                          BooleanValue(false),
                          MakeBooleanAccessor(&OranNtnChannelModel::m_correlatedShadowFading),
                          MakeBooleanChecker())
            .AddAttribute("RainRate",
                          "Rain rate in mm/h for ITU-R P.837 rain attenuation",
                          DoubleValue(25.0),
                          MakeDoubleAccessor(&OranNtnChannelModel::m_rainRate_mm_h),
                          MakeDoubleChecker<double>(0.0))
            .AddTraceSource("PathLossComputed",
                            "Trace fired when path loss is computed for a link",
                            MakeTraceSourceAccessor(&OranNtnChannelModel::m_pathLossComputed),
                            "ns3::OranNtnChannelModel::PathLossComputed")
            .AddTraceSource("MarkovTransition",
                            "Trace fired on Markov state transition",
                            MakeTraceSourceAccessor(&OranNtnChannelModel::m_markovTransition),
                            "ns3::OranNtnChannelModel::MarkovTransition")
            .AddTraceSource("FadingComputed",
                            "Trace fired when fading gain is computed",
                            MakeTraceSourceAccessor(&OranNtnChannelModel::m_fadingComputed),
                            "ns3::OranNtnChannelModel::FadingComputed");

    return tid;
}

OranNtnChannelModel::OranNtnChannelModel()
    : m_satBridge(nullptr),
      m_band("S-band"),
      m_environment("urban"),
      m_centerFreqHz(2e9),
      m_atmosphericAttenuation(false),
      m_looFading(true),
      m_markovFading(true),
      m_correlatedShadowFading(false),
      m_rainRate_mm_h(25.0),
      m_rng(std::random_device{}()),
      m_normalDist(0.0, 1.0),
      m_uniformDist(0.0, 1.0)
{
    NS_LOG_FUNCTION(this);
}

OranNtnChannelModel::~OranNtnChannelModel()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnChannelModel::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_satBridge = nullptr;
    m_markovStates.clear();
    PropagationLossModel::DoDispose();
}

int64_t
OranNtnChannelModel::DoAssignStreams(int64_t stream)
{
    NS_LOG_FUNCTION(this << stream);
    m_rng.seed(static_cast<uint32_t>(stream));
    return 1;
}

void
OranNtnChannelModel::SetSatBridge(Ptr<OranNtnSatBridge> bridge)
{
    NS_LOG_FUNCTION(this << bridge);
    m_satBridge = bridge;
}

void
OranNtnChannelModel::SetBand(const std::string& band)
{
    NS_LOG_FUNCTION(this << band);
    m_band = band;

    if (band == "Ka-band")
    {
        m_centerFreqHz = 20e9; // downlink Ka-band
    }
    else if (band == "S-band")
    {
        m_centerFreqHz = 2e9;
    }
    else if (band == "L-band")
    {
        m_centerFreqHz = 1.5e9;
    }
    else
    {
        NS_LOG_WARN("Unknown band '" << band << "', defaulting to S-band (2 GHz)");
        m_centerFreqHz = 2e9;
    }
}

void
OranNtnChannelModel::SetEnvironment(const std::string& env)
{
    NS_LOG_FUNCTION(this << env);
    m_environment = env;
}

void
OranNtnChannelModel::SetAtmosphericAttenuation(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_atmosphericAttenuation = enable;
}

void
OranNtnChannelModel::SetLooFading(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_looFading = enable;
}

void
OranNtnChannelModel::SetMarkovFading(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_markovFading = enable;
}

void
OranNtnChannelModel::SetCorrelatedShadowFading(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_correlatedShadowFading = enable;
}

void
OranNtnChannelModel::SetRainRate(double rainRate_mm_h)
{
    NS_LOG_FUNCTION(this << rainRate_mm_h);
    m_rainRate_mm_h = rainRate_mm_h;
}

uint8_t
OranNtnChannelModel::GetMarkovState(uint32_t ueId, uint32_t satId) const
{
    uint64_t key = (static_cast<uint64_t>(ueId) << 32) | satId;
    auto it = m_markovStates.find(key);
    if (it != m_markovStates.end())
    {
        return it->second.currentState;
    }
    return 0; // default: clear
}

double
OranNtnChannelModel::GetFadingGain_dB(uint32_t ueId, uint32_t satId) const
{
    uint64_t key = (static_cast<uint64_t>(ueId) << 32) | satId;
    auto it = m_markovStates.find(key);
    if (it != m_markovStates.end())
    {
        return it->second.fadingGain_dB;
    }
    return 0.0;
}

// ---------------------------------------------------------------------------
// Core channel computation
// ---------------------------------------------------------------------------

double
OranNtnChannelModel::DoCalcRxPower(double txPowerDbm,
                                    Ptr<MobilityModel> a,
                                    Ptr<MobilityModel> b) const
{
    NS_LOG_FUNCTION(this << txPowerDbm);

    // Distance between TX (satellite) and RX (UE) mobility models
    double distanceM = a->GetDistanceFrom(b);
    if (distanceM < 1.0)
    {
        distanceM = 1.0; // avoid log(0)
    }

    double freqHz = m_centerFreqHz;

    // ---- 1. Free-space path loss ----
    double fspl_dB = ComputeFreeSpaceLoss(distanceM, freqHz);

    // ---- 2. Compute elevation angle (approximate) ----
    // Elevation from UE perspective: use position difference
    Vector posA = a->GetPosition(); // satellite
    Vector posB = b->GetPosition(); // UE
    double dx = posA.x - posB.x;
    double dy = posA.y - posB.y;
    double dz = posA.z - posB.z;
    double horizontalDist = std::sqrt(dx * dx + dy * dy);
    double elevationDeg = std::atan2(dz, horizontalDist) * (180.0 / M_PI);
    elevationDeg = std::max(5.0, std::min(90.0, elevationDeg)); // clamp to [5, 90]

    NS_LOG_DEBUG("Distance=" << distanceM << " m, Elevation=" << elevationDeg
                             << " deg, FSPL=" << fspl_dB << " dB");

    // ---- 3. Atmospheric attenuation (primarily Ka-band) ----
    double atmosphericLoss_dB = 0.0;
    if (m_atmosphericAttenuation)
    {
        atmosphericLoss_dB = ComputeAtmosphericLoss(elevationDeg, freqHz);
        NS_LOG_DEBUG("Atmospheric loss=" << atmosphericLoss_dB << " dB");
    }

    // ---- 4. Loo fading gain ----
    double fadingGain_dB = 0.0;
    if (m_looFading)
    {
        fadingGain_dB = ComputeLooFading(elevationDeg, m_environment);
        NS_LOG_DEBUG("Loo fading gain=" << fadingGain_dB << " dB");
    }

    // ---- 5. Markov state-dependent fading ----
    double markovGain_dB = 0.0;
    if (m_markovFading)
    {
        // Use node IDs as ueId/satId (extract from mobility model object addresses)
        uint32_t ueId = static_cast<uint32_t>(
            reinterpret_cast<uintptr_t>(PeekPointer(b)) & 0xFFFF);
        uint32_t satId = static_cast<uint32_t>(
            reinterpret_cast<uintptr_t>(PeekPointer(a)) & 0xFFFF);

        UpdateMarkovState(ueId, satId, elevationDeg);

        uint64_t key = (static_cast<uint64_t>(ueId) << 32) | satId;
        auto it = m_markovStates.find(key);
        if (it != m_markovStates.end())
        {
            markovGain_dB = it->second.fadingGain_dB;
        }

        NS_LOG_DEBUG("Markov gain=" << markovGain_dB << " dB, state="
                                     << (int)GetMarkovState(ueId, satId));
    }

    // ---- 6. Correlated shadow fading ----
    double shadowFading_dB = 0.0;
    if (m_correlatedShadowFading)
    {
        uint32_t ueId = static_cast<uint32_t>(
            reinterpret_cast<uintptr_t>(PeekPointer(b)) & 0xFFFF);
        uint32_t satId = static_cast<uint32_t>(
            reinterpret_cast<uintptr_t>(PeekPointer(a)) & 0xFFFF);
        uint32_t beamId = 0; // default beam

        shadowFading_dB = ComputeCorrelatedShadowFading(ueId, satId, beamId);
        NS_LOG_DEBUG("Correlated shadow fading=" << shadowFading_dB << " dB");
    }

    // ---- 7. Clutter loss ----
    double clutterLoss_dB = ComputeClutterLoss(elevationDeg, m_environment);
    NS_LOG_DEBUG("Clutter loss=" << clutterLoss_dB << " dB");

    // ---- Combine all components ----
    // rxPower = txPower - FSPL - atmospheric + fadingGain + markovGain - shadowFading - clutter
    double rxPowerDbm = txPowerDbm
                        - fspl_dB
                        - atmosphericLoss_dB
                        + fadingGain_dB
                        + markovGain_dB
                        - shadowFading_dB
                        - clutterLoss_dB;

    double totalPathLoss_dB = txPowerDbm - rxPowerDbm;

    NS_LOG_INFO("NTN Channel: txPower=" << txPowerDbm << " dBm, totalPL="
                << totalPathLoss_dB << " dB, rxPower=" << rxPowerDbm << " dBm");

    // Fire trace source
    uint32_t traceUeId = static_cast<uint32_t>(
        reinterpret_cast<uintptr_t>(PeekPointer(b)) & 0xFFFF);
    uint32_t traceSatId = static_cast<uint32_t>(
        reinterpret_cast<uintptr_t>(PeekPointer(a)) & 0xFFFF);
    m_pathLossComputed(traceUeId, traceSatId, totalPathLoss_dB);
    m_fadingComputed(traceUeId, traceSatId, fadingGain_dB + markovGain_dB);

    return rxPowerDbm;
}

// ---------------------------------------------------------------------------
// Free-space path loss: FSPL = 20*log10(4*pi*d*f/c)
// ---------------------------------------------------------------------------

double
OranNtnChannelModel::ComputeFreeSpaceLoss(double distanceM, double freqHz) const
{
    NS_LOG_FUNCTION(this << distanceM << freqHz);

    double fspl = 20.0 * std::log10((4.0 * M_PI * distanceM * freqHz) / SPEED_OF_LIGHT);
    return std::max(0.0, fspl);
}

// ---------------------------------------------------------------------------
// Atmospheric loss: gaseous + rain + scintillation
// ---------------------------------------------------------------------------

double
OranNtnChannelModel::ComputeAtmosphericLoss(double elevationDeg, double freqHz) const
{
    NS_LOG_FUNCTION(this << elevationDeg << freqHz);

    double gaseousLoss = ComputeGaseousLoss(elevationDeg, freqHz);
    double rainLoss = ComputeRainLoss(elevationDeg, freqHz);
    double scintillationLoss = ComputeScintillationLoss(elevationDeg, freqHz);

    double totalAtmospheric = gaseousLoss + rainLoss + scintillationLoss;

    NS_LOG_DEBUG("Atmospheric breakdown: gaseous=" << gaseousLoss
                 << " rain=" << rainLoss << " scintillation=" << scintillationLoss);

    return totalAtmospheric;
}

// ---------------------------------------------------------------------------
// Gaseous absorption (ITU-R P.676)
// Oxygen + water vapor, scaled by slant path through atmosphere
// ---------------------------------------------------------------------------

double
OranNtnChannelModel::ComputeGaseousLoss(double elevationDeg, double freqHz) const
{
    NS_LOG_FUNCTION(this << elevationDeg << freqHz);

    double freqGHz = freqHz / 1e9;

    // Scale absorption coefficients with frequency relative to 20 GHz reference
    // Oxygen absorption increases roughly quadratically below 60 GHz
    double freqScale = (freqGHz / 20.0);
    freqScale = std::max(0.1, std::min(freqScale, 5.0));

    double oxygenAbs = OXYGEN_ABSORPTION_DB_PER_KM * freqScale;
    double waterVaporAbs = WATER_VAPOR_ABSORPTION_DB_PER_KM * freqScale;

    double totalAbsPerKm = oxygenAbs + waterVaporAbs;

    // Effective atmospheric thickness ~8 km for dry air, ~2 km for water vapor
    // Combined effective zenith path ~ 6 km for moderate humidity
    double zenithPathKm = 6.0;

    // Slant path factor: 1/sin(elevation)
    double sinElev = std::sin(elevationDeg * DEG_TO_RAD);
    sinElev = std::max(sinElev, std::sin(5.0 * DEG_TO_RAD)); // minimum 5° elevation
    double slantFactor = 1.0 / sinElev;

    double gaseousLoss_dB = totalAbsPerKm * zenithPathKm * slantFactor;

    return gaseousLoss_dB;
}

// ---------------------------------------------------------------------------
// Rain attenuation (ITU-R P.618 / P.838)
// Specific attenuation: gamma_R = k * R^alpha
// ---------------------------------------------------------------------------

double
OranNtnChannelModel::ComputeRainLoss(double elevationDeg, double freqHz) const
{
    NS_LOG_FUNCTION(this << elevationDeg << freqHz);

    if (m_rainRate_mm_h <= 0.0)
    {
        return 0.0;
    }

    double freqGHz = freqHz / 1e9;

    // ITU-R P.838 regression coefficients scale with frequency
    // Use Ka-band reference values and scale for other bands
    double kCoeff = RAIN_K_COEFF;
    double alphaCoeff = RAIN_ALPHA_COEFF;

    // Approximate frequency scaling of k coefficient (increases with freq)
    if (freqGHz < 10.0)
    {
        kCoeff *= std::pow(freqGHz / 20.0, 2.0);
    }
    else if (freqGHz > 30.0)
    {
        kCoeff *= std::pow(freqGHz / 20.0, 1.2);
    }
    else
    {
        kCoeff *= std::pow(freqGHz / 20.0, 1.6);
    }

    // Specific attenuation (dB/km) from rain rate
    double gammaR = kCoeff * std::pow(m_rainRate_mm_h, alphaCoeff);

    // Effective path length through rain (slant path, limited by rain height)
    double rainHeightKm = 4.0; // typical rain height for mid-latitudes
    double sinElev = std::sin(elevationDeg * DEG_TO_RAD);
    sinElev = std::max(sinElev, std::sin(5.0 * DEG_TO_RAD));
    double slantPathKm = rainHeightKm / sinElev;

    // Horizontal reduction factor (rain cells are finite)
    double horizontalPathKm = slantPathKm * std::cos(elevationDeg * DEG_TO_RAD);
    double reductionFactor = 1.0 / (1.0 + horizontalPathKm / RAIN_EFFECTIVE_PATH_KM);

    double rainLoss_dB = gammaR * slantPathKm * reductionFactor;

    return rainLoss_dB;
}

// ---------------------------------------------------------------------------
// Tropospheric scintillation (ITU-R P.531)
// Standard deviation model: sigma = 0.5 / sin(elev)^1.2 * (freq/4)^0.6
// ---------------------------------------------------------------------------

double
OranNtnChannelModel::ComputeScintillationLoss(double elevationDeg, double freqHz) const
{
    NS_LOG_FUNCTION(this << elevationDeg << freqHz);

    double freqGHz = freqHz / 1e9;

    double sinElev = std::sin(elevationDeg * DEG_TO_RAD);
    sinElev = std::max(sinElev, std::sin(5.0 * DEG_TO_RAD));

    // Scintillation standard deviation (dB)
    double sigma_dB = 0.5 / std::pow(sinElev, 1.2)
                      * std::pow(freqGHz / SCINTILLATION_REF_FREQ_GHZ, 0.6);

    // Generate random scintillation fade (Gaussian)
    // Use absolute value to represent fade depth (loss is always positive)
    double randomSample = m_normalDist(m_rng);
    double scintillationLoss_dB = std::abs(sigma_dB * randomSample);

    return scintillationLoss_dB;
}

// ---------------------------------------------------------------------------
// Loo fading model: three-component model
// Direct (lognormal) + scattered (Rayleigh) superposition
// ---------------------------------------------------------------------------

double
OranNtnChannelModel::ComputeLooFading(double elevationDeg, const std::string& env) const
{
    NS_LOG_FUNCTION(this << elevationDeg << env);

    LooParams params = GetLooParameters(env, elevationDeg);

    // K-factor determines the ratio of direct to scattered power
    double kFactor_dB = ComputeRicianKFactor(elevationDeg);
    double kFactor_lin = std::pow(10.0, kFactor_dB / 10.0);

    // Direct component: lognormally distributed amplitude
    // ln(amplitude) ~ N(mean_dB, std_dB) in dB domain
    double directAmplitude_dB = params.meanDirect_dB
                                 + params.stdDirect_dB * m_normalDist(m_rng);
    double directPower_lin = std::pow(10.0, directAmplitude_dB / 10.0);

    // Scale direct component by K-factor
    directPower_lin *= kFactor_lin / (1.0 + kFactor_lin);

    // Scattered (Rayleigh) component
    double scatteredPower_lin = std::pow(10.0, params.scatteredPower_dB / 10.0);
    // Rayleigh: sum of two independent Gaussian quadrature components
    double realPart = m_normalDist(m_rng);
    double imagPart = m_normalDist(m_rng);
    double rayleighPower = (realPart * realPart + imagPart * imagPart) / 2.0;
    double scattered_lin = scatteredPower_lin * rayleighPower / (1.0 + kFactor_lin);

    // Combine direct and scattered power
    double totalPower_lin = directPower_lin + scattered_lin;

    // Ensure we don't take log of zero/negative
    if (totalPower_lin <= 1e-20)
    {
        totalPower_lin = 1e-20;
    }

    // Convert to dB: gain relative to unity (no fading = 0 dB)
    double fadingGain_dB = 10.0 * std::log10(totalPower_lin);

    // Apply mean loss adjustment (environment-dependent average fading)
    double meanLoss_dB = -params.meanDirect_dB; // compensate so average ~ 0 dB
    fadingGain_dB -= meanLoss_dB;

    return fadingGain_dB;
}

// ---------------------------------------------------------------------------
// Markov state machine: clear / shadow / blocked
// Transition probabilities depend on elevation angle
// ---------------------------------------------------------------------------

void
OranNtnChannelModel::UpdateMarkovState(uint32_t ueId,
                                        uint32_t satId,
                                        double elevationDeg) const
{
    NS_LOG_FUNCTION(this << ueId << satId << elevationDeg);

    uint64_t key = (static_cast<uint64_t>(ueId) << 32) | satId;
    double now = Simulator::Now().GetSeconds();

    // Initialize state if not seen before
    auto it = m_markovStates.find(key);
    if (it == m_markovStates.end())
    {
        MarkovLinkState initState;
        initState.currentState = 0;     // start in clear
        initState.lastTransitionTime = now;
        initState.fadingGain_dB = 0.0;
        initState.shadowFading_dB = 0.0;
        m_markovStates[key] = initState;
        it = m_markovStates.find(key);
    }

    MarkovLinkState& state = it->second;

    // Minimum state duration before checking transition (100 ms)
    double stateDuration = now - state.lastTransitionTime;
    if (stateDuration < 0.1 && state.lastTransitionTime > 0.0)
    {
        // No transition check yet, just update fading for current state
        return;
    }

    // Transition probability matrices [from][to] for one time step
    // Indexed: 0=clear, 1=shadow, 2=blocked
    double P[3][3];

    if (elevationDeg > 60.0)
    {
        // High elevation: mostly clear state
        P[0][0] = 0.95;  P[0][1] = 0.04;  P[0][2] = 0.01;
        P[1][0] = 0.30;  P[1][1] = 0.60;  P[1][2] = 0.10;
        P[2][0] = 0.20;  P[2][1] = 0.30;  P[2][2] = 0.50;
    }
    else if (elevationDeg > 30.0)
    {
        // Medium elevation: moderate shadowing
        P[0][0] = 0.85;  P[0][1] = 0.10;  P[0][2] = 0.05;
        P[1][0] = 0.15;  P[1][1] = 0.70;  P[1][2] = 0.15;
        P[2][0] = 0.10;  P[2][1] = 0.25;  P[2][2] = 0.65;
    }
    else
    {
        // Low elevation (<30°): frequent blockage
        P[0][0] = 0.70;  P[0][1] = 0.20;  P[0][2] = 0.10;
        P[1][0] = 0.10;  P[1][1] = 0.65;  P[1][2] = 0.25;
        P[2][0] = 0.05;  P[2][1] = 0.20;  P[2][2] = 0.75;
    }

    // Draw random number and determine next state
    uint8_t currentState = state.currentState;
    double u = m_uniformDist(m_rng);
    uint8_t newState;

    if (u < P[currentState][0])
    {
        newState = 0; // clear
    }
    else if (u < P[currentState][0] + P[currentState][1])
    {
        newState = 1; // shadow
    }
    else
    {
        newState = 2; // blocked
    }

    // Apply state-dependent fading gain
    double fadingGain_dB = 0.0;
    switch (newState)
    {
    case 0: // Clear: small Rician fading
    {
        double kFactor_dB = ComputeRicianKFactor(elevationDeg);
        double kFactor_lin = std::pow(10.0, kFactor_dB / 10.0);
        // Rician fading variance decreases with K
        double sigma = 1.0 / std::sqrt(2.0 * (1.0 + kFactor_lin));
        double real = std::sqrt(kFactor_lin / (1.0 + kFactor_lin))
                      + sigma * m_normalDist(m_rng);
        double imag = sigma * m_normalDist(m_rng);
        double power = real * real + imag * imag;
        fadingGain_dB = 10.0 * std::log10(std::max(power, 1e-20));
        break;
    }
    case 1: // Shadow: moderate attenuation with slow fading
    {
        // Shadowed Rician: reduced mean power
        double shadowMean_dB = -5.0 - 0.1 * (60.0 - elevationDeg);
        shadowMean_dB = std::max(shadowMean_dB, -15.0);
        double shadowStd_dB = 3.0;
        fadingGain_dB = shadowMean_dB + shadowStd_dB * m_normalDist(m_rng);
        break;
    }
    case 2: // Blocked: deep fade
    {
        double blockMean_dB = -20.0 - 0.15 * (60.0 - elevationDeg);
        blockMean_dB = std::max(blockMean_dB, -35.0);
        double blockStd_dB = 5.0;
        fadingGain_dB = blockMean_dB + blockStd_dB * m_normalDist(m_rng);
        fadingGain_dB = std::min(fadingGain_dB, -10.0); // always significant loss
        break;
    }
    }

    // Fire trace if state changed
    if (newState != currentState)
    {
        NS_LOG_INFO("Markov transition: UE=" << ueId << " SAT=" << satId
                    << " state " << (int)currentState << " -> " << (int)newState
                    << " at elev=" << elevationDeg << " deg");
        m_markovTransition(ueId, satId, newState);
        state.lastTransitionTime = now;
    }

    state.currentState = newState;
    state.fadingGain_dB = fadingGain_dB;
}

// ---------------------------------------------------------------------------
// Rician K-factor: 3GPP TR 38.811 Table 6.7.2-1
// K = 10 + 0.2 * (elevation - 30) dB for elevation > 10°
// ---------------------------------------------------------------------------

double
OranNtnChannelModel::ComputeRicianKFactor(double elevationDeg) const
{
    NS_LOG_FUNCTION(this << elevationDeg);

    if (elevationDeg <= 10.0)
    {
        // Very low elevation: weak LOS, low K-factor
        return 2.0; // ~2 dB
    }

    // TR 38.811: linear increase with elevation
    double kFactor_dB = 10.0 + 0.2 * (elevationDeg - 30.0);

    // Clamp to reasonable range [2, 20] dB
    kFactor_dB = std::max(2.0, std::min(kFactor_dB, 20.0));

    return kFactor_dB;
}

// ---------------------------------------------------------------------------
// Clutter loss: environment-dependent terrestrial obstruction
// Decreases with increasing elevation (better LOS clearance)
// ---------------------------------------------------------------------------

double
OranNtnChannelModel::ComputeClutterLoss(double elevationDeg, const std::string& env) const
{
    NS_LOG_FUNCTION(this << elevationDeg << env);

    // Base clutter loss values at zenith (90° elevation) and horizon (5° elevation)
    double clutterMin_dB; // at high elevation
    double clutterMax_dB; // at low elevation

    if (env == "dense-urban")
    {
        clutterMin_dB = 15.0;
        clutterMax_dB = 25.0;
    }
    else if (env == "urban")
    {
        clutterMin_dB = 12.0;
        clutterMax_dB = 22.0;
    }
    else if (env == "suburban")
    {
        clutterMin_dB = 8.0;
        clutterMax_dB = 15.0;
    }
    else if (env == "rural")
    {
        clutterMin_dB = 3.0;
        clutterMax_dB = 10.0;
    }
    else if (env == "maritime" || env == "aeronautical")
    {
        // Minimal clutter over water/air
        clutterMin_dB = 0.0;
        clutterMax_dB = 2.0;
    }
    else
    {
        // Default: suburban-like
        clutterMin_dB = 8.0;
        clutterMax_dB = 15.0;
    }

    // Linear interpolation with elevation: lower elevation → more clutter
    // Normalize elevation from [5°, 90°] to [0, 1]
    double normalizedElev = (elevationDeg - 5.0) / 85.0;
    normalizedElev = std::max(0.0, std::min(1.0, normalizedElev));

    // Apply a smooth transition (cosine-based for more realistic rolloff)
    double smoothFactor = 0.5 * (1.0 - std::cos(M_PI * normalizedElev));

    double clutterLoss_dB = clutterMax_dB - smoothFactor * (clutterMax_dB - clutterMin_dB);

    return clutterLoss_dB;
}

// ---------------------------------------------------------------------------
// Correlated shadow fading across satellite beams
// Uses log-normal shadow fading with inter-beam spatial correlation
// ---------------------------------------------------------------------------

double
OranNtnChannelModel::ComputeCorrelatedShadowFading(uint32_t ueId,
                                                     uint32_t satId,
                                                     uint32_t beamId) const
{
    NS_LOG_FUNCTION(this << ueId << satId << beamId);

    uint64_t key = (static_cast<uint64_t>(ueId) << 32) | satId;

    // Shadow fading standard deviation depends on environment
    double sigma_dB;
    if (m_environment == "urban" || m_environment == "dense-urban")
    {
        sigma_dB = 8.0; // 3GPP typical for urban NTN
    }
    else if (m_environment == "suburban")
    {
        sigma_dB = 6.0;
    }
    else if (m_environment == "rural")
    {
        sigma_dB = 4.0;
    }
    else
    {
        sigma_dB = 3.0; // maritime/aeronautical
    }

    // Inter-beam correlation coefficient
    // Adjacent beams have high correlation (~0.8), distant beams lower
    double correlationCoeff = 0.5; // moderate default correlation

    auto it = m_markovStates.find(key);
    double previousShadow = 0.0;
    if (it != m_markovStates.end())
    {
        previousShadow = it->second.shadowFading_dB;
    }

    // Correlated shadow fading: sf_new = rho * sf_prev + sqrt(1-rho^2) * sigma * N(0,1)
    double innovation = sigma_dB * m_normalDist(m_rng);
    double correlatedShadow = correlationCoeff * previousShadow
                              + std::sqrt(1.0 - correlationCoeff * correlationCoeff)
                                * innovation;

    // Store the shadow fading value for next correlation step
    if (it != m_markovStates.end())
    {
        it->second.shadowFading_dB = correlatedShadow;
    }
    else
    {
        MarkovLinkState newState;
        newState.currentState = 0;
        newState.lastTransitionTime = Simulator::Now().GetSeconds();
        newState.fadingGain_dB = 0.0;
        newState.shadowFading_dB = correlatedShadow;
        m_markovStates[key] = newState;
    }

    // Shadow fading is a loss (positive value means attenuation)
    return correlatedShadow;
}

} // namespace ns3
