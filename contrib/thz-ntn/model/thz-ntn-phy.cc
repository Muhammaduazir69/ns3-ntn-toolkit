/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Base THz-NTN PHY Layer Model -- Implementation
 *
 * Integrates with ns-3 SpectrumValue, mmWaveInterference, MmWaveAmc, and
 * MobilityModel to produce spectrum-based SINR, MCS selection from actual
 * AMC tables, Doppler from actual velocity vectors, and proper thermal +
 * shot + hardware impairment noise.
 */

#include "thz-ntn-phy.h"

#include "thz-ntn-hardware-impairments.h"

#include <ns3/double.h>
#include <ns3/enum.h>
#include <ns3/log.h>
#include <ns3/mobility-model.h>
#include <ns3/spectrum-model.h>
#include <ns3/spectrum-value.h>
#include <ns3/uinteger.h>

#include <ns3/mmwave-amc.h>
#include <ns3/mmwave-interference.h>
#include <ns3/mmwave-phy-mac-common.h>

#include <cmath>
#include <numeric>
#include <vector>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnPhy");

NS_OBJECT_ENSURE_REGISTERED(ThzNtnPhy);

TypeId
ThzNtnPhy::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnPhy")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnPhy>()
            .AddAttribute("CenterFrequency",
                          "Center frequency in Hz.",
                          DoubleValue(225e9),
                          MakeDoubleAccessor(&ThzNtnPhy::m_centerFreqHz),
                          MakeDoubleChecker<double>(1e9, 10e12))
            .AddAttribute("Bandwidth",
                          "Channel bandwidth in Hz.",
                          DoubleValue(10e9),
                          MakeDoubleAccessor(&ThzNtnPhy::m_bandwidthHz),
                          MakeDoubleChecker<double>(1e6, 100e9))
            .AddAttribute("TxPower",
                          "Transmit power in dBm.",
                          DoubleValue(30.0),
                          MakeDoubleAccessor(&ThzNtnPhy::m_txPowerDbm),
                          MakeDoubleChecker<double>(-30.0, 60.0))
            .AddAttribute("NoiseFigure",
                          "Receiver noise figure in dB.",
                          DoubleValue(10.0),
                          MakeDoubleAccessor(&ThzNtnPhy::m_noiseFigureDb),
                          MakeDoubleChecker<double>(0.0, 30.0))
            .AddAttribute("NoiseTemperature",
                          "Receiver noise temperature in Kelvin.",
                          DoubleValue(290.0),
                          MakeDoubleAccessor(&ThzNtnPhy::m_noiseTemperatureK),
                          MakeDoubleChecker<double>(10.0, 10000.0))
            .AddAttribute("NumSubBands",
                          "Number of sub-bands in the THz spectrum model.",
                          UintegerValue(100),
                          MakeUintegerAccessor(&ThzNtnPhy::m_numSubBands),
                          MakeUintegerChecker<uint32_t>(1, 10000))
            .AddAttribute("Waveform",
                          "Waveform type for THz-NTN PHY.",
                          EnumValue(WAVEFORM_OFDM),
                          MakeEnumAccessor<WaveformType>(&ThzNtnPhy::m_waveform),
                          MakeEnumChecker(WAVEFORM_OFDM, "OFDM",
                                          WAVEFORM_DFT_S_OFDM, "DFT_S_OFDM",
                                          WAVEFORM_OTFS, "OTFS",
                                          WAVEFORM_AFDM, "AFDM",
                                          WAVEFORM_SC_FDE, "SC_FDE"))
            .AddTraceSource("SinrTrace",
                            "Reports SINR (dB), RxPower (dBm), Interference (dBm)",
                            MakeTraceSourceAccessor(&ThzNtnPhy::m_sinrTrace),
                            "ns3::ThzNtnPhy::SinrTracedCallback")
            .AddTraceSource("McsTrace",
                            "Reports MCS index and spectral efficiency",
                            MakeTraceSourceAccessor(&ThzNtnPhy::m_mcsTrace),
                            "ns3::ThzNtnPhy::McsTracedCallback")
            .AddTraceSource("DopplerTrace",
                            "Reports Doppler shift (Hz) and SCS (Hz)",
                            MakeTraceSourceAccessor(&ThzNtnPhy::m_dopplerTrace),
                            "ns3::ThzNtnPhy::DopplerTracedCallback")
            .AddTraceSource("ThroughputTrace",
                            "Reports throughput in bps",
                            MakeTraceSourceAccessor(&ThzNtnPhy::m_throughputTrace),
                            "ns3::ThzNtnPhy::ThroughputTracedCallback");
    return tid;
}

ThzNtnPhy::ThzNtnPhy()
    : m_centerFreqHz(225e9),
      m_bandwidthHz(10e9),
      m_waveform(WAVEFORM_OFDM),
      m_txPowerDbm(30.0),
      m_noiseFigureDb(10.0),
      m_noiseTemperatureK(290.0),
      m_numSubBands(100),
      m_linkState(LINK_IDLE),
      m_lastSinrDb(-100.0),
      m_spectrumModel(nullptr),
      m_interferenceModel(nullptr),
      m_amcModel(nullptr),
      m_phyMacCommon(nullptr),
      m_hwImpairments(nullptr)
{
    NS_LOG_FUNCTION(this);
}

ThzNtnPhy::~ThzNtnPhy()
{
    NS_LOG_FUNCTION(this);
}

void
ThzNtnPhy::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_spectrumModel = nullptr;
    m_interferenceModel = nullptr;
    m_amcModel = nullptr;
    m_phyMacCommon = nullptr;
    m_hwImpairments = nullptr;
    Object::DoDispose();
}

// ---------------------------------------------------------------------------
// SpectrumModel and PSD creation
// ---------------------------------------------------------------------------

void
ThzNtnPhy::EnsureSpectrumModel() const
{
    if (m_spectrumModel == nullptr)
    {
        // const_cast is needed because this is a lazy-init cache
        const_cast<ThzNtnPhy*>(this)->m_spectrumModel =
            const_cast<ThzNtnPhy*>(this)->CreateThzSpectrumModel(
                m_centerFreqHz, m_bandwidthHz, m_numSubBands);
    }
}

Ptr<SpectrumModel>
ThzNtnPhy::CreateThzSpectrumModel(double centerFreqHz,
                                    double bandwidthHz,
                                    uint32_t numSubBands) const
{
    NS_LOG_FUNCTION(this << centerFreqHz << bandwidthHz << numSubBands);

    double subBandWidth = bandwidthHz / numSubBands;
    double startFreq = centerFreqHz - bandwidthHz / 2.0;

    Bands bands;
    for (uint32_t i = 0; i < numSubBands; ++i)
    {
        BandInfo bi;
        bi.fl = startFreq + i * subBandWidth;
        bi.fc = bi.fl + subBandWidth / 2.0;
        bi.fh = bi.fl + subBandWidth;
        bands.push_back(bi);
    }

    Ptr<SpectrumModel> sm = Create<SpectrumModel>(bands);
    NS_LOG_INFO("Created THz SpectrumModel: center=" << centerFreqHz / 1e9
                << " GHz, BW=" << bandwidthHz / 1e9
                << " GHz, " << numSubBands << " sub-bands, sub-band width="
                << subBandWidth / 1e6 << " MHz");
    return sm;
}

Ptr<SpectrumValue>
ThzNtnPhy::CreateTxPsd(double txPowerDbm) const
{
    NS_LOG_FUNCTION(this << txPowerDbm);

    EnsureSpectrumModel();

    Ptr<SpectrumValue> txPsd = Create<SpectrumValue>(m_spectrumModel);

    // Convert dBm to Watts: P_W = 10^((P_dBm - 30) / 10)
    double totalPower_W = std::pow(10.0, (txPowerDbm - 30.0) / 10.0);

    // Distribute power uniformly across sub-bands
    // PSD = total power / total bandwidth (W/Hz)
    double psdValue = totalPower_W / m_bandwidthHz;

    Values::iterator vit = txPsd->ValuesBegin();
    for (; vit != txPsd->ValuesEnd(); ++vit)
    {
        *vit = psdValue;
    }

    NS_LOG_DEBUG("TxPSD: totalPower=" << txPowerDbm << " dBm ("
                 << totalPower_W * 1e3 << " mW), PSD="
                 << 10.0 * std::log10(psdValue) << " dB(W/Hz)");

    return txPsd;
}

Ptr<SpectrumValue>
ThzNtnPhy::CreateNoisePsd() const
{
    NS_LOG_FUNCTION(this);

    EnsureSpectrumModel();

    Ptr<SpectrumValue> noisePsd = Create<SpectrumValue>(m_spectrumModel);

    // Noise figure in linear
    double nfLinear = std::pow(10.0, m_noiseFigureDb / 10.0);

    // Thermal noise PSD: k * T * F (W/Hz)
    double thermalNoisePsd = BOLTZMANN_K * m_noiseTemperatureK * nfLinear;

    // Shot noise PSD: 2 * q * I_photo (W/Hz) for direct-detection receivers
    double shotNoisePsd = 2.0 * ELECTRON_CHARGE * PHOTO_CURRENT_A;

    // Hardware impairment noise contribution
    double hwNoisePsd = 0.0;
    if (m_hwImpairments != nullptr)
    {
        // Hardware impairment noise is proportional to thermal noise * kappa^2
        double kappaSq = m_hwImpairments->ComputeEvmTotal();
        kappaSq *= kappaSq; // square the EVM to get kappa^2
        hwNoisePsd = kappaSq * thermalNoisePsd;
    }

    double totalNoisePsd = thermalNoisePsd + shotNoisePsd + hwNoisePsd;

    Values::iterator vit = noisePsd->ValuesBegin();
    for (; vit != noisePsd->ValuesEnd(); ++vit)
    {
        *vit = totalNoisePsd;
    }

    NS_LOG_DEBUG("NoisePSD: thermal=" << 10.0 * std::log10(thermalNoisePsd)
                 << " dB(W/Hz), shot=" << 10.0 * std::log10(shotNoisePsd)
                 << " dB(W/Hz), hw=" << (hwNoisePsd > 0 ? 10.0 * std::log10(hwNoisePsd) : -999.0)
                 << " dB(W/Hz), total=" << 10.0 * std::log10(totalNoisePsd)
                 << " dB(W/Hz)");

    return noisePsd;
}

// ---------------------------------------------------------------------------
// Spectrum-based SINR computation
// ---------------------------------------------------------------------------

double
ThzNtnPhy::ComputeSinrFromSpectrum(Ptr<const SpectrumValue> rxPsd,
                                    Ptr<const SpectrumValue> noisePsd,
                                    Ptr<const SpectrumValue> interferencePsd) const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(rxPsd != nullptr, "rxPsd must not be null");
    NS_ASSERT_MSG(noisePsd != nullptr, "noisePsd must not be null");

    // Compute per-sub-band SINR and apply EESM mapping
    uint32_t numBands = 0;
    double sumExpNegSinr = 0.0;
    double totalRxPower_W = 0.0;
    double totalInterference_W = 0.0;
    double subBandWidth = m_bandwidthHz / m_numSubBands;

    Values::const_iterator rxIt = rxPsd->ConstValuesBegin();
    Values::const_iterator noiseIt = noisePsd->ConstValuesBegin();
    Values::const_iterator ifIt;
    bool hasInterference = (interferencePsd != nullptr);
    if (hasInterference)
    {
        ifIt = interferencePsd->ConstValuesBegin();
    }

    while (rxIt != rxPsd->ConstValuesEnd())
    {
        double rxPower = (*rxIt) * subBandWidth;   // W in this sub-band
        double noisePower = (*noiseIt) * subBandWidth;
        double ifPower = 0.0;
        if (hasInterference)
        {
            ifPower = (*ifIt) * subBandWidth;
            ++ifIt;
        }

        totalRxPower_W += rxPower;
        totalInterference_W += ifPower;

        double sinrLinear = rxPower / (noisePower + ifPower);
        if (sinrLinear > 0.0)
        {
            // EESM: sum of exp(-SINR_i / beta)
            sumExpNegSinr += std::exp(-sinrLinear / EESM_BETA);
            numBands++;
        }

        ++rxIt;
        ++noiseIt;
    }

    // Effective SINR via EESM
    double effectiveSinr_dB = -100.0;
    if (numBands > 0 && sumExpNegSinr > 0.0)
    {
        double effectiveSinrLinear = -EESM_BETA * std::log(sumExpNegSinr / numBands);
        if (effectiveSinrLinear > 0.0)
        {
            effectiveSinr_dB = 10.0 * std::log10(effectiveSinrLinear);
        }
    }

    m_lastSinrDb = effectiveSinr_dB;

    // Compute power values in dBm for tracing
    double rxPower_dBm = (totalRxPower_W > 0)
                             ? 10.0 * std::log10(totalRxPower_W * 1000.0)
                             : -200.0;
    double ifPower_dBm = (totalInterference_W > 0)
                             ? 10.0 * std::log10(totalInterference_W * 1000.0)
                             : -200.0;

    // Fire trace
    m_sinrTrace(effectiveSinr_dB, rxPower_dBm, ifPower_dBm);

    NS_LOG_INFO("SINR from spectrum: effective SINR=" << effectiveSinr_dB
                << " dB, rxPower=" << rxPower_dBm
                << " dBm, interference=" << ifPower_dBm << " dBm");

    return effectiveSinr_dB;
}

// ---------------------------------------------------------------------------
// mmWaveInterference integration
// ---------------------------------------------------------------------------

void
ThzNtnPhy::SetInterferenceModel(Ptr<mmwave::mmWaveInterference> interference)
{
    NS_LOG_FUNCTION(this << interference);
    m_interferenceModel = interference;
}

Ptr<mmwave::mmWaveInterference>
ThzNtnPhy::GetInterferenceModel() const
{
    return m_interferenceModel;
}

void
ThzNtnPhy::AddInterference(Ptr<const SpectrumValue> interferencePsd, Time duration)
{
    NS_LOG_FUNCTION(this << duration);

    if (m_interferenceModel != nullptr)
    {
        m_interferenceModel->AddSignal(interferencePsd, duration);
        NS_LOG_DEBUG("Added interference signal to mmWaveInterference, duration="
                     << duration.GetMicroSeconds() << " us");
    }
    else
    {
        NS_LOG_WARN("No interference model set, ignoring AddInterference call");
    }
}

// ---------------------------------------------------------------------------
// MmWaveAmc integration
// ---------------------------------------------------------------------------

void
ThzNtnPhy::SetAmcModel(Ptr<mmwave::MmWaveAmc> amc)
{
    NS_LOG_FUNCTION(this << amc);
    m_amcModel = amc;
}

Ptr<mmwave::MmWaveAmc>
ThzNtnPhy::GetAmcModel() const
{
    return m_amcModel;
}

uint8_t
ThzNtnPhy::GetMcsFromSinr(double sinrDb) const
{
    NS_LOG_FUNCTION(this << sinrDb);

    if (m_amcModel != nullptr)
    {
        // Convert scalar SINR to a SpectrumValue for AMC's CQI computation
        EnsureSpectrumModel();
        SpectrumValue sinrSv(m_spectrumModel);
        double sinrLinear = std::pow(10.0, sinrDb / 10.0);
        Values::iterator vit = sinrSv.ValuesBegin();
        for (; vit != sinrSv.ValuesEnd(); ++vit)
        {
            *vit = sinrLinear;
        }

        uint8_t mcsWb = 0;
        m_amcModel->CreateCqiFeedbackWbTdma(sinrSv, mcsWb);

        double se = GetSpectralEfficiencyFromMcs(mcsWb);
        m_mcsTrace(mcsWb, se);

        NS_LOG_INFO("MCS from SINR=" << sinrDb << " dB -> MCS=" << static_cast<uint32_t>(mcsWb)
                    << ", SE=" << se << " bps/Hz");
        return mcsWb;
    }

    // Fallback: Shannon-based MCS estimation without AMC model
    // Map SINR to MCS 0-28 (NR range) using approximate thresholds
    NS_LOG_WARN("No AMC model set, using Shannon-based MCS fallback");
    double sinrLinear = std::pow(10.0, sinrDb / 10.0);
    double se = std::log2(1.0 + sinrLinear);
    // Approximate MCS from SE: MCS ~ floor(SE * 3.5), capped at 28
    uint8_t mcs = static_cast<uint8_t>(std::min(28.0, std::max(0.0, se * 3.5)));
    m_mcsTrace(mcs, se);
    return mcs;
}

double
ThzNtnPhy::GetSpectralEfficiencyFromMcs(uint8_t mcs) const
{
    NS_LOG_FUNCTION(this << static_cast<uint32_t>(mcs));

    if (m_amcModel != nullptr && m_phyMacCommon != nullptr)
    {
        // Use AMC's CalculateTbSize to derive SE
        // TBS for 1 symbol gives bits per symbol, then SE = bits / (numRb * rbWidth)
        uint32_t tbs = m_amcModel->CalculateTbSize(mcs, 1); // 1 OFDM symbol
        double numRb = m_phyMacCommon->GetNumRb();
        double rbWidth = m_phyMacCommon->GetRbWidth();
        double symbolPeriod_s = m_phyMacCommon->GetSymbolPeriod().GetSeconds();

        if (numRb > 0 && rbWidth > 0 && symbolPeriod_s > 0)
        {
            // bits per second from 1 symbol
            double bitsPerSec = (tbs * 8.0) / symbolPeriod_s;
            double totalBw = numRb * rbWidth;
            double se = bitsPerSec / totalBw;
            return se;
        }
    }

    // Fallback: approximate SE from MCS index
    // Using NR Table 5.1.3.1-1 approximate values
    static const double approxSe[] = {
        0.2344, 0.3066, 0.3770, 0.4902, 0.6016, 0.7402, 0.8770, 1.0273,
        1.1758, 1.3262, 1.3281, 1.4766, 1.6953, 1.9141, 2.1602, 2.4063,
        2.5703, 2.7305, 3.0293, 3.3223, 3.6094, 3.9023, 4.2129, 4.5234,
        4.8164, 5.1152, 5.3320, 5.5547, 5.8906
    };
    uint8_t idx = std::min(mcs, static_cast<uint8_t>(28));
    return approxSe[idx];
}

// ---------------------------------------------------------------------------
// Doppler computation from MobilityModel
// ---------------------------------------------------------------------------

double
ThzNtnPhy::ComputeDopplerFromMobility(Ptr<MobilityModel> satMobility,
                                        Ptr<MobilityModel> groundMobility) const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(satMobility != nullptr, "satMobility must not be null");
    NS_ASSERT_MSG(groundMobility != nullptr, "groundMobility must not be null");

    // Get positions
    Vector satPos = satMobility->GetPosition();
    Vector groundPos = groundMobility->GetPosition();

    // Get velocities
    Vector satVel = satMobility->GetVelocity();
    Vector groundVel = groundMobility->GetVelocity();

    // Relative position vector (ground to satellite)
    Vector relPos;
    relPos.x = satPos.x - groundPos.x;
    relPos.y = satPos.y - groundPos.y;
    relPos.z = satPos.z - groundPos.z;

    // Distance
    double dist = std::sqrt(relPos.x * relPos.x +
                            relPos.y * relPos.y +
                            relPos.z * relPos.z);

    if (dist < 1.0)
    {
        NS_LOG_WARN("Satellite and ground terminal nearly co-located, returning 0 Doppler");
        return 0.0;
    }

    // Unit vector from ground to satellite
    Vector unitVec;
    unitVec.x = relPos.x / dist;
    unitVec.y = relPos.y / dist;
    unitVec.z = relPos.z / dist;

    // Relative velocity vector
    Vector relVel;
    relVel.x = satVel.x - groundVel.x;
    relVel.y = satVel.y - groundVel.y;
    relVel.z = satVel.z - groundVel.z;

    // Radial velocity: projection of relative velocity onto line-of-sight
    // Negative means approaching (positive Doppler shift)
    double radialVel = -(relVel.x * unitVec.x +
                         relVel.y * unitVec.y +
                         relVel.z * unitVec.z);

    // Doppler shift: f_D = f_c * v_radial / c
    double dopplerHz = m_centerFreqHz * radialVel / SPEED_OF_LIGHT;

    NS_LOG_INFO("DopplerFromMobility: dist=" << dist / 1e3 << " km, radialVel="
                << radialVel << " m/s, Doppler=" << dopplerHz / 1e3 << " kHz");

    return dopplerHz;
}

uint32_t
ThzNtnPhy::SelectSubcarrierSpacing_Hz(Ptr<MobilityModel> satMobility,
                                        Ptr<MobilityModel> groundMobility) const
{
    NS_LOG_FUNCTION(this);

    // Available SCS options for THz bands (Hz)
    // 120 kHz (mu=3), 480 kHz (mu=5), 960 kHz (mu=6), 3.84 MHz (extended)
    static const uint32_t scsOptions[] = {120000, 480000, 960000, 3840000};
    static const uint32_t numOptions = 4;

    // Compute actual Doppler from mobility models
    double dopplerHz = 0.0;
    if (satMobility != nullptr && groundMobility != nullptr)
    {
        dopplerHz = std::abs(ComputeDopplerFromMobility(satMobility, groundMobility));
    }
    else
    {
        // Fallback: worst-case LEO at 550 km, v=7.6 km/s, 10 deg elevation
        double satVel_m_s = 7600.0;
        double elevRad = 10.0 * M_PI / 180.0;
        double radialVel = satVel_m_s * std::cos(elevRad);
        dopplerHz = m_centerFreqHz * radialVel / SPEED_OF_LIGHT;
        NS_LOG_WARN("No mobility models provided, using worst-case Doppler estimate");
    }

    // SCS must exceed 2 * f_D_max to avoid severe ICI
    double requiredScs = 2.0 * dopplerHz;

    NS_LOG_DEBUG("SCS selection: Doppler=" << dopplerHz / 1e3
                 << " kHz, required SCS > " << requiredScs / 1e3 << " kHz");

    // Select smallest SCS that satisfies the requirement
    uint32_t selectedScs = scsOptions[numOptions - 1];
    for (uint32_t i = 0; i < numOptions; ++i)
    {
        if (static_cast<double>(scsOptions[i]) > requiredScs)
        {
            selectedScs = scsOptions[i];
            break;
        }
    }

    // Fire Doppler trace
    m_dopplerTrace(dopplerHz, static_cast<double>(selectedScs));

    NS_LOG_INFO("Selected SCS: " << selectedScs / 1000 << " kHz for Doppler="
                << dopplerHz / 1e3 << " kHz");

    return selectedScs;
}

// ---------------------------------------------------------------------------
// Noise computation
// ---------------------------------------------------------------------------

double
ThzNtnPhy::ComputeNoiseFloor_dBm() const
{
    NS_LOG_FUNCTION(this);

    // Noise figure in linear
    double nfLinear = std::pow(10.0, m_noiseFigureDb / 10.0);

    // Thermal noise: k * T * B * F (in watts)
    double thermalNoise_W = BOLTZMANN_K * m_noiseTemperatureK * m_bandwidthHz * nfLinear;

    // Shot noise: 2 * q * I_photo * B
    double shotNoise_W = 2.0 * ELECTRON_CHARGE * PHOTO_CURRENT_A * m_bandwidthHz;

    // Hardware impairment noise
    double hwNoise_W = 0.0;
    if (m_hwImpairments != nullptr)
    {
        double kappaSq = m_hwImpairments->ComputeEvmTotal();
        kappaSq *= kappaSq;
        hwNoise_W = kappaSq * thermalNoise_W;
    }

    double totalNoise_W = thermalNoise_W + shotNoise_W + hwNoise_W;

    // Convert to dBm
    double noiseFloor_dBm = 10.0 * std::log10(totalNoise_W * 1000.0);

    NS_LOG_DEBUG("NoiseFloor: thermal=" << 10.0 * std::log10(thermalNoise_W * 1000.0)
                 << " dBm, shot=" << 10.0 * std::log10(shotNoise_W * 1000.0)
                 << " dBm, total=" << noiseFloor_dBm << " dBm"
                 << " (T=" << m_noiseTemperatureK << " K, NF="
                 << m_noiseFigureDb << " dB)");

    return noiseFloor_dBm;
}

// ---------------------------------------------------------------------------
// Throughput computation
// ---------------------------------------------------------------------------

double
ThzNtnPhy::ComputeThroughput_bps(double sinrDb) const
{
    NS_LOG_FUNCTION(this << sinrDb);

    uint8_t mcs = GetMcsFromSinr(sinrDb);
    double se = GetSpectralEfficiencyFromMcs(mcs);

    // Throughput = SE * BW * (1 - overhead)
    double throughput_bps = se * m_bandwidthHz * (1.0 - OVERHEAD_FRACTION);

    // Fire trace
    m_throughputTrace(throughput_bps);

    NS_LOG_INFO("Throughput: MCS=" << static_cast<uint32_t>(mcs)
                << ", SE=" << se << " bps/Hz, BW=" << m_bandwidthHz / 1e9
                << " GHz, throughput=" << throughput_bps / 1e9 << " Gbps");

    return throughput_bps;
}

// ---------------------------------------------------------------------------
// Setters and getters
// ---------------------------------------------------------------------------

void
ThzNtnPhy::SetWaveform(WaveformType wf)
{
    NS_LOG_FUNCTION(this << wf);
    m_waveform = wf;
}

WaveformType
ThzNtnPhy::GetWaveform() const
{
    return m_waveform;
}

void
ThzNtnPhy::SetCenterFrequency(double freqHz)
{
    NS_LOG_FUNCTION(this << freqHz);
    m_centerFreqHz = freqHz;
    m_spectrumModel = nullptr; // invalidate cached model
}

double
ThzNtnPhy::GetCenterFrequency() const
{
    return m_centerFreqHz;
}

void
ThzNtnPhy::SetBandwidth(double bwHz)
{
    NS_LOG_FUNCTION(this << bwHz);
    m_bandwidthHz = bwHz;
    m_spectrumModel = nullptr; // invalidate cached model
}

double
ThzNtnPhy::GetBandwidth() const
{
    return m_bandwidthHz;
}

void
ThzNtnPhy::SetTxPower(double txPowerDbm)
{
    NS_LOG_FUNCTION(this << txPowerDbm);
    m_txPowerDbm = txPowerDbm;
}

double
ThzNtnPhy::GetTxPower() const
{
    return m_txPowerDbm;
}

void
ThzNtnPhy::SetNoiseFigure(double nfDb)
{
    NS_LOG_FUNCTION(this << nfDb);
    m_noiseFigureDb = nfDb;
}

double
ThzNtnPhy::GetNoiseFigure() const
{
    return m_noiseFigureDb;
}

void
ThzNtnPhy::SetNoiseTemperature(double tempK)
{
    NS_LOG_FUNCTION(this << tempK);
    m_noiseTemperatureK = tempK;
}

double
ThzNtnPhy::GetNoiseTemperature() const
{
    return m_noiseTemperatureK;
}

void
ThzNtnPhy::SetNumSubBands(uint32_t numSubBands)
{
    NS_LOG_FUNCTION(this << numSubBands);
    m_numSubBands = numSubBands;
    m_spectrumModel = nullptr; // invalidate cached model
}

uint32_t
ThzNtnPhy::GetNumSubBands() const
{
    return m_numSubBands;
}

void
ThzNtnPhy::SetLinkState(ThzNtnLinkState state)
{
    NS_LOG_FUNCTION(this << state);
    m_linkState = state;
}

ThzNtnLinkState
ThzNtnPhy::GetLinkState() const
{
    return m_linkState;
}

double
ThzNtnPhy::GetLastSinr_dB() const
{
    return m_lastSinrDb;
}

void
ThzNtnPhy::SetHardwareImpairments(Ptr<ThzNtnHardwareImpairments> hwImpairments)
{
    NS_LOG_FUNCTION(this << hwImpairments);
    m_hwImpairments = hwImpairments;
}

Ptr<ThzNtnHardwareImpairments>
ThzNtnPhy::GetHardwareImpairments() const
{
    return m_hwImpairments;
}

void
ThzNtnPhy::SetPhyMacCommon(Ptr<mmwave::MmWavePhyMacCommon> phyMacCommon)
{
    NS_LOG_FUNCTION(this << phyMacCommon);
    m_phyMacCommon = phyMacCommon;
}

Ptr<mmwave::MmWavePhyMacCommon>
ThzNtnPhy::GetPhyMacCommon() const
{
    return m_phyMacCommon;
}

} // namespace ns3
