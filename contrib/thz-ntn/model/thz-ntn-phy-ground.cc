/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Ground Terminal THz-NTN PHY Layer Model -- Implementation
 *
 * Integrates with satellite module's SatUtPhy for reception, generates
 * mmWave-compatible CQI feedback from actual SINR, computes Doppler
 * from received SatSignalParameters, and applies AGC from actual
 * received power variations.
 */

#include "thz-ntn-phy-ground.h"

#include <ns3/double.h>
#include <ns3/enum.h>
#include <ns3/log.h>
#include <ns3/spectrum-value.h>

#include <ns3/mmwave-amc.h>
#include <ns3/mmwave-phy-mac-common.h>
#include <ns3/satellite-phy.h>
#include <ns3/satellite-signal-parameters.h>

#include <cmath>
#include <vector>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnPhyGround");

NS_OBJECT_ENSURE_REGISTERED(ThzNtnPhyGround);

TypeId
ThzNtnPhyGround::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnPhyGround")
            .SetParent<ThzNtnPhy>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnPhyGround>()
            .AddAttribute("ReceiverType",
                          "THz receiver architecture type.",
                          EnumValue(RX_HETERODYNE),
                          MakeEnumAccessor<ReceiverType>(&ThzNtnPhyGround::m_receiverType),
                          MakeEnumChecker(RX_HETERODYNE, "HETERODYNE",
                                          RX_DIRECT_DETECTION, "DIRECT_DETECTION",
                                          RX_HOMODYNE, "HOMODYNE"))
            .AddAttribute("Polarization",
                          "Antenna polarization configuration.",
                          EnumValue(POL_DUAL_CIRCULAR),
                          MakeEnumAccessor<Polarization>(&ThzNtnPhyGround::m_polarization),
                          MakeEnumChecker(POL_SINGLE_H, "SINGLE_H",
                                          POL_SINGLE_V, "SINGLE_V",
                                          POL_DUAL_LINEAR, "DUAL_LINEAR",
                                          POL_DUAL_CIRCULAR, "DUAL_CIRCULAR"))
            .AddAttribute("PllBandwidth",
                          "PLL loop bandwidth in Hz for Doppler tracking.",
                          DoubleValue(1000.0),
                          MakeDoubleAccessor(&ThzNtnPhyGround::m_pllBandwidthHz),
                          MakeDoubleChecker<double>(10.0, 100000.0))
            .AddAttribute("AgcTargetPower",
                          "AGC target received power in Watts.",
                          DoubleValue(1.0e-9), // -60 dBm
                          MakeDoubleAccessor(&ThzNtnPhyGround::m_agcTargetPower_W),
                          MakeDoubleChecker<double>(1.0e-15, 1.0))
            .AddTraceSource("CqiTrace",
                            "Reports wideband CQI and MCS",
                            MakeTraceSourceAccessor(&ThzNtnPhyGround::m_cqiTrace),
                            "ns3::ThzNtnPhyGround::CqiTracedCallback")
            .AddTraceSource("AgcTrace",
                            "Reports AGC gain (linear)",
                            MakeTraceSourceAccessor(&ThzNtnPhyGround::m_agcTrace),
                            "ns3::ThzNtnPhyGround::AgcTracedCallback")
            .AddTraceSource("DopplerEstTrace",
                            "Reports estimated Doppler shift in Hz",
                            MakeTraceSourceAccessor(&ThzNtnPhyGround::m_dopplerEstTrace),
                            "ns3::ThzNtnPhyGround::DopplerEstTracedCallback");
    return tid;
}

ThzNtnPhyGround::ThzNtnPhyGround()
    : m_utPhy(nullptr),
      m_receiverType(RX_HETERODYNE),
      m_polarization(POL_DUAL_CIRCULAR),
      m_pllBandwidthHz(1000.0),
      m_agcTargetPower_W(1.0e-9),
      m_currentMcs(0)
{
    NS_LOG_FUNCTION(this);
}

ThzNtnPhyGround::~ThzNtnPhyGround()
{
    NS_LOG_FUNCTION(this);
}

void
ThzNtnPhyGround::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_utPhy = nullptr;
    ThzNtnPhy::DoDispose();
}

// ---------------------------------------------------------------------------
// SatUtPhy integration
// ---------------------------------------------------------------------------

void
ThzNtnPhyGround::SetUtPhy(Ptr<SatPhy> utPhy)
{
    NS_LOG_FUNCTION(this << utPhy);
    m_utPhy = utPhy;

    // Synchronize RF parameters from the UT PHY if available
    if (m_utPhy != nullptr)
    {
        double rxNoiseTemp = m_utPhy->GetRxNoiseTemperatureDbk();
        double noiseTempK = std::pow(10.0, rxNoiseTemp / 10.0);
        SetNoiseTemperature(noiseTempK);

        NS_LOG_INFO("Synchronized noise temperature from SatUtPhy: "
                     << rxNoiseTemp << " dBK (" << noiseTempK << " K)");
    }
}

Ptr<SatPhy>
ThzNtnPhyGround::GetUtPhy() const
{
    return m_utPhy;
}

// ---------------------------------------------------------------------------
// Doppler tracking from received signal
// ---------------------------------------------------------------------------

double
ThzNtnPhyGround::EstimateDopplerFromSignal(Ptr<SatSignalParameters> rxParams) const
{
    NS_LOG_FUNCTION(this);

    if (rxParams == nullptr)
    {
        NS_LOG_WARN("Null rxParams, cannot estimate Doppler");
        return 0.0;
    }

    // The carrier frequency in the received signal is the actual transmitted
    // frequency (possibly pre-compensated by the satellite).  The Doppler
    // estimate is the difference from our nominal center frequency.
    double rxCarrierFreq = rxParams->m_carrierFreq_hz;
    double dopplerEstimate = rxCarrierFreq - m_centerFreqHz;

    // Fire trace
    m_dopplerEstTrace(dopplerEstimate);

    NS_LOG_DEBUG("DopplerEstimate from signal: rxCarrier=" << rxCarrierFreq / 1e9
                 << " GHz, nominal=" << m_centerFreqHz / 1e9
                 << " GHz, Doppler=" << dopplerEstimate / 1e3 << " kHz");

    return dopplerEstimate;
}

double
ThzNtnPhyGround::ComputeDopplerTrackingError_Hz() const
{
    NS_LOG_FUNCTION(this);

    // Loop SNR estimated from last measured SINR, scaled by bandwidth ratio
    double sinrLinear = std::pow(10.0, m_lastSinrDb / 10.0);
    double bwRatio = m_bandwidthHz / m_pllBandwidthHz;
    double snrLoop = sinrLinear * bwRatio;

    // Clamp loop SNR to avoid numerical issues
    if (snrLoop < 1.0)
    {
        snrLoop = 1.0;
    }

    // RMS frequency tracking error: sigma_f = sqrt(B_L / SNR_loop)
    double sigmaF = std::sqrt(m_pllBandwidthHz / snrLoop);

    NS_LOG_DEBUG("DopplerTrackingError: PLL_BW=" << m_pllBandwidthHz
                 << " Hz, SNR_loop=" << 10.0 * std::log10(snrLoop)
                 << " dB, sigma_f=" << sigmaF << " Hz");

    return sigmaF;
}

// ---------------------------------------------------------------------------
// CQI generation compatible with mmWave pipeline
// ---------------------------------------------------------------------------

std::vector<uint8_t>
ThzNtnPhyGround::GenerateCqiFeedback(Ptr<const SpectrumValue> sinr,
                                       uint8_t& wbCqi,
                                       uint8_t& wbMcs) const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(sinr != nullptr, "SINR SpectrumValue must not be null");

    std::vector<uint8_t> rbCqi;

    if (m_amcModel != nullptr)
    {
        // Use the actual MmWaveAmc to create CQI feedback
        wbCqi = m_amcModel->CreateCqiFeedbackWbTdma(*sinr, wbMcs);

        // For per-RB CQI, we compute CQI for each sub-band
        Values::const_iterator vit = sinr->ConstValuesBegin();
        for (; vit != sinr->ConstValuesEnd(); ++vit)
        {
            double sinrLinear = *vit;
            if (sinrLinear <= 0.0)
            {
                rbCqi.push_back(0);
                continue;
            }

            // Convert to spectral efficiency and then to CQI
            double se = std::log2(1.0 + sinrLinear);
            uint8_t cqi = m_amcModel->GetCqiFromSpectralEfficiency(se);
            rbCqi.push_back(cqi);
        }

        m_currentMcs = wbMcs;
    }
    else
    {
        // Fallback: Shannon-based CQI without AMC model
        NS_LOG_WARN("No AMC model set, using Shannon-based CQI fallback");

        double sumSinr = 0.0;
        uint32_t count = 0;

        Values::const_iterator vit = sinr->ConstValuesBegin();
        for (; vit != sinr->ConstValuesEnd(); ++vit)
        {
            double sinrLinear = *vit;
            sumSinr += sinrLinear;
            count++;

            // Approximate CQI: map SINR to 0-15 range
            double sinrDb = (sinrLinear > 0)
                                ? 10.0 * std::log10(sinrLinear)
                                : -20.0;
            // CQI roughly maps to 1.89 dB per CQI index, starting at -6.7 dB
            int cqi = static_cast<int>(std::round((sinrDb + 6.7) / 1.89));
            cqi = std::max(0, std::min(15, cqi));
            rbCqi.push_back(static_cast<uint8_t>(cqi));
        }

        double avgSinr = (count > 0) ? sumSinr / count : 0.0;
        double avgSinrDb = (avgSinr > 0)
                               ? 10.0 * std::log10(avgSinr)
                               : -20.0;

        int wbCqiInt = static_cast<int>(std::round((avgSinrDb + 6.7) / 1.89));
        wbCqi = static_cast<uint8_t>(std::max(0, std::min(15, wbCqiInt)));

        // Approximate MCS from CQI
        wbMcs = static_cast<uint8_t>(std::min(28, static_cast<int>(wbCqi) * 2));
        m_currentMcs = wbMcs;
    }

    // Fire trace
    m_cqiTrace(wbCqi, wbMcs);

    NS_LOG_INFO("CQI feedback: wbCqi=" << static_cast<uint32_t>(wbCqi)
                << ", wbMcs=" << static_cast<uint32_t>(wbMcs)
                << ", numRbCqi=" << rbCqi.size());

    return rbCqi;
}

void
ThzNtnPhyGround::GenerateCqiFromSinr(double sinrDb, uint8_t& wbCqi, uint8_t& wbMcs) const
{
    NS_LOG_FUNCTION(this << sinrDb);

    // Create a uniform SpectrumValue from the scalar SINR
    EnsureSpectrumModel();
    Ptr<SpectrumValue> sinrSv = Create<SpectrumValue>(m_spectrumModel);
    double sinrLinear = std::pow(10.0, sinrDb / 10.0);

    Values::iterator vit = sinrSv->ValuesBegin();
    for (; vit != sinrSv->ValuesEnd(); ++vit)
    {
        *vit = sinrLinear;
    }

    GenerateCqiFeedback(sinrSv, wbCqi, wbMcs);
}

// ---------------------------------------------------------------------------
// AGC from actual received power variations
// ---------------------------------------------------------------------------

double
ThzNtnPhyGround::ComputeAgcGain(double instantRxPower_W, double targetRxPower_W) const
{
    NS_LOG_FUNCTION(this << instantRxPower_W << targetRxPower_W);

    if (instantRxPower_W <= 0.0)
    {
        NS_LOG_WARN("Zero or negative instantaneous power, returning max AGC gain");
        m_agcTrace(AGC_MAX_GAIN_LINEAR);
        return AGC_MAX_GAIN_LINEAR;
    }

    // AGC gain: ratio of target to instantaneous power
    double gain = targetRxPower_W / instantRxPower_W;

    // Clamp to AGC dynamic range
    if (gain > AGC_MAX_GAIN_LINEAR)
    {
        gain = AGC_MAX_GAIN_LINEAR;
        NS_LOG_DEBUG("AGC gain clamped to maximum: " << 10.0 * std::log10(gain) << " dB");
    }
    else if (gain < AGC_MIN_GAIN_LINEAR)
    {
        gain = AGC_MIN_GAIN_LINEAR;
        NS_LOG_DEBUG("AGC gain clamped to minimum: " << 10.0 * std::log10(gain) << " dB");
    }

    // Fire trace
    m_agcTrace(gain);

    NS_LOG_DEBUG("AGC: instantPower=" << 10.0 * std::log10(instantRxPower_W * 1000.0)
                 << " dBm, targetPower=" << 10.0 * std::log10(targetRxPower_W * 1000.0)
                 << " dBm, gain=" << 10.0 * std::log10(gain) << " dB");

    return gain;
}

double
ThzNtnPhyGround::ComputeAgcGainFromSignal(Ptr<SatSignalParameters> rxParams) const
{
    NS_LOG_FUNCTION(this);

    if (rxParams == nullptr)
    {
        NS_LOG_WARN("Null rxParams, returning unity AGC gain");
        return 1.0;
    }

    double rxPower_W = rxParams->m_rxPower_W;
    return ComputeAgcGain(rxPower_W, m_agcTargetPower_W);
}

// ---------------------------------------------------------------------------
// Receiver configuration
// ---------------------------------------------------------------------------

void
ThzNtnPhyGround::SetReceiverType(ReceiverType type)
{
    NS_LOG_FUNCTION(this << type);
    m_receiverType = type;
}

ReceiverType
ThzNtnPhyGround::GetReceiverType() const
{
    return m_receiverType;
}

void
ThzNtnPhyGround::SetPolarization(Polarization pol)
{
    NS_LOG_FUNCTION(this << pol);
    m_polarization = pol;
}

Polarization
ThzNtnPhyGround::GetPolarization() const
{
    return m_polarization;
}

double
ThzNtnPhyGround::ComputePolarizationLoss_dB() const
{
    NS_LOG_FUNCTION(this);

    double xpdLoss_dB = 0.0;

    switch (m_polarization)
    {
    case POL_SINGLE_H:
    case POL_SINGLE_V:
        xpdLoss_dB = 0.0;
        break;
    case POL_DUAL_LINEAR:
        xpdLoss_dB = 0.3;
        break;
    case POL_DUAL_CIRCULAR:
        xpdLoss_dB = 0.4;
        break;
    }

    NS_LOG_DEBUG("PolarizationLoss: " << xpdLoss_dB << " dB");
    return xpdLoss_dB;
}

double
ThzNtnPhyGround::ComputeReceiverSensitivity_dBm() const
{
    NS_LOG_FUNCTION(this);

    // Use actual noise PSD to compute noise floor
    double noiseFloor_dBm = ComputeNoiseFloor_dBm();

    // Required SNR margin depends on receiver architecture
    double snrMargin_dB = 0.0;
    switch (m_receiverType)
    {
    case RX_HETERODYNE:
        snrMargin_dB = 3.0;
        break;
    case RX_HOMODYNE:
        snrMargin_dB = 5.0;
        break;
    case RX_DIRECT_DETECTION:
        snrMargin_dB = 10.0;
        break;
    }

    double sensitivity_dBm = noiseFloor_dBm + snrMargin_dB;

    NS_LOG_DEBUG("ReceiverSensitivity: noiseFloor=" << noiseFloor_dBm
                 << " dBm, margin=" << snrMargin_dB
                 << " dB, sensitivity=" << sensitivity_dBm << " dBm"
                 << " (T=" << m_noiseTemperatureK << " K, NF="
                 << m_noiseFigureDb << " dB)");

    return sensitivity_dBm;
}

uint8_t
ThzNtnPhyGround::GetCurrentMcs() const
{
    return m_currentMcs;
}

} // namespace ns3
