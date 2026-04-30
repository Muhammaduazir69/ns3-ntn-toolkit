/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Satellite THz-NTN PHY Layer Model -- Implementation
 *
 * Integrates with satellite module's SatPhy, SatWaveformConf, and
 * SatSignalParameters for actual ModCod selection, Doppler pre-compensation
 * from orbital velocity vectors, and regenerative payload processing.
 */

#include "thz-ntn-phy-sat.h"

#include <ns3/boolean.h>
#include <ns3/double.h>
#include <ns3/enum.h>
#include <ns3/log.h>
#include <ns3/mobility-model.h>
#include <ns3/simulator.h>

#include <ns3/satellite-enums.h>
#include <ns3/satellite-phy.h>
#include <ns3/satellite-signal-parameters.h>
#include <ns3/satellite-wave-form-conf.h>

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnPhySat");

NS_OBJECT_ENSURE_REGISTERED(ThzNtnPhySat);

TypeId
ThzNtnPhySat::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnPhySat")
            .SetParent<ThzNtnPhy>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnPhySat>()
            .AddAttribute("PayloadMode",
                          "Satellite payload processing mode.",
                          EnumValue(PAYLOAD_REGENERATIVE),
                          MakeEnumAccessor<PayloadMode>(&ThzNtnPhySat::m_payloadMode),
                          MakeEnumChecker(PAYLOAD_TRANSPARENT, "TRANSPARENT",
                                          PAYLOAD_REGENERATIVE, "REGENERATIVE"))
            .AddAttribute("SatelliteClass",
                          "Satellite platform class determining power/antenna defaults.",
                          EnumValue(SAT_CUBESAT),
                          MakeEnumAccessor<SatelliteClass>(&ThzNtnPhySat::m_satelliteClass),
                          MakeEnumChecker(SAT_CUBESAT, "CUBESAT",
                                          SAT_SMALLSAT, "SMALLSAT",
                                          SAT_FULLSAT, "FULLSAT"))
            .AddAttribute("EnableDopplerPreComp",
                          "Enable Doppler pre-compensation at satellite transmitter.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&ThzNtnPhySat::m_enableDopplerPreComp),
                          MakeBooleanChecker())
            .AddTraceSource("ModcodTrace",
                            "Reports selected waveform ID and spectral efficiency",
                            MakeTraceSourceAccessor(&ThzNtnPhySat::m_modcodTrace),
                            "ns3::ThzNtnPhySat::ModcodTracedCallback")
            .AddTraceSource("PreCompTrace",
                            "Reports Doppler pre-compensation in Hz",
                            MakeTraceSourceAccessor(&ThzNtnPhySat::m_preCompTrace),
                            "ns3::ThzNtnPhySat::PreCompTracedCallback");
    return tid;
}

ThzNtnPhySat::ThzNtnPhySat()
    : m_satPhy(nullptr),
      m_waveformConf(nullptr),
      m_satMobility(nullptr),
      m_payloadMode(PAYLOAD_REGENERATIVE),
      m_satelliteClass(SAT_CUBESAT),
      m_enableDopplerPreComp(true)
{
    NS_LOG_FUNCTION(this);
}

ThzNtnPhySat::~ThzNtnPhySat()
{
    NS_LOG_FUNCTION(this);
}

void
ThzNtnPhySat::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_satPhy = nullptr;
    m_waveformConf = nullptr;
    m_satMobility = nullptr;
    ThzNtnPhy::DoDispose();
}

// ---------------------------------------------------------------------------
// SatPhy integration
// ---------------------------------------------------------------------------

void
ThzNtnPhySat::SetSatellitePhy(Ptr<SatPhy> satPhy)
{
    NS_LOG_FUNCTION(this << satPhy);
    m_satPhy = satPhy;

    // Synchronize RF parameters from the satellite PHY if available
    if (m_satPhy != nullptr)
    {
        // Read noise temperature and antenna gains from SatPhy
        double rxNoiseTemp = m_satPhy->GetRxNoiseTemperatureDbk();
        // Convert from dBK to linear K
        double noiseTempK = std::pow(10.0, rxNoiseTemp / 10.0);
        SetNoiseTemperature(noiseTempK);

        NS_LOG_INFO("Synchronized noise temperature from SatPhy: "
                     << rxNoiseTemp << " dBK (" << noiseTempK << " K)");
    }
}

Ptr<SatPhy>
ThzNtnPhySat::GetSatellitePhy() const
{
    return m_satPhy;
}

// ---------------------------------------------------------------------------
// SatWaveformConf-based ModCod selection
// ---------------------------------------------------------------------------

void
ThzNtnPhySat::SetWaveformConf(Ptr<SatWaveformConf> wfConf)
{
    NS_LOG_FUNCTION(this << wfConf);
    m_waveformConf = wfConf;
}

Ptr<SatWaveformConf>
ThzNtnPhySat::GetWaveformConf() const
{
    return m_waveformConf;
}

uint32_t
ThzNtnPhySat::SelectWaveformId(double cnoDb, double symbolRateInBaud) const
{
    NS_LOG_FUNCTION(this << cnoDb << symbolRateInBaud);

    if (m_waveformConf == nullptr)
    {
        NS_LOG_WARN("No SatWaveformConf set, returning default waveform ID 3");
        return 3; // Fallback to a basic waveform
    }

    // Convert C/N0 from dB to linear for SatWaveformConf
    double cnoLinear = std::pow(10.0, cnoDb / 10.0);

    uint32_t wfId = 0;
    double cnoThreshold = 0.0;

    bool found = m_waveformConf->GetBestWaveformId(cnoLinear,
                                                     symbolRateInBaud,
                                                     wfId,
                                                     cnoThreshold);

    if (!found)
    {
        // Fall back to most robust waveform
        m_waveformConf->GetMostRobustWaveformId(wfId);
        NS_LOG_WARN("C/N0=" << cnoDb << " dB below all thresholds, using most robust wfId="
                    << wfId);
    }
    else
    {
        NS_LOG_INFO("Selected wfId=" << wfId << " for C/N0=" << cnoDb
                    << " dB (threshold=" << 10.0 * std::log10(cnoThreshold) << " dB)");
    }

    // Get spectral efficiency for tracing
    double se = GetWaveformSpectralEfficiency(wfId, m_bandwidthHz, symbolRateInBaud);
    m_modcodTrace(wfId, se);

    return wfId;
}

double
ThzNtnPhySat::GetWaveformSpectralEfficiency(uint32_t wfId,
                                              double carrierBandwidthHz,
                                              double symbolRateInBaud) const
{
    NS_LOG_FUNCTION(this << wfId << carrierBandwidthHz << symbolRateInBaud);

    if (m_waveformConf == nullptr)
    {
        NS_LOG_WARN("No SatWaveformConf set, returning 0 spectral efficiency");
        return 0.0;
    }

    Ptr<SatWaveform> wf = m_waveformConf->GetWaveform(wfId);
    if (wf == nullptr)
    {
        NS_LOG_WARN("Waveform ID " << wfId << " not found in SatWaveformConf");
        return 0.0;
    }

    double se = wf->GetSpectralEfficiency(carrierBandwidthHz, symbolRateInBaud);

    NS_LOG_DEBUG("Waveform " << wfId << " SE=" << se << " bps/Hz");
    return se;
}

uint32_t
ThzNtnPhySat::ComputePayloadBits(uint32_t wfId) const
{
    NS_LOG_FUNCTION(this << wfId);

    if (m_waveformConf == nullptr)
    {
        NS_LOG_WARN("No SatWaveformConf set, returning 0 payload bits");
        return 0;
    }

    Ptr<SatWaveform> wf = m_waveformConf->GetWaveform(wfId);
    if (wf == nullptr)
    {
        NS_LOG_WARN("Waveform ID " << wfId << " not found");
        return 0;
    }

    uint32_t payloadBits = wf->GetPayloadInBytes() * 8;
    NS_LOG_DEBUG("Waveform " << wfId << " payload=" << payloadBits << " bits ("
                 << wf->GetPayloadInBytes() << " bytes)");
    return payloadBits;
}

// ---------------------------------------------------------------------------
// Doppler pre-compensation from actual MobilityModel
// ---------------------------------------------------------------------------

void
ThzNtnPhySat::SetSatelliteMobility(Ptr<MobilityModel> mobility)
{
    NS_LOG_FUNCTION(this << mobility);
    m_satMobility = mobility;
}

Ptr<MobilityModel>
ThzNtnPhySat::GetSatelliteMobility() const
{
    return m_satMobility;
}

double
ThzNtnPhySat::ComputeDopplerPreCompensation(Ptr<MobilityModel> groundMobility) const
{
    NS_LOG_FUNCTION(this);

    if (!m_enableDopplerPreComp)
    {
        NS_LOG_DEBUG("Doppler pre-compensation disabled");
        return 0.0;
    }

    if (m_satMobility == nullptr)
    {
        NS_LOG_WARN("No satellite mobility model set, cannot compute Doppler pre-compensation");
        return 0.0;
    }

    if (groundMobility == nullptr)
    {
        NS_LOG_WARN("No ground mobility model provided, cannot compute Doppler pre-compensation");
        return 0.0;
    }

    // Compute Doppler shift using base class method with actual mobility models
    double dopplerHz = ComputeDopplerFromMobility(m_satMobility, groundMobility);

    // Pre-compensation: negate the Doppler so receiver sees nominal frequency
    double preComp = -dopplerHz;

    // Fire trace
    m_preCompTrace(preComp);

    NS_LOG_INFO("DopplerPreComp: Doppler=" << dopplerHz / 1e3
                << " kHz, preComp=" << preComp / 1e3 << " kHz");

    return preComp;
}

// ---------------------------------------------------------------------------
// Regenerative payload processing
// ---------------------------------------------------------------------------

Ptr<SatSignalParameters>
ThzNtnPhySat::ProcessRegenerativePayload(Ptr<SatSignalParameters> rxParams) const
{
    NS_LOG_FUNCTION(this);

    if (m_payloadMode == PAYLOAD_TRANSPARENT)
    {
        NS_LOG_DEBUG("Transparent payload mode: pass-through, no on-board processing");
        return nullptr;
    }

    if (rxParams == nullptr)
    {
        NS_LOG_WARN("Null rxParams received for regenerative processing");
        return nullptr;
    }

    // Step 1: Extract uplink SINR from the received signal parameters
    double uplinkSinr = 0.0;
    if (rxParams->HasSinrComputed())
    {
        uplinkSinr = rxParams->GetSinr();
        NS_LOG_DEBUG("Uplink SINR from SatSignalParameters: " << 10.0 * std::log10(uplinkSinr)
                     << " dB");
    }
    else
    {
        // Compute SINR from rx power and interference
        double rxPower_W = rxParams->m_rxPower_W;
        double ifPower_W = rxParams->GetInterferencePower();
        double noiseFloor_dBm = ComputeNoiseFloor_dBm();
        double noise_W = std::pow(10.0, (noiseFloor_dBm - 30.0) / 10.0);

        if (noise_W + ifPower_W > 0)
        {
            uplinkSinr = rxPower_W / (noise_W + ifPower_W);
        }
        NS_LOG_DEBUG("Computed uplink SINR: " << 10.0 * std::log10(std::max(uplinkSinr, 1e-20))
                     << " dB");
    }

    // Step 2: Select THz-optimized waveform for downlink based on SINR
    // Convert SINR to C/N0: C/N0 = SINR * bandwidth
    double cnoLinear = uplinkSinr * m_bandwidthHz;
    double cnoDb = 10.0 * std::log10(std::max(cnoLinear, 1e-20));

    // Use AMC model (from mmwave) for MCS selection if available
    uint8_t mcs = 0;
    double sinrDb = 10.0 * std::log10(std::max(uplinkSinr, 1e-20));
    if (m_amcModel != nullptr)
    {
        mcs = GetMcsFromSinr(sinrDb);
    }

    // Step 3: Create new SatSignalParameters for downlink
    Ptr<SatSignalParameters> txParams = Create<SatSignalParameters>();

    // Copy packets from uplink
    txParams->m_packetsInBurst = rxParams->m_packetsInBurst;

    // Set THz downlink parameters
    txParams->m_carrierFreq_hz = m_centerFreqHz;
    txParams->m_duration = rxParams->m_duration; // preserve duration
    txParams->m_satId = rxParams->m_satId;
    txParams->m_beamId = rxParams->m_beamId;
    txParams->m_carrierId = rxParams->m_carrierId;

    // Set transmit power based on satellite class
    double txPower_W = std::pow(10.0, (m_txPowerDbm - 30.0) / 10.0);
    txParams->m_txPower_W = txPower_W;

    // Set downlink ModCod via SatWaveformConf if available
    if (m_waveformConf != nullptr)
    {
        double symbolRate = m_bandwidthHz * 0.8; // approximate symbol rate
        uint32_t wfId = SelectWaveformId(cnoDb, symbolRate);
        txParams->m_txInfo.waveformId = wfId;
        txParams->m_txInfo.modCod = m_waveformConf->GetModCod(wfId);
    }

    txParams->m_txInfo.packetType = rxParams->m_txInfo.packetType;
    txParams->m_txInfo.sliceId = rxParams->m_txInfo.sliceId;

    NS_LOG_INFO("Regenerative payload: uplink SINR=" << sinrDb
                << " dB, downlink MCS=" << static_cast<uint32_t>(mcs)
                << ", txPower=" << m_txPowerDbm << " dBm");

    return txParams;
}

Time
ThzNtnPhySat::ComputeOnBoardProcessingDelay() const
{
    NS_LOG_FUNCTION(this);

    if (m_payloadMode == PAYLOAD_TRANSPARENT)
    {
        NS_LOG_DEBUG("Transparent payload: zero processing delay");
        return Time(0);
    }

    // Processing delay depends on satellite class
    double delay_us = 0.0;
    switch (m_satelliteClass)
    {
    case SAT_CUBESAT:
        delay_us = 500.0;
        break;
    case SAT_SMALLSAT:
        delay_us = 200.0;
        break;
    case SAT_FULLSAT:
        delay_us = 50.0;
        break;
    }

    NS_LOG_DEBUG("OnBoardProcessingDelay: " << delay_us << " us ("
                 << (m_satelliteClass == SAT_CUBESAT ? "CubeSat" :
                     m_satelliteClass == SAT_SMALLSAT ? "SmallSat" : "FullSat")
                 << ", regenerative)");

    return MicroSeconds(static_cast<uint64_t>(delay_us));
}

// ---------------------------------------------------------------------------
// Payload and platform configuration
// ---------------------------------------------------------------------------

void
ThzNtnPhySat::SetPayloadMode(PayloadMode mode)
{
    NS_LOG_FUNCTION(this << mode);
    m_payloadMode = mode;
}

PayloadMode
ThzNtnPhySat::GetPayloadMode() const
{
    return m_payloadMode;
}

void
ThzNtnPhySat::SetSatelliteClass(SatelliteClass cls)
{
    NS_LOG_FUNCTION(this << cls);
    m_satelliteClass = cls;

    // Set default power and noise figure based on platform class
    switch (cls)
    {
    case SAT_CUBESAT:
        m_txPowerDbm = 30.0;     // 1 W
        m_noiseFigureDb = 12.0;
        NS_LOG_INFO("CubeSat class: TxPower=30 dBm, NF=12 dB");
        break;
    case SAT_SMALLSAT:
        m_txPowerDbm = 37.0;     // 5 W
        m_noiseFigureDb = 10.0;
        NS_LOG_INFO("SmallSat class: TxPower=37 dBm, NF=10 dB");
        break;
    case SAT_FULLSAT:
        m_txPowerDbm = 43.0;     // 20 W
        m_noiseFigureDb = 7.0;
        NS_LOG_INFO("FullSat class: TxPower=43 dBm, NF=7 dB");
        break;
    }
}

SatelliteClass
ThzNtnPhySat::GetSatelliteClass() const
{
    return m_satelliteClass;
}

void
ThzNtnPhySat::SetEnableDopplerPreComp(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_enableDopplerPreComp = enable;
}

bool
ThzNtnPhySat::GetEnableDopplerPreComp() const
{
    return m_enableDopplerPreComp;
}

} // namespace ns3
