/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN Inter-Satellite Link (ISL) Channel Model
 */

#include "thz-ntn-isl-channel.h"

#include "thz-ntn-hardware-impairments.h"

#include <ns3/satellite-free-space-loss.h>

#include <ns3/double.h>
#include <ns3/log.h>

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnIslChannel");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnIslChannel);

TypeId
ThzNtnIslChannel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnIslChannel")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnIslChannel>()
            .AddAttribute("Frequency",
                          "ISL operating frequency in Hz",
                          DoubleValue(300.0e9),
                          MakeDoubleAccessor(&ThzNtnIslChannel::m_frequency),
                          MakeDoubleChecker<double>(1.0e9, 10.0e12))
            .AddAttribute("TxPower",
                          "Transmit power in dBm",
                          DoubleValue(30.0),
                          MakeDoubleAccessor(&ThzNtnIslChannel::m_txPower),
                          MakeDoubleChecker<double>(-10.0, 60.0))
            .AddAttribute("TxGain",
                          "Transmit antenna gain in dBi",
                          DoubleValue(40.0),
                          MakeDoubleAccessor(&ThzNtnIslChannel::m_txGain),
                          MakeDoubleChecker<double>(0.0, 80.0))
            .AddAttribute("RxGain",
                          "Receive antenna gain in dBi",
                          DoubleValue(40.0),
                          MakeDoubleAccessor(&ThzNtnIslChannel::m_rxGain),
                          MakeDoubleChecker<double>(0.0, 80.0))
            .AddAttribute("Bandwidth",
                          "Channel bandwidth in Hz",
                          DoubleValue(20.0e9),
                          MakeDoubleAccessor(&ThzNtnIslChannel::m_bandwidth),
                          MakeDoubleChecker<double>(1.0e6, 100.0e9))
            .AddAttribute("ReceiverNoiseTemp_K",
                          "Receiver noise temperature in Kelvin",
                          DoubleValue(500.0),
                          MakeDoubleAccessor(&ThzNtnIslChannel::m_receiverNoiseTemp_K),
                          MakeDoubleChecker<double>(1.0, 10000.0));
    return tid;
}

ThzNtnIslChannel::ThzNtnIslChannel()
    : m_frequency(300.0e9),
      m_txPower(30.0),
      m_txGain(40.0),
      m_rxGain(40.0),
      m_bandwidth(20.0e9),
      m_receiverNoiseTemp_K(500.0),
      m_fsl(nullptr),
      m_hwModel(nullptr)
{
    NS_LOG_FUNCTION(this);
}

ThzNtnIslChannel::~ThzNtnIslChannel()
{
    NS_LOG_FUNCTION(this);
}

void
ThzNtnIslChannel::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_fsl = nullptr;
    m_hwModel = nullptr;
    Object::DoDispose();
}

// --------------------------------------------------------------------------
// Sub-model setters
// --------------------------------------------------------------------------

void
ThzNtnIslChannel::SetFreeSpaceLossModel(Ptr<SatFreeSpaceLoss> fsl)
{
    NS_LOG_FUNCTION(this << fsl);
    m_fsl = fsl;
}

void
ThzNtnIslChannel::SetHardwareModel(Ptr<ThzNtnHardwareImpairments> hw)
{
    NS_LOG_FUNCTION(this << hw);
    m_hwModel = hw;
}

// --------------------------------------------------------------------------
// Doppler from actual velocity vectors
// --------------------------------------------------------------------------

double
ThzNtnIslChannel::ComputeRelativeDoppler_Hz(Ptr<MobilityModel> satA,
                                             Ptr<MobilityModel> satB) const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(satA && satB, "MobilityModel pointers must not be null");

    Vector posA = satA->GetPosition();
    Vector posB = satB->GetPosition();
    Vector velA = satA->GetVelocity();
    Vector velB = satB->GetVelocity();

    // Line-of-sight unit vector from A to B
    Vector los(posB.x - posA.x, posB.y - posA.y, posB.z - posA.z);
    double distance = std::sqrt(los.x * los.x + los.y * los.y + los.z * los.z);

    if (distance < 1.0)
    {
        return 0.0;
    }

    los.x /= distance;
    los.y /= distance;
    los.z /= distance;

    // Relative velocity vector
    Vector relVel(velB.x - velA.x, velB.y - velA.y, velB.z - velA.z);

    // Project relative velocity onto LOS direction (radial component)
    double radialVelocity = relVel.x * los.x + relVel.y * los.y + relVel.z * los.z;

    // Doppler shift: f_d = f * v_radial / c
    double doppler = m_frequency * radialVelocity / SPEED_OF_LIGHT;

    NS_LOG_DEBUG("Doppler: " << doppler << " Hz (v_radial=" << radialVelocity
                 << " m/s, dist=" << distance / 1.0e3 << " km)");
    return doppler;
}

// --------------------------------------------------------------------------
// Noise temperature
// --------------------------------------------------------------------------

double
ThzNtnIslChannel::ComputeSpaceNoiseTemperature_K() const
{
    NS_LOG_FUNCTION(this);
    double totalTemp = COSMIC_BACKGROUND_K + m_receiverNoiseTemp_K;
    NS_LOG_DEBUG("Space noise temperature: " << totalTemp << " K (CMB="
                 << COSMIC_BACKGROUND_K << " K + Rx=" << m_receiverNoiseTemp_K << " K)");
    return totalTemp;
}

// --------------------------------------------------------------------------
// SNR from actual MobilityModel positions
// --------------------------------------------------------------------------

double
ThzNtnIslChannel::ComputeIslSnr_dB(Ptr<MobilityModel> satA,
                                    Ptr<MobilityModel> satB) const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(satA && satB, "MobilityModel pointers must not be null");

    // Compute FSPL using SatFreeSpaceLoss if available, otherwise fallback
    double fspl_dB;
    if (m_fsl)
    {
        fspl_dB = m_fsl->GetFsldB(satA, satB, m_frequency);
    }
    else
    {
        // Fallback: compute FSPL from positions directly
        double distance = satA->GetDistanceFrom(satB);
        if (distance <= 0.0)
        {
            return 100.0; // Effectively infinite SNR for co-located nodes
        }
        fspl_dB = 20.0 * std::log10(4.0 * PI_VAL * distance * m_frequency / SPEED_OF_LIGHT);
    }

    // Received power: Prx = Ptx + Gtx + Grx - FSPL  (all in dB/dBm/dBi)
    double rxPower_dBm = m_txPower + m_txGain + m_rxGain - fspl_dB;

    // Noise power: N = k * (T_cosmic + T_rx) * B  (watts, then convert to dBm)
    double noiseTemp_K = ComputeSpaceNoiseTemperature_K();
    double noisePower_W = BOLTZMANN_K * noiseTemp_K * m_bandwidth;
    double noisePower_dBm = 10.0 * std::log10(noisePower_W) + 30.0;

    // Raw SNR
    double snr_dB = rxPower_dBm - noisePower_dBm;

    // Apply hardware impairment ceiling if model is set
    if (m_hwModel)
    {
        double evmFactor = m_hwModel->ComputeEvmTotal();
        if (evmFactor > 0.0)
        {
            double snrRawLinear = std::pow(10.0, snr_dB / 10.0);
            double evmSqr = evmFactor * evmFactor;
            double snrEffLinear = 1.0 / (1.0 / snrRawLinear + evmSqr);
            snr_dB = 10.0 * std::log10(snrEffLinear);
        }
    }

    NS_LOG_INFO("ISL SNR: Prx=" << rxPower_dBm << " dBm, N=" << noisePower_dBm
                << " dBm, FSPL=" << fspl_dB << " dB, SNR=" << snr_dB << " dB");
    return snr_dB;
}

// --------------------------------------------------------------------------
// Capacity estimation
// --------------------------------------------------------------------------

double
ThzNtnIslChannel::ComputeIslCapacity_Gbps(double snrDb) const
{
    NS_LOG_FUNCTION(this << snrDb);

    double snrLinear = std::pow(10.0, snrDb / 10.0);

    // Shannon capacity with practical implementation loss factor of 0.8
    static constexpr double IMPLEMENTATION_LOSS = 0.8;
    double capacity_bps = IMPLEMENTATION_LOSS * m_bandwidth * std::log2(1.0 + snrLinear);
    double capacity_Gbps = capacity_bps / 1.0e9;

    NS_LOG_INFO("ISL capacity: " << capacity_Gbps << " Gbps (SNR=" << snrDb
                << " dB, BW=" << m_bandwidth / 1.0e9 << " GHz)");
    return capacity_Gbps;
}

// --------------------------------------------------------------------------
// Complete link state from MobilityModels
// --------------------------------------------------------------------------

ThzNtnIslLinkState
ThzNtnIslChannel::ComputeLinkState(Ptr<MobilityModel> satA,
                                   Ptr<MobilityModel> satB) const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(satA && satB, "MobilityModel pointers must not be null");

    ThzNtnIslLinkState state;
    state.linkId = 0;
    state.srcSatId = 0;
    state.dstSatId = 0;
    state.islType = ThzNtnIslType::INTRA_PLANE;

    // Distance from actual MobilityModel positions
    double distance_m = satA->GetDistanceFrom(satB);
    state.distance_km = distance_m / 1.0e3;

    // Relative radial velocity from velocity vectors
    Vector posA = satA->GetPosition();
    Vector posB = satB->GetPosition();
    Vector velA = satA->GetVelocity();
    Vector velB = satB->GetVelocity();

    Vector los(posB.x - posA.x, posB.y - posA.y, posB.z - posA.z);
    double dist = std::sqrt(los.x * los.x + los.y * los.y + los.z * los.z);
    if (dist > 1.0)
    {
        los.x /= dist;
        los.y /= dist;
        los.z /= dist;
    }
    Vector relVel(velB.x - velA.x, velB.y - velA.y, velB.z - velA.z);
    state.relativeVelocity_m_s = relVel.x * los.x + relVel.y * los.y + relVel.z * los.z;

    // Doppler from velocity vectors
    state.doppler_Hz = ComputeRelativeDoppler_Hz(satA, satB);

    // FSPL from SatFreeSpaceLoss
    if (m_fsl)
    {
        state.fspl_dB = m_fsl->GetFsldB(satA, satB, m_frequency);
    }
    else
    {
        if (distance_m > 0.0)
        {
            state.fspl_dB = 20.0 * std::log10(4.0 * PI_VAL * distance_m * m_frequency /
                                               SPEED_OF_LIGHT);
        }
        else
        {
            state.fspl_dB = 0.0;
        }
    }

    // SNR from actual positions
    state.snr_dB = ComputeIslSnr_dB(satA, satB);
    state.rxPower_dBm = m_txPower + m_txGain + m_rxGain - state.fspl_dB;
    state.capacity_Gbps = ComputeIslCapacity_Gbps(state.snr_dB);
    state.isEstablished = (state.snr_dB > 0.0);
    state.linkUpTime_s = 0.0;

    return state;
}

// --------------------------------------------------------------------------
// Link feasibility from actual positions
// --------------------------------------------------------------------------

bool
ThzNtnIslChannel::IsLinkFeasible(Ptr<MobilityModel> satA,
                                 Ptr<MobilityModel> satB,
                                 double minSnrDb) const
{
    NS_LOG_FUNCTION(this << minSnrDb);

    double snr = ComputeIslSnr_dB(satA, satB);
    bool feasible = (snr >= minSnrDb);
    NS_LOG_INFO("Link feasibility: SNR=" << snr << " dB vs min=" << minSnrDb
                << " dB -> " << (feasible ? "FEASIBLE" : "INFEASIBLE"));
    return feasible;
}

// --------------------------------------------------------------------------
// Maximum link distance
// --------------------------------------------------------------------------

double
ThzNtnIslChannel::ComputeMaxLinkDistance_km(double minSnrDb) const
{
    NS_LOG_FUNCTION(this << minSnrDb);

    // Noise floor
    double noiseTemp_K = ComputeSpaceNoiseTemperature_K();
    double noisePower_dBm = 10.0 * std::log10(BOLTZMANN_K * noiseTemp_K * m_bandwidth) + 30.0;

    // Required received power for min SNR
    double reqRxPower_dBm = minSnrDb + noisePower_dBm;

    // Maximum tolerable path loss
    double maxFspl_dB = m_txPower + m_txGain + m_rxGain - reqRxPower_dBm;

    // Invert FSPL formula: d = c / (4*pi*f) * 10^(FSPL_dB/20)
    double maxDistanceM = (SPEED_OF_LIGHT / (4.0 * PI_VAL * m_frequency)) *
                          std::pow(10.0, maxFspl_dB / 20.0);
    double maxDistanceKm = maxDistanceM / 1.0e3;

    NS_LOG_INFO("Max ISL distance: " << maxDistanceKm << " km at " << m_frequency / 1.0e9
                << " GHz (max FSPL=" << maxFspl_dB << " dB)");
    return maxDistanceKm;
}

} // namespace ns3
