/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN MAC Protocol — implementation
 */

#include "thz-ntn-mac.h"

#include "thz-ntn-molecular-absorption.h"

#include <ns3/double.h>
#include <ns3/enum.h>
#include <ns3/log.h>
#include <ns3/mobility-model.h>
#include <ns3/string.h>
#include <ns3/uinteger.h>

// Satellite module headers
#include <ns3/satellite-frame-conf.h>
#include <ns3/satellite-wave-form-conf.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnMac");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnMac);

// Physical constants
static constexpr double SPEED_OF_LIGHT = 299792458.0;   // m/s
static constexpr double EARTH_RADIUS_M = 6371000.0;     // m

TypeId
ThzNtnMac::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnMac")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnMac>()
            .AddAttribute("TotalBandwidth",
                          "Total system bandwidth in Hz",
                          DoubleValue(10.0e9),
                          MakeDoubleAccessor(&ThzNtnMac::m_totalBandwidthHz),
                          MakeDoubleChecker<double>(1.0e9, 100.0e9))
            .AddAttribute("SubBandWidth",
                          "Width of each sub-band in Hz",
                          DoubleValue(1.0e9),
                          MakeDoubleAccessor(&ThzNtnMac::m_subBandWidthHz),
                          MakeDoubleChecker<double>(100.0e6, 10.0e9))
            .AddAttribute("CenterFrequency",
                          "Centre frequency of the resource grid in Hz",
                          DoubleValue(225.0e9),
                          MakeDoubleAccessor(&ThzNtnMac::m_centerFreqHz),
                          MakeDoubleChecker<double>(100.0e9, 10.0e12))
            .AddAttribute("AccessMode",
                          "MAC access mode: TDMA, FDMA, HYBRID_TDMA_FDMA, OFDMA",
                          StringValue("HYBRID_TDMA_FDMA"),
                          MakeStringAccessor(&ThzNtnMac::m_accessModeStr),
                          MakeStringChecker())
            .AddAttribute("NumSlots",
                          "Number of time slots per frame",
                          UintegerValue(10),
                          MakeUintegerAccessor(&ThzNtnMac::m_numSlots),
                          MakeUintegerChecker<uint32_t>(1, 1000))
            .AddAttribute("SlotDuration_us",
                          "Duration of each time slot in microseconds",
                          DoubleValue(125.0),
                          MakeDoubleAccessor(&ThzNtnMac::m_slotDuration_us),
                          MakeDoubleChecker<double>(1.0, 10000.0))
            .AddAttribute("GuardInterval_us",
                          "Guard interval between slots in microseconds",
                          DoubleValue(5.0),
                          MakeDoubleAccessor(&ThzNtnMac::m_guardInterval_us),
                          MakeDoubleChecker<double>(0.0, 1000.0))
            .AddAttribute("PreambleDuration_us",
                          "Preamble duration per slot in microseconds",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&ThzNtnMac::m_preambleDuration_us),
                          MakeDoubleChecker<double>(0.0, 1000.0))
            .AddAttribute("PilotOverheadRatio",
                          "Fraction of slot symbols used for pilot/reference signals",
                          DoubleValue(0.04),
                          MakeDoubleAccessor(&ThzNtnMac::m_pilotOverheadRatio),
                          MakeDoubleChecker<double>(0.0, 0.5))
            .AddAttribute("MaxAbsorptionThreshold_dB",
                          "Maximum tolerable molecular absorption loss for DAMC filtering (dB)",
                          DoubleValue(10.0),
                          MakeDoubleAccessor(&ThzNtnMac::m_maxAbsorptionThreshold_dB),
                          MakeDoubleChecker<double>(0.0, 100.0))
            .AddTraceSource("SubBandAllocation",
                            "Trace fired on sub-band allocation (ueId, numBands)",
                            MakeTraceSourceAccessor(&ThzNtnMac::m_subBandAllocationTrace),
                            "ns3::TracedCallback::TwoUint32");
    return tid;
}

ThzNtnMac::ThzNtnMac()
    : m_totalBandwidthHz(10.0e9),
      m_subBandWidthHz(1.0e9),
      m_centerFreqHz(225.0e9),
      m_accessMode(ThzNtnAccessMode::HYBRID_TDMA_FDMA),
      m_accessModeStr("HYBRID_TDMA_FDMA"),
      m_numSlots(10),
      m_slotDuration_us(125.0),
      m_guardInterval_us(5.0),
      m_preambleDuration_us(2.0),
      m_pilotOverheadRatio(0.04),
      m_maxAbsorptionThreshold_dB(10.0),
      m_absorptionModel(nullptr),
      m_superframeConf(nullptr),
      m_waveformConf(nullptr)
{
    NS_LOG_FUNCTION(this);
    m_frame.frameIndex = 0;
    m_frame.numSlots = m_numSlots;
    m_frame.slotDuration_us = m_slotDuration_us;
    m_frame.guardInterval_us = m_guardInterval_us;
    m_frame.preambleDuration_us = m_preambleDuration_us;
    m_frame.pilotOverheadRatio = m_pilotOverheadRatio;
}

ThzNtnMac::~ThzNtnMac()
{
    NS_LOG_FUNCTION(this);
}

void
ThzNtnMac::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_absorptionModel = nullptr;
    m_superframeConf = nullptr;
    m_waveformConf = nullptr;
    m_subBands.clear();
    Object::DoDispose();
}

// --------------------------------------------------------------------------
// Resource grid configuration
// --------------------------------------------------------------------------

void
ThzNtnMac::ConfigureResourceGrid(double totalBandwidthHz,
                                 double subBandWidthHz,
                                 double centerFreqHz)
{
    NS_LOG_FUNCTION(this << totalBandwidthHz << subBandWidthHz << centerFreqHz);

    m_totalBandwidthHz = totalBandwidthHz;
    m_subBandWidthHz = subBandWidthHz;
    m_centerFreqHz = centerFreqHz;
    m_subBands.clear();

    uint32_t numSubBands =
        static_cast<uint32_t>(std::floor(totalBandwidthHz / subBandWidthHz));
    NS_LOG_INFO("Configuring resource grid: " << numSubBands << " sub-bands, "
                << subBandWidthHz / 1.0e9 << " GHz each");

    double startFreqHz = centerFreqHz - (totalBandwidthHz / 2.0);

    for (uint32_t i = 0; i < numSubBands; ++i)
    {
        ThzNtnSubBand sb;
        sb.index = i;
        sb.centerFreqHz = startFreqHz + (i + 0.5) * subBandWidthHz;
        sb.bandwidthHz = subBandWidthHz;
        sb.isAvailable = true;
        sb.absorptionLoss_dB = 0.0;
        sb.assignedUeId = 0;
        sb.carrierIndex = i; // default 1:1 mapping
        m_subBands.push_back(sb);
    }

    // If superframe conf is set, map sub-bands to satellite carriers
    if (m_superframeConf)
    {
        MapSubBandsToCarriers();
    }
}

// --------------------------------------------------------------------------
// Link geometry computation from MobilityModel
// --------------------------------------------------------------------------

void
ThzNtnMac::ComputeLinkGeometry(Ptr<MobilityModel> txMobility,
                               Ptr<MobilityModel> rxMobility,
                               double& distanceM,
                               double& altTx_km,
                               double& altRx_km,
                               double& elevationDeg) const
{
    NS_LOG_FUNCTION(this);

    Vector txPos = txMobility->GetPosition();
    Vector rxPos = rxMobility->GetPosition();

    // 3D Euclidean distance
    double dx = txPos.x - rxPos.x;
    double dy = txPos.y - rxPos.y;
    double dz = txPos.z - rxPos.z;
    distanceM = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Altitude estimation: in ns-3 geocentric coordinates, altitude
    // is the distance from Earth centre minus Earth radius.
    // For Cartesian positions, use magnitude from origin.
    double txR = std::sqrt(txPos.x * txPos.x + txPos.y * txPos.y + txPos.z * txPos.z);
    double rxR = std::sqrt(rxPos.x * rxPos.x + rxPos.y * rxPos.y + rxPos.z * rxPos.z);

    altTx_km = std::max(0.0, (txR - EARTH_RADIUS_M) / 1000.0);
    altRx_km = std::max(0.0, (rxR - EARTH_RADIUS_M) / 1000.0);

    // Elevation angle from the lower node's perspective
    double lowerR = std::min(txR, rxR);
    double higherR = std::max(txR, rxR);

    // Using the triangle: Earth centre, lower node, higher node
    // cos(nadir) = (lowerR^2 + d^2 - higherR^2) / (2 * lowerR * d)
    if (distanceM > 0.0 && lowerR > 0.0)
    {
        double cosNadir = (lowerR * lowerR + distanceM * distanceM -
                           higherR * higherR) /
                          (2.0 * lowerR * distanceM);
        cosNadir = std::max(-1.0, std::min(1.0, cosNadir));
        // Elevation = 90 - nadir_angle
        double nadirRad = std::acos(cosNadir);
        elevationDeg = 90.0 - nadirRad * 180.0 / M_PI;
    }
    else
    {
        elevationDeg = 90.0; // directly overhead
    }

    NS_LOG_DEBUG("LinkGeometry: dist=" << distanceM << " m, altTx=" << altTx_km
                 << " km, altRx=" << altRx_km << " km, elev=" << elevationDeg << " deg");
}

// --------------------------------------------------------------------------
// Sub-band availability update using actual absorption model
// --------------------------------------------------------------------------

void
ThzNtnMac::UpdateSubBandAvailability(Ptr<MobilityModel> txMobility,
                                     Ptr<MobilityModel> rxMobility)
{
    NS_LOG_FUNCTION(this);

    if (!m_absorptionModel)
    {
        NS_LOG_WARN("No molecular absorption model set; sub-band availability unchanged");
        return;
    }

    if (!txMobility || !rxMobility)
    {
        NS_LOG_WARN("Null MobilityModel pointer; cannot compute link geometry");
        return;
    }

    // Compute actual link geometry from node positions
    double distanceM = 0.0;
    double altTx_km = 0.0;
    double altRx_km = 0.0;
    double elevationDeg = 0.0;
    ComputeLinkGeometry(txMobility, rxMobility, distanceM, altTx_km, altRx_km, elevationDeg);

    // For each sub-band, compute actual absorption at its centre frequency
    for (auto& sb : m_subBands)
    {
        sb.absorptionLoss_dB = m_absorptionModel->ComputeAbsorptionLoss_dB(
            sb.centerFreqHz, distanceM, altTx_km, altRx_km, elevationDeg);

        sb.isAvailable = (sb.absorptionLoss_dB <= m_maxAbsorptionThreshold_dB) &&
                         (sb.assignedUeId == 0);

        NS_LOG_DEBUG("Sub-band " << sb.index << " @ " << sb.centerFreqHz / 1.0e9
                     << " GHz: absorption=" << sb.absorptionLoss_dB
                     << " dB, available=" << sb.isAvailable);
    }
}

// --------------------------------------------------------------------------
// DAMC sub-band filtering
// --------------------------------------------------------------------------

std::vector<ThzNtnSubBand>
ThzNtnMac::GetAvailableSubBands(double maxAbsorptionLoss_dB) const
{
    NS_LOG_FUNCTION(this << maxAbsorptionLoss_dB);

    std::vector<ThzNtnSubBand> available;
    for (const auto& sb : m_subBands)
    {
        if (sb.isAvailable && sb.assignedUeId == 0 &&
            sb.absorptionLoss_dB <= maxAbsorptionLoss_dB)
        {
            available.push_back(sb);
        }
    }

    NS_LOG_INFO("DAMC filter: " << available.size() << " / " << m_subBands.size()
                << " sub-bands below " << maxAbsorptionLoss_dB << " dB threshold");
    return available;
}

// --------------------------------------------------------------------------
// Distance-aware sub-band allocation (MobilityModel-based)
// --------------------------------------------------------------------------

void
ThzNtnMac::AllocateSubBands(uint32_t ueId,
                            uint32_t numSubBands,
                            Ptr<MobilityModel> txMobility,
                            Ptr<MobilityModel> rxMobility)
{
    NS_LOG_FUNCTION(this << ueId << numSubBands);

    // Step 1: update absorption for actual link geometry
    UpdateSubBandAvailability(txMobility, rxMobility);

    // Step 2: get available sub-bands below absorption threshold
    std::vector<ThzNtnSubBand> available = GetAvailableSubBands(m_maxAbsorptionThreshold_dB);

    if (available.empty())
    {
        NS_LOG_WARN("No sub-bands available for UE " << ueId);
        return;
    }

    // Step 3: sort by absorption loss (ascending) -- prefer transparency windows
    std::sort(available.begin(),
              available.end(),
              [](const ThzNtnSubBand& a, const ThzNtnSubBand& b) {
                  return a.absorptionLoss_dB < b.absorptionLoss_dB;
              });

    // Step 4: allocate the best sub-bands
    uint32_t allocated = 0;
    for (const auto& avail : available)
    {
        if (allocated >= numSubBands)
        {
            break;
        }
        for (auto& sb : m_subBands)
        {
            if (sb.index == avail.index && sb.assignedUeId == 0)
            {
                sb.assignedUeId = ueId;
                sb.isAvailable = false;
                ++allocated;
                NS_LOG_INFO("Allocated sub-band " << sb.index << " to UE " << ueId
                            << " (absorption=" << sb.absorptionLoss_dB << " dB)");
                break;
            }
        }
    }

    m_subBandAllocationTrace(ueId, allocated);

    NS_LOG_INFO("Total allocated: " << allocated << " / " << numSubBands
                << " sub-bands for UE " << ueId);
}

void
ThzNtnMac::ReleaseSubBands(uint32_t ueId)
{
    NS_LOG_FUNCTION(this << ueId);

    uint32_t released = 0;
    for (auto& sb : m_subBands)
    {
        if (sb.assignedUeId == ueId)
        {
            sb.assignedUeId = 0;
            sb.isAvailable = true;
            ++released;
        }
    }
    NS_LOG_INFO("Released " << released << " sub-bands from UE " << ueId);
}

uint32_t
ThzNtnMac::GetNumActiveSubBands() const
{
    uint32_t count = 0;
    for (const auto& sb : m_subBands)
    {
        if (sb.assignedUeId != 0)
        {
            ++count;
        }
    }
    return count;
}

// --------------------------------------------------------------------------
// Molecular absorption model
// --------------------------------------------------------------------------

void
ThzNtnMac::SetMolecularAbsorptionModel(Ptr<ThzNtnMolecularAbsorption> model)
{
    NS_LOG_FUNCTION(this << model);
    m_absorptionModel = model;
}

// --------------------------------------------------------------------------
// Satellite superframe / waveform integration
// --------------------------------------------------------------------------

void
ThzNtnMac::SetSuperframeConf(Ptr<SatSuperframeConf> sfConf)
{
    NS_LOG_FUNCTION(this);
    m_superframeConf = sfConf;
    if (!m_subBands.empty())
    {
        MapSubBandsToCarriers();
    }
}

void
ThzNtnMac::SetWaveformConf(Ptr<SatWaveformConf> wfConf)
{
    NS_LOG_FUNCTION(this);
    m_waveformConf = wfConf;
}

void
ThzNtnMac::MapSubBandsToCarriers()
{
    NS_LOG_FUNCTION(this);

    if (!m_superframeConf)
    {
        return;
    }

    // Map THz sub-bands to satellite carriers: cycle through available frames
    // and carriers. Each sub-band maps to one carrier in the superframe.
    uint32_t totalCarriers = m_superframeConf->GetCarrierCount();

    if (totalCarriers == 0)
    {
        NS_LOG_WARN("Superframe has no carriers; using default 1:1 mapping");
        return;
    }

    for (auto& sb : m_subBands)
    {
        sb.carrierIndex = sb.index % totalCarriers;
    }

    NS_LOG_INFO("Mapped " << m_subBands.size() << " sub-bands to "
                << totalCarriers << " satellite carriers");
}

uint32_t
ThzNtnMac::SelectWaveformId(double cnoDb) const
{
    if (!m_waveformConf)
    {
        return 0;
    }

    // Convert C/N0 in dB to linear and query SatWaveformConf
    // Use a nominal symbol rate for the query
    double symbolRate = m_subBandWidthHz; // 1 symbol per Hz bandwidth
    double cnoLinear = std::pow(10.0, cnoDb / 10.0);
    uint32_t wfId = 0;
    double cnoThreshold = 0.0;

    bool found = m_waveformConf->GetBestWaveformId(
        cnoLinear, symbolRate, wfId, cnoThreshold);

    if (!found)
    {
        NS_LOG_DEBUG("No waveform found for C/N0=" << cnoDb << " dB, using default");
        return 0;
    }

    return wfId;
}

// --------------------------------------------------------------------------
// Tx info creation
// --------------------------------------------------------------------------

ThzNtnTxInfo
ThzNtnMac::CreateTxInfo(uint32_t ueId,
                        const std::vector<uint32_t>& subBandIndices,
                        uint8_t modcod) const
{
    NS_LOG_FUNCTION(this << ueId << modcod);

    ThzNtnTxInfo txInfo;
    txInfo.ueId = ueId;
    txInfo.modcod = modcod;
    txInfo.sliceId = 0;
    txInfo.subBandIndices = subBandIndices;
    txInfo.waveformId = 0;

    // Compute TB size based on waveform spectral efficiency if available
    if (m_waveformConf)
    {
        // Use a proxy C/N0 to select waveform. The actual SINR-based
        // selection happens in the scheduler; here we provide structure.
        uint32_t wfId = SelectWaveformId(static_cast<double>(modcod));
        txInfo.waveformId = wfId;

        if (wfId > 0)
        {
            Ptr<SatWaveform> wf = m_waveformConf->GetWaveform(wfId);
            if (wf)
            {
                double symbolRate = m_subBandWidthHz;
                double spectralEff = wf->GetSpectralEfficiency(m_subBandWidthHz, symbolRate);
                double totalBw = static_cast<double>(subBandIndices.size()) * m_subBandWidthHz;
                // TB bits = SE * BW * slot_duration
                double slotDur_s = m_slotDuration_us * 1.0e-6;
                double tbBits = spectralEff * totalBw * slotDur_s * ComputeMacEfficiency();
                txInfo.tbSizeBytes = static_cast<uint32_t>(std::max(tbBits / 8.0, 1.0));
            }
        }
    }

    if (txInfo.tbSizeBytes == 0)
    {
        // Fallback: estimate from modcod index
        double spectralEff = 0.5 + 0.2 * static_cast<double>(modcod);
        double totalBw = static_cast<double>(subBandIndices.size()) * m_subBandWidthHz;
        double slotDur_s = m_slotDuration_us * 1.0e-6;
        double tbBits = spectralEff * totalBw * slotDur_s * ComputeMacEfficiency();
        txInfo.tbSizeBytes = static_cast<uint32_t>(std::max(tbBits / 8.0, 1.0));
    }

    NS_LOG_INFO("TxInfo: UE=" << ueId << " wfId=" << txInfo.waveformId
                << " TB=" << txInfo.tbSizeBytes << " bytes, "
                << subBandIndices.size() << " sub-bands");
    return txInfo;
}

// --------------------------------------------------------------------------
// Access mode
// --------------------------------------------------------------------------

void
ThzNtnMac::SetAccessMode(ThzNtnAccessMode mode)
{
    NS_LOG_FUNCTION(this << static_cast<uint8_t>(mode));
    m_accessMode = mode;
}

ThzNtnAccessMode
ThzNtnMac::ParseAccessMode(const std::string& mode)
{
    if (mode == "TDMA")
    {
        return ThzNtnAccessMode::TDMA;
    }
    if (mode == "FDMA")
    {
        return ThzNtnAccessMode::FDMA;
    }
    if (mode == "OFDMA")
    {
        return ThzNtnAccessMode::OFDMA;
    }
    return ThzNtnAccessMode::HYBRID_TDMA_FDMA;
}

// --------------------------------------------------------------------------
// Frame configuration (fully parameterised overhead)
// --------------------------------------------------------------------------

void
ThzNtnMac::ConfigureFrame(uint32_t numSlots,
                          double slotDuration_us,
                          double guardInterval_us,
                          double preambleDuration_us,
                          double pilotOverheadRatio)
{
    NS_LOG_FUNCTION(this << numSlots << slotDuration_us << guardInterval_us
                         << preambleDuration_us << pilotOverheadRatio);

    m_numSlots = numSlots;
    m_slotDuration_us = slotDuration_us;
    m_guardInterval_us = guardInterval_us;
    m_preambleDuration_us = preambleDuration_us;
    m_pilotOverheadRatio = pilotOverheadRatio;

    m_frame.numSlots = numSlots;
    m_frame.slotDuration_us = slotDuration_us;
    m_frame.guardInterval_us = guardInterval_us;
    m_frame.preambleDuration_us = preambleDuration_us;
    m_frame.pilotOverheadRatio = pilotOverheadRatio;
}

// --------------------------------------------------------------------------
// MAC efficiency (accounts for guard, preamble, pilot overhead)
// --------------------------------------------------------------------------

double
ThzNtnMac::ComputeMacEfficiency() const
{
    NS_LOG_FUNCTION(this);

    // Total frame time per slot = slot + guard
    double totalSlotTime_us = m_frame.slotDuration_us + m_frame.guardInterval_us;
    if (totalSlotTime_us <= 0.0)
    {
        return 0.0;
    }

    // Useful data time per slot = slot - preamble
    double usefulTime_us = m_frame.slotDuration_us - m_frame.preambleDuration_us;
    if (usefulTime_us <= 0.0)
    {
        return 0.0;
    }

    // Guard + preamble efficiency
    double timeEff = usefulTime_us / totalSlotTime_us;

    // Pilot overhead: fraction of remaining symbols used for pilots
    double dataEff = 1.0 - m_frame.pilotOverheadRatio;

    double efficiency = timeEff * dataEff;

    NS_LOG_DEBUG("MAC efficiency: timeEff=" << timeEff << " dataEff=" << dataEff
                 << " total=" << efficiency
                 << " (slot=" << m_frame.slotDuration_us
                 << " guard=" << m_frame.guardInterval_us
                 << " preamble=" << m_frame.preambleDuration_us
                 << " pilotRatio=" << m_frame.pilotOverheadRatio << ")");
    return efficiency;
}

// --------------------------------------------------------------------------
// Aggregate capacity using actual waveform spectral efficiencies
// --------------------------------------------------------------------------

double
ThzNtnMac::ComputeAggregateCapacity_Gbps(const std::vector<double>& subBandSinrs_dB) const
{
    NS_LOG_FUNCTION(this);

    double totalCapacity_bps = 0.0;
    uint32_t sinrIdx = 0;

    for (const auto& sb : m_subBands)
    {
        if (sb.assignedUeId != 0 && sinrIdx < subBandSinrs_dB.size())
        {
            double sinrDb = subBandSinrs_dB[sinrIdx];

            if (m_waveformConf)
            {
                // Use actual waveform spectral efficiency from SatWaveformConf
                double cnoLinear = std::pow(10.0, sinrDb / 10.0);
                double symbolRate = sb.bandwidthHz;
                uint32_t wfId = 0;
                double cnoThreshold = 0.0;

                bool found = m_waveformConf->GetBestWaveformId(
                    cnoLinear, symbolRate, wfId, cnoThreshold);

                if (found && wfId > 0)
                {
                    Ptr<SatWaveform> wf = m_waveformConf->GetWaveform(wfId);
                    if (wf)
                    {
                        double spectralEff = wf->GetSpectralEfficiency(
                            sb.bandwidthHz, symbolRate);
                        totalCapacity_bps += spectralEff * sb.bandwidthHz;
                    }
                }
                else
                {
                    // Below minimum waveform threshold: no capacity
                    NS_LOG_DEBUG("Sub-band " << sb.index
                                 << ": SINR too low for any waveform");
                }
            }
            else
            {
                // Fallback to Shannon with implementation loss
                double sinrLinear = std::pow(10.0, sinrDb / 10.0);
                // 3 dB implementation loss factor (0.5 in linear)
                double implLoss = 0.5;
                double capacity_bps = sb.bandwidthHz *
                    std::log2(1.0 + sinrLinear * implLoss);
                totalCapacity_bps += capacity_bps;
            }
            ++sinrIdx;
        }
    }

    // Apply MAC efficiency factor
    double efficiency = ComputeMacEfficiency();
    double effectiveCapacity_Gbps = (totalCapacity_bps * efficiency) / 1.0e9;

    NS_LOG_INFO("Aggregate capacity: " << effectiveCapacity_Gbps << " Gbps"
                << " (" << sinrIdx << " active sub-bands, efficiency="
                << efficiency << ")");
    return effectiveCapacity_Gbps;
}

} // namespace ns3
