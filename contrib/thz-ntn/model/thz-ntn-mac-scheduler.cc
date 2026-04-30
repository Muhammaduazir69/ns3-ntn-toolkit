/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN MAC Scheduler — implementation
 */

#include "thz-ntn-mac-scheduler.h"

// mmWave module headers
#include <ns3/mmwave-amc.h>
#include <ns3/mmwave-phy-mac-common.h>

// Satellite module headers
#include <ns3/satellite-wave-form-conf.h>

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/mobility-model.h>
#include <ns3/string.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnMacScheduler");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnMacScheduler);

// Physical constants
static constexpr double SPEED_OF_LIGHT = 299792458.0; // m/s

// SCS options in Hz
static constexpr uint32_t SCS_15KHZ  = 15000;
static constexpr uint32_t SCS_30KHZ  = 30000;
static constexpr uint32_t SCS_60KHZ  = 60000;
static constexpr uint32_t SCS_120KHZ = 120000;
static constexpr uint32_t SCS_240KHZ = 240000;
static constexpr uint32_t SCS_480KHZ = 480000;
static constexpr uint32_t SCS_960KHZ = 960000;

// Fallback MCS SINR thresholds (used only when no AMC module is set)
static const double MCS_SINR_THRESHOLDS[] = {
    -6.7, -4.7, -2.3, 0.2, 2.4, 4.3, 5.9, 8.1, 10.3, 11.7,
    14.1, 15.6, 17.0, 18.8, 20.4, 21.8, 23.4, 25.1, 26.8, 28.2,
    29.4, 30.6, 31.7, 32.8, 33.8, 35.0, 36.0, 37.5, 39.0
};

static const double MCS_SPECTRAL_EFFICIENCY[] = {
    0.2344, 0.3770, 0.6016, 0.8770, 1.1758, 1.4766, 1.6953,
    1.9141, 2.1602, 2.4063, 2.5703, 2.7305, 3.0293, 3.3223,
    3.6094, 3.9023, 4.2129, 4.5234, 4.8164, 5.1152, 5.3320,
    5.5547, 5.7539, 5.9297, 6.0840, 6.2266, 6.3477, 6.4531, 6.5547
};

static constexpr uint32_t NUM_MCS_ENTRIES = 29;

TypeId
ThzNtnMacScheduler::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnMacScheduler")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnMacScheduler>()
            .AddAttribute("SchedulerType",
                          "Scheduling algorithm: ROUND_ROBIN, PROPORTIONAL_FAIR, "
                          "MAX_THROUGHPUT, QOS_AWARE",
                          StringValue("QOS_AWARE"),
                          MakeStringAccessor(&ThzNtnMacScheduler::m_schedulerTypeStr),
                          MakeStringChecker())
            .AddAttribute("MaxRetransmissions",
                          "Maximum number of HARQ retransmissions",
                          UintegerValue(3),
                          MakeUintegerAccessor(&ThzNtnMacScheduler::m_maxRetransmissions),
                          MakeUintegerChecker<uint32_t>(0, 15))
            .AddAttribute("TargetBler",
                          "Target block error rate for MCS selection",
                          DoubleValue(0.01),
                          MakeDoubleAccessor(&ThzNtnMacScheduler::m_targetBler),
                          MakeDoubleChecker<double>(1.0e-6, 0.5))
            .AddAttribute("PfAlpha",
                          "Exponential moving average factor for PF throughput",
                          DoubleValue(0.01),
                          MakeDoubleAccessor(&ThzNtnMacScheduler::m_pfAlpha),
                          MakeDoubleChecker<double>(0.001, 1.0));
    return tid;
}

ThzNtnMacScheduler::ThzNtnMacScheduler()
    : m_schedulerType(ThzNtnSchedulerType::QOS_AWARE),
      m_schedulerTypeStr("QOS_AWARE"),
      m_maxRetransmissions(3),
      m_targetBler(0.01),
      m_nextHarqProcessId(0),
      m_amc(nullptr),
      m_waveformConf(nullptr),
      m_satMobility(nullptr),
      m_pfAlpha(0.01)
{
    NS_LOG_FUNCTION(this);
}

ThzNtnMacScheduler::~ThzNtnMacScheduler()
{
    NS_LOG_FUNCTION(this);
}

void
ThzNtnMacScheduler::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_amc = nullptr;
    m_waveformConf = nullptr;
    m_satMobility = nullptr;
    m_avgThroughput.clear();
    Object::DoDispose();
}

// --------------------------------------------------------------------------
// AMC integration
// --------------------------------------------------------------------------

void
ThzNtnMacScheduler::SetAmc(Ptr<mmwave::MmWaveAmc> amc)
{
    NS_LOG_FUNCTION(this);
    m_amc = amc;
}

void
ThzNtnMacScheduler::SetWaveformConf(Ptr<SatWaveformConf> wfConf)
{
    NS_LOG_FUNCTION(this);
    m_waveformConf = wfConf;
}

void
ThzNtnMacScheduler::SetSatelliteMobility(Ptr<MobilityModel> satMobility)
{
    NS_LOG_FUNCTION(this);
    m_satMobility = satMobility;
}

// --------------------------------------------------------------------------
// MCS selection using actual AMC
// --------------------------------------------------------------------------

uint8_t
ThzNtnMacScheduler::SelectMcs(double sinrDb) const
{
    NS_LOG_FUNCTION(this << sinrDb);

    if (m_amc)
    {
        // Use mmWave AMC: convert SINR to spectral efficiency proxy,
        // then query AMC for the best MCS
        double sinrLin = std::pow(10.0, sinrDb / 10.0);
        double spectralEff = std::log2(1.0 + sinrLin);
        uint8_t mcs = m_amc->GetMcsFromSpectralEfficiency(spectralEff);
        NS_LOG_INFO("AMC MCS selection: SINR=" << sinrDb << " dB -> MCS "
                    << static_cast<uint32_t>(mcs));
        return mcs;
    }

    // Fallback: use internal SINR threshold table
    // Apply a BLER back-off
    double backoff_dB = 0.0;
    if (m_targetBler < 0.001)
    {
        backoff_dB = 3.0;
    }
    else if (m_targetBler < 0.01)
    {
        backoff_dB = 1.5;
    }

    double effectiveSinr = sinrDb - backoff_dB;

    uint8_t mcs = 0;
    for (uint32_t i = 0; i < NUM_MCS_ENTRIES; ++i)
    {
        if (effectiveSinr >= MCS_SINR_THRESHOLDS[i])
        {
            mcs = static_cast<uint8_t>(i);
        }
        else
        {
            break;
        }
    }

    NS_LOG_INFO("Fallback MCS selection: SINR=" << sinrDb << " dB, effective="
                << effectiveSinr << " dB -> MCS " << static_cast<uint32_t>(mcs));
    return mcs;
}

// --------------------------------------------------------------------------
// Doppler computation from actual MobilityModel
// --------------------------------------------------------------------------

double
ThzNtnMacScheduler::ComputeDopplerHz(Ptr<MobilityModel> satMobility,
                                     Ptr<MobilityModel> ueMobility) const
{
    NS_LOG_FUNCTION(this);

    if (!satMobility || !ueMobility)
    {
        return 0.0;
    }

    Vector satPos = satMobility->GetPosition();
    Vector uePos = ueMobility->GetPosition();
    Vector satVel = satMobility->GetVelocity();
    Vector ueVel = ueMobility->GetVelocity();

    // Relative position vector (sat - ue)
    double dx = satPos.x - uePos.x;
    double dy = satPos.y - uePos.y;
    double dz = satPos.z - uePos.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist < 1.0)
    {
        return 0.0;
    }

    // Unit vector from UE to satellite
    double ux = dx / dist;
    double uy = dy / dist;
    double uz = dz / dist;

    // Relative velocity projected onto line-of-sight
    double dvx = satVel.x - ueVel.x;
    double dvy = satVel.y - ueVel.y;
    double dvz = satVel.z - ueVel.z;
    double vRadial = dvx * ux + dvy * uy + dvz * uz;

    // Doppler: f_d = -v_radial * f_c / c
    // (negative because approaching = positive Doppler)
    // Use a nominal THz centre frequency for the computation
    double fCenter = 225.0e9; // will be updated if we have sub-band info
    double dopplerHz = -vRadial * fCenter / SPEED_OF_LIGHT;

    NS_LOG_DEBUG("Doppler: vRadial=" << vRadial << " m/s, fCenter="
                 << fCenter / 1.0e9 << " GHz -> " << dopplerHz << " Hz");
    return dopplerHz;
}

uint32_t
ThzNtnMacScheduler::SelectScs(Ptr<MobilityModel> satMobility,
                               Ptr<MobilityModel> ueMobility) const
{
    NS_LOG_FUNCTION(this);

    double dopplerHz = ComputeDopplerHz(satMobility, ueMobility);
    double absDoppler = std::fabs(dopplerHz);

    // SCS must be >> Doppler spread to maintain orthogonality.
    // Rule of thumb: SCS > 10 * max Doppler shift.
    uint32_t scs;
    if (absDoppler < 1500)
    {
        scs = SCS_15KHZ;
    }
    else if (absDoppler < 3000)
    {
        scs = SCS_30KHZ;
    }
    else if (absDoppler < 6000)
    {
        scs = SCS_60KHZ;
    }
    else if (absDoppler < 12000)
    {
        scs = SCS_120KHZ;
    }
    else if (absDoppler < 24000)
    {
        scs = SCS_240KHZ;
    }
    else if (absDoppler < 48000)
    {
        scs = SCS_480KHZ;
    }
    else
    {
        scs = SCS_960KHZ;
    }

    NS_LOG_INFO("SCS selection: Doppler=" << dopplerHz << " Hz -> SCS="
                << scs / 1000 << " kHz");
    return scs;
}

// --------------------------------------------------------------------------
// Transport block size from actual AMC
// --------------------------------------------------------------------------

uint32_t
ThzNtnMacScheduler::ComputeTbSize(uint8_t mcs, uint32_t numSubBands,
                                  uint32_t numSymbols) const
{
    NS_LOG_FUNCTION(this << (uint32_t)mcs << numSubBands << numSymbols);

    if (m_amc)
    {
        // MmWaveAmc::CalculateTbSize takes MCS and number of symbols,
        // assuming all RBs are allocated. Scale by sub-band count.
        uint32_t tbPerSymSet = m_amc->CalculateTbSize(mcs, static_cast<uint8_t>(numSymbols));
        // The AMC assumes full bandwidth; scale proportionally if we use
        // a subset of sub-bands. This is an approximation.
        return tbPerSymSet;
    }

    // Fallback: use spectral efficiency table
    uint8_t effMcs = std::min(mcs, static_cast<uint8_t>(NUM_MCS_ENTRIES - 1));
    double spectralEff = MCS_SPECTRAL_EFFICIENCY[effMcs];
    // TB bits = SE * numSubBands * 12_subcarriers_per_RB * numSymbols * 2_polarizations
    double tbBits = spectralEff * static_cast<double>(numSubBands) * 12.0 *
                    static_cast<double>(numSymbols) * 2.0;
    uint32_t tbBytes = static_cast<uint32_t>(tbBits / 8.0);
    return std::max(tbBytes, 1u);
}

// --------------------------------------------------------------------------
// DCI creation for mmWave integration
// --------------------------------------------------------------------------

mmwave::DciInfoElementTdma
ThzNtnMacScheduler::CreateDci(const ThzNtnSchedulingDecision& decision) const
{
    NS_LOG_FUNCTION(this << decision.ueId);

    mmwave::DciInfoElementTdma dci;
    dci.m_rnti = decision.rnti;
    dci.m_format = mmwave::DciInfoElementTdma::DL_dci;
    dci.m_symStart = 0;
    dci.m_numSym = static_cast<uint8_t>(decision.numSymbols);
    dci.m_mcs = decision.mcsIndex;
    dci.m_tbSize = decision.tbSize_bytes;
    dci.m_ndi = 1; // new data
    dci.m_rv = 0;  // first transmission
    dci.m_harqProcess = decision.harqProcessId;

    NS_LOG_DEBUG("DCI: rnti=" << dci.m_rnti << " mcs=" << (uint32_t)dci.m_mcs
                 << " sym=" << (uint32_t)dci.m_numSym << " tbs=" << dci.m_tbSize);
    return dci;
}

// --------------------------------------------------------------------------
// Proportional Fair with actual throughput history
// --------------------------------------------------------------------------

void
ThzNtnMacScheduler::UpdateUeThroughput(uint32_t ueId, double throughput_Mbps)
{
    NS_LOG_FUNCTION(this << ueId << throughput_Mbps);

    auto it = m_avgThroughput.find(ueId);
    if (it == m_avgThroughput.end())
    {
        m_avgThroughput[ueId] = throughput_Mbps;
    }
    else
    {
        // Exponential moving average
        it->second = m_pfAlpha * throughput_Mbps + (1.0 - m_pfAlpha) * it->second;
    }
}

double
ThzNtnMacScheduler::ComputePfMetric(uint32_t ueId, double instantRate,
                                    double avgRate) const
{
    NS_LOG_FUNCTION(this << ueId << instantRate << avgRate);

    double avg = avgRate;
    auto it = m_avgThroughput.find(ueId);
    if (it != m_avgThroughput.end() && it->second > 0.0)
    {
        avg = it->second;
    }
    avg = std::max(avg, 0.001); // prevent division by zero

    return instantRate / avg;
}

// --------------------------------------------------------------------------
// Scheduler type
// --------------------------------------------------------------------------

void
ThzNtnMacScheduler::SetSchedulerType(ThzNtnSchedulerType type)
{
    NS_LOG_FUNCTION(this << static_cast<uint8_t>(type));
    m_schedulerType = type;
}

ThzNtnSchedulerType
ThzNtnMacScheduler::ParseSchedulerType(const std::string& s)
{
    if (s == "ROUND_ROBIN")
    {
        return ThzNtnSchedulerType::ROUND_ROBIN;
    }
    if (s == "PROPORTIONAL_FAIR")
    {
        return ThzNtnSchedulerType::PROPORTIONAL_FAIR;
    }
    if (s == "MAX_THROUGHPUT")
    {
        return ThzNtnSchedulerType::MAX_THROUGHPUT;
    }
    return ThzNtnSchedulerType::QOS_AWARE;
}

// --------------------------------------------------------------------------
// Scheduler metric
// --------------------------------------------------------------------------

double
ThzNtnMacScheduler::ComputeSchedulerMetric(const ThzNtnUeContext& ue,
                                           ThzNtnSchedulerType type) const
{
    switch (type)
    {
    case ThzNtnSchedulerType::ROUND_ROBIN:
        return 1.0;
    case ThzNtnSchedulerType::PROPORTIONAL_FAIR:
    {
        double sinrLin = std::pow(10.0, ue.sinr_dB / 10.0);
        double instRate = std::log2(1.0 + sinrLin);
        return ComputePfMetric(ue.ueId, instRate, ue.throughput_Mbps);
    }
    case ThzNtnSchedulerType::MAX_THROUGHPUT:
        return ue.sinr_dB;
    case ThzNtnSchedulerType::QOS_AWARE:
    {
        double classPriority = (ue.qosClass == ThzNtnQosClass::URLLC) ? 100.0
                             : (ue.qosClass == ThzNtnQosClass::EMBB)  ? 10.0
                                                                       : 1.0;
        double bufferUrgency = std::log2(1.0 + ue.bufferSize_bytes / 1000.0);
        return classPriority * bufferUrgency;
    }
    default:
        return 1.0;
    }
}

// --------------------------------------------------------------------------
// Main scheduling entry point
// --------------------------------------------------------------------------

std::vector<ThzNtnSchedulingDecision>
ThzNtnMacScheduler::Schedule(const std::vector<ThzNtnUeContext>& ues,
                             const std::vector<ThzNtnSubBand>& availableSubBands)
{
    NS_LOG_FUNCTION(this << ues.size() << availableSubBands.size());

    m_schedulerType = ParseSchedulerType(m_schedulerTypeStr);

    switch (m_schedulerType)
    {
    case ThzNtnSchedulerType::ROUND_ROBIN:
        return ScheduleRoundRobin(ues, availableSubBands);
    case ThzNtnSchedulerType::PROPORTIONAL_FAIR:
        return ScheduleProportionalFair(ues, availableSubBands);
    case ThzNtnSchedulerType::MAX_THROUGHPUT:
        return ScheduleMaxThroughput(ues, availableSubBands);
    case ThzNtnSchedulerType::QOS_AWARE:
    default:
        return ScheduleQosAware(ues, availableSubBands);
    }
}

// --------------------------------------------------------------------------
// Round Robin
// --------------------------------------------------------------------------

std::vector<ThzNtnSchedulingDecision>
ThzNtnMacScheduler::ScheduleRoundRobin(
    const std::vector<ThzNtnUeContext>& ues,
    const std::vector<ThzNtnSubBand>& subBands)
{
    NS_LOG_FUNCTION(this);

    std::vector<ThzNtnSchedulingDecision> decisions;
    if (ues.empty() || subBands.empty())
    {
        return decisions;
    }

    uint32_t bandsPerUe = std::max(1u,
        static_cast<uint32_t>(subBands.size()) / static_cast<uint32_t>(ues.size()));
    uint32_t sbIdx = 0;

    for (const auto& ue : ues)
    {
        ThzNtnSchedulingDecision dec;
        dec.ueId = ue.ueId;
        dec.rnti = ue.rnti;
        dec.waveformType = ThzNtnWaveformType::CP_OFDM;
        dec.numSymbols = 14;
        dec.harqProcessId = m_nextHarqProcessId++ % 8;

        // SCS from actual Doppler if satellite mobility is available
        if (m_satMobility && ue.mobility)
        {
            dec.scsHz = SelectScs(m_satMobility, ue.mobility);
        }
        else
        {
            // Fallback: use Doppler from UE context
            double absDoppler = std::fabs(ue.dopplerHz);
            dec.scsHz = (absDoppler < 6000) ? SCS_60KHZ :
                        (absDoppler < 24000) ? SCS_240KHZ : SCS_960KHZ;
        }

        for (uint32_t i = 0; i < bandsPerUe && sbIdx < subBands.size(); ++i, ++sbIdx)
        {
            dec.subBandIndices.push_back(subBands[sbIdx].index);
        }

        dec.mcsIndex = SelectMcs(ue.sinr_dB);
        dec.tbSize_bytes = ComputeTbSize(dec.mcsIndex,
                                         static_cast<uint32_t>(dec.subBandIndices.size()),
                                         dec.numSymbols);
        // Update throughput history
        UpdateUeThroughput(ue.ueId, ue.throughput_Mbps);

        decisions.push_back(dec);
    }

    return decisions;
}

// --------------------------------------------------------------------------
// Proportional Fair
// --------------------------------------------------------------------------

std::vector<ThzNtnSchedulingDecision>
ThzNtnMacScheduler::ScheduleProportionalFair(
    const std::vector<ThzNtnUeContext>& ues,
    const std::vector<ThzNtnSubBand>& subBands)
{
    NS_LOG_FUNCTION(this);

    // Sort UEs by PF metric (descending)
    auto sortedUes = ues;
    std::sort(sortedUes.begin(), sortedUes.end(),
              [this](const ThzNtnUeContext& a, const ThzNtnUeContext& b) {
                  return ComputeSchedulerMetric(a, ThzNtnSchedulerType::PROPORTIONAL_FAIR) >
                         ComputeSchedulerMetric(b, ThzNtnSchedulerType::PROPORTIONAL_FAIR);
              });

    std::vector<ThzNtnSchedulingDecision> decisions;
    std::vector<bool> sbUsed(subBands.size(), false);

    for (const auto& ue : sortedUes)
    {
        ThzNtnSchedulingDecision dec;
        dec.ueId = ue.ueId;
        dec.rnti = ue.rnti;
        dec.waveformType = ThzNtnWaveformType::CP_OFDM;
        dec.numSymbols = 14;
        dec.harqProcessId = m_nextHarqProcessId++ % 8;

        if (m_satMobility && ue.mobility)
        {
            dec.scsHz = SelectScs(m_satMobility, ue.mobility);
        }
        else
        {
            double absDoppler = std::fabs(ue.dopplerHz);
            dec.scsHz = (absDoppler < 6000) ? SCS_60KHZ :
                        (absDoppler < 24000) ? SCS_240KHZ : SCS_960KHZ;
        }

        uint32_t bandsToAlloc = std::max(1u,
            static_cast<uint32_t>(subBands.size()) / static_cast<uint32_t>(ues.size()));

        uint32_t allocated = 0;
        for (uint32_t i = 0; i < subBands.size() && allocated < bandsToAlloc; ++i)
        {
            if (!sbUsed[i])
            {
                dec.subBandIndices.push_back(subBands[i].index);
                sbUsed[i] = true;
                ++allocated;
            }
        }

        dec.mcsIndex = SelectMcs(ue.sinr_dB);
        dec.tbSize_bytes = ComputeTbSize(dec.mcsIndex,
                                         static_cast<uint32_t>(dec.subBandIndices.size()),
                                         dec.numSymbols);
        UpdateUeThroughput(ue.ueId, ue.throughput_Mbps);
        decisions.push_back(dec);
    }

    return decisions;
}

// --------------------------------------------------------------------------
// Max Throughput
// --------------------------------------------------------------------------

std::vector<ThzNtnSchedulingDecision>
ThzNtnMacScheduler::ScheduleMaxThroughput(
    const std::vector<ThzNtnUeContext>& ues,
    const std::vector<ThzNtnSubBand>& subBands)
{
    NS_LOG_FUNCTION(this);

    auto sortedUes = ues;
    std::sort(sortedUes.begin(), sortedUes.end(),
              [](const ThzNtnUeContext& a, const ThzNtnUeContext& b) {
                  return a.sinr_dB > b.sinr_dB;
              });

    std::vector<ThzNtnSchedulingDecision> decisions;
    uint32_t sbIdx = 0;

    for (const auto& ue : sortedUes)
    {
        if (sbIdx >= subBands.size())
        {
            break;
        }

        ThzNtnSchedulingDecision dec;
        dec.ueId = ue.ueId;
        dec.rnti = ue.rnti;
        dec.waveformType = ThzNtnWaveformType::CP_OFDM;
        dec.numSymbols = 14;
        dec.harqProcessId = m_nextHarqProcessId++ % 8;

        if (m_satMobility && ue.mobility)
        {
            dec.scsHz = SelectScs(m_satMobility, ue.mobility);
        }
        else
        {
            double absDoppler = std::fabs(ue.dopplerHz);
            dec.scsHz = (absDoppler < 6000) ? SCS_60KHZ :
                        (absDoppler < 24000) ? SCS_240KHZ : SCS_960KHZ;
        }

        uint32_t bandsToAlloc = std::max(1u,
            static_cast<uint32_t>((subBands.size() - sbIdx) / 2));

        for (uint32_t i = 0; i < bandsToAlloc && sbIdx < subBands.size(); ++i, ++sbIdx)
        {
            dec.subBandIndices.push_back(subBands[sbIdx].index);
        }

        dec.mcsIndex = SelectMcs(ue.sinr_dB);
        dec.tbSize_bytes = ComputeTbSize(dec.mcsIndex,
                                         static_cast<uint32_t>(dec.subBandIndices.size()),
                                         dec.numSymbols);
        UpdateUeThroughput(ue.ueId, ue.throughput_Mbps);
        decisions.push_back(dec);
    }

    return decisions;
}

// --------------------------------------------------------------------------
// QoS-Aware
// --------------------------------------------------------------------------

std::vector<ThzNtnSchedulingDecision>
ThzNtnMacScheduler::ScheduleQosAware(
    const std::vector<ThzNtnUeContext>& ues,
    const std::vector<ThzNtnSubBand>& subBands)
{
    NS_LOG_FUNCTION(this);

    std::vector<ThzNtnSchedulingDecision> decisions;
    if (ues.empty() || subBands.empty())
    {
        return decisions;
    }

    // Sort sub-bands by absorption loss
    auto sortedBands = subBands;
    std::sort(sortedBands.begin(), sortedBands.end(),
              [](const ThzNtnSubBand& a, const ThzNtnSubBand& b) {
                  return a.absorptionLoss_dB < b.absorptionLoss_dB;
              });

    // Count UEs by class
    uint32_t nUrllc = 0, nEmbb = 0, nMmtc = 0;
    for (const auto& ue : ues)
    {
        if (ue.qosClass == ThzNtnQosClass::URLLC) ++nUrllc;
        else if (ue.qosClass == ThzNtnQosClass::EMBB) ++nEmbb;
        else ++nMmtc;
    }

    // Reserve sub-bands: URLLC gets 20% (best), eMBB gets 60%, mMTC gets 20%
    uint32_t total = static_cast<uint32_t>(sortedBands.size());
    uint32_t urllcCount = std::max(1u, total * 20 / 100);
    uint32_t mmtcCount = std::max(1u, total * 20 / 100);
    uint32_t embbCount = total - urllcCount - mmtcCount;

    if (nUrllc == 0) { embbCount += urllcCount; urllcCount = 0; }
    if (nMmtc == 0)  { embbCount += mmtcCount; mmtcCount = 0; }
    if (nEmbb == 0)  { urllcCount += embbCount / 2; mmtcCount += embbCount - embbCount / 2; embbCount = 0; }

    std::vector<ThzNtnSubBand> urllcPool, embbPool, mmtcPool;
    uint32_t idx = 0;
    for (; idx < urllcCount && idx < total; ++idx)
    {
        urllcPool.push_back(sortedBands[idx]);
    }
    for (; idx < urllcCount + embbCount && idx < total; ++idx)
    {
        embbPool.push_back(sortedBands[idx]);
    }
    for (; idx < total; ++idx)
    {
        mmtcPool.push_back(sortedBands[idx]);
    }

    // Sort UEs by QoS priority
    auto sortedUes = ues;
    std::sort(sortedUes.begin(), sortedUes.end(),
              [this](const ThzNtnUeContext& a, const ThzNtnUeContext& b) {
                  return ComputeSchedulerMetric(a, ThzNtnSchedulerType::QOS_AWARE) >
                         ComputeSchedulerMetric(b, ThzNtnSchedulerType::QOS_AWARE);
              });

    [[maybe_unused]] uint32_t urllcIdx = 0, embbIdx = 0, mmtcIdx = 0;

    for (const auto& ue : sortedUes)
    {
        ThzNtnSchedulingDecision dec;
        dec.ueId = ue.ueId;
        dec.rnti = ue.rnti;
        dec.harqProcessId = m_nextHarqProcessId++ % 8;

        if (m_satMobility && ue.mobility)
        {
            dec.scsHz = SelectScs(m_satMobility, ue.mobility);
        }
        else
        {
            double absDoppler = std::fabs(ue.dopplerHz);
            dec.scsHz = (absDoppler < 6000) ? SCS_60KHZ :
                        (absDoppler < 24000) ? SCS_240KHZ : SCS_960KHZ;
        }

        if (ue.qosClass == ThzNtnQosClass::URLLC)
        {
            dec.waveformType = ThzNtnWaveformType::DFT_S_OFDM;
            dec.numSymbols = 2; // Mini-slot
            uint32_t maxBands = std::min(2u, static_cast<uint32_t>(urllcPool.size()));
            for (uint32_t i = 0; i < maxBands; ++i)
            {
                dec.subBandIndices.push_back(urllcPool[i].index);
            }
            // Conservative MCS with 3 dB margin
            dec.mcsIndex = SelectMcs(ue.sinr_dB - 3.0);
        }
        else if (ue.qosClass == ThzNtnQosClass::EMBB)
        {
            dec.waveformType = ThzNtnWaveformType::CP_OFDM;
            dec.numSymbols = 14;
            uint32_t share = (nEmbb > 0) ? std::max(1u,
                static_cast<uint32_t>(embbPool.size()) / nEmbb) : 0;
            for (uint32_t i = 0; i < share && embbIdx < embbPool.size(); ++i, ++embbIdx)
            {
                dec.subBandIndices.push_back(embbPool[embbIdx].index);
            }
            dec.mcsIndex = SelectMcs(ue.sinr_dB);
        }
        else
        {
            dec.waveformType = ThzNtnWaveformType::SC_FDMA;
            dec.numSymbols = 14;
            if (mmtcIdx < mmtcPool.size())
            {
                dec.subBandIndices.push_back(mmtcPool[mmtcIdx].index);
                ++mmtcIdx;
            }
            dec.mcsIndex = SelectMcs(ue.sinr_dB);
        }

        dec.tbSize_bytes = ComputeTbSize(dec.mcsIndex,
                                         static_cast<uint32_t>(dec.subBandIndices.size()),
                                         dec.numSymbols);
        UpdateUeThroughput(ue.ueId, ue.throughput_Mbps);
        decisions.push_back(dec);
    }

    NS_LOG_INFO("QoS-aware scheduling: " << decisions.size() << " decisions"
                << " (URLLC=" << nUrllc << ", eMBB=" << nEmbb << ", mMTC=" << nMmtc << ")");
    return decisions;
}

} // namespace ns3
