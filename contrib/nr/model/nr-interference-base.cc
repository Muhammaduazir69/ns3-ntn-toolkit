// Copyright (c) 2009 CTTC
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Nicola Baldo <nbaldo@cttc.es>

#include "nr-interference-base.h"

#include "nr-chunk-processor.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrInterferenceBase");

NrInterferenceBase::NrInterferenceBase()
    : m_receiving(false),
      m_lastSignalId(0),
      m_lastSignalIdBeforeReset(0)
{
    NS_LOG_FUNCTION(this);
}

NrInterferenceBase::~NrInterferenceBase()
{
    NS_LOG_FUNCTION(this);
}

void
NrInterferenceBase::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_rsPowerChunkProcessorList.clear();
    m_sinrChunkProcessorList.clear();
    m_interfChunkProcessorList.clear();
    m_rxSignal = nullptr;
    m_allSignals = nullptr;
    m_allSignalsList.clear();
    m_noise = nullptr;
    Object::DoDispose();
}

TypeId
NrInterferenceBase::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrInterferenceBase").SetParent<Object>().SetGroupName("Nr");
    return tid;
}

bool
NrInterferenceBase::StartRx(Ptr<const SpectrumValue> rxPsd)
{
    NS_LOG_FUNCTION(this << *rxPsd);
    if (!m_receiving)
    {
        NS_LOG_LOGIC("first signal");
        m_rxSignal = rxPsd->Copy();
        m_lastChangeTime = Now();
        m_receiving = true;
        for (auto it = m_rsPowerChunkProcessorList.begin(); it != m_rsPowerChunkProcessorList.end();
             ++it)
        {
            (*it)->Start();
        }
        for (auto it = m_interfChunkProcessorList.begin(); it != m_interfChunkProcessorList.end();
             ++it)
        {
            (*it)->Start();
        }
        for (auto it = m_sinrChunkProcessorList.begin(); it != m_sinrChunkProcessorList.end(); ++it)
        {
            (*it)->Start();
        }
    }
    else
    {
        NS_LOG_LOGIC("additional signal" << *m_rxSignal);
        // receiving multiple simultaneous signals, make sure they are synchronized
        NS_ASSERT(m_lastChangeTime == Now());
        // make sure they use orthogonal resource blocks
        if (Sum((*rxPsd) * (*m_rxSignal)) != 0.0)
        {
            // Keep the first signal, reject this overlapping one (it stays in the
            // all-signals set, so StartRxMimo counts it as interference). Safe to
            // pick by arrival order: NrSpectrumPhy::StartRx gates the receive path
            // by cellId and (for a UE) RNTI, so foreign interferers never reach
            // here. Colliding candidates already belong to this receiver (an
            // anomaly: same-cell grant collision or stale handover RX); the
            // rejected TB just fails to decode and HARQ recovers it.
            NS_LOG_LOGIC("Overlapping PSD detected; treating the new signal as interference");
            return false;
        }
        (*m_rxSignal) += (*rxPsd);
    }
    return true;
}

void
NrInterferenceBase::EndRx()
{
    NS_LOG_FUNCTION(this);
    if (!m_receiving)
    {
        NS_LOG_INFO("EndRx was already evaluated or RX was aborted");
    }
    else
    {
        ConditionallyEvaluateChunk();
        m_receiving = false;
        for (auto it = m_rsPowerChunkProcessorList.begin(); it != m_rsPowerChunkProcessorList.end();
             ++it)
        {
            (*it)->End();
        }
        for (auto it = m_interfChunkProcessorList.begin(); it != m_interfChunkProcessorList.end();
             ++it)
        {
            (*it)->End();
        }
        for (auto it = m_sinrChunkProcessorList.begin(); it != m_sinrChunkProcessorList.end(); ++it)
        {
            (*it)->End();
        }
    }
}

void
NrInterferenceBase::AddSignal(Ptr<const SpectrumValue> spd, const Time duration)
{
    NS_LOG_FUNCTION(this << *spd << duration);
    DoAddSignal(spd);
    uint32_t signalId = ++m_lastSignalId;
    if (signalId == m_lastSignalIdBeforeReset)
    {
        // This happens when m_lastSignalId eventually wraps around. Given that so
        // many signals have elapsed since the last reset, we hope that by now there is
        // no stale pending signal (i.e., a signal that was scheduled
        // for subtraction before the reset). So we just move the
        // boundary further.
        m_lastSignalIdBeforeReset += 0x10000000;
    }
    Simulator::Schedule(duration, &NrInterferenceBase::DoSubtractSignal, this, spd, signalId);
}

void
NrInterferenceBase::DoAddSignal(Ptr<const SpectrumValue> spd)
{
    NS_LOG_FUNCTION(this << *spd);
    ConditionallyEvaluateChunk();
    (*m_allSignals) += (*spd);
    m_allSignalsList.push_back(spd);
}

void
NrInterferenceBase::DoSubtractSignal(Ptr<const SpectrumValue> spd, uint32_t signalId)
{
    NS_LOG_FUNCTION(this << *spd);
    ConditionallyEvaluateChunk();
    auto it = std::find(m_allSignalsList.begin(), m_allSignalsList.end(), spd);
    if (it != m_allSignalsList.end())
    {
        m_allSignalsList.erase(it);
    }
    int32_t deltaSignalId = signalId - m_lastSignalIdBeforeReset;
    if (deltaSignalId > 0)
    {
        (*m_allSignals) -= (*spd);
        // The paired addition may have absorbed smaller co-resident signals
        // (leaving a negative residue here), and subtracting an infinite
        // signal leaves NaN. Heal the sum so the corruption cannot outlive
        // the expired signal; a finite but silently wrong sum is corrected
        // by the constructive rebuild in ConditionallyEvaluateChunk.
        for (auto v = m_allSignals->ConstValuesBegin(); v != m_allSignals->ConstValuesEnd(); ++v)
        {
            if (!std::isfinite(*v) || *v < 0.0)
            {
                RebuildAllSignals();
                break;
            }
        }
    }
    else
    {
        NS_LOG_INFO("ignoring signal scheduled for subtraction before last reset");
    }
}

void
NrInterferenceBase::RebuildAllSignals()
{
    NS_LOG_FUNCTION(this);
    m_allSignals = Create<SpectrumValue>(m_noise->GetSpectrumModel());
    for (const auto& spd : m_allSignalsList)
    {
        (*m_allSignals) += (*spd);
    }
}

void
NrInterferenceBase::ConditionallyEvaluateChunk()
{
    NS_LOG_FUNCTION(this);
    if (m_receiving)
    {
        NS_LOG_DEBUG(this << " Receiving");
    }
    NS_LOG_DEBUG(this << " now " << Now() << " last " << m_lastChangeTime);
    if (m_receiving && (Now() > m_lastChangeTime))
    {
        // Rebuild the sum of all signals from the live-signal list instead of
        // trusting the running sum: a saturating signal absorbs co-resident
        // smaller signals when added (S + s rounds to S), so the paired
        // subtractions leave a permanent negative bias, and an infinite
        // signal leaves NaN. The MIMO covariance path already rebuilds per
        // chunk from its own list for the same reason.
        RebuildAllSignals();
        NS_LOG_LOGIC(this << " signal = " << *m_rxSignal << " allSignals = " << *m_allSignals
                          << " noise = " << *m_noise);

        SpectrumValue interf = (*m_allSignals) - (*m_rxSignal) + (*m_noise);

        SpectrumValue sinr = (*m_rxSignal) / interf;
        Time duration = Now() - m_lastChangeTime;
        for (auto it = m_sinrChunkProcessorList.begin(); it != m_sinrChunkProcessorList.end(); ++it)
        {
            (*it)->EvaluateChunk(sinr, duration);
        }
        for (auto it = m_interfChunkProcessorList.begin(); it != m_interfChunkProcessorList.end();
             ++it)
        {
            (*it)->EvaluateChunk(interf, duration);
        }
        for (auto it = m_rsPowerChunkProcessorList.begin(); it != m_rsPowerChunkProcessorList.end();
             ++it)
        {
            (*it)->EvaluateChunk(*m_rxSignal, duration);
        }
        m_lastChangeTime = Now();
    }
}

void
NrInterferenceBase::SetNoisePowerSpectralDensity(Ptr<const SpectrumValue> noisePsd)
{
    NS_LOG_FUNCTION(this << *noisePsd);
    ConditionallyEvaluateChunk();
    m_noise = noisePsd;
    // reset m_allSignals (will reset if already set previously)
    // this is needed since this method can potentially change the SpectrumModel
    m_allSignals = Create<SpectrumValue>(noisePsd->GetSpectrumModel());
    m_allSignalsList.clear();
    if (m_receiving)
    {
        // abort rx
        m_receiving = false;
    }
    // record the last SignalId so that we can ignore all signals that
    // were scheduled for subtraction before m_allSignal
    m_lastSignalIdBeforeReset = m_lastSignalId;
}

void
NrInterferenceBase::AddRsPowerChunkProcessor(Ptr<NrChunkProcessor> p)
{
    NS_LOG_FUNCTION(this << p);
    m_rsPowerChunkProcessorList.push_back(p);
}

void
NrInterferenceBase::AddSinrChunkProcessor(Ptr<NrChunkProcessor> p)
{
    NS_LOG_FUNCTION(this << p);
    m_sinrChunkProcessorList.push_back(p);
}

void
NrInterferenceBase::AddInterferenceChunkProcessor(Ptr<NrChunkProcessor> p)
{
    NS_LOG_FUNCTION(this << p);
    m_interfChunkProcessorList.push_back(p);
}

} // namespace ns3
