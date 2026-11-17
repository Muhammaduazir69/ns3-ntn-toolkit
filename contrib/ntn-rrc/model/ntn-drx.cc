/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-drx.h"

#include <ns3/log.h>
#include <ns3/simulator.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnDrx");

namespace ntnrrc
{

bool
NtnDrxConfig::IsValid() const
{
    if (onDuration <= Time(0))
        return false;
    if (longCycle < shortCycle)
        return false;
    if (shortCycle > Time(0) && shortCycle < onDuration)
        return false;
    return true;
}

NS_OBJECT_ENSURE_REGISTERED(NtnDrxStateMachine);

TypeId
NtnDrxStateMachine::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ntnrrc::NtnDrxStateMachine")
            .SetParent<Object>()
            .SetGroupName("NtnRrc")
            .AddConstructor<NtnDrxStateMachine>()
            .AddTraceSource(
                "StateChange",
                "(prevState, newState, now) on every state transition",
                MakeTraceSourceAccessor(&NtnDrxStateMachine::m_stateChangeTrace),
                "ns3::ntnrrc::NtnDrxStateMachine::StateChangeTracedCallback");
    return tid;
}

NtnDrxStateMachine::NtnDrxStateMachine() = default;
NtnDrxStateMachine::~NtnDrxStateMachine() = default;

void
NtnDrxStateMachine::DoDispose()
{
    Stop();
    Object::DoDispose();
}

void
NtnDrxStateMachine::SetConfig(const NtnDrxConfig& cfg)
{
    NS_ASSERT_MSG(cfg.IsValid(), "Invalid DRX config (cycle ordering or zero onDuration)");
    m_cfg = cfg;
}

const NtnDrxConfig&
NtnDrxStateMachine::GetConfig() const
{
    return m_cfg;
}

void
NtnDrxStateMachine::NotifyDataActivity()
{
    if (!m_running)
    {
        return;
    }
    m_inactivityDeadline = Simulator::Now() + m_cfg.inactivityTimer;
    if (m_state != DrxState::Active)
    {
        Transition(DrxState::Active);
        if (m_event.IsPending())
        {
            Simulator::Cancel(m_event);
        }
        m_event = Simulator::Schedule(m_cfg.inactivityTimer,
                                      &NtnDrxStateMachine::TickActive, this);
    }
}

void
NtnDrxStateMachine::NotifyNextPass(Time start, Time duration)
{
    NS_ASSERT_MSG(duration > Time(0), "Pass duration must be positive");
    m_nextPassStart = start;
    m_nextPassEnd = start + duration;
}

void
NtnDrxStateMachine::SetNtnRoundTripTime(Time rtt)
{
    NS_ASSERT_MSG(rtt >= Time(0), "NTN RTT must be non-negative");
    m_cfg.ntnRtt = rtt;
}

Time
NtnDrxStateMachine::GetHarqRttTimerDlNtn() const
{
    // TS 38.321 §5.7: HARQ-RTT-TimerDL-NTN = drx-HARQ-RTT-TimerDL + UE-gNB RTT.
    return m_cfg.harqRttTimerDl + m_cfg.ntnRtt;
}

Time
NtnDrxStateMachine::GetHarqRttTimerUlNtn() const
{
    // TS 38.321 §5.7: HARQ-RTT-TimerUL-NTN = drx-HARQ-RTT-TimerUL + UE-gNB RTT.
    return m_cfg.harqRttTimerUl + m_cfg.ntnRtt;
}

void
NtnDrxStateMachine::Start()
{
    if (m_running)
    {
        return;
    }
    NS_ASSERT_MSG(m_cfg.IsValid(), "Cannot Start() with an invalid DRX config");
    m_running = true;
    m_state = DrxState::Active;
    m_stateEnteredAt = Simulator::Now();
    m_inactivityDeadline = Simulator::Now() + m_cfg.inactivityTimer;
    m_event = Simulator::Schedule(m_cfg.inactivityTimer,
                                  &NtnDrxStateMachine::TickActive, this);
}

void
NtnDrxStateMachine::Stop()
{
    if (!m_running)
    {
        return;
    }
    AccumulateTimeInState();
    m_running = false;
    if (m_event.IsPending())
    {
        Simulator::Cancel(m_event);
    }
    for (uint8_t i = 0; i < kMaxHarq; ++i)
    {
        m_dlHarqRttEvent[i].Cancel();
        m_ulHarqRttEvent[i].Cancel();
    }
}

DrxState
NtnDrxStateMachine::GetState() const
{
    return m_state;
}

bool
NtnDrxStateMachine::IsAwake() const
{
    if (m_state == DrxState::Active || m_state == DrxState::OnDuration)
    {
        return true;
    }
    if (!m_running)
    {
        return false;
    }
    const Time now = Simulator::Now();
    for (uint8_t i = 0; i < kMaxHarq; ++i)
    {
        if (m_dlRetxEnd[i] > now || m_ulRetxEnd[i] > now)
        {
            return true;
        }
    }
    return IsSrActive();
}

Time
NtnDrxStateMachine::GetHarqRttTimerDlFor(uint8_t harqId) const
{
    const bool disabled = harqId < kMaxHarq && ((m_cfg.dlHarqFeedbackDisabled >> harqId) & 1U);
    return disabled ? m_cfg.harqRttTimerDl : GetHarqRttTimerDlNtn();
}

Time
NtnDrxStateMachine::GetHarqRttTimerUlFor(uint8_t harqId) const
{
    const bool modeB = harqId < kMaxHarq && ((m_cfg.ulHarqModeB >> harqId) & 1U);
    return modeB ? m_cfg.harqRttTimerUl : GetHarqRttTimerUlNtn();
}

void
NtnDrxStateMachine::NotifyDlTransmission(uint8_t harqId, bool decoded, Time feedbackDelay)
{
    NS_ASSERT_MSG(harqId < kMaxHarq, "DL HARQ process id " << +harqId << " beyond n32");
    const Time now = Simulator::Now();
    // TS 38.321 5.7: a DL assignment for the process stops its
    // drx-RetransmissionTimerDL.
    m_dlRetxEnd[harqId] = now;
    m_dlHarqRttEvent[harqId].Cancel();
    const bool disabled = (m_cfg.dlHarqFeedbackDisabled >> harqId) & 1U;
    // Feedback enabled: the timer starts after the PUCCH carrying the ACK/NACK
    // and lasts drx-HARQ-RTT-TimerDL + UE-gNB RTT (HARQ-RTT-TimerDL-NTN).
    // Feedback disabled: nothing is fed back, so there is no RTT to wait for;
    // the bare drx-HARQ-RTT-TimerDL starts at the end of the PDSCH.
    const Time start = disabled ? Time(0) : feedbackDelay;
    const Time dur = GetHarqRttTimerDlFor(harqId);
    m_dlHarqRttEnd[harqId] = now + start + dur;
    m_dlHarqRttEvent[harqId] = Simulator::Schedule(start + dur,
                                                   &NtnDrxStateMachine::DlHarqRttExpired,
                                                   this,
                                                   harqId,
                                                   decoded);
}

void
NtnDrxStateMachine::DlHarqRttExpired(uint8_t harqId, bool decoded)
{
    // TS 38.321 5.7: on drx-HARQ-RTT-TimerDL expiry, if the data of the
    // process was not successfully decoded, start drx-RetransmissionTimerDL.
    if (!decoded && m_cfg.retransmissionTimerDl.IsStrictlyPositive())
    {
        m_dlRetxEnd[harqId] = Simulator::Now() + m_cfg.retransmissionTimerDl;
    }
}

void
NtnDrxStateMachine::NotifyUlTransmission(uint8_t harqId)
{
    NS_ASSERT_MSG(harqId < kMaxHarq, "UL HARQ process id " << +harqId << " beyond n32");
    const Time now = Simulator::Now();
    m_ulRetxEnd[harqId] = now;
    m_ulHarqRttEvent[harqId].Cancel();
    const Time dur = GetHarqRttTimerUlFor(harqId);
    m_ulHarqRttEnd[harqId] = now + dur;
    m_ulHarqRttEvent[harqId] =
        Simulator::Schedule(dur, &NtnDrxStateMachine::UlHarqRttExpired, this, harqId);
}

void
NtnDrxStateMachine::UlHarqRttExpired(uint8_t harqId)
{
    // TS 38.321 5.7: on drx-HARQ-RTT-TimerUL expiry start
    // drx-RetransmissionTimerUL.
    if (m_cfg.retransmissionTimerUl.IsStrictlyPositive())
    {
        m_ulRetxEnd[harqId] = Simulator::Now() + m_cfg.retransmissionTimerUl;
    }
}

void
NtnDrxStateMachine::NotifySrSent()
{
    m_srPending = true;
    m_srSentAt = Simulator::Now();
}

void
NtnDrxStateMachine::NotifyUlGrant()
{
    m_srPending = false;
    NotifyDataActivity();
}

bool
NtnDrxStateMachine::IsSrActive() const
{
    // Rel-17: Active Time for a pending SR begins one UE-gNB RTT after it.
    return m_srPending && Simulator::Now() >= m_srSentAt + m_cfg.ntnRtt;
}

bool
NtnDrxStateMachine::IsHarqRttTimerDlRunning(uint8_t harqId) const
{
    return harqId < kMaxHarq && m_dlHarqRttEvent[harqId].IsPending();
}

bool
NtnDrxStateMachine::IsRetransmissionTimerDlRunning(uint8_t harqId) const
{
    return harqId < kMaxHarq && m_dlRetxEnd[harqId] > Simulator::Now();
}

bool
NtnDrxStateMachine::IsHarqRttTimerUlRunning(uint8_t harqId) const
{
    return harqId < kMaxHarq && m_ulHarqRttEvent[harqId].IsPending();
}

bool
NtnDrxStateMachine::IsRetransmissionTimerUlRunning(uint8_t harqId) const
{
    return harqId < kMaxHarq && m_ulRetxEnd[harqId] > Simulator::Now();
}

Time
NtnDrxStateMachine::GetTimeInState(DrxState s) const
{
    return m_timeInState[static_cast<size_t>(s)];
}

void
NtnDrxStateMachine::AccumulateTimeInState()
{
    const Time now = Simulator::Now();
    const Time delta = now - m_stateEnteredAt;
    if (delta > Time(0))
    {
        m_timeInState[static_cast<size_t>(m_state)] += delta;
    }
    m_stateEnteredAt = now;
}

void
NtnDrxStateMachine::Transition(DrxState next)
{
    if (m_state == next)
    {
        return;
    }
    AccumulateTimeInState();
    const DrxState prev = m_state;
    m_state = next;
    m_stateChangeTrace(prev, next, Simulator::Now());
}

// Active → expires inactivity timer → enter sleep cycle.
void
NtnDrxStateMachine::TickActive()
{
    if (!m_running)
    {
        return;
    }
    if (Simulator::Now() < m_inactivityDeadline)
    {
        // Activity restarted us inside the deadline.
        m_event = Simulator::Schedule(m_inactivityDeadline - Simulator::Now(),
                                      &NtnDrxStateMachine::TickActive, this);
        return;
    }

    if (m_cfg.passAware && m_nextPassStart > Simulator::Now() &&
        m_nextPassStart - Simulator::Now() > m_cfg.longCycle)
    {
        Transition(DrxState::AwaitingPass);
        m_event = Simulator::Schedule(m_nextPassStart - Simulator::Now(),
                                      &NtnDrxStateMachine::TickAwaitingPass, this);
        return;
    }

    if (m_cfg.shortCycle > Time(0) && m_cfg.shortCycleCount > 0)
    {
        // RRC-9: TS 38.321 section 5.7 runs drx-ShortCycleTimer for
        // shortCycleCount x drx-ShortCycle from the inactivity expiry, and the
        // cycle entered here is the first of those. Loading the full count and
        // decrementing only at the end of each on-duration gave one short cycle
        // too many (three for a timer of two) before the long cycle.
        m_shortCycleRemaining = m_cfg.shortCycleCount - 1;
        Transition(DrxState::ShortSleep);
        m_event = Simulator::Schedule(m_cfg.shortCycle - m_cfg.onDuration,
                                      &NtnDrxStateMachine::TickShortSleep, this);
    }
    else
    {
        Transition(DrxState::LongSleep);
        m_event = Simulator::Schedule(m_cfg.longCycle - m_cfg.onDuration,
                                      &NtnDrxStateMachine::TickLongSleep, this);
    }
}

void
NtnDrxStateMachine::TickOnDuration()
{
    if (!m_running)
    {
        return;
    }
    // OnDuration window expired without activity → back to sleep.
    if (m_cfg.shortCycle > Time(0) && m_shortCycleRemaining > 0)
    {
        --m_shortCycleRemaining;
        Transition(DrxState::ShortSleep);
        m_event = Simulator::Schedule(m_cfg.shortCycle - m_cfg.onDuration,
                                      &NtnDrxStateMachine::TickShortSleep, this);
    }
    else
    {
        Transition(DrxState::LongSleep);
        m_event = Simulator::Schedule(m_cfg.longCycle - m_cfg.onDuration,
                                      &NtnDrxStateMachine::TickLongSleep, this);
    }
}

void
NtnDrxStateMachine::TickShortSleep()
{
    if (!m_running)
    {
        return;
    }
    Transition(DrxState::OnDuration);
    m_event = Simulator::Schedule(m_cfg.onDuration,
                                  &NtnDrxStateMachine::TickOnDuration, this);
}

void
NtnDrxStateMachine::TickLongSleep()
{
    if (!m_running)
    {
        return;
    }
    Transition(DrxState::OnDuration);
    m_event = Simulator::Schedule(m_cfg.onDuration,
                                  &NtnDrxStateMachine::TickOnDuration, this);
}

void
NtnDrxStateMachine::TickAwaitingPass()
{
    if (!m_running)
    {
        return;
    }
    // Pass started — wake into Active.
    Transition(DrxState::Active);
    m_inactivityDeadline = Simulator::Now() + m_cfg.inactivityTimer;
    m_event = Simulator::Schedule(m_cfg.inactivityTimer,
                                  &NtnDrxStateMachine::TickActive, this);
}

} // namespace ntnrrc
} // namespace ns3
