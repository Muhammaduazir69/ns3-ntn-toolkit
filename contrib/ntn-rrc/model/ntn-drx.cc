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
}

DrxState
NtnDrxStateMachine::GetState() const
{
    return m_state;
}

bool
NtnDrxStateMachine::IsAwake() const
{
    return m_state == DrxState::Active || m_state == DrxState::OnDuration;
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

    if (m_cfg.shortCycle > Time(0))
    {
        m_shortCycleRemaining = m_cfg.shortCycleCount;
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
