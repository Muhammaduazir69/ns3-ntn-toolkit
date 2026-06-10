/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
#include "ntn-platform-spec.h"

#include <ns3/abort.h>
#include <ns3/log.h>
#include <ns3/simulator.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnPlatformSpec");
NS_OBJECT_ENSURE_REGISTERED(OranNtnRoleSwitch);

const std::vector<NtnPlatformSpec::Spec>&
NtnPlatformSpec::GetAll()
{
    // Deng 2026 Table 3 (rounded class values). Endurance 0 = mission-life.
    static const std::vector<Spec> specs = {
        {Class::UavUntethered, "UAV (untethered)", 5.0, 0.5, Minutes(30),
         MilliSeconds(1), "RAN/relay"},
        {Class::UavTethered, "UAV (tethered)", 10.0, 2.0, Seconds(0),
         MilliSeconds(1), "RAN/relay"},
        {Class::Hap, "HAPS", 50.0, 5.0, Days(100), MilliSeconds(1), "RAN/edge"},
        {Class::Leo, "LEO", 500.0, 10.0, Seconds(0), MilliSeconds(40), "RAN/transport"},
        {Class::Meo, "MEO", 1000.0, 15.0, Seconds(0), MilliSeconds(150), "transport"},
        {Class::Geo, "GEO", 3000.0, 20.0, Seconds(0), MilliSeconds(600),
         "transport/broadcast"},
    };
    return specs;
}

const NtnPlatformSpec::Spec&
NtnPlatformSpec::Get(Class c)
{
    for (const auto& s : GetAll())
    {
        if (s.platformClass == c)
        {
            return s;
        }
    }
    NS_ABORT_MSG("unknown platform class");
}

void
NtnPlatformSpec::ScheduleEnduranceEnd(Class c, std::function<void()> onEnd)
{
    const Spec& s = Get(c);
    if (s.endurance.IsZero())
    {
        return; // mission-life platform
    }
    Simulator::Schedule(s.endurance, [onEnd, s] {
        NS_LOG_INFO(s.name << " endurance (" << s.endurance.GetMinutes()
                           << " min) exhausted");
        onEnd();
    });
}

// ----------------------------------------------------------- role switch

TypeId
OranNtnRoleSwitch::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnRoleSwitch")
            .SetParent<Object>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnRoleSwitch>()
            .AddTraceSource("RoleSwitch",
                            "A platform changed role (platform, from, to).",
                            MakeTraceSourceAccessor(&OranNtnRoleSwitch::m_switchTrace),
                            "ns3::OranNtnRoleSwitch::SwitchTracedCallback");
    return tid;
}

void
OranNtnRoleSwitch::RegisterPlatform(std::string name, Role initial, RoleApplier applier)
{
    PlatformState st;
    st.role = initial;
    st.applier = std::move(applier);
    m_platforms[std::move(name)] = std::move(st);
}

OranNtnRoleSwitch::Role
OranNtnRoleSwitch::GetRole(const std::string& platform) const
{
    auto it = m_platforms.find(platform);
    NS_ABORT_MSG_IF(it == m_platforms.end(), "unknown platform " << platform);
    return it->second.role;
}

void
OranNtnRoleSwitch::SetBatterySource(std::string platform, std::function<double()> fraction)
{
    m_platforms.at(platform).battery = std::move(fraction);
}

void
OranNtnRoleSwitch::SetFhLatencySource(std::string platform, std::function<Time()> measured)
{
    m_platforms.at(platform).fhLatency = std::move(measured);
}

void
OranNtnRoleSwitch::InjectFailure(std::string platform, std::string reason)
{
    // A failed platform falls back to bare O-RU duty; siblings get checked
    // for elevation on the next SMO pass.
    auto it = m_platforms.find(platform);
    NS_ABORT_MSG_IF(it == m_platforms.end(), "unknown platform " << platform);
    if (it->second.role != Role::Ru)
    {
        TriggerSwitch(platform, Role::Ru, "failure:" + reason);
    }
}

void
OranNtnRoleSwitch::Start()
{
    if (m_running)
    {
        return;
    }
    m_running = true;
    Simulator::Schedule(m_period, &OranNtnRoleSwitch::Check, this);
}

void
OranNtnRoleSwitch::Check()
{
    if (!m_running)
    {
        return;
    }
    for (auto& [name, st] : m_platforms)
    {
        if (st.switching)
        {
            continue;
        }
        // Insufficient power: offload functions (lighter role) — measured
        // battery from the C&C telemetry path.
        if (st.battery && st.battery() < m_batteryThreshold && st.role != Role::Ru)
        {
            TriggerSwitch(name, Role::Ru, "battery");
            continue;
        }
        // Excessive MEASURED fronthaul latency: elevate so the latency-
        // critical functions move onto the platform.
        if (st.fhLatency && st.fhLatency() > m_fhBound && st.role == Role::Ru)
        {
            TriggerSwitch(name, Role::RuDu, "fh-latency");
        }
    }
    Simulator::Schedule(m_period, &OranNtnRoleSwitch::Check, this);
}

void
OranNtnRoleSwitch::TriggerSwitch(const std::string& platform,
                                 Role newRole,
                                 const std::string& trigger,
                                 std::function<void()> onInterruptStart,
                                 std::function<void()> onInterruptEnd)
{
    PlatformState& st = m_platforms.at(platform);
    if (st.role == newRole || st.switching)
    {
        return;
    }
    st.switching = true;
    SwitchEvent ev;
    ev.time = Simulator::Now();
    ev.platform = platform;
    ev.from = st.role;
    ev.to = newRole;
    ev.trigger = trigger;
    ev.interruption = m_interruption;
    m_events.push_back(ev);
    NS_LOG_INFO("role switch " << platform << " " << static_cast<int>(st.role) << "->"
                               << static_cast<int>(newRole) << " (" << trigger << ")");
    if (onInterruptStart)
    {
        onInterruptStart();
    }
    // The new role goes live after the real service-interruption window.
    Simulator::Schedule(m_interruption, [this, platform, newRole, onInterruptEnd] {
        PlatformState& s = m_platforms.at(platform);
        s.role = newRole;
        s.switching = false;
        if (s.applier)
        {
            s.applier(newRole);
        }
        if (onInterruptEnd)
        {
            onInterruptEnd();
        }
        m_switchTrace(platform, static_cast<uint8_t>(s.role),
                      static_cast<uint8_t>(newRole));
    });
}

} // namespace ns3
