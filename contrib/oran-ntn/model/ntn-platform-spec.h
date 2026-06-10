/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// NtnPlatformSpec — the paper's Table 3 platform characteristics as code
// (Deng 2026; adoption plan WS4), plus OranNtnRoleSwitch — the flexible
// functional role switch (enabler 3).
//
// NtnPlatformSpec gives every NTN platform class its REAL constraints:
// payload mass, power, endurance, one-way latency class, network role. The
// endurance is enforceable: ScheduleEnduranceEnd() ends an untethered UAV's
// service after 30 minutes — a real constraint with a real consequence, not
// a footnote.
//
// OranNtnRoleSwitch is the SMO-side coordinator: it watches MEASURED
// conditions (battery fraction from C&C telemetry, measured FH latency vs
// the split bound, anomaly events, injected failures) and elevates/offloads
// O-RAN roles between platforms with a real service-interruption window —
// the paper's HAP O-RU -> O-DU elevation.

#ifndef NTN_PLATFORM_SPEC_H
#define NTN_PLATFORM_SPEC_H

#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/traced-callback.h>

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace ns3
{

class NtnPlatformSpec
{
  public:
    enum class Class : uint8_t
    {
        UavUntethered = 0,
        UavTethered = 1,
        Hap = 2,
        Leo = 3,
        Meo = 4,
        Geo = 5,
    };

    struct Spec
    {
        Class platformClass;
        std::string name;
        double payloadKg;       ///< deployable payload mass
        double powerKw;         ///< available bus power
        Time endurance;         ///< on-station endurance (0 = mission-life)
        Time userPlaneLatency;  ///< Table 3 one-way latency class
        std::string networkRole;
    };

    static const Spec& Get(Class c);
    static const std::vector<Spec>& GetAll();

    /**
     * \brief Enforce the platform's endurance: \p onEnd fires when it runs
     *        out (e.g. stop the apps, trigger a role offload). No event is
     *        scheduled for mission-life platforms (endurance 0).
     */
    static void ScheduleEnduranceEnd(Class c, std::function<void()> onEnd);
};

class OranNtnRoleSwitch : public Object
{
  public:
    /// O-RAN functional roles a platform can host (paper Fig. 8).
    enum class Role : uint8_t
    {
        Ru = 0,
        RuDu = 1,
        FullGnb = 2,
    };

    struct SwitchEvent
    {
        Time time;
        std::string platform;
        Role from;
        Role to;
        std::string trigger;
        Time interruption;
    };

    /// Applies the new role to the live scenario (re-route, re-delay, ...).
    using RoleApplier = std::function<void(Role newRole)>;

    static TypeId GetTypeId();
    OranNtnRoleSwitch() = default;

    void RegisterPlatform(std::string name, Role initial, RoleApplier applier);
    Role GetRole(const std::string& platform) const;

    /// Condition sources (all MEASURED): polled every CheckPeriod.
    void SetBatterySource(std::string platform, std::function<double()> fraction);
    void SetFhLatencySource(std::string platform, std::function<Time()> measured);
    void SetBatteryThreshold(double f) { m_batteryThreshold = f; }
    void SetFhLatencyBound(Time t) { m_fhBound = t; }
    void SetCheckPeriod(Time t) { m_period = t; }
    void SetInterruptionWindow(Time t) { m_interruption = t; }
    /// Failure injection (e.g. O-DU loss on another platform).
    void InjectFailure(std::string platform, std::string reason);

    void Start();

    /**
     * \brief Execute a role switch NOW: the applier runs after the service-
     *        interruption window (during which \p onInterruptStart/-End let
     *        the caller pause/resume real traffic).
     */
    void TriggerSwitch(const std::string& platform,
                       Role newRole,
                       const std::string& trigger,
                       std::function<void()> onInterruptStart = nullptr,
                       std::function<void()> onInterruptEnd = nullptr);

    const std::vector<SwitchEvent>& GetEvents() const { return m_events; }

  private:
    void Check();

    struct PlatformState
    {
        Role role{Role::Ru};
        RoleApplier applier;
        std::function<double()> battery;
        std::function<Time()> fhLatency;
        bool switching{false};
    };

    std::map<std::string, PlatformState> m_platforms;
    double m_batteryThreshold{0.2};
    Time m_fhBound{MilliSeconds(10)};
    Time m_period{Seconds(1.0)};
    Time m_interruption{MilliSeconds(500)};
    bool m_running{false};
    std::vector<SwitchEvent> m_events;

    TracedCallback<std::string, uint8_t, uint8_t> m_switchTrace;
};

} // namespace ns3

#endif // NTN_PLATFORM_SPEC_H
