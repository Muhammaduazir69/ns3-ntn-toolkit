/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// OranNtnRtRic — the REAL-TIME RIC tier (<10 ms) of the multi-tier O-RAN NTN
// control architecture (Deng 2026 Sec. IV-B enabler 2; adoption plan WS3).
//
// The RT-RIC is CO-LOCATED with the O-DU on the satellite, so its control
// loop sees ZERO feeder-link latency — unlike Near-RT xApps (10 ms–1 s,
// behind E2 over the feeder) and Non-RT rApps (>1 s, behind A1). Its loops
// act on the MEASURED PHY trace (per-UE SINR/TBLER from RxPacketTraceUe via
// NtnRealStackHelper accessors), enforcing fast policies: MCS cap under deep
// fades, PRB quota enforcement, beam-weight choice. The class hard-asserts
// the O-RAN RT bound: a loop period >= 10 ms refuses to start.

#ifndef ORAN_NTN_RT_RIC_H
#define ORAN_NTN_RT_RIC_H

#include <ns3/event-id.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/traced-callback.h>

#include <cstdint>
#include <functional>
#include <vector>

namespace ns3
{

/**
 * \ingroup oran-ntn
 * \brief Real-time RIC tier (<10 ms loop) co-located with the on-board O-DU.
 *
 * \note Currently exercised by unit tests only (oran-ntn-multi-tier-ric
 *       suite); no example wires an RT-RIC loop yet.
 */
class OranNtnRtRic : public Object
{
  public:
    /// Per-UE measured-KPI snapshot handed to RT actions every loop.
    struct RtKpi
    {
        uint32_t ueIndex{0};
        double sinrDb{0};
        double tbler{0};
    };

    /// An RT action: runs inside the loop, returns true if it actuated.
    using RtAction = std::function<bool(const RtKpi&)>;
    /// Measured KPI source (e.g. NtnRealStackHelper::GetUeRecentSinrDb).
    using KpiGetter = std::function<double(uint32_t ueIndex)>;

    static TypeId GetTypeId();
    OranNtnRtRic();

    /// O-RAN RT bound: must be < 10 ms (NS_ABORT otherwise at Start).
    void SetLoopPeriod(Time t) { m_period = t; }
    Time GetLoopPeriod() const { return m_period; }
    void SetNumUes(uint32_t n) { m_numUes = n; }
    /// Wire the measured PHY KPI sources (NaN-safe: NaN UEs are skipped).
    void SetKpiSources(KpiGetter sinrDb, KpiGetter tbler);
    /// Register an RT action evaluated for every UE every loop.
    void AddRtAction(std::string name, RtAction action);

    void Start();
    void Stop();

    uint64_t GetLoopCount() const { return m_loops; }
    uint64_t GetActuationCount() const { return m_actuations; }

    /// Trace: (actionName, ueIndex, sinrDb) on every actuation.
    typedef void (*ActuationTracedCallback)(std::string, uint32_t, double);

  private:
    void Loop();

    Time m_period{MilliSeconds(2)};
    uint32_t m_numUes{0};
    KpiGetter m_sinrGetter;
    KpiGetter m_tblerGetter;

    struct NamedAction
    {
        std::string name;
        RtAction fn;
    };
    std::vector<NamedAction> m_actions;

    EventId m_loopEvent;
    bool m_running{false};
    uint64_t m_loops{0};
    uint64_t m_actuations{0};

    TracedCallback<std::string, uint32_t, double> m_actuationTrace;
};

} // namespace ns3

#endif // ORAN_NTN_RT_RIC_H
