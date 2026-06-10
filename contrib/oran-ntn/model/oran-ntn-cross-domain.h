/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// Cross-domain SMO trio (Deng 2026 Sec. IV-B enabler 4; adoption plan WS3):
//
//   OranNtnNwdaf           — the CN-domain analytics function (3GPP NWDAF
//                            role): consumes NtnOranAiFlowMonitor KPM
//                            indications (MEASURED, in-band) and maintains
//                            per-slice analytics incl. an SLA-risk score
//                            from the delay trend.
//   OranNtnTpnController   — the transport (TPN) domain controller: ranks
//                            registered transport paths by live latency
//                            (provider callbacks wrap e.g. the
//                            ContactGraphRouter) and switches the active
//                            path per slice within its latency budget.
//   OranNtnCrossDomainSmo  — coordinates RAN (radio quota via A1-style
//                            callback) + TPN (path selection) + CN/edge
//                            (compute allocation) in one closed loop — the
//                            paper's representative cross-domain slice
//                            adaptation, driven by measured analytics only.

#ifndef ORAN_NTN_CROSS_DOMAIN_H
#define ORAN_NTN_CROSS_DOMAIN_H

#include <ns3/ntn-oran-ai-flow-monitor.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/ptr.h>
#include <ns3/traced-callback.h>

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace ns3
{

// ------------------------------------------------------------------- NWDAF

class OranNtnNwdaf : public Object
{
  public:
    struct SliceAnalytics
    {
        uint8_t sst{0};
        uint32_t sd{0};
        double loadMbps{0};      ///< EWMA of measured DRB.UEThpDl
        double meanDelayMs{0};   ///< EWMA of measured DRB.RlcSduDelayDl
        double lossRate{0};      ///< EWMA of measured DRB.PacketLossRateDl
        double delaySlopeMsPerS{0};
        double slaRisk{0};       ///< 0..1, from delay-budget headroom + trend
        uint64_t samples{0};
    };

    static TypeId GetTypeId();
    OranNtnNwdaf() = default;

    /// Per-slice one-way-delay SLA budget (ms) used for the risk score.
    void SetSliceDelayBudget(uint8_t sst, double budgetMs) { m_budgetMs[sst] = budgetMs; }
    /// Wire as consumer: monitor->RegisterE2Consumer(MakeCallback to this).
    void ConsumeKpm(const NtnOranAiFlowMonitor::KpmIndication& ind);

    /// Current analytics for a slice (zero-initialized if unseen).
    SliceAnalytics GetAnalytics(uint8_t sst) const;
    std::vector<SliceAnalytics> GetAllAnalytics() const;

  private:
    struct SliceState
    {
        SliceAnalytics a;
        double prevDelayMs{0};
        Time prevTime{Seconds(0)};
    };

    std::map<uint8_t, SliceState> m_slices;
    std::map<uint8_t, double> m_budgetMs;
};

// ---------------------------------------------------------- TPN controller

class OranNtnTpnController : public Object
{
  public:
    /// Live one-way latency of the path in ms (e.g. contact-graph edge sums).
    using LatencyProvider = std::function<double()>;
    /// Actuator that makes the path the active route for a slice.
    using PathSwitcher = std::function<void(uint8_t sst)>;

    static TypeId GetTypeId();
    OranNtnTpnController() = default;

    void RegisterPath(std::string name, LatencyProvider latency, PathSwitcher activate);
    /**
     * \brief Pick the lowest-latency registered path meeting \p budgetMs and
     *        activate it for \p sst. Returns the chosen path name, or "" if
     *        no path met the budget (the least-bad path is still activated).
     */
    std::string SelectPathForSlice(uint8_t sst, double budgetMs);
    /// Name of the active path per slice ("" if never selected).
    std::string GetActivePath(uint8_t sst) const;
    uint32_t GetSwitchCount() const { return m_switches; }

  private:
    struct PathEntry
    {
        std::string name;
        LatencyProvider latency;
        PathSwitcher activate;
    };

    std::vector<PathEntry> m_paths;
    std::map<uint8_t, std::string> m_active;
    uint32_t m_switches{0};
};

// -------------------------------------------------------- cross-domain SMO

class OranNtnCrossDomainSmo : public Object
{
  public:
    /// RAN-domain knob: A1-style radio resource share for a slice (0..1).
    using RadioQuotaSetter = std::function<void(uint8_t sst, double share)>;
    /// CN/edge-domain knob: computing allocation for a slice (0..1).
    using ComputeAllocSetter = std::function<void(uint8_t sst, double share)>;

    struct Decision
    {
        Time time;
        uint8_t sst{0};
        double slaRisk{0};
        double radioShare{0};
        double computeShare{0};
        std::string tpnPath;
    };

    static TypeId GetTypeId();
    OranNtnCrossDomainSmo() = default;

    void SetNwdaf(Ptr<OranNtnNwdaf> n) { m_nwdaf = n; }
    void SetTpnController(Ptr<OranNtnTpnController> t) { m_tpn = t; }
    void SetRadioQuotaSetter(RadioQuotaSetter cb) { m_radioCb = std::move(cb); }
    void SetComputeAllocSetter(ComputeAllocSetter cb) { m_computeCb = std::move(cb); }
    /// Register a managed slice with its delay budget and baseline shares.
    void AddSlice(uint8_t sst, double delayBudgetMs, double baseRadio, double baseCompute);
    void SetCoordinationPeriod(Time t) { m_period = t; }
    void Start();

    const std::vector<Decision>& GetDecisions() const { return m_decisions; }

  private:
    void Coordinate();

    struct ManagedSlice
    {
        uint8_t sst{0};
        double budgetMs{0};
        double radioShare{0};
        double computeShare{0};
    };

    Ptr<OranNtnNwdaf> m_nwdaf;
    Ptr<OranNtnTpnController> m_tpn;
    RadioQuotaSetter m_radioCb;
    ComputeAllocSetter m_computeCb;
    std::vector<ManagedSlice> m_slices;
    Time m_period{Seconds(2.0)};
    bool m_running{false};
    std::vector<Decision> m_decisions;

    TracedCallback<uint8_t, double, double> m_decisionTrace;
};

} // namespace ns3

#endif // ORAN_NTN_CROSS_DOMAIN_H
