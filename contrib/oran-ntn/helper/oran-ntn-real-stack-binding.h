/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * OranNtnRealStackBinding (Wave 2 M2, P0-1 and P0-2): binds the O-RAN RIC
 * plane of OranNtnHelper to ONE measured NR stack (ntn-traffic
 * NtnRealStackHelper).
 *
 *  - Actuator registry (P0-1). RC actions reach the radio:
 *      HANDOVER_TRIGGER     -> NtnRealStackHelper::TriggerHandover (X2
 *                              reconfiguration with sync, TS 38.331 5.3.5.5;
 *                              E2SM-RC v01.03 Control Style 3, connected-mode
 *                              mobility)
 *      SLICE_PRB_ALLOCATION -> NtnSliceQuotaScheduler::SetQuotas on the gNB MAC
 *                              (E2SM-RC Control Style 2, slice-level PRB quota;
 *                              TS 28.541 RRMPolicyRatio semantics)
 *      TX_POWER_CONTROL     -> NrGnbPhy::SetTxPower (relative step in dB)
 *      MCS_OVERRIDE         -> NR MAC scheduler FixedMcsDl / StartingMcsDl
 *    Only the actuators the configured radio supports are registered, so
 *    OranNtnHelper::CreateAdmittedXapps refuses every xApp whose actions
 *    would end as "routed, not actuated".
 *
 *  - One measured KPM producer (P0-2). Every granularity period it builds one
 *    E2KpmReport per attached UE for its serving cell's E2 node, and one per
 *    neighbour cell present in the UE's RRC MeasurementReport, from measured
 *    sources only:
 *      SINR            UE PHY RxPacketTrace (latest TB, TS 38.215 SINR)
 *      RSRP / RSRQ     RRC measResults (TS 38.331 5.5.5, TS 36.133 mapping as
 *                      the vendored stack encodes it)
 *      DRB.UEThpDl     delivered application bytes in the period
 *      DRB.PdcpSduVolumeDL  same bytes, Mbit (TS 28.552 5.1.2.1.1.1)
 *      DRB.RlcSduDelayDl gNB RLC ingress to UE PDCP delivery (TS 28.552
 *                      5.1.1.1.1; Wave 1 C6 counters)
 *      TB.TotNbrDl / TB.ErrTotNbrDl  PHY TB counts in the period
 *      RRU.PrbUsedDl   PRB x symbol/14 sum of the cell over the slots of the
 *                      period; RRU.PrbAvailDl = configured BWP PRBs
 *    A field without a measured source stays NaN (exported present=false):
 *    no RSRP = SINR - 95, no latency = slant/c + 1 ms, no RSRQ or CQI from a
 *    SINR formula, no Friis candidate SINR. Elevation, Doppler and propagation
 *    delay are ephemeris quantities (E2SM-NTN, SGP4 geometry) and are filled
 *    only when SetGeometry() is called; TTE only from SetTteProvider().
 */

#ifndef ORAN_NTN_REAL_STACK_BINDING_H
#define ORAN_NTN_REAL_STACK_BINDING_H

#include "ns3/event-id.h"
#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/oran-ntn-types.h"
#include "ns3/ptr.h"

#include <functional>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace ns3
{

class OranNtnHelper;
class NtnRealStackHelper;

/**
 * \ingroup oran-ntn
 * \brief Actuator registry + measured KPM producer on one NtnRealStackHelper.
 */
class OranNtnRealStackBinding : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnRealStackBinding();
    ~OranNtnRealStackBinding() override;

    /**
     * \brief Bind after NtnRealStackHelper::Build() and after the E2 nodes exist.
     *
     * E2 node id k (1-based, OranNtnHelper::CreateSatelliteE2Nodes) is gNB
     * index k - 1 of the stack. Registers the actuators this radio supports.
     */
    void Bind(Ptr<OranNtnHelper> helper, NtnRealStackHelper* rs);

    /// Declare a UE's slice (0 eMBB, 1 URLLC, 2 mMTC, the oran-ntn ids). The
    /// slice actuator maps it into the quota scheduler (SetSliceOfRnti).
    void SetUeSlice(uint32_t ueIndex, uint8_t sliceId);
    /// Map an oran slice id to the quota scheduler's slice index (identity by default).
    void SetSchedulerSliceIndex(uint8_t sliceId, uint32_t schedulerSlice);
    /// Initial quota shares written to every quota scheduler at Bind().
    void SetInitialQuotas(const std::vector<double>& shares) { m_initialQuotas = shares; }

    /// Optional ephemeris enrichment: satellite (per gNB index) and UE mobility.
    void SetGeometry(const std::vector<Ptr<MobilityModel>>& sats,
                     const std::vector<Ptr<MobilityModel>>& ues,
                     double carrierHz);
    /// Optional TTE provider (e.g. ntn-cho NtnTteEstimator); returns NaN when unknown.
    void SetTteProvider(std::function<double(uint32_t ueIndex, uint32_t gnbIndex)> f)
    {
        m_tte = std::move(f);
    }

    /// Start the measured KPM producer (granularity period, first report time).
    void StartKpm(Time period, Time start);
    Time GetKpmPeriod() const { return m_period; }

    /// Names of the action types this binding actuates.
    std::vector<std::string> GetRegisteredActuators() const;

    /// Measured reports submitted (serving, neighbour).
    uint64_t GetServingReports() const { return m_servingReports; }
    uint64_t GetNeighbourReports() const { return m_neighbourReports; }
    /// Actuator outcomes as seen by the binding itself.
    uint64_t GetHandoversRequested() const { return m_hoRequested; }
    uint64_t GetQuotaUpdates() const { return m_quotaUpdates; }
    uint64_t GetTxPowerUpdates() const { return m_txPowerUpdates; }
    /// Last quota vector written to gNB index g (empty if none).
    std::vector<double> GetQuotas(uint32_t gnbIndex) const;

    /// Optional observer of every measured report (observability export).
    void SetReportObserver(std::function<void(const E2KpmReport&)> f)
    {
        m_reportObserver = std::move(f);
    }

  protected:
    void DoDispose() override;

  private:
    bool ActuateHandover(E2RcAction action);
    bool ActuateSlice(E2RcAction action);
    bool ActuateTxPower(E2RcAction action);
    bool ActuateMcs(E2RcAction action);
    void KpmTick();
    void MapRntisToSlices();
    int32_t GnbIndexOfCell(uint16_t cellId) const;

    Ptr<OranNtnHelper> m_helper;
    NtnRealStackHelper* m_rs{nullptr};
    std::vector<std::string> m_registered;

    std::map<uint32_t, uint8_t> m_ueSlice;
    std::map<uint8_t, uint32_t> m_schedSlice;
    std::vector<double> m_initialQuotas;
    std::map<uint32_t, std::vector<double>> m_quotas; //!< per gNB index
    std::set<std::pair<uint16_t, uint16_t>> m_rntiMapped; //!< (cellId, rnti)
    std::map<uint32_t, double> m_initialTxPower;       //!< per gNB index (dBm)

    std::vector<Ptr<MobilityModel>> m_satMob;
    std::vector<Ptr<MobilityModel>> m_ueMob;
    double m_carrierHz{0.0};
    std::function<double(uint32_t, uint32_t)> m_tte;
    std::function<void(const E2KpmReport&)> m_reportObserver;

    Time m_period{MilliSeconds(100)};
    EventId m_tick;
    struct UePrev
    {
        bool valid{false};
        uint64_t rxBytes{0};
        uint64_t rxPkts{0};
        uint64_t lost{0};
        uint64_t tb{0};
        uint64_t tbErr{0};
        double rlcSum{0.0};
        uint64_t rlcN{0};
        double prbSym{0.0};
    };
    std::map<uint32_t, UePrev> m_prev;
    std::map<uint16_t, double> m_prevCellPrbSlots;
    bool m_prevCellValid{false};

    uint64_t m_servingReports{0};
    uint64_t m_neighbourReports{0};
    uint64_t m_hoRequested{0};
    uint64_t m_quotaUpdates{0};
    uint64_t m_txPowerUpdates{0};
};

} // namespace ns3

#endif // ORAN_NTN_REAL_STACK_BINDING_H
