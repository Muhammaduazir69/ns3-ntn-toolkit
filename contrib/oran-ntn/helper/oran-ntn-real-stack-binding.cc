/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "oran-ntn-real-stack-binding.h"

#include "oran-ntn-helper.h"

#include "ns3/boolean.h"
#include "ns3/log.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-mac-scheduler.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-slice-quota-scheduler.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnRealStackBinding");
NS_OBJECT_ENSURE_REGISTERED(OranNtnRealStackBinding);

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kC = 299792458.0;

double
Norm(const Vector& v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/// Elevation (deg) of `sat` seen from `ue`, both ECEF, local vertical = geocentric radial.
double
ElevationDeg(const Vector& ue, const Vector& sat)
{
    const Vector d(sat.x - ue.x, sat.y - ue.y, sat.z - ue.z);
    const double nu = Norm(ue);
    const double nd = Norm(d);
    if (nu <= 0.0 || nd <= 0.0)
    {
        return kNaN;
    }
    return std::asin((d.x * ue.x + d.y * ue.y + d.z * ue.z) / (nu * nd)) * 180.0 / M_PI;
}
} // namespace

TypeId
OranNtnRealStackBinding::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnRealStackBinding")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnRealStackBinding>();
    return tid;
}

OranNtnRealStackBinding::OranNtnRealStackBinding() = default;
OranNtnRealStackBinding::~OranNtnRealStackBinding() = default;

void
OranNtnRealStackBinding::DoDispose()
{
    Simulator::Cancel(m_tick);
    m_helper = nullptr;
    m_rs = nullptr;
    m_satMob.clear();
    m_ueMob.clear();
    m_tte = nullptr;
    m_reportObserver = nullptr;
    Object::DoDispose();
}

void
OranNtnRealStackBinding::Bind(Ptr<OranNtnHelper> helper, NtnRealStackHelper* rs)
{
    NS_ABORT_MSG_IF(!helper || !rs, "OranNtnRealStackBinding::Bind needs a helper and a stack");
    m_helper = helper;
    m_rs = rs;
    m_registered.clear();
    if (rs->GetRadioBackend() != NtnRealStackHelper::RadioBackend::Nr)
    {
        // The mmwave backend has no TriggerHandover, quota scheduler or NR PHY
        // power knob: register nothing, so admission refuses every xApp rather
        // than starting xApps that can only log.
        NS_LOG_WARN("OranNtnRealStackBinding: radio backend is not nr; no actuator registered");
        return;
    }
    const uint32_t nGnb = rs->GetEnbDevices().GetN();
    if (nGnb >= 2)
    {
        helper->RegisterActuator(
            E2RcActionType::HANDOVER_TRIGGER,
            MakeCallback(&OranNtnRealStackBinding::ActuateHandover, this),
            "nr-x2-handover (NtnRealStackHelper::TriggerHandover)");
        m_registered.push_back("HANDOVER_TRIGGER");
    }
    bool anyQuota = false;
    for (uint32_t g = 0; g < nGnb; ++g)
    {
        auto gnb = DynamicCast<NrGnbNetDevice>(rs->GetEnbDevices().Get(g));
        if (!gnb)
        {
            continue;
        }
        m_initialTxPower[g] = gnb->GetPhy(0)->GetTxPower();
        auto q = DynamicCast<ntnslice::NtnSliceQuotaScheduler>(gnb->GetScheduler(0));
        if (q)
        {
            anyQuota = true;
            if (!m_initialQuotas.empty())
            {
                q->SetQuotas(m_initialQuotas);
                m_quotas[g] = m_initialQuotas;
            }
        }
    }
    if (anyQuota)
    {
        helper->RegisterActuator(
            E2RcActionType::SLICE_PRB_ALLOCATION,
            MakeCallback(&OranNtnRealStackBinding::ActuateSlice, this),
            "nr-mac-slice-quota (NtnSliceQuotaScheduler::SetQuotas)");
        m_registered.push_back("SLICE_PRB_ALLOCATION");
    }
    helper->RegisterActuator(E2RcActionType::TX_POWER_CONTROL,
                             MakeCallback(&OranNtnRealStackBinding::ActuateTxPower, this),
                             "nr-gnb-phy-txpower (NrGnbPhy::SetTxPower)");
    m_registered.push_back("TX_POWER_CONTROL");
    helper->RegisterActuator(E2RcActionType::MCS_OVERRIDE,
                             MakeCallback(&OranNtnRealStackBinding::ActuateMcs, this),
                             "nr-mac-fixed-mcs (FixedMcsDl/StartingMcsDl)");
    m_registered.push_back("MCS_OVERRIDE");
}

std::vector<std::string>
OranNtnRealStackBinding::GetRegisteredActuators() const
{
    return m_registered;
}

void
OranNtnRealStackBinding::SetUeSlice(uint32_t ueIndex, uint8_t sliceId)
{
    m_ueSlice[ueIndex] = sliceId;
    if (m_helper)
    {
        m_helper->SetUeSliceId(ueIndex, sliceId);
    }
}

void
OranNtnRealStackBinding::SetSchedulerSliceIndex(uint8_t sliceId, uint32_t schedulerSlice)
{
    m_schedSlice[sliceId] = schedulerSlice;
}

void
OranNtnRealStackBinding::SetGeometry(const std::vector<Ptr<MobilityModel>>& sats,
                                     const std::vector<Ptr<MobilityModel>>& ues,
                                     double carrierHz)
{
    m_satMob = sats;
    m_ueMob = ues;
    m_carrierHz = carrierHz;
}

std::vector<double>
OranNtnRealStackBinding::GetQuotas(uint32_t gnbIndex) const
{
    auto it = m_quotas.find(gnbIndex);
    return it == m_quotas.end() ? std::vector<double>{} : it->second;
}

int32_t
OranNtnRealStackBinding::GnbIndexOfCell(uint16_t cellId) const
{
    if (!m_rs || cellId == 0)
    {
        return -1;
    }
    for (uint32_t g = 0; g < m_rs->GetEnbDevices().GetN(); ++g)
    {
        if (m_rs->GetGnbCellId(g) == cellId)
        {
            return static_cast<int32_t>(g);
        }
    }
    return -1;
}

// ---------------------------------------------------------------------------
// Actuators
// ---------------------------------------------------------------------------

bool
OranNtnRealStackBinding::ActuateHandover(E2RcAction action)
{
    if (!m_rs || action.targetGnbId == 0 || action.targetGnbId > m_rs->GetEnbDevices().GetN())
    {
        return false;
    }
    if (action.targetUeId >= m_rs->GetNumUes())
    {
        return false;
    }
    const uint16_t cell = m_rs->GetGnbCellId(action.targetGnbId - 1);
    // TriggerHandover refuses (and counts the refusal) when the UE is not
    // attached, already on the target, or handover is disabled; the RIC then
    // sees actuated = false, never a silent accept.
    const bool ok = cell != 0 && m_rs->TriggerHandover(action.targetUeId, cell);
    m_hoRequested += ok ? 1 : 0;
    return ok;
}

bool
OranNtnRealStackBinding::ActuateSlice(E2RcAction action)
{
    if (!m_rs || action.targetGnbId == 0 || action.targetGnbId > m_rs->GetEnbDevices().GetN())
    {
        return false;
    }
    const uint32_t g = action.targetGnbId - 1;
    auto gnb = DynamicCast<NrGnbNetDevice>(m_rs->GetEnbDevices().Get(g));
    auto q = gnb ? DynamicCast<ntnslice::NtnSliceQuotaScheduler>(gnb->GetScheduler(0)) : nullptr;
    if (!q || !std::isfinite(action.parameter1) || action.parameter1 < 0.0)
    {
        return false;
    }
    MapRntisToSlices();
    auto sm = m_schedSlice.find(action.targetSliceId);
    const uint32_t idx = (sm != m_schedSlice.end()) ? sm->second : action.targetSliceId;
    std::vector<double>& v = m_quotas[g];
    if (v.empty())
    {
        v = q->GetQuotas();
    }
    if (v.size() <= idx)
    {
        v.resize(idx + 1, 0.0);
    }
    // E2SM-RC Style 2 carries one slice's share per control; the scheduler
    // takes the whole vector. A share vector over 1 (the other slices still at
    // their previous value) is scaled down so that the ratios hold, which is
    // the TS 28.541 RRMPolicyRatio constraint (sum of dedicated ratios <= 100).
    v[idx] = std::min(1.0, action.parameter1);
    const double sum = std::accumulate(v.begin(), v.end(), 0.0);
    std::vector<double> applied = v;
    if (sum > 1.0)
    {
        for (double& x : applied)
        {
            x /= sum;
        }
    }
    q->SetQuotas(applied);
    ++m_quotaUpdates;
    return true;
}

bool
OranNtnRealStackBinding::ActuateTxPower(E2RcAction action)
{
    if (!m_rs || action.targetGnbId == 0 || action.targetGnbId > m_rs->GetEnbDevices().GetN() ||
        !std::isfinite(action.parameter1))
    {
        return false;
    }
    const uint32_t g = action.targetGnbId - 1;
    auto gnb = DynamicCast<NrGnbNetDevice>(m_rs->GetEnbDevices().Get(g));
    if (!gnb)
    {
        return false;
    }
    Ptr<NrGnbPhy> phy = gnb->GetPhy(0);
    const double initial = m_initialTxPower.count(g) ? m_initialTxPower[g] : phy->GetTxPower();
    // parameter1 is a relative step (dB, negative = reduction), bounded to the
    // configured power and 30 dB below it.
    const double target = std::clamp(phy->GetTxPower() + action.parameter1, initial - 30.0, initial);
    phy->SetTxPower(target);
    ++m_txPowerUpdates;
    return true;
}

bool
OranNtnRealStackBinding::ActuateMcs(E2RcAction action)
{
    if (!m_rs || action.targetGnbId == 0 || action.targetGnbId > m_rs->GetEnbDevices().GetN())
    {
        return false;
    }
    auto gnb = DynamicCast<NrGnbNetDevice>(m_rs->GetEnbDevices().Get(action.targetGnbId - 1));
    if (!gnb || !std::isfinite(action.parameter1))
    {
        return false;
    }
    Ptr<NrMacScheduler> s = gnb->GetScheduler(0);
    if (action.parameter1 < 0.0)
    {
        s->SetAttribute("FixedMcsDl", BooleanValue(false));
        return true;
    }
    s->SetAttribute("FixedMcsDl", BooleanValue(true));
    s->SetAttribute("StartingMcsDl",
                    UintegerValue(static_cast<uint8_t>(std::min(action.parameter1, 28.0))));
    return true;
}

void
OranNtnRealStackBinding::MapRntisToSlices()
{
    if (!m_rs)
    {
        return;
    }
    for (const auto& [ue, slice] : m_ueSlice)
    {
        const uint16_t cell = m_rs->GetUeServingCellId(ue);
        const uint16_t rnti = m_rs->GetUeRnti(ue);
        if (cell == 0 || rnti == 0 || m_rntiMapped.count({cell, rnti}))
        {
            continue;
        }
        const int32_t g = GnbIndexOfCell(cell);
        if (g < 0)
        {
            continue;
        }
        auto gnb = DynamicCast<NrGnbNetDevice>(m_rs->GetEnbDevices().Get(g));
        auto q = gnb ? DynamicCast<ntnslice::NtnSliceQuotaScheduler>(gnb->GetScheduler(0))
                     : nullptr;
        if (!q)
        {
            continue;
        }
        auto sm = m_schedSlice.find(slice);
        q->SetSliceOfRnti(rnti, sm != m_schedSlice.end() ? sm->second : slice);
        m_rntiMapped.insert({cell, rnti});
    }
}

// ---------------------------------------------------------------------------
// Measured KPM producer
// ---------------------------------------------------------------------------

void
OranNtnRealStackBinding::StartKpm(Time period, Time start)
{
    m_period = period;
    Simulator::Cancel(m_tick);
    m_tick = Simulator::Schedule(start, &OranNtnRealStackBinding::KpmTick, this);
}

void
OranNtnRealStackBinding::KpmTick()
{
    m_tick = Simulator::Schedule(m_period, &OranNtnRealStackBinding::KpmTick, this);
    if (!m_rs || !m_helper)
    {
        return;
    }
    MapRntisToSlices();
    const double now = Simulator::Now().GetSeconds();
    const double T = m_period.GetSeconds();
    const uint32_t nUe = m_rs->GetNumUes();
    const uint32_t nGnb = m_rs->GetEnbDevices().GetN();
    const double prbAvail = static_cast<double>(m_rs->GetBwpPrbs());
    const double slotsInPeriod = m_period.GetSeconds() / m_rs->GetSlotDuration().GetSeconds();

    // Cell level: PRB use and the UEs each cell serves.
    std::map<uint16_t, double> prbUsed;
    std::map<uint16_t, uint32_t> activeUes;
    for (uint32_t g = 0; g < nGnb; ++g)
    {
        const uint16_t cell = m_rs->GetGnbCellId(g);
        const double cum = m_rs->GetCellDlPrbSlotsUsed(cell);
        if (m_prevCellValid && m_prevCellPrbSlots.count(cell))
        {
            prbUsed[cell] = (cum - m_prevCellPrbSlots[cell]) / std::max(1.0, slotsInPeriod);
        }
        m_prevCellPrbSlots[cell] = cum;
    }
    const bool prbValid = m_prevCellValid;
    m_prevCellValid = true;

    std::vector<E2KpmReport> serving;
    std::map<uint16_t, double> cellThr;
    for (uint32_t u = 0; u < nUe; ++u)
    {
        const uint16_t cell = m_rs->GetUeServingCellId(u);
        const int32_t g = GnbIndexOfCell(cell);
        UePrev& p = m_prev[u];
        const uint64_t rx = m_rs->GetUeRxBytes(u);
        const uint64_t rxPk = m_rs->GetUeRxPackets(u);
        const uint64_t lost = m_rs->GetUeLostPackets(u);
        const NtnRealStackHelper::UeRanCounters rc = m_rs->GetUeRanCounters(u);
        if (g < 0 || m_rs->GetUeRnti(u) == 0)
        {
            p = UePrev{true, rx, rxPk, lost, rc.dlTb, rc.dlTbErr, rc.rlcSduDelaySumMs, rc.rlcSdus,
                       rc.prbSymSum};
            continue;
        }
        activeUes[cell]++;

        E2KpmReport r{};
        r.timestamp = now;
        r.gnbId = static_cast<uint32_t>(g) + 1;
        r.isNtn = true;
        r.ueId = u;
        r.granularityPeriod_s = T;
        // SINR: latest DL TB of this UE (PHY trace); NaN when no TB yet.
        r.sinr_dB = m_rs->GetUeRecentSinrDb(u);
        r.sinrMeasured = std::isfinite(r.sinr_dB);
        // RSRP / RSRQ: RRC MeasurementReport of the serving cell; NaN when the
        // stack has not reported (no periodic or event report configured).
        r.rsrp_dBm = m_rs->GetUeServingRsrpDbm(u);
        r.rsrpMeasured = std::isfinite(r.rsrp_dBm);
        r.rsrq_dB = m_rs->GetUeServingRsrqDb(u);
        r.rsrqMeasured = std::isfinite(r.rsrq_dB);
        r.cqi = 0; // no CQI is exported by the stack to this binding: not present
        r.cqiMeasured = false;
        r.harqBler = kNaN;
        r.throughput_Mbps = kNaN;
        r.latency_ms = kNaN;
        r.beamGain_dB = kNaN;
        r.beamId = 0;
        r.cellThroughput_Mbps = kNaN;
        r.sliceThroughput_Mbps = kNaN;
        r.sliceLatency_ms = kNaN;
        r.sliceReliability = kNaN;
        r.elevation_deg = kNaN;
        r.doppler_Hz = kNaN;
        r.propagationDelay_ms = kNaN;
        r.tte_s = kNaN;
        if (p.valid && rx >= p.rxBytes)
        {
            const double bits = static_cast<double>(rx - p.rxBytes) * 8.0;
            r.throughput_Mbps = bits / T / 1e6;
            r.throughputMeasured = true;
            r.pdcpVolumeDl_Mbit = bits / 1e6;
            cellThr[cell] += r.throughput_Mbps;
        }
        if (p.valid && rc.dlTb >= p.tb)
        {
            r.tbTotDl = static_cast<double>(rc.dlTb - p.tb);
            r.tbErrDl = static_cast<double>(rc.dlTbErr - p.tbErr);
            if (r.tbTotDl > 0)
            {
                r.harqBler = r.tbErrDl / r.tbTotDl;
                r.blerMeasured = true;
            }
        }
        if (p.valid && rc.rlcSdus > p.rlcN)
        {
            r.rlcSduDelayDl_ms = (rc.rlcSduDelaySumMs - p.rlcSum) / double(rc.rlcSdus - p.rlcN);
            r.latency_ms = r.rlcSduDelayDl_ms;
            r.latencyMeasured = true;
        }
        if (p.valid && rxPk + lost > p.rxPkts + p.lost)
        {
            const double d = double(rxPk - p.rxPkts);
            const double l = double(lost - p.lost);
            r.sliceReliability = d / (d + l);
        }
        r.prbAvailDl = prbAvail;
        if (p.valid && rc.prbSymSum >= p.prbSym)
        {
            r.uePrbShareDl =
                (rc.prbSymSum - p.prbSym) / std::max(1.0, prbAvail * slotsInPeriod);
        }
        if (prbValid && prbUsed.count(cell))
        {
            r.prbUsedDl = prbUsed[cell];
            r.prbUtilization = std::clamp(r.prbUsedDl / std::max(1.0, prbAvail), 0.0, 1.0);
            r.prbMeasured = true;
        }
        else
        {
            r.prbUtilization = kNaN;
        }
        auto sl = m_ueSlice.find(u);
        r.sliceId = (sl != m_ueSlice.end()) ? sl->second : 0;
        r.sliceThroughput_Mbps = r.throughput_Mbps;
        r.sliceLatency_ms = r.latency_ms;
        // Ephemeris enrichment (E2SM-NTN quantities from SGP4 geometry).
        if (static_cast<size_t>(g) < m_satMob.size() && u < m_ueMob.size() && m_satMob[g] &&
            m_ueMob[u])
        {
            const Vector sp = m_satMob[g]->GetPosition();
            const Vector up = m_ueMob[u]->GetPosition();
            r.elevation_deg = ElevationDeg(up, sp);
            const Vector d(sp.x - up.x, sp.y - up.y, sp.z - up.z);
            const double slant = Norm(d);
            r.propagationDelay_ms = slant / kC * 1e3;
            if (m_carrierHz > 0.0 && slant > 0.0)
            {
                const Vector sv = m_satMob[g]->GetVelocity();
                const Vector uv = m_ueMob[u]->GetVelocity();
                const double rr = ((sv.x - uv.x) * d.x + (sv.y - uv.y) * d.y +
                                   (sv.z - uv.z) * d.z) / slant; // range rate, m/s
                r.doppler_Hz = -rr / kC * m_carrierHz;
            }
        }
        if (m_tte)
        {
            r.tte_s = m_tte(u, static_cast<uint32_t>(g));
        }
        p = UePrev{true, rx, rxPk, lost, rc.dlTb, rc.dlTbErr, rc.rlcSduDelaySumMs, rc.rlcSdus,
                       rc.prbSymSum};
        serving.push_back(r);
    }

    for (auto& r : serving)
    {
        const uint16_t cell = m_rs->GetGnbCellId(r.gnbId - 1);
        r.activeUes = activeUes[cell];
        if (prbValid && cellThr.count(cell))
        {
            r.cellThroughput_Mbps = cellThr[cell];
        }
        if (m_helper->SubmitKpmReport(r))
        {
            ++m_servingReports;
        }
        if (m_reportObserver)
        {
            m_reportObserver(r);
        }
        // Neighbours in the UE's RRC MeasurementReport: measured RSRP/RSRQ only.
        for (const auto& [ncell, meas] : m_rs->GetUeMeasurements(r.ueId))
        {
            if (meas.serving || ncell == cell)
            {
                continue;
            }
            const int32_t ng = GnbIndexOfCell(ncell);
            if (ng < 0 || !std::isfinite(meas.rsrpDbm))
            {
                continue;
            }
            E2KpmReport n{};
            n.timestamp = now;
            n.gnbId = static_cast<uint32_t>(ng) + 1;
            n.isNtn = true;
            n.ueId = r.ueId;
            n.granularityPeriod_s = T;
            n.sinr_dB = kNaN; // no neighbour SS-SINR is reported: not present
            n.rsrp_dBm = meas.rsrpDbm;
            n.rsrpMeasured = true;
            n.rsrq_dB = meas.rsrqDb;
            n.rsrqMeasured = std::isfinite(meas.rsrqDb);
            n.throughput_Mbps = kNaN;
            n.latency_ms = kNaN;
            n.harqBler = kNaN;
            n.prbUtilization = kNaN;
            n.beamGain_dB = kNaN;
            n.cellThroughput_Mbps = kNaN;
            n.sliceThroughput_Mbps = kNaN;
            n.sliceLatency_ms = kNaN;
            n.sliceReliability = kNaN;
            n.elevation_deg = kNaN;
            n.doppler_Hz = kNaN;
            n.propagationDelay_ms = kNaN;
            n.tte_s = kNaN;
            n.sliceId = r.sliceId;
            n.prbAvailDl = prbAvail;
            if (static_cast<size_t>(ng) < m_satMob.size() && r.ueId < m_ueMob.size() &&
                m_satMob[ng] && m_ueMob[r.ueId])
            {
                const Vector sp = m_satMob[ng]->GetPosition();
                const Vector up = m_ueMob[r.ueId]->GetPosition();
                n.elevation_deg = ElevationDeg(up, sp);
                n.propagationDelay_ms =
                    Norm(Vector(sp.x - up.x, sp.y - up.y, sp.z - up.z)) / kC * 1e3;
            }
            if (m_tte)
            {
                n.tte_s = m_tte(r.ueId, static_cast<uint32_t>(ng));
            }
            if (m_helper->SubmitKpmReport(n))
            {
                ++m_neighbourReports;
            }
            if (m_reportObserver)
            {
                m_reportObserver(n);
            }
        }
    }
}

} // namespace ns3
