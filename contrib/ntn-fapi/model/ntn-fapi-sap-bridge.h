/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_FAPI_SAP_BRIDGE_H
#define NTN_FAPI_SAP_BRIDGE_H

// NtnFapiSapBridge — a REAL FAPI L1/L2 boundary for a live mmwave NR cell.
//
// Audit gap F1/F3 + CI gate 15: ntn-fapi was a header-only struct library whose
// example built DL_TTI / TX_DATA / RX_DATA / CRC.indication as local variables
// that fell out of scope unconsumed. No MAC or PHY object was ever driven by a
// FAPI message, so there was no slot timing and no measurable SAP latency.
//
// This class closes that gap by DECORATING the real mmwave MAC<->PHY SAP
// (contrib/mmwave/model/mmwave-phy-sap.h). It interposes two forwarding
// decorators between an already-wired MmWaveEnbPhy and MmWaveEnbMac:
//
//   * PHY -> MAC   MmWaveEnbPhySapUser::SlotIndication(SfnSf)
//                    forwarded unchanged, and emitted as a FAPI SLOT.indication
//                    (SCF-222.10 §3.4.4.1, msg-id 0x82) with the SfnSf-derived
//                    frame/slot.
//   * MAC -> PHY   MmWavePhySapProvider::SetSlotAllocInfo(SlotAllocInfo)
//                    forwarded unchanged, and translated into a FAPI
//                    DL_TTI.request (msg-id 0x80) populated from the real DL
//                    DCI(s) the scheduler produced (rnti, mcs, tbSize, and the
//                    REAL mmwave HARQ process id).
//
// The reverse direction (RX_DATA.indication / CRC.indication, msg-ids 0x85/0x86)
// is emitted from the real PHY reception path: feed OnPhyRx() the mmwave
// RxPacketTraceParams the error model already computed (m_corrupt == real decode
// outcome). Each FAPI message thus has a REAL producer and a REAL consumer tied
// to actual slot timing, and the request->indication latency (SLOT.indication ->
// DL_TTI.request -> CRC.indication, aligned by SFN/slot) is measurable off the
// SAP — that latency IS CI gate 15.
//
// The decorators forward every SAP call unchanged and only RECORD the FAPI
// struct as a side effect, so they observe the real mmwave slot timing without
// perturbing it (approach (a) of the audit's prescribed path).

#include "fapi-common.h"
#include "fapi-messages.h"

#include "ns3/mmwave-enb-mac.h"
#include "ns3/mmwave-enb-net-device.h"
#include "ns3/mmwave-enb-phy.h"
#include "ns3/mmwave-phy-mac-common.h"
#include "ns3/mmwave-phy-sap.h"
#include "ns3/mmwave-spectrum-phy.h"
#include "ns3/mmwave-ue-net-device.h"
#include "ns3/mobility-model.h"
#include "ns3/node-list.h"
#include "ns3/node.h"
#include "ns3/spectrum-channel.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/traced-callback.h"

#include <limits>
#include <map>

namespace ns3
{
namespace fapi
{

class FapiPhySapUserDecorator;
class FapiPhySapProviderDecorator;

/// Real FAPI L1/L2 boundary decorating a live mmwave enb MAC<->PHY SAP.
class NtnFapiSapBridge : public Object
{
  public:
    static TypeId GetTypeId();

    NtnFapiSapBridge();
    ~NtnFapiSapBridge() override;

    // ---- P5 phase (bring-up) --------------------------------------------
    /// Consume a CONFIG.request: populate CarrierConfig/CellConfig from the real
    /// mmwave PHY configuration so the claim "FAPI configures the PHY" is true.
    /// Called by InstallEnb() from the live MmWavePhyMacCommon + cell id.
    void Configure(const CarrierConfig& carrier, const CellConfig& cell);
    bool IsConfigured() const { return m_configured; }
    const CarrierConfig& GetCarrierConfig() const { return m_carrier; }
    const CellConfig& GetCellConfig() const { return m_cell; }

    // ---- Install (call once, AFTER NtnRealStackHelper::Build) -----------
    /// Interpose the two forwarding decorators on the enb device's primary
    /// component carrier. Reads the real config for the P5 CONFIG.request.
    void InstallEnb(Ptr<mmwave::MmWaveEnbNetDevice> enb);

    // ---- Reverse path (real PHY reception) ------------------------------
    /// Emit FAPI RX_DATA.indication + CRC.indication from the real per-TB decode
    /// (mmwave RxPacketTraceParams; m_corrupt == real error-model outcome). Wire
    /// this to the UE DlSpectrumPhy RxPacketTraceUe trace. Only TBs for uePhyRnti
    /// (0 == accept any) are consumed.
    void OnPhyRx(const mmwave::RxPacketTraceParams& p);
    void SetUeRnti(uint16_t rnti) { m_ueRnti = rnti; }

    /// FAPI-4 test seam: record the HARQ process id a DL_TTI.request carried
    /// for one UE in one slot, exactly as EmitDlTti does.
    ///
    /// The real path runs through the decorated MAC-PHY SAP and needs a live
    /// MmWaveEnbNetDevice, which is why the multi-UE case had no unit coverage
    /// and the per-slot key survived: the shipped examples schedule four UEs,
    /// but the only test scheduled one.
    void RecordDlHarqForTest(uint32_t frame, uint8_t sf, uint8_t slot, uint16_t rnti,
                             uint8_t harqPid)
    {
        m_dlHarqId[SlotUeKey(frame, sf, slot, rnti)] = harqPid;
        // Stamp the slot's DL_TTI time as well, as EmitDlTti does. Without it
        // the CRC path's one-shot consumption block is never entered, so a test
        // using this seam could not see the erase at all.
        m_dlTtiTimeSec[SlotKey(frame, sf, slot)] = Simulator::Now().GetSeconds();
    }

    // ---- Inspection / counters ------------------------------------------
    uint64_t GetSlotIndicationCount() const { return m_slotIndCount; }
    uint64_t GetDlTtiRequestCount() const { return m_dlTtiCount; }
    uint64_t GetDlTtiWithDataCount() const { return m_dlTtiDataCount; }
    uint64_t GetCrcIndicationCount() const { return m_crcCount; }
    uint64_t GetRxDataIndicationCount() const { return m_rxDataCount; }
    uint64_t GetMatchedLatencyCount() const { return m_latCount; }

    const SlotIndication& LastSlotIndication() const { return m_lastSlot; }
    const DlTtiRequest& LastDlTti() const { return m_lastDlTti; }

    // ---- Uplink half of the L1/L2 boundary (FAPI-1) ----------------------
    // The bridge used to emit only the four downlink-direction messages, so
    // the half of the SAP where NTN timing actually bites - PRACH over a
    // round-trip measured in tens of milliseconds, PUSCH grants, the TB bytes
    // themselves - had no representation at all. These three come from the
    // same real SAP calls the decorators already sit on; none of them is
    // synthesized.
    uint64_t GetUlTtiRequestCount() const { return m_ulTtiCount; }
    uint64_t GetUlTtiWithGrantCount() const { return m_ulTtiGrantCount; }
    uint64_t GetTxDataRequestCount() const { return m_txDataCount; }
    uint64_t GetTxDataBytes() const { return m_txDataBytes; }
    uint64_t GetRachIndicationCount() const { return m_rachCount; }

    const UlTtiRequest& LastUlTti() const { return m_lastUlTti; }
    const TxDataRequest& LastTxData() const { return m_lastTxData; }
    const RachIndication& LastRachIndication() const { return m_lastRach; }

    /// NTN geometry for the timing-advance field of RACH.indication.
    ///
    /// TS 38.213 section 4.2: the UE pre-compensates its transmission by
    /// N_TA/2 so the preamble lands inside the gNB reception window. What the
    /// gNB then measures, and what FAPI reports back in RACH.indication, is
    /// the RESIDUAL - the part of the round trip the pre-compensation did not
    /// cover. Passing the round trip with zero pre-compensation therefore
    /// reproduces the un-compensated case, which is the one TR 38.821 section
    /// 7.3 says fails.
    ///
    /// \param roundTrip two-way propagation delay on the service link.
    /// \param preCompensated the amount the UE already applied (Time() for none).
    void SetNtnTimingAdvance(Time roundTrip, Time preCompensated = Time());
    Time GetResidualTimingAdvance() const { return m_taResidual; }
    /// Residual expressed in T_C units of TS 38.211 section 4.1, which is the
    /// unit the FAPI timingAdvance field carries.
    uint32_t GetResidualTaInTc() const;

    // ---- Provenance of the SAP latency (FAPI-2) --------------------------
    /// Whether the radio channel this bridge sits on actually applies a
    /// propagation delay.
    ///
    /// This matters because the SAP latency the bridge reports is published as
    /// an NTN L1/L2 turnaround. On a channel with no PropagationDelayModel the
    /// number is a valid measurement of the slot pipeline and an INVALID
    /// measurement of an NR-NTN turnaround: no part of the round trip is in
    /// it. The flag is probed from the live SpectrumChannel at InstallEnb, not
    /// declared, so porting the bridge to a backend that does carry the delay
    /// flips it without anyone remembering to.
    bool HasAirPropagationDelay() const { return m_airPropagation; }

    /// Provenance label for the sap_latency_mean_us row, so a CSV consumer can
    /// tell the two cases apart without reading the helper's source.
    const char* GetSapLatencyProvenance() const
    {
        return m_airPropagation ? "measured-with-air-propagation"
                                : "measured-no-air-propagation";
    }

    /// One-way propagation floor implied by the current gNB/UE separation, in
    /// seconds; zero when no UE position is known. A turnaround that claims to
    /// include NTN propagation cannot be below this.
    double GetOneWayPropagationFloorSec() const { return m_oneWayFloorSec; }

    // ---- Latency (seconds), SAP-measured, SFN/slot aligned --------------
    /// request->indication latency of the most recent matched slot:
    /// CRC.indication time - DL_TTI.request time. NaN until one match exists.
    double GetLastSapLatencySec() const { return m_lastLatSec; }
    double GetMeanSapLatencySec() const
    {
        return m_latCount ? m_latSumSec / m_latCount
                          : std::numeric_limits<double>::quiet_NaN();
    }
    double GetMinSapLatencySec() const { return m_latCount ? m_latMinSec : NAN; }
    double GetMaxSapLatencySec() const { return m_latCount ? m_latMaxSec : NAN; }
    /// Scheduling-pipeline latency: SLOT.indication time - DL_TTI.request time
    /// (how many real slots the alloc is produced ahead of its slot).
    double GetMeanSchedPipelineSec() const
    {
        return m_pipeCount ? m_pipeSumSec / m_pipeCount
                          : std::numeric_limits<double>::quiet_NaN();
    }
    /// True once a matched latency is finite and its DL_TTI slot equals the
    /// CRC slot (i.e. the latency is genuinely SFN/slot aligned).
    bool HasSlotAlignedLatency() const { return m_latCount > 0; }

    // Trace sources: fire once per emitted FAPI message (real producer/consumer).
    typedef void (*SlotIndTracedCallback)(SlotIndication);
    typedef void (*DlTtiTracedCallback)(DlTtiRequest);
    typedef void (*CrcIndTracedCallback)(CrcIndication);
    typedef void (*UlTtiTracedCallback)(UlTtiRequest);
    typedef void (*TxDataTracedCallback)(TxDataRequest);
    typedef void (*RachIndTracedCallback)(RachIndication);

  private:
    friend class FapiPhySapUserDecorator;
    friend class FapiPhySapProviderDecorator;

    // Called by the decorators on the real, forwarded SAP calls.
    void OnSlotIndication(const mmwave::SfnSf& snf);
    void OnSetSlotAllocInfo(const mmwave::SlotAllocInfo& info);
    void EmitUlTti(const mmwave::SlotAllocInfo& info);
    void OnSendMacPdu(Ptr<Packet> p);
    void OnRachPreamble(uint32_t raId);
    void OnUeRachPreamble(uint8_t preambleId, uint16_t rnti);

    static uint64_t SlotKey(uint32_t frame, uint8_t sf, uint8_t slot)
    {
        return (static_cast<uint64_t>(frame) << 16) |
               (static_cast<uint64_t>(sf) << 8) | slot;
    }

    /// FAPI-4: a slot can schedule several UEs, so HARQ bookkeeping needs the
    /// RNTI as well.
    ///
    /// The HARQ process id was stored once per SLOT, from whichever DL DCI came
    /// last in the TTI loop, and read back for every CRC.indication in that
    /// slot. With more than one UE scheduled, every CRC carried the last UE's
    /// HARQ pid. Worse, the per-slot entry was erased on the first match, so a
    /// second transport block in the same slot found nothing and reported pid 0.
    /// The shipped examples default to four UEs.
    static uint64_t SlotUeKey(uint32_t frame, uint8_t sf, uint8_t slot, uint16_t rnti)
    {
        return (SlotKey(frame, sf, slot) << 16) | static_cast<uint64_t>(rnti);
    }
    uint16_t FapiSlot(uint8_t sf, uint8_t slot) const
    {
        return static_cast<uint16_t>(sf * m_slotsPerSubframe + slot);
    }
    void PruneStale(uint64_t nowKeyFrame);

    void DoDispose() override;

    void ProbeAirPropagation();

    bool m_airPropagation{false};
    double m_oneWayFloorSec{0.0};
    bool m_configured{false};
    CarrierConfig m_carrier{};
    CellConfig m_cell{};
    uint32_t m_slotsPerSubframe{1};
    uint16_t m_ueRnti{0};

    Ptr<mmwave::MmWaveEnbNetDevice> m_enb;
    // Real SAP endpoints captured at InstallEnb() (owned by mmwave objects).
    mmwave::MmWaveEnbPhySapUser* m_realUser{nullptr};
    mmwave::MmWavePhySapProvider* m_realProvider{nullptr};
    // Decorators we own and install in their place.
    FapiPhySapUserDecorator* m_userDecorator{nullptr};
    FapiPhySapProviderDecorator* m_provDecorator{nullptr};

    // Per-slot timing bookkeeping, keyed by SlotKey(frame,sf,slot).
    std::map<uint64_t, double> m_dlTtiTimeSec;   //!< DL_TTI.request emit time
    std::map<uint64_t, double> m_slotIndTimeSec; //!< SLOT.indication emit time
    /// FAPI-4: real HARQ pid per (DL slot, RNTI). Was per-slot, which
    /// collapsed every UE in a slot onto one value.
    std::map<uint64_t, uint8_t> m_dlHarqId;

    SlotIndication m_lastSlot{};
    DlTtiRequest m_lastDlTti{};
    UlTtiRequest m_lastUlTti{};
    TxDataRequest m_lastTxData{};
    RachIndication m_lastRach{};

    // Slot the MAC is currently filling. SendMacPdu carries no SFN of its own,
    // so TX_DATA.request is stamped with the slot the PHY last indicated,
    // which is the slot the MAC is building for.
    mmwave::SfnSf m_currentSfn{};
    bool m_haveCurrentSfn{false};
    /// Preamble id the UE last sent, so RACH.indication reports the real one
    /// rather than a placeholder. mmwave's ReceiveRachPreamble carries only
    /// the RA id.
    uint8_t m_lastUePreambleId{0};
    bool m_haveUePreamble{false};

    Time m_taRoundTrip{Time()};
    Time m_taPreComp{Time()};
    Time m_taResidual{Time()};

    uint64_t m_slotIndCount{0};
    uint64_t m_dlTtiCount{0};
    uint64_t m_dlTtiDataCount{0};
    uint64_t m_crcCount{0};
    uint64_t m_rxDataCount{0};
    uint64_t m_ulTtiCount{0};
    uint64_t m_ulTtiGrantCount{0};
    uint64_t m_txDataCount{0};
    uint64_t m_txDataBytes{0};
    uint64_t m_rachCount{0};

    uint64_t m_latCount{0};
    double m_latSumSec{0.0};
    double m_latMinSec{std::numeric_limits<double>::infinity()};
    double m_latMaxSec{0.0};
    double m_lastLatSec{std::numeric_limits<double>::quiet_NaN()};
    uint64_t m_pipeCount{0};
    double m_pipeSumSec{0.0};

    TracedCallback<SlotIndication> m_slotIndTrace;
    TracedCallback<DlTtiRequest> m_dlTtiTrace;
    TracedCallback<CrcIndication> m_crcTrace;
    TracedCallback<UlTtiRequest> m_ulTtiTrace;
    TracedCallback<TxDataRequest> m_txDataTrace;
    TracedCallback<RachIndication> m_rachTrace;
};

} // namespace fapi
} // namespace ns3

#endif // NTN_FAPI_SAP_BRIDGE_H
