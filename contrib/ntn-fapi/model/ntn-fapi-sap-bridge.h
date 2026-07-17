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

    // ---- Inspection / counters ------------------------------------------
    uint64_t GetSlotIndicationCount() const { return m_slotIndCount; }
    uint64_t GetDlTtiRequestCount() const { return m_dlTtiCount; }
    uint64_t GetDlTtiWithDataCount() const { return m_dlTtiDataCount; }
    uint64_t GetCrcIndicationCount() const { return m_crcCount; }
    uint64_t GetRxDataIndicationCount() const { return m_rxDataCount; }
    uint64_t GetMatchedLatencyCount() const { return m_latCount; }

    const SlotIndication& LastSlotIndication() const { return m_lastSlot; }
    const DlTtiRequest& LastDlTti() const { return m_lastDlTti; }

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

  private:
    friend class FapiPhySapUserDecorator;
    friend class FapiPhySapProviderDecorator;

    // Called by the decorators on the real, forwarded SAP calls.
    void OnSlotIndication(const mmwave::SfnSf& snf);
    void OnSetSlotAllocInfo(const mmwave::SlotAllocInfo& info);

    static uint64_t SlotKey(uint32_t frame, uint8_t sf, uint8_t slot)
    {
        return (static_cast<uint64_t>(frame) << 16) |
               (static_cast<uint64_t>(sf) << 8) | slot;
    }
    uint16_t FapiSlot(uint8_t sf, uint8_t slot) const
    {
        return static_cast<uint16_t>(sf * m_slotsPerSubframe + slot);
    }
    void PruneStale(uint64_t nowKeyFrame);

    void DoDispose() override;

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
    std::map<uint64_t, uint8_t> m_dlHarqId;      //!< real HARQ pid per DL slot

    SlotIndication m_lastSlot{};
    DlTtiRequest m_lastDlTti{};

    uint64_t m_slotIndCount{0};
    uint64_t m_dlTtiCount{0};
    uint64_t m_dlTtiDataCount{0};
    uint64_t m_crcCount{0};
    uint64_t m_rxDataCount{0};

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
};

} // namespace fapi
} // namespace ns3

#endif // NTN_FAPI_SAP_BRIDGE_H
