/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-fapi-sap-bridge.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnFapiSapBridge");

namespace fapi
{

// ---------------------------------------------------------------------------
// Forwarding decorators. Each implements the FULL mmwave SAP interface, passes
// every call to the real endpoint unchanged, and records the FAPI struct as a
// side effect on the two calls that carry slot timing. No Simulator scheduling
// happens here, so the real mmwave slot timing is observed, never perturbed.
// ---------------------------------------------------------------------------

/// Decorates the MAC's MmWaveEnbPhySapUser (PHY -> MAC direction).
class FapiPhySapUserDecorator : public mmwave::MmWaveEnbPhySapUser
{
  public:
    FapiPhySapUserDecorator(NtnFapiSapBridge* bridge, mmwave::MmWaveEnbPhySapUser* real)
        : m_bridge(bridge),
          m_real(real)
    {
    }

    void ReceivePhyPdu(Ptr<Packet> p) override { m_real->ReceivePhyPdu(p); }

    void ReceiveControlMessage(Ptr<mmwave::MmWaveControlMessage> msg) override
    {
        m_real->ReceiveControlMessage(msg);
    }

    void SlotIndication(mmwave::SfnSf snf) override
    {
        m_real->SlotIndication(snf);   // forward first: do not delay the real MAC
        m_bridge->OnSlotIndication(snf); // then emit the FAPI SLOT.indication
    }

    void UlCqiReport(mmwave::MmWaveMacSchedSapProvider::SchedUlCqiInfoReqParameters ulcqi) override
    {
        m_real->UlCqiReport(ulcqi);
    }

    void ReceiveRachPreamble(uint32_t raId) override
    {
        m_real->ReceiveRachPreamble(raId); // forward first: never delay the real MAC
        m_bridge->OnRachPreamble(raId);    // then emit the FAPI RACH.indication
    }

    void UlHarqFeedback(mmwave::UlHarqInfo params) override { m_real->UlHarqFeedback(params); }

  private:
    NtnFapiSapBridge* m_bridge;
    mmwave::MmWaveEnbPhySapUser* m_real;
};

/// Decorates the PHY's MmWavePhySapProvider (MAC -> PHY direction).
class FapiPhySapProviderDecorator : public mmwave::MmWavePhySapProvider
{
  public:
    FapiPhySapProviderDecorator(NtnFapiSapBridge* bridge, mmwave::MmWavePhySapProvider* real)
        : m_bridge(bridge),
          m_real(real)
    {
    }

    void SendMacPdu(Ptr<Packet> p) override
    {
        m_real->SendMacPdu(p);      // forward first
        m_bridge->OnSendMacPdu(p);  // then emit the FAPI TX_DATA.request
    }

    void SendControlMessage(Ptr<mmwave::MmWaveControlMessage> msg) override
    {
        m_real->SendControlMessage(msg);
    }

    void SendRachPreamble(uint8_t preambleId, uint8_t rnti) override
    {
        m_real->SendRachPreamble(preambleId, rnti);
        // Remember the real preamble index. mmwave's gNB-side
        // ReceiveRachPreamble carries only the RA id, so without this the
        // RACH.indication would have to invent a preambleIndex.
        m_bridge->OnUeRachPreamble(preambleId, rnti);
    }

    void SetSlotAllocInfo(mmwave::SlotAllocInfo slotAllocInfo) override
    {
        m_real->SetSlotAllocInfo(slotAllocInfo);      // forward the real alloc
        m_bridge->OnSetSlotAllocInfo(slotAllocInfo);  // emit FAPI DL_TTI.request
    }

  private:
    NtnFapiSapBridge* m_bridge;
    mmwave::MmWavePhySapProvider* m_real;
};

// ---------------------------------------------------------------------------
// NtnFapiSapBridge
// ---------------------------------------------------------------------------

TypeId
NtnFapiSapBridge::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::fapi::NtnFapiSapBridge")
            .SetParent<Object>()
            .SetGroupName("NtnFapi")
            .AddConstructor<NtnFapiSapBridge>()
            .AddTraceSource("SlotIndication",
                            "FAPI SLOT.indication emitted from the real PHY SlotIndication SAP call",
                            MakeTraceSourceAccessor(&NtnFapiSapBridge::m_slotIndTrace),
                            "ns3::fapi::NtnFapiSapBridge::SlotIndTracedCallback")
            .AddTraceSource("DlTtiRequest",
                            "FAPI DL_TTI.request emitted from the real MAC SetSlotAllocInfo SAP call",
                            MakeTraceSourceAccessor(&NtnFapiSapBridge::m_dlTtiTrace),
                            "ns3::fapi::NtnFapiSapBridge::DlTtiTracedCallback")
            .AddTraceSource("CrcIndication",
                            "FAPI CRC.indication emitted from the real PHY per-TB decode",
                            MakeTraceSourceAccessor(&NtnFapiSapBridge::m_crcTrace),
                            "ns3::fapi::NtnFapiSapBridge::CrcIndTracedCallback")
            .AddTraceSource("UlTtiRequest",
                            "FAPI UL_TTI.request emitted from the UL DCIs in the real MAC "
                            "SetSlotAllocInfo SAP call",
                            MakeTraceSourceAccessor(&NtnFapiSapBridge::m_ulTtiTrace),
                            "ns3::fapi::NtnFapiSapBridge::UlTtiTracedCallback")
            .AddTraceSource("TxDataRequest",
                            "FAPI TX_DATA.request emitted from the real MAC SendMacPdu SAP call, "
                            "carrying the actual PDU bytes",
                            MakeTraceSourceAccessor(&NtnFapiSapBridge::m_txDataTrace),
                            "ns3::fapi::NtnFapiSapBridge::TxDataTracedCallback")
            .AddTraceSource("RachIndication",
                            "FAPI RACH.indication emitted from the real PHY ReceiveRachPreamble "
                            "SAP call",
                            MakeTraceSourceAccessor(&NtnFapiSapBridge::m_rachTrace),
                            "ns3::fapi::NtnFapiSapBridge::RachIndTracedCallback");
    return tid;
}

NtnFapiSapBridge::NtnFapiSapBridge()
{
    NS_LOG_FUNCTION(this);
}

NtnFapiSapBridge::~NtnFapiSapBridge()
{
    delete m_userDecorator;
    delete m_provDecorator;
    m_userDecorator = nullptr;
    m_provDecorator = nullptr;
}

void
NtnFapiSapBridge::DoDispose()
{
    m_enb = nullptr;
    Object::DoDispose();
}

void
NtnFapiSapBridge::Configure(const CarrierConfig& carrier, const CellConfig& cell)
{
    m_carrier = carrier;
    m_cell = cell;
    m_configured = true;
    NS_LOG_INFO("FAPI CONFIG.request consumed: cellId=" << m_cell.phyCellId << " dlFreq="
                                                        << m_carrier.dlFrequency << " kHz dlBW="
                                                        << m_carrier.dlBandwidth << " MHz");
}

void
NtnFapiSapBridge::InstallEnb(Ptr<mmwave::MmWaveEnbNetDevice> enb)
{
    NS_LOG_FUNCTION(this << enb);
    NS_ABORT_MSG_IF(enb == nullptr, "NtnFapiSapBridge::InstallEnb: null enb device");
    m_enb = enb;

    Ptr<mmwave::MmWaveEnbPhy> phy = enb->GetPhy();
    Ptr<mmwave::MmWaveEnbMac> mac = enb->GetMac();
    NS_ABORT_MSG_IF(!phy || !mac, "NtnFapiSapBridge::InstallEnb: enb missing PHY/MAC");

    // ---- P5 CONFIG.request: populate from the REAL PHY configuration --------
    Ptr<mmwave::MmWavePhyMacCommon> cfg = phy->GetConfigurationParameters();
    NS_ABORT_MSG_IF(!cfg, "NtnFapiSapBridge::InstallEnb: PHY has no config");
    m_slotsPerSubframe = std::max<uint32_t>(1, cfg->GetSlotsPerSubframe());

    CarrierConfig carrier{};
    carrier.dlFrequency = static_cast<uint32_t>(cfg->GetCenterFrequency() / 1e3); // Hz -> kHz
    carrier.dlBandwidth = static_cast<uint32_t>(std::lround(cfg->GetBandwidth() / 1e6)); // Hz -> MHz
    carrier.ulFrequency = carrier.dlFrequency;
    carrier.ulBandwidth = carrier.dlBandwidth;
    carrier.numRxAnt = 1;
    carrier.numTxAnt = 1;
    for (uint32_t i = 0; i < 5; ++i)
    {
        carrier.dlGridSize[i] = static_cast<uint16_t>(cfg->GetNumRb());
        carrier.ulGridSize[i] = static_cast<uint16_t>(cfg->GetNumRb());
        carrier.dlK0[i] = 0;
        carrier.ulK0[i] = 0;
    }
    carrier.frequencyShift7p5khz = Numerology::kMu0_15kHz;

    ProbeAirPropagation();

    CellConfig cell{};
    cell.phyCellId = enb->GetCellId();
    cell.frameDuplexType = 1; // mmwave is TDD
    cell.pdschCpType = 0;     // normal CP
    Configure(carrier, cell);

    // ---- Interpose the two forwarding decorators on the real SAP -----------
    // The helper already cross-wired phy->SetPhySapUser(mac->GetPhySapUser()) and
    // mac->SetPhySapProvider(phy->GetPhySapProvider()); we capture those real
    // endpoints and slot our decorators in their place.
    m_realUser = mac->GetPhySapUser();
    m_realProvider = phy->GetPhySapProvider();
    NS_ABORT_MSG_IF(!m_realUser || !m_realProvider,
                    "NtnFapiSapBridge::InstallEnb: real SAP endpoints not wired (call after Build)");

    m_userDecorator = new FapiPhySapUserDecorator(this, m_realUser);
    m_provDecorator = new FapiPhySapProviderDecorator(this, m_realProvider);

    phy->SetPhySapUser(m_userDecorator);        // PHY now fires SlotIndication into us
    mac->SetPhySapProvider(m_provDecorator);    // MAC now calls SetSlotAllocInfo into us

    NS_LOG_INFO("FAPI SAP bridge installed on cellId=" << cell.phyCellId << " ("
                << m_slotsPerSubframe << " slots/subframe)");
}

void
NtnFapiSapBridge::OnSlotIndication(const mmwave::SfnSf& snf)
{
    // Real PHY told the MAC a slot started == FAPI SLOT.indication (0x82).
    SlotIndication si{};
    si.sfn = static_cast<uint16_t>(snf.m_frameNum & 0x3FF);
    si.slot = FapiSlot(snf.m_sfNum, snf.m_slotNum);
    m_lastSlot = si;
    ++m_slotIndCount;

    const uint64_t key = SlotKey(snf.m_frameNum, snf.m_sfNum, snf.m_slotNum);
    m_slotIndTimeSec[key] = Simulator::Now().GetSeconds();

    // Scheduling pipeline: this slot's alloc was produced ahead of the slot.
    auto it = m_dlTtiTimeSec.find(key);
    if (it != m_dlTtiTimeSec.end())
    {
        const double pipe = Simulator::Now().GetSeconds() - it->second;
        if (pipe >= 0.0)
        {
            m_pipeSumSec += pipe;
            ++m_pipeCount;
        }
    }
    m_currentSfn = snf;
    m_haveCurrentSfn = true;
    m_slotIndTrace(si);
}

void
NtnFapiSapBridge::OnSetSlotAllocInfo(const mmwave::SlotAllocInfo& info)
{
    // Real MAC handed the PHY the slot allocation == FAPI DL_TTI.request (0x80).
    const mmwave::SfnSf& sfn = info.m_sfnSf;

    DlTtiRequest req{};
    req.sfn = static_cast<uint16_t>(sfn.m_frameNum & 0x3FF);
    req.slot = FapiSlot(sfn.m_sfNum, sfn.m_slotNum);
    req.numGroups = 1;
    uint8_t nPdcch = 0;
    uint8_t nPdsch = 0;
    bool hasData = false;
    // FAPI-4: one HARQ pid PER UE in this slot, not one for the slot.
    std::vector<std::pair<uint16_t, uint8_t>> slotHarq;

    for (const auto& tti : info.m_ttiAllocInfo)
    {
        // Only DL data DCIs become PDCCH+PDSCH FAPI PDUs.
        if (tti.m_dci.m_format != mmwave::DciInfoElementTdma::DL_dci || tti.m_dci.m_tbSize == 0)
        {
            continue;
        }
        hasData = true;
        // REAL mmwave HARQ process id, recorded against the RNTI it belongs
        // to. This used to assign to a single slot-wide variable that the loop
        // overwrote, so the value that survived was whichever DL DCI came last.
        slotHarq.emplace_back(tti.m_dci.m_rnti, tti.m_dci.m_harqProcess);

        // PDCCH carrying the DL DCI.
        DlTtiPdu pdcchPdu{};
        pdcchPdu.type = DlTtiPdu::Type::kPdcch;
        PdcchPdu pdcch{};
        PdcchPdu::Dci dci{};
        dci.rnti = tti.m_dci.m_rnti;
        dci.aggregationLevel = 4;
        pdcch.dciList.push_back(dci);
        pdcchPdu.pdu = pdcch;
        req.pduList.push_back(pdcchPdu);
        ++nPdcch;

        // PDSCH carrying the transport block descriptor.
        DlTtiPdu pdschPdu{};
        pdschPdu.type = DlTtiPdu::Type::kPdsch;
        PdschPdu pdsch{};
        pdsch.rnti = tti.m_dci.m_rnti;
        pdsch.numCodewords = 1;
        pdsch.codewords[0].mcsIndex = tti.m_dci.m_mcs;
        pdsch.codewords[0].tbSize = tti.m_dci.m_tbSize; // real scheduled TB size (bytes)
        pdsch.harqProcessId = tti.m_dci.m_harqProcess;  // REAL HARQ pid (0..31 Rel-17)
        pdsch.nrOfLayers = 1;
        pdsch.startSymbolIndex = tti.m_dci.m_symStart;
        pdsch.nrOfSymbols = tti.m_dci.m_numSym;
        pdschPdu.pdu = pdsch;
        req.pduList.push_back(pdschPdu);
        ++nPdsch;
    }

    req.nPdusOfEachType[0] = nPdcch;
    req.nPdusOfEachType[1] = nPdsch;
    req.nPdusOfEachType[2] = 0;
    req.nPdusOfEachType[3] = 0;

    // ---- UL_TTI.request (0x81) ------------------------------------------
    // The same SlotAllocInfo carries the UL grants; the DL loop above skips
    // them. Emitting from here means the uplink half of the boundary comes
    // from the identical real MAC decision the downlink half does, at the
    // identical instant, rather than from a parallel bookkeeping path.
    EmitUlTti(info);

    ++m_dlTtiCount;
    if (!hasData)
    {
        return; // DL control-only slot: no DL_TTI data payload to time
    }

    m_lastDlTti = req;
    ++m_dlTtiDataCount;

    const uint64_t key = SlotKey(sfn.m_frameNum, sfn.m_sfNum, sfn.m_slotNum);
    m_dlTtiTimeSec[key] = Simulator::Now().GetSeconds();
    // FAPI-4: store per (slot, RNTI) so each UE's CRC.indication can find its
    // own process id.
    for (const auto& [rnti, pid] : slotHarq)
    {
        m_dlHarqId[SlotUeKey(sfn.m_frameNum, sfn.m_sfNum, sfn.m_slotNum, rnti)] = pid;
    }
    m_dlTtiTrace(req);
}

void
NtnFapiSapBridge::OnPhyRx(const mmwave::RxPacketTraceParams& p)
{
    // Reverse path: real per-TB decode outcome -> FAPI RX_DATA + CRC.indication.
    if (p.m_tbSize == 0)
    {
        return; // control TB, not a data TB
    }
    if (m_ueRnti != 0 && p.m_rnti != m_ueRnti)
    {
        return; // other UE
    }

    const uint64_t key = SlotKey(p.m_frameNum, p.m_sfNum, p.m_slotNum);
    // FAPI-4: look the HARQ pid up by (slot, RNTI). The per-slot lookup handed
    // every UE the same id and, because the entry was erased on the first
    // match, gave the second transport block in a slot no id at all.
    const uint64_t ueKey = SlotUeKey(p.m_frameNum, p.m_sfNum, p.m_slotNum, p.m_rnti);
    const bool crcOk = !p.m_corrupt; // real error-model decode, no local RNG
    const double sinrDb = 10.0 * std::log10(std::max(p.m_sinr, 1e-12));
    uint16_t harqId = 0;
    auto hit = m_dlHarqId.find(ueKey);
    if (hit != m_dlHarqId.end())
    {
        harqId = hit->second; // real HARQ pid the DL_TTI.request carried
    }

    // RX_DATA.indication (0x85) — only when the TB decoded.
    if (crcOk)
    {
        RxDataIndication rx{};
        rx.sfn = static_cast<uint16_t>(p.m_frameNum & 0x3FF);
        rx.slot = FapiSlot(p.m_sfNum, p.m_slotNum);
        RxDataIndication::PduRx prx{};
        prx.handle = static_cast<uint32_t>(key);
        prx.rnti = p.m_rnti;
        prx.harqId = harqId;
        prx.pduLength = static_cast<uint16_t>(p.m_tbSize);
        rx.pdus.push_back(prx);
        ++m_rxDataCount;
    }

    // CRC.indication (0x86) — always.
    CrcIndication crc{};
    crc.sfn = static_cast<uint16_t>(p.m_frameNum & 0x3FF);
    crc.slot = FapiSlot(p.m_sfNum, p.m_slotNum);
    CrcIndication::CrcReport rep{};
    rep.handle = static_cast<uint32_t>(key);
    rep.rnti = p.m_rnti;
    rep.harqId = harqId;
    rep.tbCrcStatusOk = crcOk;
    rep.ul_cqi = static_cast<int16_t>(std::lround(std::max(-10.0, sinrDb)));
    crc.crcList.push_back(rep);
    ++m_crcCount;
    m_crcTrace(crc);

    // ---- SAP latency (CI gate 15): DL_TTI.request -> CRC.indication --------
    // Aligned by SFN/slot: the DL_TTI.request that scheduled this exact slot.
    auto it = m_dlTtiTimeSec.find(key);
    if (it != m_dlTtiTimeSec.end())
    {
        const double lat = Simulator::Now().GetSeconds() - it->second;
        if (std::isfinite(lat) && lat >= 0.0)
        {
            m_lastLatSec = lat;
            m_latSumSec += lat;
            m_latMinSec = std::min(m_latMinSec, lat);
            m_latMaxSec = std::max(m_latMaxSec, lat);
            ++m_latCount;
        }
        // One-shot: this slot's alloc has been consumed end-to-end.
        m_dlTtiTimeSec.erase(it);
        // FAPI-4: erase only THIS UE's entry, so another UE's transport block
        // in the same slot still finds its own.
        m_dlHarqId.erase(ueKey);
        m_slotIndTimeSec.erase(key);
    }

    PruneStale(p.m_frameNum);
}

void
NtnFapiSapBridge::PruneStale(uint64_t nowFrame)
{
    // Bound memory: drop slot-timing entries older than 64 frames (~640 ms),
    // far beyond any real DL scheduling pipeline / decode horizon.
    if (nowFrame < 64)
    {
        return;
    }
    const uint64_t cutoff = SlotKey(static_cast<uint32_t>(nowFrame - 64), 0, 0);
    // FAPI-4: m_dlHarqId is now keyed by SlotUeKey, which is SlotKey shifted
    // left 16, so its cutoff must be shifted the same way or the sweep would
    // erase the whole map on the first call.
    const uint64_t harqCutoff = cutoff << 16;
    for (auto* m : {&m_dlTtiTimeSec, &m_slotIndTimeSec})
    {
        for (auto it = m->begin(); it != m->end() && it->first < cutoff;)
        {
            it = m->erase(it);
        }
    }
    for (auto it = m_dlHarqId.begin(); it != m_dlHarqId.end() && it->first < harqCutoff;)
    {
        it = m_dlHarqId.erase(it);
    }
}

// ---------------------------------------------------------------------------
// Uplink half of the L1/L2 boundary (FAPI-1).
//
// Everything below is emitted from a real forwarded SAP call. Nothing here
// schedules, delays or re-derives: if the MAC did not grant an uplink slot,
// no UL_TTI.request appears, and if no preamble arrived, no RACH.indication
// does.
// ---------------------------------------------------------------------------

void
NtnFapiSapBridge::EmitUlTti(const mmwave::SlotAllocInfo& info)
{
    const mmwave::SfnSf& sfn = info.m_sfnSf;

    UlTtiRequest req{};
    req.sfn = static_cast<uint16_t>(sfn.m_frameNum & 0x3FF);
    req.slot = FapiSlot(sfn.m_sfNum, sfn.m_slotNum);
    uint8_t nPusch = 0;

    for (const auto& tti : info.m_ttiAllocInfo)
    {
        if (tti.m_dci.m_format != mmwave::DciInfoElementTdma::UL_dci || tti.m_dci.m_tbSize == 0)
        {
            continue;
        }

        UlTtiPdu pdu{};
        pdu.type = UlTtiPdu::Type::kPusch;
        PuschPdu pusch{};
        pusch.rnti = tti.m_dci.m_rnti;
        pusch.handle = static_cast<uint32_t>(SlotKey(sfn.m_frameNum, sfn.m_sfNum, sfn.m_slotNum));
        pusch.mcsIndex = tti.m_dci.m_mcs;   // real scheduled MCS
        pusch.mcsTable = 0;                 // TS 38.214 Table 6.1.4.1-1 (qam64)
        pusch.qamModOrder = 0;              // derived by the PHY from mcsIndex/table
        pusch.nrOfLayers = 1;               // mmwave UL is single-layer
        pusch.startSymbolIndex = tti.m_dci.m_symStart;
        pusch.nrOfSymbols = tti.m_dci.m_numSym;
        pusch.tbSize = tti.m_dci.m_tbSize;  // real granted TB size
        pusch.rvIndex = tti.m_dci.m_rv;     // real redundancy version
        pusch.harqProcessId = tti.m_dci.m_harqProcess; // real HARQ pid
        pusch.transformPrecoding = 1;       // disabled (CP-OFDM), TS 38.214 6.1.3
        pusch.puschIdentity = m_cell.phyCellId;
        pdu.pdu = pusch;
        req.pduList.push_back(pdu);
        ++nPusch;
    }

    req.nPdusOfEachType[0] = 0;      // PRACH: carried by RACH.indication instead
    req.nPdusOfEachType[1] = nPusch; // PUSCH
    req.nPdusOfEachType[2] = 0;      // PUCCH
    req.nPdusOfEachType[3] = 0;      // SRS

    ++m_ulTtiCount;
    if (nPusch == 0)
    {
        return; // no uplink grant in this slot: an empty UL_TTI is not a grant
    }
    ++m_ulTtiGrantCount;
    m_lastUlTti = req;
    m_ulTtiTrace(req);
}

void
NtnFapiSapBridge::OnSendMacPdu(Ptr<Packet> p)
{
    // TX_DATA.request (0x84). SCF FAPI 222.10.02 section 3.4.2.3: this is the
    // message that carries the transport block itself, as opposed to the
    // descriptor DL_TTI.request carries. The bytes below are the real MAC PDU
    // the PHY was just handed, copied out of the packet, not filler.
    if (!p || p->GetSize() == 0)
    {
        return;
    }

    TxDataRequest req{};
    if (m_haveCurrentSfn)
    {
        req.sfn = static_cast<uint16_t>(m_currentSfn.m_frameNum & 0x3FF);
        req.slot = FapiSlot(m_currentSfn.m_sfNum, m_currentSfn.m_slotNum);
    }

    TxDataRequest::PduPayload pdu{};
    pdu.pduIndex = static_cast<uint16_t>(m_txDataCount & 0xFFFF);
    pdu.cwIndex = 0;
    pdu.tbBytes.resize(p->GetSize());
    p->CopyData(pdu.tbBytes.data(), pdu.tbBytes.size());
    req.pdus.push_back(std::move(pdu));

    ++m_txDataCount;
    m_txDataBytes += p->GetSize();
    m_lastTxData = req;
    m_txDataTrace(req);
}

void
NtnFapiSapBridge::OnUeRachPreamble(uint8_t preambleId, uint16_t rnti)
{
    m_lastUePreambleId = preambleId;
    m_haveUePreamble = true;
    NS_LOG_LOGIC("UE sent PRACH preamble " << +preambleId << " rnti " << rnti);
}

void
NtnFapiSapBridge::OnRachPreamble(uint32_t raId)
{
    // RACH.indication (0x89). The gNB PHY detected a preamble; FAPI reports it
    // upward together with the timing advance the gNB measured on it.
    RachIndication ind{};
    if (m_haveCurrentSfn)
    {
        ind.sfn = static_cast<uint16_t>(m_currentSfn.m_frameNum & 0x3FF);
        ind.slot = FapiSlot(m_currentSfn.m_sfNum, m_currentSfn.m_slotNum);
    }

    RachIndication::Preamble pre{};
    // Prefer the index the UE actually transmitted; fall back to the RA id,
    // which is what mmwave gives the gNB when no UE-side call was seen.
    pre.preambleIndex = m_haveUePreamble ? m_lastUePreambleId
                                         : static_cast<uint8_t>(raId & 0x3F);
    pre.symbolIndex = 0;
    pre.slotIndex = static_cast<uint8_t>(ind.slot & 0xFF);
    pre.freqIndex = 0;
    // The residual, not the round trip: see SetNtnTimingAdvance.
    const uint32_t taTc = GetResidualTaInTc();
    pre.timingAdvance = static_cast<uint16_t>(std::min<uint32_t>(taTc, 0xFFFF));
    pre.preamblePower = 0;
    ind.preambles.push_back(pre);

    ++m_rachCount;
    m_lastRach = ind;
    m_rachTrace(ind);
}

void
NtnFapiSapBridge::SetNtnTimingAdvance(Time roundTrip, Time preCompensated)
{
    m_taRoundTrip = roundTrip;
    m_taPreComp = preCompensated;
    // A UE that over-compensates still lands inside the window, so clamp at
    // zero rather than reporting a negative advance.
    const int64_t residualNs = roundTrip.GetNanoSeconds() - preCompensated.GetNanoSeconds();
    m_taResidual = NanoSeconds(std::max<int64_t>(residualNs, 0));
    NS_LOG_INFO("NTN TA: round trip " << roundTrip.GetMilliSeconds() << " ms, pre-compensated "
                                      << preCompensated.GetMilliSeconds() << " ms, residual "
                                      << m_taResidual.GetMilliSeconds() << " ms");
}

uint32_t
NtnFapiSapBridge::GetResidualTaInTc() const
{
    // TS 38.211 section 4.1: T_c = 1 / (delta_f_max * N_f) with
    // delta_f_max = 480 kHz and N_f = 4096, i.e. about 0.509 ns.
    constexpr double kTcSeconds = 1.0 / (480e3 * 4096.0);
    const double tc = m_taResidual.GetSeconds() / kTcSeconds;
    if (tc <= 0.0)
    {
        return 0;
    }
    return static_cast<uint32_t>(std::llround(std::min(tc, 4.0e9)));
}

void
NtnFapiSapBridge::ProbeAirPropagation()
{
    // FAPI-2. Ask the channel, do not assume. The answer decides whether the
    // SAP latency this bridge reports may be described as an NTN turnaround.
    m_airPropagation = false;
    m_oneWayFloorSec = 0.0;
    if (!m_enb)
    {
        return;
    }

    Ptr<mmwave::MmWaveEnbPhy> phy = m_enb->GetPhy();
    if (!phy)
    {
        return;
    }
    Ptr<mmwave::MmWaveSpectrumPhy> sp = phy->GetDlSpectrumPhy();
    if (!sp)
    {
        return;
    }
    Ptr<SpectrumChannel> ch = sp->GetSpectrumChannel();
    if (ch && ch->GetPropagationDelayModel())
    {
        m_airPropagation = true;
    }

    // Geometry floor: straight-line gNB-to-UE separation over c. Used by the
    // gate to check the reported latency against the physics of the link
    // rather than against a fixed constant.
    Ptr<MobilityModel> enbMob = m_enb->GetNode() ? m_enb->GetNode()->GetObject<MobilityModel>()
                                                 : nullptr;
    if (!enbMob)
    {
        return;
    }
    double nearest = std::numeric_limits<double>::infinity();
    for (uint32_t i = 0; i < NodeList::GetNNodes(); ++i)
    {
        Ptr<Node> n = NodeList::GetNode(i);
        if (!n || n == m_enb->GetNode())
        {
            continue;
        }
        Ptr<MobilityModel> m = n->GetObject<MobilityModel>();
        if (!m)
        {
            continue;
        }
        // Only nodes carrying a UE net device count as the other end of the
        // air interface; a remote host or an EPC node would not.
        bool isUe = false;
        for (uint32_t d = 0; d < n->GetNDevices(); ++d)
        {
            if (DynamicCast<mmwave::MmWaveUeNetDevice>(n->GetDevice(d)))
            {
                isUe = true;
                break;
            }
        }
        if (!isUe)
        {
            continue;
        }
        nearest = std::min(nearest, enbMob->GetDistanceFrom(m));
    }
    if (std::isfinite(nearest))
    {
        m_oneWayFloorSec = nearest / 299792458.0;
    }
    NS_LOG_INFO("air propagation " << (m_airPropagation ? "PRESENT" : "ABSENT")
                                   << ", one-way floor " << m_oneWayFloorSec * 1e3 << " ms");
}

} // namespace fapi
} // namespace ns3
