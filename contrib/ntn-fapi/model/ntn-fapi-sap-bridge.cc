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

    void ReceiveRachPreamble(uint32_t raId) override { m_real->ReceiveRachPreamble(raId); }

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

    void SendMacPdu(Ptr<Packet> p) override { m_real->SendMacPdu(p); }

    void SendControlMessage(Ptr<mmwave::MmWaveControlMessage> msg) override
    {
        m_real->SendControlMessage(msg);
    }

    void SendRachPreamble(uint8_t preambleId, uint8_t rnti) override
    {
        m_real->SendRachPreamble(preambleId, rnti);
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
                            "ns3::fapi::NtnFapiSapBridge::CrcIndTracedCallback");
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
    uint8_t harqPid = 0;

    for (const auto& tti : info.m_ttiAllocInfo)
    {
        // Only DL data DCIs become PDCCH+PDSCH FAPI PDUs.
        if (tti.m_dci.m_format != mmwave::DciInfoElementTdma::DL_dci || tti.m_dci.m_tbSize == 0)
        {
            continue;
        }
        hasData = true;
        harqPid = tti.m_dci.m_harqProcess; // REAL mmwave HARQ process id

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

    ++m_dlTtiCount;
    if (!hasData)
    {
        return; // DL control-only slot: no DL_TTI data payload to time
    }

    m_lastDlTti = req;
    ++m_dlTtiDataCount;

    const uint64_t key = SlotKey(sfn.m_frameNum, sfn.m_sfNum, sfn.m_slotNum);
    m_dlTtiTimeSec[key] = Simulator::Now().GetSeconds();
    m_dlHarqId[key] = harqPid;
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
    const bool crcOk = !p.m_corrupt; // real error-model decode, no local RNG
    const double sinrDb = 10.0 * std::log10(std::max(p.m_sinr, 1e-12));
    uint16_t harqId = 0;
    auto hit = m_dlHarqId.find(key);
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
        m_dlHarqId.erase(key);
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
    for (auto* m : {&m_dlTtiTimeSec, &m_slotIndTimeSec})
    {
        for (auto it = m->begin(); it != m->end() && it->first < cutoff;)
        {
            it = m->erase(it);
        }
    }
    for (auto it = m_dlHarqId.begin(); it != m_dlHarqId.end() && it->first < cutoff;)
    {
        it = m_dlHarqId.erase(it);
    }
}

} // namespace fapi
} // namespace ns3
