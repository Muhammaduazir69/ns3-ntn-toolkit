// Copyright (c) 2026 Yifeng He
// SPDX-License-Identifier: GPL-2.0-only
#include "nr-rlc-test-utils.h"

#include "ns3/log.h"
#include "ns3/nr-amc.h"
#include "ns3/nr-mac-header-fs-ul.h"
#include "ns3/nr-mac-header-vs.h"
#include "ns3/nr-mac-scheduler-ofdma-rr.h"
#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/nr-mac-short-bsr-ce.h"
#include "ns3/nr-phy-sap.h"
#include "ns3/nr-radio-bearer-tag.h"
#include "ns3/nr-rlc-am-header.h"
#include "ns3/nr-rlc-am.h"
#include "ns3/nr-rlc-tag.h"
#include "ns3/nr-ue-mac.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

/**
 * @file
 * @ingroup nr-test
 *
 * UE MAC STATUS PDU admission tests: the STATUS MAC subheader and the SHORT_BSR
 * must fit in the UL grant, and a deferred STATUS must eventually be transmitted.
 */

namespace ns3
{
NS_LOG_COMPONENT_DEFINE("NrTestUeMacStatus");

namespace
{
constexpr uint16_t RNTI = 1;                   ///< UE RNTI
constexpr uint8_t FIRST_LCID = 3;              ///< LCID of the first AM bearer
constexpr uint32_t SHORT_BSR_BYTES = 5;        ///< SHORT_BSR CE with its subheader
constexpr uint32_t AM_STATUS_BYTES = 4;        ///< STATUS size advertised by legacy RLC AM
constexpr uint32_t AM_ACK_MAC_BYTES = 4;       ///< ACK-only STATUS emitted with its subheader
constexpr uint32_t AM_NACK_MAC_BYTES = 6;      ///< NACK-bearing STATUS emitted with its subheader
constexpr uint32_t STATUS_SUBHEADER_BYTES = 2; ///< Subheader of a STATUS below 128 B
constexpr uint32_t NUM_HARQ = 20;              ///< UE HARQ processes
constexpr uint32_t MAX_SCHED_SLOTS = 8;        ///< Slots given to the scheduler to serve STATUS

/**
 * @ingroup nr-test
 *
 * CSCHED SAP user that ignores all confirmations.
 */
class NullCschedSap : public NrMacCschedSapUser
{
  public:
    void CschedCellConfigCnf(const CschedCellConfigCnfParameters&) override
    {
    }

    void CschedUeConfigCnf(const CschedUeConfigCnfParameters&) override
    {
    }

    void CschedLcConfigCnf(const CschedLcConfigCnfParameters&) override
    {
    }

    void CschedLcReleaseCnf(const CschedLcReleaseCnfParameters&) override
    {
    }

    void CschedUeReleaseCnf(const CschedUeReleaseCnfParameters&) override
    {
    }

    void CschedUeConfigUpdateInd(const CschedUeConfigUpdateIndParameters&) override
    {
    }

    void CschedCellConfigUpdateInd(const CschedCellConfigUpdateIndParameters&) override
    {
    }
};

/**
 * @ingroup nr-test
 *
 * SCHED SAP user that collects the new-data UL DCIs of each scheduling decision.
 */
class DciCollectorSchedSap : public NrMacSchedSapUser
{
  public:
    std::vector<std::shared_ptr<DciInfoElementTdma>> dcis; ///< Collected UL DCIs

    void SchedConfigInd(const SchedConfigIndParameters& params) override
    {
        for (const auto& alloc : params.m_slotAllocInfo.m_varTtiAllocInfo)
        {
            if (alloc.m_dci->m_format == DciInfoElementTdma::UL &&
                alloc.m_dci->m_type == DciInfoElementTdma::DATA && alloc.m_dci->m_ndi == 1)
            {
                dcis.push_back(alloc.m_dci);
            }
        }
    }

    Ptr<const SpectrumModel> GetSpectrumModel() const override
    {
        return nullptr;
    }

    uint32_t GetNumRbPerRbg() const override
    {
        return 1;
    }

    uint8_t GetNumHarqProcess() const override
    {
        return NUM_HARQ;
    }

    uint16_t GetBwpId() const override
    {
        return 0;
    }

    uint16_t GetCellId() const override
    {
        return 0;
    }

    uint32_t GetSymbolsPerSlot() const override
    {
        return 14;
    }

    Time GetSlotPeriod() const override
    {
        return MilliSeconds(1);
    }

    void BuildRarList(SlotAllocInfo&) override
    {
    }
};

/**
 * @ingroup nr-test
 *
 * PHY SAP that records the MAC sub-PDUs and forwards SR and SHORT_BSR to an
 * optional gNB scheduler.
 */
class StatusPhy : public NrPhySapProvider
{
  public:
    std::vector<Ptr<Packet>> packets; ///< MAC sub-PDUs in transmission order
    Ptr<NrMacSchedulerNs3> scheduler; ///< Optional gNB scheduler
    SfnSf now{0, 1, 0, 0};            ///< Current slot

    void SendMacPdu(const Ptr<Packet>& p, const SfnSf&, uint8_t, uint16_t) override
    {
        packets.push_back(p->Copy());
        NrRadioBearerTag tag;
        p->PeekPacketTag(tag);
        if (!scheduler || tag.GetLcid() != NrMacHeaderFsUl::SHORT_BSR)
        {
            return;
        }
        NrMacShortBsrCe bsr;
        p->PeekHeader(bsr);
        MacCeElement ce{};
        ce.m_rnti = tag.GetRnti();
        ce.m_macCeType = MacCeElement::BSR;
        ce.m_macCeValue.m_bufferStatus = {bsr.m_bufferSizeLevel_0,
                                          bsr.m_bufferSizeLevel_1,
                                          bsr.m_bufferSizeLevel_2,
                                          bsr.m_bufferSizeLevel_3};
        NrMacSchedSapProvider::SchedUlMacCtrlInfoReqParameters req{};
        req.m_sfnSf = now;
        req.m_macCeList.push_back(ce);
        scheduler->GetMacSchedSapProvider()->SchedUlMacCtrlInfoReq(req);
    }

    void SendControlMessage(Ptr<NrControlMessage> msg) override
    {
        if (scheduler && msg->GetMessageType() == NrControlMessage::SR)
        {
            NrMacSchedSapProvider::SchedUlSrInfoReqParameters sr{};
            sr.m_snfSf = now;
            sr.m_srList.push_back(DynamicCast<NrSRMessage>(msg)->GetRNTI());
            scheduler->GetMacSchedSapProvider()->SchedUlSrInfoReq(sr);
        }
    }

    void SendRachPreamble(uint8_t, uint8_t) override
    {
    }

    void SetSlotAllocInfo(const SlotAllocInfo&) override
    {
    }

    void NotifyConnectionSuccessful() override
    {
    }

    BeamId GetBeamId(uint8_t) const override
    {
        return BeamId(0, 0);
    }

    Ptr<const SpectrumModel> GetSpectrumModel() override
    {
        return nullptr;
    }

    uint16_t GetBwpId() const override
    {
        return 0;
    }

    uint16_t GetCellId() const override
    {
        return 1;
    }

    uint32_t GetSymbolsPerSlot() const override
    {
        return 14;
    }

    Time GetSlotPeriod() const override
    {
        return MilliSeconds(1);
    }

    uint32_t GetRbNum() const override
    {
        return 1;
    }

    void RegisterToGnb(uint16_t) override
    {
    }

    uint32_t GetArfcn() const override
    {
        return 0;
    }
};

/**
 * @ingroup nr-test
 *
 * MAC SAP provider that forwards RLC requests to the UE MAC and records them.
 */
class ForwardingMacSap : public NrRlcTestCaptureMac
{
  public:
    NrMacSapProvider* forward{nullptr}; ///< UE MAC SAP

    void BufferStatusReport(BufferStatusReportParameters params) override
    {
        forward->BufferStatusReport(params);
    }

    void TransmitPdu(TransmitPduParameters params) override
    {
        m_pdus.push_back(params.pdu->Copy());
        forward->TransmitPdu(params);
    }
};

/**
 * @ingroup nr-test
 *
 * Real UE MAC wired to real RLC AM entities, a recording PHY and, optionally,
 * a real gNB scheduler. Owns the objects and disposes them on destruction.
 */
class StatusBench
{
  public:
    /**
     * Constructor.
     *
     * @param lcs Number of AM logical channels.
     * @param loss Whether the received AMD PDU leaves a gap, so STATUS carries a NACK.
     * @param scheduler Empty for injected grants, otherwise "TDMA" or "OFDMA".
     */
    StatusBench(uint32_t lcs, bool loss, const std::string& scheduler)
        : m_saps(lcs),
          m_sinks(lcs)
    {
        if (!scheduler.empty())
        {
            InstallScheduler(scheduler);
        }
        mac = CreateObject<NrUeMac>();
        mac->SetPhySapProvider(&phy);
        mac->SetNumHarqProcess(NUM_HARQ);
        mac->GetUeCmacSapProvider()->SetRnti(RNTI);
        mac->TraceConnectWithoutContext("UeMacStateMachineTrace",
                                        MakeCallback(&StatusBench::State, this));
        for (uint32_t i = 0; i < lcs; ++i)
        {
            const uint8_t lcid = FIRST_LCID + i;
            auto rlc = CreateObject<NrRlcAm>();
            rlc->SetRnti(RNTI);
            rlc->SetLcId(lcid);
            m_saps[i].forward = mac->GetUeMacSapProvider();
            rlc->SetNrMacSapProvider(&m_saps[i]);
            rlc->SetNrRlcSapUser(m_sinks[i].GetSapUser());
            rlc->SetAttribute("ReorderingTimer", TimeValue(MilliSeconds(1)));
            mac->GetUeCmacSapProvider()->AddLc(lcid, {}, rlc->GetNrMacSapUser());
            if (phy.scheduler)
            {
                NrMacCschedSapProvider::CschedLcConfigReqParameters req{};
                req.m_rnti = RNTI;
                nr::LogicalChannelConfigListElement_s cfg{};
                cfg.m_logicalChannelIdentity = lcid;
                cfg.m_logicalChannelGroup = 0;
                cfg.m_direction = nr::LogicalChannelConfigListElement_s::DIR_BOTH;
                cfg.m_fiveQi = 9;
                req.m_logicalChannelConfigList.push_back(cfg);
                phy.scheduler->GetMacCschedSapProvider()->CschedLcConfigReq(req);
            }
            // A missing SN 0 triggers a NACK STATUS after reordering, a polled SN 0 an ACK.
            DeliverAmd(rlc, lcid, loss ? 3 : 0, !loss);
            m_rlcs.push_back(rlc);
        }
        // Let the reordering timer expire, so every entity reports a pending STATUS.
        Simulator::Stop(MilliSeconds(2));
        Simulator::Run();
    }

    ~StatusBench()
    {
        Simulator::Destroy();
        for (auto& rlc : m_rlcs)
        {
            rlc->Dispose();
        }
        mac->Dispose();
        if (phy.scheduler)
        {
            phy.scheduler->Dispose();
        }
    }

    /**
     * Start a new slot at the UE MAC.
     */
    void Slot()
    {
        phy.now.Add(1);
        mac->GetPhySapUser()->SlotIndication(phy.now);
    }

    /**
     * Deliver an UL grant to the UE MAC.
     *
     * @param dci Grant to deliver.
     */
    void Grant(std::shared_ptr<DciInfoElementTdma> dci)
    {
        auto msg = Create<NrUlDciMessage>(dci);
        msg->SetKDelay(0);
        mac->GetPhySapUser()->ReceiveControlMessage(msg);
    }

    /**
     * Deliver an injected UL grant to the UE MAC.
     *
     * @param tbs Transport block size of the grant.
     */
    void Grant(uint32_t tbs)
    {
        Grant(std::make_shared<DciInfoElementTdma>(RNTI,
                                                   DciInfoElementTdma::UL,
                                                   0,
                                                   1,
                                                   0,
                                                   1,
                                                   nullptr,
                                                   tbs,
                                                   1,
                                                   0,
                                                   DciInfoElementTdma::DATA,
                                                   0,
                                                   0));
    }

    /**
     * Run one UL scheduling decision of the gNB scheduler and deliver its grant.
     *
     * @return The TBS of the delivered grant, or 0 if no grant was scheduled.
     */
    uint32_t Schedule()
    {
        m_sched.dcis.clear();
        NrMacSchedSapProvider::SchedUlTriggerReqParameters req{};
        req.m_snfSf = phy.now;
        req.m_slotType = LteNrTddSlotType::UL;
        phy.scheduler->GetMacSchedSapProvider()->SchedUlTriggerReq(req);
        if (m_sched.dcis.empty())
        {
            return 0;
        }
        Grant(m_sched.dcis.front());
        return m_sched.dcis.front()->m_tbSize;
    }

    /**
     * @return Number of STATUS PDUs transmitted by all RLC entities.
     */
    uint32_t StatusSent() const
    {
        uint32_t sent = 0;
        for (const auto& sap : m_saps)
        {
            sent += sap.m_pdus.size();
        }
        return sent;
    }

    /**
     * @param from Index of the first MAC sub-PDU to count.
     * @return Bytes of the MAC sub-PDUs transmitted since @p from.
     */
    uint32_t BytesSince(std::size_t from) const
    {
        uint32_t bytes = 0;
        for (std::size_t i = from; i < phy.packets.size(); ++i)
        {
            bytes += phy.packets[i]->GetSize();
        }
        return bytes;
    }

    StatusPhy phy;             ///< Recording PHY
    Ptr<NrUeMac> mac;          ///< UE MAC under test
    uint32_t pendingStatus{0}; ///< STATUS bytes cached by the MAC after the last UL DCI
    uint32_t pendingData{0};   ///< New-data bytes cached by the MAC after the last UL DCI

  private:
    /**
     * Install a gNB scheduler that serves the UE.
     *
     * @param type "TDMA" or "OFDMA".
     */
    void InstallScheduler(const std::string& type)
    {
        phy.scheduler = type == "TDMA"
                            ? StaticCast<NrMacSchedulerNs3>(CreateObject<NrMacSchedulerTdmaRR>())
                            : StaticCast<NrMacSchedulerNs3>(CreateObject<NrMacSchedulerOfdmaRR>());
        phy.scheduler->InstallDlAmc(CreateObject<NrAmc>());
        phy.scheduler->InstallUlAmc(CreateObject<NrAmc>());
        phy.scheduler->SetMacSchedSapUser(&m_sched);
        phy.scheduler->SetMacCschedSapUser(&m_csched);
        NrMacCschedSapProvider::CschedCellConfigReqParameters cell{};
        cell.m_dlBandwidth = 10;
        cell.m_ulBandwidth = 10;
        phy.scheduler->GetMacCschedSapProvider()->CschedCellConfigReq(cell);
        NrMacCschedSapProvider::CschedUeConfigReqParameters ue{};
        ue.m_rnti = RNTI;
        ue.m_beamId = BeamId(0, 0);
        phy.scheduler->GetMacCschedSapProvider()->CschedUeConfigReq(ue);
        phy.scheduler->SetDlCtrlSyms(1);
        phy.scheduler->SetUlCtrlSyms(1);
    }

    /**
     * Deliver one AMD PDU to an RLC AM entity.
     *
     * @param rlc Receiving RLC entity.
     * @param lcid Logical channel of the entity.
     * @param sn Sequence number of the PDU.
     * @param poll Whether the PDU polls for STATUS.
     */
    static void DeliverAmd(Ptr<NrRlcAm> rlc, uint8_t lcid, uint16_t sn, bool poll)
    {
        auto p = Create<Packet>(20);
        NrRlcAmHeader h;
        h.SetDataPdu();
        h.SetSequenceNumber(nr::SequenceNumber10(sn));
        h.SetResegmentationFlag(NrRlcAmHeader::PDU);
        h.SetLastSegmentFlag(NrRlcAmHeader::LAST_PDU_SEGMENT);
        h.SetSegmentOffset(0);
        h.SetFramingInfo(0);
        h.SetPollingBit(poll ? NrRlcAmHeader::STATUS_REPORT_IS_REQUESTED
                             : NrRlcAmHeader::STATUS_REPORT_NOT_REQUESTED);
        h.PushExtensionBit(NrRlcAmHeader::DATA_FIELD_FOLLOWS);
        p->AddHeader(h);
        NrRlcTag tag(Simulator::Now());
        p->AddByteTag(tag, 1, h.GetSerializedSize());
        rlc->GetNrMacSapUser()->ReceivePdu(NrMacSapUser::ReceivePduParameters(p, RNTI, lcid));
    }

    /**
     * Record the BSR cached by the UE MAC after each UL DCI.
     *
     * @param bsr Cached BSRs, per LCID.
     * @param where Trace source location.
     */
    void State(uint64_t,
               SfnSf,
               uint16_t,
               uint16_t,
               uint8_t,
               NrUeMac::SrBsrMachine,
               std::unordered_map<uint8_t, NrMacSapProvider::BufferStatusReportParameters> bsr,
               int,
               std::string where)
    {
        if (where != "ProcessUlDci")
        {
            return;
        }
        pendingStatus = 0;
        pendingData = 0;
        for (const auto& [lcid, report] : bsr)
        {
            pendingStatus += report.statusPduSize;
            pendingData += report.txQueueSize;
        }
    }

    std::vector<ForwardingMacSap> m_saps;  ///< Per-LC MAC SAPs
    std::vector<NrRlcTestSduSink> m_sinks; ///< Per-LC RLC SDU sinks
    std::vector<Ptr<NrRlcAm>> m_rlcs;      ///< RLC AM entities
    DciCollectorSchedSap m_sched;          ///< Scheduler decision collector
    NullCschedSap m_csched;                ///< Scheduler configuration sink
};

/**
 * @ingroup nr-test
 *
 * Check that an injected grant admits only the STATUS PDUs that fit with their
 * subheader and the SHORT_BSR, and that a larger grant sends the rest once.
 */
class StatusAdmissionCase : public TestCase
{
  public:
    /**
     * Constructor.
     *
     * @param tbs Transport block size of the first grant.
     * @param lcs Number of AM logical channels with a pending STATUS.
     * @param loss Whether STATUS carries a NACK.
     */
    StatusAdmissionCase(uint32_t tbs, uint32_t lcs, bool loss)
        : TestCase("STATUS admission: TBS " + std::to_string(tbs) + ", LCs " + std::to_string(lcs) +
                   (loss ? ", NACK" : ", ACK")),
          m_tbs(tbs),
          m_lcs(lcs),
          m_loss(loss)
    {
    }

  private:
    void DoRun() override
    {
        StatusBench bench(m_lcs, m_loss, "");
        bench.Slot();
        bench.Grant(m_tbs);

        const uint32_t admissionBytes = AM_STATUS_BYTES + STATUS_SUBHEADER_BYTES;
        const uint32_t expected = std::min(m_lcs, (m_tbs - SHORT_BSR_BYTES) / admissionBytes);
        const uint32_t emittedBytes = m_loss ? AM_NACK_MAC_BYTES : AM_ACK_MAC_BYTES;
        const uint32_t sent = bench.StatusSent();
        NS_TEST_ASSERT_MSG_EQ(sent, expected, "Admission must account for subheader and BSR");
        NS_TEST_ASSERT_MSG_EQ(bench.BytesSince(0),
                              sent * emittedBytes + SHORT_BSR_BYTES,
                              "TB carries the admitted STATUS and the SHORT_BSR");
        NS_TEST_ASSERT_MSG_EQ(bench.phy.packets.back()->GetSize(),
                              SHORT_BSR_BYTES,
                              "SHORT_BSR closes the TB");
        NS_TEST_ASSERT_MSG_EQ(bench.pendingStatus,
                              AM_STATUS_BYTES * (m_lcs - sent),
                              "Deferred STATUS stays pending");

        const auto first = bench.phy.packets.size();
        const uint32_t retryTbs = m_lcs * admissionBytes + SHORT_BSR_BYTES;
        bench.Slot();
        bench.Grant(retryTbs);
        NS_TEST_ASSERT_MSG_EQ(bench.StatusSent(), m_lcs, "Every STATUS is sent exactly once");
        NS_TEST_ASSERT_MSG_EQ(bench.pendingStatus, 0, "No STATUS left pending");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(bench.BytesSince(first), retryTbs, "Retry TB respected");
    }

    uint32_t m_tbs; ///< Transport block size of the first grant
    uint32_t m_lcs; ///< Number of AM logical channels
    bool m_loss;    ///< Whether STATUS carries a NACK
};

/**
 * @ingroup nr-test
 *
 * Check that a STATUS deferred because the grant was too small, with no other
 * data in the buffers, is eventually granted by the gNB scheduler from the
 * SHORT_BSR and transmitted exactly once.
 */
class StatusEventualAllocationCase : public TestCase
{
  public:
    /**
     * Constructor.
     *
     * @param scheduler "TDMA" or "OFDMA".
     * @param lcs Number of AM logical channels with a pending STATUS.
     * @param firstTbs TBS of the first grant; 0 lets the scheduler answer the SR.
     */
    StatusEventualAllocationCase(const std::string& scheduler, uint32_t lcs, uint32_t firstTbs)
        : TestCase("STATUS eventual allocation: " + scheduler + ", LCs " + std::to_string(lcs) +
                   ", first TBS " + (firstTbs ? std::to_string(firstTbs) : "from SR")),
          m_scheduler(scheduler),
          m_lcs(lcs),
          m_firstTbs(firstTbs)
    {
    }

  private:
    void DoRun() override
    {
        StatusBench bench(m_lcs, false, m_scheduler);
        bench.Slot();
        if (m_firstTbs > 0)
        {
            bench.Grant(m_firstTbs);
            NS_TEST_ASSERT_MSG_LT(bench.StatusSent(), m_lcs, "First grant must defer STATUS");
        }
        NS_TEST_ASSERT_MSG_EQ(bench.pendingData, 0, "Only STATUS is pending");

        for (uint32_t slot = 0; slot < MAX_SCHED_SLOTS && bench.StatusSent() < m_lcs; ++slot)
        {
            bench.Slot();
            const auto first = bench.phy.packets.size();
            const uint32_t tbs = bench.Schedule();
            NS_TEST_ASSERT_MSG_LT_OR_EQ(bench.BytesSince(first), tbs, "Scheduled TB respected");
        }
        NS_TEST_ASSERT_MSG_EQ(bench.StatusSent(), m_lcs, "Deferred STATUS eventually sent once");
        NS_TEST_ASSERT_MSG_EQ(bench.pendingStatus, 0, "No STATUS left pending");
    }

    std::string m_scheduler; ///< gNB scheduler type
    uint32_t m_lcs;          ///< Number of AM logical channels
    uint32_t m_firstTbs;     ///< TBS of the first grant, 0 for none
};

/**
 * @ingroup nr-test
 *
 * MAC SAP user that emits a STATUS of a given size on each transmission
 * opportunity and reports new data from within the callback.
 */
class BoundaryStatusProducer : public NrMacSapUser
{
  public:
    /**
     * Constructor.
     *
     * @param mac UE MAC SAP.
     * @param size PDU size to emit, capped by the TxOpportunity.
     * @param reentrantBytes New data to report on each opportunity.
     */
    BoundaryStatusProducer(NrMacSapProvider* mac,
                           uint32_t size,
                           uint32_t reentrantBytes = REENTRANT_DATA_BYTES)
        : m_mac(mac),
          m_size(size),
          m_reentrantBytes(reentrantBytes)
    {
    }

    void NotifyTxOpportunity(TxOpportunityParameters params) override
    {
        ++count;
        NrMacSapProvider::TransmitPduParameters tx{};
        tx.pdu = Create<Packet>(std::min(m_size, params.bytes));
        tx.rnti = params.rnti;
        tx.lcid = params.lcid;
        tx.layer = params.layer;
        tx.harqProcessId = params.harqId;
        tx.componentCarrierId = params.componentCarrierId;
        m_mac->TransmitPdu(tx);
        NrMacSapProvider::BufferStatusReportParameters bsr{};
        bsr.rnti = params.rnti;
        bsr.lcid = params.lcid;
        bsr.txQueueSize = m_reentrantBytes;
        m_mac->BufferStatusReport(bsr);
    }

    void NotifyHarqDeliveryFailure() override
    {
    }

    void ReceivePdu(ReceivePduParameters) override
    {
    }

    static constexpr uint32_t REENTRANT_DATA_BYTES = 2; ///< New data reported on each opportunity
    uint32_t count{0};                                  ///< Opportunities received

  private:
    NrMacSapProvider* m_mac;   ///< UE MAC SAP
    uint32_t m_size;           ///< PDU size
    uint32_t m_reentrantBytes; ///< New data reported on each opportunity
};

/**
 * @ingroup nr-test
 *
 * Check STATUS admission around the 2/3-byte MAC subheader boundary, and that a
 * BSR reported from within the transmission opportunity is preserved.
 */
class StatusBoundaryCase : public TestCase
{
  public:
    /**
     * Constructor.
     *
     * @param size STATUS size.
     * @param fits Whether the first grant has room for the subheader and the SHORT_BSR.
     */
    StatusBoundaryCase(uint32_t size, bool fits)
        : TestCase("STATUS header boundary " + std::to_string(size) +
                   (fits ? " fits" : " deferred")),
          m_size(size),
          m_fits(fits)
    {
    }

  private:
    void DoRun() override
    {
        StatusBench bench(0, false, "");
        BoundaryStatusProducer producer(bench.mac->GetUeMacSapProvider(), m_size);
        bench.mac->GetUeCmacSapProvider()->AddLc(FIRST_LCID, {}, &producer);
        NrMacSapProvider::BufferStatusReportParameters bsr{};
        bsr.rnti = RNTI;
        bsr.lcid = FIRST_LCID;
        bsr.statusPduSize = m_size;
        bench.mac->GetUeMacSapProvider()->BufferStatusReport(bsr);
        bench.Slot();

        NrMacHeaderVs header;
        header.SetSize(m_size);
        const uint32_t tbs = m_size + header.GetSerializedSize() + SHORT_BSR_BYTES;
        bench.Grant(m_fits ? tbs : tbs - 1);
        NS_TEST_ASSERT_MSG_EQ(producer.count, m_fits ? 1 : 0, "Admission includes the overhead");
        NS_TEST_ASSERT_MSG_EQ(bench.pendingStatus, m_fits ? 0 : m_size, "Deferred STATUS kept");
        if (!m_fits)
        {
            bench.Grant(tbs);
        }
        NS_TEST_ASSERT_MSG_EQ(producer.count, 1, "STATUS sent exactly once");
        NS_TEST_ASSERT_MSG_EQ(bench.pendingStatus, 0, "Sent STATUS cleared");
        NS_TEST_ASSERT_MSG_EQ(bench.pendingData,
                              BoundaryStatusProducer::REENTRANT_DATA_BYTES,
                              "Reentrant BSR preserved");
        NS_TEST_ASSERT_MSG_EQ(bench.phy.packets.at(bench.phy.packets.size() - 2)->GetSize(),
                              tbs - SHORT_BSR_BYTES,
                              "STATUS serialized with its subheader");
    }

    uint32_t m_size; ///< STATUS size
    bool m_fits;     ///< Whether the first grant fits the overhead
};

/**
 * @ingroup nr-test
 *
 * Check that the SHORT_BSR of a UE with only STATUS, or only new data, pending
 * advertises exactly the PDUs, their subheaders and a single SHORT_BSR, and that a
 * grant of the advertised size sends them all.
 */
class BsrSizedGrantCase : public TestCase
{
  public:
    /**
     * Constructor.
     *
     * @param size Pending bytes of each logical channel.
     * @param lcs Number of logical channels with pending bytes.
     * @param data Whether the pending bytes are new data rather than STATUS.
     */
    BsrSizedGrantCase(uint32_t size, uint32_t lcs, bool data)
        : TestCase(std::string(data ? "TX data" : "STATUS") + " BSR-sized grant " +
                   std::to_string(size) + ", LCs " + std::to_string(lcs)),
          m_size(size),
          m_lcs(lcs),
          m_data(data)
    {
    }

  private:
    void DoRun() override
    {
        StatusBench bench(0, false, "");
        std::vector<std::unique_ptr<BoundaryStatusProducer>> producers;
        for (uint32_t i = 0; i < m_lcs; ++i)
        {
            const uint8_t lcid = FIRST_LCID + i;
            producers.push_back(
                std::make_unique<BoundaryStatusProducer>(bench.mac->GetUeMacSapProvider(),
                                                         m_size,
                                                         m_data ? 0 : REENTRANT_BYTES));
            bench.mac->GetUeCmacSapProvider()->AddLc(lcid, {}, producers.back().get());
            NrMacSapProvider::BufferStatusReportParameters bsr{};
            bsr.rnti = RNTI;
            bsr.lcid = lcid;
            if (m_data)
            {
                bsr.txQueueSize = m_size;
            }
            else
            {
                bsr.statusPduSize = m_size;
            }
            bench.mac->GetUeMacSapProvider()->BufferStatusReport(bsr);
        }
        auto sent = [&producers]() {
            uint32_t count = 0;
            for (const auto& producer : producers)
            {
                count += producer->count;
            }
            return count;
        };
        bench.Slot();

        // A grant with room for the SHORT_BSR only defers every PDU and reports them.
        bench.Grant(SHORT_BSR_BYTES);
        NS_TEST_ASSERT_MSG_EQ(sent(), 0, "PDUs deferred");
        NrMacShortBsrCe shortBsr;
        bench.phy.packets.back()->PeekHeader(shortBsr);
        const auto advertised = NrMacShortBsrCe::FromLevelToBytes(shortBsr.m_bufferSizeLevel_0);

        NrMacHeaderVs header;
        header.SetSize(m_size);
        const uint32_t needed = m_lcs * (m_size + header.GetSerializedSize()) + SHORT_BSR_BYTES;
        NS_TEST_ASSERT_MSG_EQ(
            advertised,
            NrMacShortBsrCe::FromLevelToBytes(NrMacShortBsrCe::FromBytesToLevel(needed)),
            "BSR advertises the PDUs, their subheaders and one SHORT_BSR");

        bench.Slot();
        bench.Grant(advertised);
        NS_TEST_ASSERT_MSG_EQ(sent(), m_lcs, "BSR-sized grant sends every PDU in one go");
        NS_TEST_ASSERT_MSG_EQ(bench.pendingStatus, 0, "No STATUS left pending");
        NS_TEST_ASSERT_MSG_EQ(bench.pendingData,
                              m_data ? 0 : m_lcs * REENTRANT_BYTES,
                              "Only the data reported during the TxOpportunities is left");
    }

    uint32_t m_size; ///< Pending bytes of each logical channel
    uint32_t m_lcs;  ///< Number of logical channels
    bool m_data;     ///< Whether the pending bytes are new data rather than STATUS

    /// New data reported during each STATUS opportunity
    static constexpr uint32_t REENTRANT_BYTES = BoundaryStatusProducer::REENTRANT_DATA_BYTES;
};

/**
 * @ingroup nr-test
 *
 * Check that a grant to a UE without any buffered byte, nor any logical channel that
 * ever reported, is filled by a zero SHORT_BSR rather than left empty.
 */
class PaddingBsrCase : public TestCase
{
  public:
    PaddingBsrCase()
        : TestCase("Grant without data carries a zero SHORT_BSR")
    {
    }

  private:
    void DoRun() override
    {
        StatusBench bench(0, false, "");
        bench.Slot();
        bench.Grant(SHORT_BSR_BYTES);
        NS_TEST_ASSERT_MSG_EQ(bench.phy.packets.size(), 1, "TB holds a single sub-PDU");
        const auto& pdu = bench.phy.packets.front();
        NrRadioBearerTag tag;
        pdu->PeekPacketTag(tag);
        NS_TEST_ASSERT_MSG_EQ(+tag.GetLcid(), +NrMacHeaderFsUl::SHORT_BSR, "Sub-PDU is a BSR");
        NS_TEST_ASSERT_MSG_EQ(pdu->GetSize(), SHORT_BSR_BYTES, "SHORT_BSR size");
        NrMacShortBsrCe shortBsr;
        pdu->PeekHeader(shortBsr);
        NS_TEST_ASSERT_MSG_EQ(+shortBsr.m_bufferSizeLevel_0, 0, "Nothing buffered");
    }
};

/**
 * @ingroup nr-test
 *
 * UE MAC STATUS overhead admission test suite.
 */
class NrUeMacStatusTestSuite : public TestSuite
{
  public:
    NrUeMacStatusTestSuite()
        : TestSuite("nr-ue-mac-status", Type::UNIT)
    {
        for (uint32_t tbs : {9u, 10u, 11u, 12u})
        {
            AddTestCase(new StatusAdmissionCase(tbs, 1, true), Duration::QUICK);
            AddTestCase(new StatusAdmissionCase(tbs, 1, false), Duration::QUICK);
        }
        AddTestCase(new StatusAdmissionCase(12, 2, true), Duration::QUICK);
        AddTestCase(new StatusAdmissionCase(17, 3, false), Duration::QUICK);
        for (const std::string scheduler : {"TDMA", "OFDMA"})
        {
            AddTestCase(new StatusEventualAllocationCase(scheduler, 3, 0), Duration::QUICK);
            AddTestCase(new StatusEventualAllocationCase(scheduler, 1, 9), Duration::QUICK);
            AddTestCase(new StatusEventualAllocationCase(scheduler, 3, 11), Duration::QUICK);
        }
        for (uint32_t size : {127u, 128u})
        {
            AddTestCase(new StatusBoundaryCase(size, false), Duration::QUICK);
            AddTestCase(new StatusBoundaryCase(size, true), Duration::QUICK);
        }
        // PDUs + subheaders + one SHORT_BSR land exactly on SHORT_BSR levels (28, 38, 142 and
        // 74 B), so any over-count of the overhead bumps the advertised level, and any
        // over-reservation in the data path leaves a segment behind.
        for (bool data : {false, true})
        {
            AddTestCase(new BsrSizedGrantCase(21, 1, data), Duration::QUICK);
            AddTestCase(new BsrSizedGrantCase(31, 1, data), Duration::QUICK);
            AddTestCase(new BsrSizedGrantCase(134, 1, data), Duration::QUICK);
            AddTestCase(new BsrSizedGrantCase(21, 3, data), Duration::QUICK);
        }
        AddTestCase(new PaddingBsrCase(), Duration::QUICK);
    }
};

/// Static instance of the test suite
NrUeMacStatusTestSuite g_nrUeMacStatusTestSuite;
} // namespace
} // namespace ns3
