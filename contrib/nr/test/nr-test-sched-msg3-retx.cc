// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/config.h"
#include "ns3/nr-amc.h"
#include "ns3/nr-mac-sched-sap.h"
#include "ns3/nr-mac-scheduler-ns3.h"
#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/nr-phy-mac-common.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

/**
 * @file nr-test-sched-msg3-retx.cc
 * @ingroup test
 *
 * @brief Unit tests for msg3 (UL RRC Connection Request) HARQ retransmission in
 *        the NR MAC scheduler.
 *
 * When a UE's initial msg3 UL PUSCH fails to decode at the gNB, the gNB PHY
 * generates a UL HARQ NACK for TC-RNTI / HARQ process 0. Historically that
 * feedback was dropped (the TC-RNTI is not yet a registered UE), msg4 was never
 * produced and the UE's T300 timer expired. The scheduler now tracks pending
 * msg3 transmissions and, on a NACK, issues a normal UL HARQ retransmission DCI
 * (m_type=DATA, m_format=UL, m_ndi=0, m_harqProcess=0) addressed to the TC-RNTI,
 * which makes the UE retransmit the buffered msg3. This behaviour is gated by
 * the Msg3MaxRetx attribute (0 disables it, reproducing the legacy single-shot
 * behaviour).
 *
 * These tests drive the scheduler SAP directly (no PHY) to verify, in
 * isolation, that:
 *  - With Msg3MaxRetx > 0, a msg3 NACK yields a retransmission UL DCI with the
 *    correct fields, and retransmissions stop after Msg3MaxRetx attempts.
 *  - With Msg3MaxRetx == 0, a msg3 NACK yields no retransmission and does not
 *    crash (legacy behaviour).
 *  - A msg3 ACK (decoded) does not yield a retransmission.
 */
namespace ns3
{

/**
 * @ingroup test
 * @brief Minimal CSCHED SAP user for the msg3 retx scheduler tests.
 */
class TestCschedSapUserMsg3 : public NrMacCschedSapUser
{
  public:
    TestCschedSapUserMsg3() = default;

    void CschedCellConfigCnf(
        [[maybe_unused]] const struct CschedCellConfigCnfParameters& params) override
    {
    }

    void CschedUeConfigCnf(
        [[maybe_unused]] const struct CschedUeConfigCnfParameters& params) override
    {
    }

    void CschedLcConfigCnf(
        [[maybe_unused]] const struct CschedLcConfigCnfParameters& params) override
    {
    }

    void CschedLcReleaseCnf(
        [[maybe_unused]] const struct CschedLcReleaseCnfParameters& params) override
    {
    }

    void CschedUeReleaseCnf(
        [[maybe_unused]] const struct CschedUeReleaseCnfParameters& params) override
    {
    }

    void CschedUeConfigUpdateInd(
        [[maybe_unused]] const struct CschedUeConfigUpdateIndParameters& params) override
    {
    }

    void CschedCellConfigUpdateInd(
        [[maybe_unused]] const struct CschedCellConfigUpdateIndParameters& params) override
    {
    }
};

/**
 * @ingroup test
 * @brief SCHED SAP user that forwards the scheduling decision to a callback.
 */
class TestSchedSapUserMsg3 : public NrMacSchedSapUser
{
  public:
    /**
     * @brief Constructor
     * @param schedConfigIndCallback Callback invoked with each scheduling
     *        decision (SchedConfigInd).
     */
    TestSchedSapUserMsg3(
        std::function<void(const struct SchedConfigIndParameters&)> schedConfigIndCallback)
        : NrMacSchedSapUser(),
          m_schedConfigIndCallback(std::move(schedConfigIndCallback))
    {
    }

    void SchedConfigInd(const NrMacSchedSapUser::SchedConfigIndParameters& params) override
    {
        if (m_schedConfigIndCallback)
        {
            m_schedConfigIndCallback(params);
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
        return 20;
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

    void BuildRarList(SlotAllocInfo& allocInfo) override
    {
    }

  private:
    std::function<void(const NrMacSchedSapUser::SchedConfigIndParameters&)>
        m_schedConfigIndCallback; //!< Decision callback
};

/**
 * @ingroup test
 * @brief Test case for msg3 HARQ retransmission scheduling.
 */
class NrTestSchedMsg3Retx : public TestCase
{
  public:
    /**
     * @brief Constructor
     * @param maxRetx Value of the Msg3MaxRetx attribute under test.
     * @param name Human-readable test name.
     */
    NrTestSchedMsg3Retx(uint8_t maxRetx, const std::string& name)
        : TestCase(name),
          m_maxRetx(maxRetx)
    {
    }

  protected:
    void DoRun() override;

  private:
    /**
     * @brief Collect msg3 retransmission DCIs from a scheduling decision.
     *
     * A msg3 retransmission DCI is an UL data DCI (m_type=DATA, m_format=UL)
     * with new-data-indicator 0 and HARQ process 0, addressed to the TC-RNTI.
     *
     * @param params The scheduling decision.
     */
    void OnSchedConfigInd(const NrMacSchedSapUser::SchedConfigIndParameters& params);

    /**
     * @brief Issue a UL trigger request, optionally carrying a msg3 HARQ NACK.
     * @param sched Scheduler under test.
     * @param sfn The slot to schedule.
     * @param nackRnti If non-zero, inject a UL HARQ NACK for this TC-RNTI at
     *        HARQ process 0.
     */
    void TriggerUl(const Ptr<NrMacSchedulerNs3>& sched, const SfnSf& sfn, uint16_t nackRnti);

    uint8_t m_maxRetx;                            //!< Msg3MaxRetx under test
    uint16_t m_tcRnti{42};                        //!< TC-RNTI of the msg3 UE
    std::vector<DciInfoElementTdma> m_retxDcis{}; //!< Collected msg3 retx DCIs
};

void
NrTestSchedMsg3Retx::OnSchedConfigInd(const NrMacSchedSapUser::SchedConfigIndParameters& params)
{
    for (const auto& alloc : params.m_slotAllocInfo.m_varTtiAllocInfo)
    {
        const auto& dci = alloc.m_dci;
        if (dci->m_format == DciInfoElementTdma::UL && dci->m_type == DciInfoElementTdma::DATA &&
            dci->m_ndi == 0 && dci->m_rnti == m_tcRnti && dci->m_harqProcess == 0)
        {
            m_retxDcis.push_back(*dci);
        }
    }
}

void
NrTestSchedMsg3Retx::TriggerUl(const Ptr<NrMacSchedulerNs3>& sched,
                               const SfnSf& sfn,
                               uint16_t nackRnti)
{
    NrMacSchedSapProvider::SchedUlTriggerReqParameters params;
    params.m_snfSf = sfn;
    params.m_slotType = LteNrTddSlotType::UL;
    if (nackRnti != 0)
    {
        UlHarqInfo nack;
        nack.m_rnti = nackRnti;
        nack.m_harqProcessId = 0;
        nack.m_receptionStatus = UlHarqInfo::NotOk;
        nack.m_numRetx = 0;
        params.m_ulHarqInfoList.push_back(nack);
    }
    sched->DoSchedUlTriggerReq(params);
}

void
NrTestSchedMsg3Retx::DoRun()
{
    auto cellConfig = NrMacCschedSapProvider::CschedCellConfigReqParameters();
    cellConfig.m_dlBandwidth = 10; // 10 RBGs
    cellConfig.m_ulBandwidth = 10;

    auto* schedSapUser =
        new TestSchedSapUserMsg3([this](const auto& params) { OnSchedConfigInd(params); });
    auto* cschedSapUser = new TestCschedSapUserMsg3();

    auto sched = CreateObject<NrMacSchedulerTdmaRR>();
    sched->SetAttribute("Msg3MaxRetx", UintegerValue(m_maxRetx));
    sched->InstallDlAmc(CreateObject<NrAmc>());
    sched->InstallUlAmc(CreateObject<NrAmc>());
    sched->SetMacSchedSapUser(schedSapUser);
    sched->SetMacCschedSapUser(cschedSapUser);
    sched->DoCschedCellConfigReq(cellConfig);
    sched->SetDlCtrlSyms(1);
    sched->SetUlCtrlSyms(1);

    SfnSf sfn(0, 0, 0, 0);

    // Step 1: Register a RACH (initial msg3) and trigger the initial UL grant.
    {
        NrMacSchedSapProvider::SchedDlRachInfoReqParameters rachParams;
        nr::RachListElement_s rach;
        rach.m_rnti = m_tcRnti;
        rach.m_estimatedSize = 7; // small RRC Connection Request, fits in a few symbols
        rachParams.m_rachList.push_back(rach);
        sched->DoSchedDlRachInfoReq(rachParams);
    }
    TriggerUl(sched, sfn, 0);
    // The initial msg3 grant is type MSG3 (not a retx), so nothing collected yet.
    NS_TEST_ASSERT_MSG_EQ(m_retxDcis.size(), 0, "Initial msg3 grant must not be a retx DCI");

    // Step 2: Drive a sequence of msg3 NACKs and count retransmission DCIs.
    // Each NACK is processed on the next UL trigger, which then issues the retx.
    uint32_t expectedRetx = m_maxRetx; // capped by the attribute
    uint32_t issuedRetx = 0;
    // Try a few extra NACKs beyond the budget to confirm retx stops.
    const uint32_t nackRounds = static_cast<uint32_t>(m_maxRetx) + 3;
    for (uint32_t i = 0; i < nackRounds; ++i)
    {
        sfn.Add(1);
        const size_t before = m_retxDcis.size();
        TriggerUl(sched, sfn, m_tcRnti);
        if (m_retxDcis.size() > before)
        {
            ++issuedRetx;
        }
    }

    NS_TEST_ASSERT_MSG_EQ(issuedRetx,
                          expectedRetx,
                          "Number of msg3 retransmission DCIs must equal Msg3MaxRetx");

    if (m_maxRetx > 0)
    {
        // Validate the fields of the first retransmission DCI.
        const auto& dci = m_retxDcis.front();
        NS_TEST_ASSERT_MSG_EQ(dci.m_rnti, m_tcRnti, "Retx DCI must address the TC-RNTI");
        NS_TEST_ASSERT_MSG_EQ(dci.m_format, DciInfoElementTdma::UL, "Retx DCI must be an UL DCI");
        NS_TEST_ASSERT_MSG_EQ(dci.m_type,
                              DciInfoElementTdma::DATA,
                              "Retx DCI must be of type DATA so it is dispatched as a UL DCI");
        NS_TEST_ASSERT_MSG_EQ(dci.m_ndi,
                              0,
                              "Retx DCI must have NDI=0 to trigger UE retransmission");
        NS_TEST_ASSERT_MSG_EQ(dci.m_harqProcess, 0, "msg3 lives at HARQ process 0 on the UE");
        NS_TEST_ASSERT_MSG_GT(dci.m_tbSize, 0, "Retx DCI must carry a non-zero TB size");
    }

    delete schedSapUser;
    delete cschedSapUser;
    Simulator::Destroy();
}

/**
 * @ingroup test
 * @brief Test case that a decoded (ACKed) msg3 does not trigger a retransmission.
 */
class NrTestSchedMsg3Ack : public TestCase
{
  public:
    NrTestSchedMsg3Ack()
        : TestCase("msg3 ACK does not trigger retransmission")
    {
    }

  protected:
    void DoRun() override;

  private:
    /**
     * @brief Count any msg3 retransmission DCI emitted by the scheduler.
     * @param params The scheduling decision.
     */
    void OnSchedConfigInd(const NrMacSchedSapUser::SchedConfigIndParameters& params);

    uint16_t m_tcRnti{55};   //!< TC-RNTI of the msg3 UE
    uint32_t m_retxCount{0}; //!< Number of retx DCIs observed
};

void
NrTestSchedMsg3Ack::OnSchedConfigInd(const NrMacSchedSapUser::SchedConfigIndParameters& params)
{
    for (const auto& alloc : params.m_slotAllocInfo.m_varTtiAllocInfo)
    {
        const auto& dci = alloc.m_dci;
        if (dci->m_format == DciInfoElementTdma::UL && dci->m_type == DciInfoElementTdma::DATA &&
            dci->m_ndi == 0 && dci->m_rnti == m_tcRnti && dci->m_harqProcess == 0)
        {
            ++m_retxCount;
        }
    }
}

void
NrTestSchedMsg3Ack::DoRun()
{
    auto cellConfig = NrMacCschedSapProvider::CschedCellConfigReqParameters();
    cellConfig.m_dlBandwidth = 10;
    cellConfig.m_ulBandwidth = 10;

    auto* schedSapUser =
        new TestSchedSapUserMsg3([this](const auto& params) { OnSchedConfigInd(params); });
    auto* cschedSapUser = new TestCschedSapUserMsg3();

    auto sched = CreateObject<NrMacSchedulerTdmaRR>();
    sched->SetAttribute("Msg3MaxRetx", UintegerValue(4));
    sched->InstallDlAmc(CreateObject<NrAmc>());
    sched->InstallUlAmc(CreateObject<NrAmc>());
    sched->SetMacSchedSapUser(schedSapUser);
    sched->SetMacCschedSapUser(cschedSapUser);
    sched->DoCschedCellConfigReq(cellConfig);
    sched->SetDlCtrlSyms(1);
    sched->SetUlCtrlSyms(1);

    SfnSf sfn(0, 0, 0, 0);

    NrMacSchedSapProvider::SchedDlRachInfoReqParameters rachParams;
    nr::RachListElement_s rach;
    rach.m_rnti = m_tcRnti;
    rach.m_estimatedSize = 7;
    rachParams.m_rachList.push_back(rach);
    sched->DoSchedDlRachInfoReq(rachParams);

    // Initial grant.
    NrMacSchedSapProvider::SchedUlTriggerReqParameters initTrig;
    initTrig.m_snfSf = sfn;
    initTrig.m_slotType = LteNrTddSlotType::UL;
    sched->DoSchedUlTriggerReq(initTrig);

    // Feed an ACK for msg3 (decoded). No retransmission should be scheduled.
    sfn.Add(1);
    NrMacSchedSapProvider::SchedUlTriggerReqParameters ackTrig;
    ackTrig.m_snfSf = sfn;
    ackTrig.m_slotType = LteNrTddSlotType::UL;
    UlHarqInfo ack;
    ack.m_rnti = m_tcRnti;
    ack.m_harqProcessId = 0;
    ack.m_receptionStatus = UlHarqInfo::Ok;
    ack.m_numRetx = 0;
    ackTrig.m_ulHarqInfoList.push_back(ack);
    sched->DoSchedUlTriggerReq(ackTrig);

    // A further UL slot must not produce a stray retransmission.
    sfn.Add(1);
    NrMacSchedSapProvider::SchedUlTriggerReqParameters trig3;
    trig3.m_snfSf = sfn;
    trig3.m_slotType = LteNrTddSlotType::UL;
    sched->DoSchedUlTriggerReq(trig3);

    NS_TEST_ASSERT_MSG_EQ(m_retxCount, 0, "An ACKed msg3 must not trigger a retransmission");

    delete schedSapUser;
    delete cschedSapUser;
    Simulator::Destroy();
}

/**
 * @ingroup test
 * @brief msg3 HARQ retransmission test suite.
 */
class NrTestSchedMsg3RetxSuite : public TestSuite
{
  public:
    NrTestSchedMsg3RetxSuite()
        : TestSuite("nr-test-sched-msg3-retx", Type::UNIT)
    {
        // Msg3MaxRetx == 0: legacy single-shot, no retx, no crash.
        AddTestCase(new NrTestSchedMsg3Retx(0, "Msg3MaxRetx=0 issues no retransmission"),
                    Duration::QUICK);
        // Msg3MaxRetx > 0: exactly Msg3MaxRetx retransmissions are issued.
        AddTestCase(new NrTestSchedMsg3Retx(1, "Msg3MaxRetx=1 issues one retransmission"),
                    Duration::QUICK);
        AddTestCase(new NrTestSchedMsg3Retx(4, "Msg3MaxRetx=4 issues four retransmissions"),
                    Duration::QUICK);
        AddTestCase(new NrTestSchedMsg3Ack(), Duration::QUICK);
    }
};

static NrTestSchedMsg3RetxSuite g_nrTestSchedMsg3RetxSuite; //!< msg3 retx test suite

} // namespace ns3
