// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/nr-amc.h"
#include "ns3/nr-mac-sched-sap.h"
#include "ns3/nr-mac-scheduler-ns3.h"
#include "ns3/nr-mac-scheduler-ue-info.h"
#include "ns3/nr-phy-mac-common.h"
#include "ns3/object-factory.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

/**
 * @file nr-test-sched-released-ue.cc
 * @ingroup test
 *
 * @brief Tests that channel feedback referring to an already released UE is
 *        dropped by the scheduler instead of being processed on a stale (or
 *        invalid) UE context.
 *
 * A UE can be released from a cell (handover, RRC release, radio link failure)
 * while feedback generated before the release is still in flight. Both the DL
 * CQI path and the UL CQI path must tolerate such reports: the report is
 * dropped, the still-attached UEs keep being served, and no invalid iterator is
 * dereferenced (the DL CQI path used to be guarded only by an NS_ASSERT, which
 * is compiled out in optimized builds).
 *
 * The tests drive the scheduler SAPs directly (no PHY), for both TDMA and OFDMA
 * schedulers, and for both TDD (a single BWP carrying flexible slots) and FDD
 * (a BWP dedicated to a single direction).
 */
namespace ns3
{

/**
 * @ingroup test
 * @brief Minimal CSCHED SAP user for the released-UE scheduler tests.
 */
class TestCschedSapUserReleasedUe : public NrMacCschedSapUser
{
  public:
    void CschedCellConfigCnf(const struct CschedCellConfigCnfParameters&) override
    {
    }

    void CschedUeConfigCnf(const struct CschedUeConfigCnfParameters&) override
    {
    }

    void CschedLcConfigCnf(const struct CschedLcConfigCnfParameters&) override
    {
    }

    void CschedLcReleaseCnf(const struct CschedLcReleaseCnfParameters&) override
    {
    }

    void CschedUeReleaseCnf(const struct CschedUeReleaseCnfParameters&) override
    {
    }

    void CschedUeConfigUpdateInd(const struct CschedUeConfigUpdateIndParameters&) override
    {
    }

    void CschedCellConfigUpdateInd(const struct CschedCellConfigUpdateIndParameters&) override
    {
    }
};

/**
 * @ingroup test
 * @brief SCHED SAP user forwarding the scheduling decisions to a callback.
 */
class TestSchedSapUserReleasedUe : public NrMacSchedSapUser
{
  public:
    /**
     * @brief Constructor
     * @param schedConfigIndCallback Callback invoked for each scheduling decision.
     */
    TestSchedSapUserReleasedUe(
        std::function<void(const struct SchedConfigIndParameters&)> schedConfigIndCallback)
        : NrMacSchedSapUser(),
          m_schedConfigIndCallback(std::move(schedConfigIndCallback))
    {
    }

    void SchedConfigInd(const struct SchedConfigIndParameters& params) override
    {
        if (m_schedConfigIndCallback)
        {
            m_schedConfigIndCallback(params);
        }
    }

    // For the rest, hard-coded values; there is no need for real ones here.
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

    void BuildRarList(SlotAllocInfo&) override
    {
    }

  private:
    std::function<void(const struct SchedConfigIndParameters&)>
        m_schedConfigIndCallback; //!< Decision callback
};

/**
 * @ingroup test
 * @brief Feeds DL and UL CQI reports of a released UE to the scheduler.
 *
 * The reports of the released UE must be dropped, while the reports of the
 * still-attached UE must be processed as usual.
 */
class NrSchedReleasedUeCqiTestCase : public TestCase
{
  public:
    /// Duplex mode under test
    enum class Duplex
    {
        TDD, //!< A single BWP serving both directions, with flexible slots
        FDD  //!< A BWP dedicated to a single direction
    };

    /**
     * @brief Build the name of a test case
     * @param schedType TypeId name of the scheduler under test
     * @param duplex Duplex mode
     * @return The test case name
     */
    static std::string GetTestName(const std::string& schedType, Duplex duplex)
    {
        return "CQI of a released UE is dropped, scheduler=" + schedType +
               ", duplex=" + (duplex == Duplex::TDD ? "TDD" : "FDD");
    }

    /**
     * @brief Constructor
     * @param schedType TypeId name of the scheduler under test
     * @param duplex Duplex mode
     */
    NrSchedReleasedUeCqiTestCase(const std::string& schedType, Duplex duplex)
        : TestCase(GetTestName(schedType, duplex)),
          m_schedType(schedType),
          m_duplex(duplex)
    {
    }

  private:
    void DoRun() override;

    /**
     * @brief Create the scheduler under test
     * @return The scheduler
     */
    Ptr<NrMacSchedulerNs3> CreateScheduler();

    /**
     * @brief Attach a UE, with its logical channels and a pending UL buffer
     * @param sched Scheduler under test
     * @param rnti RNTI of the UE to attach
     */
    void AttachUe(const Ptr<NrMacSchedulerNs3>& sched, uint16_t rnti);

    /**
     * @brief Callback connected to the CsiFeedbackReceived trace source
     * @param cellId Cell ID
     * @param bwpId BWP ID
     * @param ue UE whose CSI feedback was processed
     */
    void CsiFeedbackReceived(uint16_t cellId,
                             uint16_t bwpId,
                             const std::shared_ptr<NrMacSchedulerUeInfo>& ue);

    /**
     * @brief Record the UL allocation of the UE that is going to be released
     * @param params The scheduling decision
     */
    void OnSchedConfigInd(const NrMacSchedSapUser::SchedConfigIndParameters& params);

    std::string m_schedType;                //!< TypeId name of the scheduler under test
    Duplex m_duplex;                        //!< Duplex mode under test
    std::vector<uint16_t> m_reportingRntis; //!< RNTIs whose DL CQI was processed

    uint16_t m_releasedRnti{1}; //!< RNTI of the UE that gets released (got the UL grant)
    uint16_t m_attachedRnti{2}; //!< RNTI of the UE that stays attached

    SfnSf m_ulAllocSfnSf{};        //!< Slot of the recorded UL allocation
    uint8_t m_ulAllocSymStart{0};  //!< Symbol start of the recorded UL allocation
    bool m_ulAllocRecorded{false}; //!< Whether an UL allocation was recorded
};

void
NrSchedReleasedUeCqiTestCase::CsiFeedbackReceived(uint16_t,
                                                  uint16_t,
                                                  const std::shared_ptr<NrMacSchedulerUeInfo>& ue)
{
    m_reportingRntis.push_back(ue->m_rnti);
}

void
NrSchedReleasedUeCqiTestCase::OnSchedConfigInd(
    const NrMacSchedSapUser::SchedConfigIndParameters& params)
{
    for (const auto& alloc : params.m_slotAllocInfo.m_varTtiAllocInfo)
    {
        const auto& dci = alloc.m_dci;
        if (!m_ulAllocRecorded && dci->m_format == DciInfoElementTdma::UL &&
            dci->m_type == DciInfoElementTdma::DATA && dci->m_rnti != 0)
        {
            // The UE that got the UL grant is the one that will be released, so that
            // the UL CQI measured on its PUSCH arrives after it has left the cell.
            m_releasedRnti = dci->m_rnti;
            m_attachedRnti = (m_releasedRnti == 1) ? 2 : 1;
            m_ulAllocSfnSf = params.m_slotAllocInfo.m_sfnSf;
            m_ulAllocSymStart = dci->m_symStart;
            m_ulAllocRecorded = true;
        }
    }
}

Ptr<NrMacSchedulerNs3>
NrSchedReleasedUeCqiTestCase::CreateScheduler()
{
    ObjectFactory factory;
    factory.SetTypeId(m_schedType);
    return factory.Create<NrMacSchedulerNs3>();
}

void
NrSchedReleasedUeCqiTestCase::AttachUe(const Ptr<NrMacSchedulerNs3>& sched, uint16_t rnti)
{
    NrMacCschedSapProvider::CschedUeConfigReqParameters ueConfig;
    ueConfig.m_rnti = rnti;
    ueConfig.m_beamId = BeamId(0, 120.0);
    sched->DoCschedUeConfigReq(ueConfig);

    // Standard LCGs and LCs
    NrMacCschedSapProvider::CschedLcConfigReqParameters lcParams;
    lcParams.m_rnti = rnti;
    lcParams.m_reconfigureFlag = false;
    nr::LogicalChannelConfigListElement_s lc;
    lc.m_direction = nr::LogicalChannelConfigListElement_s::Direction_e::DIR_BOTH;
    lc.m_qosBearerType = nr::LogicalChannelConfigListElement_s::QosBearerType_e::QBT_NON_GBR;
    lc.m_fiveQi = 9;
    for (uint8_t i = 0; i < 4; i++)
    {
        lc.m_logicalChannelGroup = i;
        lc.m_logicalChannelIdentity = i;
        lcParams.m_logicalChannelConfigList.emplace_back(lc);
    }
    sched->DoCschedLcConfigReq(lcParams);

    // Report a buffer status, so that the UE is served in UL
    NrMacSchedSapProvider::SchedUlMacCtrlInfoReqParameters bsrParams;
    MacCeElement bsr;
    bsr.m_rnti = rnti;
    bsr.m_macCeType = MacCeElement::BSR;
    bsr.m_macCeValue.m_bufferStatus = {10, 10, 10, 10};
    bsrParams.m_macCeList.push_back(bsr);
    sched->DoSchedUlMacCtrlInfoReq(bsrParams);
}

void
NrSchedReleasedUeCqiTestCase::DoRun()
{
    auto* cschedSapUser = new TestCschedSapUserReleasedUe();
    auto* schedSapUser =
        new TestSchedSapUserReleasedUe([this](const auto& params) { OnSchedConfigInd(params); });

    auto sched = CreateScheduler();
    sched->SetMacCschedSapUser(cschedSapUser);
    sched->SetMacSchedSapUser(schedSapUser);

    NrMacCschedSapProvider::CschedCellConfigReqParameters cellConfig{};
    cellConfig.m_dlBandwidth = 10; // 10 RBGs
    cellConfig.m_ulBandwidth = 10;
    sched->DoCschedCellConfigReq(cellConfig);
    sched->InstallDlAmc(CreateObject<NrAmc>());
    sched->InstallUlAmc(CreateObject<NrAmc>());
    sched->SetDlCtrlSyms(1);
    sched->SetUlCtrlSyms(1);

    sched->TraceConnectWithoutContext(
        "CsiFeedbackReceived",
        MakeCallback(&NrSchedReleasedUeCqiTestCase::CsiFeedbackReceived, this));

    AttachUe(sched, 1);
    AttachUe(sched, 2);

    // Produce a real UL allocation; the served UE is the one that will be released.
    // In TDD
    // the slot is flexible (DL and UL share the BWP); in FDD the slot is UL-only.
    SfnSf sfn(0, 0, 0, 0);
    NrMacSchedSapProvider::SchedUlTriggerReqParameters ulTrigger;
    ulTrigger.m_snfSf = sfn;
    ulTrigger.m_slotType = (m_duplex == Duplex::TDD) ? LteNrTddSlotType::F : LteNrTddSlotType::UL;
    sched->DoSchedUlTriggerReq(ulTrigger);
    NS_TEST_ASSERT_MSG_EQ(m_ulAllocRecorded,
                          true,
                          "The UE to be released should have received an UL allocation");

    // The UE leaves the cell (handover, RRC release, radio link failure...)
    NrMacCschedSapProvider::CschedUeReleaseReqParameters releaseParams;
    releaseParams.m_rnti = m_releasedRnti;
    sched->DoCschedUeReleaseReq(releaseParams);

    // A DL CQI batch generated before the release still carries a report for the
    // released UE. Processing it must not dereference an invalid UE map entry.
    NrMacSchedSapProvider::SchedDlCqiInfoReqParameters dlCqiParams{};
    for (uint16_t rnti : {m_releasedRnti, m_attachedRnti})
    {
        DlCqiInfo cqi;
        cqi.m_rnti = rnti;
        cqi.m_wbCqi = 9;
        cqi.m_sbCqis = std::vector<uint8_t>(10, 9);
        dlCqiParams.m_cqiList.push_back(cqi);
    }
    sched->DoSchedDlCqiInfoReq(dlCqiParams);

    NS_TEST_ASSERT_MSG_EQ(m_reportingRntis.size(),
                          1,
                          "Exactly one of the two DL CQI reports should have been processed");
    NS_TEST_EXPECT_MSG_EQ(m_reportingRntis.front(),
                          m_attachedRnti,
                          "The processed DL CQI report should belong to the attached UE");

    // Same for the UL CQI measured on the PUSCH transmission of the released UE.
    NrMacSchedSapProvider::SchedUlCqiInfoReqParameters ulCqiParams{};
    ulCqiParams.m_sfnSf = m_ulAllocSfnSf;
    ulCqiParams.m_symStart = m_ulAllocSymStart;
    ulCqiParams.m_ulCqi.m_type = UlCqiInfo::PUSCH;
    ulCqiParams.m_ulCqi.m_sinr = std::vector<double>(10, 10.0);
    sched->DoSchedUlCqiInfoReq(ulCqiParams);

    // And for an UL CQI whose allocation is not in the pending map at all.
    NrMacSchedSapProvider::SchedUlCqiInfoReqParameters staleUlCqiParams = ulCqiParams;
    staleUlCqiParams.m_sfnSf.Add(100);
    sched->DoSchedUlCqiInfoReq(staleUlCqiParams);

    // The scheduler must still be able to serve the remaining UE.
    sfn.Add(1);
    NrMacSchedSapProvider::SchedDlTriggerReqParameters dlTrigger;
    dlTrigger.m_snfSf = sfn;
    dlTrigger.m_slotType = (m_duplex == Duplex::TDD) ? LteNrTddSlotType::F : LteNrTddSlotType::DL;
    sched->DoSchedDlTriggerReq(dlTrigger);

    delete schedSapUser;
    delete cschedSapUser;
    Simulator::Destroy();
}

/**
 * @ingroup test
 * @brief Test suite for channel feedback referring to released UEs
 */
class NrTestSchedReleasedUeSuite : public TestSuite
{
  public:
    NrTestSchedReleasedUeSuite()
        : TestSuite("nr-test-sched-released-ue", Type::UNIT)
    {
        for (const auto& schedType : {"ns3::NrMacSchedulerTdmaRR", "ns3::NrMacSchedulerOfdmaRR"})
        {
            for (auto duplex : {NrSchedReleasedUeCqiTestCase::Duplex::TDD,
                                NrSchedReleasedUeCqiTestCase::Duplex::FDD})
            {
                AddTestCase(new NrSchedReleasedUeCqiTestCase(schedType, duplex), Duration::QUICK);
            }
        }
    }
};

/// Test suite instance
static NrTestSchedReleasedUeSuite nrTestSchedReleasedUeSuite;

} // namespace ns3
