// Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/beam-id.h"
#include "ns3/node.h"
#include "ns3/nr-control-messages.h"
#include "ns3/nr-gnb-mac.h"
#include "ns3/nr-mac-sched-sap.h"
#include "ns3/nr-mac-scheduler-ns3.h"
#include "ns3/nr-phy-sap.h"
#include "ns3/object-factory.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <algorithm>
#include <functional>

/**
 * @file nr-test-notching.cc
 * @ingroup test
 *
 * @brief This test is used to validate the notching functionality.
 * In order to do so, it creates a fake MAC and checks in the method
 * TestNotchingGnbMac::DoSchedConfigIndication() that RBG mask is
 * in the DCI is constructed in accordance with the (tested)
 * notching mask.
 *
 * @see TestNotchingGnbMac::DoSchedConfigIndication
 */
namespace ns3
{

class TestNotchingPhySapProvider : public NrPhySapProvider
{
  public:
    TestNotchingPhySapProvider();
    ~TestNotchingPhySapProvider() override;
    uint32_t GetSymbolsPerSlot() const override;
    Ptr<const SpectrumModel> GetSpectrumModel() override;
    uint16_t GetBwpId() const override;
    uint16_t GetCellId() const override;
    Time GetSlotPeriod() const override;
    void SendMacPdu(const Ptr<Packet>& p,
                    const SfnSf& sfn,
                    uint8_t symStart,
                    uint16_t rnti) override;
    void SendControlMessage(Ptr<NrControlMessage> msg) override;
    void SendRachPreamble(uint8_t PreambleId, uint8_t Rnti) override;
    void SetSlotAllocInfo(const SlotAllocInfo& slotAllocInfo) override;
    void NotifyConnectionSuccessful() override;
    uint32_t GetRbNum() const override;
    BeamId GetBeamId(uint8_t rnti) const override;
    void SetParams(uint32_t numOfUesPerBeam, uint32_t numOfBeams);
    void RegisterToGnb(uint16_t cellId) override;
    uint32_t GetArfcn() const override;

  private:
    uint32_t m_sapNumOfUesPerBeam = 0;
    uint32_t m_sapNumOfBeams = 0;
};

TestNotchingPhySapProvider::TestNotchingPhySapProvider()
{
}

TestNotchingPhySapProvider::~TestNotchingPhySapProvider()
{
}

void
TestNotchingPhySapProvider::SetParams(uint32_t numOfUesPerBeam, uint32_t numOfBeams)
{
    m_sapNumOfUesPerBeam = numOfUesPerBeam;
    m_sapNumOfBeams = numOfBeams;
}

uint32_t
TestNotchingPhySapProvider::GetSymbolsPerSlot() const
{
    // Fixed 14 symbols per slot.
    return 14;
}

Ptr<const SpectrumModel>
TestNotchingPhySapProvider::GetSpectrumModel()
{
    return nullptr;
}

uint16_t
TestNotchingPhySapProvider::GetBwpId() const
{
    return 0;
}

uint16_t
TestNotchingPhySapProvider::GetCellId() const
{
    return 0;
}

Time
TestNotchingPhySapProvider::GetSlotPeriod() const
{
    return MilliSeconds(1);
}

void
TestNotchingPhySapProvider::SendMacPdu(const Ptr<Packet>& p,
                                       const SfnSf& sfn,
                                       uint8_t symStart,
                                       uint16_t rnti)
{
}

void
TestNotchingPhySapProvider::SendControlMessage(Ptr<NrControlMessage> msg)
{
}

void
TestNotchingPhySapProvider::SendRachPreamble(uint8_t PreambleId, uint8_t Rnti)
{
}

void
TestNotchingPhySapProvider::SetSlotAllocInfo(const SlotAllocInfo& slotAllocInfo)
{
}

void
TestNotchingPhySapProvider::NotifyConnectionSuccessful()
{
}

uint32_t
TestNotchingPhySapProvider::GetRbNum() const
{
    // If in the future the scheduler calls this method, remove this assert"
    NS_FATAL_ERROR("GetRbNum should not be called");
    return 53;
}

BeamId
TestNotchingPhySapProvider::GetBeamId(uint8_t rnti) const
{
    BeamId beamId = BeamId(0, 0.0);
    uint8_t rntiCnt = 1;
    for (uint32_t beam = 0; beam < m_sapNumOfUesPerBeam; beam++)
    {
        for (uint32_t u = 0; u < m_sapNumOfBeams; u++)
        {
            if (rnti == 0 || (rntiCnt == rnti && beam == 0))
            {
                beamId = BeamId(0, 0.0);
            }
            else if (rntiCnt == rnti && beam == 1)
            {
                beamId = BeamId(1, 120.0);
            }
            rntiCnt++;
        }
    }
    return beamId;
}

void
TestNotchingPhySapProvider::RegisterToGnb(uint16_t cellId)
{
}

uint32_t
TestNotchingPhySapProvider::GetArfcn() const
{
    return 0;
}

class TestNotchingGnbMac : public NrGnbMac
{
  public:
    static TypeId GetTypeId();
    TestNotchingGnbMac(const std::vector<bool>& inputMask);
    ~TestNotchingGnbMac() override;
    void DoSchedConfigIndication(NrMacSchedSapUser::SchedConfigIndParameters ind) override;
    void SetVerbose(bool verbose);

    /**
     * @return the number of data DCIs checked against the mask
     */
    uint32_t GetNumCheckedDcis() const
    {
        return m_numCheckedDcis;
    }

  private:
    std::vector<bool> m_inputMask;
    bool m_verboseMac = false;
    uint32_t m_numCheckedDcis{0}; //!< Number of data DCIs checked against the mask
};

NS_OBJECT_ENSURE_REGISTERED(TestNotchingGnbMac);

TypeId
TestNotchingGnbMac::GetTypeId()
{
    static TypeId tid = TypeId("ns3::TestNotchingGnbMac").SetParent<NrGnbMac>()
        //.AddConstructor<TestNotchingGnbMac> ()
        ;
    return tid;
}

TestNotchingGnbMac::TestNotchingGnbMac(const std::vector<bool>& inputMask)
{
    m_inputMask = inputMask;
}

TestNotchingGnbMac::~TestNotchingGnbMac()
{
}

void
TestNotchingGnbMac::SetVerbose(bool verbose)
{
    m_verboseMac = verbose;
}

void
TestNotchingGnbMac::DoSchedConfigIndication(NrMacSchedSapUser::SchedConfigIndParameters ind)
{
    // Check that the allocations in `ind` have the correct RBG mask
    // Will be called after SchedDlTriggerReq is called
    // test that ind.m_slotAllocInfo is ok: the sfnf, and the varAlloc deque

    for (auto& varTtiAllocInfo : ind.m_slotAllocInfo.m_varTtiAllocInfo)
    {
        // Only data DCIs carry an RBG mask to check: skip control and SRS DCIs
        if (varTtiAllocInfo.m_dci->m_rnti == 0 ||
            varTtiAllocInfo.m_dci->m_type != DciInfoElementTdma::DATA)
        {
            continue;
        }

        if (m_verboseMac)
        {
            std::ostringstream oss;
            for (auto x : varTtiAllocInfo.m_dci->m_rbgBitmask)
            {
                oss << std::to_string(x) << " ";
            }

            std::cout << "UE " << varTtiAllocInfo.m_dci->m_rnti << " assigned RBG"
                      << " with mask: " << oss.str() << std::endl;
        }

        // Count the DCIs, so that the test fails if nothing was checked
        m_numCheckedDcis++;
        NS_ASSERT_MSG(varTtiAllocInfo.m_dci->m_rbgBitmask.size() == m_inputMask.size(),
                      "dci bitmask is not of same size as the mask");

        unsigned zeroes = std::count(varTtiAllocInfo.m_dci->m_rbgBitmask.begin(),
                                     varTtiAllocInfo.m_dci->m_rbgBitmask.end(),
                                     0);

        NS_ASSERT_MSG(zeroes != m_inputMask.size(), "dci rbgBitmask is filled with zeros");

        for (unsigned index = 0; index < varTtiAllocInfo.m_dci->m_rbgBitmask.size(); index++)
        {
            if (m_inputMask[index] == 0)
            {
                NS_ASSERT_MSG(varTtiAllocInfo.m_dci->m_rbgBitmask[index] == 0,
                              "dci is diff from mask");
            }
        }
    }
}

/**
 * @brief TestCase for the notching mask
 */
class NrNotchingTestCase : public TestCase
{
  public:
    /**
     * @brief Create NrNotchingTestCase
     * @param name Name of the test
     * @param mask The mask to be tested
     * @param schedulerType The type of the scheduler to be tested
     * @param numOfUesPerBeam The number of UEs per beam to be tested
     * @param beamsNum The number beams to be tested
     * @param isUl If true, test UL allocations against the UL mask, with a DL mask that
     * notches the RBGs the UL mask allows
     */
    NrNotchingTestCase(const std::string& name,
                       const std::vector<bool>& mask,
                       const std::string& schedulerType,
                       uint32_t numOfUesPerBeam,
                       uint32_t beamsNum,
                       bool isUl = false)
        : TestCase(name),
          m_mask(mask),
          m_schedulerType(schedulerType),
          m_numOfUesPerBeam(numOfUesPerBeam),
          m_beamsNum(beamsNum),
          m_isUl(isUl)
    {
    }

    ~NrNotchingTestCase() override;

  private:
    void DoRun() override;
    Ptr<NrMacSchedulerNs3> CreateScheduler(const std::string& schedulerType) const;
    Ptr<TestNotchingGnbMac> CreateMac(
        Ptr<NrMacSchedulerNs3>& scheduler,
        NrMacCschedSapProvider::CschedCellConfigReqParameters& params) const;

    bool m_verbose = false;
    const std::vector<bool> m_mask;
    const std::string m_schedulerType;
    uint32_t m_numOfUesPerBeam;
    uint32_t m_beamsNum;
    bool m_isUl; //!< Whether UL allocations are tested
    TestNotchingPhySapProvider* m_phySapProvider;
};

NrNotchingTestCase::~NrNotchingTestCase()
{
}

Ptr<NrMacSchedulerNs3>
NrNotchingTestCase::CreateScheduler(const std::string& schedulerType) const
{
    ObjectFactory schedFactory;
    schedFactory.SetTypeId(schedulerType);
    Ptr<NrMacSchedulerNs3> sched = DynamicCast<NrMacSchedulerNs3>(schedFactory.Create());
    NS_ABORT_MSG_IF(sched == nullptr,
                    "Can't create a NrMacSchedulerNs3 from type " + schedulerType);

    return sched;
}

Ptr<TestNotchingGnbMac>
NrNotchingTestCase::CreateMac(Ptr<NrMacSchedulerNs3>& scheduler,
                              NrMacCschedSapProvider::CschedCellConfigReqParameters& params) const
{
    Ptr<TestNotchingGnbMac> mac = CreateObject<TestNotchingGnbMac>(m_mask);
    // 1 RB per RBG, as the masks assume.
    // Without it the TBS is 0 and the scheduler creates no data DCI.
    mac->SetNumRbPerRbg(1);

    mac->SetNrMacSchedSapProvider(scheduler->GetMacSchedSapProvider());
    mac->SetNrMacCschedSapProvider(scheduler->GetMacCschedSapProvider());
    scheduler->SetMacSchedSapUser(mac->GetNrMacSchedSapUser());
    scheduler->SetMacCschedSapUser(mac->GetNrMacCschedSapUser());
    if (m_isUl)
    {
        // Notch in DL exactly the RBGs that the UL mask allows.
        // UL DCIs checked against the DL mask would then hit notched RBGs.
        std::vector<bool> dlMask(m_mask.size());
        std::transform(m_mask.begin(), m_mask.end(), dlMask.begin(), std::logical_not<>());
        scheduler->SetDlNotchedRbgMask(dlMask);
    }
    else
    {
        scheduler->SetDlNotchedRbgMask(m_mask);
    }
    scheduler->SetUlNotchedRbgMask(m_mask);
    // Config sched
    scheduler->DoCschedCellConfigReq(params);

    return mac;
}

void
NrNotchingTestCase::DoRun()
{
    NrMacCschedSapProvider::CschedCellConfigReqParameters params;
    params.m_ulBandwidth = m_mask.size();
    params.m_dlBandwidth = m_mask.size();

    auto sched = CreateScheduler(m_schedulerType);
    auto mac = CreateMac(sched, params);

    m_phySapProvider = new TestNotchingPhySapProvider();
    m_phySapProvider->SetParams(m_beamsNum, m_numOfUesPerBeam);

    mac->SetPhySapProvider(m_phySapProvider);
    mac->SetVerbose(m_verbose);

    Ptr<NrAmc> amc = CreateObject<NrAmc>();
    sched->InstallDlAmc(amc);
    // The UL scheduler needs its own AMC to compute UL TBSs
    sched->InstallUlAmc(CreateObject<NrAmc>());

    // uint32_t rbPerRbg = mac->GetNumRbPerRbg ();
    // std::cout << "numRbPerRbg: " << rbPerRbg << std::endl;

    uint16_t rntiCnt = 1;
    for (uint32_t beam = 0; beam < m_beamsNum; beam++)
    {
        for (uint32_t u = 0; u < m_numOfUesPerBeam; u++)
        {
            NrMacCschedSapProvider::CschedUeConfigReqParameters paramsUe;
            paramsUe.m_rnti = rntiCnt;
            paramsUe.m_beamId = m_phySapProvider->GetBeamId(rntiCnt);

            if (m_verbose)
            {
                std::cout << "beam: " << beam << " ue: " << u << " rnti: " << paramsUe.m_rnti
                          << " beam Id: " << paramsUe.m_beamId << " scheduler: " << m_schedulerType
                          << std::endl;
                if (beam == (m_beamsNum - 1) && u == (m_numOfUesPerBeam - 1))
                {
                    std::ostringstream ss;
                    for (auto x : m_mask)
                    {
                        ss << std::to_string(x) << " ";
                    }

                    std::cout << "The defined mask is:         " << ss.str() << std::endl;
                }
            }

            // Add Users
            sched->DoCschedUeConfigReq(paramsUe); // Repeat for the number of UEs

            // Create LC
            NrMacCschedSapProvider::CschedLcConfigReqParameters paramsLc;
            paramsLc.m_rnti = rntiCnt;
            paramsLc.m_reconfigureFlag = false;

            nr::LogicalChannelConfigListElement_s lc;
            lc.m_logicalChannelIdentity = 1;
            lc.m_logicalChannelGroup = 2;
            // The logical channel carries data in the tested direction
            lc.m_direction = m_isUl ? nr::LogicalChannelConfigListElement_s::DIR_UL
                                    : nr::LogicalChannelConfigListElement_s::DIR_DL;
            lc.m_qosBearerType = nr::LogicalChannelConfigListElement_s::QBT_NON_GBR;
            lc.m_fiveQi = 9;
            paramsLc.m_logicalChannelConfigList.emplace_back(lc);

            sched->DoCschedLcConfigReq(paramsLc);

            // Update queue
            NrMacSchedSapProvider::SchedDlRlcBufferReqParameters paramsDlRlc;
            paramsDlRlc.m_rnti = rntiCnt;
            paramsDlRlc.m_logicalChannelIdentity = 1;
            paramsDlRlc.m_rlcRetransmissionHolDelay = 0;
            paramsDlRlc.m_rlcRetransmissionQueueSize = 0;
            paramsDlRlc.m_rlcStatusPduSize = 0;
            paramsDlRlc.m_rlcTransmissionQueueHolDelay = 0;
            paramsDlRlc.m_rlcTransmissionQueueSize = 1284;

            if (m_isUl)
            {
                // UL data is announced by a BSR, here 30 bytes in LCG 2
                NrMacSchedSapProvider::SchedUlMacCtrlInfoReqParameters bsrParams;
                MacCeElement bsr;
                bsr.m_rnti = rntiCnt;
                bsr.m_macCeType = MacCeElement::BSR;
                bsr.m_macCeValue.m_bufferStatus = {0, 0, 30, 0};
                bsrParams.m_macCeList.push_back(bsr);
                sched->DoSchedUlMacCtrlInfoReq(bsrParams);
            }
            else
            {
                // DL data is queued at the RLC
                sched->DoSchedDlRlcBufferReq(paramsDlRlc);
            }

            rntiCnt++;
        }
    }

    // Call scheduling for one slot in the tested direction.
    // The MAC checks the DCIs against the mask when the slot is reported.
    if (m_isUl)
    {
        NrMacSchedSapProvider::SchedUlTriggerReqParameters paramsUlTrigger;
        paramsUlTrigger.m_snfSf = SfnSf(0, 0, 0, 0);
        paramsUlTrigger.m_slotType = LteNrTddSlotType::UL;
        sched->DoSchedUlTriggerReq(paramsUlTrigger);
    }
    else
    {
        NrMacSchedSapProvider::SchedDlTriggerReqParameters paramsDlTrigger;
        paramsDlTrigger.m_snfSf = SfnSf(0, 0, 0, 0);
        paramsDlTrigger.m_slotType = LteNrTddSlotType::DL;
        sched->DoSchedDlTriggerReq(paramsDlTrigger);
    }

    // Fails if no data DCI was scheduled, which would make the test vacuous
    NS_TEST_EXPECT_MSG_GT(mac->GetNumCheckedDcis(), 0, "No data allocation was checked");

    delete m_phySapProvider;
    Simulator::Destroy();
}

class NrNotchingTestSuite : public TestSuite
{
  public:
    NrNotchingTestSuite()
        : TestSuite("nr-test-notching", Type::UNIT)
    {
        // We simulate BW of 10 MHz so the size of the mask is 53 RBGs
        // considering that 1 RBG contains 1 RB
        // NOLINTBEGIN
        std::vector<bool> notchedMask1{0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1,
                                       1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1,
                                       1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1};

        std::vector<bool> notchedMask2{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1,
                                       1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1,
                                       1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0};
        // NOLINTEND
        std::list<std::string> subdivision = {"Tdma", "Ofdma"};
        std::list<std::string> scheds = {"RR"};
        std::list<uint32_t> uesPerBeamList = {1, 2, 4, 6};
        std::list<uint32_t> beams = {1, 2};

        for (const auto& subType : subdivision)
        {
            for (const auto& sched : scheds)
            {
                for (const auto& uesPerBeam : uesPerBeamList)
                {
                    for (const auto& beam : beams)
                    {
                        std::stringstream ss;
                        std::stringstream schedName;
                        ss << ", " << subType << " " << sched << ", " << uesPerBeam
                           << " UE per beam, " << beam << " beam";

                        schedName << "ns3::NrMacScheduler" << subType << sched;

                        AddTestCase(new NrNotchingTestCase(ss.str(),
                                                           notchedMask1,
                                                           schedName.str(),
                                                           uesPerBeam,
                                                           beam),
                                    Duration::QUICK);
                        AddTestCase(new NrNotchingTestCase(ss.str(),
                                                           notchedMask2,
                                                           schedName.str(),
                                                           uesPerBeam,
                                                           beam),
                                    Duration::QUICK);
                        // Same cases in UL, with the DL mask notching the UL-allowed RBGs
                        AddTestCase(new NrNotchingTestCase("UL" + ss.str(),
                                                           notchedMask1,
                                                           schedName.str(),
                                                           uesPerBeam,
                                                           beam,
                                                           true),
                                    Duration::QUICK);
                        AddTestCase(new NrNotchingTestCase("UL" + ss.str(),
                                                           notchedMask2,
                                                           schedName.str(),
                                                           uesPerBeam,
                                                           beam,
                                                           true),
                                    Duration::QUICK);
                    }
                }
            }
        }
    }
};

static NrNotchingTestSuite nrNotchingTestSuite;

//!< Nr Notching test suite

} // namespace ns3
