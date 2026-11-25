// Copyright (c) 2025 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/config.h"
#include "ns3/nr-mac-sched-sap.h"
#include "ns3/nr-mac-scheduler-ns3.h"
#include "ns3/nr-mac-scheduler-ofdma-rr.h"
#include "ns3/nr-mac-scheduler-ofdma-symbol-per-beam.h"
#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/nr-mac-short-bsr-ce.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/object-factory.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/uinteger.h"

#include <array>
#include <map>
#include <set>

/**
 * @file nr-test-sched-harq.cc
 * @ingroup test
 *
 * @brief This file contain tests for the round-robin nature of nr-mac-scheduler-harq-rr.
 * It also tests if allocations are properly consolidated to use less symbols,
 * and maintain or increase MCS, in order to increase the change of successful decoding.
 *
 */
namespace ns3
{
class TestCschedSapUserHarq : public NrMacCschedSapUser
{
  public:
    TestCschedSapUserHarq()
        : NrMacCschedSapUser()
    {
    }

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

class TestSchedSapUserHarq : public NrMacSchedSapUser
{
  public:
    TestSchedSapUserHarq(
        std::function<void(const struct SchedConfigIndParameters&)> schedConfigIndCallback =
            [](auto params) {},
        std::function<uint32_t()> symbolsPerSlotCallback = []() { return 14; })
        : NrMacSchedSapUser(),
          m_schedConfigIndCallback(schedConfigIndCallback),
          m_symbolsPerSlotCallback(symbolsPerSlotCallback)
    {
    }

    void SchedConfigInd(const NrMacSchedSapUser::SchedConfigIndParameters& params) override
    {
        if (m_schedConfigIndCallback)
        {
            m_schedConfigIndCallback(params);
        }
    }

    // For the rest, setup some hard-coded values; for the moment, there is
    // no need to have real values here.
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
        return m_symbolsPerSlotCallback();
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
        m_schedConfigIndCallback;
    std::function<uint32_t()> m_symbolsPerSlotCallback;
};

/**
 * @brief TestSched testcase
 */
class NrTestMacSchedulerHarqRrReshape : public TestCase
{
  public:
    /**
     * @brief Create NrSchedGeneralTestCase
     * @param scheduler Scheduler to test
     * @param name Name of the test
     */
    NrTestMacSchedulerHarqRrReshape(std::vector<DciInfoElementTdma> dcis,
                                    uint8_t startingSymbol,
                                    uint8_t numSymbols,
                                    const std::string& name)
        : TestCase(name),
          m_dcis(dcis),
          m_startingSymbol(startingSymbol),
          m_numSymbols(numSymbols)
    {
    }

  protected:
    void DoRun() override;
    const std::vector<DciInfoElementTdma> m_dcis;
    const uint8_t m_startingSymbol;
    const uint8_t m_numSymbols;
};

void
NrTestMacSchedulerHarqRrReshape::DoRun()
{
    // Prepare common settings for both TDMA and OFDMA schedulers
    auto cellConfig = NrMacCschedSapProvider::CschedCellConfigReqParameters();
    cellConfig.m_dlBandwidth = 10; // 10 RBGs
    cellConfig.m_ulBandwidth = 10;

    std::vector<NrMacCschedSapProvider::CschedUeConfigReqParameters> ueConfig;
    for (const auto& dci : m_dcis)
    {
        NrMacCschedSapProvider::CschedUeConfigReqParameters config{};
        config.m_rnti = dci.m_rnti;
        config.m_transmissionMode = 0;
        config.m_beamId = BeamId(dci.m_rnti / 5, 0);
        ueConfig.push_back(config);
    }

    auto* schedSapUser = new TestSchedSapUserHarq();
    auto* cschedSapUser = new TestCschedSapUserHarq();

    for (bool isTdma : {true, false})
    {
        Ptr<NrMacSchedulerNs3> scheduler;
        if (isTdma)
        {
            scheduler = CreateObject<NrMacSchedulerTdmaRR>();
        }
        else
        {
            scheduler = CreateObject<NrMacSchedulerOfdmaRR>();
        }
        scheduler->SetMacSchedSapUser(schedSapUser);
        scheduler->SetMacCschedSapUser(cschedSapUser);
        scheduler->DoCschedCellConfigReq(cellConfig);
        for (const auto& ueConf : ueConfig)
        {
            scheduler->DoCschedUeConfigReq(ueConf);
        }
        const bool isDl = true;
        std::vector<bool> bitmask(10, true);
        auto startingSymbol = m_startingSymbol;
        auto numSymbols = m_numSymbols;
        auto reshapedDcisTdma =
            scheduler->ReshapeAllocation(m_dcis, startingSymbol, numSymbols, bitmask, isDl);

        // Check if we went above number of available symbols
        auto reshapedAllocatedSymbols = 0;
        if (!reshapedDcisTdma.empty())
        {
            auto smallestStartSymbol = std::numeric_limits<uint8_t>::max();
            auto largestFinalSymbol = m_startingSymbol;
            for (auto& dci : reshapedDcisTdma)
            {
                if (smallestStartSymbol > dci.m_symStart)
                {
                    smallestStartSymbol = dci.m_symStart;
                }
                if (largestFinalSymbol < (dci.m_symStart + dci.m_numSym))
                {
                    largestFinalSymbol = dci.m_symStart + dci.m_numSym;
                }
            }
            NS_ASSERT_MSG(smallestStartSymbol != std::numeric_limits<uint8_t>::max(),
                          "There must have been a valid starting symbol");
            reshapedAllocatedSymbols = largestFinalSymbol - smallestStartSymbol;
        }
        NS_TEST_ASSERT_MSG_LT_OR_EQ(
            reshapedAllocatedSymbols,
            +m_numSymbols,
            (isTdma ? "TDMA" : "OFDMA")
                << ": Reshaped unexpectedly into more symbols than available");

        // If the test case has no symbols, do not continue which checks, because the one above
        // should suffice
        if (m_numSymbols == 0)
        {
            continue;
        }

        // If there is no reshaped DCI, we do not continue checks
        // (temporary until reshape can handle multiple DCIs, and later multiple beams)
        if (reshapedDcisTdma.empty())
        {
            continue;
        }

        // Test we haven't changed what we are not supposed to change
        for (auto& reshapedDci : reshapedDcisTdma)
        {
            const auto originalDci =
                std::find_if(m_dcis.begin(), m_dcis.end(), [=](DciInfoElementTdma a) {
                    return reshapedDci.m_rnti == a.m_rnti;
                });
            NS_TEST_EXPECT_MSG_EQ((originalDci != m_dcis.end()),
                                  true,
                                  "Reshaped allocation changed DCI RNTI");
            NS_TEST_ASSERT_MSG_EQ(reshapedDci.m_format,
                                  originalDci->m_format,
                                  "Reshaped allocation changed DCI format");
            NS_TEST_ASSERT_MSG_EQ(+reshapedDci.m_mcs,
                                  +originalDci->m_mcs,
                                  "Reshaped allocation changed DCI MCS");
            NS_TEST_ASSERT_MSG_EQ(+reshapedDci.m_rank,
                                  +originalDci->m_rank,
                                  "Reshaped allocation changed DCI rank");
            NS_TEST_ASSERT_MSG_EQ(reshapedDci.m_precMats,
                                  originalDci->m_precMats,
                                  "Reshaped allocation changed DCI Precoding matrices");
            NS_TEST_ASSERT_MSG_EQ(reshapedDci.m_tbSize,
                                  originalDci->m_tbSize,
                                  "Reshaped allocation changed DCI TBS");
            NS_TEST_ASSERT_MSG_EQ(reshapedDci.m_ndi,
                                  originalDci->m_ndi,
                                  "Reshaped allocation changed DCI NDI");
            NS_TEST_ASSERT_MSG_EQ(reshapedDci.m_rv,
                                  originalDci->m_rv,
                                  "Reshaped allocation changed DCI HARQ RV");
            NS_TEST_ASSERT_MSG_EQ(reshapedDci.m_type,
                                  originalDci->m_type,
                                  "Reshaped allocation changed DCI type");
            NS_TEST_ASSERT_MSG_EQ(reshapedDci.m_bwpIndex,
                                  originalDci->m_bwpIndex,
                                  "Reshaped allocation changed DCI BWP index");
            NS_TEST_ASSERT_MSG_EQ(reshapedDci.m_tpc,
                                  originalDci->m_tpc,
                                  "Reshaped allocation changed DCI TPC");

            // Test if we changed what we are supposed to change
            if (isTdma)
            {
                NS_TEST_ASSERT_MSG_GT_OR_EQ(
                    +std::count(reshapedDci.m_rbgBitmask.begin(),
                                reshapedDci.m_rbgBitmask.end(),
                                true),
                    +std::count(originalDci->m_rbgBitmask.begin(),
                                originalDci->m_rbgBitmask.end(),
                                true),
                    "Reshaped TDMA allocation unexpectedly has less RBGs than the original");
                NS_TEST_ASSERT_MSG_LT_OR_EQ(
                    +reshapedDci.m_numSym,
                    +originalDci->m_numSym,
                    "Reshaped TDMA allocation unexpectedly has more symbols than the original");
                NS_TEST_ASSERT_MSG_GT_OR_EQ(
                    reshapedDci.m_numSym * std::count(reshapedDci.m_rbgBitmask.begin(),
                                                      reshapedDci.m_rbgBitmask.end(),
                                                      true),
                    originalDci->m_numSym * std::count(originalDci->m_rbgBitmask.begin(),
                                                       originalDci->m_rbgBitmask.end(),
                                                       true),
                    "Reshaped TDMA allocation unexpectedly has less resources than the original");
            }
            else
            {
            }
        }
    }
    delete schedSapUser;
    delete cschedSapUser;
    Simulator::Destroy();
}

class NrTestMacSchedulerHarqRrScheduleDlHarq : public NrTestMacSchedulerHarqRrReshape
{
  public:
    /**
     * @brief Create NrSchedGeneralTestCase
     * @param scheduler Scheduler to test
     * @param name Name of the test
     */
    NrTestMacSchedulerHarqRrScheduleDlHarq(std::vector<DciInfoElementTdma> dcis,
                                           uint8_t startingSymbol,
                                           uint8_t numSymbols,
                                           const std::string& name)
        : NrTestMacSchedulerHarqRrReshape(dcis, startingSymbol, numSymbols, name)
    {
    }

    void CheckSchedule(const NrMacSchedSapUser::SchedConfigIndParameters& params);

  protected:
    void DoRun() override;
    bool m_testingTdma{false};
};

void
NrTestMacSchedulerHarqRrScheduleDlHarq::CheckSchedule(
    const NrMacSchedSapUser::SchedConfigIndParameters& params)
{
    // Retrieve resulting scheduled HARQ DCIs
    std::vector<DciInfoElementTdma> resultingDcis;
    for (const auto& varTtiAllocInfo : params.m_slotAllocInfo.m_varTtiAllocInfo)
    {
        // We only want data, no control DCIs
        if (varTtiAllocInfo.m_dci->m_type == DciInfoElementTdma::DATA)
        {
            resultingDcis.push_back(*varTtiAllocInfo.m_dci);
        }
    }

    if (m_testingTdma)
    {
        // TDMA should minimize symbols used by HARQ allocations, to fit more HARQ retransmissions
        // in a slot
        for (auto& resultingDci : resultingDcis)
        {
            auto& originalDci =
                *std::find_if(m_dcis.begin(), m_dcis.end(), [resultingDci](auto& a) {
                    return a.m_rnti == resultingDci.m_rnti;
                });
            NS_TEST_EXPECT_MSG_LT_OR_EQ(+resultingDci.m_numSym,
                                        +originalDci.m_numSym,
                                        "Number of symbols for TDMA should be same or smaller");

            auto originalRbgs =
                std::count(originalDci.m_rbgBitmask.begin(), originalDci.m_rbgBitmask.end(), true);
            auto resultingRbgs = std::count(resultingDci.m_rbgBitmask.begin(),
                                            resultingDci.m_rbgBitmask.end(),
                                            true);
            // The reshaped grant keeps the TBS, so it must hold at least the original
            // resources, padded by less than one RBG per symbol
            NS_TEST_ASSERT_MSG_GT_OR_EQ(resultingRbgs * resultingDci.m_numSym,
                                        originalRbgs * originalDci.m_numSym,
                                        "Reshaped grant must hold the original resources");
            NS_TEST_ASSERT_MSG_LT(resultingRbgs * resultingDci.m_numSym,
                                  originalRbgs * originalDci.m_numSym + resultingDci.m_numSym,
                                  "Reshaped grant must not pad more than one RBG per symbol");
        }
    }
    // Testing OFDMA
    else
    {
        // OFDMA should maximize symbols used by HARQ retransmissions, in order to make better use
        // of RBGs for other retransmissions. However, the total number of resources of
        // retransmissions of a symbol should use the least amount of symbols possible, to (if
        // possible) have more beams in a given slot.
    }
}

void
NrTestMacSchedulerHarqRrScheduleDlHarq::DoRun()
{
    Config::SetDefault("ns3::NrMacSchedulerHarqRr::ConsolidateHarqRetx", BooleanValue(true));

    // Prepare common settings for both TDMA and OFDMA schedulers
    auto cellConfig = NrMacCschedSapProvider::CschedCellConfigReqParameters();
    cellConfig.m_dlBandwidth = 10; // 10 RBGs
    cellConfig.m_ulBandwidth = 10;

    std::vector<NrMacCschedSapProvider::CschedUeConfigReqParameters> ueConfig;
    for (const auto& dci : m_dcis)
    {
        NrMacCschedSapProvider::CschedUeConfigReqParameters config{};
        config.m_rnti = dci.m_rnti;
        config.m_transmissionMode = 0;
        config.m_beamId = BeamId(dci.m_rnti / 5, 0);
        ueConfig.push_back(config);
    }

    auto* schedSapUser = new TestSchedSapUserHarq([this](auto params) { CheckSchedule(params); },
                                                  [this]() { return m_numSymbols; });
    auto* cschedSapUser = new TestCschedSapUserHarq();

    Ptr<NrMacSchedulerNs3> sched;

    // Instead of using of reshaping straight from scheduler,
    // reproduce the conditions to call it via the scheduler->HARQ scheduler->reshape
    for (bool isTdma : {true, false})
    {
        // Create scheduler
        if (isTdma)
        {
            sched = CreateObject<NrMacSchedulerTdmaRR>();
        }
        else
        {
            sched = CreateObject<NrMacSchedulerOfdmaRR>();
        }
        sched->InstallDlAmc(CreateObject<NrAmc>());
        sched->InstallUlAmc(CreateObject<NrAmc>());

        // Configure scheduler
        sched->SetMacSchedSapUser(schedSapUser);
        sched->SetMacCschedSapUser(cschedSapUser);
        sched->DoCschedCellConfigReq(cellConfig);
        for (const auto& ueConf : ueConfig)
        {
            sched->DoCschedUeConfigReq(ueConf);
        }

        // Set starting symbol
        sched->SetDlCtrlSyms(m_startingSymbol);

        // Create scheduler parameters
        NrMacSchedSapProvider::SchedDlTriggerReqParameters paramsDlTrigger;
        paramsDlTrigger.m_snfSf = SfnSf(0, 0, 0, 0);
        paramsDlTrigger.m_slotType = LteNrTddSlotType::DL;
        paramsDlTrigger.m_dlHarqInfoList = {};

        // Activate harq processes and populate harq info list
        for (const auto& dci : m_dcis)
        {
            auto& ueInfo = sched->m_ueMap.find(dci.m_rnti)->second;
            auto& harqProcess = ueInfo->m_dlHarq.Find(dci.m_rnti)->second;
            harqProcess.m_dciElement = std::make_shared<DciInfoElementTdma>(dci);
            harqProcess.m_active = true;

            DlHarqInfo harqInfo;
            harqInfo.m_harqStatus = DlHarqInfo::NACK;
            harqInfo.m_numRetx = 0;
            harqInfo.m_rnti = dci.m_rnti;
            harqInfo.m_harqProcessId = dci.m_rnti;
            harqInfo.m_bwpIndex = 0;
            paramsDlTrigger.m_dlHarqInfoList.push_back(harqInfo);
        }

        // Indicate CheckSchedule should check for TDMA or OFDMA
        m_testingTdma = isTdma;

        // Call ScheduleDl
        sched->DoSchedDlTriggerReq(paramsDlTrigger);
    }
    delete schedSapUser;
    delete cschedSapUser;
    Simulator::Destroy();
}

/**
 * @brief Check the HARQ round-robin beam ordering when the queue of known beams
 * is larger than the set of beams with retransmissions
 *
 * NrMacSchedulerHarqRr keeps a persistent round-robin queue of every beam ever
 * seen (m_dlBeams and m_ulBeams, one per link direction), while
 * GetBeamOrderRR() returns only the beams with retransmissions in the current
 * slot. The two sizes differ as soon as a known beam has nothing to retransmit.
 *
 * This test reuses a single scheduler across several DL HARQ scheduling rounds
 * with a shrinking set of active beams, so that the persistent queue is larger
 * than the active-beam set, and checks that scheduling completes without memory
 * corruption.
 */
class NrTestMacSchedulerHarqRrBeamOrder : public TestCase
{
  public:
    NrTestMacSchedulerHarqRrBeamOrder()
        : TestCase("HARQ RR beam order does not overflow with shrinking active beams")
    {
    }

  protected:
    void DoRun() override;

  private:
    /**
     * @brief Drive a single DL HARQ scheduling round on the given scheduler.
     * @param sched Scheduler under test (reused across rounds)
     * @param activeRntis RNTIs (one per beam) whose HARQ process is NACKed in
     *                    this round
     */
    void ScheduleRound(Ptr<NrMacSchedulerNs3> sched, const std::vector<uint16_t>& activeRntis);
};

void
NrTestMacSchedulerHarqRrBeamOrder::ScheduleRound(Ptr<NrMacSchedulerNs3> sched,
                                                 const std::vector<uint16_t>& activeRntis)
{
    NrMacSchedSapProvider::SchedDlTriggerReqParameters paramsDlTrigger;
    paramsDlTrigger.m_snfSf = SfnSf(0, 0, 0, 0);
    paramsDlTrigger.m_slotType = LteNrTddSlotType::DL;
    paramsDlTrigger.m_dlHarqInfoList = {};

    for (auto rnti : activeRntis)
    {
        auto& ueInfo = sched->m_ueMap.find(rnti)->second;
        auto& harqProcess = ueInfo->m_dlHarq.Find(rnti)->second;

        DciInfoElementTdma dci(rnti,
                               DciInfoElementTdma::DL,
                               0,
                               1,
                               10,
                               1,
                               {},
                               100,
                               0,
                               0,
                               DciInfoElementTdma::DATA,
                               0,
                               0);
        dci.m_harqProcess = static_cast<uint8_t>(rnti);
        dci.m_rbgBitmask = std::vector<bool>(10, false);
        dci.m_rbgBitmask.at(0) = true;
        harqProcess.m_dciElement = std::make_shared<DciInfoElementTdma>(dci);
        harqProcess.m_active = true;
        harqProcess.m_status = HarqProcess::WAITING_FEEDBACK;
        // Each round is a fresh first transmission for this beam-ordering test;
        // reset the attempt counter so it does not accumulate to the retx cap.
        harqProcess.m_txAttempts = 1;

        DlHarqInfo harqInfo;
        harqInfo.m_harqStatus = DlHarqInfo::NACK;
        harqInfo.m_numRetx = 0;
        harqInfo.m_rnti = rnti;
        harqInfo.m_harqProcessId = static_cast<uint8_t>(rnti);
        harqInfo.m_bwpIndex = 0;
        paramsDlTrigger.m_dlHarqInfoList.push_back(harqInfo);
    }

    sched->DoSchedDlTriggerReq(paramsDlTrigger);
}

void
NrTestMacSchedulerHarqRrBeamOrder::DoRun()
{
    auto cellConfig = NrMacCschedSapProvider::CschedCellConfigReqParameters();
    cellConfig.m_dlBandwidth = 10; // 10 RBGs
    cellConfig.m_ulBandwidth = 10;

    // Configure one UE per beam, with as many beams as RNTIs.
    std::vector<NrMacCschedSapProvider::CschedUeConfigReqParameters> ueConfig;
    const uint16_t numUes = 8;
    for (uint16_t rnti = 0; rnti < numUes; ++rnti)
    {
        NrMacCschedSapProvider::CschedUeConfigReqParameters config{};
        config.m_rnti = rnti;
        config.m_transmissionMode = 0;
        config.m_beamId = BeamId(rnti, 0); // distinct beam per UE
        ueConfig.push_back(config);
    }

    auto* schedSapUser = new TestSchedSapUserHarq([](auto params) {}, []() { return 14; });
    auto* cschedSapUser = new TestCschedSapUserHarq();

    auto sched = CreateObject<NrMacSchedulerTdmaRR>();
    sched->InstallDlAmc(CreateObject<NrAmc>());
    sched->InstallUlAmc(CreateObject<NrAmc>());
    sched->SetMacSchedSapUser(schedSapUser);
    sched->SetMacCschedSapUser(cschedSapUser);
    sched->DoCschedCellConfigReq(cellConfig);
    for (const auto& ueConf : ueConfig)
    {
        sched->DoCschedUeConfigReq(ueConf);
    }
    sched->SetDlCtrlSyms(0);

    // First round: all beams active, so the persistent round-robin queue learns
    // every beam.
    std::vector<uint16_t> allRntis;
    allRntis.reserve(numUes);
    for (uint16_t rnti = 0; rnti < numUes; ++rnti)
    {
        allRntis.push_back(rnti);
    }
    ScheduleRound(sched, allRntis);

    // Subsequent rounds: progressively fewer active beams, while the persistent
    // queue still holds all of them. An out-of-bounds access when building the
    // beam order would abort the test here.
    for (uint16_t active = numUes - 1; active >= 1 && active < numUes; --active)
    {
        std::vector<uint16_t> rntis;
        rntis.reserve(active);
        for (uint16_t rnti = 0; rnti < active; ++rnti)
        {
            rntis.push_back(rnti);
        }
        ScheduleRound(sched, rntis);
    }

    NS_TEST_EXPECT_MSG_EQ(true, true, "HARQ RR beam ordering completed without memory corruption");

    delete schedSapUser;
    delete cschedSapUser;
    Simulator::Destroy();
}

/**
 * @brief Check that the DL HARQ symbol budget (symAvail) does not underflow.
 *
 * ScheduleDlHarq() tracks the symbols left in the slot in a uint8_t, and must
 * debit it exactly once for the symbols of each beam. Debiting the same symbols
 * twice underflows the budget when a beam spans more than half of the remaining
 * symbols. The retransmissions of the next beam then pass every check and are
 * placed past the end of the slot, and the span returned by ScheduleDlHarq() is
 * larger than the budget it was given, which trips the caller
 * NS_ASSERT_MSG(dlSymAvail >= usedHarq, ...).
 *
 * This test places one retransmission per beam in two distinct beams, each
 * spanning more than half of the slot, and checks that no DCI is scheduled past
 * the end of the slot (and, in a build with assertions enabled, that the caller
 * assertion is not tripped). It is exercised on both the TDMA and OFDMA
 * round-robin schedulers, since both share NrMacSchedulerHarqRr.
 */
class NrTestMacSchedulerHarqRrSymbolBudget : public TestCase
{
  public:
    /**
     * @brief Constructor
     * @param isTdma whether to test the TDMA (true) or OFDMA (false) scheduler
     */
    NrTestMacSchedulerHarqRrSymbolBudget(bool isTdma)
        : TestCase(std::string("HARQ DL symbol budget does not underflow across beams (") +
                   (isTdma ? "TDMA)" : "OFDMA)")),
          m_isTdma(isTdma)
    {
    }

  protected:
    void DoRun() override;

  private:
    /**
     * @brief Verify that no scheduled HARQ DCI extends past the end of the slot.
     * @param params the scheduled slot allocation reported by the scheduler
     */
    void CheckSchedule(const NrMacSchedSapUser::SchedConfigIndParameters& params);

    bool m_isTdma;                                 //!< Test TDMA (true) or OFDMA (false)
    static constexpr uint8_t m_symbolsPerSlot{14}; //!< Symbols available in the slot
};

void
NrTestMacSchedulerHarqRrSymbolBudget::CheckSchedule(
    const NrMacSchedSapUser::SchedConfigIndParameters& params)
{
    uint16_t lastUsedSymbol = 0;
    for (const auto& varTtiAllocInfo : params.m_slotAllocInfo.m_varTtiAllocInfo)
    {
        // Only data DCIs carry HARQ retransmissions
        if (varTtiAllocInfo.m_dci->m_type != DciInfoElementTdma::DATA)
        {
            continue;
        }
        lastUsedSymbol =
            std::max<uint16_t>(lastUsedSymbol,
                               varTtiAllocInfo.m_dci->m_symStart + varTtiAllocInfo.m_dci->m_numSym);
    }
    NS_TEST_ASSERT_MSG_LT_OR_EQ(
        +lastUsedSymbol,
        +m_symbolsPerSlot,
        (m_isTdma ? "TDMA" : "OFDMA")
            << ": HARQ retransmissions were placed past the end of the slot "
               "(symAvail underflowed and over-allocated symbols)");
}

void
NrTestMacSchedulerHarqRrSymbolBudget::DoRun()
{
    // This test targets the non-consolidating path. Set the attribute explicitly
    // so the test does not depend on the default value (and is robust to other
    // test cases changing it via Config::SetDefault).
    Config::SetDefault("ns3::NrMacSchedulerHarqRr::ConsolidateHarqRetx", BooleanValue(false));

    auto cellConfig = NrMacCschedSapProvider::CschedCellConfigReqParameters();
    cellConfig.m_dlBandwidth = 10; // 10 RBGs
    cellConfig.m_ulBandwidth = 10;

    // One UE per beam, in two distinct beams. Each retransmission spans 8 of the
    // 14 available symbols: more than half, so a single beam leaves 6 symbols,
    // and debiting its 8 symbols a second time would wrap the budget below zero,
    // letting the second beam over-allocate.
    const uint8_t numSym = 8;
    const std::vector<uint16_t> rntis = {0, 1};

    std::vector<NrMacCschedSapProvider::CschedUeConfigReqParameters> ueConfig;
    for (auto rnti : rntis)
    {
        NrMacCschedSapProvider::CschedUeConfigReqParameters config{};
        config.m_rnti = rnti;
        config.m_transmissionMode = 0;
        config.m_beamId = BeamId(rnti, 0); // distinct beam per UE
        ueConfig.push_back(config);
    }

    auto* schedSapUser = new TestSchedSapUserHarq([this](auto params) { CheckSchedule(params); },
                                                  []() { return uint32_t{m_symbolsPerSlot}; });
    auto* cschedSapUser = new TestCschedSapUserHarq();

    Ptr<NrMacSchedulerNs3> sched;
    if (m_isTdma)
    {
        sched = CreateObject<NrMacSchedulerTdmaRR>();
    }
    else
    {
        sched = CreateObject<NrMacSchedulerOfdmaRR>();
    }
    sched->InstallDlAmc(CreateObject<NrAmc>());
    sched->InstallUlAmc(CreateObject<NrAmc>());
    sched->SetMacSchedSapUser(schedSapUser);
    sched->SetMacCschedSapUser(cschedSapUser);
    sched->DoCschedCellConfigReq(cellConfig);
    for (const auto& ueConf : ueConfig)
    {
        sched->DoCschedUeConfigReq(ueConf);
    }
    sched->SetDlCtrlSyms(0);

    // Build a NACKed HARQ process per UE so ScheduleDlHarq retransmits them.
    NrMacSchedSapProvider::SchedDlTriggerReqParameters paramsDlTrigger;
    paramsDlTrigger.m_snfSf = SfnSf(0, 0, 0, 0);
    paramsDlTrigger.m_slotType = LteNrTddSlotType::DL;
    for (auto rnti : rntis)
    {
        auto& ueInfo = sched->m_ueMap.find(rnti)->second;
        auto& harqProcess = ueInfo->m_dlHarq.Find(rnti)->second;

        DciInfoElementTdma dci(rnti,
                               DciInfoElementTdma::DL,
                               0,
                               numSym,
                               10,
                               1,
                               {},
                               100,
                               0,
                               0,
                               DciInfoElementTdma::DATA,
                               0,
                               0);
        dci.m_harqProcess = static_cast<uint8_t>(rnti);
        dci.m_rbgBitmask = std::vector<bool>(10, true); // full-band allocation
        harqProcess.m_dciElement = std::make_shared<DciInfoElementTdma>(dci);
        harqProcess.m_active = true;
        harqProcess.m_status = HarqProcess::WAITING_FEEDBACK;

        DlHarqInfo harqInfo;
        harqInfo.m_harqStatus = DlHarqInfo::NACK;
        harqInfo.m_numRetx = 0;
        harqInfo.m_rnti = rnti;
        harqInfo.m_harqProcessId = static_cast<uint8_t>(rnti);
        harqInfo.m_bwpIndex = 0;
        paramsDlTrigger.m_dlHarqInfoList.push_back(harqInfo);
    }

    // An underflow of the budget either trips an assertion (when asserts are
    // enabled) or schedules a retransmission past the end of the slot, caught by
    // CheckSchedule above.
    sched->DoSchedDlTriggerReq(paramsDlTrigger);

    delete schedSapUser;
    delete cschedSapUser;
    Simulator::Destroy();
}

/**
 * @brief Verify OFDMA HARQ retransmissions in the same beam share symbols.
 *
 * In OFDMA, several UEs in the same beam are retransmitted on the same OFDM
 * symbols but disjoint RBGs (frequency multiplexing). ScheduleDlHarq() must
 * therefore debit the slot symbol budget by the beam's symbol span, not by the
 * sum of the per-UE spans. Debiting per retransmission drops every UE after the
 * first whenever the shared span exceeds half the slot, silently losing
 * retransmissions (and, with a following beam, underflows the budget). This test
 * schedules two UEs in one beam on disjoint RBGs, each
 * spanning more than half of the slot, and checks that both are scheduled within
 * the slot, with and without HARQ consolidation.
 */
class NrTestMacSchedulerHarqRrOfdmaSharing : public TestCase
{
  public:
    /**
     * @brief Constructor
     * @param consolidate whether ConsolidateHarqRetx (allocation reshaping) is enabled
     */
    NrTestMacSchedulerHarqRrOfdmaSharing(bool consolidate)
        : TestCase(std::string("OFDMA HARQ retransmissions share symbols within a beam (") +
                   (consolidate ? "consolidated)" : "not consolidated)")),
          m_consolidate(consolidate)
    {
    }

  protected:
    void DoRun() override;

  private:
    /**
     * @brief Check that both same-beam UEs were scheduled within the slot.
     * @param params the scheduled slot allocation reported by the scheduler
     */
    void CheckSchedule(const NrMacSchedSapUser::SchedConfigIndParameters& params);

    bool m_consolidate;                            //!< Reshape (consolidate) HARQ retx or not
    bool m_checked{false};                         //!< Set once the scheduler reports a slot
    static constexpr uint8_t m_symbolsPerSlot{14}; //!< Symbols available in the slot
    static constexpr uint8_t m_numSym{8};          //!< Symbols per retransmission (> half the slot)
    static constexpr uint32_t m_numUes{2};         //!< UEs sharing the single beam
};

void
NrTestMacSchedulerHarqRrOfdmaSharing::CheckSchedule(
    const NrMacSchedSapUser::SchedConfigIndParameters& params)
{
    m_checked = true;

    uint32_t numDataDcis = 0;
    uint16_t lastUsedSymbol = 0;
    for (const auto& varTtiAllocInfo : params.m_slotAllocInfo.m_varTtiAllocInfo)
    {
        if (varTtiAllocInfo.m_dci->m_type != DciInfoElementTdma::DATA)
        {
            continue;
        }
        ++numDataDcis;
        lastUsedSymbol =
            std::max<uint16_t>(lastUsedSymbol,
                               varTtiAllocInfo.m_dci->m_symStart + varTtiAllocInfo.m_dci->m_numSym);
    }

    // Both UEs must be retransmitted: since each spans more than half the slot,
    // the only way to fit both is to share the same symbols on disjoint RBGs.
    NS_TEST_ASSERT_MSG_EQ(numDataDcis,
                          m_numUes,
                          "Both same-beam OFDMA UEs should be retransmitted by sharing symbols, "
                          "but some were dropped");
    NS_TEST_ASSERT_MSG_LT_OR_EQ(+lastUsedSymbol,
                                +m_symbolsPerSlot,
                                "HARQ retransmissions were placed past the end of the slot");
}

void
NrTestMacSchedulerHarqRrOfdmaSharing::DoRun()
{
    Config::SetDefault("ns3::NrMacSchedulerHarqRr::ConsolidateHarqRetx",
                       BooleanValue(m_consolidate));

    auto cellConfig = NrMacCschedSapProvider::CschedCellConfigReqParameters();
    cellConfig.m_dlBandwidth = 10; // 10 RBGs
    cellConfig.m_ulBandwidth = 10;

    // Two UEs in the SAME beam, on disjoint RBG halves, each spanning 8 of the 14
    // available symbols. Stacking them in time would need 16 > 14 symbols, so the
    // scheduler can only fit both by sharing the symbols across the two RBG sets.
    const std::vector<uint16_t> rntis = {0, 1};

    std::vector<NrMacCschedSapProvider::CschedUeConfigReqParameters> ueConfig;
    for (auto rnti : rntis)
    {
        NrMacCschedSapProvider::CschedUeConfigReqParameters config{};
        config.m_rnti = rnti;
        config.m_transmissionMode = 0;
        config.m_beamId = BeamId(0, 0); // single shared beam
        ueConfig.push_back(config);
    }

    auto* schedSapUser = new TestSchedSapUserHarq([this](auto params) { CheckSchedule(params); },
                                                  []() { return uint32_t{m_symbolsPerSlot}; });
    auto* cschedSapUser = new TestCschedSapUserHarq();

    auto sched = CreateObject<NrMacSchedulerOfdmaRR>();
    sched->InstallDlAmc(CreateObject<NrAmc>());
    sched->InstallUlAmc(CreateObject<NrAmc>());
    sched->SetMacSchedSapUser(schedSapUser);
    sched->SetMacCschedSapUser(cschedSapUser);
    sched->DoCschedCellConfigReq(cellConfig);
    for (const auto& ueConf : ueConfig)
    {
        sched->DoCschedUeConfigReq(ueConf);
    }
    sched->SetDlCtrlSyms(0);

    NrMacSchedSapProvider::SchedDlTriggerReqParameters paramsDlTrigger;
    paramsDlTrigger.m_snfSf = SfnSf(0, 0, 0, 0);
    paramsDlTrigger.m_slotType = LteNrTddSlotType::DL;
    for (size_t i = 0; i < rntis.size(); ++i)
    {
        const auto rnti = rntis.at(i);
        auto& ueInfo = sched->m_ueMap.find(rnti)->second;
        auto& harqProcess = ueInfo->m_dlHarq.Find(rnti)->second;

        // Disjoint RBG halves: UE 0 -> RBGs [0,4], UE 1 -> RBGs [5,9].
        std::vector<bool> bitmask(10, false);
        for (size_t rbg = i * 5; rbg < (i + 1) * 5; ++rbg)
        {
            bitmask.at(rbg) = true;
        }

        DciInfoElementTdma dci(rnti,
                               DciInfoElementTdma::DL,
                               0,
                               m_numSym,
                               10,
                               1,
                               {},
                               100,
                               0,
                               0,
                               DciInfoElementTdma::DATA,
                               0,
                               0);
        dci.m_harqProcess = static_cast<uint8_t>(rnti);
        dci.m_rbgBitmask = bitmask;
        harqProcess.m_dciElement = std::make_shared<DciInfoElementTdma>(dci);
        harqProcess.m_active = true;
        harqProcess.m_status = HarqProcess::WAITING_FEEDBACK;

        DlHarqInfo harqInfo;
        harqInfo.m_harqStatus = DlHarqInfo::NACK;
        harqInfo.m_numRetx = 0;
        harqInfo.m_rnti = rnti;
        harqInfo.m_harqProcessId = static_cast<uint8_t>(rnti);
        harqInfo.m_bwpIndex = 0;
        paramsDlTrigger.m_dlHarqInfoList.push_back(harqInfo);
    }

    sched->DoSchedDlTriggerReq(paramsDlTrigger);

    NS_TEST_ASSERT_MSG_EQ(m_checked,
                          true,
                          "Scheduler did not report a slot allocation; checks did not run");

    delete schedSapUser;
    delete cschedSapUser;
    Simulator::Destroy();
}

constexpr uint16_t RNTI = 1;

constexpr std::array<uint8_t, 5> MAX_RETX_VALUES{0, 1, 3, 5, 10};

/**
 * @ingroup test
 * @brief Test that the scheduler bounds HARQ retransmissions by MaxHarqReTx.
 *
 * For one direction (DL or UL), the test NACKs a transport block until the
 * process is released, and checks the number of transmissions, the RV and NDI
 * of each DCI, and that a new process starts with one attempt, RV 0 and NDI 1.
 * It also checks that an ACK received before the limit releases the process.
 * With EnableHarqReTx=false, no retransmission is scheduled regardless of
 * MaxHarqReTx.
 */
class NrHarqRetxSchedulerCase : public TestCase
{
  public:
    /**
     * Constructor.
     *
     * @param maxHarqReTx Value of the MaxHarqReTx attribute.
     * @param downlink True to test DL, false to test UL.
     * @param enableHarqReTx Value of the EnableHarqReTx attribute.
     */
    NrHarqRetxSchedulerCase(uint8_t maxHarqReTx, bool downlink, bool enableHarqReTx = true)
        : TestCase(std::string(downlink ? "DL" : "UL") + " MaxHarqReTx=" +
                   std::to_string(maxHarqReTx) + (enableHarqReTx ? "" : " EnableHarqReTx=false")),
          m_maxHarqReTx(maxHarqReTx),
          m_downlink(downlink),
          m_enableHarqReTx(enableHarqReTx)
    {
    }

  private:
    void DoRun() override;

    Ptr<NrMacSchedulerNs3> CreateScheduler();

    DciInfoElementTdma MakeInitialDci(DciInfoElementTdma::DciFormat format) const;

    void CaptureSchedule(const NrMacSchedSapUser::SchedConfigIndParameters& params);

    void ActivateFreshDlProcess(Ptr<NrMacSchedulerNs3> sched);
    void ActivateFreshUlProcess(Ptr<NrMacSchedulerNs3> sched);
    void ActivateFreshProcess(Ptr<NrMacSchedulerNs3> sched);

    void TriggerDlFeedback(Ptr<NrMacSchedulerNs3> sched, bool ack);
    void TriggerUlFeedback(Ptr<NrMacSchedulerNs3> sched, bool ack);
    void TriggerFeedback(Ptr<NrMacSchedulerNs3> sched, bool ack);

    HarqProcess& GetProcess(Ptr<NrMacSchedulerNs3> sched) const;
    void RunDirection();

    uint8_t m_maxHarqReTx;
    bool m_downlink;
    bool m_enableHarqReTx;

    TestSchedSapUserHarq* m_schedSapUser{nullptr};
    TestCschedSapUserHarq* m_cschedSapUser{nullptr};

    std::vector<DciInfoElementTdma> m_scheduledDcis;
    uint8_t m_harqId{255};
    SfnSf m_sfn{0, 0, 0, 0};
};

void
NrHarqRetxSchedulerCase::CaptureSchedule(const NrMacSchedSapUser::SchedConfigIndParameters& params)
{
    for (const auto& varTtiAllocInfo : params.m_slotAllocInfo.m_varTtiAllocInfo)
    {
        if (varTtiAllocInfo.m_dci->m_type == DciInfoElementTdma::DATA)
        {
            m_scheduledDcis.push_back(*varTtiAllocInfo.m_dci);
        }
    }
}

Ptr<NrMacSchedulerNs3>
NrHarqRetxSchedulerCase::CreateScheduler()
{
    m_schedSapUser = new TestSchedSapUserHarq([this](auto params) { CaptureSchedule(params); });
    m_cschedSapUser = new TestCschedSapUserHarq();

    auto sched = CreateObject<NrMacSchedulerTdmaRR>();

    sched->InstallDlAmc(CreateObject<NrAmc>());
    sched->InstallUlAmc(CreateObject<NrAmc>());

    sched->SetMacSchedSapUser(m_schedSapUser);
    sched->SetMacCschedSapUser(m_cschedSapUser);

    sched->SetAttribute("MaxHarqReTx", UintegerValue(m_maxHarqReTx));
    sched->SetAttribute("EnableHarqReTx", BooleanValue(m_enableHarqReTx));

    NrMacCschedSapProvider::CschedCellConfigReqParameters cellConfig;
    cellConfig.m_dlBandwidth = 10;
    cellConfig.m_ulBandwidth = 10;
    sched->DoCschedCellConfigReq(cellConfig);

    NrMacCschedSapProvider::CschedUeConfigReqParameters ueConfig{};
    ueConfig.m_rnti = RNTI;
    ueConfig.m_transmissionMode = 0;
    ueConfig.m_beamId = BeamId(0, 0);
    sched->DoCschedUeConfigReq(ueConfig);

    sched->SetDlCtrlSyms(0);
    sched->SetUlCtrlSyms(1);

    return sched;
}

DciInfoElementTdma
NrHarqRetxSchedulerCase::MakeInitialDci(DciInfoElementTdma::DciFormat format) const
{
    DciInfoElementTdma dci(RNTI,
                           format,
                           0,
                           1,
                           10,
                           1,
                           {},
                           100,
                           1, // NDI: new transport block
                           0, // RV: first transmission
                           DciInfoElementTdma::DATA,
                           0,
                           0);

    dci.m_harqProcess = 0;

    dci.m_rbgBitmask = std::vector<bool>(10, false);
    dci.m_rbgBitmask.at(0) = true;

    return dci;
}

void
NrHarqRetxSchedulerCase::ActivateFreshDlProcess(Ptr<NrMacSchedulerNs3> sched)
{
    auto& ueInfo = sched->m_ueMap.find(RNTI)->second;

    HarqProcess fresh;
    fresh.m_dciElement =
        std::make_shared<DciInfoElementTdma>(MakeInitialDci(DciInfoElementTdma::DL));
    fresh.m_active = true;
    fresh.m_status = HarqProcess::WAITING_FEEDBACK;
    fresh.m_txAttempts = 1;

    uint8_t harqId = 255;
    const bool inserted = ueInfo->m_dlHarq.Insert(&harqId, fresh);

    NS_TEST_ASSERT_MSG_EQ(inserted, true, "Could not allocate a fresh DL HARQ process");
    NS_TEST_ASSERT_MSG_NE(harqId, 255, "DL HARQ allocator did not return a valid process ID");

    m_harqId = harqId;

    auto& process = ueInfo->m_dlHarq.Get(m_harqId);
    process.m_dciElement->m_harqProcess = m_harqId;

    NS_TEST_ASSERT_MSG_EQ(process.m_active, true, "Fresh DL HARQ process is not active");
    NS_TEST_ASSERT_MSG_EQ(+process.m_txAttempts,
                          1,
                          "Fresh DL HARQ process must start at attempt 1");
    NS_TEST_ASSERT_MSG_EQ(+process.m_dciElement->m_rv,
                          0,
                          "Fresh DL HARQ process must start with RV 0");
    NS_TEST_ASSERT_MSG_EQ(+process.m_dciElement->m_ndi,
                          1,
                          "Fresh DL HARQ process must start with NDI 1");
}

void
NrHarqRetxSchedulerCase::ActivateFreshUlProcess(Ptr<NrMacSchedulerNs3> sched)
{
    auto& ueInfo = sched->m_ueMap.find(RNTI)->second;

    HarqProcess fresh;
    fresh.m_dciElement =
        std::make_shared<DciInfoElementTdma>(MakeInitialDci(DciInfoElementTdma::UL));
    fresh.m_active = true;
    fresh.m_status = HarqProcess::WAITING_FEEDBACK;
    fresh.m_txAttempts = 1;

    uint8_t harqId = 255;
    const bool inserted = ueInfo->m_ulHarq.Insert(&harqId, fresh);

    NS_TEST_ASSERT_MSG_EQ(inserted, true, "Could not allocate a fresh UL HARQ process");
    NS_TEST_ASSERT_MSG_NE(harqId, 255, "UL HARQ allocator did not return a valid process ID");

    m_harqId = harqId;

    auto& process = ueInfo->m_ulHarq.Get(m_harqId);
    process.m_dciElement->m_harqProcess = m_harqId;

    NS_TEST_ASSERT_MSG_EQ(process.m_active, true, "Fresh UL HARQ process is not active");
    NS_TEST_ASSERT_MSG_EQ(+process.m_txAttempts,
                          1,
                          "Fresh UL HARQ process must start at attempt 1");
    NS_TEST_ASSERT_MSG_EQ(+process.m_dciElement->m_rv,
                          0,
                          "Fresh UL HARQ process must start with RV 0");
    NS_TEST_ASSERT_MSG_EQ(+process.m_dciElement->m_ndi,
                          1,
                          "Fresh UL HARQ process must start with NDI 1");
}

void
NrHarqRetxSchedulerCase::TriggerDlFeedback(Ptr<NrMacSchedulerNs3> sched, bool ack)
{
    m_scheduledDcis.clear();

    NrMacSchedSapProvider::SchedDlTriggerReqParameters trigger;
    trigger.m_snfSf = m_sfn;
    trigger.m_slotType = LteNrTddSlotType::DL;
    trigger.m_dlHarqInfoList = {};

    DlHarqInfo feedback;
    feedback.m_harqStatus = ack ? DlHarqInfo::ACK : DlHarqInfo::NACK;
    feedback.m_numRetx = 0;
    feedback.m_rnti = RNTI;
    feedback.m_harqProcessId = m_harqId;
    feedback.m_bwpIndex = 0;

    trigger.m_dlHarqInfoList.push_back(feedback);

    sched->DoSchedDlTriggerReq(trigger);
    m_sfn.Add(1);
}

void
NrHarqRetxSchedulerCase::TriggerUlFeedback(Ptr<NrMacSchedulerNs3> sched, bool ack)
{
    m_scheduledDcis.clear();

    NrMacSchedSapProvider::SchedUlTriggerReqParameters trigger;
    trigger.m_snfSf = m_sfn;
    trigger.m_slotType = LteNrTddSlotType::UL;
    trigger.m_ulHarqInfoList = {};

    UlHarqInfo feedback;
    feedback.m_receptionStatus = ack ? UlHarqInfo::Ok : UlHarqInfo::NotOk;
    feedback.m_rnti = RNTI;
    feedback.m_harqProcessId = m_harqId;
    feedback.m_bwpIndex = 0;

    trigger.m_ulHarqInfoList.push_back(feedback);

    sched->DoSchedUlTriggerReq(trigger);
    m_sfn.Add(1);
}

void
NrHarqRetxSchedulerCase::ActivateFreshProcess(Ptr<NrMacSchedulerNs3> sched)
{
    if (m_downlink)
    {
        ActivateFreshDlProcess(sched);
    }
    else
    {
        ActivateFreshUlProcess(sched);
    }
}

void
NrHarqRetxSchedulerCase::TriggerFeedback(Ptr<NrMacSchedulerNs3> sched, bool ack)
{
    if (m_downlink)
    {
        TriggerDlFeedback(sched, ack);
    }
    else
    {
        TriggerUlFeedback(sched, ack);
    }
}

HarqProcess&
NrHarqRetxSchedulerCase::GetProcess(Ptr<NrMacSchedulerNs3> sched) const
{
    auto& ueInfo = sched->m_ueMap.find(RNTI)->second;
    if (m_downlink)
    {
        return ueInfo->m_dlHarq.Get(m_harqId);
    }
    return ueInfo->m_ulHarq.Get(m_harqId);
}

void
NrHarqRetxSchedulerCase::RunDirection()
{
    auto sched = CreateScheduler();
    const std::string direction = m_downlink ? "DL" : "UL";

    ActivateFreshProcess(sched);
    auto& process = GetProcess(sched);

    if (!m_enableHarqReTx)
    {
        TriggerFeedback(sched, false);

        NS_TEST_ASSERT_MSG_EQ(m_scheduledDcis.size(),
                              0,
                              direction + " EnableHarqReTx=false produced a retransmission");
        NS_TEST_ASSERT_MSG_EQ(process.m_active,
                              false,
                              direction + " HARQ process was not released");

        ActivateFreshProcess(sched);
        auto& reusedProcess = GetProcess(sched);

        NS_TEST_ASSERT_MSG_EQ(+reusedProcess.m_txAttempts,
                              1,
                              direction + " reused process did not reset attempts");
        NS_TEST_ASSERT_MSG_EQ(+reusedProcess.m_dciElement->m_rv,
                              0,
                              direction + " reused process did not restart at RV 0");
        NS_TEST_ASSERT_MSG_EQ(+reusedProcess.m_dciElement->m_ndi,
                              1,
                              direction + " reused process did not restart with NDI 1");

        delete m_schedSapUser;
        delete m_cschedSapUser;
        m_schedSapUser = nullptr;
        m_cschedSapUser = nullptr;
        return;
    }

    for (uint8_t retx = 1; retx <= m_maxHarqReTx; ++retx)
    {
        TriggerFeedback(sched, false);

        NS_TEST_ASSERT_MSG_EQ(m_scheduledDcis.size(),
                              1,
                              direction + " NACK should produce one retransmission DCI");

        const auto& dci = m_scheduledDcis.front();

        NS_TEST_ASSERT_MSG_EQ(+dci.m_rv,
                              +retx,
                              direction + " RV does not match transmission attempt");
        NS_TEST_ASSERT_MSG_EQ(+dci.m_ndi, 0, direction + " retransmission must use NDI 0");
        NS_TEST_ASSERT_MSG_EQ(+process.m_txAttempts,
                              +(retx + 1),
                              direction + " m_txAttempts not incremented correctly");
        NS_TEST_ASSERT_MSG_EQ(process.m_active,
                              true,
                              direction + " HARQ process released before limit");
    }

    TriggerFeedback(sched, false);

    NS_TEST_ASSERT_MSG_EQ(m_scheduledDcis.size(),
                          0,
                          direction + " retransmitted after MaxHarqReTx");
    NS_TEST_ASSERT_MSG_EQ(process.m_active,
                          false,
                          direction + " HARQ process not released at limit");

    ActivateFreshProcess(sched);
    auto& earlyProcess = GetProcess(sched);

    if (m_maxHarqReTx > 0)
    {
        TriggerFeedback(sched, false);

        NS_TEST_ASSERT_MSG_EQ(m_scheduledDcis.size(),
                              1,
                              direction + " first NACK did not retransmit");
        NS_TEST_ASSERT_MSG_EQ(+m_scheduledDcis.front().m_rv,
                              1,
                              direction + " first retransmission must use RV 1");
        NS_TEST_ASSERT_MSG_EQ(+m_scheduledDcis.front().m_ndi,
                              0,
                              direction + " first retransmission must use NDI 0");
    }

    TriggerFeedback(sched, true);

    NS_TEST_ASSERT_MSG_EQ(m_scheduledDcis.size(),
                          0,
                          direction + " ACK unexpectedly produced a DCI");
    NS_TEST_ASSERT_MSG_EQ(earlyProcess.m_active,
                          false,
                          direction + " ACK did not release HARQ process");

    ActivateFreshProcess(sched);
    auto& reusedProcess = GetProcess(sched);

    NS_TEST_ASSERT_MSG_EQ(reusedProcess.m_active,
                          true,
                          direction + " released HARQ process could not be reused");
    NS_TEST_ASSERT_MSG_EQ(+reusedProcess.m_txAttempts,
                          1,
                          direction + " reused process did not reset attempts");
    NS_TEST_ASSERT_MSG_EQ(+reusedProcess.m_dciElement->m_rv,
                          0,
                          direction + " reused process did not restart at RV 0");
    NS_TEST_ASSERT_MSG_EQ(+reusedProcess.m_dciElement->m_ndi,
                          1,
                          direction + " reused process did not restart with NDI 1");

    TriggerFeedback(sched, false);

    if (m_maxHarqReTx > 0)
    {
        NS_TEST_ASSERT_MSG_EQ(m_scheduledDcis.size(),
                              1,
                              direction + " reused process was not schedulable");
        NS_TEST_ASSERT_MSG_EQ(+m_scheduledDcis.front().m_rv,
                              1,
                              direction + " reused process did not advance to RV 1");
    }
    else
    {
        NS_TEST_ASSERT_MSG_EQ(m_scheduledDcis.size(),
                              0,
                              direction + " MaxHarqReTx=0 produced a retransmission");
        NS_TEST_ASSERT_MSG_EQ(reusedProcess.m_active,
                              false,
                              direction + " MaxHarqReTx=0 process was not released");
    }

    delete m_schedSapUser;
    delete m_cschedSapUser;
    m_schedSapUser = nullptr;
    m_cschedSapUser = nullptr;
}

void
NrHarqRetxSchedulerCase::DoRun()
{
    RunDirection();
    Simulator::Destroy();
}

/**
 * @brief Verify UL HARQ retransmissions share symbols when they can, and are reshaped
 * into the symbols left when they cannot.
 *
 * Two UEs retransmit 8-symbol UL grants. When the UEs share a beam and their RBGs
 * do not overlap, ScheduleUlHarq() must place both retransmissions on the same
 * symbols, frequency multiplexed as in their first transmission. Otherwise they
 * need 16 of the 13 UL data symbols, so the second one must be reshaped into the
 * 5 symbols left over the whole band, keeping its TBS. Two retransmissions of one UE never
 * share symbols, since a UE sends a single PUSCH at a time.
 */
class NrTestMacSchedulerHarqRrUlSharing : public TestCase
{
  public:
    /**
     * @brief Constructor
     * @param sameBeam whether both UEs are in the same beam
     * @param disjointRbgs whether the UEs' RBGs are disjoint
     * @param sameUe whether both retransmissions belong to a single UE
     */
    NrTestMacSchedulerHarqRrUlSharing(bool sameBeam, bool disjointRbgs, bool sameUe = false)
        : TestCase(std::string("UL HARQ retransmissions ") + (sameUe ? "of one UE " : "") + "in " +
                   (sameBeam ? "the same beam" : "different beams") + " on " +
                   (disjointRbgs ? "disjoint" : "overlapping") + " RBGs"),
          m_sameBeam(sameBeam),
          m_disjointRbgs(disjointRbgs),
          m_sameUe(sameUe)
    {
    }

  protected:
    void DoRun() override;

  private:
    /**
     * @brief Check the UL retransmissions placed in the slot.
     * @param params the scheduled slot allocation reported by the scheduler
     */
    void CheckSchedule(const NrMacSchedSapUser::SchedConfigIndParameters& params);

    bool m_sameBeam;                               //!< Whether both UEs share a beam
    bool m_disjointRbgs;                           //!< Whether the UEs' RBGs are disjoint
    bool m_sameUe;                                 //!< Whether a single UE retransmits twice
    bool m_checked{false};                         //!< Set once the scheduler reports a slot
    static constexpr uint8_t m_symbolsPerSlot{14}; //!< Symbols available in the slot
    static constexpr uint8_t m_numSym{8};          //!< Symbols per retransmission
};

void
NrTestMacSchedulerHarqRrUlSharing::CheckSchedule(
    const NrMacSchedSapUser::SchedConfigIndParameters& params)
{
    m_checked = true;

    // Collect the UL data DCIs: here they can only be the retransmissions.
    // Also check that the slot is still sorted, since shared spans are merged in.
    std::vector<std::shared_ptr<DciInfoElementTdma>> retx;
    uint8_t prevSymStart = 0;
    for (const auto& varTtiAllocInfo : params.m_slotAllocInfo.m_varTtiAllocInfo)
    {
        NS_TEST_EXPECT_MSG_GT_OR_EQ(+varTtiAllocInfo.m_dci->m_symStart,
                                    +prevSymStart,
                                    "UL allocations must be sorted by starting symbol");
        prevSymStart = varTtiAllocInfo.m_dci->m_symStart;
        if (varTtiAllocInfo.m_dci->m_type == DciInfoElementTdma::DATA &&
            varTtiAllocInfo.m_dci->m_format == DciInfoElementTdma::UL)
        {
            retx.push_back(varTtiAllocInfo.m_dci);
        }
    }

    // Both retransmissions fit in the slot, shared or not
    NS_TEST_ASSERT_MSG_EQ(retx.size(), 2, "Unexpected number of UL retransmissions in the slot");
    for (const auto& dci : retx)
    {
        NS_TEST_EXPECT_MSG_EQ(dci->m_tbSize, 100, "Retransmission must keep its TBS");
        NS_TEST_EXPECT_MSG_LT_OR_EQ(dci->m_symStart + dci->m_numSym,
                                    m_symbolsPerSlot,
                                    "Retransmission placed past the end of the slot");
    }

    // Sharing needs a common beam, disjoint RBGs and two different UEs
    const bool shared = m_sameBeam && m_disjointRbgs && !m_sameUe;
    if (shared)
    {
        for (const auto& dci : retx)
        {
            NS_TEST_EXPECT_MSG_EQ(+dci->m_numSym,
                                  +m_numSym,
                                  "Retransmission must keep its symbols");
        }
        NS_TEST_EXPECT_MSG_EQ(+retx.at(0)->m_symStart,
                              +retx.at(1)->m_symStart,
                              "Compatible retransmissions must share their symbols");
        NS_TEST_EXPECT_MSG_EQ(params.m_slotAllocInfo.m_numSymAlloc,
                              m_numSym + 1, // the retransmission span plus UL CTRL
                              "Shared symbols must be counted once");
        return;
    }

    // One retransmission keeps its 8 symbols. The other one is reshaped into the 5 symbols
    // left, over enough RBGs to hold its resources, without overlapping the first in time.
    const auto& kept = retx.at(0)->m_numSym == m_numSym ? retx.at(0) : retx.at(1);
    const auto& reshaped = retx.at(0)->m_numSym == m_numSym ? retx.at(1) : retx.at(0);
    NS_TEST_EXPECT_MSG_EQ(+kept->m_numSym, +m_numSym, "One retransmission must keep its symbols");
    NS_TEST_EXPECT_MSG_LT(+reshaped->m_numSym,
                          +m_numSym,
                          "The other retransmission must be reshaped into fewer symbols");
    const auto originalRbgs = m_disjointRbgs ? 5 : 6;
    const auto reshapedRbgs =
        std::count(reshaped->m_rbgBitmask.begin(), reshaped->m_rbgBitmask.end(), true);
    NS_TEST_EXPECT_MSG_GT_OR_EQ(reshapedRbgs * reshaped->m_numSym,
                                originalRbgs * m_numSym,
                                "The reshaped retransmission must keep its resources");
    const bool overlapInTime = reshaped->m_symStart < kept->m_symStart + kept->m_numSym &&
                               kept->m_symStart < reshaped->m_symStart + reshaped->m_numSym;
    NS_TEST_EXPECT_MSG_EQ(overlapInTime,
                          false,
                          "The reshaped retransmission must get symbols of its own");
}

void
NrTestMacSchedulerHarqRrUlSharing::DoRun()
{
    // No SRS, so that all UL data symbols are available to the retransmissions
    Config::SetDefault("ns3::NrMacSchedulerNs3::EnableSrsInUlSlots", BooleanValue(false));

    auto cellConfig = NrMacCschedSapProvider::CschedCellConfigReqParameters();
    cellConfig.m_dlBandwidth = 10; // 10 RBGs
    cellConfig.m_ulBandwidth = 10;

    const std::vector<uint16_t> rntis = {1, 2};

    // The SAP user hands the scheduled slot to CheckSchedule()
    auto* schedSapUser = new TestSchedSapUserHarq([this](auto params) { CheckSchedule(params); },
                                                  []() { return uint32_t{m_symbolsPerSlot}; });
    auto* cschedSapUser = new TestCschedSapUserHarq();

    // OFDMA, since only it frequency multiplexes UL grants
    auto sched = CreateObject<NrMacSchedulerOfdmaRR>();
    sched->InstallDlAmc(CreateObject<NrAmc>());
    sched->InstallUlAmc(CreateObject<NrAmc>());
    sched->SetMacSchedSapUser(schedSapUser);
    sched->SetMacCschedSapUser(cschedSapUser);
    sched->DoCschedCellConfigReq(cellConfig);
    for (size_t i = 0; i < rntis.size(); ++i)
    {
        NrMacCschedSapProvider::CschedUeConfigReqParameters config{};
        config.m_rnti = rntis.at(i);
        // Same beam for both UEs, or one beam each
        config.m_beamId = m_sameBeam ? BeamId(0, 0) : BeamId(static_cast<uint16_t>(i), 0);
        sched->DoCschedUeConfigReq(config);
    }
    // 1 UL CTRL symbol leaves 13 UL data symbols
    sched->SetUlCtrlSyms(1);

    // A UL slot, with a NACK for one HARQ process of each UE
    NrMacSchedSapProvider::SchedUlTriggerReqParameters paramsUlTrigger;
    paramsUlTrigger.m_snfSf = SfnSf(0, 0, 0, 0);
    paramsUlTrigger.m_slotType = LteNrTddSlotType::UL;
    for (size_t i = 0; i < rntis.size(); ++i)
    {
        const auto rnti = m_sameUe ? rntis.front() : rntis.at(i);
        const auto harqId = static_cast<uint8_t>(i);
        auto& ueInfo = sched->m_ueMap.find(rnti)->second;
        auto& harqProcess = ueInfo->m_ulHarq.Find(harqId)->second;

        // Disjoint: UE 1 -> RBGs [0,4], UE 2 -> RBGs [5,9]. Overlapping: both on [0,5].
        std::vector<bool> bitmask(10, false);
        const size_t first = m_disjointRbgs ? i * 5 : 0;
        const size_t last = m_disjointRbgs ? (i + 1) * 5 : 6;
        for (size_t rbg = first; rbg < last; ++rbg)
        {
            bitmask.at(rbg) = true;
        }

        // First transmission of the process: 8 symbols, MCS 10, TBS 100, new data
        DciInfoElementTdma dci(rnti,
                               DciInfoElementTdma::UL,
                               0,
                               m_numSym,
                               10,
                               1,
                               {},
                               100,
                               1,
                               0,
                               DciInfoElementTdma::DATA,
                               0,
                               0);
        // Store it in the HARQ process, as if it was sent and is waiting for feedback
        dci.m_harqProcess = harqId;
        dci.m_rbgBitmask = bitmask;
        harqProcess.m_dciElement = std::make_shared<DciInfoElementTdma>(dci);
        harqProcess.m_active = true;
        harqProcess.m_status = HarqProcess::WAITING_FEEDBACK;

        // The gNB failed to decode it, so it must be retransmitted
        UlHarqInfo harqInfo;
        harqInfo.m_receptionStatus = UlHarqInfo::NotOk;
        harqInfo.m_numRetx = 0;
        harqInfo.m_rnti = rnti;
        harqInfo.m_harqProcessId = harqId;
        paramsUlTrigger.m_ulHarqInfoList.push_back(harqInfo);
    }

    // Schedule the slot; CheckSchedule() runs from inside this call
    sched->DoSchedUlTriggerReq(paramsUlTrigger);

    NS_TEST_ASSERT_MSG_EQ(m_checked,
                          true,
                          "Scheduler did not report a slot allocation; checks did not run");

    delete schedSapUser;
    delete cschedSapUser;
    Simulator::Destroy();
    // Restore the defaults changed above for the following test cases
    Config::Reset();
}

/**
 * @brief Verify DL HARQ retransmissions that cannot keep their shape are reshaped into the
 * symbols left.
 *
 * Two UEs retransmit 8-symbol PDSCHs, with 13 DL data symbols in the slot. Either both UEs
 * share a beam and their RBGs overlap, or they are in different beams, so the second
 * retransmission cannot be placed on the symbols of the first one, and its 8 symbols do not
 * fit in the 5 left. Without ConsolidateHarqRetx, it must be reshaped into those 5 symbols
 * over the whole band, keeping its TBS, instead of being deferred to a later slot.
 */
class NrTestMacSchedulerHarqRrDlReshape : public TestCase
{
  public:
    /**
     * @brief Constructor
     * @param sameBeam whether both UEs are in the same beam, with overlapping RBGs
     */
    NrTestMacSchedulerHarqRrDlReshape(bool sameBeam)
        : TestCase(std::string("DL HARQ retransmission reshaped into the symbols left, ") +
                   (sameBeam ? "same beam on overlapping RBGs" : "different beams")),
          m_sameBeam(sameBeam)
    {
    }

  protected:
    void DoRun() override;

  private:
    /**
     * @brief Check the DL retransmissions placed in the slot.
     * @param params the scheduled slot allocation reported by the scheduler
     */
    void CheckSchedule(const NrMacSchedSapUser::SchedConfigIndParameters& params);

    bool m_sameBeam;                       //!< Whether both UEs share a beam
    bool m_checked{false};                 //!< Set once the scheduler reports a slot
    static constexpr uint8_t m_numSym{8};  //!< Symbols per retransmission
    static constexpr uint8_t m_numRbg{10}; //!< RBGs of the bandwidth
};

void
NrTestMacSchedulerHarqRrDlReshape::CheckSchedule(
    const NrMacSchedSapUser::SchedConfigIndParameters& params)
{
    m_checked = true;

    std::vector<std::shared_ptr<DciInfoElementTdma>> retx;
    for (const auto& varTtiAllocInfo : params.m_slotAllocInfo.m_varTtiAllocInfo)
    {
        const auto& dci = varTtiAllocInfo.m_dci;
        if (dci->m_type == DciInfoElementTdma::DATA && dci->m_format == DciInfoElementTdma::DL)
        {
            retx.push_back(dci);
        }
    }

    NS_TEST_ASSERT_MSG_EQ(retx.size(), 2, "Both DL retransmissions must be scheduled");
    for (const auto& dci : retx)
    {
        NS_TEST_EXPECT_MSG_EQ(+dci->m_rv, 1, "Both must be retransmissions");
        NS_TEST_EXPECT_MSG_EQ(dci->m_tbSize, 100, "Retransmission must keep its TBS");
        NS_TEST_EXPECT_MSG_LT_OR_EQ(dci->m_symStart + dci->m_numSym,
                                    14,
                                    "Retransmission placed past the end of the slot");
    }

    // One retransmission keeps its 8 symbols, the other one is reshaped into the 5 left
    const auto& kept = retx.at(0)->m_numSym == m_numSym ? retx.at(0) : retx.at(1);
    const auto& reshaped = retx.at(0)->m_numSym == m_numSym ? retx.at(1) : retx.at(0);
    NS_TEST_EXPECT_MSG_EQ(+kept->m_numSym, +m_numSym, "One retransmission must keep its symbols");
    NS_TEST_EXPECT_MSG_LT(+reshaped->m_numSym,
                          +m_numSym,
                          "The other retransmission must be reshaped into fewer symbols");
    const auto reshapedRbgs =
        std::count(reshaped->m_rbgBitmask.begin(), reshaped->m_rbgBitmask.end(), true);
    NS_TEST_EXPECT_MSG_GT_OR_EQ(reshapedRbgs * reshaped->m_numSym,
                                6 * m_numSym,
                                "The reshaped retransmission must keep its resources");
    const bool overlapInTime = reshaped->m_symStart < kept->m_symStart + kept->m_numSym &&
                               kept->m_symStart < reshaped->m_symStart + reshaped->m_numSym;
    NS_TEST_EXPECT_MSG_EQ(overlapInTime,
                          false,
                          "The reshaped retransmission must get symbols of its own");
}

void
NrTestMacSchedulerHarqRrDlReshape::DoRun()
{
    auto cellConfig = NrMacCschedSapProvider::CschedCellConfigReqParameters();
    cellConfig.m_dlBandwidth = m_numRbg;
    cellConfig.m_ulBandwidth = m_numRbg;

    // The SAP user hands the scheduled slot to CheckSchedule()
    auto* schedSapUser = new TestSchedSapUserHarq([this](auto params) { CheckSchedule(params); },
                                                  []() { return uint32_t{14}; });
    auto* cschedSapUser = new TestCschedSapUserHarq();

    auto sched = CreateObject<NrMacSchedulerOfdmaRR>();
    sched->InstallDlAmc(CreateObject<NrAmc>());
    sched->InstallUlAmc(CreateObject<NrAmc>());
    sched->SetMacSchedSapUser(schedSapUser);
    sched->SetMacCschedSapUser(cschedSapUser);
    sched->DoCschedCellConfigReq(cellConfig);
    for (uint16_t rnti : {1, 2})
    {
        NrMacCschedSapProvider::CschedUeConfigReqParameters config{};
        config.m_rnti = rnti;
        // Same beam for both UEs, or one beam each
        config.m_beamId = m_sameBeam ? BeamId(0, 0) : BeamId(rnti - 1, 0);
        sched->DoCschedUeConfigReq(config);
    }
    // 1 DL CTRL symbol leaves 13 DL data symbols
    sched->SetDlCtrlSyms(1);

    // Both UEs retransmit an 8-symbol PDSCH on RBGs [0,5].
    // Store the first transmissions in their HARQ process 0, waiting for feedback.
    NrMacSchedSapProvider::SchedDlTriggerReqParameters params;
    params.m_snfSf = SfnSf(0, 0, 0, 0);
    params.m_slotType = LteNrTddSlotType::DL;
    for (uint16_t rnti : {1, 2})
    {
        auto& harqProcess = sched->m_ueMap.find(rnti)->second->m_dlHarq.Find(0)->second;
        std::vector<bool> bitmask(m_numRbg, false);
        std::fill_n(bitmask.begin(), 6, true);
        DciInfoElementTdma dci(rnti,
                               DciInfoElementTdma::DL,
                               1,
                               m_numSym,
                               10,
                               1,
                               {},
                               100,
                               1,
                               0,
                               DciInfoElementTdma::DATA,
                               0,
                               0);
        dci.m_harqProcess = 0;
        dci.m_rbgBitmask = bitmask;
        harqProcess.m_dciElement = std::make_shared<DciInfoElementTdma>(dci);
        harqProcess.m_active = true;
        harqProcess.m_status = HarqProcess::WAITING_FEEDBACK;

        // The UE failed to decode it, so it must be retransmitted
        DlHarqInfo harqInfo;
        harqInfo.m_harqStatus = DlHarqInfo::NACK;
        harqInfo.m_numRetx = 0;
        harqInfo.m_rnti = rnti;
        harqInfo.m_harqProcessId = 0;
        harqInfo.m_bwpIndex = 0;
        params.m_dlHarqInfoList.push_back(harqInfo);
    }

    // Schedule the slot; CheckSchedule() runs from inside this call
    sched->DoSchedDlTriggerReq(params);

    NS_TEST_ASSERT_MSG_EQ(m_checked,
                          true,
                          "Scheduler did not report a slot allocation; checks did not run");

    delete schedSapUser;
    delete cschedSapUser;
    Simulator::Destroy();
}

/**
 * @brief Verify UL data uses the RBGs left free by UL HARQ retransmissions with OFDMA.
 *
 * UE 1 retransmits an 8-symbol grant on RBGs [0,4], while UE 2 has new UL data. With an
 * OFDMA scheduler and both UEs in a beam, UE 2 must be scheduled on the remaining RBGs
 * over the symbols of the retransmission, instead of on separate symbols. With a TDMA
 * scheduler, or UEs in different beams, UE 2 must get symbols of its own.
 */
class NrTestMacSchedulerHarqRrUlFill : public TestCase
{
  public:
    /**
     * @brief Constructor
     * @param ofdma whether the scheduler is OFDMA (true) or TDMA (false)
     * @param sameBeam whether both UEs are in the same beam
     */
    NrTestMacSchedulerHarqRrUlFill(bool ofdma, bool sameBeam)
        : TestCase(std::string(ofdma ? "OFDMA" : "TDMA") +
                   " UL data fills the RBGs left by a retransmission in " +
                   (sameBeam ? "the same beam" : "a different beam")),
          m_ofdma(ofdma),
          m_sameBeam(sameBeam)
    {
    }

  protected:
    void DoRun() override;

  private:
    /**
     * @brief Check the UL allocations placed in the slot.
     * @param params the scheduled slot allocation reported by the scheduler
     */
    void CheckSchedule(const NrMacSchedSapUser::SchedConfigIndParameters& params);

    bool m_ofdma;                                  //!< Whether the scheduler is OFDMA
    bool m_sameBeam;                               //!< Whether both UEs share a beam
    bool m_checked{false};                         //!< Set once the scheduler reports a slot
    static constexpr uint8_t m_symbolsPerSlot{14}; //!< Symbols available in the slot
    static constexpr uint8_t m_numSym{8};          //!< Symbols of the retransmission
    static constexpr uint16_t m_retxRnti{1};       //!< UE retransmitting
    static constexpr uint16_t m_newRnti{2};        //!< UE with new data
};

void
NrTestMacSchedulerHarqRrUlFill::CheckSchedule(
    const NrMacSchedSapUser::SchedConfigIndParameters& params)
{
    m_checked = true;

    // Find the UL data DCI of each UE
    std::shared_ptr<DciInfoElementTdma> retx;
    std::shared_ptr<DciInfoElementTdma> newData;
    for (const auto& varTtiAllocInfo : params.m_slotAllocInfo.m_varTtiAllocInfo)
    {
        const auto& dci = varTtiAllocInfo.m_dci;
        if (dci->m_type != DciInfoElementTdma::DATA || dci->m_format != DciInfoElementTdma::UL)
        {
            continue;
        }
        if (dci->m_rnti == m_retxRnti)
        {
            retx = dci;
        }
        else if (dci->m_rnti == m_newRnti)
        {
            newData = dci;
        }
    }

    // Both UEs must be served in this slot, whatever the beams
    NS_TEST_EXPECT_MSG_NE(retx, nullptr, "The retransmission must be scheduled");
    NS_TEST_EXPECT_MSG_NE(newData, nullptr, "The new data must be scheduled");
    if (!retx || !newData)
    {
        return;
    }
    NS_TEST_EXPECT_MSG_EQ(newData->m_ndi, 1, "UE 2 must get a new transmission");

    // Do the two grants overlap in frequency, or in time?
    bool overlap = false;
    for (std::size_t rbg = 0; rbg < retx->m_rbgBitmask.size(); ++rbg)
    {
        overlap = overlap || (retx->m_rbgBitmask[rbg] && newData->m_rbgBitmask[rbg]);
    }
    const bool sharesSymbols = newData->m_symStart < retx->m_symStart + retx->m_numSym &&
                               retx->m_symStart < newData->m_symStart + newData->m_numSym;
    if (m_ofdma && m_sameBeam)
    {
        // Same beam: UE 2 takes the free RBGs, over exactly the retransmission's symbols
        NS_TEST_EXPECT_MSG_EQ(+newData->m_symStart,
                              +retx->m_symStart,
                              "New data must use the symbols of the retransmission");
        NS_TEST_EXPECT_MSG_EQ(+newData->m_numSym,
                              +retx->m_numSym,
                              "New data must span the symbols of the retransmission");
        NS_TEST_EXPECT_MSG_EQ(overlap, false, "New data must not use the retransmission's RBGs");
        NS_TEST_EXPECT_MSG_EQ(params.m_slotAllocInfo.m_numSymAlloc,
                              m_numSym + 1, // the retransmission span plus UL CTRL
                              "Shared symbols must be counted once");
    }
    else
    {
        // TDMA gives the symbols to a single UE, and different beams cannot be served at
        // once, so UE 2 needs its own symbols
        NS_TEST_EXPECT_MSG_EQ(sharesSymbols,
                              false,
                              "UE 2 must not share the symbols of the retransmission");
    }
}

void
NrTestMacSchedulerHarqRrUlFill::DoRun()
{
    // No SRS, so that the slot only holds the grants under test
    Config::SetDefault("ns3::NrMacSchedulerNs3::EnableSrsInUlSlots", BooleanValue(false));
    // With 1-RB RBGs, a low MCS cannot fit a minimum UL transport block in the free RBGs
    Config::SetDefault("ns3::NrMacSchedulerNs3::StartingMcsUl", UintegerValue(20));

    auto cellConfig = NrMacCschedSapProvider::CschedCellConfigReqParameters();
    cellConfig.m_dlBandwidth = 10; // 10 RBGs
    cellConfig.m_ulBandwidth = 10;

    // The SAP user hands the scheduled slot to CheckSchedule()
    auto* schedSapUser = new TestSchedSapUserHarq([this](auto params) { CheckSchedule(params); },
                                                  []() { return uint32_t{m_symbolsPerSlot}; });
    auto* cschedSapUser = new TestCschedSapUserHarq();

    Ptr<NrMacSchedulerNs3> sched;
    if (m_ofdma)
    {
        sched = CreateObject<NrMacSchedulerOfdmaRR>();
    }
    else
    {
        sched = CreateObject<NrMacSchedulerTdmaRR>();
    }
    sched->InstallDlAmc(CreateObject<NrAmc>());
    sched->InstallUlAmc(CreateObject<NrAmc>());
    sched->SetMacSchedSapUser(schedSapUser);
    sched->SetMacCschedSapUser(cschedSapUser);
    sched->DoCschedCellConfigReq(cellConfig);
    for (auto rnti : {m_retxRnti, m_newRnti})
    {
        NrMacCschedSapProvider::CschedUeConfigReqParameters config{};
        config.m_rnti = rnti;
        // UE 1 is on beam 0; UE 2 joins it or gets beam 1
        config.m_beamId = (m_sameBeam || rnti == m_retxRnti) ? BeamId(0, 0) : BeamId(1, 0);
        sched->DoCschedUeConfigReq(config);
    }
    // 1 UL CTRL symbol leaves 13 UL data symbols
    sched->SetUlCtrlSyms(1);

    // UE 2 reports a buffer to be served with new data.
    // First its UL logical channel, in LCG 2.
    NrMacCschedSapProvider::CschedLcConfigReqParameters lcParams;
    lcParams.m_rnti = m_newRnti;
    lcParams.m_reconfigureFlag = false;
    nr::LogicalChannelConfigListElement_s lc;
    lc.m_logicalChannelIdentity = 1;
    lc.m_logicalChannelGroup = 2;
    lc.m_direction = nr::LogicalChannelConfigListElement_s::DIR_UL;
    lc.m_qosBearerType = nr::LogicalChannelConfigListElement_s::QBT_NON_GBR;
    lc.m_fiveQi = 9;
    lcParams.m_logicalChannelConfigList.emplace_back(lc);
    sched->DoCschedLcConfigReq(lcParams);

    // Then a BSR of 30 bytes in that LCG
    NrMacSchedSapProvider::SchedUlMacCtrlInfoReqParameters bsrParams;
    MacCeElement bsr;
    bsr.m_rnti = m_newRnti;
    bsr.m_macCeType = MacCeElement::BSR;
    bsr.m_macCeValue.m_bufferStatus = {0, 0, NrMacShortBsrCe::FromBytesToLevel(30), 0};
    bsrParams.m_macCeList.push_back(bsr);
    sched->DoSchedUlMacCtrlInfoReq(bsrParams);

    // UE 1 retransmits an 8-symbol grant on RBGs [0,4].
    // Store the first transmission in its HARQ process 0, waiting for feedback.
    auto& harqProcess = sched->m_ueMap.find(m_retxRnti)->second->m_ulHarq.Find(0)->second;
    std::vector<bool> bitmask(10, false);
    std::fill_n(bitmask.begin(), 5, true);
    DciInfoElementTdma dci(m_retxRnti,
                           DciInfoElementTdma::UL,
                           0,
                           m_numSym,
                           10,
                           1,
                           {},
                           100,
                           1,
                           0,
                           DciInfoElementTdma::DATA,
                           0,
                           0);
    dci.m_harqProcess = 0;
    dci.m_rbgBitmask = bitmask;
    harqProcess.m_dciElement = std::make_shared<DciInfoElementTdma>(dci);
    harqProcess.m_active = true;
    harqProcess.m_status = HarqProcess::WAITING_FEEDBACK;

    // The gNB failed to decode it, so it must be retransmitted
    UlHarqInfo harqInfo;
    harqInfo.m_receptionStatus = UlHarqInfo::NotOk;
    harqInfo.m_numRetx = 0;
    harqInfo.m_rnti = m_retxRnti;
    harqInfo.m_harqProcessId = 0;

    NrMacSchedSapProvider::SchedUlTriggerReqParameters paramsUlTrigger;
    paramsUlTrigger.m_snfSf = SfnSf(0, 0, 0, 0);
    paramsUlTrigger.m_slotType = LteNrTddSlotType::UL;
    paramsUlTrigger.m_ulHarqInfoList.push_back(harqInfo);
    // Schedule the slot; CheckSchedule() runs from inside this call
    sched->DoSchedUlTriggerReq(paramsUlTrigger);

    NS_TEST_ASSERT_MSG_EQ(m_checked,
                          true,
                          "Scheduler did not report a slot allocation; checks did not run");

    delete schedSapUser;
    delete cschedSapUser;
    Simulator::Destroy();
    // Restore the defaults changed above for the following test cases
    Config::Reset();
}

/**
 * @brief Verify DL data uses the RBGs left free by DL HARQ retransmissions with OFDMA.
 *
 * UE 1 retransmits an 8-symbol PDSCH on RBGs [0,4], while both UEs have new DL data.
 * With an OFDMA scheduler and both UEs in a beam, UE 2 must be scheduled on the
 * remaining RBGs over the symbols of the retransmission, instead of on separate symbols.
 * With a TDMA scheduler, or UEs in different beams, UE 2 must get symbols of its own. UE 1 must
 * never get new data, as a UE cannot receive two PDSCHs that overlap in time (TS 38.214,
 * clause 5.1).
 */
class NrTestMacSchedulerHarqRrDlFill : public TestCase
{
  public:
    /**
     * @brief Constructor
     * @param ofdma whether the scheduler is OFDMA (true) or TDMA (false)
     * @param sameBeam whether both UEs are in the same beam
     */
    NrTestMacSchedulerHarqRrDlFill(bool ofdma, bool sameBeam)
        : TestCase(std::string(ofdma ? "OFDMA" : "TDMA") +
                   " DL data fills the RBGs left by a retransmission in " +
                   (sameBeam ? "the same beam" : "a different beam")),
          m_ofdma(ofdma),
          m_sameBeam(sameBeam)
    {
    }

  protected:
    void DoRun() override;

  private:
    /**
     * @brief Check the DL allocations placed in the slot.
     * @param params the scheduled slot allocation reported by the scheduler
     */
    void CheckSchedule(const NrMacSchedSapUser::SchedConfigIndParameters& params);

    bool m_ofdma;                                  //!< Whether the scheduler is OFDMA
    bool m_sameBeam;                               //!< Whether both UEs share a beam
    bool m_checked{false};                         //!< Set once the scheduler reports a slot
    static constexpr uint8_t m_symbolsPerSlot{14}; //!< Symbols available in the slot
    static constexpr uint8_t m_numSym{8};          //!< Symbols of the retransmission
    static constexpr uint16_t m_retxRnti{1};       //!< UE retransmitting
    static constexpr uint16_t m_newRnti{2};        //!< UE with new data only
};

void
NrTestMacSchedulerHarqRrDlFill::CheckSchedule(
    const NrMacSchedSapUser::SchedConfigIndParameters& params)
{
    m_checked = true;

    // Find the DL data DCIs of each UE
    std::vector<std::shared_ptr<DciInfoElementTdma>> retxUeDcis;
    std::shared_ptr<DciInfoElementTdma> newData;
    for (const auto& varTtiAllocInfo : params.m_slotAllocInfo.m_varTtiAllocInfo)
    {
        const auto& dci = varTtiAllocInfo.m_dci;
        if (dci->m_type != DciInfoElementTdma::DATA || dci->m_format != DciInfoElementTdma::DL)
        {
            continue;
        }
        if (dci->m_rnti == m_retxRnti)
        {
            retxUeDcis.push_back(dci);
        }
        else if (dci->m_rnti == m_newRnti)
        {
            newData = dci;
        }
    }

    // UE 1 only gets its retransmission, despite having new data as well
    NS_TEST_ASSERT_MSG_EQ(retxUeDcis.size(), 1, "UE 1 must only get its retransmission");
    const auto& retx = retxUeDcis.front();
    NS_TEST_EXPECT_MSG_EQ(+retx->m_rv, 1, "UE 1 must get a retransmission");

    // UE 2 must be served in this slot, whatever the beams
    NS_TEST_ASSERT_MSG_NE(newData, nullptr, "The new data must be scheduled");
    NS_TEST_EXPECT_MSG_EQ(+newData->m_rv, 0, "UE 2 must get a new transmission");

    // Do the two PDSCHs overlap in frequency, or in time?
    bool overlap = false;
    for (std::size_t rbg = 0; rbg < retx->m_rbgBitmask.size(); ++rbg)
    {
        overlap = overlap || (retx->m_rbgBitmask[rbg] && newData->m_rbgBitmask[rbg]);
    }
    const bool sharesSymbols = newData->m_symStart < retx->m_symStart + retx->m_numSym &&
                               retx->m_symStart < newData->m_symStart + newData->m_numSym;
    if (m_ofdma && m_sameBeam)
    {
        // Same beam: UE 2 takes the free RBGs, over exactly the retransmission's symbols
        NS_TEST_EXPECT_MSG_EQ(+newData->m_symStart,
                              +retx->m_symStart,
                              "New data must use the symbols of the retransmission");
        NS_TEST_EXPECT_MSG_EQ(+newData->m_numSym,
                              +retx->m_numSym,
                              "New data must span the symbols of the retransmission");
        NS_TEST_EXPECT_MSG_EQ(overlap, false, "New data must not use the retransmission's RBGs");
        NS_TEST_EXPECT_MSG_EQ(params.m_slotAllocInfo.m_numSymAlloc,
                              m_numSym + 1, // DL CTRL plus the retransmission span
                              "Shared symbols must be counted once");
    }
    else
    {
        // TDMA gives the symbols to a single UE, and different beams cannot be served at
        // once, so UE 2 needs its own symbols
        NS_TEST_EXPECT_MSG_EQ(sharesSymbols,
                              false,
                              "UE 2 must not share the symbols of the retransmission");
    }
}

void
NrTestMacSchedulerHarqRrDlFill::DoRun()
{
    // With 1-RB RBGs, a low MCS cannot fit a minimum DL transport block in the free RBGs
    Config::SetDefault("ns3::NrMacSchedulerNs3::StartingMcsDl", UintegerValue(20));

    auto cellConfig = NrMacCschedSapProvider::CschedCellConfigReqParameters();
    cellConfig.m_dlBandwidth = 10; // 10 RBGs
    cellConfig.m_ulBandwidth = 10;

    // The SAP user hands the scheduled slot to CheckSchedule()
    auto* schedSapUser = new TestSchedSapUserHarq([this](auto params) { CheckSchedule(params); },
                                                  []() { return uint32_t{m_symbolsPerSlot}; });
    auto* cschedSapUser = new TestCschedSapUserHarq();

    Ptr<NrMacSchedulerNs3> sched;
    if (m_ofdma)
    {
        sched = CreateObject<NrMacSchedulerOfdmaRR>();
    }
    else
    {
        sched = CreateObject<NrMacSchedulerTdmaRR>();
    }
    sched->InstallDlAmc(CreateObject<NrAmc>());
    sched->InstallUlAmc(CreateObject<NrAmc>());
    sched->SetMacSchedSapUser(schedSapUser);
    sched->SetMacCschedSapUser(cschedSapUser);
    sched->DoCschedCellConfigReq(cellConfig);
    for (auto rnti : {m_retxRnti, m_newRnti})
    {
        NrMacCschedSapProvider::CschedUeConfigReqParameters config{};
        config.m_rnti = rnti;
        // UE 1 is on beam 0; UE 2 joins it or gets beam 1
        config.m_beamId = (m_sameBeam || rnti == m_retxRnti) ? BeamId(0, 0) : BeamId(1, 0);
        sched->DoCschedUeConfigReq(config);
    }
    // 1 DL CTRL symbol leaves 13 DL data symbols
    sched->SetDlCtrlSyms(1);

    // Both UEs have a DL logical channel with data to be served
    for (auto rnti : {m_retxRnti, m_newRnti})
    {
        NrMacCschedSapProvider::CschedLcConfigReqParameters lcParams;
        lcParams.m_rnti = rnti;
        lcParams.m_reconfigureFlag = false;
        nr::LogicalChannelConfigListElement_s lc;
        lc.m_logicalChannelIdentity = 1;
        lc.m_logicalChannelGroup = 2;
        lc.m_direction = nr::LogicalChannelConfigListElement_s::DIR_DL;
        lc.m_qosBearerType = nr::LogicalChannelConfigListElement_s::QBT_NON_GBR;
        lc.m_fiveQi = 9;
        lcParams.m_logicalChannelConfigList.emplace_back(lc);
        sched->DoCschedLcConfigReq(lcParams);

        NrMacSchedSapProvider::SchedDlRlcBufferReqParameters rlcParams{};
        rlcParams.m_rnti = rnti;
        rlcParams.m_logicalChannelIdentity = 1;
        rlcParams.m_rlcTransmissionQueueSize = 100;
        sched->DoSchedDlRlcBufferReq(rlcParams);
    }

    // UE 1 retransmits an 8-symbol PDSCH on RBGs [0,4].
    // Store the first transmission in its HARQ process 0, waiting for feedback.
    auto& harqProcess = sched->m_ueMap.find(m_retxRnti)->second->m_dlHarq.Find(0)->second;
    std::vector<bool> bitmask(10, false);
    std::fill_n(bitmask.begin(), 5, true);
    DciInfoElementTdma dci(m_retxRnti,
                           DciInfoElementTdma::DL,
                           1,
                           m_numSym,
                           10,
                           1,
                           {},
                           100,
                           1,
                           0,
                           DciInfoElementTdma::DATA,
                           0,
                           0);
    dci.m_harqProcess = 0;
    dci.m_rbgBitmask = bitmask;
    harqProcess.m_dciElement = std::make_shared<DciInfoElementTdma>(dci);
    harqProcess.m_active = true;
    harqProcess.m_status = HarqProcess::WAITING_FEEDBACK;

    // The UE failed to decode it, so it must be retransmitted
    DlHarqInfo harqInfo;
    harqInfo.m_harqStatus = DlHarqInfo::NACK;
    harqInfo.m_numRetx = 0;
    harqInfo.m_rnti = m_retxRnti;
    harqInfo.m_harqProcessId = 0;
    harqInfo.m_bwpIndex = 0;

    NrMacSchedSapProvider::SchedDlTriggerReqParameters paramsDlTrigger;
    paramsDlTrigger.m_snfSf = SfnSf(0, 0, 0, 0);
    paramsDlTrigger.m_slotType = LteNrTddSlotType::DL;
    paramsDlTrigger.m_dlHarqInfoList.push_back(harqInfo);
    // Schedule the slot; CheckSchedule() runs from inside this call
    sched->DoSchedDlTriggerReq(paramsDlTrigger);

    NS_TEST_ASSERT_MSG_EQ(m_checked,
                          true,
                          "Scheduler did not report a slot allocation; checks did not run");

    delete schedSapUser;
    delete cschedSapUser;
    Simulator::Destroy();
    // Restore the defaults changed above for the following test cases
    Config::Reset();
}

/**
 * @brief Verify that filling the RBGs left by a HARQ retransmission never lowers the
 * resource utilization of the slot.
 *
 * With 10 RBGs, UE 1 retransmits on 2 symbols and RBGs [0,1], and the other UEs of its
 * beam have new data. A UE gets a single DCI per slot, so a UE scheduled on the 8 RBGs
 * left free over the retransmission symbols cannot use the symbols after them. The
 * scheduler must only move a UE there when the UEs left behind still fill the rest of
 * the slot, or when the UE's data fits there.
 */
class NrTestMacSchedulerHarqRrFillUtilization : public TestCase
{
  public:
    /**
     * @brief Constructor
     * @param isDl whether to test the DL (true) or the UL (false)
     * @param buffers bytes of new data of UEs 2, 3, ...
     * @param expectedSpanRntis UEs expected on the retransmission symbols (0 means any one UE)
     * @param leavesSpanEmpty whether the free RBGs of the retransmission symbols stay unused
     */
    NrTestMacSchedulerHarqRrFillUtilization(bool isDl,
                                            const std::vector<uint32_t>& buffers,
                                            const std::set<uint16_t>& expectedSpanRntis,
                                            bool leavesSpanEmpty)
        : TestCase(std::string(isDl ? "DL" : "UL") +
                   " utilization with a 2-symbol retx and new data of " + ToString(buffers) +
                   " bytes"),
          m_isDl(isDl),
          m_buffers(buffers),
          m_expectedSpanRntis(expectedSpanRntis),
          m_leavesSpanEmpty(leavesSpanEmpty)
    {
    }

  protected:
    void DoRun() override;

  private:
    /**
     * @brief Format the buffer sizes for the test name
     * @param buffers bytes of new data of each UE
     * @return the buffer sizes, separated by " and "
     */
    static std::string ToString(const std::vector<uint32_t>& buffers)
    {
        std::string out;
        for (auto buffer : buffers)
        {
            out += (out.empty() ? "" : " and ") + std::to_string(buffer);
        }
        return out;
    }

    /**
     * @brief Check the utilization of the slot and the UEs placed on the retx symbols.
     * @param params the scheduled slot allocation reported by the scheduler
     */
    void CheckSchedule(const NrMacSchedSapUser::SchedConfigIndParameters& params);

    bool m_isDl;                              //!< Whether the DL or the UL is tested
    std::vector<uint32_t> m_buffers;          //!< Bytes of new data of UEs 2, 3, ...
    std::set<uint16_t> m_expectedSpanRntis;   //!< UEs expected on the retx symbols
    bool m_leavesSpanEmpty;                   //!< Whether the free retx RBGs stay unused
    bool m_checked{false};                    //!< Set once the scheduler reports a slot
    static constexpr uint8_t m_numRbg{10};    //!< RBGs of the bandwidth
    static constexpr uint8_t m_retxNumSym{2}; //!< Symbols of the retransmission
    static constexpr uint8_t m_retxNumRbg{2}; //!< RBGs of the retransmission
    static constexpr uint16_t m_retxRnti{1};  //!< UE retransmitting
};

void
NrTestMacSchedulerHarqRrFillUtilization::CheckSchedule(
    const NrMacSchedSapUser::SchedConfigIndParameters& params)
{
    m_checked = true;
    const auto format = m_isDl ? DciInfoElementTdma::DL : DciInfoElementTdma::UL;

    std::shared_ptr<DciInfoElementTdma> retx;
    std::vector<std::shared_ptr<DciInfoElementTdma>> newData;
    uint32_t usedRbgSym = 0;
    for (const auto& varTtiAllocInfo : params.m_slotAllocInfo.m_varTtiAllocInfo)
    {
        const auto& dci = varTtiAllocInfo.m_dci;
        if (dci->m_type != DciInfoElementTdma::DATA || dci->m_format != format)
        {
            continue;
        }
        usedRbgSym +=
            std::count(dci->m_rbgBitmask.begin(), dci->m_rbgBitmask.end(), true) * dci->m_numSym;
        if (dci->m_rnti == m_retxRnti)
        {
            retx = dci;
        }
        else
        {
            newData.push_back(dci);
        }
    }
    NS_TEST_ASSERT_MSG_NE(retx, nullptr, "The retransmission must be scheduled");

    // UEs with new data on the symbols of the retransmission
    std::set<uint16_t> spanRntis;
    for (const auto& dci : newData)
    {
        if (dci->m_symStart < retx->m_symStart + retx->m_numSym &&
            retx->m_symStart < dci->m_symStart + dci->m_numSym)
        {
            NS_TEST_EXPECT_MSG_EQ(+dci->m_symStart,
                                  +retx->m_symStart,
                                  "New data must use the symbols of the retransmission");
            NS_TEST_EXPECT_MSG_EQ(+dci->m_numSym,
                                  +retx->m_numSym,
                                  "New data must span the symbols of the retransmission");
            spanRntis.insert(dci->m_rnti);
        }
    }
    if (m_expectedSpanRntis.contains(0))
    {
        NS_TEST_EXPECT_MSG_EQ(spanRntis.size(), 1, "One UE must use the retx symbols");
    }
    else
    {
        NS_TEST_EXPECT_MSG_EQ((spanRntis == m_expectedSpanRntis),
                              true,
                              "Unexpected UEs on the retx symbols");
    }

    // Every symbol for data, minus the free RBGs of the retx symbols if they stay unused
    const uint32_t dataSym = m_isDl ? 14 : 13; // the UL slot ends with 1 UL CTRL symbol
    const uint32_t unused = m_leavesSpanEmpty ? (m_numRbg - m_retxNumRbg) * m_retxNumSym : 0;
    NS_TEST_EXPECT_MSG_EQ(usedRbgSym,
                          dataSym * m_numRbg - unused,
                          "Unexpected number of RBGs times symbols in use");
}

void
NrTestMacSchedulerHarqRrFillUtilization::DoRun()
{
    // No SRS, so that the slot only holds the data under test
    Config::SetDefault("ns3::NrMacSchedulerNs3::EnableSrsInUlSlots", BooleanValue(false));
    // With 1-RB RBGs, a low MCS cannot fit a minimum transport block in a few RBGs
    Config::SetDefault("ns3::NrMacSchedulerNs3::StartingMcsDl", UintegerValue(20));
    Config::SetDefault("ns3::NrMacSchedulerNs3::StartingMcsUl", UintegerValue(20));

    auto cellConfig = NrMacCschedSapProvider::CschedCellConfigReqParameters();
    cellConfig.m_dlBandwidth = m_numRbg;
    cellConfig.m_ulBandwidth = m_numRbg;

    // The SAP user hands the scheduled slot to CheckSchedule()
    auto* schedSapUser = new TestSchedSapUserHarq([this](auto params) { CheckSchedule(params); },
                                                  []() { return uint32_t{14}; });
    auto* cschedSapUser = new TestCschedSapUserHarq();

    auto sched = CreateObject<NrMacSchedulerOfdmaRR>();
    sched->InstallDlAmc(CreateObject<NrAmc>());
    sched->InstallUlAmc(CreateObject<NrAmc>());
    sched->SetMacSchedSapUser(schedSapUser);
    sched->SetMacCschedSapUser(cschedSapUser);
    sched->DoCschedCellConfigReq(cellConfig);
    // All UEs share a beam
    for (std::size_t i = 0; i <= m_buffers.size(); ++i)
    {
        NrMacCschedSapProvider::CschedUeConfigReqParameters config{};
        config.m_rnti = static_cast<uint16_t>(m_retxRnti + i);
        config.m_beamId = BeamId(0, 0);
        sched->DoCschedUeConfigReq(config);
    }
    // Every symbol of a DL slot carries data; a UL slot ends with 1 UL CTRL symbol
    sched->SetDlCtrlSyms(0);
    sched->SetUlCtrlSyms(1);

    // UEs 2, 3, ... have new data
    for (std::size_t i = 0; i < m_buffers.size(); ++i)
    {
        const auto rnti = static_cast<uint16_t>(i + 2);
        NrMacCschedSapProvider::CschedLcConfigReqParameters lcParams;
        lcParams.m_rnti = rnti;
        lcParams.m_reconfigureFlag = false;
        nr::LogicalChannelConfigListElement_s lc;
        lc.m_logicalChannelIdentity = 1;
        lc.m_logicalChannelGroup = 2;
        lc.m_direction = m_isDl ? nr::LogicalChannelConfigListElement_s::DIR_DL
                                : nr::LogicalChannelConfigListElement_s::DIR_UL;
        lc.m_qosBearerType = nr::LogicalChannelConfigListElement_s::QBT_NON_GBR;
        lc.m_fiveQi = 9;
        lcParams.m_logicalChannelConfigList.emplace_back(lc);
        sched->DoCschedLcConfigReq(lcParams);

        if (m_isDl)
        {
            NrMacSchedSapProvider::SchedDlRlcBufferReqParameters rlcParams{};
            rlcParams.m_rnti = rnti;
            rlcParams.m_logicalChannelIdentity = 1;
            rlcParams.m_rlcTransmissionQueueSize = m_buffers[i];
            sched->DoSchedDlRlcBufferReq(rlcParams);
        }
        else
        {
            NrMacSchedSapProvider::SchedUlMacCtrlInfoReqParameters bsrParams;
            MacCeElement bsr;
            bsr.m_rnti = rnti;
            bsr.m_macCeType = MacCeElement::BSR;
            bsr.m_macCeValue.m_bufferStatus = {0,
                                               0,
                                               NrMacShortBsrCe::FromBytesToLevel(m_buffers[i]),
                                               0};
            bsrParams.m_macCeList.push_back(bsr);
            sched->DoSchedUlMacCtrlInfoReq(bsrParams);
        }
    }

    // UE 1 retransmits on 2 symbols and RBGs [0,1].
    // Store the first transmission in its HARQ process 0, waiting for feedback.
    auto& ueInfo = sched->m_ueMap.find(m_retxRnti)->second;
    auto& harqProcess =
        m_isDl ? ueInfo->m_dlHarq.Find(0)->second : ueInfo->m_ulHarq.Find(0)->second;
    std::vector<bool> bitmask(m_numRbg, false);
    std::fill_n(bitmask.begin(), m_retxNumRbg, true);
    DciInfoElementTdma dci(m_retxRnti,
                           m_isDl ? DciInfoElementTdma::DL : DciInfoElementTdma::UL,
                           0,
                           m_retxNumSym,
                           20,
                           1,
                           {},
                           100,
                           1,
                           0,
                           DciInfoElementTdma::DATA,
                           0,
                           0);
    dci.m_harqProcess = 0;
    dci.m_rbgBitmask = bitmask;
    harqProcess.m_dciElement = std::make_shared<DciInfoElementTdma>(dci);
    harqProcess.m_active = true;
    harqProcess.m_status = HarqProcess::WAITING_FEEDBACK;

    // It failed to be decoded, so it must be retransmitted.
    // Schedule the slot; CheckSchedule() runs from inside these calls.
    if (m_isDl)
    {
        DlHarqInfo harqInfo;
        harqInfo.m_harqStatus = DlHarqInfo::NACK;
        harqInfo.m_numRetx = 0;
        harqInfo.m_rnti = m_retxRnti;
        harqInfo.m_harqProcessId = 0;
        harqInfo.m_bwpIndex = 0;

        NrMacSchedSapProvider::SchedDlTriggerReqParameters params;
        params.m_snfSf = SfnSf(0, 0, 0, 0);
        params.m_slotType = LteNrTddSlotType::DL;
        params.m_dlHarqInfoList.push_back(harqInfo);
        sched->DoSchedDlTriggerReq(params);
    }
    else
    {
        UlHarqInfo harqInfo;
        harqInfo.m_receptionStatus = UlHarqInfo::NotOk;
        harqInfo.m_numRetx = 0;
        harqInfo.m_rnti = m_retxRnti;
        harqInfo.m_harqProcessId = 0;

        NrMacSchedSapProvider::SchedUlTriggerReqParameters params;
        params.m_snfSf = SfnSf(0, 0, 0, 0);
        params.m_slotType = LteNrTddSlotType::UL;
        params.m_ulHarqInfoList.push_back(harqInfo);
        sched->DoSchedUlTriggerReq(params);
    }

    NS_TEST_ASSERT_MSG_EQ(m_checked,
                          true,
                          "Scheduler did not report a slot allocation; checks did not run");

    delete schedSapUser;
    delete cschedSapUser;
    Simulator::Destroy();
    // Restore the defaults changed above for the following test cases
    Config::Reset();
}

/**
 * @brief Verify that UL data is spread over all the retransmission spans of a beam.
 *
 * With 12 RBGs and a beam, UE 1 retransmits on 4 symbols and RBGs [0,5], and UE 2 on
 * 6 symbols and RBGs [6,11], so the two retransmissions cannot share symbols. UEs 3 and 4
 * have 20 bytes each, and UE 5 has a configurable amount. The free RBGs of both spans must
 * be used, and each UE placed on a span must get a transport block that carries its data.
 * A UE whose data fits in no span keeps symbols of its own.
 */
class NrTestMacSchedulerHarqRrUlSpanSpread : public TestCase
{
  public:
    /**
     * @brief Constructor
     * @param lastBuffer bytes of new data of UE 5
     * @param lastInSpan whether UE 5 is expected on the symbols of a retransmission
     */
    NrTestMacSchedulerHarqRrUlSpanSpread(uint32_t lastBuffer, bool lastInSpan)
        : TestCase("UL data spread over retransmission spans of 4 and 6 symbols, with " +
                   std::to_string(lastBuffer) + " bytes for the last UE"),
          m_lastBuffer(lastBuffer),
          m_lastInSpan(lastInSpan)
    {
    }

  protected:
    void DoRun() override;

  private:
    /**
     * @brief Check where the retransmissions and the new data are placed.
     * @param params the scheduled slot allocation reported by the scheduler
     */
    void CheckSchedule(const NrMacSchedSapUser::SchedConfigIndParameters& params);

    uint32_t m_lastBuffer;                 //!< Bytes of new data of UE 5
    bool m_lastInSpan;                     //!< Whether UE 5 is expected on a retx span
    bool m_checked{false};                 //!< Set once the scheduler reports a slot
    std::map<uint16_t, uint32_t> m_buffer; //!< Bytes of new data, by RNTI
    static constexpr uint8_t m_numRbg{12}; //!< RBGs of the bandwidth
};

void
NrTestMacSchedulerHarqRrUlSpanSpread::CheckSchedule(
    const NrMacSchedSapUser::SchedConfigIndParameters& params)
{
    m_checked = true;

    std::map<uint16_t, std::shared_ptr<DciInfoElementTdma>> dcis;
    for (const auto& varTtiAllocInfo : params.m_slotAllocInfo.m_varTtiAllocInfo)
    {
        const auto& dci = varTtiAllocInfo.m_dci;
        if (dci->m_type == DciInfoElementTdma::DATA && dci->m_format == DciInfoElementTdma::UL)
        {
            dcis[dci->m_rnti] = dci;
        }
    }

    // Both retransmissions fit with their original shape
    NS_TEST_ASSERT_MSG_EQ(dcis.contains(1) && dcis.contains(2),
                          true,
                          "Both retransmissions must be scheduled");
    NS_TEST_EXPECT_MSG_EQ(+dcis.at(1)->m_numSym, 4, "UE 1 must keep its 4 symbols");
    NS_TEST_EXPECT_MSG_EQ(+dcis.at(2)->m_numSym, 6, "UE 2 must keep its 6 symbols");

    std::map<uint16_t, uint32_t> spanUes; // UEs with new data on the span, by retx RNTI
    for (const auto& [rnti, buffer] : m_buffer)
    {
        NS_TEST_ASSERT_MSG_EQ(dcis.contains(rnti), true, "Every UE with data must be served");
        const auto& data = dcis.at(rnti);
        NS_TEST_EXPECT_MSG_GT_OR_EQ(data->m_tbSize,
                                    buffer,
                                    "The transport block must carry the data of the UE");
        bool inSpan = false;
        for (uint16_t retxRnti : {1, 2})
        {
            const auto& retx = dcis.at(retxRnti);
            const bool overlapInTime = data->m_symStart < retx->m_symStart + retx->m_numSym &&
                                       retx->m_symStart < data->m_symStart + data->m_numSym;
            if (!overlapInTime)
            {
                continue;
            }
            // On a retransmission span, the new data takes exactly its symbols
            NS_TEST_EXPECT_MSG_EQ(+data->m_symStart,
                                  +retx->m_symStart,
                                  "New data must use the symbols of the retransmission");
            NS_TEST_EXPECT_MSG_EQ(+data->m_numSym,
                                  +retx->m_numSym,
                                  "New data must span the symbols of the retransmission");
            for (std::size_t rbg = 0; rbg < m_numRbg; ++rbg)
            {
                NS_TEST_EXPECT_MSG_EQ((data->m_rbgBitmask[rbg] && retx->m_rbgBitmask[rbg]),
                                      false,
                                      "New data must not use the retransmission's RBGs");
            }
            spanUes[retxRnti]++;
            inSpan = true;
        }
        if (rnti == 5)
        {
            NS_TEST_EXPECT_MSG_EQ(inSpan, m_lastInSpan, "Unexpected placement of UE 5");
        }
    }

    // The RBGs left free by each retransmission are used
    NS_TEST_EXPECT_MSG_GT(spanUes[1], 0, "New data must use the RBGs left free above UE 1");
    NS_TEST_EXPECT_MSG_GT(spanUes[2], 0, "New data must use the RBGs left free below UE 2");
}

void
NrTestMacSchedulerHarqRrUlSpanSpread::DoRun()
{
    // No SRS, so that the slot only holds the data under test
    Config::SetDefault("ns3::NrMacSchedulerNs3::EnableSrsInUlSlots", BooleanValue(false));
    // With 1-RB RBGs, a low MCS cannot fit a minimum transport block in a few RBGs
    Config::SetDefault("ns3::NrMacSchedulerNs3::StartingMcsUl", UintegerValue(20));

    auto cellConfig = NrMacCschedSapProvider::CschedCellConfigReqParameters();
    cellConfig.m_dlBandwidth = m_numRbg;
    cellConfig.m_ulBandwidth = m_numRbg;

    // The SAP user hands the scheduled slot to CheckSchedule()
    auto* schedSapUser = new TestSchedSapUserHarq([this](auto params) { CheckSchedule(params); },
                                                  []() { return uint32_t{14}; });
    auto* cschedSapUser = new TestCschedSapUserHarq();

    auto sched = CreateObject<NrMacSchedulerOfdmaRR>();
    sched->InstallDlAmc(CreateObject<NrAmc>());
    sched->InstallUlAmc(CreateObject<NrAmc>());
    sched->SetMacSchedSapUser(schedSapUser);
    sched->SetMacCschedSapUser(cschedSapUser);
    sched->DoCschedCellConfigReq(cellConfig);
    // All UEs share a beam
    for (uint16_t rnti = 1; rnti <= 5; ++rnti)
    {
        NrMacCschedSapProvider::CschedUeConfigReqParameters config{};
        config.m_rnti = rnti;
        config.m_beamId = BeamId(0, 0);
        sched->DoCschedUeConfigReq(config);
    }
    // 1 UL CTRL symbol leaves 13 UL data symbols
    sched->SetUlCtrlSyms(1);

    // UEs 3, 4 and 5 report their UL data
    m_buffer = {{3, 20}, {4, 20}, {5, m_lastBuffer}};
    for (const auto& [rnti, buffer] : m_buffer)
    {
        NrMacCschedSapProvider::CschedLcConfigReqParameters lcParams;
        lcParams.m_rnti = rnti;
        lcParams.m_reconfigureFlag = false;
        nr::LogicalChannelConfigListElement_s lc;
        lc.m_logicalChannelIdentity = 1;
        lc.m_logicalChannelGroup = 2;
        lc.m_direction = nr::LogicalChannelConfigListElement_s::DIR_UL;
        lc.m_qosBearerType = nr::LogicalChannelConfigListElement_s::QBT_NON_GBR;
        lc.m_fiveQi = 9;
        lcParams.m_logicalChannelConfigList.emplace_back(lc);
        sched->DoCschedLcConfigReq(lcParams);

        NrMacSchedSapProvider::SchedUlMacCtrlInfoReqParameters bsrParams;
        MacCeElement bsr;
        bsr.m_rnti = rnti;
        bsr.m_macCeType = MacCeElement::BSR;
        bsr.m_macCeValue.m_bufferStatus = {0, 0, NrMacShortBsrCe::FromBytesToLevel(buffer), 0};
        bsrParams.m_macCeList.push_back(bsr);
        sched->DoSchedUlMacCtrlInfoReq(bsrParams);
    }

    // UE 1 retransmits on 4 symbols and RBGs [0,5], UE 2 on 6 symbols and RBGs [6,11].
    // Store the first transmissions in their HARQ process 0, waiting for feedback.
    NrMacSchedSapProvider::SchedUlTriggerReqParameters params;
    params.m_snfSf = SfnSf(0, 0, 0, 0);
    params.m_slotType = LteNrTddSlotType::UL;
    for (uint16_t rnti : {1, 2})
    {
        const uint8_t numSym = rnti == 1 ? 4 : 6;
        auto& harqProcess = sched->m_ueMap.find(rnti)->second->m_ulHarq.Find(0)->second;
        std::vector<bool> bitmask(m_numRbg, false);
        std::fill_n(bitmask.begin() + (rnti == 1 ? 0 : 6), 6, true);
        DciInfoElementTdma dci(rnti,
                               DciInfoElementTdma::UL,
                               0,
                               numSym,
                               10,
                               1,
                               {},
                               100,
                               1,
                               0,
                               DciInfoElementTdma::DATA,
                               0,
                               0);
        dci.m_harqProcess = 0;
        dci.m_rbgBitmask = bitmask;
        harqProcess.m_dciElement = std::make_shared<DciInfoElementTdma>(dci);
        harqProcess.m_active = true;
        harqProcess.m_status = HarqProcess::WAITING_FEEDBACK;

        // The gNB failed to decode it, so it must be retransmitted
        UlHarqInfo harqInfo;
        harqInfo.m_receptionStatus = UlHarqInfo::NotOk;
        harqInfo.m_numRetx = 0;
        harqInfo.m_rnti = rnti;
        harqInfo.m_harqProcessId = 0;
        params.m_ulHarqInfoList.push_back(harqInfo);
    }

    // Schedule the slot; CheckSchedule() runs from inside this call
    sched->DoSchedUlTriggerReq(params);

    NS_TEST_ASSERT_MSG_EQ(m_checked,
                          true,
                          "Scheduler did not report a slot allocation; checks did not run");

    delete schedSapUser;
    delete cschedSapUser;
    Simulator::Destroy();
    // Restore the defaults changed above for the following test cases
    Config::Reset();
}

/**
 * @ingroup test
 * @brief Expose the HARQ sorting of the TDMA RR scheduler.
 */
class NrTestMacSchedulerHarqSort : public NrMacSchedulerTdmaRR
{
  public:
    using NrMacSchedulerNs3::SortDlHarq;
    using NrMacSchedulerNs3::SortUlHarq;
};

/**
 * @ingroup test
 * @brief Check that the RR HARQ scheduler sorts the DL and UL retransmissions
 * of each beam by decreasing number of symbols, keeping the order of ties.
 *
 * What is under test: NrMacSchedulerHarqRr::SortDlHarq() and SortUlHarq(). They
 * receive the map of beams to pending retransmissions and reorder the list of
 * each beam, which is the order in which ScheduleDlHarq() and ScheduleUlHarq()
 * later place them in the slot. SortUlHarq() reuses the DL sort, so both
 * directions are checked against the same expectation.
 *
 * How: no slot is scheduled. The test builds the map by hand, with HARQ processes
 * that differ only in their number of symbols, calls the sort directly, and
 * compares the resulting sequence of HARQ process IDs of each beam with a
 * hand-computed one. It fails when a list is not in decreasing order, when two
 * retransmissions with the same number of symbols swap places, or when a process
 * moves to the list of another beam.
 */
class NrTestMacSchedulerHarqRrSort : public TestCase
{
  public:
    /**
     * @brief Constructor
     * @param isDl Whether to sort DL or UL HARQ retransmissions
     */
    NrTestMacSchedulerHarqRrSort(bool isDl)
        : TestCase(std::string("HARQ RR sorts ") + (isDl ? "DL" : "UL") +
                   " retransmissions by decreasing number of symbols"),
          m_isDl(isDl)
    {
    }

  private:
    void DoRun() override;

    bool m_isDl; ///< Whether to sort DL or UL HARQ retransmissions
};

void
NrTestMacSchedulerHarqRrSort::DoRun()
{
    // Input: the number of symbols of each pending retransmission, per beam, in
    // the order they are inserted. HARQ process IDs are assigned sequentially
    // across beams, so beam (0,0) holds IDs 0 to 4 and beam (1,0) holds IDs 5 and 6:
    //
    //   beam (0,0): ID      0  1  2  3  4      beam (1,0): ID      5  6
    //               symbols 2  6  3  6  9                  symbols 1  4
    //
    // Beam (0,0) is unsorted and has a tie (IDs 1 and 3, 6 symbols each), to
    // check that the sort is stable. Beam (1,0) is in increasing order, to
    // check that each beam is sorted on its own.
    const std::vector<std::pair<BeamId, std::vector<uint8_t>>> numSymsPerBeam{
        {BeamId(0, 0), {2, 6, 3, 6, 9}},
        {BeamId(1, 0), {1, 4}},
    };

    // The sort operates on iterators to HARQ processes, so the processes need to
    // live in a HARQ vector, as they do in the UE info of a real scheduler. Only
    // the number of symbols of each DCI matters, the other fields are placeholders.
    NrMacHarqVector harqVector;
    harqVector.SetMaxSize(16);
    NrMacSchedulerNs3::ActiveHarqMap activeHarq;
    uint8_t harqId = 0;
    for (const auto& [beam, numSyms] : numSymsPerBeam)
    {
        for (auto numSym : numSyms)
        {
            auto it = harqVector.Find(harqId++);
            it->second.m_dciElement = std::make_shared<DciInfoElementTdma>(
                0,
                m_isDl ? DciInfoElementTdma::DL : DciInfoElementTdma::UL,
                0,
                numSym,
                10,
                1,
                nullptr,
                100,
                0,
                0,
                DciInfoElementTdma::DATA,
                0,
                0);
            activeHarq[beam].push_back(it);
        }
    }

    // The sort methods are protected in the scheduler, so they are called through
    // a subclass that makes them public. The scheduler forwards them to
    // NrMacSchedulerHarqRr, and needs no cell or UE configuration for that.
    auto sched = CreateObject<NrTestMacSchedulerHarqSort>();
    if (m_isDl)
    {
        sched->SortDlHarq(&activeHarq);
    }
    else
    {
        sched->SortUlHarq(&activeHarq);
    }

    // Expected output: the HARQ process IDs of each beam, by decreasing number
    // of symbols. Ties keep their insertion order (ID 1 before ID 3):
    //
    //   beam (0,0): symbols 9  6  6  3  2      beam (1,0): symbols 4  1
    //               ID      4  1  3  2  0                  ID      6  5
    const std::vector<std::pair<BeamId, std::vector<uint8_t>>> expectedIds{
        {BeamId(0, 0), {4, 1, 3, 2, 0}},
        {BeamId(1, 0), {6, 5}},
    };
    for (const auto& [beam, ids] : expectedIds)
    {
        const auto& sorted = activeHarq.at(beam);
        NS_TEST_ASSERT_MSG_EQ(sorted.size(), ids.size(), "Unexpected number of HARQ processes");
        for (size_t i = 0; i < ids.size(); i++)
        {
            NS_TEST_EXPECT_MSG_EQ(+sorted.at(i)->first,
                                  +ids.at(i),
                                  "Unexpected HARQ process at position " << i);
        }
    }

    Simulator::Destroy();
}

/**
 * @ingroup test
 * @brief Check that the RR HARQ scheduler serves the beams with retransmissions in
 * round-robin order.
 *
 * What is under test: NrMacSchedulerHarqRr::GetBeamOrderRR(), through
 * ScheduleDlHarq() and ScheduleUlHarq(). The HARQ scheduler keeps one queue of
 * beams per link direction. In each slot it serves the beams with pending
 * retransmissions in queue order, and then moves the beam it served first to the
 * back of the queue. When symbols run out, the beams served last are the ones
 * whose retransmissions are postponed, so rotating the queue is what prevents
 * the same beam from being postponed in every slot.
 *
 * How: three UEs, one per beam, each keep one retransmission pending in every
 * slot, for six slots (two full rounds over the three beams). The position of the
 * retransmissions in the slot tells in which order the beams were served: the
 * beam served first gets the first DL symbols, or the last UL symbols, since UL
 * retransmissions are placed from the end of the slot backwards.
 *
 * Without postponing, the retransmissions take 2 symbols each and all three fit
 * in the slot, so only the order changes between slots. With postponing, they
 * take 6 symbols each and only two fit in the 13 data symbols, so the beam served
 * last is postponed. Its feedback is retried by the scheduler in the next slot,
 * and the other two UEs report a new failure, so every beam again has one
 * retransmission pending.
 *
 * With beams A, B and C, the expected behavior is:
 *
 *   slot          0      1      2      3      4      5
 *   beam order    A B C  B C A  C A B  A B C  B C A  C A B
 *   served first  A      B      C      A      B      C
 *
 * With postponing, the beam postponed in each slot is the last one of its order:
 *
 *   slot          0      1      2      3      4      5
 *   postponed     C      A      B      C      A      B
 *
 * The test records the UE served first in each slot and checks two things:
 * during the first three slots every UE is served first exactly once, and the
 * last three slots repeat the first three. It fails when the queue does not
 * rotate (the same beam is always first), or when it rotates irregularly. With
 * postponing, it also checks that exactly one beam is postponed per slot, that
 * the beam postponed in a slot is served in the next one, and that every beam is
 * postponed once per round. Without the rotation, the same beam would be
 * postponed in every slot and its retransmission would never be served.
 *
 * The initial order (which beam is A) is not checked, because the beams are
 * learned from an unordered map and their first order is arbitrary.
 */
class NrTestMacSchedulerHarqRrBeamRoundRobin : public TestCase
{
  public:
    /**
     * @brief Constructor
     * @param isDl Whether to schedule DL or UL HARQ retransmissions
     * @param postpone Whether only two of the three retransmissions fit in a slot
     */
    NrTestMacSchedulerHarqRrBeamRoundRobin(bool isDl, bool postpone)
        : TestCase(std::string("HARQ RR serves the ") + (isDl ? "DL" : "UL") +
                   " retransmission beams in round-robin order" +
                   (postpone ? ", postponing a different beam in each slot" : "")),
          m_isDl(isDl),
          m_postpone(postpone)
    {
    }

  private:
    void DoRun() override;

    /**
     * @brief Store the RNTIs whose retransmissions were served in the slot, and the
     * one served first.
     * @param params the scheduled slot allocation reported by the scheduler
     */
    void StoreFirstServed(const NrMacSchedSapUser::SchedConfigIndParameters& params);

    bool m_isDl;                       ///< Whether to schedule DL or UL HARQ retransmissions
    bool m_postpone;                   ///< Whether only two retransmissions fit in a slot
    std::vector<uint16_t> m_firstRnti; ///< RNTI served first in each slot
    std::vector<std::set<uint16_t>> m_servedRntis; ///< RNTIs served in each slot
};

void
NrTestMacSchedulerHarqRrBeamRoundRobin::StoreFirstServed(
    const NrMacSchedSapUser::SchedConfigIndParameters& params)
{
    // Called by the scheduler with the final allocation of each slot. The
    // allocation does not say in which order the beams were served, so it is
    // inferred from the symbols of each retransmission: DL retransmissions are
    // placed from the first data symbol onwards, so the first served has the
    // lowest starting symbol. UL retransmissions are placed from the end of the
    // slot backwards, so the first served has the highest starting symbol.
    const auto format = m_isDl ? DciInfoElementTdma::DL : DciInfoElementTdma::UL;
    std::shared_ptr<DciInfoElementTdma> first;
    std::set<uint16_t> served;
    for (const auto& varTtiAllocInfo : params.m_slotAllocInfo.m_varTtiAllocInfo)
    {
        const auto& dci = varTtiAllocInfo.m_dci;
        // Skip the control allocations, only the retransmissions matter
        if (dci->m_type != DciInfoElementTdma::DATA || dci->m_format != format)
        {
            continue;
        }
        served.insert(dci->m_rnti);
        if (!first ||
            (m_isDl ? dci->m_symStart < first->m_symStart : dci->m_symStart > first->m_symStart))
        {
            first = dci;
        }
    }
    NS_TEST_ASSERT_MSG_EQ(served.size(),
                          m_postpone ? 2 : 3,
                          "Unexpected number of retransmissions in the slot");
    // There is one UE per beam, so the RNTI identifies the beam served first
    m_firstRnti.push_back(first->m_rnti);
    m_servedRntis.push_back(served);
}

void
NrTestMacSchedulerHarqRrBeamRoundRobin::DoRun()
{
    // Step 1: create a standalone scheduler, without MAC, PHY or channel.
    //
    // The test plays the role of the MAC: it triggers the scheduler by hand, once
    // per slot, and receives the resulting slot allocation in StoreFirstServed()
    // through the fake SAP user. SRS is disabled so that the UL slots hold only
    // the retransmissions.
    Config::SetDefault("ns3::NrMacSchedulerNs3::EnableSrsInUlSlots", BooleanValue(false));

    auto cellConfig = NrMacCschedSapProvider::CschedCellConfigReqParameters();
    cellConfig.m_dlBandwidth = 10; // 10 RBGs
    cellConfig.m_ulBandwidth = 10;

    auto* schedSapUser =
        new TestSchedSapUserHarq([this](auto params) { StoreFirstServed(params); });
    auto* cschedSapUser = new TestCschedSapUserHarq();

    // Step 2: register three UEs, each in a beam of its own, so that serving a
    // beam means serving exactly one UE. The TDMA and OFDMA schedulers share the
    // HARQ scheduler, so one of them is enough.
    auto sched = CreateObject<NrMacSchedulerTdmaRR>();
    sched->InstallDlAmc(CreateObject<NrAmc>());
    sched->InstallUlAmc(CreateObject<NrAmc>());
    sched->SetMacSchedSapUser(schedSapUser);
    sched->SetMacCschedSapUser(cschedSapUser);
    sched->DoCschedCellConfigReq(cellConfig);
    const std::vector<uint16_t> rntis = {1, 2, 3};
    for (size_t i = 0; i < rntis.size(); ++i)
    {
        NrMacCschedSapProvider::CschedUeConfigReqParameters config{};
        config.m_rnti = rntis.at(i);
        config.m_beamId = BeamId(static_cast<uint16_t>(i), 0);
        sched->DoCschedUeConfigReq(config);
    }
    sched->SetDlCtrlSyms(1);
    sched->SetUlCtrlSyms(1);

    // Step 3: schedule two full rounds over the beams. In every slot, each UE
    // whose retransmission was not postponed reports the failure of one transport
    // block, so all three beams always have exactly one retransmission pending and
    // compete for the first position.
    const uint8_t numSym = m_postpone ? 6 : 2;
    const size_t numSlots = 2 * rntis.size();
    for (size_t slot = 0; slot < numSlots; ++slot)
    {
        // No first transmission is ever scheduled. Instead, the HARQ process is
        // written directly in the state it would have after one (2 symbols, all
        // RBGs, waiting for feedback), and a NACK for it is handed to the
        // scheduler. Each slot uses a new process ID, because the processes of the
        // previous slots were retransmitted and wait for a feedback that never comes.
        const auto harqId = static_cast<uint8_t>(slot);
        NrMacSchedSapProvider::SchedDlTriggerReqParameters paramsDl;
        NrMacSchedSapProvider::SchedUlTriggerReqParameters paramsUl;
        const SfnSf sfnSf(0, static_cast<uint8_t>(slot), 0, 0);
        paramsDl.m_snfSf = sfnSf;
        paramsDl.m_slotType = LteNrTddSlotType::DL;
        paramsUl.m_snfSf = sfnSf;
        paramsUl.m_slotType = LteNrTddSlotType::UL;
        for (auto rnti : rntis)
        {
            // The scheduler keeps the feedback of a postponed retransmission and
            // retries it in this slot
            if (slot > 0 && m_servedRntis.back().count(rnti) == 0)
            {
                continue;
            }
            auto& ueInfo = sched->m_ueMap.find(rnti)->second;
            auto& harqProcess = m_isDl ? ueInfo->m_dlHarq.Find(harqId)->second
                                       : ueInfo->m_ulHarq.Find(harqId)->second;
            DciInfoElementTdma dci(rnti,
                                   m_isDl ? DciInfoElementTdma::DL : DciInfoElementTdma::UL,
                                   0,
                                   numSym,
                                   10,
                                   1,
                                   {},
                                   // Too large to be reshaped into the symbol left
                                   // after two 6-symbol retransmissions
                                   m_postpone ? 10000 : 100,
                                   1,
                                   0,
                                   DciInfoElementTdma::DATA,
                                   0,
                                   0);
            dci.m_harqProcess = harqId;
            dci.m_rbgBitmask = std::vector<bool>(10, true);
            harqProcess.m_dciElement = std::make_shared<DciInfoElementTdma>(dci);
            harqProcess.m_active = true;
            harqProcess.m_status = HarqProcess::WAITING_FEEDBACK;

            if (m_isDl)
            {
                DlHarqInfo harqInfo;
                harqInfo.m_harqStatus = DlHarqInfo::NACK;
                harqInfo.m_numRetx = 0;
                harqInfo.m_rnti = rnti;
                harqInfo.m_harqProcessId = harqId;
                harqInfo.m_bwpIndex = 0;
                paramsDl.m_dlHarqInfoList.push_back(harqInfo);
            }
            else
            {
                UlHarqInfo harqInfo;
                harqInfo.m_receptionStatus = UlHarqInfo::NotOk;
                harqInfo.m_numRetx = 0;
                harqInfo.m_rnti = rnti;
                harqInfo.m_harqProcessId = harqId;
                paramsUl.m_ulHarqInfoList.push_back(harqInfo);
            }
        }
        // Schedule the slot. The retransmissions span all RBGs, so they are placed
        // one after the other in time: 3 x 2 symbols fit in the 13 data symbols,
        // while with 3 x 6 symbols the third one is postponed. The scheduler reports
        // the slot allocation to StoreFirstServed() before this call returns.
        if (m_isDl)
        {
            sched->DoSchedDlTriggerReq(paramsDl);
        }
        else
        {
            sched->DoSchedUlTriggerReq(paramsUl);
        }
    }

    // Step 4: check the sequence of UEs served first, one entry per slot.
    NS_TEST_ASSERT_MSG_EQ(m_firstRnti.size(), numSlots, "Every slot should be scheduled");

    // Fairness: in the first round (slots 0 to 2), the three slots have three
    // different UEs served first, so no beam is first twice while another waits.
    const std::set<uint16_t> firstInCycle(m_firstRnti.begin(), m_firstRnti.begin() + rntis.size());
    NS_TEST_EXPECT_MSG_EQ(firstInCycle.size(), rntis.size(), "Every beam should be served first");

    // Periodicity: the second round (slots 3 to 5) serves the same UE first as
    // the slot three slots earlier, so the order is a fixed rotation.
    for (size_t slot = rntis.size(); slot < numSlots; ++slot)
    {
        NS_TEST_EXPECT_MSG_EQ(m_firstRnti.at(slot),
                              m_firstRnti.at(slot - rntis.size()),
                              "The round-robin order should repeat");
    }

    if (m_postpone)
    {
        std::vector<uint16_t> postponed;
        for (const auto& served : m_servedRntis)
        {
            for (auto rnti : rntis)
            {
                if (served.count(rnti) == 0)
                {
                    postponed.push_back(rnti);
                }
            }
        }
        NS_TEST_ASSERT_MSG_EQ(postponed.size(), numSlots, "One beam should be postponed per slot");
        for (size_t slot = 1; slot < numSlots; ++slot)
        {
            NS_TEST_EXPECT_MSG_NE(postponed.at(slot),
                                  postponed.at(slot - 1),
                                  "A beam should not be postponed in two consecutive slots");
        }
        for (size_t round = 0; round < numSlots; round += rntis.size())
        {
            const std::set<uint16_t> postponedInRound(postponed.begin() + round,
                                                      postponed.begin() + round + rntis.size());
            NS_TEST_EXPECT_MSG_EQ(postponedInRound.size(),
                                  rntis.size(),
                                  "Every beam should be postponed once per round");
        }
    }

    delete schedSapUser;
    delete cschedSapUser;
    Simulator::Destroy();
    Config::Reset();
}

class NrTestSchedHarqSuite : public TestSuite
{
  public:
    NrTestSchedHarqSuite()
        : TestSuite("nr-test-sched-harq", Type::UNIT)
    {
        AddTestCase(new NrTestMacSchedulerHarqRrBeamOrder(), Duration::QUICK);
        AddTestCase(new NrTestMacSchedulerHarqRrSort(true /* DL */), Duration::QUICK);
        AddTestCase(new NrTestMacSchedulerHarqRrSort(false /* UL */), Duration::QUICK);
        for (bool postpone : {false, true})
        {
            AddTestCase(new NrTestMacSchedulerHarqRrBeamRoundRobin(true /* DL */, postpone),
                        Duration::QUICK);
            AddTestCase(new NrTestMacSchedulerHarqRrBeamRoundRobin(false /* UL */, postpone),
                        Duration::QUICK);
        }
        AddTestCase(new NrTestMacSchedulerHarqRrSymbolBudget(true /* TDMA */), Duration::QUICK);
        AddTestCase(new NrTestMacSchedulerHarqRrSymbolBudget(false /* OFDMA */), Duration::QUICK);
        AddTestCase(new NrTestMacSchedulerHarqRrOfdmaSharing(false /* not consolidated */),
                    Duration::QUICK);
        AddTestCase(new NrTestMacSchedulerHarqRrOfdmaSharing(true /* consolidated */),
                    Duration::QUICK);
        AddTestCase(new NrTestMacSchedulerHarqRrUlSharing(true, true, true /* one UE */),
                    Duration::QUICK);
        for (bool sameBeam : {true, false})
        {
            for (bool disjointRbgs : {true, false})
            {
                AddTestCase(new NrTestMacSchedulerHarqRrUlSharing(sameBeam, disjointRbgs),
                            Duration::QUICK);
            }
            AddTestCase(new NrTestMacSchedulerHarqRrDlReshape(sameBeam), Duration::QUICK);
            for (bool ofdma : {true, false})
            {
                AddTestCase(new NrTestMacSchedulerHarqRrUlFill(ofdma, sameBeam), Duration::QUICK);
                AddTestCase(new NrTestMacSchedulerHarqRrDlFill(ofdma, sameBeam), Duration::QUICK);
            }
        }

        for (bool isDl : {true, false})
        {
            // Two UEs with plenty of data: one fills the free RBGs, the other the rest
            AddTestCase(
                new NrTestMacSchedulerHarqRrFillUtilization(isDl, {100000, 100000}, {0}, false),
                Duration::QUICK);
            // A single UE with plenty of data keeps the larger rest of the slot
            AddTestCase(new NrTestMacSchedulerHarqRrFillUtilization(isDl, {100000}, {}, true),
                        Duration::QUICK);
            // The UE with less data fills the free RBGs, the other the rest
            AddTestCase(
                new NrTestMacSchedulerHarqRrFillUtilization(isDl, {150, 100000}, {2}, false),
                Duration::QUICK);
        }
        // UE 5 fits on a retransmission span, or keeps symbols of its own
        AddTestCase(new NrTestMacSchedulerHarqRrUlSpanSpread(40, true), Duration::QUICK);
        AddTestCase(new NrTestMacSchedulerHarqRrUlSpanSpread(60, false), Duration::QUICK);

        // MaxHarqReTx cases.
        for (const auto maxHarqReTx : MAX_RETX_VALUES)
        {
            AddTestCase(new NrHarqRetxSchedulerCase(maxHarqReTx, true), Duration::QUICK);
            AddTestCase(new NrHarqRetxSchedulerCase(maxHarqReTx, false), Duration::QUICK);
        }

        AddTestCase(new NrHarqRetxSchedulerCase(5, true, false), Duration::QUICK);
        AddTestCase(new NrHarqRetxSchedulerCase(5, false, false), Duration::QUICK);

        // clang-format off
        using DIET = DciInfoElementTdma;
        std::vector<DciInfoElementTdma> dcis{
            // beam 0
            // rnti,   format, startSym, numSym, mcs, rank, precmat, tbs, ndi, rv,       type, bwp, tpc
            {     0, DIET::DL,        0,      4,  10,    4,      {}, 800,   0,  0, DIET::DATA,   0,   0},
            {     1, DIET::DL,        1,      7,  17,    1,      {}, 200,   0,  0, DIET::DATA,   0,   0},
            {     2, DIET::DL,        2,      1,  13,    3,      {}, 600,   0,  0, DIET::DATA,   0,   0},
            {     3, DIET::DL,        3,      9,  10,    2,      {}, 400,   0,  0, DIET::DATA,   0,   0},
            {     4, DIET::DL,        4,      2,   6,    4,      {}, 800,   0,  0, DIET::DATA,   0,   0},
            // beam 1
            {     5, DIET::DL,        5,      5,  20,    4,      {}, 800,   0,  0, DIET::DATA,   0,   0},
            {     6, DIET::DL,        6,      3,  10,    1,      {}, 200,   0,  0, DIET::DATA,   0,   0},
            {     7, DIET::DL,        7,      8,   1,    1,      {}, 200,   0,  0, DIET::DATA,   0,   0},
            {     8, DIET::DL,        8,      1,   7,    3,      {}, 600,   0,  0, DIET::DATA,   0,   0},
            {     9, DIET::DL,        9,      2,  19,    2,      {}, 400,   0,  0, DIET::DATA,   0,   0}
        };
        dcis.at(0).m_rbgBitmask = { true,  true, false, false, false,  true, false, false,  true,  true};
        dcis.at(1).m_rbgBitmask = {false,  true,  true, false,  true,  true,  true, false,  true, false};
        dcis.at(2).m_rbgBitmask = {false, false,  true, false, false,  true,  true,  true,  true,  true};
        dcis.at(3).m_rbgBitmask = { true, false,  true, false,  true, false,  true, false,  true, false};
        dcis.at(4).m_rbgBitmask = {false,  true, false,  true, false,  true, false,  true, false,  true};
        dcis.at(5).m_rbgBitmask = { true, false,  true,  true, false, false, false, false, false,  true};
        dcis.at(6).m_rbgBitmask = { true, false, false,  true, false,  true, false, false, false,  true};
        dcis.at(7).m_rbgBitmask = { true, false, false, false,  true,  true,  true, false, false,  true};
        dcis.at(8).m_rbgBitmask = { true, false, false, false, false,  true,  true,  true, false,  true};
        dcis.at(9).m_rbgBitmask = { true,  true, false,  true, false,  true,  true, false, false, false};
        dcis.at(0).m_harqProcess = 0;
        dcis.at(1).m_harqProcess = 1;
        dcis.at(2).m_harqProcess = 2;
        dcis.at(3).m_harqProcess = 3;
        dcis.at(4).m_harqProcess = 4;
        dcis.at(5).m_harqProcess = 5;
        dcis.at(6).m_harqProcess = 6;
        dcis.at(7).m_harqProcess = 7;
        dcis.at(8).m_harqProcess = 8;
        dcis.at(9).m_harqProcess = 9;
        // clang-format on
        for (auto [startSym, numSym] : std::vector<std::pair<uint8_t, uint8_t>>{
                 {0, 0},
                 {0, 13},
                 {6, 14},
                 {0, 1},
                 {1, 13},
             })
        {
            // Test reshaping alone
            AddTestCase(new NrTestMacSchedulerHarqRrReshape(
                            std::vector<DIET>(dcis.begin(), dcis.begin() + 4),
                            startSym,
                            numSym,
                            "Reshape: Beam   0, startSym " + std::to_string(startSym) +
                                ", numSym " + std::to_string(numSym)),
                        Duration::QUICK);
            AddTestCase(new NrTestMacSchedulerHarqRrReshape(
                            std::vector<DIET>(dcis.begin() + 6, dcis.end()),
                            startSym,
                            numSym,
                            "Reshape: Beam   1, startSym " + std::to_string(startSym) +
                                ", numSym " + std::to_string(numSym)),
                        Duration::QUICK);
            AddTestCase(new NrTestMacSchedulerHarqRrReshape(
                            std::vector<DIET>(dcis.begin() + 3, dcis.begin() + 7),
                            startSym,
                            numSym,
                            "Reshape: Beam 0+1, startSym " + std::to_string(startSym) +
                                ", numSym " + std::to_string(numSym)),
                        Duration::QUICK);
            // Test reshaping via scheduler
            AddTestCase(new NrTestMacSchedulerHarqRrScheduleDlHarq(
                            std::vector<DIET>(dcis.begin(), dcis.begin() + 1),
                            startSym,
                            numSym,
                            "Reshape with scheduler: Beam   0, startSym " +
                                std::to_string(startSym) + ", numSym " + std::to_string(numSym)),
                        Duration::QUICK);
            AddTestCase(new NrTestMacSchedulerHarqRrScheduleDlHarq(
                            std::vector<DIET>(dcis.begin() + 1, dcis.begin() + 2),
                            startSym,
                            numSym,
                            "Reshape with scheduler: Beam   0, startSym " +
                                std::to_string(startSym) + ", numSym " + std::to_string(numSym)),
                        Duration::QUICK);
            AddTestCase(new NrTestMacSchedulerHarqRrScheduleDlHarq(
                            std::vector<DIET>(dcis.begin() + 2, dcis.begin() + 3),
                            startSym,
                            numSym,
                            "Reshape with scheduler: Beam   0, startSym " +
                                std::to_string(startSym) + ", numSym " + std::to_string(numSym)),
                        Duration::QUICK);
            AddTestCase(new NrTestMacSchedulerHarqRrScheduleDlHarq(
                            std::vector<DIET>(dcis.begin() + 3, dcis.begin() + 4),
                            startSym,
                            numSym,
                            "Reshape with scheduler: Beam   0, startSym " +
                                std::to_string(startSym) + ", numSym " + std::to_string(numSym)),
                        Duration::QUICK);
            AddTestCase(new NrTestMacSchedulerHarqRrScheduleDlHarq(
                            std::vector<DIET>(dcis.begin(), dcis.begin() + 4),
                            startSym,
                            numSym,
                            "Reshape with scheduler: Beam   0, startSym " +
                                std::to_string(startSym) + ", numSym " + std::to_string(numSym)),
                        Duration::QUICK);
            AddTestCase(new NrTestMacSchedulerHarqRrScheduleDlHarq(
                            std::vector<DIET>(dcis.begin() + 4, dcis.begin() + 6),
                            startSym,
                            numSym,
                            "Reshape with scheduler: Beam 0+1, startSym " +
                                std::to_string(startSym) + ", numSym " + std::to_string(numSym)),
                        Duration::QUICK);
            AddTestCase(new NrTestMacSchedulerHarqRrScheduleDlHarq(
                            std::vector<DIET>(dcis.begin() + 3, dcis.begin() + 7),
                            startSym,
                            numSym,
                            "Reshape with scheduler: Beam 0+1, startSym " +
                                std::to_string(startSym) + ", numSym " + std::to_string(numSym)),
                        Duration::QUICK);
            AddTestCase(new NrTestMacSchedulerHarqRrScheduleDlHarq(
                            std::vector<DIET>(dcis.begin() + 2, dcis.begin() + 8),
                            startSym,
                            numSym,
                            "Reshape with scheduler: Beam 0+1, startSym " +
                                std::to_string(startSym) + ", numSym " + std::to_string(numSym)),
                        Duration::QUICK);
        }
    }
};

static NrTestSchedHarqSuite nrSchedHarqTestSuite; //!< NR HARQ scheduler test suite

} // namespace ns3
