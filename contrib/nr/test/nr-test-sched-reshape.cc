// Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/nr-mac-scheduler-ue-info.h"
#include "ns3/test.h"

#include <algorithm>

/**
 * @file nr-test-sched-reshape.cc
 * @ingroup test
 *
 * @brief Reshaping a HARQ retransmission must terminate and pick the smallest shape that fits.
 */
namespace ns3
{

/**
 * @ingroup test
 * @brief Minimal SAP user: the reshape path needs the slot geometry and nothing else.
 */
class ReshapeSchedSapUser : public NrMacSchedSapUser
{
  public:
    void SchedConfigInd(const struct SchedConfigIndParameters&) override
    {
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

    void BuildRarList(SlotAllocInfo&) override
    {
    }
};

/**
 * @ingroup test
 * @brief Minimal control SAP user: every confirmation is ignored.
 */
class ReshapeCschedSapUser : public NrMacCschedSapUser
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
 * @brief Exposes the protected reshape trampoline so a test can drive it directly.
 */
class ReshapeAccessor : public NrMacSchedulerTdmaRR
{
  public:
    /**
     * Reshape DL DCIs into the available resources.
     *
     * @param dcis DCIs to reshape.
     * @param startingSymbol First available symbol, advanced by the symbols used.
     * @param numSymbols Available symbols, reduced by the symbols used.
     * @param bitmask Available RBGs.
     * @return The reshaped DCIs.
     */
    std::vector<DciInfoElementTdma> Reshape(const std::vector<DciInfoElementTdma>& dcis,
                                            uint8_t& startingSymbol,
                                            uint8_t& numSymbols,
                                            std::vector<bool>& bitmask)
    {
        return ReshapeAllocation(dcis, startingSymbol, numSymbols, bitmask, true);
    }
};

/**
 * @ingroup test
 * @brief Reshape a HARQ retransmission into a 14-symbol slot and check the resulting shape.
 *
 * A retransmission is either refitted into the fewest symbols that hold at least its original
 * number of resources, or declined without consuming any slot resources.
 *
 * Grants of 137 RBGs over 14 symbols (1918 resources) with 13 free RBGs would need
 * ceil(1918/13) = 148 symbols, so they must be declined. A search that raises a uint8_t symbol
 * count until the resource counts match exactly never terminates on this input.
 */
class ReshapeTestCase : public TestCase
{
  public:
    /**
     * @param name Test case description.
     * @param dciRbgs Resource block groups in the original grant.
     * @param freeRbgs Available resource block groups.
     * @param dciSymbols Symbols in the original grant.
     * @param expectedSymbols Expected symbols of the reshaped grant, or 0 if it must be declined.
     * @param expectedRbgs Expected resource block groups of the reshaped grant.
     */
    ReshapeTestCase(const std::string& name,
                    std::size_t dciRbgs,
                    std::size_t freeRbgs,
                    uint8_t dciSymbols,
                    uint8_t expectedSymbols,
                    std::size_t expectedRbgs)
        : TestCase(name),
          m_dciRbgs(dciRbgs),
          m_freeRbgs(freeRbgs),
          m_dciSymbols(dciSymbols),
          m_expectedSymbols(expectedSymbols),
          m_expectedRbgs(expectedRbgs)
    {
    }

  private:
    void DoRun() override
    {
        constexpr uint16_t rnti = 1;
        // Widest NR carrier in RBGs, so that large grants can be built
        constexpr std::size_t bwpRbgs = 138;

        // Scheduler with stub SAPs: only the reshape logic is exercised
        auto sched = CreateObject<ReshapeAccessor>();
        sched->SetMacSchedSapUser(&m_schedSapUser);
        sched->SetMacCschedSapUser(&m_cschedSapUser);

        // The reshape looks up the UE, so it must be known to the scheduler
        NrMacCschedSapProvider::CschedUeConfigReqParameters ueParams{};
        ueParams.m_rnti = rnti;
        ueParams.m_beamId = BeamId(0, 0.0);
        sched->DoCschedUeConfigReq(ueParams);

        // Build the grant being retransmitted.
        // It uses the first m_dciRbgs RBGs over m_dciSymbols symbols.
        std::vector<bool> dciBitmask(bwpRbgs, false);
        std::fill_n(dciBitmask.begin(), m_dciRbgs, true);
        DciInfoElementTdma dci(rnti,
                               DciInfoElementTdma::DL,
                               0,            // symStart
                               m_dciSymbols, // numSym
                               0,            // mcs
                               1,            // rank
                               nullptr,
                               100, // tbs
                               0,   // ndi: retransmission
                               1,   // rv
                               DciInfoElementTdma::DATA,
                               0,  // bwpIndex
                               0); // tpc
        dci.m_rbgBitmask = dciBitmask;

        // What is still free when the retransmission is scheduled.
        // Only the first m_freeRbgs RBGs are available.
        std::vector<bool> bitmask(bwpRbgs, false);
        std::fill_n(bitmask.begin(), m_freeRbgs, true);

        // The whole 14-symbol slot is free, starting at symbol 0.
        // The reshape advances the start and consumes symbols if it succeeds.
        uint8_t startingSymbol = 0;
        uint8_t numSymbols = 14;
        const auto reshaped = sched->Reshape({dci}, startingSymbol, numSymbols, bitmask);

        // A declined reshape returns nothing and leaves the slot untouched
        if (m_expectedSymbols == 0)
        {
            NS_TEST_EXPECT_MSG_EQ(reshaped.empty(),
                                  true,
                                  "An unusable retransmission allocation must be declined");
            NS_TEST_EXPECT_MSG_EQ(+startingSymbol,
                                  0,
                                  "A declined reshape must not advance the start");
            NS_TEST_EXPECT_MSG_EQ(+numSymbols, 14, "A declined reshape must not consume symbols");
            return;
        }

        // A successful reshape returns one DCI for the retransmission
        NS_TEST_ASSERT_MSG_EQ(reshaped.size(), 1, "The retransmission must be reshaped");
        const auto& out = reshaped.front();
        const auto outRbgs = static_cast<std::size_t>(
            std::count(out.m_rbgBitmask.begin(), out.m_rbgBitmask.end(), true));
        // Check the shape: fewest symbols first, then fewest RBGs for those symbols
        NS_TEST_EXPECT_MSG_EQ(+out.m_symStart, 0, "Reshaped grant must start at the first symbol");
        NS_TEST_EXPECT_MSG_EQ(+out.m_numSym, +m_expectedSymbols, "Unexpected reshaped symbols");
        NS_TEST_EXPECT_MSG_EQ(outRbgs, m_expectedRbgs, "Unexpected reshaped RBGs");
        // The TBS is kept, so the new shape must hold at least the original resources
        NS_TEST_EXPECT_MSG_GT_OR_EQ(outRbgs * out.m_numSym,
                                    m_dciRbgs * m_dciSymbols,
                                    "Reshaped grant must hold the original resources");
        // Every RBG of the new shape must have been free
        for (std::size_t i = 0; i < out.m_rbgBitmask.size(); ++i)
        {
            NS_TEST_EXPECT_MSG_EQ((!out.m_rbgBitmask.at(i) || bitmask.at(i)),
                                  true,
                                  "Reshaped grant must only use free RBGs");
        }
        NS_TEST_EXPECT_MSG_EQ(+out.m_tbSize, 100, "Reshaping must keep the TBS");
        // The slot bookkeeping must reflect the symbols taken by the new shape
        NS_TEST_EXPECT_MSG_EQ(+startingSymbol,
                              +m_expectedSymbols,
                              "Reshaping must advance the start by the symbols used");
        NS_TEST_EXPECT_MSG_EQ(+numSymbols,
                              14 - m_expectedSymbols,
                              "Reshaping must consume the symbols used");
    }

    const std::size_t m_dciRbgs;          ///< Resource block groups in the original grant.
    const std::size_t m_freeRbgs;         ///< Available resource block groups.
    const uint8_t m_dciSymbols;           ///< Symbols in the original grant.
    const uint8_t m_expectedSymbols;      ///< Expected symbols after reshaping, 0 if declined.
    const std::size_t m_expectedRbgs;     ///< Expected resource block groups after reshaping.
    ReshapeSchedSapUser m_schedSapUser;   ///< Scheduler SAP user stub.
    ReshapeCschedSapUser m_cschedSapUser; ///< Scheduler control SAP user stub.
};

/**
 * @ingroup test
 * @brief Test suite for HARQ retransmission reshaping in the TDMA scheduler.
 */
class NrSchedReshapeTestSuite : public TestSuite
{
  public:
    NrSchedReshapeTestSuite()
        : TestSuite("nr-test-sched-reshape", Type::UNIT)
    {
        // Arguments: name, grant RBGs, free RBGs, grant symbols, expected symbols, expected RBGs.
        // Expected symbols 0 means the reshape must be declined.

        // 137 x 14 = 1918 resources would need 148 symbols on 13 RBGs
        AddTestCase(new ReshapeTestCase("Declines a retransmission that cannot be refitted",
                                        137,
                                        13,
                                        14,
                                        0,
                                        0),
                    Duration::QUICK);
        // No free RBGs, so nothing can be placed (and no division by zero)
        AddTestCase(
            new ReshapeTestCase("Declines a retransmission with no free RBGs", 137, 0, 14, 0, 0),
            Duration::QUICK);
        // Degenerate grants with no resources are declined
        AddTestCase(
            new ReshapeTestCase("Declines a retransmission with zero symbols", 137, 13, 0, 0, 0),
            Duration::QUICK);
        AddTestCase(
            new ReshapeTestCase("Declines a retransmission with zero RBGs", 0, 13, 14, 0, 0),
            Duration::QUICK);
        // 10 RBGs x 4 symbols fit exactly in 20 RBGs x 2 symbols
        AddTestCase(
            new ReshapeTestCase("Spreads a retransmission over more RBGs", 10, 20, 4, 2, 20),
            Duration::QUICK);
        // 137 is prime, so with 100 free RBGs no exact shape exists; 2 x 69 = 138 holds it
        AddTestCase(
            new ReshapeTestCase("Pads a retransmission with no exact shape", 137, 100, 1, 2, 69),
            Duration::QUICK);
        // 5 RBGs x 3 symbols on 4 free RBGs: 4 symbols x 4 RBGs = 16 >= 15
        AddTestCase(
            new ReshapeTestCase("Stretches a retransmission over more symbols", 5, 4, 3, 4, 4),
            Duration::QUICK);
    }
};

static NrSchedReshapeTestSuite
    g_nrSchedReshapeTestSuite; //!< Static variable for test initialization

} // namespace ns3
