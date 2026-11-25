// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/nr-fh-control.h"
#include "ns3/nr-fh-phy-sap.h"
#include "ns3/nr-fh-sched-sap.h"
#include "ns3/nr-mac-scheduler-ue-info.h"
#include "ns3/nr-phy-mac-common.h"
#include "ns3/test.h"

#include <deque>
#include <map>
#include <memory>
#include <unordered_map>

/**
 * @file nr-test-fh-control.cc
 * @ingroup test
 *
 * @brief Unit tests for NrFhControl.
 *
 * NrFhControl limits the DL allocations of a cell to the capacity of its
 * fronthaul (FH) link. The capacity is shared equally among the active BWPs,
 * i.e., the BWPs with UEs that have new data or wait for a HARQ
 * retransmission. The scheduler uses NrFhControl in two ways:
 * - DoesAllocationFit: checks whether an allocation still fits in the FH
 *   capacity left to its BWP in the slot.
 * - GetMaxRegAssignable and GetMaxMcsAssignable: the maximum REGs (at a given
 *   MCS) or MCS (for a given number of REGs) of a UE, sharing the BWP capacity
 *   among its active UEs, after removing the per-UE overhead. They are used by
 *   the OptimizeRBs and OptimizeMcs methods.
 *
 * NrFhControl is driven directly through its scheduler SAP provider, with stub
 * PHY and scheduler SAP users, so that the expected FH throughput of each
 * allocation can be computed by hand. All cases use numerology 0 (1 ms slot),
 * MCS Table 2 and 1 RB per RBG. The test cases check that:
 * - A BWP whose only UE waits for a HARQ retransmission stops being active
 *   once the retransmission is scheduled.
 * - A BWP with a UE that has both new data and a HARQ retransmission stays
 *   active until both are scheduled.
 * - The per-UE overhead is subtracted from the FH capacity, once per active
 *   UE, by GetMaxRegAssignable and GetMaxMcsAssignable.
 * - When every active UE gets the REGs or MCS returned by those methods, all
 *   the allocations fit in the FH capacity (FS 7.2, 1 and 3 UEs, every
 *   modulation order).
 * - The FH throughput of an allocation uses the antenna ports of its own BWP
 *   (FS 7.1).
 * - GetMaxRegAssignable returns 0 for a BWP that has only HARQ UEs.
 */

using namespace ns3;

namespace
{

constexpr uint8_t MCS_256QAM = 27;    //!< Highest MCS of Table 2 (modulation order 8)
constexpr uint8_t MAX_MCS_64QAM = 19; //!< Highest 64QAM MCS of Table 2 (modulation order 6)
constexpr uint8_t MAX_MCS_QPSK = 4;   //!< Highest QPSK MCS of Table 2 (modulation order 2)

/**
 * @brief Create a DL data allocation of one symbol over 10 RBGs.
 * @param rnti the UE RNTI
 * @param ndi the new data indicator (0 for a retransmission)
 * @return the allocation
 */
VarTtiAllocInfo
MakeDlDataAllocation(uint16_t rnti, uint8_t ndi)
{
    auto dci = std::make_shared<DciInfoElementTdma>(rnti,
                                                    DciInfoElementTdma::DL,
                                                    1,
                                                    1,
                                                    MCS_256QAM,
                                                    1,
                                                    nullptr,
                                                    100,
                                                    ndi,
                                                    1,
                                                    DciInfoElementTdma::DATA,
                                                    1,
                                                    0);
    dci->m_rbgBitmask = std::vector<bool>(10, true);
    return VarTtiAllocInfo(dci);
}

/**
 * @brief Stub of the gNB PHY side of the FH PHY SAP.
 */
class TestFhPhySapUser : public NrFhPhySapUser
{
  public:
    /**
     * @brief Constructor
     * @param antennaPorts the number of antenna ports reported to NrFhControl
     */
    TestFhPhySapUser(uint16_t antennaPorts)
        : m_antennaPorts(antennaPorts)
    {
    }

    uint16_t GetNumerology() const override
    {
        return 0;
    }

    uint16_t GetNumAntennaPorts() const override
    {
        return m_antennaPorts;
    }

  private:
    uint16_t m_antennaPorts; ///< Antenna ports of this BWP
};

/**
 * @brief Stub of the scheduler side of the FH scheduler SAP.
 */
class TestFhSchedSapUser : public NrFhSchedSapUser
{
  public:
    uint64_t GetNumRbPerRbgFromSched() override
    {
        return 1;
    }
};

/**
 * @brief Base class that wires one NrFhControl to stub SAP users, one per BWP.
 */
class NrFhControlTestCase : public TestCase
{
  public:
    /**
     * @brief Constructor
     * @param name the test case name
     */
    NrFhControlTestCase(const std::string& name)
        : TestCase(name)
    {
    }

  protected:
    /**
     * @brief Create the NrFhControl under test.
     * @param capacityMbps the FH capacity of the cell (in Mbps)
     * @param split the functional split
     * @param antennaPortsPerBwp the antenna ports of each BWP (its size sets the number of BWPs)
     */
    void Setup(uint64_t capacityMbps,
               NrFhControl::FunctionalSplit split,
               const std::vector<uint16_t>& antennaPortsPerBwp)
    {
        m_fhControl = CreateObject<NrFhControl>();
        m_fhControl->SetErrorModelType("ns3::NrEesmIrT2");
        m_fhControl->SetFunctionalSplit(split);
        m_fhControl->SetCellFhCapacity(capacityMbps);
        const auto numBwps = static_cast<uint16_t>(antennaPortsPerBwp.size());
        for (uint16_t bwpId = 0; bwpId < numBwps; ++bwpId)
        {
            m_phySapUsers[bwpId] = std::make_unique<TestFhPhySapUser>(antennaPortsPerBwp.at(bwpId));
            m_schedSapUsers[bwpId] = std::make_unique<TestFhSchedSapUser>();
            m_fhControl->SetFhNumerology(bwpId, 0);
            m_fhControl->SetNrFhPhySapUser(bwpId, m_phySapUsers[bwpId].get());
            m_fhControl->SetNrFhSchedSapUser(bwpId, m_schedSapUsers[bwpId].get());
        }
        m_sched = m_fhControl->GetNrFhSchedSapProvider();
    }

    void DoTeardown() override
    {
        m_fhControl = nullptr;
        m_phySapUsers.clear();
        m_schedSapUsers.clear();
    }

    Ptr<NrFhControl> m_fhControl;           ///< The object under test
    NrFhSchedSapProvider* m_sched{nullptr}; ///< Scheduler-facing SAP of m_fhControl
    std::map<uint16_t, std::unique_ptr<TestFhPhySapUser>> m_phySapUsers;     ///< PHY stubs
    std::map<uint16_t, std::unique_ptr<TestFhSchedSapUser>> m_schedSapUsers; ///< Sched stubs
};

/**
 * @brief A BWP whose only UE had a HARQ retransmission must stop counting as
 *        active once that retransmission is scheduled, so that the other BWP
 *        gets the whole FH capacity again.
 */
class NrFhControlHarqUeReleaseTestCase : public NrFhControlTestCase
{
  public:
    NrFhControlHarqUeReleaseTestCase()
        : NrFhControlTestCase("HARQ UE is released after its retransmission")
    {
    }

  private:
    void DoRun() override
    {
        Setup(100, NrFhControl::FS_7_2, {1, 1});

        // BWP 0: UE 1 with new data. BWP 1: UE 2 waiting for a retransmission.
        m_sched->SetActiveUe(0, 1, 1000);
        m_sched->SetActiveHarqUes(1, 2);

        // FS 7.2: (12 * 8 * 677 + 32 + 10000) bits per 1 ms slot = 75.024 Mbps,
        // above the 50 Mbps share of 2 active BWPs and below the 100 Mbps of 1.
        NS_TEST_ASSERT_MSG_EQ(m_sched->DoesAllocationFit(0, MCS_256QAM, 677, 1, 1),
                              false,
                              "With two active BWPs the allocation should not fit");

        m_sched->UpdateActiveUesMap(1, {MakeDlDataAllocation(2, 0)}, {});

        NS_TEST_ASSERT_MSG_EQ(m_sched->DoesAllocationFit(0, MCS_256QAM, 677, 1, 1),
                              true,
                              "BWP 1 should no longer be active after serving its HARQ UE");
    }
};

/**
 * @brief A UE with both a HARQ retransmission and new data keeps its BWP
 *        active until both are served.
 */
class NrFhControlHarqAndNewDataTestCase : public NrFhControlTestCase
{
  public:
    NrFhControlHarqAndNewDataTestCase()
        : NrFhControlTestCase("BWP stays active until HARQ and new data are served")
    {
    }

  private:
    void DoRun() override
    {
        Setup(100, NrFhControl::FS_7_2, {1, 1});

        // BWP 0: UE 1 with new data. BWP 1: UE 2 with new data and a retransmission.
        m_sched->SetActiveUe(0, 1, 1000);
        m_sched->SetActiveUe(1, 2, 1000);
        m_sched->SetActiveHarqUes(1, 2);

        // Same 75.024 Mbps allocation as in NrFhControlHarqUeReleaseTestCase
        m_sched->UpdateActiveUesMap(1, {MakeDlDataAllocation(2, 0)}, {});
        NS_TEST_ASSERT_MSG_EQ(m_sched->DoesAllocationFit(0, MCS_256QAM, 677, 1, 1),
                              false,
                              "BWP 1 should stay active while UE 2 has new data");

        // A UE without logical channels reports an empty DL buffer
        auto ue2 = std::make_shared<NrMacSchedulerUeInfo>(2, BeamId(), []() { return 1; });
        m_sched->UpdateActiveUesMap(1, {MakeDlDataAllocation(2, 1)}, {{2, ue2}});
        NS_TEST_ASSERT_MSG_EQ(m_sched->DoesAllocationFit(0, MCS_256QAM, 677, 1, 1),
                              true,
                              "BWP 1 should no longer be active after serving UE 2");
    }
};

/**
 * @brief OptimizeRBs and OptimizeMcs must subtract the per-UE overhead from
 *        the FH capacity of the slot.
 */
class NrFhControlOverheadTestCase : public NrFhControlTestCase
{
  public:
    NrFhControlOverheadTestCase()
        : NrFhControlTestCase("Per-UE overhead is subtracted from the FH capacity")
    {
    }

  private:
    void DoRun() override
    {
        Setup(100, NrFhControl::FS_7_2, {1});
        m_sched->SetActiveUe(0, 1, 1000);

        // 100 Mbps * 1 ms = 100000 bits, minus one UE overhead (32 + 10000 + 240)
        // leaves 89728 bits.
        // REGs at modulation order 8: 89728 / (12 * 8) = 934
        NS_TEST_ASSERT_MSG_EQ(m_sched->GetMaxRegAssignable(0, MCS_256QAM, 1, 1),
                              934,
                              "Wrong maximum number of REGs");
        // Modulation order for 1000 REGs: 89728 / (12 * 1000) = 7 -> 64QAM
        NS_TEST_ASSERT_MSG_EQ(+m_sched->GetMaxMcsAssignable(0, 1000, 1, 1),
                              +MAX_MCS_64QAM,
                              "Wrong maximum MCS");

        // With two UEs the overhead is subtracted twice: 100000 - 2 * 10272 = 79456 bits,
        // shared by the two UEs.
        Setup(100, NrFhControl::FS_7_2, {1});
        m_sched->SetActiveUe(0, 1, 1000);
        m_sched->SetActiveUe(0, 2, 1000);
        // REGs at modulation order 8: 79456 / (2 * 12 * 8) = 413
        NS_TEST_ASSERT_MSG_EQ(m_sched->GetMaxRegAssignable(0, MCS_256QAM, 1, 1),
                              413,
                              "Wrong maximum number of REGs with two UEs");
        // Modulation order for 1000 REGs: 79456 / (2 * 12 * 1000) = 3 -> QPSK
        NS_TEST_ASSERT_MSG_EQ(+m_sched->GetMaxMcsAssignable(0, 1000, 1, 1),
                              +MAX_MCS_QPSK,
                              "Wrong maximum MCS with two UEs");
    }
};

/**
 * @brief When every active UE gets the maximum REGs (or MCS) returned by
 *        NrFhControl, all the allocations must fit in the FH capacity.
 */
class NrFhControlHintFitsTestCase : public NrFhControlTestCase
{
  public:
    NrFhControlHintFitsTestCase()
        : NrFhControlTestCase("Allocations sized by the FH hints fit in the FH capacity")
    {
    }

  private:
    /**
     * @brief Create the NrFhControl with numUes active UEs in BWP 0.
     * @param numUes the number of active UEs
     */
    void SetupUes(uint16_t numUes)
    {
        Setup(100, NrFhControl::FS_7_2, {1});
        for (uint16_t rnti = 1; rnti <= numUes; ++rnti)
        {
            m_sched->SetActiveUe(0, rnti, 1000);
        }
    }

    void DoRun() override
    {
        for (uint16_t numUes : {1, 3})
        {
            // One MCS per modulation order of Table 2
            for (uint8_t mcs : {0, 5, 11, 20, 27})
            {
                SetupUes(numUes);
                uint32_t nRegs = m_sched->GetMaxRegAssignable(0, mcs, 1, 1);
                NS_TEST_ASSERT_MSG_GT(nRegs, 0, "No REGs for " << numUes << " UEs");
                for (uint16_t ue = 1; ue <= numUes; ++ue)
                {
                    NS_TEST_ASSERT_MSG_EQ(m_sched->DoesAllocationFit(0, mcs, nRegs, 1, 1),
                                          true,
                                          "UE " << ue << " of " << numUes << " with " << nRegs
                                                << " REGs at MCS " << +mcs << " does not fit");
                }
            }

            // Below QPSK there is no lower MCS to fall back to, so the REGs are chosen to
            // fit at least at QPSK
            for (uint32_t nRegs : {100, 300})
            {
                SetupUes(numUes);
                uint8_t mcs = m_sched->GetMaxMcsAssignable(0, nRegs, 1, 1);
                for (uint16_t ue = 1; ue <= numUes; ++ue)
                {
                    NS_TEST_ASSERT_MSG_EQ(m_sched->DoesAllocationFit(0, mcs, nRegs, 1, 1),
                                          true,
                                          "UE " << ue << " of " << numUes << " with " << nRegs
                                                << " REGs at MCS " << +mcs << " does not fit");
                }
            }
        }
    }
};

/**
 * @brief The FH throughput of an allocation must use the antenna ports of
 *        the BWP that carries it (relevant for FS 7.1).
 */
class NrFhControlAntennaPortsTestCase : public NrFhControlTestCase
{
  public:
    NrFhControlAntennaPortsTestCase()
        : NrFhControlTestCase("FH throughput uses the antenna ports of its own BWP")
    {
    }

  private:
    void DoRun() override
    {
        Setup(100, NrFhControl::FS_7_1, {1, 4});

        // FS 7.1: (12 * 32 * 100 * ports + 2000) bits per 1 ms slot:
        // 40.4 Mbps with 1 port (BWP 0), 155.6 Mbps with 4 ports (BWP 1).
        NS_TEST_ASSERT_MSG_EQ(m_sched->DoesAllocationFit(0, MCS_256QAM, 100, 1, 1),
                              true,
                              "1-port allocation should fit in 100 Mbps");
        NS_TEST_ASSERT_MSG_EQ(m_sched->DoesAllocationFit(1, MCS_256QAM, 100, 1, 1),
                              false,
                              "4-port allocation should not fit in 100 Mbps");
    }
};

/**
 * @brief Asking for the maximum REGs of a BWP that has only HARQ UEs must not
 *        throw.
 */
class NrFhControlHarqOnlyBwpTestCase : public NrFhControlTestCase
{
  public:
    NrFhControlHarqOnlyBwpTestCase()
        : NrFhControlTestCase("Maximum REGs of a BWP with only HARQ UEs")
    {
    }

  private:
    void DoRun() override
    {
        Setup(100, NrFhControl::FS_7_2, {1});
        m_sched->SetActiveHarqUes(0, 1);

        NS_TEST_ASSERT_MSG_EQ(m_sched->GetMaxRegAssignable(0, MCS_256QAM, 1, 1),
                              0,
                              "No UE with new data shares the capacity");
    }
};

} // namespace

/**
 * @ingroup test
 * @brief NrFhControl test suite.
 */
class NrFhControlTestSuite : public TestSuite
{
  public:
    NrFhControlTestSuite()
        : TestSuite("nr-test-fh-control", Type::UNIT)
    {
        AddTestCase(new NrFhControlHarqUeReleaseTestCase(), Duration::QUICK);
        AddTestCase(new NrFhControlHarqAndNewDataTestCase(), Duration::QUICK);
        AddTestCase(new NrFhControlOverheadTestCase(), Duration::QUICK);
        AddTestCase(new NrFhControlHintFitsTestCase(), Duration::QUICK);
        AddTestCase(new NrFhControlAntennaPortsTestCase(), Duration::QUICK);
        AddTestCase(new NrFhControlHarqOnlyBwpTestCase(), Duration::QUICK);
    }
};

static NrFhControlTestSuite g_nrFhControlTestSuite; //!< Static test suite instance
