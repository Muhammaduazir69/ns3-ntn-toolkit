// Copyright (c) 2025 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/config.h"
#include "ns3/nr-mac-sched-sap.h"
#include "ns3/nr-mac-scheduler-lc-alg.h"
#include "ns3/nr-mac-scheduler-lcg.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/object-factory.h"
#include "ns3/test.h"

#include <unordered_map>

/**
 * @file nr-test-sched-lc.cc
 * @ingroup test
 *
 * @brief This file contain tests for the round-robin nature of nr-mac-scheduler-lc-rr.
 * It tests that different logical channels get scheduled the necessary amount of bytes,
 * according to their requirements. And if there are leftover bytes, they are distributed
 * properly, so all bytes in a txop are available for use by LCs.
 *
 */
namespace ns3
{

using LCG = std::unordered_map<uint8_t, LCGPtr>;

class NrTestMacSchedLcRr : public TestCase
{
  public:
    /**
     * @brief Constructor for NrTestMacSchedLcRr.
     *
     * Initializes the test case with the given description, logical channel scheduler type,
     * an allocation map of logical channel groups (LCGs), transport block size (TBS),
     * and the expected assigned bytes for each logical channel group.
     *
     * @param isDl True for downlink, false for uplink
     * @param description The description of the test case.
     * @param lcType The type identifier for the logical channel scheduler implementation tested.
     * @param lcgToAllocate A vector representing the LCGs and their associated byte allocations,
     *                      given as pairs containing the LCG ID and its bytes.
     * @param tbs The transport block size (TBS) to be allocated for scheduling.
     * @param assignedBytes A vector containing expected assigned byte allocations per LCG,
     *                      given as pairs of LCG ID and byte count.
     *
     * The constructor performs the following:
     * - Initializes the base TestCase class with the given description.
     * - Stores the specified transport block size and expected assigned bytes.
     * - Configures the logical channel (LC) scheduler factory with the given lcType.
     * - Constructs logical channel group (LCG) structures and logical channels within each LCG
     * using the provided allocation map.
     * - Updates the LCGs with their respective byte sizes for scheduling purposes.
     */
    NrTestMacSchedLcRr(bool isDl,
                       const std::string description,
                       const std::string lcType,
                       std::vector<std::pair<uint8_t, uint32_t>> lcgToAllocate,
                       uint32_t tbs,
                       std::vector<std::pair<uint8_t, uint32_t>> assignedBytes)
        : TestCase(description),
          m_tbSize(tbs),
          m_expectedAssignedBytes(assignedBytes),
          m_isDl(isDl)
    {
        // Set factory of LC scheduler to tested type
        m_lcFactory.SetTypeId(TypeId::LookupByName(lcType));
        // Build LCG structure from vector of pairs
        for (auto [lcgId, lcgBytes] : lcgToAllocate)
        {
            auto lcgEntry = std::make_unique<NrMacSchedulerLCG>(lcgId);
            m_lcg.emplace(lcgId, std::move(lcgEntry));
            nr::LogicalChannelConfigListElement_s config{.m_fiveQi = 5};
            auto lcEntry = std::make_unique<NrMacSchedulerLC>(config);
            // Limit the test check to DATA logical channels only.
            // CTRL logical channels are handled by priority,
            // and the logical channel assignment logic does not apply to them.
            lcEntry->m_id = lcgId;
            m_lcg.at(lcgId)->Insert(std::move(lcEntry));
            NrMacSchedSapProvider::SchedDlRlcBufferReqParameters params{};
            params.m_logicalChannelIdentity = lcgId;
            params.m_rlcTransmissionQueueSize = lcgBytes;
            m_lcg.at(lcgId)->UpdateInfo(params);
        }
    }

  protected:
    void DoRun() override;
    ObjectFactory m_lcFactory;
    LCG m_lcg;
    uint32_t m_tbSize;
    std::vector<std::pair<uint8_t, uint32_t>> m_expectedAssignedBytes;
    bool m_isDl;
};

void
NrTestMacSchedLcRr::DoRun()
{
    auto schedLc = DynamicCast<NrMacSchedulerLcAlgorithm>(m_lcFactory.Create());

    // Call DL/UL assign bytes functions
    std::vector<NrMacSchedulerLcAlgorithm::Assignation> assignedBytes;
    if (m_isDl)
    {
        assignedBytes = schedLc->AssignBytesToDlLC(m_lcg, m_tbSize, MilliSeconds(0));
    }
    else
    {
        assignedBytes = schedLc->AssignBytesToUlLC(m_lcg, m_tbSize);
    }

    // Check if both allocate the expected number of bytes per LCG
    uint32_t totalAssignedBytes = 0;
    for (auto [lcgId, lcgBytes] : m_expectedAssignedBytes)
    {
        auto lcgAssigned = std::find_if(assignedBytes.begin(),
                                        assignedBytes.end(),
                                        [lcg = lcgId](auto& entry) { return entry.m_lcg == lcg; });
        if (lcgAssigned != assignedBytes.end())
        {
            NS_TEST_ASSERT_MSG_EQ(lcgAssigned->m_bytes,
                                  lcgBytes,
                                  "Expected " << lcgBytes << " bytes assigned for LCG " << +lcgId);
            totalAssignedBytes += lcgAssigned->m_bytes;
        }
        else if (lcgBytes != 0)
        {
            NS_TEST_ASSERT_MSG_EQ(false, true, "Expected LCG " << +lcgId << " to be assigned");
        }
    }

    uint32_t expectedTotalAssignedBytes = 0;
    for (const auto& [lcgId, lcgBytes] : m_expectedAssignedBytes)
    {
        expectedTotalAssignedBytes += lcgBytes;
    }
    NS_TEST_ASSERT_MSG_EQ(totalAssignedBytes,
                          expectedTotalAssignedBytes,
                          "Expected " << expectedTotalAssignedBytes
                                      << " bytes to be assigned in total");

    if (m_tbSize == 0)
    {
        NS_TEST_ASSERT_MSG_EQ(totalAssignedBytes, 0, "Expected no LCGs to be assigned");
    }
}

class NrTestSchedLcSuite : public TestSuite
{
  public:
    NrTestSchedLcSuite()
        : TestSuite("nr-test-sched-lc", Type::UNIT)
    {
        bool isDl = true;
        // clang-format off
        // The LC RR scheduler refuses to split a TB into sub-PDUs smaller
        // than 10 bytes (3 MAC subheader + 7 RLC AM data PDU minimum), and
        // it leaves SRB allocation to AssignControlBytes(). Small TBs are
        // deferred to a later slot, and the expected per-LCG byte counts
        // below reflect that round-up: each served data LC gets at least
        // 10 bytes per round before the remainder is poured into the LC
        // that still has data to send.
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (), tbs = 0",                   "ns3::NrMacSchedulerLcRR",{},                        0, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (), tbs = 1",                   "ns3::NrMacSchedulerLcRR",{},                        1, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:0B), tbs = 0",               "ns3::NrMacSchedulerLcRR",{{1,0}},                   0, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B), tbs = 0",               "ns3::NrMacSchedulerLcRR",{{1,1}},                   0, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B), tbs = 1",               "ns3::NrMacSchedulerLcRR",{{1,1}},                   1, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B), tbs = 2",               "ns3::NrMacSchedulerLcRR",{{1,1}},                   2, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B,2:1B), tbs = 0",          "ns3::NrMacSchedulerLcRR",{{1,1}, {2,1}},            0, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B,2:1B), tbs = 1",          "ns3::NrMacSchedulerLcRR",{{1,1}, {2,1}},            1, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B,2:1B), tbs = 2",          "ns3::NrMacSchedulerLcRR",{{1,1}, {2,1}},            2, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B,2:1B), tbs = 3",          "ns3::NrMacSchedulerLcRR",{{1,1}, {2,1}},            3, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B,2:1B), tbs = 4",          "ns3::NrMacSchedulerLcRR",{{1,1}, {2,1}},            4, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B,2:2B,3:3B), tbs = 0",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     0, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B,2:2B,3:3B), tbs = 1",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     1, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B,2:2B,3:3B), tbs = 2",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     2, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B,2:2B,3:3B), tbs = 3",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     3, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B,2:2B,3:3B), tbs = 4",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     4, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B,2:2B,3:3B), tbs = 5",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     5, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B,2:2B,3:3B), tbs = 6",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     6, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B,2:2B,3:3B), tbs = 7",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     7, {{1,7}}                ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B,2:2B,3:3B), tbs = 8",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     8, {{1,8}}                ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B,2:2B,3:3B), tbs = 9",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     9, {{1,8}}                ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B,2:2B,3:300B), tbs = 300", "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,300}}, 300, {{1,8}, {2,10}, {3,282}}), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B,2:2B,3:297B), tbs = 309", "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,297}}, 309, {{1,8}, {2,10}, {3,291}}), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr(!isDl, "LcRR UL flows (1:1B,2:2B,3:300B), tbs = 315", "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,300}}, 315, {{1,8}, {2,10}, {3,297}}), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (), tbs = 0",                   "ns3::NrMacSchedulerLcRR",{},                        0, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (), tbs = 1",                   "ns3::NrMacSchedulerLcRR",{},                        1, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:0B), tbs = 0",               "ns3::NrMacSchedulerLcRR",{{1,0}},                   0, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B), tbs = 0",               "ns3::NrMacSchedulerLcRR",{{1,1}},                   0, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B), tbs = 1",               "ns3::NrMacSchedulerLcRR",{{1,1}},                   1, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B), tbs = 2",               "ns3::NrMacSchedulerLcRR",{{1,1}},                   2, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B,2:1B), tbs = 0",          "ns3::NrMacSchedulerLcRR",{{1,1}, {2,1}},            0, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B,2:1B), tbs = 1",          "ns3::NrMacSchedulerLcRR",{{1,1}, {2,1}},            1, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B,2:1B), tbs = 2",          "ns3::NrMacSchedulerLcRR",{{1,1}, {2,1}},            2, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B,2:1B), tbs = 3",          "ns3::NrMacSchedulerLcRR",{{1,1}, {2,1}},            3, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B,2:1B), tbs = 4",          "ns3::NrMacSchedulerLcRR",{{1,1}, {2,1}},            4, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B,2:2B,3:3B), tbs = 0",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     0, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B,2:2B,3:3B), tbs = 1",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     1, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B,2:2B,3:3B), tbs = 2",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     2, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B,2:2B,3:3B), tbs = 3",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     3, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B,2:2B,3:3B), tbs = 4",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     4, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B,2:2B,3:3B), tbs = 5",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     5, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B,2:2B,3:3B), tbs = 6",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     6, {}                     ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B,2:2B,3:3B), tbs = 7",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     7, {{1,7}}                ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B,2:2B,3:3B), tbs = 8",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     8, {{1,8}}                ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B,2:2B,3:3B), tbs = 9",     "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,3}},     9, {{1,8}}                ), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B,2:2B,3:300B), tbs = 300", "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,300}}, 300, {{1,8}, {2,10}, {3,282}}), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B,2:2B,3:297B), tbs = 309", "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,297}}, 309, {{1,8}, {2,10}, {3,291}}), Duration::QUICK);
        AddTestCase(new NrTestMacSchedLcRr( isDl, "LcRR DL flows (1:1B,2:2B,3:300B), tbs = 313", "ns3::NrMacSchedulerLcRR",{{1,1}, {2,2}, {3,300}}, 313, {{1,8}, {2,10}, {3,295}}), Duration::QUICK);
        // clang-format on
    }
};

static NrTestSchedLcSuite nrSchedLcTestSuite; //!< NR LC scheduler test suite

} // namespace ns3
