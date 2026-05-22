/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/fapi-helpers.h"
#include "ns3/fapi-messages.h"
#include "ns3/fapi-pdu-types.h"
#include "ns3/test.h"

#include <variant>

using namespace ns3;
using namespace ns3::fapi;

class FapiMessageIdsStableTest : public TestCase
{
  public:
    FapiMessageIdsStableTest()
        : TestCase("FAPI message IDs and string names are SCF 222.10.02 and .04 stable")
    {
    }

  private:
    void DoRun() override
    {
        // SCF FAPI 222.10.02 + 222.10.04 message-type codes; FlexRIC / Aerial
        // depend on these byte values.
        NS_TEST_EXPECT_MSG_EQ(static_cast<uint16_t>(kDlTtiRequest),
                              0x0080u,
                              "DL_TTI.request code");
        NS_TEST_EXPECT_MSG_EQ(static_cast<uint16_t>(kUlTtiRequest),
                              0x0081u,
                              "UL_TTI.request code");
        NS_TEST_EXPECT_MSG_EQ(static_cast<uint16_t>(kSlotIndication),
                              0x0082u,
                              "SLOT.indication code");
        NS_TEST_EXPECT_MSG_EQ(static_cast<uint16_t>(kRachIndication),
                              0x0089u,
                              "RACH.indication code");
        NS_TEST_EXPECT_MSG_EQ(std::string(MessageIdName(kDlTtiRequest)),
                              "DL_TTI.request",
                              "DL_TTI label");
        NS_TEST_EXPECT_MSG_EQ(std::string(MessageIdName(kCrcIndication)),
                              "CRC.indication",
                              "CRC label");
    }
};

class FapiNumerologyTest : public TestCase
{
  public:
    FapiNumerologyTest()
        : TestCase("Numerology -> SCS kHz matches TS 38.211 Table 4.2-1")
    {
    }

  private:
    void DoRun() override
    {
        NS_TEST_EXPECT_MSG_EQ(SubCarrierSpacingKhz(Numerology::kMu0_15kHz),
                              15.0,
                              "mu=0");
        NS_TEST_EXPECT_MSG_EQ(SubCarrierSpacingKhz(Numerology::kMu1_30kHz),
                              30.0,
                              "mu=1");
        NS_TEST_EXPECT_MSG_EQ(SubCarrierSpacingKhz(Numerology::kMu2_60kHz),
                              60.0,
                              "mu=2");
        NS_TEST_EXPECT_MSG_EQ(SubCarrierSpacingKhz(Numerology::kMu3_120kHz),
                              120.0,
                              "mu=3");
        NS_TEST_EXPECT_MSG_EQ(SubCarrierSpacingKhz(Numerology::kMu4_240kHz),
                              240.0,
                              "mu=4");
    }
};

class FapiDmrsRoundTripTest : public TestCase
{
  public:
    FapiDmrsRoundTripTest()
        : TestCase("DMRS bitmap <-> symbol-index list round-trips")
    {
    }

  private:
    void DoRun() override
    {
        // Typical Type1 DMRS at symbols 2 and 11 in a 14-symbol slot.
        const uint16_t mask = (uint16_t{1} << 2) | (uint16_t{1} << 11);
        const auto idx = DmrsFapiToBitArray(mask);
        NS_TEST_ASSERT_MSG_EQ(idx.size(), 2u, "two symbols");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(idx[0]), 2, "symbol 2");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(idx[1]), 11, "symbol 11");

        const uint16_t back = DmrsBitArrayToFapi(idx);
        NS_TEST_EXPECT_MSG_EQ(back, mask, "round-trip mask");

        // Empty bitmap -> empty list.
        NS_TEST_EXPECT_MSG_EQ(DmrsFapiToBitArray(0).size(),
                              0u,
                              "empty mask");

        // Out-of-range indices in input are silently dropped (lenient).
        const uint16_t back2 =
            DmrsBitArrayToFapi(std::vector<uint8_t>{0, 13, 14, 250});
        NS_TEST_EXPECT_MSG_EQ(back2,
                              static_cast<uint16_t>((1U << 0) | (1U << 13)),
                              "drop oversized indices");
    }
};

class FapiDlTtiAssemblyTest : public TestCase
{
  public:
    FapiDlTtiAssemblyTest()
        : TestCase("DL_TTI.request can carry PDCCH + PDSCH PDUs in order")
    {
    }

  private:
    void DoRun() override
    {
        DlTtiRequest req{};
        req.sfn = 100;
        req.slot = 7;
        req.nPdusOfEachType[0] = 1; // PDCCH
        req.nPdusOfEachType[1] = 1; // PDSCH

        PdcchPdu pdcch{};
        pdcch.startSymbolIndex = 0;
        pdcch.durationSymbols = 1;
        PdcchPdu::Dci dci{};
        dci.rnti = 0xC001;
        dci.aggregationLevel = 4;
        dci.cceIndex = 0;
        pdcch.dciList.push_back(dci);
        DlTtiPdu w1{};
        w1.type = DlTtiPdu::Type::kPdcch;
        w1.pdu = pdcch;
        req.pduList.push_back(w1);

        PdschPdu pdsch{};
        pdsch.rnti = 0xC001;
        pdsch.nrOfLayers = 1;
        pdsch.codewords[0].mcsIndex = 11;
        pdsch.codewords[0].tbSize = 1024;
        pdsch.dmrs.dmrsSymbPos = (uint16_t{1} << 2);
        DlTtiPdu w2{};
        w2.type = DlTtiPdu::Type::kPdsch;
        w2.pdu = pdsch;
        req.pduList.push_back(w2);

        NS_TEST_ASSERT_MSG_EQ(req.pduList.size(), 2u, "two PDUs");
        NS_TEST_EXPECT_MSG_EQ(static_cast<uint16_t>(DlTtiRequest::kId),
                              static_cast<uint16_t>(kDlTtiRequest),
                              "kId tag matches enum");

        // Round-trip the embedded RNTI via std::get on the variant.
        const auto& got1 = std::get<PdcchPdu>(req.pduList[0].pdu);
        NS_TEST_EXPECT_MSG_EQ(got1.dciList[0].rnti, 0xC001u, "PDCCH RNTI");
        const auto& got2 = std::get<PdschPdu>(req.pduList[1].pdu);
        NS_TEST_EXPECT_MSG_EQ(got2.codewords[0].tbSize,
                              1024u,
                              "PDSCH TB size");

        // DMRS bitmap inside the PDSCH PDU expands cleanly.
        const auto sym = DmrsFapiToBitArray(got2.dmrs.dmrsSymbPos);
        NS_TEST_ASSERT_MSG_EQ(sym.size(), 1u, "one DMRS symbol");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(sym[0]), 2, "DMRS at sym 2");
    }
};

class FapiUlIndicationShapesTest : public TestCase
{
  public:
    FapiUlIndicationShapesTest()
        : TestCase("UL indications carry CRC, SRS, RACH report shapes")
    {
    }

  private:
    void DoRun() override
    {
        CrcIndication crc{};
        crc.sfn = 1;
        crc.slot = 2;
        CrcIndication::CrcReport rep{};
        rep.tbCrcStatusOk = true;
        rep.cbCrcStatusOk = {true, true, false};
        rep.ul_cqi = -3;
        rep.timingAdvance = 42;
        crc.crcList.push_back(rep);
        NS_TEST_EXPECT_MSG_EQ(crc.crcList[0].cbCrcStatusOk.size(),
                              3u,
                              "3 CBGs");
        NS_TEST_EXPECT_MSG_EQ(crc.crcList[0].cbCrcStatusOk[2],
                              false,
                              "CBG[2] failed");

        SrsIndication srs{};
        SrsIndication::Report sr{};
        sr.numSymbols = 4;
        sr.numRbs = 32;
        sr.wbSnrPerRbDb.assign(32, 18);
        srs.reports.push_back(sr);
        NS_TEST_EXPECT_MSG_EQ(srs.reports[0].wbSnrPerRbDb.size(),
                              32u,
                              "per-RB SNR length");

        RachIndication rach{};
        RachIndication::Preamble p{};
        p.preambleIndex = 5;
        p.timingAdvance = 200;
        rach.preambles.push_back(p);
        NS_TEST_EXPECT_MSG_EQ(rach.preambles[0].preambleIndex,
                              5u,
                              "preamble idx");
    }
};

class NtnFapiTestSuite : public TestSuite
{
  public:
    NtnFapiTestSuite()
        : TestSuite("ntn-fapi", Type::UNIT)
    {
        AddTestCase(new FapiMessageIdsStableTest, TestCase::Duration::QUICK);
        AddTestCase(new FapiNumerologyTest, TestCase::Duration::QUICK);
        AddTestCase(new FapiDmrsRoundTripTest, TestCase::Duration::QUICK);
        AddTestCase(new FapiDlTtiAssemblyTest, TestCase::Duration::QUICK);
        AddTestCase(new FapiUlIndicationShapesTest, TestCase::Duration::QUICK);
    }
};

static NtnFapiTestSuite g_ntnFapiTestSuite;
