/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/config.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/fapi-helpers.h"
#include "ns3/fapi-messages.h"
#include "ns3/fapi-pdu-types.h"
#include "ns3/mmwave-enb-net-device.h"
#include "ns3/mmwave-phy-mac-common.h"
#include "ns3/node-container.h"
#include "ns3/ntn-fapi-sap-bridge.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-static-extra-loss-model.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/vector.h"

#include <array>
#include <cmath>
#include <map>
#include <variant>

using namespace ns3;
using namespace ns3::fapi;

/// FAPI-4: a slot that schedules several UEs must give each its own HARQ id.
///
/// The HARQ process id was stored once per SLOT, taken from whichever DL DCI
/// came last in the TTI loop, and read back for every CRC.indication in that
/// slot. With more than one UE scheduled, every CRC carried the last UE's pid.
/// And because the per-slot entry was erased on the first match, a second
/// transport block in the same slot found nothing and reported pid 0.
///
/// The shipped examples default to four UEs; the only test scheduled one, which
/// is exactly why the key survived.
class FapiHarqIdIsPerUeTest : public TestCase
{
  public:
    FapiHarqIdIsPerUeTest()
        : TestCase("FAPI-4: CRC.indication carries each UE's own HARQ process id")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<fapi::NtnFapiSapBridge> bridge = CreateObject<fapi::NtnFapiSapBridge>();

        // Two UEs scheduled in ONE slot, with different HARQ processes.
        const uint32_t frame = 3;
        const uint8_t sf = 4;
        const uint8_t slot = 1;
        bridge->RecordDlHarqForTest(frame, sf, slot, /*rnti=*/11, /*pid=*/5);
        bridge->RecordDlHarqForTest(frame, sf, slot, /*rnti=*/22, /*pid=*/9);

        std::vector<std::pair<uint16_t, uint16_t>> seen; // (rnti, harqId)
        bridge->TraceConnectWithoutContext(
            "CrcIndication",
            MakeCallback(&FapiHarqIdIsPerUeTest::OnCrc, this));
        m_seen = &seen;

        auto rx = [&](uint16_t rnti) {
            mmwave::RxPacketTraceParams p{};
            p.m_frameNum = frame;
            p.m_sfNum = sf;
            p.m_slotNum = slot;
            p.m_rnti = rnti;
            p.m_tbSize = 100;
            p.m_corrupt = false;
            p.m_sinr = 100.0;
            bridge->OnPhyRx(p);
        };

        // Deliberately deliver the SECOND-scheduled UE first. Under the old
        // per-slot key the first arrival erased the entry, so whichever came
        // second got pid 0 regardless of order.
        rx(22);
        rx(11);

        NS_TEST_ASSERT_MSG_EQ(seen.size(), 2u,
                              "both transport blocks must produce a CRC.indication; the per-slot "
                              "entry used to be erased on the first match, so the second found "
                              "nothing");

        uint16_t harqFor11 = 0xffff;
        uint16_t harqFor22 = 0xffff;
        for (const auto& [rnti, hid] : seen)
        {
            if (rnti == 11)
            {
                harqFor11 = hid;
            }
            if (rnti == 22)
            {
                harqFor22 = hid;
            }
        }
        NS_TEST_ASSERT_MSG_EQ(harqFor11, 5,
                              "UE 11 was scheduled on HARQ process 5 and must be told so");
        NS_TEST_ASSERT_MSG_EQ(harqFor22, 9,
                              "UE 22 was scheduled on HARQ process 9; reporting the other UE's "
                              "id is the defect");
        NS_TEST_ASSERT_MSG_NE(harqFor11, harqFor22,
                              "and the two must differ, or one slot-wide value is still being "
                              "handed to every UE");

        // P0-9: with no RNTI filter every TB of the cell is accounted, per RNTI
        // and in total (SCF 222.10: one CRC report per TB).
        const auto& per = bridge->GetPerRntiStats();
        NS_TEST_ASSERT_MSG_EQ(per.size(), 2u, "both RNTIs accounted");
        NS_TEST_ASSERT_MSG_EQ(per.at(11).crc, 1u, "UE 11: one CRC.indication");
        NS_TEST_ASSERT_MSG_EQ(per.at(22).rxBytes, 100u, "UE 22: its TB bytes in RX_DATA");
        NS_TEST_ASSERT_MSG_EQ(bridge->GetRxDataBytes(), 200u, "cell RX_DATA bytes = sum of TBs");
    }

    void OnCrc(fapi::CrcIndication ind)
    {
        if (m_seen)
        {
            for (const auto& c : ind.crcList)
            {
                m_seen->emplace_back(c.rnti, c.harqId);
            }
        }
    }

    std::vector<std::pair<uint16_t, uint16_t>>* m_seen{nullptr};
};

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

namespace
{
/// Global handle so a plain Config trace callback can feed the real per-TB
/// decode into the bridge (mirrors the example wiring).
Ptr<NtnFapiSapBridge> g_testBridge = nullptr;
NtnRealStackHelper* g_testRs = nullptr;
uint16_t g_testUeRnti = 0;

void
TestFapiRx(mmwave::RxPacketTraceParams p)
{
    if (g_testUeRnti == 0 && g_testRs)
    {
        g_testUeRnti = g_testRs->GetUeRnti(0);
        if (g_testUeRnti != 0 && g_testBridge)
        {
            g_testBridge->SetUeRnti(g_testUeRnti);
        }
    }
    if (g_testBridge)
    {
        g_testBridge->OnPhyRx(p);
    }
}
} // namespace

/// E2E: the FAPI SAP bridge decorates a LIVE mmwave enb MAC<->PHY SAP. Over a
/// real Simulator::Run(): SLOT.indications fire at the real slot cadence, at
/// least one DL_TTI.request is produced from a real DL allocation, and the
/// request->indication latency is finite and SFN/slot aligned (CI gate 15).
class FapiRealSapBridgeTest : public TestCase
{
  public:
    FapiRealSapBridgeTest()
        : TestCase("FAPI SAP bridge drives real mmwave slot timing; gate-15 latency finite")
    {
    }

  private:
    void DoRun() override
    {
        const double simTime = 3.0;

        // Minimal real geometry: one LEO gNB 600 km straight above a static
        // ground UE (same rig the ntn-traffic real-stack test uses).
        NodeContainer sat;
        sat.Create(1);
        Ptr<ConstantVelocityMobilityModel> satMob = CreateObject<ConstantVelocityMobilityModel>();
        satMob->SetPosition(Vector(0.0, 0.0, 600e3));
        satMob->SetVelocity(Vector(7560.0, 0.0, 0.0));
        sat.Get(0)->AggregateObject(satMob);

        NodeContainer ue;
        ue.Create(1);
        Ptr<ConstantPositionMobilityModel> ueMob = CreateObject<ConstantPositionMobilityModel>();
        ueMob->SetPosition(Vector(0.0, 0.0, 0.0));
        ue.Get(0)->AggregateObject(ueMob);

        NtnRealStackHelper rs;
        rs.SetSimTime(Seconds(simTime));
        rs.Build(sat, ue); // mmwave backend (default): real SpectrumPhy+MAC+HARQ
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                          Seconds(0.5), Seconds(simTime - 0.2));
        g_testRs = &rs;

        // Install the real FAPI SAP bridge on the live mmwave enb cell.
        Ptr<mmwave::MmWaveEnbNetDevice> enb =
            DynamicCast<mmwave::MmWaveEnbNetDevice>(rs.GetEnbDevices().Get(0));
        NS_TEST_ASSERT_MSG_NE((enb == nullptr), true, "mmwave enb device present");
        g_testBridge = CreateObject<NtnFapiSapBridge>();
        g_testBridge->InstallEnb(enb);

        // P5: the CONFIG.request was consumed from the real PHY config.
        NS_TEST_ASSERT_MSG_EQ(g_testBridge->IsConfigured(), true,
                              "P5 CONFIG.request consumed at bring-up");
        NS_TEST_ASSERT_MSG_GT(g_testBridge->GetCarrierConfig().dlFrequency, 0u,
                              "CONFIG.request carries a real DL carrier frequency");

        // Reverse FAPI path off the real per-TB decode trace.
        Config::ConnectWithoutContextFailSafe(
            "/NodeList/*/DeviceList/*/ComponentCarrierMap/*/MmWaveUePhy/DlSpectrumPhy/"
            "RxPacketTraceUe",
            MakeCallback(&TestFapiRx));

        Simulator::Stop(Seconds(simTime));
        Simulator::Run();

        const uint64_t slotInds = g_testBridge->GetSlotIndicationCount();
        const uint64_t dlTtiData = g_testBridge->GetDlTtiWithDataCount();
        const uint64_t crcInds = g_testBridge->GetCrcIndicationCount();
        const uint64_t matched = g_testBridge->GetMatchedLatencyCount();
        const double meanLat = g_testBridge->GetMeanSapLatencySec();
        const double lastLat = g_testBridge->GetLastSapLatencySec();

        // 1. SLOT.indications fire at the real slot cadence (many times, not once).
        NS_TEST_ASSERT_MSG_GT(slotInds, 100u,
                              "FAPI SLOT.indication fired at the real mmwave slot cadence");

        // 2. At least one DL_TTI.request produced from a real DL allocation.
        NS_TEST_ASSERT_MSG_GT(dlTtiData, 0u,
                              "at least one DL_TTI.request built from a real DL allocation");

        // 3. Reverse path really emitted CRC.indications from the real decode.
        NS_TEST_ASSERT_MSG_GT(crcInds, 0u, "CRC.indication emitted from real per-TB decode");

        // 4. request->indication latency finite and SFN/slot aligned (gate 15).
        NS_TEST_ASSERT_MSG_GT(matched, 0u,
                              "at least one SFN/slot-aligned request->indication match");
        NS_TEST_ASSERT_MSG_EQ(g_testBridge->HasSlotAlignedLatency(), true,
                              "gate 15: a slot-aligned SAP latency exists");
        NS_TEST_ASSERT_MSG_EQ(std::isfinite(meanLat), true, "gate 15: mean SAP latency finite");
        NS_TEST_ASSERT_MSG_EQ(std::isfinite(lastLat), true, "gate 15: last SAP latency finite");
        NS_TEST_ASSERT_MSG_GT(lastLat, 0.0,
                              "gate 15: request->indication latency strictly positive");
        // FAPI-2: the gate has to know WHAT it measured.
        //
        // The old bound here was `meanLat < 1.0` s, which a channel with no
        // propagation delay at all passes trivially - and that is exactly the
        // channel this backend has. The resulting figure was published as the
        // NTN L1/L2 turnaround while containing no part of the NTN round trip.
        // The bound is now anchored to the geometry and is two-sided, so it
        // fails whichever way the truth and the label disagree.
        const bool airProp = g_testBridge->HasAirPropagationDelay();
        const double floorSec = g_testBridge->GetOneWayPropagationFloorSec();
        NS_TEST_ASSERT_MSG_GT(floorSec, 0.0,
                              "the bridge must recover the real gNB-UE separation; without it "
                              "there is nothing to anchor the latency bound to");
        // 600 km straight up is 2.0018 ms one way.
        NS_TEST_ASSERT_MSG_EQ_TOL(floorSec, 600e3 / 299792458.0, 1e-6,
                                  "the floor comes from the real geometry of this scenario");

        if (airProp)
        {
            // If the channel does carry propagation, a request->indication
            // turnaround must contain at least the one-way flight.
            NS_TEST_ASSERT_MSG_GT(meanLat, floorSec,
                                  "a turnaround measured over a channel WITH propagation delay "
                                  "cannot be shorter than the one-way flight time");
        }
        else
        {
            // If it does not, the number must be BELOW the floor. That is not
            // a weaker check: it is the assertion that the published figure is
            // the slot pipeline and nothing else. Should someone later switch
            // this backend to a delay-carrying channel without relabeling the
            // KPI, the branch above starts applying and this one stops.
            NS_TEST_ASSERT_MSG_LT(meanLat, floorSec,
                                  "this backend has no air propagation, so the SAP latency must "
                                  "sit below the one-way floor; a value above it would mean the "
                                  "number silently acquired a delay the label does not admit");
        }
        NS_TEST_ASSERT_MSG_EQ(std::string(g_testBridge->GetSapLatencyProvenance()),
                              airProp ? "measured-with-air-propagation"
                                      : "measured-no-air-propagation",
                              "the provenance label tracks the probed channel, so a CSV row "
                              "carries the qualification with the number");
        NS_TEST_ASSERT_MSG_LT(meanLat, 1.0, "SAP latency within a physical horizon (<1 s)");

        // 5. FAPI-1: TX_DATA.request carries the real transport blocks. The
        // bridge used to emit only descriptors; the bytes themselves had no
        // FAPI representation, so a translator to a real PHY would have had
        // nothing to send.
        NS_TEST_ASSERT_MSG_GT(g_testBridge->GetTxDataRequestCount(), 0u,
                              "TX_DATA.request emitted from the real MAC SendMacPdu SAP call");
        NS_TEST_ASSERT_MSG_GT(g_testBridge->GetTxDataBytes(), 100000u,
                              "TX_DATA.request carries real PDU bytes in bulk, not a token PDU: "
                              "an eMBB stream over 3 s moves hundreds of kilobytes");
        // ns-3 assertions can be configured to continue past a failure, so the
        // container is checked before it is indexed: a missing emitter must
        // produce a clean FAIL, never a dereference of an empty vector.
        const TxDataRequest& lastTx = g_testBridge->LastTxData();
        NS_TEST_ASSERT_MSG_EQ(lastTx.pdus.empty(), false,
                              "the last TX_DATA.request has a payload PDU");
        if (!lastTx.pdus.empty())
        {
            NS_TEST_ASSERT_MSG_GT(lastTx.pdus.front().tbBytes.size(), 0u,
                                  "TX_DATA payload is the real PDU copied out of the packet");
        }

        // 6. FAPI-1: RACH.indication fires on the real initial access. The
        // decorator has always sat on ReceiveRachPreamble; it just forwarded.
        NS_TEST_ASSERT_MSG_GT(g_testBridge->GetRachIndicationCount(), 0u,
                              "RACH.indication emitted from the real PHY preamble reception; "
                              "a UE that attached must have sent one");
        NS_TEST_ASSERT_MSG_EQ(g_testBridge->LastRachIndication().preambles.empty(), false,
                              "RACH.indication carries a preamble record");

        // The UL_TTI counter must advance with the slots even when this
        // downlink-only profile grants nothing, which is what distinguishes
        // "no uplink was scheduled" from "the emitter is not wired".
        NS_TEST_ASSERT_MSG_GT(g_testBridge->GetUlTtiRequestCount(), 100u,
                              "the UL_TTI path is evaluated on every real slot allocation");

        Simulator::Destroy();
        g_testBridge = nullptr;
        g_testRs = nullptr;
        g_testUeRnti = 0;
    }
};


/// FAPI-1: the uplink grant path. The downlink-only profile above exercises
/// TX_DATA and RACH but never produces a UL DCI, so UL_TTI.request would stay
/// at zero grants no matter how it were wired. This runs the same rig with
/// uplink traffic enabled and asserts the grants appear and carry the real
/// scheduler decision.
class FapiUlTtiFromRealGrantsTest : public TestCase
{
  public:
    FapiUlTtiFromRealGrantsTest()
        : TestCase("FAPI-1: UL_TTI.request built from real mmwave uplink grants")
    {
    }

  private:
    void DoRun() override
    {
        const double simTime = 3.0;

        NodeContainer sat;
        sat.Create(1);
        Ptr<ConstantVelocityMobilityModel> satMob = CreateObject<ConstantVelocityMobilityModel>();
        satMob->SetPosition(Vector(0.0, 0.0, 600e3));
        satMob->SetVelocity(Vector(7560.0, 0.0, 0.0));
        sat.Get(0)->AggregateObject(satMob);

        NodeContainer ue;
        ue.Create(1);
        Ptr<ConstantPositionMobilityModel> ueMob = CreateObject<ConstantPositionMobilityModel>();
        ueMob->SetPosition(Vector(0.0, 0.0, 0.0));
        ue.Get(0)->AggregateObject(ueMob);

        NtnRealStackHelper rs;
        rs.SetSimTime(Seconds(simTime));
        rs.SetUplink(true); // the difference from the test above
        rs.Build(sat, ue);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                          Seconds(0.5), Seconds(simTime - 0.2));

        Ptr<mmwave::MmWaveEnbNetDevice> enb =
            DynamicCast<mmwave::MmWaveEnbNetDevice>(rs.GetEnbDevices().Get(0));
        NS_TEST_ASSERT_MSG_NE((enb == nullptr), true, "mmwave enb device present");
        Ptr<NtnFapiSapBridge> bridge = CreateObject<NtnFapiSapBridge>();
        bridge->InstallEnb(enb);

        Simulator::Stop(Seconds(simTime));
        Simulator::Run();

        const uint64_t grants = bridge->GetUlTtiWithGrantCount();
        NS_TEST_ASSERT_MSG_GT(grants, 0u,
                              "with uplink traffic the real scheduler issues UL DCIs, so "
                              "UL_TTI.request must carry grants; zero here means the UL branch "
                              "of the slot allocation is not being read");

        const UlTtiRequest& last = bridge->LastUlTti();
        NS_TEST_ASSERT_MSG_EQ(last.pduList.empty(), false, "the last UL_TTI carries a PDU");
        if (!last.pduList.empty())
        {
            NS_TEST_ASSERT_MSG_EQ(last.nPdusOfEachType[1], last.pduList.size(),
                                  "the PUSCH count must agree with the PDU list length");
            NS_TEST_ASSERT_MSG_EQ(static_cast<int>(last.pduList.front().type),
                                  static_cast<int>(UlTtiPdu::Type::kPusch),
                                  "an uplink data grant becomes a PUSCH PDU");

            // The PDU must carry the scheduler's real numbers, not defaults. A
            // zero TB size or a zero RNTI would mean the DCI was not read.
            const PuschPdu* pusch = std::get_if<PuschPdu>(&last.pduList.front().pdu);
            NS_TEST_ASSERT_MSG_NE((pusch == nullptr), true, "the UL PDU is a PUSCH");
            if (pusch)
            {
                NS_TEST_ASSERT_MSG_GT(pusch->tbSize, 0u,
                                      "the PUSCH PDU carries the real granted TB size");
                NS_TEST_ASSERT_MSG_GT(pusch->rnti, 0u, "the PUSCH PDU carries the real RNTI");
                NS_TEST_ASSERT_MSG_GT(pusch->nrOfSymbols, 0u,
                                      "the PUSCH PDU carries the real symbol allocation");
                NS_TEST_ASSERT_MSG_LT(pusch->harqProcessId, kMaxHarqProcessesRel17,
                                      "the PUSCH HARQ process id is within the Rel-17 range");
            }
        }

        Simulator::Destroy();
    }
};

/// FAPI-1 / RRC-4: the timing-advance field of RACH.indication is the RESIDUAL
/// the gNB measures, not the round trip. TS 38.213 section 4.2.
class FapiRachTimingAdvanceTest : public TestCase
{
  public:
    FapiRachTimingAdvanceTest()
        : TestCase("FAPI-1: RACH.indication timing advance is the residual, in T_C units")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<NtnFapiSapBridge> b = CreateObject<NtnFapiSapBridge>();

        // 600 km straight up: one way 2.0018 ms, round trip 4.0036 ms.
        const Time rtt = NanoSeconds(4003600);

        // Un-compensated: the whole round trip is residual.
        b->SetNtnTimingAdvance(rtt);
        NS_TEST_ASSERT_MSG_EQ_TOL(b->GetResidualTimingAdvance().GetSeconds(), rtt.GetSeconds(),
                                  1e-12, "with no pre-compensation the residual is the round trip");
        // T_C = 1/(480e3*4096) s. 4.0036 ms / T_C = 7,870,193 units.
        const uint32_t expectTc =
            static_cast<uint32_t>(std::llround(rtt.GetSeconds() * 480e3 * 4096.0));
        NS_TEST_ASSERT_MSG_EQ(b->GetResidualTaInTc(), expectTc,
                              "the residual is reported in the T_C unit of TS 38.211 section 4.1");
        NS_TEST_ASSERT_MSG_GT(expectTc, 1000000u,
                              "an NTN round trip is millions of T_C, which is the point: it "
                              "cannot fit the 12-bit terrestrial TA command");

        // Fully pre-compensated: the preamble lands where the gNB expects it.
        b->SetNtnTimingAdvance(rtt, rtt);
        NS_TEST_ASSERT_MSG_EQ(b->GetResidualTaInTc(), 0u,
                              "full pre-compensation leaves no residual advance to report");

        // Over-compensation must clamp rather than wrap through unsigned.
        b->SetNtnTimingAdvance(rtt, rtt + MilliSeconds(1));
        NS_TEST_ASSERT_MSG_EQ(b->GetResidualTaInTc(), 0u,
                              "over-compensation clamps at zero instead of wrapping");

        // Half compensated: half the residual, which is the case that separates
        // a real subtraction from returning one of the two inputs.
        b->SetNtnTimingAdvance(rtt, NanoSeconds(rtt.GetNanoSeconds() / 2));
        NS_TEST_ASSERT_MSG_EQ_TOL(b->GetResidualTimingAdvance().GetSeconds(),
                                  rtt.GetSeconds() / 2.0, 1e-9,
                                  "the residual tracks the pre-compensation actually applied");
    }
};

// ===========================================================================
// In-depth cases (Nov 2026 test campaign). Every case below drives a live
// mmwave cell built by NtnRealStackHelper (ntn-traffic) through
// Simulator::Run(), decorates its MAC<->PHY SAP with the bridge, and checks
// the FAPI stream against the real PHY decode, the application counters, or a
// published reference (SCF 222.10.02, TS 38.211 / 38.213 / 38.214 / 38.321).
// ===========================================================================

namespace
{

/// One LEO-600 cell per satellite, `uesPerSat` UEs at its nadir (geodetic
/// placement, so the bridge's one-way floor is the true 600 km).
struct FapiRig
{
    NodeContainer sats;
    NodeContainer ues;
    NtnRealStackHelper rs;

    void Build(uint32_t nSat, uint32_t uesPerSat, double simTime, bool uplink = false)
    {
        sats.Create(nSat);
        ues.Create(nSat * uesPerSat);
        for (uint32_t s = 0; s < nSat; ++s)
        {
            // Satellites 20 deg of longitude apart: separate cells, no overlap.
            auto m = CreateObject<ConstantPositionMobilityModel>();
            m->SetPosition(NtnRealStackHelper::GeodeticToEcef(10.0, 20.0 * s, 600e3));
            sats.Get(s)->AggregateObject(m);
            for (uint32_t u = 0; u < uesPerSat; ++u)
            {
                auto um = CreateObject<ConstantPositionMobilityModel>();
                um->SetPosition(
                    NtnRealStackHelper::GeodeticToEcef(10.0 + 0.01 * u, 20.0 * s, 0.0));
                ues.Get(s * uesPerSat + u)->AggregateObject(um);
            }
        }
        rs.SetSimTime(Seconds(simTime));
        rs.SetUplink(uplink);
        rs.Build(sats, ues);
    }

    Ptr<mmwave::MmWaveEnbNetDevice> Enb(uint32_t i) const
    {
        return DynamicCast<mmwave::MmWaveEnbNetDevice>(rs.GetEnbDevices().Get(i));
    }
};

/// Independent per-cell, per-RNTI tally of the real DL decode, taken from the
/// same RxPacketTraceUe the bridge consumes, so the FAPI stream can be checked
/// against the PHY rather than against itself.
struct PhyTally
{
    struct Counts
    {
        uint64_t tb{0};
        uint64_t corrupt{0};
        uint64_t okBytes{0};
    };

    std::map<std::pair<uint64_t, uint16_t>, Counts> perCellRnti;
    std::vector<Ptr<NtnFapiSapBridge>> bridges;

    void OnRx(mmwave::RxPacketTraceParams p)
    {
        if (p.m_tbSize > 0)
        {
            auto& c = perCellRnti[{p.m_cellId, p.m_rnti}];
            ++c.tb;
            c.corrupt += p.m_corrupt ? 1 : 0;
            c.okBytes += p.m_corrupt ? 0 : p.m_tbSize;
        }
        for (auto& b : bridges)
        {
            b->OnPhyRx(p);
        }
    }

    void Connect()
    {
        Config::ConnectWithoutContextFailSafe(
            "/NodeList/*/DeviceList/*/ComponentCarrierMap/*/MmWaveUePhy/DlSpectrumPhy/"
            "RxPacketTraceUe",
            MakeCallback(&PhyTally::OnRx, this));
    }
};

uint32_t
SlotsPerSubframe(Ptr<mmwave::MmWaveEnbNetDevice> enb)
{
    return enb->GetPhy()->GetConfigurationParameters()->GetSlotsPerSubframe();
}

} // namespace

/// SCF 222.10.02 section 3.4.1: one SLOT.indication per slot, and TS 38.211
/// section 4.3.1: a frame is 10 subframes, SFN runs 0..1023. Over 1 s of a live
/// cell the indications must arrive at exactly 1 / slot period, with every
/// consecutive pair one slot apart (no gap, no duplicate), and the scheduling
/// pipeline (SLOT.indication minus DL_TTI.request for the same slot) is
/// non-negative and below one frame.
class FapiSlotCadenceAndSfnTest : public TestCase
{
  public:
    FapiSlotCadenceAndSfnTest()
        : TestCase("SCF 3.4.1 - TS 38.211 4.3.1: one SLOT.indication per slot, contiguous SFN-slot")
    {
    }

  private:
    std::vector<SlotIndication> m_seen;

    void OnSlot(SlotIndication s) { m_seen.push_back(s); }

    void DoRun() override
    {
        const double simTime = 1.0;
        FapiRig rig;
        rig.Build(1, 1, simTime);
        rig.rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::UrllcPings,
                              Seconds(0.2),
                              Seconds(simTime - 0.05));
        auto bridge = CreateObject<NtnFapiSapBridge>();
        bridge->InstallEnb(rig.Enb(0));
        bridge->TraceConnectWithoutContext("SlotIndication",
                                           MakeCallback(&FapiSlotCadenceAndSfnTest::OnSlot, this));
        const auto cfg = rig.Enb(0)->GetPhy()->GetConfigurationParameters();
        const uint32_t spsf = cfg->GetSlotsPerSubframe();
        const double slotS = cfg->GetSlotPeriod().GetSeconds();
        Simulator::Stop(Seconds(simTime));
        Simulator::Run();

        const double expected = simTime / slotS;
        NS_TEST_ASSERT_MSG_EQ_TOL(static_cast<double>(bridge->GetSlotIndicationCount()),
                                  expected,
                                  expected * 0.005 + 2.0,
                                  "one SLOT.indication per slot (FAPI-5), 1/slot period per s");
        NS_TEST_ASSERT_MSG_EQ(m_seen.size(), bridge->GetSlotIndicationCount(),
                              "the trace fires once per counted indication");
        const uint32_t slotsPerFrame = 10 * spsf;
        uint64_t gaps = 0;
        bool rangeOk = true;
        for (std::size_t i = 0; i < m_seen.size(); ++i)
        {
            rangeOk = rangeOk && m_seen[i].sfn <= 1023 && m_seen[i].slot < slotsPerFrame;
            if (i > 0)
            {
                const uint64_t a = m_seen[i - 1].sfn * slotsPerFrame + m_seen[i - 1].slot;
                const uint64_t b = m_seen[i].sfn * slotsPerFrame + m_seen[i].slot;
                const uint64_t mod = 1024ull * slotsPerFrame;
                gaps += ((a + 1) % mod == b) ? 0 : 1;
            }
        }
        NS_TEST_ASSERT_MSG_EQ(rangeOk, true, "SFN in 0..1023, slot in 0..10*2^mu-1");
        NS_TEST_ASSERT_MSG_EQ(gaps, 0u, "consecutive SLOT.indications are exactly one slot apart");
        const double pipe = bridge->GetMeanSchedPipelineSec();
        NS_TEST_ASSERT_MSG_EQ(std::isfinite(pipe), true, "the scheduling pipeline was measured");
        NS_TEST_ASSERT_MSG_GT(pipe + 1e-12, 0.0, "DL_TTI.request precedes its slot");
        NS_TEST_ASSERT_MSG_LT(pipe, 0.010, "and by less than one 10 ms frame");
        Simulator::Destroy();
    }
};

/// P5 CONFIG.request is filled from the live PHY (SCF 222.10.02 section 3.3.2,
/// carrier and cell config TLVs): carrier frequency in kHz and bandwidth in MHz
/// equal what the helper configured, the grid size is set for the ACTIVE
/// numerology only (the TLV is per numerology; other entries are 0), the cell
/// id is the real cell id, and the duplex type is TDD. Failure path: a bridge
/// that was never installed reports no configuration and no latency, not a
/// fabricated number.
class FapiConfigRequestFromPhyTest : public TestCase
{
  public:
    FapiConfigRequestFromPhyTest()
        : TestCase("SCF 3.3.2 CONFIG.request mirrors the live PHY; uninstalled bridge reports nothing")
    {
    }

  private:
    void DoRun() override
    {
        {
            auto idle = CreateObject<NtnFapiSapBridge>();
            NS_TEST_ASSERT_MSG_EQ(idle->IsConfigured(), false, "not configured before install");
            NS_TEST_ASSERT_MSG_EQ(std::isnan(idle->GetMeanSapLatencySec()), true,
                                  "no latency before any match");
            NS_TEST_ASSERT_MSG_EQ(std::isnan(idle->GetLastSapLatencySec()), true,
                                  "no last latency either");
            NS_TEST_ASSERT_MSG_EQ(idle->HasSlotAlignedLatency(), false, "no aligned latency");
            NS_TEST_ASSERT_MSG_EQ(idle->GetSlotIndicationCount(), 0u, "no slots");
            // A decode for an RNTI the filter excludes is ignored; a control TB is ignored.
            idle->SetUeRnti(5);
            mmwave::RxPacketTraceParams p{};
            p.m_rnti = 6;
            p.m_tbSize = 100;
            idle->OnPhyRx(p);
            p.m_rnti = 5;
            p.m_tbSize = 0;
            idle->OnPhyRx(p);
            NS_TEST_ASSERT_MSG_EQ(idle->GetCrcIndicationCount(), 0u,
                                  "other-RNTI and control TBs produce no CRC.indication");
        }

        const double simTime = 0.5;
        FapiRig rig;
        rig.Build(1, 1, simTime);
        auto bridge = CreateObject<NtnFapiSapBridge>();
        bridge->InstallEnb(rig.Enb(0));
        const auto cfg = rig.Enb(0)->GetPhy()->GetConfigurationParameters();
        const CarrierConfig& c = bridge->GetCarrierConfig();
        NS_TEST_ASSERT_MSG_EQ(bridge->IsConfigured(), true, "CONFIG.request consumed");
        NS_TEST_ASSERT_MSG_EQ(c.dlFrequency,
                              static_cast<uint32_t>(cfg->GetCenterFrequency() / 1e3),
                              "dlFrequency is the PHY center frequency in kHz");
        NS_TEST_ASSERT_MSG_EQ_TOL(static_cast<double>(c.dlFrequency) * 1e3,
                                  rig.rs.GetCarrierFrequencyHz(),
                                  1e3,
                                  "and equals the carrier the helper configured");
        NS_TEST_ASSERT_MSG_EQ(c.dlBandwidth,
                              static_cast<uint32_t>(std::lround(cfg->GetBandwidth() / 1e6)),
                              "dlBandwidth in MHz");
        const uint32_t mu = static_cast<uint32_t>(std::lround(std::log2(SlotsPerSubframe(rig.Enb(0)))));
        NS_TEST_ASSERT_MSG_LT(mu, 5u, "numerology index 0..4 (TS 38.211 Table 4.2-1)");
        for (uint32_t i = 0; i < 5; ++i)
        {
            const uint16_t want = (i == mu) ? static_cast<uint16_t>(cfg->GetNumRb()) : 0;
            NS_TEST_ASSERT_MSG_EQ(c.dlGridSize[i], want,
                                  "dlGridSize is set for the active numerology " << mu
                                                                                 << " only, i=" << i);
            NS_TEST_ASSERT_MSG_EQ(c.ulGridSize[i], want, "ulGridSize likewise, i=" << i);
        }
        NS_TEST_ASSERT_MSG_EQ(bridge->GetCellConfig().phyCellId, rig.rs.GetGnbCellId(0),
                              "phyCellId is the real cell id");
        NS_TEST_ASSERT_MSG_EQ(+bridge->GetCellConfig().frameDuplexType, 1, "mmwave is TDD");
        Simulator::Stop(Seconds(simTime));
        Simulator::Run();
        NS_TEST_ASSERT_MSG_GT(bridge->GetSlotIndicationCount(), 100u, "the cell ran");
        Simulator::Destroy();
    }
};

/// Every DL_TTI.request built from the real scheduler decision is well formed
/// against the specifications: PDCCH and PDSCH counts match the PDU list, each
/// PDSCH follows the PDCCH that carries its DCI with the same RNTI, MCS is in
/// TS 38.214 Table 5.1.3.1-1 (0..28), the HARQ id is below the configured
/// process count (TS 38.321 section 5.3.2: 16, extended to 32 for NTN in
/// Rel-17), the TB is
/// non-empty and the symbols fit in the slot.
class FapiDlTtiPduConformanceTest : public TestCase
{
  public:
    FapiDlTtiPduConformanceTest()
        : TestCase("DL_TTI.request PDUs: PDCCH-PDSCH pairing, MCS 0..28, HARQ id, symbols in slot")
    {
    }

  private:
    uint64_t m_reqs{0};
    uint64_t m_bad{0};
    uint64_t m_pdsch{0};
    uint64_t m_schedBytes{0};
    uint32_t m_maxHarq{16};
    uint32_t m_symPerSlot{14};
    std::string m_firstBad;

    void Flag(const std::string& why)
    {
        if (m_bad++ == 0)
        {
            m_firstBad = why;
        }
    }

    void OnDlTti(DlTtiRequest r)
    {
        ++m_reqs;
        if (r.pduList.size() != 2u * r.nPdusOfEachType[1] ||
            r.nPdusOfEachType[0] != r.nPdusOfEachType[1] || r.pduList.empty())
        {
            Flag("PDU counts disagree with the list");
            return;
        }
        for (std::size_t i = 0; i + 1 < r.pduList.size(); i += 2)
        {
            const auto* pdcch = std::get_if<PdcchPdu>(&r.pduList[i].pdu);
            const auto* pdsch = std::get_if<PdschPdu>(&r.pduList[i + 1].pdu);
            if (!pdcch || !pdsch || pdcch->dciList.empty())
            {
                Flag("PDCCH/PDSCH order");
                continue;
            }
            ++m_pdsch;
            m_schedBytes += pdsch->codewords[0].tbSize;
            if (pdcch->dciList[0].rnti != pdsch->rnti || pdsch->rnti == 0)
            {
                Flag("RNTI mismatch");
            }
            if (pdsch->codewords[0].mcsIndex > 28)
            {
                Flag("MCS > 28");
            }
            if (pdsch->harqProcessId >= m_maxHarq)
            {
                Flag("HARQ id out of range");
            }
            if (pdsch->codewords[0].tbSize == 0)
            {
                Flag("empty TB");
            }
            if (pdsch->nrOfSymbols == 0 ||
                pdsch->startSymbolIndex + pdsch->nrOfSymbols > m_symPerSlot)
            {
                Flag("symbols exceed the slot");
            }
        }
    }

    void DoRun() override
    {
        const double simTime = 1.5;
        FapiRig rig;
        rig.Build(1, 2, simTime);
        rig.rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                              Seconds(0.3),
                              Seconds(simTime - 0.1));
        auto bridge = CreateObject<NtnFapiSapBridge>();
        bridge->InstallEnb(rig.Enb(0));
        const auto cfg = rig.Enb(0)->GetPhy()->GetConfigurationParameters();
        m_maxHarq = cfg->GetNumHarqProcess();
        m_symPerSlot = cfg->GetSymbPerSlot();
        bridge->TraceConnectWithoutContext("DlTtiRequest",
                                           MakeCallback(&FapiDlTtiPduConformanceTest::OnDlTti, this));
        Simulator::Stop(Seconds(simTime));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(m_reqs, bridge->GetDlTtiWithDataCount(),
                              "the trace fires for every DL_TTI.request with data");
        NS_TEST_ASSERT_MSG_GT(m_pdsch, 200u, "two saturating UEs fill hundreds of PDSCHs");
        NS_TEST_ASSERT_MSG_EQ(m_bad, 0u, "every PDU conforms; first violation: " << m_firstBad);
        NS_TEST_ASSERT_MSG_LT(m_maxHarq, 33u,
                              "the configured HARQ process count is within the Rel-17 NTN 32");
        NS_TEST_ASSERT_MSG_GT(m_schedBytes, 100000u, "the scheduled TBs carry the eMBB load");
        NS_TEST_ASSERT_MSG_GT(bridge->GetDlTtiRequestCount(), bridge->GetDlTtiWithDataCount(),
                              "control-only slots are counted but carry no data PDU");
        Simulator::Destroy();
    }
};

/// CRC.indication and RX_DATA.indication are the real PHY decode, TB for TB:
/// the CRC count equals the data TBs the UE PHY decoded, the CRC failures equal
/// the error model's corrupt TBs, RX_DATA counts and bytes equal the successful
/// TBs, and the application bytes the UE received never exceed the RX_DATA
/// bytes (the MAC/RLC/PDCP/IP headers sit between them).
class FapiCrcMatchesPhyDecodeTest : public TestCase
{
  public:
    FapiCrcMatchesPhyDecodeTest()
        : TestCase("CRC-RX_DATA.indication equal the PHY decode TB for TB; app bytes <= RX_DATA bytes")
    {
    }

  private:
    void DoRun() override
    {
        const double simTime = 2.0;
        FapiRig rig;
        rig.Build(1, 1, simTime);
        rig.rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                              Seconds(0.3),
                              Seconds(simTime - 0.1));
        auto bridge = CreateObject<NtnFapiSapBridge>();
        bridge->InstallEnb(rig.Enb(0));
        PhyTally tally;
        tally.bridges.push_back(bridge);
        tally.Connect();
        Simulator::Stop(Seconds(simTime));
        Simulator::Run();
        rig.rs.Collect();

        const uint16_t rnti = rig.rs.GetUeRnti(0);
        const auto& per = bridge->GetPerRntiStats();
        NS_TEST_ASSERT_MSG_EQ(per.count(rnti), 1u, "the UE's RNTI is accounted");
        const auto phy = tally.perCellRnti[{rig.rs.GetGnbCellId(0), rnti}];
        const auto fapi = per.count(rnti) ? per.at(rnti) : NtnFapiSapBridge::RntiStats{};
        NS_TEST_ASSERT_MSG_GT(phy.tb, 500u, "the PHY decoded hundreds of data TBs");
        NS_TEST_ASSERT_MSG_EQ(fapi.crc, phy.tb, "one CRC.indication report per data TB");
        NS_TEST_ASSERT_MSG_EQ(fapi.crcFail, phy.corrupt, "CRC failures = corrupt TBs");
        NS_TEST_ASSERT_MSG_EQ(fapi.rxData, phy.tb - phy.corrupt, "RX_DATA only for decoded TBs");
        NS_TEST_ASSERT_MSG_EQ(fapi.rxBytes, phy.okBytes, "RX_DATA bytes = decoded TB bytes");
        NS_TEST_ASSERT_MSG_EQ(bridge->GetCrcIndicationCount(), fapi.crc, "single-UE totals");
        const uint64_t appBytes = rig.rs.GetUeRxBytes(0);
        NS_TEST_ASSERT_MSG_GT(appBytes, 0u, "the application received data");
        NS_TEST_ASSERT_MSG_LT(appBytes, fapi.rxBytes + 1,
                              "app bytes cannot exceed the decoded MAC bytes");
        NS_TEST_ASSERT_MSG_GT(appBytes, fapi.rxBytes / 2,
                              "and the UDP payload is most of them (1400 B packets)");
        NS_TEST_ASSERT_MSG_GT(bridge->GetMatchedLatencyCount(), phy.tb / 2,
                              "most CRCs pair with their DL_TTI.request (SFN/slot aligned)");
        Simulator::Destroy();
    }
};

/// Two cells, one bridge per cell: each bridge must account only the TBs of its
/// own cell. RNTIs are allocated per cell (TS 38.321 section 7.1), so the same
/// RNTI exists in both cells; a bridge that did not filter on the cell id
/// counted the other cell's decode as its own.
class FapiMultiCellIsolationTest : public TestCase
{
  public:
    FapiMultiCellIsolationTest()
        : TestCase("Two cells, two bridges: each accounts only its own cell's TBs")
    {
    }

  private:
    void DoRun() override
    {
        const double simTime = 1.5;
        FapiRig rig;
        rig.Build(2, 1, simTime);
        rig.rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                              Seconds(0.3),
                              Seconds(simTime - 0.1));
        auto b0 = CreateObject<NtnFapiSapBridge>();
        auto b1 = CreateObject<NtnFapiSapBridge>();
        b0->InstallEnb(rig.Enb(0));
        b1->InstallEnb(rig.Enb(1));
        PhyTally tally;
        tally.bridges = {b0, b1};
        tally.Connect();
        Simulator::Stop(Seconds(simTime));
        Simulator::Run();

        const uint16_t c0 = rig.rs.GetGnbCellId(0);
        const uint16_t c1 = rig.rs.GetGnbCellId(1);
        NS_TEST_ASSERT_MSG_NE(c0, c1, "two distinct cells");
        uint64_t phy0 = 0;
        uint64_t phy1 = 0;
        for (const auto& [key, cnt] : tally.perCellRnti)
        {
            (key.first == c0 ? phy0 : phy1) += cnt.tb;
        }
        NS_TEST_ASSERT_MSG_GT(phy0, 100u, "cell 0 carried data");
        NS_TEST_ASSERT_MSG_GT(phy1, 100u, "cell 1 carried data");
        NS_TEST_ASSERT_MSG_EQ(b0->GetCrcIndicationCount(), phy0, "bridge 0 sees only cell 0");
        NS_TEST_ASSERT_MSG_EQ(b1->GetCrcIndicationCount(), phy1, "bridge 1 sees only cell 1");
        NS_TEST_ASSERT_MSG_EQ(rig.rs.GetUeServingCellId(0), c0, "UE 0 under satellite 0");
        NS_TEST_ASSERT_MSG_EQ(rig.rs.GetUeServingCellId(1), c1, "UE 1 under satellite 1");
        Simulator::Destroy();
    }
};

/// RACH.indication timing advance on the live initial access, in TA command
/// units (TS 38.213 section 4.2: N_TA = T_A * 16 * 64 / 2^mu T_c; the RAR TA
/// command is 12 bits, 0..3846, TS 38.321 section 6.2.3). Three runs:
/// a 10 us residual maps to round(10 us / (1024 / 2^mu T_c)); the
/// uncompensated 600 km round trip (4.0036 ms) saturates at 3846; full
/// pre-compensation reports 0.
class FapiRachTaCommandOnAttachTest : public TestCase
{
  public:
    FapiRachTaCommandOnAttachTest()
        : TestCase("TS 38.213 4.2: RACH.indication TA command on live attach (residual, saturation, 0)")
    {
    }

  private:
    static std::pair<uint64_t, uint16_t> Attach(Time rtt, Time preComp, uint32_t& spsf)
    {
        FapiRig rig;
        rig.Build(1, 1, 0.4);
        auto bridge = CreateObject<NtnFapiSapBridge>();
        bridge->InstallEnb(rig.Enb(0));
        spsf = SlotsPerSubframe(rig.Enb(0));
        bridge->SetNtnTimingAdvance(rtt, preComp);
        Simulator::Stop(Seconds(0.4));
        Simulator::Run();
        const auto n = bridge->GetRachIndicationCount();
        const auto& r = bridge->LastRachIndication();
        const uint16_t ta = r.preambles.empty() ? 0xFFFF : r.preambles.front().timingAdvance;
        Simulator::Destroy();
        return {n, ta};
    }

    void DoRun() override
    {
        const Time rtt = NanoSeconds(4003600); // 2 x 600 km / c
        uint32_t spsf = 1;
        const auto partial = Attach(rtt, rtt - MicroSeconds(10), spsf);
        NS_TEST_ASSERT_MSG_GT(partial.first, 0u, "the UE's preamble reached the gNB");
        const double tc = 1.0 / (480e3 * 4096.0);
        const double unit = 16.0 * 64.0 / spsf * tc;
        const auto expect = static_cast<uint16_t>(std::llround(10e-6 / unit));
        NS_TEST_ASSERT_MSG_EQ(partial.second, expect,
                              "10 us residual in TA-command units at mu=log2(" << spsf << ")");
        const auto none = Attach(rtt, Time(), spsf);
        NS_TEST_ASSERT_MSG_EQ(none.second, 3846u,
                              "the uncompensated NTN round trip saturates the 12-bit command");
        const auto full = Attach(rtt, rtt, spsf);
        NS_TEST_ASSERT_MSG_EQ(full.second, 0u, "full pre-compensation leaves nothing to report");
    }
};

/// Link outage on the live cell (failure path): an 80 dB excess loss between
/// 0.8 s and 1.3 s. The L1/L2 boundary must keep its slot cadence through the
/// outage (SLOT.indication is a timing message, not a data message), the decode
/// must fail or stop during it, and the stream must recover after it.
class FapiLinkOutageTest : public TestCase
{
  public:
    FapiLinkOutageTest()
        : TestCase("Failure path: an 80 dB outage keeps SLOT cadence, fails-stops CRC, then recovers")
    {
    }

  private:
    struct Snap
    {
        uint64_t slots{0};
        uint64_t crc{0};
        uint64_t ok{0};
    };

    static Snap Take(Ptr<NtnFapiSapBridge> b)
    {
        return {b->GetSlotIndicationCount(), b->GetCrcIndicationCount(), b->GetRxDataIndicationCount()};
    }

    void DoRun() override
    {
        const double simTime = 2.0;
        FapiRig rig;
        rig.Build(1, 1, simTime);
        rig.rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                              Seconds(0.3),
                              Seconds(simTime - 0.05));
        auto bridge = CreateObject<NtnFapiSapBridge>();
        bridge->InstallEnb(rig.Enb(0));
        PhyTally tally;
        tally.bridges.push_back(bridge);
        tally.Connect();
        auto outage = CreateObject<NtnStaticExtraLossModel>();
        outage->SetLossDb(0.0);
        rig.rs.AddExtraPropagationLoss(outage);
        const double slotS =
            rig.Enb(0)->GetPhy()->GetConfigurationParameters()->GetSlotPeriod().GetSeconds();
        std::array<Snap, 4> s{};
        Simulator::Schedule(Seconds(0.8), [&]() {
            s[0] = Take(bridge);
            outage->SetLossDb(80.0);
        });
        // Leave 100 ms for in-flight HARQ to drain before measuring the outage.
        Simulator::Schedule(Seconds(0.9), [&]() { s[1] = Take(bridge); });
        Simulator::Schedule(Seconds(1.3), [&]() {
            s[2] = Take(bridge);
            outage->SetLossDb(0.0);
        });
        Simulator::Schedule(Seconds(1.95), [&]() { s[3] = Take(bridge); });
        Simulator::Stop(Seconds(simTime));
        Simulator::Run();

        const double outSlots = static_cast<double>(s[2].slots - s[1].slots);
        NS_TEST_ASSERT_MSG_EQ_TOL(outSlots, 0.4 / slotS, 0.4 / slotS * 0.01 + 2,
                                  "SLOT.indication keeps the slot cadence during the outage");
        const double okBefore = static_cast<double>(s[0].ok) ;
        const uint64_t okDuring = s[2].ok - s[1].ok;
        const uint64_t okAfter = s[3].ok - s[2].ok;
        NS_TEST_ASSERT_MSG_GT(okBefore, 100.0, "TBs decode before the outage");
        NS_TEST_ASSERT_MSG_LT(okDuring, 5u, "nothing decodes 80 dB below the budget");
        NS_TEST_ASSERT_MSG_GT(okAfter, 100u, "the stream recovers after the outage");
        Simulator::Destroy();
    }
};

/// Determinism: the same RngRun reproduces the FAPI stream exactly; another run
/// stays inside the physical bounds (a fade can move the CRC failures, not the
/// slot cadence).
class FapiDeterminismTest : public TestCase
{
  public:
    FapiDeterminismTest()
        : TestCase("Determinism: same RngRun gives an identical FAPI stream, another run stays bounded")
    {
    }

  private:
    struct Out
    {
        uint64_t slots;
        uint64_t dlData;
        uint64_t crc;
        uint64_t rxBytes;
        uint64_t txBytes;
    };

    static Out Run(uint64_t run)
    {
        RngSeedManager::SetSeed(1);
        RngSeedManager::SetRun(run);
        // Stream numbers are handed out in creation order for the whole
        // process; reset them so a second run in the same process repeats.
        RngSeedManager::ResetNextStreamIndex();
        FapiRig rig;
        rig.Build(1, 1, 1.0);
        rig.rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                              Seconds(0.3),
                              Seconds(0.95));
        auto bridge = CreateObject<NtnFapiSapBridge>();
        bridge->InstallEnb(rig.Enb(0));
        PhyTally tally;
        tally.bridges.push_back(bridge);
        tally.Connect();
        Simulator::Stop(Seconds(1.0));
        Simulator::Run();
        Out o{bridge->GetSlotIndicationCount(),
              bridge->GetDlTtiWithDataCount(),
              bridge->GetCrcIndicationCount(),
              bridge->GetRxDataBytes(),
              bridge->GetTxDataBytes()};
        Simulator::Destroy();
        return o;
    }

    void DoRun() override
    {
        const Out a = Run(1);
        const Out b = Run(1);
        const Out c = Run(7);
        RngSeedManager::SetRun(1);
        NS_TEST_ASSERT_MSG_EQ(a.slots, b.slots, "same run: same slots");
        NS_TEST_ASSERT_MSG_EQ(a.dlData, b.dlData, "same run: same DL_TTI data count");
        NS_TEST_ASSERT_MSG_EQ(a.crc, b.crc, "same run: same CRC count");
        NS_TEST_ASSERT_MSG_EQ(a.rxBytes, b.rxBytes, "same run: same RX_DATA bytes");
        NS_TEST_ASSERT_MSG_EQ(a.txBytes, b.txBytes, "same run: same TX_DATA bytes");
        NS_TEST_ASSERT_MSG_EQ(c.slots, a.slots, "slot cadence does not depend on the seed");
        NS_TEST_ASSERT_MSG_GT(c.crc, 0u, "another seed still decodes");
        NS_TEST_ASSERT_MSG_EQ_TOL(static_cast<double>(c.rxBytes),
                                  static_cast<double>(a.rxBytes),
                                  0.25 * a.rxBytes,
                                  "and delivers the same load within 25%");
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
        AddTestCase(new FapiRealSapBridgeTest, TestCase::Duration::QUICK);
        AddTestCase(new FapiUlTtiFromRealGrantsTest, TestCase::Duration::QUICK);
        AddTestCase(new FapiRachTimingAdvanceTest, TestCase::Duration::QUICK);
        AddTestCase(new FapiHarqIdIsPerUeTest, TestCase::Duration::QUICK);
        AddTestCase(new FapiSlotCadenceAndSfnTest, TestCase::Duration::QUICK);
        AddTestCase(new FapiConfigRequestFromPhyTest, TestCase::Duration::QUICK);
        AddTestCase(new FapiDlTtiPduConformanceTest, TestCase::Duration::QUICK);
        AddTestCase(new FapiCrcMatchesPhyDecodeTest, TestCase::Duration::QUICK);
        AddTestCase(new FapiMultiCellIsolationTest, TestCase::Duration::QUICK);
        AddTestCase(new FapiRachTaCommandOnAttachTest, TestCase::Duration::QUICK);
        AddTestCase(new FapiLinkOutageTest, TestCase::Duration::QUICK);
        AddTestCase(new FapiDeterminismTest, TestCase::Duration::QUICK);
    }
};

static NtnFapiTestSuite g_ntnFapiTestSuite;
