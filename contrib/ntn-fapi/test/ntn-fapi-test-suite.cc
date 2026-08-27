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
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/vector.h"

#include <cmath>
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
    }
};

static NtnFapiTestSuite g_ntnFapiTestSuite;
