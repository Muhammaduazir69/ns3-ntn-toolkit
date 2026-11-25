/*
 * Copyright (c) 2011, 2012, 2013 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Lluis Parcerisa <lparcerisa@cttc.cat> (TestUtils from test-asn1-encoding.cc)
 *         Nicola Baldo <nbaldo@cttc.es> (actual test)
 */

/**
 * @ingroup test
 * @file nr-test-rlc-header.cc
 *
 * @brief Unit test suite `nr-rlc-header`: serialization and deserialization of the RLC AM STATUS
 * PDU header. Each case builds an NrRlcAmHeader control PDU with a given ACK SN and list of NACK
 * SNs, adds it to a packet and compares the serialized bytes against a precomputed hex test
 * vector. The header is then removed from the packet and the test asserts that the deserialized
 * ACK SN matches and that the NACK list is recovered exactly, with neither missing nor spurious
 * entries.
 */

#include "ns3/log.h"
#include "ns3/nr-rlc-am-header.h"
#include "ns3/packet.h"
#include "ns3/ptr.h"
#include "ns3/test.h"

#include <bitset>
#include <iomanip>
#include <list>
#include <vector>

NS_LOG_COMPONENT_DEFINE("TestNrRlcHeader");

namespace ns3
{

/**
 * @ingroup nr-test
 *
 * @brief Test Utils
 */
class TestUtils
{
  public:
    /**
     * Function to convert packet contents in hex format
     * @param pkt the packet
     * @returns a text string
     */
    static std::string sprintPacketContentsHex(Ptr<Packet> pkt)
    {
        std::vector<uint8_t> buffer(pkt->GetSize());
        std::ostringstream oss(std::ostringstream::out);
        pkt->CopyData(buffer.data(), buffer.size());
        for (auto b : buffer)
        {
            oss << std::setfill('0') << std::setw(2) << std::hex << (uint32_t)b;
        }
        return oss.str();
    }

    /**
     * Function to convert packet contents in binary format
     * @param pkt the packet
     * @returns a text string
     */
    static std::string sprintPacketContentsBin(Ptr<Packet> pkt)
    {
        std::vector<uint8_t> buffer(pkt->GetSize());
        std::ostringstream oss(std::ostringstream::out);
        pkt->CopyData(buffer.data(), buffer.size());
        for (auto b : buffer)
        {
            oss << (std::bitset<8>(b));
        }
        return std::string(oss.str() + "\n");
    }

    /**
     * Function to log packet contents
     * @param pkt the packet
     */
    static void LogPacketContents(Ptr<Packet> pkt)
    {
        NS_LOG_DEBUG("---- SERIALIZED PACKET CONTENTS (HEX): -------");
        NS_LOG_DEBUG("Hex: " << TestUtils::sprintPacketContentsHex(pkt));
        NS_LOG_DEBUG("Bin: " << TestUtils::sprintPacketContentsBin(pkt));
    }

    /**
     * Log packet info function
     * @param source T
     * @param s text string to log
     */
    template <class T>
    static void LogPacketInfo(T source, std::string s)
    {
        NS_LOG_DEBUG("--------- " << s.data() << " INFO: -------");
        std::ostringstream oss(std::ostringstream::out);
        source.Print(oss);
        NS_LOG_DEBUG(oss.str());
    }
};

/**
 * @ingroup nr-test
 *
 * @brief Rlc Am Status Pdu Test Case
 */
class NrRlcAmStatusPduTestCase : public TestCase
{
  public:
    /**
     * Constructor
     *
     * @param ackSn the sequence number
     * @param nackSnList list of nack sequence numbers
     * @param hex string
     */
    NrRlcAmStatusPduTestCase(nr::SequenceNumber10 ackSn,
                             std::list<nr::SequenceNumber10> nackSnList,
                             std::string hex);

  protected:
    void DoRun() override;

    nr::SequenceNumber10 m_ackSn;                 ///< ack sequence number
    std::list<nr::SequenceNumber10> m_nackSnList; ///< list of nack sequence numbers
    std::string m_hex;                            ///< hex string
};

NrRlcAmStatusPduTestCase::NrRlcAmStatusPduTestCase(nr::SequenceNumber10 ackSn,
                                                   std::list<nr::SequenceNumber10> nackSnList,
                                                   std::string hex)
    : TestCase(hex),
      m_ackSn(ackSn),
      m_nackSnList(nackSnList),
      m_hex(hex)
{
    NS_LOG_FUNCTION(this << hex);
}

void

NrRlcAmStatusPduTestCase::DoRun()
{
    NS_LOG_FUNCTION(this);

    Ptr<Packet> p = Create<Packet>();
    NrRlcAmHeader h;
    h.SetControlPdu(NrRlcAmHeader::STATUS_PDU);
    h.SetAckSn(m_ackSn);
    for (auto it = m_nackSnList.begin(); it != m_nackSnList.end(); ++it)
    {
        h.PushNack(it->GetValue());
    }
    p->AddHeader(h);

    TestUtils::LogPacketContents(p);
    std::string hex = TestUtils::sprintPacketContentsHex(p);
    NS_TEST_ASSERT_MSG_EQ(m_hex,
                          hex,
                          "serialized packet content " << hex << " differs from test vector "
                                                       << m_hex);

    NrRlcAmHeader h2;
    p->RemoveHeader(h2);
    nr::SequenceNumber10 ackSn = h2.GetAckSn();
    NS_TEST_ASSERT_MSG_EQ(ackSn, m_ackSn, "deserialized ACK SN differs from test vector");

    for (auto it = m_nackSnList.begin(); it != m_nackSnList.end(); ++it)
    {
        int nackSn = h2.PopNack();
        NS_TEST_ASSERT_MSG_GT(nackSn, -1, "not enough elements in deserialized NACK list");
        NS_TEST_ASSERT_MSG_EQ(nackSn,
                              it->GetValue(),
                              "deserialized NACK SN  differs from test vector");
    }
    int retVal = h2.PopNack();
    NS_TEST_ASSERT_MSG_LT(retVal, 0, "too many elements in deserialized NACK list");
}

/**
 * @ingroup nr-test
 *
 * @brief Test of NrRlcAmHeader::OneMoreNackWouldFitIn against the actual serialized
 * growth of the STATUS PDU.
 *
 * A NACK is 12 bits, so appending one to an even-sized NACK list grows the
 * serialized header by 2 bytes and to an odd-sized list by 1 byte. The fit check
 * used to have the two cases swapped, letting the STATUS PDU serialize 1 byte over
 * the transmission opportunity (and wasting 1 byte in the other case).
 */
class NrRlcAmStatusFitTestCase : public TestCase
{
  public:
    NrRlcAmStatusFitTestCase()
        : TestCase("Check OneMoreNackWouldFitIn against actual serialized STATUS growth")
    {
    }

  private:
    void DoRun() override
    {
        NrRlcAmHeader header;
        header.SetControlPdu(NrRlcAmHeader::STATUS_PDU);
        header.SetAckSn(nr::SequenceNumber10(8));

        // 0 NACKs: 2 bytes serialized; the next NACK costs 2 bytes.
        NS_TEST_ASSERT_MSG_EQ(header.GetSerializedSize(), 2, "STATUS with 0 NACKs is 2 bytes");
        NS_TEST_ASSERT_MSG_EQ(header.OneMoreNackWouldFitIn(3),
                              false,
                              "a NACK appended to an even-sized list needs 2 more bytes");
        NS_TEST_ASSERT_MSG_EQ(header.OneMoreNackWouldFitIn(4),
                              true,
                              "a 4-byte opportunity fits the first NACK");

        // 1 NACK: 4 bytes serialized; the next NACK costs 1 byte.
        header.PushNack(1);
        NS_TEST_ASSERT_MSG_EQ(header.GetSerializedSize(), 4, "STATUS with 1 NACK is 4 bytes");
        NS_TEST_ASSERT_MSG_EQ(header.OneMoreNackWouldFitIn(4),
                              false,
                              "a NACK appended to an odd-sized list needs 1 more byte");
        NS_TEST_ASSERT_MSG_EQ(header.OneMoreNackWouldFitIn(5),
                              true,
                              "a 5-byte opportunity fits the second NACK");

        // 2 NACKs: 5 bytes serialized; the next NACK costs 2 bytes. The old check
        // accepted a 6-byte opportunity here and serialized 7 bytes.
        header.PushNack(2);
        NS_TEST_ASSERT_MSG_EQ(header.GetSerializedSize(), 5, "STATUS with 2 NACKs is 5 bytes");
        NS_TEST_ASSERT_MSG_EQ(header.OneMoreNackWouldFitIn(6),
                              false,
                              "the STATUS PDU must never serialize beyond the opportunity");
        NS_TEST_ASSERT_MSG_EQ(header.OneMoreNackWouldFitIn(7),
                              true,
                              "a 7-byte opportunity fits the third NACK");
        header.PushNack(3);
        NS_TEST_ASSERT_MSG_EQ(header.GetSerializedSize(), 7, "STATUS with 3 NACKs is 7 bytes");
    }
};

/**
 * @ingroup nr-test
 *
 * @brief Round-trip test of the AMD PDU segment fields (LSF and 15-bit SO).
 *
 * The deserializer used to drop the high byte of the Segment Offset (missing
 * left shift), corrupting any SO larger than 255.
 */
class NrRlcAmSegmentOffsetTestCase : public TestCase
{
  public:
    NrRlcAmSegmentOffsetTestCase()
        : TestCase("Check AMD PDU segment offset serialization round-trip")
    {
    }

  private:
    void DoRun() override
    {
        NrRlcAmHeader header;
        header.SetDataPdu();
        header.SetSequenceNumber(nr::SequenceNumber10(5));
        header.SetResegmentationFlag(NrRlcAmHeader::SEGMENT);
        header.SetLastSegmentFlag(NrRlcAmHeader::LAST_PDU_SEGMENT);
        header.SetSegmentOffset(300); // > 255: exercises the high byte
        header.SetFramingInfo(NrRlcAmHeader::FIRST_BYTE | NrRlcAmHeader::LAST_BYTE);
        header.SetPollingBit(NrRlcAmHeader::STATUS_REPORT_NOT_REQUESTED);
        header.PushExtensionBit(NrRlcAmHeader::DATA_FIELD_FOLLOWS);

        Ptr<Packet> packet = Create<Packet>(20);
        packet->AddHeader(header);

        NrRlcAmHeader deserialized;
        packet->RemoveHeader(deserialized);
        NS_TEST_ASSERT_MSG_EQ(deserialized.GetSequenceNumber().GetValue(),
                              5,
                              "the sequence number must round-trip");
        NS_TEST_ASSERT_MSG_EQ((uint16_t)deserialized.GetLastSegmentFlag(),
                              (uint16_t)NrRlcAmHeader::LAST_PDU_SEGMENT,
                              "the last segment flag must round-trip");
        NS_TEST_ASSERT_MSG_EQ(deserialized.GetSegmentOffset(),
                              300,
                              "the 15-bit segment offset must round-trip, including its high "
                              "byte");
    }
};

/**
 * @ingroup nr-test
 *
 * @brief Nr Rlc Header Test Suite
 */
class NrRlcHeaderTestSuite : public TestSuite
{
  public:
    NrRlcHeaderTestSuite();
} staticNrRlcHeaderTestSuiteInstance; ///< the test suite

NrRlcHeaderTestSuite::NrRlcHeaderTestSuite()
    : TestSuite("nr-rlc-header", Type::UNIT)
{
    NS_LOG_FUNCTION(this);

    {
        nr::SequenceNumber10 ackSn(8);
        std::list<nr::SequenceNumber10> nackSnList;
        std::string hex("0020");
        AddTestCase(new NrRlcAmStatusPduTestCase(ackSn, nackSnList, hex),
                    TestCase::Duration::QUICK);
    }

    {
        nr::SequenceNumber10 ackSn(873);
        std::list<nr::SequenceNumber10> nackSnList;
        std::string hex("0da4");
        AddTestCase(new NrRlcAmStatusPduTestCase(ackSn, nackSnList, hex),
                    TestCase::Duration::QUICK);
    }

    {
        nr::SequenceNumber10 ackSn(2);
        const std::list<nr::SequenceNumber10> nackSnList{
            nr::SequenceNumber10(873),
        };
        std::string hex("000bb480");
        AddTestCase(new NrRlcAmStatusPduTestCase(ackSn, nackSnList, hex),
                    TestCase::Duration::QUICK);
    }

    {
        nr::SequenceNumber10 ackSn(2);
        const std::list<nr::SequenceNumber10> nackSnList{
            nr::SequenceNumber10(1021),
            nr::SequenceNumber10(754),
        };
        std::string hex("000bfed790");
        AddTestCase(new NrRlcAmStatusPduTestCase(ackSn, nackSnList, hex),
                    TestCase::Duration::QUICK);
    }

    {
        nr::SequenceNumber10 ackSn(2);
        const std::list<nr::SequenceNumber10> nackSnList{
            nr::SequenceNumber10(1021),
            nr::SequenceNumber10(754),
            nr::SequenceNumber10(947),
        };
        std::string hex("000bfed795d980");
        AddTestCase(new NrRlcAmStatusPduTestCase(ackSn, nackSnList, hex),
                    TestCase::Duration::QUICK);
    }

    {
        nr::SequenceNumber10 ackSn(2);
        const std::list<nr::SequenceNumber10> nackSnList{
            nr::SequenceNumber10(1021),
            nr::SequenceNumber10(754),
            nr::SequenceNumber10(947),
            nr::SequenceNumber10(347),
        };
        std::string hex("000bfed795d9cad8");
        AddTestCase(new NrRlcAmStatusPduTestCase(ackSn, nackSnList, hex),
                    TestCase::Duration::QUICK);
    }

    AddTestCase(new NrRlcAmStatusFitTestCase(), TestCase::Duration::QUICK);
    AddTestCase(new NrRlcAmSegmentOffsetTestCase(), TestCase::Duration::QUICK);
}

} // namespace ns3
