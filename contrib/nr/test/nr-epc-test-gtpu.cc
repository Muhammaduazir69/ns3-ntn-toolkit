/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Jaume Nin <jaume.nin@cttc.cat>
 */

/**
 * @ingroup test
 * @file nr-epc-test-gtpu.cc
 *
 * @brief Test suite for the GTP-U header (NrGtpuHeader) serialization code. A single test case
 * fills every header field (version, protocol type, extension header / sequence number / N-PDU
 * number flags, length, message type, TEID, sequence number, N-PDU number and next extension
 * type) with non-default values, adds the header to a packet, removes it into a second header
 * instance, and asserts that the decoded header compares equal to the original one.
 */

#include "nr-epc-test-gtpu.h"

#include "ns3/log.h"
#include "ns3/nr-epc-gtpu-header.h"
#include "ns3/object.h"
#include "ns3/packet.h"

#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrEpcGtpuTest");

/**
 * TestSuite
 */

NrEpsGtpuTestSuite::NrEpsGtpuTestSuite()
    : TestSuite("nr-epc-gtpu", Type::SYSTEM)
{
    AddTestCase(new NrEpsGtpuHeaderTestCase(), TestCase::Duration::QUICK);
}

static NrEpsGtpuTestSuite nrEpsGtpuTestSuite;

/**
 * TestCase
 */

NrEpsGtpuHeaderTestCase::NrEpsGtpuHeaderTestCase()
    : TestCase("Check header coding and decoding")
{
    NS_LOG_INFO("Creating EpsGtpuHeaderTestCase");
}

NrEpsGtpuHeaderTestCase::~NrEpsGtpuHeaderTestCase()
{
}

void
NrEpsGtpuHeaderTestCase::DoRun()
{
    auto logLevel = (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_TIME | LOG_LEVEL_ALL);

    LogComponentEnable("NrEpcGtpuTest", logLevel);
    NrGtpuHeader h1;
    h1.SetLength(1234);
    h1.SetMessageType(123);
    h1.SetNPduNumber(123);
    h1.SetNPduNumberFlag(true);
    h1.AddExtensionHeader(123, {1, 2, 3, 4, 5});
    h1.SetProtocolType(true);
    h1.SetSequenceNumber(1234);
    h1.SetSequenceNumberFlag(true);
    h1.SetTeid(1234567);
    h1.SetVersion(123);

    Packet p;
    NrGtpuHeader h2;
    p.AddHeader(h1);
    p.RemoveHeader(h2);

    NS_TEST_ASSERT_MSG_EQ(h1, h2, "Wrong value!");

    NS_TEST_ASSERT_MSG_EQ(h2.GetExtensionHeaderFlag(), true, "Wrong E flag");
    NS_TEST_ASSERT_MSG_EQ(h2.GetNextExtensionType(), 123, "Wrong next extension type");
    NS_TEST_ASSERT_MSG_EQ(h2.GetExtensionHeaders().front().content.size(),
                          6,
                          "Extension header not padded to 4 octets");

    // without flag, the optional fields are absent (TS 29.281, section 5.1)
    NrGtpuHeader plain;
    plain.SetSequenceNumberFlag(false);
    plain.SetNPduNumberFlag(false);
    plain.SetTeid(42);
    NS_TEST_ASSERT_MSG_EQ(plain.GetSerializedSize(), 8, "Optional fields without flag");
    Packet plainPacket;
    plainPacket.AddHeader(plain);
    NrGtpuHeader plain2;
    plainPacket.RemoveHeader(plain2);
    NS_TEST_ASSERT_MSG_EQ(plain, plain2, "Wrong header without optional fields");

    // PDU Session Container, in both directions, following another extension header
    for (auto pduType :
         {NrGtpuHeader::UL_PDU_SESSION_INFORMATION, NrGtpuHeader::DL_PDU_SESSION_INFORMATION})
    {
        NrGtpuHeader h3;
        h3.SetTeid(7654321);
        h3.SetMessageType(255);
        h3.AddExtensionHeader(0x40, {0xaa, 0xbb});
        h3.SetPduSessionContainer(pduType, 9);
        Packet payload(100);
        h3.SetLength(payload.GetSize() + h3.GetSerializedSize() - 8);
        payload.AddHeader(h3);
        NS_TEST_ASSERT_MSG_EQ(payload.GetSize(), 120, "Wrong size of the header");
        NrGtpuHeader h4;
        payload.RemoveHeader(h4);
        NS_TEST_ASSERT_MSG_EQ(h3, h4, "Wrong extension headers");
        NS_TEST_ASSERT_MSG_EQ(h4.HasPduSessionContainer(), true, "No PDU Session Container");
        NS_TEST_ASSERT_MSG_EQ(h4.GetPduType(), pduType, "Wrong PDU type");
        NS_TEST_ASSERT_MSG_EQ(h4.GetQfi(), 9, "Wrong QFI");
        NS_TEST_ASSERT_MSG_EQ(payload.GetSize(), 100, "Wrong payload size");
    }

    // octets of an uplink G-PDU with a PDU Session Container, as sent to a UPF
    NrGtpuHeader n3;
    n3.SetSequenceNumberFlag(false);
    n3.SetNPduNumberFlag(false);
    n3.SetTeid(0x01020304);
    n3.SetPduSessionContainer(NrGtpuHeader::UL_PDU_SESSION_INFORMATION, 5);
    n3.SetLength(8);
    Packet n3Packet;
    n3Packet.AddHeader(n3);
    std::vector<uint8_t> octets(n3Packet.GetSize());
    n3Packet.CopyData(octets.data(), octets.size());
    const std::vector<uint8_t> expected{0x34,
                                        0xff,
                                        0x00,
                                        0x08,
                                        0x01,
                                        0x02,
                                        0x03,
                                        0x04,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x85,
                                        0x01,
                                        0x10,
                                        0x05,
                                        0x00};
    NS_TEST_ASSERT_MSG_EQ((octets == expected), true, "Wrong octets on N3");
}
