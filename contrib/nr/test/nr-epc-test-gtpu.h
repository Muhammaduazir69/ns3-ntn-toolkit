/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Jaume Nin <jaume.nin@cttc.cat>
 */

/**
 * @ingroup test
 * @file nr-epc-test-gtpu.h
 *
 * @brief Test suite for the GTP-U header (NrGtpuHeader) serialization code. A single test case
 * fills every header field (version, protocol type, extension header / sequence number / N-PDU
 * number flags, length, message type, TEID, sequence number, N-PDU number and next extension
 * type) with non-default values, adds the header to a packet, removes it into a second header
 * instance, and asserts that the decoded header compares equal to the original one.
 */

#ifndef NR_EPC_TEST_GTPU_H
#define NR_EPC_TEST_GTPU_H

#include "ns3/nr-epc-gtpu-header.h"
#include "ns3/test.h"

using namespace ns3;

/**
 * @ingroup nr
 * @ingroup tests
 * @defgroup nr-test nr module tests
 */

/**
 * @ingroup nr-test
 *
 * @brief Test suite for testing GPRS tunnelling protocol header coding and decoding.
 */
class NrEpsGtpuTestSuite : public TestSuite
{
  public:
    NrEpsGtpuTestSuite();
};

/**
 * Test 1.Check header coding and decoding
 */
class NrEpsGtpuHeaderTestCase : public TestCase
{
  public:
    NrEpsGtpuHeaderTestCase();
    ~NrEpsGtpuHeaderTestCase() override;

  private:
    void DoRun() override;
};

#endif /* NR_EPC_TEST_GTPU_H */
