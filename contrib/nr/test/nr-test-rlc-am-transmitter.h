/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Manuel Requena <manuel.requena@cttc.es>
 */

/**
 * @ingroup test
 * @file nr-test-rlc-am-transmitter.h
 *
 * @brief Declarations for the `nr-rlc-am-transmitter` suite. NrRlcAmTransmitterTestCase builds
 * the transmitter-only topology (test PDCP <-> real NrRlcAm <-> test MAC connected via SAPs) and
 * provides scheduled string comparisons of the data received by the MAC. Derived cases cover one
 * SDU mapped to one PDU, segmentation of a single SDU into several PDUs, concatenation of several
 * SDUs into a single PDU, the buffer status report primitive parameters, and the
 * max-retransmission case in which a never-acknowledged PDU must raise the max-retx (RLF)
 * indication exactly once.
 */

#ifndef NR_TEST_RLC_AM_TRANSMITTER_H
#define NR_TEST_RLC_AM_TRANSMITTER_H

#include "ns3/nstime.h"
#include "ns3/ptr.h"
#include "ns3/test.h"

namespace ns3
{

class NrTestRrc;
class NrTestMac;
class NrTestPdcp;
class NrRlc;

} // namespace ns3

using namespace ns3;

/**
 * @ingroup nr-test
 *
 * @brief TestSuite 4.1.1 RLC AM: Only transmitter functionality.
 */
class NrRlcAmTransmitterTestSuite : public TestSuite
{
  public:
    NrRlcAmTransmitterTestSuite();
};

/**
 * @ingroup nr-test
 *
 * @brief Test case used by NrRlcAmTransmitterOneSduTestCase to create topology
 * and to implement functionalities and check if data received corresponds to
 * data sent.
 */
class NrRlcAmTransmitterTestCase : public TestCase
{
  public:
    /**
     * Constructor
     *
     * @param name the reference name
     */
    NrRlcAmTransmitterTestCase(std::string name);
    NrRlcAmTransmitterTestCase();
    ~NrRlcAmTransmitterTestCase() override;

    /**
     * Check data received function
     * @param time the time to check
     * @param shouldReceived should have received indicator
     * @param assertMsg the assert message
     */
    void CheckDataReceived(Time time, std::string shouldReceived, std::string assertMsg);

  protected:
    void DoRun() override;

    Ptr<NrTestPdcp> txPdcp; ///< the transmit PDCP
    Ptr<NrRlc> txRlc;       ///< the RLC
    Ptr<NrTestMac> txMac;   ///< the MAC

  private:
    /**
     * Check data received function
     * @param shouldReceived should have received indicator
     * @param assertMsg the assert message
     */
    void DoCheckDataReceived(std::string shouldReceived, std::string assertMsg);
};

/**
 * @ingroup nr-test
 *
 * @brief Test 4.1.1.1 Test that SDU transmitted at PDCP corresponds to PDU
 * received by MAC.
 */
class NrRlcAmTransmitterOneSduTestCase : public NrRlcAmTransmitterTestCase
{
  public:
    /**
     * Constructor
     *
     * @param name the reference name
     */
    NrRlcAmTransmitterOneSduTestCase(std::string name);
    NrRlcAmTransmitterOneSduTestCase();
    ~NrRlcAmTransmitterOneSduTestCase() override;

  private:
    void DoRun() override;
};

/**
 * @ingroup nr-test
 *
 * @brief Test 4.1.1.2 Test the correct functionality of the Segmentation.
 * Test check that single SDU is properly segmented to n PDUs.
 */
class NrRlcAmTransmitterSegmentationTestCase : public NrRlcAmTransmitterTestCase
{
  public:
    /**
     * Constructor
     *
     * @param name the reference name
     */
    NrRlcAmTransmitterSegmentationTestCase(std::string name);
    NrRlcAmTransmitterSegmentationTestCase();
    ~NrRlcAmTransmitterSegmentationTestCase() override;

  private:
    void DoRun() override;
};

/**
 * @ingroup nr-test
 *
 * @brief Test 4.1.1.3 Test that concatenation functionality works properly.
 * Test check if n SDUs are correctly contactenate to single PDU.
 */
class NrRlcAmTransmitterConcatenationTestCase : public NrRlcAmTransmitterTestCase
{
  public:
    /**
     * Constructor
     *
     * @param name the reference name
     */
    NrRlcAmTransmitterConcatenationTestCase(std::string name);
    NrRlcAmTransmitterConcatenationTestCase();
    ~NrRlcAmTransmitterConcatenationTestCase() override;

  private:
    void DoRun() override;
};

/**
 * @ingroup nr-test
 *
 * @brief Test 4.1.1.4 Test checks functionality of Buffer Status Report by
 * testing primitive parameters.
 */
class NrRlcAmTransmitterBufferStatusReportTestCase : public NrRlcAmTransmitterTestCase
{
  public:
    /**
     * Constructor
     *
     * @param name the reference name
     */
    NrRlcAmTransmitterBufferStatusReportTestCase(std::string name);
    NrRlcAmTransmitterBufferStatusReportTestCase();
    ~NrRlcAmTransmitterBufferStatusReportTestCase() override;

  private:
    void DoRun() override;
};

/**
 * @ingroup nr-test
 *
 * @brief Test that an RLC-AM entity raises its max-retx indication exactly once
 * when a PDU cannot be acknowledged within maxRetxThreshold retransmissions
 * (TS 38.331 5.3.10.3 radio-link-failure trigger). The transmitter-only harness
 * never feeds back a STATUS PDU, so the poll-retransmit timer drives the PDU to
 * max-retx; the SetMaxRetxReachedCallback hook must then fire once.
 */
class NrRlcAmTransmitterMaxRetxTestCase : public NrRlcAmTransmitterTestCase
{
  public:
    /**
     * Constructor
     *
     * @param name the reference name
     */
    NrRlcAmTransmitterMaxRetxTestCase(std::string name);
    ~NrRlcAmTransmitterMaxRetxTestCase() override;

  private:
    void DoRun() override;

    /// Callback target wired to NrRlc::SetMaxRetxReachedCallback; counts invocations.
    void OnMaxRetxReached();

    /**
     * Scheduled assertion: the max-retx callback fired the expected number of times.
     * @param expected expected invocation count
     * @param assertMsg the assert message
     */
    void CheckMaxRetx(uint32_t expected, std::string assertMsg);

    uint32_t m_maxRetxCount{0}; ///< number of times the max-retx callback fired
};

#endif // NR_TEST_RLC_AM_TRANSMITTER_H
