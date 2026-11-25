// Copyright (c) 2011, 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only and NIST-Software
// Authors: Gaurav Sathe <gaurav.sathe@tcs.com>
//          Tom Henderson <thomas.henderson@nist.gov>

/**
 * @ingroup test
 * @file nr-test-deactivate-bearer.h
 *
 * @brief Test suite (nr-test-deactivate-bearer) for dynamic QoS flow (data radio bearer)
 * activation and deactivation. A single gNB and a nearby UE exchange IPv4 and IPv6 UDP traffic
 * in both directions while dedicated QoS flows with different QoS rule precedences are activated
 * and later deactivated at scheduled times. Using the RLC statistics calculator, the test
 * compares the per-LCID received byte counts inside each time window against the expected
 * traffic volume, asserting that traffic always rides the active flow with the lowest precedence
 * rule and falls back to the remaining flows, and finally to the default bearer, as the flows
 * are deactivated.
 */

#ifndef NR_TEST_DEACTIVATE_BEARER_H
#define NR_TEST_DEACTIVATE_BEARER_H

#include "ns3/simulator.h"
#include "ns3/test.h"

namespace ns3
{

/**
 * @ingroup nr-test
 *
 * @brief Test case for dynamic QoS bearer (data radio bearer) activation and deactivation.
 *
 * This test verifies that QoS flows can be dynamically activated and deactivated,
 * with traffic correctly routed to the appropriate logical channels based on
 * QoS rule precedence. The test validates that traffic moves to QoS flows (data radio bearers)
 * with lower QoS rule precedence as they are activated, and then the traffic falls back to
 * the bearers with higher precedence rules when lower precedence flows/rules are deactivated.
 *
 * Test sequence:
 * - 0.03-1.0s:   Default bearer active (QFI=1, DRBID=3, LCID=3) only; no QosRule
 * - 1.0-1.5s:    QFI=3 bearer activated (DRBID=5, LCID=5) with QosRule precedence=10
 * - 1.5-2.0s:    QFI=4 bearer activated (DRBID=6, LCID=6) with QosRule precedence=5
 * - 2.0-2.5s:    QFI=4 deactivated; traffic reverts to QFI=3 (LCID=5)
 * - 2.5-3.0s:    QFI=3 deactivated; traffic reverts to default (LCID=3)
 *
 * IPv4 traffic uses 7.0.0.0/24 and IPv6 traffic uses 6001:db80::/64 with QoS
 * rules configured to match all traffic in these ranges.
 *
 * The test uses four flows: IPv4/v6 downlink and IPv4/v6 uplink.
 */

class NrDeactivateBearerTestCase : public TestCase
{
  public:
    /**
     * Constructor
     *
     * @param ueDistances vector of distances between UEs and gNB (in meters)
     * @param estimatedDlThroughput estimated downlink throughput reference values (not actively
     * used)
     * @param packetSizes vector of packet sizes for traffic flows (in bytes)
     * @param trafficIntervalMs interval between consecutive UDP packets (in milliseconds)
     * @param errorModelEnabled whether PHY layer error model is enabled
     * @param useIdealRrc whether to use ideal RRC behavior
     */
    NrDeactivateBearerTestCase(std::vector<uint16_t> ueDistances,
                               std::vector<uint16_t> packetSizes,
                               Time trafficInterval,
                               bool errorModelEnabled,
                               bool useIdealRrc);
    ~NrDeactivateBearerTestCase() override;

  private:
    /**
     * Build a descriptive name string for the test case
     *
     * @param numberOfUEs number of UE nodes being tested
     * @param ueDistances distances of UEs from gNB in meters
     * @returns formatted name string for test identification
     */
    static std::string BuildNameString(uint16_t numberOfUEs, std::vector<uint16_t> ueDistances);
    void DoRun() override;
    uint16_t m_numberOfUEs;                        ///< Number of UE nodes
    std::vector<uint16_t> m_ueDistances;           ///< Distance of each UE from gNB (in meters)
    std::vector<uint16_t> m_packetSizes;           ///< Packet size for each flow (in bytes)
    Time m_trafficInterval;                        ///< Interval between packets
    std::vector<uint32_t> m_estimatedDlThroughput; ///< Reference DL throughput values
    bool m_errorModelEnabled;                      ///< Whether PHY error model is enabled
};

/**
 * @ingroup nr-test
 *
 * @brief The test suite class for the NrDeactivateBearerTestCase.
 */

class NrTestBearerDeactivateSuite : public TestSuite
{
  public:
    NrTestBearerDeactivateSuite();
};

} // namespace ns3

#endif
