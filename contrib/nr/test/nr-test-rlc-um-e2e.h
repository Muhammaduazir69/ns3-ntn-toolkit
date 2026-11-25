/*
 * Copyright (c) 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Manuel Requena <manuel.requena@cttc.es>
 */

/**
 * @ingroup test
 * @file nr-test-rlc-um-e2e.h
 *
 * @brief Declarations for the `nr-rlc-um-e2e` suite. NrRlcUmE2eTestCase runs bidirectional RRC
 * PDU traffic between a gNB and a UE over real RLC UM entities on lossy SimpleChannel devices and
 * asserts that, per direction, transmitted PDUs equal received PDUs plus counted device-level
 * drops; it is parametrized by the RNG seed and the packet loss rate (0 to 1).
 * NrRlcUmE2eTestSuite instantiates one case per loss rate and seed combination.
 */

#ifndef NR_TEST_RLC_UM_E2E_H
#define NR_TEST_RLC_UM_E2E_H

#include "ns3/ptr.h"
#include "ns3/test.h"

namespace ns3
{
class Packet;
}

using namespace ns3;

/**
 * @ingroup nr-test
 *
 * @brief Test suite for RlcUmE2eTestCase
 */
class NrRlcUmE2eTestSuite : public TestSuite
{
  public:
    NrRlcUmE2eTestSuite();
};

/**
 * @ingroup nr-test
 *
 * @brief Test end-to-end flow when RLC UM is being used.
 */
class NrRlcUmE2eTestCase : public TestCase
{
  public:
    /**
     * Constructor
     *
     * @param name the reference name
     * @param seed the random variable seed
     * @param losses the error rate
     */
    NrRlcUmE2eTestCase(std::string name, uint32_t seed, double losses);
    NrRlcUmE2eTestCase();
    ~NrRlcUmE2eTestCase() override;

  private:
    void DoRun() override;

    /**
     * DL drop event
     * @param p the packet
     */
    void DlDropEvent(Ptr<const Packet> p);
    /**
     * UL drop event
     * @param p the packet
     */
    void UlDropEvent(Ptr<const Packet> p);

    uint32_t m_dlDrops; ///< number of Dl drops
    uint32_t m_ulDrops; ///< number of UL drops

    uint32_t m_seed; ///< random number seed
    double m_losses; ///< error rate
};

#endif // NR_TEST_RLC_UM_E2E_H
