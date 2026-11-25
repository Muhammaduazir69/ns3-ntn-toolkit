//
// Copyright (c) 2018 Fraunhofer ESK
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Vignesh Babu <ns3-dev@esk.fraunhofer.de>
//

/**
 * @ingroup test
 * @file nr-test-radio-link-failure.h
 *
 * @brief Declarations for the radio link failure tests. NrRadioLinkFailureTestCase simulates one
 * or two gNBs plus a UE that jumps far away mid-simulation to provoke out-of-sync indications and
 * T310 expiry; it is parametrized by the RRC protocol model (ideal or real), the
 * duplexing/pattern setup (TDD DL/UL, TDD mixed, TDD all-flexible or FDD), node counts and
 * positions, the number of unaffected background UEs and whether uplink traffic is enabled.
 * NrRadioLinkFailureTestSuite is instantiated once per duplexing setup, each instance generating
 * the cases of that setup for every RRC model, gNB count and background-UE count, so the suites
 * can run in parallel test-runner processes.
 */

#ifndef NR_TEST_RADIO_LINK_FAILURE_H
#define NR_TEST_RADIO_LINK_FAILURE_H

#include "ns3/mobility-model.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/nr-ue-rrc.h"
#include "ns3/nstime.h"
#include "ns3/test.h"
#include "ns3/vector.h"

#include <vector>

namespace ns3
{

class NrUeNetDevice;

}

using namespace ns3;

/**
 * @ingroup nr
 *
 * @brief Testing the cell reselection procedure by UE at IDLE state
 */
class NrRadioLinkFailureTestCase : public TestCase
{
  public:
    enum TestFddTddSetupType
    {
        TDD_DL_UL,
        TDD_ALL_FLEXIBLE,
        TDD_MIXED_DL_UL_FLEXIBLE,
        FDD
    };

    /**
     * @brief Creates an instance of the radio link failure test case.
     *
     * @param numGnbs number of gNBs
     * @param numUes number of UEs that will suffer the RLF
     * @param numBackgroundUes the number of the background UEs which will not suffer the RLF;
     * instead will stay connected and continue to operate normally
     * @param simTime the simulation time
     * @param isIdealRrc if true, simulation uses Ideal RRC protocol, otherwise
     *                   simulation uses Real RRC protocol
     * @param uePositionList Position of the UEs
     * @param gnbPositionList Position of the gNBs
     * @param ueJumpAwayPosition Vector holding the UE jump away coordinates
     * @param checkConnectedList the time at which UEs should have an active RRC connection
     * @param setup the duplexing/pattern setup used by the test
     * @param enableUplinkTraffic if true, UEs will generate uplink traffic
     */
    NrRadioLinkFailureTestCase(uint32_t numGnbs,
                               uint32_t numUes,
                               uint32_t numBackgroundUes,
                               Time simTime,
                               bool isIdealRrc,
                               std::vector<Vector> uePositionList,
                               std::vector<Vector> gnbPositionList,
                               Vector ueJumpAwayPosition,
                               std::vector<Time> checkConnectedList,
                               TestFddTddSetupType setup,
                               bool enableUplinkTraffic = true);

    ~NrRadioLinkFailureTestCase() override;

  private:
    /**
     * Builds the test name string based on provided parameter values
     * @param numGnbs the number of gNB nodes
     * @param numUes the number of UE nodes
     * @param numBackgroundUes the number of the UE nodes connected normally throughput the
     * simulation, now RLF
     * @param isIdealRrc True if the Ideal RRC protocol is used
     * @param setup the duplexing/pattern setup used by the test
     * @param enableUplinkTraffic True if the uplink traffic is enabled along with the DL traffic
     * @returns the name string
     */
    std::string BuildNameString(uint32_t numGnbs,
                                uint32_t numUes,
                                uint32_t numBackgroundUes,
                                bool isIdealRrc,
                                TestFddTddSetupType setup,
                                bool enableUplinkTraffic);
    /**
     * @brief Setup the simulation according to the configuration set by the
     *        class constructor, run it, and verify the result.
     */
    void DoRun() override;

    /**
     * Check connected function
     * @param ueDevice the UE device
     * @param gnbDevices the gNB devices
     */
    void CheckConnected(Ptr<NetDevice> ueDevice, NetDeviceContainer gnbDevices);

    /**
     * Check if the UE is in idle state
     * @param ueDevice the UE device
     * @param gnbDevices the gNB devices
     */
    void CheckIdle(Ptr<NetDevice> ueDevice, NetDeviceContainer gnbDevices);

    /**
     * @brief Check if the UE exist at the gNB
     * @param rnti the RNTI of the UE
     * @param gnbDevice the gNB device
     * @return true if the UE exist at the gNB, otherwise false
     */
    bool CheckUeExistAtGnb(uint16_t rnti, Ptr<NetDevice> gnbDevice);

    /**
     * @brief State transition callback function
     * @param context the context string
     * @param imsi the IMSI
     * @param cellId the cell ID
     * @param rnti the RNTI
     * @param oldState the old state
     * @param newState the new state
     */
    void UeStateTransitionCallback(std::string context,
                                   uint64_t imsi,
                                   uint16_t cellId,
                                   uint16_t rnti,
                                   NrUeRrc::State oldState,
                                   NrUeRrc::State newState);

    /**
     * @brief Connection established at UE callback function
     * @param context the context string
     * @param imsi the IMSI
     * @param cellId the cell ID
     * @param rnti the RNTI
     */
    void ConnectionEstablishedUeCallback(std::string context,
                                         uint64_t imsi,
                                         uint16_t cellId,
                                         uint16_t rnti);

    /**
     * @brief Connection established at gNB callback function
     * @param context the context string
     * @param imsi the IMSI
     * @param cellId the cell ID
     * @param rnti the RNTI
     */
    void ConnectionEstablishedGnbCallback(std::string context,
                                          uint64_t imsi,
                                          uint16_t cellId,
                                          uint16_t rnti);

    /**
     * @brief This callback function is executed when UE context is removed at gNB
     * @param context the context string
     * @param imsi the IMSI
     * @param cellId the cell ID
     * @param rnti the RNTI
     */
    void ConnectionReleaseAtGnbCallback(std::string context,
                                        uint64_t imsi,
                                        uint16_t cellId,
                                        uint16_t rnti);

    /**
     * @brief This callback function is executed when UE RRC receives an in-sync or out-of-sync
     * indication
     * @param context the context string
     * @param imsi the IMSI
     * @param rnti the RNTI
     * @param cellId the cell ID
     * @param type in-sync or out-of-sync indication
     * @param count the number of in-sync or out-of-sync indications
     */
    void PhySyncDetectionCallback(std::string context,
                                  uint64_t imsi,
                                  uint16_t rnti,
                                  uint16_t cellId,
                                  std::string type,
                                  uint8_t count);

    /**
     * @brief This callback function is executed when radio link failure is detected
     * @param context the context string
     * @param imsi the IMSI
     * @param rnti the RNTI
     * @param cellId the cell ID
     */
    void RadioLinkFailureCallback(std::string context,
                                  uint64_t imsi,
                                  uint16_t cellId,
                                  uint16_t rnti);

    /**
     * @brief Jump away function
     *
     * @param UeJumpAwayPositionList A list of positions where UE would jump
     */
    void JumpAway(Vector UeJumpAwayPositionList);

    uint32_t m_numGnbs;                    ///< number of gNBs
    uint32_t m_numUes;                     ///< number of UEs
    uint32_t m_numBackgroundUes;           ///< number of the background UEs
    Time m_simTime;                        ///< simulation time
    bool m_isIdealRrc;                     ///< whether the NR is configured to use ideal RRC
    std::vector<Vector> m_uePositionList;  ///< Position of the UEs
    std::vector<Vector> m_gnbPositionList; ///< Position of the gNBs
    std::vector<Time>
        m_checkConnectedList;    ///< the time at which UEs should have an active RRC connection
    Vector m_ueJumpAwayPosition; ///< Position where the UE(s) would jump
    TestFddTddSetupType m_setup; ///< duplexing/pattern setup used by the test

    /// The current UE RRC state.
    NrUeRrc::State m_lastState;

    bool m_radioLinkFailureDetected;              ///< true if radio link fails
    uint32_t m_numOfInSyncIndications;            ///< number of in-sync indications detected
    uint32_t m_numOfOutOfSyncIndications;         ///< number of out-of-sync indications detected
    std::vector<Ptr<MobilityModel>> m_ueMobility; ///< UE mobility model
    bool m_enableUplinkTraffic{true};
}; // end of class NrRadioLinkFailureTestCase

/**
 * @brief Radio link failure test suite.
 *
 * One suite instance is created per duplexing/pattern setup (TDD with fixed
 * DL/UL pattern, TDD with mixed fixed/flexible pattern, TDD with all-flexible
 * pattern, and FDD), each holding the RLF cases of that setup for every
 * RRC protocol model and network topology. Partitioning the cases by what
 * they test, instead of keeping a single suite with every combination, also
 * lets the test runner execute the suites in parallel processes.
 *
 * \sa ns3::NrRadioLinkFailureTestCase
 */
class NrRadioLinkFailureTestSuite : public TestSuite
{
  public:
    /**
     * @brief Creates a suite with the RLF cases of one duplexing setup.
     *
     * @param name the test suite name
     * @param setup the duplexing/pattern setup used by all cases of this suite
     * @param idealRrcFlags the RRC protocol models to generate cases for
     *        (true for the Ideal RRC protocol, false for the Real one)
     */
    NrRadioLinkFailureTestSuite(const std::string& name,
                                NrRadioLinkFailureTestCase::TestFddTddSetupType setup,
                                const std::vector<bool>& idealRrcFlags);
};

#endif /* NR_TEST_RADIO_LINK_FAILURE_H */
