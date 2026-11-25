/*
 * Copyright (c) 2013 Budiarto Herman
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Budiarto Herman <budiarto.herman@magister.fi>
 *
 */

/**
 * @ingroup test
 * @file nr-test-cell-selection.h
 *
 * @brief Test suite (nr-cell-selection) for the initial cell selection procedure executed by
 * NrUeRrc in IDLE state at the beginning of the simulation. The scenario deploys four cells (two
 * of them CSG) and six UEs whose positions, CSG membership, checkpoint times and expected
 * serving cells are given as test vectors. The simulation runs with the EPC and ideal RRC; UE
 * state transitions and the InitialCellSelectionEndOk/EndError traces are recorded, and at each
 * UE checkpoint the test asserts that the UE is attached to one of its expected cells and has
 * reached the CONNECTED_NORMALLY state.
 */

#ifndef NR_TEST_CELL_SELECTION_H
#define NR_TEST_CELL_SELECTION_H

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
 * @brief Test suite for executing the cell selection test cases in without-EPC
 *        and with-EPC scenarios.
 *
 * \sa ns3::NrCellSelectionTestCase
 */
class NrCellSelectionTestSuite : public TestSuite
{
  public:
    NrCellSelectionTestSuite();
};

/**
 * @ingroup nr
 *
 * @brief Testing the initial cell selection procedure by UE at IDLE state in
 *        the beginning of simulation.
 */
class NrCellSelectionTestCase : public TestCase
{
  public:
    /**
     * @brief A set of input parameters for setting up a UE in the simulation.
     */
    struct UeSetup_t
    {
        Vector position;  ///< The position, relative to the inter site distance, where the UE will
                          ///< be spawned in the simulation.
        bool isCsgMember; ///< Whether UE is allowed access to CSG cell.
        Time checkPoint;  ///< The time in simulation when the UE is verified by the test script.
        uint16_t expectedCellId1; ///< The cell ID that the UE is expected to attach to (0 means
                                  ///< that the UE should not attach to any cell).
        uint16_t expectedCellId2; ///< An alternative cell ID that the UE is expected to attach to
                                  ///< (0 means that this no alternative cell is expected).
        /**
         * @brief UE test setup function.
         * @param relPosX relative position to the inter site distance in X
         * @param relPosY relative position to the inter site distance in Y
         * @param isCsgMember if true, simulation is allowed access to CSG cell
         * @param checkPoint the time in the simulation when the UE is verified
         * @param expectedCellId1 the cell ID that the UE is expected to attach to
         * (0 means that the UE should not attach to any cell).
         * @param expectedCellId2 an alternative cell ID that the UE is expected to attach to
         * (0 means that this no alternative cell is expected).
         */
        UeSetup_t(double relPosX,
                  double relPosY,
                  bool isCsgMember,
                  Time checkPoint,
                  uint16_t expectedCellId1,
                  uint16_t expectedCellId2);
    };

    /**
     * @brief Creates an instance of the initial cell selection test case.
     * @param name name of this test
     * @param isEpcMode set to true for setting up simulation with EPC enabled
     * @param isIdealRrc if true, simulation uses Ideal RRC protocol, otherwise
     *                   simulation uses Real RRC protocol
     * @param interSiteDistance the distance between gNB in meters
     * @param ueSetupList a list of UE configuration to be installed in the
     *                    simulation
     */
    NrCellSelectionTestCase(std::string name,
                            bool isEpcMode,
                            bool isIdealRrc,
                            double interSiteDistance,
                            std::vector<UeSetup_t> ueSetupList);

    ~NrCellSelectionTestCase() override;

  private:
    /**
     * @brief Setup the simulation according to the configuration set by the
     *        class constructor, run it, and verify the result.
     */
    void DoRun() override;

    /**
     * @brief Verifies if the given UE is attached to either of the given two
     *        cells and in a CONNECTED_NORMALLY state.
     * @param ueDev the UE device
     * @param expectedCellId1 the first cell ID
     * @param expectedCellId2 the second cell ID
     */
    void CheckPoint(Ptr<NrUeNetDevice> ueDev, uint16_t expectedCellId1, uint16_t expectedCellId2);

    /**
     * @brief State transition callback function
     * @param context the context string
     * @param imsi the IMSI
     * @param cellId the cell ID
     * @param rnti the RNTI
     * @param oldState the old state
     * @param newState the new state
     */
    void StateTransitionCallback(std::string context,
                                 uint64_t imsi,
                                 uint16_t cellId,
                                 uint16_t rnti,
                                 NrUeRrc::State oldState,
                                 NrUeRrc::State newState);
    /**
     * @brief Initial cell selection end ok callback function
     * @param context the context string
     * @param imsi the IMSI
     * @param cellId the cell ID
     */
    void InitialCellSelectionEndOkCallback(std::string context, uint64_t imsi, uint16_t cellId);
    /**
     * @brief Initial cell selection end error callback function
     * @param context the context string
     * @param imsi the IMSI
     * @param cellId the cell ID
     */
    void InitialCellSelectionEndErrorCallback(std::string context, uint64_t imsi, uint16_t cellId);
    /**
     * @brief Connection established callback function
     * @param context the context string
     * @param imsi the IMSI
     * @param cellId the cell ID
     * @param rnti the RNTI
     */
    void ConnectionEstablishedCallback(std::string context,
                                       uint64_t imsi,
                                       uint16_t cellId,
                                       uint16_t rnti);

    bool m_isEpcMode;                     ///< whether NR is configured to use the EPC
    bool m_isIdealRrc;                    ///< whether NR is configured to use ideal RRC
    double m_interSiteDistance;           ///< inter site distance
    std::vector<UeSetup_t> m_ueSetupList; ///< UE setup list

    /// The current UE RRC state.
    std::vector<NrUeRrc::State> m_lastState;
};

#endif /* NR_TEST_CELL_SELECTION_H */
