// Copyright (c) 2023 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_MAC_SCHEDULER_LC_ALGORITHM_H
#define NR_MAC_SCHEDULER_LC_ALGORITHM_H

#include "nr-mac-scheduler-lcg.h"

#include "ns3/nr-export.h"
#include "ns3/object.h"

#include <vector>

namespace ns3
{

using Lcg = uint8_t;
using LcId = uint8_t;
using UnassignedBytes = uint32_t;
using AssignedBytes = uint32_t;

/**
 * @ingroup scheduler
 *
 * @brief This class is the interface for the creation of various scheduling
 * algorithms for the distribution of the assigned bytes to the different LCGs/LCs
 * of a UE.
 *
 * Notice that in the past, the AssignBytesToLC was a method of NrMacSchedulerNs3.
 * This however, did not allow other algorithms to be used for the byte distribution.
 * For this, we have moved it to a class and defined the NrMacSchedulerLcRR
 * as the default type so that the default distribution will be done in a Round Robin
 * manner. Other algorithms can be included by implementing additional classes.
 *
 * Moreover, we have separated the function call into DL and UL direction, due to the
 * scheduler limitation to distinguish among the various LCs of an LCG (it considers
 * only the first LC of an LCG). This way we can allow more sophisticated algorithms
 * to be applied in the DL direction, while the UL can be kept simpler.
 *
 */
class NR_EXPORT NrMacSchedulerLcAlgorithm : public Object
{
  public:
    /**
     * @brief GetTypeId
     * @return The TypeId of the class
     */
    static TypeId GetTypeId();

    /**
     * @brief NrMacSchedulerLcAlgorithm constructor
     */
    NrMacSchedulerLcAlgorithm();

    /**
     * @brief ~NrMacSchedulerLc deconstructor
     */
    ~NrMacSchedulerLcAlgorithm() override;

    /**
     * @brief Represent an assignation of bytes to a LCG/LC
     */
    struct NR_EXPORT Assignation
    {
        /**
         * @brief Assignation constructor (deleted)
         */
        Assignation() = delete;
        /**
         * @brief Assignation copy constructor (deleted)
         * @param o other instance
         */
        Assignation(const Assignation& o) = delete;
        /**
         * @brief Assignation move constructor (default)
         * @param o other instance
         */
        Assignation(Assignation&& o) = default;

        /**
         * @brief Assignation constructor with parameters
         * @param lcg LCG ID
         * @param lcId LC ID
         * @param bytes Assigned bytes
         */
        Assignation(uint8_t lcg, uint8_t lcId, uint32_t bytes)
            : m_lcg(lcg),
              m_lcId(lcId),
              m_bytes(bytes)
        {
        }

        /**
         * @brief Default deconstructor
         */
        ~Assignation() = default;

        uint8_t m_lcg{0};    //!< LCG ID
        uint8_t m_lcId{0};   //!< LC ID
        uint32_t m_bytes{0}; //!< Bytes assigned to the LC
    };

    /**
     * @brief Method to decide how to distribute the assigned bytes to the different LCs
     *        for the DL direction. It first allocates bytes to control channels, and then
     *        applies the custom policy to remaining data channels with DoAssignBytesToDlLC.
     * @param ueLCG LCG of an UE
     * @param tbs TBS to divide between the LCG/LC
     * @return A vector of Assignation
     */
    std::vector<Assignation> AssignBytesToDlLC(const std::unordered_map<uint8_t, LCGPtr>& ueLCG,
                                               uint32_t tbs,
                                               Time slotPeriod) const;

    /**
     * @brief Method to decide how to distribute the assigned bytes to the different LCs
     *        for the UL direction. It first allocates bytes to control channels, and then
     *        applies the custom policy to remaining data channels with DoAssignBytesToUlLC.
     * @param ueLCG LCG of an UE
     * @param tbs TBS to divide between the LCG/LC
     * @return A vector of Assignation
     */
    std::vector<Assignation> AssignBytesToUlLC(const std::unordered_map<uint8_t, LCGPtr>& ueLCG,
                                               uint32_t tbs) const;

    static std::map<std::pair<Lcg, LcId>, std::pair<UnassignedBytes, AssignedBytes>>
    RetrieveActiveLcs(const std::unordered_map<uint8_t, LCGPtr>& ueLCG, bool isDl);

  private:
    /**
     * @brief Method to decide how to distribute the assigned bytes to the different LCs
     *        for the DL direction. Notice that in the DL more sophisticated algorithms
     *        can be applied since there is no limitation in the distinction among the
     *        various LCs as there is in the UL (in the UL the scheduler considers only
     *        the first created LC inside the same LCG).
     * @param ueLCG LCG of an UE
     * @param tbs TBS to divide between the LCG/LC
     * @return A vector of Assignation
     */
    virtual std::vector<Assignation> DoAssignBytesToDlLC(
        const std::unordered_map<uint8_t, LCGPtr>& ueLCG,
        uint32_t tbs,
        Time slotPeriod) const = 0;

    /**
     * @brief Method to decide how to distribute the assigned bytes to the different LCs
     *        for the UL direction. Notice that in the UL there is a limitation in the
     *        distinction among the various LCs since the scheduler considers only the
     *        first created LC inside the same LCG.
     * @param ueLCG LCG of an UE
     * @param tbs TBS to divide between the LCG/LC
     * @return A vector of Assignation
     */
    virtual std::vector<Assignation> DoAssignBytesToUlLC(
        const std::unordered_map<uint8_t, LCGPtr>& ueLCG,
        uint32_t tbs) const = 0;

    static std::vector<NrMacSchedulerLcAlgorithm::Assignation> AssignControlBytes(
        const std::unordered_map<uint8_t, LCGPtr>& ueLCG,
        uint32_t tbs,
        bool isDl);

    std::vector<NrMacSchedulerLcAlgorithm::Assignation> AssignBytesToLC(
        const std::unordered_map<uint8_t, LCGPtr>& ueLCG,
        uint32_t tbs,
        Time slotPeriod,
        bool isDl) const;
};
} // namespace ns3

#endif /*NR_MAC_SCHEDULER_LC_ALGORITHM_H*/
