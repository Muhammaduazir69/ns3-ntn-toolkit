// Copyright (c) 2024 Seoul National University (SNU)
// Copyright (c) 2024 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#pragma once

#include "nr-mac-scheduler-ue-info-qos.h"

#include "ns3/nr-export.h"

namespace ns3
{
/**
 * @ingroup scheduler
 * @brief UE representation for a AI-based scheduler
 *
 * The representation stores the weights of a UE, which are also referred to as actions in the RL
 * model, in response to sending the predefined observation. The observation is a vector of
 * LcObservation, each representing an observation of a flow. In addition to RL-related operations,
 * it updates the metrics in NrMacSchedulerUeInfoQos by inheriting from the NrMacSchedulerUeInfoQos
 * class. In resource allocation per symbol, we can design the reward function of a UE using the QoS
 * metrics.
 *
 * @see LcObservation
 * @see Weights
 * @see NrMacSchedulerUeInfoQos
 */
class NR_EXPORT NrMacSchedulerUeInfoAi : public NrMacSchedulerUeInfoQos
{
  public:
    /**
     * @brief NrMacSchedulerUeInfoAi constructor
     * @param alpha The fairness metric
     * @param rnti RNTI of the UE
     * @param beamId BeamId of the UE
     * @param fn A function that tells how many RB per RBG
     */
    NrMacSchedulerUeInfoAi(float alpha,
                           uint16_t numerology,
                           uint16_t rnti,
                           BeamId beamId,
                           const GetRbPerRbgFn& fn)
        : NrMacSchedulerUeInfoQos(alpha, rnti, beamId, fn),
          m_slotPeriod(Seconds(0.001 / std::pow(2, numerology)))
    {
    }

    /**
     * @typedef Weights
     * @brief A hash map for weights
     *
     * A hash map for weights that maps a pair of uint8_t to a double.
     * The key is the LC ID, and the value is the weight of the LC as a double.
     */
    typedef std::unordered_map<uint8_t, double> Weights;
    /**
     * @typedef UeWeightsMap
     * @brief A hash map for UE weights
     *
     * A hash map for UE weights that maps a uint8_t to a Weights.
     * The key is the RNTI, and the value is the Weights of the UE.
     */
    typedef std::unordered_map<uint8_t, Weights> UeWeightsMap;

    /***
     * @struct LcObservation
     * @brief A struct for an observation of a flow
     *
     * A struct for an observation of a flow that stores the RNTI, LCG ID, LC ID, 5QI, priority, and
     * head-of-line delay of the flow.
     */
    struct NR_EXPORT UeObservation
    {
        uint16_t rnti;
        uint8_t numLcs;
        uint8_t lcId;
        uint8_t fiveQi;
        uint8_t priority;
        uint16_t holDelay;
        uint32_t assignedBytes;
        float avgTput;
    };

    /**
     * @typedef UpdateAllUeWeightsFn
     * @brief A function type for updating the weights of all UEs.
     */
    typedef std::function<void(const Weights&)> UpdateAllUeWeightsFn;
    /**
     * @typedef NotifyCb
     * @brief A callback type for notifying with specific parameters.
     *
     * This callback takes the following parameters:
     * - An Observation object representing the observations
     * - A boolean value indicating whether the game is over (true) or not (false)
     * - A float value representing the reward
     * - A string value representing extra information
     * - A pointer to a const NrMacSchedulerOfdmaAi instance
     */
    typedef Callback<void,
                     const std::vector<UeObservation>&,
                     bool,
                     float,
                     const std::string&,
                     const UpdateAllUeWeightsFn&>
        NotifyCb;

    /**
     * @brief Reset DL AI scheduler info
     *
     * Clear the weights for the downlink.
     * It calls also NrMacSchedulerUeInfoQos::ResetDlSchedInfo.
     */
    void ResetDlSchedInfo() override
    {
        m_weightDl = 0.0;
        NrMacSchedulerUeInfoQos::ResetDlSchedInfo();
    }

    /**
     * @brief Reset UL AI scheduler info
     *
     * Clear the weights for the uplink.
     * It also calls NrMacSchedulerUeInfoQos::ResetUlSchedInfo.
     */
    void ResetUlSchedInfo() override
    {
        m_weightUl = 0.0;
        NrMacSchedulerUeInfoQos::ResetUlSchedInfo();
    }

    /**
     * @brief Update the QoS metric for downlink
     * @param totAssigned the resources assigned
     * @param timeWindow the time window
     */
    void UpdateDlAiMetric(bool isAssigned,
                          const NrMacSchedulerNs3::FTResources& totAssigned,
                          double timeWindow);

    /**
     * @brief Update the QoS metric for uplink
     * @param totAssigned the resources assigned
     * @param timeWindow the time window
     */
    void UpdateUlAiMetric(bool isAssigned,
                          const NrMacSchedulerNs3::FTResources& totAssigned,
                          double timeWindow);

    /**
     * @brief Get the current observation for downlink
     * @param ue the UE
     * @return a vector of LcObservation with the current observation
     *
     * Get the current observation for downlink by iterating over the active LCs of the UE.
     * The observation is stored in a vector of LcObservation and each consists of the RNTI,
     * LCG ID, LC ID, 5QI, priority, and head-of-line delay of the flow.
     */
    UeObservation GetDlObservation();

    /**
     * @brief Get the current observation for uplink
     * @param ue the UE
     * @return a vector of LcObservation with the current observation
     *
     * Get the current observation for uplink by iterating over the active LCs of the UE.
     * The observation is stored in a vector of LcObservation and each consists of the RNTI,
     * LCG ID, LC ID, 5QI, priority, and head-of-line delay of the flow.
     */
    UeObservation GetUlObservation();

    /**
     * @brief Update the weights for downlink
     * @param weights The weights assigned to a UE
     *
     * Update m_weights by copying the weights assigned to a UE.
     * The weights consist of an unordered_map of (key, value) pairs where the lcId is the key,
     * and the weight of the lcId is the value. The higher the weight, the
     * higher the priority of the flow in scheduling.
     */
    void UpdateDlWeight(double weight);

    /**
     * @brief Update the weights for uplink
     * @param weights the weights assigned to a UE
     *
     * Update m_weights by copying the weights assigned to a UE.
     * The weights consist of an unordered_map of (key, value) pairs where the combination of lcgId
     * and lcId is the key, and the weight of the lcId is the value. The higher the weight, the
     * higher the priority of the flow in scheduling.
     */
    void UpdateUlWeight(double weight);

    /**
     * @brief Get the reward for downlink
     * @return The reward for the downlink
     *
     * Calculate the reward for the downlink based on the latest observation and the given weights.
     * The reward is calculated as the sum of the rewards of the active LCs.
     * The reward for an LC \( i \) is calculated as
     * \f$ \text{reward}_{i} = \frac{\text{std::pow}(\text{potentialTput}, \alpha)}{\max(1E-9,
     * \text{avgTput})} \times P_{i} \times \text{HOL}_{i} \f$.
     *
     * @alpha is a fairness metric. \( P \) is the priority associated with the 5QI.
     * HOL is the head-of-line delay of the LC.
     * Please note that the throughput is calculated in bit/symbol.
     */
    float GetDlReward();

    /**
     * @brief Get the reward for uplink
     * @return The reward for the uplink
     *
     * Calculate the reward for the uplink based on the latest observation and the given weights.
     * The reward is calculated as the sum of the rewards of the active LCs.
     * The reward for an LC \( i \) is calculated as
     * \f$ \text{reward}_{i} = \frac{\text{std::pow}(\text{potentialTput}, \alpha)}{\max(1E-9,
     * \text{avgTput})} \times P_{i} \times \text{HOL}_{i} \f$.
     *
     * @alpha is a fairness metric. \( P \) is the priority associated with the 5QI.
     * HOL is the head-of-line delay of the LC.
     * Please note that the throughput is calculated in bit/symbol.
     */
    float GetUlReward();

    /**
     * @brief comparison function object (i.e. an object that satisfies the
     * requirements of Compare) which returns ​true if the first argument is less
     * than (i.e. is ordered before) the second.
     * @param lue Left UE
     * @param rue Right UE
     * @return true if the AI metric of the left UE is higher than the right UE
     *
     * The AI metric is calculated in CalculateDlWeight()
     */
    static bool CompareUeWeightsDl(const NrMacSchedulerNs3::UePtrAndBufferReq& lue,
                                   const NrMacSchedulerNs3::UePtrAndBufferReq& rue)
    {
        auto lUePtr = dynamic_cast<NrMacSchedulerUeInfoAi*>(lue.first.get());
        auto rUePtr = dynamic_cast<NrMacSchedulerUeInfoAi*>(rue.first.get());

        return lUePtr->m_weightDl > rUePtr->m_weightDl;
    }

    /**
     * @brief comparison function object (i.e. an object that satisfies the
     * requirements of Compare) which returns ​true if the first argument is less
     * than (i.e. is ordered before) the second.
     * @param lue Left UE
     * @param rue Right UE
     * @return true if the AI metric of the left UE is higher than the right UE
     *
     * The AI metric is calculated in CalculateUlWeight()
     */
    static bool CompareUeWeightsUl(const NrMacSchedulerNs3::UePtrAndBufferReq& lue,
                                   const NrMacSchedulerNs3::UePtrAndBufferReq& rue)
    {
        auto lUePtr = dynamic_cast<NrMacSchedulerUeInfoAi*>(lue.first.get());
        auto rUePtr = dynamic_cast<NrMacSchedulerUeInfoAi*>(rue.first.get());

        return lUePtr->m_weightUl > rUePtr->m_weightUl;
    }

  private:
    uint8_t m_selectedLcResTypeDl; //!< The selected LC resource type in the downlink
    uint8_t m_selectedLcResTypeUl; //!< The selected LC resource type in the uplink
    double m_weightDl;             //!< The weight of the UE in the downlink
    double m_weightUl;             //!< The weight of the UE in the uplink
    bool m_selectedDl; //!< A boolean value indicating whether the UE is selected in the downlink
    bool m_selectedUl; //!< A boolean value indicating whether the UE is selected in the uplink
    Time m_slotPeriod; //!< The slot period
};

} // namespace ns3
