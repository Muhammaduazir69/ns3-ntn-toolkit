// Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_EESM_IR_H
#define NR_EESM_IR_H

#include "nr-eesm-error-model.h"

#include "ns3/nr-export.h"

namespace ns3
{

/**
 * @ingroup error-models
 * @brief Eesm error model, based on the IR HARQ
 *
 * In HARQ-IR, every retransmission contains different coded bits than the previous
 * one. The different retransmissions typically use a different set of coding bits.
 * Therefore, both the effective SINR and the equivalent ECR need to be recomputed
 * after each retransmission.
 *
 * In HARQ-IR, the HARQ history contains the last computed effective SINR and
 * number of coded bits of each of the previous retransmissions. Given the current
 * SINR vector and the HARQ history, the effective SINR is computed according to EESM.
 *
 * NOTE: The method GetMcsEq() must be called after ComputeSINR(), as it uses
 * the value m_Reff.
 *
 * Please, don't use this class directly, but one between NrEesmIrT1 or NrEesmIrT2,
 * depending on what table you want to use.
 *
 */
class NR_EXPORT NrEesmIr : public NrEesmErrorModel
{
  public:
    /**
     * @brief Get the type id of the object
     * @return the type id of the object
     */
    static TypeId GetTypeId();
    /**
     * @brief NrEesmIr constructor
     */
    NrEesmIr();
    /**
     * @brief ~NrEesmIr deconstructor
     */
    ~NrEesmIr() override;

  protected:
    // Inherited from NrEesmErrorModel
    /**
     * @brief Computes the effective SINR after retransmission combining with HARQ-IR.
     * Also, it updates the equivalent ECR after retransmissions (m_Reff).
     *
     * @param sinr the SINR vector of current transmission
     * @param map the RB map of current transmission
     * @param sizeBit the Transport block size in bits
     * @param mcs the MCS
     * @param sinrHistory the History of the previous transmissions of the same block
     * @return The effective SINR
     */
    double ComputeSINR(const SpectrumValue& sinr,
                       const std::vector<int>& map,
                       uint8_t mcs,
                       uint32_t sizeBit,
                       const NrErrorModel::NrErrorModelHistory& sinrHistory) const override;

    /**
     * @brief Returns the MCS corresponding to the ECR after retransmissions. In case of
     * HARQ-IR the equivalent ECR changes after retransmissions, and it is updated
     * inside ComputeSINR function. GetMcsEq gets the closest ECR to m_Reff from
     * the available ones that belong to the same modulation order.
     *
     * @param mcsTx the MCS of the transmission
     * @return The equivalent MCS after retransmissions
     */
    double GetMcsEq(uint8_t mcsTx) const override;

  private:
    double m_Reff{0.0}; //!< equivalent effective code rate after retransmissions
    /// Whether the effective code rate below the lowest code rate of the modulation order is
    /// credited as a repetition gain of the effective SINR
    bool m_creditRateBelowModulationFloor{true};
};

} // namespace ns3
#endif // NR_EESM_IR_H
