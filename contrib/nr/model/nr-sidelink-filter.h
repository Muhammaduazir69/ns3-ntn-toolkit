// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_SIDELINK_FILTER_H
#define NR_SIDELINK_FILTER_H

#include "ns3/nr-export.h"
#include "ns3/spectrum-transmit-filter.h"

namespace ns3
{

/**
 * Spectrum transmit filter that discards NR signals exchanged between two
 * gNBs or between two UEs, so that the spectrum channel only computes the
 * propagation loss and channel matrices of gNB-to-UE and UE-to-gNB links.
 *
 * Signals sent or received by a SpectrumPhy that is not an NrSpectrumPhy
 * (e.g., in coexistence scenarios with other technologies) are never filtered.
 *
 * @warning This filter discards every UE-to-UE signal, including sidelink
 * transmissions, so it must not be installed when evaluating sidelink
 * scenarios. It should also be used only with a TDD pattern composed of pure
 * DL and UL slots, where the discarded signals do not represent interference.
 */
class NR_EXPORT NrSidelinkFilter : public SpectrumTransmitFilter
{
  public:
    NrSidelinkFilter();

    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * @brief Ignore the signal being received if both the transmitter and the
     * receiver are NrSpectrumPhy instances of the same role (gNB-gNB or UE-UE).
     *
     * @param params the parameters of the signal being received
     * @param receiverPhy the SpectrumPhy of the receiver
     * @return whether the signal being received should be ignored
     */
    bool DoFilter(Ptr<const SpectrumSignalParameters> params,
                  Ptr<const SpectrumPhy> receiverPhy) override;

  protected:
    int64_t DoAssignStreams(int64_t stream) override;
};

} // namespace ns3

#endif /* NR_SIDELINK_FILTER_H */
