// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_COEXISTENCE_FILTER_H
#define NR_COEXISTENCE_FILTER_H

#include "ns3/nr-export.h"
#include "ns3/spectrum-transmit-filter.h"

namespace ns3
{

/**
 * Spectrum transmit filter that discards the signals exchanged between an NR
 * device and a device of another technology (e.g., Wi-Fi), so that the
 * spectrum channel does not compute the propagation loss and channel matrices
 * of cross-technology links.
 *
 * A device is considered to be NR if its SpectrumPhy is an NrSpectrumPhy.
 * Signals exchanged between two NR devices, or between two non-NR devices,
 * are never filtered.
 *
 * @warning This filter removes the interference between NR and the other
 * technologies sharing the channel, so it must not be installed when
 * evaluating coexistence scenarios. It is intended for simulations in which
 * the technologies share a spectrum channel but their mutual interference is
 * not of interest.
 */
class NR_EXPORT NrCoexistenceFilter : public SpectrumTransmitFilter
{
  public:
    NrCoexistenceFilter();

    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * @brief Ignore the signal being received if exactly one of the
     * transmitter and the receiver is an NrSpectrumPhy.
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

#endif /* NR_COEXISTENCE_FILTER_H */
