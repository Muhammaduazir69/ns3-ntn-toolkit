// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_SPECTRUM_TRANSMIT_FILTER_H
#define NR_SPECTRUM_TRANSMIT_FILTER_H

#include "ns3/nr-export.h"
#include "ns3/spectrum-transmit-filter.h"

#include <string>

namespace ns3
{

/**
 * Base class of the NR spectrum transmit filters (NrCsiRsFilter,
 * NrSidelinkFilter and NrCoexistenceFilter), which only need to implement
 * DoFilter, since none of them uses random variables.
 */
class NR_EXPORT NrSpectrumTransmitFilter : public SpectrumTransmitFilter
{
  public:
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();

  protected:
    int64_t DoAssignStreams(int64_t stream) override
    {
        return 0;
    }

    /**
     * Check whether a SpectrumPhy belongs to an NR device.
     *
     * @param phy the SpectrumPhy
     * @return true if the SpectrumPhy is an NrSpectrumPhy
     */
    static bool IsNr(Ptr<const SpectrumPhy> phy);

    /**
     * Create the TypeId of a filter derived from this class.
     *
     * @tparam T the derived filter class
     * @param name the name of the TypeId
     * @return the TypeId of the derived filter
     */
    template <typename T>
    static TypeId MakeFilterTypeId(const std::string& name)
    {
        return TypeId(name)
            .SetParent<NrSpectrumTransmitFilter>()
            .SetGroupName("Nr")
            .template AddConstructor<T>();
    }
};

} // namespace ns3

#endif /* NR_SPECTRUM_TRANSMIT_FILTER_H */
