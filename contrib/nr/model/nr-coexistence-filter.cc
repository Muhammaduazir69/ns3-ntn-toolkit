// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-coexistence-filter.h"

#include "nr-spectrum-phy.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrCoexistenceFilter");

NS_OBJECT_ENSURE_REGISTERED(NrCoexistenceFilter);

NrCoexistenceFilter::NrCoexistenceFilter()
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrCoexistenceFilter::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrCoexistenceFilter")
                            .SetParent<SpectrumTransmitFilter>()
                            .AddConstructor<NrCoexistenceFilter>();
    return tid;
}

bool
NrCoexistenceFilter::DoFilter(Ptr<const SpectrumSignalParameters> params,
                              Ptr<const SpectrumPhy> receiverPhy)
{
    NS_LOG_FUNCTION(this << params << receiverPhy);

    bool txIsNr = DynamicCast<const NrSpectrumPhy>(params->txPhy) != nullptr;
    bool rxIsNr = DynamicCast<const NrSpectrumPhy>(receiverPhy) != nullptr;
    return txIsNr != rxIsNr;
}

int64_t
NrCoexistenceFilter::DoAssignStreams(int64_t stream)
{
    return 0;
}

} // namespace ns3
