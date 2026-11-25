// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-sidelink-filter.h"

#include "nr-spectrum-phy.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrSidelinkFilter");

NS_OBJECT_ENSURE_REGISTERED(NrSidelinkFilter);

NrSidelinkFilter::NrSidelinkFilter()
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrSidelinkFilter::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrSidelinkFilter")
                            .SetParent<SpectrumTransmitFilter>()
                            .AddConstructor<NrSidelinkFilter>();
    return tid;
}

bool
NrSidelinkFilter::DoFilter(Ptr<const SpectrumSignalParameters> params,
                           Ptr<const SpectrumPhy> receiverPhy)
{
    NS_LOG_FUNCTION(this << params << receiverPhy);

    auto nrTxPhy = DynamicCast<const NrSpectrumPhy>(params->txPhy);
    auto nrRxPhy = DynamicCast<const NrSpectrumPhy>(receiverPhy);
    if (!nrTxPhy || !nrRxPhy)
    {
        // Not an NR-to-NR link (e.g., coexistence with another technology), do not filter.
        return false;
    }

    // Keep only gNB-to-UE and UE-to-gNB links
    return nrTxPhy->IsGnb() == nrRxPhy->IsGnb();
}

int64_t
NrSidelinkFilter::DoAssignStreams(int64_t stream)
{
    return 0;
}

} // namespace ns3
