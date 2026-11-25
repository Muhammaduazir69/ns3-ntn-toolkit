// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-spectrum-transmit-filter.h"

#include "nr-spectrum-phy.h"

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(NrSpectrumTransmitFilter);

TypeId
NrSpectrumTransmitFilter::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrSpectrumTransmitFilter")
                            .SetParent<SpectrumTransmitFilter>()
                            .SetGroupName("Nr");
    return tid;
}

bool
NrSpectrumTransmitFilter::IsNr(Ptr<const SpectrumPhy> phy)
{
    return DynamicCast<const NrSpectrumPhy>(phy) != nullptr;
}

} // namespace ns3
