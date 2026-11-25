// Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-eesm-ir.h"

#include "ns3/boolean.h"
#include "ns3/log.h"

#include <algorithm>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrEesmIr");
NS_OBJECT_ENSURE_REGISTERED(NrEesmIr);

NrEesmIr::NrEesmIr()
{
    NS_LOG_FUNCTION(this);
}

NrEesmIr::~NrEesmIr()
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrEesmIr::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrEesmIr")
            .SetParent<NrEesmErrorModel>()
            .AddAttribute("CreditRateBelowModulationFloor",
                          "If true, the effective code rate of the combined transmissions "
                          "below the lowest code rate of their modulation order is credited "
                          "as a repetition gain of the effective SINR",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NrEesmIr::m_creditRateBelowModulationFloor),
                          MakeBooleanChecker());
    return tid;
}

double
NrEesmIr::ComputeSINR(const SpectrumValue& sinr,
                      const std::vector<int>& map,
                      uint8_t mcs,
                      uint32_t sizeBit,
                      const NrErrorModel::NrErrorModelHistory& sinrHistory) const
{
    NS_LOG_FUNCTION(this);
    // HARQ INCREMENTAL REDUNDANCY: update SINReff and ECR after retx, assuming
    // no repetition of coded bits.

    // compute equivalent effective code rate after retransmissions and total map size
    uint32_t codeBitsSum = 0;
    uint32_t infoBits = DynamicCast<NrEesmErrorModelOutput>(sinrHistory.front())
                            ->m_infoBits; // information bits of the first TB
    double mapSumSize = 0.0;

    for (const Ptr<NrErrorModelOutput>& output : sinrHistory)
    {
        Ptr<NrEesmErrorModelOutput> sinrHistorytemp = DynamicCast<NrEesmErrorModelOutput>(output);
        NS_ASSERT(sinrHistorytemp != nullptr);

        NS_LOG_DEBUG(" Exponential SINR sum " << sinrHistorytemp->m_sinrExp << " codeBits "
                                              << sinrHistorytemp->m_codeBits
                                              << " infoBits: " << sinrHistorytemp->m_infoBits);

        codeBitsSum += sinrHistorytemp->m_codeBits;
        mapSumSize += sinrHistorytemp->m_map.size();
    }
    mapSumSize += map.size();
    codeBitsSum += sizeBit / GetMcsEcrTable()->at(mcs);
    ;
    const_cast<NrEesmIr*>(this)->m_Reff = infoBits / static_cast<double>(codeBitsSum);

    NS_LOG_INFO(" Reff " << m_Reff << " HARQ history (previous) " << sinrHistory.size());

    // compute effective SINR with expSINR_previousTx and mapSumSize
    double expSINR_previousTx = DynamicCast<NrEesmErrorModelOutput>(sinrHistory.back())->m_sinrExp;
    double sinrEff = SinrEff(sinr, map, mcs, expSINR_previousTx, mapSumSize);

    // GetMcsEq() cannot go below the lowest code rate of the modulation order of the
    // transmission. The coded bits beyond that rate still carry energy, which is credited
    // as a repetition (Chase) gain, a lower bound of their incremental redundancy gain.
    if (m_creditRateBelowModulationFloor)
    {
        const uint8_t modOrder = GetMcsMTable()->at(mcs);
        double floorEcr = GetMcsEcrTable()->at(mcs);
        for (uint8_t i = 0; i < mcs; ++i)
        {
            if (GetMcsMTable()->at(i) == modOrder)
            {
                floorEcr = std::min(floorEcr, GetMcsEcrTable()->at(i));
            }
        }
        if (m_Reff < floorEcr)
        {
            sinrEff *= floorEcr / m_Reff;
        }
    }
    return sinrEff;
}

double
NrEesmIr::GetMcsEq(uint8_t mcsTx) const
{
    NS_LOG_FUNCTION(this);
    // PHY abstraction for HARQ-IR retx -> get closest ECR to Reff from the
    // available ones that belong to the same modulation order

    uint8_t mcs_eq = mcsTx;

    uint8_t ModOrder = GetMcsMTable()->at(mcsTx);

    NS_LOG_INFO(" Modulation order: " << +ModOrder);
    NS_LOG_INFO(" Reff: " << m_Reff);

    for (uint8_t mcsindex = (mcsTx - 1); mcsindex != 255; mcsindex--)
    // search from MCS=mcs-1 to MCS=0. end at 255 to account for wrap around of uint
    {
        if ((GetMcsMTable()->at(mcsindex) == ModOrder) && (GetMcsEcrTable()->at(mcsindex) > m_Reff))
        {
            mcs_eq--;
        }
    }

    return mcs_eq;
}

} // namespace ns3
