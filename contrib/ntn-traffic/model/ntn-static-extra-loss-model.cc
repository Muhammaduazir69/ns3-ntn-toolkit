// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Muhammad Uzair
#include "ntn-static-extra-loss-model.h"

#include "ns3/double.h"
#include "ns3/log.h"

#include <algorithm>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnStaticExtraLossModel");
NS_OBJECT_ENSURE_REGISTERED(NtnStaticExtraLossModel);

TypeId
NtnStaticExtraLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnStaticExtraLossModel")
            .SetParent<PropagationLossModel>()
            .SetGroupName("NtnTraffic")
            .AddConstructor<NtnStaticExtraLossModel>()
            .AddAttribute("Loss",
                          "Excess loss applied to every transmission (dB).",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NtnStaticExtraLossModel::m_lossDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("Floor",
                          "Lowest allowed net loss (dB); 0 = never amplify.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NtnStaticExtraLossModel::m_floorDb),
                          MakeDoubleChecker<double>());
    return tid;
}

NtnStaticExtraLossModel::NtnStaticExtraLossModel() = default;

NtnStaticExtraLossModel::~NtnStaticExtraLossModel() = default;

double
NtnStaticExtraLossModel::DoCalcRxPower(double txPowerDbm,
                                       Ptr<MobilityModel> /*a*/,
                                       Ptr<MobilityModel> /*b*/) const
{
    return txPowerDbm - std::max(m_lossDb, m_floorDb);
}

int64_t
NtnStaticExtraLossModel::DoAssignStreams(int64_t /*stream*/)
{
    return 0;
}

} // namespace ns3
