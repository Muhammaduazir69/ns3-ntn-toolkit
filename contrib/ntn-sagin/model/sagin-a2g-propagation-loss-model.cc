/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 */
#include "sagin-a2g-propagation-loss-model.h"

#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SaginA2gPropagationLossModel");
NS_OBJECT_ENSURE_REGISTERED(SaginA2gPropagationLossModel);

TypeId
SaginA2gPropagationLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SaginA2gPropagationLossModel")
            .SetParent<PropagationLossModel>()
            .SetGroupName("NtnSagin")
            .AddConstructor<SaginA2gPropagationLossModel>()
            .AddAttribute("FrequencyGHz",
                          "Carrier frequency (GHz) for the TR 36.777 model.",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&SaginA2gPropagationLossModel::m_fcGHz),
                          MakeDoubleChecker<double>())
            .AddAttribute("Scenario",
                          "TR 36.777 air-to-ground scenario.",
                          EnumValue(A2gScenario::UMa_AV),
                          MakeEnumAccessor<A2gScenario>(&SaginA2gPropagationLossModel::m_scenario),
                          MakeEnumChecker(A2gScenario::UMa_AV, "UMa_AV",
                                          A2gScenario::RMa_AV, "RMa_AV",
                                          A2gScenario::UMi_AV, "UMi_AV"))
            .AddAttribute("Link",
                          "LOS or NLOS branch.",
                          EnumValue(A2gLink::NLOS),
                          MakeEnumAccessor<A2gLink>(&SaginA2gPropagationLossModel::m_link),
                          MakeEnumChecker(A2gLink::LOS, "LOS", A2gLink::NLOS, "NLOS"))
            .AddAttribute("UavAltitudeM",
                          "UAV (UT) altitude AGL in m; <=0 derives from geometry.",
                          DoubleValue(-1.0),
                          MakeDoubleAccessor(&SaginA2gPropagationLossModel::m_uavAltM),
                          MakeDoubleChecker<double>());
    return tid;
}

SaginA2gPropagationLossModel::SaginA2gPropagationLossModel() = default;
SaginA2gPropagationLossModel::~SaginA2gPropagationLossModel() = default;

double
SaginA2gPropagationLossModel::DoCalcRxPower(double txPowerDbm,
                                            Ptr<MobilityModel> a,
                                            Ptr<MobilityModel> b) const
{
    const Vector pa = a->GetPosition();
    const Vector pb = b->GetPosition();
    const double dx = pa.x - pb.x;
    const double dy = pa.y - pb.y;
    const double dz = pa.z - pb.z;
    double d3d = std::sqrt(dx * dx + dy * dy + dz * dz);
    d3d = std::max(d3d, 1.0);

    // The aerial node (UAV) is the higher endpoint; its altitude is h_UT.
    const double hUt = (m_uavAltM > 0.0) ? m_uavAltM : std::max(pa.z, pb.z);

    const double totalPlDb =
        A2gChannelTr36777::PathLossDb(m_scenario, m_link, d3d, m_fcGHz, hUt);

    // Free-space reference at the same geometry/frequency (matches the base
    // FriisPropagationLossModel: L = 20log10(d) + 20log10(f) - 147.55 dB).
    const double fcHz = m_fcGHz * 1e9;
    const double fsplDb = 20.0 * std::log10(d3d) + 20.0 * std::log10(fcHz) - 147.55;

    // Charge only the excess over free space so the chained net == TR 36.777.
    m_lastLossDb = totalPlDb;
    m_lastExcessDb = std::max(0.0, totalPlDb - fsplDb);
    return txPowerDbm - m_lastExcessDb;
}

int64_t
SaginA2gPropagationLossModel::DoAssignStreams(int64_t /*stream*/)
{
    return 0;
}

} // namespace ns3
