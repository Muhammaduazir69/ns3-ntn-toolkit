/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 */
#include "sagin-a2g-propagation-loss-model.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

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
                          MakeDoubleChecker<double>())
            .AddAttribute("DrawStochasticLos",
                          "Draw LOS/NLOS per Tx/Rx pair from TR 36.777 Table B-1.1 "
                          "instead of using the static Link attribute.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&SaginA2gPropagationLossModel::m_drawLos),
                          MakeBooleanChecker())
            .AddAttribute("ApplyShadowFading",
                          "Add a per-pair TR 36.777 Table B-1.2 log-normal shadow "
                          "fading term to the path loss.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&SaginA2gPropagationLossModel::m_applyShadowFading),
                          MakeBooleanChecker())
            .AddAttribute("DecorrelationDistanceM",
                          "Endpoint displacement (m) beyond which the cached LOS/"
                          "shadowing realisation for a pair is re-drawn.",
                          DoubleValue(10.0),
                          MakeDoubleAccessor(&SaginA2gPropagationLossModel::m_decorrM),
                          MakeDoubleChecker<double>(0.0));
    return tid;
}

SaginA2gPropagationLossModel::SaginA2gPropagationLossModel()
    : m_losRng(CreateObject<UniformRandomVariable>()),
      m_sfRng(CreateObject<NormalRandomVariable>())
{
    m_losRng->SetAttribute("Min", DoubleValue(0.0));
    m_losRng->SetAttribute("Max", DoubleValue(1.0));
    m_sfRng->SetAttribute("Mean", DoubleValue(0.0));
    m_sfRng->SetAttribute("Variance", DoubleValue(1.0));
}

SaginA2gPropagationLossModel::~SaginA2gPropagationLossModel() = default;

namespace
{
/// Order-independent key for a mobility-model pair.
uint64_t
PairKey(Ptr<MobilityModel> a, Ptr<MobilityModel> b)
{
    const auto pa = reinterpret_cast<uintptr_t>(PeekPointer(a));
    const auto pb = reinterpret_cast<uintptr_t>(PeekPointer(b));
    const uint64_t lo = std::min<uint64_t>(pa, pb);
    const uint64_t hi = std::max<uint64_t>(pa, pb);
    return lo * 1000003ULL + hi;
}
} // namespace

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
    // 2-D (ground-projected) separation feeds the LOS-probability model.
    const double d2d = std::sqrt(dx * dx + dy * dy);

    // --- Per-pair cached stochastic realisation (LOS state + shadowing) -----
    // Re-drawn only when the pair is new or has decorrelated (either endpoint
    // moved > m_decorrM), NOT on every call — otherwise the fading would become
    // per-transport-block white noise.
    const uint64_t key = PairKey(a, b);
    PairState& st = m_cache[key];
    const bool moved = st.valid &&
                       (CalculateDistance(pa, st.posA) > m_decorrM ||
                        CalculateDistance(pb, st.posB) > m_decorrM);
    if (!st.valid || moved)
    {
        if (m_drawLos)
        {
            const double pLos = A2gChannelTr36777::LosProbability(m_scenario, d2d, hUt);
            st.los = (m_losRng->GetValue() < pLos);
        }
        else
        {
            st.los = (m_link == A2gLink::LOS);
        }
        const A2gLink link = st.los ? A2gLink::LOS : A2gLink::NLOS;
        if (m_applyShadowFading)
        {
            const double sigma =
                A2gChannelTr36777::ShadowFadingSigmaDb(m_scenario, link, hUt);
            st.sfDb = m_sfRng->GetValue() * sigma; // N(0, sigma^2)
        }
        else
        {
            st.sfDb = 0.0;
        }
        st.posA = pa;
        st.posB = pb;
        st.valid = true;
    }

    const A2gLink link = st.los ? A2gLink::LOS : A2gLink::NLOS;
    const double totalPlDb =
        A2gChannelTr36777::PathLossDb(m_scenario, link, d3d, m_fcGHz, hUt) + st.sfDb;

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
SaginA2gPropagationLossModel::DoAssignStreams(int64_t stream)
{
    m_losRng->SetStream(stream);
    m_sfRng->SetStream(stream + 1);
    return 2;
}

} // namespace ns3
