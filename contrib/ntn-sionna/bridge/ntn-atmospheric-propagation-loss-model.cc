/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 */
#include "ntn-atmospheric-propagation-loss-model.h"

#include "ns3/log.h"
#include "ns3/mobility-model.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnAtmosphericPropagationLossModel");
NS_OBJECT_ENSURE_REGISTERED(NtnAtmosphericPropagationLossModel);

namespace
{
constexpr double kEarthRadiusM = 6371000.0;

double
Norm(const Vector& v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
} // namespace

TypeId
NtnAtmosphericPropagationLossModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NtnAtmosphericPropagationLossModel")
                            .SetParent<PropagationLossModel>()
                            .SetGroupName("NtnSionna")
                            .AddConstructor<NtnAtmosphericPropagationLossModel>();
    return tid;
}

NtnAtmosphericPropagationLossModel::NtnAtmosphericPropagationLossModel() = default;

NtnAtmosphericPropagationLossModel::~NtnAtmosphericPropagationLossModel() = default;

void
NtnAtmosphericPropagationLossModel::SetChain(Ptr<NtnAtmosphericLossChain> chain)
{
    m_chain = chain;
}

Ptr<NtnAtmosphericLossChain>
NtnAtmosphericPropagationLossModel::GetChain() const
{
    if (!m_chain)
    {
        m_chain = CreateObject<NtnAtmosphericLossChain>();
    }
    return m_chain;
}

double
NtnAtmosphericPropagationLossModel::DoCalcRxPower(double txPowerDbm,
                                                  Ptr<MobilityModel> a,
                                                  Ptr<MobilityModel> b) const
{
    Ptr<NtnAtmosphericLossChain> chain = GetChain();
    const Vector pa = a->GetPosition();
    const Vector pb = b->GetPosition();

    // Identify the ground endpoint: in ECEF (positions near/above Earth
    // radius) it is the smaller-norm point; in a local ENU frame it is the
    // lower-z point.
    Vector gnd = pa;
    Vector sat = pb;
    const bool ecef = std::max(Norm(pa), Norm(pb)) > 0.9 * kEarthRadiusM;
    const bool aIsGround = ecef ? (Norm(pa) <= Norm(pb)) : (pa.z <= pb.z);
    if (!aIsGround)
    {
        gnd = pb;
        sat = pa;
    }

    double lossDb = chain->ComputeAttenuationDb(gnd, sat);
    if (chain->GetEnableLms())
    {
        lossDb += chain->StepLmsDb();
    }
    m_lastLossDb = lossDb;
    return txPowerDbm - lossDb;
}

int64_t
NtnAtmosphericPropagationLossModel::DoAssignStreams(int64_t stream)
{
    return m_chain ? m_chain->AssignStreams(stream) : 0;
}

} // namespace ns3
