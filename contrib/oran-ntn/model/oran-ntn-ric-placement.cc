/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
#include "oran-ntn-ric-placement.h"

#include "oran-ntn-e2-interface.h"

#include <ns3/abort.h>
#include <ns3/log.h>
#include <ns3/simulator.h>

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnRicPlacement");
NS_OBJECT_ENSURE_REGISTERED(OranNtnRicPlacement);

namespace
{
constexpr double kC = 299792458.0;
} // namespace

TypeId
OranNtnRicPlacement::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnRicPlacement")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnRicPlacement>();
    return tid;
}

Time
OranNtnRicPlacement::ComputeE2Delay(double slantRangeM) const
{
    switch (m_site)
    {
    case Site::OnBoardSatellite:
        // Co-located with the O-DU: no propagation leg at all.
        return m_processing;
    case Site::Haps:
    case Site::GroundGateway:
        return Seconds(slantRangeM / kC) + m_processing;
    case Site::GroundCloud:
    default:
        return Seconds(slantRangeM / kC) + m_cloudBackhaul + m_processing;
    }
}

void
OranNtnRicPlacement::BindToGeometry(Ptr<MobilityModel> duMobility,
                                    Ptr<MobilityModel> ricSiteMobility,
                                    Ptr<OranNtnE2Node> e2,
                                    Time period)
{
    NS_ABORT_MSG_IF(!duMobility || !e2, "BindToGeometry: DU mobility and E2 node required");
    NS_ABORT_MSG_IF(m_site != Site::OnBoardSatellite && !ricSiteMobility,
                    "BindToGeometry: RIC site mobility required for off-board placement");
    m_du = duMobility;
    m_ricSite = ricSiteMobility;
    m_e2 = e2;
    m_period = period;
    GeometryTick();
}

void
OranNtnRicPlacement::GeometryTick()
{
    double slantM = 0.0;
    if (m_site != Site::OnBoardSatellite)
    {
        slantM = m_du->GetDistanceFrom(m_ricSite);
    }
    m_lastApplied = ComputeE2Delay(slantM);
    m_e2->SetFeederLinkDelay(m_lastApplied);
    Simulator::Schedule(m_period, &OranNtnRicPlacement::GeometryTick, this);
}

} // namespace ns3
