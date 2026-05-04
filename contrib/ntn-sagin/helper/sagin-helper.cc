/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W5)
 */
#include "sagin-helper.h"

#include "ns3/double.h"

namespace ns3
{

Ptr<HapsMobilityModel>
SaginHelper::CreateHaps(const Vector& centerXyz, double altitudeM, double radiusM)
{
    Ptr<HapsMobilityModel> haps = CreateObject<HapsMobilityModel>();
    haps->SetAttribute("Altitude", DoubleValue(altitudeM));
    haps->SetAttribute("StationKeepRadius", DoubleValue(radiusM));
    haps->SetCenter(centerXyz);
    return haps;
}

Ptr<UavWaypointMobilityModel>
SaginHelper::CreateUavWaypoint(const Box& box, double speedMps)
{
    Ptr<UavWaypointMobilityModel> uav = CreateObject<UavWaypointMobilityModel>();
    uav->SetAttribute("Speed", DoubleValue(speedMps));
    uav->SetBox(box);
    return uav;
}

Ptr<UavPatrolMobilityModel>
SaginHelper::CreateUavPatrol(const Vector& a, const Vector& b, double speedMps)
{
    Ptr<UavPatrolMobilityModel> uav = CreateObject<UavPatrolMobilityModel>();
    uav->SetAttribute("Speed", DoubleValue(speedMps));
    uav->SetEndpoints(a, b);
    return uav;
}

Ptr<UavSearchPatternMobilityModel>
SaginHelper::CreateUavSearch(const Vector& origin, double legLengthM,
                             double stripWidthM, int nStrips, double altitudeM)
{
    Ptr<UavSearchPatternMobilityModel> uav = CreateObject<UavSearchPatternMobilityModel>();
    uav->SetSearchArea(origin, legLengthM, stripWidthM, nStrips, altitudeM);
    return uav;
}

Ptr<AeronauticalMobilityModel>
SaginHelper::CreateFlight(const Vector& dep, const Vector& arr,
                          double cruiseAltM, double speedMps)
{
    Ptr<AeronauticalMobilityModel> ac = CreateObject<AeronauticalMobilityModel>();
    ac->SetFlightPlan(dep, arr, cruiseAltM, speedMps);
    return ac;
}

Ptr<MultiLayerRouter>
SaginHelper::CreateRouter() const
{
    return CreateObject<MultiLayerRouter>();
}

} // namespace ns3
