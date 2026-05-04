/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W5)
 */
#ifndef NTN_SAGIN_HELPER_H
#define NTN_SAGIN_HELPER_H

#include "ns3/aeronautical-scenario.h"
#include "ns3/box.h"
#include "ns3/haps-mobility-model.h"
#include "ns3/multi-layer-router.h"
#include "ns3/uav-mobility-models.h"
#include "ns3/vector.h"

namespace ns3
{

/**
 * @brief Convenience installer for SAGIN scenarios.
 *
 * Tiny helper that bundles the four classes above so a one-liner can spin up
 * a multi-layer scenario with a few HAPS, a UAV swarm, and an aircraft.
 */
class SaginHelper
{
  public:
    Ptr<HapsMobilityModel> CreateHaps(const Vector& centerXyz,
                                      double altitudeM = 20000.0,
                                      double radiusM = 5000.0);

    Ptr<UavWaypointMobilityModel> CreateUavWaypoint(const Box& box,
                                                    double speedMps = 20.0);

    Ptr<UavPatrolMobilityModel> CreateUavPatrol(const Vector& a, const Vector& b,
                                                double speedMps = 15.0);

    Ptr<UavSearchPatternMobilityModel> CreateUavSearch(const Vector& origin,
                                                       double legLengthM,
                                                       double stripWidthM,
                                                       int nStrips,
                                                       double altitudeM);

    Ptr<AeronauticalMobilityModel> CreateFlight(const Vector& dep, const Vector& arr,
                                                double cruiseAltM = 11000.0,
                                                double speedMps = 250.0);

    Ptr<MultiLayerRouter> CreateRouter() const;
};

} // namespace ns3

#endif // NTN_SAGIN_HELPER_H
