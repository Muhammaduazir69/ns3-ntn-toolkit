/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W7)
 */
#include "ntn-v2x-helper.h"

#include <iomanip>

namespace ns3
{
namespace ntnv2x
{

bool
NtnV2xHelper::WriteDeterministicTestFcdCsv(const std::string& path,
                                           std::size_t nVehicles, double roadLengthM,
                                           double simSeconds, double dtSec,
                                           double vMin, double vMax)
{
    std::ofstream f(path);
    if (!f.is_open())
    {
        return false;
    }
    f << "time,vehid,x,y,z,speed\n";
    // Deterministic per-vehicle speed ramp across [vMin, vMax] — no RNG, so the
    // fixture is reproducible bit-for-bit. This is a disclosed CI fixture, not
    // a stand-in for real measured/SUMO mobility.
    std::vector<double> speeds(nVehicles);
    std::vector<double> startX(nVehicles);
    for (std::size_t i = 0; i < nVehicles; ++i)
    {
        speeds[i] = (nVehicles > 0)
                        ? vMin + static_cast<double>(i) * (vMax - vMin) /
                                     static_cast<double>(nVehicles)
                        : vMin;
        startX[i] = -static_cast<double>(i) * (roadLengthM / nVehicles);
    }

    int nSteps = static_cast<int>(std::round(simSeconds / dtSec));
    for (int step = 0; step <= nSteps; ++step)
    {
        double t = step * dtSec;
        for (std::size_t i = 0; i < nVehicles; ++i)
        {
            double x = startX[i] + speeds[i] * t;
            // wrap around when off the east edge
            double laneY = (i % 2 == 0) ? 0.0 : 3.5;
            f << std::fixed << std::setprecision(2)
              << t << ",veh" << i << "," << x << "," << laneY << ",0.5,"
              << speeds[i] << "\n";
        }
    }
    return true;
}

} // namespace ntnv2x
} // namespace ns3
