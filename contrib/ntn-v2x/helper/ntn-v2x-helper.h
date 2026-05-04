/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W7)
 */
#ifndef NTN_V2X_HELPER_H
#define NTN_V2X_HELPER_H

#include "ns3/maritime-scenario.h"
#include "ns3/sumo-traci-bridge.h"
#include "ns3/v2x-leo-direct.h"
#include "ns3/v2x-leo-relay.h"

#include <fstream>
#include <random>
#include <string>

namespace ns3
{
namespace ntnv2x
{

class NtnV2xHelper
{
  public:
    /// Generate a synthetic FCD CSV trace useful for CI when SUMO isn't available.
    /// `nVehicles` cars on a straight east-bound highway, length `roadLengthM`,
    /// `simSeconds` long, sampled every `dtSec`. Vehicles spaced and given
    /// uniform random speeds in [vMin, vMax]. Returns true on success.
    static bool WriteSyntheticFcdCsv(const std::string& path,
                                     std::size_t nVehicles, double roadLengthM,
                                     double simSeconds, double dtSec = 1.0,
                                     double vMin = 18.0, double vMax = 28.0);
};

} // namespace ntnv2x
} // namespace ns3

#endif // NTN_V2X_HELPER_H
