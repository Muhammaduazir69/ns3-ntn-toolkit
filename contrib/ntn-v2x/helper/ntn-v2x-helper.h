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
#include <string>

namespace ns3
{
namespace ntnv2x
{

class NtnV2xHelper
{
  public:
    /// Write a DETERMINISTIC synthetic FCD-format CSV fixture for the CI test
    /// suite. This is a disclosed deterministic test fixture — a synthetic
    /// constant-speed CSV (not a SUMO microsimulation; the loader reads a CSV
    /// dialect, not SUMO's native fcd-output XML). `nVehicles` cars on a
    /// straight east-bound highway of length `roadLengthM`, `simSeconds` long,
    /// sampled every `dtSec`. Each vehicle i gets a FIXED speed
    /// vMin + i*(vMax-vMin)/nVehicles (no RNG), so the generated trace is
    /// reproducible bit-for-bit. Returns true on success. Examples take an
    /// --fcdTrace CSV file (the shipped fixtures, or a CSV converted from a
    /// SUMO fcd-export).
    static bool WriteDeterministicTestFcdCsv(const std::string& path,
                                             std::size_t nVehicles,
                                             double roadLengthM,
                                             double simSeconds,
                                             double dtSec = 1.0,
                                             double vMin = 18.0,
                                             double vMax = 28.0);
};

} // namespace ntnv2x
} // namespace ns3

#endif // NTN_V2X_HELPER_H
