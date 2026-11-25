// Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup examples
 * @file lena-v1-utils.h
 *
 * @brief Declaration of the LenaV1Utils class, which encapsulates the LENA v1 (LTE module)
 * configuration of the comparison scenario. Its SetLenaV1SimulatorParameters() creates the
 * LteHelper/EPC and sets the 3GPP pathloss model matching the scenario (UMa/UMi/RMa), eNB and UE
 * transmit power and noise figure, PF or RR scheduler, per-sector EARFCNs implementing the
 * overlapping or non-overlapping frequency reuse, isotropic (calibration) or cosine sectorized
 * antennas, and installs the per-sector eNB and UE devices. It also declares the callbacks that
 * forward the LteUePhy "ReportCurrentCellRsrpSinr" trace to SinrOutputStats and the
 * "ReportPowerSpectralDensity" trace to PowerOutputStats, so that LTE runs populate the same
 * SQLite tables as the NR runs.
 */

#ifndef LENA_V1_UTILS_H
#define LENA_V1_UTILS_H

#include "sinr-output-stats.h"

#include "ns3/hexagonal-grid-scenario-helper.h"
#include "ns3/lte-module.h"
#include "ns3/nr-module.h"

namespace ns3
{

class SinrOutputStats;
class PowerOutputStats;
class SlotOutputStats;

class LenaV1Utils
{
  public:
    static void SetLenaV1SimulatorParameters(const double sector0AngleRad,
                                             std::string scenario,
                                             NodeContainer enbSector1Container,
                                             NodeContainer enbSector2Container,
                                             NodeContainer enbSector3Container,
                                             NodeContainer ueSector1Container,
                                             NodeContainer ueSector2Container,
                                             NodeContainer ueSector3Container,
                                             Ptr<PointToPointEpcHelper>& epcHelper,
                                             Ptr<LteHelper>& lteHelper,
                                             NetDeviceContainer& enbSector1NetDev,
                                             NetDeviceContainer& enbSector2NetDev,
                                             NetDeviceContainer& enbSector3NetDev,
                                             NetDeviceContainer& ueSector1NetDev,
                                             NetDeviceContainer& ueSector2NetDev,
                                             NetDeviceContainer& ueSector3NetDev,
                                             bool calibration,
                                             bool enableUlPc,
                                             SinrOutputStats* sinrStats,
                                             PowerOutputStats* powerStats,
                                             const std::string& scheduler,
                                             uint32_t bandwidthMHz,
                                             uint32_t freqScenario,
                                             double downtiltAngle);

    static void ReportSinrLena(SinrOutputStats* stats,
                               uint16_t cellId,
                               uint16_t rnti,
                               double power,
                               double avgSinr,
                               uint8_t bwpId);
    static void ReportPowerLena(PowerOutputStats* stats, uint16_t rnti, Ptr<SpectrumValue> txPsd);
};

} // namespace ns3

#endif // LENA_V1_UTILS_H
