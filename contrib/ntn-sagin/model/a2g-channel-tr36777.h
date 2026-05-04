/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W5)
 *
 * 3GPP TR 36.777 (Release 15) air-to-ground channel model for UAVs.
 *  - Scenario: gNB serves a UAV-as-UE between 1.5 m and 300 m AGL.
 *  - Closed-form path loss for UMa-AV / RMa-AV / UMi-AV (table 6.2-1).
 *  - LOS probability per table 7.6.3.1-2 (height-dependent).
 *
 * Reference: 3GPP TR 36.777 V15.0.0 (2017-12).
 */
#ifndef NTN_SAGIN_A2G_CHANNEL_TR36777_H
#define NTN_SAGIN_A2G_CHANNEL_TR36777_H

#include "ns3/object.h"

namespace ns3
{

enum class A2gScenario
{
    UMa_AV,   ///< Urban Macro - Aerial Vehicle (gNB at 25 m, UAV 22.5–300 m)
    RMa_AV,   ///< Rural Macro - Aerial Vehicle (gNB at 35 m)
    UMi_AV    ///< Urban Micro - Aerial Vehicle (gNB at 10 m)
};

enum class A2gLink
{
    LOS,
    NLOS
};

class A2gChannelTr36777 : public Object
{
  public:
    static TypeId GetTypeId();
    A2gChannelTr36777();

    /**
     * @brief Path loss in dB for the given geometry.
     *
     * @param scenario  TR 36.777 scenario.
     * @param link      LOS / NLOS branch.
     * @param d3dM      3D distance gNB → UAV in metres.
     * @param fcGHz     Carrier frequency in GHz (TR 36.777 supports 2 GHz).
     * @param hUtM      UAV (UT) altitude AGL in metres (1.5 m ≤ h ≤ 300 m).
     * @return Path loss in dB.
     *
     * Formulas: TR 36.777 v15.0.0 §6.2 (table 6.2-1).
     */
    static double PathLossDb(A2gScenario scenario, A2gLink link,
                             double d3dM, double fcGHz, double hUtM);

    /**
     * @brief Probability the link is in LOS as a function of geometry.
     *
     * Per TR 36.777 §7.6.3.1, table 7.6.3.1-2. For UAV altitudes above the
     * scenario's threshold the probability monotonically rises to 1.
     */
    static double LosProbability(A2gScenario scenario, double d2dM, double hUtM);
};

} // namespace ns3

#endif // NTN_SAGIN_A2G_CHANNEL_TR36777_H
