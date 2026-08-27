/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W5)
 *
 * 3GPP TR 36.777 (Release 15) air-to-ground channel model for UAVs.
 *  - Scenario: gNB serves a UAV-as-UE.
 *  - Closed-form path loss for UMa-AV / RMa-AV / UMi-AV (Table B-1.2).
 *  - Height-dependent LOS probability per Table B-1.1.
 *  - Shadow-fading standard deviation per Table B-1.2.
 *
 * Validity bands (Table B-1.1 / B-1.2): the aerial-vehicle formulas hold only
 * above the scenario height threshold — 22.5 m < h <= 300 m (UMa-AV, UMi-AV)
 * and 10 m < h <= 300 m (RMa-AV). BELOW the threshold TR 36.777 mandates the
 * TR 38.901 GROUND formulas, which this class applies as a fallback.
 *
 * Reference: 3GPP TR 36.777 V15.0.0 (2017-12), Annex B; TR 38.901 §7.4.
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
    /// SAGIN-8: upper height of the TR 36.777 aerial-vehicle validation band.
    ///
    /// Table B-1.1/B-1.2 fit the AV coefficients over 1.5 m to 300 m. The model
    /// guarded only the LOWER bound, so a HAPS at 20 km or a satellite silently
    /// evaluated 300 m coefficients an order of magnitude outside the data they
    /// were fitted to, with no warning and no way for the caller to notice.
    static constexpr double kMaxValidatedHeightM = 300.0;

    /// True if the most recent PathLossDb() call was made above
    /// kMaxValidatedHeightM, i.e. the result is an extrapolation rather than a
    /// TR 36.777 prediction.
    static bool WasLastCallAboveValidatedHeight();

    static double PathLossDb(A2gScenario scenario, A2gLink link,
                             double d3dM, double fcGHz, double hUtM);

    /**
     * @brief Probability the link is in LOS as a function of geometry.
     *
     * Per TR 36.777 Table B-1.1. In the aerial band the spec form is
     *   P_LOS = 1                                      for d2D <= d1
     *         = d1/d2D + exp(-d2D/p1)*(1 - d1/d2D)     for d2D >  d1
     * with height-dependent (d1, p1). Below the scenario threshold the
     * TR 38.901 ground LOS-probability is used.
     */
    static double LosProbability(A2gScenario scenario, double d2dM, double hUtM);

    /**
     * @brief Shadow-fading standard deviation (dB) per TR 36.777 Table B-1.2.
     *
     * UMa-AV  LOS: 4.64*exp(-0.0066*h)   NLOS: 6 dB
     * RMa-AV  LOS: 4.2 *exp(-0.0046*h)   NLOS: 8 dB
     * UMi-AV  LOS: 4.64*exp(-0.0066*h)   NLOS: 8 dB
     */
    static double ShadowFadingSigmaDb(A2gScenario scenario, A2gLink link, double hUtM);
};

} // namespace ns3

#endif // NTN_SAGIN_A2G_CHANNEL_TR36777_H
