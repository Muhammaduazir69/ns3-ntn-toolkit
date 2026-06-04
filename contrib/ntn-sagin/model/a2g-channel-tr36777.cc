/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W5)
 */
#include "a2g-channel-tr36777.h"

#include "ns3/log.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("A2gChannelTr36777");
NS_OBJECT_ENSURE_REGISTERED(A2gChannelTr36777);

TypeId
A2gChannelTr36777::GetTypeId()
{
    static TypeId tid = TypeId("ns3::A2gChannelTr36777")
                           .SetParent<Object>()
                           .SetGroupName("NtnSagin")
                           .AddConstructor<A2gChannelTr36777>();
    return tid;
}

A2gChannelTr36777::A2gChannelTr36777() = default;

namespace
{

// Common helper — TR 36.777 LOS path loss kernel for UAV-UT scenarios.
// All scenarios follow PL_LOS = α + β·log10(d3D) + 20·log10(fc[GHz]),
// with α and β depending on scenario and h_UT.
struct LosCoeffs
{
    double alpha;
    double beta;
};

LosCoeffs
GetLosCoeffs(A2gScenario s, double hUtM)
{
    // TR 36.777 v15.0.0 table 6.2-1.
    switch (s)
    {
    case A2gScenario::UMa_AV:
        // TR 36.777 table B-1 gives a single UMa-AV LOS coefficient set that
        // applies to all UT heights: PL_LOS = 28.0 + 22*log10(d3D) + 20*log10(fc).
        return {28.0, 22.0};
    case A2gScenario::RMa_AV:
        // PL_LOS = max(23.9 - 1.8*log10(h_UT), 20) * log10(d3D) + 20*log10(fc)
        // Spec uses a height-dependent slope; flatten for simplicity at h≥10m.
        {
            double slope = std::max(23.9 - 1.8 * std::log10(std::max(hUtM, 1.5)), 20.0);
            return {28.0, slope};
        }
    case A2gScenario::UMi_AV:
        return {30.9, 22.25};   // PL_LOS = 30.9 + 22.25*log10(d3D) + 20*log10(fc)
    }
    return {28.0, 22.0};
}

LosCoeffs
GetNlosCoeffs(A2gScenario s, double hUtM)
{
    // TR 36.777 v15.0.0 table 6.2-1, NLOS.
    switch (s)
    {
    case A2gScenario::UMa_AV:
        // PL_NLOS = -17.5 + (46 - 7*log10(h_UT))*log10(d3D) + 20*log10(40π·fc/3)
        // Reduce to α + β·log10(d3D) + 20·log10(fc) form:
        //   20*log10(40π/3) ≈ 32.44 dB
        //   constant term = -17.5 + 32.44 = 14.94 dB
        {
            double slope = std::max(46.0 - 7.0 * std::log10(std::max(hUtM, 1.5)), 30.0);
            return {14.94, slope};
        }
    case A2gScenario::RMa_AV:
        // PL_NLOS = -12 + (35 - 5.3*log10(h_UT)) * log10(d3D) + 20*log10(fc)
        {
            double slope = std::max(35.0 - 5.3 * std::log10(std::max(hUtM, 1.5)), 22.0);
            return {-12.0 + 32.44, slope};
        }
    case A2gScenario::UMi_AV:
        // PL_NLOS = 32.4 + (43.2 - 7.6*log10(h_UT)) * log10(d3D) + 20*log10(fc)
        {
            double slope = std::max(43.2 - 7.6 * std::log10(std::max(hUtM, 1.5)), 28.0);
            return {32.4, slope};
        }
    }
    return {32.4, 35.0};
}

} // namespace

double
A2gChannelTr36777::PathLossDb(A2gScenario scenario, A2gLink link,
                              double d3dM, double fcGHz, double hUtM)
{
    if (d3dM < 1.0)
    {
        d3dM = 1.0; // floor — no near-field
    }
    if (fcGHz < 0.1 || fcGHz > 100.0)
    {
        NS_LOG_WARN("fcGHz=" << fcGHz << " outside TR 36.777 validation band");
    }
    const double fcTermDb = 20.0 * std::log10(fcGHz);
    const LosCoeffs los = GetLosCoeffs(scenario, hUtM);
    const double losDb = los.alpha + los.beta * std::log10(d3dM) + fcTermDb;
    if (link == A2gLink::LOS)
    {
        return losDb;
    }
    // TR 36.777 defines NLOS path loss as max(PL_LOS, PL'_NLOS): an obstructed
    // link can never have LESS loss than the LOS reference. At short range the
    // raw NLOS formula can dip below LOS, so apply the spec-mandated floor.
    const LosCoeffs nl = GetNlosCoeffs(scenario, hUtM);
    const double nlosDb = nl.alpha + nl.beta * std::log10(d3dM) + fcTermDb;
    return std::max(losDb, nlosDb);
}

double
A2gChannelTr36777::LosProbability(A2gScenario scenario, double d2dM, double hUtM)
{
    // TR 36.777 §7.6.3.1, table 7.6.3.1-2.
    if (d2dM < 1.0)
    {
        return 1.0;
    }
    auto sigmoid = [](double a, double b, double d) {
        // P_LOS,2D-floor + (1 - floor) * exp(-d2D / a)  → use the spec form:
        // For UAV: P_LOS = 1 if d2D ≤ d_1, else d_1/d2D + exp(-d2D/p_1)*(1-d_1/d2D)
        return std::min(1.0,
                        a / std::max(d, 1.0) +
                        std::exp(-d / b) * (1.0 - a / std::max(d, 1.0)));
    };

    switch (scenario)
    {
    case A2gScenario::UMa_AV:
    {
        // TR 36.777 table 7.6.3.1-2 UMa-AV: piecewise in h_UT.
        if (hUtM <= 22.5)
        {
            // 38.901 standard UMa
            double d1 = 18.0;
            double p1 = 63.0;
            return sigmoid(d1, p1, d2dM);
        }
        // Above 22.5 m, P_LOS rises linearly with altitude up to 100 m:
        double frac = std::min(1.0, (hUtM - 22.5) / (100.0 - 22.5));
        double low = sigmoid(18.0, 63.0, d2dM);
        return low + (1.0 - low) * frac;
    }
    case A2gScenario::RMa_AV:
    {
        if (hUtM <= 10.0)
        {
            return d2dM <= 10.0 ? 1.0 : std::exp(-(d2dM - 10.0) / 1000.0);
        }
        double frac = std::min(1.0, (hUtM - 10.0) / (40.0 - 10.0));
        double low = d2dM <= 10.0 ? 1.0 : std::exp(-(d2dM - 10.0) / 1000.0);
        return low + (1.0 - low) * frac;
    }
    case A2gScenario::UMi_AV:
    {
        if (hUtM <= 22.5)
        {
            double d1 = 18.0;
            double p1 = 36.0;
            return sigmoid(d1, p1, d2dM);
        }
        double frac = std::min(1.0, (hUtM - 22.5) / (100.0 - 22.5));
        double low = sigmoid(18.0, 36.0, d2dM);
        return low + (1.0 - low) * frac;
    }
    }
    return 1.0;
}

} // namespace ns3
