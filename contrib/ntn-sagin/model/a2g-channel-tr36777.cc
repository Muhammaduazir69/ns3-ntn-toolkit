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

// 20*log10(40*pi/3) = 32.44 dB — the frequency-independent constant shared by
// the RMa-AV (LOS/NLOS) and UMa-AV (NLOS) TR 36.777 path-loss equations, which
// carry the 20*log10(40*pi*fc/3) term.  fc[GHz] is added separately as
// 20*log10(fc), so 20*log10(40*pi*fc/3) = 32.44 + 20*log10(fc).
constexpr double k40Pi3Db = 32.44;

/// SAGIN-8: set by PathLossDb when the height leaves the validation band.
/// File-scope because PathLossDb is static and stateless by design; this is
/// diagnostic only and never feeds the returned value.
bool g_lastAboveValidatedHeight = false;

// Lower height threshold of the aerial-vehicle band (TR 36.777 Table B-1.1/2).
double
AerialHeightThreshold(A2gScenario s)
{
    return (s == A2gScenario::RMa_AV) ? 10.0 : 22.5;
}

// TR 36.777 aerial-vehicle band path-loss kernel.  PL = alpha + beta*log10(d3D)
// + 20*log10(fc[GHz]); alpha absorbs the frequency-independent constant (incl.
// the 32.44 dB term where applicable).  Only valid for h above the threshold.
struct LosCoeffs
{
    double alpha;
    double beta;
};

LosCoeffs
GetLosCoeffs(A2gScenario s, double hUtM)
{
    // TR 36.777 v15.0.0 Table B-1.2, LOS, aerial-vehicle band.
    switch (s)
    {
    case A2gScenario::UMa_AV:
        // PL_LOS = 28.0 + 22*log10(d3D) + 20*log10(fc)   (22.5 m < h <= 300 m)
        return {28.0, 22.0};
    case A2gScenario::RMa_AV:
        // PL_LOS = max(23.9 - 1.8*log10(h), 20)*log10(d3D) + 20*log10(40*pi*fc/3)
        {
            double slope = std::max(23.9 - 1.8 * std::log10(hUtM), 20.0);
            return {k40Pi3Db, slope};
        }
    case A2gScenario::UMi_AV:
        // PL_LOS = 30.9 + (22.25 - 0.5*log10(h))*log10(d3D) + 20*log10(fc)
        {
            double slope = 22.25 - 0.5 * std::log10(hUtM);
            return {30.9, slope};
        }
    }
    return {28.0, 22.0};
}

LosCoeffs
GetNlosCoeffs(A2gScenario s, double hUtM)
{
    // TR 36.777 v15.0.0 Table B-1.2, NLOS, aerial-vehicle band.
    switch (s)
    {
    case A2gScenario::UMa_AV:
        // PL_NLOS = -17.5 + (46 - 7*log10(h))*log10(d3D) + 20*log10(40*pi*fc/3)
        //   constant = -17.5 + 32.44 = 14.94 dB
        return {-17.5 + k40Pi3Db, 46.0 - 7.0 * std::log10(hUtM)};
    case A2gScenario::RMa_AV:
        // PL_NLOS = -12 + (35 - 5.3*log10(h))*log10(d3D) + 20*log10(40*pi*fc/3)
        //   constant = -12 + 32.44 = 20.44 dB
        return {-12.0 + k40Pi3Db, 35.0 - 5.3 * std::log10(hUtM)};
    case A2gScenario::UMi_AV:
        // PL_NLOS = 32.4 + (43.2 - 7.6*log10(h))*log10(d3D) + 20*log10(fc)
        return {32.4, 43.2 - 7.6 * std::log10(hUtM)};
    }
    return {32.4, 35.0};
}

// TR 38.901 §7.4.1 ground path loss — used by TR 36.777 for UT heights BELOW the
// aerial-vehicle band threshold.  Structurally different from the AV formulas
// (breakpoint-distance LOS, NLOS = max(LOS, NLOS')), so we implement it verbatim
// rather than clamp height into the AV coefficients.
double
GroundPathLossDb(A2gScenario s, A2gLink link, double d3dM, double fcGHz, double hUtM)
{
    const double c = 299792458.0;
    const double fcHz = fcGHz * 1e9;
    const double L = std::log10(d3dM);
    const double hBs = (s == A2gScenario::UMa_AV) ? 25.0
                       : (s == A2gScenario::UMi_AV) ? 10.0
                                                    : 35.0;
    switch (s)
    {
    case A2gScenario::UMa_AV:
    {
        const double dBp = 4.0 * hBs * hUtM * fcHz / c;
        double losDb = (d3dM < dBp)
                           ? 28.0 + 22.0 * L + 20.0 * std::log10(fcGHz)
                           : 28.0 + 40.0 * L + 20.0 * std::log10(fcGHz) -
                                 9.0 * std::log10(dBp * dBp + (hBs - hUtM) * (hBs - hUtM));
        if (link == A2gLink::LOS)
        {
            return losDb;
        }
        const double nlos = 13.54 + 39.08 * L + 20.0 * std::log10(fcGHz) - 0.6 * (hUtM - 1.5);
        return std::max(losDb, nlos);
    }
    case A2gScenario::UMi_AV:
    {
        const double dBp = 4.0 * hBs * hUtM * fcHz / c;
        double losDb = (d3dM < dBp)
                           ? 32.4 + 21.0 * L + 20.0 * std::log10(fcGHz)
                           : 32.4 + 40.0 * L + 20.0 * std::log10(fcGHz) -
                                 9.5 * std::log10(dBp * dBp + (hBs - hUtM) * (hBs - hUtM));
        if (link == A2gLink::LOS)
        {
            return losDb;
        }
        const double nlos = 35.3 * L + 22.4 + 21.3 * std::log10(fcGHz) - 0.3 * (hUtM - 1.5);
        return std::max(losDb, nlos);
    }
    case A2gScenario::RMa_AV:
    {
        const double W = 20.0;    // avg street width (m)
        const double hB = 5.0;    // avg building height (m)
        const double dBp = 2.0 * M_PI * hBs * hUtM * fcHz / c;
        auto rmaLos = [&](double d) {
            const double ld = std::log10(d);
            return 20.0 * std::log10(40.0 * M_PI * d * fcGHz / 3.0) +
                   std::min(0.03 * std::pow(hB, 1.72), 10.0) * ld -
                   std::min(0.044 * std::pow(hB, 1.72), 14.77) + 0.002 * std::log10(hB) * d;
        };
        double losDb = (d3dM < dBp) ? rmaLos(d3dM) : rmaLos(dBp) + 40.0 * std::log10(d3dM / dBp);
        if (link == A2gLink::LOS)
        {
            return losDb;
        }
        const double nlos =
            161.04 - 7.1 * std::log10(W) + 7.5 * std::log10(hB) -
            (24.37 - 3.7 * (hB / hBs) * (hB / hBs)) * std::log10(hBs) +
            (43.42 - 3.1 * std::log10(hBs)) * (L - 3.0) + 20.0 * std::log10(fcGHz) -
            (3.2 * std::pow(std::log10(11.75 * hUtM), 2.0) - 4.97);
        return std::max(losDb, nlos);
    }
    }
    return 0.0;
}

} // namespace

bool
A2gChannelTr36777::WasLastCallAboveValidatedHeight()
{
    return g_lastAboveValidatedHeight;
}

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
    // TR 36.777 Table B-1.1/B-1.2: the aerial-vehicle formulas are valid only
    // above the scenario height threshold. Below it, delegate to the TR 38.901
    // ground model (do NOT clamp height into the AV coefficients).
    if (hUtM <= AerialHeightThreshold(scenario))
    {
        g_lastAboveValidatedHeight = false;
        return GroundPathLossDb(scenario, link, d3dM, fcGHz, hUtM);
    }

    // SAGIN-8: the upper bound of the validation band was never checked.
    //
    // TR 36.777 Table B-1.1/B-1.2 fit these coefficients over 1.5 m to 300 m.
    // The model guarded only the LOWER threshold, so a HAPS at 20 km, or a
    // SaginA2gPropagationLossModel that derives h_UT as max(pa.z, pb.z) on an
    // unconfigured link, evaluated 300 m coefficients two orders of magnitude
    // outside the data behind them - silently, with no warning and no way for
    // the caller to tell a prediction from an extrapolation.
    //
    // The result is NOT clamped. Clamping would return the 300 m answer for a
    // 20 km link, which is a different wrong number wearing a guard. The
    // extrapolation is returned and declared, so a caller can refuse it.
    g_lastAboveValidatedHeight = (hUtM > kMaxValidatedHeightM);
    if (g_lastAboveValidatedHeight)
    {
        NS_LOG_WARN("h_UT=" << hUtM << " m is above the TR 36.777 aerial-vehicle validation "
                    << "band (" << A2gChannelTr36777::kMaxValidatedHeightM << " m); the returned path "
                    << "loss is an EXTRAPOLATION of coefficients fitted to 1.5-300 m, not a "
                    << "TR 36.777 prediction. No HAPS-band channel exists in this module; see "
                    << "WasLastCallAboveValidatedHeight().");
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
A2gChannelTr36777::ShadowFadingSigmaDb(A2gScenario scenario, A2gLink link, double hUtM)
{
    // TR 36.777 v15.0.0 Table B-1.2 shadow-fading standard deviations.
    const double h = std::max(hUtM, 1.0);
    switch (scenario)
    {
    case A2gScenario::UMa_AV:
        return (link == A2gLink::LOS) ? 4.64 * std::exp(-0.0066 * h) : 6.0;
    case A2gScenario::RMa_AV:
        return (link == A2gLink::LOS) ? 4.2 * std::exp(-0.0046 * h) : 8.0;
    case A2gScenario::UMi_AV:
        return (link == A2gLink::LOS) ? 4.64 * std::exp(-0.0066 * h) : 8.0;
    }
    return 6.0;
}

double
A2gChannelTr36777::LosProbability(A2gScenario scenario, double d2dM, double hUtM)
{
    // TR 36.777 v15.0.0 Table B-1.1 (height-dependent LOS probability).
    if (d2dM < 1.0)
    {
        d2dM = 1.0;
    }

    // Aerial-vehicle sigmoid: P = 1 for d2D <= d1, else the spec expression.
    auto avSigmoid = [&](double d1, double p1) {
        if (d2dM <= d1)
        {
            return 1.0;
        }
        const double r = d1 / d2dM;
        return r + std::exp(-d2dM / p1) * (1.0 - r);
    };

    const double logh = std::log10(std::max(hUtM, 1.0));

    switch (scenario)
    {
    case A2gScenario::UMa_AV:
    {
        // Below 22.5 m: TR 38.901 UMa ground LOS probability (d1=18, p1=63).
        if (hUtM <= 22.5)
        {
            return (d2dM <= 18.0)
                       ? 1.0
                       : 18.0 / d2dM + std::exp(-d2dM / 63.0) * (1.0 - 18.0 / d2dM);
        }
        // 22.5 m < h <= 100 m: sigmoid; above 100 m the eNB is fully below the
        // UAV so P_LOS = 1 (TR 36.777 UMa-AV upper threshold).
        if (hUtM > 100.0)
        {
            return 1.0;
        }
        const double d1 = std::max(460.0 * logh - 700.0, 18.0);
        const double p1 = 4300.0 * logh - 3800.0;
        return avSigmoid(d1, p1);
    }
    case A2gScenario::RMa_AV:
    {
        // Below 10 m: TR 38.901 RMa ground LOS probability.
        if (hUtM <= 10.0)
        {
            return (d2dM <= 10.0) ? 1.0 : std::exp(-(d2dM - 10.0) / 1000.0);
        }
        // 10 m < h <= 40 m: sigmoid; above 40 m P_LOS = 1 (RMa-AV upper thr).
        if (hUtM > 40.0)
        {
            return 1.0;
        }
        const double d1 = std::max(1350.8 * logh - 1602.0, 18.0);
        const double p1 = std::max(15021.0 * logh - 16053.0, 1000.0);
        return avSigmoid(d1, p1);
    }
    case A2gScenario::UMi_AV:
    {
        // Below 22.5 m: TR 38.901 UMi ground LOS probability (d1=18, p1=36).
        if (hUtM <= 22.5)
        {
            return (d2dM <= 18.0)
                       ? 1.0
                       : 18.0 / d2dM + std::exp(-d2dM / 36.0) * (1.0 - 18.0 / d2dM);
        }
        // 22.5 m < h <= 300 m: sigmoid retained all the way to 300 m (no upper
        // saturation to 1 — the eNB is below rooftop in UMi).
        const double lh = std::log10(std::min(hUtM, 300.0));
        const double d1 = std::max(294.05 * lh - 432.94, 18.0);
        const double p1 = 233.98 * lh - 0.95;
        return avSigmoid(d1, p1);
    }
    }
    return 1.0;
}

} // namespace ns3
