// SPDX-License-Identifier: GPL-2.0-only
//
// See ntn-tr38811-excess-loss-model.h. Math ported verbatim (same constants)
// from ntn-cho/model/ntn-measurement-model.cc lines ~124-181, where it had been
// stranded in an Object-based oracle that never reached the radio.

#include "ntn-tr38811-excess-loss-model.h"

#include <algorithm>

#include "ns3/simulator.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("Ntn38811ExcessLossModel");
NS_OBJECT_ENSURE_REGISTERED(Ntn38811ExcessLossModel);

// Mean Earth radius (m) — used to discriminate ECEF vs local-ENU coordinates.
static constexpr double EARTH_RADIUS_M = 6371000.0;

TypeId
Ntn38811ExcessLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::Ntn38811ExcessLossModel")
            .SetParent<PropagationLossModel>()
            .SetGroupName("NtnTraffic")
            .AddConstructor<Ntn38811ExcessLossModel>()
            .AddAttribute("Scenario",
                          "TR 38.811 deployment scenario (clutter + shadow-fading sigma).",
                          EnumValue(Suburban),
                          MakeEnumAccessor<NtnScenario>(&Ntn38811ExcessLossModel::m_scenario),
                          MakeEnumChecker(DenseUrban,
                                          "DenseUrban",
                                          Urban,
                                          "Urban",
                                          Suburban,
                                          "Suburban",
                                          Rural,
                                          "Rural"))
            .AddAttribute("CarrierFrequencyHz",
                          "Carrier frequency (Hz); selects the P.676 gas-absorption band.",
                          DoubleValue(2.0e9),
                          MakeDoubleAccessor(&Ntn38811ExcessLossModel::m_freqHz),
                          MakeDoubleChecker<double>())
            .AddAttribute("EnableShadowFading",
                          "Apply TR 38.811 Table 6.6.2-1 elevation-dependent shadow fading.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&Ntn38811ExcessLossModel::m_enableShadowFading),
                          MakeBooleanChecker())
            .AddAttribute("Tr38811Scintillation",
                          "C3 (Wave 1): replace the legacy P.618 tropospheric draw with the "
                          "TR 38.811 clause 6.6.6 fade term the ns-3 NTN models use: below "
                          "6 GHz the ionospheric term 6.22 f_GHz^-1.5 dB (clause 6.6.6.1.4; "
                          "1.93 dB at 2.185 GHz), above it the tropospheric Table 6.6.6.2.1-1 "
                          "value at the quantized elevation. Default false (legacy).",
                          BooleanValue(false),
                          MakeBooleanAccessor(&Ntn38811ExcessLossModel::m_tr38811Scint),
                          MakeBooleanChecker())
            .AddAttribute("EnableScintillation",
                          "Apply ITU-R P.618 tropospheric scintillation.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&Ntn38811ExcessLossModel::m_enableScintillation),
                          MakeBooleanChecker())
            .AddAttribute("EnableFastFading",
                          "Apply TR 38.811 §6.7/6.9 Rician small-scale fading "
                          "(elevation-dependent K-factor). DEFAULT OFF (gap S6): both radio "
                          "backends keep the 3GPP phased-array spectrum model, which already "
                          "applies small-scale fading on the same link - enabling this too "
                          "multiplies two independent fast-fading processes onto one link. "
                          "Enable only when no 3GPP spectrum model is in the path. NT-08: this "
                          "text used to say 'with Doppler'; the channel is frozen at t=0 unless "
                          "NtnRealStackHelper::SetChannelUpdatePeriod is set, so no Doppler "
                          "process runs by default.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&Ntn38811ExcessLossModel::m_enableFastFading),
                          MakeBooleanChecker())
            .AddAttribute("ShadowCorrelationDistanceM",
                          "S6: distance (m) the ground node must move before the shadow-fading "
                          "sample is redrawn (TR 38.811 §6.6.2 correlation distance).",
                          DoubleValue(50.0),
                          MakeDoubleAccessor(&Ntn38811ExcessLossModel::m_sfCorrDistanceM),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("ShadowCorrelationElevationDeg",
                          "S6: elevation change (deg) that also decorrelates the shadowing. For "
                          "a static UE the geometry changes because the satellite moves, so "
                          "displacement alone never decorrelates a LEO pass.",
                          DoubleValue(5.0),
                          MakeDoubleAccessor(&Ntn38811ExcessLossModel::m_sfCorrElevDeg),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("ScintillationCoherenceS",
                          "S6: ITU-R P.618 scintillation coherence time (s); the sample is held "
                          "for this long instead of being redrawn per transport block.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&Ntn38811ExcessLossModel::m_scintCoherenceS),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("P618Scintillation",
                          "Wave 2 core: ITU-R P.618-13 2.4.1 tropospheric scintillation "
                          "(sigma_ref = 3.6e-3 + 1e-4 N_wet, effective path length with "
                          "h_L = 1000 m, aperture averaging g(x), sigma = sigma_ref f^(7/12) "
                          "g(x) / sin(el)^1.2) as a zero-mean Gaussian AR(1) process in "
                          "simulated time with time constant ScintillationCoherenceS. "
                          "Replaces the legacy draw (0.5 sqrt(f/2)/sin(el)^(11/12), redrawn "
                          "independently every coherence time). Default false (legacy). "
                          "Tr38811Scintillation takes precedence when both are set.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&Ntn38811ExcessLossModel::m_p618Scint),
                          MakeBooleanChecker())
            .AddAttribute("P618WetRefractivity",
                          "P.618-13 2.4.1 step 1: wet term of the surface refractivity N_wet "
                          "(ppm), from ITU-R P.453 for the site (median about 40-50 at mid "
                          "latitudes).",
                          DoubleValue(42.5),
                          MakeDoubleAccessor(&Ntn38811ExcessLossModel::m_p618NwetPpm),
                          MakeDoubleChecker<double>(0.0, 200.0))
            .AddAttribute("P618AntennaDiameterM",
                          "P.618-13 2.4.1 step 4: ground antenna diameter (m) for the "
                          "aperture-averaging factor; 0 = point receiver (handheld), g = 1.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&Ntn38811ExcessLossModel::m_p618DiameterM),
                          MakeDoubleChecker<double>(0.0, 100.0))
            .AddAttribute("P618AntennaEfficiency",
                          "P.618-13 2.4.1 step 4: antenna efficiency eta (0.5 if unknown).",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&Ntn38811ExcessLossModel::m_p618Efficiency),
                          MakeDoubleChecker<double>(0.01, 1.0))
            .AddAttribute("EnableGasAbsorption",
                          "Apply the band-zenith P.676 gas term / sin(el). Turn off when a "
                          "spectrum plugin (per-RB P.676) already carries gas, to avoid "
                          "counting it twice.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&Ntn38811ExcessLossModel::m_enableGas),
                          MakeBooleanChecker())
            .AddAttribute("SkipSpaceToSpace",
                          "Return no excess loss (gas, shadowing, scintillation, fast "
                          "fading) on a link whose two ends are both above 100 km: an ISL "
                          "or sat-sat interference path crosses no troposphere, and the "
                          "Earth-space terms evaluated at its near-zero 'elevation' reach "
                          "tens of dB. Default false because skipping those links also "
                          "skips their random draws, which shifts the shared random "
                          "streams of every other link in existing runs.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&Ntn38811ExcessLossModel::m_skipSpaceToSpace),
                          MakeBooleanChecker());
    return tid;
}

Ntn38811ExcessLossModel::Ntn38811ExcessLossModel()
{
    m_sfRng = CreateObject<NormalRandomVariable>();
    m_sfRng->SetAttribute("Mean", DoubleValue(0.0));
    m_sfRng->SetAttribute("Variance", DoubleValue(1.0));
    // Bound the tail at 3 sigma so a single packet cannot see an unphysical fade.
    m_sfRng->SetAttribute("Bound", DoubleValue(3.0));
    m_scintRng = CreateObject<NormalRandomVariable>();
    m_scintRng->SetAttribute("Mean", DoubleValue(0.0));
    m_scintRng->SetAttribute("Variance", DoubleValue(1.0));
    m_scintRng->SetAttribute("Bound", DoubleValue(3.0));
    // Rician small-scale fading scatter components: unit normals scaled in the
    // sampler. Bounded at 4 sigma so a single packet cannot see an unphysical fade.
    m_fadeIRng = CreateObject<NormalRandomVariable>();
    m_fadeIRng->SetAttribute("Mean", DoubleValue(0.0));
    m_fadeIRng->SetAttribute("Variance", DoubleValue(1.0));
    m_fadeIRng->SetAttribute("Bound", DoubleValue(4.0));
    m_fadeQRng = CreateObject<NormalRandomVariable>();
    m_fadeQRng->SetAttribute("Mean", DoubleValue(0.0));
    m_fadeQRng->SetAttribute("Variance", DoubleValue(1.0));
    m_fadeQRng->SetAttribute("Bound", DoubleValue(4.0));
}

Ntn38811ExcessLossModel::~Ntn38811ExcessLossModel()
{
}

double
Ntn38811ExcessLossModel::ElevationDeg(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
{
    Vector pa = a->GetPosition();
    Vector pb = b->GetPosition();
    // The ground node is the one closer to the Earth centre / lower altitude.
    double ra = std::sqrt(pa.x * pa.x + pa.y * pa.y + pa.z * pa.z);
    double rb = std::sqrt(pb.x * pb.x + pb.y * pb.y + pb.z * pb.z);
    Vector ground = (ra <= rb) ? pa : pb;
    Vector sat = (ra <= rb) ? pb : pa;
    double rGround = std::min(ra, rb);

    // LOS vector ground -> satellite.
    Vector los(sat.x - ground.x, sat.y - ground.y, sat.z - ground.z);
    double d = std::sqrt(los.x * los.x + los.y * los.y + los.z * los.z);
    if (d < 1.0)
    {
        return 90.0;
    }

    // Local "up" at the ground node. ECEF if the radius is near/above an Earth
    // radius; otherwise treat the frame as local ENU (flat-Earth, up = +z).
    Vector up;
    if (rGround > 0.5 * EARTH_RADIUS_M)
    {
        up = Vector(ground.x / rGround, ground.y / rGround, ground.z / rGround);
    }
    else
    {
        up = Vector(0.0, 0.0, 1.0);
    }

    double sinElev = (los.x * up.x + los.y * up.y + los.z * up.z) / d;
    sinElev = std::max(-1.0, std::min(1.0, sinElev));
    return std::asin(sinElev) * 180.0 / M_PI;
}

// --- TR 38.811 S-band LOS tables, elevation 10..90 deg (10-deg step). ---
// Shadow-fading sigma (dB): Tables 6.6.2-1 (Dense Urban), 6.6.2-2 (Urban),
// 6.6.2-3 (Suburban AND Rural share one table). LOS column.
static const double kSigmaSf[4][9] = {
    {3.5, 3.4, 2.9, 3.0, 3.1, 2.7, 2.5, 2.3, 1.2},          // Dense Urban
    {4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0},          // Urban
    {1.79, 1.14, 1.14, 0.92, 1.42, 1.56, 0.85, 0.72, 0.72}, // Suburban
    {1.79, 1.14, 1.14, 0.92, 1.42, 1.56, 0.85, 0.72, 0.72}, // Rural (= Suburban table)
};
// Rician K-factor mean mu_K (dB): Tables 6.7.2-1a/3a/5a/7a, LOS, S-band.
static const double kRicianK[4][9] = {
    {4.4, 9.0, 9.3, 7.9, 7.4, 7.0, 6.9, 6.5, 6.8},                   // Dense Urban
    {31.83, 18.78, 10.49, 7.46, 6.52, 5.47, 4.54, 4.03, 3.68},       // Urban
    {11.40, 19.45, 20.80, 21.20, 21.60, 19.75, 12.00, 12.85, 12.85}, // Suburban
    {24.72, 12.31, 8.05, 6.21, 5.04, 4.42, 3.92, 3.65, 3.59},        // Rural
};

double
Ntn38811ExcessLossModel::InterpByElevation(const double (&table)[9], double elevDeg) const
{
    const double e = std::max(10.0, std::min(90.0, elevDeg));
    const double idxF = (e - 10.0) / 10.0; // 0..8
    const int i0 = static_cast<int>(std::floor(idxF));
    const int i1 = std::min(i0 + 1, 8);
    const double frac = idxF - i0;
    return table[i0] * (1.0 - frac) + table[i1] * frac;
}

double
Ntn38811ExcessLossModel::ShadowSigmaDb(double elevDeg) const
{
    return InterpByElevation(kSigmaSf[static_cast<int>(m_scenario)], elevDeg);
}

double
Ntn38811ExcessLossModel::RicianKdB(double elevDeg) const
{
    return InterpByElevation(kRicianK[static_cast<int>(m_scenario)], elevDeg);
}

double
Ntn38811ExcessLossModel::DoCalcRxPower(double txPowerDbm,
                                       Ptr<MobilityModel> a,
                                       Ptr<MobilityModel> b) const
{
    if (m_skipSpaceToSpace && IsSpaceToSpace(a->GetPosition(), b->GetPosition()))
    {
        return txPowerDbm; // no troposphere on the path
    }
    const double elevDeg = ElevationDeg(a, b);
    // Clamp at 5 deg minimum elevation for the slant-path scaling (TR 38.811
    // does not define geometry below the horizon mask).
    const double elevRad = std::max(elevDeg, 5.0) * M_PI / 180.0;
    const double sinElev = std::sin(elevRad);

    // --- Atmospheric gas absorption (ITU-R P.676), zenith value by band,
    //     scaled by 1/sin(elev) for the slant path. ---
    const double zenithGasDb = (m_freqHz < 6.0e9)    ? 0.04   // S/L-band
                               : (m_freqHz < 30.0e9) ? 0.10   // C/Ku-band
                                                     : 0.30;  // Ka-band+
    const double atmosphericDb = m_enableGas ? zenithGasDb / sinElev : 0.0;

    // --- Clutter loss (TR 38.811 §6.6.2): the spec sets CL = 0 dB in LOS, and
    //     this model runs the always-LOS satellite link, so clutter is 0. (The
    //     Table 6.6.2-x NLOS clutter values apply only under an NLOS condition.) ---
    const double clutterDb = 0.0;

    // --- S6: correlated large-scale parameters -----------------------------
    // Shadow fading and scintillation are CORRELATED processes; re-drawing them
    // per call (per transport block) made them white noise. Look up / refresh a
    // per-node-pair cache instead. Key on the mobility-model pointers, ordered
    // so the DL and UL of the same link share one state.
    const uintptr_t ka = reinterpret_cast<uintptr_t>(PeekPointer(a));
    const uintptr_t kb = reinterpret_cast<uintptr_t>(PeekPointer(b));
    const auto key = std::minmax(ka, kb);
    LargeScaleState& st = m_lssCache[{key.first, key.second}];

    // The ground end is whichever node is lower; that is the one whose
    // displacement decorrelates the shadowing per TR 38.811 §6.6.2.
    const Vector pa = a->GetPosition();
    const Vector pb = b->GetPosition();
    const double ra = std::sqrt(pa.x * pa.x + pa.y * pa.y + pa.z * pa.z);
    const double rb = std::sqrt(pb.x * pb.x + pb.y * pb.y + pb.z * pb.z);
    const Vector groundPos = (ra <= rb) ? pa : pb;

    const double moved = std::sqrt(std::pow(groundPos.x - st.lastPos.x, 2) +
                                   std::pow(groundPos.y - st.lastPos.y, 2) +
                                   std::pow(groundPos.z - st.lastPos.z, 2));
    const double elevMoved = std::fabs(elevDeg - st.lastElevDeg);

    // --- Shadow fading: log-normal, sigma from TR 38.811 Table 6.6.2-x S-band
    //     LOS (per scenario, interpolated by elevation). Zero-mean.
    //
    // NT-10: this used to hold one sample until a threshold was crossed and then
    // draw a fully INDEPENDENT replacement, which put a discontinuity inside a
    // single transport block - a measured +1.205 -> -2.496 dB, a 3.7 dB step -
    // where TR 38.811 6.6.2 describes a spatially correlated process. A study
    // that keys on the SINR derivative (A3 hysteresis, time-to-trigger tuning)
    // reads that step as physics.
    //
    // It is now the AR(1) recursion ns-3's own 3GPP model uses
    // (ThreeGppPropagationLossModel::GetShadowing):
    //     new = rho*old + sqrt(1 - rho^2)*N(0,1),   rho = exp(-d/d_corr)
    // generalized to the two things that decorrelate an NTN link. Displacement
    // alone never decorrelates a LEO pass for a static terminal, because the
    // geometry changes at the satellite end; so elevation change enters as its
    // own decorrelation driver and the two add in the exponent. With one driver
    // held still this reduces exactly to the terrestrial form.
    //
    // The state is unit-variance and sigma is applied at read time, so the
    // marginal variance stays sigma(elev)^2 as sigma moves with elevation.
    if (m_enableShadowFading)
    {
        if (!st.valid)
        {
            st.sfUnit = m_sfRng->GetValue();
            st.valid = true;
        }
        else
        {
            const double dCorr = std::max(m_sfCorrDistanceM, 1e-9);
            const double eCorr = std::max(m_sfCorrElevDeg, 1e-9);
            const double x = moved / dCorr + elevMoved / eCorr;
            const double rho = std::exp(-x);
            st.sfUnit = rho * st.sfUnit +
                        std::sqrt(std::max(0.0, 1.0 - rho * rho)) * m_sfRng->GetValue();
        }
        st.shadowDb = ShadowSigmaDb(elevDeg) * st.sfUnit;
        st.lastPos = groundPos;
        st.lastElevDeg = elevDeg;
    }
    else
    {
        st.shadowDb = 0.0;
    }
    const double shadowDb = st.shadowDb;

    // --- Tropospheric scintillation (ITU-R P.618): sigma grows with sqrt(freq)
    //     and shrinks with elevation^(-11/12). Coherence is seconds-scale, so
    //     hold the sample for m_scintCoherenceS rather than redrawing per TB. ---
    if (m_enableScintillation && m_tr38811Scint)
    {
        // TR 38.811 clause 6.6.6.1.4 (ionospheric, f < 6 GHz) and Table
        // 6.6.6.2.1-1 (tropospheric, f >= 6 GHz, elevation quantized to 10 deg,
        // clamped at 10 deg), the fade terms of the mainline ThreeGppNTN models.
        const double fGHz = m_freqHz / 1.0e9;
        if (fGHz < 6.0)
        {
            st.scintDb = 6.22 / std::pow(fGHz, 1.5);
        }
        else
        {
            static const double kTropo[] = {1.08, 0.48, 0.30, 0.22, 0.17, 0.13, 0.12, 0.12, 0.12};
            const int q = (elevDeg < 10.0) ? 10 : static_cast<int>(std::round(elevDeg / 10.0) * 10);
            st.scintDb = kTropo[std::clamp(q / 10 - 1, 0, 8)];
        }
    }
    else if (m_enableScintillation && m_p618Scint)
    {
        // P.618-13 2.4.1: zero-mean Gaussian with the recommendation's sigma,
        // correlated in time: x <- rho x + sqrt(1 - rho^2) N(0,1),
        // rho = exp(-dt / Tc), Tc = ScintillationCoherenceS. Unit-variance
        // state scaled at read time, so the marginal stays sigma(el)^2 as the
        // elevation moves.
        const Time now = Simulator::Now();
        if (!st.scintValid)
        {
            st.scintUnit = m_scintRng->GetValue();
            st.scintValid = true;
            st.lastScintTime = now;
        }
        else if (now > st.lastScintTime)
        {
            const double dt = (now - st.lastScintTime).GetSeconds();
            const double rho = (m_scintCoherenceS > 0.0) ? std::exp(-dt / m_scintCoherenceS) : 0.0;
            st.scintUnit = rho * st.scintUnit +
                           std::sqrt(std::max(0.0, 1.0 - rho * rho)) * m_scintRng->GetValue();
            st.lastScintTime = now;
        }
        st.scintDb = P618ScintillationSigmaDb(m_freqHz,
                                              elevDeg,
                                              m_p618NwetPpm,
                                              m_p618DiameterM,
                                              m_p618Efficiency) *
                     st.scintUnit;
    }
    else if (m_enableScintillation)
    {
        const Time now = Simulator::Now();
        if ((now - st.lastScintTime).GetSeconds() >= m_scintCoherenceS)
        {
            const double fGHz = m_freqHz / 1.0e9;
            const double sigmaXi =
                0.5 * std::sqrt(fGHz / 2.0) / std::pow(sinElev, 11.0 / 12.0);
            st.scintDb = sigmaXi * m_scintRng->GetValue();
            st.lastScintTime = now;
        }
    }
    else
    {
        st.scintDb = 0.0;
    }
    const double scintDb = st.scintDb;

    // --- Small-scale (fast) fading: TR 38.811 §6.7/§6.9 Rician process with the
    //     elevation-dependent K-factor from Table 6.7.2-Xa. Draw the complex
    //     fading gain (LOS specular s + scattered (X,Y)), normalised to unit mean
    //     power E[r^2]=1; the loss is -10log10(r^2) (a deep fade is positive loss).
    //     This is a single-tap (flat-fading) realisation of the NTN-TDL; the full
    //     multi-tap frequency-selective TDL (Table 6.9.2-x) is the remaining
    //     refinement (see SCOPE_AND_LIMITATIONS A1). ---
    double fadeLossDb = 0.0;
    if (m_enableFastFading)
    {
        const double kdB = RicianKdB(elevDeg);
        const double k = std::pow(10.0, kdB / 10.0);
        const double sigma = std::sqrt(1.0 / (2.0 * (k + 1.0)));
        const double s = std::sqrt(k / (k + 1.0));
        const double xi = s + sigma * m_fadeIRng->GetValue();
        const double yq = sigma * m_fadeQRng->GetValue();
        const double r2 = xi * xi + yq * yq; // Rician power gain, E[r2] = 1
        fadeLossDb = -10.0 * std::log10(std::max(r2, 1e-6));
        fadeLossDb = std::max(-10.0, std::min(20.0, fadeLossDb)); // clamp tails
    }

    const double excessDb = atmosphericDb + clutterDb + shadowDb + scintDb + fadeLossDb;
    NS_LOG_DEBUG("elev=" << elevDeg << "deg gas=" << atmosphericDb << " sf=" << shadowDb
                         << " scint=" << scintDb << " fade=" << fadeLossDb
                         << " excess=" << excessDb << "dB");
    return txPowerDbm - excessDb;
}

double
Ntn38811ExcessLossModel::P618ScintillationSigmaDb(double freqHz,
                                                  double elevDeg,
                                                  double nWetPpm,
                                                  double diameterM,
                                                  double efficiency)
{
    // ITU-R P.618-13 section 2.4.1 (valid 4-55 GHz, elevation >= 5 deg; used
    // here as the documented extrapolation outside that range).
    const double fGHz = freqHz / 1.0e9;
    const double el = std::max(elevDeg, 5.0) * M_PI / 180.0;
    const double sinEl = std::sin(el);
    // Step 2: sigma_ref = 3.6e-3 + 1e-4 N_wet (dB).
    const double sigmaRef = 3.6e-3 + 1.0e-4 * nWetPpm;
    // Step 3: effective path length, h_L = 1000 m.
    constexpr double hL = 1000.0;
    const double L = 2.0 * hL / (std::sqrt(sinEl * sinEl + 2.35e-4) + sinEl);
    // Step 4: effective diameter.
    const double dEff = std::sqrt(efficiency) * diameterM;
    // Step 5: antenna averaging factor g(x), x = 1.22 D_eff^2 f / L.
    double g = 1.0;
    if (dEff > 0.0)
    {
        const double x = 1.22 * dEff * dEff * (fGHz / L);
        const double arg = 3.86 * std::pow(x * x + 1.0, 11.0 / 12.0) *
                               std::sin(11.0 / 6.0 * std::atan(1.0 / x)) -
                           7.08 * std::pow(x, 5.0 / 6.0);
        // Above x = 7.0 the recommendation says the prediction is not valid
        // (the argument turns negative); clamp at full averaging.
        g = (arg > 0.0) ? std::sqrt(arg) : 0.0;
    }
    else
    {
        // x = 0: g = sqrt(3.86 sin(11 pi / 12)) = 0.99954.
        g = std::sqrt(3.86 * std::sin(11.0 / 6.0 * M_PI / 2.0));
    }
    // Step 6: sigma = sigma_ref f^(7/12) g(x) / sin(el)^1.2.
    return sigmaRef * std::pow(fGHz, 7.0 / 12.0) * g / std::pow(sinEl, 1.2);
}

bool
Ntn38811ExcessLossModel::IsSpaceToSpace(const Vector& pa, const Vector& pb)
{
    auto altM = [](const Vector& p) {
        const double r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        // ECEF when the radius is near/above an Earth radius, else local ENU
        // (same frame test as ElevationDeg).
        return (r > 0.5 * EARTH_RADIUS_M) ? (r - EARTH_RADIUS_M) : p.z;
    };
    return altM(pa) > 100e3 && altM(pb) > 100e3;
}

int64_t
Ntn38811ExcessLossModel::DoAssignStreams(int64_t stream)
{
    m_sfRng->SetStream(stream);
    m_scintRng->SetStream(stream + 1);
    m_fadeIRng->SetStream(stream + 2);
    m_fadeQRng->SetStream(stream + 3);
    return 4;
}

} // namespace ns3
