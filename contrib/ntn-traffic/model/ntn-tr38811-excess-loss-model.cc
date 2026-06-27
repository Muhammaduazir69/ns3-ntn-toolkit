// SPDX-License-Identifier: GPL-2.0-only
//
// See ntn-tr38811-excess-loss-model.h. Math ported verbatim (same constants)
// from ntn-cho/model/ntn-measurement-model.cc lines ~124-181, where it had been
// stranded in an Object-based oracle that never reached the radio.

#include "ntn-tr38811-excess-loss-model.h"

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
            .AddAttribute("EnableScintillation",
                          "Apply ITU-R P.618 tropospheric scintillation.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&Ntn38811ExcessLossModel::m_enableScintillation),
                          MakeBooleanChecker())
            .AddAttribute("EnableFastFading",
                          "Apply TR 38.811 §6.7/6.9 Rician small-scale fading "
                          "(elevation-dependent K-factor).",
                          BooleanValue(true),
                          MakeBooleanAccessor(&Ntn38811ExcessLossModel::m_enableFastFading),
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
    const double atmosphericDb = zenithGasDb / sinElev;

    // --- Clutter loss (TR 38.811 §6.6.2): the spec sets CL = 0 dB in LOS, and
    //     this model runs the always-LOS satellite link, so clutter is 0. (The
    //     Table 6.6.2-x NLOS clutter values apply only under an NLOS condition.) ---
    const double clutterDb = 0.0;

    // --- Shadow fading: log-normal, sigma from TR 38.811 Table 6.6.2-x S-band
    //     LOS (per scenario, interpolated by elevation). Zero-mean. ---
    double shadowDb = 0.0;
    if (m_enableShadowFading)
    {
        shadowDb = ShadowSigmaDb(elevDeg) * m_sfRng->GetValue();
    }

    // --- Tropospheric scintillation (ITU-R P.618): sigma grows with sqrt(freq)
    //     and shrinks with elevation^(-11/12). ---
    double scintDb = 0.0;
    if (m_enableScintillation)
    {
        const double fGHz = m_freqHz / 1.0e9;
        const double sigmaXi =
            0.5 * std::sqrt(fGHz / 2.0) / std::pow(sinElev, 11.0 / 12.0);
        scintDb = sigmaXi * m_scintRng->GetValue();
    }

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
