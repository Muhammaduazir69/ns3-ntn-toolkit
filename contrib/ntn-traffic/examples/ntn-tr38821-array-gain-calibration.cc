/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-tr38821-array-gain-calibration — reviewer experiment R2.8: is the
// satellite array gain used in the TR 38.821 calibration ANGLE-DEPENDENT?
//
// The concern. examples/ntn-tr38821-calibration.cc reports a ~21.5 dB constant
// difference between the MEASURED CNR and the TR 38.821 Sec. 6.1.3.1 closed
// form, and attributes it to the satellite array gain that the closed-form
// parabolic-antenna budget does not carry. A constant offset invites the
// reading that the array gain was hard-wired as a scalar, whereas a real array
// gain varies with the off-boresight angle.
//
// The answer this example demonstrates. The angle dependence IS modelled — by
// NtnSatBeamGainModel, the TR 38.811 Sec. 6.4.1 normalized circular-aperture
// (Airy) pattern G(theta)/Gmax = 4|J1(u)/u|^2 with u pinned by the TR 38.821
// Set-1 LEO-600 S-band 3 dB beamwidth (4.4127 deg). It is simply not EXERCISED
// by the original calibration, because that scenario runs a STEERED spot beam:
// the boresight tracks the terminal, so theta == 0 for the whole pass and the
// pattern contributes exactly 0 dB by construction. The offset is therefore
// constant for a physical reason (a tracking beam), not because a constant was
// substituted for a function.
//
// The experiment. The same real LEO-600 pass (SGP4 element, S-band DL, EIRP
// 48.77 dBW, G/T -31.62 dB/K, 30 MHz) is run TWICE against the same measured
// mmwave PHY, changing ONE thing — where the beam points:
//
//   Regime A (steered): BoresightMode::TrackUe. Expect theta == 0 for every
//     sample and residual = measured - TR closed form ~ constant. This
//     reproduces the published calibration and IS the 21.5 dB offset.
//
//   Regime B (fixed):  the boresight is pinned (default: a fixed ground point
//     offset along the ground track; --boresightMode=nadir pins it to the
//     satellite's nadir instead, the canonical TR 38.821 fixed-beam-layout
//     case). The UE now traverses the lobe as the satellite passes, so theta
//     sweeps a real range and the residual must TRACK the analytic pattern
//     through the mainlobe, the first null and the sidelobes.
//
// The fidelity metric is a PAIRED difference. Both regimes sample identical
// geometry at identical times, so
//     pattern_err(t) = [residual_B(t) - residual_A(t)] - G_analytic(theta(t))
// cancels the elevation-dependent link budget (FSPL, TR 38.811 atmospheric and
// clutter terms) and the constant array-gain offset alike, leaving ONLY the
// angular term. G_analytic comes from NtnSatBeamGainModel::GainDbAtThetaDeg(),
// a pure closed form that touches no simulation state — so the same column can
// be recomputed by a reviewer from the pattern equation alone.
//
// Residual scatter of ~1.4 dB is expected and honest: it is the measured
// plane's own small-scale fading (see the tracking-gate note in
// ntn-tr38821-calibration.cc), and the two regimes draw independent fading
// realisations, so the paired difference carries ~2 dB of it against a ~40 dB
// pattern dynamic range.
//
// Outputs (--outputDir, default ntn-tr38821-array-gain-output):
//   array_gain_calibration.csv  per-sample, both regimes
//   array_gain_summary.csv      per-regime aggregates + the pattern-fidelity error
//
// Quick run:  --simSeconds=60 --samplePeriodS=1.0
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-sat-beam-gain-model.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <system_error>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnTr38821ArrayGainCalibration");

namespace
{
constexpr double kC = 299792458.0;
// TR 38.821 Set-1 LEO-600 S-band DL — identical study case to
// examples/ntn-tr38821-calibration.cc, so regime A is directly comparable.
constexpr double kFreqHz = 2.0e9;
constexpr double kBwHz = 30e6;
constexpr double kEirpDbw = 48.77;   // 34 dBW/MHz + 10log10(30)
constexpr double kGtDbK = -31.62;    // handheld: 0 dBi, NF 7
constexpr double kBoltzDbwHzK = -228.6;
constexpr double kEarthRadiusM = 6371000.0;
const double kNan = std::numeric_limits<double>::quiet_NaN();

double
FsplDb(double dM)
{
    return 20.0 * std::log10(4.0 * M_PI * dM * kFreqHz / kC);
}

double
TrCnrDb(double slantM)
{
    // CNR = EIRP - FSPL + G/T - 10log10(k) - 10log10(B)   [TR 38.821 6.1.3.1]
    return kEirpDbw - FsplDb(slantM) + kGtDbK - kBoltzDbwHzK - 10.0 * std::log10(kBwHz);
}

/// Everything the two regime runs share.
struct RunConfig
{
    double simSeconds{120.0};
    double samplePeriodS{1.0};
    double beamwidthDeg{4.4127};   ///< TR 38.821 Set-1 LEO-600 S-band
    double rolloffFloorDb{-40.0};  ///< deepest applied sidelobe roll-off
    double boresightOffsetKm{60.0}; ///< regime B, "point" mode: cross-track offset
    std::string boresightMode{"point"}; ///< point | nadir | direction
    std::string radio{"mmwave"};
    std::string outputDir{"ntn-tr38821-array-gain-output"};
    bool excessLoss{true}; ///< keep the TR 38.811 excess-loss chain (paired diff cancels it)
};

/// One measured sample of the pass.
struct Sample
{
    double t{0.0};
    double elevDeg{0.0};
    double slantKm{0.0};
    double thetaDeg{0.0};       ///< off-boresight angle recomputed from geometry
    double modelThetaDeg{0.0};  ///< the same angle as seen by the propagation model
    double modelRolloffDb{0.0}; ///< roll-off the model actually applied
    double analyticDb{0.0};     ///< GainDbAtThetaDeg(thetaDeg) — the closed form
    double trCnrDb{0.0};
    double measCnrDb{0.0};
};

struct RegimeOut
{
    std::string name;
    std::string boresight;
    Vector boresightPoint{0.0, 0.0, 0.0};
    std::vector<Sample> s;
};

double
Norm(const Vector& v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/**
 * \brief Run one regime of the pass and return its measured samples.
 *
 * \p tracking selects regime A (boresight steered at the UE) vs regime B
 * (boresight pinned per \p cfg.boresightMode). Everything else — orbit, UE,
 * radio, traffic, sampling instants — is bit-identical between the two calls,
 * which is what makes the paired differencing in main() legitimate.
 */
RegimeOut
RunRegime(const RunConfig& cfg, bool tracking)
{
    RegimeOut out;
    out.name = tracking ? "A_steered" : "B_fixed";

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    // Real SGP4 LEO-600 element; the local ENU frame is referenced at the
    // sub-satellite point at t = 0, so the UE starts exactly at zenith.
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = 600.0;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(elements[0]);
    double subLat, subLon, subAlt;
    satSgp4->GetGeodetic(subLat, subLon, subAlt);
    Ptr<NtnEnuProjectionMobilityModel> satEnu = CreateObject<NtnEnuProjectionMobilityModel>();
    satEnu->SetSource(satSgp4);
    satEnu->SetReference(subLat, subLon, 0.0);
    satNodes.Get(0)->AggregateObject(satEnu);

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(0.0, 0.0, 1.5));
    mob.SetPositionAllocator(pos);
    mob.Install(ueNodes);
    Ptr<MobilityModel> ueMob = ueNodes.Get(0)->GetObject<MobilityModel>();

    // ---- the one thing that differs between the regimes: the boresight ----
    Ptr<NtnSatBeamGainModel> beam = CreateObject<NtnSatBeamGainModel>();
    beam->SetBeamwidth3dBDeg(cfg.beamwidthDeg);
    beam->SetRolloffFloorDb(cfg.rolloffFloorDb);
    // ApplyPeakGain stays FALSE on purpose: the mmwave array already supplies
    // the boresight peak, so adding it here would double-count it. What this
    // experiment isolates is the ANGULAR term on top of that peak.
    beam->SetApplyPeakGain(false);

    Vector borePoint(0.0, 0.0, 0.0);
    if (tracking)
    {
        // BoresightMode::TrackUe is the default; state it explicitly so the
        // regime is readable from the code, not from a default.
        beam->SetBeamCenter(Ptr<MobilityModel>());
        out.boresight = "track-ue";
    }
    else if (cfg.boresightMode == "nadir")
    {
        // Beam fixed in the satellite BODY frame. In this local ENU frame the
        // geocentre sits at (0, 0, -R_earth), so sat -> geocentre is exact
        // nadir; theta is then the UE's off-nadir angle and sweeps fast.
        beam->SetBoresightNadir(Vector(0.0, 0.0, -kEarthRadiusM));
        out.boresight = "nadir";
    }
    else if (cfg.boresightMode == "direction")
    {
        // Attitude-locked beam along the ENU down-axis: nadir near the frame
        // origin, drifting from true nadir as the satellite leaves it.
        beam->SetBoresightFixed(Vector(0.0, 0.0, -1.0));
        out.boresight = "fixed-direction-enu-down";
    }
    else
    {
        // Default: a fixed GROUND cell centre offset along the ground track.
        // The angular separation between that cell centre and the (static) UE,
        // seen from the moving satellite, decreases slowly through the pass —
        // so the UE drifts from a sidelobe, across the first null, up into the
        // mainlobe over ~2 minutes. That is a far better-conditioned angular
        // sweep than nadir (which crosses the whole mainlobe in ~8 s).
        const Vector v0 = satEnu->GetVelocity();
        const double h = std::hypot(v0.x, v0.y);
        const Vector u = (h > 1.0) ? Vector(v0.x / h, v0.y / h, 0.0) : Vector(1.0, 0.0, 0.0);
        borePoint = Vector(u.x * cfg.boresightOffsetKm * 1e3,
                           u.y * cfg.boresightOffsetKm * 1e3,
                           0.0);
        beam->SetBoresightFixedPoint(borePoint);
        out.boresight = "fixed-ground-point";
    }
    out.boresightPoint = borePoint;

    // Independent re-derivation of the off-boresight angle from the raw
    // geometry. Deliberately NOT a call into the model: it is the cross-check
    // that the model's own GetLastThetaDeg() is reporting the same angle.
    auto ThetaDeg = [&](const Vector& sat, const Vector& ue) {
        const Vector dUe(ue.x - sat.x, ue.y - sat.y, ue.z - sat.z);
        Vector dBore = dUe;
        if (!tracking)
        {
            if (cfg.boresightMode == "nadir")
            {
                dBore = Vector(-sat.x, -sat.y, -kEarthRadiusM - sat.z);
            }
            else if (cfg.boresightMode == "direction")
            {
                dBore = Vector(0.0, 0.0, -1.0);
            }
            else
            {
                dBore = Vector(borePoint.x - sat.x, borePoint.y - sat.y, borePoint.z - sat.z);
            }
        }
        const double nB = Norm(dBore);
        const double nU = Norm(dUe);
        if (nB < 1e-9 || nU < 1.0)
        {
            return 0.0;
        }
        double c = (dBore.x * dUe.x + dBore.y * dUe.y + dBore.z * dUe.z) / (nB * nU);
        c = std::max(-1.0, std::min(1.0, c));
        return std::acos(c) * 180.0 / M_PI;
    };

    // Handheld UE noise figure per the TR study case (mmwave-specific attribute).
    if (cfg.radio == "mmwave")
    {
        Config::SetDefault("ns3::MmWaveUePhy::NoiseFigure", DoubleValue(7.0));
    }

    NtnRealStackHelper rs;
    rs.SetRadioBackend(cfg.radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                             : NtnRealStackHelper::RadioBackend::Nr);
    if (cfg.radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(cfg.simSeconds));
    rs.SetOutputDir(cfg.outputDir);
    rs.SetRunTag("ntn-tr38821-array-gain-" + out.name);
    rs.SetCarrierFrequencyHz(kFreqHz);
    rs.SetBandwidthHz(kBwHz);
    // NT-02 FIX (2026-08-24): kEirpDbw is the TR 38.821 Set-1 TOTAL EIRP, and
    // the closed-form reference above uses it as such. Passing it to the
    // CONDUCTED-power setter made the simulator radiate it PLUS the array
    // gain, so the run sat ~18 dB above the very standard it calibrates
    // against, and the excess was then reported as "the array gain the closed
    // form does not carry". Declaring the density makes the radiated EIRP
    // equal kEirpDbw, so measured and reference are on the same budget.
    rs.SetSatEirpDensityDbwMhz(34.0); // TR 38.821 Set-1 S-band DL
    rs.SetTr38811ExcessLoss(cfg.excessLoss);
    rs.Build(satNodes, ueNodes);
    // The beam pattern is a real link in the propagation chain, so its roll-off
    // reaches the MEASURED SINR through the PHY — not a post-hoc correction.
    rs.AddExtraPropagationLoss(beam);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(cfg.simSeconds - 0.5));

    uint64_t lastTb = 0;
    rs.RegisterPeriodicCallback(Seconds(cfg.samplePeriodS), [&](Time now) {
        const Vector u = ueMob->GetPosition();
        const Vector s = satEnu->GetPosition();
        const double slant = std::hypot(s.x - u.x, s.y - u.y, s.z - u.z);
        const double horiz = std::hypot(s.x - u.x, s.y - u.y);
        const double meas = rs.GetUeRecentSinrDb(0);
        // Freshness: GetUeRecentSinrDb() latches the last PHY sample, so in a
        // deep null (where the link may briefly stop decoding) it would report
        // a stale value. Only count a sample if transport blocks were actually
        // decoded since the previous tick.
        // NB: GetPhyRxTb() is only filled by Collect() at end-of-run and reads 0
        // during the simulation, which would reject every sample. The live PHY
        // trace counter is the one that answers "was a TB decoded since the last
        // tick?".
        const uint64_t tb = rs.GetPhyRxTbLive();
        const bool fresh = (tb > lastTb);
        lastTb = tb;
        if (std::isnan(meas) || !fresh)
        {
            return;
        }
        Sample smp;
        smp.t = now.GetSeconds();
        smp.slantKm = slant / 1e3;
        smp.elevDeg = std::atan2(s.z - u.z, horiz) * 180.0 / M_PI;
        smp.thetaDeg = ThetaDeg(s, u);
        smp.modelThetaDeg = beam->GetLastThetaDeg();
        smp.modelRolloffDb = beam->GetLastRolloffDb();
        smp.analyticDb = beam->GainDbAtThetaDeg(smp.thetaDeg);
        smp.trCnrDb = TrCnrDb(slant);
        smp.measCnrDb = meas;
        out.s.push_back(smp);
    });

    std::printf("# regime %s: boresight=%s", out.name.c_str(), out.boresight.c_str());
    if (out.boresight == "fixed-ground-point")
    {
        std::printf(" at ENU (%.1f, %.1f, 0.0) km", borePoint.x / 1e3, borePoint.y / 1e3);
    }
    std::printf(" — running %.0f s\n", cfg.simSeconds);

    Simulator::Stop(Seconds(cfg.simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();
    Simulator::Destroy();

    std::printf("#   %zu measured samples\n", out.s.size());
    return out;
}

/// Mean / population standard deviation of a series (NaN-safe on empty input).
void
MeanStd(const std::vector<double>& v, double& mean, double& stddev)
{
    if (v.empty())
    {
        mean = kNan;
        stddev = kNan;
        return;
    }
    double s = 0.0;
    for (double x : v)
    {
        s += x;
    }
    mean = s / v.size();
    double q = 0.0;
    for (double x : v)
    {
        q += (x - mean) * (x - mean);
    }
    stddev = std::sqrt(q / v.size());
}

} // namespace

int
main(int argc, char* argv[])
{
    RunConfig cfg;
    // Gate on the paired pattern-fidelity RMS error. 3.0 dB admits the two
    // regimes' independent fading realisations (~1.4 dB each -> ~2 dB paired)
    // while still failing a pattern that is wrong by a lobe.
    double patternTolDb = 3.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration per regime (s)", cfg.simSeconds);
    cmd.AddValue("samplePeriodS", "Sampling period (s)", cfg.samplePeriodS);
    cmd.AddValue("beamwidthDeg", "Satellite beam 3 dB beamwidth (deg)", cfg.beamwidthDeg);
    cmd.AddValue("rolloffFloorDb", "Deepest applied sidelobe roll-off (dB)", cfg.rolloffFloorDb);
    cmd.AddValue("boresightMode",
                 "Regime B boresight: point (fixed ground cell centre) | nadir | direction",
                 cfg.boresightMode);
    cmd.AddValue("boresightOffsetKm",
                 "Regime B 'point' mode: cell-centre offset along the ground track (km)",
                 cfg.boresightOffsetKm);
    cmd.AddValue("excessLoss", "Keep the TR 38.811 excess-loss chain", cfg.excessLoss);
    cmd.AddValue("radio", "Radio backend: mmwave (calibrated) or nr (experimental)", cfg.radio);
    cmd.AddValue("patternTolDb", "Pattern-fidelity RMS gate (dB)", patternTolDb);
    cmd.AddValue("outputDir", "Output directory", cfg.outputDir);
    cmd.Parse(argc, argv);

    std::printf("# ntn-tr38821-array-gain-calibration (R2.8: is the array gain "
                "angle-dependent?)\n");
    std::printf("#   TR 38.821 Set-1 LEO-600 S-band DL; TR reference at zenith: "
                "FSPL=%.2f dB CNR=%.2f dB\n",
                FsplDb(600e3), TrCnrDb(600e3));
    std::printf("#   TR 38.811 6.4.1 Airy pattern, 3 dB BW = %.4f deg (half-beamwidth "
                "%.4f deg), floor %.1f dB\n",
                cfg.beamwidthDeg, 0.5 * cfg.beamwidthDeg, cfg.rolloffFloorDb);

    // A standalone pattern instance for the analytic cut printed below — the
    // same closed form the CSV's analytic_pattern_db column uses.
    Ptr<NtnSatBeamGainModel> ref = CreateObject<NtnSatBeamGainModel>();
    ref->SetBeamwidth3dBDeg(cfg.beamwidthDeg);
    ref->SetRolloffFloorDb(cfg.rolloffFloorDb);
    std::printf("#   analytic cut: G(0)=%.3f dB  G(theta_3dB/2)=%.3f dB  G(2x)=%.3f dB "
                "G(3x)=%.3f dB\n",
                ref->GainDbAtThetaDeg(0.0),
                ref->GainDbAtThetaDeg(0.5 * cfg.beamwidthDeg),
                ref->GainDbAtThetaDeg(cfg.beamwidthDeg),
                ref->GainDbAtThetaDeg(1.5 * cfg.beamwidthDeg));

    const RegimeOut a = RunRegime(cfg, true);
    const RegimeOut b = RunRegime(cfg, false);

    // ---- pair regime B against regime A at identical sampling instants ----
    // Same orbit, same UE, same tick period => the geometry (slant, elevation)
    // is identical, so the difference isolates the beam term.
    std::vector<double> pairedErr;     // (resB - resA) - analytic, in-lobe only
    std::vector<double> pairedErrAll;  // ... over every paired sample
    std::vector<double> bPatternErr(b.s.size(), kNan);
    {
        size_t ia = 0;
        for (size_t ib = 0; ib < b.s.size(); ++ib)
        {
            while (ia < a.s.size() && a.s[ia].t < b.s[ib].t - 1e-6)
            {
                ++ia;
            }
            if (ia >= a.s.size() || std::abs(a.s[ia].t - b.s[ib].t) > 1e-6)
            {
                continue; // no partner sample at this instant
            }
            const double resA = a.s[ia].measCnrDb - a.s[ia].trCnrDb;
            const double resB = b.s[ib].measCnrDb - b.s[ib].trCnrDb;
            const double err = (resB - resA) - b.s[ib].analyticDb;
            bPatternErr[ib] = err;
            pairedErrAll.push_back(err);
            // "In-lobe" = not clamped at the roll-off floor. At an Airy null the
            // true pattern is singular and the model floors it, so a floored
            // sample tests the floor, not the pattern.
            if (b.s[ib].analyticDb > cfg.rolloffFloorDb + 1.0)
            {
                pairedErr.push_back(err);
            }
        }
    }

    std::error_code ec;
    std::filesystem::create_directories(cfg.outputDir, ec);

    // ---- per-sample CSV ----
    const std::string perSample = cfg.outputDir + "/array_gain_calibration.csv";
    {
        std::ofstream f(perSample);
        f << "regime,boresight,t_s,elevation_deg,slant_km,theta_offboresight_deg,"
             "tr_cnr_db,measured_cnr_db,residual_db,analytic_pattern_db,"
             "model_theta_deg,model_rolloff_db,paired_pattern_err_db\n";
        auto dump = [&](const RegimeOut& r, const std::vector<double>* err) {
            for (size_t i = 0; i < r.s.size(); ++i)
            {
                const Sample& s = r.s[i];
                f << r.name << ',' << r.boresight << ',' << s.t << ',' << s.elevDeg << ','
                  << s.slantKm << ',' << s.thetaDeg << ',' << s.trCnrDb << ',' << s.measCnrDb
                  << ',' << (s.measCnrDb - s.trCnrDb) << ',' << s.analyticDb << ','
                  << s.modelThetaDeg << ',' << s.modelRolloffDb << ',';
                if (err && i < err->size() && !std::isnan((*err)[i]))
                {
                    f << (*err)[i];
                }
                else
                {
                    f << "nan";
                }
                f << '\n';
            }
        };
        dump(a, nullptr);
        dump(b, &bPatternErr);
    }

    // ---- per-regime summary ----
    struct RegimeStats
    {
        double thMin{kNan};
        double thMax{kNan};
        double resMin{kNan};
        double resMax{kNan};
        double resMean{kNan};
        double resStd{kNan};
        double maxAbsRolloff{0.0}; ///< largest roll-off the model actually applied
    };
    auto summarise = [](const RegimeOut& r) {
        RegimeStats st;
        if (r.s.empty())
        {
            return st;
        }
        std::vector<double> res;
        st.thMin = std::numeric_limits<double>::infinity();
        st.thMax = -std::numeric_limits<double>::infinity();
        st.resMin = std::numeric_limits<double>::infinity();
        st.resMax = -std::numeric_limits<double>::infinity();
        for (const Sample& s : r.s)
        {
            const double d = s.measCnrDb - s.trCnrDb;
            res.push_back(d);
            st.thMin = std::min(st.thMin, s.thetaDeg);
            st.thMax = std::max(st.thMax, s.thetaDeg);
            st.resMin = std::min(st.resMin, d);
            st.resMax = std::max(st.resMax, d);
            st.maxAbsRolloff = std::max(st.maxAbsRolloff, std::abs(s.modelRolloffDb));
        }
        MeanStd(res, st.resMean, st.resStd);
        return st;
    };
    const RegimeStats sa = summarise(a);
    const RegimeStats sb = summarise(b);

    double errMean, errStd;
    MeanStd(pairedErr, errMean, errStd);
    double errMaxAbs = 0.0;
    double errRms = 0.0;
    for (double e : pairedErr)
    {
        errMaxAbs = std::max(errMaxAbs, std::abs(e));
        errRms += e * e;
    }
    errRms = pairedErr.empty() ? kNan : std::sqrt(errRms / pairedErr.size());

    const std::string summaryCsv = cfg.outputDir + "/array_gain_summary.csv";
    {
        std::ofstream f(summaryCsv);
        f << "regime,boresight,n_samples,theta_min_deg,theta_max_deg,residual_mean_db,"
             "residual_std_db,residual_range_db,max_abs_rolloff_db,n_paired_inlobe,"
             "pattern_max_abs_err_db,pattern_rms_err_db,pattern_mean_err_db\n";
        f << a.name << ',' << a.boresight << ',' << a.s.size() << ',' << sa.thMin << ','
          << sa.thMax << ',' << sa.resMean << ',' << sa.resStd << ','
          << (sa.resMax - sa.resMin) << ',' << sa.maxAbsRolloff << ",0,nan,nan,nan\n";
        f << b.name << ',' << b.boresight << ',' << b.s.size() << ',' << sb.thMin << ','
          << sb.thMax << ',' << sb.resMean << ',' << sb.resStd << ','
          << (sb.resMax - sb.resMin) << ',' << sb.maxAbsRolloff << ',' << pairedErr.size()
          << ',' << errMaxAbs << ',' << errRms << ',' << errMean << '\n';
    }

    // ---- stdout summary ----
    std::printf("\n# === R2.8 array-gain angle dependence ===\n");
    std::printf("# regime A (steered, boresight tracks the UE)\n");
    std::printf("#   samples=%zu  theta in [%.6f, %.6f] deg  residual mean=%.2f dB "
                "std=%.2f dB range=%.2f dB  max|rolloff applied|=%.2e dB\n",
                a.s.size(), sa.thMin, sa.thMax, sa.resMean, sa.resStd,
                sa.resMax - sa.resMin, sa.maxAbsRolloff);
    std::printf("#   -> the beam is steered at the terminal, so theta ~ 0 and the "
                "pattern contributes 0 dB:\n"
                "#      the measured-minus-TR offset is CONSTANT for a physical "
                "reason, not by construction.\n");
    std::printf("# regime B (fixed boresight = %s)\n", b.boresight.c_str());
    std::printf("#   samples=%zu  theta in [%.4f, %.4f] deg  residual mean=%.2f dB "
                "std=%.2f dB range=%.2f dB  max|rolloff applied|=%.2f dB\n",
                b.s.size(), sb.thMin, sb.thMax, sb.resMean, sb.resStd,
                sb.resMax - sb.resMin, sb.maxAbsRolloff);
    std::printf("#   -> theta sweeps %.3f deg (%.2f x the 3 dB beamwidth) and the "
                "residual swings %.2f dB with it.\n",
                sb.thMax - sb.thMin, (sb.thMax - sb.thMin) / cfg.beamwidthDeg,
                sb.resMax - sb.resMin);
    std::printf("# pattern fidelity (paired, in-lobe): n=%zu  max|err|=%.2f dB  "
                "rms=%.2f dB  mean=%.2f dB (tol rms < %.2f dB)\n",
                pairedErr.size(), errMaxAbs, errRms, errMean, patternTolDb);

    // ---- gates ----
    // theta is derived through an acos, so "exactly boresight" lands within a
    // few 1e-7 deg of 0; assert on the roll-off the model ACTUALLY applied as
    // well, which is the physically meaningful statement.
    const bool gateSteered = (!a.s.empty()) && (sa.thMax < 1e-4) &&
                             (sa.maxAbsRolloff < 1e-6) && (sa.resStd < 2.0);
    const bool gateSweep = (!b.s.empty()) && ((sb.thMax - sb.thMin) > 0.5 * cfg.beamwidthDeg);
    const bool gatePattern = (pairedErr.size() >= 5) && !std::isnan(errRms) &&
                             (errRms < patternTolDb);
    std::printf("# gates: steered(theta==0, offset constant)=%s  sweep(theta spans "
                ">half the 3 dB BW)=%s  pattern(rms<%.2f dB)=%s  -> %s\n",
                gateSteered ? "PASS" : "FAIL",
                gateSweep ? "PASS" : "FAIL",
                patternTolDb,
                gatePattern ? "PASS" : "FAIL",
                (gateSteered && gateSweep && gatePattern) ? "ANGLE-DEPENDENCE VERIFIED"
                                                          : "FAIL");
    std::printf("# wrote %s\n#        %s\n", perSample.c_str(), summaryCsv.c_str());

    return (gateSteered && gateSweep && gatePattern) ? 0 : 1;
}
