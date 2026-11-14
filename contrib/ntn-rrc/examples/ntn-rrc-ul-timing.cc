/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Uplink timing pre-compensation over a real LEO pass (TVT study, Section V-B).
//
// A satellite flies its own TLE through Vallado SGP4 (ntn-constellation). The network side
// (NtnSib19Broadcaster) fills SIB19 every --sibPeriod from that orbit: ECEF state vector and the
// common-delay polynomial for the chosen uplink synchronization reference point, quantized to
// the TS 38.331 field steps. The UE side (NtnUlTimingPrecompensator) decodes SIB19 every
// --readPeriod (or when ul-SyncValidityDuration expires), propagates the satellite with the
// chosen model from its last GNSS fix, and forms its timing advance. Every 100 ms the example
// compares that advance with the true two-way delay over the actual geometry and writes
// time, information age, UE elevation and timing error to --out.
//
//   ./ns3 run "ntn-rrc-ul-timing --tleLine1=... --tleLine2=... --model=kepler-j2 --ref=gateway"

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ntn-sib19.h"
#include "ns3/ntn-timing-advance.h"
#include "ns3/ntn-ul-timing-precompensator.h"
#include "ns3/orbital-elements.h"
#include "ns3/sgp4-mobility-model.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>

using namespace ns3;
using namespace ns3::ntnrrc;

namespace
{
constexpr double kC = 299792458.0;
constexpr double kRe = 6378137.0;

Vector
Unit(const Vector& v)
{
    const double n = v.GetLength();
    return Vector(v.x / n, v.y / n, v.z / n);
}

Vector
Cross(const Vector& a, const Vector& b)
{
    return Vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

double
Dot(const Vector& a, const Vector& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Point on the sphere of radius kRe at central angle psi from unit vector u, toward unit w.
Vector
Rotate(const Vector& u, const Vector& w, double psi)
{
    return Vector(kRe * (std::cos(psi) * u.x + std::sin(psi) * w.x),
                  kRe * (std::cos(psi) * u.y + std::sin(psi) * w.y),
                  kRe * (std::cos(psi) * u.z + std::sin(psi) * w.z));
}

double
ElevationDeg(const Vector& ground, const Vector& sat)
{
    const Vector up = Unit(ground);
    const Vector los = sat - ground;
    return std::asin(Dot(up, los) / los.GetLength()) * 180.0 / M_PI;
}

// Network orbit-determination error: a radial/along-track/cross-track offset drawn per orbit
// determination (every g_odPeriod seconds) plus a velocity error carried since that determination.
Ptr<ntncon::Sgp4MobilityModel> g_sat;
double g_odPos = 0.0;
double g_odVel = 0.0;
double g_odPeriod = 60.0;
Ptr<NormalRandomVariable> g_odRng;
std::map<int64_t, std::pair<Vector, Vector>> g_odDraws;

std::pair<Vector, Vector>
OdErrorFor(int64_t k, const Vector& p, const Vector& v)
{
    auto it = g_odDraws.find(k);
    if (it != g_odDraws.end())
    {
        return it->second;
    }
    // unit vectors of the radial, along-track and cross-track frame at the satellite
    const Vector rhat = Unit(p);
    const Vector chat = Unit(Cross(p, v));
    const Vector ahat = Cross(chat, rhat);
    const double er = g_odRng->GetValue(0.0, std::pow(g_odPos / 3.0, 2));
    const double ea = g_odRng->GetValue(0.0, g_odPos * g_odPos);
    const double ec = g_odRng->GetValue(0.0, std::pow(g_odPos / 3.0, 2));
    const double va = g_odRng->GetValue(0.0, g_odVel * g_odVel);
    const Vector dp(er * rhat.x + ea * ahat.x + ec * chat.x, er * rhat.y + ea * ahat.y + ec * chat.y,
                    er * rhat.z + ea * ahat.z + ec * chat.z);
    const Vector dv(va * ahat.x, va * ahat.y, va * ahat.z);
    g_odDraws[k] = {dp, dv};
    return g_odDraws[k];
}

/// The network's prediction of the satellite state at Now() + offset.
void
NetworkOrbit(Time offset, Vector& p, Vector& v)
{
    g_sat->GetEcefStateAt(offset, p, v);
    if (g_odPos <= 0.0 && g_odVel <= 0.0)
    {
        return;
    }
    // the determination in force is the one made at the start of the current period
    const double now = Simulator::Now().GetSeconds();
    const int64_t k = static_cast<int64_t>(std::floor(now / g_odPeriod));
    const auto err = OdErrorFor(k, p, v);
    const double age = now + offset.GetSeconds() - k * g_odPeriod;
    p = Vector(p.x + err.first.x + err.second.x * age, p.y + err.first.y + err.second.y * age,
               p.z + err.first.z + err.second.z * age);
    v = Vector(v.x + err.second.x, v.y + err.second.y, v.z + err.second.z);
}

UeEphemerisModel
ParseModel(const std::string& s)
{
    if (s == "linear")
    {
        return UeEphemerisModel::Linear;
    }
    if (s == "gravity-ecef")
    {
        return UeEphemerisModel::GravityEcef;
    }
    if (s == "kepler")
    {
        return UeEphemerisModel::Kepler;
    }
    return UeEphemerisModel::KeplerJ2;
}
} // namespace

int
main(int argc, char* argv[])
{
    std::string tleLine1 =
        "1 44714U 19074B   26274.55916299  .00027027  00000+0  26602-3 0  9994";
    std::string tleLine2 =
        "2 44714  53.1459 272.8136 0002818  85.3455 274.7883 15.65813331380801";
    double closestApproachS = 450.0; // time of the UE's closest approach after start
    double crossTrackKm = 0.0;       // UE offset from the ground track
    double gwCrossTrackKm = 300.0;   // gateway offset from the ground track
    double gwAlongTrackKm = 0.0;     // gateway offset along the track
    double ueSpeedMps = 0.0;         // vehicle speed along the track direction
    double sibPeriodS = 1.0;
    double readPeriodS = 5.0;
    double simTimeS = 900.0;
    double maskDeg = 25.0;
    double gnssSigmaM = 0.0;
    double gnssPeriodS = 1.0;
    std::string model = "kepler-j2";
    std::string ref = "gateway";
    std::string fit = "taylor";
    std::string validity = "budget";
    double fitWindowS = 10.0;
    std::string out = "ul_timing.csv";
    uint32_t seed = 1;
    double odPos = 0.0;       // network orbit error, 1 sigma along track (m)
    double odVel = 0.0;       // network orbit velocity error, 1 sigma (m/s)
    double odPeriod = 60.0;   // orbit determination period (s)
    bool odBudget = true;     // whether the validity rule budgets the orbit error
    bool ueProject = false;   // UE projects its GNSS fix with the measured velocity
    double ueAllowanceNs = 0.0; // part of Te reserved for UE position error and motion
    double teNs = 716.15;     // Te_NTN, FR1-NTN 30/30 kHz (TS 38.133 Table 7.1C.2-1)
    double commonShare = 0.5; // share of Te given to the common-delay polynomial

    CommandLine cmd(__FILE__);
    cmd.AddValue("tleLine1", "TLE line 1 (spaces may be written as _)", tleLine1);
    cmd.AddValue("tleLine2", "TLE line 2 (spaces may be written as _)", tleLine2);
    cmd.AddValue("closestApproachS", "time of closest approach after start (s)", closestApproachS);
    cmd.AddValue("crossTrackKm", "UE cross-track offset (km)", crossTrackKm);
    cmd.AddValue("gwCrossTrackKm", "gateway cross-track offset (km)", gwCrossTrackKm);
    cmd.AddValue("gwAlongTrackKm", "gateway along-track offset (km)", gwAlongTrackKm);
    cmd.AddValue("ueSpeed", "UE ground speed along the track (m/s)", ueSpeedMps);
    cmd.AddValue("sibPeriod", "SIB19 refresh period (s)", sibPeriodS);
    cmd.AddValue("readPeriod", "UE SIB19 re-read period (s)", readPeriodS);
    cmd.AddValue("simTime", "simulated time (s)", simTimeS);
    cmd.AddValue("mask", "elevation mask (deg)", maskDeg);
    cmd.AddValue("gnssSigma", "UE GNSS horizontal 1-sigma error (m)", gnssSigmaM);
    cmd.AddValue("gnssPeriod", "UE GNSS fix period (s)", gnssPeriodS);
    cmd.AddValue("model", "UE propagator: linear|gravity-ecef|kepler|kepler-j2", model);
    cmd.AddValue("ref", "uplink sync reference point: gateway|satellite", ref);
    cmd.AddValue("fit", "SIB19 coefficient fit: taylor|econ|window", fit);
    cmd.AddValue("validity", "ul-SyncValidity rule: slot|budget|fixed (fixed uses --fitWindow)", validity);
    cmd.AddValue("fitWindow", "fit window when validity=slot (s)", fitWindowS);
    cmd.AddValue("out", "output CSV", out);
    cmd.AddValue("seed", "RNG run number", seed);
    cmd.AddValue("teNs", "uplink timing error limit Te_NTN (ns)", teNs);
    cmd.AddValue("odPos", "network orbit error, 1 sigma along track (m)", odPos);
    cmd.AddValue("odVel", "network orbit velocity error, 1 sigma (m/s)", odVel);
    cmd.AddValue("odPeriod", "network orbit determination period (s)", odPeriod);
    cmd.AddValue("odBudget", "validity rule budgets the orbit error", odBudget);
    cmd.AddValue("ueProject", "UE projects its GNSS fix with the measured velocity", ueProject);
    cmd.AddValue("ueAllowanceNs", "part of Te reserved for UE position error and motion (ns)",
                 ueAllowanceNs);
    cmd.AddValue("commonShare", "share of Te for the common-delay polynomial", commonShare);
    cmd.Parse(argc, argv);
    RngSeedManager::SetRun(seed);

    // ---- satellite: Vallado SGP4 from the TLE
    // CommandLine stops a string value at the first space, so lines may be passed with '_'
    std::replace(tleLine1.begin(), tleLine1.end(), '_', ' ');
    std::replace(tleLine2.begin(), tleLine2.end(), '_', ' ');
    ntncon::TleRecord tle;
    tle.line1 = tleLine1;
    tle.line2 = tleLine2;
    Ptr<ntncon::Sgp4MobilityModel> sat = CreateObject<ntncon::Sgp4MobilityModel>();
    NS_ABORT_MSG_UNLESS(sat->SetTle(tle), "TLE did not parse");
    NS_ABORT_MSG_UNLESS(sat->IsUsingSgp4(), "SGP4 backend not active");
    // The position cache defaults to 100 ms. At a LEO range rate of up to 7 km/s that is up to
    // 0.7 km of stale range, several microseconds of delay, so a timing study must not use it.
    sat->SetCacheGranularity(MicroSeconds(1));

    // ---- geometry of the pass: UE near the ground track at closest approach
    Vector s0;
    Vector v0;
    sat->GetEcefStateAt(Seconds(closestApproachS), s0, v0);
    const Vector nadir = Unit(s0);
    const Vector cross = Unit(Cross(nadir, v0));
    const Vector along = Unit(Cross(cross, nadir));
    const Vector uePos0 = Rotate(nadir, cross, crossTrackKm * 1e3 / kRe);
    Vector gwBase = Rotate(nadir, cross, gwCrossTrackKm * 1e3 / kRe);
    const Vector gwPos = Rotate(Unit(gwBase), along, gwAlongTrackKm * 1e3 / kRe);

    Ptr<MobilityModel> ue;
    if (ueSpeedMps > 0.0)
    {
        // vehicle on a great circle through the UE start point, waypoints every 10 s
        Ptr<WaypointMobilityModel> w = CreateObject<WaypointMobilityModel>();
        const Vector u0 = Unit(uePos0);
        const Vector dir = Unit(along - Vector(u0.x * Dot(along, u0), u0.y * Dot(along, u0),
                                                u0.z * Dot(along, u0)));
        for (double t = 0.0; t <= simTimeS + 10.0; t += 10.0)
        {
            const double psi = ueSpeedMps * (t - closestApproachS) / kRe;
            w->AddWaypoint(Waypoint(Seconds(t), Rotate(u0, dir, psi)));
        }
        ue = w;
    }
    else
    {
        Ptr<ConstantPositionMobilityModel> c = CreateObject<ConstantPositionMobilityModel>();
        c->SetPosition(uePos0);
        ue = c;
    }
    Ptr<ConstantPositionMobilityModel> gw = CreateObject<ConstantPositionMobilityModel>();
    gw->SetPosition(gwPos);

    // ---- network side
    Ptr<NtnTimingAdvance> ta = CreateObject<NtnTimingAdvance>();
    ta->SetUeMobility(ue);
    ta->SetSatelliteMobility(sat);
    ta->SetGatewayMobility(gw);
    ta->SetPayloadMode(ref == "gateway" ? PayloadMode::Transparent : PayloadMode::RegenerativeFull);
    g_sat = sat;
    g_odPos = odPos;
    g_odVel = odVel;
    g_odPeriod = odPeriod;
    g_odRng = CreateObject<NormalRandomVariable>();
    g_odRng->SetStream(1000 + seed);
    ta->SetSatelliteStateProvider(MakeCallback(&NetworkOrbit));

    Ptr<NtnSib19Broadcaster> sib = CreateObject<NtnSib19Broadcaster>();
    sib->SetSatelliteMobility(sat);
    sib->SetTimingAdvance(ta);
    sib->SetPeriod(Seconds(sibPeriodS));
    sib->SetUlSyncReference(ref == "gateway" ? UlSyncReference::Gateway
                                             : UlSyncReference::Satellite);
    sib->SetGatewayPosition(gwPos);
    sib->SetCoefficientFit(fit == "econ"     ? Sib19CoefficientFit::Economized
                           : fit == "window" ? Sib19CoefficientFit::Window
                                             : Sib19CoefficientFit::Taylor);
    sib->SetFitWindow(Seconds(fitWindowS));
    sib->SetValidityRule(validity == "budget"  ? Sib19ValidityRule::ErrorBudget
                         : validity == "fixed" ? Sib19ValidityRule::Fixed
                                               : Sib19ValidityRule::SlotOverDrift);
    sib->SetAssumedUeModel(ParseModel(model));
    sib->SetTimingErrorLimit(NanoSeconds(teNs), commonShare);
    if (odBudget)
    {
        sib->SetOrbitErrorBudget(odPos + odVel * odPeriod, odVel);
    }
    sib->SetUeAllowance(NanoSeconds(ueAllowanceNs));

    // ---- UE side
    Ptr<NtnUlTimingPrecompensator> pc = CreateObject<NtnUlTimingPrecompensator>();
    pc->SetUeMobility(ue);
    pc->SetEphemerisModel(ParseModel(model));
    pc->SetGnss(Seconds(gnssPeriodS), gnssSigmaM);
    pc->SetPositionProjection(ueProject);
    pc->AssignStreams(seed);

    std::ofstream csv(out);
    csv << "t_s,age_s,validity_s,elev_ue_deg,elev_gw_deg,ta_true_us,err_ns,synced\n";
    csv << std::setprecision(10);
    uint64_t nSamples = 0;
    uint64_t nReads = 0;
    double maxAbs = 0.0;
    uint64_t nViol = 0;

    std::function<void()> sample = [&]() {
        const Vector s = sat->GetPosition();
        const Vector p = ue->GetPosition();
        const double eu = ElevationDeg(p, s);
        const double eg = ElevationDeg(gwPos, s);
        const bool visible = eu >= maskDeg && (ref != "gateway" || eg >= maskDeg);
        if (visible)
        {
            if (!pc->HasSib19() || !pc->IsUplinkSynchronized() ||
                pc->GetInformationAge().GetSeconds() >= readPeriodS)
            {
                pc->ReceiveSib19(sib->GetLatest());
                ++nReads;
            }
            double truth = 2.0 * CalculateDistance(s, p) / kC;
            if (ref == "gateway")
            {
                truth += 2.0 * CalculateDistance(s, gwPos) / kC;
            }
            const double est = pc->ComputeTa().GetSeconds();
            const double err = (est - truth) * 1e9;
            maxAbs = std::max(maxAbs, std::fabs(err));
            nViol += (std::fabs(err) > teNs) ? 1 : 0;
            csv << Simulator::Now().GetSeconds() << "," << pc->GetInformationAge().GetSeconds()
                << "," << pc->GetHeldValidity().GetSeconds() << "," << eu << "," << eg
                << "," << truth * 1e6 << "," << err << "," << pc->IsUplinkSynchronized() << "\n";
            ++nSamples;
        }
        if (Simulator::Now() + MilliSeconds(100) <= Seconds(simTimeS))
        {
            Simulator::Schedule(MilliSeconds(100), sample);
        }
    };

    sib->Start();
    Simulator::Schedule(MilliSeconds(50), sample);
    Simulator::Stop(Seconds(simTimeS));
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "propagator=" << sat->GetPropagatorName() << " samples=" << nSamples
              << " reads=" << nReads << " max_abs_err_ns=" << maxAbs << " viol=" << nViol
              << " infeasible=" << sib->GetInfeasibleCount()
              << " out=" << out << "\n";
    return 0;
}
