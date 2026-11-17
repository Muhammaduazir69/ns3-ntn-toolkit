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
//
// Wave 7A, data plane. After the study run above (whose output is unchanged), a second run
// carries the same SIB19 machinery over a packet-level NR-NTN link for --dataPlaneS seconds
// centred on the closest approach (0 disables it): the same TLE through SGP4, epoch-shifted so
// the window starts at closestApproachS - dataPlaneS/2, is the gNB of a 5G-LENA cell
// (NtnRealStackHelper, TR 38.821 Set-1 S-band link budget); the UE is a TR 38.811 UE class
// (ntn-cho) at the study UE's location; the feeder runs to the study's gateway. The network's
// SIB19 wire image (Sib19Codec, 124 bytes) is sent every --sibPeriod over the EPC and the NR
// downlink, and the UE precompensator only ever consumes what it decoded from those bytes. The
// broadcast cellSpecificKoffset can drive the nr scheduler's K_offset (--kOffset, TS 38.213 4.2;
// off by default, see the argument). DL and UL
// CbrApplication flows (ntn-traffic) carry an in-band SeqTsSizeHeader, so delay, IPDV and loss
// are measured per direction. Every 100 ms the decoded-SIB19 timing advance is compared with
// the true two-way delay of the live geometry (ul_timing_dataplane.csv), and the helper writes
// ue_kpi.csv, cell_kpi.csv and sim_health.csv. Note: nr v3.3 has no UE timing advance, so the
// TA is evaluated against the geometry, not applied to the PUSCH timing.

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ntn-traffic-module.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/nr-phy-mac-common.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/seq-ts-size-header.h"
#include "ns3/ntn-sib19.h"
#include "ns3/ntn-timing-advance.h"
#include "ns3/ntn-ul-timing-precompensator.h"
#include "ns3/orbital-elements.h"
#include "ns3/sgp4-mobility-model.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
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

/// In-band one-way delay, RFC 3393 IPDV and sequence-gap loss of one CBR direction.
struct DirStats
{
    uint64_t rx{0};
    uint64_t highestSeq{0};
    bool any{false};
    double sumDelayMs{0.0};
    uint64_t lastSeq{0};
    double lastDelayMs{0.0};
    uint64_t ipdvN{0};
    double ipdvSumMs{0.0};

    void Rx(Ptr<const Packet>, const Address&, const Address&, const SeqTsSizeHeader& h)
    {
        const double d = (Simulator::Now() - h.GetTs()).GetSeconds() * 1e3;
        ++rx;
        sumDelayMs += d;
        if (any && h.GetSeq() == lastSeq + 1)
        {
            ipdvSumMs += std::fabs(d - lastDelayMs);
            ++ipdvN;
        }
        highestSeq = any ? std::max<uint64_t>(highestSeq, h.GetSeq()) : h.GetSeq();
        any = true;
        lastSeq = h.GetSeq();
        lastDelayMs = d;
    }

    double Loss(uint64_t sent) const
    {
        return sent > 0 ? 1.0 - static_cast<double>(rx) / static_cast<double>(sent) : 0.0;
    }
};

/// PUSCH transport blocks decoded at the gNB (nr RxPacketTraceGnb).
struct UlTbStats
{
    uint64_t tbs{0};
    uint64_t corrupt{0};
    double sumSinrLin{0.0};
    uint64_t sumRb{0};
    uint64_t sumMcs{0};

    void Rx(RxPacketTraceParams p)
    {
        ++tbs;
        corrupt += p.m_corrupt ? 1 : 0;
        sumSinrLin += p.m_sinr;
        sumRb += p.m_rbAssignedNum;
        sumMcs += p.m_mcs;
    }

    double MeanSinrDb() const
    {
        return tbs > 0 ? 10.0 * std::log10(sumSinrLin / tbs) : std::nan("");
    }
};

/// The SIB19 the UE decoded most recently from bytes received over the NR downlink.
struct DecodedSib19
{
    Sib19Content sib;
    bool have{false};
    bool fresh{false}; ///< not yet handed to the precompensator
    uint64_t received{0};
    uint64_t parseFailures{0};

    void Recv(Ptr<Socket> sock)
    {
        Ptr<Packet> pkt;
        Address from;
        while ((pkt = sock->RecvFrom(from)))
        {
            std::vector<uint8_t> buf(pkt->GetSize());
            pkt->CopyData(buf.data(), buf.size());
            Sib19Content parsed;
            if (Sib19Codec::Parse(buf.data(), buf.size(), parsed))
            {
                sib = parsed;
                have = true;
                fresh = true;
                ++received;
            }
            else
            {
                ++parseFailures;
            }
        }
    }
};

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
    double dataPlaneS = 10.0;
    // Off by default: the nr v3.3 data plane carries no air-interface delay at LEO
    // (WF-03), so there is no round trip for K_offset to cover, and consuming the
    // broadcast value (14 cell slots here) throttled PUSCH from 2904 to 225 TBs
    // in a 10 s window (measured, Legacy budget). The SIB19 still carries and the
    // UE still decodes the value.
    bool kOffset = false;
    std::string outputDir = "ntn-rrc-ul-timing-output";
    cmd.AddValue("dataPlaneS",
                 "Wave 7A: seconds of packet-level NR-NTN data plane centred on the closest "
                 "approach, with SIB19 delivered over the radio (0 = study run only)",
                 dataPlaneS);
    cmd.AddValue("kOffset",
                 "data plane: the nr scheduler consumes the broadcast cellSpecificKoffset",
                 kOffset);
    cmd.AddValue("outputDir",
                 "data plane: directory for ul_timing_dataplane.csv, ue_kpi.csv, cell_kpi.csv "
                 "and sim_health.csv",
                 outputDir);
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
    if (dataPlaneS <= 0.0)
    {
        return 0;
    }

    // ==== Wave 7A: the same SIB19 machinery over a packet-level NR-NTN data plane ====
    NS_ABORT_MSG_IF(dataPlaneS < 3.0, "--dataPlaneS must be 0 or >= 3 s");
    std::error_code ec;
    std::filesystem::create_directories(outputDir, ec);
    const double t0 = std::max(0.0, closestApproachS - dataPlaneS / 2.0);
    double ueLat;
    double ueLon;
    double ueAlt;
    ntngeo::EcefToGeodetic(uePos0, ueLat, ueLon, ueAlt);
    double gwLat;
    double gwLon;
    double gwAlt;
    ntngeo::EcefToGeodetic(gwPos, gwLat, gwLon, gwAlt);

    // Satellite: the same TLE through SGP4, sim t = 0 at study time t0.
    Ptr<ntncon::Sgp4MobilityModel> sat2 = CreateObject<ntncon::Sgp4MobilityModel>();
    NS_ABORT_MSG_UNLESS(sat2->SetTle(tle), "TLE did not parse");
    sat2->SetSimulationEpochUnix(sat->SimBaseUnix() + t0);
    sat2->SetCacheGranularity(MicroSeconds(1));
    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    gnbNodes.Get(0)->AggregateObject(sat2);

    // UE: a TR 38.811 UE class at the study UE's location (WGS-84).
    NodeContainer ueNodes;
    ueNodes.Create(1);
    NtnTr38811MobilityHelper ueMobility(seed);
    ueMobility.Install(ueNodes,
                       NtnMobilityScenarios::MixedContinental(),
                       ueLat - 0.002,
                       ueLat + 0.002,
                       ueLon - 0.002,
                       ueLon + 0.002);
    Ptr<MobilityModel> ue2 = ueNodes.Get(0)->GetObject<MobilityModel>();

    NtnRealStackHelper rs;
    rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
    rs.SetNumerology(1);
    rs.SetSimTime(Seconds(dataPlaneS));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-rrc-ul-timing");
    rs.SetPayloadOption(ref == "gateway" ? NtnRealStackHelper::PayloadOption::Transparent
                                         : NtnRealStackHelper::PayloadOption::FullGnb);

    // Network side on the live geometry; the gateway is the helper's feeder gateway.
    Ptr<NtnTimingAdvance> ta2 = CreateObject<NtnTimingAdvance>();
    Ptr<NtnSib19Broadcaster> sib2 = CreateObject<NtnSib19Broadcaster>();
    g_sat = sat2;
    g_odDraws.clear();
    ta2->SetUeMobility(ue2);
    ta2->SetSatelliteMobility(sat2);
    ta2->SetPayloadMode(ref == "gateway" ? PayloadMode::Transparent
                                         : PayloadMode::RegenerativeFull);
    ta2->SetSatelliteStateProvider(MakeCallback(&NetworkOrbit));
    sib2->SetSatelliteMobility(sat2);
    sib2->SetTimingAdvance(ta2);
    sib2->SetPeriod(Seconds(sibPeriodS));
    sib2->SetUlSyncReference(ref == "gateway" ? UlSyncReference::Gateway
                                              : UlSyncReference::Satellite);
    sib2->SetCoefficientFit(fit == "econ"     ? Sib19CoefficientFit::Economized
                            : fit == "window" ? Sib19CoefficientFit::Window
                                              : Sib19CoefficientFit::Taylor);
    sib2->SetFitWindow(Seconds(fitWindowS));
    sib2->SetValidityRule(validity == "budget"  ? Sib19ValidityRule::ErrorBudget
                          : validity == "fixed" ? Sib19ValidityRule::Fixed
                                                : Sib19ValidityRule::SlotOverDrift);
    sib2->SetAssumedUeModel(ParseModel(model));
    sib2->SetTimingErrorLimit(NanoSeconds(teNs), commonShare);
    if (odBudget)
    {
        sib2->SetOrbitErrorBudget(odPos + odVel * odPeriod, odVel);
    }
    sib2->SetUeAllowance(NanoSeconds(ueAllowanceNs));

    // The study's gateway (WGS-84) is both the SIB19 reference point and the
    // helper's feeder gateway.
    Ptr<ConstantPositionMobilityModel> gw2 = CreateObject<ConstantPositionMobilityModel>();
    gw2->SetPosition(ntngeo::GeodeticToEcef(gwLat, gwLon, 0.0));
    const Vector gw2Pos = gw2->GetPosition();
    ta2->SetGatewayMobility(gw2);
    sib2->SetGatewayPosition(gw2Pos);
    // The cell reference point of the common TA (and so of K_offset) is the
    // served location. Left unset it is the Earth's centre, and the broadcast
    // K_offset covers a 49 ms "round trip" (50 reference slots at 550 km)
    // instead of the 6 ms cell round trip.
    ta2->SetReferencePosition(ue2->GetPosition());
    sib2->SetReferencePosition(ue2->GetPosition());
    sib2->RefreshNow();
    // TS 38.213 4.2: the scheduler applies the K_offset the cell broadcasts
    // (15 kHz reference slots; the helper converts to cell slots).
    rs.SetKOffsetConsumption(kOffset);
    if (kOffset)
    {
        rs.SetBroadcastKOffsetSlots(sib2->GetLatest().cellSpecificKoffset);
    }
    rs.Build(gnbNodes, ueNodes);
    rs.SetFeederGateway(gw2);
    sib2->SetCellId(rs.GetServingCellId());
    if (kOffset)
    {
        sib2->SetKOffsetSink(Callback<void, uint32_t>([&rs](uint32_t k) {
            rs.SetBroadcastKOffsetSlots(k);
        }));
    }
    sib2->Start();

    Ptr<NtnUlTimingPrecompensator> pc2 = CreateObject<NtnUlTimingPrecompensator>();
    pc2->SetUeMobility(ue2);
    pc2->SetEphemerisModel(ParseModel(model));
    pc2->SetGnss(Seconds(gnssPeriodS), gnssSigmaM);
    pc2->SetPositionProjection(ueProject);
    pc2->AssignStreams(seed);

    UlTbStats ulTb;
    for (uint32_t d = 0; d < gnbNodes.Get(0)->GetNDevices(); ++d)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(gnbNodes.Get(0)->GetDevice(d));
        if (gnb)
        {
            gnb->GetPhy(0)->GetSpectrumPhy()->TraceConnectWithoutContext(
                "RxPacketTraceGnb",
                MakeCallback(&UlTbStats::Rx, &ulTb));
        }
    }

    // SIB19 wire image over EPC + NR downlink, every sibPeriod.
    const Ipv4Address ueAddr = rs.GetUeIpv4Address(0);
    const uint16_t sibPort = 9876;
    Ptr<Socket> sibTx = Socket::CreateSocket(rs.GetRemoteHost(), UdpSocketFactory::GetTypeId());
    sibTx->Bind();
    DecodedSib19 decoded;
    Ptr<Socket> sibRx = Socket::CreateSocket(ueNodes.Get(0), UdpSocketFactory::GetTypeId());
    sibRx->Bind(InetSocketAddress(Ipv4Address::GetAny(), sibPort));
    sibRx->SetRecvCallback(MakeCallback(&DecodedSib19::Recv, &decoded));
    uint64_t sibSent = 0;
    std::function<void()> sendSib = [&]() {
        const std::vector<uint8_t>& wire = sib2->GetLatestSerialised();
        if (!wire.empty())
        {
            sibTx->SendTo(Create<Packet>(wire.data(), wire.size()),
                          0,
                          InetSocketAddress(ueAddr, sibPort));
            ++sibSent;
        }
        if (Simulator::Now() + Seconds(sibPeriodS) < Seconds(dataPlaneS))
        {
            Simulator::Schedule(Seconds(sibPeriodS), sendSib);
        }
    };
    Simulator::Schedule(Seconds(0.5), sendSib);

    // DL and UL CBR flows with in-band SeqTsSizeHeader.
    const uint16_t dlPort = 4000;
    const uint16_t ulPort = 4001;
    DirStats dl;
    DirStats ul;
    PacketSinkHelper dlSinkH("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), dlPort));
    dlSinkH.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    ApplicationContainer dlSink = dlSinkH.Install(ueNodes.Get(0));
    dlSink.Get(0)->TraceConnectWithoutContext("RxWithSeqTsSize", MakeCallback(&DirStats::Rx, &dl));
    CbrHelper dlCbr("ns3::UdpSocketFactory", InetSocketAddress(ueAddr, dlPort));
    dlCbr.SetConstantTraffic(MilliSeconds(5), 1024);
    dlCbr.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    ApplicationContainer dlApp = dlCbr.Install(rs.GetRemoteHost());
    PacketSinkHelper ulSinkH("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), ulPort));
    ulSinkH.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    ApplicationContainer ulSink = ulSinkH.Install(rs.GetRemoteHost());
    ulSink.Get(0)->TraceConnectWithoutContext("RxWithSeqTsSize", MakeCallback(&DirStats::Rx, &ul));
    CbrHelper ulCbr("ns3::UdpSocketFactory",
                    InetSocketAddress(rs.GetRemoteHostIpv4Address(), ulPort));
    ulCbr.SetConstantTraffic(MilliSeconds(20), 256);
    ulCbr.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    ApplicationContainer ulApp = ulCbr.Install(ueNodes.Get(0));
    const Time trafficStart = Seconds(1.0);
    const Time trafficStop = Seconds(dataPlaneS - 0.5);
    dlSink.Start(Seconds(0.0));
    ulSink.Start(Seconds(0.0));
    dlApp.Start(trafficStart);
    ulApp.Start(trafficStart);
    dlApp.Stop(trafficStop);
    ulApp.Stop(trafficStop);
    rs.AttachExternalDlFlow(0, dlApp.Get(0), dlSink.Get(0), trafficStart, trafficStop);

    std::ofstream dp(outputDir + "/ul_timing_dataplane.csv");
    dp << "t_s,t_study_s,age_s,elev_ue_deg,ta_true_us,err_ns,synced,sib19_decoded,"
          "ue_sinr_db,koffset_slots\n";
    dp << std::setprecision(10);
    uint64_t dpSamples = 0;
    uint64_t dpReads = 0;
    uint64_t dpViol = 0;
    double dpMaxAbs = 0.0;
    std::function<void()> sample2 = [&]() {
        const Vector sp = sat2->GetPosition();
        const Vector up = ue2->GetPosition();
        const double eu = ElevationDeg(up, sp);
        if (decoded.have &&
            (decoded.fresh &&
             (!pc2->HasSib19() || !pc2->IsUplinkSynchronized() ||
              pc2->GetInformationAge().GetSeconds() >= readPeriodS)))
        {
            pc2->ReceiveSib19(decoded.sib);
            decoded.fresh = false;
            ++dpReads;
        }
        if (pc2->HasSib19())
        {
            double truth = 2.0 * CalculateDistance(sp, up) / kC;
            if (ref == "gateway")
            {
                truth += 2.0 * CalculateDistance(sp, gw2Pos) / kC;
            }
            const double err = (pc2->ComputeTa().GetSeconds() - truth) * 1e9;
            dpMaxAbs = std::max(dpMaxAbs, std::fabs(err));
            dpViol += (std::fabs(err) > teNs) ? 1 : 0;
            ++dpSamples;
            dp << Simulator::Now().GetSeconds() << "," << t0 + Simulator::Now().GetSeconds()
               << "," << pc2->GetInformationAge().GetSeconds() << "," << eu << ","
               << truth * 1e6 << "," << err << "," << pc2->IsUplinkSynchronized() << ","
               << decoded.received << "," << rs.GetUeRecentSinrDb(0) << ","
               << rs.GetConsumedKOffsetSlots() << "\n";
        }
        if (Simulator::Now() + MilliSeconds(100) < Seconds(dataPlaneS))
        {
            Simulator::Schedule(MilliSeconds(100), sample2);
        }
    };
    Simulator::Schedule(MilliSeconds(550), sample2);

    Simulator::Stop(Seconds(dataPlaneS));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    const uint64_t dlSent = DynamicCast<CbrApplication>(dlApp.Get(0))->GetSent() / 1024;
    const uint64_t ulSent = DynamicCast<CbrApplication>(ulApp.Get(0))->GetSent() / 256;
    std::cout << "dataplane window=[" << t0 << "," << t0 + dataPlaneS << "] s"
              << " sib19_sent=" << sibSent << " sib19_decoded=" << decoded.received
              << " sib19_parse_fail=" << decoded.parseFailures << " reads=" << dpReads
              << " samples=" << dpSamples << " max_abs_err_ns=" << dpMaxAbs << " viol=" << dpViol
              << " sib19_koffset_ref_slots=" << sib2->GetLatest().cellSpecificKoffset
              << " koffset_cell_slots=" << rs.GetConsumedKOffsetSlots() << "\n"
              << "dataplane dl: sent=" << dlSent << " rx=" << dl.rx << " loss=" << dl.Loss(dlSent)
              << " owd_ms=" << (dl.rx ? dl.sumDelayMs / dl.rx : 0.0)
              << " ipdv_ms=" << (dl.ipdvN ? dl.ipdvSumMs / dl.ipdvN : 0.0)
              << " sinr_db=" << rs.GetMeanDlSinrDb() << " tbler=" << rs.GetMeanDlTbler() << "\n"
              << "dataplane ul: sent=" << ulSent << " rx=" << ul.rx << " loss=" << ul.Loss(ulSent)
              << " owd_ms=" << (ul.rx ? ul.sumDelayMs / ul.rx : 0.0)
              << " ipdv_ms=" << (ul.ipdvN ? ul.ipdvSumMs / ul.ipdvN : 0.0)
              << " pusch_tbs=" << ulTb.tbs << " pusch_corrupt=" << ulTb.corrupt
              << " pusch_sinr_db=" << ulTb.MeanSinrDb()
              << " pusch_mean_rb=" << (ulTb.tbs ? static_cast<double>(ulTb.sumRb) / ulTb.tbs : 0.0)
              << " pusch_mean_mcs="
              << (ulTb.tbs ? static_cast<double>(ulTb.sumMcs) / ulTb.tbs : 0.0) << "\n"
              << "dataplane outputs in " << outputDir << "\n";
    sibTx->Close();
    sibRx->Close();
    Simulator::Destroy();
    return 0;
}
