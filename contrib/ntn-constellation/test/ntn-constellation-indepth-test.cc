// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// In-depth ntn-constellation validation (18 Nov 2026 test campaign).
//
// The orbit cases propagate inside Simulator::Run() and check the measured
// motion against closed-form astrodynamics (Vallado, "Fundamentals of
// Astrodynamics and Applications", 4th ed.):
//   - Walker-Delta T/P/F geometry (Walker 1984): RAAN step 360/P, in-plane step
//     360/S, inter-plane phase F x 360/T;
//   - Kepler's third law and the J2 draconic period (Vallado eq. 9-41);
//   - J2 secular RAAN drift (Vallado eq. 9-41): -4.5 deg/day for a 550 km,
//     53 deg shell, +0.9856 deg/day for a 550 km sun-synchronous orbit;
//   - the vis-viva equation at perigee and apogee.
// The link cases carry real packets: XnAP (TS 38.423 clause 8.2) over an ISL
// timed by NtnRangeDelayChannel, and a Ka downlink to a WGS-84 ground terminal
// from ntn-cho with the TR 38.821 budget, measured with FlowMonitor.

#include "ns3/contact-graph-scheduler.h"
#include "ns3/error-model.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ntn-geodetic-fixed-mobility-model.h"
#include "ns3/ntn-range-delay-channel.h"
#include "ns3/ntn-sat-link-error-model.h"
#include "ns3/ntn-xn-handover.h"
#include "ns3/ntn-xnap-messages.h"
#include "ns3/on-off-helper.h"
#include "ns3/orbital-elements.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/pointer.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <tuple>
#include <vector>

using namespace ns3;
using namespace ns3::ntncon;

namespace
{

constexpr double kC = 299792458.0;
constexpr double kDeg = M_PI / 180.0;

KeplerianElements
Circular(double altKm, double incDeg, double raanDeg = 0.0, double maDeg = 0.0, double e = 0.0)
{
    KeplerianElements el;
    el.semi_major_axis_m = kEarthRadiusM + altKm * 1e3;
    el.eccentricity = e;
    el.inclination_rad = incDeg * kDeg;
    el.raan_rad = raanDeg * kDeg;
    el.mean_anomaly_rad = maDeg * kDeg;
    el.epoch_unix_s = 1767225600.0; // 2026-01-01T00:00:00Z
    return el;
}

double
Angle(const Vector& a, const Vector& b)
{
    const double c = (a.x * b.x + a.y * b.y + a.z * b.z) / (a.GetLength() * b.GetLength());
    return std::acos(std::max(-1.0, std::min(1.0, c)));
}

Vector
Cross(const Vector& a, const Vector& b)
{
    return Vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

/// RAAN (rad) of the orbit plane through two nearby ECI positions.
double
RaanFrom(const Vector& r0, const Vector& r1)
{
    const Vector h = Cross(r0, r1);
    return std::atan2(h.x, -h.y);
}

/// J2 RAAN rate (rad/s), Vallado eq. 9-41.
double
RaanDotJ2(const KeplerianElements& el)
{
    const double p = el.semi_major_axis_m * (1 - el.eccentricity * el.eccentricity);
    return -1.5 * kEarthJ2 * std::pow(kEarthRadiusM / p, 2) * el.MeanMotionRadS() *
           std::cos(el.inclination_rad);
}

} // namespace

// ---------------------------------------------------------------------------
/// Walker-Delta 24/3/1 at 53 deg: element-level geometry, then inside a run the
/// in-plane neighbours keep their 45 deg spacing and adjacent planes keep the
/// F x 360 / T = 15 deg phase for a whole orbit. Failure path: T not divisible
/// by P returns nothing. Walker-Star spreads the planes over 180 deg.
class ConWalkerGeometryTest : public TestCase
{
  public:
    ConWalkerGeometryTest()
        : TestCase("Walker T/P/F geometry holds at build time and through a propagated orbit")
    {
    }

  private:
    void DoRun() override
    {
        WalkerConfig w;
        w.total_sats = 24;
        w.num_planes = 3;
        w.phasing_f = 1;
        w.altitude_km = 550.0;
        w.inclination_deg = 53.0;
        w.epoch_unix_s = 1767225600.0;
        auto el = WalkerConstellation::BuildDelta(w);
        NS_TEST_ASSERT_MSG_EQ(el.size(), 24u, "T satellites");
        for (uint32_t p = 0; p < 3; ++p)
        {
            for (uint32_t s = 0; s < 8; ++s)
            {
                const auto& e = el[p * 8 + s];
                NS_TEST_ASSERT_MSG_EQ_TOL(e.raan_rad / kDeg, 120.0 * p, 1e-9, "RAAN step 360/P");
                NS_TEST_ASSERT_MSG_EQ_TOL(e.mean_anomaly_rad / kDeg,
                                          std::fmod(45.0 * s + 15.0 * p, 360.0), 1e-9,
                                          "in-plane 360/S plus phase F 360/T");
            }
        }
        WalkerConfig bad = w;
        bad.total_sats = 25;
        NS_TEST_ASSERT_MSG_EQ(WalkerConstellation::BuildDelta(bad).size(), 0u, "T % P != 0");
        WalkerConfig star = w;
        star.inclination_deg = 87.0;
        auto st = WalkerConstellation::BuildStar(star);
        NS_TEST_ASSERT_MSG_EQ_TOL(st[16].raan_rad / kDeg, 120.0, 1e-9, "Star: 180/P = 60 deg step");
        NS_TEST_ASSERT_MSG_EQ_TOL(st[8].raan_rad / kDeg, 60.0, 1e-9, "Star plane 1 at 60 deg");

        std::vector<Ptr<Sgp4MobilityModel>> sats;
        for (const auto& e : el)
        {
            Ptr<Sgp4MobilityModel> m = CreateObject<Sgp4MobilityModel>();
            m->SetElementsSgp4(e);
            sats.push_back(m);
        }
        double maxInPlaneErr = 0;
        for (int t = 0; t <= 5800; t += 290)
        {
            Simulator::Schedule(Seconds(t), [&]() {
                for (uint32_t p = 0; p < 3; ++p)
                {
                    for (uint32_t s = 0; s < 8; ++s)
                    {
                        const Vector a = sats[p * 8 + s]->GetEciPosition();
                        const Vector b = sats[p * 8 + (s + 1) % 8]->GetEciPosition();
                        maxInPlaneErr = std::max(maxInPlaneErr, std::abs(Angle(a, b) / kDeg - 45.0));
                    }
                }
            });
        }
        Simulator::Run();
        Simulator::Destroy();
        // SGP4 short-period terms move satellites by a few km along track
        NS_TEST_ASSERT_MSG_LT(maxInPlaneErr, 0.15, "in-plane spacing stays 45 deg over an orbit");
    }
};

// ---------------------------------------------------------------------------
/// The nodal (draconic) period measured from ascending-node crossings inside a
/// run equals Kepler's third law T = 2 pi sqrt(a^3 / mu) to within the J2
/// correction, and matches the J2 draconic period (Vallado eq. 9-41).
class ConNodalPeriodTest : public TestCase
{
  public:
    ConNodalPeriodTest()
        : TestCase("Measured nodal period matches Kepler's third law and the J2 draconic period")
    {
    }

  private:
    void DoRun() override
    {
        for (bool sgp4 : {false, true})
        {
            KeplerianElements el = Circular(550.0, 53.0, 0.0, 90.0);
            Ptr<Sgp4MobilityModel> m = CreateObject<Sgp4MobilityModel>();
            if (sgp4)
            {
                NS_TEST_ASSERT_MSG_EQ(m->SetElementsSgp4(el), true, "SGP4 backend");
            }
            else
            {
                m->SetElements(el);
            }
            std::vector<double> crossings;
            double lastZ = 0;
            bool have = false;
            for (int t = 0; t <= 12000; ++t)
            {
                Simulator::Schedule(Seconds(t), [&, t]() {
                    const double z = m->GetEciPosition().z;
                    if (have && lastZ < 0 && z >= 0)
                    {
                        crossings.push_back(t - 1 + (-lastZ) / (z - lastZ));
                    }
                    lastZ = z;
                    have = true;
                });
            }
            Simulator::Run();
            Simulator::Destroy();
            NS_TEST_ASSERT_MSG_EQ(crossings.size(), 2u, "two ascending nodes in 12000 s");
            if (crossings.size() < 2)
            {
                continue;
            }
            const double tn = crossings[1] - crossings[0];
            const double a = el.semi_major_axis_m;
            const double tk = 2 * M_PI * std::sqrt(a * a * a / kEarthMuM3S2);
            NS_TEST_ASSERT_MSG_EQ_TOL(tk, 5739.0, 1.0, "Kepler period of a 550 km orbit (95.65 min)");
            NS_TEST_ASSERT_MSG_EQ_TOL(tn, tk, 0.002 * tk, "nodal period within 0.2% of Kepler");
            // draconic: n + dM/dt(J2) + domega/dt(J2)
            const double n = el.MeanMotionRadS();
            const double k = 0.75 * kEarthJ2 * std::pow(kEarthRadiusM / a, 2) * n;
            const double s2 = std::pow(std::sin(el.inclination_rad), 2);
            const double mdot = n + k * 2.0 * (1.0 - 1.5 * s2);
            const double wdot = k * (4.0 - 5.0 * s2);
            const double td = 2 * M_PI / (mdot + wdot);
            NS_TEST_ASSERT_MSG_EQ_TOL(tn, td, sgp4 ? 3.0 : 1.0,
                                      (sgp4 ? "SGP4" : "Kepler+J2")
                                          << " nodal period equals the J2 draconic period");
        }
    }
};

// ---------------------------------------------------------------------------
/// J2 nodal regression measured over one simulated day: the 550 km, 53 deg
/// Starlink-like shell drifts at -4.49 deg/day; a 550 km orbit at 97.59 deg is
/// sun-synchronous (+0.9856 deg/day = 360 deg per tropical year).
class ConJ2PrecessionTest : public TestCase
{
  public:
    ConJ2PrecessionTest()
        : TestCase("J2 RAAN drift over a day: -4.49 deg/day at 53 deg, sun-synchronous at 97.59 deg")
    {
    }

  private:
    /// Mean nodal drift in deg/day, from the RAAN at an ascending-node
    /// crossing near t = 0 and at one near t = 1 day.
    ///
    /// The plane through two nearby positions (r(t) x r(t + dt), as this test
    /// first did) is the OSCULATING plane, and the osculating node of a J2
    /// orbit oscillates about the mean node with the argument of latitude u:
    /// in the Kepler+J2 model the plane turns at Omega_dot while the satellite
    /// moves, so r x dr/dt picks up Omega_dot z x r, a tilt of order
    /// Omega_dot / n (0.05 deg) times a function of u; real J2 adds its own
    /// short-period node terms (Brouwer), proportional to sin 2u for a circular
    /// orbit. Sampled at u = 0 (t = 0) and at whatever u the satellite had one
    /// day later, the difference read 0.38 % low (-4.472 against -4.489
    /// deg/day) for any dt. At a node crossing u = 0, both effects vanish and
    /// the RAAN is atan2(y, x) of the crossing point, so measuring node to node
    /// returns the secular rate the closed form describes.
    double Measure(const KeplerianElements& el, bool sgp4)
    {
        Ptr<Sgp4MobilityModel> m = CreateObject<Sgp4MobilityModel>();
        if (sgp4)
        {
            m->SetElementsSgp4(el);
        }
        else
        {
            m->SetElements(el);
        }
        // (time, RAAN) of the first ascending node in each window.
        std::vector<std::pair<double, double>> nodes;
        for (double w0 : {0.0, 86400.0})
        {
            auto prev = std::make_shared<std::pair<double, Vector>>(-1.0, Vector());
            auto done = std::make_shared<bool>(false);
            for (int k = 0; k <= 6000; ++k)
            {
                const double t = w0 + k;
                Simulator::Schedule(Seconds(t), [&nodes, m, t, prev, done]() {
                    const Vector r = m->GetEciPosition();
                    if (!*done && prev->first >= 0.0 && prev->second.z < 0.0 && r.z >= 0.0)
                    {
                        // Linear interpolation to z = 0 over a 1 s step: the
                        // chord point's direction differs from the arc's by
                        // (n dt)^2 / 8 ~ 1e-7 rad.
                        const double f = -prev->second.z / (r.z - prev->second.z);
                        const double x = prev->second.x + f * (r.x - prev->second.x);
                        const double y = prev->second.y + f * (r.y - prev->second.y);
                        nodes.emplace_back(prev->first + f * (t - prev->first), std::atan2(y, x));
                        *done = true;
                    }
                    *prev = {t, r};
                });
            }
        }
        Simulator::Run();
        Simulator::Destroy();
        if (nodes.size() != 2)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }
        const double d = std::remainder(nodes[1].second - nodes[0].second, 2 * M_PI);
        return d / (nodes[1].first - nodes[0].first) * 86400.0 / kDeg; // deg per day
    }

    void DoRun() override
    {
        const KeplerianElements shell = Circular(550.0, 53.0, 30.0);
        const double analytic = RaanDotJ2(shell) * 86400.0 / kDeg;
        NS_TEST_ASSERT_MSG_EQ_TOL(analytic, -4.49, 0.03, "published Starlink shell drift");
        // The Kepler+J2 model applies exactly this rate to the mean node, so the
        // node-to-node measurement must agree to the interpolation error.
        NS_TEST_ASSERT_MSG_EQ_TOL(Measure(shell, false), analytic, 0.001 * std::abs(analytic),
                                  "Kepler+J2 drift within 0.1%");
        // SGP4 adds J4 and second-order J2 secular terms and its own
        // (Brouwer) mean motion (Hoots and Roehrich 1980), each of order
        // J4/J2 (R/p)^2 or J2, about 0.1 %, relative to the first-order rate;
        // measured 0.013 % here and 0.17 % for the SSO below. Bound: 0.3 %.
        NS_TEST_ASSERT_MSG_EQ_TOL(Measure(shell, true), analytic, 0.003 * std::abs(analytic),
                                  "SGP4 drift (J2 + J4) within 0.3%");

        const KeplerianElements sso = Circular(550.0, 97.59, 30.0);
        const double analyticSso = RaanDotJ2(sso) * 86400.0 / kDeg;
        NS_TEST_ASSERT_MSG_EQ_TOL(analyticSso, 0.9856, 0.01, "97.59 deg at 550 km is sun-synchronous");
        NS_TEST_ASSERT_MSG_EQ_TOL(Measure(sso, false), analyticSso, 0.001 * std::abs(analyticSso),
                                  "Kepler+J2 SSO drift within 0.1%");
        NS_TEST_ASSERT_MSG_EQ_TOL(Measure(sso, true), analyticSso, 0.003 * std::abs(analyticSso),
                                  "SGP4 SSO drift within 0.3%");

        // a polar orbit does not precess (cos 90 deg = 0)
        NS_TEST_ASSERT_MSG_EQ_TOL(Measure(Circular(550.0, 90.0, 30.0), false), 0.0, 1e-6,
                                  "polar orbit: no nodal drift");
    }
};

// ---------------------------------------------------------------------------
/// Vis-viva on an eccentric orbit (a = 7378 km, e = 0.05) at the critical
/// inclination 63.435 deg, where J2 does not rotate the apsides: the ECI speed
/// measured by differencing positions inside a run equals
/// v^2 = mu (2 / r - 1 / a) at perigee and apogee, and r_p, r_a = a(1 -/+ e).
class ConVisVivaTest : public TestCase
{
  public:
    ConVisVivaTest()
        : TestCase("Eccentric-orbit speed and radius follow vis-viva at perigee and apogee")
    {
    }

  private:
    void DoRun() override
    {
        KeplerianElements el = Circular(1000.0, 63.4349, 0.0, 0.0, 0.05);
        Ptr<Sgp4MobilityModel> m = CreateObject<Sgp4MobilityModel>();
        m->SetElements(el);
        const double a = el.semi_major_axis_m;
        const double half = el.PeriodSeconds() / 2.0;
        std::vector<Vector> r;
        for (double t : {0.0, 0.1, half, half + 0.1})
        {
            Simulator::Schedule(Seconds(t), [&r, m]() { r.push_back(m->GetEciPosition()); });
        }
        Simulator::Run();
        Simulator::Destroy();
        const double rp = r[0].GetLength();
        const double ra = r[2].GetLength();
        NS_TEST_ASSERT_MSG_EQ_TOL(rp, a * 0.95, 1.0, "perigee radius a(1 - e)");
        NS_TEST_ASSERT_MSG_EQ_TOL(ra, a * 1.05, 1.0, "apogee radius a(1 + e)");
        const double vp = CalculateDistance(r[1], r[0]) / 0.1;
        const double va = CalculateDistance(r[3], r[2]) / 0.1;
        NS_TEST_ASSERT_MSG_EQ_TOL(vp, std::sqrt(kEarthMuM3S2 * (2 / rp - 1 / a)), 5.0,
                                  "vis-viva perigee (nodal drift adds under 4 m/s)");
        NS_TEST_ASSERT_MSG_EQ_TOL(va, std::sqrt(kEarthMuM3S2 * (2 / ra - 1 / a)), 5.0,
                                  "vis-viva apogee");
        // angular momentum conservation: r_p v_p = r_a v_a
        NS_TEST_ASSERT_MSG_EQ_TOL(rp * vp, ra * va, 1e-3 * rp * vp, "Kepler's second law");
    }
};

// ---------------------------------------------------------------------------
/// fixes-3b: the Kepler+J2 velocity is the time derivative of the propagated
/// position. The node regresses (Omega_dot) and the perigee advances
/// (omega_dot), so d(pos)/dt carries Omega_dot z x r + omega_dot h x r besides
/// the in-orbit term; without them the reported velocity was about 5 m/s off on
/// a 550 km, 53 deg shell. Inside a run, at 13 instants over an orbit, a central
/// difference of GetEcefStateAt(+-h) with h = 2^-10 s (about 1 ms; a power of
/// two keeps the Unix-second sums exact in double) must equal GetVelocity()
/// (ECEF, what Doppler consumers read) within 1 cm/s, for a 53 deg shell, a
/// sun-synchronous orbit, a polar orbit and an eccentric (e = 0.05) orbit.
/// The truncation error of the difference is |jerk| h^2 / 6, about 1e-9 m/s.
/// The omitted J2 terms are computed in closed form and shown to be more than
/// 100 times the tolerance, so the check could not pass with them left out.
class ConVelocityIsDerivativeTest : public TestCase
{
  public:
    ConVelocityIsDerivativeTest()
        : TestCase("Kepler+J2 velocity equals the 1 ms central difference of the position within 1 cm/s")
    {
    }

  private:
    void DoRun() override
    {
        const std::vector<KeplerianElements> orbits = {Circular(550.0, 53.0, 30.0, 10.0),
                                                       Circular(550.0, 97.59, 30.0, 10.0),
                                                       Circular(550.0, 90.0, 30.0, 10.0),
                                                       [] {
                                                           KeplerianElements e =
                                                               Circular(1000.0, 53.0, 30.0, 10.0, 0.05);
                                                           e.arg_perigee_rad = 40.0 * kDeg;
                                                           return e;
                                                       }()};
        const Time h = NanoSeconds(976562); // rounds to exactly 2^-10 s in the Unix-second sum
        const double hS = 1.0 / 1024.0;
        double worst = 0.0;
        uint32_t samples = 0;
        for (const auto& el : orbits)
        {
            Ptr<Sgp4MobilityModel> m = CreateObject<Sgp4MobilityModel>();
            m->SetElements(el);
            for (int k = 0; k <= 12; ++k)
            {
                Simulator::Schedule(Seconds(500.0 * k), [&, m]() {
                    Vector pp;
                    Vector pm;
                    Vector v;
                    m->GetEcefStateAt(h, pp, v);
                    m->GetEcefStateAt(NanoSeconds(-976562), pm, v);
                    const Vector fd((pp.x - pm.x) / (2 * hS),
                                    (pp.y - pm.y) / (2 * hS),
                                    (pp.z - pm.z) / (2 * hS));
                    const Vector rep = m->GetVelocity();
                    worst = std::max(worst, CalculateDistance(fd, rep));
                    Vector p0;
                    Vector v0;
                    m->GetEcefStateAt(Seconds(0), p0, v0);
                    worst = std::max(worst, CalculateDistance(fd, v0));
                    ++samples;
                });
            }
            Simulator::Run();
            Simulator::Destroy();
        }
        NS_TEST_ASSERT_MSG_EQ(samples, 4u * 13u, "every sample taken");
        NS_TEST_ASSERT_MSG_LT(worst, 0.01, "velocity = d(position)/dt within 1 cm/s");

        // Sensitivity: the J2 rotation terms on the 53 deg shell at the node.
        const KeplerianElements el = orbits[0];
        const double a = el.semi_major_axis_m;
        const double k = (kEarthRadiusM / a) * (kEarthRadiusM / a) * kEarthJ2 * el.MeanMotionRadS();
        const double raanDot = -1.5 * k * std::cos(el.inclination_rad);
        const double argpDot = 0.75 * k * (4.0 - 5.0 * std::pow(std::sin(el.inclination_rad), 2));
        // At the node r is along the node line, so |z x r| = a and |h x r| = a.
        const double omitted = a * std::hypot(raanDot, argpDot);
        NS_TEST_ASSERT_MSG_GT(omitted, 100 * 0.01, "the omitted terms (about 5 m/s) exceed 100x the tolerance");
    }
};

// ---------------------------------------------------------------------------
/// XnAP over a real ISL (TS 38.423 clause 8.2): the four handover messages
/// travel as UDP packets over NtnRangeDelayChannel between two SGP4
/// satellites. The handover completes in 4 x (light time + serialization),
/// carries the PDCP SN/HFN to the target (SN STATUS TRANSFER, clause 8.2.2),
/// and the failure paths behave as the procedure text says: a target that
/// rejects answers with HANDOVER PREPARATION FAILURE after one round trip; a
/// dead ISL triggers TXnRELOCprep and HANDOVER CANCEL; a bent-pipe payload
/// cannot originate Xn.
class ConXnOverIslTest : public TestCase
{
  public:
    ConXnOverIslTest()
        : TestCase("XnAP handover over a light-time ISL: timing, SN transfer, rejection, TXnRELOCprep")
    {
    }

  private:
    struct Gnb
    {
        Ptr<NtnXnHandover> xn;
        Ptr<Socket> sock;
        Ipv4Address peerAddr;
    };

    static void Send(Gnb* g, uint32_t /*peer*/, NtnXnapHeader h, Time /*delay*/)
    {
        Ptr<Packet> p = Create<Packet>();
        p->AddHeader(h);
        g->sock->SendTo(p, 0, InetSocketAddress(g->peerAddr, 38422));
    }

    static void Recv(Gnb* g, Ptr<Socket> s)
    {
        Ptr<Packet> p;
        while ((p = s->Recv()))
        {
            NtnXnapHeader h;
            p->RemoveHeader(h);
            g->xn->Receive(h);
        }
    }

    struct Result
    {
        bool done{false};
        bool ok{false};
        double elapsed{0};
        double oneWay{0};
        double txTime{0};
        uint32_t dlSnAtTarget{0};
        uint32_t hfnAtTarget{0};
        uint32_t cancels{0};
        uint32_t failures{0};
    };

    static void Done(Result* r, uint32_t, Time e, bool ok)
    {
        r->done = true;
        r->ok = ok;
        r->elapsed = e.GetSeconds();
    }

    Result RunOnce(bool admit, bool outage)
    {
        Result res;
        WalkerConfig w;
        w.total_sats = 20;
        w.num_planes = 1;
        w.altitude_km = 550.0;
        w.epoch_unix_s = 1767225600.0;
        auto el = WalkerConstellation::BuildDelta(w);
        NodeContainer n;
        n.Create(2);
        Ptr<Sgp4MobilityModel> ma = CreateObject<Sgp4MobilityModel>();
        ma->SetElementsSgp4(el[0]);
        Ptr<Sgp4MobilityModel> mb = CreateObject<Sgp4MobilityModel>();
        mb->SetElementsSgp4(el[1]);
        n.Get(0)->AggregateObject(ma);
        n.Get(1)->AggregateObject(mb);
        Ptr<NtnRangeDelayChannel> ch;
        const DataRate rate("100Mbps");
        NetDeviceContainer d = NtnRangeDelayChannel::Connect(n.Get(0), n.Get(1), rate, "100p", ch);
        InternetStackHelper inet;
        inet.Install(n);
        Ipv4AddressHelper ip;
        ip.SetBase("10.38.0.0", "255.255.255.252");
        Ipv4InterfaceContainer ifs = ip.Assign(d);
        if (outage)
        {
            for (uint32_t i = 0; i < 2; ++i)
            {
                Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
                em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
                em->SetRate(1.0);
                d.Get(i)->SetAttribute("ReceiveErrorModel", PointerValue(em));
            }
        }
        Gnb g[2];
        for (uint32_t i = 0; i < 2; ++i)
        {
            g[i].xn = CreateObject<NtnXnHandover>();
            g[i].xn->SetGnbId(100 + i);
            g[i].sock = Socket::CreateSocket(n.Get(i), UdpSocketFactory::GetTypeId());
            g[i].sock->Bind(InetSocketAddress(Ipv4Address::GetAny(), 38422));
            g[i].sock->SetRecvCallback(MakeBoundCallback(&Recv, &g[i]));
            g[i].peerAddr = ifs.GetAddress(1 - i);
            g[i].xn->SetSendCallback(MakeBoundCallback(&Send, &g[i]));
        }
        g[1].xn->SetAdmitIncoming(admit);
        g[0].xn->SetCompleteCallback(MakeBoundCallback(&Done, &res));

        Simulator::Schedule(Seconds(1), [&]() {
            res.oneWay = CalculateDistance(ma->GetPosition(), mb->GetPosition()) / kC;
            NtnXnapHeader probe;
            probe.SetMessageType(XnapMessageType::HandoverRequest);
            // UDP 8 + IPv4 20 + PPP 2 bytes around the XnAP header
            res.txTime = rate.CalculateBytesTxTime(probe.GetSerializedSize() + 30).GetSeconds();
            NS_TEST_EXPECT_MSG_EQ(g[0].xn->StartHandover(7, 101, 0x1234567ULL, 4095, 17, 3), true,
                                  "a regenerative source starts Xn");
        });
        Simulator::Stop(Seconds(3));
        Simulator::Run();
        res.dlSnAtTarget = g[1].xn->GetLastReceivedDlPdcpSn();
        res.hfnAtTarget = g[1].xn->GetLastReceivedHfn();
        res.cancels = g[0].xn->GetCancelsSent();
        res.failures = g[1].xn->GetFailuresSent();
        NS_TEST_EXPECT_MSG_EQ(g[0].xn->GetPendingCount(), 0u, "no handover left pending");
        Simulator::Destroy();
        return res;
    }

    void DoRun() override
    {
        Result ok = RunOnce(true, false);
        NS_TEST_ASSERT_MSG_EQ((ok.done && ok.ok), true, "handover completes");
        // 20-sat plane: 18 deg spacing, chord = 2 a sin(9 deg) = 2167 km, 7.23 ms
        NS_TEST_ASSERT_MSG_EQ_TOL(ok.oneWay, 7.23e-3, 0.03e-3, "in-plane ISL light time");
        NS_TEST_ASSERT_MSG_EQ_TOL(ok.elapsed, 4 * (ok.oneWay + ok.txTime), 2e-6,
                                  "Request, Ack, SN Status Transfer, Context Release: 4 hops");
        NS_TEST_ASSERT_MSG_EQ(ok.dlSnAtTarget, 4095u, "DL COUNT SN reaches the target");
        NS_TEST_ASSERT_MSG_EQ(ok.hfnAtTarget, 3u, "HFN reaches the target");

        Result rej = RunOnce(false, false);
        NS_TEST_ASSERT_MSG_EQ((rej.done && !rej.ok), true, "rejected handover reports failure");
        NS_TEST_ASSERT_MSG_EQ(rej.failures, 1u, "target sent HANDOVER PREPARATION FAILURE");
        NS_TEST_ASSERT_MSG_EQ_TOL(rej.elapsed, 2 * (rej.oneWay + rej.txTime), 2e-6,
                                  "failure arrives after one round trip");
        NS_TEST_ASSERT_MSG_EQ(rej.dlSnAtTarget, 0u, "no SN transfer after a rejection");

        Result dead = RunOnce(true, true);
        NS_TEST_ASSERT_MSG_EQ((dead.done && !dead.ok), true, "a dead ISL ends in failure, not a hang");
        NS_TEST_ASSERT_MSG_EQ_TOL(dead.elapsed, 1.0, 1e-9, "TXnRELOCprep (1 s) expires");
        NS_TEST_ASSERT_MSG_EQ(dead.cancels, 1u, "HANDOVER CANCEL sent on expiry");

        Ptr<NtnXnHandover> bent = CreateObject<NtnXnHandover>();
        bent->SetRegenerative(false);
        bent->SetSendCallback(MakeCallback(+[](uint32_t, NtnXnapHeader, Time) {}));
        NS_TEST_ASSERT_MSG_EQ(bent->StartHandover(1, 2, 0, 0, 0, 0), false,
                              "a bent-pipe payload has no on-board gNB for Xn");
        Ptr<NtnXnHandover> unwired = CreateObject<NtnXnHandover>();
        NS_TEST_ASSERT_MSG_EQ(unwired->StartHandover(1, 2, 0, 0, 0, 0), false,
                              "no transport: not started");
    }
};

// ---------------------------------------------------------------------------
/// Cross-module (ntn-cho WGS-84 ground terminal): a Ka downlink over one LEO
/// pass. UDP CBR through NtnRangeDelayChannel + NtnSatLinkErrorModel (TR
/// 38.821 Set-1 Ka, DVB-S2 MODCOD designed for 30 deg elevation, ITU-R
/// P.676/P.618 terms). The minimum measured delay is the altitude light time
/// at culmination; every packet arrives while the elevation is well above the
/// design point, and the link drops below it (failure path: low elevation).
class ConKaPassFlowMonitorTest : public TestCase
{
  public:
    ConKaPassFlowMonitorTest()
        : TestCase("Ka downlink over an SGP4 pass to a WGS-84 terminal: delay floor and elevation-gated loss")
    {
    }

  private:
    struct Bins
    {
        std::map<int, uint32_t> rx;
    };

    static void OnRx(Bins* b, Ptr<const Packet>, const Address&)
    {
        b->rx[static_cast<int>(Simulator::Now().GetSeconds())]++;
    }

    void DoRun() override
    {
        WalkerConfig w;
        w.total_sats = 10;
        w.num_planes = 1;
        w.altitude_km = 550.0;
        w.epoch_unix_s = 1767225600.0;
        auto el = WalkerConstellation::BuildDelta(w);
        Ptr<Sgp4MobilityModel> sat = CreateObject<Sgp4MobilityModel>();
        sat->SetElementsSgp4(el[0]);
        double lat;
        double lon;
        double alt;
        sat->GetGeodetic(lat, lon, alt);
        Ptr<NtnGeodeticFixedMobilityModel> gs = CreateObject<NtnGeodeticFixedMobilityModel>();
        gs->SetGeodetic(lat, lon, 0.0); // the satellite culminates at zenith at t = 0

        NodeContainer n;
        n.Create(2);
        n.Get(0)->AggregateObject(sat);
        n.Get(1)->AggregateObject(gs);
        Ptr<NtnRangeDelayChannel> ch;
        NetDeviceContainer d = NtnRangeDelayChannel::Connect(n.Get(0), n.Get(1), DataRate("100Mbps"), "100p", ch);
        Ptr<NtnSatLinkErrorModel> em = CreateObject<NtnSatLinkErrorModel>();
        em->ConfigureTr38821Set1KaGsl(true, 100e6, lat, 30.0, 550.0);
        em->SetEndpoints(sat, gs);
        d.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(em));
        InternetStackHelper inet;
        inet.Install(n);
        Ipv4AddressHelper ip;
        ip.SetBase("10.20.0.0", "255.255.255.252");
        Ipv4InterfaceContainer ifs = ip.Assign(d);

        OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(ifs.GetAddress(1), 9));
        onoff.SetConstantRate(DataRate("400kbps"), 500); // 100 packets per second
        ApplicationContainer src = onoff.Install(n.Get(0));
        src.Start(Seconds(0.0));
        src.Stop(Seconds(400.0));
        PacketSinkHelper ps("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 9));
        ApplicationContainer snk = ps.Install(n.Get(1));
        Bins bins;
        snk.Get(0)->TraceConnectWithoutContext("Rx", MakeBoundCallback(&OnRx, &bins));
        std::map<int, double> elev;
        for (int t = 0; t < 400; ++t)
        {
            Simulator::Schedule(Seconds(t + 0.5), [&elev, sat, lat, lon, t]() {
                elev[t] = sat->GetElevationDeg(lat, lon);
            });
        }
        FlowMonitorHelper fmh;
        Ptr<FlowMonitor> fm = fmh.InstallAll();
        Simulator::Stop(Seconds(401));
        Simulator::Run();
        uint64_t tx = 0;
        uint64_t rx = 0;
        for (const auto& [id, st] : fm->GetFlowStats())
        {
            tx += st.txPackets;
            rx += st.rxPackets;
        }
        const double minDelay = ch->GetMinDelayS();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ_TOL(minDelay, alt / kC, 2e-6, "delay floor = altitude / c at zenith");
        uint32_t highSec = 0;
        uint32_t highLoss = 0;
        uint32_t lowSec = 0;
        uint32_t lowRx = 0;
        for (const auto& [t, e] : elev)
        {
            if (t < 1 || t > 398)
            {
                continue;
            }
            const uint32_t got = bins.rx.count(t) ? bins.rx[t] : 0;
            if (e > 40.0)
            {
                ++highSec;
                highLoss += got < 100 ? 100 - got : 0;
            }
            else if (e < 15.0)
            {
                ++lowSec;
                lowRx += got;
            }
        }
        NS_TEST_ASSERT_MSG_GT(highSec, 60u, "a minute or more above 40 deg");
        NS_TEST_ASSERT_MSG_GT(lowSec, 30u, "the pass reaches below 15 deg");
        NS_TEST_ASSERT_MSG_EQ(highLoss, 0u, "no loss well above the 30 deg design point");
        NS_TEST_ASSERT_MSG_LT(lowRx, lowSec * 10, "under 15 deg the 30 deg MODCOD fails");
        NS_TEST_ASSERT_MSG_EQ(tx - rx, em->GetBlerDrops(), "every FlowMonitor loss is a BLER drop");
    }
};

// ---------------------------------------------------------------------------
/// Determinism: orbital contact events are a function of the geometry only.
/// The contact plan (time, endpoints, direction) is identical under different
/// RngRun values, and every GSL up event respects the elevation mask.
class ConContactPlanDeterminismTest : public TestCase
{
  public:
    ConContactPlanDeterminismTest()
        : TestCase("Contact plan is identical across RngRun values and respects the elevation mask")
    {
    }

  private:
    using Ev = std::tuple<double, uint32_t, uint32_t, bool, bool>;

    static void Rec(std::vector<Ev>* v, std::vector<double>* el, ContactEvent e)
    {
        v->emplace_back(e.timestamp_s, e.node_a, e.node_b, e.is_isl, e.up);
        if (!e.is_isl && e.up)
        {
            el->push_back(e.elevation_deg);
        }
    }

    std::vector<Ev> RunOnce(uint64_t run, std::vector<double>& upElev)
    {
        RngSeedManager::SetRun(run);
        WalkerConfig w;
        w.total_sats = 12;
        w.num_planes = 3;
        w.altitude_km = 550.0;
        w.epoch_unix_s = 1767225600.0;
        auto el = WalkerConstellation::BuildDelta(w);
        Ptr<ContactGraphScheduler> s = CreateObject<ContactGraphScheduler>();
        s->SetSamplingInterval(Seconds(5));
        s->SetMinElevationDeg(25.0);
        for (uint32_t i = 0; i < el.size(); ++i)
        {
            Ptr<Sgp4MobilityModel> m = CreateObject<Sgp4MobilityModel>();
            m->SetElementsSgp4(el[i]);
            s->RegisterSatellite(i + 1, m);
        }
        s->RegisterGroundStation(1000, 33.7, 73.0);
        s->RegisterGroundStation(1001, 51.5, -0.1);
        std::vector<Ev> ev;
        s->m_contactUp.ConnectWithoutContext(MakeBoundCallback(&Rec, &ev, &upElev));
        s->m_contactDown.ConnectWithoutContext(MakeBoundCallback(&Rec, &ev, &upElev));
        s->Start();
        Simulator::Stop(Seconds(5400));
        Simulator::Run();
        Simulator::Destroy();
        return ev;
    }

    void DoRun() override
    {
        const uint64_t saved = RngSeedManager::GetRun();
        std::vector<double> e1;
        std::vector<double> e2;
        auto a = RunOnce(1, e1);
        auto b = RunOnce(9, e2);
        RngSeedManager::SetRun(saved);
        NS_TEST_ASSERT_MSG_GT(a.size(), 10u, "contacts happen over 90 min");
        NS_TEST_ASSERT_MSG_EQ((a == b), true, "same plan under any RngRun");
        NS_TEST_ASSERT_MSG_GT(e1.size(), 0u, "some GSL comes up");
        for (double e : e1)
        {
            NS_TEST_ASSERT_MSG_GT_OR_EQ(e, 25.0 - 1e-6, "GSL up only above the elevation mask");
        }
    }
};

// ---------------------------------------------------------------------------
/// TLE failure paths inside a running propagation: mixed lines from two
/// objects, a corrupted checksum digit, a truncated line and an impossible
/// inclination are all refused, and a refused SetTle leaves the satellite on
/// its previous orbit.
class ConTleFailurePathsTest : public TestCase
{
  public:
    ConTleFailurePathsTest()
        : TestCase("Malformed TLEs are refused and never displace a propagating satellite")
    {
    }

  private:
    void DoRun() override
    {
        TleRecord iss;
        iss.line1 = "1 25544U 98067A   19282.92426354  .00001417  00000-0  29888-4 0  9991";
        iss.line2 = "2 25544  51.6447  91.8123 0007381 152.7392 207.3922 15.50284294192054";
        KeplerianElements el;
        NS_TEST_ASSERT_MSG_EQ(iss.ToKeplerian(el), true, "reference TLE parses");

        TleRecord mixed = iss;
        mixed.line2 = "2 25545  51.6447  91.8123 0007381 152.7392 207.3922 15.50284294192054";
        NS_TEST_ASSERT_MSG_EQ(mixed.ToKeplerian(el), false, "line 1 and line 2 of different objects");

        TleRecord trunc = iss;
        trunc.line2 = trunc.line2.substr(0, 60);
        NS_TEST_ASSERT_MSG_EQ(trunc.ToKeplerian(el), false, "truncated line");

        TleRecord inc = iss;
        inc.line2 = "2 25544 251.6447  91.8123 0007381 152.7392 207.3922 15.50284294192054";
        NS_TEST_ASSERT_MSG_EQ(inc.ToKeplerian(el), false, "inclination above 180 deg");

        TleRecord flip = iss;
        flip.line2[30] = flip.line2[30] == '1' ? '2' : '1'; // one eccentricity digit
        if (iss.ChecksumOk())
        {
            NS_TEST_ASSERT_MSG_EQ(flip.ChecksumOk(), false, "modulo-10 checksum catches one digit");
        }

        Ptr<Sgp4MobilityModel> m = CreateObject<Sgp4MobilityModel>();
        Ptr<Sgp4MobilityModel> ref = CreateObject<Sgp4MobilityModel>();
        NS_TEST_ASSERT_MSG_EQ(m->SetTle(iss), true, "valid TLE accepted");
        ref->SetTle(iss);
        double maxDev = 0;
        Simulator::Schedule(Seconds(60), [&]() {
            NS_TEST_EXPECT_MSG_EQ(m->SetTle(mixed), false, "refused mid-run");
            NS_TEST_EXPECT_MSG_EQ(m->SetTle(inc), false, "refused mid-run");
        });
        for (int t = 0; t <= 600; t += 30)
        {
            Simulator::Schedule(Seconds(t) + MilliSeconds(1), [&]() {
                maxDev = std::max(maxDev, CalculateDistance(m->GetPosition(), ref->GetPosition()));
            });
        }
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_LT(maxDev, 1e-3, "a refused TLE does not move the satellite");
    }
};

class NtnConstellationInDepthTestSuite : public TestSuite
{
  public:
    NtnConstellationInDepthTestSuite()
        : TestSuite("ntn-constellation-indepth", Type::UNIT)
    {
        AddTestCase(new ConWalkerGeometryTest, Duration::QUICK);
        AddTestCase(new ConNodalPeriodTest, Duration::QUICK);
        AddTestCase(new ConJ2PrecessionTest, Duration::QUICK);
        AddTestCase(new ConVisVivaTest, Duration::QUICK);
        AddTestCase(new ConVelocityIsDerivativeTest, Duration::QUICK);
        AddTestCase(new ConXnOverIslTest, Duration::QUICK);
        AddTestCase(new ConKaPassFlowMonitorTest, Duration::QUICK);
        AddTestCase(new ConContactPlanDeterminismTest, Duration::QUICK);
        AddTestCase(new ConTleFailurePathsTest, Duration::QUICK);
    }
};

static NtnConstellationInDepthTestSuite g_ntnConstellationInDepthTestSuite;
