/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ntn-ul-timing-precompensator.h"

#include "ntn-timing-advance.h"

#include <ns3/log.h>
#include <ns3/mobility-model.h>
#include <ns3/simulator.h>

#include <cmath>

namespace ns3
{
namespace ntnrrc
{

NS_LOG_COMPONENT_DEFINE("NtnUlTimingPrecompensator");
NS_OBJECT_ENSURE_REGISTERED(NtnUlTimingPrecompensator);

namespace
{
constexpr double kMu = 3.986004418e14;
constexpr double kRe = 6378137.0;
constexpr double kJ2 = 1.08262668e-3;
constexpr double kOmegaE = 7.2921150e-5;

void
Accel(const double* x, bool j2, double* a)
{
    const double r2 = x[0] * x[0] + x[1] * x[1] + x[2] * x[2];
    const double r = std::sqrt(r2);
    const double k = -kMu / (r2 * r);
    a[0] = k * x[0];
    a[1] = k * x[1];
    a[2] = k * x[2];
    if (j2)
    {
        const double z2 = x[2] * x[2] / r2;
        const double f = 1.5 * kJ2 * kMu * kRe * kRe / (r2 * r2 * r);
        a[0] += f * x[0] * (5 * z2 - 1);
        a[1] += f * x[1] * (5 * z2 - 1);
        a[2] += f * x[2] * (5 * z2 - 3);
    }
}

void
Deriv(const double* y, bool j2, double* dy)
{
    dy[0] = y[3];
    dy[1] = y[4];
    dy[2] = y[5];
    Accel(y, j2, dy + 3);
}

void
Rk4(double* y, double dt, bool j2)
{
    double k1[6], k2[6], k3[6], k4[6], t[6];
    Deriv(y, j2, k1);
    for (int i = 0; i < 6; ++i)
    {
        t[i] = y[i] + 0.5 * dt * k1[i];
    }
    Deriv(t, j2, k2);
    for (int i = 0; i < 6; ++i)
    {
        t[i] = y[i] + 0.5 * dt * k2[i];
    }
    Deriv(t, j2, k3);
    for (int i = 0; i < 6; ++i)
    {
        t[i] = y[i] + dt * k3[i];
    }
    Deriv(t, j2, k4);
    for (int i = 0; i < 6; ++i)
    {
        y[i] += dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
}
} // namespace

TypeId
NtnUlTimingPrecompensator::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntnrrc::NtnUlTimingPrecompensator")
                            .SetParent<Object>()
                            .SetGroupName("NtnRrc")
                            .AddConstructor<NtnUlTimingPrecompensator>();
    return tid;
}

NtnUlTimingPrecompensator::NtnUlTimingPrecompensator()
    : m_noise(CreateObject<NormalRandomVariable>())
{
}

NtnUlTimingPrecompensator::~NtnUlTimingPrecompensator() = default;

int64_t
NtnUlTimingPrecompensator::AssignStreams(int64_t stream)
{
    m_noise->SetStream(stream);
    return 1;
}

void
NtnUlTimingPrecompensator::SetUeMobility(Ptr<MobilityModel> ue)
{
    m_ue = ue;
}

void
NtnUlTimingPrecompensator::SetGnss(Time fixPeriod, double sigmaHorizontalM)
{
    m_fixPeriod = fixPeriod;
    m_sigmaH = sigmaHorizontalM;
}

void
NtnUlTimingPrecompensator::ReceiveSib19(const Sib19Content& sib)
{
    m_sib = sib;
    m_have = true;
    // initial integration state from the broadcast ECEF state vector
    const Vector& p = sib.ephemeris.positionEcefM;
    const Vector& v = sib.ephemeris.velocityEcefMps;
    m_cacheTau = 0.0;
    m_y[0] = p.x;
    m_y[1] = p.y;
    m_y[2] = p.z;
    if (m_model == UeEphemerisModel::Kepler || m_model == UeEphemerisModel::KeplerJ2)
    {
        // inertial frame aligned with ECEF at the epoch: v_i = v_ecef + omega x r
        m_y[3] = v.x - kOmegaE * p.y;
        m_y[4] = v.y + kOmegaE * p.x;
        m_y[5] = v.z;
    }
    else
    {
        m_y[3] = v.x;
        m_y[4] = v.y;
        m_y[5] = v.z;
    }
}

void
NtnUlTimingPrecompensator::UpdateGnssFix()
{
    NS_ASSERT_MSG(m_ue, "UE mobility must be set");
    const Time now = Simulator::Now();
    if (now - m_lastFix < m_fixPeriod)
    {
        return;
    }
    m_lastFix = now;
    const Vector p = m_ue->GetPosition();
    // local east-north-up noise mapped to ECEF
    const double r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    const double lat = std::asin(p.z / r);
    const double lon = std::atan2(p.y, p.x);
    const double e = m_sigmaH > 0 ? m_noise->GetValue(0.0, m_sigmaH * m_sigmaH) : 0.0;
    const double n = m_sigmaH > 0 ? m_noise->GetValue(0.0, m_sigmaH * m_sigmaH) : 0.0;
    const double u = m_sigmaH > 0 ? m_noise->GetValue(0.0, 0.25 * m_sigmaH * m_sigmaH) : 0.0;
    const double sl = std::sin(lat);
    const double cl = std::cos(lat);
    const double so = std::sin(lon);
    const double co = std::cos(lon);
    m_fix = Vector(p.x - so * e - sl * co * n + cl * co * u,
                   p.y + co * e - sl * so * n + cl * so * u,
                   p.z + cl * n + sl * u);
    m_fixVel = m_ue->GetVelocity();
}

void
NtnUlTimingPrecompensator::Propagate(double tau)
{
    if (m_model == UeEphemerisModel::Linear)
    {
        return; // closed form in PredictSatellitePosition
    }
    if (tau < m_cacheTau)
    {
        ReceiveSib19(m_sib); // restart from the epoch
    }
    const bool j2 = (m_model == UeEphemerisModel::KeplerJ2);
    const double h = m_step.GetSeconds();
    while (m_cacheTau + 1e-12 < tau)
    {
        const double dt = std::min(h, tau - m_cacheTau);
        Rk4(m_y, dt, j2);
        m_cacheTau += dt;
    }
}

Vector
NtnUlTimingPrecompensator::PredictSatellitePosition()
{
    NS_ASSERT_MSG(m_have, "no SIB19 decoded");
    const double tau = (Simulator::Now() - m_sib.ephemeris.epoch).GetSeconds();
    const Vector& p = m_sib.ephemeris.positionEcefM;
    const Vector& v = m_sib.ephemeris.velocityEcefMps;
    if (m_model == UeEphemerisModel::Linear)
    {
        return Vector(p.x + v.x * tau, p.y + v.y * tau, p.z + v.z * tau);
    }
    Propagate(tau);
    if (m_model == UeEphemerisModel::GravityEcef)
    {
        return Vector(m_y[0], m_y[1], m_y[2]);
    }
    // inertial -> ECEF, the Earth has turned by omega * tau since the epoch
    const double th = kOmegaE * tau;
    const double c = std::cos(th);
    const double s = std::sin(th);
    return Vector(c * m_y[0] + s * m_y[1], -s * m_y[0] + c * m_y[1], m_y[2]);
}

Time
NtnUlTimingPrecompensator::ComputeTa()
{
    UpdateGnssFix();
    const Vector s = PredictSatellitePosition();
    const double tau = (Simulator::Now() - m_sib.ephemeris.epoch).GetSeconds();
    Vector p = m_fix;
    if (m_project)
    {
        const double age = (Simulator::Now() - m_lastFix).GetSeconds();
        p = Vector(p.x + m_fixVel.x * age, p.y + m_fixVel.y * age, p.z + m_fixVel.z * age);
    }
    const double dx = s.x - p.x;
    const double dy = s.y - p.y;
    const double dz = s.z - p.z;
    const double service = 2.0 * std::sqrt(dx * dx + dy * dy + dz * dz) /
                           NtnTimingAdvance::kSpeedOfLight;
    const double common = m_sib.taCommon.GetSeconds() + m_sib.taCommonDriftRate * tau +
                          m_sib.taCommonDriftVariation * tau * tau;
    return Seconds(service + common);
}

Time
NtnUlTimingPrecompensator::GetInformationAge() const
{
    return m_have ? Simulator::Now() - m_sib.ephemeris.epoch : Time::Max();
}

bool
NtnUlTimingPrecompensator::IsUplinkSynchronized() const
{
    return m_have && GetInformationAge() <= m_sib.ulSyncValidity;
}

} // namespace ntnrrc
} // namespace ns3
