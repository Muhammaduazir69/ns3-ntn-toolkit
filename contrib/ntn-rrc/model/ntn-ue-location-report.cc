/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-ue-location-report.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/mobility-model.h>
#include <ns3/simulator.h>

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnUeLocationReport");

namespace ntnrrc
{

namespace
{

constexpr double kWgs84A = 6378137.0;                        // semi-major axis (m)
constexpr double kWgs84F = 1.0 / 298.257223563;              // flattening
constexpr double kWgs84B = kWgs84A * (1.0 - kWgs84F);        // semi-minor (m)
constexpr double kWgs84E2 = kWgs84F * (2.0 - kWgs84F);       // first eccentricity^2
constexpr double kWgs84Ep2 = kWgs84E2 / (1.0 - kWgs84E2);    // second eccentricity^2
constexpr double kRadToDeg = 57.29577951308232;              // 180/pi
constexpr double kDegToRad = 0.017453292519943295;           // pi/180

} // namespace

void
EcefToGeodeticWgs84(const Vector& ecef, double& latDeg, double& lonDeg, double& altMetres)
{
    // Heikkinen 1982 — closed-form, no iteration.
    const double x = ecef.x;
    const double y = ecef.y;
    const double z = ecef.z;

    const double r = std::sqrt(x * x + y * y);
    if (r < 1e-9 && std::abs(z) < 1e-9)
    {
        latDeg = 0.0;
        lonDeg = 0.0;
        altMetres = -kWgs84A;
        return;
    }

    const double F = 54.0 * kWgs84B * kWgs84B * z * z;
    const double G = r * r + (1.0 - kWgs84E2) * z * z - kWgs84E2 * (kWgs84A * kWgs84A - kWgs84B * kWgs84B);
    const double c = (kWgs84E2 * kWgs84E2 * F * r * r) / (G * G * G);
    const double s = std::cbrt(1.0 + c + std::sqrt(c * c + 2.0 * c));
    const double P = F / (3.0 * std::pow(s + 1.0 + 1.0 / s, 2) * G * G);
    const double Q = std::sqrt(1.0 + 2.0 * kWgs84E2 * kWgs84E2 * P);
    const double r0 = -(P * kWgs84E2 * r) / (1.0 + Q) +
                      std::sqrt(0.5 * kWgs84A * kWgs84A * (1.0 + 1.0 / Q) -
                                (P * (1.0 - kWgs84E2) * z * z) / (Q * (1.0 + Q)) -
                                0.5 * P * r * r);
    const double Ux = r - kWgs84E2 * r0;
    const double U = std::sqrt(Ux * Ux + z * z);
    const double V = std::sqrt(Ux * Ux + (1.0 - kWgs84E2) * z * z);
    const double Z0 = (kWgs84B * kWgs84B * z) / (kWgs84A * V);

    altMetres = U * (1.0 - (kWgs84B * kWgs84B) / (kWgs84A * V));
    const double latRad = std::atan((z + kWgs84Ep2 * Z0) / r);
    const double lonRad = std::atan2(y, x);
    latDeg = latRad * kRadToDeg;
    lonDeg = lonRad * kRadToDeg;
}

Vector
GeodeticWgs84ToEcef(double latDeg, double lonDeg, double altMetres)
{
    const double latRad = latDeg * kDegToRad;
    const double lonRad = lonDeg * kDegToRad;
    const double sinLat = std::sin(latRad);
    const double cosLat = std::cos(latRad);
    const double N = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sinLat * sinLat);
    return Vector{(N + altMetres) * cosLat * std::cos(lonRad),
                  (N + altMetres) * cosLat * std::sin(lonRad),
                  (N * (1.0 - kWgs84E2) + altMetres) * sinLat};
}

NS_OBJECT_ENSURE_REGISTERED(NtnUeLocationReporter);

TypeId
NtnUeLocationReporter::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ntnrrc::NtnUeLocationReporter")
            .SetParent<Object>()
            .SetGroupName("NtnRrc")
            .AddConstructor<NtnUeLocationReporter>()
            .AddAttribute("ReportingPeriod",
                          "Period between reports (or polling cadence in event-triggered mode)",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&NtnUeLocationReporter::m_period),
                          MakeTimeChecker(MilliSeconds(1)))
            .AddAttribute("AccuracyMetres",
                          "1-sigma horizontal accuracy claimed by the UE GNSS",
                          DoubleValue(5.0),
                          MakeDoubleAccessor(&NtnUeLocationReporter::m_accuracyMetres),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("EventTriggerDistanceMetres",
                          "Distance threshold for event-triggered mode",
                          DoubleValue(50.0),
                          MakeDoubleAccessor(&NtnUeLocationReporter::m_triggerDistanceM),
                          MakeDoubleChecker<double>(0.0))
            .AddTraceSource("Report",
                            "Fires on every fresh report",
                            MakeTraceSourceAccessor(&NtnUeLocationReporter::m_reportTrace),
                            "ns3::ntnrrc::NtnUeLocationReporter::ReportTracedCallback");
    return tid;
}

NtnUeLocationReporter::NtnUeLocationReporter()
{
    m_rv = CreateObject<NormalRandomVariable>();
    m_rv->SetAttribute("Mean", DoubleValue(0.0));
    m_rv->SetAttribute("Variance", DoubleValue(1.0));
}

NtnUeLocationReporter::~NtnUeLocationReporter() = default;

void
NtnUeLocationReporter::DoDispose()
{
    Stop();
    m_ue = nullptr;
    m_rv = nullptr;
    Object::DoDispose();
}

void
NtnUeLocationReporter::SetUeMobility(Ptr<MobilityModel> ue)
{
    m_ue = ue;
}

void
NtnUeLocationReporter::SetReportingMode(LocationReportMode mode)
{
    m_mode = mode;
}

void
NtnUeLocationReporter::SetReportingPeriod(Time period)
{
    NS_ASSERT_MSG(period > Time(0), "Reporting period must be positive");
    m_period = period;
}

void
NtnUeLocationReporter::SetAccuracyMetres(double sigma)
{
    NS_ASSERT_MSG(sigma >= 0.0, "Accuracy sigma must be non-negative");
    m_accuracyMetres = sigma;
}

void
NtnUeLocationReporter::SetEventTriggerDistanceMetres(double metres)
{
    NS_ASSERT_MSG(metres >= 0.0, "Trigger distance must be non-negative");
    m_triggerDistanceM = metres;
}

void
NtnUeLocationReporter::SetRandomVariable(Ptr<NormalRandomVariable> rv)
{
    m_rv = rv;
}

bool
NtnUeLocationReporter::HasReport() const
{
    return m_hasReport;
}

const UeLocationReport&
NtnUeLocationReporter::GetLatestReport() const
{
    return m_latest;
}

UeLocationReport
NtnUeLocationReporter::SampleReport()
{
    NS_ASSERT_MSG(m_ue, "UE mobility must be set before sampling");
    const Vector raw = m_ue->GetPosition();
    const Vector vel = m_ue->GetVelocity();

    const double sigmaH = m_accuracyMetres;
    Vector noisy = raw;
    if (sigmaH > 0.0)
    {
        const double nx = m_rv->GetValue() * sigmaH;
        const double ny = m_rv->GetValue() * sigmaH;
        const double nz = m_rv->GetValue() * (sigmaH * 0.5); // vertical typically tighter
        noisy.x += nx;
        noisy.y += ny;
        noisy.z += nz;
    }

    UeLocationReport rep;
    rep.timestamp = Simulator::Now();
    EcefToGeodeticWgs84(noisy, rep.latDeg, rep.lonDeg, rep.altMetres);
    rep.velocityEcefMps = vel;
    rep.accuracyMetres = sigmaH;
    rep.reportSequence = ++m_seq;
    m_lastReportedEcef = raw;
    return rep;
}

UeLocationReport
NtnUeLocationReporter::ReportNow()
{
    m_latest = SampleReport();
    m_hasReport = true;
    m_reportTrace(m_latest);
    return m_latest;
}

void
NtnUeLocationReporter::Start()
{
    if (m_running)
    {
        return;
    }
    m_running = true;
    if (m_mode == LocationReportMode::OnDemand)
    {
        // No periodic ticking; caller drives via ReportNow().
        return;
    }
    Tick();
}

void
NtnUeLocationReporter::Stop()
{
    m_running = false;
    if (m_event.IsPending())
    {
        Simulator::Cancel(m_event);
    }
}

void
NtnUeLocationReporter::Tick()
{
    if (!m_running || !m_ue)
    {
        return;
    }

    bool emit = false;
    if (m_mode == LocationReportMode::Periodic)
    {
        emit = true;
    }
    else if (m_mode == LocationReportMode::EventTriggered)
    {
        if (!m_hasReport)
        {
            emit = true;
        }
        else
        {
            const Vector now = m_ue->GetPosition();
            const double moved = std::sqrt(std::pow(now.x - m_lastReportedEcef.x, 2) +
                                           std::pow(now.y - m_lastReportedEcef.y, 2) +
                                           std::pow(now.z - m_lastReportedEcef.z, 2));
            emit = (moved >= m_triggerDistanceM);
        }
    }

    if (emit)
    {
        m_latest = SampleReport();
        m_hasReport = true;
        m_reportTrace(m_latest);
    }

    m_event = Simulator::Schedule(m_period, &NtnUeLocationReporter::Tick, this);
}

} // namespace ntnrrc
} // namespace ns3
