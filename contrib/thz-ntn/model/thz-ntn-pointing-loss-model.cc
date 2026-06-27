// SPDX-License-Identifier: GPL-2.0-only
//
// See thz-ntn-pointing-loss-model.h: this re-homes the THz pointing impairment
// (gap A2) onto the measured plane. Beam squint over the wide THz band is still
// NOT modelled (single-frequency array factor) — that remains open.

#include "thz-ntn-pointing-loss-model.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnPointingLossModel");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnPointingLossModel);

// Mean Earth radius (m) — used to discriminate ECEF vs local-ENU coordinates.
static constexpr double EARTH_RADIUS_M = 6371000.0;
// Earth gravitational parameter (m^3/s^2), for the circular-orbit velocity.
static constexpr double MU_EARTH = 3.986004e14;

TypeId
ThzNtnPointingLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnPointingLossModel")
            .SetParent<PropagationLossModel>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnPointingLossModel>()
            .AddAttribute("Beamwidth3dBDeg",
                          "Half-power (3 dB) beamwidth (deg) used to map the "
                          "off-boresight pointing error onto a gain loss.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&ThzNtnPointingLossModel::m_beamwidth3dBDeg),
                          MakeDoubleChecker<double>(1.0e-3, 180.0));
    return tid;
}

ThzNtnPointingLossModel::ThzNtnPointingLossModel()
{
    m_pointingError = CreateObject<ThzNtnPointingError>();
}

ThzNtnPointingLossModel::~ThzNtnPointingLossModel() = default;

void
ThzNtnPointingLossModel::SetVibrationRmsDeg(double rmsDeg)
{
    m_pointingError->SetVibrationRms_deg(rmsDeg);
}

void
ThzNtnPointingLossModel::SetTrackingUpdateRateHz(double rateHz)
{
    m_pointingError->SetTrackingUpdateRate_Hz(rateHz);
}

void
ThzNtnPointingLossModel::SetTrackingLatencyMs(double latencyMs)
{
    // No dedicated setter on ThzNtnPointingError; drive its attribute instead.
    m_pointingError->SetAttribute("TrackingLatency_ms", DoubleValue(latencyMs));
}

void
ThzNtnPointingLossModel::SetPointingError(Ptr<ThzNtnPointingError> pe)
{
    NS_ASSERT_MSG(pe, "Null ThzNtnPointingError injected");
    m_pointingError = pe;
}

double
ThzNtnPointingLossModel::ElevationDeg(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
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

double
ThzNtnPointingLossModel::DoCalcRxPower(double txPowerDbm,
                                       Ptr<MobilityModel> a,
                                       Ptr<MobilityModel> b) const
{
    const double elevDeg = ElevationDeg(a, b);

    // Satellite altitude (km) from the higher node. ECEF: geocentric radius -
    // Earth radius; local-ENU (small magnitude): height above ground = z.
    Vector pa = a->GetPosition();
    Vector pb = b->GetPosition();
    double ra = std::sqrt(pa.x * pa.x + pa.y * pa.y + pa.z * pa.z);
    double rb = std::sqrt(pb.x * pb.x + pb.y * pb.y + pb.z * pb.z);
    const Vector& sat = (ra <= rb) ? pb : pa;
    double rSat = std::max(ra, rb);
    double rGround = std::min(ra, rb);

    double satAltKm;
    if (rGround > 0.5 * EARTH_RADIUS_M)
    {
        satAltKm = (rSat - EARTH_RADIUS_M) / 1000.0;
    }
    else
    {
        satAltKm = sat.z / 1000.0;
    }
    // Keep the geometry inside a sane LEO/MEO band for the orbital-velocity map.
    satAltKm = std::max(200.0, std::min(satAltKm, 36000.0));

    // ThzNtnPointingError parameterises the J2/tracking terms by the satellite
    // ground-track velocity; derive it from the altitude assuming a circular
    // orbit: v = sqrt(mu / r).
    double rOrbit_m = EARTH_RADIUS_M + satAltKm * 1000.0;
    double vSat_km_s = std::sqrt(MU_EARTH / rOrbit_m) / 1000.0;

    // Deterministic RMS misalignment for this geometry...
    double errRmsDeg = m_pointingError->ComputePointingError_deg(elevDeg, vSat_km_s);
    // ...plus a fresh per-call Rayleigh jitter sample, so pointing jitter shows
    // up as honest SINR variance instead of a static offset.
    double errJitterDeg = m_pointingError->GetPointingErrorSample_deg();
    double errTotalDeg = std::sqrt(errRmsDeg * errRmsDeg + errJitterDeg * errJitterDeg);

    m_lastLossDb = m_pointingError->ComputePointingLoss_dB(errTotalDeg, m_beamwidth3dBDeg);

    NS_LOG_DEBUG("elev=" << elevDeg << "deg satAlt=" << satAltKm << "km vSat="
                         << vSat_km_s << "km/s errRms=" << errRmsDeg << " errJit="
                         << errJitterDeg << " -> pointingLoss=" << m_lastLossDb << "dB");
    return txPowerDbm - m_lastLossDb;
}

int64_t
ThzNtnPointingLossModel::DoAssignStreams(int64_t stream)
{
    return m_pointingError->AssignStreams(stream);
}

} // namespace ns3
