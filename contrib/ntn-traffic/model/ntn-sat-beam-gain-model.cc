// SPDX-License-Identifier: GPL-2.0-only
//
// See ntn-sat-beam-gain-model.h. Pattern: TR 38.811 §6.4.1 (V15.x); 3 dB
// beamwidth + peak gain cross-checked to TR 38.821 Table 6.1.1.1-1 (Set-1,
// LEO-600, S-band: 30 dBi, 4.4127 deg).

#include "ntn-sat-beam-gain-model.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnSatBeamGainModel");
NS_OBJECT_ENSURE_REGISTERED(NtnSatBeamGainModel);

// Half-power crossing of 4|J1(x)/x|^2 : x where the pattern = 0.5 (-3 dB).
static constexpr double kHalfPowerX = 1.6163;
static constexpr double kEarthRadiusM = 6371000.0;

TypeId
NtnSatBeamGainModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnSatBeamGainModel")
            .SetParent<PropagationLossModel>()
            .SetGroupName("NtnTraffic")
            .AddConstructor<NtnSatBeamGainModel>()
            .AddAttribute("Beamwidth3dBDeg",
                          "Satellite beam 3 dB beamwidth (deg). Default = TR 38.821 "
                          "Set-1 LEO-600 S-band (4.4127 deg).",
                          DoubleValue(4.4127),
                          MakeDoubleAccessor(&NtnSatBeamGainModel::m_beamwidth3dBDeg),
                          MakeDoubleChecker<double>(1e-3))
            .AddAttribute("PeakGainDbi",
                          "Boresight peak gain (dBi). Applied ONLY when ApplyPeakGain "
                          "is true; on the mmwave spine the array already supplies it, "
                          "so the default path adds the roll-off alone.",
                          DoubleValue(30.0),
                          MakeDoubleAccessor(&NtnSatBeamGainModel::m_peakGainDbi),
                          MakeDoubleChecker<double>())
            .AddAttribute("ApplyPeakGain",
                          "Apply PeakGainDbi on top of the off-boresight roll-off, so "
                          "the model yields the ABSOLUTE pattern G(theta). Default "
                          "false — every pre-existing scenario keeps roll-off only.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NtnSatBeamGainModel::m_applyPeakGain),
                          MakeBooleanChecker())
            .AddAttribute("RolloffFloorDb",
                          "Deepest applied off-boresight roll-off (dB).",
                          DoubleValue(-40.0),
                          MakeDoubleAccessor(&NtnSatBeamGainModel::m_rolloffFloorDb),
                          MakeDoubleChecker<double>());
    return tid;
}

NtnSatBeamGainModel::NtnSatBeamGainModel()
{
}

NtnSatBeamGainModel::~NtnSatBeamGainModel()
{
}

void
NtnSatBeamGainModel::SetBeamCenter(Ptr<MobilityModel> c)
{
    m_beamCenter = c;
    // Historical contract: a null beam centre means "track the rx UE".
    m_boresightMode = c ? BoresightMode::FixedPoint : BoresightMode::TrackUe;
}

void
NtnSatBeamGainModel::SetBoresightFixedPoint(Vector point)
{
    m_beamCenter = nullptr;
    m_boresightPoint = point;
    m_boresightMode = BoresightMode::FixedPoint;
}

void
NtnSatBeamGainModel::SetBoresightFixed(Vector direction)
{
    const double n = std::sqrt(direction.x * direction.x + direction.y * direction.y +
                               direction.z * direction.z);
    // Normalise so the caller may pass any magnitude (and so the degenerate-
    // geometry guard in DoCalcRxPower only trips on a genuinely null vector).
    m_boresightDir = (n > 1e-12) ? Vector(direction.x / n, direction.y / n, direction.z / n)
                                 : Vector(0.0, 0.0, 0.0);
    m_boresightMode = BoresightMode::FixedDirection;
}

void
NtnSatBeamGainModel::SetBoresightNadir(Vector geocentre)
{
    m_geocentre = geocentre;
    m_boresightMode = BoresightMode::Nadir;
}

double
NtnSatBeamGainModel::BesselJ1(double x)
{
    // Numerical Recipes bessj1 rational approximation.
    const double ax = std::abs(x);
    if (ax < 8.0)
    {
        const double y = x * x;
        const double p1 =
            x * (72362614232.0 +
                 y * (-7895059235.0 +
                      y * (242396853.1 +
                           y * (-2972611.439 + y * (15704.48260 + y * (-30.16036606))))));
        const double p2 =
            144725228442.0 +
            y * (2300535178.0 +
                 y * (18583304.74 + y * (99447.43394 + y * (376.9991397 + y * 1.0))));
        return p1 / p2;
    }
    const double z = 8.0 / ax;
    const double y = z * z;
    const double xx = ax - 2.356194491;
    const double p1 =
        1.0 + y * (0.183105e-2 +
                   y * (-0.3516396496e-4 +
                        y * (0.2457520174e-5 + y * (-0.240337019e-6))));
    const double p2 =
        0.04687499995 +
        y * (-0.2002690873e-3 +
             y * (0.8449199096e-5 + y * (-0.88228987e-6 + y * 0.105787412e-6)));
    const double ans =
        std::sqrt(0.636619772 / ax) * (std::cos(xx) * p1 - z * std::sin(xx) * p2);
    return (x < 0.0) ? -ans : ans;
}

double
NtnSatBeamGainModel::RolloffDbAtThetaRad(double thetaRad) const
{
    // TR 38.811 §6.4.1: G(theta)/Gmax = 4 |J1(u)/u|^2 with u = k a sin(theta).
    // k a is pinned by the configured 3 dB beamwidth: the half-power crossing of
    // 4|J1(u)/u|^2 is u = 1.6163 (kHalfPowerX), which must occur at the HALF
    // beamwidth theta_3dB/2, hence u = 1.6163 * sin(theta) / sin(theta_3dB/2).
    const double theta3dBHalf = 0.5 * m_beamwidth3dBDeg * M_PI / 180.0;
    const double sinHalf = std::sin(theta3dBHalf);
    const double t = std::abs(thetaRad);
    if (t <= 1e-9 || sinHalf <= 1e-12)
    {
        return 0.0; // boresight: exactly 0 dB roll-off
    }
    const double u = kHalfPowerX * std::sin(t) / sinHalf;
    if (u <= 1e-6)
    {
        return 0.0;
    }
    const double j1 = BesselJ1(u);
    const double g = 4.0 * (j1 / u) * (j1 / u); // G/Gmax in [0,1]
    const double rolloffDb = 10.0 * std::log10(std::max(g, 1e-9));
    // Clamp: never a gain (<= 0 dB) and never deeper than the configured floor
    // (the Airy nulls are singular; a real feed/aperture has a finite floor).
    return std::max(m_rolloffFloorDb, std::min(0.0, rolloffDb));
}

double
NtnSatBeamGainModel::GainDbAtThetaDeg(double thetaDeg) const
{
    // Pure closed form — touches no simulation state, so a reviewer can diff the
    // measured plane against it sample by sample.
    return RolloffDbAtThetaRad(thetaDeg * M_PI / 180.0) +
           (m_applyPeakGain ? m_peakGainDbi : 0.0);
}

double
NtnSatBeamGainModel::DoCalcRxPower(double txPowerDbm,
                                   Ptr<MobilityModel> a,
                                   Ptr<MobilityModel> b) const
{
    // Satellite = the higher node (larger geocentric radius / altitude).
    Vector pa = a->GetPosition();
    Vector pb = b->GetPosition();
    const double ra = std::sqrt(pa.x * pa.x + pa.y * pa.y + pa.z * pa.z);
    const double rb = std::sqrt(pb.x * pb.x + pb.y * pb.y + pb.z * pb.z);
    const Vector sat = (ra >= rb) ? pa : pb;
    const Vector ue = (ra >= rb) ? pb : pa;

    // Boresight direction from the satellite. See BoresightMode in the header:
    // only the steered/tracking default forces theta = 0; every fixed-boresight
    // mode lets the UE traverse the lobe, which is where the pattern's angle
    // dependence actually shows up in the measured plane.
    const Vector dUe(ue.x - sat.x, ue.y - sat.y, ue.z - sat.z);
    Vector dBore = dUe;
    switch (m_boresightMode)
    {
    case BoresightMode::FixedPoint: {
        const Vector bore = m_beamCenter ? m_beamCenter->GetPosition() : m_boresightPoint;
        dBore = Vector(bore.x - sat.x, bore.y - sat.y, bore.z - sat.z);
        break;
    }
    case BoresightMode::FixedDirection:
        dBore = m_boresightDir;
        break;
    case BoresightMode::Nadir:
        dBore = Vector(m_geocentre.x - sat.x, m_geocentre.y - sat.y, m_geocentre.z - sat.z);
        break;
    case BoresightMode::TrackUe:
    default:
        break; // dBore == dUe -> theta = 0
    }
    const double nB = std::sqrt(dBore.x * dBore.x + dBore.y * dBore.y + dBore.z * dBore.z);
    const double nU = std::sqrt(dUe.x * dUe.x + dUe.y * dUe.y + dUe.z * dUe.z);
    // Degenerate geometry (co-located nodes / null boresight): behave as
    // boresight. dBore is a metric displacement in every mode except
    // FixedDirection, where SetBoresightFixed() normalises it to unit length,
    // so only a genuinely null vector trips this.
    if (nB < 1e-9 || nU < 1.0)
    {
        m_lastThetaDeg = 0.0;
        m_lastRolloffDb = 0.0;
        return txPowerDbm + (m_applyPeakGain ? m_peakGainDbi : 0.0);
    }
    double cosT = (dBore.x * dUe.x + dBore.y * dUe.y + dBore.z * dUe.z) / (nB * nU);
    cosT = std::max(-1.0, std::min(1.0, cosT));
    const double theta = std::acos(cosT); // off-boresight angle (rad)
    m_lastThetaDeg = theta * 180.0 / M_PI;

    const double rolloffDb = RolloffDbAtThetaRad(theta);
    m_lastRolloffDb = rolloffDb;
    // The absolute peak is added only on request (default off: the radio's array
    // already supplies it — see the header's no-double-count note).
    const double peakDb = m_applyPeakGain ? m_peakGainDbi : 0.0;
    NS_LOG_DEBUG("theta=" << m_lastThetaDeg << "deg rolloff=" << rolloffDb << "dB peak="
                          << peakDb << "dB");
    // Roll-off is a (<=0) gain term: add it to rx power (= a loss off-boresight).
    return txPowerDbm + rolloffDb + peakDb;
}

int64_t
NtnSatBeamGainModel::DoAssignStreams(int64_t /*stream*/)
{
    return 0; // deterministic pattern, no RNG
}

} // namespace ns3
