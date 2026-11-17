// SPDX-License-Identifier: GPL-2.0-only
#include "thz-ntn-propagation-loss-model.h"

#include "thz-ntn-link-geometry.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/pointer.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnPropagationLossModel");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnPropagationLossModel);

TypeId
ThzNtnPropagationLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnPropagationLossModel")
            .SetParent<PropagationLossModel>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnPropagationLossModel>()
            .AddAttribute("Frequency",
                          "Operating frequency (Hz) for absorption/rain models.",
                          DoubleValue(100.0e9),
                          MakeDoubleAccessor(&ThzNtnPropagationLossModel::m_freqHz),
                          MakeDoubleChecker<double>())
            .AddAttribute("RainRate",
                          "Rain rate (mm/h); 0 = clear sky.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&ThzNtnPropagationLossModel::m_rainRateMmH),
                          MakeDoubleChecker<double>())
            .AddAttribute("ApparentElevation",
                          "Convert the free-space elevation of the geometry to the "
                          "apparent elevation (ITU-R P.834-9 eq. 14) before the slant-path "
                          "gas, rain and cloud terms. False reproduces runs before Wave 7C.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&ThzNtnPropagationLossModel::m_apparentElevation),
                          MakeBooleanChecker())
            .AddAttribute("CloudModel",
                          "ITU-R P.840-9 cloud attenuation on the path (set its Mode to Off "
                          "to reproduce runs before Wave 7C).",
                          PointerValue(),
                          MakePointerAccessor(&ThzNtnPropagationLossModel::m_cloud),
                          MakePointerChecker<ThzNtnCloudAttenuation>());
    return tid;
}

ThzNtnPropagationLossModel::ThzNtnPropagationLossModel()
{
    m_absorption = CreateObject<ThzNtnMolecularAbsorption>();
    m_weather = CreateObject<ThzNtnWeatherAttenuation>();
    m_cloud = CreateObject<ThzNtnCloudAttenuation>();
}

ThzNtnPropagationLossModel::~ThzNtnPropagationLossModel() = default;

double
ThzNtnPropagationLossModel::DoCalcRxPower(double txPowerDbm,
                                          Ptr<MobilityModel> a,
                                          Ptr<MobilityModel> b) const
{
    // Wave 7C: the shared Earth-space geometry (ECEF ends such as SGP4 use
    // WGS-84 altitudes and the ellipsoid normal; flat ENU ends use z). This
    // used to read z as the altitude in every frame, which is wrong for ECEF.
    const ThzNtnLinkGeometry g = ThzNtnLinkGeometry::Compute(PeekPointer(a), PeekPointer(b));
    const double groundAltKm = g.lowerAltKm;
    const double satAltKm = g.upperAltKm;
    m_lastTrueElev = g.elevationDeg;
    double elevDeg = std::max(0.0, g.elevationDeg);
    if (m_apparentElevation && groundAltKm < 100.0)
    {
        if (groundAltKm > itu::P834::kMaxStationHeightKm ||
            !itu::P834::IsVisible(g.elevationDeg, groundAltKm))
        {
            ++m_p834OutOfRange;
        }
        elevDeg = std::max(0.0, itu::P834::ApparentElevationDeg(g.elevationDeg, groundAltKm));
    }
    m_lastAppElev = elevDeg;
    double gaseousDb =
        m_absorption->ComputeSlantPathAbsorption(m_freqHz, elevDeg, groundAltKm, satAltKm);
    double rainDb = (m_rainRateMmH > 0.0)
                        ? m_weather->ComputeRainAttenuation_dB(m_freqHz, elevDeg, m_rainRateMmH)
                        : 0.0;
    double fogDb = (m_fogLwcGm3 > 0.0)
                       ? m_weather->ComputeFogAttenuation_dB(m_freqHz, elevDeg, m_fogLwcGm3)
                       : 0.0;
    double snowDb =
        (m_snowRateMmH > 0.0)
            ? m_weather->ComputeSnowAttenuation_dB(m_freqHz, elevDeg, m_snowRateMmH, m_snowWet)
            : 0.0;
    // Cloud only on Earth-space paths with the ground end in the troposphere.
    double cloudDb = 0.0;
    if (m_cloud && groundAltKm < 10.0 && satAltKm > groundAltKm)
    {
        const double lat = g.ecef ? g.lowerLatDeg : std::numeric_limits<double>::quiet_NaN();
        const double lon = g.ecef ? g.lowerLonDeg : std::numeric_limits<double>::quiet_NaN();
        cloudDb = m_cloud->ComputeAttenuationDb(m_freqHz, elevDeg, lat, lon);
    }
    m_lastCloudDb = cloudDb;
    m_lastLossDb = gaseousDb + rainDb + fogDb + snowDb + cloudDb;
    m_lastGasDb = gaseousDb;
    m_lastRainDb = rainDb;
    m_lastRainExtrap = (rainDb > 0.0) && m_weather->WasLastRainExtrapolated();
    return txPowerDbm - m_lastLossDb;
}

int64_t
ThzNtnPropagationLossModel::DoAssignStreams(int64_t stream)
{
    return m_cloud ? m_cloud->AssignStreams(stream) : 0;
}

} // namespace ns3
