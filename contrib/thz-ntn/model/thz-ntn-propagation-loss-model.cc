// SPDX-License-Identifier: GPL-2.0-only
#include "thz-ntn-propagation-loss-model.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"

#include <algorithm>
#include <cmath>

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
                          MakeDoubleChecker<double>());
    return tid;
}

ThzNtnPropagationLossModel::ThzNtnPropagationLossModel()
{
    m_absorption = CreateObject<ThzNtnMolecularAbsorption>();
    m_weather = CreateObject<ThzNtnWeatherAttenuation>();
}

ThzNtnPropagationLossModel::~ThzNtnPropagationLossModel() = default;

double
ThzNtnPropagationLossModel::DoCalcRxPower(double txPowerDbm,
                                          Ptr<MobilityModel> a,
                                          Ptr<MobilityModel> b) const
{
    Vector pa = a->GetPosition();
    Vector pb = b->GetPosition();
    // Ground endpoint = lower altitude; space endpoint = higher.
    const Vector& low = (pa.z <= pb.z) ? pa : pb;
    const Vector& high = (pa.z <= pb.z) ? pb : pa;
    double dx = high.x - low.x;
    double dy = high.y - low.y;
    double dz = high.z - low.z;
    double slant = std::sqrt(dx * dx + dy * dy + dz * dz);
    double elevDeg = std::asin(std::max(0.0, dz) / std::max(1.0, slant)) * 180.0 / M_PI;
    double groundAltKm = low.z / 1000.0;
    double satAltKm = high.z / 1000.0;

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
    m_lastLossDb = gaseousDb + rainDb + fogDb + snowDb;
    return txPowerDbm - m_lastLossDb;
}

int64_t
ThzNtnPropagationLossModel::DoAssignStreams(int64_t /*stream*/)
{
    return 0;
}

} // namespace ns3
