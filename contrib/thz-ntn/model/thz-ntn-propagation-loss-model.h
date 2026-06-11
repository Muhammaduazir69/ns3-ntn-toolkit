// SPDX-License-Identifier: GPL-2.0-only
//
// ThzNtnPropagationLossModel — re-homes the thz-ntn molecular-absorption and
// weather-attenuation CALCULATORS as a real ns-3 PropagationLossModel (Phase 1
// of 2026-06 protocol-fidelity audit, channel-plugin pattern).
//
// Before, these physics were only invoked as Compute*() in user-space loops and
// written to CSV; packets never saw them. As a PropagationLossModel this can be
// chained onto a real spectrum channel (e.g. via NtnRealStackHelper::
// AddExtraPropagationLoss), so THz atmospheric loss actually attenuates packets
// and shows up in the MEASURED SINR.

#ifndef THZ_NTN_PROPAGATION_LOSS_MODEL_H
#define THZ_NTN_PROPAGATION_LOSS_MODEL_H

#include "thz-ntn-molecular-absorption.h"
#include "thz-ntn-weather-attenuation.h"

#include "ns3/propagation-loss-model.h"

namespace ns3
{

/**
 * \brief THz atmospheric loss (gaseous molecular absorption + rain) as a real
 *        PropagationLossModel, computed per transmission from the Tx/Rx geometry.
 */
class ThzNtnPropagationLossModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId();
    ThzNtnPropagationLossModel();
    ~ThzNtnPropagationLossModel() override;

    /// Set the operating frequency (Hz) used for the absorption/rain models.
    void SetFrequency(double freqHz) { m_freqHz = freqHz; }
    /// Set the rain rate (mm/h) for weather attenuation (0 = clear sky).
    void SetRainRate(double mmPerHour) { m_rainRateMmH = mmPerHour; }
    /// Set the fog liquid water content (g/m^3); 0 = no fog.
    void SetFogLwc(double gPerM3) { m_fogLwcGm3 = gPerM3; }
    /// Set the snow rate (mm/h water-equivalent); 0 = no snow.
    void SetSnowRate(double mmPerHour, bool wet = true)
    {
        m_snowRateMmH = mmPerHour;
        m_snowWet = wet;
    }
    /// Last total atmospheric loss applied (dB) — for logging/inspection.
    double GetLastLossDb() const { return m_lastLossDb; }

  private:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;
    int64_t DoAssignStreams(int64_t stream) override;

    Ptr<ThzNtnMolecularAbsorption> m_absorption;
    Ptr<ThzNtnWeatherAttenuation> m_weather;
    double m_freqHz{100.0e9};
    double m_rainRateMmH{0.0};
    double m_fogLwcGm3{0.0};
    double m_snowRateMmH{0.0};
    bool m_snowWet{true};
    mutable double m_lastLossDb{0.0};
};

} // namespace ns3

#endif // THZ_NTN_PROPAGATION_LOSS_MODEL_H
