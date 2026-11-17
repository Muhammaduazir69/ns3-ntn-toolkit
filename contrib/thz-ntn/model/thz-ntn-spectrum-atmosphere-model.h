/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef THZ_NTN_SPECTRUM_ATMOSPHERE_MODEL_H
#define THZ_NTN_SPECTRUM_ATMOSPHERE_MODEL_H

#include "thz-ntn-itu-recommendations.h"
#include "thz-ntn-link-geometry.h"
#include "thz-ntn-molecular-absorption.h"
#include "thz-ntn-rain-cell-field.h"
#include "thz-ntn-scintillation.h"
#include "thz-ntn-weather-attenuation.h"

#include "ns3/spectrum-propagation-loss-model.h"

#include <map>
#include <tuple>
#include <vector>

namespace ns3
{

/**
 * \ingroup thz-ntn
 * \brief Per-resource-block atmospheric attenuation for the measured NR
 *        plane (NtnRealStackHelper::AddSpectrumChannelLoss).
 *
 * Each band of the received PSD (one band per RB in 5G-LENA) is attenuated at
 * its OWN centre frequency, instead of one scalar dB per link:
 *  - gaseous absorption: ITU-R P.676-13 Annex 1 line-by-line O2 + H2O over
 *    the P.835-6 reference atmosphere (ThzNtnMolecularAbsorption), integrated
 *    along the slant path from the lower end to min(upper end, 100 km);
 *  - rain, one of two modes:
 *      uniform rate R: ITU-R P.618-13 2.2.1.1 slant-path rain attenuation
 *      with the P.838-3 k(f), alpha(f) of each RB (Itu618LossModel; R is
 *      treated as the R_0.01 point rate, i.e. the 0.01 % exceedance value);
 *      rain field: the P.838-3 specific attenuation of each RB integrated
 *      through a ThzNtnRainCellField along the live slant path (weather
 *      caused by geometry and time, see ThzNtnRainCellField);
 *  - cloud/fog: liquid-water specific attenuation K_l(f) (ITU-R P.840
 *    double-Debye form as implemented in ThzNtnWeatherAttenuation) times the
 *    liquid water content over the layer path;
 *  - optional tropospheric amplitude scintillation (ITU-R P.618-13 2.4.1
 *    sigma with the aperture-averaging factor of the declared receive dish,
 *    ThzNtnScintillation): a time-correlated AR(1) sample in simulated time,
 *    applied flat over the band (sigma varies as f^(7/12), < 0.1 dB across
 *    a 400 MHz channel at 100 GHz). One ThzNtnScintillation state per model,
 *    so attach one model per Earth-space link when scintillation is on.
 *
 * Both ends above 100 km (ISL) see no atmosphere and the PSD passes through.
 *
 * Cost control: the gaseous vector is cached per (elevation quantum, lower
 * altitude, upper altitude capped at 100 km, band set); the rain field is
 * re-sampled at most every \p RainRefresh per link. Elevation below the
 * P.676 integrator floor (5 deg) is evaluated AT the floor and counted
 * (GetClampedEvaluations()).
 */
class ThzNtnSpectrumAtmosphereModel : public SpectrumPropagationLossModel
{
  public:
    static TypeId GetTypeId();
    ThzNtnSpectrumAtmosphereModel();
    ~ThzNtnSpectrumAtmosphereModel() override;

    void SetGasEnabled(bool e) { m_gas = e; }
    /// P.835-6 reference atmosphere name (see ThzNtnMolecularAbsorption).
    void SetAtmosphereProfile(const std::string& profile);
    /// Uniform point rain rate R_0.01 (mm/h), P.618-13 mode; 0 = no rain.
    void SetUniformRainRate(double mmPerHour) { m_uniformRain = mmPerHour; }
    double GetUniformRainRate() const { return m_uniformRain; }
    /// Rain-cell field mode (overrides the uniform rate when set).
    void SetRainField(Ptr<ThzNtnRainCellField> field) { m_field = field; }
    Ptr<ThzNtnRainCellField> GetRainField() const { return m_field; }
    /// P.618 settings for the uniform mode.
    void SetClimateRegion(itu::Itu618LossModel::ClimateRegion r);
    void SetStationLatitudeDeg(double lat);
    void SetPolarization(itu::Polarization p) { m_pol = p; }
    /// Cloud / fog liquid water content (g/m^3); 0 = none.
    void SetCloudLwc(double gPerM3) { m_lwc = gPerM3; }
    /// P.618 amplitude scintillation (null = off, the default).
    void SetScintillation(Ptr<ThzNtnScintillation> s) { m_scint = s; }
    double GetLastScintillationDb() const { return m_lastScint; }

    /// Per-band loss (dB) of the last evaluated link, band order of the PSD.
    std::vector<double> GetLastLossDb() const { return m_lastLoss; }
    double GetLastMeanLossDb() const;
    double GetLastMinLossDb() const;
    double GetLastMaxLossDb() const;
    double GetLastGasCentreDb() const { return m_lastGasCentre; }
    double GetLastRainCentreDb() const { return m_lastRainCentre; }
    /// Wave 6B: true if the last link carried a non-zero rain term on any RB
    /// above 55 GHz, the validated limit of ITU-R P.618-14 2.2.1.1 (P.838-3
    /// k, alpha are fitted to 1000 GHz; the path method is extrapolated).
    bool WasLastRainExtrapolated() const { return m_lastRainExtrap; }
    /// Link evaluations that applied rain above 55 GHz.
    uint64_t GetExtrapolatedRainCount() const { return m_rainExtrapCount; }
    double GetLastElevationDeg() const { return m_lastElev; }
    /// Rain rate on the path below h_R (max over samples) of the last link.
    double GetLastPathRainRate() const { return m_lastPathRate; }
    uint64_t GetEvaluations() const { return m_evals; }
    uint64_t GetClampedEvaluations() const { return m_clamped; }

    /// Loss vector (dB) for a link geometry and a list of frequencies (Hz).
    /// Used by DoCalcRxPowerSpectralDensity and by tests.
    std::vector<double> ComputeLossDb(const ThzNtnLinkGeometry& g,
                                      const std::vector<double>& freqsHz,
                                      Time t) const;

  private:
    Ptr<SpectrumValue> DoCalcRxPowerSpectralDensity(Ptr<const SpectrumSignalParameters> params,
                                                    Ptr<const MobilityModel> a,
                                                    Ptr<const MobilityModel> b) const override;
    int64_t DoAssignStreams(int64_t stream) override;

    std::vector<double> GasVector(const ThzNtnLinkGeometry& g,
                                  const std::vector<double>& freqsHz) const;

    Ptr<ThzNtnMolecularAbsorption> m_abs;
    Ptr<ThzNtnWeatherAttenuation> m_weather;
    Ptr<itu::Itu618LossModel> m_p618;
    Ptr<ThzNtnRainCellField> m_field;
    Ptr<ThzNtnScintillation> m_scint;
    mutable double m_lastScint{0.0};
    mutable Time m_lastScintTime{Seconds(-1)};
    bool m_gas{true};
    double m_uniformRain{0.0};
    double m_lwc{0.0};
    double m_elevQuantumDeg{0.02};
    Time m_rainRefresh{MilliSeconds(10)};
    itu::Polarization m_pol{itu::Polarization::circular};

    struct GasKey
    {
        int64_t elevQ;
        int64_t lowM;
        int64_t highKm;
        double f0;
        size_t n;
        bool operator<(const GasKey& o) const
        {
            return std::tie(elevQ, lowM, highKm, f0, n) <
                   std::tie(o.elevQ, o.lowM, o.highKm, o.f0, o.n);
        }
    };
    mutable std::map<GasKey, std::vector<double>> m_gasCache;

    struct RainEntry
    {
        Time t{Seconds(-1)};
        std::vector<double> loss;
        double pathRate{0.0};
    };
    mutable std::map<std::pair<const void*, const void*>, RainEntry> m_rainCache;

    mutable std::vector<double> m_lastLoss;
    mutable double m_lastGasCentre{0.0};
    mutable double m_lastRainCentre{0.0};
    mutable bool m_lastRainExtrap{false};
    mutable uint64_t m_rainExtrapCount{0};
    mutable double m_lastElev{0.0};
    mutable double m_lastPathRate{0.0};
    mutable uint64_t m_evals{0};
    mutable uint64_t m_clamped{0};
};

} // namespace ns3

#endif // THZ_NTN_SPECTRUM_ATMOSPHERE_MODEL_H
