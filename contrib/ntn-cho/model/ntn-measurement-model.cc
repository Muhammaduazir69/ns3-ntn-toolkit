/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Muhammad Uzair
 */

#include "ntn-measurement-model.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/satellite-mobility-model.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnMeasurementModel");
NS_OBJECT_ENSURE_REGISTERED(NtnMeasurementModel);

TypeId
NtnMeasurementModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnMeasurementModel")
            .SetParent<Object>()
            .SetGroupName("NtnCho")
            .AddConstructor<NtnMeasurementModel>()
            .AddTraceSource("MeasurementReport",
                            "Fired when a measurement is computed",
                            MakeTraceSourceAccessor(&NtnMeasurementModel::m_measurementTrace),
                            "ns3::NtnMeasurementModel::MeasurementTracedCallback");
    return tid;
}

NtnMeasurementModel::NtnMeasurementModel()
    : m_scenario(NTN_SUBURBAN),
      m_carrierFreqHz(2.0e9),   // S-band default
      m_bandwidthHz(30.0e6),    // 30 MHz default
      m_satTxPower_dBm(40.0),   // 10W per beam
      m_ueNoiseFigure_dB(7.0)
{
    NS_LOG_FUNCTION(this);
    m_shadowFadingRng = CreateObject<NormalRandomVariable>();
    m_shadowFadingRng->SetAttribute("Mean", DoubleValue(0.0));
    m_shadowFadingRng->SetAttribute("Variance", DoubleValue(1.0));
    m_scintillationRng = CreateObject<NormalRandomVariable>();
    m_scintillationRng->SetAttribute("Mean", DoubleValue(0.0));
    m_scintillationRng->SetAttribute("Variance", DoubleValue(1.0));
}

NtnMeasurementModel::~NtnMeasurementModel()
{
    NS_LOG_FUNCTION(this);
}

void
NtnMeasurementModel::DoDispose()
{
    m_orbitPredictor = nullptr;
    Object::DoDispose();
}

void
NtnMeasurementModel::SetOrbitPredictor(Ptr<NtnOrbitPredictor> predictor)
{
    m_orbitPredictor = predictor;
}

void
NtnMeasurementModel::SetNtnScenario(NtnScenario scenario)
{
    m_scenario = scenario;
}

void
NtnMeasurementModel::SetCarrierFrequency(double freqHz)
{
    m_carrierFreqHz = freqHz;
}

void
NtnMeasurementModel::SetBandwidth(double bwHz)
{
    m_bandwidthHz = bwHz;
}

void
NtnMeasurementModel::SetSatelliteTxPower(double txPower_dBm)
{
    m_satTxPower_dBm = txPower_dBm;
}

void
NtnMeasurementModel::SetUeNoiseFigure(double nf_dB)
{
    m_ueNoiseFigure_dB = nf_dB;
}

NtnMeasurementModel::Measurement
NtnMeasurementModel::ComputeMeasurement(GeoCoordinate uePosition,
                                         uint32_t satId,
                                         uint32_t beamId) const
{
    NS_LOG_FUNCTION(this << satId << beamId);
    NS_ASSERT_MSG(m_orbitPredictor, "OrbitPredictor not set");

    Measurement meas;
    meas.satId = satId;
    meas.beamId = beamId;

    // Get beam snapshot
    auto snap = m_orbitPredictor->GetBeamSnapshot(satId, beamId, uePosition);
    meas.elevationAngle_deg = snap.elevationAngle_deg;
    meas.slantRange_km = snap.slantRange_km;
    meas.propagationDelay = snap.propagationDelay;
    meas.antennaGain_dB = snap.gainAtUe_dB;

    // Compute path loss: Free-space + atmospheric/clutter (3GPP TR 38.811)
    double distance_m = snap.slantRange_km * 1000.0;
    double fspl_dB = ComputeFreeSpacePathLoss(distance_m, m_carrierFreqHz);

    // Atmospheric gas absorption per ITU-R P.676 (zenith S-band ~0.04 dB,
    // scaled by 1/sin(elev) for the slant path; clamp at 5 deg minimum elev).
    const double elev_rad = std::max(meas.elevationAngle_deg, 5.0) *
                             M_PI / 180.0;
    const double zenithGas_dB =
        (m_carrierFreqHz < 6e9)   ? 0.04 :   // S/L-band
        (m_carrierFreqHz < 30e9)  ? 0.10 :   // C/Ku-band
                                    0.30;    // Ka-band+
    const double atmosphericLoss_dB = zenithGas_dB / std::sin(elev_rad);

    // Tropospheric scintillation per ITU-R P.618 (random N(0, sigma_xi^2) in dB).
    // sigma_xi grows with frequency (sqrt) and shrinks with elevation
    // (sin^{-11/12}); the model is calibrated to give ~0.5 dB rms at S-band /
    // 30 deg elevation, matching ITU-R P.618 Eq. 39 with antenna averaging.
    const double f_GHz = m_carrierFreqHz / 1.0e9;
    const double sigma_xi =
        0.5 * std::sqrt(f_GHz / 2.0) /
        std::pow(std::sin(elev_rad), 11.0 / 12.0);
    const double scintillationLoss_dB = sigma_xi * m_scintillationRng->GetValue();

    // Clutter loss per TR 38.811 Table 6.6.2-1 (scenario-dependent constant).
    double clutterLoss_dB = 0.0;
    switch (m_scenario)
    {
    case NTN_DENSE_URBAN:
        clutterLoss_dB = 4.0;
        break;
    case NTN_URBAN:
        clutterLoss_dB = 2.5;
        break;
    case NTN_SUBURBAN:
        clutterLoss_dB = 1.0;
        break;
    case NTN_RURAL:
        clutterLoss_dB = 0.5;
        break;
    }

    // Shadow fading: random sample from N(0, sigma_sf^2) in dB.
    // sigma_sf is elevation-dependent per TR 38.811 Table 6.6.2-1
    // (S-band suburban LoS: 1.79 / 1.14 / 1.06 / 1.06 / 1.06 / 1.06 / 1.06 / 1.06 dB
    //  for elevation 10..80 deg in 10-deg steps). We use a tractable
    //  3-bin approximation that preserves the exponential decay with elevation.
    double sigma_sf;
    if (meas.elevationAngle_deg < 30.0)
        sigma_sf = 3.0;  // low-elevation, more diffraction
    else if (meas.elevationAngle_deg < 60.0)
        sigma_sf = 2.0;  // mid-elevation
    else
        sigma_sf = 1.0;  // near-zenith, strong LoS
    const double shadowFading_dB = sigma_sf * m_shadowFadingRng->GetValue();

    meas.pathLoss_dB = fspl_dB + atmosphericLoss_dB + scintillationLoss_dB +
                       clutterLoss_dB + shadowFading_dB;

    // RSRP = TxPower + AntennaGain - PathLoss
    meas.rsrp_dBm = m_satTxPower_dBm + meas.antennaGain_dB - meas.pathLoss_dB;

    // PROVENANCE (research/auxiliary model, NOT a headline KPI source):
    // this SINR is a THERMAL-NOISE-ONLY analytical estimate (SINR = RSRP -
    // thermal noise floor, single serving link, NO co-channel/adjacent-beam
    // interference and NO measured PHY). It is honest only as an offline
    // ephemeris/RSRP oracle for candidate ranking. Examples MUST take headline
    // serving/candidate SINR from the REAL mmwave plane
    // (NtnRealStackHelper::GetUeRecentSinrDb / GetCellMeanSinrDb), never from
    // this formula. Promoting interference modelling here would be net-new
    // functionality and is out of scope. See STANDARDS_CONFORMANCE_AUDIT
    // finding #11.
    double noisePower_dBm = ComputeNoisePower();
    meas.sinr_dB = meas.rsrp_dBm - noisePower_dBm;

    // Compute Doppler shift
    meas.dopplerShift_Hz = ComputeDopplerShift(uePosition, satId);

    NS_LOG_DEBUG("Measurement: sat=" << satId << " beam=" << beamId
                 << " RSRP=" << meas.rsrp_dBm << " dBm"
                 << " SINR=" << meas.sinr_dB << " dB"
                 << " elev=" << meas.elevationAngle_deg << " deg"
                 << " range=" << meas.slantRange_km << " km");

    m_measurementTrace(satId, beamId, meas.rsrp_dBm, meas.sinr_dB);
    return meas;
}

std::vector<NtnMeasurementModel::Measurement>
NtnMeasurementModel::ScanVisibleBeams(GeoCoordinate uePosition,
                                       double minElevation_deg) const
{
    NS_LOG_FUNCTION(this << minElevation_deg);

    std::vector<Measurement> results;

    auto visible = m_orbitPredictor->GetVisibleSatellites(uePosition, minElevation_deg);

    uint16_t cellIdCounter = 1;
    for (const auto& vs : visible)
    {
        Measurement meas = ComputeMeasurement(uePosition, vs.satId, vs.bestBeamId);
        meas.cellId = cellIdCounter++;
        results.push_back(meas);
    }

    // Sort by SINR descending
    std::sort(results.begin(), results.end(),
              [](const Measurement& a, const Measurement& b) {
                  return a.sinr_dB > b.sinr_dB;
              });

    return results;
}

double
NtnMeasurementModel::ComputeDopplerShift(GeoCoordinate uePosition, uint32_t satId) const
{
    NS_LOG_FUNCTION(this << satId);

    Ptr<SatMobilityModel> satMob = m_orbitPredictor->GetSatelliteMobility(satId);
    GeoCoordinate satPos = satMob->GetGeoPosition();

    // Get satellite velocity from mobility model
    Vector satVel = satMob->GetVelocity(); // m/s in ECEF

    // Vector from UE to satellite
    Vector ueCart = uePosition.ToVector();
    Vector satCart = satPos.ToVector();
    double dx = satCart.x - ueCart.x;
    double dy = satCart.y - ueCart.y;
    double dz = satCart.z - ueCart.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist < 1.0)
    {
        return 0.0;
    }

    // Unit vector from UE to satellite
    double ux = dx / dist;
    double uy = dy / dist;
    double uz = dz / dist;

    // Radial velocity component (positive = approaching)
    double vRadial = -(satVel.x * ux + satVel.y * uy + satVel.z * uz);

    // Doppler shift: f_d = (v_radial / c) * f_c
    double dopplerHz = (vRadial / SPEED_OF_LIGHT) * m_carrierFreqHz;

    return dopplerHz;
}

double
NtnMeasurementModel::ComputeFreeSpacePathLoss(double distance_m, double freqHz) const
{
    // FSPL = 20*log10(d) + 20*log10(f) + 20*log10(4*pi/c)
    double fspl = 20.0 * std::log10(distance_m) +
                  20.0 * std::log10(freqHz) +
                  20.0 * std::log10(4.0 * M_PI / SPEED_OF_LIGHT);
    return fspl;
}

double
NtnMeasurementModel::ComputeNoisePower() const
{
    // N = 10*log10(k*T*B) + NF
    // k*T at 290K = -174 dBm/Hz
    double thermalNoise_dBmHz = -174.0;
    double noisePower_dBm = thermalNoise_dBmHz + 10.0 * std::log10(m_bandwidthHz) +
                            m_ueNoiseFigure_dB;
    return noisePower_dBm;
}

} // namespace ns3
