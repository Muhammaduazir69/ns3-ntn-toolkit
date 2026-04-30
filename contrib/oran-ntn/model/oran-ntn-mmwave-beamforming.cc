/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * NTN-Aware mmWave Beamforming Model - Implementation
 *
 * Implements satellite-specific beamforming with elevation-dependent steering,
 * Doppler-induced beam squint correction per 3GPP TR 38.821, adaptive beam
 * width for low-elevation angles, and predictive beam tracking for LEO motion.
 */

#include "oran-ntn-mmwave-beamforming.h"

#include "oran-ntn-sat-bridge.h"

#include <ns3/boolean.h>
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/mobility-model.h>
#include <ns3/net-device.h>
#include <ns3/node.h>
#include <ns3/phased-array-model.h>
#include <ns3/simulator.h>
#include <ns3/nstime.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>
#include <complex>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnMmWaveBeamforming");
NS_OBJECT_ENSURE_REGISTERED(OranNtnMmWaveBeamforming);

// Physical constants
static const double SPEED_OF_LIGHT = 299792458.0;  // m/s
static const double DEG_TO_RAD = M_PI / 180.0;
static const double RAD_TO_DEG = 180.0 / M_PI;

// NTN beamforming defaults
static const double DEFAULT_BEAM_WIDTH_ELEV_DEG = 5.0;   // satellite beam half-power BW
static const double DEFAULT_BEAM_WIDTH_AZIM_DEG = 5.0;
static const double DEFAULT_CARRIER_FREQ_HZ = 28.0e9;    // Ka-band
static const double LOW_ELEVATION_THRESHOLD_DEG = 30.0;
static const double HIGH_ELEVATION_THRESHOLD_DEG = 60.0;
static const double MIN_ELEVATION_FOR_BEAM_DEG = 5.0;

TypeId
OranNtnMmWaveBeamforming::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnMmWaveBeamforming")
            .SetParent<mmwave::MmWaveBeamformingModel>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnMmWaveBeamforming>()
            .AddAttribute("BeamWidthElevation",
                          "Half-power beamwidth in elevation (degrees)",
                          DoubleValue(DEFAULT_BEAM_WIDTH_ELEV_DEG),
                          MakeDoubleAccessor(&OranNtnMmWaveBeamforming::m_beamWidthElev_deg),
                          MakeDoubleChecker<double>(0.1, 90.0))
            .AddAttribute("BeamWidthAzimuth",
                          "Half-power beamwidth in azimuth (degrees)",
                          DoubleValue(DEFAULT_BEAM_WIDTH_AZIM_DEG),
                          MakeDoubleAccessor(&OranNtnMmWaveBeamforming::m_beamWidthAzim_deg),
                          MakeDoubleChecker<double>(0.1, 90.0))
            .AddAttribute("TrackingInterval",
                          "Interval for periodic beam tracking updates",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&OranNtnMmWaveBeamforming::m_trackingInterval),
                          MakeTimeChecker())
            .AddAttribute("CarrierFrequency",
                          "Carrier frequency in Hz for wavelength computation",
                          DoubleValue(DEFAULT_CARRIER_FREQ_HZ),
                          MakeDoubleAccessor(&OranNtnMmWaveBeamforming::m_carrierFreqHz),
                          MakeDoubleChecker<double>(1e8, 100e9))
            .AddAttribute("DopplerSquintCorrection",
                          "Enable Doppler-aware beam squint correction",
                          BooleanValue(true),
                          MakeBooleanAccessor(
                              &OranNtnMmWaveBeamforming::m_dopplerSquintCorrection),
                          MakeBooleanChecker())
            .AddAttribute("PredictiveSteering",
                          "Enable predictive beam steering using SGP4 extrapolation",
                          BooleanValue(false),
                          MakeBooleanAccessor(
                              &OranNtnMmWaveBeamforming::m_predictiveSteering),
                          MakeBooleanChecker())
            .AddAttribute("PredictionHorizon",
                          "Look-ahead time for predictive beam steering",
                          TimeValue(MilliSeconds(50)),
                          MakeTimeAccessor(&OranNtnMmWaveBeamforming::m_predictionHorizon),
                          MakeTimeChecker())
            .AddAttribute("InterferenceNullSteering",
                          "Enable cross-beam interference null steering",
                          BooleanValue(false),
                          MakeBooleanAccessor(
                              &OranNtnMmWaveBeamforming::m_interferenceNullSteering),
                          MakeBooleanChecker())
            .AddAttribute("SatelliteId",
                          "Satellite ID for this beamforming instance",
                          UintegerValue(0),
                          MakeUintegerAccessor(&OranNtnMmWaveBeamforming::m_satId),
                          MakeUintegerChecker<uint32_t>())
            .AddTraceSource("BeamUpdated",
                            "A beam vector was updated (satId, elevation, azimuth)",
                            MakeTraceSourceAccessor(
                                &OranNtnMmWaveBeamforming::m_beamUpdated),
                            "ns3::OranNtnMmWaveBeamforming::BeamUpdatedTracedCallback")
            .AddTraceSource("SquintCorrected",
                            "Doppler squint correction applied (ueId, correctionDeg)",
                            MakeTraceSourceAccessor(
                                &OranNtnMmWaveBeamforming::m_squintCorrected),
                            "ns3::OranNtnMmWaveBeamforming::SquintCorrectedTracedCallback");
    return tid;
}

OranNtnMmWaveBeamforming::OranNtnMmWaveBeamforming()
    : m_satBridge(nullptr),
      m_satId(0),
      m_beamWidthElev_deg(DEFAULT_BEAM_WIDTH_ELEV_DEG),
      m_beamWidthAzim_deg(DEFAULT_BEAM_WIDTH_AZIM_DEG),
      m_trackingInterval(MilliSeconds(100)),
      m_dopplerSquintCorrection(true),
      m_predictiveSteering(false),
      m_predictionHorizon(MilliSeconds(50)),
      m_interferenceNullSteering(false),
      m_carrierFreqHz(DEFAULT_CARRIER_FREQ_HZ)
{
    NS_LOG_FUNCTION(this);
}

OranNtnMmWaveBeamforming::~OranNtnMmWaveBeamforming()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnMmWaveBeamforming::DoDispose()
{
    NS_LOG_FUNCTION(this);

    if (m_trackingEvent.IsPending())
    {
        Simulator::Cancel(m_trackingEvent);
    }

    m_satBridge = nullptr;
    m_beamStates.clear();

    mmwave::MmWaveBeamformingModel::DoDispose();
}

// --------------------------------------------------------------------------
//  Configuration setters
// --------------------------------------------------------------------------

void
OranNtnMmWaveBeamforming::SetSatBridge(Ptr<OranNtnSatBridge> bridge)
{
    NS_LOG_FUNCTION(this << bridge);
    m_satBridge = bridge;
}

void
OranNtnMmWaveBeamforming::SetSatelliteId(uint32_t satId)
{
    NS_LOG_FUNCTION(this << satId);
    m_satId = satId;
}

void
OranNtnMmWaveBeamforming::SetBeamWidth(double elevDeg, double azimDeg)
{
    NS_LOG_FUNCTION(this << elevDeg << azimDeg);
    m_beamWidthElev_deg = elevDeg;
    m_beamWidthAzim_deg = azimDeg;
}

void
OranNtnMmWaveBeamforming::SetTrackingInterval(Time interval)
{
    NS_LOG_FUNCTION(this << interval);
    m_trackingInterval = interval;

    // If tracking is already running, restart with the new interval
    if (m_trackingEvent.IsPending())
    {
        Simulator::Cancel(m_trackingEvent);
        m_trackingEvent =
            Simulator::Schedule(m_trackingInterval,
                                &OranNtnMmWaveBeamforming::PeriodicTrackingCallback,
                                this);
    }
}

void
OranNtnMmWaveBeamforming::SetDopplerSquintCorrection(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_dopplerSquintCorrection = enable;
}

void
OranNtnMmWaveBeamforming::SetPredictiveSteering(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_predictiveSteering = enable;
}

void
OranNtnMmWaveBeamforming::SetPredictionHorizon(Time horizon)
{
    NS_LOG_FUNCTION(this << horizon);
    m_predictionHorizon = horizon;
}

void
OranNtnMmWaveBeamforming::SetInterferenceNullSteering(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_interferenceNullSteering = enable;
}

void
OranNtnMmWaveBeamforming::SetCarrierFrequency(double freqHz)
{
    NS_LOG_FUNCTION(this << freqHz);
    m_carrierFreqHz = freqHz;
}

// --------------------------------------------------------------------------
//  State queries
// --------------------------------------------------------------------------

double
OranNtnMmWaveBeamforming::GetBeamTrackingError(uint32_t ueId) const
{
    auto it = m_beamStates.find(ueId);
    if (it == m_beamStates.end())
    {
        return 0.0;
    }
    return it->second.trackingError_deg;
}

uint32_t
OranNtnMmWaveBeamforming::GetActiveBeamCount() const
{
    return static_cast<uint32_t>(m_beamStates.size());
}

// --------------------------------------------------------------------------
//  Core beamforming override
// --------------------------------------------------------------------------

void
OranNtnMmWaveBeamforming::SetBeamformingVectorForDevice(
    Ptr<NetDevice> otherDevice,
    Ptr<PhasedArrayModel> otherAntenna)
{
    NS_LOG_FUNCTION(this << otherDevice << otherAntenna);

    NS_ASSERT_MSG(m_antenna, "Local antenna not set");
    NS_ASSERT_MSG(otherDevice, "Other device is null");
    NS_ASSERT_MSG(otherAntenna, "Other antenna is null");

    // Obtain the UE node ID to use as the UE identifier for the sat bridge
    Ptr<Node> otherNode = otherDevice->GetNode();
    NS_ASSERT_MSG(otherNode, "Other device has no node");
    uint32_t ueId = otherNode->GetId();

    NS_LOG_INFO("OranNtnMmWaveBeamforming: computing beam for UE " << ueId
                << " on satellite " << m_satId);

    // If no sat bridge, fall back to geometry-only DFT approach
    if (!m_satBridge)
    {
        NS_LOG_WARN("No SatBridge set, falling back to geometry-only beamforming");

        Ptr<MobilityModel> thisMob = m_device->GetNode()->GetObject<MobilityModel>();
        Ptr<MobilityModel> otherMob = otherNode->GetObject<MobilityModel>();
        NS_ASSERT_MSG(thisMob && otherMob, "Mobility models required for fallback");

        Vector aPos = thisMob->GetPosition();
        Vector bPos = otherMob->GetPosition();

        Angles angleToUe(bPos, aPos);
        PhasedArrayModel::ComplexVector bfWeights =
            m_antenna->GetBeamformingVector(angleToUe);
        m_antenna->SetBeamformingVector(bfWeights);

        Angles angleToSat(aPos, bPos);
        PhasedArrayModel::ComplexVector otherWeights =
            otherAntenna->GetBeamformingVector(angleToSat);
        otherAntenna->SetBeamformingVector(otherWeights);
        return;
    }

    // --- NTN-aware beamforming using the satellite bridge ---

    // Query real-time satellite-UE geometry from SGP4
    double elevationDeg = m_satBridge->ComputeElevationAngle(ueId, m_satId);
    double dopplerHz = m_satBridge->ComputeDopplerShift(ueId, m_satId, m_carrierFreqHz);

    // Compute azimuth from 3D positions
    // Get positions and compute azimuth in the UE local frame
    GeoCoordinate uePos = m_satBridge->GetUePosition(ueId);
    GeoCoordinate satPos = m_satBridge->GetSatellitePosition(m_satId);

    // Azimuth: angle in the horizontal plane from North, measured clockwise
    // Use the difference in longitude and latitude for azimuth estimation
    double dLon = (satPos.GetLongitude() - uePos.GetLongitude()) * DEG_TO_RAD;
    double ueLat = uePos.GetLatitude() * DEG_TO_RAD;
    double satLat = satPos.GetLatitude() * DEG_TO_RAD;

    // Standard geodetic azimuth formula
    double azimuthRad = std::atan2(
        std::sin(dLon) * std::cos(satLat),
        std::cos(ueLat) * std::sin(satLat) -
            std::sin(ueLat) * std::cos(satLat) * std::cos(dLon));
    double azimuthDeg = azimuthRad * RAD_TO_DEG;
    if (azimuthDeg < 0.0)
    {
        azimuthDeg += 360.0;
    }

    NS_LOG_INFO("  Elevation=" << elevationDeg << " deg, Azimuth=" << azimuthDeg
                << " deg, Doppler=" << dopplerHz << " Hz");

    // Clamp elevation for safety
    if (elevationDeg < MIN_ELEVATION_FOR_BEAM_DEG)
    {
        NS_LOG_WARN("Elevation " << elevationDeg
                    << " deg is below minimum threshold, clamping to "
                    << MIN_ELEVATION_FOR_BEAM_DEG);
        elevationDeg = MIN_ELEVATION_FOR_BEAM_DEG;
    }

    // Compute NTN beamforming vector for satellite antenna (local side)
    uint32_t numLocalElements = static_cast<uint32_t>(m_antenna->GetNumElems());
    PhasedArrayModel::ComplexVector localWeights =
        ComputeNtnBeamformingVector(elevationDeg, azimuthDeg, dopplerHz, numLocalElements);
    m_antenna->SetBeamformingVector(localWeights);

    // For the UE side, use a simpler direct-pointing approach
    // The UE steers its beam toward the satellite (elevation, azimuth in UE frame)
    // Convert elevation/azimuth to ns3::Angles (phi = azimuth, theta = 90 - elevation)
    double thetaRad = (90.0 - elevationDeg) * DEG_TO_RAD;
    double phiRad = azimuthDeg * DEG_TO_RAD;
    Angles angleToSat(phiRad, thetaRad);
    PhasedArrayModel::ComplexVector ueWeights =
        otherAntenna->GetBeamformingVector(angleToSat);
    otherAntenna->SetBeamformingVector(ueWeights);

    // Update beam state tracking
    BeamState& state = m_beamStates[ueId];
    double prevElev = state.currentElevation_deg;
    double prevAzim = state.currentAzimuth_deg;
    state.currentElevation_deg = elevationDeg;
    state.currentAzimuth_deg = azimuthDeg;
    state.currentDoppler_Hz = dopplerHz;
    state.weights = localWeights;

    // Compute tracking error as angular distance from previous pointing direction
    if (prevElev != 0.0 || prevAzim != 0.0)
    {
        double dElev = elevationDeg - prevElev;
        double dAzim = azimuthDeg - prevAzim;
        // Handle azimuth wrap-around
        if (dAzim > 180.0)
        {
            dAzim -= 360.0;
        }
        else if (dAzim < -180.0)
        {
            dAzim += 360.0;
        }
        state.trackingError_deg =
            std::sqrt(dElev * dElev + dAzim * dAzim * std::cos(elevationDeg * DEG_TO_RAD) *
                                          std::cos(elevationDeg * DEG_TO_RAD));
    }
    else
    {
        state.trackingError_deg = 0.0;
    }

    // Fire trace
    m_beamUpdated(m_satId, elevationDeg, azimuthDeg);

    // Start periodic tracking if not already running
    if (!m_trackingEvent.IsPending())
    {
        m_trackingEvent =
            Simulator::Schedule(m_trackingInterval,
                                &OranNtnMmWaveBeamforming::PeriodicTrackingCallback,
                                this);
    }
}

// --------------------------------------------------------------------------
//  Core NTN beamforming vector computation
// --------------------------------------------------------------------------

PhasedArrayModel::ComplexVector
OranNtnMmWaveBeamforming::ComputeNtnBeamformingVector(
    double elevationDeg,
    double azimuthDeg,
    double dopplerHz,
    uint32_t numElements) const
{
    NS_LOG_FUNCTION(this << elevationDeg << azimuthDeg << dopplerHz << numElements);

    PhasedArrayModel::ComplexVector weights(numElements);

    if (numElements == 0)
    {
        return weights;
    }

    // --- Step 1: Convert elevation/azimuth to steering direction ---
    // In antenna-local coordinates: theta is the zenith angle (0 = boresight for nadir-pointing),
    // phi is the azimuth angle in the antenna plane.
    // For a satellite looking down, theta = 90 - elevation when the antenna boresight = nadir.
    double thetaSteering_rad = (90.0 - elevationDeg) * DEG_TO_RAD;
    double phiSteering_rad = azimuthDeg * DEG_TO_RAD;

    // --- Step 2: Apply Doppler squint correction if enabled ---
    double squintCorrection_deg = 0.0;
    if (m_dopplerSquintCorrection && std::abs(dopplerHz) > 1.0)
    {
        squintCorrection_deg = ComputeBeamSquint(dopplerHz, elevationDeg);
        // Pre-correct the steering angle by subtracting the squint
        // The squint displaces the beam in the elevation plane along the velocity direction,
        // so we apply the correction to the theta (zenith) steering angle
        thetaSteering_rad -= squintCorrection_deg * DEG_TO_RAD;

        NS_LOG_INFO("  Doppler squint correction: " << squintCorrection_deg
                    << " deg for Doppler=" << dopplerHz << " Hz");
    }

    // Clamp theta to valid range [0, pi]
    thetaSteering_rad = std::max(0.0, std::min(M_PI, thetaSteering_rad));

    // --- Step 3: Compute steering direction cosines ---
    // u = sin(theta) * cos(phi), v = sin(theta) * sin(phi), w = cos(theta)
    double sinTheta = std::sin(thetaSteering_rad);
    [[maybe_unused]] double cosTheta = std::cos(thetaSteering_rad);
    double sinPhi = std::sin(phiSteering_rad);
    double cosPhi = std::cos(phiSteering_rad);

    double u = sinTheta * cosPhi;
    double v = sinTheta * sinPhi;

    // --- Step 4: Compute adaptive beam width ---
    double adaptiveBW = ComputeAdaptiveBeamWidth(elevationDeg);
    NS_LOG_DEBUG("  Adaptive beam width: " << adaptiveBW << " deg (nominal="
                 << m_beamWidthElev_deg << " deg)");

    // --- Step 5: Compute uniform planar array (UPA) response vector ---
    // Assume a square-ish UPA: Nrows x Ncols ~ sqrt(N) x sqrt(N)
    uint32_t Ncols = static_cast<uint32_t>(std::ceil(std::sqrt(static_cast<double>(numElements))));
    uint32_t Nrows = (numElements + Ncols - 1) / Ncols;

    // Wavelength
    double lambda = SPEED_OF_LIGHT / m_carrierFreqHz;
    // Inter-element spacing (half-wavelength)
    double d = lambda / 2.0;

    // Phase progression per element
    // For a UPA with elements at positions (m*d, n*d) in the antenna plane,
    // the steering vector element is:
    //   w_{m,n} = exp(j * 2*pi/lambda * (m*d*u + n*d*v)) * taper(m,n)
    // where u,v are direction cosines

    double k = 2.0 * M_PI / lambda;  // wavenumber

    // --- Step 6: Apply elevation-adaptive amplitude taper ---
    // At low elevation, apply a wider taper (Taylor-like) to broaden the beam.
    // The taper factor controls the sidelobe/mainlobe tradeoff.
    // Nominal beam: no taper (uniform). Widened beam: cosine or Taylor taper.
    double taperSigma = 0.0;
    if (adaptiveBW > m_beamWidthElev_deg * 1.01)
    {
        // Gaussian taper: w(r) = exp(-r^2 / (2*sigma^2))
        // A wider beam needs a narrower taper in the aperture domain.
        // The relationship: beamwidth ~ lambda / (D_eff), where D_eff is effective aperture.
        // To widen by factor F, reduce effective aperture by F => taper sigma = D/(2*F)
        double wideningFactor = adaptiveBW / m_beamWidthElev_deg;
        // Normalize sigma relative to half the array extent
        taperSigma = 1.0 / (wideningFactor * 1.2);  // empirical scaling
    }

    // Compute total power for normalization
    double totalPower = 0.0;
    std::vector<std::complex<double>> rawWeights(numElements);

    for (uint32_t idx = 0; idx < numElements; ++idx)
    {
        uint32_t row = idx / Ncols;
        uint32_t col = idx % Ncols;

        // Element position in the aperture plane (centered)
        double mPos = (static_cast<double>(col) - 0.5 * (Ncols - 1));
        double nPos = (static_cast<double>(row) - 0.5 * (Nrows - 1));

        // Phase for steering
        double phase = k * d * (mPos * u + nPos * v);

        // Amplitude taper
        double amplitude = 1.0;
        if (taperSigma > 0.0)
        {
            // Normalized distance from array center
            double rNorm = std::sqrt(
                (mPos * mPos) / (0.25 * (Ncols - 1) * (Ncols - 1) + 1e-12) +
                (nPos * nPos) / (0.25 * (Nrows - 1) * (Nrows - 1) + 1e-12));
            amplitude = std::exp(-0.5 * rNorm * rNorm / (taperSigma * taperSigma));
        }

        rawWeights[idx] = amplitude * std::exp(std::complex<double>(0.0, phase));
        totalPower += amplitude * amplitude;
    }

    // Normalize so that ||w||^2 = 1 (unit-norm beamforming vector)
    double normFactor = std::sqrt(totalPower);
    if (normFactor < 1e-15)
    {
        normFactor = 1.0;
    }

    for (uint32_t idx = 0; idx < numElements; ++idx)
    {
        weights[idx] = rawWeights[idx] / normFactor;
    }

    NS_LOG_INFO("  NTN BF vector computed: " << numElements << " elements, "
                << Nrows << "x" << Ncols << " UPA, theta="
                << thetaSteering_rad * RAD_TO_DEG << " deg, phi="
                << phiSteering_rad * RAD_TO_DEG << " deg");

    return weights;
}

// --------------------------------------------------------------------------
//  Doppler-induced beam squint computation (3GPP TR 38.821 Sec 6.1.4)
// --------------------------------------------------------------------------

double
OranNtnMmWaveBeamforming::ComputeBeamSquint(double dopplerHz, double elevationDeg) const
{
    NS_LOG_FUNCTION(this << dopplerHz << elevationDeg);

    // Beam squint arises because the Doppler shift causes a frequency-dependent
    // phase progression across the array aperture. For a broadband signal, the
    // beam direction shifts slightly from the intended steering angle.
    //
    // The squint angle is:
    //   squint = arcsin( (f_d / f_c) * sin(elevation_rad) )
    //
    // where f_d is the Doppler shift, f_c is the carrier frequency,
    // and elevation_rad is the elevation angle.
    //
    // This formula follows from the fact that the effective beam direction
    // changes proportionally to the fractional frequency offset projected
    // onto the scan direction.

    double elevationRad = elevationDeg * DEG_TO_RAD;
    double sinElev = std::sin(elevationRad);

    // Fractional frequency offset
    double freqRatio = dopplerHz / m_carrierFreqHz;

    // Argument of arcsin: must be in [-1, 1]
    double sinArg = freqRatio * sinElev;
    sinArg = std::max(-1.0, std::min(1.0, sinArg));

    double squintRad = std::asin(sinArg);
    double squintDeg = squintRad * RAD_TO_DEG;

    NS_LOG_DEBUG("  BeamSquint: Doppler=" << dopplerHz << " Hz, fc=" << m_carrierFreqHz
                 << " Hz, elev=" << elevationDeg << " deg => squint=" << squintDeg << " deg");

    return squintDeg;
}

// --------------------------------------------------------------------------
//  Elevation-adaptive beam width
// --------------------------------------------------------------------------

double
OranNtnMmWaveBeamforming::ComputeAdaptiveBeamWidth(double elevationDeg) const
{
    NS_LOG_FUNCTION(this << elevationDeg);

    // At high elevation (> 60 deg), the satellite is nearly overhead and the
    // beam footprint on the ground is compact. Use the nominal beam width.
    //
    // At low elevation (< 30 deg), the beam footprint elongates due to the
    // oblique incidence angle. The ground-level footprint expands by a factor
    // of approximately 1/sin(elevation). To maintain adequate coverage, we
    // widen the beam accordingly.
    //
    // Between 30 deg and 60 deg, linearly interpolate.

    if (elevationDeg >= HIGH_ELEVATION_THRESHOLD_DEG)
    {
        // High elevation: use nominal beam width
        return m_beamWidthElev_deg;
    }

    // Clamp to minimum for numerical stability
    double clampedElev = std::max(elevationDeg, MIN_ELEVATION_FOR_BEAM_DEG);

    if (clampedElev <= LOW_ELEVATION_THRESHOLD_DEG)
    {
        // Low elevation: widen beam by 1/sin(elevation)
        double sinElev = std::sin(clampedElev * DEG_TO_RAD);
        // Limit the maximum widening to prevent unreasonably wide beams
        // At 5 deg elevation, sin(5) ~ 0.087, so widening factor ~ 11.5
        // Cap at 4x nominal to remain practical
        double wideningFactor = std::min(1.0 / sinElev, 4.0);
        double adaptiveWidth = m_beamWidthElev_deg * wideningFactor;

        NS_LOG_DEBUG("  Low-elev adaptive BW: elev=" << clampedElev
                     << " deg, widening=" << wideningFactor
                     << "x, BW=" << adaptiveWidth << " deg");
        return adaptiveWidth;
    }

    // Mid-range: linear interpolation between low-elev widened and nominal
    // At LOW_ELEVATION_THRESHOLD_DEG: use low-elevation formula
    // At HIGH_ELEVATION_THRESHOLD_DEG: use nominal
    double sinLowElev = std::sin(LOW_ELEVATION_THRESHOLD_DEG * DEG_TO_RAD);
    double wideningAtLow = std::min(1.0 / sinLowElev, 4.0);
    double bwAtLow = m_beamWidthElev_deg * wideningAtLow;

    double t = (clampedElev - LOW_ELEVATION_THRESHOLD_DEG) /
               (HIGH_ELEVATION_THRESHOLD_DEG - LOW_ELEVATION_THRESHOLD_DEG);

    double adaptiveWidth = bwAtLow + t * (m_beamWidthElev_deg - bwAtLow);

    NS_LOG_DEBUG("  Mid-elev adaptive BW: elev=" << clampedElev
                 << " deg, t=" << t << ", BW=" << adaptiveWidth << " deg");
    return adaptiveWidth;
}

// --------------------------------------------------------------------------
//  Beam tracking update
// --------------------------------------------------------------------------

void
OranNtnMmWaveBeamforming::UpdateBeamTracking()
{
    NS_LOG_FUNCTION(this);

    if (!m_satBridge || !m_antenna)
    {
        NS_LOG_WARN("Cannot update beam tracking: SatBridge or antenna not set");
        return;
    }

    NS_LOG_INFO("OranNtnMmWaveBeamforming: updating beam tracking for "
                << m_beamStates.size() << " tracked UEs");

    for (auto& entry : m_beamStates)
    {
        uint32_t ueId = entry.first;
        BeamState& state = entry.second;

        // Store previous angles for tracking error computation
        double prevElev = state.currentElevation_deg;
        double prevAzim = state.currentAzimuth_deg;

        // Query updated satellite-UE geometry
        double newElevation = m_satBridge->ComputeElevationAngle(ueId, m_satId);
        double newDoppler = m_satBridge->ComputeDopplerShift(ueId, m_satId, m_carrierFreqHz);

        // Compute updated azimuth
        GeoCoordinate uePos = m_satBridge->GetUePosition(ueId);
        GeoCoordinate satPos = m_satBridge->GetSatellitePosition(m_satId);

        double dLon = (satPos.GetLongitude() - uePos.GetLongitude()) * DEG_TO_RAD;
        double ueLat = uePos.GetLatitude() * DEG_TO_RAD;
        double satLat = satPos.GetLatitude() * DEG_TO_RAD;

        double azimuthRad = std::atan2(
            std::sin(dLon) * std::cos(satLat),
            std::cos(ueLat) * std::sin(satLat) -
                std::sin(ueLat) * std::cos(satLat) * std::cos(dLon));
        double newAzimuth = azimuthRad * RAD_TO_DEG;
        if (newAzimuth < 0.0)
        {
            newAzimuth += 360.0;
        }

        // Skip update if elevation is below minimum
        if (newElevation < MIN_ELEVATION_FOR_BEAM_DEG)
        {
            NS_LOG_DEBUG("  UE " << ueId << " below minimum elevation ("
                         << newElevation << " deg), skipping tracking update");
            continue;
        }

        // Recompute NTN beamforming vector
        uint32_t numElements = static_cast<uint32_t>(m_antenna->GetNumElems());
        PhasedArrayModel::ComplexVector newWeights =
            ComputeNtnBeamformingVector(newElevation, newAzimuth, newDoppler, numElements);

        // Update antenna with new weights
        m_antenna->SetBeamformingVector(newWeights);

        // Compute tracking error: angular distance between old and new pointing
        double dElev = newElevation - prevElev;
        double dAzim = newAzimuth - prevAzim;
        if (dAzim > 180.0)
        {
            dAzim -= 360.0;
        }
        else if (dAzim < -180.0)
        {
            dAzim += 360.0;
        }
        // Great-circle angular distance approximation for small angles
        double cosElev = std::cos(newElevation * DEG_TO_RAD);
        double trackingError = std::sqrt(dElev * dElev + dAzim * dAzim * cosElev * cosElev);

        // Update state
        state.currentElevation_deg = newElevation;
        state.currentAzimuth_deg = newAzimuth;
        state.currentDoppler_Hz = newDoppler;
        state.trackingError_deg = trackingError;
        state.weights = newWeights;

        // Apply Doppler squint correction trace
        if (m_dopplerSquintCorrection && std::abs(newDoppler) > 1.0)
        {
            double squint = ComputeBeamSquint(newDoppler, newElevation);
            state.squintCorrection_deg = squint;
            m_squintCorrected(ueId, squint);
        }

        NS_LOG_INFO("  UE " << ueId << ": elev=" << newElevation
                    << " deg, azim=" << newAzimuth << " deg, Doppler="
                    << newDoppler << " Hz, tracking_error=" << trackingError << " deg");

        // Fire beam updated trace
        m_beamUpdated(m_satId, newElevation, newAzimuth);
    }

    // If predictive steering is enabled, apply forward-looking correction
    if (m_predictiveSteering)
    {
        PredictiveSteeringUpdate();
    }
}

// --------------------------------------------------------------------------
//  Periodic tracking callback
// --------------------------------------------------------------------------

void
OranNtnMmWaveBeamforming::PeriodicTrackingCallback()
{
    NS_LOG_FUNCTION(this);

    UpdateBeamTracking();

    // Re-schedule the next tracking update
    m_trackingEvent =
        Simulator::Schedule(m_trackingInterval,
                            &OranNtnMmWaveBeamforming::PeriodicTrackingCallback,
                            this);
}

// --------------------------------------------------------------------------
//  Predictive beam steering
// --------------------------------------------------------------------------

void
OranNtnMmWaveBeamforming::PredictiveSteeringUpdate()
{
    NS_LOG_FUNCTION(this);

    if (!m_satBridge)
    {
        return;
    }

    // Get current satellite velocity from SGP4 propagation
    Vector satVelocity = m_satBridge->GetSatelliteVelocity(m_satId);

    // Current satellite position
    GeoCoordinate satPos = m_satBridge->GetSatellitePosition(m_satId);

    // Prediction horizon in seconds
    double dtSec = m_predictionHorizon.GetSeconds();

    NS_LOG_INFO("PredictiveSteeringUpdate: horizon=" << dtSec << " s, satVel=("
                << satVelocity.x << "," << satVelocity.y << "," << satVelocity.z << ") m/s");

    // Extrapolate future satellite position using first-order linear prediction.
    // For LEO at ~7.5 km/s, 50 ms look-ahead = ~375 m displacement.
    // This is a linearized approximation; for longer horizons, use SGP4 directly.
    //
    // The velocity is in ECEF, so we convert the position to ECEF,
    // apply the velocity extrapolation, and convert back to geodetic.

    // Earth radius for local conversion (approximate)
    static const double EARTH_RADIUS_M = 6371000.0;

    double satLatRad = satPos.GetLatitude() * DEG_TO_RAD;
    double satLonRad = satPos.GetLongitude() * DEG_TO_RAD;
    double satAlt = satPos.GetAltitude();
    double R = EARTH_RADIUS_M + satAlt;

    // Current ECEF position
    double cosLat = std::cos(satLatRad);
    double sinLat = std::sin(satLatRad);
    double cosLon = std::cos(satLonRad);
    double sinLon = std::sin(satLonRad);

    double ecefX = R * cosLat * cosLon;
    double ecefY = R * cosLat * sinLon;
    double ecefZ = R * sinLat;

    // Extrapolate
    double futX = ecefX + satVelocity.x * dtSec;
    double futY = ecefY + satVelocity.y * dtSec;
    double futZ = ecefZ + satVelocity.z * dtSec;

    // Convert future ECEF back to geodetic (simplified)
    double futR = std::sqrt(futX * futX + futY * futY + futZ * futZ);
    double futLatRad = std::asin(futZ / futR);
    double futLonRad = std::atan2(futY, futX);
    double futAlt = futR - EARTH_RADIUS_M;

    double futLatDeg = futLatRad * RAD_TO_DEG;
    double futLonDeg = futLonRad * RAD_TO_DEG;

    NS_LOG_DEBUG("  Predicted sat position: lat=" << futLatDeg << ", lon=" << futLonDeg
                 << ", alt=" << futAlt / 1000.0 << " km");

    // For each tracked UE, compute future steering angles and apply correction
    for (auto& entry : m_beamStates)
    {
        uint32_t ueId = entry.first;
        BeamState& state = entry.second;

        GeoCoordinate uePos = m_satBridge->GetUePosition(ueId);
        double ueLat = uePos.GetLatitude() * DEG_TO_RAD;
        double ueLon = uePos.GetLongitude() * DEG_TO_RAD;

        // Compute future azimuth from UE to predicted satellite position
        double dLon = (futLonDeg * DEG_TO_RAD) - ueLon;
        double futAzimRad = std::atan2(
            std::sin(dLon) * std::cos(futLatRad),
            std::cos(ueLat) * std::sin(futLatRad) -
                std::sin(ueLat) * std::cos(futLatRad) * std::cos(dLon));
        double futAzimDeg = futAzimRad * RAD_TO_DEG;
        if (futAzimDeg < 0.0)
        {
            futAzimDeg += 360.0;
        }

        // Compute future elevation angle
        // Use the UE ECEF position
        double ueR = EARTH_RADIUS_M;  // approximate for ground UE
        double ueEcefX = ueR * std::cos(ueLat) * std::cos(ueLon);
        double ueEcefY = ueR * std::cos(ueLat) * std::sin(ueLon);
        double ueEcefZ = ueR * std::sin(ueLat);

        // Direction from UE to future satellite
        double dx = futX - ueEcefX;
        double dy = futY - ueEcefY;
        double dz = futZ - ueEcefZ;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist < 1.0)
        {
            continue;
        }

        dx /= dist;
        dy /= dist;
        dz /= dist;

        // UE local up vector
        double ueNorm = std::sqrt(ueEcefX * ueEcefX + ueEcefY * ueEcefY + ueEcefZ * ueEcefZ);
        double upX = ueEcefX / ueNorm;
        double upY = ueEcefY / ueNorm;
        double upZ = ueEcefZ / ueNorm;

        double dotUp = dx * upX + dy * upY + dz * upZ;
        dotUp = std::max(-1.0, std::min(1.0, dotUp));
        double futElevDeg = std::asin(dotUp) * RAD_TO_DEG;

        if (futElevDeg < MIN_ELEVATION_FOR_BEAM_DEG)
        {
            continue;
        }

        NS_LOG_DEBUG("  UE " << ueId << " predictive: futElev=" << futElevDeg
                     << " deg, futAzim=" << futAzimDeg << " deg");

        // Blend current and predicted steering angles.
        // Use weighted average: weight current by (1-alpha), predicted by alpha.
        // alpha depends on how far into the future we predict relative to tracking interval.
        double alpha = std::min(
            dtSec / std::max(m_trackingInterval.GetSeconds(), 1e-6), 1.0);

        double blendedElev =
            (1.0 - alpha) * state.currentElevation_deg + alpha * futElevDeg;
        double blendedAzim = state.currentAzimuth_deg;

        // Handle azimuth blending with wrap-around
        double azimDiff = futAzimDeg - state.currentAzimuth_deg;
        if (azimDiff > 180.0)
        {
            azimDiff -= 360.0;
        }
        else if (azimDiff < -180.0)
        {
            azimDiff += 360.0;
        }
        blendedAzim = state.currentAzimuth_deg + alpha * azimDiff;
        if (blendedAzim < 0.0)
        {
            blendedAzim += 360.0;
        }
        else if (blendedAzim >= 360.0)
        {
            blendedAzim -= 360.0;
        }

        // Recompute beamforming vector with predicted angles
        uint32_t numElements = static_cast<uint32_t>(m_antenna->GetNumElems());
        PhasedArrayModel::ComplexVector predictedWeights =
            ComputeNtnBeamformingVector(blendedElev, blendedAzim,
                                        state.currentDoppler_Hz, numElements);

        m_antenna->SetBeamformingVector(predictedWeights);
        state.weights = predictedWeights;

        NS_LOG_INFO("  UE " << ueId << " predictive steering applied: blendedElev="
                    << blendedElev << " deg, blendedAzim=" << blendedAzim << " deg"
                    << ", alpha=" << alpha);
    }
}

// --------------------------------------------------------------------------
//  Interference null steering
// --------------------------------------------------------------------------

double
OranNtnMmWaveBeamforming::ComputeNullDirection(
    uint32_t targetBeamId,
    const std::vector<uint32_t>& interferingBeams) const
{
    NS_LOG_FUNCTION(this << targetBeamId << interferingBeams.size());

    // Compute the angular direction where a null should be placed to
    // suppress interference from adjacent beams.
    //
    // For each interfering beam, determine the angular separation from the
    // target beam. Place the null at the centroid of the interfering directions.

    if (interferingBeams.empty() || !m_satBridge)
    {
        return 0.0;
    }

    // Get the target beam's current pointing direction
    auto targetIt = m_beamStates.find(targetBeamId);
    if (targetIt == m_beamStates.end())
    {
        return 0.0;
    }

    double targetAzim = targetIt->second.currentAzimuth_deg;
    double targetElev = targetIt->second.currentElevation_deg;

    // Compute mean interferer direction relative to target
    double sumDeltaAzim = 0.0;
    double sumDeltaElev = 0.0;
    uint32_t count = 0;

    for (uint32_t intfId : interferingBeams)
    {
        auto intfIt = m_beamStates.find(intfId);
        if (intfIt == m_beamStates.end())
        {
            continue;
        }

        double dAzim = intfIt->second.currentAzimuth_deg - targetAzim;
        if (dAzim > 180.0)
        {
            dAzim -= 360.0;
        }
        else if (dAzim < -180.0)
        {
            dAzim += 360.0;
        }
        double dElev = intfIt->second.currentElevation_deg - targetElev;

        sumDeltaAzim += dAzim;
        sumDeltaElev += dElev;
        ++count;
    }

    if (count == 0)
    {
        return 0.0;
    }

    // Null direction as angular offset from target
    double nullAzimOffset = sumDeltaAzim / count;
    double nullElevOffset = sumDeltaElev / count;

    // Return the overall angular offset magnitude
    double cosElev = std::cos(targetElev * DEG_TO_RAD);
    double nullAngle = std::sqrt(nullElevOffset * nullElevOffset +
                                 nullAzimOffset * nullAzimOffset * cosElev * cosElev);

    NS_LOG_DEBUG("  Null direction for beam " << targetBeamId << ": offset="
                 << nullAngle << " deg towards " << interferingBeams.size()
                 << " interferers");

    return nullAngle;
}

} // namespace ns3
