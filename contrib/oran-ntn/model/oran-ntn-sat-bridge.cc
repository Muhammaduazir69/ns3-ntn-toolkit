/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * O-RAN NTN Satellite Bridge - Implementation
 */

#include "oran-ntn-sat-bridge.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/string.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnSatBridge");
NS_OBJECT_ENSURE_REGISTERED(OranNtnSatBridge);

// Physical constants
static const double SPEED_OF_LIGHT = 299792458.0;     // m/s
static const double BOLTZMANN = 1.380649e-23;          // J/K
static const double TEMPERATURE_K = 290.0;             // K
static const double EARTH_RADIUS = 6371000.0;          // m
static const double DEG_TO_RAD = M_PI / 180.0;
static const double RAD_TO_DEG = 180.0 / M_PI;

TypeId
OranNtnSatBridge::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnSatBridge")
            .SetParent<Object>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnSatBridge>()
            .AddAttribute("NtnScenario",
                          "NTN propagation scenario (DenseUrban, Urban, Suburban, Rural)",
                          StringValue("Urban"),
                          MakeStringAccessor(&OranNtnSatBridge::m_ntnScenario),
                          MakeStringChecker())
            .AddAttribute("CarrierFrequency",
                          "Carrier frequency in Hz",
                          DoubleValue(2e9),
                          MakeDoubleAccessor(&OranNtnSatBridge::m_carrierFreqHz),
                          MakeDoubleChecker<double>())
            .AddAttribute("TxPower",
                          "Satellite TX power per beam in dBm",
                          DoubleValue(40.0),
                          MakeDoubleAccessor(&OranNtnSatBridge::m_txPower_dBm),
                          MakeDoubleChecker<double>())
            .AddAttribute("Bandwidth",
                          "System bandwidth in Hz",
                          DoubleValue(30e6),
                          MakeDoubleAccessor(&OranNtnSatBridge::m_bandwidth_Hz),
                          MakeDoubleChecker<double>())
            .AddAttribute("NoiseFigure",
                          "Receiver noise figure in dB",
                          DoubleValue(7.0),
                          MakeDoubleAccessor(&OranNtnSatBridge::m_noiseFigure_dB),
                          MakeDoubleChecker<double>())
            .AddTraceSource("KpmGenerated",
                            "A KPM report was generated from satellite state",
                            MakeTraceSourceAccessor(&OranNtnSatBridge::m_kpmGenerated),
                            "ns3::OranNtnSatBridge::KpmGeneratedTracedCallback")
            .AddTraceSource("FeederLinkChanged",
                            "Feeder link availability changed for a satellite",
                            MakeTraceSourceAccessor(&OranNtnSatBridge::m_feederLinkChanged),
                            "ns3::OranNtnSatBridge::FeederLinkChangedTracedCallback")
            .AddTraceSource("ElevationComputed",
                            "Elevation angle computed between UE and satellite",
                            MakeTraceSourceAccessor(&OranNtnSatBridge::m_elevationComputed),
                            "ns3::OranNtnSatBridge::ElevationComputedTracedCallback");
    return tid;
}

OranNtnSatBridge::OranNtnSatBridge()
    : m_numPlanes(0),
      m_satsPerPlane(0),
      m_antennaPatterns(nullptr),
      m_ntnPathloss(nullptr),
      m_ntnScenario("Urban"),
      m_carrierFreqHz(2e9),
      m_txPower_dBm(40.0),
      m_bandwidth_Hz(30e6),
      m_noiseFigure_dB(7.0),
      m_boltzmann(BOLTZMANN),
      m_temperature_K(TEMPERATURE_K),
      m_speedOfLight(SPEED_OF_LIGHT),
      m_kpmInterval(Seconds(1.0)),
      m_kpmFeedActive(false)
{
    NS_LOG_FUNCTION(this);
}

OranNtnSatBridge::~OranNtnSatBridge()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnSatBridge::DoDispose()
{
    NS_LOG_FUNCTION(this);
    StopKpmFeed();
    m_satStates.clear();
    m_ueStates.clear();
    m_antennaPatterns = nullptr;
    m_ntnPathloss = nullptr;
    Object::DoDispose();
}

// ---- Helper: GeoCoordinate -> ECEF Cartesian ----
static Vector
GeoToEcef(const GeoCoordinate& geo)
{
    double latRad = geo.GetLatitude() * DEG_TO_RAD;
    double lonRad = geo.GetLongitude() * DEG_TO_RAD;
    double alt = geo.GetAltitude();
    double r = EARTH_RADIUS + alt;
    double x = r * std::cos(latRad) * std::cos(lonRad);
    double y = r * std::cos(latRad) * std::sin(lonRad);
    double z = r * std::sin(latRad);
    return Vector(x, y, z);
}

// ---- Constellation setup ----

void
OranNtnSatBridge::InitializeConstellation(
    NodeContainer satNodes,
    uint32_t numPlanes,
    uint32_t satsPerPlane,
    Ptr<SatAntennaGainPatternContainer> antennaPatterns)
{
    NS_LOG_FUNCTION(this << satNodes.GetN() << numPlanes << satsPerPlane);

    m_numPlanes = numPlanes;
    m_satsPerPlane = satsPerPlane;
    m_antennaPatterns = antennaPatterns;

    for (uint32_t i = 0; i < satNodes.GetN(); ++i)
    {
        Ptr<Node> node = satNodes.Get(i);
        Ptr<SatMobilityModel> mobility = node->GetObject<SatMobilityModel>();

        SatelliteBridgeState state;
        state.satId = i;
        state.planeId = i / satsPerPlane;
        state.posInPlane = i % satsPerPlane;
        state.node = node;
        state.mobility = mobility;
        state.antennaPatterns = antennaPatterns;
        state.e2Node = nullptr;
        state.spaceRic = nullptr;
        state.feederLinkAvailable = true;
        state.servingGatewayId = 0;

        if (mobility)
        {
            GeoCoordinate pos = mobility->GetGeoPosition();
            state.lastPosition = pos;
            state.altitude_km = pos.GetAltitude() / 1000.0;
            state.lastVelocity = mobility->GetVelocity();
            state.groundSpeed_kmps =
                std::sqrt(state.lastVelocity.x * state.lastVelocity.x +
                          state.lastVelocity.y * state.lastVelocity.y) /
                1000.0;
        }
        else
        {
            state.lastPosition = GeoCoordinate(0, 0, 0);
            state.altitude_km = 0;
            state.lastVelocity = Vector(0, 0, 0);
            state.groundSpeed_kmps = 0;
        }

        m_satStates[i] = state;
        NS_LOG_INFO("Initialized satellite " << i << " plane=" << state.planeId
                                              << " pos=" << state.posInPlane
                                              << " alt=" << state.altitude_km << " km");
    }
}

void
OranNtnSatBridge::RegisterUeNodes(NodeContainer ueNodes)
{
    NS_LOG_FUNCTION(this << ueNodes.GetN());

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<Node> node = ueNodes.Get(i);
        UeBridgeState state;
        state.ueId = i;
        state.node = node;
        state.ueMobility = node->GetObject<SatMobilityModel>();
        state.servingSatId = 0;
        state.servingBeamId = 0;
        state.servingSinr_dB = 0;
        state.servingRsrp_dBm = -140;
        state.servingTte_s = 0;
        state.servingElevation_deg = 0;
        state.servingDoppler_Hz = 0;
        state.servingDelay_ms = 0;
        state.sliceId = 0;
        state.mobilityClass = UeBridgeState::STATIC;

        if (state.ueMobility)
        {
            state.position = state.ueMobility->GetGeoPosition();
            state.velocity = state.ueMobility->GetVelocity();
        }
        else
        {
            state.position = GeoCoordinate(0, 0, 0);
            state.velocity = Vector(0, 0, 0);
        }

        m_ueStates[i] = state;
    }
}

void
OranNtnSatBridge::AttachE2Nodes(const std::vector<Ptr<OranNtnE2Node>>& e2Nodes)
{
    NS_LOG_FUNCTION(this << e2Nodes.size());
    for (uint32_t i = 0; i < e2Nodes.size() && i < m_satStates.size(); ++i)
    {
        auto it = m_satStates.find(i);
        if (it != m_satStates.end())
        {
            it->second.e2Node = e2Nodes[i];
        }
    }
}

void
OranNtnSatBridge::AttachSpaceRics(const std::vector<Ptr<OranNtnSpaceRic>>& spaceRics)
{
    NS_LOG_FUNCTION(this << spaceRics.size());
    for (uint32_t i = 0; i < spaceRics.size() && i < m_satStates.size(); ++i)
    {
        auto it = m_satStates.find(i);
        if (it != m_satStates.end())
        {
            it->second.spaceRic = spaceRics[i];
        }
    }
}

// ---- Real-time satellite queries ----

GeoCoordinate
OranNtnSatBridge::GetSatellitePosition(uint32_t satId) const
{
    auto it = m_satStates.find(satId);
    NS_ASSERT_MSG(it != m_satStates.end(), "Invalid satId: " << satId);
    if (it->second.mobility)
    {
        return it->second.mobility->GetGeoPosition();
    }
    return it->second.lastPosition;
}

Vector
OranNtnSatBridge::GetSatelliteVelocity(uint32_t satId) const
{
    auto it = m_satStates.find(satId);
    NS_ASSERT_MSG(it != m_satStates.end(), "Invalid satId: " << satId);
    if (it->second.mobility)
    {
        return it->second.mobility->GetVelocity();
    }
    return it->second.lastVelocity;
}

GeoCoordinate
OranNtnSatBridge::GetUePosition(uint32_t ueId) const
{
    auto it = m_ueStates.find(ueId);
    NS_ASSERT_MSG(it != m_ueStates.end(), "Invalid ueId: " << ueId);
    if (it->second.ueMobility)
    {
        return it->second.ueMobility->GetGeoPosition();
    }
    return it->second.position;
}

double
OranNtnSatBridge::ComputeElevationAngle(uint32_t ueId, uint32_t satId) const
{
    NS_LOG_FUNCTION(this << ueId << satId);

    GeoCoordinate ueGeo = GetUePosition(ueId);
    GeoCoordinate satGeo = GetSatellitePosition(satId);

    Vector ueEcef = GeoToEcef(ueGeo);
    Vector satEcef = GeoToEcef(satGeo);

    // Direction from UE to satellite
    Vector satDir(satEcef.x - ueEcef.x,
                  satEcef.y - ueEcef.y,
                  satEcef.z - ueEcef.z);

    double satDist =
        std::sqrt(satDir.x * satDir.x + satDir.y * satDir.y + satDir.z * satDir.z);
    if (satDist < 1.0)
    {
        return 90.0;
    }

    // Normalize
    satDir.x /= satDist;
    satDir.y /= satDist;
    satDir.z /= satDist;

    // UE local up vector (radial direction from Earth center)
    double ueDist =
        std::sqrt(ueEcef.x * ueEcef.x + ueEcef.y * ueEcef.y + ueEcef.z * ueEcef.z);
    Vector upDir(ueEcef.x / ueDist, ueEcef.y / ueDist, ueEcef.z / ueDist);

    // Elevation = arcsin(dot(satDir, upDir))
    double dot = satDir.x * upDir.x + satDir.y * upDir.y + satDir.z * upDir.z;
    dot = std::max(-1.0, std::min(1.0, dot));
    double elevationDeg = std::asin(dot) * RAD_TO_DEG;

    m_elevationComputed(ueId, satId, elevationDeg);

    return elevationDeg;
}

double
OranNtnSatBridge::ComputeSlantRange(uint32_t ueId, uint32_t satId) const
{
    NS_LOG_FUNCTION(this << ueId << satId);

    GeoCoordinate ueGeo = GetUePosition(ueId);
    GeoCoordinate satGeo = GetSatellitePosition(satId);

    Vector ueEcef = GeoToEcef(ueGeo);
    Vector satEcef = GeoToEcef(satGeo);

    double dx = satEcef.x - ueEcef.x;
    double dy = satEcef.y - ueEcef.y;
    double dz = satEcef.z - ueEcef.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Time
OranNtnSatBridge::ComputePropagationDelay(uint32_t ueId, uint32_t satId) const
{
    double range = ComputeSlantRange(ueId, satId);
    return Seconds(range / m_speedOfLight);
}

double
OranNtnSatBridge::ComputeDopplerShift(uint32_t ueId,
                                       uint32_t satId,
                                       double carrierFreqHz) const
{
    NS_LOG_FUNCTION(this << ueId << satId << carrierFreqHz);

    GeoCoordinate ueGeo = GetUePosition(ueId);
    GeoCoordinate satGeo = GetSatellitePosition(satId);

    Vector ueEcef = GeoToEcef(ueGeo);
    Vector satEcef = GeoToEcef(satGeo);

    // Line-of-sight unit vector from UE to satellite
    Vector los(satEcef.x - ueEcef.x, satEcef.y - ueEcef.y, satEcef.z - ueEcef.z);
    double dist = std::sqrt(los.x * los.x + los.y * los.y + los.z * los.z);
    if (dist < 1.0)
    {
        return 0.0;
    }
    los.x /= dist;
    los.y /= dist;
    los.z /= dist;

    // Get satellite velocity
    Vector satVel = GetSatelliteVelocity(satId);

    // Radial velocity = projection of satellite velocity onto LOS
    double vRadial = satVel.x * los.x + satVel.y * los.y + satVel.z * los.z;

    // Doppler shift: negative vRadial means approaching -> positive Doppler
    return (-vRadial / m_speedOfLight) * carrierFreqHz;
}

double
OranNtnSatBridge::GetBeamGainAtUe(uint32_t ueId, uint32_t satId, uint32_t beamId) const
{
    NS_LOG_FUNCTION(this << ueId << satId << beamId);

    if (!m_antennaPatterns)
    {
        NS_LOG_WARN("No antenna patterns configured, returning 0 dB gain");
        return 0.0;
    }

    auto satIt = m_satStates.find(satId);
    NS_ASSERT_MSG(satIt != m_satStates.end(), "Invalid satId: " << satId);

    GeoCoordinate ueGeo = GetUePosition(ueId);
    Ptr<SatMobilityModel> satMobility = satIt->second.mobility;

    if (!satMobility)
    {
        return 0.0;
    }

    Ptr<SatAntennaGainPattern> pattern = m_antennaPatterns->GetAntennaGainPattern(beamId);
    if (!pattern)
    {
        NS_LOG_WARN("No antenna pattern for beam " << beamId);
        return 0.0;
    }

    double gainLin = pattern->GetAntennaGain_lin(ueGeo, satMobility);
    if (gainLin <= 0.0)
    {
        return -100.0; // Very low gain floor
    }
    return 10.0 * std::log10(gainLin);
}

// ---- Link budget ----

double
OranNtnSatBridge::ComputeNtnRsrp(uint32_t ueId, uint32_t satId, uint32_t beamId) const
{
    NS_LOG_FUNCTION(this << ueId << satId << beamId);

    double slantRange = ComputeSlantRange(ueId, satId);

    // Free-space path loss: FSPL = 20*log10(4*pi*d*f/c)
    double fspl = 20.0 * std::log10(4.0 * M_PI * slantRange * m_carrierFreqHz / m_speedOfLight);

    // Beam gain
    double beamGain = GetBeamGainAtUe(ueId, satId, beamId);

    // RSRP = txPower + beamGain - FSPL
    double rsrp = m_txPower_dBm + beamGain - fspl;

    NS_LOG_DEBUG("RSRP: txPower=" << m_txPower_dBm << " beamGain=" << beamGain
                                  << " FSPL=" << fspl << " RSRP=" << rsrp);

    return rsrp;
}

double
OranNtnSatBridge::ComputeNtnSinr(uint32_t ueId, uint32_t satId, uint32_t beamId) const
{
    NS_LOG_FUNCTION(this << ueId << satId << beamId);

    double rsrp = ComputeNtnRsrp(ueId, satId, beamId);

    // Thermal noise: N = 10*log10(k * T * BW) + NF
    double thermalNoise_W = m_boltzmann * m_temperature_K * m_bandwidth_Hz;
    double noise_dBm = 10.0 * std::log10(thermalNoise_W) + 30.0 + m_noiseFigure_dB;

    // SINR = RSRP - N (simplified, no inter-beam interference)
    double sinr = rsrp - noise_dBm;

    NS_LOG_DEBUG("SINR: RSRP=" << rsrp << " noise=" << noise_dBm << " SINR=" << sinr);

    return sinr;
}

E2KpmReport
OranNtnSatBridge::ComputeFullLinkBudget(uint32_t ueId,
                                          uint32_t satId,
                                          uint32_t beamId) const
{
    NS_LOG_FUNCTION(this << ueId << satId << beamId);

    E2KpmReport report;
    report.timestamp = Simulator::Now().GetSeconds();
    report.gnbId = satId;
    report.isNtn = true;
    report.ueId = ueId;

    report.rsrp_dBm = ComputeNtnRsrp(ueId, satId, beamId);
    report.sinr_dB = ComputeNtnSinr(ueId, satId, beamId);
    report.rsrq_dB = report.sinr_dB - 3.0; // Simplified RSRQ approximation

    // CQI mapping from SINR (simplified)
    if (report.sinr_dB < -6.0)
        report.cqi = 0;
    else if (report.sinr_dB > 20.0)
        report.cqi = 15;
    else
        report.cqi = static_cast<uint8_t>((report.sinr_dB + 6.0) * 15.0 / 26.0);

    report.elevation_deg = ComputeElevationAngle(ueId, satId);
    report.doppler_Hz = ComputeDopplerShift(ueId, satId, m_carrierFreqHz);
    report.propagationDelay_ms = ComputePropagationDelay(ueId, satId).GetMilliSeconds();
    report.beamId = beamId;
    report.beamGain_dB = GetBeamGainAtUe(ueId, satId, beamId);

    // TTE
    report.tte_s = ComputeTte(ueId, satId, beamId);

    // Throughput estimate from SINR (Shannon-based, simplified)
    double spectralEff = std::log2(1.0 + std::pow(10.0, report.sinr_dB / 10.0));
    report.throughput_Mbps = spectralEff * m_bandwidth_Hz / 1e6;
    report.latency_ms = report.propagationDelay_ms * 2.0; // RTT

    // Cell-level (defaults)
    auto satIt = m_satStates.find(satId);
    if (satIt != m_satStates.end())
    {
        auto loadIt = satIt->second.beamLoads.find(beamId);
        report.prbUtilization = (loadIt != satIt->second.beamLoads.end()) ? loadIt->second : 0.0;
        auto ueCountIt = satIt->second.beamActiveUes.find(beamId);
        report.activeUes =
            (ueCountIt != satIt->second.beamActiveUes.end()) ? ueCountIt->second : 0;
    }
    else
    {
        report.prbUtilization = 0.0;
        report.activeUes = 0;
    }
    report.cellThroughput_Mbps = report.throughput_Mbps;

    // Slice defaults
    auto ueIt = m_ueStates.find(ueId);
    report.sliceId = (ueIt != m_ueStates.end()) ? ueIt->second.sliceId : 0;
    report.sliceThroughput_Mbps = report.throughput_Mbps;
    report.sliceLatency_ms = report.latency_ms;
    report.sliceReliability = 0.999;

    return report;
}

// ---- Visibility and coverage ----

std::vector<uint32_t>
OranNtnSatBridge::GetVisibleSatellites(uint32_t ueId, double minElevationDeg) const
{
    NS_LOG_FUNCTION(this << ueId << minElevationDeg);

    std::vector<std::pair<double, uint32_t>> visible;

    for (const auto& entry : m_satStates)
    {
        double elev = ComputeElevationAngle(ueId, entry.first);
        if (elev > minElevationDeg)
        {
            visible.push_back(std::make_pair(elev, entry.first));
        }
    }

    // Sort by elevation descending
    std::sort(visible.begin(),
              visible.end(),
              [](const std::pair<double, uint32_t>& a, const std::pair<double, uint32_t>& b) {
                  return a.first > b.first;
              });

    std::vector<uint32_t> result;
    result.reserve(visible.size());
    for (const auto& v : visible)
    {
        result.push_back(v.second);
    }

    return result;
}

uint32_t
OranNtnSatBridge::FindBestBeam(uint32_t ueId, uint32_t satId) const
{
    NS_LOG_FUNCTION(this << ueId << satId);

    if (!m_antennaPatterns)
    {
        return 1;
    }

    double bestGain = -1e30;
    uint32_t bestBeam = 1;

    for (uint32_t beamId = 1; beamId <= 72; ++beamId)
    {
        double gain = GetBeamGainAtUe(ueId, satId, beamId);
        if (gain > bestGain)
        {
            bestGain = gain;
            bestBeam = beamId;
        }
    }

    return bestBeam;
}

double
OranNtnSatBridge::ComputeTte(uint32_t ueId, uint32_t satId, uint32_t beamId) const
{
    NS_LOG_FUNCTION(this << ueId << satId << beamId);

    auto satIt = m_satStates.find(satId);
    NS_ASSERT_MSG(satIt != m_satStates.end(), "Invalid satId: " << satId);

    if (!satIt->second.mobility || !m_antennaPatterns)
    {
        return 120.0; // Default max TTE if no mobility/patterns
    }

    // Current beam gain as threshold reference
    double currentGain = GetBeamGainAtUe(ueId, satId, beamId);
    double threshold = currentGain - 3.0; // 3 dB drop threshold

    GeoCoordinate ueGeo = GetUePosition(ueId);

    // Forward-propagate satellite position in 1-second steps
    // Check if beam gain drops below threshold
    double tte = 120.0; // Max TTE
    Time now = Simulator::Now();

    for (int step = 1; step <= 120; ++step)
    {
        // Estimate future satellite position
        // Use velocity-based linear extrapolation
        Vector vel = satIt->second.mobility->GetVelocity();
        Vector curPos = satIt->second.mobility->GetPosition();

        double futureX = curPos.x + vel.x * step;
        double futureY = curPos.y + vel.y * step;
        double futureZ = curPos.z + vel.z * step;

        // Convert to GeoCoordinate for gain calculation
        GeoCoordinate futureGeo(Vector(futureX, futureY, futureZ));

        // Compute gain at UE from future satellite position
        // We use the slant range + FSPL approach as proxy since we
        // cannot directly call antenna pattern with a hypothetical mobility
        Vector ueEcef = GeoToEcef(ueGeo);
        double dx = futureX - ueEcef.x;
        double dy = futureY - ueEcef.y;
        double dz = futureZ - ueEcef.z;
        double futureRange = std::sqrt(dx * dx + dy * dy + dz * dz);

        // Current range
        double currentRange = ComputeSlantRange(ueId, satId);

        // Approximate gain change from range change
        double rangeRatio = currentRange / std::max(futureRange, 1.0);
        double futureGain = currentGain + 20.0 * std::log10(rangeRatio);

        // Also check elevation
        double futureDist =
            std::sqrt(ueEcef.x * ueEcef.x + ueEcef.y * ueEcef.y + ueEcef.z * ueEcef.z);
        Vector upDir(ueEcef.x / futureDist, ueEcef.y / futureDist, ueEcef.z / futureDist);

        Vector losDir(dx / futureRange, dy / futureRange, dz / futureRange);
        double dot = losDir.x * upDir.x + losDir.y * upDir.y + losDir.z * upDir.z;
        double futureElev = std::asin(std::max(-1.0, std::min(1.0, dot))) * RAD_TO_DEG;

        if (futureGain < threshold || futureElev < 10.0)
        {
            tte = static_cast<double>(step);
            break;
        }
    }

    return tte;
}

// ---- Feeder link management ----

void
OranNtnSatBridge::UpdateFeederLinkStatus(NodeContainer gatewayNodes)
{
    NS_LOG_FUNCTION(this << gatewayNodes.GetN());

    for (auto& satEntry : m_satStates)
    {
        bool anyVisible = false;
        uint32_t bestGw = 0;
        double bestElev = -90.0;

        GeoCoordinate satGeo = GetSatellitePosition(satEntry.first);
        Vector satEcef = GeoToEcef(satGeo);

        for (uint32_t g = 0; g < gatewayNodes.GetN(); ++g)
        {
            Ptr<SatMobilityModel> gwMob = gatewayNodes.Get(g)->GetObject<SatMobilityModel>();
            if (!gwMob)
            {
                continue;
            }

            GeoCoordinate gwGeo = gwMob->GetGeoPosition();
            Vector gwEcef = GeoToEcef(gwGeo);

            // Compute elevation from gateway to satellite
            Vector los(satEcef.x - gwEcef.x, satEcef.y - gwEcef.y, satEcef.z - gwEcef.z);
            double dist = std::sqrt(los.x * los.x + los.y * los.y + los.z * los.z);
            if (dist < 1.0)
            {
                continue;
            }
            los.x /= dist;
            los.y /= dist;
            los.z /= dist;

            double gwDist =
                std::sqrt(gwEcef.x * gwEcef.x + gwEcef.y * gwEcef.y + gwEcef.z * gwEcef.z);
            Vector upDir(gwEcef.x / gwDist, gwEcef.y / gwDist, gwEcef.z / gwDist);

            double dot = los.x * upDir.x + los.y * upDir.y + los.z * upDir.z;
            double elev = std::asin(std::max(-1.0, std::min(1.0, dot))) * RAD_TO_DEG;

            if (elev > 5.0)
            {
                anyVisible = true;
                if (elev > bestElev)
                {
                    bestElev = elev;
                    bestGw = g;
                }
            }
        }

        bool previousStatus = satEntry.second.feederLinkAvailable;
        satEntry.second.feederLinkAvailable = anyVisible;
        satEntry.second.servingGatewayId = bestGw;

        // Update E2 node
        if (satEntry.second.e2Node)
        {
            satEntry.second.e2Node->SetFeederLinkAvailable(anyVisible);
        }

        // Update Space RIC
        if (satEntry.second.spaceRic)
        {
            if (anyVisible && !previousStatus)
            {
                satEntry.second.spaceRic->ExitAutonomousMode();
            }
            else if (!anyVisible && previousStatus)
            {
                satEntry.second.spaceRic->EnterAutonomousMode();
            }
        }

        if (previousStatus != anyVisible)
        {
            m_feederLinkChanged(satEntry.first, anyVisible);
            NS_LOG_INFO("Satellite " << satEntry.first << " feeder link "
                                      << (anyVisible ? "AVAILABLE" : "UNAVAILABLE")
                                      << " via gateway " << bestGw);
        }
    }
}

// ---- Periodic KPM generation ----

void
OranNtnSatBridge::StartRealisticKpmFeed(Time interval)
{
    NS_LOG_FUNCTION(this << interval);
    m_kpmInterval = interval;
    m_kpmFeedActive = true;
    m_kpmFeedEvent = Simulator::Schedule(m_kpmInterval,
                                          &OranNtnSatBridge::PeriodicKpmCallback,
                                          this);
}

void
OranNtnSatBridge::StopKpmFeed()
{
    NS_LOG_FUNCTION(this);
    m_kpmFeedActive = false;
    if (m_kpmFeedEvent.IsPending())
    {
        Simulator::Cancel(m_kpmFeedEvent);
    }
}

void
OranNtnSatBridge::PeriodicKpmCallback()
{
    NS_LOG_FUNCTION(this);

    if (!m_kpmFeedActive)
    {
        return;
    }

    UpdateSatelliteStates();
    UpdateUeStates();

    for (auto& ueEntry : m_ueStates)
    {
        uint32_t ueId = ueEntry.first;

        // Find serving satellite (highest elevation)
        std::vector<uint32_t> visible = GetVisibleSatellites(ueId, 10.0);
        if (visible.empty())
        {
            continue;
        }

        uint32_t servingSatId = visible[0]; // Highest elevation
        uint32_t bestBeam = FindBestBeam(ueId, servingSatId);

        // Compute full link budget
        E2KpmReport report = ComputeFullLinkBudget(ueId, servingSatId, bestBeam);

        // Update UE state
        ueEntry.second.servingSatId = servingSatId;
        ueEntry.second.servingBeamId = bestBeam;
        ueEntry.second.servingSinr_dB = report.sinr_dB;
        ueEntry.second.servingRsrp_dBm = report.rsrp_dBm;
        ueEntry.second.servingTte_s = report.tte_s;
        ueEntry.second.servingElevation_deg = report.elevation_deg;
        ueEntry.second.servingDoppler_Hz = report.doppler_Hz;
        ueEntry.second.servingDelay_ms = report.propagationDelay_ms;

        // Submit to E2 node
        auto satIt = m_satStates.find(servingSatId);
        if (satIt != m_satStates.end() && satIt->second.e2Node)
        {
            satIt->second.e2Node->SubmitKpmMeasurement(report);
        }

        m_kpmGenerated(ueId, report);
    }

    // Schedule next callback
    m_kpmFeedEvent = Simulator::Schedule(m_kpmInterval,
                                          &OranNtnSatBridge::PeriodicKpmCallback,
                                          this);
}

// ---- NTN scenario configuration ----

void
OranNtnSatBridge::SetNtnScenario(const std::string& scenario)
{
    NS_LOG_FUNCTION(this << scenario);
    m_ntnScenario = scenario;

    if (scenario == "DenseUrban")
    {
        m_ntnPathloss = CreateObject<ThreeGppNTNDenseUrbanPropagationLossModel>();
    }
    else if (scenario == "Urban")
    {
        m_ntnPathloss = CreateObject<ThreeGppNTNUrbanPropagationLossModel>();
    }
    else if (scenario == "Suburban")
    {
        m_ntnPathloss = CreateObject<ThreeGppNTNSuburbanPropagationLossModel>();
    }
    else if (scenario == "Rural")
    {
        m_ntnPathloss = CreateObject<ThreeGppNTNRuralPropagationLossModel>();
    }
    else
    {
        NS_LOG_WARN("Unknown NTN scenario: " << scenario << ", using Urban");
        m_ntnPathloss = CreateObject<ThreeGppNTNUrbanPropagationLossModel>();
    }

    if (m_ntnPathloss)
    {
        m_ntnPathloss->SetAttribute("Frequency", DoubleValue(m_carrierFreqHz));
    }
}

void
OranNtnSatBridge::SetCarrierFrequency(double freqHz)
{
    NS_LOG_FUNCTION(this << freqHz);
    m_carrierFreqHz = freqHz;
    if (m_ntnPathloss)
    {
        m_ntnPathloss->SetAttribute("Frequency", DoubleValue(freqHz));
    }
}

void
OranNtnSatBridge::SetSatelliteTxPower(double txPower_dBm)
{
    NS_LOG_FUNCTION(this << txPower_dBm);
    m_txPower_dBm = txPower_dBm;
}

void
OranNtnSatBridge::SetBandwidth(double bwHz)
{
    NS_LOG_FUNCTION(this << bwHz);
    m_bandwidth_Hz = bwHz;
}

// ---- State access ----

const SatelliteBridgeState&
OranNtnSatBridge::GetSatState(uint32_t satId) const
{
    auto it = m_satStates.find(satId);
    NS_ASSERT_MSG(it != m_satStates.end(), "Invalid satId: " << satId);
    return it->second;
}

const UeBridgeState&
OranNtnSatBridge::GetUeState(uint32_t ueId) const
{
    auto it = m_ueStates.find(ueId);
    NS_ASSERT_MSG(it != m_ueStates.end(), "Invalid ueId: " << ueId);
    return it->second;
}

uint32_t
OranNtnSatBridge::GetNumSatellites() const
{
    return static_cast<uint32_t>(m_satStates.size());
}

uint32_t
OranNtnSatBridge::GetNumUes() const
{
    return static_cast<uint32_t>(m_ueStates.size());
}

// ---- Private helpers ----

void
OranNtnSatBridge::UpdateSatelliteStates()
{
    for (auto& entry : m_satStates)
    {
        if (entry.second.mobility)
        {
            entry.second.lastPosition = entry.second.mobility->GetGeoPosition();
            entry.second.lastVelocity = entry.second.mobility->GetVelocity();
            entry.second.altitude_km = entry.second.lastPosition.GetAltitude() / 1000.0;

            double vx = entry.second.lastVelocity.x;
            double vy = entry.second.lastVelocity.y;
            entry.second.groundSpeed_kmps = std::sqrt(vx * vx + vy * vy) / 1000.0;
        }
    }
}

void
OranNtnSatBridge::UpdateUeStates()
{
    for (auto& entry : m_ueStates)
    {
        if (entry.second.ueMobility)
        {
            entry.second.position = entry.second.ueMobility->GetGeoPosition();
            entry.second.velocity = entry.second.ueMobility->GetVelocity();
        }
    }
}

// ============================================================================
//  Phase 1: Deep Satellite Integration Methods
// ============================================================================

// DVB-S2X ModCod SINR thresholds (SINR_dB -> ModCod index)
// Based on DVB-S2X Table 13 (normal frames, pilot on)
const std::vector<std::pair<double, uint8_t>> OranNtnSatBridge::s_modcodThresholds = {
    {-2.35, 0},   // QPSK 1/4
    {-1.24, 1},   // QPSK 1/3
    {-0.30, 2},   // QPSK 2/5
    {1.00, 3},    // QPSK 1/2
    {2.23, 4},    // QPSK 3/5
    {3.10, 5},    // QPSK 2/3
    {4.03, 6},    // QPSK 3/4
    {4.68, 7},    // QPSK 4/5
    {5.18, 8},    // QPSK 5/6
    {5.50, 9},    // QPSK 8/9
    {5.97, 10},   // QPSK 9/10
    {6.55, 11},   // 8PSK 3/5
    {7.91, 12},   // 8PSK 2/3
    {9.35, 13},   // 8PSK 3/4
    {10.69, 14},  // 8PSK 5/6
    {11.10, 15},  // 8PSK 8/9
    {11.56, 16},  // 8PSK 9/10
    {11.84, 17},  // 16APSK 2/3
    {12.89, 18},  // 16APSK 3/4
    {13.64, 19},  // 16APSK 4/5
    {14.28, 20},  // 16APSK 5/6
    {15.69, 21},  // 16APSK 8/9
    {16.05, 22},  // 16APSK 9/10
    {16.98, 23},  // 32APSK 3/4
    {17.73, 24},  // 32APSK 4/5
    {18.53, 25},  // 32APSK 5/6
    {20.10, 26},  // 32APSK 8/9
    {20.48, 27},  // 32APSK 9/10
};

double
OranNtnSatBridge::ComputeNtnSinrWithFading(uint32_t ueId,
                                             uint32_t satId,
                                             uint32_t beamId) const
{
    NS_LOG_FUNCTION(this << ueId << satId << beamId);

    // Start with base SINR (FSPL + antenna gain)
    double baseSinr = ComputeNtnSinr(ueId, satId, beamId);

    // Apply Markov fading model
    double elevationDeg = ComputeElevationAngle(ueId, satId);

    // Determine fading state transition probabilities based on elevation
    // High elevation: mostly clear-sky, Low elevation: more blockage
    double pClear, pShadow;
    if (elevationDeg > 60.0)
    {
        pClear = 0.90;
        pShadow = 0.08;
    }
    else if (elevationDeg > 30.0)
    {
        pClear = 0.70;
        pShadow = 0.20;
    }
    else
    {
        pClear = 0.40;
        pShadow = 0.35;
    }

    // Update Markov state
    uint8_t currentState = 0;
    auto stateIt = m_markovFadingStates.find(satId * 10000 + ueId);
    if (stateIt != m_markovFadingStates.end())
    {
        currentState = stateIt->second;
    }

    // Simple state transition based on probabilities
    double r = static_cast<double>(std::rand()) / RAND_MAX;
    uint8_t newState;
    if (currentState == 0) // Clear
    {
        if (r < pClear)
            newState = 0;
        else if (r < pClear + pShadow * 0.3)
            newState = 1;
        else
            newState = 2;
    }
    else if (currentState == 1) // Shadow
    {
        if (r < 0.3)
            newState = 0;
        else if (r < 0.3 + 0.5)
            newState = 1;
        else
            newState = 2;
    }
    else // Blocked
    {
        if (r < 0.15)
            newState = 0;
        else if (r < 0.15 + 0.35)
            newState = 1;
        else
            newState = 2;
    }
    m_markovFadingStates[satId * 10000 + ueId] = newState;

    // Apply state-dependent fading
    double fadingGain_dB = 0.0;
    switch (newState)
    {
    case 0: // Clear sky - Rician fading with high K-factor
    {
        double kFactor_dB = 10.0 + 0.2 * (elevationDeg - 30.0);
        kFactor_dB = std::max(2.0, std::min(20.0, kFactor_dB));
        // Approximate Rician fade: small fluctuation around 0 dB
        fadingGain_dB = (static_cast<double>(std::rand()) / RAND_MAX - 0.5) *
                        2.0 / (1.0 + std::pow(10.0, kFactor_dB / 10.0));
        break;
    }
    case 1: // Shadowed - lognormal shadowing
    {
        double shadowStd = 3.0 + (90.0 - elevationDeg) * 0.1; // 3-12 dB std
        fadingGain_dB = -std::abs((static_cast<double>(std::rand()) / RAND_MAX - 0.5) *
                                   2.0 * shadowStd);
        break;
    }
    case 2: // Blocked - deep fade
    {
        fadingGain_dB = -15.0 - (static_cast<double>(std::rand()) / RAND_MAX) * 20.0;
        break;
    }
    }

    // Add inter-beam interference
    double interference_dBm = ComputeInterBeamInterference(ueId, satId, beamId);

    // Convert interference to SINR degradation
    double rsrp = ComputeNtnRsrp(ueId, satId, beamId);
    double thermalNoise_W = m_boltzmann * m_temperature_K * m_bandwidth_Hz;
    double noise_dBm = 10.0 * std::log10(thermalNoise_W) + 30.0 + m_noiseFigure_dB;
    double totalInterferenceAndNoise_lin =
        std::pow(10.0, noise_dBm / 10.0) + std::pow(10.0, interference_dBm / 10.0);
    double totalIN_dBm = 10.0 * std::log10(totalInterferenceAndNoise_lin);

    double sinrWithFading = rsrp + fadingGain_dB - totalIN_dBm;

    NS_LOG_DEBUG("SINR with fading: base=" << baseSinr << " fading=" << fadingGain_dB
                  << " state=" << (int)newState << " interference=" << interference_dBm
                  << " final=" << sinrWithFading);

    return sinrWithFading;
}

uint8_t
OranNtnSatBridge::SelectModCod(uint32_t ueId, uint32_t satId) const
{
    NS_LOG_FUNCTION(this << ueId << satId);

    uint32_t beamId = FindBestBeam(ueId, satId);
    double sinr = ComputeNtnSinrWithFading(ueId, satId, beamId);

    // Select highest ModCod whose SINR threshold is met (with 1 dB margin)
    uint8_t selectedModCod = 0;
    for (const auto& [threshold, modcod] : s_modcodThresholds)
    {
        if (sinr >= threshold + 1.0) // 1 dB implementation margin
        {
            selectedModCod = modcod;
        }
        else
        {
            break;
        }
    }

    NS_LOG_DEBUG("ModCod selection: SINR=" << sinr << " -> ModCod=" << (int)selectedModCod);
    return selectedModCod;
}

double
OranNtnSatBridge::ComputeInterBeamInterference(uint32_t ueId,
                                                 uint32_t satId,
                                                 uint32_t beamId) const
{
    NS_LOG_FUNCTION(this << ueId << satId << beamId);

    if (!m_antennaPatterns)
    {
        return -200.0; // No interference if no antenna patterns
    }

    double totalInterference_lin = 0.0;
    GeoCoordinate ueGeo = GetUePosition(ueId);
    auto satIt = m_satStates.find(satId);
    if (satIt == m_satStates.end() || !satIt->second.mobility)
    {
        return -200.0;
    }

    // Sum interference from all other active beams on the same satellite
    for (uint32_t otherBeamId = 1; otherBeamId <= 72; ++otherBeamId)
    {
        if (otherBeamId == beamId)
        {
            continue;
        }

        Ptr<SatAntennaGainPattern> pattern =
            m_antennaPatterns->GetAntennaGainPattern(otherBeamId);
        if (!pattern)
        {
            continue;
        }

        double gainLin = pattern->GetAntennaGain_lin(ueGeo, satIt->second.mobility);
        if (gainLin <= 1e-10)
        {
            continue;
        }

        // Check if this beam is active (has traffic)
        auto loadIt = satIt->second.beamLoads.find(otherBeamId);
        double beamLoad = (loadIt != satIt->second.beamLoads.end()) ? loadIt->second : 0.0;
        if (beamLoad < 0.01)
        {
            continue;
        }

        // Interference from this beam = TX power * sidelobe gain - FSPL
        double slantRange = ComputeSlantRange(ueId, satId);
        double fspl = 20.0 * std::log10(4.0 * M_PI * slantRange * m_carrierFreqHz / m_speedOfLight);
        double gain_dB = 10.0 * std::log10(gainLin);
        double interference_dBm = m_txPower_dBm + gain_dB - fspl;
        totalInterference_lin += std::pow(10.0, interference_dBm / 10.0) * beamLoad;
    }

    // Also consider interference from neighboring satellites
    for (const auto& otherSat : m_satStates)
    {
        if (otherSat.first == satId)
        {
            continue;
        }

        double otherElev = ComputeElevationAngle(ueId, otherSat.first);
        if (otherElev < 10.0)
        {
            continue; // Not visible
        }

        // Simplified inter-satellite interference (main beam only)
        double otherRange = ComputeSlantRange(ueId, otherSat.first);
        double otherFspl = 20.0 * std::log10(4.0 * M_PI * otherRange * m_carrierFreqHz / m_speedOfLight);

        // Assume worst-case sidelobe: -20 dB below main beam
        double interSatGain = -20.0;
        double interSatInterference_dBm = m_txPower_dBm + interSatGain - otherFspl;

        // Scale by frequency reuse factor (assuming 4-color reuse)
        if (otherSat.first % 4 == satId % 4) // Same frequency
        {
            totalInterference_lin += std::pow(10.0, interSatInterference_dBm / 10.0) * 0.5;
        }
    }

    if (totalInterference_lin < 1e-20)
    {
        return -200.0;
    }
    return 10.0 * std::log10(totalInterference_lin);
}

void
OranNtnSatBridge::SetupIslTopology(NodeContainer satNodes, double islDataRate_Mbps)
{
    NS_LOG_FUNCTION(this << satNodes.GetN() << islDataRate_Mbps);

    m_islLinks.clear();

    for (uint32_t i = 0; i < m_numPlanes; ++i)
    {
        for (uint32_t j = 0; j < m_satsPerPlane; ++j)
        {
            uint32_t satId = i * m_satsPerPlane + j;

            // Intra-plane links: connect to next satellite in same plane
            uint32_t nextInPlane = i * m_satsPerPlane + ((j + 1) % m_satsPerPlane);

            IslLinkState intraLink;
            intraLink.satId1 = satId;
            intraLink.satId2 = nextInPlane;
            intraLink.dataRate_Mbps = islDataRate_Mbps;
            intraLink.utilization = 0.0;
            intraLink.active = true;

            // Compute distance between satellites
            GeoCoordinate pos1 = GetSatellitePosition(satId);
            GeoCoordinate pos2 = GetSatellitePosition(nextInPlane);
            Vector ecef1 = GeoToEcef(pos1);
            Vector ecef2 = GeoToEcef(pos2);
            double dx = ecef2.x - ecef1.x;
            double dy = ecef2.y - ecef1.y;
            double dz = ecef2.z - ecef1.z;
            intraLink.distance_km = std::sqrt(dx * dx + dy * dy + dz * dz) / 1000.0;
            intraLink.delay_ms = intraLink.distance_km / (SPEED_OF_LIGHT / 1000.0) * 1000.0;

            auto key = std::make_pair(std::min(satId, nextInPlane),
                                       std::max(satId, nextInPlane));
            m_islLinks[key] = intraLink;

            // Inter-plane links: connect to same position in adjacent plane
            if (m_numPlanes > 1)
            {
                uint32_t adjacentPlane = ((i + 1) % m_numPlanes) * m_satsPerPlane + j;

                IslLinkState interLink;
                interLink.satId1 = satId;
                interLink.satId2 = adjacentPlane;
                interLink.dataRate_Mbps = islDataRate_Mbps;
                interLink.utilization = 0.0;
                interLink.active = true;

                GeoCoordinate pos3 = GetSatellitePosition(adjacentPlane);
                Vector ecef3 = GeoToEcef(pos3);
                double dx2 = ecef3.x - ecef1.x;
                double dy2 = ecef3.y - ecef1.y;
                double dz2 = ecef3.z - ecef1.z;
                interLink.distance_km = std::sqrt(dx2 * dx2 + dy2 * dy2 + dz2 * dz2) / 1000.0;
                interLink.delay_ms = interLink.distance_km / (SPEED_OF_LIGHT / 1000.0) * 1000.0;

                auto interKey = std::make_pair(std::min(satId, adjacentPlane),
                                                std::max(satId, adjacentPlane));
                m_islLinks[interKey] = interLink;
            }
        }
    }

    NS_LOG_INFO("ISL topology: " << m_islLinks.size() << " links created for "
                << satNodes.GetN() << " satellites");
}

IslLinkState
OranNtnSatBridge::GetIslLinkState(uint32_t satId1, uint32_t satId2) const
{
    auto key = std::make_pair(std::min(satId1, satId2), std::max(satId1, satId2));
    auto it = m_islLinks.find(key);
    if (it != m_islLinks.end())
    {
        return it->second;
    }
    IslLinkState empty;
    empty.satId1 = satId1;
    empty.satId2 = satId2;
    empty.active = false;
    empty.distance_km = 0;
    empty.delay_ms = 0;
    empty.dataRate_Mbps = 0;
    empty.utilization = 0;
    return empty;
}

std::vector<uint32_t>
OranNtnSatBridge::GetIslNeighbors(uint32_t satId) const
{
    std::vector<uint32_t> neighbors;
    for (const auto& [key, link] : m_islLinks)
    {
        if (!link.active)
        {
            continue;
        }
        if (key.first == satId)
        {
            neighbors.push_back(key.second);
        }
        else if (key.second == satId)
        {
            neighbors.push_back(key.first);
        }
    }
    return neighbors;
}

double
OranNtnSatBridge::ComputeCno(uint32_t ueId, uint32_t satId, uint32_t beamId) const
{
    NS_LOG_FUNCTION(this << ueId << satId << beamId);

    double rsrp_dBm = ComputeNtnRsrp(ueId, satId, beamId);
    double rsrp_dBW = rsrp_dBm - 30.0;

    // N0 = k*T in dBW/Hz
    double n0_dBW_Hz = 10.0 * std::log10(m_boltzmann * m_temperature_K);

    // C/N0 = RSRP (dBW) - N0 (dBW/Hz)
    double cno_dBHz = rsrp_dBW - n0_dBW_Hz;

    return cno_dBHz;
}

uint8_t
OranNtnSatBridge::GetMarkovState(uint32_t satId) const
{
    // Return average Markov state across UEs for this satellite
    uint32_t totalClear = 0, totalShadow = 0, totalBlocked = 0;
    for (const auto& [key, state] : m_markovFadingStates)
    {
        if (key / 10000 == satId)
        {
            switch (state)
            {
            case 0: totalClear++; break;
            case 1: totalShadow++; break;
            case 2: totalBlocked++; break;
            }
        }
    }
    if (totalBlocked > totalClear && totalBlocked > totalShadow)
        return 2;
    if (totalShadow > totalClear)
        return 1;
    return 0;
}

} // namespace ns3
