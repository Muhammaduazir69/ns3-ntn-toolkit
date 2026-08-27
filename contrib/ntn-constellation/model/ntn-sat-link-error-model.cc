/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-sat-link-error-model.h"

#include "ns3/double.h"
#include "ns3/thz-ntn-scintillation.h"
#include "ns3/thz-ntn-itu-recommendations.h"
#include "ns3/boolean.h"
#include "ns3/satellite-enums.h"
#include "ns3/satellite-link-results.h"
#include "ns3/uinteger.h"
#include "ns3/log.h"

#include <cmath>

namespace ns3
{
namespace ntncon
{

NS_LOG_COMPONENT_DEFINE("NtnSatLinkErrorModel");
NS_OBJECT_ENSURE_REGISTERED(NtnSatLinkErrorModel);

namespace
{
constexpr double kC = 299792458.0;
constexpr double kBoltzDbwHzK = -228.6; // 10*log10(k), k = 1.380649e-23 J/K
} // namespace

TypeId
NtnSatLinkErrorModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ntncon::NtnSatLinkErrorModel")
            .SetParent<ErrorModel>()
            .SetGroupName("NtnConstellation")
            .AddConstructor<NtnSatLinkErrorModel>()
            .AddAttribute("EirpDbw", "Transmit EIRP (dBW).", DoubleValue(20.0),
                          MakeDoubleAccessor(&NtnSatLinkErrorModel::m_eirpDbw),
                          MakeDoubleChecker<double>())
            .AddAttribute("RxGtDbK", "Receiver figure of merit G/T (dB/K).", DoubleValue(1.0),
                          MakeDoubleAccessor(&NtnSatLinkErrorModel::m_rxGtDbK),
                          MakeDoubleChecker<double>())
            .AddAttribute("BandwidthHz", "Noise bandwidth (Hz).", DoubleValue(30e6),
                          MakeDoubleAccessor(&NtnSatLinkErrorModel::m_bandwidthHz),
                          MakeDoubleChecker<double>(1.0))
            .AddAttribute("CarrierHz", "Carrier frequency (Hz).", DoubleValue(20e9),
                          MakeDoubleAccessor(&NtnSatLinkErrorModel::m_carrierHz),
                          MakeDoubleChecker<double>(1.0))
            .AddAttribute("ThresholdEsNoDb",
                          "DVB-S2 MODCOD Es/No quasi-error-free threshold (dB).",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&NtnSatLinkErrorModel::m_thresholdEsNoDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("WaterfallDb", "BLER waterfall steepness (dB).", DoubleValue(1.0),
                          MakeDoubleAccessor(&NtnSatLinkErrorModel::m_waterfallDb),
                          MakeDoubleChecker<double>(0.05))
            // ---- CON-2: atmospheric excess loss ----
            .AddAttribute("EnableGaseous",
                          "Apply ITU-R P.676-13 gaseous absorption to the budget.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NtnSatLinkErrorModel::m_enableGaseous),
                          MakeBooleanChecker())
            .AddAttribute("EnableRain",
                          "Apply ITU-R P.618-13 slant-path rain attenuation. Needs RainRateMmH.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NtnSatLinkErrorModel::m_enableRain),
                          MakeBooleanChecker())
            .AddAttribute("EnableScintillation",
                          "Apply ITU-R P.618-13 tropospheric scintillation fade depth.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NtnSatLinkErrorModel::m_enableScintillation),
                          MakeBooleanChecker())
            .AddAttribute("RainRateMmH",
                          "Point rainfall rate exceeded 0.01 percent of an average year (mm/h). "
                          "0 means clear sky and disables the rain term. There is no sensible "
                          "default here: R_0.01 is a property of the site, so a scenario that "
                          "wants rain has to say where it is.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NtnSatLinkErrorModel::m_rainRateMmH),
                          MakeDoubleChecker<double>(0.0, 500.0))
            .AddAttribute("GroundAltKm", "Ground-station altitude (km MSL).", DoubleValue(0.05),
                          MakeDoubleAccessor(&NtnSatLinkErrorModel::m_groundAltKm),
                          MakeDoubleChecker<double>(0.0, 12.0))
            .AddAttribute("StationLatitudeDeg",
                          "Ground-station latitude (deg), used by the P.618 adjustment factors.",
                          DoubleValue(45.0),
                          MakeDoubleAccessor(&NtnSatLinkErrorModel::m_stationLatDeg),
                          MakeDoubleChecker<double>(-90.0, 90.0))
            .AddAttribute("AntennaDiameterM",
                          "Receive aperture diameter (m), for the scintillation "
                          "aperture-averaging factor.",
                          DoubleValue(0.6),
                          MakeDoubleAccessor(&NtnSatLinkErrorModel::m_antennaDiameterM),
                          MakeDoubleChecker<double>(0.01, 100.0))
            // ---- CON-2: MODCOD curve ----
            .AddAttribute("UseDvbS2LinkResults",
                          "Take BLER from the vendored SNS3 DVB-S2 forward-link look-up tables "
                          "instead of the generic erfc waterfall. Off by default because "
                          "switching an existing scenario onto a different BLER curve changes "
                          "its results.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NtnSatLinkErrorModel::m_useDvbS2LinkResults),
                          MakeBooleanChecker())
            .AddAttribute("Modcod",
                          "MODCOD index into SatEnums::SatModcod_t, used when "
                          "UseDvbS2LinkResults is set. Default 2 = QPSK 1/2.",
                          UintegerValue(2),
                          MakeUintegerAccessor(&NtnSatLinkErrorModel::m_modcod),
                          MakeUintegerChecker<uint32_t>(1, 40));
    return tid;
}

NtnSatLinkErrorModel::NtnSatLinkErrorModel()
{
    m_rng = CreateObject<UniformRandomVariable>();
}

void
NtnSatLinkErrorModel::SetEndpoints(Ptr<MobilityModel> tx, Ptr<MobilityModel> rx)
{
    m_tx = tx;
    m_rx = rx;
}

double
NtnSatLinkErrorModel::SlantRangeM() const
{
    if (!m_tx || !m_rx)
    {
        return -1.0;
    }
    const Vector a = m_tx->GetPosition();
    const Vector b = m_rx->GetPosition();
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

double
NtnSatLinkErrorModel::ElevationDegFor(const Vector& lower, const Vector& upper) const
{
    // CON-2. Both endpoints are ECEF, so the local zenith at the lower one is
    // its own position vector. The elevation of the upper endpoint is then
    // 90 degrees minus the angle between that zenith and the line of sight.
    const double lr = lower.GetLength();
    if (lr < 1.0)
    {
        return 90.0; // degenerate; treat as no atmospheric path
    }
    const Vector los(upper.x - lower.x, upper.y - lower.y, upper.z - lower.z);
    const double lsLen = los.GetLength();
    if (lsLen < 1.0)
    {
        return 90.0;
    }
    const double cosZenith =
        (lower.x * los.x + lower.y * los.y + lower.z * los.z) / (lr * lsLen);
    const double clamped = std::max(-1.0, std::min(1.0, cosZenith));
    return 90.0 - std::acos(clamped) * 180.0 / M_PI;
}

double
NtnSatLinkErrorModel::ExcessLossDbFor(double elevationDeg) const
{
    m_lastGaseousDb = 0.0;
    m_lastRainDb = 0.0;
    m_lastScintDb = 0.0;

    // An inter-satellite link, or anything looking up from above the
    // troposphere, crosses no weather. Guard on elevation rather than on an
    // explicit link-type flag so the geometry decides.
    if (elevationDeg <= 0.0)
    {
        return 0.0;
    }
    // The ITU slant-path models are not defined below a few degrees, and the
    // toolkit's own scenarios floor service links at 5 to 10 degrees. Clamp
    // rather than extrapolate into a regime the recommendations exclude.
    const double elev = std::max(elevationDeg, 3.0);

    if (m_enableGaseous)
    {
        if (!m_gas)
        {
            m_gas = CreateObject<itu::Itu676AbsorptionModel>();
        }
        m_lastGaseousDb = DynamicCast<itu::Itu676AbsorptionModel>(m_gas)
                              ->SlantPathAttenuationDb(m_carrierHz, elev, m_groundAltKm);
    }
    if (m_enableRain && m_rainRateMmH > 0.0)
    {
        if (!m_rain)
        {
            Ptr<itu::Itu618LossModel> r = CreateObject<itu::Itu618LossModel>();
            r->SetStationLatitudeDeg(m_stationLatDeg);
            m_rain = r;
        }
        m_lastRainDb = DynamicCast<itu::Itu618LossModel>(m_rain)->SlantPathRainAttenuationDb(
            m_carrierHz, elev, m_rainRateMmH, m_groundAltKm, itu::Polarization::circular);
    }
    if (m_enableScintillation)
    {
        if (!m_scint)
        {
            Ptr<ThzNtnScintillation> sc = CreateObject<ThzNtnScintillation>();
            // The aperture-averaging factor of P.618-13 section 2.4.1 needs the
            // receive aperture; without it a large dish is charged the fade of
            // a point receiver.
            sc->SetAntennaDiameter(m_antennaDiameterM);
            m_scint = sc;
        }
        // Quoted at the same 0.01 percent exceedance the rain term uses, so the
        // two fades describe the same availability rather than being summed
        // across different ones.
        m_lastScintDb = DynamicCast<ThzNtnScintillation>(m_scint)
                            ->ComputeScintillationFadeDepth_dB(m_carrierHz, elev, 0.01);
    }
    return m_lastGaseousDb + m_lastRainDb + m_lastScintDb;
}

double
NtnSatLinkErrorModel::EsNoDbFor(double slantM) const
{
    if (slantM <= 0.0)
    {
        return -1e9;
    }
    // Free-space path loss (dB).
    const double fspl = 20.0 * std::log10(4.0 * M_PI * slantM * m_carrierHz / kC);
    // CON-2: the atmosphere the budget used to ignore entirely.
    const double excess = ExcessLossDbFor(CurrentElevationDeg());
    // Es/No = EIRP - FSPL - excess + G/T - k(dB) - 10log10(B).
    return m_eirpDbw - fspl - excess + m_rxGtDbK - kBoltzDbwHzK -
           10.0 * std::log10(m_bandwidthHz);
}

double
NtnSatLinkErrorModel::CurrentElevationDeg() const
{
    if (!m_tx || !m_rx)
    {
        return 90.0;
    }
    const Vector a = m_tx->GetPosition();
    const Vector b = m_rx->GetPosition();
    // The lower endpoint is the ground station; the atmosphere is above it.
    return (a.GetLength() <= b.GetLength()) ? ElevationDegFor(a, b) : ElevationDegFor(b, a);
}

double
NtnSatLinkErrorModel::CurrentExcessLossDb() const
{
    return ExcessLossDbFor(CurrentElevationDeg());
}

double
NtnSatLinkErrorModel::BlerFor(double esNoDb) const
{
    // CON-2: the real DVB-S2 curve when the scenario asks for it. The vendored
    // SNS3 module ships the forward-link look-up tables measured per MODCOD, so
    // this is a published curve rather than a sigmoid with two tunable numbers.
    if (m_useDvbS2LinkResults)
    {
        if (!m_linkResults)
        {
            Ptr<SatLinkResultsDvbS2> lr = CreateObject<SatLinkResultsDvbS2>();
            lr->Initialize();
            m_linkResults = lr;
        }
        return DynamicCast<SatLinkResultsDvbS2>(m_linkResults)
            ->GetBler(static_cast<SatEnums::SatModcod_t>(m_modcod),
                      SatEnums::NORMAL_FRAME,
                      esNoDb);
    }
    // Fallback: a generic erfc waterfall around the configured QEF threshold.
    // This is an ABSTRACTION, not DVB-S2 - it has the right shape and no
    // MODCOD behind it. The comment here used to call it "DVB-S2-style", which
    // read as a standards claim it could not support.
    const double x = (esNoDb - m_thresholdEsNoDb) / m_waterfallDb;
    return 0.5 * std::erfc(x);
}

double
NtnSatLinkErrorModel::CurrentSlantRangeM() const
{
    return SlantRangeM();
}

double
NtnSatLinkErrorModel::CurrentEsNoDb() const
{
    return EsNoDbFor(SlantRangeM());
}

double
NtnSatLinkErrorModel::CurrentBler() const
{
    return BlerFor(EsNoDbFor(SlantRangeM()));
}

bool
NtnSatLinkErrorModel::DoCorrupt(Ptr<Packet> p)
{
    if (!IsEnabled())
    {
        return false;
    }
    const double slant = SlantRangeM();
    if (slant < 0.0)
    {
        return false; // no geometry configured -> pass
    }
    m_lastEsNoDb = EsNoDbFor(slant);
    m_lastBler = BlerFor(m_lastEsNoDb);
    return m_rng->GetValue(0.0, 1.0) < m_lastBler;
}

void
NtnSatLinkErrorModel::DoReset()
{
    m_lastEsNoDb = 0.0;
    m_lastBler = 0.0;
}

} // namespace ntncon
} // namespace ns3
