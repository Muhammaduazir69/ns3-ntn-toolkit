/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-sat-link-error-model.h"

#include "ns3/double.h"
#include "ns3/geographic-positions.h"
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
            .AddAttribute("GroundAltKm",
                          "Altitude (km MSL) of the lower endpoint (ground station, aircraft, "
                          "HAPS). Wave 2 core: the cap was 12 km, so a HAPS at 20 km was "
                          "evaluated at 12 km; P.676 integrates from this height to 100 km, so "
                          "any value below 100 km is valid.",
                          DoubleValue(0.05),
                          MakeDoubleAccessor(&NtnSatLinkErrorModel::m_groundAltKm),
                          MakeDoubleChecker<double>(0.0, 99.0))
            .AddAttribute("GroundAltFromGeometry",
                          "Take the lower endpoint's altitude from its own position (WGS-84 "
                          "geodetic height in ECEF, z in a local ENU frame) instead of "
                          "GroundAltKm, so one model serves a UE, an aircraft and a HAPS. "
                          "Default false (GroundAltKm).",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NtnSatLinkErrorModel::m_groundAltFromGeometry),
                          MakeBooleanChecker())
            .AddAttribute("ExcessLossElevationStepDeg",
                          "Elevation bin (deg) of the excess-loss cache. 0 (default) caches "
                          "exact inputs only, which is bit-identical to evaluating P.676/P.618 "
                          "every packet. A positive step (e.g. 0.01) evaluates once per bin "
                          "(at the bin centre) per ground altitude, carrier and rain rate.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NtnSatLinkErrorModel::m_excessElevStepDeg),
                          MakeDoubleChecker<double>(0.0, 5.0))
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

void
NtnSatLinkErrorModel::ConfigureTr38821Set1KaGsl(bool satToGround,
                                                 double linkRateBps,
                                                 double stationLatDeg,
                                                 double designElevDeg,
                                                 double satAltKm,
                                                 double marginDb)
{
    // EN 302 307-1 V1.4.1 Table 13: spectral efficiency (b/symbol, normal
    // FECFRAME, no pilots) and the Es/No for QEF on AWGN.
    struct Mc
    {
        SatEnums::SatModcod_t id;
        const char* name;
        double eff;
        double qefDb;
    };
    static const Mc kTable[] = {
        {SatEnums::SAT_MODCOD_8PSK_9_TO_10, "8PSK 9/10", 2.679207, 10.98},
        {SatEnums::SAT_MODCOD_8PSK_8_TO_9, "8PSK 8/9", 2.646012, 10.69},
        {SatEnums::SAT_MODCOD_8PSK_5_TO_6, "8PSK 5/6", 2.478562, 9.35},
        {SatEnums::SAT_MODCOD_8PSK_3_TO_4, "8PSK 3/4", 2.228124, 7.91},
        {SatEnums::SAT_MODCOD_8PSK_2_TO_3, "8PSK 2/3", 1.980636, 6.62},
        {SatEnums::SAT_MODCOD_QPSK_9_TO_10, "QPSK 9/10", 1.788612, 6.42},
        {SatEnums::SAT_MODCOD_QPSK_8_TO_9, "QPSK 8/9", 1.766451, 6.20},
        {SatEnums::SAT_MODCOD_QPSK_5_TO_6, "QPSK 5/6", 1.654663, 5.18},
        {SatEnums::SAT_MODCOD_QPSK_4_TO_5, "QPSK 4/5", 1.587196, 4.68},
        {SatEnums::SAT_MODCOD_QPSK_3_TO_4, "QPSK 3/4", 1.487473, 4.03},
        {SatEnums::SAT_MODCOD_QPSK_2_TO_3, "QPSK 2/3", 1.322253, 3.10},
        {SatEnums::SAT_MODCOD_QPSK_3_TO_5, "QPSK 3/5", 1.188304, 2.23},
        {SatEnums::SAT_MODCOD_QPSK_1_TO_2, "QPSK 1/2", 0.988858, 1.00},
        {SatEnums::SAT_MODCOD_QPSK_1_TO_3, "QPSK 1/3", 0.656448, -1.24},
    };
    if (satToGround)
    {
        m_rxGtDbK = 15.9;
        m_carrierHz = 20e9;
    }
    else
    {
        m_eirpDbw = 46.2;
        m_rxGtDbK = 13.0;
        m_carrierHz = 30e9;
    }
    m_groundAltFromGeometry = true;
    m_stationLatDeg = stationLatDeg;
    m_excessElevStepDeg = 0.05;
    // The measured DVB-S2 LDPC (64800-bit frame) curve, not the erfc abstraction.
    m_useDvbS2LinkResults = true;

    // Design slant range at designElevDeg (spherical Earth, mean radius).
    const double re = 6371.0e3;
    const double rs = re + satAltKm * 1e3;
    const double el = designElevDeg * M_PI / 180.0;
    const double slant = std::sqrt(rs * rs - std::pow(re * std::cos(el), 2.0)) - re * std::sin(el);
    const double excess = ExcessLossDbFor(designElevDeg);
    const Mc* pick = &kTable[sizeof(kTable) / sizeof(kTable[0]) - 1];
    for (const Mc& mc : kTable)
    {
        const double bw = linkRateBps / mc.eff;
        const double eirp = satToGround ? 4.0 + 10.0 * std::log10(bw / 1e6) : m_eirpDbw;
        const double fspl = 20.0 * std::log10(4.0 * M_PI * slant * m_carrierHz / kC);
        const double esNo =
            eirp - fspl - excess + m_rxGtDbK - kBoltzDbwHzK - 10.0 * std::log10(bw);
        if (esNo >= mc.qefDb + marginDb)
        {
            pick = &mc;
            m_designEsNoDb = esNo;
            break;
        }
        m_designEsNoDb = esNo;
    }
    m_bandwidthHz = linkRateBps / pick->eff;
    if (satToGround)
    {
        m_eirpDbw = 4.0 + 10.0 * std::log10(m_bandwidthHz / 1e6);
    }
    m_thresholdEsNoDb = pick->qefDb;
    m_modcod = static_cast<uint32_t>(pick->id);
    m_modcodName = pick->name;
    m_modcodEff = pick->eff;
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
    const Vector los(upper.x - lower.x, upper.y - lower.y, upper.z - lower.z);
    const double lsLen = los.GetLength();
    if (lsLen < 1.0)
    {
        return 90.0;
    }
    // Wave 2 core: a local ENU (flat) frame has no Earth centre at the origin,
    // so the position vector is not the zenith there. Same frame test as
    // Ntn38811ExcessLossModel::ElevationDeg: below half an Earth radius the
    // frame is ENU and up is +z. ECEF positions are unchanged.
    if (lr < 0.5 * 6371000.0)
    {
        const double s = std::max(-1.0, std::min(1.0, los.z / lsLen));
        return std::asin(s) * 180.0 / M_PI;
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
    double elev = std::max(elevationDeg, 3.0);
    const double altKm = EffectiveGroundAltKm();

    // Wave 2 core: P.676 line-by-line is about 1.6 ms per call and used to run
    // for every packet (sagin-sgp4-routed-traffic 2.3 s -> 259 s wall). Cache
    // the three terms on their inputs; exact inputs by default (bit-identical),
    // or per ExcessLossElevationStepDeg bin.
    if (m_excessElevStepDeg > 0.0)
    {
        elev = (std::floor(elev / m_excessElevStepDeg) + 0.5) * m_excessElevStepDeg;
    }
    // The Enable* switches are part of the key, so toggling one mid-run never
    // serves a term computed under the other setting.
    const int flags = (m_enableGaseous ? 1 : 0) + (m_enableRain ? 2 : 0) +
                      (m_enableScintillation ? 4 : 0);
    const auto key = std::make_tuple(elev,
                                     altKm,
                                     m_carrierHz,
                                     m_rainRateMmH,
                                     flags,
                                     m_antennaDiameterM,
                                     m_stationLatDeg);
    // Wave 7B: on the binned grid the terms depend on the key alone (the ITU
    // model objects are built with defaults plus the keyed station latitude
    // and aperture), so one process-wide table serves every link: a
    // constellation run configures hundreds of ground links on the same grid
    // and each P.676-13 Annex 1 ray trace costs milliseconds.
    static decltype(m_excessCache) sharedBinned;
    auto& cache = (m_excessElevStepDeg > 0.0) ? sharedBinned : m_excessCache;
    {
        auto it = cache.find(key);
        if (it != cache.end())
        {
            m_lastGaseousDb = it->second.gas;
            m_lastRainDb = it->second.rain;
            m_lastScintDb = it->second.scint;
            return m_lastGaseousDb + m_lastRainDb + m_lastScintDb;
        }
    }
    if (m_excessElevStepDeg <= 0.0 && m_excessCache.size() > 4096)
    {
        m_excessCache.clear(); // exact-input cache: bound the memory on long runs
    }
    ++m_excessEvaluations;

    if (m_enableGaseous)
    {
        if (!m_gas)
        {
            m_gas = CreateObject<itu::Itu676AbsorptionModel>();
        }
        m_lastGaseousDb = DynamicCast<itu::Itu676AbsorptionModel>(m_gas)
                              ->SlantPathAttenuationDb(m_carrierHz, elev, altKm);
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
            m_carrierHz, elev, m_rainRateMmH, altKm, itu::Polarization::circular);
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
    cache[key] = ExcessTerms{m_lastGaseousDb, m_lastRainDb, m_lastScintDb};
    return m_lastGaseousDb + m_lastRainDb + m_lastScintDb;
}

double
NtnSatLinkErrorModel::EffectiveGroundAltKm() const
{
    if (!m_groundAltFromGeometry || !m_tx || !m_rx)
    {
        return m_groundAltKm;
    }
    const Vector a = m_tx->GetPosition();
    const Vector b = m_rx->GetPosition();
    const Vector lower = (a.GetLength() <= b.GetLength()) ? a : b;
    double hM = 0.0;
    if (lower.GetLength() < 0.5 * 6371000.0)
    {
        hM = lower.z; // local ENU frame
    }
    else
    {
        hM = GeographicPositions::CartesianToGeographicCoordinates(
                 lower,
                 GeographicPositions::EarthSpheroidType::WGS84)
                 .z;
    }
    return std::max(0.0, std::min(99.0, hM / 1000.0));
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
    ++m_seen;
    if (!m_contactUp)
    {
        ++m_gateDrops;
        return true;
    }
    const double slant = SlantRangeM();
    if (slant < 0.0)
    {
        return false; // no geometry configured -> pass
    }
    m_lastEsNoDb = EsNoDbFor(slant);
    m_lastBler = BlerFor(m_lastEsNoDb);
    ++m_blerN;
    m_blerSum += m_lastBler;
    const bool lost = m_rng->GetValue(0.0, 1.0) < m_lastBler;
    m_blerDrops += lost ? 1 : 0;
    return lost;
}

void
NtnSatLinkErrorModel::DoReset()
{
    m_lastEsNoDb = 0.0;
    m_lastBler = 0.0;
}

} // namespace ntncon
} // namespace ns3
