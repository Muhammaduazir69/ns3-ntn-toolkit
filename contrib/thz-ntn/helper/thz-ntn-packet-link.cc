/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
#include "thz-ntn-packet-link.h"

#include "ns3/thz-ntn-link-geometry.h"

#include "ns3/data-rate.h"
#include "ns3/log.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/pointer.h"
#include "ns3/satellite-utils.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace ns3
{
namespace thzntn
{

NS_LOG_COMPONENT_DEFINE("ThzNtnPacketLink");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnPacketLink);

namespace
{
constexpr double kC = 299792458.0;
/// ITU-R P.618-14 Section 3: mean radiating temperature of the atmosphere.
constexpr double kTmr = 275.0;
constexpr double kTcmb = 2.7;
/// Antenna temperature of a satellite receive aperture filled by the Earth.
constexpr double kTEarth = 290.0;

double
SkyTemperatureK(double atmLossDb)
{
    const double t = std::pow(10.0, -atmLossDb / 10.0);
    return kTmr * (1.0 - t) + kTcmb * t;
}

const char*
ModcodName(SatEnums::SatModcod_t m)
{
    switch (m)
    {
    case SatEnums::SAT_MODCOD_QPSK_1_TO_2: return "QPSK-1/2";
    case SatEnums::SAT_MODCOD_QPSK_3_TO_5: return "QPSK-3/5";
    case SatEnums::SAT_MODCOD_QPSK_2_TO_3: return "QPSK-2/3";
    case SatEnums::SAT_MODCOD_QPSK_3_TO_4: return "QPSK-3/4";
    case SatEnums::SAT_MODCOD_QPSK_4_TO_5: return "QPSK-4/5";
    case SatEnums::SAT_MODCOD_QPSK_5_TO_6: return "QPSK-5/6";
    case SatEnums::SAT_MODCOD_QPSK_8_TO_9: return "QPSK-8/9";
    case SatEnums::SAT_MODCOD_QPSK_9_TO_10: return "QPSK-9/10";
    case SatEnums::SAT_MODCOD_8PSK_3_TO_5: return "8PSK-3/5";
    case SatEnums::SAT_MODCOD_8PSK_2_TO_3: return "8PSK-2/3";
    case SatEnums::SAT_MODCOD_8PSK_3_TO_4: return "8PSK-3/4";
    case SatEnums::SAT_MODCOD_8PSK_5_TO_6: return "8PSK-5/6";
    case SatEnums::SAT_MODCOD_8PSK_8_TO_9: return "8PSK-8/9";
    case SatEnums::SAT_MODCOD_8PSK_9_TO_10: return "8PSK-9/10";
    case SatEnums::SAT_MODCOD_16APSK_2_TO_3: return "16APSK-2/3";
    case SatEnums::SAT_MODCOD_16APSK_3_TO_4: return "16APSK-3/4";
    case SatEnums::SAT_MODCOD_16APSK_4_TO_5: return "16APSK-4/5";
    case SatEnums::SAT_MODCOD_16APSK_5_TO_6: return "16APSK-5/6";
    case SatEnums::SAT_MODCOD_16APSK_8_TO_9: return "16APSK-8/9";
    case SatEnums::SAT_MODCOD_16APSK_9_TO_10: return "16APSK-9/10";
    case SatEnums::SAT_MODCOD_32APSK_3_TO_4: return "32APSK-3/4";
    case SatEnums::SAT_MODCOD_32APSK_4_TO_5: return "32APSK-4/5";
    case SatEnums::SAT_MODCOD_32APSK_5_TO_6: return "32APSK-5/6";
    case SatEnums::SAT_MODCOD_32APSK_8_TO_9: return "32APSK-8/9";
    default: return "other";
    }
}
} // namespace

TypeId
ThzNtnPacketLink::GetTypeId()
{
    static TypeId tid = TypeId("ns3::thzntn::ThzNtnPacketLink")
                            .SetParent<Object>()
                            .SetGroupName("ThzNtn")
                            .AddConstructor<ThzNtnPacketLink>();
    return tid;
}

ThzNtnPacketLink::ThzNtnPacketLink() = default;

ThzNtnPacketLink::~ThzNtnPacketLink()
{
    if (m_ts)
    {
        m_ts->close();
    }
}

const std::vector<SatEnums::SatModcod_t>&
ThzNtnPacketLink::DvbS2ModcodSet()
{
    static const std::vector<SatEnums::SatModcod_t> set = [] {
        std::vector<SatEnums::SatModcod_t> v = {
            SatEnums::SAT_MODCOD_QPSK_1_TO_2,   SatEnums::SAT_MODCOD_QPSK_3_TO_5,
            SatEnums::SAT_MODCOD_QPSK_2_TO_3,   SatEnums::SAT_MODCOD_QPSK_3_TO_4,
            SatEnums::SAT_MODCOD_QPSK_4_TO_5,   SatEnums::SAT_MODCOD_QPSK_5_TO_6,
            SatEnums::SAT_MODCOD_QPSK_8_TO_9,   SatEnums::SAT_MODCOD_QPSK_9_TO_10,
            SatEnums::SAT_MODCOD_8PSK_3_TO_5,   SatEnums::SAT_MODCOD_8PSK_2_TO_3,
            SatEnums::SAT_MODCOD_8PSK_3_TO_4,   SatEnums::SAT_MODCOD_8PSK_5_TO_6,
            SatEnums::SAT_MODCOD_8PSK_8_TO_9,   SatEnums::SAT_MODCOD_8PSK_9_TO_10,
            SatEnums::SAT_MODCOD_16APSK_2_TO_3, SatEnums::SAT_MODCOD_16APSK_3_TO_4,
            SatEnums::SAT_MODCOD_16APSK_4_TO_5, SatEnums::SAT_MODCOD_16APSK_5_TO_6,
            SatEnums::SAT_MODCOD_16APSK_8_TO_9, SatEnums::SAT_MODCOD_16APSK_9_TO_10,
            SatEnums::SAT_MODCOD_32APSK_3_TO_4, SatEnums::SAT_MODCOD_32APSK_4_TO_5,
            SatEnums::SAT_MODCOD_32APSK_5_TO_6, SatEnums::SAT_MODCOD_32APSK_8_TO_9};
        std::stable_sort(v.begin(), v.end(), [](auto a, auto b) {
            return SpectralEfficiency(a) < SpectralEfficiency(b);
        });
        return v;
    }();
    return set;
}

double
ThzNtnPacketLink::SpectralEfficiency(SatEnums::SatModcod_t m)
{
    // ETSI EN 302 307-1 Table 13, normal FECFRAME (64800 bits), no pilots: the
    // efficiency after BCH/LDPC overhead and the 90-symbol PLHEADER. The
    // nominal bits x code rate used before overstated every MODCOD (QPSK 1/2
    // by 1.1%, 32APSK 3/4 by 1.3%), and with it the line rate the link set.
    switch (m)
    {
    case SatEnums::SAT_MODCOD_QPSK_1_TO_2: return 0.988858;
    case SatEnums::SAT_MODCOD_QPSK_3_TO_5: return 1.188304;
    case SatEnums::SAT_MODCOD_QPSK_2_TO_3: return 1.322253;
    case SatEnums::SAT_MODCOD_QPSK_3_TO_4: return 1.487473;
    case SatEnums::SAT_MODCOD_QPSK_4_TO_5: return 1.587196;
    case SatEnums::SAT_MODCOD_QPSK_5_TO_6: return 1.654663;
    case SatEnums::SAT_MODCOD_QPSK_8_TO_9: return 1.766451;
    case SatEnums::SAT_MODCOD_QPSK_9_TO_10: return 1.788612;
    case SatEnums::SAT_MODCOD_8PSK_3_TO_5: return 1.779991;
    case SatEnums::SAT_MODCOD_8PSK_2_TO_3: return 1.980636;
    case SatEnums::SAT_MODCOD_8PSK_3_TO_4: return 2.228124;
    case SatEnums::SAT_MODCOD_8PSK_5_TO_6: return 2.478562;
    case SatEnums::SAT_MODCOD_8PSK_8_TO_9: return 2.646012;
    case SatEnums::SAT_MODCOD_8PSK_9_TO_10: return 2.679207;
    case SatEnums::SAT_MODCOD_16APSK_2_TO_3: return 2.637201;
    case SatEnums::SAT_MODCOD_16APSK_3_TO_4: return 2.966728;
    case SatEnums::SAT_MODCOD_16APSK_4_TO_5: return 3.165623;
    case SatEnums::SAT_MODCOD_16APSK_5_TO_6: return 3.300184;
    case SatEnums::SAT_MODCOD_16APSK_8_TO_9: return 3.523143;
    case SatEnums::SAT_MODCOD_16APSK_9_TO_10: return 3.567342;
    case SatEnums::SAT_MODCOD_32APSK_3_TO_4: return 3.703295;
    case SatEnums::SAT_MODCOD_32APSK_4_TO_5: return 3.951571;
    case SatEnums::SAT_MODCOD_32APSK_5_TO_6: return 4.119540;
    case SatEnums::SAT_MODCOD_32APSK_8_TO_9: return 4.397854;
    default: return SatUtils::GetModulatedBits(m) * SatUtils::GetCodingRate(m);
    }
}

bool
ThzNtnPacketLink::SelectModcod(double esNoDb, SatEnums::SatModcod_t& out) const
{
    bool found = false;
    double bestEta = -1.0;
    for (auto m : DvbS2ModcodSet())
    {
        const double req = ThzNtnLinkErrorModel::DvbS2RequiredEsNoDb(m, m_cfg.blerTarget);
        const double eta = SpectralEfficiency(m);
        if (req <= esNoDb - m_cfg.acmMarginDb && eta > bestEta)
        {
            bestEta = eta;
            out = m;
            found = true;
        }
    }
    return found;
}

double
ThzNtnPacketLink::GetSymbolRateHz() const
{
    return m_cfg.bandwidthHz / (1.0 + m_cfg.rollOff);
}

NetDeviceContainer
ThzNtnPacketLink::Install(Ptr<Node> space,
                          Ptr<Node> ground,
                          Ptr<MobilityModel> spaceMob,
                          Ptr<MobilityModel> groundMob)
{
    NS_ABORT_MSG_IF(!space || !ground || !spaceMob || !groundMob,
                    "ThzNtnPacketLink::Install needs both nodes and both mobility models");
    m_spaceMob = spaceMob;
    m_groundMob = groundMob;

    // Friis -> ThzNtnPropagationLossModel: free space plus P.676 gas, P.840
    // cloud and P.838/P.618 rain at the live (apparent) elevation; no 3GPP
    // 100 GHz cap anywhere on the chain.
    Ptr<FriisPropagationLossModel> friis = CreateObject<FriisPropagationLossModel>();
    friis->SetFrequency(m_cfg.freqHz);
    m_atm = CreateObject<ThzNtnPropagationLossModel>();
    m_atm->SetFrequency(m_cfg.freqHz);
    m_atm->SetRainRate(m_cfg.rainRateMmH);
    Ptr<PropagationLossModel> tail = friis;
    if (m_cfg.pointing)
    {
        const double lambda = kC / m_cfg.freqHz;
        m_pointSpace = CreateObject<ThzNtnPointingLossModel>();
        m_pointSpace->SetBeamwidth3dBDeg(
            70.0 * lambda / ApertureDiameterM(m_cfg.spaceGainDbi, m_cfg.freqHz));
        m_pointGround = CreateObject<ThzNtnPointingLossModel>();
        m_pointGround->SetBeamwidth3dBDeg(
            70.0 * lambda / ApertureDiameterM(m_cfg.groundGainDbi, m_cfg.freqHz));
        tail->SetNext(m_pointSpace);
        m_pointSpace->SetNext(m_pointGround);
        tail = m_pointGround;
    }
    tail->SetNext(m_atm);
    if (m_cfg.scintillation)
    {
        m_scint = CreateObject<ThzNtnScintillation>();
        m_scint->SetAntennaDiameter(ApertureDiameterM(m_cfg.groundGainDbi, m_cfg.freqHz));
    }
    m_chain = friis;

    const double rs = GetSymbolRateHz();
    auto makeEm = [&](Ptr<MobilityModel> tx, Ptr<MobilityModel> rx, double gTx, double gRx) {
        Ptr<ThzNtnLinkErrorModel> em = CreateObject<ThzNtnLinkErrorModel>();
        em->SetEndpoints(tx, rx);
        em->SetPropagationChain(m_chain);
        em->SetTxPowerDbm(m_cfg.txPowerDbm);
        em->SetAntennaGainsDb(gTx, gRx);
        em->SetBandwidthHz(rs); // matched-filter noise bandwidth = symbol rate
        em->SetNoiseFigureDb(m_cfg.noiseFigureDb);
        em->SetDecodeRule(m_cfg.thresholdDecode ? ThzNtnLinkErrorModel::THRESHOLD
                                                : ThzNtnLinkErrorModel::DVBS2_WATERFALL);
        if (m_cfg.thresholdDecode)
        {
            em->SetDecodeSnrDb(std::isnan(m_cfg.decodeSnrDb)
                                   ? ThzNtnLinkErrorModel::DvbS2RequiredEsNoDb(m_cfg.fixedModcod,
                                                                               m_cfg.blerTarget)
                                   : m_cfg.decodeSnrDb);
        }
        em->SetModcod(m_cfg.fixedModcod);
        return em;
    };
    m_emDown = makeEm(spaceMob, groundMob, m_cfg.spaceGainDbi, m_cfg.groundGainDbi);
    m_emUp = makeEm(groundMob, spaceMob, m_cfg.groundGainDbi, m_cfg.spaceGainDbi);
    if (m_cfg.skyNoise)
    {
        m_emUp->SetAntennaNoiseTemperatureK(kTEarth);
    }

    const ThzNtnLinkGeometry g = ThzNtnLinkGeometry::Compute(PeekPointer(spaceMob),
                                                             PeekPointer(groundMob));
    PointToPointHelper p2p;
    const double eta0 = SpectralEfficiency(m_cfg.fixedModcod);
    p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate(static_cast<uint64_t>(rs * eta0))));
    p2p.SetChannelAttribute("Delay", TimeValue(Seconds(g.slantM / kC)));
    NetDeviceContainer devs = p2p.Install(space, ground);
    m_devSpace = DynamicCast<PointToPointNetDevice>(devs.Get(0));
    m_devGround = DynamicCast<PointToPointNetDevice>(devs.Get(1));
    m_channel = DynamicCast<PointToPointChannel>(m_devSpace->GetChannel());
    m_devGround->SetAttribute("ReceiveErrorModel", PointerValue(m_emDown));
    m_devSpace->SetAttribute("ReceiveErrorModel", PointerValue(m_emUp));
    m_modcod = m_cfg.fixedModcod;

    // First refresh at t = 0 (or now), then every update period.
    m_event = Simulator::ScheduleNow(&ThzNtnPacketLink::Update, this);
    return devs;
}

void
ThzNtnPacketLink::EnableTimeSeries(const std::string& path)
{
    m_ts = std::make_unique<std::ofstream>(path);
    if (!m_ts->is_open())
    {
        NS_LOG_WARN("ThzNtnPacketLink: cannot open " << path);
        m_ts.reset();
        return;
    }
    *m_ts << "t_s,elev_true_deg,elev_apparent_deg,range_km,delay_ms,gas_db,cloud_db,rain_db,"
             "atm_db,pointing_db,scint_db,t_ant_k,esno_db,csi_esno_db,modcod,eta,rate_mbps,"
             "ul_esno_db,outage\n";
}

double
ThzNtnPacketLink::ApertureDiameterM(double gainDbi, double freqHz)
{
    const double lambda = kC / freqHz;
    return lambda / M_PI * std::sqrt(std::pow(10.0, gainDbi / 10.0) / 0.6);
}

void
ThzNtnPacketLink::SetExtraLossDb(double db)
{
    m_extraLossDb = db;
    ApplyGains();
}

void
ThzNtnPacketLink::ApplyGains()
{
    if (m_emDown)
    {
        // Extra loss and the scintillation sample (positive = enhancement)
        // act at the receive aperture of each direction.
        const double l = m_extraLossDb - m_scintDb;
        m_emDown->SetAntennaGainsDb(m_cfg.spaceGainDbi, m_cfg.groundGainDbi - l);
        m_emUp->SetAntennaGainsDb(m_cfg.groundGainDbi, m_cfg.spaceGainDbi - l);
    }
}

void
ThzNtnPacketLink::ApplyRate(double bps)
{
    const auto rate = DataRate(static_cast<uint64_t>(std::max(1.0, bps)));
    m_devSpace->SetDataRate(rate);
    m_lastRateMbps = bps / 1e6;
}

void
ThzNtnPacketLink::Update()
{
    const Time now = Simulator::Now();
    const ThzNtnLinkGeometry g = ThzNtnLinkGeometry::Compute(PeekPointer(m_spaceMob),
                                                             PeekPointer(m_groundMob));
    // Live propagation delay, both directions share the channel.
    const Time owd = Seconds(g.slantM / kC);
    m_channel->SetAttribute("Delay", TimeValue(owd));
    m_lastDelayMs = owd.GetSeconds() * 1e3;
    m_lastRangeKm = g.slantM / 1e3;

    // Scintillation: one AR(1) step per update at the live elevation.
    if (m_scint && g.lowerAltKm < 10.0 && g.elevationDeg > 0.0)
    {
        m_scintDb = m_scint->GetScintillationSample_dB(m_cfg.freqHz, std::max(g.elevationDeg, 5.0));
        m_sumScintSq += m_scintDb * m_scintDb;
        if (m_cfg.freqHz > 55e9)
        {
            ++m_scintExtrap;
        }
        ApplyGains();
    }

    // Downlink: evaluate the chain (updates the per-term breakdown), set the
    // ground antenna temperature from the path loss, then take the SNR.
    m_emDown->CurrentRxPowerDbm();
    const double atmDb = m_atm->GetLastLossDb();
    const double pointDb = m_pointSpace ? m_pointSpace->GetLastLossDb() + m_pointGround->GetLastLossDb() : 0.0;
    m_sumPointing += pointDb;
    const double gasDb = m_atm->GetLastGasDb();
    const double cloudDb = m_atm->GetLastCloudDb();
    const double rainDb = m_atm->GetLastRainDb();
    const double tAnt = SkyTemperatureK(atmDb);
    if (m_cfg.skyNoise)
    {
        m_emDown->SetAntennaNoiseTemperatureK(tAnt);
    }
    const double esNo = m_emDown->CurrentSnrDb();
    const double ulEsNo = m_emUp->CurrentSnrDb();
    m_lastSnrDown = esNo;

    // CSI: the receiver's measurement reaches the gateway scheduler one
    // round trip later, so the decision now uses the sample of now - RTT.
    m_csi.emplace_back(now, esNo);
    const Time rtt = owd + owd;
    while (m_csi.size() > 1 && m_csi[1].first <= now - rtt)
    {
        m_csi.pop_front();
    }
    // Front = newest sample at least one RTT old (the oldest at run start).
    const double csi = m_csi.front().second;

    bool outage = false;
    SatEnums::SatModcod_t mc = m_cfg.fixedModcod;
    if (m_cfg.acm)
    {
        if (!SelectModcod(csi, mc))
        {
            outage = true;
            mc = DvbS2ModcodSet().front();
        }
    }
    else
    {
        outage = ThzNtnLinkErrorModel::DvbS2RequiredEsNoDb(mc, m_cfg.blerTarget) > esNo;
    }
    if (m_haveModcod && mc != m_modcod)
    {
        ++m_modcodChanges;
    }
    m_haveModcod = true;
    m_modcod = mc;
    m_emDown->SetModcod(mc);
    if (m_cfg.thresholdDecode && std::isnan(m_cfg.decodeSnrDb))
    {
        m_emDown->SetDecodeSnrDb(ThzNtnLinkErrorModel::DvbS2RequiredEsNoDb(mc, m_cfg.blerTarget));
    }
    const double eta = SpectralEfficiency(mc);
    ApplyRate(GetSymbolRateHz() * eta);

    // Uplink (return) direction: its own ACM decision on its own SNR, on the
    // ground device's line rate.
    SatEnums::SatModcod_t mcUp = m_cfg.fixedModcod;
    if (m_cfg.acm && !SelectModcod(ulEsNo, mcUp))
    {
        mcUp = DvbS2ModcodSet().front();
    }
    m_emUp->SetModcod(mcUp);
    if (m_cfg.thresholdDecode && std::isnan(m_cfg.decodeSnrDb))
    {
        m_emUp->SetDecodeSnrDb(ThzNtnLinkErrorModel::DvbS2RequiredEsNoDb(mcUp, m_cfg.blerTarget));
    }
    m_devGround->SetDataRate(
        DataRate(static_cast<uint64_t>(std::max(1.0, GetSymbolRateHz() * SpectralEfficiency(mcUp)))));

    const double dt = m_cfg.updatePeriod.GetSeconds();
    if (outage)
    {
        m_outageS += dt;
    }
    ++m_updates;
    m_sumRate += m_lastRateMbps;
    m_sumGas += gasDb;
    m_sumCloud += cloudDb;
    m_sumRain += rainDb;
    m_minDelayMs = std::min(m_minDelayMs, m_lastDelayMs);
    m_maxDelayMs = std::max(m_maxDelayMs, m_lastDelayMs);
    m_minSnr = std::min(m_minSnr, esNo);
    m_maxSnr = std::max(m_maxSnr, esNo);
    m_minElev = std::min(m_minElev, g.elevationDeg);
    m_maxElev = std::max(m_maxElev, g.elevationDeg);

    if (m_ts)
    {
        char line[512];
        std::snprintf(line,
                      sizeof(line),
                      "%.3f,%.4f,%.4f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.3f,%.3f,%s,"
                      "%.4f,%.3f,%.3f,%d\n",
                      now.GetSeconds(), g.elevationDeg, m_atm->GetLastApparentElevationDeg(),
                      m_lastRangeKm, m_lastDelayMs, gasDb, cloudDb, rainDb, atmDb, pointDb,
                      m_scintDb,
                      m_cfg.skyNoise ? tAnt : 290.0, esNo, csi, ModcodName(mc), eta,
                      m_lastRateMbps, ulEsNo, outage ? 1 : 0);
        *m_ts << line;
    }
    m_event = Simulator::Schedule(m_cfg.updatePeriod, &ThzNtnPacketLink::Update, this);
}

std::string
ThzNtnPacketLink::GetReport() const
{
    char buf[2000];
    const double n = std::max<double>(1.0, m_updates);
    std::snprintf(
        buf,
        sizeof(buf),
        "THz packet link: f=%.1f GHz B=%.3f GHz R_s=%.3f GBd, %llu updates of %.0f ms\n"
        "  geometry: elevation %.2f..%.2f deg (free-space), one-way delay %.3f..%.3f ms (live)\n"
        "  atmosphere (mean): gas %.3f dB, cloud %.3f dB, rain %.3f dB; P.834 out-of-range %llu\n"
        "  pointing (mean, both apertures) %.3f dB; scintillation rms %.3f dB (%llu samples "
        "above the 55 GHz P.618 limit)\n"
        "  %s\n"
        "  Es/N0 %.2f..%.2f dB (sky noise %s); ACM %s margin %.1f dB BLER target %.0e: "
        "%llu MODCOD changes, mean line rate %.1f Mb/s, outage %.2f s\n"
        "  decode (%s): downlink %llu checked / %llu corrupted (expected %.3g from the BLER "
        "curve), uplink %llu / %llu",
        m_cfg.freqHz / 1e9, m_cfg.bandwidthHz / 1e9, GetSymbolRateHz() / 1e9,
        static_cast<unsigned long long>(m_updates), m_cfg.updatePeriod.GetMilliSeconds() * 1.0,
        m_updates ? m_minElev : 0.0, m_updates ? m_maxElev : 0.0, m_updates ? m_minDelayMs : 0.0,
        m_maxDelayMs, m_sumGas / n, m_sumCloud / n, m_sumRain / n,
        static_cast<unsigned long long>(m_atm ? m_atm->GetP834OutOfRangeCount() : 0),
        m_sumPointing / n, std::sqrt(m_sumScintSq / n),
        static_cast<unsigned long long>(m_scintExtrap),
        m_atm ? m_atm->GetCloudModel()->GetRunReport().c_str() : "cloud: -",
        m_updates ? m_minSnr : 0.0, m_updates ? m_maxSnr : 0.0, m_cfg.skyNoise ? "on" : "off",
        m_cfg.acm ? "on" : "off", m_cfg.acmMarginDb, m_cfg.blerTarget,
        static_cast<unsigned long long>(m_modcodChanges), GetMeanRateMbps(), m_outageS,
        m_cfg.thresholdDecode ? "threshold" : "DVB-S2 BLER",
        static_cast<unsigned long long>(m_emDown ? m_emDown->GetPacketsChecked() : 0),
        static_cast<unsigned long long>(m_emDown ? m_emDown->GetPacketsCorrupted() : 0),
        m_emDown ? m_emDown->GetExpectedCorrupted() : 0.0,
        static_cast<unsigned long long>(m_emUp ? m_emUp->GetPacketsChecked() : 0),
        static_cast<unsigned long long>(m_emUp ? m_emUp->GetPacketsCorrupted() : 0));
    return buf;
}

int64_t
ThzNtnPacketLink::AssignStreams(int64_t stream)
{
    int64_t n = 0;
    if (m_emDown)
    {
        n += m_emDown->AssignStreams(stream + n);
        n += m_emUp->AssignStreams(stream + n);
    }
    if (m_atm)
    {
        n += m_atm->AssignStreams(stream + n);
    }
    if (m_pointSpace)
    {
        n += m_pointSpace->AssignStreams(stream + n);
        n += m_pointGround->AssignStreams(stream + n);
    }
    if (m_scint)
    {
        n += m_scint->AssignStreams(stream + n);
    }
    return n;
}

} // namespace thzntn
} // namespace ns3
