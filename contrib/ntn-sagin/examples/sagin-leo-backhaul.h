/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// sagin-leo-backhaul.h (Wave 7B, example-side, shared by sagin-uav-swarm and
// sagin-a2g-real-stack) - the space segment of a SAGIN cell: the terrestrial
// or aerial gNB of the scenario reaches its core over a bent-pipe LEO hop.
//
// What it does, on the NtnRealStackHelper user plane:
//  - a Walker-delta shell (ntn-constellation WalkerConstellation + Vallado
//    SGP4, Sgp4MobilityModel::SetElementsSgp4), by default 72 x 22 at 550 km
//    and 53 deg;
//  - the serving satellite is one that both the site terminal (the gNB's
//    backhaul VSAT) and the gateway see above their elevation masks; the one
//    with the highest elevation at the site is taken and kept until it drops
//    below either mask (break-before-make, no prediction);
//  - every Step (default 100 ms) the one-way backhaul delay becomes
//    (|site - sat| + |sat - gateway|) / c from the EXACT SGP4 state at that
//    instant (GetEcefStateAt, not the 100 ms position cache), written as the
//    helper's backhaul delay (the helper adds its folded service-link term
//    on its own refresh, which is set to the same step);
//  - packet loss on the backhaul point-to-point link from two ntn-constellation
//    NtnSatLinkErrorModel legs per direction (FSPL from the live slant, ITU-R
//    P.676 gas, P.618 scintillation, DVB-S2 QPSK 1/2 waterfall, threshold
//    1.0 dB Es/No): downlink = gateway -> satellite (30 GHz) then satellite
//    -> site (20 GHz); uplink the reverse. During an outage (no satellite
//    above both masks) every packet is dropped.
//
// Declared budget (the TR 38.821 Table 6.1.1.1-1/-2 Set-1 Ka figures the
// store-and-forward example already uses): satellite Ka EIRP density
// 4 dBW/MHz, satellite Ka G/T 13 dB/K, site terminal and gateway VSAT class
// G/T 15.9 dB/K and EIRP 46.2 dBW (33 dBm + 43.2 dBi). The noise bandwidth is
// the symbol rate the backhaul DataRate needs at QPSK 1/2 (0.988 b/symbol),
// and the backhaul point-to-point DataRate is set to that rate.
// Elevation masks default to 25 deg, the minimum elevation of the SpaceX
// Gen1 Ku/Ka authorization.
//
// Topology note: NtnRealStackHelper carries the backhaul as the remote-host
// to PGW point-to-point link, so the satellite hop sits between the core and
// the internet host rather than between the gNB and the core. Every user-plane
// packet crosses it once per direction either way; control-plane (S1-AP/NGAP)
// is not routed over it.

#ifndef SAGIN_LEO_BACKHAUL_H
#define SAGIN_LEO_BACKHAUL_H

#include "ns3/core-module.h"
#include "ns3/error-model.h"
#include "ns3/ntn-geodetic-fixed-mobility-model.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-sat-link-error-model.h"
#include "ns3/point-to-point-module.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace ns3
{

/// Two NtnSatLinkErrorModel legs in series behind a visibility gate.
class SaginBentPipeErrorModel : public ErrorModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::SaginBentPipeErrorModel").SetParent<ErrorModel>();
        return tid;
    }

    bool up{false};
    Ptr<ntncon::NtnSatLinkErrorModel> leg1, leg2;
    uint64_t rx{0}, lostOutage{0}, lostRadio{0};
    double esno1Sum{0}, esno2Sum{0};
    uint64_t esnoN{0};

  private:
    bool DoCorrupt(Ptr<Packet> p) override
    {
        ++rx;
        if (!up)
        {
            ++lostOutage;
            return true;
        }
        const bool b1 = leg1->IsCorrupt(p);
        const bool b2 = leg2->IsCorrupt(p);
        esno1Sum += leg1->LastEsNoDb();
        esno2Sum += leg2->LastEsNoDb();
        ++esnoN;
        if (b1 || b2)
        {
            ++lostRadio;
            return true;
        }
        return false;
    }

    void DoReset() override
    {
    }
};

struct SaginLeoBackhaulConfig
{
    double siteLatDeg{46.0};
    double siteLonDeg{-100.0};
    double siteAltM{0.0};
    double gwLatDeg{46.88};
    double gwLonDeg{-96.79};
    double siteMaskDeg{25.0};
    double gwMaskDeg{25.0};
    uint32_t planes{72};
    uint32_t perPlane{22};
    uint32_t phasing{39};
    double altKm{550.0};
    double incDeg{53.0};
    double epochUnix{1735689600.0}; // 2025-01-01 00:00 UTC
    Time step{MilliSeconds(100)};
    double rateBps{100e6};
};

class SaginLeoBackhaul
{
  public:
    explicit SaginLeoBackhaul(const SaginLeoBackhaulConfig& c)
        : m_cfg(c)
    {
        ntncon::WalkerConfig w;
        w.num_planes = c.planes;
        w.total_sats = c.planes * c.perPlane;
        w.phasing_f = c.phasing;
        w.altitude_km = c.altKm;
        w.inclination_deg = c.incDeg;
        w.epoch_unix_s = c.epochUnix;
        for (const auto& el : ntncon::WalkerConstellation::BuildDelta(w))
        {
            Ptr<ntncon::Sgp4MobilityModel> m = CreateObject<ntncon::Sgp4MobilityModel>();
            m->SetElementsSgp4(el);
            m_sats.push_back(m);
        }
        m_siteMob = CreateObject<NtnGeodeticFixedMobilityModel>();
        m_siteMob->SetGeodetic(c.siteLatDeg, c.siteLonDeg, c.siteAltM);
        m_gwMob = CreateObject<NtnGeodeticFixedMobilityModel>();
        m_gwMob->SetGeodetic(c.gwLatDeg, c.gwLonDeg, 0.0);
        m_site = m_siteMob->GetPosition();
        m_gw = m_gwMob->GetPosition();
    }

    /// Before NtnRealStackHelper::Build(): the helper re-folds the backhaul
    /// delay at the same step as this model updates it.
    void Configure(NtnRealStackHelper& rs) const
    {
        rs.SetBackhaulRefresh(m_cfg.step);
    }

    /// After Build(): take over the backhaul link and start stepping.
    void Attach(NtnRealStackHelper& rs, const std::string& csvPath)
    {
        m_rs = &rs;
        m_chan = DynamicCast<PointToPointChannel>(rs.GetBackhaulChannel());
        NS_ABORT_MSG_UNLESS(m_chan, "LEO backhaul: the helper's backhaul is not point-to-point");
        const double noiseBw = m_cfg.rateBps / 0.988; // QPSK 1/2, EN 302 307-1 Table 13
        const double bwDb = 10.0 * std::log10(noiseBw / 1e6);
        auto leg = [&](double eirpDbw, double gt, double fHz, double latDeg) {
            Ptr<ntncon::NtnSatLinkErrorModel> e = CreateObject<ntncon::NtnSatLinkErrorModel>();
            e->SetAttribute("EirpDbw", DoubleValue(eirpDbw));
            e->SetAttribute("RxGtDbK", DoubleValue(gt));
            e->SetAttribute("BandwidthHz", DoubleValue(noiseBw));
            e->SetAttribute("CarrierHz", DoubleValue(fHz));
            e->SetAttribute("ThresholdEsNoDb", DoubleValue(1.0));
            e->SetAttribute("GroundAltKm", DoubleValue(0.0));
            e->SetAttribute("StationLatitudeDeg", DoubleValue(latDeg));
            return e;
        };
        const double satEirp = 4.0 + bwDb;
        // device 0 = PGW side (downlink arrives here), device 1 = remote host (uplink).
        m_dl = CreateObject<SaginBentPipeErrorModel>();
        m_dl->leg1 = leg(46.2, 13.0, 30e9, m_cfg.gwLatDeg);    // gateway -> satellite
        m_dl->leg2 = leg(satEirp, 15.9, 20e9, m_cfg.siteLatDeg); // satellite -> site
        m_ul = CreateObject<SaginBentPipeErrorModel>();
        m_ul->leg1 = leg(46.2, 13.0, 30e9, m_cfg.siteLatDeg);  // site -> satellite
        m_ul->leg2 = leg(satEirp, 15.9, 20e9, m_cfg.gwLatDeg); // satellite -> gateway
        for (uint32_t i = 0; i < 2; ++i)
        {
            Ptr<NetDevice> nd = m_chan->GetDevice(i);
            nd->SetAttribute("ReceiveErrorModel", PointerValue(i == 0 ? m_dl : m_ul));
            nd->SetAttribute("DataRate", DataRateValue(DataRate(static_cast<uint64_t>(m_cfg.rateBps))));
        }
        m_csv.open(csvPath);
        m_csv << "t_s,sat,site_elev_deg,gw_elev_deg,one_way_ms,switched\n";
        Step();
        // the channel delay for [0, step): helper backhaul + its folded service term
        m_chan->SetAttribute("Delay", TimeValue(rs.GetBackhaulDelay() + rs.ComputeServiceLinkDelay()));
    }

    void Stop()
    {
        m_stopped = true;
        m_csv.close();
    }

    // ---- results ----
    uint32_t Switches() const { return m_switches; }
    double OutageS() const { return m_outageSteps * m_cfg.step.GetSeconds(); }
    double DelayMinMs() const { return m_dMin * 1e3; }
    double DelayMaxMs() const { return m_dMax * 1e3; }
    double DelayMeanMs() const { return m_n ? 1e3 * m_dSum / m_n : 0.0; }
    Ptr<SaginBentPipeErrorModel> Dl() const { return m_dl; }
    Ptr<SaginBentPipeErrorModel> Ul() const { return m_ul; }
    size_t NumSats() const { return m_sats.size(); }

    void PrintSummary(const char* tag) const
    {
        auto ratio = [](uint64_t a, uint64_t b) { return b ? double(a) / b : 0.0; };
        std::printf("# %s LEO backhaul (Walker %ux%u, %.0f km, %.0f deg, SGP4): site (%.2f, %.2f) "
                    "gateway (%.2f, %.2f), masks %.0f/%.0f deg\n"
                    "#   one-way delay min/mean/max %.3f/%.3f/%.3f ms, satellite switches %u, "
                    "outage %.1f s\n"
                    "#   DL packets %lu lost %lu (outage %lu, radio %lu) loss %.5f, mean Es/No "
                    "uplink-leg %.2f dB downlink-leg %.2f dB\n"
                    "#   UL packets %lu lost %lu loss %.5f\n",
                    tag, m_cfg.planes, m_cfg.perPlane, m_cfg.altKm, m_cfg.incDeg, m_cfg.siteLatDeg,
                    m_cfg.siteLonDeg, m_cfg.gwLatDeg, m_cfg.gwLonDeg, m_cfg.siteMaskDeg,
                    m_cfg.gwMaskDeg, DelayMinMs(), DelayMeanMs(), DelayMaxMs(), m_switches, OutageS(),
                    (unsigned long)m_dl->rx, (unsigned long)(m_dl->lostOutage + m_dl->lostRadio),
                    (unsigned long)m_dl->lostOutage, (unsigned long)m_dl->lostRadio,
                    ratio(m_dl->lostOutage + m_dl->lostRadio, m_dl->rx),
                    m_dl->esnoN ? m_dl->esno1Sum / m_dl->esnoN : 0.0,
                    m_dl->esnoN ? m_dl->esno2Sum / m_dl->esnoN : 0.0, (unsigned long)m_ul->rx,
                    (unsigned long)(m_ul->lostOutage + m_ul->lostRadio),
                    ratio(m_ul->lostOutage + m_ul->lostRadio, m_ul->rx));
    }

  private:
    static double ElevDeg(const Vector& g, const Vector& s)
    {
        const Vector l = s - g;
        const double se = (g.x * l.x + g.y * l.y + g.z * l.z) / (g.GetLength() * l.GetLength());
        return std::asin(std::max(-1.0, std::min(1.0, se))) * 180.0 / M_PI;
    }

    Vector Exact(uint32_t k) const
    {
        Vector p, v;
        m_sats[k]->GetEcefStateAt(Simulator::Now(), p, v);
        return p;
    }

    void Step()
    {
        if (m_stopped)
        {
            return;
        }
        bool switched = false;
        double eS = -90, eG = -90;
        if (m_cur >= 0)
        {
            const Vector p = Exact(m_cur);
            eS = ElevDeg(m_site, p);
            eG = ElevDeg(m_gw, p);
            if (eS < m_cfg.siteMaskDeg || eG < m_cfg.gwMaskDeg)
            {
                m_cur = -1;
            }
        }
        if (m_cur < 0)
        {
            double best = -90.0;
            for (uint32_t k = 0; k < m_sats.size(); ++k)
            {
                const Vector p = Exact(k);
                const double s = ElevDeg(m_site, p);
                if (s < m_cfg.siteMaskDeg || s <= best)
                {
                    continue;
                }
                const double g = ElevDeg(m_gw, p);
                if (g < m_cfg.gwMaskDeg)
                {
                    continue;
                }
                best = s;
                m_cur = static_cast<int32_t>(k);
                eS = s;
                eG = g;
            }
            if (m_cur >= 0)
            {
                switched = m_hadSat;
                m_switches += switched ? 1 : 0;
                m_hadSat = true;
                m_dl->leg1->SetEndpoints(m_gwMob, m_sats[m_cur]);
                m_dl->leg2->SetEndpoints(m_sats[m_cur], m_siteMob);
                m_ul->leg1->SetEndpoints(m_siteMob, m_sats[m_cur]);
                m_ul->leg2->SetEndpoints(m_sats[m_cur], m_gwMob);
            }
        }
        const bool up = m_cur >= 0;
        m_dl->up = up;
        m_ul->up = up;
        double oneWay = std::numeric_limits<double>::quiet_NaN();
        if (up)
        {
            const Vector p = Exact(m_cur);
            oneWay = (CalculateDistance(m_site, p) + CalculateDistance(p, m_gw)) / 299792458.0;
            m_rs->SetBackhaulDelay(Seconds(oneWay));
            m_dMin = std::min(m_dMin, oneWay);
            m_dMax = std::max(m_dMax, oneWay);
            m_dSum += oneWay;
            ++m_n;
        }
        else
        {
            ++m_outageSteps;
        }
        m_csv << Simulator::Now().GetSeconds() << "," << m_cur << "," << eS << "," << eG << ","
              << oneWay * 1e3 << "," << switched << "\n";
        // Fire just before the helper's own refresh at the same instant, so it
        // folds this step's value without a one-step lag.
        const Time now = Simulator::Now();
        const int64_t k = (now.GetNanoSeconds() + 1000) / m_cfg.step.GetNanoSeconds() + 1;
        Simulator::Schedule(NanoSeconds(k * m_cfg.step.GetNanoSeconds()) - MicroSeconds(1) - now,
                            &SaginLeoBackhaul::Step, this);
    }

    SaginLeoBackhaulConfig m_cfg;
    std::vector<Ptr<ntncon::Sgp4MobilityModel>> m_sats;
    Ptr<NtnGeodeticFixedMobilityModel> m_siteMob, m_gwMob;
    Vector m_site, m_gw;
    NtnRealStackHelper* m_rs{nullptr};
    Ptr<PointToPointChannel> m_chan;
    Ptr<SaginBentPipeErrorModel> m_dl, m_ul;
    std::ofstream m_csv;
    int32_t m_cur{-1};
    bool m_hadSat{false};
    bool m_stopped{false};
    uint32_t m_switches{0};
    uint64_t m_outageSteps{0};
    double m_dMin{std::numeric_limits<double>::infinity()};
    double m_dMax{0.0};
    double m_dSum{0.0};
    uint64_t m_n{0};
};

} // namespace ns3

#endif // SAGIN_LEO_BACKHAUL_H
