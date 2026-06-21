/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * O-RAN NTN Full Scenario — constellation-scale RIC on REAL orbital geometry.
 *
 * What is REAL here (audit fix 2026-06-12, CRITICAL #1 — the previous version
 * of this example fed the RIC from a sine-formula KPM generator):
 *   - All satellites (default 6 planes x 11 = 66, Walker Delta) are ns-3 nodes
 *     under ns3::ntncon::Sgp4MobilityModel (Kepler+J2, ECEF). Elevation, slant
 *     range, Doppler and time-to-exit in every KPM report are derived from the
 *     live mobility models.
 *   - Serving selection is a STICKY (hysteretic) mobility model, not per-tick
 *     max-elevation reselection: a UE keeps its serving satellite and rides it
 *     down until the predicted time-to-exit the service elevation falls below
 *     the ground-handover threshold, then is handed over to the best visible
 *     candidate — real CHO behavior.
 *
 * Space-RIC autonomy demonstration (what the user asked to surface):
 *   - A regional feeder-link outage (default: first 3 planes, ~t=24..96 s)
 *     puts those satellites' on-board Space-RICs into autonomous mode. While a
 *     satellite's feeder is down the GROUND RIC cannot send handover commands,
 *     so a UE served by it is STRANDED and keeps riding the receding satellite.
 *     As the serving elevation crosses the service elevation (default 30 deg,
 *     a realistic NTN minimum service elevation, TR 38.821 VSAT cases) the
 *     serving TTE drops below 5 s and the on-board RIC AUTONOMOUSLY initiates
 *     the handover — see space_ric_metrics.csv (autonomous_decisions > 0) and
 *     the "Space RIC Summary" print. This is the Deng-2026 on-orbit autonomy
 *     use case, driven by real geometry rather than a scripted event.
 *   - UEs move under 3GPP TR 38.811 §6.1.1.1 class mobility at real ground
 *     positions (clusters under the anchored satellites' t=0 sub-points plus a
 *     wide scale-out field).
 *   - UEs anchored to the first --numRealCells satellites ride a REAL mmwave
 *     NR NTN cell (NtnRealStackHelper: SpectrumPhy + MAC + RLC/PDCP + RRC +
 *     EPC, real packets): their KPM SINR is MEASURED off the PHY trace.
 *     KPM provenance: "phy-trace".
 *   - The remaining scale-out UEs get their SINR from a TR 38.821-style CNR
 *     link budget evaluated over the SAME live geometry, with the SAME radio
 *     constants the anchored cells actually run (EIRP, S-band carrier, 50 MHz,
 *     5 dB UE noise figure, mmwave default 8x8/2x2 array gains). KPM
 *     provenance: "geometry-budget". No synthetic/sine formulas anywhere.
 *   - Terrestrial gNB measurements (TN-NTN steering input) come from a
 *     TR 38.901 UMa-NLOS budget over the real UE—gNB distance
 *     (provenance "geometry-budget").
 *
 * What is ABSTRACTED (read before citing):
 *   - E2 transport: E2AP-over-SCTP is NOT simulated. Indications and RC
 *     actions are delay-modeled events — one feeder-link delay each way — and
 *     this example sets AlignToControlLoop=true on every E2 node, so the
 *     modeled loop is measure -> feeder -> RIC loop tick -> feeder -> apply.
 *     Same substitution ns-3 mainline applies to S1-AP/X2-AP.
 *   - Scale-out radio: budget-derived, interference-free CNR (no per-packet
 *     PHY). mmwave does not scale to 66 cells x N UEs; the anchored cell(s)
 *     provide the measured calibration (the accepted ns-O-RAN pattern).
 *
 * Every KPM row injected into the RIC is also logged with its provenance to
 * <outputDir>/kpm_feed.csv (column "provenance": phy-trace | geometry-budget).
 *
 * Usage:
 *   ./ns3 run "oran-ntn-full-scenario --duration=120 --numUes=30 --numRealCells=1"
 *   ./ns3 run "oran-ntn-full-scenario --numRealCells=0 --duration=400"  (budget +
 *       Space-RIC over a full pass, no real cell, ~45 s wall, ~500 autonomous HOs)
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/oran-ntn-a1-interface.h"
#include "ns3/oran-ntn-e2-interface.h"
#include "ns3/oran-ntn-helper.h"
#include "ns3/oran-ntn-near-rt-ric.h"
#include "ns3/oran-ntn-space-ric.h"
#include "ns3/oran-ntn-types.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <vector>

using namespace ns3;
using ns3::ntncon::Sgp4MobilityModel;
using ns3::ntncon::WalkerConfig;
using ns3::ntncon::WalkerConstellation;

NS_LOG_COMPONENT_DEFINE("OranNtnFullScenario");

// ============================================================================
//  Simulation parameters
// ============================================================================

struct SimParams
{
    double duration = 120.0;       // seconds (keep <=120 s: 1 real mmwave cell ~
                                   // 1.4x real time; long enough that serving
                                   // sats cross the service elevation and the
                                   // on-board RIC is exercised)
    uint32_t numPlanes = 6;
    uint32_t satsPerPlane = 11;
    double altitudeKm = 550.0;
    double inclinationDeg = 53.0;
    uint32_t numTnGnbs = 5;
    uint32_t numUes = 30;
    uint32_t numRealCells = 1;     // anchored measured mmwave cells (sats 0..N-1)
    uint32_t realUesPerCell = 3;   // UEs on each anchored cell's real radio
    double measuredWindowS = 90.0; // real mmwave cell carries traffic for this
                                   // long; the geometry+Space-RIC feed runs the
                                   // full (longer) duration so passes set
    double kpmInterval = 0.1;      // seconds
    double minElevDeg = 10.0;      // hard coverage floor (reacquire below this)
    double serviceElevDeg = 30.0;  // high-quality service elevation the TTE
                                   // predictor counts down to (real NTN min
                                   // service elevation, TR 38.821 VSAT cases);
                                   // the on-board RIC hands over near here
    bool stickyServing = true;     // true: hysteretic CHO serving (enables the
                                   // feeder-outage autonomy demonstration).
                                   // false: per-tick max-elevation reselection
                                   // (the full-controller LOAD configuration:
                                   // strong link, busy handover-prediction and
                                   // steering xApps, high E2 action count)
    double groundHoTteS = 20.0;    // serving TTE at which the GROUND RIC hands
                                   // a UE over (sticky/hysteretic mobility); a
                                   // stranded UE rides past this to the Space
                                   // RIC's autonomous TTE<5 s trigger
    double satEirpDbm = 55.0;      // shared by measured cells and the budget
    double freqGhz = 2.0;          // S-band (3GPP NR-NTN FR1)
    double bwMhz = 50.0;
    std::string outputDir = "oran-ntn-output";
    std::string conflictStrategy = "priority";
    bool enableSpaceRic = true;
    bool enableFederatedLearning = false;
    uint32_t feederLinkOutageStart = 200;   // seconds
    uint32_t feederLinkOutageDuration = 30; // seconds
};

// ============================================================================
//  Real-geometry KPM feed
//
//  Replaces the former synthetic generator: serving-satellite selection,
//  elevation, slant, Doppler and TTE all come from the live SGP4 + TR 38.811
//  mobility models. SINR is measured (anchored UEs) or budget-derived over
//  the real geometry (scale-out UEs); every row carries its provenance.
// ============================================================================

class RealGeometryKpmFeed
{
  public:
    RealGeometryKpmFeed(Ptr<OranNtnHelper> helper,
                        const std::vector<Ptr<Sgp4MobilityModel>>& satMobs,
                        const std::vector<Ptr<NtnTr38811MobilityModel>>& ueModels,
                        const std::vector<Vector>& tnGnbEcef,
                        const SimParams& params)
        : m_helper(helper),
          m_satMobs(satMobs),
          m_ueModels(ueModels),
          m_tnGnbEcef(tnGnbEcef),
          m_p(params),
          m_anchoredUes(std::min<uint32_t>(params.numRealCells * params.realUesPerCell,
                                           params.numUes))
    {
        std::filesystem::create_directories(m_p.outputDir);
        m_csv.open(m_p.outputDir + "/kpm_feed.csv");
        m_csv << "time_s,ue_id,gnb_id,role,sinr_db,rsrp_dbm,elev_deg,doppler_hz,"
                 "tte_s,provenance\n";
    }

    /// The real mmwave stack of the anchored cells (nullptr if numRealCells=0).
    void SetRealStack(NtnRealStackHelper* rs) { m_rs = rs; }

    /// Route per-UE KPM to the on-board Space-RICs too, so that during a feeder
    /// outage they have live measurements to make autonomous decisions on.
    void SetSpaceRics(std::vector<Ptr<OranNtnSpaceRic>>* sr) { m_spaceRics = sr; }

    uint32_t GetCoverageGapSamples() const { return m_coverageGaps; }

    void Tick()
    {
        const double t = Simulator::Now().GetSeconds();
        const uint32_t numSats = m_satMobs.size();

        // Propagate each satellite once per tick (Sgp4 position cache grain is
        // 100 ms, matching the default KPM interval).
        std::vector<Vector> satPos(numSats);
        std::vector<Vector> satVel(numSats);
        for (uint32_t s = 0; s < numSats; ++s)
        {
            satPos[s] = m_satMobs[s]->GetPosition();
            satVel[s] = m_satMobs[s]->GetVelocity();
        }

        for (uint32_t ue = 0; ue < m_p.numUes; ++ue)
        {
            const Vector uePos = m_ueModels[ue]->GetPosition();
            const Vector ueVel = m_ueModels[ue]->GetVelocity();

            // ---- REAL serving selection: max elevation over the constellation
            uint32_t best = 0;
            uint32_t second = 0;
            double bestElev = -90.0;
            double secondElev = -90.0;
            for (uint32_t s = 0; s < numSats; ++s)
            {
                const double e = ntngeo::ElevationDeg(uePos, satPos[s]);
                if (e > bestElev)
                {
                    second = best;
                    secondElev = bestElev;
                    best = s;
                    bestElev = e;
                }
                else if (e > secondElev)
                {
                    second = s;
                    secondElev = e;
                }
            }
            if (bestElev < 5.0)
            {
                ++m_coverageGaps; // real constellation gap — nothing to report
                continue;
            }

            // ---- Serving report --------------------------------------------
            uint32_t servingSat;
            double sinr;
            double rsrp;
            const char* provenance;
            // Serving TTE: computed ONCE per (ue,sat) per tick (EstimateTteS
            // mutates its elevation memory, so a second call the same tick
            // would read a zero rate). chosenTte carries it from the sticky
            // decision to the report when the UE stays on its serving sat.
            double chosenTte = -1.0;
            if (ue < m_anchoredUes && m_rs)
            {
                // Anchored UE: RRC-attached to its real cell; SINR is MEASURED
                // off the mmwave PHY trace of that cell.
                servingSat = ue / m_p.realUesPerCell; // cells = sats 0..N-1
                sinr = m_rs->GetUeRecentSinrDb(ue);
                if (std::isnan(sinr))
                {
                    continue; // no PHY sample yet — never substitute a formula
                }
                rsrp = sinr - 95.0; // module convention (see real-stack scenario)
                provenance = "phy-trace";
            }
            else if (!m_p.stickyServing)
            {
                // Load-scenario serving model: per-tick max-elevation
                // reselection. Every UE always rides its best visible
                // satellite, so the link stays strong (high delivery) and the
                // frequent reselections keep the handover-prediction and
                // steering xApps busy. This is the configuration the
                // full-controller LOAD result uses.
                servingSat = best;
                m_serving[ue] = servingSat;
                sinr = BudgetSinrDb(uePos, satPos[servingSat]);
                rsrp = BudgetRxPowerDbm(uePos, satPos[servingSat]);
                provenance = "geometry-budget";
            }
            else
            {
                // Sticky (hysteretic) serving model — real CHO behavior, NOT
                // per-tick max-elevation reselection. A UE keeps its serving
                // satellite and rides it down until the predicted time-to-exit
                // falls below the ground-assisted handover threshold, then is
                // handed over to the best visible candidate. But when the
                // serving satellite's feeder link is down, the ground RIC can
                // no longer issue that handover command: the UE is STRANDED on
                // the receding satellite until the on-board (Space) RIC
                // autonomously hands it over. That stranding is what drives the
                // serving TTE below the Space RIC's autonomous trigger (TTE<5 s)
                // — the on-orbit autonomy demonstration. This is the
                // configuration the AUTONOMY result uses.
                uint32_t serv;
                auto sit = m_serving.find(ue);
                if (sit != m_serving.end())
                {
                    const uint32_t prev = sit->second;
                    const double prevElev = ntngeo::ElevationDeg(uePos, satPos[prev]);
                    if (prevElev < m_p.minElevDeg)
                    {
                        serv = best; // serving has set below the horizon — reacquire
                    }
                    else
                    {
                        const bool feederDown =
                            (m_spaceRics && prev < m_spaceRics->size() &&
                             (*m_spaceRics)[prev] && (*m_spaceRics)[prev]->IsAutonomous());
                        const double prevTte = EstimateTteS(ue, prev, prevElev);
                        if (!feederDown && prevTte < m_p.groundHoTteS && best != prev &&
                            bestElev > prevElev)
                        {
                            serv = best; // ground-assisted CHO before the link sets
                        }
                        else
                        {
                            serv = prev;        // stay attached (or stranded if feederDown)
                            chosenTte = prevTte; // reuse — do not re-mutate this tick
                        }
                    }
                }
                else
                {
                    serv = best; // first acquisition
                }
                servingSat = serv;
                m_serving[ue] = servingSat;
                sinr = BudgetSinrDb(uePos, satPos[servingSat]);
                rsrp = BudgetRxPowerDbm(uePos, satPos[servingSat]);
                provenance = "geometry-budget";
            }

            const double elev = ntngeo::ElevationDeg(uePos, satPos[servingSat]);
            const double doppler = ntngeo::DopplerHz(uePos, ueVel, satPos[servingSat],
                                                     satVel[servingSat], m_p.freqGhz * 1e9);
            const double tte =
                (chosenTte >= 0.0) ? chosenTte : EstimateTteS(ue, servingSat, elev);
            m_helper->InjectKpmReport(servingSat + 1, ue, sinr, rsrp, tte, elev, doppler);
            LogRow(t, ue, servingSat + 1, "serving", sinr, rsrp, elev, doppler, tte,
                   provenance);

            // ---- Candidate report: best VISIBLE satellite that isn't serving -
            uint32_t cand = (best != servingSat) ? best : second;
            double candElev = (best != servingSat) ? bestElev : secondElev;
            bool haveCand = (candElev > 5.0 && cand != servingSat);
            double cSinr = 0.0;
            double cElev = 0.0;
            double cTte = 0.0;
            if (haveCand)
            {
                cSinr = BudgetSinrDb(uePos, satPos[cand]);
                const double cRsrp = BudgetRxPowerDbm(uePos, satPos[cand]);
                cElev = ntngeo::ElevationDeg(uePos, satPos[cand]);
                const double cDoppler = ntngeo::DopplerHz(uePos, ueVel, satPos[cand],
                                                          satVel[cand], m_p.freqGhz * 1e9);
                cTte = EstimateTteS(ue, cand, cElev);
                m_helper->InjectKpmReport(cand + 1, ue, cSinr, cRsrp, cTte, cElev,
                                          cDoppler);
                LogRow(t, ue, cand + 1, "candidate", cSinr, cRsrp, cElev, cDoppler,
                       cTte, "geometry-budget");
            }

            // ---- Feed the serving sat's on-board Space-RIC (if autonomous):
            // the serving report plus a candidate on a different beam let the
            // autonomous control loop trigger handovers when the link degrades.
            if (m_spaceRics && servingSat < m_spaceRics->size())
            {
                Ptr<OranNtnSpaceRic> sric = (*m_spaceRics)[servingSat];
                if (sric && sric->IsAutonomous())
                {
                    E2KpmReport sr{};
                    sr.timestamp = t;
                    sr.gnbId = servingSat + 1;
                    sr.isNtn = true;
                    sr.ueId = ue;
                    sr.sinr_dB = sinr;
                    sr.rsrp_dBm = rsrp;
                    sr.tte_s = tte;
                    sr.elevation_deg = elev;
                    sr.doppler_Hz = doppler;
                    sr.beamId = servingSat + 1;
                    sric->ProcessLocalKpm(sr);
                    if (haveCand)
                    {
                        E2KpmReport candR{};
                        candR.timestamp = t;
                        candR.gnbId = cand + 1;
                        candR.isNtn = true;
                        candR.ueId = ue + m_p.numUes;
                        candR.sinr_dB = cSinr;
                        candR.tte_s = cTte;
                        candR.elevation_deg = cElev;
                        candR.beamId = cand + 1;
                        sric->ProcessLocalKpm(candR);
                    }
                }
            }

            // ---- Terrestrial measurement (TN-NTN steering input) ------------
            // Half the UEs are TN-capable; the report is a TR 38.901 UMa-NLOS
            // budget over the REAL distance to the nearest terrestrial gNB.
            if (ue % 2 == 0 && !m_tnGnbEcef.empty())
            {
                uint32_t tnIdx = 0;
                double tnDist = std::numeric_limits<double>::max();
                for (uint32_t g = 0; g < m_tnGnbEcef.size(); ++g)
                {
                    const double d = ntngeo::SlantRangeM(uePos, m_tnGnbEcef[g]);
                    if (d < tnDist)
                    {
                        tnDist = d;
                        tnIdx = g;
                    }
                }
                const double tnRx = TnRxPowerDbm(tnDist);
                const double tnSinr = std::min(30.0, tnRx - NoiseDbm());
                const uint32_t tnGnbId = 10001 + tnIdx;
                m_helper->InjectKpmReport(tnGnbId, ue, tnSinr, tnRx, 999.0, 90.0, 0.0);
                LogRow(t, ue, tnGnbId, "tn", tnSinr, tnRx, 90.0, 0.0, 999.0,
                       "geometry-budget");
            }
        }

        // Re-schedule (stop cleanly before Simulator::Stop so no event leaks
        // past the configured duration).
        const Time next = Seconds(m_p.kpmInterval);
        if (Simulator::Now() + next < Seconds(m_p.duration))
        {
            Simulator::Schedule(next, &RealGeometryKpmFeed::Tick, this);
        }
    }

  private:
    // mmwave defaults the anchored cells actually run: 8x8 gNB panel, 2x2 UE
    // panel (MmWaveHelper phased-array factories), 5 dB UE noise figure
    // (MmWaveUePhy::NoiseFigure). Keeping the budget on the same constants is
    // what makes "geometry-budget" rows comparable to "phy-trace" rows.
    static constexpr double kEnbArrayGainDb = 18.06; // 10*log10(64)
    static constexpr double kUeArrayGainDb = 6.02;   // 10*log10(4)
    static constexpr double kUeNoiseFigureDb = 5.0;
    static constexpr double kTnEirpDbm = 49.0;       // macro gNB
    static constexpr double kTnAntennaGainDb = 17.0;

    double NoiseDbm() const
    {
        return -174.0 + 10.0 * std::log10(m_p.bwMhz * 1e6) + kUeNoiseFigureDb;
    }

    /// Received power (dBm) of the satellite downlink at the UE — free-space
    /// (Friis) over the live slant range, as on the anchored cells' channel.
    double BudgetRxPowerDbm(const Vector& uePos, const Vector& satPos) const
    {
        const double slantM = std::max(1.0, ntngeo::SlantRangeM(uePos, satPos));
        const double fsplDb =
            20.0 * std::log10(4.0 * M_PI * slantM * (m_p.freqGhz * 1e9) / 299792458.0);
        return m_p.satEirpDbm + kEnbArrayGainDb + kUeArrayGainDb - fsplDb;
    }

    /// TR 38.821-style interference-free CNR (dB) over the live geometry.
    double BudgetSinrDb(const Vector& uePos, const Vector& satPos) const
    {
        return BudgetRxPowerDbm(uePos, satPos) - NoiseDbm();
    }

    /// Terrestrial received power (dBm): TR 38.901 UMa-NLOS path loss over the
    /// real 3D distance to the gNB.
    double TnRxPowerDbm(double dist3dM) const
    {
        const double d = std::max(10.0, dist3dM);
        const double plDb =
            13.54 + 39.08 * std::log10(d) + 20.0 * std::log10(m_p.freqGhz);
        return kTnEirpDbm + kTnAntennaGainDb - plDb;
    }

    /// TTE (s) until the satellite drops below the SERVICE elevation
    /// (serviceElevDeg, the high-quality-link handover horizon — 25-40 deg in
    /// real NTN systems, TR 38.821 VSAT cases), from the measured elevation
    /// rate over the KPM period. This is distinct from minElevDeg, the hard
    /// coverage floor below which the UE is dropped/reacquired: a UE riding its
    /// serving satellite from the service elevation down to the floor reports
    /// TTE<5 s the whole way, which is the on-board RIC's autonomous trigger.
    double EstimateTteS(uint32_t ue, uint32_t satIdx, double elevDeg)
    {
        const uint64_t key = static_cast<uint64_t>(ue) * 1000u + satIdx;
        double tte = 600.0; // rising/flat pass: bounded "long" TTE
        auto it = m_lastElev.find(key);
        if (it != m_lastElev.end())
        {
            const double rate = (elevDeg - it->second) / m_p.kpmInterval; // deg/s
            if (rate < -1e-3)
            {
                // Below the service elevation the link is already past its
                // usable horizon: clamp TTE to ~0 (still inside coverage, but
                // the on-board RIC must have handed over by now).
                tte = std::max(1.0, (elevDeg - m_p.serviceElevDeg) / -rate);
            }
        }
        m_lastElev[key] = elevDeg;
        return tte;
    }

    void LogRow(double t, uint32_t ue, uint32_t gnbId, const char* role, double sinr,
                double rsrp, double elev, double doppler, double tte,
                const char* provenance)
    {
        m_csv << std::fixed << std::setprecision(3) << t << "," << ue << "," << gnbId
              << "," << role << "," << sinr << "," << rsrp << "," << elev << ","
              << doppler << "," << tte << "," << provenance << "\n";
    }

    Ptr<OranNtnHelper> m_helper;
    std::vector<Ptr<Sgp4MobilityModel>> m_satMobs;
    std::vector<Ptr<NtnTr38811MobilityModel>> m_ueModels;
    std::vector<Vector> m_tnGnbEcef;
    SimParams m_p;
    uint32_t m_anchoredUes;
    NtnRealStackHelper* m_rs{nullptr};
    std::vector<Ptr<OranNtnSpaceRic>>* m_spaceRics{nullptr};
    std::map<uint64_t, double> m_lastElev; // (ue,sat) -> last elevation
    std::map<uint32_t, uint32_t> m_serving; // ue -> serving sat (sticky mobility)
    std::ofstream m_csv;
    uint32_t m_coverageGaps{0};
};

// ============================================================================
//  Feeder Link Outage Simulator
// ============================================================================

void
SimulateFeederLinkOutage(std::vector<Ptr<OranNtnE2Node>>& satE2Nodes,
                           std::vector<Ptr<OranNtnSpaceRic>>& spaceRics,
                           uint32_t startSat, uint32_t endSat)
{
    NS_LOG_INFO("=== FEEDER LINK OUTAGE: Satellites " << startSat << "-" << endSat
                << " losing feeder link ===");

    for (uint32_t i = startSat; i <= endSat && i < satE2Nodes.size(); i++)
    {
        satE2Nodes[i]->SetFeederLinkAvailable(false);
        if (i < spaceRics.size())
        {
            spaceRics[i]->EnterAutonomousMode();
        }
    }
}

void
RestoreFeederLink(std::vector<Ptr<OranNtnE2Node>>& satE2Nodes,
                   std::vector<Ptr<OranNtnSpaceRic>>& spaceRics,
                   uint32_t startSat, uint32_t endSat)
{
    NS_LOG_INFO("=== FEEDER LINK RESTORED: Satellites " << startSat << "-" << endSat
                << " ===");

    for (uint32_t i = startSat; i <= endSat && i < satE2Nodes.size(); i++)
    {
        satE2Nodes[i]->SetFeederLinkAvailable(true);
        if (i < spaceRics.size())
        {
            spaceRics[i]->ExitAutonomousMode();
        }
    }
}

// ============================================================================
//  Main
// ============================================================================

int
main(int argc, char* argv[])
{
    SimParams params;

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", params.duration);
    cmd.AddValue("numPlanes", "Number of orbital planes", params.numPlanes);
    cmd.AddValue("satsPerPlane", "Satellites per plane", params.satsPerPlane);
    cmd.AddValue("altitude", "Satellite altitude (km)", params.altitudeKm);
    cmd.AddValue("inclination", "Orbital inclination (deg)", params.inclinationDeg);
    cmd.AddValue("numTnGnbs", "Number of terrestrial gNBs", params.numTnGnbs);
    cmd.AddValue("numUes", "Number of UEs (anchored + scale-out)", params.numUes);
    cmd.AddValue("numRealCells",
                 "Satellites 0..N-1 carry a REAL measured mmwave cell (0 = "
                 "geometry-budget only)",
                 params.numRealCells);
    cmd.AddValue("realUesPerCell", "UEs on each anchored cell's real radio",
                 params.realUesPerCell);
    cmd.AddValue("measuredWindowS",
                 "Seconds the real mmwave cell carries traffic (phy-trace KPIs); "
                 "the geometry + Space-RIC feed runs the full duration",
                 params.measuredWindowS);
    cmd.AddValue("kpmInterval", "KPM reporting interval (s)", params.kpmInterval);
    cmd.AddValue("minElev", "Hard coverage floor — reacquire below this (deg)",
                 params.minElevDeg);
    cmd.AddValue("serviceElev",
                 "High-quality service elevation the TTE predictor counts down "
                 "to (real NTN min service elevation; the on-board RIC hands "
                 "over near here)",
                 params.serviceElevDeg);
    cmd.AddValue("stickyServing",
                 "true: hysteretic CHO serving that enables the feeder-outage "
                 "autonomy demonstration; false: per-tick max-elevation "
                 "reselection (the full-controller LOAD configuration with busy "
                 "xApps and a strong link)",
                 params.stickyServing);
    cmd.AddValue("groundHoTteS",
                 "Serving TTE (s) at which the ground RIC hands a UE over "
                 "(sticky mobility; a stranded UE rides past it to the Space "
                 "RIC's autonomous TTE<5 s trigger)",
                 params.groundHoTteS);
    cmd.AddValue("satEirpDbm", "Satellite EIRP (measured cells AND budget)",
                 params.satEirpDbm);
    cmd.AddValue("freqGhz", "Carrier frequency (GHz)", params.freqGhz);
    cmd.AddValue("bwMhz", "Bandwidth (MHz)", params.bwMhz);
    cmd.AddValue("outputDir", "Output directory", params.outputDir);
    cmd.AddValue("conflictStrategy",
                 "Conflict resolution: priority, temporal, merge, reject_lower "
                 "(alias of priority: the lower-priority action is rejected)",
                 params.conflictStrategy);
    cmd.AddValue("enableSpaceRic", "Enable Space RICs", params.enableSpaceRic);
    cmd.AddValue("enableFL", "Enable federated learning", params.enableFederatedLearning);
    cmd.Parse(argc, argv);

    const uint32_t totalSats = params.numPlanes * params.satsPerPlane;
    params.numRealCells = std::min(params.numRealCells, totalSats);
    params.realUesPerCell = std::max(1u, params.realUesPerCell);
    const uint32_t anchoredUes =
        std::min(params.numRealCells * params.realUesPerCell, params.numUes);

    std::cout << "\n"
              << "============================================================\n"
              << "  O-RAN NTN Full Scenario (real SGP4 geometry)\n"
              << "============================================================\n"
              << "  Constellation: " << params.numPlanes << " planes x "
              << params.satsPerPlane << " sats = " << totalSats
              << " satellites (Sgp4MobilityModel, Walker Delta)\n"
              << "  Altitude: " << params.altitudeKm << " km, Inclination: "
              << params.inclinationDeg << " deg\n"
              << "  Terrestrial gNBs: " << params.numTnGnbs << "\n"
              << "  UEs: " << params.numUes << " TR 38.811 (" << anchoredUes
              << " on " << params.numRealCells
              << " REAL mmwave cell(s) [phy-trace], "
              << (params.numUes - anchoredUes)
              << " scale-out [geometry-budget])\n"
              << "  Duration: " << params.duration << " s\n"
              << "  Space RIC: " << (params.enableSpaceRic ? "ON" : "OFF") << "\n"
              << "  Federated Learning: "
              << (params.enableFederatedLearning ? "ON" : "OFF") << "\n"
              << "  Conflict Strategy: " << params.conflictStrategy << "\n"
              << "  Output: " << params.outputDir << "\n"
              << "============================================================\n\n";

    // ---- Real Walker-Delta constellation: every satellite is a node under a
    //      live Sgp4MobilityModel (ECEF) ----
    WalkerConfig wcfg;
    wcfg.num_planes = params.numPlanes;
    wcfg.total_sats = totalSats;
    wcfg.altitude_km = params.altitudeKm;
    wcfg.inclination_deg = params.inclinationDeg;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = WalkerConstellation::BuildDelta(wcfg);

    NodeContainer satNodes;
    satNodes.Create(totalSats);
    std::vector<Ptr<Sgp4MobilityModel>> satMobs(totalSats);
    for (uint32_t s = 0; s < totalSats; ++s)
    {
        satMobs[s] = CreateObject<Sgp4MobilityModel>();
        satMobs[s]->SetElements(elements[s]);
        satNodes.Get(s)->AggregateObject(satMobs[s]);
    }

    // ---- TR 38.811 UEs at real ground positions ----
    // Anchored UEs cluster under their serving satellite's t=0 sub-point (the
    // mmwave cell footprint); scale-out UEs cover a wide field around sat 0 so
    // the live max-elevation selection exercises the whole constellation.
    NodeContainer ueNodes;
    ueNodes.Create(params.numUes);
    NtnTr38811MobilityHelper ueMobility(1);
    auto profile = NtnMobilityScenarios::MixedContinental();
    std::vector<Ptr<NtnTr38811MobilityModel>> ueModels(params.numUes);

    for (uint32_t c = 0; c < params.numRealCells && c * params.realUesPerCell < anchoredUes;
         ++c)
    {
        double subLat;
        double subLon;
        double subAlt;
        satMobs[c]->GetGeodetic(subLat, subLon, subAlt);
        NodeContainer cellUes;
        for (uint32_t u = c * params.realUesPerCell;
             u < std::min((c + 1) * params.realUesPerCell, anchoredUes); ++u)
        {
            cellUes.Add(ueNodes.Get(u));
        }
        auto models = ueMobility.Install(cellUes, profile, subLat - 0.03, subLat + 0.03,
                                         subLon - 0.03, subLon + 0.03);
        for (uint32_t i = 0; i < models.size(); ++i)
        {
            ueModels[c * params.realUesPerCell + i] = models[i];
        }
    }
    if (anchoredUes < params.numUes)
    {
        double fieldLat;
        double fieldLon;
        double fieldAlt;
        satMobs[0]->GetGeodetic(fieldLat, fieldLon, fieldAlt);
        NodeContainer scaleOutUes;
        for (uint32_t u = anchoredUes; u < params.numUes; ++u)
        {
            scaleOutUes.Add(ueNodes.Get(u));
        }
        auto models = ueMobility.Install(scaleOutUes, profile, fieldLat - 4.0,
                                         fieldLat + 4.0, fieldLon - 4.0, fieldLon + 4.0);
        for (uint32_t i = 0; i < models.size(); ++i)
        {
            ueModels[anchoredUes + i] = models[i];
        }
    }

    // ---- Terrestrial gNBs: fixed infrastructure at real ground positions
    //      spread across the scale-out field ----
    NodeContainer gnbNodes;
    gnbNodes.Create(params.numTnGnbs);
    std::vector<Vector> tnGnbEcef;
    {
        MobilityHelper tnMob;
        tnMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        Ptr<ListPositionAllocator> tnPos = CreateObject<ListPositionAllocator>();
        // Terrestrial macro cells co-located with the anchored UE clusters so
        // the TN-capable UEs actually sit inside terrestrial coverage (a few km
        // of a tower) and therefore have a genuine TN-vs-NTN choice for the
        // steering xApp to resolve. A macro cell has ~km range; the previous
        // 2-degree (~220 km) ring placed every UE far outside terrestrial reach,
        // so the TN link was never viable and the steering xApp never fired.
        // Anchored cell c clusters its UEs within +/-0.03 deg (~3 km) of sat c's
        // t=0 sub-point; one gNB per cell sub-point covers them, and any extra
        // towers are jittered around cell-0's cluster.
        for (uint32_t g = 0; g < params.numTnGnbs; ++g)
        {
            const uint32_t cell = (g < params.numRealCells) ? g : 0;
            double sLat;
            double sLon;
            double sAlt;
            satMobs[cell]->GetGeodetic(sLat, sLon, sAlt);
            // Deterministic small offset (<~1.5 km) so co-located towers do not
            // overlap; the first tower per cell sits on the sub-point itself.
            const double jit = (g < params.numRealCells) ? 0.0 : 0.012;
            const double ang = 2.0 * M_PI * g / std::max(1u, params.numTnGnbs);
            const Vector ecef = ntngeo::GeodeticToEcef(sLat + jit * std::sin(ang),
                                                       sLon + jit * std::cos(ang), 30.0);
            tnPos->Add(ecef);
            tnGnbEcef.push_back(ecef);
        }
        tnMob.SetPositionAllocator(tnPos);
        tnMob.Install(gnbNodes);
    }

    // ---- Create helper ----
    auto helper = CreateObject<OranNtnHelper>();
    helper->SetOutputDirectory(params.outputDir);

    // ---- Create O-RAN architecture ----
    std::cout << "[1/7] Creating Non-RT RIC..." << std::endl;
    auto nonRtRic = helper->CreateNonRtRic();

    std::cout << "[2/7] Creating Near-RT RIC..." << std::endl;
    auto nearRtRic = helper->CreateNearRtRic();

    // Connect A1 interface
    helper->ConnectA1Interface(nearRtRic, nonRtRic);

    // Set conflict resolution strategy. "reject_lower" maps to PRIORITY_BASED:
    // the conflict manager's priority resolution rejects the lower-priority
    // xApp's action (logged as resolution=priority in conflict_log.csv).
    auto cm = nearRtRic->GetConflictManager();
    if (params.conflictStrategy == "temporal")
    {
        cm->SetResolutionStrategy(ConflictResolutionStrategy::TEMPORAL);
    }
    else if (params.conflictStrategy == "merge")
    {
        cm->SetResolutionStrategy(ConflictResolutionStrategy::MERGE);
    }
    else if (params.conflictStrategy == "priority" ||
             params.conflictStrategy == "reject_lower")
    {
        cm->SetResolutionStrategy(ConflictResolutionStrategy::PRIORITY_BASED);
    }
    else
    {
        std::cerr << "Unknown --conflictStrategy '" << params.conflictStrategy
                  << "', falling back to priority\n";
        cm->SetResolutionStrategy(ConflictResolutionStrategy::PRIORITY_BASED);
    }

    // ---- Create E2 nodes ----
    std::cout << "[3/7] Creating " << totalSats << " satellite E2 nodes + "
              << params.numTnGnbs << " terrestrial E2 nodes..." << std::endl;
    auto satE2Nodes = helper->CreateSatelliteE2Nodes(satNodes, nearRtRic);
    auto tnE2Nodes = helper->CreateTerrestrialE2Nodes(gnbNodes, nearRtRic);

    // E2 loop-timing realism (audit issue #12): indications are aligned to the
    // Near-RT RIC control-loop tick instead of executing xApps inline, so the
    // modeled loop is measure -> feeder -> loop tick -> feeder -> apply.
    for (auto& n : satE2Nodes)
    {
        n->SetAttribute("AlignToControlLoop", BooleanValue(true));
    }
    for (auto& n : tnE2Nodes)
    {
        n->SetAttribute("AlignToControlLoop", BooleanValue(true));
    }
    {
        TimeValue feeder;
        TimeValue loop;
        satE2Nodes.front()->GetAttribute("FeederLinkDelay", feeder);
        satE2Nodes.front()->GetAttribute("ControlLoopPeriod", loop);
        std::cout << "      E2 loop timing: +" << feeder.Get().As(Time::MS)
                  << " feeder uplink -> next " << loop.Get().As(Time::MS)
                  << " RIC tick -> +" << feeder.Get().As(Time::MS)
                  << " feeder downlink (AlignToControlLoop=true; E2AP/SCTP "
                  << "not simulated, delay-modeled events)" << std::endl;
    }

    // ---- Create Space RICs ----
    std::vector<Ptr<OranNtnSpaceRic>> spaceRics;
    if (params.enableSpaceRic)
    {
        std::cout << "[4/7] Creating " << totalSats << " Space RICs..." << std::endl;
        spaceRics = helper->CreateSpaceRics(satNodes, params.numPlanes,
                                              params.satsPerPlane, nearRtRic);
    }
    else
    {
        std::cout << "[4/7] Space RICs disabled." << std::endl;
    }

    // ---- Generate A1 policies ----
    std::cout << "[5/7] Generating constellation A1 policies..." << std::endl;
    helper->GenerateConstellationPolicies(nonRtRic, params.numPlanes,
                                           params.satsPerPlane,
                                           params.inclinationDeg,
                                           params.altitudeKm);

    // ---- Create and start xApps ----
    std::cout << "[6/7] Creating and starting 5 xApps..." << std::endl;
    auto xapps = helper->CreateAllXapps(nearRtRic);
    helper->StartAllXapps(nearRtRic);

    // Print xApp info
    for (const auto& [name, xapp] : xapps)
    {
        std::cout << "  xApp: " << name << " (id=" << xapp->GetXappId()
                  << ", priority=" << (int)xapp->GetPriority()
                  << ", interval=" << xapp->GetDecisionInterval().As(Time::MS)
                  << ")" << std::endl;
    }

    // ---- Anchored measured cell(s): REAL mmwave NR NTN radio on the first
    //      numRealCells satellites ----
    NtnRealStackHelper rs;
    const bool haveRealCells = (params.numRealCells > 0 && anchoredUes > 0);
    if (haveRealCells)
    {
        std::cout << "[7/7] Building " << params.numRealCells
                  << " REAL mmwave cell(s) (" << anchoredUes << " UEs measured)..."
                  << std::endl;
        NodeContainer realGnbs;
        for (uint32_t c = 0; c < params.numRealCells; ++c)
        {
            realGnbs.Add(satNodes.Get(c));
        }
        NodeContainer realUes;
        for (uint32_t u = 0; u < anchoredUes; ++u)
        {
            realUes.Add(ueNodes.Get(u));
        }
        rs.SetSimTime(Seconds(params.duration));
        rs.SetOutputDir(params.outputDir);
        rs.SetRunTag("oran-ntn-full-scenario");
        rs.SetCarrierFrequencyHz(params.freqGhz * 1e9);
        rs.SetBandwidthHz(params.bwMhz * 1e6);
        rs.SetSatEirpDbm(params.satEirpDbm);
        rs.Build(realGnbs, realUes);
        // The measured mmwave cell carries traffic only for the first
        // measuredWindowS — long enough to populate phy-trace KPIs — while the
        // constellation-scale geometry + Space-RIC feed runs the full duration
        // (a complete LEO pass) so satellites actually set and the on-board RIC
        // is exercised. Running the real cell the whole pass is ~1.4x real time.
        const double measWin = std::min(params.duration, params.measuredWindowS);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet,
                          Seconds(1.0), Seconds(measWin - 0.5));
        rs.EnableAiFlowMonitor(params.outputDir + "/full_scenario");
    }
    else
    {
        std::cout << "[7/7] numRealCells=0: no measured cell, all KPM rows are "
                     "geometry-budget."
                  << std::endl;
    }

    // ---- Start the real-geometry KPM feed ----
    RealGeometryKpmFeed kpmFeed(helper, satMobs, ueModels, tnGnbEcef, params);
    if (haveRealCells)
    {
        kpmFeed.SetRealStack(&rs);
    }
    if (params.enableSpaceRic)
    {
        kpmFeed.SetSpaceRics(&spaceRics);
    }
    Simulator::Schedule(Seconds(1.0), &RealGeometryKpmFeed::Tick, &kpmFeed);

    // ---- Schedule feeder link outage event ----
    if (params.enableSpaceRic)
    {
        // The feeder-link outage is the only trigger of Space-RIC autonomy, so
        // it must fall inside the run. For short (smoke) runs where the default
        // 200 s start would never fire, scale the outage to ~30% of duration so
        // space_ric_metrics is always populated.
        double outageStart = params.feederLinkOutageStart;
        double outageDur = params.feederLinkOutageDuration;
        if (outageStart + outageDur >= params.duration)
        {
            // Span most of the run so stranded UEs cycling through their
            // ground-handover point (TTE~groundHoTteS) have time to ride down
            // to the Space RIC's autonomous TTE<5 s trigger and back.
            outageStart = params.duration * 0.2;
            outageDur = params.duration * 0.6;
        }
        // Regional gateway outage: the first ~3 planes lose their feeder link.
        // Spanning several planes (rather than just plane 0) ensures some of
        // the satellites running autonomously are also serving UEs at the end
        // of a pass, so the on-board control loop actually issues autonomous
        // handover/beam decisions (not just accrues autonomous time).
        const uint32_t outageEndSat =
            std::min(totalSats, 3u * params.satsPerPlane) - 1u;
        Simulator::Schedule(
            Seconds(outageStart),
            &SimulateFeederLinkOutage,
            std::ref(satE2Nodes), std::ref(spaceRics),
            (uint32_t)0, outageEndSat);

        Simulator::Schedule(
            Seconds(outageStart + outageDur),
            &RestoreFeederLink,
            std::ref(satE2Nodes), std::ref(spaceRics),
            (uint32_t)0, outageEndSat);
    }

    // ---- Run simulation ----
    std::cout << "\nSimulation running..." << std::endl;
    Simulator::Stop(Seconds(params.duration));
    Simulator::Run();

    // ---- Collect and write results ----
    std::cout << "\n============================================================\n"
              << "  SIMULATION COMPLETE - Writing Results\n"
              << "============================================================\n";

    if (haveRealCells)
    {
        rs.Collect();
        rs.WriteHealthReport();
        std::cout << "\n--- Measured radio (anchored cells, phy-trace) ---\n"
                  << "  mean DL SINR:   " << rs.GetMeanDlSinrDb() << " dB\n"
                  << "  mean DL TBLER:  " << rs.GetMeanDlTbler() << "\n"
                  << "  DL throughput:  " << rs.GetRxThroughputMbps() << " Mbps\n";
    }

    helper->WriteAllMetrics(nearRtRic);

    // Print summary
    auto ricMetrics = nearRtRic->GetMetrics();
    std::cout << "\n--- RIC Summary ---\n"
              << "  Active xApps: " << ricMetrics.activeXapps << "\n"
              << "  E2 Nodes: " << ricMetrics.totalE2Nodes << "\n"
              << "  Actions Processed: " << ricMetrics.totalActionsProcessed << "\n"
              << "  Conflicts: " << ricMetrics.totalConflicts << "\n"
              << "  Policy Violations: " << ricMetrics.totalPolicyViolations << "\n"
              << "  Coverage-gap KPM samples skipped: "
              << kpmFeed.GetCoverageGapSamples() << "\n";

    std::cout << "\n--- Per-xApp Summary ---\n";
    for (const auto& [name, xapp] : xapps)
    {
        auto m = xapp->GetMetrics();
        std::cout << "  " << std::left << std::setw(18) << name
                  << " | decisions: " << std::setw(6) << m.totalDecisions
                  << " | actions: " << std::setw(5) << m.successfulActions << "/"
                  << (m.successfulActions + m.failedActions)
                  << " | conflicts: " << m.conflictsEncountered
                  << " | confidence: " << std::fixed << std::setprecision(3)
                  << m.avgConfidence << "\n";
    }

    if (params.enableSpaceRic)
    {
        uint32_t totalAutonomous = 0;
        double totalAutoTime = 0;
        for (const auto& sric : spaceRics)
        {
            auto sm = sric->GetMetrics();
            totalAutonomous += sm.totalAutonomousDecisions;
            totalAutoTime += sm.totalAutonomousTime.GetSeconds();
        }
        std::cout << "\n--- Space RIC Summary ---\n"
                  << "  Total autonomous decisions: " << totalAutonomous << "\n"
                  << "  Total autonomous time: " << totalAutoTime << " s\n";
    }

    std::cout << "\nResults written to: " << params.outputDir
              << "/ (kpm_feed.csv carries the per-row provenance: "
              << "phy-trace | geometry-budget)\n"
              << std::endl;

    // Cleanup
    Simulator::Destroy();
    return 0;
}
