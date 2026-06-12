/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-sionna-constellation-handover-traffic — a ground UE is served by a
 * Walker constellation: same-plane satellites cross the sky in sequence and
 * the UE always selects the one with the best link. REAL traffic flows
 * continuously on a REAL mmwave NR NTN cell; as the serving satellite recedes
 * and a neighbour rises, the serving selection flips (hand-over) and is
 * logged with genuine orbital timing.
 *
 * Audit fix (2026-06 protocol-fidelity audit): the old version ran
 * one P2P link per satellite with sigmoid SnrToPer() RateErrorModels — no
 * packet crossed a radio, and the satellites were ConstantVelocity
 * placeholders. Here every satellite flies a genuine SGP4 orbit; the radio
 * anchor satellite carries the real cell whose DL SINR is MEASURED off the
 * PHY trace, and each candidate's link quality is predicted from the SAME
 * measurement via the real ephemeris Friis ratio
 *     pred_i = measured + 20*log10(anchorSlant / slant_i)
 * (the standard NTN measurement-projection used by the cho/oran modules).
 * The hand-over decision rides measured radio + real orbital geometry; the
 * ITU-R atmospheric cascade stays in the packet path.
 *
 * Quick test:  --simSeconds=60 --hysteresisDb=1
 */
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-atmospheric-loss-chain.h"
#include "ns3/ntn-atmospheric-propagation-loss-model.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnSionnaConstellationHandoverTraffic");

namespace
{

double
DistM(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double
ElevDegEnu(const Vector& gnd, const Vector& sat)
{
    const double dx = sat.x - gnd.x;
    const double dy = sat.y - gnd.y;
    const double dz = sat.z - gnd.z;
    const double horiz = std::max(std::sqrt(dx * dx + dy * dy), 1e-3);
    return std::atan2(dz, horiz) * 180.0 / M_PI;
}

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 60.0;
    double freqGHz = 12.0;
    double satEirpDbm = 75.0;
    uint32_t numSats = 3; // serving + trailing/leading same-plane neighbours
    double hysteresisDb = 1.0;
    std::string outputDir = "ntn-sionna-constellation-handover-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("numSats", "Number of tracked satellites (>=2)", numSats);
    cmd.AddValue("hysteresisDb", "Hand-over hysteresis (dB)", hysteresisDb);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);
    numSats = std::max(2u, numSats);

    std::printf("# ntn-sionna-constellation-handover-traffic (REAL radio + SGP4 "
                "constellation)\n");
    std::printf("#   sim=%.0fs freq=%.1fGHz EIRP=%.1fdBm sats=%u hysteresis=%.1fdB\n",
                simSeconds, freqGHz, satEirpDbm, numSats, hysteresisDb);

    // Same-plane Walker: element 0 is at zenith over the UE at t=0 and
    // recedes; its plane neighbours (elements 1, 79, 2, ...) trail/lead by
    // 4.5 deg (~550 km) and rise as it sets.
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = 550.0;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);

    NodeContainer satNodes;
    satNodes.Create(1); // the radio anchor (real cell)
    NodeContainer gndNodes;
    gndNodes.Create(1);

    // Tracked satellites: 0 (anchor), then alternating neighbours 1, 79, 2...
    std::vector<uint32_t> satIdx;
    satIdx.push_back(0);
    for (uint32_t k = 1; satIdx.size() < numSats; ++k)
    {
        satIdx.push_back(k);
        if (satIdx.size() < numSats)
        {
            satIdx.push_back(80 - k);
        }
    }

    Ptr<ns3::ntncon::Sgp4MobilityModel> anchorSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    anchorSgp4->SetElements(elements[0]);
    double subLat, subLon, subAlt;
    anchorSgp4->GetGeodetic(subLat, subLon, subAlt);

    std::vector<Ptr<NtnEnuProjectionMobilityModel>> satMob(numSats);
    for (uint32_t i = 0; i < numSats; ++i)
    {
        Ptr<ns3::ntncon::Sgp4MobilityModel> sgp4 =
            (i == 0) ? anchorSgp4 : CreateObject<ns3::ntncon::Sgp4MobilityModel>();
        if (i != 0)
        {
            sgp4->SetElements(elements[satIdx[i]]);
        }
        satMob[i] = CreateObject<NtnEnuProjectionMobilityModel>();
        satMob[i]->SetSource(sgp4);
        satMob[i]->SetReference(subLat, subLon, 0.0);
    }
    satNodes.Get(0)->AggregateObject(satMob[0]);

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> gndPos = CreateObject<ListPositionAllocator>();
    gndPos->Add(Vector(0.0, 0.0, 1.5));
    mob.SetPositionAllocator(gndPos);
    mob.Install(gndNodes);

    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-sionna-constellation-handover-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, gndNodes);

    Ptr<NtnAtmosphericLossChain> chain = CreateObject<NtnAtmosphericLossChain>();
    chain->SetFrequencyHz(freqGHz * 1e9);
    Ptr<NtnAtmosphericPropagationLossModel> atmo =
        CreateObject<NtnAtmosphericPropagationLossModel>();
    atmo->SetChain(chain);
    rs.AddExtraPropagationLoss(atmo);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("ntn-sionna-constellation-handover-traffic"); // WS2 KPM series (TS 28.552 names)

    std::printf("# %5s  %7s  %8s  %8s  %8s  %9s\n",
                "t_s", "serving", "servSinr", "bestSinr", "tbler", "goodput");

    Ptr<MobilityModel> gndMob = gndNodes.Get(0)->GetObject<MobilityModel>();
    uint32_t serving = 0;
    uint32_t handovers = 0;
    uint64_t lastRx = 0;
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [&rs, &satMob, &satIdx, gndMob, &serving, &handovers, &lastRx, hysteresisDb,
         numSats](Time now) {
            const Vector g = gndMob->GetPosition();
            const double measured = rs.GetUeRecentSinrDb(0); // anchor sat, MEASURED
            if (std::isnan(measured))
            {
                return;
            }
            const double anchorSlant = DistM(g, satMob[0]->GetPosition());

            // Project the one real measurement onto every tracked satellite
            // through the real ephemeris Friis ratio.
            std::vector<double> pred(numSats);
            for (uint32_t i = 0; i < numSats; ++i)
            {
                const double slant = DistM(g, satMob[i]->GetPosition());
                const double elev = ElevDegEnu(g, satMob[i]->GetPosition());
                pred[i] = (elev > 5.0)
                              ? measured + 20.0 * std::log10(anchorSlant / slant)
                              : -1e9; // below the service mask
            }

            uint32_t best = serving;
            for (uint32_t i = 0; i < numSats; ++i)
            {
                if (pred[i] > pred[best])
                {
                    best = i;
                }
            }
            if (best != serving && pred[best] > pred[serving] + hysteresisDb)
            {
                ++handovers;
                std::printf("  %5.1f  HANDOVER sat%u(WALKER-0-%u) -> sat%u(WALKER-0-%u) "
                            " (%.2f -> %.2f dB, real orbital crossover)\n",
                            now.GetSeconds(), serving, satIdx[serving], best,
                            satIdx[best], pred[serving], pred[best]);
                serving = best;
            }

            const uint64_t rx = rs.GetUeRxBytes(0);
            const double mbps = (rx - lastRx) * 8.0 / 1e6;
            lastRx = rx;
            std::printf("  %5.1f  sat%-4u  %8.2f  %8.2f  %8.3f  %9.3f\n",
                        now.GetSeconds(), serving, pred[serving],
                        pred[best > serving ? best : serving], rs.GetUeRecentTbler(0),
                        mbps);
        });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::printf("# === summary ===  hand-overs=%u  measured cell SINR=%.2f dB "
                "TBLER=%.4f throughput=%.3f Mbps (serving selection on measured "
                "radio + real SGP4 geometry)\n",
                handovers, rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(),
                rs.GetRxThroughputMbps());

    Simulator::Destroy();
    return 0;
}
