/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-sionna-mimo-traffic — two ground UEs served by the SAME passing LEO
 * satellite on a REAL mmwave NR NTN cell, one with a SISO terminal and one
 * with an N×N MIMO terminal. REAL downlink traffic flows to BOTH; the MIMO
 * UE sustains a higher MEASURED SINR — and therefore a higher AMC modulation
 * order and delivered throughput — because its rank-1 LOS NTN beamforming
 * array gain (10*log10(N_elements) dB) is applied as a real gain in its
 * packet path.
 *
 * Audit fix (2026-06 protocol-fidelity audit, channel-plugin recipe):
 * the old version computed two closed-form SNRs and drove two P2P
 * RateErrorModels through a sigmoid SnrToPer() — no packet crossed a radio.
 * Here both UEs ride ONE real cell; the MIMO terminal's array gain is a
 * per-UE PropagationLossModel in the live channel chain, so the SISO/MIMO
 * gap is MEASURED off the PHY trace (SINR, TBLER, per-UE PacketSink bytes),
 * not asserted. The ITU-R atmospheric cascade also stays in the packet path.
 * The rows×cols descriptor is the same array config a live Sionna RT server
 * consumes for true spatial synthesis; standalone runs use the closed-form
 * rank-1 array gain so no GPU is required.
 *
 * Mobility is real: SGP4 satellite (ENU-projected), fixed ground terminals.
 *
 * Quick test:  --simSeconds=40 --rows=4 --cols=4
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

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnSionnaMimoTraffic");

namespace
{

/**
 * Per-UE array gain as a real PropagationLossModel: applies the MIMO
 * terminal's beamforming gain ONLY to transmissions whose Rx (or Tx) end is
 * the target mobility model, leaving the SISO UE untouched. This keeps both
 * terminals on one shared real cell while their packet paths differ exactly
 * by the array gain.
 */
class PerUeGainLossModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("PerUeGainLossModel")
                                .SetParent<PropagationLossModel>()
                                .AddConstructor<PerUeGainLossModel>();
        return tid;
    }

    void SetTarget(Ptr<MobilityModel> target) { m_target = target; }
    void SetGainDb(double gainDb) { m_gainDb = gainDb; }

  private:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override
    {
        if (m_target && (a == m_target || b == m_target))
        {
            return txPowerDbm + m_gainDb;
        }
        return txPowerDbm;
    }

    int64_t DoAssignStreams(int64_t /*stream*/) override { return 0; }

    Ptr<MobilityModel> m_target;
    double m_gainDb{0.0};
};

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 40.0;
    double freqGHz = 12.0;
    double satEirpDbm = 62.0; // keeps the SISO UE in the AMC-sensitive region
    uint32_t rows = 4;
    uint32_t cols = 4;
    std::string radio = "nr"; // radio spine: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    std::string outputDir = "ntn-sionna-mimo-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("rows", "MIMO terminal array rows", rows);
    cmd.AddValue("cols", "MIMO terminal array cols", cols);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    const double mimoGainDb = 10.0 * std::log10(std::max(1u, rows * cols));

    std::printf("# ntn-sionna-mimo-traffic (REAL radio, SISO vs %ux%u MIMO measured)\n",
                rows, cols);
    std::printf("#   sim=%.0fs freq=%.1fGHz EIRP=%.1fdBm arrayGain=%.1fdB (rank-1 LOS)\n",
                simSeconds, freqGHz, satEirpDbm, mimoGainDb);

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(2); // UE0 = SISO, UE1 = MIMO

    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = 550.0;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(elements[0]);
    double subLat, subLon, subAlt;
    satSgp4->GetGeodetic(subLat, subLon, subAlt);
    Ptr<NtnEnuProjectionMobilityModel> satEnu = CreateObject<NtnEnuProjectionMobilityModel>();
    satEnu->SetSource(satSgp4);
    satEnu->SetReference(subLat, subLon, 0.0);
    satNodes.Get(0)->AggregateObject(satEnu);

    // Two fixed terminals at the sub-point, 500 m apart (same pass geometry).
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    uePos->Add(Vector(0.0, 0.0, 1.5));   // SISO
    uePos->Add(Vector(500.0, 0.0, 1.5)); // MIMO
    mob.SetPositionAllocator(uePos);
    mob.Install(ueNodes);

    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-sionna-mimo-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);

    // ITU-R atmospheric excess in the shared packet path.
    Ptr<NtnAtmosphericLossChain> chain = CreateObject<NtnAtmosphericLossChain>();
    chain->SetFrequencyHz(freqGHz * 1e9);
    Ptr<NtnAtmosphericPropagationLossModel> atmo =
        CreateObject<NtnAtmosphericPropagationLossModel>();
    atmo->SetChain(chain);
    rs.AddExtraPropagationLoss(atmo);

    // The MIMO terminal's array gain, applied ONLY to UE1's packet path.
    Ptr<PerUeGainLossModel> mimoGain = CreateObject<PerUeGainLossModel>();
    mimoGain->SetTarget(ueNodes.Get(1)->GetObject<MobilityModel>());
    mimoGain->SetGainDb(mimoGainDb);
    rs.AddExtraPropagationLoss(mimoGain);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("ntn-sionna-mimo-traffic"); // WS2 KPM series (TS 28.552 names)

    std::printf("# %5s  %9s  %9s  %9s  %9s  %10s  %10s\n",
                "t_s", "sisoSinr", "mimoSinr", "sisoTbler", "mimoTbler",
                "sisoMbps", "mimoMbps");

    uint64_t lastRx0 = 0, lastRx1 = 0;
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [&rs, &lastRx0, &lastRx1](Time now) {
            const uint64_t rx0 = rs.GetUeRxBytes(0);
            const uint64_t rx1 = rs.GetUeRxBytes(1);
            const double mbps0 = (rx0 - lastRx0) * 8.0 / 1e6;
            const double mbps1 = (rx1 - lastRx1) * 8.0 / 1e6;
            lastRx0 = rx0;
            lastRx1 = rx1;
            std::printf("  %5.1f  %9.2f  %9.2f  %9.3f  %9.3f  %10.3f  %10.3f\n",
                        now.GetSeconds(), rs.GetUeRecentSinrDb(0),
                        rs.GetUeRecentSinrDb(1), rs.GetUeRecentTbler(0),
                        rs.GetUeRecentTbler(1), mbps0, mbps1);
        });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    const double sisoSinr = rs.GetUeMeanSinrDb(0);
    const double mimoSinr = rs.GetUeMeanSinrDb(1);
    const double sisoMbps = rs.GetUeRxBytes(0) * 8.0 / simSeconds / 1e6;
    const double mimoMbps = rs.GetUeRxBytes(1) * 8.0 / simSeconds / 1e6;
    std::printf("# === summary ===  MEASURED SISO SINR=%.2f dB / %.3f Mbps  vs  "
                "MIMO SINR=%.2f dB / %.3f Mbps  (measured gap %.2f dB, array gain "
                "%.1f dB)\n",
                sisoSinr, sisoMbps, mimoSinr, mimoMbps, mimoSinr - sisoSinr,
                mimoGainDb);

    Simulator::Destroy();
    return 0;
}
