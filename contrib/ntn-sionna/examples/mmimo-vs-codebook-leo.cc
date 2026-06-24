/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Roadmap §4.2.12)
 *
 * mmimo-vs-codebook-leo — compares a SISO baseline against an 8x8 cross-pol
 * mMIMO planar array at the satellite end as a LEO pass sweeps across the
 * UE. Both runs share the same geometry trace and the same Sionna RT server
 * — the only differences are the `tx_array` descriptor sent on each query
 * and the ground side's matching rx_array configuration. The example also
 * applies the full atmospheric cascade (P.676 gaseous + P.618/P.838 rain)
 * so the printed Rx values are comparable to a published TR 38.821 link
 * budget rather than free-space.
 *
 * Run:
 *   # terminal 1 (optional — without a server the channel falls back to FSPL)
 *   python3 contrib/ntn-sionna/bridge/sionna-server.py --port 8765
 *   # terminal 2
 *   ./ns3 run "mmimo-vs-codebook-leo --rainMmH=25 --rows=8 --cols=8"
 *
 * Output: per-sample table of (t, elevation, PL_siso, PL_mmimo, delta_db,
 * rain_db, gas_db) followed by aggregate min/max/mean of each column.
 */
#include "ns3/command-line.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/simulator.h"

#include "ns3/ns3-sionna-channel.h"
#include "ns3/ntn-atmospheric-loss-chain.h"
#include "ns3/ntn-sionna-cascade-channel.h"
#include "ns3/sionna-caching-transport.h"
#include "ns3/sionna-udp-transport.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("MmimoVsCodebookLeo");

namespace
{

struct Sample
{
    double t_s;
    double elev_deg;
    double pl_siso;
    double pl_mmimo;
    double rain_db;
    double gas_db;
};

void
TakeSample(double t,
            Ptr<NtnSionnaCascadeChannel> chSiso,
            Ptr<NtnSionnaCascadeChannel> chMmimo,
            Ptr<MobilityModel> sat,
            Ptr<MobilityModel> ue,
            std::vector<Sample>* out)
{
    const double txDbm = 30.0;
    const double rxSiso = chSiso->CalcRxPower(txDbm, sat, ue);
    const double rxMmimo = chMmimo->CalcRxPower(txDbm, sat, ue);
    auto comps = chMmimo->GetLastComponents();
    out->push_back({t,
                     comps.elevationDeg,
                     txDbm - rxSiso,
                     txDbm - rxMmimo,
                     comps.rainDb,
                     comps.gaseousDb});
}

void
PrintAggregate(const char* label, const std::vector<double>& v)
{
    if (v.empty())
    {
        return;
    }
    const double minv = *std::min_element(v.begin(), v.end());
    const double maxv = *std::max_element(v.begin(), v.end());
    const double mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    std::printf("  %-12s  min=%8.3f  max=%8.3f  mean=%8.3f  (n=%zu)\n",
                label, minv, maxv, mean, v.size());
}

} // namespace

int
main(int argc, char* argv[])
{
    std::printf("[analytic-tool] SISO-vs-MIMO LINK-BUDGET PROBE — intrinsically a full-PL\n"
                "Sionna delta. The array gain is EMBEDDED in the RT path_loss_db (the only\n"
                "difference between the two queries is the tx_array/rx_array descriptor), so\n"
                "it cannot be carried onto the real Friis plane without re-introducing a\n"
                "double-counted ~169 dB FSPL. It therefore stays an analytic comparison and\n"
                "does NOT simulate a packet data plane. For the MEASURED-radio MIMO example\n"
                "(SISO vs NxN array gain on ONE real mmwave NR cell), see\n"
                "ntn-sionna-mimo-traffic.cc.\n\n");
    std::string host = "127.0.0.1";
    uint16_t port = 8765;
    double freqHz = 12.0e9;     // Ku band
    double altKm = 550.0;        // Starlink altitude
    double rainMmH = 0.0;
    uint32_t steps = 30;
    uint32_t rows = 8;
    uint32_t cols = 8;
    uint32_t timeoutMs = 200;

    CommandLine cmd(__FILE__);
    cmd.AddValue("host", "Sionna server host", host);
    cmd.AddValue("port", "Sionna server UDP port", port);
    cmd.AddValue("freqHz", "Carrier frequency (Hz)", freqHz);
    cmd.AddValue("altKm", "Satellite altitude (km)", altKm);
    cmd.AddValue("rainMmH", "Rain rate (mm/h, 0 disables)", rainMmH);
    cmd.AddValue("steps", "Geometry steps in pass", steps);
    cmd.AddValue("rows", "PlanarArray rows", rows);
    cmd.AddValue("cols", "PlanarArray cols", cols);
    cmd.AddValue("timeoutMs", "Per-query timeout (ms)", timeoutMs);
    cmd.Parse(argc, argv);

    // --- Build SISO channel (cascade + base, no MIMO) ---
    Ptr<NtnSionnaChannel> baseSiso = CreateObject<NtnSionnaChannel>();
    baseSiso->SetServer(host, port);
    baseSiso->SetFrequencyHz(freqHz);
    baseSiso->SetTimeoutMs(timeoutMs);

    Ptr<NtnAtmosphericLossChain> chainSiso =
        CreateObject<NtnAtmosphericLossChain>();
    chainSiso->SetFrequencyHz(freqHz);
    chainSiso->SetRainRateMmH(rainMmH);

    Ptr<NtnSionnaCascadeChannel> chSiso =
        CreateObject<NtnSionnaCascadeChannel>();
    chSiso->SetSionnaChannel(baseSiso);
    chSiso->SetAtmosphericChain(chainSiso);

    // --- Build mMIMO channel (cascade + base + 8x8 cross-pol PlanarArray) ---
    Ptr<NtnSionnaChannel> baseMmimo = CreateObject<NtnSionnaChannel>();
    baseMmimo->SetServer(host, port);
    baseMmimo->SetFrequencyHz(freqHz);
    baseMmimo->SetTimeoutMs(timeoutMs);
    MimoArrayConfig txArr;
    txArr.rows = static_cast<uint8_t>(rows);
    txArr.cols = static_cast<uint8_t>(cols);
    txArr.spacing_lambda = 0.5;
    txArr.pattern = "tr38901";
    txArr.polarization = "VH";
    baseMmimo->SetTxArray(txArr);
    MimoArrayConfig rxArr;
    rxArr.rows = 2;
    rxArr.cols = 2;
    rxArr.spacing_lambda = 0.5;
    rxArr.pattern = "iso";
    rxArr.polarization = "VH";
    baseMmimo->SetRxArray(rxArr);

    Ptr<NtnAtmosphericLossChain> chainMmimo =
        CreateObject<NtnAtmosphericLossChain>();
    chainMmimo->SetFrequencyHz(freqHz);
    chainMmimo->SetRainRateMmH(rainMmH);

    Ptr<NtnSionnaCascadeChannel> chMmimo =
        CreateObject<NtnSionnaCascadeChannel>();
    chMmimo->SetSionnaChannel(baseMmimo);
    chMmimo->SetAtmosphericChain(chainMmimo);

    // --- Mobility: UE static at origin, sat sweeps overhead ---
    Ptr<ConstantPositionMobilityModel> ue =
        CreateObject<ConstantPositionMobilityModel>();
    ue->SetPosition(Vector(0, 0, 0));
    // Real SGP4 orbit projected into the local ENU frame (genuine pass).
    ns3::ntncon::WalkerConfig wcfgSat;
    wcfgSat.num_planes = 1;
    wcfgSat.total_sats = 80;
    wcfgSat.altitude_km = altKm;
    wcfgSat.inclination_deg = 53.0;
    wcfgSat.epoch_unix_s = 1735689600.0;
    const auto satElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfgSat);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(satElements[0]);
    double satSubLat, satSubLon, satSubAlt;
    satSgp4->GetGeodetic(satSubLat, satSubLon, satSubAlt);
    Ptr<NtnEnuProjectionMobilityModel> sat = CreateObject<NtnEnuProjectionMobilityModel>();
    sat->SetSource(satSgp4);
    sat->SetReference(satSubLat, satSubLon, 0.0);

    std::vector<Sample> samples;
    const Time totalSpan = Seconds(30);
    const Time dt = totalSpan / steps;
    for (uint32_t i = 1; i <= steps; ++i)
    {
        const Time when = dt * i;
        Simulator::Schedule(when,
                            &TakeSample,
                            when.GetSeconds(),
                            chSiso,
                            chMmimo,
                            Ptr<MobilityModel>(sat),
                            Ptr<MobilityModel>(ue),
                            &samples);
    }
    Simulator::Stop(totalSpan + Seconds(1));
    Simulator::Run();
    Simulator::Destroy();

    std::printf("# mmimo-vs-codebook-leo: freq=%.3f GHz alt=%.0f km rain=%.1f mm/h "
                "MIMO=%ux%u (VH cross-pol)\n",
                freqHz / 1e9, altKm, rainMmH, rows, cols);
    std::printf("# %-3s %-7s %-8s %-10s %-10s %-8s %-8s %-8s\n",
                "i", "t_s", "elev", "PL_SISO", "PL_MIMO", "delta", "rain", "gas");

    std::vector<double> delta, rain, gas;
    for (size_t i = 0; i < samples.size(); ++i)
    {
        const auto& s = samples[i];
        const double d = s.pl_siso - s.pl_mmimo;
        delta.push_back(d);
        rain.push_back(s.rain_db);
        gas.push_back(s.gas_db);
        std::printf("  %-3zu %-7.2f %-8.2f %-10.3f %-10.3f %-8.3f %-8.3f %-8.3f\n",
                    i + 1, s.t_s, s.elev_deg,
                    s.pl_siso, s.pl_mmimo, d,
                    s.rain_db, s.gas_db);
    }

    std::printf("# aggregate:\n");
    PrintAggregate("delta_dB", delta);
    PrintAggregate("rain_dB", rain);
    PrintAggregate("gas_dB", gas);
    return 0;
}
