/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Roadmap §4.2.12)
 *
 * ris-assisted-leo-link — demonstrates Roadmap §4.2.3 RIS support.
 *
 * Geometry: a UE on the ground at the origin, an LEO satellite passing
 * overhead at 550 km, and a Reconfigurable Intelligent Surface mounted on
 * a building 700 m away at 50 m altitude. The RIS focal point is steered
 * at the UE. We compare the link budget with and without the RIS by
 * running TWO Sionna queries per geometry sample on the same UDP server:
 *
 *   1. baseline: no `ris` field → Sionna only computes the direct path
 *   2. RIS:      `ris` field with 32×32 elements, focus phase profile
 *
 * The before/after Rx-power difference is the RIS gain. Without a live
 * Sionna server the mock-style FSPL fall-back applies and the gain
 * collapses to ~0 dB; with a live server you'll see the focused
 * reflection contribute several dB of additional received power.
 *
 * Run:
 *   python3 contrib/ntn-sionna/bridge/sionna-server.py --port 8765 &
 *   ./ns3 run "ris-assisted-leo-link --freqHz=28e9 --rows=32 --cols=32"
 */
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/simulator.h"

#include "ns3/ns3-sionna-channel.h"
#include "ns3/ntn-atmospheric-loss-chain.h"
#include "ns3/ntn-sionna-cascade-channel.h"
#include "ns3/sionna-transport.h"
#include "ns3/sionna-udp-transport.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("RisAssistedLeoLink");

namespace
{

struct Sample
{
    double t_s;
    double elev_deg;
    double rx_noris_dbm;
    double rx_ris_dbm;
};

void
Tick(double t,
      Ptr<NtnSionnaCascadeChannel> chNoRis,
      Ptr<NtnSionnaCascadeChannel> chRis,
      Ptr<MobilityModel> sat,
      Ptr<MobilityModel> ue,
      std::vector<Sample>* out)
{
    const double txDbm = 30.0;
    const double rxNo = chNoRis->CalcRxPower(txDbm, sat, ue);
    const double rxYes = chRis->CalcRxPower(txDbm, sat, ue);
    out->push_back({t, chRis->GetLastComponents().elevationDeg, rxNo, rxYes});
}

} // namespace

int
main(int argc, char* argv[])
{
    std::string host = "127.0.0.1";
    uint16_t port = 8765;
    double freqHz = 28.0e9;
    double altKm = 550.0;
    double rainMmH = 0.0;
    uint32_t steps = 30;
    uint32_t risRows = 32;
    uint32_t risCols = 32;
    double risPosX = 706.5; // halfway between UE and sub-sat ground point
    double risPosZ = 50.0;
    std::string phaseProfile = "focus";
    uint32_t timeoutMs = 200;

    CommandLine cmd(__FILE__);
    cmd.AddValue("host", "Sionna server host", host);
    cmd.AddValue("port", "Sionna server UDP port", port);
    cmd.AddValue("freqHz", "Carrier frequency (Hz)", freqHz);
    cmd.AddValue("altKm", "Satellite altitude (km)", altKm);
    cmd.AddValue("rainMmH", "Rain rate (mm/h)", rainMmH);
    cmd.AddValue("steps", "Geometry steps", steps);
    cmd.AddValue("rows", "RIS rows", risRows);
    cmd.AddValue("cols", "RIS cols", risCols);
    cmd.AddValue("risPosX", "RIS x (m)", risPosX);
    cmd.AddValue("risPosZ", "RIS altitude (m)", risPosZ);
    cmd.AddValue("phaseProfile", "RIS phase profile (focus/flat/random)",
                  phaseProfile);
    cmd.AddValue("timeoutMs", "Per-query timeout (ms)", timeoutMs);
    cmd.Parse(argc, argv);

    // --- No-RIS channel ---
    Ptr<NtnSionnaChannel> baseNoRis = CreateObject<NtnSionnaChannel>();
    baseNoRis->SetServer(host, port);
    baseNoRis->SetFrequencyHz(freqHz);
    baseNoRis->SetTimeoutMs(timeoutMs);

    Ptr<NtnAtmosphericLossChain> chainNoRis =
        CreateObject<NtnAtmosphericLossChain>();
    chainNoRis->SetFrequencyHz(freqHz);
    chainNoRis->SetRainRateMmH(rainMmH);

    Ptr<NtnSionnaCascadeChannel> chNoRis =
        CreateObject<NtnSionnaCascadeChannel>();
    chNoRis->SetSionnaChannel(baseNoRis);
    chNoRis->SetAtmosphericChain(chainNoRis);

    // --- RIS-assisted channel ---
    Ptr<NtnSionnaChannel> baseRis = CreateObject<NtnSionnaChannel>();
    baseRis->SetServer(host, port);
    baseRis->SetFrequencyHz(freqHz);
    baseRis->SetTimeoutMs(timeoutMs);
    RisConfig ris;
    ris.pos_x = risPosX;
    ris.pos_y = 0.0;
    ris.pos_z = risPosZ;
    ris.normal_x = 0.0;
    ris.normal_y = 0.0;
    ris.normal_z = 1.0;
    ris.rows = static_cast<uint16_t>(risRows);
    ris.cols = static_cast<uint16_t>(risCols);
    ris.spacing_lambda = 0.5;
    ris.phase_profile = phaseProfile;
    ris.focal_x = 0.0;
    ris.focal_y = 0.0;
    ris.focal_z = 0.0;
    baseRis->SetRis(ris);

    Ptr<NtnAtmosphericLossChain> chainRis =
        CreateObject<NtnAtmosphericLossChain>();
    chainRis->SetFrequencyHz(freqHz);
    chainRis->SetRainRateMmH(rainMmH);

    Ptr<NtnSionnaCascadeChannel> chRis =
        CreateObject<NtnSionnaCascadeChannel>();
    chRis->SetSionnaChannel(baseRis);
    chRis->SetAtmosphericChain(chainRis);

    // --- Mobility ---
    Ptr<ConstantPositionMobilityModel> ue =
        CreateObject<ConstantPositionMobilityModel>();
    ue->SetPosition(Vector(0, 0, 0));
    Ptr<ConstantVelocityMobilityModel> sat =
        CreateObject<ConstantVelocityMobilityModel>();
    sat->SetPosition(Vector(-300e3, 0, altKm * 1000.0));
    sat->SetVelocity(Vector(20e3, 0, 0));

    std::vector<Sample> samples;
    const Time totalSpan = Seconds(30);
    const Time dt = totalSpan / steps;
    for (uint32_t i = 1; i <= steps; ++i)
    {
        const Time when = dt * i;
        Simulator::Schedule(when,
                            &Tick,
                            when.GetSeconds(),
                            chNoRis,
                            chRis,
                            Ptr<MobilityModel>(sat),
                            Ptr<MobilityModel>(ue),
                            &samples);
    }
    Simulator::Stop(totalSpan + Seconds(1));
    Simulator::Run();
    Simulator::Destroy();

    std::printf("# ris-assisted-leo-link: freq=%.3f GHz alt=%.0f km RIS=%ux%u "
                "phase=%s focal_xyz=(%.1f,%.1f,%.1f)\n",
                freqHz / 1e9, altKm, risRows, risCols,
                phaseProfile.c_str(), ris.focal_x, ris.focal_y, ris.focal_z);
    std::printf("# %-3s %-7s %-8s %-12s %-12s %-9s\n",
                "i", "t_s", "elev", "rx_noRIS", "rx_RIS", "gain_dB");
    std::vector<double> gains;
    for (size_t i = 0; i < samples.size(); ++i)
    {
        const auto& s = samples[i];
        const double gain = s.rx_ris_dbm - s.rx_noris_dbm;
        gains.push_back(gain);
        std::printf("  %-3zu %-7.2f %-8.2f %-12.3f %-12.3f %-9.3f\n",
                    i + 1, s.t_s, s.elev_deg,
                    s.rx_noris_dbm, s.rx_ris_dbm, gain);
    }
    if (!gains.empty())
    {
        const double mn = *std::min_element(gains.begin(), gains.end());
        const double mx = *std::max_element(gains.begin(), gains.end());
        const double mean =
            std::accumulate(gains.begin(), gains.end(), 0.0) / gains.size();
        std::printf("# RIS gain (n=%zu)  min=%.3f  max=%.3f  mean=%.3f dB\n",
                    gains.size(), mn, mx, mean);
    }
    return 0;
}
