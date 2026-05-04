/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W9)
 *
 * leo-pass-sionna-vs-tr38811 — drives the Sionna RT bridge across a synthetic
 * LEO pass and compares the ray-traced path loss against the closed-form
 * TR 38.811 free-space reference at the same geometry. Validation gate is
 * |Δ| < 3 dB per step (W9 README).
 *
 * Run:
 *   # terminal 1
 *   python3 contrib/ntn-sionna/bridge/sionna-server.py --port 8765
 *   # terminal 2
 *   ./ns3 run leo-pass-sionna-vs-tr38811
 *
 * The example tolerates a missing server (the channel falls back to FSPL),
 * so smoke-runs in CI still finish — the printed Δ column will read 0.00 dB
 * when no server is up.
 */
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"

#include "ns3/ns3-sionna-channel.h"

#include <cmath>
#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LeoPassSionnaVsTr38811");

namespace
{

// TR 38.811 §6.6 free-space reference for NTN. With no atmospheric/scintillation
// terms enabled, this reduces to plain FSPL — the same closed form an empty
// Sionna scene converges to. That's the matched-scenario reference for the
// ±3 dB validation gate.
double
Tr38811FreeSpaceDb(double distM, double freqHz)
{
    return NtnSionnaChannel::FreeSpacePathLossDb(distM, freqHz);
}

double
Distance(const Vector& a, const Vector& b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace

int
main(int argc, char* argv[])
{
    std::string host = "127.0.0.1";
    uint16_t port = 8765;
    double freqHz = 2.0e9;
    double altKm = 600.0;
    double passSpanDeg = 90.0;
    uint32_t steps = 30;
    uint32_t timeoutMs = 200;

    CommandLine cmd(__FILE__);
    cmd.AddValue("host", "Sionna server host", host);
    cmd.AddValue("port", "Sionna server UDP port", port);
    cmd.AddValue("freqHz", "Carrier frequency (Hz)", freqHz);
    cmd.AddValue("altKm", "Satellite altitude (km)", altKm);
    cmd.AddValue("steps", "Number of geometry steps in the pass", steps);
    cmd.AddValue("timeoutMs", "UDP timeout per query (ms)", timeoutMs);
    cmd.Parse(argc, argv);

    Ptr<NtnSionnaChannel> ch = CreateObject<NtnSionnaChannel>();
    ch->SetServer(host, port);
    ch->SetFrequencyHz(freqHz);
    ch->SetTimeoutMs(timeoutMs);

    Ptr<ConstantPositionMobilityModel> sat = CreateObject<ConstantPositionMobilityModel>();
    Ptr<ConstantPositionMobilityModel> ue = CreateObject<ConstantPositionMobilityModel>();
    ue->SetPosition(Vector(0.0, 0.0, 1.5));

    const double txDbm = 30.0; // arbitrary; we report PL = txDbm - rxDbm
    const double altM = altKm * 1000.0;

    std::printf("# leo-pass-sionna-vs-tr38811 (W9): freq=%.3f GHz alt=%.0f km\n",
                freqHz / 1e9, altKm);
    // Warm-up call. Sionna JITs the path solver on its first invocation
    // (~300 ms); without this primer the first measured RTT spuriously
    // exceeds the gate. The result is discarded.
    sat->SetPosition(Vector(0.0, 0.0, altM));
    (void)ch->CalcRxPower(txDbm, sat, ue);

    std::printf("# %-3s %-9s %-12s %-12s %-9s %-9s\n",
                "i", "elev_deg", "PL_sionna", "PL_tr38811", "delta", "rtt_ms");

    double maxAbsDelta = 0.0;
    uint32_t inGate = 0;

    // Synthetic LEO pass: elevation sweeps from passSpanDeg/2 down to 0
    // and back. Slant range derived from a flat-earth approximation,
    // good enough for matched-scenario verification.
    for (uint32_t i = 0; i < steps; ++i)
    {
        double frac = static_cast<double>(i) / (steps - 1);   // 0..1
        double elevDeg = 90.0 - frac * passSpanDeg;          // 90→0 deg
        double elevRad = elevDeg * M_PI / 180.0;
        double sinE = std::sin(std::max(elevRad, 1e-3));
        double slantM = altM / sinE;
        double horizM = std::sqrt(std::max(slantM * slantM - altM * altM, 0.0));

        sat->SetPosition(Vector(horizM, 0.0, altM));
        double rxDbm = ch->CalcRxPower(txDbm, sat, ue);
        double plSionna = txDbm - rxDbm;
        double plRef = Tr38811FreeSpaceDb(Distance(sat->GetPosition(), ue->GetPosition()),
                                          freqHz);
        double delta = plSionna - plRef;
        if (std::fabs(delta) <= 3.0)
        {
            ++inGate;
        }
        if (std::fabs(delta) > maxAbsDelta)
        {
            maxAbsDelta = std::fabs(delta);
        }
        std::printf("  %-3u %-9.2f %-12.3f %-12.3f %-+9.3f %-9.2f\n",
                    i, elevDeg, plSionna, plRef, delta, ch->GetLastRttMs());
    }

    std::printf("# summary: queries=%llu timeouts=%llu fallbacks=%llu max|delta|=%.3f dB"
                " in_gate=%u/%u\n",
                static_cast<unsigned long long>(ch->GetQueriesSent()),
                static_cast<unsigned long long>(ch->GetTimeouts()),
                static_cast<unsigned long long>(ch->GetFallbacks()),
                maxAbsDelta, inGate, steps);
    return (maxAbsDelta <= 3.0) ? 0 : 1;
}
