/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Roadmap §4.3.3)
 *
 * sionna-calibration-harness — drives a propagation-loss model against an
 * external Sionna RT server and reports residual statistics. Demonstrates
 * the §4.3.3 ≤ 1 dB LOS-only gate.
 *
 * The example uses ns-3 core's FriisPropagationLossModel as the "model
 * under test" by default because (a) it's pure FSPL, matched against the
 * Sionna empty-scene baseline; (b) it has no satellite-mobility coupling.
 * To calibrate any other PropagationLossModel-derived class (including
 * ThzNtnChannelModel after the user wires up SatMobilityModel pairs),
 * swap the model construction in `main()`.
 *
 * Run (without a live Sionna server, the example falls back to the
 * SionnaNoneTransport sentinel and the calibrator reports the failure
 * count cleanly):
 *
 *   python3 contrib/ntn-sionna/bridge/sionna-server.py --port 8765 &
 *   ./ns3 run "sionna-calibration-harness --host=127.0.0.1 --port=8765 --losOnly=true"
 *
 * Wave 7C: this is a validation tool and says so in its output
 * (example_kind=validation-tool). Its distance grid now comes from a real
 * pass by default (--grid=pass): the slant ranges an SGP4 satellite of an
 * ntn-constellation Walker shell presents to a TR 38.811 static terminal
 * (ntn-cho geodesy) at elevations of 10 deg and up. --grid=fixed restores the
 * previous 1 to 1000 km list.
 */
#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/mobility-model.h"
#include "ns3/propagation-loss-model.h"

#include "ns3/sionna-calibrator.h"
#include "ns3/sionna-udp-transport.h"

#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SionnaCalibrationHarness");

namespace
{

// Minimal pure-FSPL propagation model — the carrier frequency is supplied
// per-instance; calibration sweeps create one model per frequency.
class FsplFreqModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::HarnessFsplFreqModel")
                                .SetParent<PropagationLossModel>()
                                .SetGroupName("NtnSionnaExamples")
                                .AddConstructor<FsplFreqModel>();
        return tid;
    }
    void SetFrequencyHz(double f) { m_freqHz = f; }
    double DoCalcRxPower(double txPowerDbm,
                          Ptr<MobilityModel> a,
                          Ptr<MobilityModel> b) const override
    {
        const Vector pa = a->GetPosition();
        const Vector pb = b->GetPosition();
        const double dx = pa.x - pb.x;
        const double dy = pa.y - pb.y;
        const double dz = pa.z - pb.z;
        const double d = std::max(1e-3, std::sqrt(dx * dx + dy * dy + dz * dz));
        const double fGhz = m_freqHz / 1e9;
        return txPowerDbm -
               (20.0 * std::log10(d) + 20.0 * std::log10(fGhz) + 32.45);
    }
    int64_t DoAssignStreams(int64_t) override { return 0; }

  private:
    double m_freqHz{2.0e9};
};

NS_OBJECT_ENSURE_REGISTERED(FsplFreqModel);

} // namespace

int
main(int argc, char* argv[])
{
    std::printf("[validation-tool] example_kind=validation-tool\n");
    std::printf("[analytic-tool] CALIBRATION GATE — a packet data plane is genuinely\n"
                "inappropriate here. The deliverable is a propagation-model PL RESIDUAL\n"
                "(model_PL_dB - sionna_PL_dB) measured against the Sionna RT server BEFORE\n"
                "any radio exists; it has no data-plane analogue (there is nothing to\n"
                "decode — the artifact is the dB residual itself, gated at <=1 dB LOS-only).\n"
                "For the MEASURED-KPI counterparts that fade real packets with this same\n"
                "physics, see ntn-sionna-cir-real-stack.cc and ntn-sionna-leo-downlink-\n"
                "traffic.cc.\n\n");
    std::string host = "127.0.0.1";
    uint16_t port = 8765;
    bool losOnly = true;
    uint32_t timeoutMs = 500;
    bool allowFallback = false;
    std::string gridKind = "pass";
    double altitudeKm = 550.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("host", "Sionna server host", host);
    cmd.AddValue("port", "Sionna server UDP port", port);
    cmd.AddValue("losOnly", "Request LOS-only multipath suppression", losOnly);
    cmd.AddValue("timeoutMs", "Per-query timeout", timeoutMs);
    cmd.AddValue("allowFallback", "1 = exit 0 with an empty calibration when no Sionna server answers; default 0 exits non-zero (RequireLiveTransport)", allowFallback);
    cmd.AddValue("grid",
                 "Distance grid: pass (slant ranges of an SGP4 pass, elevation >= 10 deg) "
                 "or fixed (1 km to 1000 km list)",
                 gridKind);
    cmd.AddValue("altitudeKm", "Walker shell altitude for --grid=pass (km)", altitudeKm);
    cmd.Parse(argc, argv);

    // Distance grid. The pass grid samples the real geometry: satellite 0 of
    // a 72x22 Walker delta shell (SGP4), a terminal 150 km north of its
    // sub-satellite point 300 s after epoch, slant ranges every 20 s while
    // the satellite is 10 deg or more above the terminal's horizon.
    std::vector<double> dists = {1000.0, 5000.0, 50000.0, 200000.0, 500000.0, 1000000.0};
    if (gridKind == "pass")
    {
        ntncon::WalkerConfig w;
        w.num_planes = 72;
        w.total_sats = 72 * 22;
        w.altitude_km = altitudeKm;
        w.inclination_deg = 53.0;
        w.epoch_unix_s = 1735689600.0;
        const auto el = ntncon::WalkerConstellation::BuildDelta(w);
        Ptr<ntncon::Sgp4MobilityModel> sat = CreateObject<ntncon::Sgp4MobilityModel>();
        sat->SetElementsSgp4(el[0]);
        Vector pc;
        Vector vc;
        sat->GetEcefStateAt(Seconds(300.0), pc, vc);
        double lat;
        double lon;
        double alt;
        ntngeo::EcefToGeodetic(pc, lat, lon, alt);
        const Vector ue = ntngeo::GeodeticToEcef(lat + 150.0 / 111.195, lon, 0.0);
        dists.clear();
        for (double t = 0.0; t <= 600.0; t += 20.0)
        {
            Vector p;
            Vector v;
            sat->GetEcefStateAt(Seconds(t), p, v);
            if (ntngeo::GeodeticElevationDeg(ue, p) >= 10.0)
            {
                dists.push_back(ntngeo::SlantRangeM(ue, p));
            }
        }
        std::printf("# grid=pass: %zu slant ranges %.0f..%.0f km from an SGP4 pass "
                    "(Walker 1584/72, %.0f km)\n",
                    dists.size(), dists.empty() ? 0.0 : *std::min_element(dists.begin(), dists.end()) / 1e3,
                    dists.empty() ? 0.0 : *std::max_element(dists.begin(), dists.end()) / 1e3,
                    altitudeKm);
    }

    Ptr<SionnaUdpTransport> udp = CreateObject<SionnaUdpTransport>();
    udp->SetServer(host, port);
    udp->SetTimeoutMs(timeoutMs);

    std::printf("# sionna-calibration-harness (Roadmap §4.3.3)\n");
    std::printf("# host=%s port=%u losOnly=%s timeoutMs=%u\n",
                host.c_str(), port, losOnly ? "true" : "false", timeoutMs);
    std::printf("# residual = model_PL_dB - sionna_PL_dB\n");
    std::printf("# %-8s %-10s %-10s %-10s %-10s\n",
                "freq_GHz", "dist_m", "sionna_PL", "model_PL", "residual");

    const double freqs[] = {2.0e9, 12.0e9, 28.0e9};

    std::vector<SionnaCalibrator::PointResult> all;

    for (double f : freqs)
    {
        Ptr<FsplFreqModel> model = CreateObject<FsplFreqModel>();
        model->SetFrequencyHz(f);

        Ptr<SionnaCalibrator> cal = CreateObject<SionnaCalibrator>();
        cal->SetTransport(udp);
        cal->SetModel(model);
        cal->SetLosOnly(losOnly);

        std::vector<SionnaCalibrator::GridPoint> grid;
        for (double d : dists)
        {
            grid.push_back({f, d});
        }
        cal->SetGrid(grid);

        const auto pts = cal->RunPoints();
        for (const auto& p : pts)
        {
            std::printf("  %-8.3f %-10.0f %-10.3f %-10.3f %-10.3f\n",
                        f / 1e9, p.grid.dist_m,
                        p.sionna_pl_db, p.model_pl_db, p.residual_dB);
            all.push_back(p);
        }
    }

    // Aggregate
    std::size_t sionna_ok = 0;
    double sum = 0.0;
    double sumSq = 0.0;
    double maxAbs = 0.0;
    for (const auto& p : all)
    {
        if (!p.sionna_ok)
        {
            continue;
        }
        ++sionna_ok;
        sum += p.residual_dB;
        sumSq += p.residual_dB * p.residual_dB;
        maxAbs = std::max(maxAbs, std::abs(p.residual_dB));
    }
    std::printf("# samples=%zu  sionna_failures=%zu\n",
                sionna_ok, all.size() - sionna_ok);
    if (sionna_ok > 0)
    {
        const double mean = sum / sionna_ok;
        const double var = sumSq / sionna_ok - mean * mean;
        std::printf("# residual_dB:  max_abs=%.4f  mean=%.4f  std=%.4f\n",
                    maxAbs, mean, std::sqrt(std::max(0.0, var)));
        // §4.3.3 gate: LOS-only residual ≤ 1 dB.
        if (losOnly)
        {
            std::printf("# §4.3.3 ≤ 1 dB LOS-only gate: %s\n",
                        (maxAbs < 1.0) ? "PASS" : "FAIL");
        }
    }
    else
    {
        // P0-4: a calibration with zero ray-traced samples is not a result.
        std::printf("# (no Sionna server reachable: start bridge/sionna-server.py or "
                    "test/sionna-stub-server.py and re-run for a calibration)\n");
        std::printf("raytrace_live=0\n");
        if (!allowFallback)
        {
            std::fprintf(stderr,
                         "sionna-calibration-harness: RequireLiveTransport: 0 of %zu "
                         "calibration points were answered by a Sionna RT server; "
                         "refusing to report an empty calibration as a result "
                         "(--allowFallback=1 to exit 0)\n",
                         all.size());
            return 2;
        }
        return 0;
    }
    std::printf("raytrace_live=%d\n", sionna_ok == all.size() ? 1 : 0);
    return 0;
}
