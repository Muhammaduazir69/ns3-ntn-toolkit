/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Roadmap §4.2.12)
 *
 * city-block-4ue-cache — AODT-style 4-UE city block scenario that
 * demonstrates the §4.2.4 caching transport and the §4.2.7 atmospheric
 * cascade simultaneously.
 *
 * Geometry: 4 UEs arranged on a 200 m × 200 m grid (NE, NW, SE, SW
 * corners), one LEO satellite at 550 km altitude moving in +x. Each UE
 * has its own NtnSionnaCascadeChannel sharing a single
 * SionnaCachingTransport on top of a SionnaUdpTransport.
 *
 * SIONNA-07: the satellite is a real SGP4 element and moves about 760 m per
 * 100 ms sample, so with the default 50 m spatial grid its quantized cell
 * changes on every sample and the cache key (which includes the satellite
 * cell) never repeats: expect a hit rate near 0 at the defaults, not the
 * ~80% this header used to promise for a slow placeholder satellite. Hits
 * appear when the sample cadence is short against the grid (for example
 * --spatialResM=1000), at the cost of binning UEs 200 m apart together.
 *
 * Run:
 *   python3 contrib/ntn-sionna/bridge/sionna-server.py --port 8765 &
 *   ./ns3 run "city-block-4ue-cache --freqHz=2e9 --steps=60"
 */
#include "ns3/ntn-dataset-writer.h"
#include "ns3/command-line.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"

#include <iostream>
#include "ns3/simulator.h"

#include "ns3/ns3-sionna-channel.h"
#include "ns3/ntn-atmospheric-loss-chain.h"
#include "ns3/ntn-sionna-cascade-channel.h"
#include "ns3/sionna-caching-transport.h"
#include "ns3/sionna-udp-transport.h"

#include <array>
#include <cstdio>
#include <vector>

using namespace ns3;


// WF-12: the channel this run used, kept at file scope so main() can print the
// provenance of its numbers. GetFallbacks() existed and no example read it, so on
// a host without Sionna every run here printed Sionna framing over pure
// free-space results with nothing in the output to show it.
static Ptr<NtnSionnaChannel> base;

NS_LOG_COMPONENT_DEFINE("CityBlock4ueCache");

namespace
{

struct Sample
{
    double t_s;
    std::array<double, 4> rx_dbm;
    uint64_t hits;
    uint64_t misses;
};

void
Tick(double t,
      Ptr<NtnSionnaCascadeChannel> chA,
      Ptr<NtnSionnaCascadeChannel> chB,
      Ptr<NtnSionnaCascadeChannel> chC,
      Ptr<NtnSionnaCascadeChannel> chD,
      Ptr<MobilityModel> sat,
      Ptr<MobilityModel> ueA,
      Ptr<MobilityModel> ueB,
      Ptr<MobilityModel> ueC,
      Ptr<MobilityModel> ueD,
      Ptr<SionnaCachingTransport> cache,
      std::vector<Sample>* out)
{
    const double txDbm = 30.0;
    Sample s{t,
              {chA->CalcRxPower(txDbm, sat, ueA),
               chB->CalcRxPower(txDbm, sat, ueB),
               chC->CalcRxPower(txDbm, sat, ueC),
               chD->CalcRxPower(txDbm, sat, ueD)},
              cache->GetHits(),
              cache->GetMisses()};
    out->push_back(s);
}

} // namespace

int
main(int argc, char* argv[])
{
    std::printf("[analytic-tool] TRANSPORT CACHING BENCHMARK — the headline KPI is the\n"
                "SionnaCachingTransport cache-HIT RATIO under 4 co-located UE queries sharing\n"
                "one cache, NOT a radio KPI. Converting to a packet plane would obscure the\n"
                "cache-hit deliverable (its actual point). It therefore does NOT simulate a\n"
                "packet data plane. For the MEASURED-radio multi-UE counterpart (numUes=4 on\n"
                "a real mmwave NR cell), see ntn-sionna-cir-real-stack.cc.\n\n");
    std::string host = "127.0.0.1";
    uint16_t port = 8765;
    double freqHz = 2.0e9;
    double altKm = 550.0;
    double rainMmH = 0.0;
    uint32_t steps = 60;
    uint32_t timeoutMs = 200;
    bool allowFallback = false;
    double spatialResM = 50.0;
    uint64_t timeBucketUs = 500000; // 500 ms

    CommandLine cmd(__FILE__);
    cmd.AddValue("host", "Sionna server host", host);
    cmd.AddValue("port", "Sionna server UDP port", port);
    cmd.AddValue("freqHz", "Carrier frequency (Hz)", freqHz);
    cmd.AddValue("altKm", "Satellite altitude (km)", altKm);
    cmd.AddValue("rainMmH", "Rain rate (mm/h)", rainMmH);
    cmd.AddValue("steps", "Geometry steps", steps);
    cmd.AddValue("timeoutMs", "Per-query timeout (ms)", timeoutMs);
    cmd.AddValue("spatialResM", "Cache spatial grid (m)", spatialResM);
    cmd.AddValue("timeBucketUs", "Cache time bucket (us)", timeBucketUs);
    cmd.AddValue("allowFallback", "1 = run free space (labeled) when no Sionna server answers; default 0 aborts (RequireLiveTransport)", allowFallback);
    cmd.Parse(argc, argv);
    // P0-4 (Wave 2 M3): no silent free-space fallback for a ray-traced
    // headline. RequireLiveTransport is ON unless --allowFallback=1; without
    // a Sionna RT server the first query aborts the run (non-zero exit).
    if (!allowFallback)
    {
        std::printf("# RequireLiveTransport=1: the run aborts if the Sionna RT server at "
                    "the configured host:port does not answer (start "
                    "bridge/sionna-server.py or test/sionna-stub-server.py); "
                    "--allowFallback=1 runs closed-form free space, labeled as such\n");
    }

    // --- Shared transport stack: UDP wrapped by caching layer ---
    Ptr<SionnaUdpTransport> udp = CreateObject<SionnaUdpTransport>();
    udp->SetServer(host, port);
    udp->SetTimeoutMs(timeoutMs);

    Ptr<SionnaCachingTransport> cache = CreateObject<SionnaCachingTransport>();
    cache->SetInner(udp);
    cache->SetSpatialResolutionM(spatialResM);
    cache->SetTemporalBucketUs(timeBucketUs);
    cache->SetMaxEntries(8192);

    // --- 4 cascade channels sharing the cached transport ---
    auto buildChannel = [&]() {
        base = CreateObject<NtnSionnaChannel>();
        base->SetTransport(cache);
        base->SetFrequencyHz(freqHz);
        base->SetRequireLiveTransport(!allowFallback);
        Ptr<NtnAtmosphericLossChain> chain =
            CreateObject<NtnAtmosphericLossChain>();
        chain->SetFrequencyHz(freqHz);
        chain->SetRainRateMmH(rainMmH);
        Ptr<NtnSionnaCascadeChannel> ch =
            CreateObject<NtnSionnaCascadeChannel>();
        ch->SetSionnaChannel(base);
        ch->SetAtmosphericChain(chain);
        return ch;
    };
    // SIONNA-07: keep every UE's base channel; `base` alone is the last one
    // built, so the provenance line used to count UE SW's queries only.
    std::vector<Ptr<NtnSionnaChannel>> bases;
    auto chA = buildChannel();
    bases.push_back(base);
    auto chB = buildChannel();
    bases.push_back(base);
    auto chC = buildChannel();
    bases.push_back(base);
    auto chD = buildChannel();
    bases.push_back(base);

    // --- Mobility: 4 UEs on a 200 m × 200 m grid ---
    Ptr<ConstantPositionMobilityModel> ueA =
        CreateObject<ConstantPositionMobilityModel>();
    ueA->SetPosition(Vector(100.0, 100.0, 0)); // NE
    Ptr<ConstantPositionMobilityModel> ueB =
        CreateObject<ConstantPositionMobilityModel>();
    ueB->SetPosition(Vector(-100.0, 100.0, 0)); // NW
    Ptr<ConstantPositionMobilityModel> ueC =
        CreateObject<ConstantPositionMobilityModel>();
    ueC->SetPosition(Vector(100.0, -100.0, 0)); // SE
    Ptr<ConstantPositionMobilityModel> ueD =
        CreateObject<ConstantPositionMobilityModel>();
    ueD->SetPosition(Vector(-100.0, -100.0, 0)); // SW

    // Real SGP4 orbit projected into the scenario's local ENU frame (genuine
    // pass dynamics; replaces the straight-line placeholder satellite).
    ns3::ntncon::WalkerConfig wcfgSat;
    wcfgSat.num_planes = 1;
    wcfgSat.total_sats = 80;
    wcfgSat.altitude_km = altKm; // SIONNA-07: was hard-coded 550, --altKm ignored
    wcfgSat.inclination_deg = 53.0;
    wcfgSat.epoch_unix_s = 1735689600.0;
    const auto satElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfgSat);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElementsSgp4(satElements[0]);
    double satSubLat, satSubLon, satSubAlt;
    satSgp4->GetGeodetic(satSubLat, satSubLon, satSubAlt);
    Ptr<NtnEnuProjectionMobilityModel> sat = CreateObject<NtnEnuProjectionMobilityModel>();
    sat->SetSource(satSgp4);
    sat->SetReference(satSubLat, satSubLon, 0.0);
    std::vector<Sample> samples;
    for (uint32_t i = 1; i <= steps; ++i)
    {
        const Time when = MilliSeconds(100 * i);
        Simulator::Schedule(when,
                            &Tick,
                            when.GetSeconds(),
                            chA,
                            chB,
                            chC,
                            chD,
                            Ptr<MobilityModel>(sat),
                            Ptr<MobilityModel>(ueA),
                            Ptr<MobilityModel>(ueB),
                            Ptr<MobilityModel>(ueC),
                            Ptr<MobilityModel>(ueD),
                            cache,
                            &samples);
    }
    Simulator::Stop(MilliSeconds(100 * steps + 200));
    Simulator::Run();
    // ntn-dataset@1: standard manifest + sim_health for this analytic example
    // (plan/MASTER_PLAN_GLOBE_DATASET_E2E_2026-11.md).
    ns3::ntnobs::WriteDatasetRecord(".", "city-block-4ue-cache", "analytic", argc, argv);
    Simulator::Destroy();

    std::printf("# city-block-4ue-cache: freq=%.3f GHz alt=%.0f km rain=%.1f mm/h "
                "spatial_res=%.0f m time_bucket=%lu us\n",
                freqHz / 1e9, altKm, rainMmH, spatialResM,
                (unsigned long)timeBucketUs);
    std::printf("# %-3s %-7s %-10s %-10s %-10s %-10s %-8s %-8s %-7s\n",
                "i", "t_s",
                "rx_NE", "rx_NW", "rx_SE", "rx_SW",
                "hits", "misses", "hit_rt");
    for (size_t i = 0; i < samples.size(); ++i)
    {
        const auto& s = samples[i];
        const uint64_t total = s.hits + s.misses;
        const double hitRate = total ? static_cast<double>(s.hits) / total : 0.0;
        std::printf("  %-3zu %-7.2f %-10.3f %-10.3f %-10.3f %-10.3f %-8lu %-8lu %-7.3f\n",
                    i + 1, s.t_s,
                    s.rx_dbm[0], s.rx_dbm[1], s.rx_dbm[2], s.rx_dbm[3],
                    (unsigned long)s.hits, (unsigned long)s.misses, hitRate);
    }
    std::printf("# final cache: hits=%lu misses=%lu evictions=%lu hit_rate=%.3f entries=%lu\n",
                (unsigned long)cache->GetHits(),
                (unsigned long)cache->GetMisses(),
                (unsigned long)cache->GetEvictions(),
                cache->GetHitRate(),
                (unsigned long)cache->GetEntries());

    // WF-12: say what produced these numbers. GetFallbacks() existed and no
    // example read it, so on a host without Sionna every run here printed
    // Sionna framing over pure free-space results with nothing to show it.
    uint64_t fbAll = 0;
    for (const auto& b : bases)
    {
        std::cout << b->ProvenanceLine() << std::endl;
        fbAll += b->GetFallbacks();
    }
    // P0-4: label the headline by what produced it.
    std::printf(fbAll > 0 ? "# headline: free space (no ray tracing)\nraytrace_live=0\n"
                          : "raytrace_live=1\n");
    return 0;
}
