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
 * SionnaCachingTransport on top of a SionnaUdpTransport. With four UEs at
 * 100 ms sampling cadence and a 500 ms cache time bucket, the cache
 * collapses ~80% of queries into hits — typical of a real digital-twin
 * replay loop.
 *
 * Run:
 *   python3 contrib/ntn-sionna/bridge/sionna-server.py --port 8765 &
 *   ./ns3 run "city-block-4ue-cache --freqHz=2e9 --steps=60"
 */
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
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
    std::string host = "127.0.0.1";
    uint16_t port = 8765;
    double freqHz = 2.0e9;
    double altKm = 550.0;
    double rainMmH = 0.0;
    uint32_t steps = 60;
    uint32_t timeoutMs = 200;
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
    cmd.Parse(argc, argv);

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
        Ptr<NtnSionnaChannel> base = CreateObject<NtnSionnaChannel>();
        base->SetTransport(cache);
        base->SetFrequencyHz(freqHz);
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
    auto chA = buildChannel();
    auto chB = buildChannel();
    auto chC = buildChannel();
    auto chD = buildChannel();

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

    Ptr<ConstantVelocityMobilityModel> sat =
        CreateObject<ConstantVelocityMobilityModel>();
    sat->SetPosition(Vector(-300e3, 0, altKm * 1000.0));
    sat->SetVelocity(Vector(20e3, 0, 0));

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
    return 0;
}
