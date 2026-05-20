# ntn-traffic — NTN-oriented traffic generators + realistic packet plane

`ntn-traffic` ships two pieces:

1. **Traffic generators** with NTN-appropriate defaults — CBR,
   NRTV over TCP/UDP, 3GPP HTTP over a satellite bent-pipe /
   regenerative link, and a `TrafficTimeTag` for per-packet
   latency tracking.
2. **`NtnRealisticTrafficHelper`** — drops a real ns-3 traffic
   plane (Internet stack, per-UE P2P links, OnOff/PacketSink
   applications, periodic `Simulator::Schedule` callbacks, and a
   sim-realism health gate) into *any* NTN example so that
   `Simulator::Run()` actually exercises packets, schedulers, and
   the protocol stack across the declared `simTime`.

The helper is what makes the other twelve toolkit modules
(`ntn-cho`, `ntn-rrc`, `ntn-sagin`, `ntn-slice`, `ntn-v2x`,
`ntn-observability`, `oran-ntn`, `thz-ntn`, etc.) emit a
`sim_health.csv` after each run and survive the CI realism gates.

- ns-3 version: `release ns-3.43`
- Version: `1.1.0`
- License: GPL-2.0-only
- Maintainer: Muhammad Uzair (ORCID 0009-0002-4104-2680)

> **Name change:** the internal name of this module used to be
> `traffic`. It was renamed to `ntn-traffic` before the App Store
> submission because the unqualified name `traffic` is ambiguous
> in a public catalogue.

## Quick start

```bash
cd ns-3-dev
./ns3 configure --enable-examples --enable-tests \
    --enable-modules=ntn-traffic
./ns3 build ntn-traffic nrtv-p2p-example
./ns3 run "nrtv-p2p-example --simTime=60"
./ns3 test --suite=ntn-traffic
```

### Embed the realistic traffic plane in your own example

```cpp
#include "ns3/ntn-realistic-traffic-helper.h"
using namespace ns3;
NtnRealisticTrafficHelper traffic;
traffic.SetSimTime(Seconds(simTime));
traffic.SetOutputDir(outputDir);          // sim_health.csv lands here
traffic.SetRunTag("my-scenario");
traffic.SetProfile(
    NtnRealisticTrafficHelper::TrafficProfile::MixedBouquet);
NodeContainer ues = traffic.InstallUes(numUes);

// schedule your analytical tick on the ns-3 event queue:
traffic.RegisterPeriodicCallback(MilliSeconds(200),
    [&](Time now){ /* your per-tick code */ });

traffic.Wire();
Simulator::Stop(Seconds(simTime + 0.5));
Simulator::Run();
traffic.WriteHealthReport();   // emits sim_health.csv + asserts gates
Simulator::Destroy();
```

### Traffic profiles

| Profile | Rate / pkt size | Use case |
|---|---|---|
| `NbIotPeriodic`         | 1 kbps / 64 B periodic | low-power IoT uplink |
| `EmbbStreaming`         | 5 Mbps / 1 280 B sustained | video, web browsing |
| `UrllcPings`            | 1 Mbps / 256 B bursts | low-latency control |
| `DigitalTwinTelemetry`  | 256 kbps / 512 B | per-node telemetry to twin |
| `MixedBouquet`          | round-robin over the four above | realistic multi-service mix |

### Sim-realism gates

`WriteHealthReport()` writes a `sim_health.csv` with these columns
and asserts each row clears the configured `HealthGates`:

```
sim_time_s, wall_clock_s, packets_tx, packets_rx,
tx_per_sim_sec, rx_over_tx, analytical_events,
heartbeat_ticks, ues, run_tag
```

Defaults: `minWallClockPerSec = 0.015`, `minPacketsPerSimSec = 100`,
`minRxOverTxRatio = 0.85`, `minAnalyticalCallbacks = 0`. A scenario
that returns instantly with an empty event queue (e.g. only
analytical math in user-space, no `Simulator::Schedule`) fails the
gate. The umbrella CI workflow
`.github/workflows/sim-health.yml` runs all fourteen example
scenarios through `tools/check_simulation_health.py` and fails the
build if any miss.

## What's in the module

```
helper/   — traffic-helper, ntn-realistic-traffic-helper
model/    — cbr-application, nrtv-{header, tcp-client, tcp-server,
            udp-server, variables, video-worker},
            three-gpp-http-satellite-{client, variables},
            traffic-time-tag
examples/ — nrtv-p2p-example, nrtv-variables-plot,
            three-gpp-http-example
test/     — 2 suites: nrtv-traces, three-gpp-http
```

## CSV outputs

| Example | Output | Schema |
|---|---|---|
| `nrtv-p2p-example` | `*.tr`, `*.pcap` (FlowMonitor) | per-flow throughput / loss |
| `nrtv-variables-plot` | gnuplot-format `.dat` | NRTV session-length / pkt-size CDF |
| `three-gpp-http-example` | stdout summary | per-page request / response timings |
| any caller of `NtnRealisticTrafficHelper` | `sim_health.csv` | see schema above |

## Documentation

- 3GPP TR 25.892 — NRTV reference traffic model
- 3GPP TR 38.913 — 5G service requirements (eMBB, URLLC, mMTC)
- `helper/ntn-realistic-traffic-helper.h` — full API reference
- `SIMULATION_REALITY_FIX_PLAN.md` (umbrella repo root) — design
  rationale for the v2 event-driven scenarios.

## Cite this work

```bibtex
@misc{uzair2026ntntraffic,
  author = {Uzair, Muhammad},
  title  = {ntn-traffic: NTN traffic generators and realistic
            sim-health plane for ns-3.43},
  year   = {2026},
  url    = {https://github.com/Muhammaduazir69/ns3-ntn-toolkit}
}
```

## Part of the ns3-ntn-toolkit

| Module | Repo |
|---|---|
| Toolkit (umbrella) | [ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit) |
| ntn-constellation | [ntn-constellation](https://github.com/Muhammaduazir69/ntn-constellation) |
| ntn-rrc | [ntn-rrc](https://github.com/Muhammaduazir69/ntn-rrc) |
| ntn-observability | [ntn-observability](https://github.com/Muhammaduazir69/ntn-observability) |
| ns3-ai (fork) | [ns3-ai](https://github.com/Muhammaduazir69/ns3-ai) |
| ntn-sagin | [ntn-sagin](https://github.com/Muhammaduazir69/ntn-sagin) |
| ntn-slice | [ntn-slice](https://github.com/Muhammaduazir69/ntn-slice) |
| ntn-v2x | [ntn-v2x](https://github.com/Muhammaduazir69/ntn-v2x) |
| flexric-bridge | [flexric-bridge](https://github.com/Muhammaduazir69/flexric-bridge) |
| ntn-sionna | [ntn-sionna](https://github.com/Muhammaduazir69/ntn-sionna) |
| ntn-digital-twin | [ntn-digital-twin](https://github.com/Muhammaduazir69/ntn-digital-twin) |
| ntn-cho | [ntn-cho-framework](https://github.com/Muhammaduazir69/ntn-cho-framework) |
| oran-ntn | [oran-ntn](https://github.com/Muhammaduazir69/oran-ntn) |
| thz-ntn | [ns3-thz-ntn](https://github.com/Muhammaduazir69/ns3-thz-ntn) |
| **ntn-traffic** | this directory |

## License

GPL-2.0-only. See `LICENSE`.
