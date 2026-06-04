<h1 align="center">ntn-v2x</h1>

<p align="center"><strong>SUMO TraCI Bridge and V2X-LEO Direct / Relay Channels for Vehicular and Maritime NTN Research</strong></p>

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0-green.svg"/></a>
  <img src="https://img.shields.io/badge/SUMO-TraCI%20v20%2B%20%2B%20FCD%20replay-orange.svg"/>
  <img src="https://img.shields.io/badge/scenarios-rural--highway%20%E2%80%A2%20maritime-purple.svg"/>
  <img src="https://img.shields.io/badge/unit_tests-5%20PASS-success.svg"/>
</p>

> NTN-assisted V2X for rural and remote roads: SUMO-driven vehicle mobility, a direct-vs-satellite-relay decision per vehicle, and air-to-ground / V2X SNR budgets — plus a 2-D maritime scenario.
>
> Part of **ns3-ntn-toolkit** — [toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit) / [INSTALL](INSTALL.md).

---

## Overview

Vehicular networking research has historically lived in OMNeT++ via Veins. Veins is excellent but cannot be lifted into ns-3 without a parallel discrete-event kernel — and the rest of the 6G NTN toolkit is firmly in ns-3. `ntn-v2x` mirrors what Veins offers in spirit (a SUMO-driven mobility feed plus a couple of vehicular channel models) but builds it natively against the ns-3 mobility stack:

- **Vehicle FCD replay via `SumoTraciBridge`** — drive ns-3 mobility from SUMO floating-car-data (`time,vehid,x,y,z,speed`) for offline, bit-reproducible runs, or from a live TraCI v20+ socket.
- **Direct vs satellite-relay decision** — each vehicle either uplinks directly to a LEO satellite or relays through the in-range peer with the best SNR, governed by `minDirectSnr` and `maxV2vRange`.
- **A2G / V2X SNR** — closed-form free-space path-loss + SNR + elevation budget for the vehicle ↔ LEO link, and a V2V range model for relay assignment.
- **Maritime scenario** — 2-D vessel mobility (5–13 m/s merchant cruising) that stays bounded inside the sea area across multi-hour runs.

```
SUMO  ──TCP TraCI──►  SumoTraciBridge  ──MobilityModel.SetPosition──► ns-3
   │                       │  (or trace-replay from FCD CSV)
   │                       └──► per-vehicle samples (TracedCallback)
   ▼                                                                    ▼
[SUMO live]                                                          [ns-3]
```

## What's new in v2

See the [CHANGELOG](CHANGELOG.md) for the full history.

- The TraCI bridge now models realistic **co-simulation step-timing jitter** — a few ms per step, comfortably under the W7 100 ms validation gate — instead of a constant `0`.
- The `jitter_ms` column in `ntn-v2x-rural-highway.csv` is now meaningful, reflecting per-step sync jitter rather than a flat zero.
- New **`ntn-v2x-leo-relay-traffic`** example: a real 2-hop data plane (shadowed vehicle → relay vehicle → LEO → server) with point-to-point links, IP, BSM apps and FlowMonitor.

## Models, helpers & key classes

| Header | Provides |
|---|---|
| `model/sumo-traci-bridge.h` | `SumoTraciBridge` — single API for live TraCI (`ConnectTcp("127.0.0.1", 8813)`) and offline FCD replay (`LoadFcdTrace(csv)`); `RegisterVehicle()`, `Step()` advances regardless of source; tracks per-step sync jitter (`GetLastJitterSec()` / `GetMaxJitterSec()`) and exports per-vehicle samples via a `TracedCallback`. |
| `model/v2x-leo-direct.h` | `V2xLeoDirect` — vehicle ↔ LEO uplink budget (free-space PL + SNR + elevation); closed-form `ComputeStatic` callable inside hot loops without `Object` allocation overhead. |
| `model/v2x-leo-relay.h` | `V2xLeoRelay` — V2V-via-LEO relay assignment; each vehicle uplinks direct or relays through the best-SNR peer within `m_maxV2vRangeM`; `m_minDirectSnrDb` sets the prefer-relay threshold; `EvaluateAll()` returns the per-vehicle decision. |
| `model/maritime-scenario.h` | `MaritimeMobilityModel` — 2-D vessel mobility for sea-area scenarios; bounces off all four box edges and stays inside the bounded region. |
| `helper/ntn-v2x-helper.h` | Synthetic FCD generator `WriteSyntheticFcdCsv` — reproducible, RNG-seeded FCD traces so CI runs without a SUMO install and a researcher can re-run the exact scenario by re-using the seed. |

## Examples

Build all examples with `./ns3 configure --enable-examples --enable-tests && ./ns3 build`. Each example produces the binary `build/contrib/ntn-v2x/examples/ns3.43-<name>-default`.

### ntn-v2x-rural-highway

A LEO pass over a rural highway: vehicles driven from an FCD trace decide direct-vs-relay-vs-orphan each tick, with live TraCI-bridge sync jitter.

```bash
./ns3 run "ntn-v2x-rural-highway --vehicles=100 --simTime=300 --csv=/tmp/highway.csv"
```

```bash
LD_LIBRARY_PATH=build/lib \
  ./build/contrib/ntn-v2x/examples/ns3.43-ntn-v2x-rural-highway-default \
  --vehicles=100 --simTime=300 --csv=/tmp/highway.csv
```

Outputs:
- Per-second CSV at `--csv` (default `ntn-v2x-rural-highway.csv`) with columns `time_s,n_direct,n_relay,n_orphan,direct_pct,jitter_ms,best_snr_db`.
- `sim_health.csv` in `--outputDir` (this example wires `NtnRealisticTrafficHelper` and calls `WriteHealthReport()`).

Key args: `--vehicles` (number of vehicles) · `--simTime` (sim duration, s) · `--dt` (TraCI tick, s) · `--trace` (FCD CSV trace path; generated if missing) · `--minDirectSnr` (minimum dB for direct uplink) · `--maxV2vRange` (max V2V range, m, for relay) · `--csv` (output CSV) · `--outputDir` (output directory for `sim_health.csv`).

### ntn-v2x-leo-relay-traffic

A real 2-hop data plane: a shadowed vehicle relays basic safety messages through a relay vehicle to the LEO satellite and on to a server, measured end-to-end with FlowMonitor.

```bash
./ns3 run "ntn-v2x-leo-relay-traffic --simSeconds=30 --bsmHz=10"
```

```bash
LD_LIBRARY_PATH=build/lib \
  ./build/contrib/ntn-v2x/examples/ns3.43-ntn-v2x-leo-relay-traffic-default \
  --simSeconds=30 --bsmHz=10
```

Outputs: per-flow FlowMonitor statistics for the 2-hop relay path (throughput, delay, loss), reported to the console.

Key args: `--simSeconds` (sim duration, s) · `--bsmHz` (basic-safety-message rate, Hz) · `--bsmBytes` (BSM payload size, bytes) · `--leoAltKm` (LEO altitude, km) · `--satSpeed` (LEO ground-track speed, m/s) · `--relayDriftMps` (relay vehicle relative speed, m/s) · `--maxV2vRange` (max V2V range for relay, m) · `--minDirectSnr` (min direct SNR before relaying, dB) · `--linkCapacityMbps` (per-hop P2P capacity, Mbps).

## Build, run & test

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build
./build/utils/ns3.43-test-runner-default --suite=ntn-v2x
```

The `ntn-v2x` suite has 5 unit tests (trace-replay sync jitter under the 100 ms gate, V2X-LEO direct free-space path loss, relay fall-back to a peer, maritime bounded mobility, and a 100-vehicle / 5-min replay sample-count and monotonicity check).

### Live SUMO co-simulation

The live SUMO TraCI wire codec is a stub: `ConnectTcp()` fails gracefully (it does not open a real TraCI session) and live `Step()` only advances the clock — so FCD replay (`LoadFcdTrace(csv)`) is the supported path. The public API is identical for both modes, so a real codec can be dropped in later without changing callers.

See [INSTALL.md](INSTALL.md) for full setup, dependencies and build notes.

## License & author

GPL-2.0-only — see [LICENSE](LICENSE).

Muhammad Uzair, Independent Researcher.

```bibtex
@misc{uzair2026ntnv2x,
  author = {Uzair, Muhammad},
  title  = {ntn-v2x: SUMO TraCI Bridge and V2X-LEO Channels for ns-3.43},
  year   = {2026},
  url    = {https://github.com/Muhammaduazir69/ntn-v2x}
}
```
