<h1 align="center">ntn-slice</h1>

<p align="center"><strong>3GPP Network Slicing for Non-Terrestrial Networks: eMBB / URLLC / mMTC / V2X with GEO Mode-Skip Routing</strong></p>

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0-green.svg"/></a>
  <img src="https://img.shields.io/badge/3GPP-TS%2023.501%20%2F%20TS%2022.261-orange.svg"/>
  <img src="https://img.shields.io/badge/SST-eMBB%20%E2%80%A2%20URLLC%20%E2%80%A2%20mMTC%20%E2%80%A2%20V2X-purple.svg"/>
  <img src="https://img.shields.io/badge/unit_tests-7%20PASS-success.svg"/>
</p>

> Per-slice PRB orchestration for LEO + GEO non-terrestrial networks: eMBB / URLLC / mMTC / V2X slices with TS 22.261 demand profiles, per-slice latency / reliability KPIs, and a GEO mode-skip policy that keeps URLLC traffic off the orbital latency floor.
>
> Part of **ns3-ntn-toolkit** — [README](../../README.md) / [INSTALL](../../INSTALL.md).

---

## Overview

5G slicing is straightforward when every cell is a few kilometres away. In NTN the latency budget of a URLLC slice (5 ms p99) is hard-bounded by orbital geometry — a single GEO hop adds ~270 ms one-way, blowing the budget by two orders of magnitude regardless of any radio-side allocator decision. `ntn-slice` is a 3GPP-compliant slicing layer that *understands the orbital floor*:

- **Per-slice PRB orchestration** with min-throughput reservation (the classical isolation guarantee), then priority-weighted distribution of the remainder by unmet demand — or by externally supplied RL shares.
- **TS 22.261 demand profiles** — default per-slice KPIs (latency budget, min/max throughput, reliability target, priority, `allowGeo`) seeded from TS 22.261 Table 7.1-1.
- **Per-slice latency / reliability KPIs** — a rolling-window isolation monitor that emits a `BreachEvent` whenever a slice exceeds its p99-latency or loss-rate target.
- **GEO mode-skip policy** — keeps URLLC traffic on LEO satellites whose round-trip time is compatible with the slice latency target; `ShouldSkipGeo(slice)` is true exactly for slices with `allowGeo == false`.

The orchestrator's `(observation, action)` shape matches the `SliceEnv` Gymnasium wrapper in [`ns3-ai`](https://github.com/Muhammaduazir69/ns3-ai), so a trained RL policy can drive the C++ allocator directly via `StepWithShares()`.

| Slice | SST | Default min throughput | Default p99 latency budget | GEO allowed |
|---|---:|---:|---:|:---:|
| eMBB  | 1 | 100 Mbps | 100 ms | yes |
| URLLC | 2 |   5 Mbps |   5 ms | **no — mode-skip** |
| mMTC  | 3 |   1 Mbps |   1 s  | yes |
| V2X   | 4 |   2 Mbps |  10 ms | yes |

## What's new in v2

See the toolkit [CHANGELOG](../../CHANGELOG.md) for the full history.

- The **three-slice LEO/GEO example** now produces genuinely *differentiated* slices: URLLC achieves low latency and high reliability, eMBB drives high throughput, and mMTC supports many low-rate devices — each within its own KPI envelope.
- `served_mbps` is now clamped so it **never exceeds the per-slice demand** (no over-reporting of served load).
- New **`ntn-slice-isolation-traffic`** example with a real ns-3 data plane (point-to-point links + IP + UDP apps + FlowMonitor), feeding live per-flow results through the selector and isolation monitor.

## Models, helpers & key classes

| Header | Provides |
|---|---|
| `model/ntn-slice-types.h` | `SliceSst` enum (eMBB / URLLC / mMTC / V2X); `Snssai` (8-bit SST + 24-bit SD with packed / unpacked codecs); `SliceProfile` (latency budget, min/max throughput, reliability target, priority, `allowGeo`); default profiles per TS 22.261 Table 7.1-1. |
| `model/ntn-slice-selector.h` | `NtnSliceSelector` — DSCP / port-range / app-label rule chain, first-match-wins, default fallback; maps a flow to its S-NSSAI. |
| `model/slice-orchestrator-xapp.h` | `SliceOrchestratorXapp` — PRB allocator with min-throughput reservation + priority-weighted unmet-demand sharing; `Step()` / `StepWithShares({snssai → fraction})` for RL override; `Tick` trace source exports per-slice allocated PRBs, served Mbps, satisfaction. |
| `model/slice-isolation-monitor.h` | `SliceIsolationMonitor` — rolling per-slice latency / loss-rate windows; `RecordPacket()` / `EvaluateAll()` returns `BreachEvent` records for latency-p99 and reliability breaches. |
| `helper/ntn-slice-helper.h` | `NtnSliceHelper::ThreeSliceDefault()` — ready-wired selector + orchestrator + monitor + the three default slices; GEO helpers `ShouldSkipGeo(slice)` and `IsGeoSatellite(altKm)` (35 786 km threshold). |

## Examples

Build all examples with `./ns3 configure --enable-examples --enable-tests && ./ns3 build`. Each example produces the binary `build/contrib/ntn-slice/examples/ns3.43-<name>-default`.

### ntn-three-slice-leo-geo

Three co-existing slices over a LEO + GEO topology; demonstrates the GEO mode-skip policy and differentiated per-slice KPIs.

```bash
./ns3 run "ntn-three-slice-leo-geo --simTime=3600 --csv=/tmp/three-slice.csv"
```

```bash
LD_LIBRARY_PATH=build/lib \
  ./build/contrib/ntn-slice/examples/ns3.43-ntn-three-slice-leo-geo-default \
  --simTime=3600 --csv=/tmp/three-slice.csv
```

Outputs:
- Per-slice KPI CSV at `--csv` (default `ntn-three-slice.csv`) — allocated PRBs, served / demand Mbps, satisfaction, p99 latency and loss rate per slice per tick.
- `sim_health.csv` in `--outputDir` (this example wires `NtnRealisticTrafficHelper` and calls `WriteHealthReport()`).

Key args: `--simTime` (sim duration, s) · `--urllcViaGeo` (force URLLC over GEO; gates the GEO-skip test) · `--totalPrb` (total PRB budget) · `--csv` (per-slice KPI CSV path) · `--outputDir` (output directory for `sim_health.csv`).

### ntn-slice-isolation-traffic

Real per-slice traffic with a live ns-3 data plane: point-to-point links, IP, UDP apps and FlowMonitor feeding the `NtnSliceSelector` + `SliceIsolationMonitor`.

```bash
./ns3 run "ntn-slice-isolation-traffic --simSeconds=30 --dataRateMbps=20"
```

```bash
LD_LIBRARY_PATH=build/lib \
  ./build/contrib/ntn-slice/examples/ns3.43-ntn-slice-isolation-traffic-default \
  --simSeconds=30 --dataRateMbps=20
```

Outputs: per-flow FlowMonitor statistics fed through the slice classifier and isolation monitor, reported to the console (per-slice served throughput, latency and any isolation breaches) — nothing is hardcoded.

Key args: `--simSeconds` (sim duration, s) · `--dataRateMbps` (per-slice offered load, Mbps) · `--packetBytes` (UDP payload size, bytes) · `--leoAltKm` (LEO altitude, km) · `--satSpeed` (LEO ground-track speed, m/s) · `--linkCapacityMbps` (per-slice P2P capacity, Mbps).

## Build, run & test

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build
./build/utils/ns3.43-test-runner-default --suite=ntn-slice
```

The `ntn-slice` suite has 7 unit tests (S-NSSAI pack/unpack round-trip, selector first-match-wins, URLLC min-throughput preservation under contention, isolation-monitor breach detection, URLLC GEO mode-skip, three-slice co-existence, RL-shares honoured).

See [../../INSTALL.md](../../INSTALL.md) for full setup, dependencies and toolkit-wide build notes.

## License & author

GPL-2.0-only — see [LICENSE](LICENSE).

Muhammad Uzair, Independent Researcher.

```bibtex
@misc{uzair2026ntnslice,
  author = {Uzair, Muhammad},
  title  = {ntn-slice: 3GPP Network Slicing with GEO Mode-Skip for Non-Terrestrial Networks},
  year   = {2026},
  url    = {https://github.com/Muhammaduazir69/ntn-slice}
}
```
