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
> Part of **ns3-ntn-toolkit** — [toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit) / [INSTALL](INSTALL.md).

---

## Overview

5G slicing is straightforward when every cell is a few kilometres away. In NTN the latency budget of a URLLC slice (5 ms p99) is hard-bounded by orbital geometry — a GEO leg adds ~120 ms of one-way slant propagation (~240 ms ground–satellite–ground), blowing the budget by more than an order of magnitude regardless of any radio-side allocator decision. `ntn-slice` is a 3GPP-compliant slicing layer that *understands the orbital floor*:

- **Per-slice PRB orchestration** with min-throughput reservation (the classical isolation guarantee), then priority-weighted distribution of the remainder by unmet demand — or by externally supplied RL shares.
- **TS 22.261 demand profiles** — default per-slice KPIs (latency budget, min/max throughput, reliability target, priority, `allowGeo`) seeded from TS 22.261 Table 7.1-1.
- **Per-slice latency / reliability KPIs** — a rolling-window isolation monitor that emits a `BreachEvent` whenever a slice exceeds its p99-latency or loss-rate target.
- **GEO mode-skip policy** — keeps URLLC traffic on LEO satellites whose round-trip time is compatible with the slice latency target; `ShouldSkipGeo(slice)` is true exactly for slices with `allowGeo == false`.

The orchestrator's `(observation, action)` shape matches the `SliceEnv` Gymnasium wrapper in [`ns3-ai`](https://github.com/Muhammaduazir69/ns3-ai), so a trained RL policy can drive the C++ allocator directly via `StepWithShares()`.

| Slice | SST | Default min throughput | Default latency budget | GEO allowed |
|---|---:|---:|---:|:---:|
| eMBB  | 1 | 50 Mbps | 50 ms | yes |
| URLLC | 2 |  5 Mbps |  5 ms | **no — mode-skip** |
| mMTC  | 3 |  1 Mbps |  1 s  | yes |

The V2X SST enum value (SST 4) is defined, but no default V2X profile ships yet — only the three slices above have a default-profile factory.

## What's new in v2

See the [CHANGELOG](CHANGELOG.md) for the full history.

- **All three examples now run on a measured radio.** Every example builds a real mmwave NR NTN cell (`NtnRealStackHelper` from `ntn-traffic`: SpectrumPhy + MAC + RLC/PDCP + RRC + EPC) on a real SGP4 Walker satellite pass with TR 38.811 UE placement — no point-to-point stand-ins, no closed-form SNR curves.
- **Traffic is `NtnOranApplication` QoS flows** (`ntn-traffic`), not `OnOffApplication`: each packet carries a real 24-byte in-band payload header (5QI / S-NSSAI / QFI / sequence number / TX timestamp), and per-UE delivered bytes are measured at the `NtnOranSink`. The `MixedBouquet` profile spreads UEs across eMBB (CBR-saturating, 5QI 2), URLLC (periodic pings, 5QI 82) and mMTC (periodic NB-IoT-style, 5QI 9) flows.
- New **`ntn-slice-real-stack`** example: all slices contend for **one** shared mmwave cell, so PRB-level isolation is genuinely contested rather than trivially guaranteed by separate links.
- **`ntn-slice-isolation-traffic`** was rebuilt on the same shared-cell recipe: the eMBB slice saturates the cell while URLLC and mMTC must hold their SLAs; per-slice SINR / throughput are measured and fed to the real `SliceIsolationMonitor`.
- The **three-slice LEO/GEO example** validates URLLC GEO mode-skip against real geometry — LEO slant/c for LEO-served slices, the real ~36 000 km GEO slant/c (~120 ms one-way) for GEO-served ones — with delivery gated by the cell's measured TBLER.
- `served_mbps` is clamped so it **never exceeds the per-slice demand** (no over-reporting of served load).

## Models, helpers & key classes

| Header | Provides |
|---|---|
| `model/ntn-slice-types.h` | `SliceSst` enum (eMBB / URLLC / mMTC / V2X); `Snssai` (8-bit SST + 24-bit SD with packed / unpacked codecs); `SliceProfile` (latency budget, min/max throughput, reliability target, priority, `allowGeo`); default profiles per TS 22.261 Table 7.1-1. |
| `model/ntn-slice-selector.h` | `NtnSliceSelector` — DSCP / port-range / app-label rule chain, first-match-wins, default fallback; maps a flow to its S-NSSAI. |
| `model/slice-orchestrator-xapp.h` | `SliceOrchestratorXapp` — PRB allocator with min-throughput reservation + priority-weighted unmet-demand sharing; `Step()` / `StepWithShares({snssai → fraction})` for RL override; `Tick` trace source exports per-slice allocated PRBs, served Mbps, satisfaction. |
| `model/slice-isolation-monitor.h` | `SliceIsolationMonitor` — rolling per-slice latency / loss-rate windows; `RecordPacket()` / `EvaluateAll()` returns `BreachEvent` records for latency-p99 and reliability breaches. |
| `helper/ntn-slice-helper.h` | `NtnSliceHelper::ThreeSliceDefault()` — ready-wired selector + orchestrator + monitor + the three default slices; GEO helpers `ShouldSkipGeo(slice)` and `IsGeoSatellite(altKm)` (altitudes at or above the conservative 30 000 km boundary are treated as GEO-class; GEO proper is 35 786 km). |

A pre-built Grafana dashboard (`dashboards/ntn-slice.json` — per-slice satisfaction, served-vs-demand panels) can be imported into the Grafana stack that ships with the [ntn-observability](https://github.com/Muhammaduazir69/ntn-observability) module.

## Examples

Build all examples with `./ns3 configure --enable-examples --enable-tests && ./ns3 build`. Each example produces the binary `build/contrib/ntn-slice/examples/ns3.43-<name>-default`.

### ntn-three-slice-leo-geo

Three slices (eMBB / URLLC / mMTC) coexisting on a LEO + GEO NTN, with URLLC enforcing GEO mode-skip routing. The orchestrator runs at 1 Hz on top of a real mmwave NR cell (SGP4 LEO pass, TR 38.811 UEs): delivery is gated by the cell's *measured* TBLER, and per-slice latency is the *real* geometry — LEO slant/c for LEO-served slices, the ~36 000 km GEO slant/c (~120 ms one-way) for GEO-served ones — so the GEO-skip decision is validated against physics, not a hardcoded constant.

```bash
./ns3 run "ntn-three-slice-leo-geo --simTime=30 --csv=/tmp/three-slice.csv"
```

```bash
LD_LIBRARY_PATH=build/lib \
  ./build/contrib/ntn-slice/examples/ns3.43-ntn-three-slice-leo-geo-default \
  --simTime=30 --csv=/tmp/three-slice.csv
```

Outputs:
- Per-slice KPI CSV at `--csv` (default `ntn-three-slice.csv`) — per tick: demand / served Mbps, allocated PRBs, satisfaction, p99 latency and latency / reliability breach flags per slice.
- `sim_health.csv` in `--outputDir` (default `ntn-three-slice-output`), written by `NtnRealStackHelper::WriteHealthReport()`.
- Console summary: measured LEO SINR / TBLER, max URLLC p99 latency and breach counts.

Key args: `--simTime` (sim duration, s; default 30) · `--numUes` (UEs on the measured LEO cell; default 3) · `--urllcViaGeo` (force URLLC over GEO; gates the GEO-skip test) · `--totalPrb` (total PRB budget; default 273) · `--satEirpDbm` (LEO EIRP / gNB Tx power, dBm) · `--csv` (per-slice KPI CSV path) · `--outputDir` (output directory).

### ntn-slice-isolation-traffic

Three slices **share one real mmwave NR cell** under genuine contention: the eMBB slice saturates the cell while URLLC and mMTC must hold their SLAs. UEs are assigned round-robin to slices (`ue % 3`) and carry `NtnOranApplication` per-slice traffic profiles (`MixedBouquet`). Per-slice SINR comes off the mmwave PHY trace, per-slice delivered bytes off the sinks, latency from the real slant geometry — all fed to the real `SliceIsolationMonitor`, which flags breaches.

```bash
./ns3 run "ntn-slice-isolation-traffic --simSeconds=15 --numUes=9"
```

```bash
LD_LIBRARY_PATH=build/lib \
  ./build/contrib/ntn-slice/examples/ns3.43-ntn-slice-isolation-traffic-default \
  --simSeconds=15 --numUes=9
```

Outputs: console isolation summary (measured cell SINR / TBLER / total throughput, per-slice measured throughput and SINR, SLA breach verdict under eMBB saturation) plus `sim_health.csv` in `--outputDir` (default `ntn-slice-isolation-output`) — nothing is hardcoded.

Key args: `--simSeconds` (sim duration, s; default 15) · `--numUes` (UEs on the shared cell, slice = `ue % 3`; default 9) · `--satEirpDbm` (satellite EIRP / gNB Tx power, dBm) · `--backhaulMs` (feeder + core one-way delay, ms) · `--outputDir` (output directory).

### ntn-slice-real-stack

The shared-cell isolation recipe in its minimal form: all slices on **one** real mmwave cell, per-slice SINR and throughput measured from that cell, and the `SliceIsolationMonitor` evaluating SLA breaches against measured delivery plus the real geometric NTN latency — URLLC's 5 ms budget realistically breaches over the LEO slant while eMBB's 50 ms budget holds.

```bash
./ns3 run "ntn-slice-real-stack --duration=15 --numUes=9"
```

```bash
LD_LIBRARY_PATH=build/lib \
  ./build/contrib/ntn-slice/examples/ns3.43-ntn-slice-real-stack-default \
  --duration=15 --numUes=9
```

Outputs: per-slice measured throughput / SINR table, per-slice SLA breach evaluation on the console, and `sim_health.csv` in `--outputDir` (default `ntn-slice-real-stack-output`).

Key args: `--duration` (sim duration, s; default 15) · `--numUes` (UEs sharing the cell; default 9) · `--altitude` (satellite altitude, km; default 550) · `--satEirpDbm` (satellite EIRP, dBm) · `--outputDir` (output directory).

## Build, run & test

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build
./test.py -s ntn-slice
```

The `ntn-slice` suite has 7 unit tests (S-NSSAI pack/unpack round-trip, selector first-match-wins, URLLC min-throughput preservation under contention, isolation-monitor breach detection, URLLC GEO mode-skip, three-slice co-existence, RL-shares honoured).

See [INSTALL.md](INSTALL.md) for setup and dependencies. For the full toolkit, see [ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit).

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
