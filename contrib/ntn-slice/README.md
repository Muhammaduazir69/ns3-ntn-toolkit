<h1 align="center">ntn-slice</h1>

<p align="center"><strong>Network slicing over NTN: eMBB, URLLC and mMTC with per-5QI bearers and SLA percentiles taken from a distribution</strong></p>

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg" alt="ns-3.43"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0-green.svg" alt="GPL-2.0"/></a>
  <img src="https://img.shields.io/badge/3GPP-TS%2023.501%20%2F%2038.300-orange.svg" alt="3GPP TS 23.501 and TS 38.300"/>
  <img src="https://img.shields.io/badge/slices-eMBB%20%C2%B7%20URLLC%20%C2%B7%20mMTC%20%C2%B7%20V2X-purple.svg" alt="four slice types"/>
  <img src="https://img.shields.io/badge/examples-3-informational.svg" alt="3 examples"/>
</p>

<p align="center">
  <a href="https://github.com/Muhammaduazir69/ns3-ntn-toolkit">Toolkit</a>
  &nbsp;·&nbsp;
  <a href="INSTALL.md">Install</a>
  &nbsp;·&nbsp;
  <a href="#examples">Examples</a>
  &nbsp;·&nbsp;
  <a href="https://muhammaduazir69.github.io/ns3-ntn-toolkit/modules/ntn-slice/">Docs</a>
</p>

---

A slice is only a slice if the radio treats it differently. This module maps S-NSSAI and 5QI onto dedicated EPS bearers with TFT filters on a real NR NTN cell, so a URLLC flow and an eMBB flow on the same satellite genuinely contend for different resources rather than sharing one pipe with different labels.

The SLA reporting is where slicing simulations usually quietly fail, and it is worth being explicit about the failure mode: if every packet is stamped with the same scalar mean delay, then the p99 of that slice is its mean, and a reliability target of one in a hundred thousand is unobservable no matter how long you run. The sink here retains a bounded delay histogram, so the three slices report 12.50, 11.50 and 11.50 ms at p99 rather than each reporting its own average.

A GEO mode-skip policy keeps latency-bound traffic off the orbital floor when a GEO hop would blow the budget.

## Quick start

Inside the toolkit, where the module is already present and built:

```bash
./ns3 run ntn-three-slice-leo-geo
./ns3 run ntn-slice-demo
```

Standalone, into an existing ns-3.43 tree:

```bash
git clone -b ntn-slice-v2 https://github.com/Muhammaduazir69/ntn-slice.git contrib/ntn-slice
./ns3 configure --enable-modules='' --enable-examples --enable-tests
./ns3 build
```

`INSTALL.md` in this directory carries the full dependency list. Most examples in
this module build on `ntn-traffic`, the toolkit's real-stack spine, so the
toolkit tree is the path of least resistance.

## Overview

5G slicing is straightforward when every cell is a few kilometres away. In NTN the latency budget of a URLLC slice (5 ms p99) is hard-bounded by orbital geometry — a GEO leg adds ~120 ms of one-way slant propagation (~240 ms ground–satellite–ground), blowing the budget by more than an order of magnitude regardless of any radio-side allocator decision. `ntn-slice` is a 3GPP-compliant slicing layer that *understands the orbital floor*:

- **Per-slice PRB orchestration** with min-throughput reservation, then priority-weighted distribution of the remainder by unmet demand — or by externally supplied RL shares. **This orchestrator is an open-loop *shadow* allocator**: its PRB decisions are computed and logged but do **not** actuate the mmwave scheduler. All slices share **one slice-agnostic cell**; the 5QI / S-NSSAI is a packet label the MAC never reads; and per-slice isolation is **observed statistically** on the measured plane, not *enforced* by the allocator. There is no per-slice isolation guarantee — the reservation is the allocator's own bookkeeping, not a scheduler-enforced PRB partition.
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

## What changed in v2.5

See the [CHANGELOG](CHANGELOG.md) for the full history.

- **All three examples now run on a measured radio.** Every example builds a real mmwave NR NTN cell (`NtnRealStackHelper` from `ntn-traffic`: SpectrumPhy + MAC + RLC/PDCP + RRC + EPC) on a real SGP4 Walker satellite pass with TR 38.811 UE placement — no point-to-point stand-ins, no closed-form SNR curves.
- **Traffic is `NtnOranApplication` QoS flows** (`ntn-traffic`), not `OnOffApplication`: each packet carries a real 24-byte in-band payload header (5QI / S-NSSAI / QFI / sequence number / TX timestamp), and per-UE delivered bytes are measured at the `NtnOranSink`. The `MixedBouquet` profile spreads UEs across eMBB (CBR-saturating, 5QI 2), URLLC (periodic pings, 5QI 82) and mMTC (periodic NB-IoT-style, 5QI 9) flows.
- New **`ntn-slice-real-stack`** example: all slices contend for **one** shared mmwave cell, so per-slice service is genuinely contested rather than trivially separated by distinct links — but isolation here is *measured* on the shared cell, not *enforced* by a per-slice PRB scheduler (the orchestrator's PRB split does not drive the mmwave MAC).
- **`ntn-slice-isolation-traffic`** was rebuilt on the same shared-cell recipe: the eMBB slice saturates the cell while URLLC and mMTC must hold their SLAs; per-slice SINR / throughput are measured and fed to the real `SliceIsolationMonitor`. It now also writes a **per-slice `slices.csv`** — per-BWP SINR, measured throughput, measured one-way delay, delivered / lost packet counts, and each slice's SLA breach verdict (`latency_breach` / `reliability_breach`) — making the real per-BWP isolation visible where the aggregate `sim_health.csv` hid it. A representative run shows eMBB saturating the cell (~15 Mbps) and URLLC's 5 ms budget breaching over the LEO slant (~8.56 ms measured, `latency_breach=1`) — the expected orbital-floor result, not a bug.
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

Outputs: console isolation summary (measured cell SINR / TBLER / total throughput, per-slice measured throughput and SINR, SLA breach verdict under eMBB saturation); a **per-slice breakdown `slices.csv`** (columns `slice,five_qi,sst,bwp,sinr_db,throughput_mbps,mean_owd_ms,rx_pkts,lost_pkts,latency_breach,reliability_breach,provenance`, one row each for eMBB / URLLC / mMTC); and `sim_health.csv` — all in `--outputDir` (default `ntn-slice-isolation-output`), nothing is hardcoded. `slices.csv` exposes the real per-BWP isolation (distinct per-slice SINR / throughput / delay) that the aggregate `sim_health.csv` hides. In a representative run the eMBB slice saturates the shared cell (~15 Mbps) while URLLC's 5 ms latency budget **breaches** over the LEO slant (~8.56 ms measured one-way delay, `latency_breach=1`) — that breach is the expected, physically-correct result of the orbital latency floor (TS 23.501 5QI PDB judged against real geometry), not a simulator defect.

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

---

## Standards implemented

3GPP TS 23.501 (network slicing, S-NSSAI, 5QI and the standardized QoS characteristics), TS 22.261 (service requirements and demand profiles), TS 38.300 (QoS flow to DRB mapping), TS 38.101-5 (NTN FR1 channel bandwidths, which constrain what a slice can be allocated), TR 38.821 (NTN latency budgets).

## Keywords

network slicing, 5G slicing, S-NSSAI, 5QI, slice SLA, eMBB, URLLC, mMTC, V2X slice, QoS flow, dedicated bearer, TFT, PRB allocation, slice isolation, latency percentile, p99 latency, reliability target, GEO latency floor, satellite slicing, non-terrestrial network, ns-3.

## Author

**Muhammad Uzair**, Independent Researcher
[ORCID 0009-0002-4104-2680](https://orcid.org/0009-0002-4104-2680)

Part of the [ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit),
a pre-integrated ns-3.43 platform for 6G non-terrestrial network research.
Mirrored on [GitLab](https://gitlab.com/ns3-ntn-toolkit).

## License

GPL-2.0-only, matching ns-3.
