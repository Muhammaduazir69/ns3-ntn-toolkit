<h1 align="center">ntn-v2x</h1>

<p align="center"><strong>SUMO TraCI Bridge, NR PC5 Sidelink (Mode 2) and V2X-LEO Direct / Relay Channels for Vehicular and Maritime NTN Research</strong></p>

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0-green.svg"/></a>
  <img src="https://img.shields.io/badge/SUMO-TraCI%20v20%2B%20%2B%20FCD%20replay-orange.svg"/>
  <img src="https://img.shields.io/badge/PC5-NR%20sidelink%20Mode%202%20(TS%2038.321)-red.svg"/>
  <img src="https://img.shields.io/badge/scenarios-rural--highway%20%E2%80%A2%20platoon--URLLC%20%E2%80%A2%20PC5--sidelink%20%E2%80%A2%20maritime-purple.svg"/>
  <img src="https://img.shields.io/badge/unit_tests-7%20PASS-success.svg"/>
</p>

> NTN-assisted V2X for rural and remote roads: SUMO-driven vehicle mobility, a direct-vs-satellite-relay decision per vehicle made on the **measured** SINR of a real mmwave NR LEO cell, and a URLLC platoon-control flagship with edge-AI placement — plus a 2-D maritime scenario.
>
> Part of **ns3-ntn-toolkit** — [toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit) / [INSTALL](INSTALL.md).

---

<p align="center">
  <img src="docs/ntn_v2x_demo.gif" alt="module live demo" width="900"/>
</p>

## Overview

Vehicular networking research has historically lived in OMNeT++ via Veins. Veins is excellent but cannot be lifted into ns-3 without a parallel discrete-event kernel — and the rest of the 6G NTN toolkit is firmly in ns-3. `ntn-v2x` mirrors what Veins offers in spirit (a SUMO-driven mobility feed plus vehicular channel models) but builds it natively against the ns-3 mobility stack, and puts the radio on a real NR protocol stack:

- **Vehicle FCD replay via `SumoTraciBridge`** — drive ns-3 mobility from SUMO floating-car-data (`time,vehid,x,y,z,speed`) for offline, bit-reproducible runs, or from a live TraCI v20+ socket.
- **NR PC5 sidelink Mode 2 (direct V2V, no gNB)** — a genuine direct-V2V safety layer: vehicles broadcast SAE J2735 BSMs over PC5 with autonomous, sensing-based resource selection (`NtnNrSidelink`, per 3GPP TS 38.321 §5.22 / TS 38.331 SL-ResourcePool), evaluated with the TS 38.885 PRR-vs-distance methodology. This is the *sidelink* counterpart to the Uu/P2P relay path below, which instead routes traffic through a satellite gNB.
- **Direct vs satellite-relay decision on measured radio** — vehicles are UEs on a real mmwave NR NTN cell (`NtnRealStackHelper`, SGP4 LEO); each vehicle's direct uplink quality is its measured DL SINR minus a 3GPP-style per-vehicle NLOS blockage loss, and a vehicle that cannot clear `minDirectSnr` relays through the nearest in-range peer (`maxV2vRange`).
- **Closed-form reference** — `V2xLeoDirect` keeps the free-space PL + SNR + elevation budget as a comparison baseline; `ntn-v2x-real-stack` prints it next to the measured SINR so the formula-vs-measurement gap is visible.
- **Measured traffic KPIs** — data-plane examples carry `NtnOranApplication` traffic with an in-band `NtnOranPayloadHeader`; `NtnOranSink` measures one-way delay, jitter, PDR and goodput from the packets themselves.
- **Maritime scenario** — 2-D vessel mobility (5–13 m/s merchant cruising) that stays bounded inside the sea area across multi-hour runs.

```
SUMO  ──TCP TraCI──►  SumoTraciBridge  ──MobilityModel.SetPosition──► ns-3
   │                       │  (or trace-replay from FCD CSV)
   │                       └──► per-vehicle samples (TracedCallback)
   ▼                                                                    ▼
[SUMO live]                                              [real mmwave NR LEO cell]
```

## What's new (July 2026)

- **NR PC5 sidelink Mode 2 (`model/ntn-nr-sidelink.{h,cc}`)** — a direct-V2V safety layer with no gNB in the loop: `NtnSlResourcePool` + `NtnSlUeMac` (autonomous, sensing-based Mode-2 selection per TS 38.321 §5.22 over the TS 38.331 SL-ResourcePool) + `NtnSlChannel` (half-duplex, co-channel collisions, TS 38.885 PRR). A MAC / resource-pool abstraction — not a PSCCH/PSSCH PHY.
- **New example `ntn-v2x-pc5-sidelink-bsm`** — vehicles broadcast SAE J2735 BSMs over PC5 and report PRR vs distance; PRR degrades gracefully as vehicle density saturates the pool.
- **SAE J2735 BSM Part I header (`model/ntn-v2x-bsm-header.{h,cc}`)** — a real 23-byte BSMcoreData `Header` (msgCnt/id/secMark/lat/lon/elev/speed/heading), so a BSM carries genuine vehicle state.
- Two new unit tests bring the suite to 7: a J2735 BSM header round-trip and `NtnSidelinkMode2Test` (Mode-2 selection, half-duplex, wide-pool vs 1-subchannel PRR).

## What's new (June 2026)

See the [CHANGELOG](CHANGELOG.md) for the full history.

- **New flagship example `ntn-v2x-edge-urllc`** — a V2X platoon on a real LEO mmwave NR cell with a URLLC platoon-control slice (5QI 82) and an eMBB sensor slice, a hazard-detection xApp consuming measured KPM features every 100 ms, and `--edge=sat` vs `--edge=ground` inference placement. The measured decision latency (feature sense → brake command applied) is the experiment outcome, alongside 5GAA-style measured one-way-delay / PDR KPIs.
- **New `ntn-v2x-real-stack` example** — every vehicle is a UE on a real mmwave NR NTN cell; the direct-vs-relay decision rides the measured per-UE DL SINR, with the superseded `V2xLeoDirect` closed-form value printed next to it for comparison.
- **`ntn-v2x-rural-highway` converted to measured radio** — the direct/relay/orphan split is now driven by the measured baseline SINR of a real cell (minus per-vehicle blockage), not the closed-form budget; the CSV's last column is now `measured_baseline_db`.
- **`ntn-v2x-leo-relay-traffic` migrated to `NtnOranApplication`** — BSM rate / latency / jitter / loss are measured end-to-end by `NtnOranSink` from the in-band payload header (`OnOffApplication` and FlowMonitor are gone).

## Models, helpers & key classes

| Header | Provides |
|---|---|
| `model/sumo-traci-bridge.h` | `SumoTraciBridge` — single API for live TraCI (`ConnectTcp("127.0.0.1", 8813)`) and offline FCD replay (`LoadFcdTrace(csv)`); `RegisterVehicle()`, `Step()` advances regardless of source; tracks per-step sync jitter (`GetLastJitterSec()` / `GetMaxJitterSec()`) and exports per-vehicle samples via a `TracedCallback`. |
| `model/ntn-nr-sidelink.h` | NR PC5 sidelink Mode 2 (TS 38.321 §5.22). `NtnSlResourcePool` — the SL-ResourcePool (subchannels × slots, TS 38.331). `NtnSlUeMac` — one vehicle's SL MAC with sensing-based autonomous selection (candidate set in the T1..T2 selection window, exclusion of resources a sensed neighbour reserved above an SL-RSRP threshold, the ≥20%-remaining rule with the +3 dB step, uniform-random pick, periodic reservation driven by a reselection counter). `NtnSlChannel` — the shared PC5 medium: log-distance SL-RSRP, half-duplex (a UE cannot receive in a slot it transmits in), co-channel collisions, and TS 38.885 PRR (`GetPrr`, `GetPrrWithinRange`). MAC / resource-pool abstraction only — it does **not** model PSCCH/PSSCH bit-level decoding, HARQ combining, or 2nd-stage SCI formats. |
| `model/ntn-v2x-bsm-header.h` | `ntnv2x::NtnV2xBsmHeader` — a real SAE J2735 (2016) BSM Part I core (BSMcoreData) as an ns-3 `Header`: serialise/deserialise of `msgCnt`/`id`/`secMark`/`lat`/`lon`/`elev`/`speed`/`heading` in J2735 encoding resolutions (1/10 µ-degree, 1 dm, 0.02 m/s, 0.0125°); 23-byte wire core, `SetFromState()` fills it from SI quantities. So a BSM carries genuine vehicle state, not opaque padding. |
| `model/v2x-leo-direct.h` | `V2xLeoDirect` — closed-form vehicle ↔ LEO uplink budget (free-space PL + SNR + elevation); `ComputeStatic` callable inside hot loops without `Object` allocation overhead. Kept as a reference baseline — the examples decide on measured SINR. |
| `model/v2x-leo-relay.h` | `V2xLeoRelay` — V2V-via-LEO relay assignment; each vehicle uplinks direct or relays through the best peer within `m_maxV2vRangeM`; `m_minDirectSnrDb` sets the prefer-relay threshold; `EvaluateAll()` returns the per-vehicle decision. |
| `model/maritime-scenario.h` | `MaritimeMobilityModel` — DEPRECATED / offline-test-only synthetic box-bounce vessel motion. The real maritime path reuses ntn-sagin `AisMobilityModel` (recorded AIS replay) via the `ntn-v2x-maritime-ais` example. |
| `helper/ntn-v2x-helper.h` | Deterministic CI fixture writer `WriteDeterministicTestFcdCsv` — a disclosed, RNG-free FCD-format CSV trace for the test suite only (fixed per-vehicle speed ramp). The shipped example traces are likewise synthetic constant-speed FCD-format CSV fixtures (not SUMO microsimulations; the loader reads a CSV dialect, not SUMO's native fcd-output XML). Examples require an `--fcdTrace`/`--trace` CSV file (see `traces/`); a CSV converted from a SUMO fcd-export also works. |

## Examples

Build all examples with `./ns3 configure --enable-examples --enable-tests && ./ns3 build`. Each example produces the binary `build/contrib/ntn-v2x/examples/ns3.43-<name>-default`.

### ntn-v2x-edge-urllc  *(flagship)*

A two-vehicle highway platoon (100 km/h, 50 m headway) rides a real mmwave NR NTN cell from an SGP4 LEO:

- **URLLC slice (5QI 82, SST 2):** periodic platoon-control messages — 5GAA-style KPIs (one-way delay against a < 100 ms target, PDR, jitter) measured in-band by `NtnOranSink`;
- **eMBB slice (5QI 2, SST 1):** sensor/diagnostics upload.

A hazard-detection xApp (`OranNtnOnnxXapp`: an `.onnx` model when ONNX Runtime is built in, a registered heuristic otherwise) consumes the URLLC flow's measured KPM feature window every 100 ms and decides whether to issue a brake command. The inference runs `--edge=sat` (on the regenerative payload, processing-only E2 latency) or `--edge=ground` (slant-range/c + core), via `OranNtnRicPlacement`; the decision rides the placement latency both ways (KPM up, command down), so the **measured decision latency** is the sat-vs-ground experiment outcome. Run both and compare:

```bash
./ns3 run "ntn-v2x-edge-urllc --edge=sat"
./ns3 run "ntn-v2x-edge-urllc --edge=ground"
```

Outputs:
- Console: 5GAA-style measured KPIs for the control flow (one-way delay vs the 100 ms target, PDR, jitter) and a summary with inference mode, decision count, mean decision latency, brake commands and measured cell SINR.
- `sim_health.csv` and `kpm_series.csv` in `--outputDir`.

Key args: `--simSeconds` (sim duration, s) · `--edge` (inference placement: `sat` | `ground`) · `--outputDir`.

### ntn-v2x-pc5-sidelink-bsm

A line of highway vehicles broadcast SAE J2735 BSMs **directly over PC5** with NR sidelink Mode 2 — autonomous, sensing-based resource selection, **no gNB in the loop**. Each UE selects a single-subchannel resource in its selection window, reserves it periodically, and the shared `NtnSlChannel` resolves half-duplex and co-channel collisions to produce the TS 38.885 KPI: Packet Reception Ratio (PRR) vs distance. This is the genuine direct-V2V safety layer, in contrast to `ntn-v2x-real-stack` / `ntn-v2x-leo-relay-traffic`, which route V2X through a satellite gNB (Uu / P2P relay).

```bash
./ns3 run "ntn-v2x-pc5-sidelink-bsm --numVehicles=20 --numSubchannels=5 --duration=4"
```

Outputs: console summary of total TX BSMs, RX attempts/successes, and PRR bucketed by distance (within 50/100/200/300/500 m). As vehicle density rises the resource pool saturates and PRR degrades gracefully — e.g. ≈1.00 at 10 vehicles / 5 subchannels, falling to ≈0.96 at 60 vehicles.

Key args: `--numVehicles` (vehicles on the highway) · `--numSubchannels` (SL-ResourcePool subchannels) · `--spacingM` (inter-vehicle spacing, m) · `--bsmPeriodMs` (BSM period / reservation interval, ms) · `--duration` (s) · `--txPowerDbm` (PC5 Tx power, dBm).

### ntn-v2x-rural-highway

A LEO pass over a rural highway: vehicles replayed from a SUMO FCD trace (`SumoTraciBridge`, trace-replay mode, so CI needs no live SUMO) decide direct-vs-relay-vs-orphan each tick. The LEO link quality is measured off a real mmwave NR cell over representative highway terminals; each vehicle's direct uplink SINR is that measured baseline minus its NLOS blockage, so the direct/relay/orphan split is driven by the measured radio.

```bash
./ns3 run "ntn-v2x-rural-highway --vehicles=40 --simTime=30"
```

Outputs:
- Per-second CSV at `<outputDir>/ntn-v2x-rural-highway.csv` with columns `time_s,n_direct,n_relay,n_orphan,direct_pct,jitter_ms,measured_baseline_db`.
- `sim_health.csv` in `--outputDir`.

Key args: `--vehicles` (number of vehicles) · `--simTime` (sim duration, s) · `--numCellUes` (representative real-cell terminals) · `--satEirpDbm` (satellite EIRP / gNB Tx power, dBm) · `--blockageDb` (NLOS blockage on shadowed vehicles, dB) · `--dt` (TraCI tick, s) · `--trace` (FCD CSV trace path; generated if missing) · `--minDirectSnr` (minimum dB for direct uplink) · `--maxV2vRange` (max V2V range, m, for relay) · `--outputDir`.

### ntn-v2x-real-stack

Every vehicle is a UE on a real mmwave NR NTN cell (SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC), so each vehicle's direct LEO link quality is the measured per-UE DL SINR. NLOS vehicles carry an additional blockage loss; a vehicle whose measured-minus-blockage SINR falls below the threshold relays through the nearest in-range peer. A mid-run elevation descent collapses the baseline so relay decisions actually flip over the pass. The summary prints the old `V2xLeoDirect` closed-form SNR next to the measured value.

```bash
./ns3 run "ntn-v2x-real-stack --duration=20 --numVehicles=8"
```

Outputs: console summary (closed-form vs measured UE0 SINR, measured cell SINR/throughput, direct/relay/outage decision counts); `sim_health.csv` in `--outputDir`.

Key args: `--duration` (s) · `--numVehicles` · `--altitude` (satellite altitude, km) · `--satEirpDbm` · `--blockageDb` (NLOS blockage on shadowed vehicles, dB) · `--minDirectSnr` (dB) · `--maxV2vRange` (m) · `--outputDir`.

### ntn-v2x-leo-relay-traffic

A real 2-hop data plane: a shadowed vehicle (veh0) relays basic-safety messages through a peer vehicle (veh1) and a LEO satellite to a ground server (`veh0 → veh1 → LEO → server`), with real IPv4 forwarding. Each hop's propagation delay is the real slant range / c, and a hop carries traffic only while in contact — the V2V hop within range, the uplink above the minimum elevation. Delivered BSM rate / latency / jitter / loss are measured end-to-end by `NtnOranSink` from the in-band `NtnOranPayloadHeader`; the `V2xLeoRelay` engine logs when veh0 must relay vs go direct.

```bash
./ns3 run "ntn-v2x-leo-relay-traffic --simSeconds=60 --bsmHz=10"
```

Outputs: per-second contact/decision trace and a measured end-to-end summary (BSM txPackets, rxPackets, PDR, delay, jitter) on the console.

Key args: `--simSeconds` (s) · `--bsmHz` (basic-safety-message rate, Hz) · `--bsmBytes` (BSM payload, bytes) · `--leoAltKm` · `--satSpeed` (LEO ground-track speed, m/s) · `--relayDriftMps` (relay vehicle relative speed, m/s) · `--maxV2vRange` (m) · `--minDirectSnr` (dB) · `--linkCapacityMbps` (per-hop P2P capacity) · `--outputDir`.

## Build, run & test

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build
./test.py -s ntn-v2x
```

The `ntn-v2x` suite has 7 unit tests: trace-replay sync jitter under the 100 ms gate, V2X-LEO direct free-space path loss within 0.1 dB of the closed form, relay fall-back to a peer when direct SNR is below threshold, maritime bounded mobility, a 100-vehicle / 5-min replay that must complete inside the test budget, a SAE J2735 BSM header round-trip through a packet (each field within its encoding resolution), and `NtnSidelinkMode2Test` — NR PC5 Mode-2 selection with the half-duplex rule (no self-reception), high in-range PRR under a wide pool, and measurably lower PRR under a degenerate 1-subchannel pool (proving the sensing + collision model is real).

### Live SUMO co-simulation

`ConnectTcp()` opens a real TCP connection to a TraCI endpoint (and fails gracefully when no SUMO is listening), but the live wire codec is deliberately minimal: live `Step()` only advances the clock without decoding TraCI messages. FCD replay (`LoadFcdTrace(csv)`) is therefore the supported path. The public API is identical for both modes, so a full TraCI client can be dropped in later without changing callers.

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
