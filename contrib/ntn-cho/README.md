# ntn-cho

> Time-to-Exit (TTE)-aware 3GPP Rel-17 Conditional Handover for LEO satellite NTN, in ns-3.43.

- ns-3 version: `release ns-3.43`
- Version: `1.0.0`
- License: GPL-2.0-only
- Maintainer: Muhammad Uzair, Independent Researcher (ORCID 0009-0002-4104-2680)

See [INSTALL.md](INSTALL.md) for setup and the dependency list. This module
is also distributed as part of the
[ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit).

## Dependencies

- **Required:** the SNS3 [`satellite`](https://github.com/sns3/sns3-satellite)
  module — `NtnOrbitPredictor`/`NtnTteEstimator` use its `SatSGP4MobilityModel`
  and antenna-gain patterns for SGP4 propagation and beam geometry.
- **Examples:** the real-stack examples (`ntn-cho-leo-basic`,
  `ntn-cho-handover-traffic`, `ntn-cho-real-stack`) additionally use the
  sibling toolkit modules `ntn-traffic` (`NtnRealStackHelper` — real mmwave
  NR NTN cell with measured traffic) and `ntn-constellation`
  (`Sgp4MobilityModel` + `WalkerConstellation`), plus the in-tree `mmwave`
  and `lte` stacks. `ntn-cho-full-constellation` uses
  `NtnRealisticTrafficHelper` from `ntn-traffic`; in the standalone App Store
  package this helper is **vendored into the module** (self-contained).
- **Optional:** the `ns3-ai` module, only for `NtnAiInterface` (the
  learning-based path). The C++ triggers build and run without it.

## Overview

`ntn-cho` implements **3GPP Release-17 Conditional Handover (CHO)** for **Non-Terrestrial Networks (NTN)**, with a focus on LEO satellite constellations where rapid beam-coverage changes drive frequent, often premature, handovers. The module adds a **Time-to-Exit (TTE)-aware** candidate selection that admits a target beam only when it will stay in coverage long enough to be worth the switch. Alongside the TTE-aware novelty it implements the NTN handover trigger classes with precise standards positioning — **Rel-17 normative CondEvents** measurement-based A4-style (event A3 baseline), location-based (CondEventD1) and time-based (CondEventT1, ephemeris-scheduled), the **Rel-18 CondEventD2** (distance with MOVING ephemeris-derived reference locations, TS 38.331 §5.5.4.15a), and the **TR 38.821 §6-studied** elevation-based and timing-advance-based mechanisms (studied, not standardized CondEvents) — plus two forward-looking mechanisms: **Rel-19 conditional LTM** (L1-filtered measurements with a MAC-CE-style fast cell switch) and **trajectory-predictive CHO** (forecast serving outage, maximum predicted time-of-stay), with optional **RACH-less execution** from ephemeris/GNSS TA pre-compensation. It is built around a 3GPP-aligned CHO state machine, an orbit/beam predictor, and a 3GPP TR 38.811 NTN measurement model so that handover decisions fall out of live geometry rather than hardcoded scripts.

## What's new

See the [CHANGELOG](CHANGELOG.md).

- **Six NTN trigger classes** in `NtnChoAlgorithm` (Rel-17 A3/D1/T1 + Rel-18 D2 + TR 38.821-studied elevation/TA; `combineWithA4` enforces the Rel-17 rule that T1/D1/D2 are configured together with the A4 measurement leg):
  `TRIGGER_EVENT_A3`, `TRIGGER_LOCATION_D1`, `TRIGGER_TIME_T1` (CondEventT1
  handover window from the serving cell's remaining time-of-service),
  `TRIGGER_ELEVATION` (serving elevation below `elevationMinDeg`, candidate
  above floor + `elevationHystDeg`, elevation derived from the ephemeris/GNSS
  slant range at `orbitAltitudeKm`), and `TRIGGER_TIMING_ADVANCE` (serving TA
  above `taServingMax`, or a candidate at least `taAdvantage` lower). For
  these classes the admitted set already encodes the standardized condition,
  so selection takes the strongest **measured** candidate (max SINR).
- **Real-stack examples** — `ntn-cho-leo-basic`, `ntn-cho-handover-traffic`
  and the new `ntn-cho-real-stack` run on a real mmwave NR NTN cell
  (`NtnRealStackHelper`: SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC) with
  SGP4 Walker satellite orbits and 3GPP TR 38.811 §6.1.1.1 class UE mobility;
  the serving SINR is **measured off the mmwave PHY trace**, not closed-form.
- **`NtnTr38811MobilityModel`** — the TR 38.811 UE classes are now a real
  ns-3 `MobilityModel` (ECEF, SGP4-compatible frame), so the per-class motion
  drives the radio stack, Doppler, and the TTE estimator directly.
- **Standards-validation test suite** (`ntn-standards-validation`) — checks
  the mobility architecture against orbital theory and published NTN figures
  (see *Build, run & test* below).
- **Doppler is SIGNED** in the measurement model — the shift flips from
  positive to negative across a LEO pass, so approaching vs. receding
  geometry is modelled correctly.
- **CSV sentinel hygiene** — no `serving_sat=4294967295` or `sinr=-100`
  sentinel values leaking into `ue_tracks`, `handover_events`, or
  `kpi_timeseries`; `avg_sinr` averages only currently-served UEs; the
  first-handover `time_of_stay` is no longer inflated.

## Models, helpers & key classes

Model (`model/`):

- `NtnChoAlgorithm` (`ntn-cho-algorithm.h`) — 3GPP Rel-17 CHO algorithm with TTE-aware candidate selection and the `CHO_IDLE → CHO_PREPARED → CHO_CONDITION_MONITORING → CHO_EXECUTING → CHO_COMPLETED` state machine. Trigger types: `TRIGGER_EVENT_A3`, `TRIGGER_LOCATION_D1`, `TRIGGER_TIME_BASED`, `TRIGGER_TTE_AWARE`, `TRIGGER_THZ_BEAM_QUALITY`, `TRIGGER_LTM_CONDITIONAL` (Rel-19 conditional LTM), `TRIGGER_TRAJECTORY_PREDICTIVE` (PCHO), and the standardized NTN classes `TRIGGER_TIME_T1`, `TRIGGER_ELEVATION`, `TRIGGER_TIMING_ADVANCE`. `ChoConfig` carries the per-class parameters (`t1WindowDuration`, `elevationMinDeg`/`elevationHystDeg`, `orbitAltitudeKm`, `taServingMax`/`taAdvantage`, the LTM/PCHO knobs, and `rachLess` + `rachDuration`/`choExecutionDelay` for RACH-less execution); `GetMechanismStats()` reports LTM switches, PCHO triggers, RACH-less vs RACH executions, and per-handover interruption.
- `NtnTteEstimator` (`ntn-tte-estimator.h`) — estimates Time-to-Exit for satellite beam coverage, per-candidate and in batch.
- `NtnOrbitPredictor` (`ntn-orbit-predictor.h`) — predicts satellite/beam positions and coverage over time and reports visible satellites and best beams per UE position.
- `NtnMeasurementModel` (`ntn-measurement-model.h`) — computes RSRP/SINR from satellite beams using the 3GPP TR 38.811 NTN channel scenarios.
- `NtnTr38811MobilityModel` (`ntn-tr38811-mobility-model.h`) — the 3GPP TR 38.811 §6.1.1.1 NTN UE classes (handheld static/pedestrian, vehicular, HST, maritime, aviation, fixed IoT) as a **real ns-3 `MobilityModel`** in ECEF, the same frame as the satellite side's SGP4 models; includes the `ntngeo` geometry utilities (geodetic↔ECEF, elevation, slant range) and `NtnTr38811MobilityHelper` for installing UE populations.
- `NtnAiInterface` (`ntn-ai-interface.h`) — ns3-ai shared-memory bridge exposing a candidate-cell observation/action space for AI-driven handover decisions.

Helper (`helper/`):

- `NtnChoHelper` (`ntn-cho-helper.h`) — top-level helper that wires up a CHO scenario (channel scenario, trigger type, carrier frequency) and reports aggregated KPI results.
- `NtnRealisticMobilityHelper` (`ntn-realistic-mobility.h`) — generates UE populations with realistic per-class motion following the seven 3GPP TR 38.811 §6.1.1.1 NTN UE classes, with built-in scenario profiles (`NtnMobilityScenarios`).

## Examples

All five examples build under `build/contrib/ntn-cho/examples/`. Each can be launched either through `./ns3 run` or directly via the built binary with `LD_LIBRARY_PATH=build/lib`.

### ntn-cho-leo-basic

Smoke test for the CHO algorithm on a **real mmwave NR NTN cell** (`NtnRealStackHelper`): one SGP4 serving satellite, TR 38.811 class UEs under its sub-point, real UDP traffic over the radio, and the CHO algorithm exercised on a 200 ms cadence with the **measured** per-UE SINR from the mmwave PHY trace.

```bash
./ns3 run "ntn-cho-leo-basic --trigger=tte-aware --simTime=12 --numUes=4"
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-cho/examples/ns3.43-ntn-cho-leo-basic-default --trigger=tte-aware --simTime=12 --numUes=4
```

Outputs: `sim_health.csv` (written via `NtnRealStackHelper::WriteHealthReport()`) in `--outputDir`, plus the measured mean SINR and throughput on stdout.
Key args: `simTime`, `trigger` (a3|location|tte-aware), `tteMinimum`, `numUes`, `satEirpDbm`, `outputDir`.

### ntn-cho-full-constellation

Full Walker constellation NTN-CHO run: multi-beam satellites, proper initial serving assignment, calibrated TTE values and a realistic HO-failure model, with the four algorithms (a3 / location / time / tte-aware) selectable for comparison. A real UDP data plane (via `NtnRealisticTrafficHelper`) runs alongside the constellation-scale measurement loop.

```bash
./ns3 run "ntn-cho-full-constellation --algorithm=tte-aware --numUes=50 --outputDir=/tmp/ntn-full"
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-cho/examples/ns3.43-ntn-cho-full-constellation-default --algorithm=tte-aware --numUes=50 --outputDir=/tmp/ntn-full
```

Outputs (in `--outputDir`): `handover_events.csv`, `measurements.csv`, `tte_computations.csv`, `satellite_tracks.csv`, `ue_tracks.csv`, `kpi_timeseries.csv`, `kpi_summary.txt`, the GeoJSON layers (`satellite_positions.geojson`, `ue_positions.geojson`, `beam_footprints.geojson`, `handover_events.geojson`), and `sim_health.csv` (via `NtnRealisticTrafficHelper`).
Key args: `simTime`, `numUes`, `scenario`, `algorithm` (a3|location|time|tte-aware), `d1Threshold`, `qualityTh`, `tteMinimum`, `outputDir`, `rngRun`, `verbose`, `numPlanes`, `satsPerPlane`, `trafficUes` (UEs carrying the real UDP plane; 0 = all).

### ntn-cho-handover-traffic

Real UDP downlink to TR 38.811 UEs on a **real mmwave NR NTN cell**, handed over by the actual `NtnChoAlgorithm` while the constellation flies real SGP4 Walker orbits: the serving satellite passes zenith and recedes, the in-plane neighbour approaches, and the handover falls out of the genuine orbital crossover. The serving SINR fed to the algorithm is **measured** from the PHY; the candidate SINR is ephemeris-predicted (measured baseline plus the real Friis slant-range ratio). All nine trigger mechanisms are selectable — including Rel-17 A3/D1/T1, Rel-18 D2 and the TR 38.821-studied elevation/TA classes.

```bash
./ns3 run "ntn-cho-handover-traffic --simSeconds=60 --trigger=elevation"
./ns3 run "ntn-cho-handover-traffic --trigger=t1 --rachLess=1"
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-cho/examples/ns3.43-ntn-cho-handover-traffic-default --simSeconds=60 --trigger=ta
```

Outputs: `sim_health.csv` in `--outputDir`, per-handover lines on stdout (measured serving SINR, predicted candidate SINR, interruption), and a summary line with the measured serving-cell goodput across the handover, mean SINR, SINR at handover, last interruption, and RACH-less execution count.
Key args: `simSeconds`, `numUes`, `leoAltKm`, `freqGHz`, `satEirpDbm`, `tteMinSec`, `trigger` (tte-aware|ltm|pcho|a3|d1|t1|elevation|ta), `rachLess`, `satsPerPlane`, `outputDir`.

### ntn-cho-real-stack

Real-stack flagship: the TTE-aware CHO (or Rel-19 LTM / trajectory-predictive PCHO) decides on the **measured** mmwave SINR while serving and candidate satellites fly real SGP4 Walker-Delta orbits and the UEs move under TR 38.811 class mobility. The summary reports the mechanism counters (`GetMechanismStats()`): LTM fast switches, PCHO trajectory triggers, RACH-less vs RACH executions, last interruption, and the last ephemeris-pre-computed TA.

```bash
./ns3 run "ntn-cho-real-stack --duration=60 --trigger=pcho --rachLess=1"
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-cho/examples/ns3.43-ntn-cho-real-stack-default --duration=60 --trigger=pcho --rachLess=1
```

Outputs: `sim_health.csv` in `--outputDir` and the CHO/mechanism summary on stdout (measured mean SINR, measured DL throughput, evaluations, handovers, mechanism counters).
Key args: `duration`, `numUes`, `altitude`, `satEirpDbm`, `freqGhz`, `tteMin`, `trigger` (tte-aware|ltm|pcho), `rachLess`, `satsPerPlane`, `outputDir`.

### ntn-realistic-mobility-demo

Demonstrates the per-class realistic mobility generator: spawns one UE per 3GPP TR 38.811 §6.1.1.1 class and writes its trajectory to CSV for inspection/plotting.

```bash
./ns3 run "ntn-realistic-mobility-demo --outputDir=/tmp/mob_demo --simTime=600"
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-cho/examples/ns3.43-ntn-realistic-mobility-demo-default --outputDir=/tmp/mob_demo --simTime=600
```

Outputs: `mobility_trace.csv` in `--outputDir`.
Key args: `outputDir`, `simTime`, `dt`, `rngRun`.

## Build, run & test

```bash
./ns3 configure --enable-examples --enable-tests && ./ns3 build
./test.py -s ntn-cho
./test.py -s ntn-standards-validation
```

Two test suites ship with the module:

- `ntn-cho` — unit tests for the CHO algorithm, the CHO state machine, and the NTN measurement model.
- `ntn-standards-validation` — validates the mobility architecture against orbital theory and published NTN figures: SGP4 propagation vs Keplerian theory (orbital radius, speed `sqrt(mu/a)`, quarter-period arc at 550 km / 53°), the zenith-pass Doppler envelope (Doppler null at culmination, S-band shift inside the published LEO envelope), ENU pass geometry (culmination at zenith, monotonic elevation decay), and the spherical-Earth elevation/slant relation used by the elevation trigger — including the TR 38.821 LEO-600 reference point (~1932 km slant at 10° elevation).

See [INSTALL.md](INSTALL.md) for full setup.

## Citing

```bibtex
@misc{uzair2026ntncho,
  author = {Muhammad Uzair},
  title  = {ntn-cho: Time-to-Exit-Aware Conditional Handover for
            Non-Terrestrial Networks in ns-3},
  year   = {2026},
  note   = {ns-3 App Store module, v1.0.0. ORCID 0009-0002-4104-2680}
}
```

## License & author

GPL-2.0-only. See `LICENSE`.

Author: Muhammad Uzair, Independent Researcher (ORCID 0009-0002-4104-2680).
