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
- **Traffic helper:** the `ntn-cho-leo-basic` and `ntn-cho-full-constellation`
  examples use `NtnRealisticTrafficHelper`. In the standalone App Store package
  this helper is **vendored into the module** (self-contained); inside
  `ns3-ntn-toolkit` it is provided by the sibling `ntn-traffic` module.
- **Optional:** the `ns3-ai` module, only for `NtnAiInterface` (the
  learning-based path). The C++ triggers build and run without it.

## Overview

`ntn-cho` implements **3GPP Release-17 Conditional Handover (CHO)** for **Non-Terrestrial Networks (NTN)**, with a focus on LEO satellite constellations where rapid beam-coverage changes drive frequent, often premature, handovers. The module adds a **Time-to-Exit (TTE)-aware** candidate selection that admits a target beam only when it will stay in coverage long enough to be worth the switch, alongside event-A3, location, and time triggers for comparison. It is built around a 3GPP-aligned CHO state machine, an orbit/beam predictor, and a 3GPP TR 38.811 NTN measurement model so that handover decisions fall out of live geometry rather than hardcoded scripts.

## What's new in v2

See the [CHANGELOG](CHANGELOG.md).

- **Doppler is now SIGNED** — the shift flips from positive to negative across a LEO pass (previously magnitude-only), so approaching vs. receding geometry is modelled correctly.
- **CSV sentinel hygiene** — no more `serving_sat=4294967295` or `sinr=-100` sentinel values leaking into `ue_tracks`, `handover_events`, or `kpi_timeseries`; `avg_sinr` now averages only currently-served UEs; and the first-handover `time_of_stay` is no longer inflated.

## Models, helpers & key classes

Model (`model/`):

- `NtnChoAlgorithm` (`ntn-cho-algorithm.h`) — 3GPP Rel-17 CHO algorithm with TTE-aware candidate selection and the `CHO_IDLE → CHO_PREPARED → CHO_CONDITION_MONITORING → CHO_EXECUTING → CHO_COMPLETED` state machine; supports a3 / location / time / tte-aware triggers.
- `NtnTteEstimator` (`ntn-tte-estimator.h`) — estimates Time-to-Exit for satellite beam coverage, per-candidate and in batch.
- `NtnOrbitPredictor` (`ntn-orbit-predictor.h`) — predicts satellite/beam positions and coverage over time and reports visible satellites and best beams per UE position.
- `NtnMeasurementModel` (`ntn-measurement-model.h`) — computes RSRP/SINR from satellite beams using the 3GPP TR 38.811 NTN channel scenarios.
- `NtnAiInterface` (`ntn-ai-interface.h`) — ns3-ai shared-memory bridge exposing a candidate-cell observation/action space for AI-driven handover decisions.

Helper (`helper/`):

- `NtnChoHelper` (`ntn-cho-helper.h`) — top-level helper that wires up a CHO scenario (channel scenario, trigger type, carrier frequency) and reports aggregated KPI results.
- `NtnRealisticMobilityHelper` (`ntn-realistic-mobility.h`) — generates UE populations with realistic per-class motion following the seven 3GPP TR 38.811 §6.1.1.1 NTN UE classes, with built-in scenario profiles (`NtnMobilityScenarios`).

## Examples

All four examples build under `build/contrib/ntn-cho/examples/`. Each can be launched either through `./ns3 run` or directly via the built binary with `LD_LIBRARY_PATH=build/lib`.

### ntn-cho-leo-basic

Smoke test for the realistic event-driven path: spawns a few UEs, runs real UDP traffic through the ns-3 stack toward a remote host, and exercises the CHO algorithm on a 200 ms cadence so `Simulator::Run()` advances in proportion to `simTime`.

```bash
./ns3 run "ntn-cho-leo-basic --trigger=tte-aware --trafficProfile=mixed --simTime=120"
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-cho/examples/ns3.43-ntn-cho-leo-basic-default --trigger=tte-aware --trafficProfile=mixed --simTime=120
```

Outputs: `sim_health.csv` (written via `NtnRealisticTrafficHelper::WriteHealthReport()`) in `--outputDir`.
Key args: `simTime`, `scenario` (dense-urban|urban|suburban|rural), `trigger` (a3|location|tte-aware), `d1Threshold`, `qualityTh`, `tteMinimum`, `numUes`, `outputDir`, `trafficProfile` (nb-iot|embb|urllc|dt|mixed), `strict`.

### ntn-cho-full-constellation

Full Walker constellation NTN-CHO run: multi-beam satellites, proper initial serving assignment, calibrated TTE values and a realistic HO-failure model, with the four algorithms (a3 / location / time / tte-aware) selectable for comparison.

```bash
./ns3 run "ntn-cho-full-constellation --algorithm=tte-aware --numUes=50 --outputDir=/tmp/ntn-full"
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-cho/examples/ns3.43-ntn-cho-full-constellation-default --algorithm=tte-aware --numUes=50 --outputDir=/tmp/ntn-full
```

Outputs (in `--outputDir`): `handover_events.csv`, `measurements.csv`, `tte_computations.csv`, `satellite_tracks.csv`, `ue_tracks.csv`, `kpi_timeseries.csv`, `kpi_summary.txt`, the GeoJSON layers (`satellite_positions.geojson`, `ue_positions.geojson`, `beam_footprints.geojson`, `handover_events.geojson`), and `sim_health.csv` (via `NtnRealisticTrafficHelper`).
Key args: `simTime`, `numUes`, `scenario`, `algorithm` (a3|location|time|tte-aware), `d1Threshold`, `qualityTh`, `tteMinimum`, `outputDir`, `rngRun`, `verbose`, `numPlanes`, `satsPerPlane`.

### ntn-cho-handover-traffic

Real UDP downlink to a ground UE that is handed over between two passing LEO satellites by the actual `NtnChoAlgorithm`. Each second the per-satellite SINR is computed from live geometry, fed to the algorithm, and the chosen satellite's PointToPoint link is opened (the other closed) — so the data plane follows the CHO decision and FlowMonitor shows the UDP flow surviving the handover.

```bash
./ns3 run "ntn-cho-handover-traffic --simSeconds=120 --tteMinSec=20 --dataRateMbps=10"
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-cho/examples/ns3.43-ntn-cho-handover-traffic-default --simSeconds=120 --tteMinSec=20 --dataRateMbps=10
```

Outputs: no CSV; prints FlowMonitor flow statistics (delivered goodput across the handover) to stdout.
Key args: `simSeconds`, `leoAltKm`, `satSpeed`, `freqGHz`, `dataRateMbps`, `packetBytes`, `txPowerDbm`, `antennaGainDb`, `tteMinSec`, `linkCapacityMbps`.

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
./build/utils/ns3.43-test-runner-default --suite=ntn-cho
```

The `ntn-cho` suite covers the CHO algorithm, the CHO state machine, and the NTN measurement model. See [INSTALL.md](INSTALL.md) for full setup.

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
