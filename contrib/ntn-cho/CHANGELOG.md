# Changelog

All notable changes to this module are documented here. The format is
based on [Keep a Changelog](https://keepachangelog.com/) and this
project adheres to Semantic Versioning.

## [1.0.0] — 2026-04-23

### Added

- Initial public release of `ntn-cho`.

- **`NtnChoAlgorithm`** — 3GPP Release-17 conditional-handover state
  machine (`CHO_IDLE → CHO_PREPARED → CHO_CONDITION_MONITORING →
  CHO_EXECUTING → CHO_COMPLETED`) with selectable triggers:
  event-A3, location (condEventD1), time-based, and the proposed
  TTE-aware candidate selection.

- **`NtnTteEstimator`** — Time-to-Exit estimator. Propagates the
  candidate satellite forward (via the SNS3 `satellite` module's SGP4
  mobility) and refines the beam-exit instant with a binary search;
  per-candidate and batch queries.

- **`NtnOrbitPredictor`** — wraps the `satellite` module's
  `SatSGP4MobilityModel` and antenna-gain-pattern container to predict
  satellite/beam positions, report visible satellites, and pick the
  best beam per UE position.

- **`NtnMeasurementModel`** — RSRP / SINR / elevation / Doppler from
  satellite beams using 3GPP TR 38.811 NTN channel scenarios.

- **`NtnAiInterface`** — optional ns3-ai shared-memory bridge exposing
  a candidate-cell observation/action space for AI-driven handover
  decisions. Requires the `ns3-ai` module; the C++ triggers run
  without it.

- **`NtnChoHelper`** — top-level helper that wires up a CHO scenario
  (channel scenario, trigger type, carrier frequency) and reports
  aggregated KPIs.

- **`NtnRealisticMobilityHelper`** (+ `NtnMobilityScenarios`) — UE
  populations with per-class motion for the seven 3GPP TR 38.811
  §6.1.1.1 NTN UE classes, with built-in scenario profiles.

- **Four example scripts** under `examples/`:
  - `ntn-cho-leo-basic` — event-driven smoke test with real UDP
    traffic through the ns-3 stack.
  - `ntn-cho-full-constellation` — multi-satellite Walker constellation
    (built with the `satellite` module) running any of the
    `a3 / location / time / tte-aware` algorithms, writing per-event
    CSV/GeoJSON and a KPI summary.
  - `ntn-cho-handover-traffic` — a UDP flow that follows the UE across
    a satellite handover over real PointToPoint links, with FlowMonitor
    confirming continuity across the switch.
  - `ntn-realistic-mobility-demo` — the seven-class mobility helper.

- **One unit-test suite** (`test/ntn-cho-test-suite.cc`, suite name
  `ntn-cho`) covering the CHO algorithm, the CHO state machine, and the
  NTN measurement model.

### Dependencies

- Requires the SNS3 [`satellite`](https://github.com/sns3/sns3-satellite)
  module (SGP4 mobility + antenna-gain patterns).
- `NtnAiInterface` additionally requires the `ns3-ai` module; it is only
  needed for the learning-based path.

### Notes

- SGP4 propagation and the Walker constellation geometry are provided
  by the `satellite` module — this module **consumes** them, it does not
  re-implement them.
