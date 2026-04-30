# Changelog

All notable changes to this module are documented here. The format is
based on [Keep a Changelog](https://keepachangelog.com/) and this
project adheres to Semantic Versioning.

## [1.0.0] — 2026-04-23

### Added
- Initial public release of `ntn-cho`.
- TTE (Time-to-Exit) estimator `NtnTteEstimator` with a cached
  `O(log n)` binary search over precomputed beam-dwell trajectories.
- Four CHO baselines:
  - `NtnTteCondHandoverAlgorithm` (proposed, TTE-aware)
  - `NtnA3CondHandoverAlgorithm` (event-A3 baseline)
  - `NtnTimeBasedCondHandoverAlgorithm` (time-to-trigger baseline)
  - `NtnLocationBasedCondHandoverAlgorithm` (geometry-only baseline)
- SGP4 orbit propagation (`NtnSgp4Mobility`) and Walker-Star
  constellation generator (`NtnConstellationHelper`).
- mmWave + satellite integration classes:
  `MmWaveSatChannel`, `MmWaveSatPropagationLossModel`,
  `MmWaveSatSpectrumPropagationLossModel`, `MmWaveSatUeNetDevice`.
- Kinematics + geometry utilities: ECI ↔ ECEF ↔ topocentric (ENU)
  conversion helpers.
- 12 example scripts under `examples/`, including
  `ntn-cho-scenario-a`, `ntn-cho-scenario-b-mc`, and
  `ntn-cho-walker-star`.
- Unit tests under `test/` covering geometry, SGP4 drift over one
  orbit, CHO state transitions, and TTE monotonicity.
- Monte-Carlo harness
  (`examples/mc_runner.cc` + `papers/sim_runs/run_mc_sweep.sh`)
  producing per-seed KPIs with 95 % CIs.
- Visualisation helpers under `visualization/`
  (`track_plotter.py`, `beam_dwell_plot.py`).
