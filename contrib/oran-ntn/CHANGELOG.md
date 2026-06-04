# Changelog

All notable changes to this module are documented here. The format
follows Keep a Changelog and the project uses semantic versioning.

## [2.0.0] — 2026-06

### Changed (v2 realism release)
- RSRQ clamped to the 3GPP reporting range [-19.5, -3] dB.
- KPM latency and propagation delay derived from slant range.
- Per-slice 5QI mapping (eMBB→9, URLLC→82, mMTC→79).
- Space-RIC autonomous mode now fires in the default scenario:
  scheduled feeder-outage windows produce real autonomous decisions
  recorded in `space_ric_metrics.csv`.
- `oran-ntn-full-scenario` installs a real ns-3 traffic plane and
  emits `sim_health.csv` realism gates.

## [1.0.0] — 2026-04

### Added
- **Near-RT RIC** (`OranNtnNearRtRic`) with E2 node management,
  subscription handling, and a per-xApp action pipeline.
- **13 xApps** inheriting `OranNtnXappBase`: ho-predict, beam-hop,
  slice-manager, doppler-comp, tn-ntn-steering, energy-harvest,
  interference-mgmt, multi-conn, predictive-alloc, isac,
  thz-spectrum, thz-beam-mgmt, thz-ris.
- **29 E2SM-RC action types** and **11 A1 policy types**
  (`OranNtnA1PolicyManager` / `OranNtnA1Adapter`).
- **Service models**: E2SM-KPM, E2SM-RC v1.03, E2SM-CCC, and an
  NTN-ephemeris extension.
- **Conflict manager** (`OranNtnConflictManager`) with five
  resolution strategies (PRIORITY, TEMPORAL, MERGE, A1_GUIDED,
  ML_BASED) over resource-key contracts.
- **Space RIC** (`OranNtnSpaceRic`) with autonomous mode under
  feeder outage and ISL message types (8) for KPM/model exchange.
- **Four federated aggregators** (FedAvg, FedProx, FedNova,
  SCAFFOLD) selectable through the FL_PARTICIPATION A1 policy.
- **Two examples**: `oran-ntn-full-scenario` (66-sat constellation,
  5 live xApps) and `oran-ntn-ric-controlled-traffic` (RIC actions
  steering a real UDP flow).
- **One unit-test suite** (`oran-ntn`) with 58 test cases.

### Dependencies
- satellite (SNS3), mmwave, and the ns3-ai bridge for the
  learning-based xApp paths.
