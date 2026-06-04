# Changelog

All notable changes to this module are documented here. The format is
based on [Keep a Changelog](https://keepachangelog.com/) and this
project adheres to Semantic Versioning.

## [Unreleased]

### Changed

- Flight-LEO KPM now uses a full Ka-band link budget (EIRP + antenna gains ->
  RSRP, noise-based SINR, Shannon/CQI throughput, relative-velocity Doppler),
  replacing the earlier affine SINR proxy and the binary 0/80 Mbps throughput.
- The TR 36.777 A2G channel applies the NLOS floor `max(PL_LOS, PL'_NLOS)`, so
  an obstructed link never reports less loss than the LOS reference.

## [1.0.0]

### Added

- Initial release of `ntn-sagin` — a three-layer (space / air / ground)
  Space-Air-Ground Integrated Network for ns-3.43.
- **Space:** LEO passes via analytic ground-track velocity or, through
  `ntn-constellation`, SGP4 + contact-graph ISL routing.
- **Air:** HAPS station-keeping (`HapsMobilityModel`,
  `HapsTrajectoryMobilityModel`), three UAV mobility patterns
  (`UavWaypointMobilityModel`, `UavPatrolMobilityModel`,
  `UavSearchPatternMobilityModel`) and aeronautical cruise
  (`AeronauticalMobilityModel`).
- **Ground / mobile terminals:** maritime AIS (`AisMobilityModel`), OpenSky
  ADS-B (`OpenSkyMobilityModel`) and high-speed-train (`HstMobilityModel`)
  mobility, each with a replayable trace feed.
- **Channel:** `A2gChannelTr36777` — TR 36.777 v15.0.0 A2G path loss (LOS +
  NLOS floor) and LOS probability for UMa-AV / RMa-AV / UMi-AV.
- **Routing:** `MultiLayerRouter` (greedy max-elevation Ground->UAV->HAPS->LEO)
  and `SaginSliceRouter` (QFI -> S-NSSAI slice-aware layer selection).
- **`SaginHelper`** — factory that builds a 4-layer scene in one call.
- Ten example programs under `examples/` and a unit-test suite
  (`test/ntn-sagin-test-suite.cc`, suite name `ntn-sagin`).
