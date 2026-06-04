# Changelog

## [2.0.0] - 2026

### Added
- Opt-in `NtnSionnaChannel` propagation loss model that queries Sionna RT over
  a pluggable transport, with FSPL fall-back on timeout.
- Transports: `SionnaUdpTransport` (live GPU server), `SionnaPybindTransport`
  (in-process), `SionnaCachingTransport` (4-D LRU decorator),
  `SionnaReplayTransport` (record/replay, no GPU), `SionnaNoneTransport`.
- `NtnSionnaCascadeChannel` composing Sionna RT with the ITU-R
  `NtnAtmosphericLossChain` (gaseous + rain + LMS shadowing).
- `SionnaBatchClient` for async batched queries and `CirDopplerSynthesizer` for
  Doppler-shifted CIR synthesis.
- `SionnaCalibrator` and a calibration harness checking the ray-traced channel
  against the closed-form TR 38.811 reference within a configurable dB gate.
- Real data-plane `*-traffic` examples (P2P + IP + apps + FlowMonitor) and
  physics-only channel/calibration examples.
- Geospatial tooling under `tools/` (OSM → scene, LiDAR/DEM ingest, env probe).
- Test suite with 38 C++ test cases and 3 Python integration tests; the Python
  tests run against a stub server, so no Sionna RT or GPU is required.
