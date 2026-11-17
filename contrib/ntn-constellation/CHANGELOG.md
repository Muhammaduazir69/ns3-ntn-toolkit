# Changelog

All notable changes to **ntn-constellation** are recorded here.

## Wave 7B (Nov 2026)

- `NtnRangeDelayChannel`: point-to-point channel with a per-packet light-time
  delay on the exact SGP4 state (no 100 ms position cache), receiver motion and
  Sagnac rotation during the flight. Used by route-lifetime, isl-routed-traffic
  and real-routed (`--delayModel=fixed` keeps the old refreshed delay).
- `ContactGraphScheduler`: `IslMarginSkip` skips the ISL look-ahead of a pair
  until its state could first change (range and limb margins over vis-viva
  speed bounds); transitions are identical, route-lifetime CPU 19.6 s -> 6.8 s
  for 600 s at the 100 ms step.
- `NtnSatLinkErrorModel`: contact gate on the receive path with per-cause
  counters; `ConfigureTr38821Set1KaGsl` (TR 38.821 Set-1 Ka budget, DVB-S2
  MODCOD sized for the mask elevation from EN 302 307-1 Table 13, SNS3 DVB-S2
  link-result curve); the binned P.676/P.618 excess-loss table is shared
  across links.
- route-lifetime runs through ntn-cho (WGS-84 ground terminals), ntn-traffic
  (NtnOranApplication/NtnOranSink, NtnOranAiFlowMonitor: flowmon.xml, kpm.csv)
  and ntn-observability (`--czml`, `--netSimOut`); GSLs carry the Ka error model
  (`--linkModel=gate` for the old gate, `--rainRate` for a P.618 fade).
- Tests: suite `ntn-constellation-exact-link` (4 cases).

## v0.1.0 (v2)

First public release as part of the [ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit).

### C++ ns-3 module (`ns3::ntncon`)

- `WalkerConstellation` — Walker-Delta / Walker-Star constellation generation
  (planes, satellites per plane, altitude, inclination).
- `Sgp4MobilityModel` — ns-3 `MobilityModel` using an analytic Kepler propagator
  with secular J2 corrections and an SGP4-compatible TLE interface. The TLE drag
  (B*) field is parsed but not yet used. Full Vallado SGP4 is a planned follow-on.
- `ContactGraphScheduler` — GSL/ISL visibility evaluation with link up/down events.
- `ContactGraphRouter` — routing over the time-varying contact graph.
- `Tr38821CorpusReader` and `CalibrationHarness` — TR 38.821 reference scenarios /
  link budgets plus a Starlink-EU latency/station corpus.
- 20 C++ unit test cases (`test/ntn-constellation-test-suite.cc`).

### Python package (`ntn_constellation`)

- Live TLE feeds: `CelesTrakFeed`, `TleCache` (TTL on-disk cache).
- Constellation API: `Constellation`, `Satellite` over the `sgp4` library and
  Skyfield (canonical SGP4/SDP4).
- Walker generators: `walker_delta`, `walker_star`.
- ISL topology builder: `build_isl_topology`.
- Exporters: `write_sns3_scenario` (SNS3 layout), `write_czml` (CesiumJS).
- `ntn-fetch` CLI entry point.
- 10 Python unit tests (`tests/test_basic.py`).

### Examples

- `ntn-constellation-walker-traffic`
- `ntn-constellation-sgp4-mobility-traffic`
- `ntn-constellation-isl-routed-traffic`
