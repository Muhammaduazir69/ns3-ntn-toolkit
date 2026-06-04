# Changelog

All notable changes to **ntn-constellation** are recorded here.

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
