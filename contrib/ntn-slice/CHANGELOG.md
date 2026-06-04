# Changelog

All notable changes to this module are documented here. The format is
based on [Keep a Changelog](https://keepachangelog.com/) and this
project adheres to Semantic Versioning.

## [Unreleased]

### Changed

- The three-slice LEO/GEO example now produces genuinely differentiated
  slices: URLLC reaches low latency / high reliability, eMBB drives high
  throughput, and mMTC supports many low-rate devices, each within its own
  KPI envelope.
- `served_mbps` is now clamped so it never exceeds the per-slice demand.

### Added

- New `ntn-slice-isolation-traffic` example with a real ns-3 data plane
  (point-to-point links + IP + UDP apps + FlowMonitor) feeding live per-flow
  results through the selector and isolation monitor.

## [1.0.0]

### Added

- Initial release of `ntn-slice` — a 3GPP network-slicing layer for LEO + GEO
  non-terrestrial networks.
- **`ntn-slice-types.h`** — `SliceSst` enum (eMBB / URLLC / mMTC; V2X SST
  value defined), `Snssai` (8-bit SST + 24-bit SD with packed/unpacked
  codecs), `SliceProfile`, and the `DefaultEmbb` / `DefaultUrllc` /
  `DefaultMmtc` profile factories per TS 22.261 Table 7.1-1.
- **`NtnSliceSelector`** — DSCP / port-range / app-label rule chain,
  first-match-wins, default fallback; maps a flow to its S-NSSAI.
- **`SliceOrchestratorXapp`** — PRB allocator with min-throughput reservation
  plus priority-weighted unmet-demand sharing; `Step()` /
  `StepWithShares()` for RL override.
- **`SliceIsolationMonitor`** — rolling per-slice latency / loss-rate windows
  emitting `BreachEvent` records.
- **`NtnSliceHelper`** — `ThreeSliceDefault()` plus the GEO mode-skip helpers
  `ShouldSkipGeo()` and `IsGeoSatellite()`.
- Two example programs and a unit-test suite
  (`test/ntn-slice-test-suite.cc`, suite name `ntn-slice`, 7 cases).

### Notes

- `IsGeoSatellite()` treats altitudes at or above a conservative 30 000 km
  boundary as GEO-class (GEO proper is 35 786 km).
- The V2X SST enum value (SST 4) is reserved; no default V2X profile factory
  ships yet.
