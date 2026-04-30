# Changelog

## [1.0.0] — 2026-04-23

### Added
- Initial public release of `ntn-traffic` (formerly an internal
  `traffic` module; renamed to avoid namespace collision in the
  App Store).
- CBR application (`CbrApplication`) with NTN-appropriate defaults.
- NRTV over TCP (`NrtvTcpClient`, `NrtvTcpServer`) and UDP
  (`NrtvUdpServer`) with `NrtvVariables` ON/OFF/BURST distributions.
- 3GPP HTTP satellite client (`ThreeGppHttpSatelliteClient`) and
  `ThreeGppHttpVariables` tuned for bent-pipe / regenerative RTTs.
- `TrafficTimeTag` for per-packet latency budget tracking.
- 3 example scripts.

### Changed
- Renamed from `traffic` → `ntn-traffic`.
- Ported build system from `wscript` to `CMakeLists.txt`.
