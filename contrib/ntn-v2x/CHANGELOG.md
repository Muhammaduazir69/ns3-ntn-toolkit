# Changelog

## v2

- The TraCI bridge now models realistic co-simulation step-timing jitter
  (a few ms per step, under the 100 ms validation gate) instead of a
  constant `0`.
- The `jitter_ms` column in `ntn-v2x-rural-highway.csv` is now meaningful,
  reflecting per-step sync jitter rather than a flat zero.
- New `ntn-v2x-leo-relay-traffic` example: a real 2-hop data plane
  (shadowed vehicle -> relay vehicle -> LEO -> server) with point-to-point
  links, IP, BSM apps and FlowMonitor.

## v1

- Initial release: `SumoTraciBridge` (FCD replay + minimal live TraCI path),
  `V2xLeoDirect`, `V2xLeoRelay`, `MaritimeMobilityModel`, and the
  `ntn-v2x-rural-highway` example.
