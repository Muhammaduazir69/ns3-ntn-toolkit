# Changelog

All notable changes to **ntn-rrc** are recorded here.

## v2

Part of the [ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit).
3GPP Rel-17 NR-NTN RRC procedures for ns-3.43.

### Models, helpers & key classes

- `NtnTimingAdvance` — ephemeris-driven TA pre-compensation (common +
  UE-specific), drift rate, slant range.
- `Sib19Content`, `EphemerisInfo`, `Sib19Codec`, `NtnSib19Broadcaster` — SIB19
  ephemeris broadcast (fixed-layout 124-byte codec, default 160 ms cadence).
- `NtnDrxStateMachine`, `NtnDrxConfig`, `DrxState` — NR connected-mode DRX with an
  NTN `AwaitingPass` deep-sleep state.
- `NtnUeLocationReporter`, `UeLocationReport`, `LocationReportMode` — GNSS-assisted
  location reporting with closed-form ECEF<->WGS-84 conversion.
- `NtnRrcHelper` — install façade for all four components.

### Examples

- `ntn-rrc-leo-pass`
- `ntn-rrc-full-stack` (now flies a `SatSGP4MobilityModel` from the bundled ISS TLE)
- `ntn-rrc-from-tle` (ships a default ISS TLE; runs with zero arguments)
- `ntn-rrc-drx-data-traffic`

### Notes

- `ta_drift_rate` is emitted in µs/s in CSV output (`*_us_per_s` columns); the
  underlying model and the SIB19 codec ABI keep their native s/s convention.

### Tests

- 16 C++ unit test cases (`test/ntn-rrc-test-suite.cc`).
