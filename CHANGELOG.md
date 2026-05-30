# Changelog — ns3-ntn-toolkit

All notable changes to the toolkit and its custom modules. The toolkit and each
standalone module are versioned as release lines; branch `ntn-integration-v2`
(toolkit) and `<module>-v2` (per module) carry the v2 work below.

## [v2] — 2026-05

This release closes an end-to-end **CSV output-realism audit**: every column of
every example's output was checked against NTN/LEO/THz physics, gaps were fixed,
and each fix was re-verified against freshly regenerated CSVs. All affected
module test suites still PASS. It also lands a set of **cross-module examples**
so the contrib modules compose each other's APIs over a real ns-3 data plane.

### Added — cross-module examples (real data plane)

- **ntn-constellation** — `ntn-constellation-walker-traffic` (Walker-Delta +
  `ContactGraphScheduler` + `NtnRealisticTrafficHelper` UDP data plane) and
  `ntn-constellation-sgp4-mobility-traffic` (single SGP4 LEO; ground station
  auto-placed under the t=0 sub-point so a real GSL up/down pass always occurs).
- **ntn-fapi** — `ntn-fapi-leo-pass-slotloop`: the SCF-222 FAPI L1↔L2 data ABI
  driven by a real SGP4 pass from `ntn-constellation`; per-slot SINR follows the
  live elevation, with HARQ retransmission and Shannon-tracked goodput.
- **ntn-rrc** — `ntn-rrc-from-tle` now ships a bundled ISS TLE
  (`contrib/ntn-rrc/data/iss-zarya.tle`) and runs with zero arguments.

### Fixed — physical correctness (were impossible/wrong CSV values)

- **CHO** Doppler is now signed (flips +→− across a pass) instead of
  magnitude-only; sentinels (`serving_sat=4294967295`, `sinr=-100`, inflated
  `time_of_stay`) no longer leak into `ue_tracks`/`kpi`/`handover` CSVs, and
  `avg_sinr` averages only served UEs.
- **RRC** `ta_drift_rate` is now emitted in µs/s (column renamed
  `*_us_per_s`); the model/SIB19 ABI keep their s/s convention.
- **ORAN** RSRQ is clamped to the 3GPP range [−19.5, −3] dB (was reaching
  +17 dB); `action_log` success now reflects xApp confidence instead of a
  hardcoded `true`.
- **SAGIN** NLOS path loss now applies the TR 36.777 `max(LOS, NLOS)` floor.

### Changed — model realism (coarse → physical)

- **SAGIN** flight-LEO KPM now uses a full Ka-band link budget (EIRP + antenna
  gains → RSRP ≈ −73…−80 dBm), noise-based SINR, Shannon/CQI-driven throughput,
  and relative-velocity Doppler (replaces the affine/binary proxies).
- **ORAN** latency/propagation now derive from the slant range (elevation →
  range); per-slice 5QI (eMBB→9, URLLC→82, mMTC→79); `slice_id` is serialized
  correctly; cell PRB/active-UE/throughput vary with time and cell; the
  Space-RIC is now fed live KPM and the feeder outage scales to the run length,
  so on-board autonomous decisions/handovers are actually exercised.
- **RRC** `ntn-rrc-full-stack` now flies an orbital `SatSGP4MobilityModel`
  (bundled ISS TLE) instead of a straight-line model that climbed out of orbit;
  the UE sits at ground level (Islamabad) and the beam-centre reference is
  offset so the UE-specific TA residual is non-zero.
- **V2X** the TraCI bridge now models realistic co-simulation step-timing jitter
  (a few ms, under the W7 100 ms gate) instead of a constant 0.

### Notes (investigated, intentionally unchanged)

- THz atmospheric-window `peakTransmittance` and `maxZenithAttenuation_dB` are
  independent inputs consumed multiplicatively (not inverses); header doc
  clarified.
- ORAN `conflict_log` is legitimately empty for the shipped xApp mix (the active
  xApps contend on disjoint resource keys).

See `CSV_REALISM_FIXES_2026-05.md` for the full column-by-column audit and
`EXAMPLE_AUDIT_2026-05.md` for the example-execution audit.
