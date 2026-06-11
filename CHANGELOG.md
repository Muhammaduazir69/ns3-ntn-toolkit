# Changelog — ns3-ntn-toolkit

All notable changes to the toolkit and its custom modules. The toolkit and each
standalone module are versioned as release lines; branch `ntn-integration-v2`
(toolkit) and `<module>-v2` (per module) carry the v2 work below.

## [v2.1] — 2026-06

This release lands the **AI-Native ORAN-NTN adoption**: a real measured data
plane and O-RAN control plane across the whole toolkit. Synthetic traffic
generators and closed-form KPI shortcuts are gone — every delay, jitter, loss
and throughput figure in every example is now measured in-band on a real
mmWave NR NTN cell with SGP4 satellite mobility.

### Added — NTN/O-RAN application layer (`ntn-traffic`)

- **`NtnOranApplication` suite** replaces `OnOffApplication` toolkit-wide:
  6 traffic profiles with 5QI-correct defaults (conversational voice, eMBB
  video, URLLC periodic, mMTC periodic, Poisson background, CBR saturating),
  a 24-byte in-band **`NtnOranPayloadHeader`** (version, type, sequence,
  TX timestamp, 5QI, S-NSSAI SST/SD, QFI, src/dst id — real wire bytes, so it
  survives GTP tunneling), **`NtnOranSink`** (per-flow one-way delay,
  RFC 3550 jitter, sequence-gap loss, throughput, measured from received
  bytes) and **`NtnCommandAndControlApp`** (real-mobility telemetry +
  battery model, 5QI 69).
- **`NtnOranAiFlowMonitor`**: flow classifier/probe built on ns-3's
  FlowMonitor infrastructure, keyed by srcId+dstId+5QI+S-NSSAI, exporting
  TS 28.552 / E2SM-KPM-named series (`DRB.UEThpDl`, `DRB.RlcSduDelayDl`,
  `DRB.PacketLossRateDl`, `DRB.PdcpSduVolumeDl`, `L1M.RS-SINR`), AI feature
  windows, EWMA z-score anomaly events, and XML/CSV/InfluxDB/E2 exporters.
- **`NtnRealStackHelper`**: one call builds a real mmWave NR NTN cell
  (SpectrumPhy/MAC/RLC/PDCP/RRC/EPC) under an SGP4 satellite, with
  `InstallOranFlow()`, `EnableOranFlowMonitor()`, regenerative-payload
  options and live feeder geometry.

### Added — multi-tier RIC and payload architecture (`oran-ntn`)

- RT-RIC tier with an enforced <10 ms control-loop bound; RIC placement
  model (on-board / gateway / ground-cloud) with E2 latency from live slant
  geometry; NWDAF slice analytics + transport-path controller + cross-domain
  SMO loop; ONNX Runtime xApp inference as an optional CMake-detected
  dependency (heuristic fallback otherwise).
- Payload options A/B (transparent vs regenerative RU / RU+DU / full gNB),
  fronthaul split model (Opt 2 / 7.2a / 7.2b / Opt 8), platform latency
  classes (UAV/HAPS/LEO/MEO/GEO) with endurance enforcement, and
  measured-trigger role switching.
- New examples: `oran-ntn-ric-placement-ab`, `oran-ntn-cross-domain-slice`,
  `oran-ntn-payload-options-ab`, `ntn-platform-latency-validation`,
  `oran-ntn-emergency-communication`.

### Added — standards validation campaign

- **TR 38.821 Set-1 LEO-600 S-band calibration** (`ntn-tr38821-calibration`):
  measured CNR-vs-elevation tracks the study-case link budget (constant
  array-gain offset, FSPL slope within 0.2 dB of theory).
- **Five 3GPP NTN handover trigger classes** in `ntn-cho`: measurement (A3),
  location (D1), time (T1), elevation, and timing-advance — all executing
  handovers on the real radio (`ntn-cho-handover-traffic --trigger=...`).
- Orbital-theory test suite (SGP4 vs Kepler, Doppler envelope, ENU pass
  geometry, slant/elevation relation incl. the TR 38.821 1932 km @ 10° case).
- Repo-wide gates: `tools/check_protocol_fidelity.py` (36 checks) and
  `tools/check_ntn_standards.py` (12 checks) both PASS.

### Added — use-case flagships

- `oran-ntn-emergency-communication` — disaster scenario: payload role-switch
  to full gNB, SST=5 emergency slice, eMBB throttled live.
- `ntn-sagin-remote-coverage` — multi-MNO shared LEO cell with measured
  cost split.
- `ntn-v2x-edge-urllc` — platoon URLLC on a real LEO cell with on-board vs
  ground edge-AI placement and measured decision latency.

### Removed

- `OnOffApplication`-based traffic and every remaining closed-form
  `SnrToPer`-style KPI path in toolkit modules.

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
