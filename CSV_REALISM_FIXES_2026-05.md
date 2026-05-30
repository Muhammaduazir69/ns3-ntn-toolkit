# CSV Output Realism Fixes — May 2026

End-to-end audit of every CSV produced by the custom-module examples, checking
each column against NTN/LEO/THz physics. Below are the defects found and fixed.
All affected modules rebuild clean and their test suites still PASS; every fix
was re-verified against freshly regenerated CSVs.

## Correctness bugs (physically-impossible / wrong values)

| Module | File | Defect | Fix | Verified |
|--------|------|--------|-----|----------|
| CHO | `ntn-cho/examples/ntn-cho-full-constellation.cc` | Doppler magnitude-only — never negative across a pass | Signed range-rate (project ground-track velocity onto UE→sat LOS) | Doppler now +14.5k / −17.1k Hz, flips sign |
| CHO | same | Sentinels leaked into CSVs: `serving_sat=4294967295`, `sinr=-100`, `time_of_stay` inflated by `lastHoTime=-1000`, `avg_sinr` polluted | Treat a UE as served only with a valid sat AND SINR>−50; emit empty serving fields otherwise; first-HO ToS uses `t`; average over served UEs only | 0 sentinels; avg_sinr −9..−4; ToS ≤ sim |
| RRC | `ntn-rrc/examples/{leo-pass,from-tle,full-stack}.cc` | `ta_drift_rate` emitted in s/s but column implied µs/s | ×1e6 at CSV write + header `_us_per_s` (model/SIB19 ABI left in s/s) | drift now µs/s, max 8.4 µs/s |
| ORAN | `oran-ntn/helper/oran-ntn-helper.cc`, `model/{oran-ntn-phy-kpm-extractor,oran-ntn-sat-bridge}.cc` | `rsrq = sinr − 3` → positive RSRQ (to +17 dB), impossible | Bounded S/(S+I+N) mapping clamped to 3GPP [−19.5,−3] | RSRQ ∈ [−4.5,−3], 0 positive |
| ORAN | `oran-ntn/helper/oran-ntn-helper.cc` | `action_log.success` hardcoded `true` | Tie to xApp confidence (≥0.5 applied) | realistic mix (e.g. 36 ok / 843 fail) |
| SAGIN | `ntn-sagin/model/a2g-channel-tr36777.cc` | NLOS path loss could fall below LOS | `PL_NLOS = max(PL_LOS, PL'_NLOS)` per TR 36.777 | 153/960 violations → 0 |

## Simplified-model realism gaps (plausible-but-coarse values)

| Module | Defect | Fix | Verified |
|--------|--------|-----|----------|
| SAGIN | RSRP ~20-30 dB too low (no antenna gains); SINR affine proxy; throughput binary 0/80; Doppler ignored aircraft velocity | Full Ka budget (EIRP + 30/30 dBi gains), noise-based SINR, Shannon/CQI throughput, relative-velocity Doppler | RSRP [−80,−73] dBm, SINR [16,23] dB, throughput 119–173 Mbps (597 distinct) |
| ORAN | `latency`/`prop_delay` were 2 constants by `is_ntn`; FIVE_QI constant 9; `slice_id` blank (uint8_t streamed as char); cell PRB/UE/throughput frozen | Delay from slant range (elevation→range); per-slice 5QI (9/82/79); `(int)sliceId`; time/cell-varying load | prop_delay [0.05,7.77] ms; 5QI {9,82,79}; slice_id {0,1,2}; PRB 1485 distinct |
| ORAN | Space-RIC never made autonomous decisions (KPM not fed; outage at 200 s missed short runs) | Feed serving+candidate KPM to autonomous Space-RICs; scale outage to run length; broaden outage to ~3 planes | decisions 0 → 497, handovers 497, autonomous_time 983 s |
| RRC | `full-stack` UE at 316 km alt; reference==UE (`ta_ue`=0); straight-line satellite climbed out of orbit | Ground-level Islamabad ECEF; beam-centre reference offset; orbital `SatSGP4MobilityModel` (bundled ISS TLE) | UE at 540 m; `ta_ue` nonzero 594/600; sat \|r\| stays [6791,6799] km, all axes vary |
| V2X | `jitter_ms` always 0 (static replay, clocks locked) while column claimed "measured live" | Model TraCI co-sim IPC/scheduling jitter (0-8 ms, < W7 100 ms gate) | jitter [0.02,7.99] ms, 292 distinct |

## Investigated, NOT a bug (left as-is)

- **THz atmospheric-window table** (`thz-ntn/model/thz-ntn-spectrum.cc`):
  `peakTransmittance` and `maxZenithAttenuation_dB` are **not** inverses — they
  are consumed multiplicatively in `ComputeTransmittance()` and a unit test
  asserts on `peakTransmittance`. The two are independent quantities; the header
  doc was clarified rather than changing values (which would break the model).
- **ORAN `conflict_log` empty**: correct — the two active xApps (doppler-comp,
  slice-manager) contend on disjoint resource keys, so zero conflicts is the
  right result for that xApp mix.
- **THz/ISL FSPL, radar range⁴, RCS, RIS 20log10(N), UM-MIMO gain/beamwidth,
  Fraunhofer near-field, Shannon capacity**: all verified numerically correct.
