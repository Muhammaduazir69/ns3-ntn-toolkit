# ns3-ntn-toolkit — Toolkit-Wide Standards, Security & Data-Plane Audit

**Date:** 2026-07-14  **Scope:** all 14 custom `contrib/` NTN modules + cross-cutting tree scans
**Method:** 14 parallel per-module deep audits (each verifying current code against 7 dimensions —
data transmission, payload headers, mobility, propagation, standards conformance, vulnerabilities,
broken links — with `file:line` evidence), plus a repo-wide broken-link scan (165 docs), a
secrets/credentials sweep, and a network-binding sweep. Findings were cross-checked against the
prior `STANDARDS_CONFORMANCE_AUDIT_2026-06-24.md`, `SCOPE_AND_LIMITATIONS.md`, and
`PROTOCOL_FIDELITY_AUDIT_AND_FIX_2026-06.md`.

Reference bar for "a proper ns-3 module" = the vendored `mmwave` / `nr` (5G-LENA) / `satellite`
(SNS3) / `ns3-ai` modules: real `MobilityModel`, real `Propagation`/`SpectrumPropagationLossModel`,
serialized `Header`s, custom `FlowMonitor`, and a real data plane (`NetDevice`+IP+App+`Simulator::Run`).

---

## Executive verdict

**The toolkit is, on balance, professionally engineered and unusually honest about its own
boundaries.** There are **no critical security vulnerabilities** of the remote-code-execution or
data-exfiltration class, **no hardcoded secrets**, and **no genuinely broken links in the shipped
module docs**. Most modules stand up a **real ns-3 data plane** with **real SGP4 / TR 38.811
mobility** and **real propagation in the packet path** — not closed-form KPIs. Standards *naming* is
mostly careful and provenance-tagged.

The real issues are four kinds, in priority order:
1. **One results-integrity bug** — `ntn-slice` reports per-slice KPIs under inverted slice labels.
2. **A cluster of unguarded-parse crash/DoS bugs** on live/malformed input (opt-build OOB or abort).
3. **Web-service hardening gaps** in the FastAPI digital-twin.
4. **"Decision computed but not actuated" gaps** (handover, slice PRB, RIC) — all *disclosed* in
   `SCOPE_AND_LIMITATIONS.md`, but the headline "handover"/"closed-loop" claims need the wiring —
   and `ntn-cho` in particular is now fixable because the real A3/X2 handover landed this week.

Plus a handful of standards-value slips and three doc/interop overclaims.

### Module scorecard

| Module | Verdict | Real data plane? | Top issue |
|---|---|---|---|
| **ntn-traffic** | Strong | Yes (mmwave/nr PHY, measured KPIs) | Unguarded `PeekHeader` in flow probe (MED) |
| **ntn-constellation** | Strong | Yes (P2P/ISL + real C/N0→BLER error model) | TLE checksum computed but not enforced (LOW) |
| **ntn-sagin** | Strong | Yes (13/13 examples) | Non-standard 5QI grouping (MED) |
| **ns3-ai-ntn** | Strong | N/A (RL bridge) | Exemplary honesty; `shell=True` nit (LOW) |
| **oran-ntn** | Adequate | Yes (real PER codec + SCTP) | E2 termination has no `Simulator::Run`; E2AP PDUs dummy (HIGH) |
| **thz-ntn** | Adequate | Yes (absorption in packet path) | HITRAN-2024 label overclaim + missing calib test (HIGH) |
| **ntn-observability** | Adequate | Yes (measured KPIs) | "netsimulyzer-1.0" JSON is NOT the official schema (HIGH) |
| **ntn-v2x** | Adequate | Yes (5/5 examples) | `std::stod` crash on malformed CSV (HIGH); no PC5/J2735 |
| **ntn-rrc** | Adequate | Yes (SIB19 crosses the plane) | Regenerative Timing-Advance physically wrong ×2 (HIGH) |
| **ntn-cho** | Partial | Yes, but handover **not actuated** | CHO decides but never moves a UE (HIGH) |
| **ntn-slice** | Partial | Yes (via ntn-traffic), decide-only | Slice↔UE label inversion corrupts KPIs (HIGH) |
| **ntn-sionna** | Partial | Yes, but Sionna **not in packet path** | Ray-traced gains never reach measured SINR (HIGH) |
| **ntn-fapi** | Partial | N/A (struct model) | "ABI" overclaim; structs never exchanged (HIGH) |
| **ntn-digital-twin** | Partial | N/A (Python service, no ns-3) | FastAPI compute-DoS + unauth on 0.0.0.0 (HIGH) |

---

## Prioritized findings

### HIGH — results integrity (fix first; affects published numbers)

- **`ntn-slice` slice labels are inverted vs the actual traffic.**
  `NtnRealStackHelper` MixedBouquet maps `u%3` → **{0:mMTC(5QI 9), 1:eMBB(5QI 2), 2:URLLC(5QI 82)}**
  (`contrib/ntn-traffic/helper/ntn-real-stack-helper.cc:852-855`), but all three ntn-slice examples
  label the UEs **{0:eMBB, 1:URLLC, 2:mMTC}** (`ntn-slice-isolation-traffic.cc:39,121`;
  `ntn-slice-real-stack.cc:151`; `ntn-slice-three-slice.cc:163-173`). So the saturating eMBB stream is
  reported as "URLLC" and the tiny NB-IoT flow as "eMBB" — the "eMBB saturates / URLLC holds SLA"
  narrative is rotated. **Fix:** remap the examples' index→name to `{mMTC, eMBB, URLLC}`, or reorder
  the MixedBouquet cases in the helper. Also: S-NSSAI SST is hardcoded to `1` for every UE
  (`ntn-real-stack-helper.cc:882`), so slices are not differentiated on the wire.

### HIGH — standards correctness

- **`ntn-rrc` regenerative Timing Advance is off by 2×.** `ntn-timing-advance.cc:83-95` returns the
  one-way `d/c` for the regenerative case; TA is inherently **round-trip** (`2·d_service/c`), so the UE
  under-compensates uplink by 2×. Transparent also omits the feeder RTT. Enshrined in the test
  (`test:75-95`), so the test is wrong too. **Fix:** regenerative = `2·d_service/c`; transparent adds
  `2·d_sat-gw/c`.
- **`ntn-v2x` uses the wrong V2X 5QI.** Flows use 5QI **82** (Discrete Automation) labeled "URLLC
  V2X"; the canonical V2X value is **79** (and 80 for low-latency), TS 23.501. No NR sidelink/PC5
  (TS 23.287 / 38.885) is modeled — "V2V" is plain P2P over a real cell, i.e. V2N. Honest in the
  README, but the V2X-standards surface is thin.
- **`ntn-rrc` SIB19 is a raw-`double` byte dump, not TS 38.331 ASN.1 PER** (`ntn-sib19.cc:136-141`);
  the header claims 1.25 m position quantization that the serializer does not apply
  (`ntn-sib19.h:34-36`). It crosses the plane as real bytes, but the wire format is non-conformant.

### HIGH — robustness / crash / DoS (unguarded parse on live or malformed input)

- **`ntn-v2x`** `sumo-traci-bridge.cc:142-148` — `std::stod` on CSV fields with no try/catch; one
  non-numeric field aborts the run. **Fix:** per-row try/catch + `continue`.
- **`ntn-sionna`** `sionna-replay-transport.cc:174` — `m_records.reserve(count)` with an
  attacker/file-controlled `uint64 count`; a malformed `.bin` → `bad_alloc`/`length_error` → terminate.
  **Fix:** bound `count` against remaining file size before reserve.
- **`ntn-traffic`** `ntn-oran-ai-flow-monitor.cc:96-97,109-110` — `NtnOranFlowProbe::ReportRx/Tx`
  calls `PeekHeader` with no size guard; a foreign <24-byte packet reaching the sink port causes a
  debug abort / opt-build OOB heap read (the intended `PeekHeader()==0` guard is dead code). **Fix:**
  `if (packet->GetSize() < NtnOranPayloadHeader::SERIALIZED_SIZE) return;` before `PeekHeader`.
- **`ntn-rrc`** `ntn-sib19.cc:123` — the serialize size guard is `NS_ASSERT` only, compiled out in
  optimized builds → OOB write on a short buffer (header says "Throws" but never does). `real-stack.cc:80`
  asserts on a received-packet cellId → aborts on any corrupt/reordered SIB.

### HIGH — web-facing (ntn-digital-twin FastAPI service)

- **Unbounded compute → CPU DoS.** `PredictHandoverRequest` has no field bounds (`schemas.py:42-48`);
  `n_steps = horizon/step` drives an O(n_steps × n_sats) loop (`server.py:175`), so
  `horizon_min=1e9, step_sec=1e-3` hangs the server. **Fix:** `Field(gt=0, le=…)` + cap `n_steps`.
- **`step_sec=0` → ZeroDivisionError → unhandled 500** (`server.py:175`). Same fix.
- **Unauthenticated on `0.0.0.0:8090`** (`server.py:226`, systemd unit) with no auth/CORS/rate-limit.
  Combined with the above = trivial remote hang. **Fix:** default-bind `127.0.0.1`; add an API key or
  document a reverse proxy. (Note: this module ships **no ns-3 C++ at all** — it is mis-shelved under
  `contrib/`; consider relocating to `tools/` or `services/`.)

### HIGH — "decision computed, not actuated" (disclosed in SCOPE, but headline claims need wiring)

- **`ntn-cho` never actuates a handover.** `ExecuteHandover()` only mutates counters + fires an
  unbound callback (`ntn-cho-algorithm.cc:796`); no example calls `rs.SetHandover()` even though
  `NtnRealStackHelper` now exposes the real A3-RSRP + X2 inter-gNB handover (landed 2026-07-14,
  `ntn-real-stack-helper.h:289-296`). The candidate cell is a fictitious `servingCellId+100` and
  candidate SINR is closed-form Friis. **Also:** the flagship *TTE-aware* selector is inert on the
  live plane (its orbit predictor is never set → `SelectBestCandidate` returns 0), and the TTE binary
  search is degenerate (`ntn-tte-estimator.cc:307-309` never advances the satellite between evals).
  **Fix (high-value, now unblocked):** wire ≥2 gNBs + `rs.SetHandover(true)` and bind the CHO decision
  to the real handover so the UE actually moves; feed the predictor from the live pass.
- **`ntn-slice` orchestrator and `oran-ntn` RIC are open-loop** — PRB/beam decisions feed a CSV/trace,
  not the scheduler (SCOPE A4). Honestly disclosed; only the inlined beam example closes the loop.

### HIGH — interop / documentation overclaim

- **`ntn-observability`** emits a home-grown JSON labeled `"schema":"netsimulyzer-1.0"` with
  camelCase `NodeMove`/`SeriesSample` keys (`exporter.cc:156-216`); the **official** schema uses
  kebab-case `node-position`/`xy-series-append`. **These files will not open in NetSimulyzer.** The
  `ntn-netsimulyzer-official-demo` already does it correctly via the real `Orchestrator`. **Fix:** drop
  the `netsimulyzer-1.0` label (call it a native scene format) or emit the real schema.
- **`thz-ntn`** cites a `thz-ntn-absorption-calibration` test for the ±1 dB P.676-13 cross-check that
  **does not exist** (`molecular-absorption.cc:23-24`); and `kHitranRelease="HITRAN-2024"`
  (`hitran-lut.h:40`, `doc/thz-ntn.rst:35-37`) overclaims vs the honest "continuum-approximation" the
  README/CSV state. **Fix:** add the numeric zenith-opacity assertion (or delete the claim), and retag
  to "continuum-approximation / HITRAN-2020-baseline".
- **`ntn-fapi`** README/INSTALL call it a binary **ABI** "cuPHY/OAI nfapi can link against"
  (`README:8,12`); the structs embed `std::vector`/`std::variant` (`fapi-messages.h:53,116,134`) and
  have **no fixed byte layout** — no serialization exists anywhere. **Fix:** reword to "C++ struct
  model mirroring SCF-222 field names," or add real `Serialize/Deserialize` to make the ABI claim
  literal. (Message-type IDs 0x80–0x89 are correct SCF-222.)

### MEDIUM

- **`ntn-sionna`** — the ray-traced channel never reaches measured SINR: all channel classes are
  narrowband `PropagationLossModel` (0 `SpectrumPropagationLossModel`), the Sionna server returns a
  scalar `path_loss_db` with no taps, and packet-carrying examples chain a **hand-authored synthetic
  Rician CIR** (labeled "NOT ray-traced"). ~8/12 examples run a real plane; 4 are probe/calibration
  only. Honestly disclosed, but "ray-traced channel in ns-3" is aspirational for the data plane.
- **`oran-ntn`** — `oran-ntn-e2-termination` uses a real SCTP association but **no `Simulator::Run`**
  and the E2AP outer PDUs are dummy bytes (only the E2SM-KPM blob is real PER). **Fix:** wrap in an
  ns-3 app on the real plane; ASN.1-encode the E2AP PDU.
- **`ntn-observability`** — InfluxDB line-protocol escaping omits `\n`/`\r` (`influx-sink.cc:37-84`),
  and CZML track labels are written unescaped (`scene-recorder.cc:450`); a newline/quote in a CLI
  `--runId`/label injects a forged LP line or malformed CZML. **Fix:** reject/escape `\n\r` and route
  labels through a JSON escaper.
- **`ntn-sagin`** — slice→layer 5QI grouping (QFI 1–4→URLLC, 5–9→eMBB, 10–63→mMTC,
  `sagin-slice-router.cc:42-54`) is a homemade convention, not TS 23.501 Table 5.7.4-1. The
  multi-layer/slice/contact-graph routers are advisory geometry scorers, not the IP forwarding plane.
- **`thz-ntn`** — `ThzNtnPointingLossModel` draws a fresh Rayleigh sample per `DoCalcRxPower` call
  (`pointing-loss-model.cc:153`); the spectrum engine calls it per signal *and* per interferer, so
  pointing loss is non-reciprocal within a slot (realism bug, not memory safety).

### LOW (representative — full list in the per-module notes)

- `ntn-constellation` — `TleRecord::ChecksumOk()` is never enforced (`orbital-elements.cc:160`);
  corrupt-but-well-sized TLEs parse silently. Kepler+J2 fallback rotation is simplified (Vallado is
  primary and validated).
- `ntn-traffic` — `TB.ErrTotNbrDl` (a TS 28.552 *count*) is assigned a TBLER *ratio*
  (`ntn-oran-ai-flow-monitor.cc:214`).
- `ntn-fapi` — `DlTtiPdu` enumerates `kSsb=3` with no `SsbPdu` variant alternative
  (`fapi-messages.h:37,41`).
- `ns3-ai-ntn` — `ns3ai_utils.py` uses `subprocess.Popen(shell=True)`; `inference-channel-tcp.cc`
  binds `INADDR_ANY` unauthenticated (opt-in, 16 MiB frame cap enforced).
- `ntn-slice` — `DefaultUrllc` docstring says "1 ms" but sets `latencyBudgetMs=5.0`.

---

## Standards conformance by domain

- **3GPP NTN (TR 38.811/38.821, TS 38.331 SIB19, TS 38.213 TA):** Mostly correct and spec-cited.
  Gaps: TA round-trip error (`ntn-rrc`), SIB19 not ASN.1 PER, CHO triggers defined correctly but not
  actuated. TR 38.811 large-scale loss only (no fast fading) — disclosed (SCOPE A1).
- **O-RAN (E2AP v7.00, E2SM-KPM v6.00, E2SM-RC v7.00, A1 v5.00, O1 v15.00):** KPM metric names are
  real TS 28.552 counters with explicit provenance (no invented names); real aligned-PER codec,
  bounds-checked and fuzz-safe. Transport is either in-process-delay or real-SCTP-with-dummy-E2AP; RIC
  control is open-loop (SCOPE A4).
- **ITU-R / THz (P.676-13, P.838-3, IEEE 802.15.3d):** Gaseous absorption is a real
  `PropagationLossModel` in the packet path (excess-only, no FSPL double-count) — genuinely good. Line
  centers correct; HITRAN label overclaims; P.838-3 rain coeff ~10% high at 100 GHz; 802.15.3d
  channelization not implemented (not claimed).
- **V2X (3GPP C-V2X / NR sidelink, SAE J2735 / ETSI CAM-DENM):** Weakest domain. No PC5 sidelink, no
  J2735/CAM-DENM header, wrong V2X 5QI. AIS (ITU-R M.1371) trajectory replay is handled correctly.
- **AI/ML (ns3-ai):** Bridge is a real Boost.Interprocess shared-memory interface; the NTN RL
  environments are synthetic and **labeled as synthetic in every file, docstring and README**, with
  active `RuntimeError` honesty guards. MAPPO/MASAC honestly downgraded to shared-policy baselines.
  This is the integrity model the rest of the toolkit already mostly follows.

## Broken links (verified across 165 docs)

- **Actionable:** `distribution/docker/assets/README.md` → `../../../branding/{logo_icon.svg,README.md}`
  (missing); `contrib/oran-ntn/flexric-bridge/README.md` → `INSTALL.md` (missing).
- **Benign:** `docs/UPSTREAM_NS3_README.md` (4 links — this is upstream ns-3's own README).
- **NOT broken (verified):** the `github.com/Muhammaduazir69` URLs (108 occurrences) — this exactly
  matches the live git remote, so the links resolve. Two audit agents mis-flagged it as a typo; **do
  not "correct" it** or the links will break. It is at most a cosmetic username transposition.
- All 14 module README/INSTALL cross-links otherwise resolve.

## What is genuinely strong (do not touch)

- **Real data plane discipline** — `Simulator::Run()` + real NetDevice/App/FlowMonitor + measured
  SINR/TBLER/throughput/delay, not closed-form, across the great majority of examples.
- **Real orbital mobility** — Vallado SGP4 validated against the SGP4-VER cat-00005 reference vector
  (`ntn-constellation`), reused by the SNS3 `Sgp4MobilityModel`, plus TR 38.811 UE classes.
- **Real propagation plugins that attenuate packets** — THz gaseous absorption, TR 36.777 A2G, and a
  per-packet C/N0→BLER `ErrorModel` — chained onto the real spectrum channel, not offline scalars.
- **A genuine honesty culture** — `SCOPE_AND_LIMITATIONS.md`, per-KPI `provenance` tags, "SYNTHETIC"
  labels, and `RuntimeError` guards that refuse to fake ns-3. Most disclosed limitations are exactly
  where the code says they are.
- **Security hygiene** — no hardcoded secrets; bounds-checked O-RAN PER codec; robust defensive CSV
  parsers in `ntn-sagin`; HTTPS-with-verification for TLE feeds.

## Recommended fix order

1. **`ntn-slice` label inversion** (results integrity — anything you publish from it is mislabeled).
2. **The four unguarded-parse crashers** (`ntn-v2x` stod, `ntn-sionna` reserve, `ntn-traffic`
   PeekHeader, `ntn-rrc` assert-guards) — small, high-value robustness fixes.
3. **`ntn-digital-twin` FastAPI** field bounds + loopback bind (network-facing).
4. **`ntn-rrc` timing-advance** round-trip fix (standards correctness) + its test.
5. **`ntn-cho` handover actuation** — now unblocked by the real A3/X2 handover; turns the module's
   headline claim real.
6. **Doc/interop de-overclaim** — `ntn-observability` NetSimulyzer label, `thz-ntn` HITRAN + calib
   test, `ntn-fapi` ABI wording.
7. **Two real broken links** (branding assets, flexric-bridge INSTALL).

## Honest bottom line

This is a **credible, professionally built research toolkit** that mostly does what it says and is
refreshingly candid about what it does not. It is **not** a toy: the radio, orbits, propagation and
measurement are real ns-3, and the O-RAN/PER, SGP4, and THz-absorption paths are standards-grounded.
The gaps that remain are **specific and fixable**, not architectural rot: one KPI-label inversion, a
short list of input-validation crashers, a few actuation wirings (one now unblocked), and some
doc/label overclaims to bring in line with the code's own honesty elsewhere. None of them is a
security emergency; all of them are worth closing before the next paper/release.
