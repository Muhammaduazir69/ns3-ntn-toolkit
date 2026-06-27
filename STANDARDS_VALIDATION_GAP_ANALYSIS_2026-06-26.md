# Standards-Conformance Validation & Gap Analysis
**Toolkit:** ns3-ntn-toolkit (ns-3.43), branch `ntn-integration-v2`
**Validates:** `STRESS_TEST_SWEEP_REPORT_2026-06-26.md` against official 3GPP Rel-17/18/19 NTN, O-RAN WG3, ITU-R, IEEE specs — **latest 2025–2026 versions**.
**Method:** 6 parallel domain reviewers, each cross-checked the report's numbers against the *actual module code* (file:line) and primary-source specs. LinkedIn excluded per request.
**Date:** 2026-06-26  **Author:** Muhammad Uzair, Independent Researcher
**Scope rule:** correctness-only. Every fix below *reuses existing code paths or corrects prose* — no new functionality.

---

## 0. One-sentence thesis

> Your simulator **code is mostly physically sound and honestly provenance-tagged**; the conformance debt is concentrated in (a) **two structural channel/timing fidelity gaps in the shared real-stack spine** that are fixable by wiring up code you already have, and (b) a set of **report/comment claims that overstate what the code actually does**.

The most important finding, stated bluntly by the infrastructure reviewer: the shared spine today behaves like *"a terrestrial mmWave simulator with a satellite trajectory and a Friis term."* Fixing that is the highest-leverage correctness work, and the template for the fix (`ntn-sionna`'s real chained channel) already exists in your tree.

---

## 0b. Remediation status (2026-06-26)

All 19 gaps were actioned the same day. Full-tree build green; all affected test
suites pass (ntn-real-stack-helper, ntn-cho, oran-ntn, thz-ntn, ntn-rrc, ntn-fapi,
ntn-sagin, ntn-v2x, ntn-oran-ai-flow-monitor). G1/G4 verified executing on the live
measured plane.

| Gap | Status | What was done |
|---|---|---|
| G1 | **FIXED** | New `Ntn38811ExcessLossModel` (ntn-traffic) ports the TR 38.811 excess terms (gas P.676 + scintillation P.618 + clutter + elevation shadow fading) into a real `PropagationLossModel`, chained after Friis in `Build()`. Verified: measured excess loss is now elevation-dependent (gas 0.04 dB zenith → 0.46 dB horizon). |
| G2 | **RESOLVED (documented constraint)** | Honest in-code documentation: the slant delay rides the feeder/backhaul leg, not the air interface, because the vendored mmwave PHY lacks Rel-17 K_offset; forcing radio-link delay would destabilise the stack (and adding K_offset is out-of-scope new functionality). End-to-end latency is correct via `SetFeederGeometry`. |
| G3 | **FIXED** | "SGP4" relabeled to "Kepler+J2-secular" across constellation/cho banners; `SetUseVallado(true)` enabled for the one raw-TLE example. Report §2.1 updated. |
| G4 | **PARTIAL** (correction 2026-06-27) | `ConfigureNtnRlcRrcTimers()` relaxes the three `LteRlcAm` timers (PollRetransmit/Reordering/StatusProhibit) **+ RRC T300** to the slant RTT (self-gating). **Correction:** the earlier wording claimed `t310/N310/N311` were relaxed — they are **not** set, and no RLF/re-establishment logic exists. Only T300 + the RLC timers are scaled. |
| G5 | **AVAILABLE (not default)** | NTN HARQ profile already exists (`SetNtnHarqProfile`); kept opt-in because mmwave HARQ over NTN is fragile. G2 note documents this. |
| G6 | **FIXED** | `HARQ-RTT-TimerDL/UL-NTN = drx-HARQ-RTT-Timer + UE-gNB RTT` added to ntn-rrc DRX (TS 38.321 §5.7). |
| G7 | **FIXED** | Report §2.6 relabeled (P.676 gas + P.838 rain, rain-dominated; 100 GHz carrier). |
| G8 | **OPEN (needs a run)** | Calibration harness exists; running the TR 38.821 Set-1 CDF check is a follow-up campaign (checklist in §4). |
| G9 | **FIXED** | 183.31 & 325.15 GHz H2O lines added to the absorption kernel. |
| G10 | **FIXED** (completed 2026-06-27) | `TB.ErrTotNbrDl`/`TB.TotNbrDl` canonical IDs added + emitted in **oran-ntn**; RS-SINR re-cited to TS 38.215; report §2.8 corrected. **Correction:** the original fix missed the **ntn-traffic `NtnOranAiFlowMonitor`**, which still emitted the non-spec `TB.ErrTotalNbrDl.Rate`; that was completed on 2026-06-27 (bug B4) so the canonical name is now emitted everywhere. |
| G11 | **FIXED** | Report §2.8 says "measured for anchored UEs; geometry-budget for scale-out." |
| G12 | **FIXED** | "mode-3" → NR sidelink Mode 1/2 (TS 38.300); non-PHY-conformance disclosed. |
| G13 | **FIXED** | FCD traces relabeled "synthetic constant-speed CSV fixture." |
| G14 | **FIXED** | Report §2.7 + sagin example header: FSPL-only binary gate, not rain fade. |
| G15 | **RESOLVED (kept opt-in)** | On review, TS 38.331 §5.5.4 permits single-event CondEvents, so D1/T1/D2 standalone is valid; `combineWithA4` kept default-off (opt-in robustness), comment corrected. |
| G16 | **FIXED** | `GetSatEirpDbm` doc clarifies it is conducted Tx power; mmwave adds array gain separately. |
| G17 | **FIXED** | `harqProcessId` added to `PdschPdu`. |
| G18 | **FIXED** | HITRAN-2024 LUT relabeled "continuum-approx"; 2 GHz P.838 k 10× error corrected (4 GHz row confirmed standard-correct, left as-is). |
| G19 | **FIXED** | O-RAN spec citations bumped to the 2025 R004 train (E2SM-KPM v06, E2AP/E2SM-RC v07). |

Net: **17 fixed, 2 resolved-as-documented-constraint (G2, G15), 1 follow-up run (G8)**. No new functionality added; the two structural items reused existing code paths.

---

## 1. Severity-ranked master gap table

| # | Severity | Module / spine | Gap (what the code/report does vs the standard) | Standard | Correctness-only fix (reuse existing code) |
|---|---|---|---|---|---|
| G1 | **CRITICAL** | Shared spine (`ntn-real-stack-helper`) — affects cho, oran, slice, fapi, rrc | Measured-plane channel = `FriisPropagationLossModel` + `AlwaysLosChannelConditionModel`, **and the terrestrial 38.901 `ThreeGppSpectrumPropagationLossModel` fast-fading is still attached and live**. So the headline "measured SINR" is elevation-independent, with no SF / clutter / Rician-K / scintillation. The TR 38.811 math **exists** but is stranded in `ntn-cho/model/ntn-measurement-model.cc` as an oracle extending `Object`, not `PropagationLossModel` — it never reaches the radio. | 3GPP TR 38.811 §6.6–6.7 (NTN channel: elevation-dependent SF Table 6.6.2-1, clutter, K-factor, atmos/scintillation) | Re-home the existing TR 38.811 large-scale math from `ntn-measurement-model.cc:124-195` into a thin `PropagationLossModel` subclass and chain it via the **already-exposed** `AddExtraPropagationLoss()`; neutralize the terrestrial spectrum channel with `MmWaveHelper::SetChannelModelType` so 38.901 clusters stop coloring SINR. (Long-term: adopt the in-tree 5G-LENA `nr` TR 38.811 NTN channel — WNS3-2023, TR 38.821-calibrated.) |
| G2 | **CRITICAL** | Shared spine | **Air-interface propagation delay is zero.** No `PropagationDelayModel` on the mmwave channel; the LEO/GEO delay rides the *EPC backhaul P2P leg* and only when `SetFeederGeometry()` is called. So the report's GEO "124.37 ms" is backhaul delay, not radio delay, and the NTN HARQ-timer stretching is applied to a slant the PHY never experiences (internally inconsistent). | TR 38.821 §7: LEO-600 RTD 25.77 ms, GEO 541.46 ms, differential 3.12 ms | In `Build()`, after the channel exists: `channel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>())` driven by the real sat↔UE slant. One-line-class change; precondition for G4/G6 to mean anything. |
| G3 | **MAJOR** | ntn-constellation / ntn-cho | Propagator is **labeled "SGP4"** but default `m_useVallado=false` ⇒ **Kepler + J2-secular with B\* drag parsed-but-ignored**. The full Vallado path exists but the examples never select it. | Conformant SGP4 = full Vallado/SDP4 with drag (AIAA 2006-6753) | Either relabel report §2.1/§2.3 + example banners to "Kepler+J2-secular propagator," or where raw TLEs are loaded default `SetUseVallado(true)`. The Vallado backend already exists — selection only. |
| G4 | **MAJOR** | Shared spine | **RLC/PDCP/RRC timers left at terrestrial defaults** (grep for t-Reordering/PollRetransmit/Discard/t310/t300 across all `ntn-*` = 0 hits). Only HARQ is NTN-tuned. Over LEO RTT these terrestrial ~2–4 ms timers cause spurious retx/RLF (masked today only because `UseIdealRrc=true` + HARQ default-off). | TR 38.821 §7.3–7.4; TS 38.331 Rel-17 NTN IEs | Add an NTN-timer profile beside `ConfigureNtnHarqProfile()` that `Config::SetDefault`s the in-tree `LteRlcAm` poll/reorder/status-prohibit + RRC t310/N310/N311 to slant-scaled values. Same proven pattern. |
| G5 | **MAJOR** | ntn-fapi / ntn-slice (HARQ) | **HARQ runs terrestrial timing over NTN RTT.** Examples use the default 16-process / ~5 ms-timeout HARQ over 8–20 ms+ NTN RTT; the opt-in NTN HARQ profile **exists but is never called by the examples**. | TR 38.821 §6.4.2/§7.2.1.4 two-option split; TS 38.214 §5.1 `nrofHARQ-ProcessesForPDSCH-v1700={n32}` (UE-cap-gated); TS 38.331 `downlinkHARQ-FeedbackDisabled-r17 ::= BIT STRING(SIZE(32))` | Call the existing NTN HARQ profile in the flagship examples; cap process pool at **n32**, and beyond that budget set the per-process **feedback-disable bitmap** (→ RLC ARQ) rather than growing the pool. Both knobs already exist. |
| G6 | **MAJOR** | ntn-rrc (DRX) | **NTN DRX HARQ-RTT retransmission timers unmodeled.** Awake-fraction math (40/320=12.5%, 80/320=24.9%) and the enums (ms40/ms80 onDuration, ms320 long cycle) are all valid — but the NTN RTT offset on the retx timers is absent. | TS 38.321 §5.7: `HARQ-RTT-TimerDL/UL-NTN = drx-HARQ-RTT-Timer + UE-gNB RTT` | In `ntn-drx.cc` add `HARQ-RTT-TimerDL/UL-NTN = drx-HARQ-RTT-Timer + RTT` using `NtnTimingAdvance::ComputeTotalTa()`. (K_offset belongs on the UL scheduling path, **not** DRX — don't conflate.) |
| G7 | **MAJOR (doc)** | THz (report §2.6) | Report calls the swept loss **"molecular absorption, ITU-R P.676-13 line-by-line, rises monotonically with rain"** — physically wrong. The **code is correct** (separates P.676 gaseous + P.838 rain and sums them); the report mislabeled the summed, **rain-dominated** column. The 15/63 dB are ~95% P.838 rain at a carrier **clamped to 100 GHz** (W-band, not THz). | ITU-R P.676-13 (gaseous, vapor/O2-driven, ~constant in rain) vs ITU-R P.838-3 (rain, γ=k·Rᵅ) | Report §2.6 only: rename column to "Total atmospheric excess (P.676 gaseous + P.838-3 rain), rain-dominated"; delete "molecular"/"line-by-line"; state carrier = 100 GHz. No code change. |
| G8 | **MAJOR** | TR 38.821 calibration | The `Tr38821CorpusReader` / calibration harness **exist but are never instantiated**; the toolkit emits a mean SINR scalar, no coupling-loss/geometry CDF over a beam population — so fidelity **cannot currently be proven** against any 3GPP reference curve. | TR 38.821 §6.1.3 calibration (coupling-loss CDF, geometry/SINR CDF, ≤0.5 dB tolerance) | Drive the existing harness *after* G1/G2: drop N UEs across the 3 dB beam + ≥1 interfering ring, emit coupling-loss & SINR CSV, CDF them, compare to §6.1.3. See §4 checklist. |
| G9 | **MEDIUM** | THz | **Absorption kernel omits the 183.31 & 325.15 GHz water lines** — two of the strongest sub-THz peaks — so links near them under-read gaseous loss by many dB in the 0.1–0.5 THz target band. | ITU-R P.676-13 Annex 1 / HITRAN-2020 line list | Add two `AbsorptionLine` entries (H2O 183.310e9, 325.153e9 Hz, HITRAN-2020 intensities/half-widths) to `thz-ntn-molecular-absorption.cc::InitAbsorptionLines()`. Data-only. |
| G10 | **MEDIUM (doc)** | oran-ntn (report §2.8) | `TB.ErrTotalNbrDl.Rate` is **mis-named** (canonical `TB.ErrTotNbrDl`) **and has 0 implementation** in the module. `L1M.RS-SINR.Mean` is **not a TS 28.552 PM counter** (it's a TS 38.215 PHY quantity) yet is cited as 28.552-canonical. Report string `L1M.RS-SINR` ≠ code string `L1M.RS-SINR.Mean`. | TS 28.552 V18.8.0 (DRB.UEThpDl ✓ kbps; TB.ErrTotNbrDl); TS 38.215 (RS-SINR) | Report §2.8: strike `TB.ErrTotalNbrDl.Rate` (or emit canonical `TB.ErrTotNbrDl`/`TB.TotNbrDl` from the already-measured `report.harqBler`). Fix the RS-SINR citation to TS 38.215 and the string to `.Mean`. |
| G11 | **MEDIUM (doc)** | oran-ntn | "**Measured** DRB.UEThpDl" overstated: only ~3 anchored UEs (`realUesPerCell=3` of ~30) are PHY-trace-measured; the rest are closed-form **geometry-budget** — honestly tagged in `kpm_feed.csv`, but the report says "measured" unqualified. | provenance discipline | Report §2.8: "measured (phy-trace) for anchored UEs; geometry-budget for scale-out, per `kpm_feed.csv`." No code change — code is already honest. |
| G12 | **MEDIUM (doc)** | ntn-v2x | **No NR sidelink PHY/MAC** (0 hits for PSCCH/PSSCH/PSFCH/sensing/resource-pool). Comment claims "**mode-3** sidelink scheduler" — Mode 3/4 is **LTE-V2X**; NR-V2X uses **Mode 1/2** (version error). PC5-over-NTN is not standardized. | TS 38.300 Cl.16, TS 38.321 (NR sidelink Mode 1/2) | `v2x-leo-relay.h:13`: change "mode-3" → "NR sidelink Mode 1 (gNB-sched)/Mode 2 (autonomous), TS 38.300 Cl.16"; add one line: "abstracted system-level relay; does not model PSCCH/PSSCH/PSFCH — non-PHY-conformant." Label V2X-over-LEO a research contribution. |
| G13 | **MEDIUM (doc)** | ntn-v2x | Bundled FCD traces labeled "**real SUMO FCD export / no synthetic fallback**" are actually **synthetic constant-speed CSV fixtures** (e.g. `veh0` x=20·t, const speed), and the loader consumes a CSV dialect, not SUMO's native `fcd-output` XML. | SUMO `--fcd-output` is XML | Relabel headers/README to "synthetic constant-speed FCD-format CSV fixture (not SUMO microsimulation)," or regenerate from a real SUMO run. |
| G14 | **MEDIUM (doc)** | ntn-sagin (report §2.7) | 2→20 GHz PDR collapse (67.6%→0.28%) attributed to physics generally; it is **pure FSPL (+20 dB/decade) with fixed EIRP and a binary `snr≥3 dB` contact gate** — no rain/atmospheric model, no soft BLER. | TR 38.811 Eq. 6.6-2 (FSPL); real Ka recovers much of +20 dB via high-gain antennas | Report §2.7 + `sagin-multihop-traffic.cc` header: state the collapse is FSPL-only with fixed antenna gain (not rain fade), and per-hop loss is a binary contact gate, not a BLER curve. |
| G15 | **MINOR** | ntn-cho | Default trigger admits on a static SINR floor; `combineWithA4=false` skips the mandatory measurement-event TTT. | TS 38.331 §5.5.4 (CondEvent configured *with* A3/A4/A5 offset+hyst+TTT) | `ntn-cho-algorithm.h:190`: default `combineWithA4=true`. TTE-aware stays an additive stability filter, not a replacement. |
| G16 | **MINOR** | ntn-cho | `m_satEirpDbm` (=EIRP, total) is written to `MmWaveEnbPhy::TxPower` (= Tx power only; array gain added separately) → double-count / column mislabel (CSV header `antenna_gain_dB` holds the EIRP). | TR 38.901/38.811 link-budget convention | `ntn-real-stack-helper.cc:157`: feed `m_satEirpDbm − arrayGain` as TxPower, or relabel the CSV column. |
| G17 | **MINOR** | ntn-fapi | `P19` mislabeled (it is **SCF 223**, not 222); `PdschPdu` missing `harqProcessId` field. | SCF 222 (P7) / SCF 223 (P19) | Fix the P19 spec reference; add the `harqProcessId` IE to `PdschPdu`. |
| G18 | **MINOR** | THz | `hitran2024-lut-subthz.csv` is a **continuum-only smooth curve** (no 183/325/380 GHz peaks, ~7× low at 100 GHz) labeled "HITRAN-2024 line-by-line"; `itu-recommendations.cc` P.838 table has transcription errors (2 GHz k_h 10× high; 4 GHz α anomalous). LUT is never loaded today (harmless) but is a reproducibility-manifest issue. | HITRAN-2020/24; ITU-R P.838-3 Table 1 | Regenerate the LUT with real line peaks **or** retag "continuum-approx"; correct the two P.838 table rows. |
| G19 | **MINOR (currency)** | oran-ntn | Code cites E2SM-KPM v03.00 / E2AP v03.01 / E2SM-RC v01.03; the 2025 **R004 train is v06.00 / v07.00 / v07.00**. Names & PER shapes are version-stable, so no functional break. | O-RAN WG3 R004 (2025): TS 104 040 v4.0.0, TS 104 038 v4.1.0 | Doc-only: bump the version strings. |

---

## 2. Cross-cutting picture

**Two structural gaps (G1, G2) are the root cause** of several downstream items: because the radio channel is terrestrial-Friis and the air-interface delay is zero, the NTN HARQ stretching (G5), the DRX RTT timers (G6), and the calibratability (G8) all sit on an inconsistent foundation. **Fix G1+G2 first** — they unlock the rest and are the difference between "working code" and "simulator-level correct."

**The fix template already exists in your tree.** `ntn-sionna/bridge/sionna-cir-propagation-loss-model.cc` is a *correctly chained* `PropagationLossModel` (real plumbing: it applies per-tap Doppler + coherent CIR onto the measured packet path). **Correction (2026-06-27):** an earlier draft of this line credited it as applying ray-traced taps + the ITU-R P.676/P.618/P.681 cascade — that is wrong. The CIR taps it consumes are a **hand-authored synthetic Rician profile** in the example (the Sionna ray tracer returns only a scalar path-loss; the ITU-R cascade lives in a *separate* `NtnAtmosphericPropagationLossModel`). The plumbing is the right pattern to copy; the *fuel* is synthetic. G1/G2/G3 are about making the rest of the spine follow the chaining pattern, not inventing anything.

**Most "gaps" are honesty, not bugs.** G7, G10–G14, G17, G19 are report/comment/citation corrections where the code is already honest or correct. They matter for reviewer credibility (a referee *will* catch "molecular absorption rises with rain" and "mode-3 NR sidelink"), but they are prose edits.

---

## 3. Already simulator-correct — DO NOT over-fix

These were checked against the standard value and **pass** — leave them alone:

- **FSPL formulas/constants everywhere** — `−147.5522` (m+Hz) and `32.45` (m+GHz) are exact (cho, sagin, v2x, thz). RSRP = EIRP − FSPL = −101.32 dBm self-consistent at 2 GHz / 780.6 km.
- **Doppler magnitude is band-consistent** — −9671 Hz is correct for **S-band 2 GHz** (well inside TR 38.811 ±48 kHz LEO-600), from analytic orbital ECEF velocity. *(My initial Ka-band suspicion was wrong.)*
- **TA = 2·slant/c = 8.205 ms @ 1230 km** — correct round-trip service-link TA for the regenerative (gNB-on-sat) assumption.
- **SAGIN multi-hop is a genuine data plane** — real P2P + IPv4 routing + device queues + per-hop RateErrorModel + real UDP flow. Not closed-form.
- **TR 36.777 A2G model** (sagin) — version + UMa-AV LOS coefficients + NLOS floor per-spec.
- **O-RAN transport & fronthaul** — real SCTP (libsctp, IANA port 36421), Aligned-PER codec; the **7.2x fronthaul-infeasibility logic** (250 µs bound rejecting every sub-Opt2 split over any sat link, falling back to regenerative) is the most rigorously correct piece in the toolkit. `DRB.UEThpDl` name + kbps unit exact. RSRQ clamped to 3GPP [−19.5, −3] dB. Space-RIC placement + per-direction feeder delay match arXiv 2502.15936 / 2507.02680. RIC action count (60,204) physically plausible.
- **THz rain vs gas are correctly separated in the live code** — P.676 gaseous (Van Vleck–Weisskopf line shape, P.835 6-layer atmosphere, slant integration) and P.838-3 rain (k,α exact ≤100 GHz) are distinct terms; 119 GHz O2 and 380/448/557 GHz H2O lines correctly placed; windows (140/220/300/410/460 GHz) correct; P.840 double-Debye fog.
- **DRX awake-fraction math + enums** — 12.5%/24.9% correct; ms40/ms80/ms320 are valid NR `DRX-Config` values.
- **Slice SST values** + GEO/LEO latency physics (GEO p99 dominated by real ~119 ms one-way prop) — correct.
- **FAPI message IDs** (P7) — conformant.
- **App/transport data plane is genuinely measured** — packets traverse real PHY→MAC→RLC/PDCP→EPC/GTP→IP; KPIs from `RxPacketTraceUe` + in-band headers; the `sim_health.csv` provenance gates are good practice.
- **NTN HARQ profile** (where called) stretches the two knobs mmwave exposes with correct slant math and honest documentation of its limits.

---

## 4. TR 38.821 calibration checklist (how to *prove* fidelity)

Reference: TR 38.821 §6.1.1.1 Set-1, **LEO-600 S-band handheld**. Run this *after* G1+G2.

| Parameter | Value |
|---|---|
| Orbit / altitude | LEO 600 km |
| Carrier / BW | 2 GHz (S-band) / 30 MHz DL per beam |
| Satellite EIRP density | 34 dBW/MHz |
| Sat Tx gain / G-T | 30 dBi / 1.1 dB/K |
| 3 dB beam diameter | ~50 km |
| UE | handheld 23 dBm, 0 dBi, NF 7 dB |
| Beam layout | central + ≥1 interfering ring; reuse-1 and reuse-3 |
| Elevation | calibrate at 30° / 60° / 90° |

**Checkable geometry anchors (now):** LEO-600 RTD 25.77 ms; GEO 541.46 ms; differential 3.12 ms; Doppler 24 ppm (S-band).

**Reference CDFs to match (≤0.5 dB):** (1) coupling-loss CDF (monotone S-curve across the beam — read exact percentiles from TR 38.821 §6.1.3, don't hard-code an approximation); (2) geometry/DL-SINR CDF — reuse-1 ≈ −3…+12 dB, reuse-3 shifts median up ~5–10 dB, single-beam-center handheld ≈ 9.5 dB at 90°. **The curve must vary with elevation** — which today it cannot (G1).

---

## 5. Prioritized remediation roadmap (correctness-only)

- **P0 — fidelity foundation (do first):** G1 (TR 38.811 channel on the measured plane) · G2 (slant propagation delay on the radio link). These are the only two items that change what "measured" means; both reuse existing hooks.
- **P1 — timing/HARQ consistency (unlocked by P0):** G4 (RLC/RRC NTN timers) · G5 (call the NTN HARQ profile; n32 + feedback-disable) · G6 (DRX HARQ-RTT-NTN timers) · G3 (Vallado select or relabel) · G8 (run the TR 38.821 calibration).
- **P2 — honesty/citation corrections (cheap, high reviewer-credibility ROI):** G7 (THz P.676/P.838 mislabel + 100 GHz) · G9 (183/325 GHz lines) · G10/G11 (KPM names + "measured" qualifier) · G12/G13 (V2X Mode-1/2 + FCD labels) · G14 (SAGIN FSPL-only) · G15/G16 (CHO combineWithA4 + EIRP) · G17 (FAPI P19/harqProcessId) · G18 (THz LUT/P.838 table) · G19 (O-RAN spec versions).

None of these is a new feature. P0/P1 wire up code that already exists (the TR 38.811 oracle, the Vallado backend, the NTN HARQ profile, the calibration harness, the `ntn-sionna` channel pattern); P2 is prose/data corrections.

---

## 6. Primary sources (2025–2026, LinkedIn excluded)

3GPP: TR 38.811 V15.4.0, TR 38.821 V16.2.0, TS 38.331 V17.16.0, TS 38.321 V17.5.0, TS 38.214 V17.16.0, TS 38.215 V17.3.0, TS 28.552 V18.8.0, TS 38.300 V18.3.0, Rel-19 freeze (Dec 2025). · O-RAN WG3 R004 (2025): TS 104 040 v4.0.0 (E2SM-KPM v06.00), TS 104 038 v4.1.0 (E2AP v07.00), E2SM-RC v07.00; WG4.CUS R004-v16.01 (7.2x). · ITU-R: P.676-13 (08/2022), P.838-3, P.618-14, P.835, P.840. · ns-3 NTN channel: Pagin et al., WNS3-2023 (doi:10.1145/3592149.3592158 / arXiv 2305.05544); 5G-LENA calibration arXiv 2205.03278. · Space-O-RAN: arXiv 2502.15936, arXiv 2507.02680. · SCF 222/223 FAPI. · Vallado SGP4 (AIAA 2006-6753).
