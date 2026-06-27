# Standards-Conformance & Synthetic-Data Audit — ns3-ntn-toolkit

**Date:** 2026-06-24
**Method:** 6 parallel agents — 3 web-research (3GPP NTN, O-RAN/Space-O-RAN, THz, all current to 2025-2026) building a 100+ assertion conformance rubric; 3 code/doc agents reading the *current* tree (doc claims-vs-gaps, synthetic-data hunt, per-module utilization).
**Constraint from user:** NO new functionality. Make existing code execute dynamically, eliminate hardcoded/synthetic data, conform to protocol standards. Examples must exercise each module's real functionality in depth.

> Note: the in-repo roadmap/audit docs are **stale** relative to recent code. Several gaps they list (full Vallado SGP4, real E2AP/SCTP, ntn-cho-full-constellation placeholder, SnrToPer stragglers) were closed in prior sessions. This document supersedes them for *current* truth; the code-audit findings below were read live on 2026-06-24.

---

## 1. Current synthetic-data verdict (live code, 2026-06-24)

| Module | Verdict | Summary |
|---|---|---|
| ntn-rrc | CLEAN | Real SGP4 mobility, measured PHY SINR/TBLER, real ephemeris in real packets |
| ntn-observability | CLEAN | KPI exporters NaN-guarded, pulled from real stack |
| ntn-traffic | CLEAN | Measured-radio backbone (FlowMonitor deltas); RNGs are 3GPP traffic distributions (load, not KPIs) |
| ntn-constellation | CLEAN | Real Vallado SGP4 + live CelesTrak/Space-Track TLEs + measured link KPIs |
| ntn-sionna | CLEAN | Bridge runs Sionna RT `paths.cir()`; FSPL only as counted fallback |
| ntn-sagin | CLEAN | Real OpenSky/AIS trace ingestion + TR 38.901 HST; KPIs from sinks/FlowMonitor |
| ntn-digital-twin | CLEAN | Live TLE→SGP4→geometry; "synthetic" tokens are honest offline test presets |
| ntn-fapi | MINOR | ABI clean; 3 examples derive CRC/goodput from coin-flip vs measured BLER, not real HARQ decode |
| ntn-slice | MINOR | 2 examples/models real; `ntn-three-slice-leo-geo.cc` fabricates per-slice KPI CSV (disclosed) |
| thz-ntn | MINOR | Real VVW absorption, but 23-line HITRAN subset vs full 2024; HW-impairment SNR computed-then-commented-out |
| ntn-cho | SYNTHETIC-HEAVY | Real-stack examples genuine; `ntn-cho-full-constellation.cc` is a standalone formula simulator emitting all headline CSVs; `NtnMeasurementModel` SINR closed-form |
| ntn-v2x | SYNTHETIC-HEAVY | Radio measured, but synthetic FCD generator + billiard maritime motion, live SUMO/TraCI stub, random 0-8 ms jitter injected into KPI |
| ns3-ai-ntn | SYNTHETIC-HEAVY | All 4 RL envs fabricate RSRP/SINR/BLER/throughput from formulas + np.random, never boot ns-3 |
| oran-ntn | SYNTHETIC-HEAVY | Real PHY path exists but flagship `full-scenario`, `InjectKpmReport`, `OranNtnSatBridge`, all `generate_visualizations*` figures are formula/np.random; PHY extractor auto-attach is a no-op |

**Score: 7 CLEAN, 3 MINOR, 4 SYNTHETIC-HEAVY.**

### Root cause (cross-cutting)
A correct measured path exists (`NtnRealStackHelper::GetMeanDlSinrDb` ← `RxPacketTraceUe`; FlowMonitor goodput), but a **parallel synthetic path** coexists, and the *headline/flagship* artifacts in cho/oran/v2x/ns3-ai use the synthetic one. Fix everywhere = redirect headline examples onto the real-stack PHY trace; make figure tools plot exported CSVs.

### Top synthetic findings (file:line)
1. `ns3-ai-ntn/.../envs/__init__.py:3` — "mimics... without spinning up ns-3" — HIGH
2. `ns3-ai-ntn/.../envs/handover_env.py:99` — `sinr = rsrp - noise + rng.normal(0,1.5)` — HIGH
3. `ns3-ai-ntn/.../envs/power_ctrl_env.py:50` — `bler = 1/(1+exp((snr-1)*0.7))` — HIGH
4. `oran-ntn/examples/oran-ntn-full-scenario.cc:450` — `BudgetSinrDb()=BudgetRxPowerDbm()-NoiseDbm()` — HIGH
5. `oran-ntn/helper/oran-ntn-helper.cc:411` — `throughput_Mbps = 10*(1+sinr/30)` — HIGH
6. `oran-ntn/helper/oran-ntn-helper.cc:435` — `prbUtilization = 0.55 + 0.30*sin(Now())` — HIGH
7. `oran-ntn/model/oran-ntn-sat-bridge.cc:517` — `throughput = log2(1+10^(SINR/10))*BW` — HIGH
8. `oran-ntn/model/oran-ntn-phy-kpm-extractor.cc:108` — `AttachToEnbPhy(){/*no-op*/}` — HIGH
9. `oran-ntn/tools/generate_visualizations.py:461` — `tte_550 = np.random.gamma(4,12,3000)` — HIGH
10. `ntn-cho/examples/ntn-cho-full-constellation.cc:182,352,56` — formula RSRP/SINR, fake CSVs, circular-orbit not SGP4 — HIGH
11. `ntn-cho/model/ntn-measurement-model.cc:180` — `sinr_dB = rsrp - noisePower` "no interference" — HIGH
12. `ntn-v2x/model/sumo-traci-bridge.cc:215,221` — random jitter into KPI; live TraCI stub `return 0` — HIGH
13. `ntn-v2x/helper/ntn-v2x-helper.cc:26` — `WriteSyntheticFcdCsv` fabricated tracks — HIGH
14. `ntn-slice/examples/ntn-three-slice-leo-geo.cc:159,161` — synthetic samples + coin-flip delivery → CSV — HIGH
15. `ntn-fapi/examples/ntn-fapi-leo-pass-slotloop.cc:89` — `crcOk = rng > bler` → synthetic goodput — MED-HIGH
16. `thz-ntn/model/thz-ntn-channel-model.cc:431` — HW-impairment SNR commented out of Rx — MED
17. `thz-ntn/model/thz-ntn-molecular-absorption.cc:194` — 23-line catalog vs full HITRAN-2024 — MED

---

## 2. Utilization gaps (examples not exercising module functionality)

### oran-ntn — SHALLOW
- **11 of 16 xApps never instantiated** by any example: energy-harvest, interference-mgmt, isac, multi-conn, predictive-alloc, thz-beam-mgmt, thz-ris, thz-spectrum, mmimo-precoder (logic reimplemented inline in ric-controlled-traffic). Only 5 reachable, and only via `OranNtnHelper::CreateAllXapps` (hardcodes ho-predict, beam-hop, slice-manager, doppler-comp, tn-ntn-steering).
- **All 5 gym envs unused** (GymBeamHop/Handover/Predictive/Slice/Steering — zero references).
- **3 of 4 service models unused**: RC (+rc-style3), CCC, NtnEphemeris. Only KPM used (e2-termination).
- **F1Interface, OfhInterface, FederatedLearning (gated off), DualConnectivity, SplitGnbEntity/Helper, MmWaveBeamforming, PhyKpmExtractor, both DataRepository backends, SpaceRicInference** — all unexercised.
- `oran-ntn-e2-termination.cc` runs real TCP/UDP E2 sockets but **no Simulator::Run / no ns-3 packet plane**.

### ntn-v2x — PARTIAL
- `MaritimeMobilityModel` never instantiated (entire maritime scenario dead from examples).
- `V2xLeoRelay` instantiated in only 1 of 4 examples, and even there `EvaluateAll()` output is **printed, not used to route** packets; two examples reimplement relay selection inline.
- `V2xLeoDirect::Compute()` instance method + setters never used; only static `ComputeStatic` called once to print a "superseded" baseline.
- `SumoTraciBridge::ConnectTcp`/`RunReplay`/`GetMaxJitterSec`/`SampleTrace` unused.

### Remaining modules (verdict | top dead capability | configure-but-no-real-plane example)
| module | verdict | top under-utilized (present, never driven) | no-real-plane example |
|---|---|---|---|
| ntn-cho | PARTIAL | `NtnOrbitPredictor`, `NtnTteEstimator`, `NtnMeasurementModel`, `NtnAiInterface` pipeline; `NtnChoAlgorithm::SelectBaselineA3`/`SelectBaselineLocationOnly` | `ntn-realistic-mobility-demo.cc` |
| ntn-rrc | PARTIAL | `Sib19Codec::Serialise/Parse` (124-byte wire never crosses link); `NtnDrxStateMachine` never reacts to real traffic | none |
| ntn-constellation | PARTIAL | `ContactGraphRouter` never instantiated; `Tr38821CorpusReader`+`CalibrationHarness`; `NtnSatLinkErrorModel::CurrentBler` | none |
| ntn-fapi | SHALLOW | `DmrsFapiToBitArray` (only real code, 0 examples); 7 of 10 FAPI msgs untouched; structs built inline, never serialized | none (structs inline) |
| ntn-observability | PARTIAL | `NtnSceneRecorder`+`NtnSceneHelper` (0 own examples); `NtnReproManifest` (entire §T9) | `ntn-netsimulyzer-official-demo.cc` |
| ntn-sagin | PARTIAL | `MultiLayerRouter` RL scorer API; `SaginSliceRouter::RouteForSlice`; CSV importers; `HstMobilityModel::GetDopplerHz` | `ntn-sagin-orphan-mobility-showcase.cc` |
| ntn-sionna | PARTIAL | `NtnSionnaChannel`/`CascadeChannel`+`SionnaCalibrator` probe-only; `SionnaReplayTransport`/`PybindTransport` never instantiated | 5: `leo-pass-sionna-vs-tr38811`, `ris-assisted-leo-link`, `mmimo-vs-codebook-leo`, `city-block-4ue-cache`, `sionna-calibration-harness` |
| ntn-slice | PARTIAL | `NtnSliceSelector` never instantiated (UEs assigned by `u%3`); `SliceOrchestratorXapp::StepWithShares` RL hook | none |
| ntn-traffic | SHALLOW | `CbrApplication`+`CbrHelper` (0 examples); `NrtvTcpClient` QoE traces unconnected; `NtnRealisticTrafficHelper` | `nrtv-variables-plot.cc` |
| thz-ntn | PARTIAL | `ThzNtnIslLink`, `ThzNtnIsacProcessor`, RIS O-RAN stack (`RisController`/`RisServiceModel`/`RisXapp`), `ThzNtnWaveform` never instantiated; own channel/PHY/MAC bypassed (slim `ThzNtnPropagationLossModel` used) | 8: dband-constellation, demo, full-stack, isac, isl, leo-ground, ris-assisted, um-mimo |
| ns3-ai-ntn | N/A | Vendored upstream ns3-ai framework (shared RL/gym transport); no NTN model classes — out of scope as a "module" | upstream demos only |

**Pervasive pattern:** examples reach the data plane through the external mmwave `NtnRealStackHelper` and inject the module's own physics/logic only as a scalar or slim adapter, leaving the module's integrated channel/router/service-model classes never instantiated or configured-but-dead. ~14 examples run no real packet plane (sionna ×5, thz ×8, plus cho/observability/sagin/traffic ×1 each).

---

## 3. Conformance rubric (abbreviated — full per-domain checklists held separately)

### 3GPP NTN (TR 38.811 / 38.821 / TS 38.331/213/321) — verified vs primary spec PDFs
Key testable items the radio/RRC layers must satisfy:
- PL = FSPL + SF + CL with **elevation+band-dependent** SF/CL from TR 38.811 Tables 6.6.2-1/2/3 (not a closed-form constant); CL=0 in LOS.
- Elevation-dependent Rician K (Table 6.7.2-1a/1b); atmospheric gas for elev <10°; iono+tropo scintillation.
- TR 38.821 reference geometry: LEO-600 RTD 25.77 ms / Doppler 24 ppm / diff-delay 3.12 ms; GEO 541.46 ms.
- SIB19: ephemeris (state-vector OR orbital elements), ta-Common + drift, Koffset, Kmac, epochTime, ntn-UlSyncValidityDuration, t-Service, referenceLocation+distanceThresh (50 m), neighbour list.
- CondEvents D1 (fixed ref), D2 (moving ref, Rel-18), T1 (UTC time); A3/A4/A5 retained.
- HARQ ≤16 DL processes; per-process feedback disabling via RRC bitmap → RLC ARQ; DRX timers extended by RTT.
- SCS 15/30 kHz FR1-NTN, 60/120 kHz FR2-NTN; bands n255/n256 (Rel-17), n510-512 Ka (Rel-18).
- Rel-19 (frozen Dec 2025): regenerative payload = full gNB onboard + Xn/ISL; store-and-forward (IoT); RedCap-NTN; MBS-over-NTN.

### O-RAN / Space-O-RAN (current spec versions: E2AP v7.00, E2SM-KPM v6.00, E2SM-RC v7.00, A1 v5.00, O1 v15.00, O2 v8.01)
- E2AP PDUs ASN.1-APER over SCTP; E2 Setup w/ RAN Function list; RIC Subscription/Indication/Control; Service Update/Reset/Error.
- E2SM-KPM: 3GPP TS 28.552 measurement names (DRB.UEThpDl, RRU.PrbUsedDl…); granularity period; Indication Formats 1/2/3.
- E2SM-RC: numbered Control Styles + Action IDs; Style 3/Action 1 HO, Action 2 CHO; Style 2 slice PRB quota; Style 1 RBC.
- **7.2x fronthaul ≤500 µs ⇒ cannot cross a satellite link**; F1 1.5-10 ms can cross a LEO hop; ground Near-RT RIC OK for LEO (RTT 5-20 ms) but NOT GEO (>250 ms).
- Payload mode transparent vs regenerative explicit; multi-tier RIC (ground + onboard Space-RIC + dApps); ISL ~1.8-3.6 ms/hop.

### THz (IEEE 802.15.3d-2017 + ITU-R P.676-13, equations verified)
- 802.15.3d carriers 252.72-321.84 GHz, BW = integer × 2.16 GHz; WRC-19 windows 275-450 GHz, exclude RR 5.340 passive bands.
- Gaseous absorption from **ITU-R P.676-13 line-by-line** (Eqs 1-8, Van Vleck-Weisskopf, O₂+H₂O tables + N₂ continuum), frequency-resolved — NOT a constant dB/km.
- Slant path: altitude-layered integration (Eqs 13-14) + P.835 atmospheres; **scale with elevation**; molecular ≈0 for ISL.
- UM-MIMO array gain 10log₁₀(N); beam squint on wideband; LEO Doppler from SGP4; RIS (d₁·d₂)² + N²; ISAC CRB from Fisher info.
- Honesty: above 100 GHz there is **no ratified 6G/NTN channel standard** — reward physics fidelity, not nonexistent-standard conformance.

---

## 4. Prioritized fix plan (no new features — remove synthetic, wire to real)

Ordered by leverage:
1. **ns3-ai-ntn envs** → step real C++ models over the shipped ns3-ai shared-memory bridge (pattern: `examples/lte-cqi/use-msg/run_online_lstm.py:105`). All downstream training inherits the fix.
2. **oran-ntn** → remove synthetic KPM path (`InjectKpmReport`, `OranNtnSatBridge`, full-scenario budget formulas); self-wire `OranNtnPhyKpmExtractor::AttachToEnbPhy`; make `generate_visualizations*` plot exported CSVs; instantiate the dead xApps/service-models/gyms in examples so the surface is exercised.
3. **ntn-cho** → port `ntn-cho-full-constellation.cc` onto `Sgp4MobilityModel` + `NtnRealStackHelper` (like sibling real-stack examples); measured SINR not closed-form.
4. **ntn-v2x** → replace `WriteSyntheticFcdCsv`/maritime billiard motion with real SUMO/AIS traces; measure jitter from real IPC; route via `V2xLeoRelay`; implement live bridge or clearly gate it.
5. **ntn-slice / ntn-fapi** → drive delivery/CRC from real PacketSink/HARQ decode, not coin-flips.
6. **thz-ntn** → wire HW-impairment model into Rx power; ingest full HITRAN-2024.
7. **Standards conformance pass** → audit each module's radio/RRC against §3 rubric (elevation/band SF+CL tables, SIB19 IEs, E2SM measurement names, P.676 line-by-line), correcting non-conformant constants in place.

---

## 5. Execution status (workflow `wf_5bee6946-846`, 2026-06-24)

**Plan+Verify:** all 13 work items planned and adversarially verified. **11 approved**; 2 not approved:
- `ns3-ai-ntn` — planner marked infeasible without new C++ (out of scope as a "module": it's the vendored upstream ns3-ai framework). Needs a narrower plan (remove np.random KPI fabrication + source observations from the real ns3-ai message channel only).
- `thz-ntn` — verifier rejected the plan as a **false premise**: the claimed "dead-LUT bug" does not exist (the HITRAN LUT *is* consulted per sub-layer in `ComputeSlantPathAbsorption` lines 632-647 + `GetTransmittance` 722-724). Real residual is only: a stale header comment (molecular-absorption.h:269-271), the genuinely-dead HW-impairment EVM penalty (`ComputeEffectiveSnr_dB` exists, wire it behind the default-off `m_enableHardware`), and full HITRAN-2024 ingest. ISL/ISAC/RIS example re-homing flagged `newFeatureRisk` → defer.

**Apply:** **`oran-ntn` GREEN — applied + independently verified** (build exit 0, `test.py -s oran-ntn` 1/1 PASS). 12 files, +537/-21. The other 10 approved modules **stalled on a monthly account spend limit** (billing, not technical) and were NOT applied.

### oran-ntn KPI deltas (paper-delta deliverable, part 1)
| Example | Metric | Before | After |
|---|---|---|---|
| oran-ntn-full-scenario | Active xApps on RIC | 5 | 13 |
| oran-ntn-full-scenario | RIC actions (measured KPM feed) | 5595 | 7866 |
| oran-ntn-full-scenario | kpm throughput_Mbps mean (anchored UEs measured) | 12.7138 | 12.1856 |
| oran-ntn-full-scenario | phy rows where DRB.UEThpDl ≠ old SINR formula | 0 | 864/2312 |
| ntn-e2e-full-stack | xApp actions on measured KPM | 69 | 326 |
| oran-ntn-real-stack-scenario | dead xApps now acting (isac/thz-beam/thz-ris/thz-spectrum) | 0 | 186/657/378/365 |
| oran-ntn-ric-controlled-traffic | E2 service models on live data (NTN-Eph/RC-Style3-Act2-CHO/CCC) | 0 | 61B/15B/27B encoded |

What changed: synthetic KPM/throughput/PRB formulas removed/guarded; `InjectKpmReport` now takes measured params (DRB.UEThpDl from real byte-delta goodput, RRU.PrbUsedDl measured); 8 previously-dead xApps registered + acting on the measured feed; RC/CCC/NTN-Ephemeris service models encoded on live SGP4+handover data; both `generate_visualizations*.py` now plot exported `kpm_dataset.csv` (np.random gated behind explicit `--demo`). Radio-health KPIs (sim_health SINR/TBLER) unchanged — the radio was already measured; the synthetic leak was the *KPM report fed to the RIC*, now fixed. Uncommitted on `ntn-integration-v2`.

Deferred in oran-ntn (newFeatureRisk): true `AttachToEnbPhy` auto-attach (trace-signature mismatch); dedicated DRB.PdcpSduVolumeDL field; radio-less scale-out UEs stay on provenance-tagged geometry-budget fallback.

### FINAL STATUS (resume `wf_5bee6946-846` completed 2026-06-24)

**10 of 13 modules GREEN — applied + independently verified.** Full-tree `./ns3 build` exit 0; all 10 module test suites PASS (oran-ntn, ntn-cho, ntn-v2x, ntn-fapi, ntn-slice, ntn-constellation, ntn-sionna, ntn-rrc, ntn-observability, ntn-traffic [cbr-test/nrtv/flow-monitor/application/real-stack-helper]). All changes uncommitted on `ntn-integration-v2`.

**UPDATE — all 3 held-back modules resolved in a corrected follow-up pass (`wf_ccaf8ff3-c41`), committed:**
- `thz-ntn` (commit `aaa779bda`) — narrow feature-neutral fix: corrected the stale molecular-absorption header comment, removed the dead HW-impairment block (real transform `ComputeEffectiveSnr_dB` already wired at link-budget.cc:231/348 where noise is known; dB invariant preserved), registered `thz-ntn-demo` as a build target. Deferred as new-feature (honestly): full HITRAN-2024 .par ingest (23-line LUT is <0.5% delta), re-homing 4 analytic examples (would duplicate existing `thz-ntn-real-stack`/`*-traffic` siblings), ISL/RIS/ISAC real-plane glue. thz-ntn 1/1 PASS.
- `ntn-sagin` (commit `474d07d4b`) — full utilization fix: `ntn-sagin-orphan-mobility-showcase` now runs a real P2P+Internet+NtnOranApplication→Sink plane (PDR 99.97%, 19.8 Mbps measured) and exercises every previously-dead class — MultiLayerRouter scorer (2 candidates), SaginSliceRouter URLLC route (2 hops, QFI→S-NSSAI), all 3 CSV importers (37 HAPS/3 ADS-B/4 AIS rows), HstMobilityModel::GetDopplerHz (−926.6 Hz). ntn-sagin 1/1 PASS.
- `ns3-ai-ntn` (nested-repo commit `b496e3a`) — honest relabel (full real-env wiring = new C++, out of scope): the 4 RL envs are now loudly documented as SYNTHETIC placeholders ("NO ns-3, never report as measured"), np.random KPI fabrication removed (only seeded RNG remains), `ns3gym_compat` raises `RuntimeError` instead of silently discarding ns-3 connection fields. pytest 15/15.

**Original hold-back rationale (for the record) — verifier correctly rejected the first plans:**
- `ns3-ai-ntn` — planner infeasible without new C++ (it's the vendored upstream ns3-ai framework, not an NTN model module). Needs a narrower pass: remove np.random KPI fabrication + source observations from the real ns3-ai message channel only. No fix applied.
- `thz-ntn` — plan rejected as **false premise**: the claimed HITRAN-LUT bug does not exist (LUT already consulted per sub-layer in `ComputeSlantPathAbsorption` 632-647 + `GetTransmittance` 722-728; the `withLut` test at test-suite.cc:879 already passes *because* it is wired). Legit residual only: stale header comment; HW-impairment EVM penalty (use `ComputeEffectiveSnr_dB` at the SNR stage, not a power-domain proxy); full HITRAN-2024 ingest; re-home demo/leo-ground/um-mimo/dband/full-stack onto the existing `thz-ntn-real-stack` pattern (approved edits #5/#7/#8/#9/#10). ISL/ISAC/RIS real-plane glue = newFeatureRisk → defer.
- `ntn-sagin` — plan would not compile: missing mandatory `examples/CMakeLists.txt` edit (needs `${libpoint-to-point} ${libinternet} ${libapplications} ${libntn-traffic}` on the orphan-showcase target); wrong lib name (NtnOranApplication/Sink live in `libntn-traffic`, not `ntn-oran`); HST edit used `Ptr<MobilityModel>` where `sagin::HstMobilityModel::GetDopplerHz` requires a DynamicCast; `sat` out-of-scope in `LinkProbe`. ntn-sagin is CLEAN on synthetic data — this is utilization-only. Needs a corrected plan.

### KPI deltas (paper-delta deliverable) — full table
Captured before→after for every headline number that moved across the 10 green modules; see the workflow result for `wf_5bee6946-846` (parsed in conversation). Highlights: oran-ntn serving thpt 15.66→4.9 Mbps; ntn-fapi goodput 22.8→4.62 Mbps; ntn-cho serving SINR −10.9→+12.9–14.4 dB & HO 5→0; ntn-slice URLLC p99 52.9→6.9 ms (LEO) / 124 ms (real GEO); many `n/a→measured` for newly-real planes. Radio-health KPIs of already-measured examples unchanged (regression checks held).
