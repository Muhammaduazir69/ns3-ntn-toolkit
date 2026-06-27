# Whole-Toolkit Standards-Accuracy Judgment — ns3-ntn-toolkit
**Date:** 2026-06-27  **Toolkit:** ns-3.43, branch `ntn-integration-v2`
**Author:** Muhammad Uzair, Independent Researcher
**Method:** fresh external-reviewer audit, organized by LAYER across all 14 NTN modules + vendored deps. Six independent auditors read the actual code (file:line) + the project docs + the latest official 2025-2026 specs (3GPP / O-RAN / ITU-R / IEEE), each told to judge accuracy directly and to *extend*, not repeat, the prior `STANDARDS_VALIDATION_GAP_ANALYSIS_2026-06-26.md`. LinkedIn excluded.

---

## 0. The one-paragraph verdict (read this first)

> **This is a genuinely real ns-3 NR *data plane* with real link-level physics, wrapped in a control/standards/AI layer that is mostly honest abstraction but partly overclaimed labels.** The user/data plane (MAC/HARQ/RLC/PDCP/GTP-U, AMC/MCS/LDPC-BLER, measured KPIs with provenance) is the solid, defensible core — keep it. But the **frequency/PHY layer is not 3GPP-NTN-conformant** (FR2 numerology on an S-band label, no real NR-NTN band, no satellite antenna), the **NTN control surface is oracle/label-only** (SIB19 is a trace not a broadcast; K_offset is dead metadata; no RACH; ideal RRC; EPC not 5GC), the **O-RAN control loop is open** (xApps log decisions to CSV, actuate nothing; "FlexRIC interop" is a loopback stub), and the **AI/FL layer is names without weights** (DQN = linear dot-product; FedNova ≡ FedAvg; SCAFFOLD absent; no trained model ships). The code's *in-line comments are unusually candid* about all this; the overreach lives in **module names, READMEs, and the prior remediation doc**. Nothing here is fraud — it is a transparent simulation whose labels write cheques the implementation does not cash. For a researcher about to publish: **fix the ~6 genuine correctness bugs, relabel the ~12 overclaims honestly, and scope the ~5 architectural items as future work** — then it is a credible, referee-proof framework.

---

## 1. Module × layer accuracy matrix (rollup)

Legend: ✅ accurate/real · 🟂 honest approximation · ⚠️ mislabeled/overclaimed · ⛔ stub/absent/bug

| Module | Radio/PHY | Mobility/geom | L2/L3 + iface | Control/O-RAN/AI | Channel/signal | Verdict |
|---|---|---|---|---|---|---|
| ntn-traffic (spine) | ⚠️ FR2 numerology, no sat antenna | — | ✅ real MAC/HARQ/RLC/PDCP/GTP-U | — | 🟂 Friis+G1 excess (large-scale only) | **Real data plane, wrong frequency layer** |
| ntn-cho | 🟂 | ⚠️ "SGP4"=Kepler+J2; TA no feeder | ⚠️ ideal RRC, RACH=accounting, no condReconfig | — | 🟂 | Real serving cell, analytic HO signaling |
| ntn-constellation | — | ✅ TLE/Vallado/Walker math; ⛔ ISL ignores Earth, dual Earth model, toy shells | 🟂 | — | — | Real primitives, stub system geometry |
| ntn-rrc | — | — | ✅ DRX FSM + HARQ-RTT-NTN formula; ⚠️ not wired to mmwave MAC | — | — | Correct in isolation, unwired |
| ntn-fapi | ✅ AMC/BLER real | — | 🟂 FAPI scaffold (no scheduler drive) | — | — | Honest scaffold |
| ntn-slice | — | — | ⚠️ open-loop orchestrator, no 5QI scheduler | — | — | Measurement + shadow controller |
| ntn-sagin | 🟂 FSPL binary gate | 🟂 traces real; ⚠️ HAPS/UAV flat-Cartesian | — | — | 🟂 | Real multihop, no atmospheric/BLER |
| ntn-v2x | — | ✅ AIS/ADS-B replay | ⚠️ no NR sidelink PHY | — | — | Abstracted relay (honest now) |
| ntn-sionna | ⚠️ | — | — | — | ⛔ "Sionna RT CIR" taps synthetic | RT real for path-loss only |
| thz-ntn | ✅ array/beamforming/pointing physics | — | — | — | 🟂 gas/rain; ⚠️ antenna/pointing not in measured path, no squint, no 802.15.3d | Strong physics, mis-placed |
| oran-ntn | — | — | 🟂 EPC-not-5GC, F1 out-of-plane | ⚠️/⛔ open loop, A1 hollow, O1/O2 absent, FlexRIC stub | — | Real codecs (test-only), open control |
| ns3-ai-ntn | — | — | — | ⛔ real bridge but unused; RL envs synthetic; FL variants fake | — | Plumbing real, AI heuristic |
| ntn-digital-twin | — | ⛔ ECI-as-ECEF elevation bug | — | ⛔ decoupled from sim | — | Live orbit dashboard, not a twin |
| ntn-observability | — | — | — | ⚠️ 1 non-spec KPM name | ✅ Influx LP, CZML, NetSimulyzer positions | Encoders accurate, 1 label wrong |

---

## 2. The genuinely accurate core — DO NOT touch or over-fix

These passed a skeptical external read and are the toolkit's credibility:
- **The NR user/data plane is real ns-3:** `MmWaveFlexTtiMacScheduler` (BSR/SR/CQI→MCS link adaptation), stop-and-wait HARQ, `LteRlcAm/Um`, PDCP, GTP-U; packets are real bytes end-to-end; **SINR/TBLER from `RxPacketTraceUe`, OWD/jitter/loss from in-band headers** — measured, not synthesized. The `sim_health.csv` provenance discipline is good practice.
- **AMC / MCS / LDPC-BLER chain** (CP-OFDM, CQI→MCS, 3GPP MiErrorModel) — real link-level physics, the strongest part of the radio.
- **Orbital primitives:** TLE parser (column-exact, B* mantissa, checksum), real Vallado SGP4 *when fed a TLE*, Walker generator math, Doppler physics (correct sign/ECEF), WGS84 geodetics on the UE/SAGIN side, TR 38.811 UE classes with **real AIS + ADS-B replay**. SNS3 is genuinely integrated.
- **Encoders/exporters:** InfluxDB line-protocol (escaping, sorted tags, types, ns timestamps), Python CZML (FIXED ECEF + real UTC), NetSimulyzer positions (real `MobilityModel` on real `Simulator::Now()`), S-NSSAI SST/SD packing (TS 23.501 Annex A exact).
- **THz array physics:** real UPA/UCA array factor, Cassegrain aperture, codebook beamforming with phase-shifter quantization, pointing-error model (vibration+J2+P.834). The gaseous-absorption core (VVW line shape, P.835 atmosphere, slant integration) is sound.
- **O-RAN engineering-in-isolation:** kernel SCTP transport; a self-consistent (test-passing) hand-PER codec for E2SM-KPM Fmt-1 and E2SM-RC Style-3; correct **FedAvg**; genuine conflict **detection** + resource locks; real onnxruntime/LibTorch/ns3-ai API usage.
- **The G1 fix landed and works** — measured SINR is now elevation-dependent (verified live).

---

## 3. Correctness BUGS — wrong, should be fixed (not just relabeled)

**STATUS 2026-06-27: all six FIXED and verified** — full-tree build green; suites
ntn-constellation / ntn-real-stack-helper / ntn-oran-ai-flow-monitor / oran-ntn /
ntn-cho all pass; constellation Python tests 10/10. (B1 limb-occultation test added;
B2 WGS-84 ellipsoid on the satellite/GS side; B3 twin now emits true ECEF via a new
`Satellite.ecef_m()`; B4 canonical `TB.ErrTotNbrDl` emitted; B5 FL variants warn +
honest scope, SCAFFOLD explicitly unimplemented; B6 HARQ pool clamped at n32.)

| # | Bug | Where | Fix |
|---|---|---|---|
| B1 | **ISL contacts ignore Earth occultation** — range-only gating links satellites *through* the planet | `contrib/ntn-constellation/model/contact-graph-scheduler.cc:157-196` | Add a limb/LOS test (reject if the line crosses an Earth-radius sphere); optionally a +grid topology constraint |
| B2 | **Dual Earth model** — satellite side is a sphere at equatorial radius, UE/SAGIN side is WGS84 → up to ~21 km mismatch contaminating range/elevation/TA/Doppler | `sgp4-mobility-model.cc:346-369`, `contact-graph-scheduler.cc:131-136` | Use WGS84 on the satellite side too (the UE-side helper already exists) |
| B3 | **Digital-twin ECI-as-ECEF** — `r_eci_km` (TEME/ECI) fed into an ECEF elevation calc → every predicted handover off by the Earth-rotation angle; same mislabel poisons the Influx `sat_*_m` push | `ntn-digital-twin/.../server.py:185-189`, `twin_loop.py:91-99` | Use the existing correct topocentric helper (`propagator.py:97-107`); rotate ECI→ECEF by GMST before elevation |
| B4 | **AiFlowMonitor emits non-spec KPM name** `TB.ErrTotalNbrDl.Rate` (G10 was only fixed in oran-ntn, not here) | `contrib/ntn-traffic/model/ntn-oran-ai-flow-monitor.cc:214,342,358` | Emit canonical `TB.ErrTotNbrDl`/`TB.TotNbrDl` (as already done in oran-ntn) |
| B5 | **FL variants are mislabels** — FedNova ≡ FedAvg (τ=samples), FedProx applies the proximal term server-side (wrong), SCAFFOLD absent; "gradients" are weight-vector proxies | `oran-ntn/.../federated-learning.cc:488-644`, `space-ric.cc:469,806` | Either implement them correctly or relabel to "FedAvg only" |
| B6 | **HARQ profile over-caps at 255**, violating Rel-17 `n32` ceiling | `ntn-real-stack-helper.cc:120-171` | Clamp `NumHarqProcess` at 32, not 255 |

---

## 4. OVERCLAIMS — honest relabeling needed (code is fine, the name/README/doc lies)

**STATUS 2026-06-27: all relabeled** (text/doc/comment only; full-tree build green,
Python docstrings parse). O1/O2 (ntn-traffic spine + cho/slice/rrc READMEs), O3 (sionna
README + CIR example), O4 (oran README control-loop section), O5/O5b (flexric-bridge +
asn1 codec honesty note), O6 (FL header), O7 (DQN/LSTM/ONNX + ns3-ai READMEs), O8 (twin
README), O9 (slice README/orchestrator), O10 (constellation/sagin READMEs), O12 (rrc/cho
SIB19/K_offset/RACH/ideal-RRC), O14 (scene-recorder beam note). O11 (EPC-vs-5GC) and O13
(802.15.3d) were verified as **already honest** in the code (no false claim) — left as-is.
Refinement: the "example with no real ISL" is `ntn-constellation-real-routed`; the
`isl-routed-traffic` example *does* carry a genuine satA↔satB ISL hop (label left accurate).

A competent referee will catch each of these. The fix is words, not engineering.

| # | Claim | Reality | Where to relabel |
|---|---|---|---|
| O1 | "real NR-NTN **FR1 S-band** air interface" | FR2 numerology (60 kHz SCS, mmwave-only); no 3GPP band number; 2.0 GHz is the n256 *uplink* used for DL; 50 MHz default exceeds the 20/30 MHz NTN-FR1 max | spine header, READMEs |
| O2 | "satellite beam / EIRP" | terrestrial 8×8 UPA with SVD beamforming; no reflector pattern, 3 dB beamwidth, or footprint; EIRP scalar back-solved to a target SINR | spine, cho, slice docs |
| O3 | "**Sionna RT** ray-traced CIR (multipath + per-tap Doppler)" | ray tracer collapses all paths to one scalar path-loss; the CIR taps fed to ns-3 are hand-authored synthetic Rician in the example | ntn-sionna README + example header + **prior gap doc §C** |
| O4 | "real RIC control / RIC-controlled toolkit" | open loop — all 13 framework xApps end at a CSV-logging handler; nothing actuates (only one inlined beam example truly closes the loop) | oran-ntn README, AI-native plan |
| O5 | "FlexRIC interop (live asn1c/cffi)" | JSON loopback to its own TCP stub; live asn1c path is vaporware (0 grep hits); C++ PER and Python JSON codecs are mutually incompatible | oran-ntn README, flexric-bridge |
| O5b | "E2AP/E2SM over SCTP, end-to-end" | the hand-PER codec is **not APER-bit-conformant** (full-byte CHOICE index, octet-aligned preambles, no extension markers / constrained-INTEGER bit-packing) → would not decode on an asn1c peer; and the live `e2-listener` ignores payloads (frame-type echo). The codec is real but **test-only**, never on the live control path | oran-ntn README, asn1/ |
| O6 | "federated learning FedAvg/FedProx/FedNova/SCAFFOLD" | only FedAvg is real (see B5) | oran-ntn README |
| O7 | "DQN / LSTM / ONNX / Triton / gRPC inference" | DQN=linear dot-product, LSTM=EWMA, ONNX/Triton/gRPC are real engines with **no shipped model** + mock runtimes; net behavior = heuristics | ns3-ai-ntn, oran-ntn AI xApps |
| O8 | "digital **twin**" | live dashboard of the *real-world* CelesTrak constellation; never ingests ns-3 state; one-way push | ntn-digital-twin README |
| O9 | "per-slice isolation guarantee" | open-loop shadow allocator; PRB decisions actuate no scheduler; one slice-agnostic cell; 5QI is a packet label never read by the MAC | ntn-slice README |
| O10 | "ISL / SAGIN **routing protocol**" | path-calculation libraries (`: public Object`) that install no forwarding table; one example carries packets over a true ISL hop and it uses stock global routing | constellation/sagin READMEs |
| O11 | "5GC core" (if claimed) | LTE **EPC** (MME/SGW/PGW, S1-AP); GTP-U real; no NGAP/N2/N3/AMF/SMF/UPF | keep calling it EPC |
| O12 | "SIB19 broadcast" / "K_offset" | SIB19 is a `TracedCallback` to a private buffer (no BCCH/PDSCH, no UE decode); K_offset is stored metadata applied to zero scheduling decisions | rrc/cho docs |
| O13 | "IEEE 802.15.3d THz" (if ever claimed) | window-based 6G carriers; no 2.16 GHz raster, channel index, or SC PHY (README is currently honest — keep it) | thz-ntn — do not add the claim |
| O14 | "NetSimulyzer 3D beam rendering" | `m_beams` is collected but never emitted; dead code | scene-recorder header |

---

## 5. WIRED-OFFLINE / architectural items (real models, not in the measured path, or a vendored-stack ceiling)

**STATUS 2026-06-27: documented as explicit scope** in `SCOPE_AND_LIMITATIONS.md`
(A1–A5, each with what-is-bounded · why · effect · may/may-not-claim · how-to-close),
with pointer notes added to the ntn-traffic, thz-ntn, ntn-rrc, ntn-slice and oran-ntn
READMEs. These remain open as engineering work; they are now bounded in writing.

Bigger lifts — scope as future work or document the boundary:
- **A1 — TR 38.811 fast fading missing:** the G1 excess model is large-scale only; no NTN-TDL multipath / Rician-K; clutter & SF constants are invented, not the §6.6.2 tables.
- **A2 — THz antenna/pointing/beam not in the measured packet path:** the dominant THz impairment lives only in the offline `ThzNtnLinkBudget` calculator; the `*-traffic` KPIs see FSPL+atmosphere only. Beam squint over the 10-20 GHz band is unmodeled.
- **A3 — DRX / TA / SIB19 are oracles not bound to the mmwave MAC/RRC:** DRX never gates PDCCH; TA never sets UL timing; SIB19 never reaches a UE.
- **A4 — Slice & RIC actuation:** the orchestrator's PRB split and the xApps' decisions don't drive the scheduler — would need a 5QI/slice-aware queue disc or scheduler hook.
- **A5 — Numerology & satellite antenna are mmwave ceilings:** the vendored NYU PHY can't do 15/30 kHz SCS or a reflector beam. A real FR1-NTN waveform/antenna would need a different PHY (e.g. 5G-LENA `nr`, which supports μ0/μ1) — a strategic decision, not a patch.

---

## 6. Corrections to the prior remediation doc (`STANDARDS_VALIDATION_GAP_ANALYSIS_2026-06-26.md`)

The fresh audit found my own prior doc overstated three items:
- **G10 partial, not complete** — canonical TB name added in oran-ntn only; the ntn-traffic AiFlowMonitor still emits `TB.ErrTotalNbrDl.Rate` (see B4).
- **G4 overstated** — claims RRC `t310/N310/N311` were relaxed; `ConfigureNtnRlcRrcTimers()` sets only `T300` + the three RLC timers. T310/N310/N311 and all RLF/reestablishment logic are absent.
- **§C miscredit** — the doc credited `sionna-cir-propagation-loss-model.cc` as a correct chained channel applying the ITU-R cascade; it applies Doppler/CIR on **synthetic** taps and does **not** apply the ITU-R cascade (see O3).
- **G5 note** — the shipped HARQ profile caps at 255, not the recommended n32 (B6); the `downlinkHARQ-FeedbackDisabled-r17` bitmap remains absent.

---

## 7. Direct recommendation

1. **Fix the 6 correctness bugs (B1-B6).** These are wrong, cheap, and a reviewer will treat them as defects, not simplifications. ISL-occultation, dual-Earth-model, twin ECI/ECEF, the KPM name, the FL mislabels, the HARQ cap.
2. **Relabel the 14 overclaims (O1-O14).** This is the single highest-leverage credibility action — the engineering can stay exactly as is; the names/READMEs/badges must stop claiming "real FR1 / satellite antenna / Sionna RT CIR / real RIC / FlexRIC interop / FedProx / DQN / digital twin / slice isolation guarantee / routing protocol / 5GC." Say what the code does: honest abstraction is publishable; disguised abstraction is not.
3. **Document the 5 architectural boundaries (A1-A5)** as explicit scope statements, and decide separately whether the FR1-numerology/antenna ceiling justifies migrating the PHY off NYU-mmwave to 5G-LENA `nr`.

The toolkit's substance — a real measured NR data plane, real orbital mechanics, real link-level/array physics, honest provenance — is genuinely strong and defensible. The exposure is entirely in the gap between what the labels claim and what the code does. Close that gap and it is referee-proof.
