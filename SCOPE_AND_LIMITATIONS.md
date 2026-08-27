# Scope & Limitations — ns3-ntn-toolkit
**Date:** 2026-06-27  **Author:** Muhammad Uzair, Independent Researcher

This is the authoritative statement of what the toolkit **does** and **does not**
model at the architecture level. It is the public record of the boundaries; the
layer-by-layer accuracy judgments and the channel, HARQ and timer fix logs that
produced them are internal engineering notes and are not published with the
toolkit. Anything a reader needs in order to judge a result is stated here rather
than left to a citation of a document they cannot open.

The items below (A1–A5) are **architectural boundaries**, not bugs: the underlying
models are real and correct in isolation, but they are either computed *outside* the
measured packet path or constrained by the vendored PHY. They are listed so that
papers, READMEs, and reviewers can rely on an explicit scope rather than an inferred one.

For each: **what is bounded · why · effect on results · what a paper may/▽may-not claim ·
what closing it would require.**

---

> **STATUS 2026-06-27 — A1 & A5(ii) largely CLOSED on the mmwave spine; A5(i)/A3/A4 unblocked by the nr integration.**
> A1: the model now uses the **real TR 38.811 §6.6.2 σ_SF tables** (per scenario, elevation-interpolated), **CL=0 for LOS** (spec-correct), and a **Rician small-scale fading term** with the §6.7.2 elevation-dependent K-factor. **NT-07 correction:** that term defaults to OFF and is not executing on the measured plane, deliberately: both radio backends already apply small-scale fading through the 3GPP phased-array spectrum model, and running the Rician process as well would multiply two independent fading realizations onto one link. Enable it only for a link with no 3GPP spectrum model in the path. Its normalization (unit mean power) and its elevation dependence are covered by `Tr38811FastFadingStatisticsTest`; before that they were asserted only by the comment above the code. *Remaining:* the full multi-tap frequency-selective NTN-TDL (§6.9.2). A5(ii): the **TR 38.811 §6.4.1 J1-Airy satellite beam pattern** is implemented (`NtnSatBeamGainModel`, opt-in via `NtnRealStackHelper::SetSatelliteBeam`) and verified (0 dB boresight, −3.01 dB at the half-beamwidth). **NT-07:** that setter had no callers anywhere in the tree. The pattern was still exercised on the measured plane by `ntn-tr38821-array-gain-calibration`, which constructs the model directly, but no scenario reached it through the helper, so no run assembled it into a helper-built propagation chain. `ntn-real-stack-smoke` now exposes `--satBeam`, `--beamwidthDeg` and `--beamCenterXKm`, and a fixed beam centre 300 km off the terminal costs a measured 11.07 dB (16.20 → 5.13 dB SINR) while a tracking beam costs 0 dB, which is the same reason a steered-beam calibration reports a constant offset. `NtnChannelExtrasReachTheChainTest` asserts the model is in the chain rather than merely constructible. `SetNtnScenario` had no callers either, so every run used the Suburban shadow-fading bins; it is now `--ntnScenario`. A2 (THz pointing) is wired into the measured path. **5G-LENA `nr` is now in the tree** (see A5), so A5(i)/A3/A4 are reachable on an nr spine.

## A1 — The measured channel carries TR 38.811 *large-scale* loss only (no fast fading)

- **Bounded:** the G1 `Ntn38811ExcessLossModel` adds elevation-dependent gaseous
  absorption (P.676), tropospheric scintillation (P.618), a scenario clutter
  constant, and elevation-binned shadow fading on top of Friis FSPL. It does **not**
  add NTN-TDL multipath or a Rician-K small-scale fading process (TR 38.811 §6.9),
  and its clutter (0.5–4 dB) and shadow-fading σ (3/2/1 dB) are tractable
  approximations, **not** the verbatim TR 38.811 Table 6.6.2-x values.
- **Why:** wiring the full NTN-TDL tapped-delay-line into the mmwave spectrum channel
  is net-new channel modelling; the large-scale terms were the high-value, low-risk
  re-use of the existing oracle math.
- **Effect:** measured SINR now varies correctly with elevation and scenario (verified),
  but it does not exhibit small-scale fast-fading variance; BLER/throughput reflect a
  smoother-than-real channel near the link margin.
- **May claim:** "elevation- and scenario-dependent large-scale NTN channel (TR 38.811
  §6.6 gas/clutter/shadow + P.618 scintillation, applied to the measured plane)."
  **May not claim:** "full TR 38.811 channel," "NTN-TDL," or "Rician fast fading."
- **To close:** add an `NtnTdlSpectrumPropagationLossModel` (TR 38.811 §6.9 taps +
  K-factor) chained on the spectrum channel; replace the σ/clutter constants with the
  §6.6.2 elevation tables. Code: `contrib/ntn-traffic/model/ntn-tr38811-excess-loss-model.*`.

## A2 — THz antenna / pointing / beam physics is offline, not in the measured packet path

- **Bounded:** the (genuinely strong) THz array-factor, beamforming codebook, and
  pointing-error models live in the standalone `ThzNtnLinkBudget` calculator. The
  `PropagationLossModel` chained into the *measured* `thz-ntn-*-traffic` examples adds
  only gaseous + rain + fog + snow; the dominant THz impairments (pointing loss, beam
  mispoint, **beam squint** over the 10–20 GHz band) do not reach the measured KPIs.
- **Why:** the array/pointing models predate the measured-plane wiring and were never
  re-homed as a chained loss; beam squint (frequency-dependent steering) is unmodeled.
- **Effect:** measured THz SINR/TBLER/goodput reflect FSPL + atmosphere only and are
  optimistic versus a real pencil-beam THz link.
- **May claim:** "THz atmospheric (P.676/P.838/P.840) effects on the measured plane;
  array/beam/pointing analysis via the offline link-budget calculator."
  **May not claim:** measured KPIs that "include THz pointing/beam-squint losses."
- **To close:** wrap the pointing/array loss as a chained `PropagationLossModel` on the
  THz traffic path; add a frequency-dependent (true-time-delay vs phase-shifter) array
  factor for squint. Code: `contrib/thz-ntn/model/thz-ntn-{antenna-array,pointing-error,propagation-loss-model}.*`.

## A3 — DRX, Timing-Advance and SIB19 are correct *oracles* not bound to the mmwave MAC/RRC

- **Bounded:** the NTN DRX state machine (+ HARQ-RTT-NTN timers), the common/UE-specific
  Timing-Advance model, and the SIB19 content/codec are each spec-faithful in isolation,
  but none is wired into the live stack: DRX never gates PDCCH monitoring on the mmwave
  UE-MAC, TA never sets the UL transmit timing, and SIB19 is emitted on a TracedCallback
  rather than carried on BCCH/PDSCH to a decoding UE. K_offset is stored metadata applied
  to no scheduling decision.
- **Why:** the vendored mmwave UE-MAC/RRC (forced `UseIdealRrc=true`) exposes no hooks to
  drive DRX active-time, UL timing, or SI acquisition from these external objects.
- **Effect:** DRX awake-fraction, TA, and SIB19 are reported as analytic/structural
  results, not as behaviours the data plane experienced.
- **May claim:** "an NTN DRX/TA/SIB19 model (TS 38.321/38.213/38.331-faithful) evaluated
  alongside the measured data plane." **May not claim:** "DRX-gated PDCCH," "TA-corrected
  UL scheduling," or "UE acquires SIB19 over the air."
- **To close:** bind the DRX FSM to the mmwave UE-MAC PDCCH-monitoring gate, feed TA into
  the UL allocation timing, and deliver SIB19 on a modelled BCCH. Code: `contrib/ntn-rrc/`,
  `contrib/ntn-cho/model/ntn-timing-advance.*`.

## A4 — Slice and RIC decisions are computed but do not actuate the scheduler

- **Bounded:** the slice orchestrator's per-slice PRB split and the O-RAN xApps'
  E2SM-RC decisions are real algorithms on measured (or honestly-tagged geometry-budget)
  inputs, but their outputs are logged (CSV/SDL/traces), not applied: there is one
  slice-agnostic mmwave cell, and the framework RC-action handler records decisions
  without changing HO/PRB/beam. The single genuine closed loop is the inlined
  `oran-ntn-ric-controlled-traffic` beam example.
- **Why:** the mmwave scheduler is not slice/5QI-aware and exposes no per-slice queue or
  generic RC-actuation hook; only the beam example reaches into a tunable parameter.
- **Effect:** "isolation held/violated" and xApp "control" are observed statistically on a
  shared cell, not enforced; the control loop is open except the one beam example.
- **May claim:** "a measured-input slice-orchestration / xApp decision layer, with a
  closed-loop beam-control example." **May not claim:** "enforced per-slice RRM isolation"
  or "RIC-controlled scheduling/HO" for the framework at large.
- **To close:** add a 5QI/slice-aware queue-disc or scheduler hook the orchestrator drives,
  and a generic RC-action→RRC/MAC actuation path. Code: `contrib/ntn-slice/model/slice-orchestrator-xapp.*`,
  `contrib/oran-ntn/` (RC action handler).

## A5 — Vendored mmwave PHY ceiling: FR2 numerology and no satellite-antenna pattern

- **Bounded:** the shared spine reuses the NYU `mmwave` PHY, which supports only 60/120 kHz
  SCS (FR2 numerology) and models the gNB as a terrestrial UniformPlanarArray with SVD
  beamforming. It therefore **cannot** produce the 15/30 kHz numerology that L/S-band
  NR-NTN FR1 uses, nor a satellite reflector beam with a defined 3 dB footprint / roll-off.
- **Why:** this is an architectural property of the vendored PHY, not a configurable knob.
- **Effect:** "S-band" runs are an FR2-numerology mmWave waveform at an S-band carrier; the
  "satellite beam" is a terrestrial array. The link-level AMC/MCS/LDPC-BLER chain on top is
  real, but the frequency-domain/antenna framing is not 3GPP-NTN-conformant.
- **May claim:** "a real link-level NR data plane (AMC/MCS/LDPC-BLER, measured KPIs) at an
  S-band carrier." **May not claim:** "3GPP NR-NTN FR1 waveform/numerology" or a "satellite
  antenna/beam pattern" *on the mmwave spine*.
- **UNBLOCKED 2026-06-27:** **5G-LENA `nr` (5g-lena-v3.3.y) is now integrated** at
  `contrib/nr` and builds clean on ns-3.43 (one compat patch: an `#undef MIN_NO_CC/MAX_NO_CC`
  guard in `nr-common.h` for the lte-macro collision); `cttc-nr-demo` runs. `nr` supports
  μ0/μ1 (15/30 kHz FR1 numerology), real FR1-NTN bands/BWPs, and is TR 38.821-calibratable.
  The migration is therefore now a code path, not a missing dependency.
- **DONE 2026-06-27 (spine + first flagship):** `NtnNrStackHelper`
  (`contrib/ntn-traffic/helper/ntn-nr-stack-helper.*`) is built on `NrHelper` /
  `NrPointToPointEpcHelper`, **alongside** `NtnRealStackHelper`. It runs **FR1 numerology
  (μ0=15 kHz / μ1=30 kHz) at S-band** with real measured KPIs (`RxPacketTraceUe` SINR/TBLER
  + FlowMonitor). Demo `ntn-nr-fr1-demo`: μ1 → 22.96 dB SINR / 73.8 Mbps; μ0 → 22.97 dB /
  73.7 Mbps. **First flagship migrated:** `ntn-cho-nr-real-stack` (non-destructive sibling of
  `ntn-cho-real-stack`) runs the full CHO trigger set on the FR1 plane — 60 s pcho pass:
  measured SINR 23.4→15.6 dB tracking the 551→673 km slant, 1 RACH-less handover (cell 1→101
  at t=42 s, pre-computed TA). This concretely closes **A5(i)**.
  - **Random access over the NTN round trip (RRC-4, 2026-08):** handovers remain RACH-less as
    above, but the RAR window is no longer unexamined. `NtnRachWindow` (contrib/ntn-rrc) sizes
    `ra-ResponseWindow` from the real service-link round trip, and
    `NtnRealStackHelper::SetNtnRachWindow(true)` writes it onto every live `NrGnbMac` — the
    value the UE receives via `GetRachConfig`, so it reaches the UE's own timeout. nr arms that
    timeout at `slotPeriod × (6 + N)` from the preamble (TS 38.321 §5.1.4), which makes the
    question purely arithmetic. **LEO-600 at 30 kHz needs N=4; nr's default of 3 buys 4.500 ms
    against a 4.5036 ms requirement and misses by 3 µs** — so the default really does fail, and
    it fails in a way that would read as an unexplained attach failure. **The cap is the real
    limit:** the attribute is bounded at 10, so the same orbit at 60 kHz (N=12), LEO-1200
    (N=12) and GEO (N=473) **cannot complete random access on this stack at all**. That is a
    structural gap, not a tuning one: TR 38.821 §7.3 resolves it by offsetting the window
    *start* with `ta-Common`, which nr v3.3 does not implement. The helper reports the
    shortfall via `GetRachWindowVerdict()` rather than clamping silently.
  - **NTN gotcha for nr-spine modules:** the in-tree 3GPP UMi pathloss assumes a local-ENU
    frame; modules feeding ECEF (SGP4/TR 38.811) coordinates must override the BWP
    large-scale loss with `FriisPropagationLossModel` (3D slant-range, frame-independent) —
    as the spine now does — while keeping the 3GPP spatial model for array gain.
- **FULL IN-PLACE ROLLOUT DONE 2026-06-27:** `NtnRealStackHelper` itself is now **dual-backend**
  (`SetRadioBackend(Mmwave|Nr)` + `SetNumerology`, radio-agnostic `GetServingCellId()`); the nr
  RxPacketTraceUe feeds the *same* accumulators, so per-UE SINR, health gates, ORAN flows and the
  AI flow monitor work on nr unchanged. The **existing** real-stack examples were migrated **in
  place** (no siblings) to a `--radio` flag **defaulting to nr** — **~54 of ~58 now run on FR1 nr
  by default**, mmwave preserved via `--radio=mmwave` (verified zero-regression). Verified: full
  tree builds; nr smoke sweep PASS across every distinct path (smoke 24.9 dB, slice+ORAN 26.0,
  oran E2/RIC 25.4, thz-chain 1.76, observability/scene 18.5, constellation multi-UE 23.4 dB);
  6 test suites PASS; mmwave fallback PASS. nr gNB array lifted to 8×8 so default EIRPs give a
  healthy link.
  - **4 examples kept mmwave-default (documented, not forced):** the 3 ntn-fapi slot-loop examples
    (per-TB FAPI CRC loop reads the mmwave-specific `RxPacketTraceUe` callback) and
    ntn-tr38821-calibration (its offset gate is validated against mmwave-internal array gain).
    All 4 still carry the `--radio` plumbing (nr selects the air interface).
- **Remaining follow-ons:** a radio-agnostic per-TB trace hook on the helper (lets ntn-fapi run on
  nr too); chain the TR 38.811 large-scale model (A1) + §6.4.1 beam (A5(ii)) onto nr's BWP channel;
  bind DRX (A3) and per-5QI QoS scheduling (A4) via nr's native support. The numerology ceiling
  (A5(i)) is closed and nr is now the default radio across the toolkit.

## A6 — The air-interface propagation delay cannot be enabled at LEO altitudes

`NtnRealStackHelper::SetAirInterfaceDelay(true)` puts a real
`ConstantSpeedPropagationDelayModel` on the radio channel, so the service-link slant is borne
by the air interface rather than folded into the backhaul. It exists, it is documented, and
until 25 August 2026 it had **zero callers anywhere in the tree**: no example and no test had
ever switched it on, so nothing would have noticed if it had stopped working.

Switching it on shows why. Probed with one UE, a light periodic profile and SIB19 K_offset
consumption enabled, on the vendored nr v3.3 stack:

| Slant | Result | | Slant | Result |
|---|---|---|---|---|
| 50 km | works | | 300 km | **aborts** |
| 200 km | works | | 350 km | works |
| 250 km | works | | 400 km | works |
| 500 km | works | | 600 km | **aborts** |

The failures are `Cannot TX while RX` inside `nr-spectrum-phy`. Two things follow. First, the
pattern is **not a threshold**: 300 km fails while 350 km and 500 km pass, so the delay is
interacting with the TDD slot pattern and whether a geometry survives is not predictable from
the slant alone. Second, LEO-600, the toolkit's own reference shell, is among the geometries
that do not work, and a saturating downlink trips the same assertion even at a slant that
otherwise passes.

An earlier note on the setter claimed that consuming the K_offset unlocked this for a single UE.
That was wrong, and it has been corrected in the header.

**What this means in practice.** No shipped LEO scenario carries the slant on the air interface;
every one folds it into the backhaul, where the measured end-to-end one-way delay is still
physically correct. What is missing is the delay being borne by the air interface itself, which
matters for HARQ timing, scheduler behaviour and anything sensitive to where in the stack the
latency appears. A regression test now exercises the path at 350 km so the code does not rot,
and that is the honest extent of the capability until the ns-3.48 migration brings a stack with
real per-UE timing advance.

---

## One-line guidance for a manuscript

Frame the toolkit as **"a real link-level NR data plane and real orbital/array physics, with
an NTN large-scale channel on the measured plane"** — and state A1–A6 as explicit scope. The
genuinely-defensible headline claims are the measured KPI provenance (PHY-trace SINR/TBLER +
in-band OWD/jitter/loss), the SGP4/Doppler/WGS-84 geometry, the THz/array physics (as an
offline study), and the honest sim-health gating — not end-to-end 3GPP-NTN protocol/PHY
conformance, enforced slicing/RIC control, Sionna-RT multipath, or a deep-learning RIC.

---

## A7 — No 3GPP Rel-19 AI/ML life-cycle management anywhere (AI-12)

**Stated because it is absent, not because it is partial.** A grep across
`contrib/ns3-ai-ntn` and `contrib/ntn-digital-twin` for the vocabulary of TR 38.843
life-cycle management (`lcm`, `life-cycle`, `model_id`, `drift`, `38.843`, `Rel-19`)
returns exactly one hit, and it is the English phrase "Typical lifecycle:" in a C++
header describing object construction.

The inference request carries `model_name`, `ue_id`, `nr_cgi`, `sim_time_s` and a
tensor. It carries no **model identity** distinct from a display name, no
**functionality identifier**, no **activation / deactivation / switching / fallback**,
no **applicability conditions**, no **performance monitoring** or drift signal, and
no **UE capability reporting** for AI/ML. None of that is stubbed or partial: none
of it exists.

**Not implemented here, deliberately.** A Rel-19 LCM framework is a feature, not a
defect fix: it would mean a model registry, an identity and applicability scheme, a
monitoring path with a drift metric, and a control surface to activate and fall back.
Sketching a subset would produce exactly the shape this audit keeps finding, a
standards-named field that no standard produced. What is owed today is that nobody
reads the AI/ML support as covering it, which is what this section is for.

**What does exist:** a Gymnasium/ns3-ai bridge with a version and schema handshake
(AI-11), an ONNX/Triton-shaped inference transport, and offline SB3 and PyG
sandboxes. The multi-agent trainers are single-agent PPO/SAC over N independent
environment copies, not MAPPO/MASAC (AI-09).
