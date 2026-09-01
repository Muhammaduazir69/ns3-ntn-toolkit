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

## A12 — The S-band scenarios now sit inside a legal n256 downlink channel

**What was wrong.** The helper defaulted to a 2.0 GHz carrier in a 30 MHz channel,
and roughly forty examples inherited or restated it. Neither value is legal for a
downlink. TS 38.101-5 Table 5.2-1 puts n256's **uplink** at 1980-2010 MHz and its
**downlink** at 2170-2200 MHz, so 2.0 GHz is an uplink frequency being used as the
downlink carrier. Table 5.3.5-1 allows 5, 10, 15 and 20 MHz channels on n256;
30 MHz is the width of the *block*, not a permitted channel.

Every run said so in its own health record, `air_interface` with `pass=0`, which is
how it was found. It had been reported honestly and left unfixed, across about
forty-five examples, which is not a corner case.

**Fixed.** The helper now defaults to **2185 MHz**, the centre of the n256 downlink
block, in a **20 MHz** channel, the widest legal one. Fourteen examples carrying a
hardcoded 2.0 GHz and four carrying 30 or 50 MHz were moved with it. The health tag
changes from `nr-fr1-n256-uplinkcarrier` to `nr-fr1-ntn-n256` and `pass` goes to 1.

**What it cost, measured** on `ntn-real-stack-smoke` at 60 s with 4 UEs:

| carrier | channel | DL SINR | throughput | conformant |
|---|---|---:|---:|---|
| 2.0 GHz | 30 MHz | 30.39 dB | 20.00 Mbps | no |
| 2.185 GHz | 30 MHz | 29.11 dB | 20.00 Mbps | no |
| **2.185 GHz** | **20 MHz** | **31.21 dB** | **20.00 Mbps** | **yes** |

Moving the carrier up costs 1.28 dB, which is the extra free-space loss and nothing
else. Narrowing the channel to a legal width returns more than that, because the
noise floor falls faster than the bandwidth is lost. Application throughput does
not move at all, being offered-load limited. Conformance was therefore close to
free here, which is the main reason it should have been done sooner.

**Coverage, measured by re-running every example.** Before: about forty-five
scenarios non-conformant. After: **two**, and both on purpose.
`ntn-tr38821-calibration` and `ntn-tr38821-array-gain-calibration` stay at 2.0 GHz
because they calibrate against TR 38.821 Set-1, whose S-band study case is
specified there. Calibrating against a study at a carrier the study does not use
would be the more serious error, so they report `pass=0` and carry a band note
saying why. Everything that represents a deployment sits at 2185 MHz.

Bands other than n255/n256 are unaffected: the FR2 and THz scenarios are not NTN
FR1 and are not judged against these tables.

**Enforced, not just reported.** The flag had read `pass=0` for months with nothing
failing on it. `tools/check_band_conformance.py` is now gate 17 of
`check_ntn_standards.py`, runs six scenarios across six modules, and fails on
either an uplink-block carrier or an unsupported channel width. Both arms are
tamper-tested.

**Consequence for existing results.** Any figure measured before 2026-09-01 was
taken at 2.0 GHz in a 30 MHz channel and is not band-conformant. Numbers move by
about a decibel, in the favourable direction, and need re-running before they can
be described as NTN FR1 results.

## A20 — The CHO success rate is sampled before the handover can have completed

`ntn-cho-full-constellation` decides a handover, actuates it, and grades it in
three consecutive statements:

```
const bool requested   = g_rs->TriggerHandover(0, chosen);
const uint32_t completions = g_rs->GetHandoverCount();
const bool success = requested && (completions > g_hoCompletionsSeen);
```

`GetHandoverCount()` counts `NrGnbRrc` **HandoverEndOk** events. It is read on the
statement after the request is issued. An RRC reconfiguration-with-sync cannot
complete in zero simulated time: it is signalling across an X2 and an air
interface with propagation delay. The counter therefore cannot have advanced, and
`success` is false by construction.

Measured: seeds 1, 2 and 3 each report one handover at a 0.00 percent success
rate. That is the accounting, not the radio. The `failure_reason` column, which
was declared in the schema and never written until now, records
`no-rrc-completion-observed` for exactly this reason: the request went out and
the completion had not arrived a statement later.

There is a second-order effect once more than one handover occurs. The counter is
cumulative and `g_hoCompletionsSeen` is updated only here, so a completion that
lands between two decision ticks is credited to the NEXT request rather than to
the one it belongs to.

**Fixed 2026-09-01.** This one had a single correct implementation rather than a
choice to make, so deferring it alongside A19 was wrong: A19 asks which policy is
intended, while this asks only that an outcome be read after it happens. The
verdict now comes from the RRC's own `HandoverEndOk` for the cell the handover
was aimed at, matched by cell id, with the event row and the GeoJSON feature
buffered and emitted when it resolves on the following decision tick.

That pattern was already in the tree: `ntn-cho-real-stack` binds the same trace
and attributes completion by cell id. This scenario, the one the campaign runs,
inferred the outcome from a cumulative counter instead, which can only say that
some handover finished rather than which one. The right implementation was one
file away and was not reused.

Re-measured at 300 s with `--d1Threshold=761341`, seeds 1, 2 and 3: one handover
each at a 100 percent success rate, with an empty `failure_reason`. The handover
was completing all along. What was broken was reading `GetHandoverCount()` on the
statement after issuing the request, one tick before the completion could arrive.

The number moved from 0 to 100 percent and neither figure was ever a measurement
of the radio, so nothing that quoted the old one should be carried forward. A19
above is untouched and still open.

Note the history: this line replaced `const bool success = true;`, which made the
rate 100 percent by construction. The current form makes it 0 percent by
construction. The measurement has never been of the radio.

## A21 — The campaign scenario actuates the radio behind its own algorithm's back

`ntn-cho-full-constellation` calls `NtnRealStackHelper::TriggerHandover` directly
from its decision tick and never registers
`NtnChoAlgorithm::SetHandoverExecutionCallback`. With no callback, `ExecuteHandover`
takes its standalone decision-model branch, logs that no radio was actuated, and
self-completes with `NotifyHandoverComplete(target, true)`.

So the radio IS actuated, the log says it is not, and the algorithm's own state
records every handover as a success whatever the RRC does. `GetMechanismStats()`
figures that ride on that state, the interruption and RACH-less counters, are
assumptions rather than observations. The scenario's REPORTED success rate is not
affected: it comes from the example's own counters, which since A20 are attributed
from the `HandoverEndOk` trace.

`ntn-cho-real-stack` wires this correctly, so the pattern exists one file away:
the algorithm actuates through the callback, the RRC trace feeds
`NotifyHandoverComplete` back, and a refusal is reported instead of being masked
by T304.

**Fixed 2026-09-01, after two wrong diagnoses of my own.** The algorithm now
actuates through `SetHandoverExecutionCallback`, and the `HandoverEndOk` trace
hands the outcome back with `Simulator::ScheduleNow` so the completion runs with
the stack unwound.

The first two attempts segfaulted every seed, and I recorded here that the cause
was re-entrancy, `ExecuteHandover` invoking the callback while the completion path
mutated the candidate map underneath it. That was a guess and it was wrong. A
backtrace put the crash in `Ptr<CallbackImplBase>::operator=` called from `main`,
i.e. at setup rather than in the simulation: I had registered the callback 48
lines before `g_cho = g_choHelper->CreateChoAlgorithm()`, so it was a method call
on a null pointer. Moving the registration after the object exists fixes it.

Measured at 300 s with `--d1Threshold=761341`, seeds 1, 2 and 3: exit 0, one
handover each at 100 percent with an empty `failure_reason`, and the "no handover
callback registered" line gone from the log, so `ExecuteHandover` no longer takes
its standalone branch and the algorithm's own state follows the radio rather than
assuming success. The `ntn-cho` suite passes.

The lesson worth keeping: a plausible mechanism written into a boundary document
is still a guess until something measures it. Two attempts were spent on a
re-entrancy theory that a thirty-second backtrace refuted.

## A19 — TTE-aware selection never compares against the serving cell it is leaving

The module describes the novelty as admitting "a target beam only when it will
stay in coverage long enough to be **worth the switch**". The implementation has
the first half and not the second. `SelectBestCandidate`'s admissible filter is

```
info.admitted && info.d1Met && info.sinr_dB >= qualityThreshold && info.tte >= tteMinimum
```

then sort by time-to-exit descending. Nothing in it reads the serving cell's own
time-to-exit, and nothing requires the candidate to be better than what the
terminal already has. `servingTte` is computed in exactly one place, the T1
trigger arm, and never here. The serving cell is also excluded from the candidate
map by design, so it cannot enter the comparison even implicitly.

Measured consequence, `ntn-cho-full-constellation` at 300 s with
`--d1Threshold=761341`, on the first decision tick:

```
t=5.0s  cell 1 -> 3   servSINR 29.73 dB   candSINR 27.90 dB
        elevation_before 87.2 deg   elevation_after 51.5 deg
```

The terminal leaves a satellite at 87 degrees for one at 51 degrees that is
1.8 dB worse. The reason is worth stating precisely, because the obvious reading
is wrong: the serving satellite IS descending, reaching 37.7 degrees by t=130 and
19.9 by t=230, so leaving it eventually is right.

What makes the timing indefensible is the horizon. `m_maxPredictionWindow` is
120 s, and at t=5 both candidates report exactly 120.00 s, i.e. saturated: their
exit is further away than the estimator looks. The serving cell, still above
19 degrees more than 200 s later, would saturate too. So this is a tie at the cap
between staying and switching, and the rule has no way to see it, because the
serving cell's time-to-exit is never computed and the serving cell is excluded
from the candidate set. Facing a tie it cannot observe, the policy switches.

A rule that compared against staying would break that tie the other way: a
handover has a cost and an equal predicted time-of-stay does not pay for it. Note
also that time-to-exit is genuinely time-varying over the run, taking values from
11.31 s upward, so the ranking dimension is real; it is saturated at this
particular instant, which is exactly when the missing comparison matters most.

Two smaller things visible in the same row. The handover fails, and
`failure_reason` is empty, so a failed handover records no cause. And across
seeds 1, 2 and 3 the run produces one handover and a 0 percent success rate every
time, with 1 admission out of 108, 102 and 100 evaluations, so this configuration
carries almost no stochastic spread for a Monte Carlo campaign to average over.

The decision is deliberately unchanged: which way that tie should break decides
what the manuscript's mechanism is, and that is the author's call rather than a
fix to land quietly.

What has changed is that the missing quantity is now recorded.
`handover_events.csv` carries a `serving_tte_s` column, computed with the same
estimator and the same TR 38.821 ten-degree gain threshold the candidates are
ranked by, so the two are directly comparable. On the run above it reads:

```
t=5.000  1 -> 3   tte_target=120.00   tte_serving=120.00   success=1
```

The handover gained exactly nothing in predicted time-of-stay, and the artifact
now says so on the row rather than leaving it to be inferred. Deciding the policy
needs that number; producing it does not require deciding the policy.

## A18 — The DRX duty cycle is a function of the poll cadence, not of the terminal

`ntn-rrc-drx-data-traffic` reports a DRX duty cycle and a power saving derived
from it. The gate itself is real: when the state machine says asleep the example
calls `SetTransmitEnabled(false)` and the downlink flow genuinely stops, which
was a fix (RRC-3) for an earlier version that multiplied the goodput by the awake
fraction afterwards and called that an effect.

What is not real is the sleep schedule driving the gate. The state machine learns
about traffic from `DrxPoll`, which samples the sink's byte counter and calls
`NotifyDataActivity()` when it has grown. That is sampling, not reacting to
packets, so the inactivity timer's behaviour follows the poll cadence. Measured
by changing only that cadence and nothing else:

| poll interval | reported awake fraction | measured goodput |
|---|---|---|
| 320 ms (one DRX long cycle) | 12.5% | 24.910 Mbps |
| 10 ms (a quarter of the on-duration) | 93.4% | 24.992 Mbps |

Neither is the terminal's duty cycle. The 12.5% coincides with
`onDuration / longCycle`, which is what makes it look like a result.

Two further consequences worth stating. The throughput barely moves in either
case, 24.910 against 24.997 Mbps ungated, so the gate is not costing what an
87 percent sleep would cost, and the "DRX effective goodput" line is still
goodput multiplied by the awake fraction. And the honest fix, notifying activity
from the sink's per-packet `Rx` trace, would make the deeper problem visible
rather than hide it: under this example's own `CBR_SATURATING` flow the
inactivity timer restarts on every arrival, so a conforming terminal barely
sleeps and there is no power saving to demonstrate without a bursty profile.

The poll cadence is deliberately left where it was. Swapping one arbitrary
sampling rate for another would replace a known artifact with an unknown one.

## A17 — Several advertised stack features are available but off by default

`NtnRealStackHelper` carries the features the documentation lists, and most of
them are opt-in. Counted over the NTN example sources (186 `.cc` files across the
twelve modules; the sweep runs the 94 that are built):

| capability | setter | examples using it |
|---|---|---|
| RLC AM | `SetRlcAmEnabled` | **0** |
| HARQ | `SetHarqEnabled` | 1 |
| MIMO | `SetMimo` | 1 |
| uplink traffic | `SetUplink` | 1 |
| satellite beam pattern | `SetSatelliteBeam` | 2 |
| NTN-stretched HARQ pool | `SetNtnHarqProfile` | 2 |
| K_offset consumption | `SetKOffsetConsumption` | 2 |
| strict health gates | `SetStrictGates` | 2 |
| handover | `SetHandover` | 8 |

None of this is a defect: an opt-in feature is a legitimate design, and the
per-run state is recorded in `sim_health.csv` for the ones that change the
measured plane. It matters because prose that lists what a stack *contains*
reads as a description of what the shipped scenarios *do*. The `ntn-traffic`
README said "real HARQ with an NTN-stretched process pool" in its overview and
"HARQ off by default" one section later, in the same document.

The practical consequence worth stating: **every shipped run carries RLC UM, not
AM**, on both backends, because `SetRlcAmEnabled` has no callers. A study that
needs acknowledged-mode retransmission behaviour has to turn it on, and none of
the committed results were produced with it.

## A16 — The band-conformance gate covers the FR1 spine, not the whole tree

`tools/check_band_conformance.py` runs six scenarios. Its verdict line used to
read "6 scenario(s) inside a legal NTN band", which is true and reads like a
statement about the toolkit; it is a statement about six examples. Counted over
the full sweep, 66 examples write a band row.

| | count | |
|---|---|---|
| conformant, inside an n256 downlink channel | 45 | |
| outside any FR1 NTN band, by design | 21 | |
| — `thz-ntn` at 100 GHz | 8 | no FR1 band applies to a terahertz carrier |
| — Ku/Ka scenarios at 12, 20, 28 GHz | 11 | `ntn-sionna` ray tracing, `oran-ntn` RIC placement, `ntn-sagin` flight links, `ntn-observability` |
| — TR 38.821 calibration pair at 2.000 GHz | 2 | see below |

The last pair is a real standards tension rather than an oversight.
`ntn-tr38821-calibration` and `ntn-tr38821-array-gain-calibration` sit at
2.000 GHz with a 30 MHz channel. TS 38.101-5 Table 5.2-1 puts the n256 downlink
at 2170 to 2200 MHz and Table 5.3.5-1 permits 5, 10, 15 and 20 MHz, so neither
value is deployable. They are correct anyway: TR 38.821's Set-1 reference
configuration is specified at 2 GHz S-band, and a calibration run has to use the
study's own carrier or it is not calibrating against that study. Moving them into
n256 would make them conformant and meaningless.

What the gate does not do is catch a NEW example hardcoding an illegal S-band
carrier outside its six-scenario list. The gate's own output now says so.

## A15 — `dl_sinr_db` is a decibel-domain mean, and it is only safe on a tight distribution

The reported mean SINR is the arithmetic mean of the per-transport-block SINRs
*in decibels*, which is the geometric mean of the linear values, taken with a
1e-12 floor at -120 dB. When the sample set is bimodal, the floored samples
dominate and the number stops describing the link the decoder saw.

Found by checking measured throughput against the Shannon bound of the reported
SINR across the whole example sweep. One scenario violates it:
`thz-ntn-isac-coexist-traffic` delivers 4.27 Mbps over 20 MHz at a reported
-85.19 dB, where the bound is essentially zero. Its linear-domain mean is
+35.70 dB, a 121 dB difference, and that value is consistent with the rest of the
record: 8613 blocks decoded, 78.5 percent block error, 4.27 Mbps at the sink
against a bound near 237 Mbps.

How far this reaches, measured:

| scenario | dB-domain mean | linear mean | gap |
|---|---|---|---|
| `ntn-real-stack-smoke` | 31.56 dB | 31.83 dB | 0.26 dB |
| `ntn-cho-handover-traffic` | 18.37 dB | 20.64 dB | 2.26 dB |
| `thz-ntn-isac-coexist-traffic` | -85.19 dB | +35.70 dB | 121 dB |

The manuscript's SINR figures come from the tight-distribution cases, where the
two statistics agree to well under a decibel, so no published number moves. The
definition is deliberately left unchanged for that reason: redefining it would
silently shift every SINR the toolkit has ever reported. `dl_sinr_db_linear_mean`
now ships beside it, so a bimodal run announces itself rather than presenting one
misleading figure, and any scenario quoting SINR near a noise floor should use
the linear-mean row.

## A13 — A CHO trigger that fires is not always the thing that moves the terminal

Until 2026-09-01 none of the six standardized handover trigger classes was
running at all. Four independent defects each made that certain: the radio's own
X2 handovers drained the candidate map after two executions (CHO-20), the
geometric classes evaluated against a UE position nothing ever set (CHO-21),
D2/A3/D1 were admitted by their evaluator and then refused by a time-to-exit
filter belonging to a different trigger (CHO-19), and A3 contained no A3
condition (CHO-18). The six-trigger conformance gate stayed green throughout,
because the number it read was produced by the scenario's fallback rule, which is
identical for all six classes. All four are fixed and the gate now reads a
per-trigger fire count that only that trigger's condition can produce.

What remains, and is a boundary rather than a defect: firing is not actuation.
On the shipped two-cell pass the vendored NR A3-RSRP and X2 machinery performs
the cell change on its own schedule, and it usually gets there first. Measured
over 300 s, five of the six classes fire and decide zero handovers; only D2
decided one. `ntn-cho-handover-traffic` therefore reports trigger-decided and
fallback-decided handovers as separate counts, and they must not be summed or
quoted as though the CHO layer caused every cell change in the run.

A study that needs the CHO decision to be the *cause* of the handover has to
suppress or widen the underlying A3 hysteresis (`--hoHystDb`) so the radio stops
pre-empting it. That is a scenario design choice, not something the module can
decide, so it is left explicit here.

## A14 — The flagship CHO campaign does not reproduce on the current code

`ntn-cho-full-constellation` produces the manuscript's CHO table. Run today at
the campaign's own settings (600 s, seed 1) it produces **zero** CHO handovers,
against 137 in the committed April 2026 data. Two separate reasons, both
measured rather than inferred.

First, `tte_computations.csv`, the file that documents the TTE-aware admission
behaviour, did not record the algorithm. Its `admitted` column was computed by
the example from a parallel GEO oracle, and an oracle cannot disagree with the
rule that produced it. Logging the algorithm's own verdict beside it, over a
120 s pass: the oracle admitted on 46 of 46 evaluations, and the algorithm had
admitted none of them. Both columns now ship, named `oracle_admitted` and
`cho_admitted`.

Second, the reason the algorithm admitted nothing is a parameter sized against
the wrong quantity. `d1Threshold` is 50 km, a beam-footprint radius, but
CondEventD1 here measures the terminal against the sub-satellite point, which on
a 550 km shell is 23 km away at 87 degrees elevation, 280 km at 61 degrees and
545 km at 42 degrees. D1 therefore holds only near zenith and everything gated on
it stays silent. Raising the threshold to 600 km restores admission and produces
a handover in the same run.

The default is deliberately left as it is. Which D1 semantics the study wants,
and therefore which threshold is correct, decides what the CHO results mean, and
that is an author decision rather than something to change quietly under a
manuscript. The scenario now computes the answer for whoever makes it: the
warning reports the lowest serving elevation the run reached and the threshold
that would have covered that whole pass. On the 780 km shell, seed 1, the serving
satellite descends to 40.5 degrees, where the terminal is 761 km from the
sub-satellite point, so `--d1Threshold=761341` covers the pass end to end. That
also explains the 600 km probe above admitting only intermittently: 600 km is
inside 761 km, so it covers the high-elevation part of the arc and not the rest. What has changed is that the scenario can no longer fail silently: a
run whose algorithm admitted no candidate on any tick now prints a warning naming
the threshold and the geometry, instead of writing a full set of plausible CSVs
and reporting a 100 percent success rate over zero handovers.

Re-running the campaign is priced in A11. This boundary is the reason it needs
re-running, and it stands independently of the stale-binary problem the
`run_mc_sweep.sh` guard already covers.

## A11 — The published Monte Carlo campaign is no longer affordable as configured

`papers/sim_runs/run_mc_sweep.sh` runs four algorithms across ten seeds, 600 s of
simulated time each, 30 UEs. Forty runs.

That configuration was affordable in April 2026 because the binary it used was the
pre-real-stack version of `ntn-cho-full-constellation`. The 24 June commit
"real-stack full-constellation, measured SINR and real CHO algorithm" replaced the
analytic plane with a full NR spectrum PHY, and the cost changed by orders of
magnitude.

Measured on this machine, uncontended: one run of `ntn-cho-full-constellation` at
**20 s** of simulated time with 30 UEs takes **1004 s** of wall clock, a slowdown
of **50x against real time**. Extrapolating linearly in simulated time, which is
reasonable for a steady-state radio run but is an extrapolation and not a
measurement, the campaign's 600 s puts one run at about **8.4 hours** and the
forty-run campaign at about **335 hours, near fourteen days** of continuous
compute.

**Why this matters.** The committed `mc_table.csv` was produced on 29 April by the
cheap binary, so its numbers describe code the manuscript no longer documents.
Regenerating them on the current code is the right thing to do and is not a
session-scale task; it is a machine-time decision for the author.

**Options, in the order I would consider them.** Reduce the seed count and report a
wider confidence interval, which the table already carries. Reduce simulated time
per run, at the cost of fewer handovers per seed. Reduce UE count, noting that mean
DL SINR on one beam falls from 28.45 dB at 4 UEs to 2.61 dB at 30, so the UE count
is not a free parameter. Or run the campaign on `ntn-cho-real-stack`, which is
cheaper and also exercises the real trigger set.

**What a paper may not do** is quote the April numbers against the current code.
The staleness guard now in both sweep scripts refuses to run against a binary older
than its sources, so the specific mistake that produced them cannot recur silently.

## A10 — One shipped example exits non-zero, by design

Measured by executing all 93 buildable examples. Two of the three that failed when
this audit started are fixed; the third is deliberate.

**`ntn-tr38821-array-gain-calibration`** exits non-zero on its own pattern-fidelity
gate: paired in-lobe rms is 3.36 dB against a 3.00 dB tolerance. This is deliberate
and predates the audit. The gate is not relaxed and the manuscript reports it as
failing rather than moving the bar. Its other two gates pass, one of them only
after this audit replaced a per-sample spread bound with a test on the pinned mean.

Worth carrying into any re-run: the corrected spatial channel moves this figure.
Mean residual drops from 3.48 to 1.62 dB while rms drops only from 3.98 to 3.36, so
the bias-removed rms the manuscript quotes goes from about 2.09 dB to about 2.94 dB.

**Fixed since.** `ntn-cho-leo-basic` aborted on its own default trigger, first for
want of a time-to-exit estimator and then on a beam id the GEO pattern grid does
not define; it now enables the analytic TR 38.811 6.4.1 beam like the other three
CHO examples and runs on all its triggers. `oran-ntn-e2-termination` is a
two-process demo that defaulted to the listener role, so standalone it waited for a
peer nobody started; a self-contained `role=both` now runs the agent on a thread
against its own listener and completes the real exchange over loopback SCTP.

**What a paper may claim.** Results from the examples that run. A figure sourced
from the array-gain calibration must carry its failing gate.

## A9 — Half-duplex collisions are dropped and counted, not fatal

**What was bounded.** Scenarios on the 5G-LENA `nr` backend aborted above a UE
count that depended on the scenario: `ntn-real-stack-smoke` at 30,
`ntn-cho-full-constellation` at 8. Two separate causes, both now closed.

**Capacity, closed.** `NrGnbRrc::SrsPeriodicity` defaults to 40 and the toolkit
never set it, so once SRS configuration indices ran out
`DoAllocateTemporaryCellRnti` returned 0 and that refusal was not honoured
downstream: several refused UEs collided on RNTI 0 and a UE received two
random-access responses matching its own preamble. A probe caught it exactly, IMSI
22 processing preamble 40 twice at one instant. The helper sizes the periodicity to
the UE count now, never below the default of 40, and the spine carries 100 UEs.

**Half duplex, closed.** `NrSpectrumPhy` called `NS_FATAL_ERROR` whenever a TDD
node was asked to transmit while receiving, or to receive an SRS while
transmitting. That is not a physical impossibility; it is the absence of a
scheduling rule. A real half-duplex radio simply does not perform the
transmission, and TS 38.213 defines the prioritisation. Three sites now drop and
count instead of aborting.

Reached over NTN because the cell-specific K_offset widens the gap between an
uplink grant and the transmission it authorises from about 2 slots to about 13 at
780 km and 30 kHz SCS, and the vendored scheduler does not reserve the target slot
when it issues the grant. Terrestrial spacing hides this.

**The drops are reported, so they can never be silent.** `sim_health.csv` carries
`halfduplex_ulctrl_drops`, `halfduplex_data_drops` and `halfduplex_srs_rx_drops`
on every `nr` run. Measured on `ntn-cho-full-constellation` over 20 s:

| UEs | transport blocks | ctrl drops | data drops | SRS drops | share |
|---:|---:|---:|---:|---:|---:|
| 4 | 33,562 | 0 | 0 | 0 | 0 |
| 8 | 143,979 | 4 | 0 | 0 | 0.003% |
| 16 | 390,306 | 16 | 8 | 0 | 0.006% |
| 30 | 709,031 | 2 | 0 | 0 | 0.0003% |

The count is not monotonic in load, which is expected: a collision needs a grant
and a downlink to land on the same slot for the same UE, and that depends on the
scheduling pattern rather than simply on how busy the cell is.

30 UEs is the configuration the Monte Carlo campaign under `papers/sim_runs/` uses,
so that campaign is runnable on this example again. Read its results knowing the
cell is heavily loaded at that size: mean DL SINR over 20 s falls 28.45, 23.77,
10.76, 2.61 dB across 4, 8, 16 and 30 UEs on one beam, while aggregate throughput
rises 20.0, 37.9, 48.9, 54.0 Mbps. That is ordinary multi-user sharing, not an
artefact of the drops, which are four orders of magnitude too rare to explain it.

**The patch is inert where the fault does not occur.** `ntn-real-stack-smoke` at
60 s with 4 UEs still gives `dl_sinr_db` 27.8332, bit-identical to before, with all
three counters at zero. No committed measurement moves unless the run was
previously aborting.

**What a paper may claim.** Results at these UE counts, reading the drop counters
alongside `phy_rx_tb`. A large count means the node is over-subscribed and its
throughput figure means something different; single-digit counts against hundreds
of thousands of transport blocks do not change a result.

**What remains upstream.** Teaching the scheduler to reserve the slot at grant
time, so the collision does not arise at all. `NtnRealStackHelper::SetTddPattern()`
offers an explicit DL/UL pattern as a partial mitigation, deliberately not the
default because the slot pattern changes how every scenario schedules.

## A8 — Most shipped scenarios propagate with Kepler + J2, not SGP4

**What is bounded.** `Sgp4MobilityModel` has two propagation paths and picks one
from how it was initialised. Given a two-line element it runs the full Vallado
SGP4, drag and B* included. Given Keplerian elements it runs a Kepler propagator
with J2 secular rates instead, because a generated shell has no TLE for SGP4 to
consume.

**Why it matters.** The Walker-Delta and Walker-Star generators seed shells from
orbital elements, so scenarios built on them are on the Kepler + J2 path. Counted
over the examples that instantiate the model, **4 of 78 run SGP4**; the rest are
analytic. The class name says SGP4 and the captions used to as well, which is the
part that was wrong. Both propagators are real orbital mechanics and neither is a
placeholder, but they are not the same model and should not be described as one.

**Effect on results.** Measured over a 45-minute horizon on an ISS-class TLE the
two separate by roughly 5 to 11 km, which at orbital speed is about a second of
along-track lag. That is enough to shift an argmax-elevation crossover, so
handover instants move; it is far too small to matter for a link budget.

**What a paper may claim.** That the toolkit propagates real TLEs with Vallado
SGP4, if the scenario loads a TLE. For a generated Walker shell, say Kepler with
J2 secular rates. `Sgp4MobilityModel::GetPropagatorName()` returns
`sgp4-vallado` or `kepler-j2` so a run can state which it used, and
`GetSgp4FallbackCount()` is non-zero if the SGP4 path failed mid-run and
degraded silently, which the optimized build otherwise would not report.

**What closing it would require.** Synthesising a conforming TLE from the
generated elements and feeding SGP4, which the constellation README already
claims the presets can do. That is a real change to the geometry every scenario
sees, so it belongs behind a flag and a re-run, not a silent default flip.

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

> **Update 2026-09-01.** A frequency-selective tapped-delay-line now reaches a
> spectrum channel. `NtnTdlSpectrumLossModel` had always computed a per-subcarrier
> transfer function, but it is a plain Object rather than a
> `SpectrumPropagationLossModel`, so nothing could attach it and its `ApplyTo()`
> had two callers in the whole tree, both inside its own unit test.
> `NtnTdlSpectrumPropagationLossModel` is the adapter, and a scenario opts in
> through the helper's existing `AddSpectrumChannelLoss()`. Measured across
> 20 MHz at S band: 2.34 dB of variation with mean gain 1.024, against 0 dB for
> a flat model, and the regression test fails if the shaping is replaced by a
> flat scale.
>
> **This does not make the claim below claimable.** The tap table is a documented
> stand-in, not TR 38.811 Table 6.9.2-1 through -4, because those values could
> not be verified here and attributing invented taps to a 3GPP table is the exact
> defect this audit found in the THz gaseous model. A paper may claim a
> frequency-selective channel with a stated delay spread. It still may not claim
> "NTN-TDL".

## A2 — THz antenna / pointing / beam physics is offline, not in the measured packet path

- **Bounded:** the (genuinely strong) THz array-factor, beamforming codebook, and
  pointing-error models live in the standalone `ThzNtnLinkBudget` calculator. The
  `PropagationLossModel` chained into the *measured* `thz-ntn-*-traffic` examples adds
  gaseous + rain + fog + snow.

  **Corrected 2026-09-01: pointing loss now DOES reach the measured KPIs**, and this
  entry said otherwise. `ThzNtnPointingLossModel` is chained into the measured path
  by `thz-ntn-weather-traffic`, `thz-ntn-ris-relay-traffic`,
  `thz-ntn-isac-coexist-traffic` and `thz-ntn-leo-ground-downlink-traffic`, and
  `thz-ntn-beam-tracking` applies a live tracker-derived pointing loss through
  `NtnStaticExtraLossModel`, driving it to 200 dB on beam failure and to the
  computed mispoint loss otherwise. Five examples, not zero.

  What remains outside the measured path is the array factor and beamforming
  codebook in the standalone `ThzNtnLinkBudget` calculator, and **beam squint**
  across the 10-20 GHz band, which no chained model represents.
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
  rather than carried on BCCH/PDSCH to a decoding UE.

  **Two corrections, 2026-09-01.** This used to end "K_offset is stored metadata
  applied to no scheduling decision", which is now false on the `nr` backend: RRC-1
  wired the broadcast `cellSpecificKoffset` through to `NrGnbPhy::N2Delay`, so the
  offset does change how the network schedules, and it changes it enough to have
  exposed the half-duplex fault in A9. It remains true on the mmwave backend, which
  is what the heading scopes this entry to, and the distinction was missing.

  DRX likewise gates something real now: `ntn-rrc-drx-data-traffic` drives
  `SetTransmitEnabled()` from `IsAwake()`, so the awake fraction starts and stops an
  actual flow. It still does not gate PDCCH monitoring on the UE MAC, which is what
  the claim below is about.
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
  without changing HO/PRB/beam unless the SCENARIO wires it. Updated 2026-09-01:
  the framework hook is `OranNtnE2Interface::SetRcActionCallback`, and **two of the
  eleven** oran-ntn examples install one. `oran-ntn-ric-controlled-traffic` drives
  a beam parameter, and `oran-ntn-gym-handover-example` drives
  `NtnRealStackHelper::TriggerHandover`, so a handover half of this boundary is
  now genuinely closed in that scenario. This entry previously named the beam
  example as the single closed loop, which understated it.
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
- **SUPERSEDED 2026-09-01.** The note below describes `NtnNrStackHelper`, a second
  spine that no longer exists. It was written when `NtnRealStackHelper` was
  mmwave-only, and by the time it was deleted it carried none of the audit fixes:
  no SRS periodicity sizing, no live X2 delay, no TR 38.811 excess-loss chain and
  no health record. FR1 now comes from the one spine, via
  `NtnRealStackHelper::SetRadioBackend(RadioBackend::Nr)` plus `SetNumerology()`,
  which is what most examples already use. The measured figures below stand; they
  were produced by an equivalent FR1 configuration.

  This also dates the "may not claim" line above: it is scoped *on the mmwave
  spine*, and on the `nr` backend a run genuinely is FR1 numerology at S band.
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

**Retested 2026-09-01, and the boundary stands.** The `Cannot TX while RX` aborts
in that table are the same fault as A9, and A9's fix converts three of them into
counted drops. That was enough for `ntn-cho-full-constellation` and not enough
here. With the air-interface delay on, the abort moves to a fourth site and then
to its mirror, `Cannot RX UL CTRL while TX`, which is deliberately NOT patched:
past that point the state machine is being forced rather than corrected.

The 1200 km run is the reason to stop. It completes, exit code 0, and reports
`dl_sinr_db` 0, `app_owd_ms` 0 and **606 dropped downlink control messages**. A
hollow run that looks like a success. It is visible only because A9 added the drop
counters, which is the case for having them, and it is exactly why forcing the
remaining sites would be the wrong kind of fix: it would turn a crash into a
plausible-looking result.

**Fifth attempt, 2026-09-01, and this one is the decisive evidence.** The
receive-side mirror was patched too: the two `case TX:` arms in `StartRxUlCtrl` and
the SRS reception path, where a transmitting half-duplex node is asked to receive.
That is as physically defensible as the transmit-side drops, and the arms that
concern simultaneous receptions were deliberately left alone, since the function's
own comment notes a gNB can receive from several UEs at once.

With it, 300 km finally carries traffic: 34,370 transport blocks at 36.77 dB. And
the application one-way delay is **7779 ms** on a link whose true delay is about
1.2 ms, with 1086 dropped receptions corrupting it. 600 km and 1200 km still carry
nothing at all.

So the patch turns an honest abort into a run that reports a healthy-looking SINR
alongside a delay wrong by three orders of magnitude. That is worse than the crash
it replaces, and it is the reason the change was reverted rather than kept. The
`app_owd_ms` row is what exposed it, which is the argument for provenance rows on
quantities nobody expects to have to check.

`ntn-real-stack-smoke --airDelay=1 --altKm=<km>` reproduces all of this, so the
boundary is checkable rather than asserted. Five attempts, all measured, all
recorded. A6 stands.

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
