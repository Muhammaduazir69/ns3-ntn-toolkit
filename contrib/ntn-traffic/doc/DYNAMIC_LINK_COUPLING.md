# Analysis: static data plane in geometry-driven scenarios (and the fix)

Status: implemented (v2.1 work, Jun 2026)
Scope: `ntn-traffic` helper + `ntn-cho-full-constellation` example
Driver: platform-paper Table 5 showed identical `packets_tx`/RSS across
constellation sizes and for >32 UEs, which correctly reflected the code
but hid that the packet plane was not coupled to orbital geometry.

## 1. Diagnosis (verified against source)

| Symptom | Root cause | Location |
|---|---|---|
| `packets_tx` identical across 66..1584 sats | UDP plane runs over a static P2P link; constellation geometry never touches it | `helper/ntn-realistic-traffic-helper.cc` `Wire()`: `Delay("15ms")` fixed, comment "variable rate / delay later" |
| `packets_tx` identical for 60 and 120 UEs | example caps traffic plane at 32 UEs | `ntn-cho-full-constellation.cc`: `trafficUes = min(numUes, 32)` |
| delivery ratio ~0.986 everywhere | **no error model installed at all**; only queue drops | helper `Wire()` |
| RSS flat at ~49 MB | analytic constellation (by design) + capped traffic plane | both |

The static plane was a deliberate v2.0 bootstrap ("keep the event queue
honest") and is documented as such; the coupling below removes the
limitation while keeping every health gate intact.

## 2. Modification design

### 2.1 Helper API (backward compatible)
- `UpdateUeLink(ueIndex, oneWayDelay, per)`: sets the per-UE P2P channel
  `Delay` attribute and the packet-error rate of `RateErrorModel`s
  installed on both directions of that UE's access link at `Wire()`
  time (rate 0 until first update, so existing users see no change).
- Error-model RNG uses a fixed stream offset per UE for seed
  reproducibility.

### 2.2 Scenario coupling (`ntn-cho-full-constellation`)
Each analytic step (1 s) and for every traffic-carrying UE:
- **Delay**: one-way delay = (service slant + feeder slant)/c + 1 ms
  processing. Feeder leg approximated by the service slant (gateway in
  the same region), so owd in [~6.2, ~15.7] ms at 780 km, consistent
  with 3GPP TR 38.821 LEO transparent-payload budgets.
- **Loss**: serving-SINR-driven PER via logistic map
  `per = 1/(1+exp(0.6*(sinr_dB+14)))`, clamped to [5e-4, 0.95]:
  ~0.1 % at 0 dB, ~1 % at -7 dB, ~8 % at -10 dB. No serving cell ->
  per = 1.0 (outage).
- **Handover interruption**: on execution the link drops (per = 1.0)
  for 30 ms under CHO variants (target pre-prepared, RACH-less
  assumption) and 80 ms under baseline A3, then restores; values follow
  the Rel-17 CHO motivation (interruption cut versus baseline HO) and
  TS 38.821 mobility discussion.
- **Cap removed**: `--trafficUes` flag (0 = all UEs) replaces the fixed
  32-UE cap; default remains "all" so packet counts scale with UEs.

### 2.3 What now varies, and why
- UE count -> TX packets and RSS scale (more apps/sockets).
- Constellation size -> coverage/outage and serving geometry change ->
  delivery ratio and mean one-way delay change (TX stays app-driven,
  which is correct: sources do not stop because the sky changed).
- simTime -> TX/RX scale linearly (continuous OnOff sources).
- Algorithm -> interruption count x window + outage time -> delivery.

## 3. Validation protocol
1. ns-3 test suites: `ntn-traffic`, `ntn-cho` must stay green.
2. Health gates unchanged (tx/sim-s, delivery floor vs new loss model:
   floor checked against tte-aware 66x30 reference run).
3. Cross-check delay against open data (anchors, Jun 2026):
   - TR 38.821 LEO transparent one-way budgets (few..15 ms class);
     our model at 780 km gives owd in [6.2, 15.7] ms.
   - Published Starlink measurements: bent-pipe segment RTT mean
     31-35 ms (sigma 11-12 ms, WetLinks/European probes); regional
     medians 33-40 ms toward PoPs (large-scale IPv6 study, arXiv
     2412.18243). Our access RTT = 2 x owd + 4 ms backbone lands in
     16.4..35.4 ms - inside/below the measured bent-pipe band, as
     expected for a model without wide-area PoP backhaul.
   - Residual-loss curve anchored to NR-NTN low-MCS operation
     (~0.1% at -5 dB, ~0.7% at -10 dB, ~12% at -16 dB serving SINR).
4. Sweep reruns (sats x UEs x simTime) archived under
   `papers/sim_runs/scalability/` with delivery and owd columns.
