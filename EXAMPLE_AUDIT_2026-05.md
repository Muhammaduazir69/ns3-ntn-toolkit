# Custom-Module Example Audit — May 2026

Goal (from the realism roadmap brief): **every custom-module example must
execute perfectly under real `Simulator::Run()` sim-time, and every example
that is supposed to move data must show real packet transmission between
network nodes.** Secondary goal: contrib modules should compose each other's
APIs / helper classes rather than living in isolation.

## Method

Each built example binary under `build/contrib/<module>/examples/` was run with
a short, safe argument set (no-args first, then a single time flag tried
individually to avoid the ns-3 `CommandLine` help-exit false-negative). Each run
was classified:

- **DATA** — exits 0 **and** emits packet/goodput/delivery evidence on stdout
  (`sim_health` gates, `Mbps`, `deliveredKB`, `delivered`, `goodput`, `PDR`,
  `throughput`).
- **RUNS** — exits 0; physics/geometry-only by design (no data plane expected),
  or emits its data evidence to a file rather than stdout.
- **FAIL** — crash / nonzero exit / timeout.

## Result

| Status | Count |
|--------|-------|
| DATA   | 46    |
| RUNS   | 11    |
| FAIL   | **0** |
| total  | 57    |

All 57 custom-module examples execute cleanly under real sim-time. No crashes,
no nonzero exits, no hangs.

### DATA examples (46) — real packet transmission confirmed on stdout

`ntn-cho` (3), `ntn-constellation` (3), `ntn-fapi` (2), `ntn-observability` (2),
`ntn-rrc` (4), `ntn-sagin` (10), `ntn-sionna` (6), `ntn-slice` (2),
`ntn-v2x` (2), `oran-ntn` (2), `thz-ntn` (10). Each writes either a
`sim_health.csv` whose realism gates (`packets_tx`, `rx_over_tx ≥ 0.85`,
`tx_per_sim_sec ≥ 100`) all PASS, or an explicit goodput / delivered-bytes /
Mbps / PDR figure.

### RUNS examples (11) — verified intentional, not broken

| Example | Why RUNS (not DATA) |
|---------|---------------------|
| `ntn-traffic/nrtv-p2p-example` | **Transmits** real NRTV video over P2P (764 Rx data points, up to 1221 B); evidence lands in `NRTV-UDP-client-trace.plt`, not stdout. |
| `ntn-traffic/nrtv-variables-plot` | Samples the NRTV traffic-model RNG and plots its variables; no network by design. |
| `ntn-cho/ntn-realistic-mobility-demo` | Mobility/geometry demonstration. |
| `ntn-sionna/city-block-4ue-cache` | Ray-traced channel + cache physics. |
| `ntn-sionna/leo-pass-sionna-vs-tr38811` | Channel-model comparison. |
| `ntn-sionna/mmimo-vs-codebook-leo` | Beamforming comparison. |
| `ntn-sionna/ris-assisted-leo-link` | RIS link-budget physics. |
| `ntn-sionna/sionna-calibration-harness` | Calibration harness. |
| `thz-ntn/thz-ntn-dband-constellation` | D-band geometry/link budget. |
| `thz-ntn/thz-ntn-ris-assisted` | THz-RIS physics. |
| `thz-ntn/thz-ntn-um-mimo` | Ultra-massive MIMO physics. |

The physics/geometry examples legitimately have no data plane — they compute
and report channel/geometry quantities, which is their purpose.

## Gaps found and fixed in this pass

1. **`ntn-rrc-from-tle` required `--tle` and exited 2 with no args.** Bundled a
   default ISS TLE (`contrib/ntn-rrc/data/iss-zarya.tle`), added a path search
   and a default start UTC so it now runs with zero args (10009 packets / 15 s,
   all `sim_health` gates PASS). — commit `6bc51ba9b`

2. **Cross-module composition gap: `ntn-constellation` examples used no other
   contrib module.** Added two examples composing the `ntn-traffic`
   helper:
   - `ntn-constellation-walker-traffic` — Walker-Delta constellation +
     `ContactGraphScheduler` (live ISL up/down) + real UDP data plane
     (13676 pkt tx/rx, ISL up=10).
   - `ntn-constellation-sgp4-mobility-traffic` — single SGP4 LEO + GSL
     up/down. The ground station now auto-places under the satellite's t=0
     sub-point, so a single LEO always produces a demonstrative GSL pass
     (GSL up=1 / down=1, 331 k packets). — commit `b91f32756`

3. **Cross-module composition gap: `ntn-fapi` example used no other contrib
   module.** Added `ntn-fapi-leo-pass-slotloop` — the SCF-222 FAPI L1↔L2 data
   ABI driven by a **real SGP4 pass** (`ntn-constellation`): per-slot SINR is
   derived from the live elevation, so delivered FAPI goodput tracks genuine
   orbital geometry (1.2 M slots, HARQ 645 k ok / 555 k retx, 967 MB,
   12.9 Mbps; out-of-contact slots correctly fall to BLER=1.0). — commit
   `344cbd9ee`

## Remaining (non-actionable) cross-module notes

- **`ntn-traffic`** is the base helper provider every other module composes; it
  is correct for it to be self-contained.
- **`ns3-ai-ntn`** ships no C++ examples — it is a Python/C++ AI bridge whose
  examples live on the Python side.

## Bottom line

Every custom-module example runs to completion under real sim-time with the
correct data-plane behaviour for its category, and the previously-isolated
`ntn-constellation` and `ntn-fapi` modules now compose sibling contrib APIs.
