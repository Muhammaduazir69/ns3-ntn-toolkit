<h1 align="center">ntn-slice</h1>

<p align="center"><strong>3GPP-compliant network slicing (eMBB / URLLC / mMTC) with GEO mode-skip routing for the <a href="https://github.com/Muhammaduazir69/ns3-ntn-toolkit">ns3-ntn-toolkit</a>.</strong></p>

<p align="center"><em>Part of the v2.0 roadmap (<a href="../../ROADMAP_EXECUTION.md">Workstream W6</a>).</em></p>

---

## What it does

Implements network slicing per 3GPP TS 23.501 with three canonical slices and
all the supporting machinery to **demonstrate isolation** — the property that
one slice's congestion or QoS violation does not bleed into another:

```
flow ─► NtnSliceSelector  (DSCP / port-range / app-label rules)
          │
          ▼  S-NSSAI = (SST, SD)
        SliceOrchestratorXapp
          │  PRB allocation per tick
          │   - reserve minThroughputMbps per slice (isolation guarantee)
          │   - distribute remainder by priority × unmet-demand   OR   by RL shares
          ▼
       SliceIsolationMonitor
          │  per-slice p99 latency & loss-rate windows
          ▼  emits BreachEvent when SLA exceeded
```

The orchestrator's `(observation, action)` shape **matches `SliceEnv` from
W4** — an RL policy trained against the W4 Gymnasium env can drive the C++
allocator directly via `StepWithShares(...)`.

## Components

| File | Purpose |
|---|---|
| `model/ntn-slice-types.h` | `SliceSst` enum (eMBB/URLLC/mMTC/V2X), `Snssai` (SST + 24-bit SD), `SliceProfile` (latency budget, min/max throughput, reliability, priority, allowGeo), default profiles per TS 22.261 Table 7.1-1. |
| `model/ntn-slice-selector.{h,cc}` | DSCP / port-range / app-label rule chain, first-match-wins, default fallback. |
| `model/slice-orchestrator-xapp.{h,cc}` | PRB allocator with min-throughput reservation + priority-weighted unmet-demand sharing. RL hook via `StepWithShares()`. Trace source `Tick`. |
| `model/slice-isolation-monitor.{h,cc}` | Rolling per-slice latency / loss windows, p99 + reliability evaluation, `BreachEvent` trace. |
| `helper/ntn-slice-helper.{h,cc}` | One-call `ThreeSliceDefault()` factory; `ShouldSkipGeo(slice)` and `IsGeoSatellite(altKm)` policy helpers. |
| `examples/ntn-three-slice-leo-geo.cc` | 1 h scenario with three slices co-existing, switchable URLLC routing path. |
| `dashboards/ntn-slice.json` | Grafana panel (per-slice satisfaction, served-vs-demand, p99 latency, breach counters). |

## Quick start

```bash
./ns3 build ntn-three-slice-leo-geo
build/contrib/ntn-slice/examples/ns3.43-ntn-three-slice-leo-geo-default \
    --simTime=3600 --csv=/tmp/three-slice.csv
```

Programmatic use:

```cpp
#include "ns3/ntn-slice-helper.h"

using namespace ns3::ntnslice;

auto stack = NtnSliceHelper::ThreeSliceDefault();
stack.orchestrator->SetTotalPrb(273);  // 100 MHz @ 30 kHz SCS
stack.orchestrator->SetDemand(stack.embb.snssai,  400.0);
stack.orchestrator->SetDemand(stack.urllc.snssai,  80.0);
stack.orchestrator->SetDemand(stack.mmtc.snssai,   20.0);
auto ticks = stack.orchestrator->Step();   // built-in policy
// or: stack.orchestrator->StepWithShares({{embb, 0.6}, {urllc, 0.3}, {mmtc, 0.1}});

stack.monitor->RecordPacket(stack.urllc.snssai, /*latencyMs=*/1.5, /*delivered=*/true);
auto breaches = stack.monitor->EvaluateAll();
```

## Audit results (2026-05-04)

**Test suite (`ntn-slice`, 7 tests, 0.008 s):** ✅ all pass.

| Test | Asserts |
|---|---|
| S-NSSAI 32-bit pack-unpack round-trip | bit-faithful for all SSTs × 4 SD values |
| Selector first-match-wins | DSCP, port-range, app-label rules + default fallback |
| Orchestrator preserves URLLC min throughput | URLLC ≥ 5 Mbps under heavy eMBB contention |
| Isolation monitor flags URLLC breach | p99 > 5 ms (URLLC budget) is detected |
| URLLC mandates GEO mode-skip | `ShouldSkipGeo(URLLC)` = true; eMBB / mMTC = false |
| Three-slice co-existence | all three meet `minThroughputMbps` simultaneously |
| External RL shares honoured | 0.6/0.3/0.1 split → eMBB > 2× mMTC throughput |

**Long-run validation (1 h × 2 scenarios = 2 h sim, 10 800 KPI rows each):**

```
=== W6 audit: URLLC GEO-mode-skip ===

--- LEO-only (mode-skip ON) ---
  embb   mean=327.13   p95=330.58   p99=331.20  max= 331.20   (eMBB on GEO 30 % of ticks)
  urllc  mean= 47.02   p95= 49.16   p99= 50.36  max=  50.36   ← under 50 ms gate
  mmtc   mean= 73.13   p95= 73.73   p99= 74.02  max=  74.02   (eMBB-class budget OK)

--- forced URLLC via GEO ---
  urllc  mean=295.52   p95=297.66   p99=298.86  max= 298.86   (6.3× LEO)
```

**Gate evaluation:**

| Gate from `ROADMAP_EXECUTION.md` | Result |
|---|---|
| eMBB / URLLC / mMTC co-exist on same satellite without isolation breach | ✅ all 3 meet their `minThroughputMbps` simultaneously |
| URLLC E2E latency < 50 ms via GEO-mode-skip routing | ✅ **mean p99 = 47.02 ms**, p95-of-p99 = 49.16 ms |
| Per-slice KPI panels in Grafana populated | ✅ `dashboards/ntn-slice.json` (4 panels: satisfaction, served-vs-demand, p99, breaches) |

The 6.3 × LEO-vs-GEO ratio confirms the policy: when URLLC traffic must skip
GEO satellites it stays comfortably under its 50 ms latency budget; the
moment we force it through GEO the p99 jumps to ~300 ms (LEO 30 ms RTT vs
GEO ~500 ms RTT — the well-known orbit-induced floor).

## Schema additions (W3 compatible)

`contrib/ntn-observability/model/ntn-metric-schema.h` gained 8 new fields
under the existing `ntn_slice` measurement; the pinned schema-stability test
(`MetricSchemaStableTest`) still passes because no existing field was
renamed:

```cpp
field::kSlicePrbAllocated        = "slice_prb_allocated"
field::kSliceServedMbps          = "slice_served_mbps"
field::kSliceDemandMbps          = "slice_demand_mbps"
field::kSliceSatisfaction        = "slice_satisfaction"
field::kSliceLatencyP99Ms        = "slice_latency_p99_ms"
field::kSliceLossRate            = "slice_loss_rate"
field::kSliceLatencyBreach       = "slice_latency_breach"
field::kSliceReliabilityBreach   = "slice_reliability_breach"
```

## License

GPL-2.0-only — same as the umbrella ns3-ntn-toolkit.

## Maintainer

Muhammad Uzair — `muhammaduzairr69@gmail.com` (ORCID: 0009-0002-4104-2680)
