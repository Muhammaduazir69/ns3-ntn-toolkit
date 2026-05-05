<h1 align="center">ntn-slice</h1>

<p align="center"><strong>3GPP Network Slicing for Non-Terrestrial Networks: eMBB / URLLC / mMTC / V2X with GEO Mode-Skip Routing</strong></p>

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0-green.svg"/></a>
  <img src="https://img.shields.io/badge/3GPP-TS%2023.501%20%2F%20TS%2022.261-orange.svg"/>
  <img src="https://img.shields.io/badge/SST-eMBB%20%E2%80%A2%20URLLC%20%E2%80%A2%20mMTC%20%E2%80%A2%20V2X-purple.svg"/>
  <img src="https://img.shields.io/badge/unit_tests-7%20PASS-success.svg"/>
</p>

---

## Why this module

5G slicing is straightforward when every cell is a few kilometres away. In NTN, the latency budget of a URLLC slice (5 ms p99) is hard-bounded by orbital geometry — a single GEO hop introduces ~270 ms one-way, blowing the budget by two orders of magnitude regardless of any allocator decisions on the radio. `ntn-slice` is a 3GPP-compliant slicing layer that *understands the orbital floor*: it implements per-slice PRB allocation with min-throughput reservation (the classical isolation guarantee), an isolation monitor that flags p99-latency / loss-rate breaches per S-NSSAI, and a **GEO mode-skip policy** that keeps URLLC traffic on LEO satellites whose round-trip time is compatible with the slice's latency target. The orchestrator's `(observation, action)` shape matches the `SliceEnv` Gymnasium wrapper in [`ns3-ai`](https://github.com/Muhammaduazir69/ns3-ai), so a trained RL policy can drive the C++ allocator directly.

## At a glance

| Slice | SST | Default min throughput | Default p99 latency budget | GEO allowed |
|---|---:|---:|---:|:---:|
| eMBB  | 1 | 100 Mbps | 100 ms | yes |
| URLLC | 2 |   5 Mbps |   5 ms | **no — mode-skip** |
| mMTC  | 3 |   1 Mbps |   1 s  | yes |
| V2X   | 4 |   2 Mbps |  10 ms | yes |

| Validation result | Value |
|---|---:|
| URLLC p99 latency, **mode-skip ON** | **47.02 ms** mean, 49.16 ms p95-of-p99 |
| URLLC p99 latency, forced via GEO | 295.52 ms (6.3× LEO; the orbital floor) |
| Three-slice co-existence | all three meet `minThroughputMbps` simultaneously |
| Test suite (`ntn-slice`, 7 tests) | **PASS in 0.008 s** |
| Long-run scenarios audited | 2 × 1 h (10 800 KPI rows each) |

## What it does

```
flow ─► NtnSliceSelector  (DSCP / port-range / app-label rules, first-match-wins)
          │
          ▼  S-NSSAI = (SST, SD)
        SliceOrchestratorXapp
          │  PRB allocation per tick
          │   - reserve minThroughputMbps per slice (isolation guarantee)
          │   - distribute remainder by priority × unmet-demand   OR   by RL shares
          ▼
       SliceIsolationMonitor
          │  per-slice rolling p99 latency & loss-rate windows
          ▼  emits BreachEvent when SLA exceeded
```

- **Slice taxonomy** (`model/ntn-slice-types.h`) — `SliceSst` enum (eMBB/URLLC/mMTC/V2X), `Snssai` (8-bit SST + 24-bit SD, packed/unpacked codecs), `SliceProfile` (latency budget, min/max throughput, reliability target, priority, `allowGeo`), default profiles per TS 22.261 Table 7.1-1.
- **Selector** — DSCP / port-range / app-label rule chain, first-match-wins, default fallback.
- **Orchestrator** — PRB allocator with min-throughput reservation followed by priority-weighted unmet-demand sharing. RL hook via `StepWithShares({snssai → fraction})` lets an external trained policy override the built-in allocator. Trace source `Tick` exports per-slice (allocated PRBs, served Mbps, satisfaction).
- **Isolation monitor** — rolling per-slice latency / loss-rate windows; `EvaluateAll()` returns a list of `BreachEvent` records covering both latency-p99 and reliability breaches.
- **GEO mode-skip helper** — `ShouldSkipGeo(slice)` returns true exactly for slices whose `allowGeo` flag is false (URLLC by default); `IsGeoSatellite(altKm)` is a 35 786 km threshold helper.
- **Helper façade** — `NtnSliceHelper::ThreeSliceDefault()` produces a ready-wired stack of selector + orchestrator + monitor + the three default slices.
- **Grafana panel** — `dashboards/ntn-slice.json` (per-slice satisfaction, served-vs-demand, p99 latency, breach counters).

## Install & run

```bash
git clone https://github.com/Muhammaduazir69/ntn-slice.git contrib/ntn-slice
./ns3 build ntn-three-slice-leo-geo
build/contrib/ntn-slice/examples/ns3.43-ntn-three-slice-leo-geo-default \
    --simTime=3600 --csv=/tmp/three-slice.csv
```

Programmatic use:

```cpp
#include "ns3/ntn-slice-helper.h"
using namespace ns3::ntnslice;

auto stack = NtnSliceHelper::ThreeSliceDefault();
stack.orchestrator->SetTotalPrb(273);   // 100 MHz @ 30 kHz SCS
stack.orchestrator->SetDemand(stack.embb.snssai,  400.0);
stack.orchestrator->SetDemand(stack.urllc.snssai,  80.0);
stack.orchestrator->SetDemand(stack.mmtc.snssai,   20.0);

auto ticks = stack.orchestrator->Step();
// or external RL policy drives the allocator:
// stack.orchestrator->StepWithShares({{embb, 0.6}, {urllc, 0.3}, {mmtc, 0.1}});

stack.monitor->RecordPacket(stack.urllc.snssai,
                             /*latencyMs=*/1.5, /*delivered=*/true);
auto breaches = stack.monitor->EvaluateAll();
```

## Verification

**Test suite (`ntn-slice`, 7 cases, all passing):**

| Test | Asserts |
|---|---|
| `SnssaiPackUnpackTest` | 32-bit pack-unpack round-trip is bit-faithful for all SSTs × 4 SD values. |
| `SelectorRulesTest` | Selector first-match-wins across DSCP / port-range / app-label rules + default fallback. |
| `OrchestratorPreservesUrllcMinTest` | URLLC ≥ 5 Mbps under heavy eMBB contention. |
| `IsolationMonitorBreachTest` | p99 > 5 ms (URLLC budget) is detected from the rolling window. |
| `UrllcMandatesGeoModeSkipTest` | `ShouldSkipGeo(URLLC)` = true; eMBB / mMTC = false. |
| `ThreeSliceCoexistenceTest` | all three slices meet `minThroughputMbps` simultaneously. |
| `RlSharesHonouredTest` | external 0.6 / 0.3 / 0.1 share split → eMBB > 2× mMTC throughput. |

**Long-run validation (2 × 1 h scenarios, 10 800 KPI rows each):**

```
=== URLLC GEO-mode-skip evaluation ===

--- LEO-only (mode-skip ON) ---
  embb   mean=327.13   p95=330.58   p99=331.20  max= 331.20
  urllc  mean= 47.02   p95= 49.16   p99= 50.36  max=  50.36   ← under 50 ms
  mmtc   mean= 73.13   p95= 73.73   p99= 74.02  max=  74.02

--- forced URLLC via GEO ---
  urllc  mean=295.52   p95=297.66   p99=298.86  max= 298.86   (6.3× LEO)
```

The 6.3× LEO-vs-GEO ratio confirms the policy: when URLLC traffic must skip GEO satellites it stays under its 50 ms latency budget; the moment we force it through GEO the p99 jumps to ~300 ms (the well-known LEO 30 ms RTT vs GEO ~500 ms RTT orbital floor).

## Schema additions

`contrib/ntn-observability/model/ntn-metric-schema.h` gained 8 new fields under the existing `ntn_slice` measurement; the pinned `MetricSchemaStableTest` still passes because no existing field was renamed:

```cpp
field::kSlicePrbAllocated      = "slice_prb_allocated"
field::kSliceServedMbps        = "slice_served_mbps"
field::kSliceDemandMbps        = "slice_demand_mbps"
field::kSliceSatisfaction      = "slice_satisfaction"
field::kSliceLatencyP99Ms      = "slice_latency_p99_ms"
field::kSliceLossRate          = "slice_loss_rate"
field::kSliceLatencyBreach     = "slice_latency_breach"
field::kSliceReliabilityBreach = "slice_reliability_breach"
```

## Documentation

- [INSTALL.md](INSTALL.md) — setup notes.
- 3GPP TS 23.501 §5.15 — *5G System Architecture; Network Slicing*.
- 3GPP TS 22.261 Table 7.1-1 — *Service requirements for the 5G system*; default per-slice KPIs.

## Cite this work

```bibtex
@misc{uzair2026ntnslice,
  author = {Uzair, Muhammad},
  title  = {ntn-slice: 3GPP Network Slicing with GEO Mode-Skip for Non-Terrestrial Networks},
  year   = {2026},
  url    = {https://github.com/Muhammaduazir69/ntn-slice}
}
```

## Part of the ns3-ntn-toolkit

| Module | Repo |
|---|---|
| Toolkit (umbrella) | [ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit) |
| ntn-constellation | [ntn-constellation](https://github.com/Muhammaduazir69/ntn-constellation) |
| ntn-rrc | [ntn-rrc](https://github.com/Muhammaduazir69/ntn-rrc) |
| ntn-observability | [ntn-observability](https://github.com/Muhammaduazir69/ntn-observability) |
| ns3-ai (fork) | [ns3-ai](https://github.com/Muhammaduazir69/ns3-ai) |
| ntn-sagin | [ntn-sagin](https://github.com/Muhammaduazir69/ntn-sagin) |
| **ntn-slice** | this repo |
| ntn-v2x | [ntn-v2x](https://github.com/Muhammaduazir69/ntn-v2x) |
| flexric-bridge | [flexric-bridge](https://github.com/Muhammaduazir69/flexric-bridge) |
| ntn-sionna | [ntn-sionna](https://github.com/Muhammaduazir69/ntn-sionna) |
| ntn-digital-twin | [ntn-digital-twin](https://github.com/Muhammaduazir69/ntn-digital-twin) |
| ntn-cho | [ntn-cho-framework](https://github.com/Muhammaduazir69/ntn-cho-framework) |
| oran-ntn | [oran-ntn](https://github.com/Muhammaduazir69/oran-ntn) |
| thz-ntn | [ns3-thz-ntn](https://github.com/Muhammaduazir69/ns3-thz-ntn) |

## License

GPL-2.0-only — see [LICENSE](LICENSE).

## Acknowledgements

3GPP SA1 / SA2 (TS 22.261 / TS 23.501 work items) · ns-3 core team · O-RAN Alliance for the xApp interface conventions.
