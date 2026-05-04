<h1 align="center">ntn-sagin</h1>

<p align="center"><strong>Space-Air-Ground Integrated Network (SAGIN) layer for the <a href="https://github.com/Muhammaduazir69/ns3-ntn-toolkit">ns3-ntn-toolkit</a>: HAPS, UAV mobility, 3GPP TR 36.777 A2G channel, multi-layer router.</strong></p>

<p align="center"><em>Part of the v2.0 roadmap (<a href="../../ROADMAP_EXECUTION.md">Workstream W5</a>).</em></p>

---

## What it does

Extends the toolkit beyond satellite-only into the **air layer** (HAPS, UAVs,
commercial flights) and ties everything together with a greedy multi-layer
router that picks Ground → UAV → HAPS → LEO hops by maximum elevation:

```
ground UE ─► UAV-relay ─► HAPS (20 km) ─► LEO (550 km)
            ▲ TR 36.777 A2G channel ▲ free-space (LEO)
```

## Components

| File | Purpose |
|---|---|
| `model/haps-mobility-model.{h,cc}` | 20 km station-keeping with figure-8 racetrack; vertical envelope ±50 m by tanh-clamp. |
| `model/uav-mobility-models.{h,cc}` | Three UAV mobility patterns: random-waypoint (`UavWaypoint…`), patrol (`UavPatrol…`), lawnmower search (`UavSearchPattern…`). |
| `model/a2g-channel-tr36777.{h,cc}` | Closed-form path loss for UMa-AV / RMa-AV / UMi-AV (LOS + NLOS) per 3GPP TR 36.777 v15.0.0 table 6.2-1; LOS-probability per §7.6.3.1. |
| `model/multi-layer-router.{h,cc}` | Greedy Ground→UAV→HAPS→LEO routing by max-elevation per layer. |
| `model/aeronautical-scenario.{h,cc}` | Commercial-flight cruise mobility (constant altitude, constant ground speed). |
| `helper/sagin-helper.{h,cc}` | One-call factory wrapping the above. |
| `examples/sagin-haps-leo-relay.cc` | 1 h SAGIN scenario with full 4-layer path emitted per second. |
| `examples/sagin-uav-swarm.cc` | 8-UAV swarm (mix of patterns) with TR 36.777 PL spot-checks. |
| `examples/sagin-aeronautical.cc` | 1 h commercial flight under LEO satellite. |
| `test/ntn-sagin-test-suite.cc` | 6 unit tests, all green in 0.008 s. |

## Quick start

```bash
./ns3 build sagin-haps-leo-relay sagin-uav-swarm sagin-aeronautical
build/contrib/ntn-sagin/examples/ns3.43-sagin-haps-leo-relay-default \
    --simTime=3600 --csv=/tmp/sagin.csv
```

The CSV contains per-second hop selections:

```
time_s,n_hops,uav_el_deg,uav_range_km,haps_el_deg,haps_range_km,leo_el_deg,leo_range_km
0.000,3,2.870,1.998,75.910,20.517,14.821,2071.934
```

## Programmatic use

```cpp
#include "ns3/sagin-helper.h"

SaginHelper helper;
auto haps = helper.CreateHaps(Vector{0,0,0}, /*alt=*/20000.0, /*radius=*/3000.0);
auto uav  = helper.CreateUavPatrol(Vector{-2000,0,100}, Vector{2000,0,100}, 25.0);
auto leo  = CreateObject<ConstantVelocityMobilityModel>();
leo->SetPosition(Vector{-2.0e6, 0, 550e3});
leo->SetVelocity(Vector{7590, 0, 0});

auto router = helper.CreateRouter();
router->AddNode(SaginLayer::Uav,  uav);
router->AddNode(SaginLayer::Haps, haps);
router->AddNode(SaginLayer::Leo,  leo);

auto path = router->Route(ueMob);  // ordered hops, ground→…→LEO
```

## Audit results (2026-05-04)

| Validation gate | Result |
|---|---|
| Test suite green (`ntn-sagin`, 6 tests) | ✅ **PASS in 0.008 s** |
| HAPS altitude held within ±50 m for 1 h | ✅ (61 samples, no breach) |
| UAV patrol returns to start within 50 m after one round-trip | ✅ |
| TR 36.777 PL spot-checks within ±2 dB of spec | ✅ (RMa-AV / UMa-AV / UMi-AV LOS) |
| LOS probability monotonic in UAV altitude | ✅ |
| Router emits 4-layer path with 50 nodes/layer in <5 s | ✅ (sub-millisecond) |
| Aircraft reaches arrival point within expected ETA | ✅ |

**Long-run smoke (1 h):**

| Example | Lines | Comment |
|---|---:|---|
| `sagin-haps-leo-relay` | 3 601 | every step shows 3-hop path; HAPS elevation tracks figure-8 phase 75.9 → 84.0 deg |
| `sagin-uav-swarm` (10 min, 8 UAVs) | 961 | TR 36.777 RMa-AV PL recovered analytically: 28 + slope·log10(d) + 20·log10(fc) — measured 75.43 dB at d3D=117.6 m, h_UT=152 m matches spec to 0.02 dB |
| `sagin-aeronautical` | 721 | aircraft 250 m/s at 11 km, LEO range 2 072 km → 25 320 km as sat exits horizon |

## Test plan vs. validation gates

| Gate from `ROADMAP_EXECUTION.md` | Result |
|---|---|
| HAPS altitude within ±50 m for 1 h | ✅ enforced by tanh-clamp + verified by `HapsAltitudeStableTest` |
| PL spot-checks within TR 36.777 ±2 dB | ✅ 3 scenarios spot-checked; live demo agrees to 0.02 dB |
| Multi-layer routing converges in <5 s | ✅ sub-ms with 50 nodes/layer (no iteration — closed-form greedy) |

## License

GPL-2.0-only — same as the umbrella ns3-ntn-toolkit.

## Maintainer

Muhammad Uzair — `muhammaduzairr69@gmail.com` (ORCID: 0009-0002-4104-2680)
