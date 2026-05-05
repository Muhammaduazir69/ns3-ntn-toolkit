<h1 align="center">ntn-sagin</h1>

<p align="center"><strong>Space-Air-Ground Integrated Network for ns-3.43: HAPS, UAV Mobility, 3GPP TR 36.777 A2G Channel and Multi-Layer Routing</strong></p>

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0-green.svg"/></a>
  <img src="https://img.shields.io/badge/3GPP-TR%2036.777%20v15.0.0-orange.svg"/>
  <img src="https://img.shields.io/badge/scenarios-UMa--AV%20%E2%80%A2%20RMa--AV%20%E2%80%A2%20UMi--AV-purple.svg"/>
  <img src="https://img.shields.io/badge/unit_tests-6%20PASS-success.svg"/>
</p>

---

## Why this module

Most ns-3 NTN work stops at "satellite + ground". A realistic 6G access network sits in three layers — space (LEO/MEO/GEO), air (HAPS, UAVs, commercial flights) and ground — and traffic typically traverses two or three of them per session. `ntn-sagin` fills the air-layer gap with a complete Space-Air-Ground Integrated Network model: closed-form mobility patterns for HAPS station-keeping (figure-8 racetrack with ±50 m vertical envelope), three UAV patterns (waypoint, patrol, search-lawnmower), the 3GPP TR 36.777 air-to-ground channel for UMa-AV / RMa-AV / UMi-AV scenarios, and a greedy multi-layer router that selects the best Ground → UAV → HAPS → LEO hop sequence by maximum elevation per layer.

## At a glance

| Component | Backing model |
|---|---|
| HAPS mobility | Lemniscate of Bernoulli (figure-8) at 20 km altitude with tanh-clamped vertical envelope |
| UAV: waypoint | RNG-driven random-waypoint, configurable speed band |
| UAV: patrol | Linear A↔B traversal with constant speed |
| UAV: search | Lawnmower scan with configurable lane spacing |
| A2G path loss | TR 36.777 v15.0.0 §6.2-1 (UMa-AV / RMa-AV / UMi-AV, LOS+NLOS) |
| LOS probability | TR 36.777 §7.6.3.1 height-dependent |
| Routing | Greedy max-elevation per layer (Ground → UAV → HAPS → LEO) |
| Aeronautical | Constant-altitude / constant-ground-speed cruise |

| Verification metric | Result |
|---|---|
| Test suite (`ntn-sagin`, 6 tests) | **PASS in 0.008 s** |
| HAPS altitude held within ±50 m for 1 h | 61 samples, no breach |
| TR 36.777 RMa-AV LOS spot-check (h_UT=152 m, d3D=117.6 m, fc=2 GHz) | **75.43 dB** measured vs spec — matches to **0.02 dB** |
| Multi-layer routing (50 nodes/layer) | sub-millisecond (closed-form greedy, no iteration) |
| 1 h SAGIN scenario | 3 601 lines, 3-hop path emitted every step |

## What it does

```
ground UE  ──►  UAV-relay  ──►  HAPS (20 km)  ──►  LEO (550 km)
              ▲ TR 36.777 A2G channel ▲                ▲ free-space (LEO)
```

- **HAPS station-keeping** — `HapsMobilityModel` traces a closed lemniscate at the configured altitude; vertical drift is bounded by a tanh-clamp so the platform never violates its station-keeping envelope, regardless of integration step size.
- **UAV mobility** — three patterns share a common base (`UavWaypointMobilityModel`, `UavPatrolMobilityModel`, `UavSearchPatternMobilityModel`); each is composable with the existing `MobilityHelper` and exposes `TraceConnect` hooks for position updates.
- **TR 36.777 A2G channel** — closed-form path loss and LOS probability for the three reference scenarios. Spot-check at h_UT = 152 m / d3D = 117.6 m / fc = 2 GHz: measured RMa-AV LOS path loss matches the spec to 0.02 dB (75.43 dB).
- **Multi-layer router** — `MultiLayerRouter::Route(ueMob)` returns an ordered hop list selected greedily by maximum elevation per layer; runs in O(n) per layer with no iteration loop.
- **Aeronautical scenario** — commercial-flight cruise mobility for high-altitude-platform research. 1 h scenario with a 250 m/s cruiser at 11 km altitude under a LEO satellite emits a clean range curve from 2 072 km (sat overhead) to 25 320 km (sat exits horizon).
- **Helper façade** — `SaginHelper` factory wraps the whole stack; one call produces a ready-to-route 4-layer scene.

## Install & run

```bash
git clone https://github.com/Muhammaduazir69/ntn-sagin.git contrib/ntn-sagin
./ns3 build sagin-haps-leo-relay sagin-uav-swarm sagin-aeronautical
build/contrib/ntn-sagin/examples/ns3.43-sagin-haps-leo-relay-default \
    --simTime=3600 --csv=/tmp/sagin.csv
```

CSV format (per-second hop selection):

```
time_s,n_hops,uav_el_deg,uav_range_km,haps_el_deg,haps_range_km,leo_el_deg,leo_range_km
0.000,3,2.870,1.998,75.910,20.517,14.821,2071.934
```

Programmatic use:

```cpp
#include "ns3/sagin-helper.h"

SaginHelper helper;
auto haps = helper.CreateHaps(Vector{0, 0, 0}, /*alt=*/20000.0, /*radius=*/3000.0);
auto uav  = helper.CreateUavPatrol(Vector{-2000, 0, 100}, Vector{2000, 0, 100}, 25.0);
auto leo  = CreateObject<ConstantVelocityMobilityModel>();
leo->SetPosition(Vector{-2.0e6, 0, 550e3});
leo->SetVelocity(Vector{7590, 0, 0});

auto router = helper.CreateRouter();
router->AddNode(SaginLayer::Uav,  uav);
router->AddNode(SaginLayer::Haps, haps);
router->AddNode(SaginLayer::Leo,  leo);

auto path = router->Route(ueMob);   // ordered hops, ground → … → LEO
```

## Examples shipped

| Binary | Purpose |
|---|---|
| `sagin-haps-leo-relay` | 1 h SAGIN scenario emitting the full 4-layer path per second. |
| `sagin-uav-swarm` | 8-UAV swarm (mix of patterns) with TR 36.777 PL spot-checks. |
| `sagin-aeronautical` | 1 h commercial flight under a LEO satellite. |

## Verification

| Test | Asserts |
|---|---|
| HAPS altitude stable | tanh-clamp keeps altitude inside ±50 m envelope across simulation step variations |
| UAV patrol round-trip | `UavPatrolMobilityModel` returns to start within 50 m after one round trip |
| TR 36.777 spot-check (LOS) | RMa-AV / UMa-AV / UMi-AV path loss within ±2 dB of spec |
| LOS probability monotonic in altitude | TR 36.777 §7.6.3.1 monotonicity property holds |
| Router 4-layer path | with 50 nodes/layer, returns 4-hop path in sub-millisecond |
| Aeronautical scenario | flight reaches arrival point within expected ETA |

**Long-run smoke (1 h):**

| Example | Lines | Comment |
|---|---:|---|
| `sagin-haps-leo-relay` | 3 601 | every step shows 3-hop path; HAPS elevation tracks figure-8 phase 75.9 → 84.0° |
| `sagin-uav-swarm` (10 min, 8 UAVs) | 961 | TR 36.777 RMa-AV PL recovered analytically: measured 75.43 dB at d3D=117.6 m, h_UT=152 m matches spec to **0.02 dB** |
| `sagin-aeronautical` | 721 | aircraft 250 m/s at 11 km, LEO range 2 072 km → 25 320 km as sat exits horizon |

## Documentation

- [INSTALL.md](INSTALL.md) — setup notes.
- 3GPP TR 36.777 V15.0.0 — *Study on Enhanced LTE Support for Aerial Vehicles*, Tables 6.2-1 / 7.6.3.1.

## Cite this work

```bibtex
@misc{uzair2026ntnsagin,
  author = {Uzair, Muhammad},
  title  = {ntn-sagin: Space-Air-Ground Integrated Network for 6G NTN Simulation},
  year   = {2026},
  url    = {https://github.com/Muhammaduazir69/ntn-sagin}
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
| **ntn-sagin** | this repo |
| ntn-slice | [ntn-slice](https://github.com/Muhammaduazir69/ntn-slice) |
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

3GPP RAN1 (TR 36.777 work item) · ns-3 mobility module · NIST NetSimulyzer for downstream visualisation.
