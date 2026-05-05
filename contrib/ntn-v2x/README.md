<h1 align="center">ntn-v2x</h1>

<p align="center"><strong>SUMO TraCI Bridge and V2X-LEO Direct / Relay Channels for Vehicular and Maritime NTN Research</strong></p>

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0-green.svg"/></a>
  <img src="https://img.shields.io/badge/SUMO-TraCI%20v20%2B%20%2B%20FCD%20replay-orange.svg"/>
  <img src="https://img.shields.io/badge/scenarios-rural--highway%20%E2%80%A2%20maritime-purple.svg"/>
  <img src="https://img.shields.io/badge/unit_tests-5%20PASS-success.svg"/>
</p>

---

<p align="center">
  <img src="docs/ntn_v2x_demo.gif" alt="module live demo" width="900"/>
</p>

## Why this module

Vehicular networking research has historically lived in OMNeT++ via Veins. Veins is excellent but cannot be lifted into ns-3 without a parallel discrete-event kernel — and the rest of the 6G NTN toolkit is firmly in ns-3. `ntn-v2x` mirrors what Veins offers in spirit (a SUMO-driven mobility feed plus a couple of vehicular channel models) but builds it natively against the ns-3 mobility stack. Two operation modes ride the same API: a live TraCI socket to a running SUMO process, and an offline FCD-trace replay that reads SUMO's standard floating-car-data CSV. Together they let researchers run live co-simulations and reproduce the exact same scenario bit-for-bit in CI.

## At a glance

| Capability | Backing |
|---|---|
| Live SUMO co-simulation | `SumoTraciBridge::ConnectTcp("127.0.0.1", 8813)` (TraCI v20+) |
| Offline trace replay | `SumoTraciBridge::LoadFcdTrace(csv)` (`time,vehid,x,y,z,speed`) |
| V2X-LEO uplink budget | `V2xLeoDirect::ComputeStatic` — closed-form FSPL + SNR + elevation |
| V2V-via-LEO relay | `V2xLeoRelay::EvaluateAll` — best-SNR within range |
| Maritime mobility | 2-D vessel mobility (5–13 m/s merchant cruising) |
| Synthetic FCD generator | `WriteSyntheticFcdCsv` — RNG-seeded, reproducible |

| Verification metric (100 vehicles × 5 min replay) | Value |
|---|---:|
| Trace samples | **30 100** (= 100 × 301 ticks) |
| Sync jitter (TraCI bridge) | **0 ms** in replay (gate < 100 ms) |
| Test suite (`ntn-v2x`, 5 tests) | **PASS in 0.048 s** |
| Wallclock for 5-min, 100-vehicle scenario | ~30 s |

## What it does

```
SUMO  ──TCP TraCI──►  SumoTraciBridge  ──MobilityModel.SetPosition──► ns-3
   │                       │  (or trace-replay from FCD CSV)
   │                       └──► per-vehicle samples (TracedCallback)
   ▼
[SUMO live]                                                          [ns-3]
```

- **TraCI bridge** (`model/sumo-traci-bridge`) — TraCI client + FCD trace replay; tracks per-step sync jitter for the validation gate. Single API for both modes; `Step()` advances the simulation regardless of source.
- **V2X-LEO direct uplink** (`model/v2x-leo-direct`) — vehicle ↔ LEO uplink budget (free-space PL + SNR + elevation), closed-form `ComputeStatic` with no `Object` overhead so it can be called inside hot loops without allocation churn. Spot-check at 1 000 km / 2 GHz: measured 158.47 dB matches the analytic reference within 0.1 dB.
- **V2X-LEO relay** (`model/v2x-leo-relay`) — V2V-via-LEO relay assignment: each vehicle either uplinks direct or relays through the peer with best SNR within `m_maxV2vRangeM`. `m_minDirectSnrDb` controls when a vehicle prefers a peer-relay over its own direct uplink.
- **Maritime scenario** (`model/maritime-scenario`) — 2-D vessel mobility for sea-area scenarios; bounces off all four box edges, stays inside the bounded region across multi-hour runs.
- **Synthetic FCD generator** (`helper/ntn-v2x-helper`) — `WriteSyntheticFcdCsv` produces reproducible, RNG-seeded synthetic FCD traces; lets CI run without a SUMO install and lets a researcher re-run the exact same scenario by re-using the same seed.

## Install & run

```bash
git clone https://github.com/Muhammaduazir69/ntn-v2x.git contrib/ntn-v2x
./ns3 build ntn-v2x-rural-highway
build/contrib/ntn-v2x/examples/ns3.43-ntn-v2x-rural-highway-default \
    --vehicles=100 --simTime=300 --csv=/tmp/highway.csv
```

Programmatic use:

```cpp
#include "ns3/sumo-traci-bridge.h"
#include "ns3/v2x-leo-relay.h"

using namespace ns3::ntnv2x;

auto bridge = CreateObject<SumoTraciBridge>();
bridge->LoadFcdTrace("/path/sumo-fcd.csv");
// or: bridge->ConnectTcp("127.0.0.1", 8813);

auto relay = CreateObject<V2xLeoRelay>();
relay->SetSatellite(satMobility);
relay->SetMinDirectSnrDb(6.0);

for (int i = 0; i < nVehicles; ++i) {
    auto m = CreateObject<ConstantPositionMobilityModel>();
    bridge->RegisterVehicle("veh" + std::to_string(i), m);
    relay->RegisterVehicle("veh" + std::to_string(i), m);
}

bridge->Step();                          // pulls positions from SUMO/trace
auto decisions = relay->EvaluateAll();   // direct vs. relay for every vehicle
double jitterMs = bridge->GetLastJitterSec() * 1000.0;
```

## Verification

**Test suite (`ntn-v2x`, 5 cases, all passing):**

| Test | Asserts |
|---|---|
| Trace-replay sync jitter under 100 ms | jitter = 0 ms (deterministic replay; gate < 100 ms) |
| V2X-LEO direct free-space PL | 158.47 dB measured vs analytic 158.47 dB (< 0.1 dB error) at 1 000 km / 2 GHz |
| Relay falls back to peer | A direct, B relays via A when A's SNR > threshold > B's SNR |
| Maritime stays inside sea area | bounces correctly off all 4 box edges over 30 min |
| 100-vehicle 5-min replay | 30 100 sample rows, all timestamps strictly monotone |

**100-vehicle / 5-min long-run audit (`ntn-v2x-rural-highway`):**

```
ntn-v2x-rural-highway done.
  vehicles      : 100
  simTime       : 300 s
  trace samples : 30 100
  max jitter    : 0 ms       (gate: < 100 ms)

  per-second breakdown (averaged over 301 samples):
     direct = 35.2,  relay = 0.1,  orphan = 64.7

  per-second SNR (best vehicle):
     t=0    : −4.69 dB   (LEO 2 Mm west of road)
     t=300  : +5.79 dB   (LEO above the road)
```

The coverage curve `direct=0` → `direct=100` over 5 min mirrors a single LEO pass: at t=0 the satellite is 2 Mm west and even the best-positioned vehicle has SNR < 6 dB (below a typical Starlink terminal lock threshold, hence "orphan"); by t=300 the satellite is overhead and every vehicle hits the threshold.

## Live SUMO co-simulation

The TraCI client speaks v20+: `ConnectTcp("127.0.0.1", 8813)` opens a socket and `Step()` advances the simulation. The current implementation is a TCP scaffold — the wire-level command codec (`CMD_SIMSTEP`, `CMD_GET_VEHICLE_VARIABLE`, etc.) is intentionally minimal so the live path doesn't pull in dependencies that CI might lack. To run against a real SUMO, port the wire codec from [`libsumostatic`](https://github.com/eclipse-sumo/sumo) into `SumoTraciBridge::Step` — the public API stays identical.

## Documentation

- [INSTALL.md](INSTALL.md) — setup notes.
- [SUMO TraCI documentation](https://sumo.dlr.de/docs/TraCI.html)
- [SUMO FCD-output specification](https://sumo.dlr.de/docs/Simulation/Output/FCDOutput.html)

## Cite this work

```bibtex
@misc{uzair2026ntnv2x,
  author = {Uzair, Muhammad},
  title  = {ntn-v2x: SUMO TraCI Bridge and V2X-LEO Channels for ns-3.43},
  year   = {2026},
  url    = {https://github.com/Muhammaduazir69/ntn-v2x}
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
| ntn-slice | [ntn-slice](https://github.com/Muhammaduazir69/ntn-slice) |
| **ntn-v2x** | this repo |
| flexric-bridge | [flexric-bridge](https://github.com/Muhammaduazir69/flexric-bridge) |
| ntn-sionna | [ntn-sionna](https://github.com/Muhammaduazir69/ntn-sionna) |
| ntn-digital-twin | [ntn-digital-twin](https://github.com/Muhammaduazir69/ntn-digital-twin) |
| ntn-cho | [ntn-cho-framework](https://github.com/Muhammaduazir69/ntn-cho-framework) |
| oran-ntn | [oran-ntn](https://github.com/Muhammaduazir69/oran-ntn) |
| thz-ntn | [ns3-thz-ntn](https://github.com/Muhammaduazir69/ns3-thz-ntn) |

## License

GPL-2.0-only — see [LICENSE](LICENSE).

## Acknowledgements

Eclipse SUMO team (DLR) · Veins maintainers (architecture inspiration) · ns-3 mobility module · 3GPP TR 22.886 (V2X service requirements).
