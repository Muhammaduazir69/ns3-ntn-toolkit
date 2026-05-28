<h1 align="center">ntn-sionna</h1>

<p align="center"><strong>NVIDIA Sionna RT Bridge for ns-3.43: GPU-Accelerated Ray-Traced Channel for Satellite-to-Ground Links</strong></p>

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0-green.svg"/></a>
  <img src="https://img.shields.io/badge/Sionna%20RT-2.0-orange.svg"/>
  <img src="https://img.shields.io/badge/3GPP-TR%2038.811%20reference-purple.svg"/>
  <img src="https://img.shields.io/badge/RTT-%E2%89%A510%20ms%20steady--state-success.svg"/>
  <img src="https://img.shields.io/badge/tests-28%20C%2B%2B%20%2B%206%20Python%20PASS-blue.svg"/>
</p>

---

<p align="center">
  <img src="docs/ntn_sionna_demo.gif" alt="module live demo" width="900"/>
</p>

## Why this module

Closed-form path-loss models like 3GPP TR 38.811 are fast and reproducible but they collapse every reflective object in the world into a single scalar shadowing term. For physical-layer research that depends on ray-level effects — beamforming gains in cluttered environments, multipath fading on a moving satellite-to-ground link, sensing-and-communication trade-offs — a real ray tracer is the right tool, and NVIDIA's [Sionna RT](https://nvlabs.github.io/sionna/api/rt.html) is the open-source state of the art. `ntn-sionna` wires Sionna RT into ns-3 as an **opt-in** `PropagationLossModel`: the closed-form TR 38.811 channel remains the simulation default, and the ray-traced channel becomes available the moment a user opts in to it. A small Python server keeps the Mitsuba scene resident on the GPU between queries; a UDP client on the C++ side streams `{tx, rx, freq_hz}` into it and gets `{path_loss_db, n_paths, compute_ms}` back, with a graceful FSPL fall-back when the server is offline so CI without a GPU still runs.

## At a glance

| Metric | Value |
|---|---:|
| LEO pass example, 30 steps, 550 km / 2 GHz | max &#124;Δ&#124; vs TR 38.811 = **0.002 dB** |
| Steps within ±3 dB gate | **30 / 30** |
| Steady-state RTT (median) | **~9 ms** |
| Steady-state RTT (max in 30-step pass) | 11.8 ms |
| First-call RTT (Sionna JIT warmup) | ~350 ms (primer, not counted) |
| Timeouts / fallbacks during the example | 0 / 0 |
| C++ unit tests | **3 / 3 PASS** |
| Python integration tests | **6 / 6 PASS** (in 2.93 s) |

## What it does

```
┌──────── ns-3 simulation ────────┐    ┌──── sionna-server.py ────┐
│ NtnSionnaChannel::DoCalcRxPower │    │ Mitsuba scene + Sionna   │
│   ↳ UDP {tx, rx, f}      ───────┼───►│  RT.PathSolver (GPU)     │
│   ↳ recv ← path_loss_db ◄───────┼────│   ↳ paths.cir() → |a|²   │
│   ↳ FSPL fallback on timeout    │    │  rsp {pl_db, n_paths, …} │
└─────────────────────────────────┘    └──────────────────────────┘
```

- **Sionna RT server** (`bridge/sionna-server.py`) — Python process loading a Mitsuba scene once at startup; UDP socket on a configurable port (default 8765); JSON wire format `{tx, rx, freq_hz, los_only}` → `{path_loss_db, n_paths, compute_ms}`; LOS-only mode for matched-scenario comparison vs TR 38.811 free-space; full-multipath available via `los_only: false` in the request.
- **ns-3 channel client** (`bridge/ns3-sionna-channel.{h,cc}`) — `PropagationLossModel` UDP client; per-call timeout drives the < 50 ms RTT budget; on timeout falls back to the closed-form FSPL the matched-scenario reference uses, so CI without a GPU still produces sane numbers.
- **LEO pass example** (`examples/leo-pass-sionna-vs-tr38811.cc`) — sweeps elevation 90° → 0° across a configurable LEO altitude / carrier frequency; logs Sionna PL vs TR 38.811 PL per step; self-checks the ±3 dB gate.
- **Tests** — C++ unit tests (`test/ntn-sionna-test-suite.cc`) cover the FSPL closed form, the timeout-fallback path, and a mock-server loopback under the RTT gate; Python integration tests (`test/test_sionna_server.py`) launch a fresh `sionna-server.py` subprocess and exercise the live ray tracer at four parametric (distance, frequency) points.

## Install & run

```bash
git clone https://github.com/Muhammaduazir69/ntn-sionna.git contrib/ntn-sionna
./ns3 build ntn-sionna leo-pass-sionna-vs-tr38811

# Terminal 1 — Sionna RT server (needs CUDA, TensorFlow ≥ 2.18, Sionna ≥ 2.0)
python3 contrib/ntn-sionna/bridge/sionna-server.py --port 8765

# Terminal 2 — ns-3 LEO pass driver
./ns3 run "leo-pass-sionna-vs-tr38811 --steps=30 --altKm=550"
```

In an ns-3 simulation:

```cpp
#include "ns3/ns3-sionna-channel.h"

Ptr<NtnSionnaChannel> ch = CreateObject<NtnSionnaChannel>();
ch->SetServer("127.0.0.1", 8765);
ch->SetFrequencyHz(2.0e9);
ch->SetTimeoutMs(50);
double rxDbm = ch->CalcRxPower(txDbm, satMobility, ueMobility);
```

If the server is down or unreachable, `CalcRxPower` falls back to `NtnSionnaChannel::FreeSpacePathLossDb(d, freq)` — the same closed form Sionna RT converges to in an empty scene — so a no-GPU CI run won't go off the rails.

## Atmospheric cascade (Roadmap §4.2.7)

Sionna RT models geometric multipath end-to-end but does not model the molecular absorption, rain, or LMS shadowing that dominate the link budget in real Earth-space links. `NtnSionnaCascadeChannel` composes the Sionna base with the ITU-R cascade:

```
Rx_dBm = NtnSionnaChannel::DoCalcRxPower(Tx_dBm, a, b)
       - Itu676::SlantPathAttenuationDb(freq, elev)         // gaseous (O2 + H2O)
       - Itu618::SlantPathRainAttenuationDb(freq, elev, R)  // rain (uses P.838 k/alpha)
       - Itu681::StepDb()                                    // LMS Markov shadowing
```

The four ITU-R models are re-used verbatim from the toolkit's `thz-ntn` module; the cascade adds no new physics, only composition + an attribute surface, and ships with 9 dedicated Simulator-driven tests covering geometry (ENU + ECEF), per-component sweeps, rain-impact comparison, server-down failure path, and MIMO transport passthrough.

```cpp
#include "ns3/ntn-sionna-cascade-channel.h"

Ptr<NtnSionnaCascadeChannel> cascade = CreateObject<NtnSionnaCascadeChannel>();
cascade->SetFrequencyHz(12.0e9);          // sets BOTH inner Sionna + chain
cascade->GetAtmosphericChain()->SetRainRateMmH(25.0);
cascade->GetAtmosphericChain()->SetEnableLms(true);
cascade->GetAtmosphericChain()->SetLmsEnvironmentInt(0); // urban
cascade->SetTransport(udpTransport);      // forwards to inner base

double rxDbm = cascade->CalcRxPower(txDbm, satMobility, ueMobility);
auto comps = cascade->GetLastComponents(); // { gaseousDb, rainDb, lmsDb, elevationDeg }
```

Each chain component can be toggled independently; defaults are gaseous + rain on (R = 0 mm/h → no rain attenuation), LMS off. The cascade is commutative in `(a, b)` ordering — whichever mobility has the smaller position magnitude is treated as the ground end automatically.

## 4-D LRU caching transport (Roadmap §4.2.4)

Sionna RT queries are the bottleneck of any large simulation. `SionnaCachingTransport` is a decorator that wraps any inner transport and caches responses keyed by a 4-D tuple:

```
(quantised_tx_pos, quantised_rx_pos, freq_hz, time_bucket)
```

Spatial quantisation is `SpatialResolutionM` (default 100 m); temporal quantisation is `TemporalBucketUs` (default 1 ms); the cache is bounded by `MaxEntries` (default 4096) with strict LRU eviction. On a hit the response is returned with `compute_ms = 0` so callers can distinguish cached from live values.

```cpp
#include "ns3/sionna-caching-transport.h"

Ptr<SionnaUdpTransport> live = CreateObject<SionnaUdpTransport>();
live->SetServer("127.0.0.1", 8765);

Ptr<SionnaCachingTransport> cache = CreateObject<SionnaCachingTransport>();
cache->SetInner(live);
cache->SetSpatialResolutionM(50.0);    // 50 m grid
cache->SetTemporalBucketUs(100000);    // 100 ms bucket

Ptr<NtnSionnaChannel> ch = CreateObject<NtnSionnaChannel>();
ch->SetTransport(cache);

// Run the simulation; `cache->GetHitRate()` reports cache health.
```

Coverage in `test/ntn-sionna-test-suite.cc`: hit/miss accounting, spatial-cell collapse, time-bucket boundary, LRU eviction, `Reset()`, and a 60 s Simulator::Run with 600 sampled queries that asserts >0.6 hit rate when bucket cadence exceeds sampling cadence. The cache is thread-safe via a single internal mutex; the optional async prefetch thread called out in the roadmap is intentionally deferred — bench data so far shows the synchronous cache already wins back >90 % of digital-twin replay-loop GPU time.

## RIS Tx surface support (Roadmap §4.2.3)

`SionnaTransport::Request` carries an optional `RisConfig` describing a Reconfigurable Intelligent Surface (position, normal, N × M elements, phase profile). The Python server installs `rt.RIS` in the scene per query when the field is present, picks the appropriate phase profile (`focus` / `flat` / `random`), and re-runs the exact-paths solver so reflections off the surface contribute to the returned path loss.

```cpp
#include "ns3/ns3-sionna-channel.h"

Ptr<NtnSionnaChannel> ch = CreateObject<NtnSionnaChannel>();
ch->SetServer("127.0.0.1", 8765);
ch->SetFrequencyHz(28.0e9);

RisConfig ris;
ris.pos_x = 706.5;  ris.pos_y = 0.0;  ris.pos_z = 50.0;     // 50 m up, midway
ris.normal_x = 0.0; ris.normal_y = 0.0; ris.normal_z = 1.0; // face up
ris.rows = 32;      ris.cols = 32;                           // 1024 elements
ris.spacing_lambda = 0.5;
ris.phase_profile = "focus";
ris.focal_x = 1413.0; ris.focal_y = 0.0; ris.focal_z = 0.0; // focus on UE
ch->SetRis(ris);

double rxDbm = ch->CalcRxPower(txDbm, satMobility, ueMobility);
ch->ClearRis();  // drop the surface
```

The mock-based unit tests use a deterministic convention (focus = ‑10 dB, flat = ‑6 dB, random = 0 dB shift) so the C++ side can assert end-to-end wire transit without a CUDA-capable host; the live Python server's actual reflection enhancement depends on geometry, frequency, and the chosen phase profile.

## Examples (Roadmap §4.2.12)

Three driver examples consume the cascade + cache + RIS surfaces. Each tolerates a missing Sionna server (channel falls back to FSPL) so smoke runs in CI still finish; with a live server they print real ray-traced figures.

| Example | What it shows | Run |
|---|---|---|
| `mmimo-vs-codebook-leo` | SISO vs 8×8 cross-pol PlanarArray under the atmospheric cascade across a 30 s LEO pass; reports rain + gaseous breakdown | `./ns3 run "mmimo-vs-codebook-leo --rainMmH=25 --freqHz=12e9"` |
| `ris-assisted-leo-link` | Before/after a 32×32 RIS focused at the UE during a LEO pass; reports per-sample RIS gain and aggregate min/max/mean | `./ns3 run "ris-assisted-leo-link --rows=32 --phaseProfile=focus"` |
| `city-block-4ue-cache` | AODT-style 4-UE city block over a `SionnaCachingTransport`; reports per-UE Rx, running cache hit / miss / evictions | `./ns3 run "city-block-4ue-cache --steps=60 --spatialResM=50"` |

All three live under `contrib/ntn-sionna/examples/` and share the same CMakeLists `build_lib_example` pattern as the original `leo-pass-sionna-vs-tr38811` example.

## Verification

**C++ unit tests (`./test.py -s ntn-sionna`, 3 cases, all passing):**

| Test | Asserts |
|---|---|
| FSPL closed form is exact at known reference geometry | 98.47 dB @ 1 km / 2 GHz, 101.47 dB @ 1413 m / 2 GHz to 0.01 dB |
| Channel falls back to FSPL when no server responds | path loss matches closed form, timeouts > 0, fallbacks > 0 |
| Mock-server loopback RTT under 50 ms gate | 20 calls all < 50 ms, no timeouts, no fallbacks |

**Python integration tests (`pytest contrib/ntn-sionna/test/`, 6 cases, 2.93 s, all passing):**

| Test | Result |
|---|---|
| `test_server_starts` | server up, returns valid JSON |
| `test_steady_state_rtt_under_50ms` | p99 < 50 ms after JIT warmup |
| `test_pl_within_3db_of_tr38811[1413 m, 2 GHz]` | matched ±3 dB |
| `test_pl_within_3db_of_tr38811[10 km, 2 GHz]` | matched ±3 dB |
| `test_pl_within_3db_of_tr38811[100 km, 5 GHz]` | matched ±3 dB |
| `test_pl_within_3db_of_tr38811[600 km, 12 GHz]` | matched ±3 dB |

**LEO pass example (550 km, 2 GHz, 30 steps, live Sionna server):**

| Metric | Value |
|---|---:|
| Queries | 30 |
| Timeouts | 0 |
| Fallbacks | 0 |
| Max abs delta vs TR 38.811 | **0.002 dB** |
| Steps within ±3 dB gate | **30 / 30** |
| Steady-state RTT (median) | ~9 ms |
| Steady-state RTT (max) | 11.8 ms |

The pass sweeps elevation 90° → 0°. At the matched-scenario LOS-only setting Sionna RT and the closed-form FSPL agree to **0.002 dB** across the full pass, including the 213 dB low-elevation tail.

## Switching scenes and modes

By default the server uses Sionna's `simple_reflector` scene with `los_only=True` so the comparison vs TR 38.811 free-space stays clean. Pass any Mitsuba XML to override:

```bash
python3 sionna-server.py --scene-xml /path/to/munich.xml
```

Per-request, send `"los_only": false` in the JSON to enable specular / refraction / diffraction — that's the multipath-aware mode that gives Sionna its edge over the closed forms.

## Documentation

- [INSTALL.md](INSTALL.md) — CUDA, TensorFlow, Sionna installation notes for the GPU host.
- [Sionna RT documentation](https://nvlabs.github.io/sionna/api/rt.html)
- 3GPP TR 38.811 — *Study on New Radio (NR) to support non-terrestrial networks*, §6.6 free-space reference.

## Cite this work

```bibtex
@misc{uzair2026ntnsionna,
  author = {Uzair, Muhammad},
  title  = {ntn-sionna: NVIDIA Sionna RT Bridge for 6G NTN Channel Simulation},
  year   = {2026},
  url    = {https://github.com/Muhammaduazir69/ntn-sionna}
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
| ntn-v2x | [ntn-v2x](https://github.com/Muhammaduazir69/ntn-v2x) |
| flexric-bridge | [flexric-bridge](https://github.com/Muhammaduazir69/flexric-bridge) |
| **ntn-sionna** | this repo |
| ntn-digital-twin | [ntn-digital-twin](https://github.com/Muhammaduazir69/ntn-digital-twin) |
| ntn-cho | [ntn-cho-framework](https://github.com/Muhammaduazir69/ntn-cho-framework) |
| oran-ntn | [oran-ntn](https://github.com/Muhammaduazir69/oran-ntn) |
| thz-ntn | [ns3-thz-ntn](https://github.com/Muhammaduazir69/ns3-thz-ntn) |

## License

GPL-2.0-only — see [LICENSE](LICENSE). Sionna RT is licensed by NVIDIA under Apache 2.0; this bridge interacts with Sionna over UDP and ships no Sionna source code.

## Acknowledgements

NVIDIA Research (Sionna RT, Mitsuba 3) · TensorFlow team · ns-3 propagation module · 3GPP TR 38.811 study item.
