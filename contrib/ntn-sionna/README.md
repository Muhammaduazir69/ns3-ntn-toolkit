<h1 align="center">ntn-sionna</h1>

<p align="center"><strong>NVIDIA Sionna RT bridge for the <a href="https://github.com/Muhammaduazir69/ns3-ntn-toolkit">ns3-ntn-toolkit</a> — GPU-accelerated ray-traced channel for satellite-to-ground links.</strong></p>

<p align="center"><em>Part of the v2.0 roadmap (<a href="../../ROADMAP_EXECUTION.md">Workstream W9</a>).</em></p>

---

## What it does

Replaces the closed-form TR 38.811 free-space channel with an opt-in
ray-traced one. A Python process (`bridge/sionna-server.py`) loads a
Sionna RT scene on the GPU once and serves UDP path-loss queries; the
ns-3 side (`bridge/ns3-sionna-channel.{h,cc}`) is a `PropagationLossModel`
that sends position + carrier-frequency to the server and consumes the
answer.

```
┌──────── ns-3 simulation ────────┐    ┌──── sionna-server.py ────┐
│ NtnSionnaChannel::DoCalcRxPower │    │ Mitsuba scene + Sionna   │
│   ↳ UDP {tx, rx, f}      ───────┼───►│  RT.PathSolver (GPU)     │
│   ↳ recv ← path_loss_db ◄───────┼────│   ↳ paths.cir() → |a|²   │
│   ↳ FSPL fallback on timeout    │    │  rsp {pl_db, n_paths,…}  │
└─────────────────────────────────┘    └──────────────────────────┘
```

The TR 38.811 closed form remains the simulation default; the ray-traced
channel becomes available the moment the user opts in to it.

## Components

```
ntn-sionna/
├── bridge/
│   ├── sionna-server.py        # Python — Sionna RT + UDP server
│   ├── ns3-sionna-channel.h    # ns-3 PropagationLossModel client
│   └── ns3-sionna-channel.cc
├── examples/
│   └── leo-pass-sionna-vs-tr38811.cc
└── test/
    ├── ntn-sionna-test-suite.cc   # C++: FSPL + fallback + mock-loopback RTT
    └── test_sionna_server.py      # Python: live-server gates 1–3
```

## Quick start

```bash
# 1) Start the server (needs CUDA + Sionna + TF on the host)
python3 contrib/ntn-sionna/bridge/sionna-server.py --port 8765

# 2) Drive the LEO pass example
./ns3 run "leo-pass-sionna-vs-tr38811 --steps=30 --altKm=550"
```

In an ns-3 simulation:

```cpp
Ptr<NtnSionnaChannel> ch = CreateObject<NtnSionnaChannel> ();
ch->SetServer ("127.0.0.1", 8765);
ch->SetFrequencyHz (2.0e9);
ch->SetTimeoutMs (50);
ch->CalcRxPower (txDbm, satMobility, ueMobility);
```

If the server is down or unreachable, `CalcRxPower` falls back to the
closed-form FSPL (`NtnSionnaChannel::FreeSpacePathLossDb`) — the same
function the matched-scenario reference uses, so a CI run with no GPU
won't go off the rails.

## Audit results (2026-05-05)

**C++ test suite (`./test.py -s ntn-sionna`, 3 tests):** ✅ all pass.

| Test | Asserts |
|---|---|
| FSPL closed form is exact at known reference geometry | 98.47 dB @ 1 km / 2 GHz, 101.47 dB @ 1413 m / 2 GHz to 0.01 dB |
| Channel falls back to FSPL when no server responds | path loss matches closed form, timeouts > 0, fallbacks > 0 |
| Mock-server loopback RTT under 50 ms gate | 20 calls all under 50 ms, no timeouts, no fallbacks |

**Python integration suite (`pytest contrib/ntn-sionna/test/`, 6 tests, 2.93 s):** ✅ all pass.
Each test launches a fresh `sionna-server.py` subprocess.

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
| First-call RTT (JIT warmup) | ~350 ms (primer, not counted) |

The pass sweeps elevation 90° → 0°. At the matched-scenario LOS-only
setting Sionna RT and the closed-form FSPL agree to 0.002 dB across
the full pass, including the 213 dB low-elevation tail.

## Validation gates (per `ROADMAP_EXECUTION.md`)

| Gate | Result |
|---|---|
| Sionna server starts on GPU host | ✅ `test_server_starts` |
| ns-3 channel queries Sionna and gets responses with < 50 ms RTT | ✅ `test_steady_state_rtt_under_50ms` (p99 < 50 ms) and example (max 11.8 ms steady-state) |
| Path loss within ±3 dB of TR 38.811 reference under matched scenario | ✅ `test_pl_within_3db_of_tr38811` parameterised across 4 (d, f) points + example (max delta 0.002 dB across 30-step LEO pass) |

## Switching scenes

By default the server uses Sionna's `simple_reflector` scene with
`los_only=True` so the comparison vs. TR 38.811 free-space stays clean.
Pass a Mitsuba XML to override:

```bash
python3 sionna-server.py --scene-xml /path/to/munich.xml
```

Per-request, send `"los_only": false` in the JSON to enable specular /
refraction / diffraction — that's the multipath-aware mode that gives
Sionna its edge over the closed forms.

## License

GPL-2.0-only — same as the umbrella ns3-ntn-toolkit.

## Maintainer

Muhammad Uzair — `muhammaduzairr69@gmail.com` (ORCID: 0009-0002-4104-2680)
