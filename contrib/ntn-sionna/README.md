<h1 align="center">ntn-sionna</h1>

<p align="center"><strong>NVIDIA Sionna RT GPU ray-tracing bridged into ns-3 for non-terrestrial channels: cascade composition, caching/replay transports, RIS relay, and TR 38.811 calibration.</strong></p>

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0-green.svg"/></a>
  <img src="https://img.shields.io/badge/Sionna%20RT-2.0-orange.svg"/>
  <img src="https://img.shields.io/badge/3GPP-TR%2038.811%20reference-purple.svg"/>
  <img src="https://img.shields.io/badge/loopback%20RTT%20gate-%3C50%20ms-success.svg"/>
  <img src="https://img.shields.io/badge/tests-38%20C%2B%2B%20%2B%203%20Python%20PASS-blue.svg"/>
</p>

> Part of the [ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit). See [INSTALL.md](INSTALL.md) and [CHANGELOG.md](CHANGELOG.md).

---

## Overview

Closed-form path-loss models like 3GPP TR 38.811 are fast and reproducible, but they collapse every reflective object in the world into a single scalar shadowing term. For physical-layer research that depends on ray-level effects — beamforming in cluttered scenes, multipath fading on a moving satellite-to-ground link, RIS-assisted recovery of a blocked NLOS link — a real ray tracer is the right tool, and NVIDIA's [Sionna RT](https://nvlabs.github.io/sionna/api/rt.html) is the open-source state of the art.

`ntn-sionna` wires Sionna RT into ns-3 as an **opt-in** `PropagationLossModel`. A small Python server keeps the Mitsuba scene resident on the GPU between queries; the C++ side streams `{tx, rx, freq_hz}` into it over a pluggable transport and gets `{path_loss_db, n_paths, compute_ms}` back. Around that core the module adds:

- **Cascade composition** — the GPU-traced geometry is composed with the ITU-R atmospheric chain (gaseous + rain + LMS shadowing) so the link budget reflects molecular absorption and weather that Sionna RT does not model.
- **Headless-friendly transports** — UDP for a live GPU server, plus **caching** and **replay** transports that let an entire simulation run with **no live Sionna GPU at all**, falling back to closed-form FSPL when nothing answers.
- **Calibration against 3GPP TR 38.811** — a residual calibrator and harness that check the ray-traced channel against the closed-form reference within a configurable dB gate.

The closed-form TR 38.811 channel remains the simulation default; the ray-traced channel is available the moment a user opts in.

## What's new in v2

See [CHANGELOG.md](CHANGELOG.md) for the full history.

- **Cascade / caching / RIS channel bridges** — `NtnSionnaCascadeChannel` composes Sionna RT with the ITU-R `NtnAtmosphericLossChain`; `SionnaCachingTransport` is a 4-D LRU decorator over any transport; RIS Tx surfaces are carried in the request and installed in the scene per query.
- **Sionna calibrator** — `SionnaCalibrator` measures the residual of the ray-traced channel against a reference propagation model and feeds the calibration harness.
- **Batch client + replay transport + CIR Doppler synthesis** — `SionnaBatchClient` for async batched queries, `SionnaReplayTransport` (writer + reader) for record/replay runs without a GPU, and `CirDopplerSynthesizer` to advance a captured CIR snapshot to a Doppler-shifted offset.
- **Channel plug-ins for real radio stacks** — `NtnAtmosphericPropagationLossModel` re-homes the ITU-R chain as a real `PropagationLossModel` charging pure atmospheric *excess* (no FSPL double-count), and `SionnaCirPropagationLossModel` keeps the **full CIR** (multipath taps + per-tap Doppler) so the measured SINR exhibits genuine constructive/destructive fading. Both chain onto any spectrum channel.
- **Measured-radio `*-traffic` examples** — seven drivers run on a **real mmwave NR NTN cell** (`NtnRealStackHelper` from `contrib/ntn-traffic`: SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC) with genuine SGP4 Walker mobility. The channel physics sits in the packet path as chained `PropagationLossModel`s; traffic is `NtnOranApplication` QoS flows (in-band 5QI / S-NSSAI / seq / timestamp payload headers) with KPIs measured at `NtnOranSink` and the PHY trace — SINR, TBLER and goodput are measured, not asserted.
- **Verification** — physics-only examples (channel-model comparisons, RIS link budget, calibration harness) run cleanly with or without a live GPU server; the `*-traffic` examples carry real packets over the real radio.

## Models / bridges / key classes

| Class | Header | Role |
|---|---|---|
| `NtnSionnaChannel` | `bridge/ns3-sionna-channel.h` | Opt-in `PropagationLossModel`; queries Sionna over a transport, FSPL fall-back on timeout |
| `SionnaTransport` / `SionnaNoneTransport` | `bridge/sionna-transport.h` | Abstract transport + null transport; carries the request incl. optional `RisConfig` |
| `SionnaUdpTransport` | `bridge/sionna-udp-transport.h` | UDP client to a live `sionna-server.py` |
| `SionnaPybindTransport` | `bridge/sionna-pybind-transport.h` | In-process pybind transport (no socket) |
| `SionnaCachingTransport` | `bridge/sionna-caching-transport.h` | 4-D LRU caching decorator over any inner transport |
| `SionnaReplayTransport` (`SionnaReplayWriter` / `SionnaReplayReader`) | `bridge/sionna-replay-transport.h` | Record/replay transport — run without a live GPU |
| `SionnaBatchClient` | `bridge/sionna-batch-client.h` | Async batched query client |
| `NtnSionnaCascadeChannel` | `bridge/ntn-sionna-cascade-channel.h` | Sionna RT base + ITU-R atmospheric cascade |
| `NtnAtmosphericLossChain` (`Itu618`/`Itu676`/`Itu681`) | `bridge/ntn-atmospheric-loss-chain.h` | Composite gaseous + rain + LMS attenuation chain |
| `NtnAtmosphericPropagationLossModel` | `bridge/ntn-atmospheric-propagation-loss-model.h` | ITU-R atmospheric **excess** as a real `PropagationLossModel` — chains onto a Friis spectrum channel without double-counting FSPL |
| `SionnaCirPropagationLossModel` | `bridge/sionna-cir-propagation-loss-model.h` | Full-CIR multipath fading (per-tap Doppler via `CirDopplerSynthesizer`) as a real `PropagationLossModel` |
| `SionnaCalibrator` | `bridge/sionna-calibrator.h` | Residual calibration of Sionna RT vs a reference propagation model |
| `CirDopplerSynthesizer` | `bridge/cir-doppler-synth.h` | Synthesise a Doppler-shifted CIR from a captured snapshot |

Python side: `bridge/sionna-server.py` (live GPU server loading a Mitsuba scene once, JSON-over-UDP wire format, LOS-only and full-multipath modes).

## Examples

All 12 examples build to `build/contrib/ntn-sionna/examples/ns3.43-<NAME>-default`. Each can be run two ways — through the `./ns3 run` wrapper or by invoking the built binary directly:

```bash
./ns3 run "<NAME> --arg=value"
./build/contrib/ntn-sionna/examples/ns3.43-<NAME>-default --arg=value
```

No GPU is required for any example. The physics examples tolerate a missing Sionna server — with no transport answering, the channel falls back to closed-form FSPL so smoke runs still finish; with a live (or replay/caching) transport they print real ray-traced figures. The measured-radio examples run the ITU-R chain and array/CIR plug-ins closed-form, so they are fully headless.

### Measured-radio examples

These run a real mmwave NR NTN cell (`NtnRealStackHelper`: SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC) with SGP4 Walker satellite mobility projected into the local ENU frame. The channel physics is chained into the packet path as real `PropagationLossModel`s (`AddExtraPropagationLoss`), traffic is `NtnOranApplication` QoS flows, and SINR / TBLER / goodput are measured off the PHY trace and `NtnOranSink`. They link the sibling toolkit modules `ntn-traffic`, `ntn-cho`, `ntn-constellation` and the in-tree `mmwave` stack (all present in the toolkit tree).

| Example | What it shows | Key args |
|---|---|---|
| `ntn-sionna-leo-downlink-traffic` | Ku-band LEO downlink with the ITU-R cascade (P.676 gaseous + P.618/P.838 rain + optional P.681 LMS) live in the packet path; per-second elevation / attenuation / SINR / TBLER / goodput | `simSeconds freqGHz satEirpDbm rainMmH lms outputDir` |
| `ntn-sionna-rain-event-traffic` | Convective rain cell sweeps over a Ka-band gateway mid-pass; the rain schedule reconfigures the live chain, so the measured SINR and goodput dip and recover | `simSeconds freqGHz satEirpDbm peakRainMmH outputDir` |
| `ntn-sionna-ris-relay-traffic` | Direct path blocked mid-sim (NLOS), then a RIS engages; blockage and RIS gain are live channel reconfigurations the measured SINR responds to | `simSeconds freqGHz satEirpDbm blockageDb risRows risCols blockFraction risOnFraction outputDir` |
| `ntn-sionna-mimo-traffic` | SISO vs N×N MIMO terminals on one shared cell; the array gain is a per-UE channel plug-in, so the SISO/MIMO gap is measured, not asserted | `simSeconds freqGHz satEirpDbm rows cols outputDir` |
| `ntn-sionna-constellation-handover-traffic` | UE follows the best satellite of an SGP4 Walker constellation; candidates are projected from the one measured link via the ephemeris Friis ratio; hand-overs logged with genuine orbital timing | `simSeconds freqGHz satEirpDbm numSats hysteresisDb outputDir` |
| `ntn-sionna-composed-channel-traffic` | Standard ns-3 `SetNext()` composition — Friis → ITU-R excess → Nakagami fading → spectrum PHY; the `oran-ntn` TR 38.811 model is evaluated on the same live geometry beside the measured SINR | `simSeconds freqGHz satEirpDbm rainRateMmH outputDir` |
| `ntn-sionna-cir-real-stack` | Full Sionna-RT-style CIR (LOS + reflected taps, per-tap Doppler) on `SionnaCirPropagationLossModel`; the measured SINR exhibits genuine multipath fading variance a scalar path loss cannot reproduce | `duration numUes altitude satEirpDbm freqGhz platformSpeed outputDir` |

```bash
./ns3 run "ntn-sionna-leo-downlink-traffic --simSeconds=60 --rainMmH=10 --lms=1"
./build/contrib/ntn-sionna/examples/ns3.43-ntn-sionna-ris-relay-traffic-default --risRows=32 --risCols=32 --blockageDb=30
```

**Outputs:** per-second trace tables (elevation, attenuation, SINR, TBLER, goodput) and a measured-KPI summary on stdout, plus per-run KPI files and a `sim_health.csv` gate report under `--outputDir`.

### Physics / channel examples

These exercise the channel models, link budget, and calibration directly (no data plane).

| Example | What it shows | Key args |
|---|---|---|
| `leo-pass-sionna-vs-tr38811` | Sweeps elevation 90°→0° across a LEO pass; logs Sionna PL vs TR 38.811 PL per step and self-checks the ±3 dB gate | `host port freqHz altKm steps timeoutMs` |
| `mmimo-vs-codebook-leo` | SISO vs N×N cross-pol array under the atmospheric cascade across a pass; reports rain + gaseous breakdown | `host port freqHz altKm rainMmH steps rows cols timeoutMs` |
| `ris-assisted-leo-link` | Before/after a RIS focused at the UE during a pass; per-sample RIS gain + aggregate min/max/mean | `host port freqHz altKm rainMmH steps rows cols risPosX risPosZ phaseProfile timeoutMs` |
| `city-block-4ue-cache` | AODT-style 4-UE city block over a `SionnaCachingTransport`; per-UE Rx and running cache hit / miss / evictions | `host port freqHz altKm rainMmH steps timeoutMs spatialResM timeBucketUs` |
| `sionna-calibration-harness` | Drives `SionnaCalibrator` to measure ray-traced residual vs the reference model | `host port losOnly timeoutMs` |

```bash
./ns3 run "leo-pass-sionna-vs-tr38811 --steps=30 --altKm=550 --freqHz=2e9"
./build/contrib/ntn-sionna/examples/ns3.43-city-block-4ue-cache-default --steps=60 --spatialResM=50 --timeBucketUs=100000
```

**Outputs:** per-step PL / Rx / gain / residual tables on stdout; the LEO-pass and calibration drivers self-check their dB gate.

## Sionna setup

The bridge does **not** require a live Sionna GPU to run:

- **Live GPU path** — start `bridge/sionna-server.py` (needs CUDA, TensorFlow, Sionna RT) and point `SionnaUdpTransport` at it. See [INSTALL.md](INSTALL.md) for the GPU prerequisites and version matrix.
- **No-GPU paths** — `SionnaReplayTransport` replays a previously recorded query log, and `SionnaCachingTransport` serves cached responses; both run headless. With no transport answering at all, `NtnSionnaChannel` falls back to closed-form FSPL.
- **Stub server** — `test/sionna-stub-server.py` mimics the wire protocol using closed-form FSPL, so the C++ tests run without Sionna RT installed.
- **Geospatial tooling** — `tools/` ships `osm_to_sionna_scene.py` (OSM → Sionna scene), `lidar_dem_ingest.py` (AW3D30 + LiDAR → elevation grid), and `probe_sionna_env.py` (env / version gate). See [tools/README.md](tools/README.md).

## Build, run & test

```bash
# from the ns-3-dev root
./ns3 configure --enable-examples --enable-tests
./ns3 build ntn-sionna

# run an example (live, replay/cached, or FSPL-fallback — all work)
./ns3 run "leo-pass-sionna-vs-tr38811 --steps=30 --altKm=550"
```

Tests (suite name `ntn-sionna`):

```bash
./test.py -s ntn-sionna                 # C++ suite
pytest contrib/ntn-sionna/test/         # Python integration tests
pytest contrib/ntn-sionna/tools/tests/  # geospatial tool tests
```

For CUDA / TensorFlow / Sionna RT installation and the supported version matrix, see [INSTALL.md](INSTALL.md).

## Cite this work

```bibtex
@misc{uzair2026ntnsionna,
  author = {Uzair, Muhammad},
  title  = {ntn-sionna: NVIDIA Sionna RT Bridge for 6G NTN Channel Simulation},
  year   = {2026},
  url    = {https://github.com/Muhammaduazir69/ntn-sionna}
}
```

## License & author

GPL-2.0-only — see [LICENSE](LICENSE). Author: **Muhammad Uzair, Independent Researcher**.

Sionna RT is licensed by NVIDIA under Apache 2.0; this bridge interacts with Sionna over a transport and ships no Sionna source code.

## Acknowledgements

NVIDIA Research (Sionna RT, Mitsuba 3) · TensorFlow team · ns-3 propagation module · 3GPP TR 38.811 study item.
