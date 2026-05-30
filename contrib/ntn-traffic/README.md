# ntn-traffic

> The shared realistic-traffic helper (plus 3GPP HTTP + NRTV traffic models) that every other toolkit module composes for a real data plane.
> Part of **ns3-ntn-toolkit** — [README](../../README.md) / [INSTALL](../../INSTALL.md).

## Overview

`ntn-traffic` provides **`NtnRealisticTrafficHelper`** — a helper that drops a real ns-3 OnOff/PacketSink UDP data plane (InternetStack + per-UE point-to-point links + FlowMonitor) into **any** scenario, so that `Simulator::Run()` actually exercises packets, schedulers, and the protocol stack instead of returning instantly. Each install gives you a remote-host (CN) node, a feeder-gateway node, per-UE P2P links with /16 + /24 IPv4 addressing, one ramped OnOff app per UE, a periodic SimHealth heartbeat, and an end-of-run **`sim_health.csv`** that asserts realism gates. This is the helper every other module pulls in (via a single include, no circular deps) to give its examples a genuine data plane.

The module also ships NTN-oriented traffic generators with NTN-appropriate defaults: the **3GPP HTTP** model (over a satellite bent-pipe / regenerative link), the **NRTV** near-real-time video model over TCP/UDP, a CBR application, and a `TrafficTimeTag` for per-packet latency tracking.

## What's new in v2

See the toolkit [CHANGELOG](../../CHANGELOG.md).

- `NtnRealisticTrafficHelper` is the **backbone of the new cross-module data-plane examples** added across the toolkit (e.g. `ntn-constellation`, `ntn-fapi`, and the rest): instead of analytical math that returns instantly, those examples now move real packets and emit a `sim_health.csv`.
- The `sim_health` realism gates are **verified across all modules** in CI, so every example proves it kept the event queue and protocol stack busy for its full declared sim time.

## Models, helpers & key classes

- **`helper/ntn-realistic-traffic-helper.h` (`NtnRealisticTrafficHelper`)** — the headline API:
  - Config: `SetSimTime(Time)`, `SetOutputDir(dir)`, `SetRunTag(tag)`, `SetProfile(TrafficProfile)`, `SetUeBackboneRate`/`SetGatewayRate`, `SetGates(HealthGates)`, `SetStrictGates(bool)`, `SetHeartbeatPeriod`, `SetServerPort`.
  - Node setup: `InstallUes(numUes)` (creates and returns the UE container) or `AttachExistingUes(...)` (when the example owns nodes with a custom MobilityModel), then `Wire()` (installs stack, links, addressing, apps, and periodic gates — call before `Simulator::Run()`).
  - Hooks: `RegisterPeriodicCallback(period, cb)` to fire your analytical step (CHO eval, KPM emit, beam-hop) on the real event queue; `NotePacketTx`/`NotePacketRx`/`NoteAnalyticalEvent` counters for xApps/analytical modules.
  - Reporting: `WriteHealthReport()` writes `sim_health.csv`, prints a one-line summary, and (if strict gates are on) `NS_FATAL_ERROR`s on any missed floor.
  - `TrafficProfile`: `NbIotPeriodic`, `EmbbStreaming`, `UrllcPings`, `DigitalTwinTelemetry`, `MixedBouquet` (1/3 NB-IoT + 1/3 eMBB + 1/3 URLLC — the default, calibrated so a 60 s run with ≥6 UEs passes the gates).
  - `HealthGates`: `minWallClockPerSec` (0.015), `minPacketsPerSimSec` (≥100 tx/sim-sec), `minRxOverTxRatio` (≥0.85 delivery), `minAnalyticalCallbacks` (0). Floors scale with sim time; a scenario that returns instantly with an empty event queue fails the gate.
- **`sim_health.csv`** columns: `sim_time_s, wall_clock_s, packets_tx, packets_rx, tx_per_sim_sec, rx_over_tx, analytical_events, heartbeat_ticks, ues, run_tag`.
- **Traffic models** under `model/`: 3GPP HTTP (`three-gpp-http-satellite-client`, `three-gpp-http-variables`) with `three-gpp-http-satellite-helper`; NRTV (`nrtv-header`, `nrtv-tcp-client`, `nrtv-tcp-server`, `nrtv-udp-server`, `nrtv-variables`, `nrtv-video-worker`) with `nrtv-helper`; `cbr-application` with `cbr-helper`; `traffic-time-tag` for per-packet latency. Plotting helpers: `client-rx-trace-plot`, `histogram-plot-helper`.

## Examples

Listed in `examples/CMakeLists.txt`; both build to `build/contrib/ntn-traffic/examples/`.

### nrtv-p2p-example

Two nodes over a point-to-point link: an NRTV video server streams to a client; a `ClientRxTracePlot` records the Rx traffic.

```sh
./ns3 run "nrtv-p2p-example --time=10 --protocol=UDP"
```
```sh
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-traffic/examples/ns3.43-nrtv-p2p-example-default --time=10 --protocol=UDP
```

- **Outputs:** an NRTV client trace gnuplot file `NRTV-<protocol>-client-trace.plt` (e.g. `NRTV-UDP-client-trace.plt`) in the working directory — render it with `gnuplot NRTV-UDP-client-trace.plt`. This example does **not** print metrics to stdout.
- **Key args:** `--time` (simulation time, s), `--protocol` (`TCP` or `UDP`, upper case), `--verbose` (enable trace logging).

### nrtv-variables-plot

Draws samples from the NRTV traffic-model random-variable distributions and plots histograms.

```sh
./ns3 run "nrtv-variables-plot --numOfSamples=100000"
```
```sh
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-traffic/examples/ns3.43-nrtv-variables-plot-default --numOfSamples=100000
```

- **Outputs:** gnuplot `.plt` files in the working directory — `nrtv-num-of-frames.plt`, `nrtv-slice-size.plt`, `nrtv-slice-encoding-delay.plt`, `nrtv-idle-time.plt` (render with `gnuplot *.plt`).
- **Key args:** `--numOfSamples` (number of samples drawn per distribution; default 100000).

## Build, run & test

```sh
./ns3 configure --enable-examples --enable-tests
./ns3 build
./test.py --suite=cbr-test
./test.py --suite=nrtv
```

The module ships the `cbr-test` (unit) and `nrtv` (system) test suites. See [INSTALL](../../INSTALL.md) for full toolkit setup.

## License & author

GPL-2.0-only. Muhammad Uzair, Independent Researcher.
