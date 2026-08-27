<h1 align="center">ntn-observability</h1>

<p align="center"><strong>One scene recorder feeding NetSimulyzer, Cesium, InfluxDB and Grafana, with topology and a real time anchor</strong></p>

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg" alt="ns-3.43"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0-green.svg" alt="GPL-2.0"/></a>
  <img src="https://img.shields.io/badge/sinks-NetSimulyzer%20%C2%B7%20CZML%20%C2%B7%20InfluxDB-purple.svg" alt="NetSimulyzer CZML InfluxDB"/>
  <img src="https://img.shields.io/badge/metrics-TS%2028.552%20names-orange.svg" alt="TS 28.552 measurement names"/>
  <img src="https://img.shields.io/badge/examples-3-informational.svg" alt="3 examples"/>
</p>

<p align="center">
  <a href="https://github.com/Muhammaduazir69/ns3-ntn-toolkit">Toolkit</a>
  &nbsp;·&nbsp;
  <a href="INSTALL.md">Install</a>
  &nbsp;·&nbsp;
  <a href="#examples">Examples</a>
  &nbsp;·&nbsp;
  <a href="https://muhammaduazir69.github.io/ns3-ntn-toolkit/modules/ntn-observability/">Docs</a>
</p>

---

Visualizing an orbital simulation is mostly a question of what you refuse to draw. A globe that shows satellites moving is easy; a globe that shows which satellite is actually serving which terminal, at a time that maps back to simulation time, is the one that can catch a bug.

This module records the scene once and exports it four ways: NetSimulyzer JSON, CZML for a Cesium globe, InfluxDB line protocol, and CSV under TS 28.552 measurement names. Links travel with positions, so serving, feeder and inter-satellite edges appear rather than being inferred client-side. The time axis is anchored to a documented scene epoch instead of collapsing onto ingest time, and the sink reports which anchor it used.

User-controlled labels are escaped on the way out, because a node name with a quote in it should not be able to corrupt a line-protocol stream. `tools/check_dashboard_producers.py` walks every shipped Grafana panel back through the metric schema to the code that emits it, and fails when a panel queries something nothing produces.

## Quick start

Inside the toolkit, where the module is already present and built:

```bash
./ns3 run ntn-observability-traffic
./ns3 run ntn-observability-demo
```

Standalone, into an existing ns-3.43 tree:

```bash
git clone -b ntn-observability-v2 https://github.com/Muhammaduazir69/ntn-observability.git contrib/ntn-observability
./ns3 configure --enable-modules='' --enable-examples --enable-tests
./ns3 build
```

`INSTALL.md` in this directory carries the full dependency list. Most examples in
this module build on `ntn-traffic`, the toolkit's real-stack spine, so the
toolkit tree is the path of least resistance.

## Overview

A 6G NTN simulation produces every kind of telemetry a real RAN does — RSRP, SINR, BLER, MCS, timing-advance, sat ECEF, ISL load, handover counters — but ns-3's default trace surface stops at flat ASCII files. That is fine for unit tests; it does not scale to a multi-hour, multi-satellite scenario where reviewers want a dashboard rather than a pile of `.tr` files. `ntn-observability` plugs the simulator into the same observability stack production RANs already use, and pairs every run with a reproducibility manifest:

- **Structured run manifests** — `NtnReproManifest` captures the toolkit git SHA, ns-3 version, scenario name, TLE epoch, constellation, service models, CLI argv and arbitrary extras, then serialises to / parses from JSON so a run is reproducible byte-for-byte.
- **KPI / observability sinks** — `NtnInfluxSink` emits InfluxDB v2 line protocol over UDP or to a file; `NtnNetSimulyzerExporter` streams NetSimulyzer 1.0 JSON for 3D playback.
- **Canonical metric schema** — `ntn-metric-schema.h` names every measurement, tag and field exactly once so downstream dashboards never break when new modules add metrics.

```
ns-3 traces  ─┐
              ├──► NtnInfluxSink ──► InfluxDB v2 ──► Grafana (4 dashboards)
              │     (UDP-line-protocol or file mode)
              │
              └──► NtnNetSimulyzerExporter ──► NetSimulyzer 3D viewer
                    (JSON 1.0 schema)
```

## What changed in v2.5

See the [CHANGELOG](CHANGELOG.md) for the full history.

- New **`ntn-repro-manifest` model** (`NtnReproManifest`) — a reproducibility manifest that records git SHA, ns-3 version, scenario, TLE epoch, constellation, Sionna / HITRAN / ITU versions, service models, CLI argv and extras, with `WriteJson()` / `LoadJson()` round-tripping.
- **Both examples now observe a measured radio.** Each builds a real mmwave NR NTN cell (`NtnRealStackHelper` from `ntn-traffic`: SpectrumPhy + MAC + RLC/PDCP + RRC + EPC) on a real SGP4 Walker satellite pass. The exported SINR / TBLER come from the mmwave PHY trace, goodput from the UE sink's byte counter, elevation and slant range from live SGP4 geometry — the dashboard observes a genuine link, not a synthetic curve.
- **Traffic is `NtnOranApplication` QoS flows** (`ntn-traffic`), not `OnOffApplication`: every packet carries a real 24-byte in-band payload header (5QI / S-NSSAI / QFI / sequence number / TX timestamp), so the goodput the sinks report is a real application-layer measurement.
- **Interop:** `ntn-traffic`'s `NtnOranAiFlowMonitor` exports per-flow KPIs under official 3GPP TS 28.552 / O-RAN E2SM-KPM metric names as CSV, FlowMonitor-style XML or InfluxDB line protocol (measurement `ntn_oran_kpm`); those `.lp` files ingest through the same Docker stack and Grafana data source as this module's sinks. The **canonical wiring is `NtnRealStackHelper::EnableAiFlowMonitor(outputPrefix)`** — one call that attaches the monitor to every helper-installed flow and auto-exports `<outputPrefix>_kpm_series.csv` / `.lp` at end of simulation; `ntn-observability-traffic` uses it.
- **Bounded buffering:** `NtnInfluxSink` caps its in-memory buffer via the `MaxBufferPoints` attribute (default 1,000,000 points); when exceeded the oldest points are dropped (warned once, counted via `GetDroppedPoints()`), so long runs without flushes degrade gracefully instead of exhausting RAM.
- **Honest RSRP:** `ntn-observability-demo` exports the `radio` measurement only when a measured SINR sample exists, deriving RSRP from the cell's configured noise floor (`-174 dBm/Hz + NF + 10 log10(BW)`) instead of the old `SINR - 95` heuristic / `-120 dBm` fallback.

## Models, helpers & key classes

| Header | Provides |
|---|---|
| `model/ntn-metric-schema.h` | Header-only canonical schema — `measurement::`, `tag::`, `field::` string tables naming every KPI exactly once; pinned by `MetricSchemaStableTest`. |
| `model/ntn-influx-sink.h` | `NtnInfluxSink` — InfluxDB v2 line-protocol encoder (escapes commas / spaces / equals in keys), buffered file or UDP transport, run-id tagging; `Push(Point)`, `Start()`. |
| `model/ntn-netsimulyzer-exporter.h` | `NtnNetSimulyzerExporter` — streaming JSON writer for the NIST NetSimulyzer 1.0 schema (`configuration` / `nodes` / `series` / `events` in valid order). |
| `model/ntn-repro-manifest.h` | `NtnReproManifest` — fluent setters (`SetToolkitGitSha`, `SetNs3Version`, `SetScenarioName`, `SetTleEpoch`, `SetConstellation`, `AddServiceModel`, `AddExtra`, …), `WriteJson()` / `ToJson()` / `LoadJson()` / `ParseJson()`. |
| `helper/ntn-observability-helper.h` | `NtnObservabilityHelper` — one-call setup of both sinks with consistent run-id tagging (`SetRunId`, `SetInfluxUdp`, `SetNetSimulyzerOutput`, `InstallInfluxSink`). |

Four pre-built Grafana dashboards (Overview, Handover, Radio, ISL) auto-load under the `ns3-ntn-toolkit` folder when the bundled Docker stack (InfluxDB 2.7 + Grafana 10.4 + Telegraf sidecar) starts.

## Examples

Build all examples with `./ns3 configure --enable-examples --enable-tests && ./ns3 build`. Each example produces the binary `build/contrib/ntn-observability/examples/ns3.43-<name>-default`.

### ntn-observability-demo

End-to-end telemetry demo: a real mmwave NR cell (SGP4 satellite, TR 38.811 UE, saturating eMBB `NtnOranApplication` stream) runs alongside the `ntn-rrc` NTN control-plane components (timing advance, SIB19 broadcast, UE location reports, DRX), and every KPI is pushed through `NtnObservabilityHelper` into the Influx sink and a NetSimulyzer JSON trace. The `radio` measurement carries the *measured* SINR from the cell's PHY trace.

```bash
./ns3 run "ntn-observability-demo --simTime=20 --runId=local-1 --influxFile=/tmp/run.lp --netSim=/tmp/run.json"
```

```bash
LD_LIBRARY_PATH=build/lib \
  ./build/contrib/ntn-observability/examples/ns3.43-ntn-observability-demo-default \
  --simTime=20 --runId=local-1 --influxFile=/tmp/run.lp --netSim=/tmp/run.json
```

Outputs:
- InfluxDB line-protocol file at `--influxFile` (default `/tmp/ntn-observability-demo.lp`) — `influx write -f` it, or send to `--udpHost:--udpPort` for live ingest.
- NetSimulyzer JSON at `--netSim` (default `/tmp/ntn-observability-demo.json`) for 3D playback.
- `sim_health.csv` in `--outputDir` (default `.`), written by `NtnRealStackHelper::WriteHealthReport()`.

Key args: `--simTime` (sim duration, s; default 20) · `--runId` (tag value attached to every point) · `--influxFile` (line-protocol output file) · `--udpHost` (InfluxDB UDP host; if set, overrides file output) · `--udpPort` (InfluxDB UDP listener port) · `--netSim` (NetSimulyzer JSON output) · `--outputDir` (output directory for `sim_health.csv`).

### ntn-observability-traffic

Instruments a real LEO downlink with the observability stack: a real mmwave NR cell on a genuine SGP4 pass, driven by a saturating eMBB `NtnOranApplication` stream. Every second an `NtnInfluxSink` exports the *measured* KPIs as line protocol under measurement `ntn_downlink` — fields `goodput_mbps` (UE sink byte counter), `sinr_db` / `tbler` (mmwave PHY trace), `elevation_deg` / `slant_range_km` (live SGP4 geometry) and `rx_bytes_total` — so the exported time series is a faithful telemetry trace of the pass.

```bash
./ns3 run "ntn-observability-traffic --simSeconds=40 --out=/tmp/ntn-obs.lp"
```

```bash
LD_LIBRARY_PATH=build/lib \
  ./build/contrib/ntn-observability/examples/ns3.43-ntn-observability-traffic-default \
  --simSeconds=40 --out=/tmp/ntn-obs.lp
```

Outputs: InfluxDB line-protocol file at `--out` (default `ntn-observability-traffic.lp`) carrying the measured KPIs — optionally streamed over UDP to `--influxHost:--influxPort` instead of a file — plus per-flow KPM series `ntn-observability-traffic_kpm_series.csv` / `.lp` (auto-exported by `EnableAiFlowMonitor`), a per-second console table and `sim_health.csv` in `--outputDir` (default `ntn-observability-traffic-output`).

Key args: `--simSeconds` (sim duration, s; default 40) · `--leoAltKm` (satellite altitude, km; default 550) · `--freqGHz` (carrier frequency, GHz; default 12, Ku-band) · `--satEirpDbm` (satellite EIRP / gNB Tx power, dBm) · `--out` (line-protocol output file) · `--influxHost` (InfluxDB/Telegraf UDP host; empty = file) · `--influxPort` (InfluxDB/Telegraf UDP port) · `--outputDir` (output directory for `sim_health.csv`).

### ntn-netsimulyzer-official-demo

Path B of the toolkit's 3D-visualization plan: drives the **official usnistgov NetSimulyzer** ns-3 module (schema-exact NetSimulyzer-1.0 JSON) from a live NR / mmwave NTN cell, so satellites and UEs render with per-UE, SINR-driven state. This is the standards-module path alongside the home-grown `NtnSceneRecorder`.

```sh
./ns3 run "ntn-netsimulyzer-official-demo --sats=6 --ues=4 --duration=30 --radio=nr"
```

Outputs: an official NetSimulyzer JSON scene at `--out` (default `ntn-official.json`) — open it in the NetSimulyzer desktop application.

Key args: `--sats` (satellites to render; default 6) · `--ues` (UEs to render; default 4) · `--duration` (sim duration, s; default 30) · `--altitude` (constellation altitude, km; default 550) · `--radio` (radio backend: `nr` = 5G-LENA FR1 or `mmwave` = FR2; default `nr`) · `--out` (NetSimulyzer JSON output path; default `ntn-official.json`).

## Build, run & test

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build
./test.py -s ntn-observability
```

The `ntn-observability` suite has 10 unit tests. Five cover the sinks and schema (line-protocol basic encode, line-protocol escaping, Influx file-sink round-trip, NetSimulyzer JSON shape, and the pinned metric-schema-stability test). The other five cover the repro-manifest family: JSON round-trip, schema-header presence, tolerance of unknown keys, rejection of malformed JSON, and escape round-trip.

### Grafana stack (Docker)

```bash
cd contrib/ntn-observability/docker
docker compose up -d   # InfluxDB 2.7 + Grafana 10.4 + Telegraf sidecar
```

Open Grafana at http://localhost:3000 (admin / admin) — the 4 NTN dashboards (Overview, Handover, Radio, ISL) are pre-loaded.

See [INSTALL.md](INSTALL.md) for setup, dependencies and the Telegraf sidecar configuration that rewrites simulation-time stamps to ingest time. For the full toolkit, see [ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit).

---

## Standards implemented

3GPP TS 28.552 (5G performance measurements, the naming used for every exported series), TS 32.425. O-RAN E2SM-KPM measurement identities. CZML for time-dynamic geospatial scenes, InfluxDB line protocol, NetSimulyzer scene JSON.

## Keywords

network observability, simulation telemetry, NetSimulyzer, Cesium, CZML, InfluxDB, Grafana dashboard, time series, TS 28.552, E2SM-KPM, KPI export, scene recorder, 3D visualization, satellite ground track, link topology, provenance, reproducibility manifest, non-terrestrial network, ns-3.

## Author

**Muhammad Uzair**, Independent Researcher
[ORCID 0009-0002-4104-2680](https://orcid.org/0009-0002-4104-2680)

Part of the [ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit),
a pre-integrated ns-3.43 platform for 6G non-terrestrial network research.
Mirrored on [GitLab](https://gitlab.com/ns3-ntn-toolkit).

## License

GPL-2.0-only, matching ns-3.
