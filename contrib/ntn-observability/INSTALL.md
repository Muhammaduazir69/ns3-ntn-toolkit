# Install & run — ntn-observability

<p align="center">
  <a href="README.md">Module README</a>
  &nbsp;·&nbsp;
  <a href="https://github.com/Muhammaduazir69/ns3-ntn-toolkit">Toolkit</a>
  &nbsp;·&nbsp;
  <a href="https://github.com/Muhammaduazir69/ns3-ntn-toolkit/blob/ntn-integration-v2/INSTALL.md">Toolkit install guide</a>
  &nbsp;·&nbsp;
  <a href="https://muhammaduazir69.github.io/ns3-ntn-toolkit/">Docs site</a>
</p>

> **The fastest path is the container.** `docker pull uzairdocker69/ns3-ntn-toolkit:latest`
> ships this module already built alongside the other thirteen and the vendored
> stacks, so nothing below is needed to simply run the examples. Build from source
> when you intend to change the module.

---

`ntn-observability` is an ns-3.43 contributed module: InfluxDB sinks, a
NetSimulyzer JSON exporter, reproducibility manifests, and pre-built Grafana
dashboards. The recommended way to run it is inside the
[ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit) tree (branch
`ntn-integration-v2`), where every dependency below is already present. It also
builds on a vanilla ns-3.43 tree once the sibling toolkit modules in section 2
are added.

---

## 1. System requirements

| Component | Version |
|---|---|
| OS | Linux (Ubuntu 22.04+ / Fedora 39+ recommended) |
| C++ compiler | gcc ≥ 11 or clang ≥ 14 |
| CMake | ≥ 3.24 |
| Python | ≥ 3.10 |
| ns-3 | **3.43** |
| Disk | ~6 GB after build (incl. SNS3 TLE data) |

The optional Grafana stack needs Docker + Docker Compose (InfluxDB 2.7,
Grafana 10.4, Telegraf sidecar).

---

## 2. Dependencies

The library (`CMakeLists.txt`) links only ns-3 `core`, `network`, and `mobility`
— no other module dependencies for the library itself. The examples link extra
sibling modules through their own `examples/CMakeLists.txt`; install whichever you
need under `contrib/` before configuring.

### 2a. Toolkit modules (REQUIRED for the examples)

Both packet examples build a real mmwave NR NTN cell (`NtnRealStackHelper` from
`ntn-traffic`) on a real SGP4 Walker pass and carry `NtnOranApplication` QoS
flows, so they link `ntn-traffic`, `ntn-cho`, and `ntn-constellation`
(`ntn-observability-demo` additionally links `ntn-rrc` for the control-plane
components). Inside `ns3-ntn-toolkit` all four are already in `contrib/`.

### 2b. mmWave NR PHY (REQUIRED for the examples)

Both packet examples run real mmwave NR NTN cells, so `contrib/mmwave` (and its
bundled `lte` dependency) must be present:

```bash
cd contrib/
git clone https://github.com/nyuwireless-unipd/ns3-mmwave.git mmwave
cd ..
```

### 2c. NetSimulyzer (BUNDLED — v1.0.13)

The NIST NetSimulyzer module (**v1.0.13**) is now **bundled** in the toolkit
under `contrib/netsimulyzer`. When present, the optional
`ntn-netsimulyzer-official-demo` example is built (it is guarded by
`if(TARGET ${libnetsimulyzer})`), driving the official NetSimulyzer module from
the toolkit's real SGP4 + TR 38.811 mobility. `NtnNetSimulyzerExporter` (this
module's own streaming JSON writer) needs no external module and always builds.

---

## 3. Install the module

### Inside the toolkit (recommended)

Already present in `ns3-ntn-toolkit/contrib/ntn-observability` (along with the
bundled `netsimulyzer`). Clone the toolkit (branch `ntn-integration-v2`):

```bash
git clone -b ntn-integration-v2 \
  https://github.com/Muhammaduazir69/ns3-ntn-toolkit.git
# GitLab mirror: https://gitlab.com/ns3-ntn-toolkit/ns3-ntn-toolkit
```

Docker image (everything pre-built): `uzairdocker69/ns3-ntn-toolkit:latest`
(or `:latest`).

### Standalone repo

Clone the standalone module into `contrib/ntn-observability`, pinning its current
branch:

```bash
cd contrib/
git clone -b ntn-observability-v2 \
  https://github.com/Muhammaduazir69/ntn-observability.git ntn-observability
cd ..
```

You still need the section 2 dependencies (`ntn-traffic`, `ntn-cho`,
`ntn-constellation`, `ntn-rrc`, `mmwave`) in `contrib/` for the examples; add
`contrib/netsimulyzer` v1.0.13 to also build `ntn-netsimulyzer-official-demo`.

---

## 4. Configure & build

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build ntn-observability
./ns3 show profile | grep ntn-observability   # expect: ... ntn-observability ...
```

---

## 5. Run the examples

Binaries land in `build/contrib/ntn-observability/examples/` as
`ns3.43-<name>-default`.

### 5a. ntn-observability-demo — end-to-end telemetry

```bash
./ns3 run "ntn-observability-demo --simTime=20 --runId=local-1 --influxFile=/tmp/run.lp --netSim=/tmp/run.json"
```
A real mmwave NR cell (SGP4 satellite, TR 38.811 UE, saturating eMBB
`NtnOranApplication` stream) alongside the `ntn-rrc` control-plane components
(TA, SIB19, UE location reports, DRX); every KPI is pushed through
`NtnObservabilityHelper` into the Influx sink and a NetSimulyzer JSON trace.
Outputs: line-protocol file at `--influxFile` (default
`/tmp/ntn-observability-demo.lp`), NetSimulyzer JSON at `--netSim` (default
`/tmp/ntn-observability-demo.json`), and `sim_health.csv` in `--outputDir`. Args:
`simTime`, `runId`, `influxFile`, `udpHost`, `udpPort`, `netSim`, `outputDir`.

### 5b. ntn-observability-traffic — instrumented real LEO downlink

```bash
./ns3 run "ntn-observability-traffic --simSeconds=40 --out=/tmp/ntn-obs.lp"
```
A real mmwave NR cell on a genuine SGP4 pass driven by a saturating eMBB
`NtnOranApplication` stream; every second an `NtnInfluxSink` exports the measured
KPIs as line protocol under measurement `ntn_downlink`. Outputs: line-protocol
file at `--out` (default `ntn-observability-traffic.lp`), per-flow KPM series
`ntn-observability-traffic_kpm_series.csv` / `.lp` (auto-exported by
`EnableAiFlowMonitor`), a per-second console table, and `sim_health.csv` in
`--outputDir`. Args: `simSeconds`, `leoAltKm`, `freqGHz`, `satEirpDbm`, `out`,
`influxHost`, `influxPort`, `outputDir`.

### 5c. ntn-netsimulyzer-official-demo — official NetSimulyzer (optional)

Built only when `contrib/netsimulyzer` (v1.0.13) is present.

```bash
./ns3 run "ntn-netsimulyzer-official-demo --sats=66 --ues=10 --duration=120 --out=/tmp/official.json"
```
Drives the official usnistgov NetSimulyzer module from the toolkit's real SGP4 +
TR 38.811 mobility, writing the official NetSimulyzer JSON to `--out`. Args:
`sats`, `ues`, `duration`, `altitude`, `out`.

---

## 6. Run the unit tests

```bash
./test.py --suite=ntn-observability
```
The suite has 10 unit tests: five for the sinks and schema (line-protocol encode,
line-protocol escaping, Influx file-sink round-trip, NetSimulyzer JSON shape,
pinned metric-schema stability) and five for the repro-manifest family (JSON
round-trip, schema-header presence, unknown-key tolerance, malformed-JSON
rejection, escape round-trip).

---

## 7. Grafana stack (Docker)

```bash
cd contrib/ntn-observability/docker
docker compose up -d   # InfluxDB 2.7 + Grafana 10.4 + Telegraf sidecar
```

Open Grafana at http://localhost:3000 (admin / admin). The four NTN dashboards
(Overview, Handover, Radio, ISL) are pre-loaded. The Telegraf sidecar rewrites
simulation-time stamps to ingest time.

---

## 8. Common issues

**Examples missing after configure** — `ntn-observability-demo` /
`ntn-observability-traffic` need `ntn-traffic`, `ntn-cho`, `ntn-constellation`
(and `ntn-rrc` for the demo) plus `mmwave` in `contrib/` (section 2).

**`ntn-netsimulyzer-official-demo` missing** — it is built only when
`contrib/netsimulyzer` (v1.0.13) is present; it ships bundled inside the toolkit.

---

## 9. Uninstall

```bash
rm -rf contrib/ntn-observability
./ns3 configure --enable-examples
./ns3 build
```