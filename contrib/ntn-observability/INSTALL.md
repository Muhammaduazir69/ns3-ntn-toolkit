# Install & run — ntn-observability

`ntn-observability` is an ns-3.43 contributed module. It builds on top of a
vanilla ns-3.43 tree, or as part of the
[ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit).

---

## 1. System requirements

| Component | Version |
|---|---|
| OS | Linux (Ubuntu 22.04+ / Fedora 39+ recommended) |
| C++ compiler | gcc >= 11 or clang >= 14 |
| CMake | >= 3.24 |
| Python | >= 3.10 |
| ns-3 | **3.43** |

The optional Grafana stack needs Docker + Docker Compose (InfluxDB 2.7,
Grafana 10.4, Telegraf sidecar).

---

## 2. Dependencies

The module library (`CMakeLists.txt`) links the ns-3 **core** and **network**
modules. There are no other module dependencies for the library itself.

The examples link extra sibling modules through their own example
`CMakeLists.txt`:

- `ntn-observability-demo` links **`ntn-rrc`** and **`ntn-traffic`**
  (`NtnRealisticTrafficHelper`) plus `mobility`, `internet`, `applications`
  and `point-to-point`.
- `ntn-observability-traffic` links `mobility`, `internet`, `applications`,
  `point-to-point` and `flow-monitor`.

Install whichever siblings you need under `contrib/` before configuring.

---

## 3. Configure & build

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build ntn-observability
./ns3 show profile | grep ntn-observability   # expect: ... ntn-observability ...
```

---

## 4. Run the examples

```bash
# End-to-end telemetry: line protocol + NetSimulyzer JSON.
./ns3 run "ntn-observability-demo --simTime=300 --runId=local-1 --influxFile=/tmp/run.lp --netSim=/tmp/run.json"

# Real LEO downlink whose InfluxDB KPIs come from a live data plane.
./ns3 run "ntn-observability-traffic --simSeconds=120 --dataRateMbps=5 --out=/tmp/ntn-obs.lp"
```

See the README for the full per-example argument list.

---

## 5. Run the unit tests

```bash
./test.py --suite=ntn-observability
```

The suite has 10 unit tests: five for the sinks and schema (line-protocol
encode, line-protocol escaping, Influx file-sink round-trip, NetSimulyzer JSON
shape, pinned metric-schema stability) and five for the repro-manifest family
(JSON round-trip, schema-header presence, unknown-key tolerance,
malformed-JSON rejection, escape round-trip).

---

## 6. Grafana stack (Docker)

```bash
cd contrib/ntn-observability/docker
docker compose up -d   # InfluxDB 2.7 + Grafana 10.4 + Telegraf sidecar
```

Open Grafana at http://localhost:3000 (admin / admin). The four NTN dashboards
(Overview, Handover, Radio, ISL) are pre-loaded. The Telegraf sidecar rewrites
simulation-time stamps to ingest time.

---

## 7. Uninstall

```bash
rm -rf contrib/ntn-observability
./ns3 configure --enable-examples
./ns3 build
```
