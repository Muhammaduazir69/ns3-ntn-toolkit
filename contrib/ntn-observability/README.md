<h1 align="center">ntn-observability</h1>

<p align="center"><strong>InfluxDB line-protocol sink, NetSimulyzer JSON exporter, and pre-built Grafana dashboards for the <a href="https://github.com/Muhammaduazir69/ns3-ntn-toolkit">ns3-ntn-toolkit</a>.</strong></p>

<p align="center">
  <em>Part of the v2.0 roadmap (<a href="../../ROADMAP_EXECUTION.md">Workstream W3</a>).</em>
</p>

---

## What it does

Streams every relevant ns-3 trace into a time-series database and a 3D-viewer-ready JSON file:

```
ns-3 traces  ─┐
              ├──► NtnInfluxSink ──► InfluxDB v2 ──► Grafana (4 dashboards)
              │     (file or UDP)
              │
              └──► NtnNetSimulyzerExporter ──► NetSimulyzer 3D viewer
                    (JSON 1.0 schema)
```

A canonical metric schema (`model/ntn-metric-schema.h`) names every KPI exactly once so dashboards never break when new modules add metrics.

## Components

| File | Purpose |
|---|---|
| `model/ntn-metric-schema.h` | Canonical measurement / tag / field names. Header-only, zero runtime cost. |
| `model/ntn-influx-sink.{h,cc}` | `Point` struct + `EncodeLineProtocol` + buffered sink (UDP or file). |
| `model/ntn-netsimulyzer-exporter.{h,cc}` | Streaming JSON writer compatible with NIST NetSimulyzer 1.0 schema. |
| `helper/ntn-observability-helper.{h,cc}` | One-call setup of both sinks with consistent run-id tagging. |
| `examples/ntn-observability-demo.cc` | Drives the W2 full-stack scenario, pumping every KPI into both sinks. |
| `dashboards/*.json` | 4 Grafana dashboards (overview, handover, radio, ISL). |
| `docker/docker-compose.yml` | Fully-provisioned InfluxDB 2.7 + Grafana 10.4 stack. |

## Quick start — file mode (no Docker)

```bash
./ns3 build ntn-observability-demo
build/contrib/ntn-observability/examples/ns3.43-ntn-observability-demo-default \
    --simTime=300 --runId=local-1 \
    --influxFile=/tmp/run.lp \
    --netSim=/tmp/run.json
```

Outputs:

- `/tmp/run.lp` — InfluxDB line protocol you can `influx write -f /tmp/run.lp` into a v2 server later.
- `/tmp/run.json` — open in NetSimulyzer for 3D playback with timeline.

## Quick start — Grafana dashboards (Docker)

```bash
cd contrib/ntn-observability/docker
docker compose up -d
# Wait ~5 s for InfluxDB to finish first-run setup.

# In a separate shell, push live data:
build/contrib/ntn-observability/examples/ns3.43-ntn-observability-demo-default \
    --simTime=600 --runId=docker-1 \
    --udpHost=127.0.0.1 --udpPort=8089 \
    --netSim=/tmp/docker-1.json
```

Then open Grafana at <http://localhost:3000> (admin / admin) — the 4 NTN dashboards are pre-loaded under the `ns3-ntn-toolkit` folder.

| Dashboard | Panels | Use it for |
|---|---|---|
| `NTN Overview` | TA, RSRP/SINR, sat ECEF | quick scenario sanity check |
| `NTN Handover` | CHO trigger / exec / fail counters, TA-jump view | debugging CHO algorithm changes |
| `NTN Radio` | RSRP, SINR, BLER, MCS per UE×cell | PHY / MAC tuning |
| `NTN ISL` | ISL range, ISL load, sat ECEF | constellation + W1 wiring sanity |

## Programmatic use

```cpp
#include "ns3/ntn-observability-helper.h"
#include "ns3/ntn-metric-schema.h"

NtnObservabilityHelper obs;
obs.SetRunId("my-run");
obs.SetInfluxUdp("influxdb-host", 8089);
obs.SetNetSimulyzerOutput("/tmp/run.json");

Ptr<NtnInfluxSink> sink = obs.InstallInfluxSink();
sink->Start();

ntnobs::Point p;
p.measurement = ntnobs::measurement::kRadio;
p.tags[ntnobs::tag::kUeImsi] = "100001";
p.tags[ntnobs::tag::kCellId] = "C-1";
p.fieldsFloat[ntnobs::field::kRsrpDbm] = -97.5;
p.fieldsFloat[ntnobs::field::kSinrDb] = 12.3;
sink->Push(p);
// ... or attach Push() to any ns-3 TraceSource.

sink->Stop();
```

## Testing

**ns-3 unit tests (5 cases):**

```bash
./ns3 build ntn-observability-test
build/utils/ns3.43-test-runner-default --suite=ntn-observability --verbose
```

| Test | Asserts |
|---|---|
| `LineProtocolBasicEncodeTest` | encoder emits the expected `meas,tag=v field=v ts` shape with newline. |
| `LineProtocolEscapeTest` | commas / spaces / equals are properly escaped in tag/measurement keys. |
| `InfluxFileSinkRoundTripTest` | file-mode sink buffers + flushes correctly across simulation events. |
| `NetSimulyzerJsonShapeTest` | output JSON has `configuration` / `nodes` / `series` / `events` sections, escapes inner quotes correctly, opens with `{` and closes with `}`. |
| `MetricSchemaStableTest` | the canonical KPI names are pinned (downstream dashboards depend on them). |

**Pipeline integration test:**

```bash
cd contrib/ntn-constellation
.venv/bin/python ../ntn-observability/tests-py/test_e2e_pipeline.py
```

Spawns the demo, validates that all 6 expected measurements (`ntn_radio`, `ntn_timing_advance`, `ntn_sat_pos`, `ntn_drx`, `ntn_sib19`, `ntn_ue_report`) and 4 critical fields (`rsrp_dbm`, `ta_total_us`, `sat_x_m`, `lat_deg`) appear in the line-protocol output. If InfluxDB is up at `127.0.0.1:8086` it also runs a UDP-mode round-trip query; otherwise that step is skipped with a clear note.

## Audit results (2026-05-04)

Stress-tested at 1800 s (30 min) in the W1–W4 integration audit
(`AUDIT_W1_W4.md`):

| Check | Result |
|---|---|
| LP file size | 1.95 MB (18 810 points) |
| `ntn_drx` count | 1 800 (= 1 Hz × 1800 s, exact) |
| `ntn_radio` count | 1 800 (exact) |
| `ntn_sat_pos` count | 1 800 (exact) |
| `ntn_timing_advance` count | 1 800 (exact) |
| `ntn_sib19` count | 11 250 (= 1800 / 0.160 s, exact) |
| `ntn_ue_report` count | 360 (= 1800 / 5 s, exact) |
| `run_id` tag coverage | 18 810 / 18 810 records |
| Timestamp span | 0 – 1 799.84 s (continuous, no skips) |
| Critical fields present | 14 / 14 |
| NetSimulyzer JSON | 2 nodes + 2 series + 5 759 events (NodeMove 1800, SeriesSample 3600, LogMessage 359) |
| **W2→W3 TA fidelity** (analytic vs LP) | mean &#124;err&#124; 0.49 µs, max 1.0 µs — **bit-exact within int µs truncation** |

The W2→W3 fidelity check parses every `ntn_timing_advance` LP point and
compares it to a closed-form analytic ground truth using the demo's exact
mobility (UE position + sat position/velocity). 100 % of 1 800 samples land
within ≤ 1 µs — meaning every byte the sink emits matches the W2 source of
truth to the limit of `Time::GetMicroSeconds()` int truncation.

## Schema stability promise

`model/ntn-metric-schema.h` names are **stable across versions**. Adding new measurements/fields is fine; renaming an existing one breaks every shipped dashboard. The `MetricSchemaStableTest` unit test pins the names so you cannot accidentally rename them without the test going red.

## License

GPL-2.0-only. Same as the umbrella ns3-ntn-toolkit.

## Maintainer

Muhammad Uzair — `muhammaduzairr69@gmail.com` (ORCID: 0009-0002-4104-2680)
