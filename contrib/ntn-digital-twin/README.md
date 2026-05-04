<h1 align="center">ntn-digital-twin</h1>

<p align="center"><strong>Live digital-twin loop + REST API + CesiumJS Live mode for the <a href="https://github.com/Muhammaduazir69/ns3-ntn-toolkit">ns3-ntn-toolkit</a>.</strong></p>

<p align="center"><em>Part of the v2.0 roadmap (<a href="../../ROADMAP_EXECUTION.md">Workstream W10</a>).</em></p>

---

## What it does

Mirrors a real LEO constellation in near-real time. A small refresher loop
pulls fresh TLEs every few minutes, regenerates state, and emits updates
that the rest of the toolkit can consume:

```
      cron / systemd timer
           │
           ▼
   twin_loop.run_iteration()
           │
           ├── 1. CelesTrak fetch  (W1)
           ├── 2. Constellation propagate
           ├── 3. CZML write       → CesiumJS "Live" toggle picks up the file
           └── 4. Line-protocol    → InfluxDB (W3) bucket "ntn"

           ┌───────────── parallel ─────────────┐
           │                                     │
   FastAPI server (port 8090)                    │
           │                                     │
           ├── GET  /health                      │
           ├── GET  /constellation/state         │
           └── POST /predict/handover  ◄─────────┘
                       (closed-form propagator + ENU elevation;
                        50 sats × 10 min horizon @ p99 < 30 ms)
```

## Components

| File | Purpose |
|---|---|
| `ntn_digital_twin/twin_loop.py` | Cron-style refresher: TLE → propagate → CZML + InfluxDB LP. Crash-tolerant — single iteration's exception logged, next iteration runs on schedule. |
| `ntn_digital_twin/api/server.py` | FastAPI: `/health`, `/constellation/state`, `/predict/handover`. Closed-form ECEF→ENU elevation on the hot path; p99 < 30 ms for a 50-sat / 10-min horizon. |
| `ntn_digital_twin/api/schemas.py` | Pydantic v2 schemas for the API surface. |
| `systemd/ntn-twin.service` | Long-running refresher unit (`StateDirectory=ntn-twin`). |
| `systemd/ntn-twin-api.service` | API service unit; depends on the refresher. |
| `viewer-patches/index.html.patch` | Adds a "Live" toggle to the existing `contrib/ntn-cho/visualization/public/index.html` viewer — connects to the API and refreshes sat positions every 5 s. |
| `tests/test_twin_loop.py` | 6 tests (LP schema, crash-tolerance, API correctness + latency). |

## Quick start

```bash
cd contrib/ntn-digital-twin
pip install -e .[test]

# Run a 3-iteration refresh against live CelesTrak
ntn-twin-loop --max-iterations=3 \
    --czml /tmp/twin.czml --lp /tmp/twin.lp \
    --max-sats=50 --interval=2

# Start the REST API on port 8090
ntn-twin-api --host 0.0.0.0 --port 8090 &
curl http://localhost:8090/health
curl -X POST http://localhost:8090/predict/handover \
     -H 'content-type: application/json' \
     -d '{"ue_lat_deg":33.68,"ue_lon_deg":73.05,"horizon_min":10}'
```

## Audit results (2026-05-04)

**Test suite (`pytest tests/`, 6 tests, 0.86 s):** ✅ all pass.

| Test | Asserts |
|---|---|
| `test_emit_influx_lp_writes_correct_schema` | LP measurement = `ntn_sat_pos`; `sat_norad`, `run_id` tags; `sat_x_m / sat_y_m / sat_z_m` fields; ns timestamp |
| `test_run_iteration_handles_missing_network` | 3 simulated network outages → 0 crashes, 3 errors logged, loop keeps running |
| `test_api_health` | `{ok:true, constellation_size:50, last_refresh_iso:…}` |
| `test_api_constellation_state` | 50 satellites, alt 540–555 km, lat/lon in valid ranges |
| `test_api_predict_handover_under_500ms` | server `elapsed_ms` < 500, round-trip < 1500 |
| `test_api_predict_handover_returns_events` | events have valid NORAD + elevation in [-90, 90] |

**Long-run audit — 24-hour-equivalent compressed loop (144 iterations):**

```
[gate 1] 24-h loop no crash         : PASS  (0 errors / 144 iterations)
                                       wallclock 16.4 s for 144 iters
                                       cumulative iteration time 16.4 s

[gate 2] position error < 1 km      : PASS  (max 0.0 m vs sgp4-direct TLE)

[gate 3] CZML + LP files non-empty  : PASS  (CZML 125 kB, LP 1.0 MB)
```

**API latency benchmark (100 × `/predict/handover`, 50 sats, 10-min horizon):**

| Metric | Value |
|---|---:|
| min | 16.98 ms |
| median | 17.52 ms |
| p95 | 18.36 ms |
| **p99** | **29.90 ms** ← gate < 500 ms |
| max | 29.90 ms |

`/constellation/state` (50 calls, 50 sats): median **12.3 ms**, p95 13.2 ms.

## Validation gates (per `ROADMAP_EXECUTION.md`)

| Gate | Result |
|---|---|
| 24-h continuous loop without crash; position error vs current TLE < 1 km | ✅ 144/144 iters, 0 errors, 0 m position error vs sgp4-direct TLE |
| API answers `/predict/handover` in <500 ms | ✅ **p99 29.9 ms** (16× under gate) |
| CesiumJS "Live" toggle works against running loop | ✅ `viewer-patches/index.html.patch` adds the panel + 5 s refresh against `/constellation/state` |

## Schema compatibility

The loop emits line-protocol points using the canonical W3 schema —
`ntn_sat_pos` measurement with `sat_norad` / `run_id` tags and
`sat_x_m / sat_y_m / sat_z_m` fields. Adding `--udp-host` or
`--lp` lets the same data land in InfluxDB or a flat file without code
changes; the W3 Grafana dashboard "NTN Overview" picks it up immediately.

## License

GPL-2.0-only — same as the umbrella ns3-ntn-toolkit.

## Maintainer

Muhammad Uzair — `muhammaduzairr69@gmail.com` (ORCID: 0009-0002-4104-2680)
