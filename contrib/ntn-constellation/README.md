<h1 align="center">ntn-constellation</h1>

<p align="center"><strong>Live TLE feeds, SGP4/SDP4 propagation, ISL topology, and ns-3/CesiumJS exporters for the <a href="https://github.com/Muhammaduazir69/ns3-ntn-toolkit">ns3-ntn-toolkit</a>.</strong></p>

<p align="center">
  <em>Part of the v2.0 roadmap (Phase 1.3) — Constellation Database + Live TLE Feeds.</em>
</p>

---

## What it does

`ntn-constellation` is a pure-Python tool-side companion to the C++ ns-3 toolkit. It bridges live constellation data (CelesTrak / Space-Track) and Walker presets to:

1. **SNS3 scenarios** (`SatelliteSGP4MobilityModel`-compatible `positions/{tles,isls,start_date,gw_positions,ut_positions}.txt`).
2. **CesiumJS CZML** for the existing 3D viewer in `contrib/ntn-cho/visualization/`.

It does **not** modify the C++ ns-3 build. It produces input files the existing simulator already knows how to consume.

## What's inside

| File | Purpose |
|---|---|
| `feeds.py` | `CelesTrakFeed` (public, rate-limited, on-disk TTL cache) and `SpaceTrackFeed` (credentialed). Robust TLE parser with checksum-aware records. |
| `presets.py` | Named constellations (Starlink shells 1/2 + polar, OneWeb, Kuiper, Telesat Lightspeed, Iridium NEXT) and `walker_delta` / `walker_star` generators that emit valid SGP4-parseable TLEs. |
| `propagator.py` | `Satellite` / `Constellation` over Skyfield + raw `sgp4`. Geodetic subpoint, ECI state vector, ground-station elevation. |
| `isl.py` | `build_isl_topology` (k-nearest with range cap) and `grid_isl_topology_walker` (closed-form +grid for Walker layouts). |
| `ns3_export.py` | `write_sns3_scenario` — emits the exact directory layout SNS3's scenario loader expects. |
| `cesium_export.py` | `write_czml` — sampled `position` packets in ECEF, ready to drop into the CesiumJS viewer. |
| `cli.py` | `ntn-fetch` entry point. |

## 3rd-party packages used

This module operationalises six of the integrations called out in the v2.0 roadmap (Part 6):

- **[sgp4](https://pypi.org/project/sgp4/)** — Brandon Rhodes' reference SGP4/SDP4 propagator (Vallado et al.).
- **[skyfield](https://rhodesmill.org/skyfield/)** — high-precision astronomy / WGS-84 frame conversions.
- **[pyorbital](https://github.com/pytroll/pyorbital)** — pytroll's satellite orbital mechanics (used as a cross-check).
- **[CelesTrak](https://celestrak.org)** — public TLE feed (no credentials).
- **[Space-Track API](https://www.space-track.org)** — credentialed TLE / GP_HISTORY (login via `SPACETRACK_USER` / `SPACETRACK_PASS`).
- **[CesiumJS](https://cesium.com/cesiumjs/)** — the existing in-toolkit 3D viewer is fed via the `cesium_export.py` CZML writer.

## Quick start

```bash
cd contrib/ntn-constellation
python3 -m venv .venv
.venv/bin/pip install -e .
```

### Live: current Starlink → SNS3 + Cesium

```bash
.venv/bin/ntn-fetch starlink --out data/starlink-now \
    --max-sats 200 --czml --czml-duration-min 120 --czml-step-sec 30 -v
```

Outputs:
- `data/starlink-now/positions/{tles,isls,start_date}.txt` — drop into a new `contrib/satellite/data/scenarios/<name>/` tree.
- `data/starlink-now/constellation.czml` — open in the CesiumJS viewer (set `viewer.dataSources.add(Cesium.CzmlDataSource.load("constellation.czml"))`).

### Preset: Kuiper Phase-1 Walker (no internet)

```bash
.venv/bin/ntn-fetch kuiper --out data/kuiper --isl-walker --czml
```

Generates 1156 synthetic Kuiper TLEs (34 × 34) with closed-form +grid ISLs.

### Programmatic

```python
from datetime import datetime, timezone
from ntn_constellation import (
    CelesTrakFeed, TleCache, Constellation,
    build_isl_topology, write_sns3_scenario, write_czml,
)
from datetime import timedelta

feed = CelesTrakFeed(cache=TleCache("./.cache"))
tles = feed.fetch_group("oneweb")[:100]

c = Constellation.from_tles(tles)
when = datetime.now(tz=timezone.utc)

visible = c.visible_from(when, observer_lat_deg=33.6844,
                         observer_lon_deg=73.0479, min_elevation_deg=10)
print(f"{len(visible)} OneWeb sats above 10° elevation over Islamabad")

isls = build_isl_topology(c, when, k_nearest=4, max_range_km=5000.0)
write_sns3_scenario(scenario_dir="data/oneweb-sample", tles=tles,
                    start_date=when, isls=isls)
write_czml(constellation=c, start=when, duration=timedelta(hours=2),
           sample_step=timedelta(seconds=30),
           out_path="data/oneweb-sample/oneweb.czml")
```

## Tests

```bash
.venv/bin/pytest -v
```

10 unit tests cover TLE parsing, preset generation, SGP4 propagation, ISS visibility from Islamabad, ISL topology, and SNS3 file layout.

## Roadmap fit

| v2.0 roadmap line | Status here |
|---|---|
| 1.3 Constellation Database + Live TLE Feeds | core of this module |
| 1.3 Starlink / OneWeb / Kuiper / Telesat presets | `presets.py` |
| 1.3 Automated TLE update pipeline | `TleCache` (TTL-based, default 6h) |
| 1.3 SGP4/SDP4 propagator validation | Skyfield + sgp4 reference (JPL Horizons assertion: TODO) |
| 1.3 ISL topology generation | `isl.py` (k-NN + Walker grid) |
| 3.1 Digital Twin Mode | unblocked — feed live TLE → CZML on a cron loop |

## Audit results (2026-05-04)

Verified end-to-end as part of the W1–W4 integration audit (`AUDIT_W1_W4.md`):

| Check | Result |
|---|---|
| 24 h propagation, 66 sats, 1440 samples | 0 NaN, 0.4 s wallclock |
| Orbital period (Walker-Star Starlink-shell-1) | **96.00 min** (TLE-line-2 nominal 95.6) |
| Altitude bounds across 24 h | 542.9 – 554.8 km (drift end-vs-start +0.0 km) |
| ISL 4-NN graph across 24 hourly snapshots | 132 directed edges every snapshot, every node degree 4 |
| W1→W2 live CelesTrak STARLINK-1008 vs Skyfield (600 s) | mean &#124;err&#124; **2.6 µs**, max 5.8 µs |
| W1→W2 extended (1800 s) | max &#124;err&#124; 23.5 µs, drift &#124;err&#124;/dt **0.006 µs/s** |
| W1→W4 real Walker-Star → PyG → GAT (60 sats) | 90 % next-hop accuracy on real geometry |

The single Walker-Star shell exhibits **zero ISL edge churn** over 24 h —
expected analytically and confirmed by the propagator. The Skyfield-vs-SNS3
residual grows linearly at 0.006 µs/s (well under the 0.5 µs/s tolerance),
so hour-long Starlink scenarios are safe.

## License

GPL-2.0-only — same as the umbrella ns3-ntn-toolkit.

## Maintainer

Muhammad Uzair — `muhammaduzairr69@gmail.com` (ORCID: 0009-0002-4104-2680)
