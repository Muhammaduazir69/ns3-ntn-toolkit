<h1 align="center">ntn-constellation</h1>

<p align="center"><strong>Constellation Database, Live TLE Feeds and SGP4/SDP4 Propagation for 6G NTN Research</strong></p>

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0-green.svg"/></a>
  <img src="https://img.shields.io/badge/SGP4%2FSDP4-Vallado%20et%20al.-orange.svg"/>
  <img src="https://img.shields.io/badge/presets-Starlink%20%E2%80%A2%20OneWeb%20%E2%80%A2%20Kuiper%20%E2%80%A2%20Iridium-purple.svg"/>
  <img src="https://img.shields.io/badge/exporters-SNS3%20%E2%80%A2%20CesiumJS-success.svg"/>
</p>

---

## Why this module

Reproducible NTN research requires three things that, separately, are easy and, together, are surprisingly painful: **fresh ephemerides** (today's TLEs, not last year's), a **propagator that an ns-3 simulation actually trusts** (SGP4/SDP4 with frame conversions that don't drift), and **export shapes** that drop into both the simulator (SNS3 scenario layout) and the visualiser (CesiumJS CZML). `ntn-constellation` is a pure-Python, tool-side companion that produces all three in one pass — pulling from CelesTrak or Space-Track, propagating with the canonical Vallado SGP4 reference, and emitting the exact files that `SatSGP4MobilityModel` and the in-toolkit 3D viewer already know how to consume. It does not modify the C++ ns-3 build; it produces inputs the existing simulator already understands.

## At a glance

| Metric | Value |
|---|---|
| Built-in presets | **Starlink** (shells 1+2 + polar) · **OneWeb** · **Kuiper** · **Telesat Lightspeed** · **Iridium NEXT** |
| Walker generators | `walker_delta` · `walker_star` (emit valid SGP4-parseable TLEs) |
| Propagator backends | `sgp4` (Brandon Rhodes / Vallado) + `skyfield` cross-check |
| ISL topology builders | k-NN with range cap · closed-form Walker +grid |
| Live feeds | CelesTrak (no credentials) · Space-Track (`SPACETRACK_USER`/`PASS`) |
| TLE cache | TTL-based, default 6 h, on-disk |
| **24 h propagation, 66 sats, 1440 samples** | **0 NaN, 0.4 s wallclock** |
| **W1→W2 vs Skyfield** (1800 s pass, STARLINK-1008) | **max 23.5 µs, drift 0.006 µs/s** |
| **W1→W4 GAT next-hop accuracy** (60 sats, real Walker-Star geometry) | **90 %** |

## What it does

- 3GPP-compliant ephemeris pipeline: live CelesTrak / Space-Track fetch with on-disk TTL cache, robust TLE parser with checksum-aware records.
- Six built-in named constellations and two Walker generators emitting valid SGP4-parseable TLEs (used as the input layer for every other toolkit module).
- `Satellite` / `Constellation` API over Skyfield + raw `sgp4` for sub-millisecond per-tick propagation; geodetic subpoint, ECI state vector, ground-station elevation/azimuth/range.
- ISL topology generation (`build_isl_topology` k-nearest with range cap; `grid_isl_topology_walker` closed-form +grid for Walker layouts) — verified zero edge churn over 24 h on a Walker-Star shell.
- Two exporter sinks: `write_sns3_scenario` emits the exact `positions/{tles,isls,start_date,gw_positions,ut_positions}.txt` layout `SatSGP4MobilityModel` (SNS3) consumes, and `write_czml` emits sampled-position CesiumJS CZML for the in-toolkit 3D viewer.
- `ntn-fetch` CLI entry point for one-line scenario builds (preset or live).

## Install & run

```bash
git clone https://github.com/Muhammaduazir69/ntn-constellation.git contrib/ntn-constellation
cd contrib/ntn-constellation
python3 -m venv .venv
.venv/bin/pip install -e .[test]
```

Live: current Starlink → SNS3 + Cesium

```bash
.venv/bin/ntn-fetch starlink --out data/starlink-now \
    --max-sats 200 --czml --czml-duration-min 120 --czml-step-sec 30 -v
```

Preset (no internet): Kuiper Phase-1 Walker

```bash
.venv/bin/ntn-fetch kuiper --out data/kuiper --isl-walker --czml
```

Programmatic:

```python
from datetime import datetime, timedelta, timezone
from ntn_constellation import (
    CelesTrakFeed, TleCache, Constellation,
    build_isl_topology, write_sns3_scenario, write_czml,
)

feed = CelesTrakFeed(cache=TleCache("./.cache"))
tles = feed.fetch_group("oneweb")[:100]

c = Constellation.from_tles(tles)
when = datetime.now(tz=timezone.utc)

visible = c.visible_from(when, observer_lat_deg=33.6844,
                         observer_lon_deg=73.0479, min_elevation_deg=10)
isls = build_isl_topology(c, when, k_nearest=4, max_range_km=5000.0)
write_sns3_scenario(scenario_dir="data/oneweb-sample", tles=tles,
                    start_date=when, isls=isls)
write_czml(constellation=c, start=when, duration=timedelta(hours=2),
           sample_step=timedelta(seconds=30),
           out_path="data/oneweb-sample/oneweb.czml")
```

## Verification

10 unit tests cover TLE parsing, preset generation, SGP4 propagation, ISS visibility from Islamabad, ISL topology, and SNS3 file layout. End-to-end pipeline numbers measured against an independent Skyfield reference:

| Check | Result |
|---|---|
| 24 h propagation, 66 sats, 1440 samples | 0 NaN, 0.4 s wallclock |
| Orbital period (Walker-Star Starlink-shell-1) | **96.00 min** (TLE-line-2 nominal 95.6) |
| Altitude bounds across 24 h | 542.9 – 554.8 km (drift end-vs-start +0.0 km) |
| ISL 4-NN graph across 24 hourly snapshots | 132 directed edges every snapshot, every node degree 4 |
| Live CelesTrak STARLINK-1008 vs Skyfield (600 s) | mean &#124;err&#124; **2.6 µs**, max 5.8 µs |
| Extended pass (1800 s) | max &#124;err&#124; 23.5 µs, drift &#124;err&#124;/dt **0.006 µs/s** |
| Real Walker-Star → PyG → GAT next-hop accuracy (60 sats) | **90 %** on real geometry |

The single Walker-Star shell exhibits zero ISL edge churn over 24 h — expected analytically and confirmed by the propagator. The Skyfield-vs-SNS3 residual grows linearly at 0.006 µs/s (well under the 0.5 µs/s tolerance), so hour-long Starlink scenarios stay tightly aligned with ground-truth ephemerides.

## Documentation

- [INSTALL.md](INSTALL.md) — full setup with the CelesTrak/Space-Track environment variables.
- Reference: B. Rhodes, *sgp4 — Python implementation of SGP4 by Vallado et al.*, https://pypi.org/project/sgp4/
- 3GPP TR 38.821 §6.1 — Reference satellite parameters.

## Cite this work

```bibtex
@misc{uzair2026ntnconstellation,
  author = {Uzair, Muhammad},
  title  = {ntn-constellation: Live TLE Feeds, SGP4/SDP4 Propagation and SNS3/CesiumJS Exporters for 6G NTN Research},
  year   = {2026},
  url    = {https://github.com/Muhammaduazir69/ntn-constellation}
}
```

## Part of the ns3-ntn-toolkit

This module is part of [**ns3-ntn-toolkit**](https://github.com/Muhammaduazir69/ns3-ntn-toolkit) — a pre-integrated ns-3.43 distribution for 6G NTN research:

| Module | Repo |
|---|---|
| Toolkit (umbrella) | [ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit) |
| **ntn-constellation** | this repo |
| ntn-rrc | [ntn-rrc](https://github.com/Muhammaduazir69/ntn-rrc) |
| ntn-observability | [ntn-observability](https://github.com/Muhammaduazir69/ntn-observability) |
| ns3-ai (fork) | [ns3-ai](https://github.com/Muhammaduazir69/ns3-ai) |
| ntn-sagin | [ntn-sagin](https://github.com/Muhammaduazir69/ntn-sagin) |
| ntn-slice | [ntn-slice](https://github.com/Muhammaduazir69/ntn-slice) |
| ntn-v2x | [ntn-v2x](https://github.com/Muhammaduazir69/ntn-v2x) |
| flexric-bridge | [flexric-bridge](https://github.com/Muhammaduazir69/flexric-bridge) |
| ntn-sionna | [ntn-sionna](https://github.com/Muhammaduazir69/ntn-sionna) |
| ntn-digital-twin | [ntn-digital-twin](https://github.com/Muhammaduazir69/ntn-digital-twin) |
| ntn-cho | [ntn-cho-framework](https://github.com/Muhammaduazir69/ntn-cho-framework) |
| oran-ntn | [oran-ntn](https://github.com/Muhammaduazir69/oran-ntn) |
| thz-ntn | [ns3-thz-ntn](https://github.com/Muhammaduazir69/ns3-thz-ntn) |

## License

GPL-2.0-only — see [LICENSE](LICENSE).

## Acknowledgements

Brandon Rhodes (`sgp4`, Skyfield) · CelesTrak (Dr. T. S. Kelso) · Space-Track / 18th Space Defense Squadron · CesiumJS · pytroll (`pyorbital`) · ns-3 core team · SNS3 maintainers.
