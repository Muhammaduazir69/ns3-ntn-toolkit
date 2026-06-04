# ntn-constellation

> Walker constellation generation, orbital propagation, contact-graph scheduling/routing, and a TR 38.821 + Starlink calibration corpus for ns-3 6G NTN research. Part of **ns3-ntn-toolkit** — [README](https://github.com/Muhammaduazir69/ns3-ntn-toolkit) / [INSTALL](INSTALL.md).

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0-green.svg"/></a>
  <img src="https://img.shields.io/badge/C%2B%2B%20propagator-Kepler%20%2B%20secular%20J2-orange.svg"/>
  <img src="https://img.shields.io/badge/Python-sgp4%20%2B%20Skyfield-orange.svg"/>
  <img src="https://img.shields.io/badge/presets-Starlink%20%E2%80%A2%20OneWeb%20%E2%80%A2%20Kuiper%20%E2%80%A2%20Iridium-purple.svg"/>
  <img src="https://img.shields.io/badge/exporters-SNS3%20%E2%80%A2%20CesiumJS-success.svg"/>
</p>

<p align="center">
  <img src="docs/ntn_constellation_demo.gif" alt="module live demo" width="900"/>
</p>

## Overview

`ntn-constellation` provides the orbital-mechanics foundation for the toolkit: it
generates LEO constellations, propagates them with high fidelity inside an ns-3
simulation, and turns the resulting time-varying geometry into the link-up/link-down
events that the rest of the data plane consumes. Everything lives in the
`ns3::ntncon` namespace.

- **Walker-Delta / Walker-Star generator** — `WalkerConstellation` builds a full
  constellation from a `WalkerConfig` (planes, satellites per plane, altitude,
  inclination), emitting SGP4-parseable orbital state.
- **Orbital propagation** — `Sgp4MobilityModel` is an ns-3 `MobilityModel` that
  propagates a satellite from a `TleRecord` (or `KeplerianElements`) using an
  analytic Kepler propagator with secular J2 corrections (RAAN and
  argument-of-perigee) and an SGP4-compatible TLE interface, so position queries
  during `Simulator::Run()` follow the orbit. Full Vallado SGP4 is a planned
  follow-on; the TLE drag (B*) field is parsed but not yet used by this model.
- **Contact-graph scheduling** — `ContactGraphScheduler` evaluates GSL (ground↔sat)
  and ISL (sat↔sat) visibility over the simulation timeline and raises
  `ContactEvent`s as links come up and go down.
- **Contact-graph routing** — `ContactGraphRouter` turns the contact graph into
  forwarding decisions across the time-varying topology.
- **Calibration corpus** — `tr38821-corpus` (`Tr38821CorpusReader`,
  `CalibrationHarness`) ships 3GPP TR 38.821 reference scenarios/link budgets plus
  a Starlink-EU latency/station corpus to validate toolkit predictions against
  published references.

This module also ships a pip-installable Python companion (`ntn_constellation`) for
the tool side — see [Python package](#python-package).

**Two propagators, two fidelity levels.** The in-simulation C++ model
(`Sgp4MobilityModel`) is an analytic Kepler + secular-J2 propagator with an
SGP4-compatible TLE interface — it is *not* a full SGP4 implementation yet. The
tool-side Python package (`ntn_constellation`) is different: it uses the real
`sgp4` library and Skyfield for canonical SGP4/SDP4 propagation when it generates
TLEs, ephemerides, and export files offline. So "SGP4" claims below apply to the
Python side; the C++ side is Kepler+J2 for now.

## What's new in v2

See [CHANGELOG.md](CHANGELOG.md) for this module's changelog.

- **New cross-module examples driving a REAL UDP data plane** via
  `NtnRealisticTrafficHelper` (from the `ntn-traffic` module), so packets actually
  traverse the time-varying topology instead of a static round-trip:
  - **`ntn-constellation-walker-traffic`** — a Walker-Delta constellation whose ISL
    connectivity is sampled every second from the `ContactGraphScheduler` and logged
    alongside the live UDP flow.
  - **`ntn-constellation-sgp4-mobility-traffic`** — a single LEO pass propagated by
    the `Sgp4MobilityModel` (Kepler + secular J2); the ground station is auto-placed
    under the satellite's t=0 sub-point so a real GSL up/down pass always occurs
    regardless of TLE epoch.

## Models, helpers & key classes

Derived from `model/*.h`:

| Header | Key types | Role |
|---|---|---|
| `walker-constellation.h` | `WalkerConfig`, `WalkerConstellation` | Walker-Delta / Walker-Star constellation generation (planes, sats/plane, altitude, inclination). |
| `sgp4-mobility-model.h` | `Sgp4MobilityModel` | ns-3 `MobilityModel` that propagates a satellite during the simulation via an analytic Kepler + secular-J2 propagator with an SGP4-compatible TLE interface (full Vallado SGP4 planned). |
| `orbital-elements.h` | `TleRecord`, `KeplerianElements` | TLE / Keplerian element records consumed by the mobility model. |
| `contact-graph-scheduler.h` | `ContactGraphScheduler`, `ContactEvent` | Computes GSL/ISL visibility and emits link up/down events over time. |
| `contact-graph-router.h` | `ContactGraphRouter` | Routes over the time-varying contact graph. |
| `tr38821-corpus.h` | `Tr38821CorpusReader`, `Tr38821Scenario`, `Tr38821LinkBudget`, `StarlinkLatencySample`, `StarlinkStation`, `CalibrationResidual` | TR 38.821 + Starlink calibration corpus and harness. |

## Examples

Built binaries land in `build/contrib/ntn-constellation/examples/` as
`ns3.43-<NAME>-default`. Each can be launched through `./ns3 run` (from the repo
root) or invoked directly.

### ntn-constellation-walker-traffic

A Walker-Delta constellation generated in-sim, with a real UDP data plane carried
by `NtnRealisticTrafficHelper`. The `ContactGraphScheduler` is sampled every second
to log live ISL connectivity changes alongside the flow.

```bash
# via ns3 (from repo root)
./ns3 run "ntn-constellation-walker-traffic --simSeconds=120 --numPlanes=6 --satsPerPlane=11 --altKm=550 --inclinationDeg=53 --outputDir=results/walker"
```

```bash
# direct binary
./build/contrib/ntn-constellation/examples/ns3.43-ntn-constellation-walker-traffic-default \
    --simSeconds=120 --numPlanes=6 --satsPerPlane=11 --altKm=550 \
    --inclinationDeg=53 --islRangeCapKm=5000 --outputDir=results/walker
```

**Outputs:** `sim_health.csv` (packets_tx, … health counters) in `--outputDir`, plus
a summary block printed to stdout:

```
# === ntn-constellation-walker-traffic summary ===
#   GS-A=(lat=35,lon=-75)  GS-B=(lat=35,lon=15)  ISL cap=<N> km
#   GSL up=<N> down=<N>  ISL up=<N> down=<N>
```

**Key args:** `--simSeconds`, `--numPlanes`, `--satsPerPlane`, `--altKm`,
`--inclinationDeg`, `--islRangeCapKm`, `--outputDir`.

### ntn-constellation-sgp4-mobility-traffic

A single LEO satellite (propagated by the `Sgp4MobilityModel`: Kepler + secular J2)
passing over a ground station, with a real
UDP data plane and GSL up/down sampling. If `--gsLat`/`--gsLon` are not supplied, the
ground station is auto-placed beneath the satellite's t=0 sub-point so a real GSL
up+down pass always occurs.

```bash
# via ns3 (from repo root)
./ns3 run "ntn-constellation-sgp4-mobility-traffic --simSeconds=600 --tle=contrib/ntn-rrc/data/iss-zarya.tle --outputDir=results/sgp4"
```

```bash
# direct binary (pin a fixed ground site)
./build/contrib/ntn-constellation/examples/ns3.43-ntn-constellation-sgp4-mobility-traffic-default \
    --simSeconds=600 --tle=contrib/ntn-rrc/data/iss-zarya.tle \
    --gsLat=33.6844 --gsLon=73.0479 --minElev=10 --outputDir=results/sgp4
```

**Outputs:** `sim_health.csv` in `--outputDir`, plus a summary block printed to
stdout:

```
# === ntn-constellation-sgp4-mobility-traffic summary ===
#   GSL up=<N>  down=<N>
```

**Key args:** `--simSeconds`, `--tle` (default: `contrib/ntn-rrc/data/iss-zarya.tle`),
`--gsLat` / `--gsLon` (default: satellite t=0 sub-point), `--minElev`, `--outputDir`.

### ntn-constellation-isl-routed-traffic

Real ISL-routed packet forwarding `GS1 -> satA -> [ISL] -> satB -> GS2` over a
point-to-point data plane with `FlowMonitor` instrumentation.

```bash
# via ns3 (from repo root)
./ns3 run ntn-constellation-isl-routed-traffic
```

```bash
# direct binary
./build/contrib/ntn-constellation/examples/ns3.43-ntn-constellation-isl-routed-traffic-default
```

## Python package

This module also ships a pip-installable Python companion, `ntn_constellation` —
a pure-Python, tool-side pipeline for **fresh ephemerides**, **canonical SGP4/SDP4
propagation**, and **export shapes** that drop into both the simulator (SNS3 scenario
layout) and the CesiumJS 3D viewer. It does not modify the C++ ns-3 build; it
produces inputs the simulator already understands.

- Built-in presets: **Starlink** (shells 1+2 + polar) · **OneWeb** · **Kuiper** ·
  **Telesat Lightspeed** · **Iridium NEXT**.
- Walker generators (`walker_delta`, `walker_star`) emitting valid SGP4-parseable TLEs.
- `Satellite` / `Constellation` API over Skyfield + raw `sgp4`; geodetic subpoint,
  ECI state vector, ground-station elevation/azimuth/range.
- ISL topology builders (`build_isl_topology` k-NN with range cap;
  `grid_isl_topology_walker` closed-form +grid for Walker layouts).
- Live feeds: CelesTrak (no credentials) and Space-Track
  (`SPACETRACK_USER`/`SPACETRACK_PASS`), with a TTL-based on-disk TLE cache (default 6 h).
- Exporters: `write_sns3_scenario` (SNS3 `positions/{tles,isls,start_date,gw_positions,ut_positions}.txt`)
  and `write_czml` (CesiumJS CZML).
- `ntn-fetch` CLI entry point for one-line scenario builds (preset or live).

Install and run:

```bash
git clone https://github.com/Muhammaduazir69/ntn-constellation.git contrib/ntn-constellation
cd contrib/ntn-constellation
python3 -m venv .venv
.venv/bin/pip install -e .[test]
```

```bash
# Live: current Starlink → SNS3 + Cesium
.venv/bin/ntn-fetch starlink --out data/starlink-now \
    --max-sats 200 --czml --czml-duration-min 120 --czml-step-sec 30 -v

# Preset (no internet): Kuiper Phase-1 Walker
.venv/bin/ntn-fetch kuiper --out data/kuiper --isl-walker --czml
```

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

The Python side ships 10 unit tests (TLE parsing, preset generation, SGP4
propagation, ISS visibility from Islamabad, ISL topology, SNS3 file layout) and is
validated against an independent Skyfield reference (24 h / 66 sats / 1440 samples:
0 NaN, 0.4 s wallclock; STARLINK-1008 1800 s pass: max 23.5 µs error, drift
0.006 µs/s). See [INSTALL.md](INSTALL.md) for the full Python setup including
CelesTrak/Space-Track environment variables.

## Build, run & test

From the repository root:

```bash
# Configure & build
./ns3 configure --enable-examples --enable-tests
./ns3 build

# Run an example
./ns3 run ntn-constellation-isl-routed-traffic

# Run this module's test suite
./test.py --suite=ntn-constellation
```

For prerequisites and per-module setup, see [INSTALL.md](INSTALL.md). For the full
toolkit build, see the [toolkit repository](https://github.com/Muhammaduazir69/ns3-ntn-toolkit).

## License & author

GPL-2.0-only — see [LICENSE](LICENSE).

**Muhammad Uzair**, Independent Researcher.

```bibtex
@misc{uzair2026ntnconstellation,
  author = {Uzair, Muhammad},
  title  = {ntn-constellation: Walker Constellation Generation, SGP4 Propagation,
            Contact-Graph Scheduling/Routing and a TR 38.821 Calibration Corpus
            for 6G NTN Research},
  year   = {2026},
  url    = {https://github.com/Muhammaduazir69/ntn-constellation}
}
```

## Acknowledgements

Brandon Rhodes (`sgp4`, Skyfield) · CelesTrak (Dr. T. S. Kelso) · Space-Track / 18th
Space Defense Squadron · CesiumJS · pytroll (`pyorbital`) · ns-3 core team · SNS3
maintainers.
