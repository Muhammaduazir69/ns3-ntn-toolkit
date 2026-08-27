# Install & run — ntn-constellation

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

`ntn-constellation` is an ns-3.43 contributed module with two independent parts:
an in-simulation **C++ ns-3 module** and a tool-side **Python package**
(`ntn_constellation`). You can install either or both. The recommended way to
run the C++ side is inside the
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

---

## 2. Dependencies

### 2a. SNS3 `satellite` (REQUIRED)

The library links the SNS3 `satellite` module (its reference `sgp4unit` backs the
full-SGP4 path) — `ntn-constellation` will not register without it:

```bash
cd contrib/
git clone https://github.com/sns3/sns3-satellite.git satellite
cd ..
```

> Size note: SNS3 + bundled TLE data is ~3.7 GB.

### 2b. Toolkit modules `ntn-traffic` + `ntn-cho` (REQUIRED for the examples)

All four examples link the toolkit's `ntn-traffic` module (`NtnRealStackHelper`,
`NtnOranApplication`/`NtnOranSink`, the standards traffic applications) and
`ntn-cho` (`NtnTr38811MobilityHelper`, `NtnEnuProjectionMobilityModel`). Inside
`ns3-ntn-toolkit` both modules are already in `contrib/`; on a vanilla ns-3.43
tree they ship with the toolkit checkout.

### 2c. mmWave NR PHY (REQUIRED for the two access-link examples)

`ntn-constellation-walker-traffic` and `ntn-constellation-sgp4-mobility-traffic`
run real mmwave NR NTN cells, so `contrib/mmwave` (and its bundled `lte`
dependency) must be present. The two routed examples
(`ntn-constellation-isl-routed-traffic`, `ntn-constellation-real-routed`) do
**not** need it:

```bash
cd contrib/
git clone https://github.com/nyuwireless-unipd/ns3-mmwave.git mmwave
cd ..
```

---

## 3. Install the module

### Inside the toolkit (recommended)

Already present in `ns3-ntn-toolkit/contrib/ntn-constellation`. Clone the toolkit
(branch `ntn-integration-v2`):

```bash
git clone -b ntn-integration-v2 \
  https://github.com/Muhammaduazir69/ns3-ntn-toolkit.git
# GitLab mirror: https://gitlab.com/ns3-ntn-toolkit/ns3-ntn-toolkit
```

Docker image (everything pre-built): `uzairdocker69/ns3-ntn-toolkit:latest`
(or `:latest`).

### Standalone repo

Clone the standalone module into `contrib/ntn-constellation`, pinning its current
branch:

```bash
cd contrib/
git clone -b ntn-constellation-v2 \
  https://github.com/Muhammaduazir69/ntn-constellation.git ntn-constellation
cd ..
```

You still need the section 2 dependencies (`satellite`, `ntn-traffic`,
`ntn-cho`, and `mmwave` for the access-link examples) in `contrib/`.

---

## 4. Configure & build

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build ntn-constellation
./ns3 show profile | grep ntn-constellation   # expect: ... ntn-constellation ...
```

---

## 5. Run the examples

Built binaries land in `build/contrib/ntn-constellation/examples/` as
`ns3.43-<NAME>-default`.

### 5a. ntn-constellation-walker-traffic — Walker-Delta + one real mmwave cell

```bash
./ns3 run "ntn-constellation-walker-traffic --simSeconds=20 --numPlanes=4 --satsPerPlane=11 --altKm=550 --inclinationDeg=53 --outputDir=results/walker"
```
A Walker-Delta constellation provides the ephemeris/contact-graph context while
ONE serving satellite carries a real mmwave NR NTN cell to TR 38.811 ground UEs;
SINR/TBLER/throughput are measured off the mmwave PHY trace. Writes
`sim_health.csv` to `--outputDir`. Args: `simSeconds`, `numPlanes`,
`satsPerPlane`, `altKm`, `inclinationDeg`, `islRangeCapKm`, `numUes`,
`satEirpDbm`, `outputDir`.

### 5b. ntn-constellation-sgp4-mobility-traffic — single LEO pass, real mmwave cell

```bash
./ns3 run "ntn-constellation-sgp4-mobility-traffic --simSeconds=20 --tle=contrib/ntn-rrc/data/iss-zarya.tle --outputDir=results/sgp4"
```
A single LEO satellite from a TLE drives a real mmwave NR NTN cell toward TR
38.811 ground UEs, with GSL up/down sampling from the live geometry. If
`--gsLat`/`--gsLon` are omitted the ground station is auto-placed beneath the
satellite's t=0 sub-point. Writes `sim_health.csv`. Args: `simSeconds`, `tle`
(default `contrib/ntn-rrc/data/iss-zarya.tle`), `gsLat`, `gsLon`, `minElev`,
`numUes`, `satEirpDbm`, `outputDir`.

### 5c. ntn-constellation-isl-routed-traffic — ISL-routed forwarding

```bash
./ns3 run "ntn-constellation-isl-routed-traffic --duration=40"
```
Real Ipv4 forwarding `GS1 -> satA -> [ISL] -> satB -> GS2` with per-hop
propagation delay = real slant/ISL range over c and a geometry contact gate per
hop. Traffic is an `NtnOranApplication` CBR flow; delivery/delay/jitter/loss are
measured at the `NtnOranSink`. Writes `sim_health.csv`. Args: `duration`,
`altKm`, `satSpeed`, `minElev`, `islRangeCapKm`, `outputDir`.

### 5d. ntn-constellation-real-routed — adaptive reroute flagship

```bash
./ns3 run "ntn-constellation-real-routed --duration=40"
```
The geometry-driven routing decision actually INSTALLS Ipv4 static routes, so
real UDP packets are forwarded through the satellite nodes and adaptively reroute
as the constellation moves (`gs1 -> satA -> gs2`, then `gs1 -> satB -> gs2` once
satA sets and satB rises). Writes `sim_health.csv`. Args: `duration`, `altKm`,
`satSpeed`, `outputDir`.

---

## 6. Run the C++ unit tests

```bash
./test.py --suite=ntn-constellation
```
The suite has 20 unit tests covering TLE parsing/checksums, periodic-return and
ECEF-altitude propagation, Walker-Delta geometry, contact scheduling (LEO pass +
same-plane ISL pair), contact-graph routing (direct edges, BFS, weighted
Dijkstra, regenerative-only — including 600 s `Simulator::Run()` cases), and the
TR 38.821 + Starlink calibration harness.

---

## 7. Python package (`ntn_constellation`)

A pure-Python, offline pipeline (fresh ephemerides, canonical SGP4/SDP4
propagation via the `sgp4` and Skyfield libraries, SNS3 / CesiumJS export
shapes). It does not touch the ns-3 build.

```bash
cd contrib/ntn-constellation
python3 -m venv .venv
.venv/bin/pip install -e .[dev]
```

Runtime deps (from `pyproject.toml`): `sgp4`, `skyfield`, `pyorbital`,
`requests`. The `dev` extra adds `pytest` and `pytest-cov`.

Live feeds: CelesTrak needs no credentials; Space-Track needs `SPACETRACK_USER`
and `SPACETRACK_PASS`.

Run the Python tests (10 unit tests in `tests/test_basic.py`):

```bash
.venv/bin/pytest tests/
```

---

## 8. Common issues

**`ntn-constellation` not registered after configure** — the SNS3 `satellite`
module is missing (step 2a); the library will not register without
`contrib/satellite/`.

**Examples missing after configure** — the examples need `ntn-traffic` and
`ntn-cho` in `contrib/` (step 2b); the two access-link examples additionally need
`mmwave` (step 2c).

---

## 9. Uninstall

```bash
rm -rf contrib/ntn-constellation
./ns3 configure --enable-examples
./ns3 build
```