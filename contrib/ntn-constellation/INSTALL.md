# Installing ntn-constellation

This module has two independent parts: an in-simulation **C++ ns-3 module** and a
tool-side **Python package**. You can install either or both.

## C++ ns-3 module

### Dependencies

The library (`build_lib` in `CMakeLists.txt`) links only ns-3 `core` and
`mobility`. The three examples additionally use `network`, `internet`,
`applications`, `point-to-point`, `flow-monitor`, and the `ntn-traffic` contrib
module (two of them), so build the toolkit with those modules present.

### Build

Place this directory under `contrib/ntn-constellation/` in an ns-3.43 tree, then
from the repository root:

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build
```

### Run the examples

There are three examples (see `examples/CMakeLists.txt`):

```bash
# Walker-Delta constellation with a real UDP data plane, ISL sampling
./ns3 run ntn-constellation-walker-traffic

# Single LEO pass propagated by Sgp4MobilityModel (Kepler + secular J2)
./ns3 run "ntn-constellation-sgp4-mobility-traffic --tle=contrib/ntn-rrc/data/iss-zarya.tle"

# ISL-routed packet forwarding GS1 -> satA -> [ISL] -> satB -> GS2
./ns3 run ntn-constellation-isl-routed-traffic
```

### Run the C++ test suite

```bash
./test.py --suite=ntn-constellation
```

## Python package (ntn_constellation)

The Python companion is a pure-Python, offline pipeline (fresh ephemerides,
canonical SGP4/SDP4 propagation via the `sgp4` and Skyfield libraries, and SNS3 /
CesiumJS export shapes). It does not touch the ns-3 build.

### Install

```bash
cd contrib/ntn-constellation
python3 -m venv .venv
.venv/bin/pip install -e .[dev]
```

Runtime dependencies (from `pyproject.toml`): `sgp4`, `skyfield`, `pyorbital`,
`requests`. Optional `dev` extra adds `pytest` and `pytest-cov`.

### Environment variables (live feeds)

- CelesTrak needs no credentials.
- Space-Track needs `SPACETRACK_USER` and `SPACETRACK_PASS`.

### Run the Python tests

```bash
.venv/bin/pytest tests/
```

The Python side ships 10 unit tests (`tests/test_basic.py`).
