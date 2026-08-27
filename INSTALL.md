# Install and run: ns3-ntn-toolkit

From a fresh machine to a running 6G non-terrestrial network simulation. Three
paths, in increasing order of effort and control:

| Path | Time | Use it when |
|---|---|---|
| [Docker](#1-docker-the-fastest-path) | 5 minutes | You want to run a scenario, reproduce a result, or try the toolkit out |
| [Source build](#2-source-build) | 30 to 60 minutes | You are going to modify a module or write a new scenario |
| [Single module](#3-installing-one-module-into-an-existing-ns-3-tree) | 10 minutes | You already have an ns-3 tree and want one capability from this one |

---

## 1. Docker, the fastest path

The published image carries a fully built tree: ns-3.43, all fourteen custom
modules, the SNS3 `satellite` stack, the mmWave and 5G-LENA `nr` NR stacks, the
Python tooling and the digital-twin server. Nothing to compile.

```bash
docker pull uzairdocker69/ns3-ntn-toolkit:latest

# Run a scenario and keep its output
docker run --rm -v "$PWD/out:/out" uzairdocker69/ns3-ntn-toolkit:latest \
  ./ns3 run "ntn-cho-real-stack --trigger=d2 --duration=60 --outputDir=/out"

# Or work inside the built tree
docker run --rm -it -v "$PWD/out:/out" uzairdocker69/ns3-ntn-toolkit:latest bash
```

This is also the **reproducible path**. The SNS3 `satellite` tree is a
compile-time dependency that this repository does not vendor, so a clone plus a
build is not a single reproducible step, while the image is.

Version-pinned tags are published alongside `latest`. Pin one in any paper or
artifact statement rather than citing `latest`, which moves.

---

## 2. Source build

### 2.1 System requirements

| Component | Version |
|---|---|
| OS | Linux. Ubuntu 22.04 or later, Fedora 39 or later |
| C++ compiler | gcc 11 or newer, or clang 14 or newer |
| CMake | 3.24 or newer |
| Python | 3.10 or newer, tested through 3.13 |
| Boost | 1.74 or newer, for interprocess |
| Disk | 20 GB after a build, including the SNS3 TLE corpus and outputs |
| RAM | 8 GB to build, 4 GB to run |

An NVIDIA GPU is optional and only needed for `ntn-sionna` ray tracing. Every
other module runs on CPU.

### 2.2 Toolchain

Ubuntu and Debian:

```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build git python3 python3-pip \
    libboost-all-dev libgsl-dev libxml2-dev libsqlite3-dev libpcap-dev \
    libeigen3-dev pybind11-dev libprotobuf-dev protobuf-compiler
```

`libeigen3-dev` is needed by the NR MIMO path, and `pybind11-dev` plus the
protobuf packages by the `ns3-ai-ntn` shared-memory bridge. Everything else is
the standard ns-3 toolchain.

Python packages, only if you will use the reinforcement-learning bridge, the
digital twin or the figure pipeline:

```bash
pip install "numpy>=2.0" "gymnasium>=1.0" "torch>=2.0" \
            matplotlib pandas fastapi uvicorn sgp4
```

### 2.3 Clone

```bash
git clone --branch ntn-integration-v2 \
  https://github.com/Muhammaduazir69/ns3-ntn-toolkit.git
cd ns3-ntn-toolkit
```

`ntn-integration-v2` is the current release line. The GitLab mirror carries the
same branch: <https://gitlab.com/ns3-ntn-toolkit/ns3-ntn-toolkit>.

### 2.4 Add the SNS3 `satellite` module, which is required

The SNS3 satellite module is not bundled, because its TLE and antenna-pattern
corpus is around 3.7 GB. Clone it into `contrib/`:

```bash
git clone https://github.com/sns3/sns3-satellite.git contrib/satellite
```

Without it, `ntn-cho`, `ntn-constellation`, `oran-ntn` and `thz-ntn` will not
register, and the build will simply not contain them rather than failing loudly.

### 2.5 Configure and build

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build
```

Confirm the modules registered:

```bash
./ns3 show profile | grep -E 'ntn-|oran-ntn|thz-ntn|satellite|mmwave|nr'
```

If a stale build cache filters modules out, force a clean module set. An empty
`--enable-modules=''` re-enables every autoloaded module:

```bash
./ns3 configure --enable-modules='' --enable-tests --enable-examples
```

### 2.6 First run

```bash
./ns3 run ntn-real-stack-smoke
```

That builds one NR NTN cell under satellite mobility, runs traffic across it, and
writes a health record. If it prints measured SINR and a nonzero throughput, the
tree is working.

Then the cross-module flagship:

```bash
./ns3 run "ntn-e2e-full-stack --duration=60 --numUes=8 --altitude=600 --outputDir=out/"
```

One real NR NTN cell under SGP4 mobility whose measured PHY SINR feeds the O-RAN
E2SM-KPM flow monitor, the near-real-time RIC xApps and the in-band QoS sink at
the same time, exercising `ntn-traffic`, `oran-ntn`, `ntn-constellation` and the
vendored `mmwave` and `satellite` stacks in one binary.

---

## 3. Installing one module into an existing ns-3 tree

Twelve of the fourteen modules are also published standalone. Drop one into your
own `contrib/` and reconfigure. Each carries its own `README.md` and `INSTALL.md`
listing what else it needs.

| Module | Standalone repository | Branch |
|---|---|---|
| `ntn-cho` | `Muhammaduazir69/ntn-cho-framework` | `main` |
| `oran-ntn` | `Muhammaduazir69/oran-ntn` | `oran-ntn-v2` |
| `thz-ntn` | `Muhammaduazir69/ns3-thz-ntn` | `thz-ntn-v2` |
| `ntn-constellation` | `Muhammaduazir69/ntn-constellation` | `ntn-constellation-v2` |
| `ntn-rrc` | `Muhammaduazir69/ntn-rrc` | `ntn-rrc-v2` |
| `ntn-sagin` | `Muhammaduazir69/ntn-sagin` | `ntn-sagin-v2` |
| `ntn-sionna` | `Muhammaduazir69/ntn-sionna` | `ntn-sionna-v2` |
| `ntn-slice` | `Muhammaduazir69/ntn-slice` | `ntn-slice-v2` |
| `ntn-v2x` | `Muhammaduazir69/ntn-v2x` | `ntn-v2x-v2` |
| `ntn-observability` | `Muhammaduazir69/ntn-observability` | `ntn-observability-v2` |
| `ntn-digital-twin` | `Muhammaduazir69/ntn-digital-twin` | `ntn-digital-twin-v2` |
| `ns3-ai-ntn` | `Muhammaduazir69/ns3-ai` | `fix/ns3-43-compatibility-and-critical-bugs` |
| `ntn-fapi` | bundled only | |
| `ntn-traffic` | bundled only | |

Each repository is mirrored on GitLab under
<https://gitlab.com/ns3-ntn-toolkit>.

```bash
git clone -b oran-ntn-v2 \
  https://github.com/Muhammaduazir69/oran-ntn.git contrib/oran-ntn
./ns3 configure --enable-modules='' --enable-examples --enable-tests
./ns3 build
```

`ntn-traffic` and `ntn-fapi` ship only inside this toolkit tree. `ntn-traffic` in
particular is the spine most other modules' examples build on, so a standalone
module usually needs this toolkit rather than a bare ns-3.

---

## 4. Verifying the install

### 4.1 The standards gates

```bash
python3 tools/check_ntn_standards.py
```

Sixteen gates: the TR 38.821 Set-1 LEO-600 link-budget calibration, orbital
geometry, the published platform-latency bands, all five NTN handover trigger
classes, and the documentation-claim checks. It exits nonzero on any failure, so
it works as a CI step.

```bash
python3 tools/check_protocol_fidelity.py    # every example runs on a measured radio plane
python3 tools/check_doc_claims.py           # no document claims what the code contradicts
python3 tools/check_dashboard_producers.py  # every dashboard panel traces back to a producer
```

### 4.2 The test suites

```bash
./test.py -s ntn-cho
./test.py -s oran-ntn
./test.py -s thz-ntn
./test.py -s ntn-constellation
./test.py -s ntn-standards-validation
```

`./test.py` with no arguments runs everything including the upstream ns-3 suites,
which takes a long time.

---

## 5. Per-module examples

| Module | Try this |
|---|---|
| `ntn-cho` | `./ns3 run "ntn-cho-real-stack --trigger=d2 --duration=60"` |
| `ntn-cho` | `./ns3 run "ntn-cho-full-constellation --algorithm=tte-aware --simTime=600"` |
| `oran-ntn` | `./ns3 run "oran-ntn-full-scenario --simTime=600 --xapps=ho,beamhop,slice,doppler,tnntn"` |
| `oran-ntn` | `./ns3 run oran-ntn-ric-placement-ab` |
| `ntn-traffic` | `./ns3 run ntn-tr38821-calibration` |
| `ntn-traffic` | `./ns3 run ntn-oran-qos-flows` |
| `ntn-rrc` | `./ns3 run ntn-rrc-real-stack` |
| `thz-ntn` | `./ns3 run "thz-ntn-demo --example=8"` |
| `ntn-v2x` | `./ns3 run "ntn-v2x-pc5-sidelink-bsm --numVehicles=20 --duration=4"` |
| `ntn-sagin` | `./ns3 run sagin-a2g-real-stack` |
| `ntn-sionna` | `./ns3 run "leo-pass-sionna-vs-tr38811 --sionnaServer=127.0.0.1:8899"` |
| `ntn-fapi` | `./ns3 run ntn-fapi-real-stack` |
| `ns3-ai-ntn` | `cd contrib/ns3-ai-ntn/examples/a-plus-b/use-gym/ && python3 a-plus-b.py` |

Every example writes a standardized bundle to `--outputDir`: `sim_health.csv`
with a provenance label on every row, the per-example KPI CSVs, and where the
module exports telemetry, an E2SM-KPM series under TS 28.552 names.

---

## 6. Optional components

### 6.1 GPU ray tracing with Sionna RT

`ntn-sionna` talks to a Sionna RT server over a socket, so the GPU does not have
to be on the same machine as the simulation.

```bash
pip install sionna
python3 contrib/ntn-sionna/bridge/sionna-server.py --port 8899
./ns3 run "leo-pass-sionna-vs-tr38811 --sionnaServer=127.0.0.1:8899"
```

Without a reachable server the bridge falls back to the closed-form TR 38.811
model, warns once, and every run prints which path produced its numbers. If a
traced result is load-bearing for your experiment, set `RequireLiveTransport` so
a missing server aborts instead of silently substituting free space.

### 6.2 Telemetry: InfluxDB, Grafana, NetSimulyzer, Cesium

`ntn-observability` writes InfluxDB line protocol, NetSimulyzer JSON and CZML for
a Cesium globe from one scene recorder. The shipped Grafana dashboards are under
`contrib/ntn-observability/dashboards/`, and `tools/check_dashboard_producers.py`
verifies that every panel traces back to code that actually emits its measurement.

### 6.3 The digital twin service

```bash
pip install fastapi uvicorn sgp4
python3 -m uvicorn ntn_digital_twin.api.server:app --port 8000
```

Predicts handovers from live ephemeris and can actuate them back into a running
simulation. Prediction and actuation share one guard implementation, so they
cannot drift apart.

### 6.4 Constellation TLE data

The SNS3 corpus under `contrib/satellite/data/constellations/` ships real
two-line elements for Iridium-NEXT (66 satellites, 780 km, 86.4 degrees),
Starlink shell 1 (1,584 satellites, 550 km, 53 degrees) and Kuiper 1
(1,156 satellites, 590 km, 33 degrees).

```bash
cd contrib/satellite/data/constellations/ && ./refresh_tle.sh
```

`ntn-constellation` can also pull live elements from CelesTrak.

---

## 7. Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `SatMobilityModel not found` | The SNS3 `satellite` clone in [2.4](#24-add-the-sns3-satellite-module-which-is-required) was skipped. It is required. |
| A module is missing from `./ns3 show profile` | A stale build cache filtered it. Run `./ns3 configure --enable-modules=''`. |
| `MmWaveAmc not found` | `contrib/mmwave/` is absent or was not configured. Reconfigure. |
| `MmWaveComponentCarrierConf already defined` | Two copies of mmWave are in `contrib/`. Delete one. |
| Eigen or MIMO build errors under `nr` | `libeigen3-dev` is missing. Install it and reconfigure. |
| ns3-ai `ImportError: dynamic module does not define module export function` | An LTO issue in the upstream ns3-ai. The bundled `contrib/ns3-ai-ntn/` fork already carries the fix; make sure you are not shadowing it with an upstream clone. |
| A Sionna example prints free-space numbers | No server was reachable. See [6.1](#61-gpu-ray-tracing-with-sionna-rt); the run says so in its provenance line. |
| A scenario prints zeros for a KPI | Check `sim_health.csv` in the output directory. Its provenance column says whether the value was measured, modeled or configured. |

---

## 8. Where to go next

- **[README.md](README.md)** for what each module does and the measured results
- **[SCOPE_AND_LIMITATIONS.md](SCOPE_AND_LIMITATIONS.md)** for what the toolkit
  deliberately does not model, which is worth reading before you design an
  experiment around it
- **[CONTRIBUTING.md](CONTRIBUTING.md)** for adding a module or a scenario
- `contrib/<module>/README.md` and `contrib/<module>/INSTALL.md` for per-module
  detail
- The [documentation site](https://muhammaduazir69.github.io/ns3-ntn-toolkit/)
