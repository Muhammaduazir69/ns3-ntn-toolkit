# Install & run — ntn-sionna

`ntn-sionna` is an ns-3.43 contributed module that bridges NVIDIA Sionna RT
into ns-3 as an opt-in propagation loss model. It can be built on a vanilla
ns-3.43 tree or as part of the
[ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit)
(branch `ntn-integration-v2`).

The module builds and runs without a GPU or a Sionna install. A live Sionna
GPU server is optional; the tests use a stub server instead.

---

## 1. System requirements

| Component | Version |
|---|---|
| OS | Linux (Ubuntu 22.04+ / Fedora 39+) |
| C++ compiler | gcc ≥ 11 or clang ≥ 14 |
| CMake | ≥ 3.24 |
| Python | ≥ 3.10 |
| ns-3 | **3.43** |

---

## 2. Dependencies

### 2a. ns-3 modules (REQUIRED for the library)

The library links the following ns-3 libraries (see `CMakeLists.txt`):

- `core`
- `network`
- `mobility`
- `propagation`
- `thz-ntn` — the sibling contrib module. Clone it under `contrib/thz-ntn`
  (it in turn needs the SNS3 `satellite` and `mmwave` modules; see its own
  `INSTALL.md`):

```bash
cd contrib/
git clone https://github.com/Muhammaduazir69/ns3-thz-ntn.git thz-ntn
cd ..
```

### 2b. Toolkit modules (REQUIRED for the measured-radio examples)

The measured-radio examples (`ntn-sionna-leo-downlink-traffic`,
`ntn-sionna-rain-event-traffic`, `ntn-sionna-ris-relay-traffic`,
`ntn-sionna-mimo-traffic`, `ntn-sionna-constellation-handover-traffic`,
`ntn-sionna-composed-channel-traffic`, `ntn-sionna-cir-real-stack`) run a real
mmwave NR NTN cell via `NtnRealStackHelper` and project SGP4 Walker mobility
into the local ENU frame. They additionally link `ntn-traffic`, `ntn-cho`,
`ntn-constellation`, `oran-ntn`, and the in-tree `mmwave` stack (with its
bundled `lte`). All are already present inside the `ns3-ntn-toolkit` tree; on a
vanilla ns-3.43 tree clone `mmwave`
(`https://github.com/nyuwireless-unipd/ns3-mmwave.git mmwave`) and the toolkit
modules into `contrib/`. The library itself and the physics/channel examples
build without them.

### 2c. Sionna RT GPU server (OPTIONAL)

`SionnaUdpTransport` talks to `bridge/sionna-server.py`, which needs CUDA,
TensorFlow, and Sionna RT. This is only required for live ray-traced figures.
Without it the channel falls back to closed-form FSPL, and the
`SionnaReplayTransport` / `SionnaCachingTransport` paths run fully headless.

### 2d. Stub server (for tests, no Sionna needed)

`test/sionna-stub-server.py` mimics the Sionna wire protocol using closed-form
FSPL, so the C++ test suite and the Python integration tests run without Sionna
RT installed.

---

## 3. Install the module

```bash
cd contrib/
git clone -b ntn-sionna-v2 https://github.com/Muhammaduazir69/ntn-sionna.git ntn-sionna
cd ..
```

> GitLab mirror / Docker: the whole toolkit (with `ntn-sionna` already in
> `contrib/`) is mirrored at
> [gitlab.com/ns3-ntn-toolkit/ns3-ntn-toolkit](https://gitlab.com/ns3-ntn-toolkit/ns3-ntn-toolkit)
> and shipped as `uzairdocker69/ns3-ntn-toolkit:2.2.1` (or `:latest`):
>
> ```bash
> docker pull uzairdocker69/ns3-ntn-toolkit:2.2.1
> docker run -it uzairdocker69/ns3-ntn-toolkit:2.2.1
> ```

---

## 4. Configure & build

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build ntn-sionna
./ns3 show profile | grep ntn-sionna   # expect: ... ntn-sionna ...
```

---

## 5. Run examples

There are **12 examples** in two groups. Every example tolerates a missing
Sionna server (the channel falls back to closed-form FSPL), so they all run
as-is with no GPU.

### 5a. Measured-radio examples (real packet plane)

These drive a real mmwave NR NTN cell through `NtnRealStackHelper`
(SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC) with the channel physics
chained into the packet path as real `PropagationLossModel`s and
`NtnOranApplication` QoS flows; SINR / TBLER / goodput are measured off the PHY
trace. They need the toolkit/mmwave modules from section 2b.

```bash
./ns3 run "ntn-sionna-leo-downlink-traffic --simSeconds=60 --freqGHz=12 --rainMmH=10 --lms=1 --outputDir=/tmp/dl"
./ns3 run "ntn-sionna-rain-event-traffic --simSeconds=60 --peakRainMmH=40 --outputDir=/tmp/rain"
./ns3 run "ntn-sionna-ris-relay-traffic --risRows=32 --risCols=32 --blockageDb=30 --outputDir=/tmp/ris"
./ns3 run "ntn-sionna-mimo-traffic --rows=4 --cols=4 --outputDir=/tmp/mimo"
./ns3 run "ntn-sionna-constellation-handover-traffic --numSats=8 --hysteresisDb=2 --outputDir=/tmp/cho"
./ns3 run "ntn-sionna-composed-channel-traffic --rainRateMmH=10 --outputDir=/tmp/comp"
./ns3 run "ntn-sionna-cir-real-stack --duration=30 --numUes=2 --altitude=550 --outputDir=/tmp/cir"
```

Each writes per-second trace tables, per-run KPI files, and a `sim_health.csv`
gate report under `--outputDir`.

### 5b. Physics / channel examples (no data plane)

These exercise the channel models, link budget, caching, and calibration
directly. The calibration harness drives the real packet plane only when a
Sionna server answers; otherwise it reports the failure count cleanly.

```bash
./ns3 run "leo-pass-sionna-vs-tr38811 --steps=30 --altKm=550 --freqHz=2e9"
./ns3 run "mmimo-vs-codebook-leo --rows=4 --cols=4 --rainMmH=10 --steps=30"
./ns3 run "ris-assisted-leo-link --risRows=32 --risCols=32 --steps=30"
./ns3 run "city-block-4ue-cache --steps=60 --spatialResM=50 --timeBucketUs=100000"
./ns3 run "sionna-calibration-harness --host=127.0.0.1 --port=8765 --losOnly=true"
```

See the README for the full per-example argument tables.

---

## 6. Run the tests

```bash
./test.py -s ntn-sionna                 # C++ suite (38 test cases)
pytest contrib/ntn-sionna/test/         # Python integration tests (3 tests)
```

The Python tests start `test/sionna-stub-server.py` automatically, so no Sionna
RT or GPU is required.

---

## 7. Common issues

**`ntn-sionna` not registered after configure** — the `thz-ntn` module is
missing; clone it under `contrib/thz-ntn` (and its own dependencies).

**Measured-radio examples missing after configure** — the `ntn-sionna-*-traffic`
and `ntn-sionna-cir-real-stack` examples need `ntn-traffic`, `ntn-cho`,
`ntn-constellation`, `oran-ntn`, and `mmwave` in `contrib/` (section 2b). The
library and the physics/channel examples build without them.

**`SionnaUdpTransport` times out** — no live Sionna server is answering. This
is expected without a GPU; the channel falls back to closed-form FSPL. Start
`bridge/sionna-server.py` (or use the replay/caching transports) for ray-traced
results.

---

## 8. Uninstall

```bash
rm -rf contrib/ntn-sionna
./ns3 configure --enable-examples
./ns3 build
```
