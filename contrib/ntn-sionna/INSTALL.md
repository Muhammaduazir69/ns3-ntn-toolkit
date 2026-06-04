# Install & run — ntn-sionna

`ntn-sionna` is an ns-3.43 contributed module that bridges NVIDIA Sionna RT
into ns-3 as an opt-in propagation loss model. It can be built on a vanilla
ns-3.43 tree or as part of the
[ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit).

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

### 2a. ns-3 modules (REQUIRED)

The module links the following ns-3 libraries (see `CMakeLists.txt`):

- `core`
- `network`
- `mobility`
- `propagation`
- `thz-ntn` — the sibling contrib module. Clone it under `contrib/thz-ntn`
  (it in turn needs the SNS3 `satellite` and `mmwave` modules; see its own
  `INSTALL.md`).

### 2b. Sionna RT GPU server (OPTIONAL)

`SionnaUdpTransport` talks to `bridge/sionna-server.py`, which needs CUDA,
TensorFlow, and Sionna RT. This is only required for live ray-traced figures.
Without it the channel falls back to closed-form FSPL, and the
`SionnaReplayTransport` / `SionnaCachingTransport` paths run fully headless.

### 2c. Stub server (for tests, no Sionna needed)

`test/sionna-stub-server.py` mimics the Sionna wire protocol using closed-form
FSPL, so the C++ test suite and the Python integration tests run without Sionna
RT installed.

---

## 3. Install the module

```bash
cd contrib/
git clone https://github.com/Muhammaduazir69/ntn-sionna.git ntn-sionna
cd ..
```

---

## 4. Configure & build

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build ntn-sionna
./ns3 show profile | grep ntn-sionna   # expect: ... ntn-sionna ...
```

---

## 5. Run examples

Every example tolerates a missing Sionna server (it falls back to FSPL), so the
following run as-is:

```bash
./ns3 run "leo-pass-sionna-vs-tr38811 --steps=30 --altKm=550 --freqHz=2e9"
./ns3 run "ntn-sionna-leo-downlink-traffic --simSeconds=30 --altKm=550 --rainMmH=10 --freqHz=12e9"
./ns3 run "ntn-sionna-ris-relay-traffic --risRows=32 --risCols=32 --blockageDb=20"
```

See the README for the full example list and arguments.

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
