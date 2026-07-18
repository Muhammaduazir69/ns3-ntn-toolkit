# Install & run — ntn-v2x

`ntn-v2x` is an ns-3.43 contributed module. The recommended way to run it is
inside the [ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit)
tree (branch `ntn-integration-v2`), where every dependency below is already
present. It also builds on a vanilla ns-3.43 tree — the library links only core
ns-3 modules; the examples additionally need the sibling toolkit modules listed
in section 2.

---

## 1. System requirements

| Component | Version |
|---|---|
| OS | Linux (Ubuntu 22.04+ / Fedora 39+ recommended) |
| C++ compiler | gcc ≥ 11 or clang ≥ 14 |
| CMake | ≥ 3.24 |
| Python | ≥ 3.10 |
| ns-3 | **3.43** |

SUMO is **not** required — FCD replay is the supported mobility path (see the
README). The live SUMO TraCI path uses only standard POSIX sockets.

---

## 2. Dependencies

### 2a. Library

The module library (`CMakeLists.txt`) links only core ns-3 modules: `core`,
`network`, `mobility`. No external libraries are required.

### 2b. Toolkit siblings (REQUIRED for the examples)

The real-stack examples run a real mmwave NR LEO cell with measured radio, so
they link the toolkit's **`ntn-traffic`** (`NtnRealStackHelper`,
`NtnOranApplication` / `NtnOranSink`), **`ntn-cho`**, **`ntn-constellation`**
(`Sgp4MobilityModel`), and **`mmwave`** (and its bundled `lte`). A few examples
pull in extra siblings:

- `ntn-v2x-edge-urllc` additionally links **`oran-ntn`** (xApp / E2-KPM,
  sat-vs-ground inference placement).
- `ntn-v2x-maritime-ais` additionally links **`ntn-sagin`** — it reuses
  ntn-sagin's `AisMobilityModel` (`AisDanishImporter`) to replay a real AIS
  trace.
- The pure-routing example `ntn-v2x-leo-relay-traffic` builds without mmwave
  (it uses `point-to-point` + IPv4 forwarding) but still needs `ntn-traffic`,
  `ntn-cho` and `ntn-constellation`.

Inside `ns3-ntn-toolkit` these are already in `contrib/`; on a vanilla tree,
clone `ntn-traffic`, `ntn-cho`, `ntn-constellation`, and (for the maritime
example) `ntn-sagin` from the toolkit into `contrib/`, and clone mmwave:

```bash
cd contrib/
git clone https://github.com/nyuwireless-unipd/ns3-mmwave.git mmwave
cd ..
```

---

## 3. Install the module

### As part of the toolkit (recommended)

```bash
git clone -b ntn-integration-v2 https://github.com/Muhammaduazir69/ns3-ntn-toolkit.git
# ntn-v2x is already in contrib/, alongside its sibling modules
```

GitLab mirror: `https://gitlab.com/ns3-ntn-toolkit/ns3-ntn-toolkit`.
Docker: `uzairdocker69/ns3-ntn-toolkit:2.2.1` (or `:latest`).

### Standalone repo (into a vanilla ns-3.43 tree)

```bash
cd contrib/
git clone -b ntn-v2x-v2 https://github.com/Muhammaduazir69/ntn-v2x.git
cd ..
```

Then add the sibling example modules from section 2 under `contrib/`. The
module ships its own FCD traces under `traces/` (`rural-highway-fcd.csv`,
`leo-relay-fcd.csv`); the maritime example reuses ntn-sagin's
`data/ais-sample-trace.csv`.

---

## 4. Configure & build

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build ntn-v2x
./ns3 show profile | grep ntn-v2x   # expect: ... ntn-v2x ...
```

---

## 5. Run the examples

Six example programs ship under `examples/`:

```bash
# Direct V2V over NR PC5 sidelink Mode 2 (no gNB); PRR vs distance (TS 38.885).
# Needs no toolkit siblings — links only core ns-3 + the ntn-v2x library.
./ns3 run "ntn-v2x-pc5-sidelink-bsm --numVehicles=20 --numSubchannels=5 --duration=4"

# Flagship: V2X platoon URLLC with sat-vs-ground edge-AI inference placement.
./ns3 run "ntn-v2x-edge-urllc --edge=sat"
./ns3 run "ntn-v2x-edge-urllc --edge=ground"

# LEO pass over a rural highway; direct/relay/orphan on measured radio.
# Defaults to the shipped contrib/ntn-v2x/traces/rural-highway-fcd.csv.
./ns3 run "ntn-v2x-rural-highway --vehicles=40 --simTime=30"

# Every vehicle a UE on a real mmwave NR cell; closed-form vs measured SINR.
./ns3 run "ntn-v2x-real-stack --duration=20 --numVehicles=8"

# Real 2-hop relay veh0 → veh1 → LEO → server; NtnOran measured KPIs.
# --fcdTrace is REQUIRED; use the shipped synthetic constant-speed CSV fixture.
./ns3 run "ntn-v2x-leo-relay-traffic --simSeconds=60 --bsmHz=10 \
  --fcdTrace=contrib/ntn-v2x/traces/leo-relay-fcd.csv"

# Real maritime NTN: AIS-replayed vessel UE (ntn-sagin AisMobilityModel) on a
# real mmwave NR cell. Defaults to the shipped ntn-sagin AIS sample trace.
./ns3 run "ntn-v2x-maritime-ais --simSeconds=120 --aisTrace=contrib/ntn-sagin/data/ais-sample-trace.csv"
```

Example target names: `ntn-v2x-pc5-sidelink-bsm`, `ntn-v2x-edge-urllc`,
`ntn-v2x-rural-highway`, `ntn-v2x-real-stack`, `ntn-v2x-leo-relay-traffic`,
`ntn-v2x-maritime-ais`.
Each produces the binary `build/contrib/ntn-v2x/examples/ns3.43-<name>-default`.
See the README for the full per-example argument list.

---

## 6. Run the unit tests

```bash
./test.py --suite=ntn-v2x
```

Or run the test runner directly:

```bash
./build/utils/ns3.43-test-runner-default --suite=ntn-v2x
```

The suite registers as `TestSuite("ntn-v2x")` and has 7 unit tests:
trace-replay sync jitter under the 100 ms gate, V2X-LEO direct free-space path
loss within 0.1 dB of the closed form, relay fall-back to a peer when direct
SNR is below threshold, maritime bounded mobility, a 100-vehicle / 5-min
replay that must complete inside the test budget, a SAE J2735 BSM header
round-trip through a packet, and an NR PC5 sidelink Mode-2 test (half-duplex
rule, high in-range PRR under a wide pool vs collisions under a 1-subchannel
pool).

---

## 7. Common issues

**Examples missing after configure** — the real-stack examples need
`ntn-traffic`, `ntn-cho`, `ntn-constellation` and `mmwave` in `contrib/`
(section 2); `ntn-v2x-edge-urllc` also needs `oran-ntn`, and
`ntn-v2x-maritime-ais` also needs `ntn-sagin`. The library builds without any
of them; those examples do not.

**`ntn-v2x-leo-relay-traffic` aborts immediately** — it requires `--fcdTrace`.
Pass the shipped `contrib/ntn-v2x/traces/leo-relay-fcd.csv` (a synthetic
constant-speed FCD-format CSV fixture — not a SUMO microsimulation; the loader
reads a CSV dialect, not SUMO's native fcd-output XML) or your own CSV
converted from a SUMO fcd-export.

---

## 8. Uninstall

```bash
rm -rf contrib/ntn-v2x
./ns3 configure --enable-examples
./ns3 build
```
