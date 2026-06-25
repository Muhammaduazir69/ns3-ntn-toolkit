# Install & run — ntn-slice

`ntn-slice` is an ns-3.43 contributed module. The recommended way to run it is
inside the [ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit)
tree (branch `ntn-integration-v2`), where every dependency below is already
present. It also builds on a vanilla ns-3.43 tree — the library links only the
ns-3 core; the examples additionally need the sibling toolkit modules listed in
section 2.

---

## 1. System requirements

| Component | Version |
|---|---|
| OS | Linux (Ubuntu 22.04+ / Fedora 39+ recommended) |
| C++ compiler | gcc ≥ 11 or clang ≥ 14 |
| CMake | ≥ 3.24 |
| Python | ≥ 3.10 |
| ns-3 | **3.43** |

---

## 2. Dependencies

### 2a. Library

The module library (`CMakeLists.txt`) links only the ns-3 **core** module.
There are no other module dependencies for the library itself.

### 2b. Toolkit siblings (REQUIRED for the examples)

All three examples (`ntn-three-slice-leo-geo`, `ntn-slice-isolation-traffic`,
`ntn-slice-real-stack`) run real per-slice traffic over a real mmwave NR cell,
so they link the toolkit's **`ntn-traffic`** (`NtnRealStackHelper`, the
standards traffic applications), **`ntn-cho`**, **`ntn-constellation`**
(`Sgp4MobilityModel`), and **`mmwave`** (and its bundled `lte`).
`ntn-slice-real-stack` additionally links **`ntn-observability`**. Inside
`ns3-ntn-toolkit` these are already in `contrib/`; on a vanilla tree, clone
`ntn-traffic`, `ntn-cho`, `ntn-constellation` and (if needed)
`ntn-observability` from the toolkit into `contrib/`, and clone mmwave:

```bash
cd contrib/
git clone https://github.com/nyuwireless-unipd/ns3-mmwave.git mmwave
cd ..
```

### 2c. `ns3-ai` (OPTIONAL — RL path only)

The orchestrator's `(observation, action)` shape matches the `SliceEnv`
Gymnasium wrapper in `ns3-ai`; that is only needed for the optional RL path,
not for the C++ allocator. The C++ allocator and all three examples build and
run **without** `ns3-ai`.

---

## 3. Install the module

### As part of the toolkit (recommended)

```bash
git clone -b ntn-integration-v2 https://github.com/Muhammaduazir69/ns3-ntn-toolkit.git
# ntn-slice is already in contrib/, alongside its sibling modules
```

GitLab mirror: `https://gitlab.com/ns3-ntn-toolkit/ns3-ntn-toolkit`.
Docker: `uzairdocker69/ns3-ntn-toolkit:2.2.1` (or `:latest`).

### Standalone repo (into a vanilla ns-3.43 tree)

```bash
cd contrib/
git clone -b ntn-slice-v2 https://github.com/Muhammaduazir69/ntn-slice.git
cd ..
```

Then add the sibling example modules from section 2 under `contrib/`.

---

## 4. Configure & build

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build ntn-slice
./ns3 show profile | grep ntn-slice   # expect: ... ntn-slice ...
```

---

## 5. Run the examples

```bash
# Three co-existing slices over LEO + GEO; GEO mode-skip + per-slice KPIs.
./ns3 run "ntn-three-slice-leo-geo --simTime=3600 --numUes=30 --csv=/tmp/three-slice.csv"

# Real per-slice traffic with NtnSliceSelector + SliceIsolationMonitor.
./ns3 run "ntn-slice-isolation-traffic --simSeconds=30 --numUes=12"

# Phase-2 RAN recipe: slices share ONE real mmwave cell; per-slice KPIs measured.
./ns3 run "ntn-slice-real-stack --duration=20 --numUes=12"
```

Example target names: `ntn-three-slice-leo-geo`,
`ntn-slice-isolation-traffic`, `ntn-slice-real-stack`. See the README for the
full per-example argument list.

---

## 6. Run the unit tests

```bash
./test.py --suite=ntn-slice
```

The suite registers as `TestSuite("ntn-slice")` and has 7 unit tests (S-NSSAI
pack/unpack round-trip, selector first-match-wins, URLLC min-throughput
preservation under contention, isolation-monitor breach detection, URLLC GEO
mode-skip, three-slice co-existence, RL-shares honoured).

---

## 7. Common issues

**Examples missing after configure** — the examples need `ntn-traffic`,
`ntn-cho`, `ntn-constellation` and `mmwave` in `contrib/` (section 2), and
`ntn-slice-real-stack` also needs `ntn-observability`; the library builds
without them, the examples do not.

---

## 8. Uninstall

```bash
rm -rf contrib/ntn-slice
./ns3 configure --enable-examples
./ns3 build
```
