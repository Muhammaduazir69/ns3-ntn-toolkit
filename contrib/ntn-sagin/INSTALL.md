# Install & run — ntn-sagin

`ntn-sagin` is an ns-3.43 contributed module. It builds on top of a vanilla
ns-3.43 tree, or as part of the
[ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit).

---

## 1. System requirements

| Component | Version |
|---|---|
| OS | Linux (Ubuntu 22.04+ / Fedora 39+ recommended) |
| C++ compiler | gcc >= 11 or clang >= 14 |
| CMake | >= 3.24 |
| Python | >= 3.10 |
| ns-3 | **3.43** |

---

## 2. Dependencies

The module library (`CMakeLists.txt`) links the ns-3 core, network and
mobility modules plus the sibling **`ntn-slice`** module. `ntn-slice` is
required: `SaginSliceRouter` uses its `SliceProfile` / S-NSSAI types, so the
module will not build or register without `contrib/ntn-slice/`.

A few examples pull in extra sibling modules through their own example
`CMakeLists.txt`:

- The traffic examples link **`ntn-traffic`** (`NtnRealisticTrafficHelper`).
- `sagin-flight-leo-e2` links **`oran-ntn`** (O-RAN E2-KPM indications).
- `sagin-sgp4-routed-traffic` links **`ntn-constellation`** (SGP4 mobility +
  contact-graph routing).

Install whichever siblings you need under `contrib/` before configuring. The
core module and most examples build with just `ntn-slice` present.

---

## 3. Configure & build

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build ntn-sagin
./ns3 show profile | grep ntn-sagin   # expect: ... ntn-sagin ...
```

Build a single example:

```bash
./ns3 build sagin-haps-leo-relay
```

---

## 4. Run the examples

Ten example programs ship under `examples/`. A few representative ones:

```bash
# 4-layer SAGIN hop path, per-second, 1 h.
./ns3 run "sagin-haps-leo-relay --simTime=3600 --csv=/tmp/sagin.csv"

# 8-UAV swarm with TR 36.777 A2G path-loss spot-checks.
./ns3 run "sagin-uav-swarm --simTime=600 --fcGHz=2.0 --csv=/tmp/uav.csv"

# Real end-to-end multi-hop Ground->UAV->HAPS->LEO (FlowMonitor summary).
./ns3 run "sagin-multihop-traffic --simSeconds=60 --dataRateMbps=20 --eirpDbm=43"

# Cross-layer slice steering (URLLC/eMBB/mMTC over HAPS/LEO/GEO).
./ns3 run "sagin-slice-traffic --simSeconds=60 --dataRateMbps=10"
```

The traffic examples (`-traffic`, multihop, slice, sgp4) build a real ns-3 IP
data plane (NetDevice + IPv4 + applications) and print a FlowMonitor summary to
the console. See the README for the full list and per-example arguments.

---

## 5. Run the unit tests

```bash
./test.py --suite=ntn-sagin
```

---

## 6. Uninstall

```bash
rm -rf contrib/ntn-sagin
./ns3 configure --enable-examples
./ns3 build
```
