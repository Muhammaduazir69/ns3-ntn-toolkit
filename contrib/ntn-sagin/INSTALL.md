# Install & run — ntn-sagin

`ntn-sagin` is an ns-3.43 contributed module. The recommended way to run it is
inside the [ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit)
tree (branch `ntn-integration-v2`), where every dependency below is already
present. It also builds on a vanilla ns-3.43 tree, provided you add the sibling
toolkit modules listed in section 2 (the library itself needs `ntn-slice`; the
real-stack examples additionally need `ntn-traffic`, `ntn-cho`,
`ntn-constellation`, `mmwave`, and — for `sagin-flight-leo-e2` — `oran-ntn`).

---

## 1. System requirements

| Component | Version |
|---|---|
| OS | Linux (Ubuntu 22.04+ / Fedora 39+ recommended) |
| C++ compiler | gcc ≥ 11 or clang ≥ 14 |
| CMake | ≥ 3.24 |
| Python | ≥ 3.10 |
| ns-3 | **3.43** |
| Disk | ~6 GB after build (incl. SNS3 TLE data, if the mmwave examples pull `satellite`) |

---

## 2. Dependencies

### 2a. `ntn-slice` (REQUIRED for the library)

The module library (`CMakeLists.txt`) links the ns-3 core, network, mobility
and propagation modules plus the sibling **`ntn-slice`** module.
`SaginSliceRouter` uses its `SliceProfile` / S-NSSAI types, so `ntn-sagin` will
not build or register without `contrib/ntn-slice/`.

### 2b. Toolkit siblings for the real-stack examples (REQUIRED for the examples)

The mmwave real-stack examples (`sagin-haps-leo-relay`, `sagin-uav-swarm`,
`sagin-a2g-real-stack`, `sagin-aeronautical`, `sagin-maritime-leo-traffic`,
`sagin-hst-leo-traffic`, `sagin-adsb-flight-leo-traffic`,
`ntn-sagin-remote-coverage`) link the toolkit's **`ntn-traffic`**
(`NtnRealStackHelper`, the standards traffic applications), **`ntn-cho`**, and
**`ntn-constellation`** (`Sgp4MobilityModel`), plus **`mmwave`** (and its
bundled `lte`). Inside `ns3-ntn-toolkit` these are already in `contrib/`; on a
vanilla tree, clone `ntn-traffic`, `ntn-cho` and `ntn-constellation` from the
toolkit into `contrib/`, and clone mmwave:

```bash
cd contrib/
git clone https://github.com/nyuwireless-unipd/ns3-mmwave.git mmwave
cd ..
```

A few examples pull in extra siblings:

- `sagin-flight-leo-e2` additionally links **`oran-ntn`** (O-RAN E2-KPM
  indications).
- `ntn-sagin-orphan-mobility-showcase` additionally links **`ntn-v2x`** and
  **`ntn-observability`**.
- The pure-routing examples (`sagin-multihop-traffic`, `sagin-slice-traffic`,
  `sagin-sgp4-routed-traffic`) build without mmwave — they use
  `point-to-point` + IPv4 global routing — but still need `ntn-traffic`,
  `ntn-cho` and `ntn-constellation`.

Install whichever siblings you need under `contrib/` before configuring. The
core module builds with just `ntn-slice` present.

---

## 3. Install the module

### As part of the toolkit (recommended)

```bash
git clone -b ntn-integration-v2 https://github.com/Muhammaduazir69/ns3-ntn-toolkit.git
# ntn-sagin is already in contrib/, alongside its sibling modules
```

GitLab mirror: `https://gitlab.com/ns3-ntn-toolkit/ns3-ntn-toolkit`.
Docker: `uzairdocker69/ns3-ntn-toolkit:2.2.1` (or `:latest`).

### Standalone repo (into a vanilla ns-3.43 tree)

```bash
cd contrib/
git clone -b ntn-sagin-v2 https://github.com/Muhammaduazir69/ntn-sagin.git
cd ..
```

Then add the sibling modules from section 2 under `contrib/`.

---

## 4. Configure & build

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

## 5. Run the examples

Thirteen example programs ship under `examples/`. A few representative ones:

```bash
# 4-layer SAGIN over a real mmwave NR LEO cell, per-second, 1 h.
./ns3 run "sagin-haps-leo-relay --simTime=3600 --numUes=10 --csv=/tmp/sagin.csv"

# 8-UAV swarm with TR 36.777 A2G path-loss on a real cell.
./ns3 run "sagin-uav-swarm --simTime=600 --fcGHz=2.0 --outputDir=/tmp/uav"

# Real end-to-end multi-hop Ground→UAV→HAPS→LEO (NtnOran measured KPIs).
./ns3 run "sagin-multihop-traffic --simSeconds=60 --dataRateMbps=20 --eirpDbm=43"

# Cross-layer slice steering (URLLC/eMBB/mMTC over HAPS/LEO/GEO).
./ns3 run "sagin-slice-traffic --simSeconds=60 --dataRateMbps=10"

# Maritime AIS vessel terminal + LEO downlink, real mmwave cell.
./ns3 run "sagin-maritime-leo-traffic --simSeconds=120 --vesselSpeedKn=12"

# Commercial flight + LEO with O-RAN E2-KPM indications (needs oran-ntn).
./ns3 run "sagin-flight-leo-e2 --simSeconds=120 --reportMs=100"
```

Full example list (target names): `sagin-haps-leo-relay`, `sagin-uav-swarm`,
`sagin-a2g-real-stack`, `sagin-aeronautical`, `sagin-flight-leo-e2`,
`sagin-multihop-traffic`, `sagin-slice-traffic`, `sagin-maritime-leo-traffic`,
`sagin-hst-leo-traffic`, `sagin-adsb-flight-leo-traffic`,
`sagin-sgp4-routed-traffic`, `ntn-sagin-remote-coverage`,
`ntn-sagin-orphan-mobility-showcase`.

The traffic examples build a real ns-3 IP data plane (NetDevice + IPv4 +
applications) and print measured KPIs to the console. The mobility models read
the bundled sample traces under `data/` (`ais-sample-trace.csv`,
`opensky-sample-trace.csv`). See the README for the full per-example argument
list.

---

## 6. Run the unit tests

```bash
./test.py --suite=ntn-sagin
```

The suite registers as `TestSuite("ntn-sagin")` and covers the A2G TR 36.777
channel, the multi-layer / slice-aware router, and the HAPS / UAV / ADS-B / AIS
/ HST mobility models and trace importers.

---

## 7. Common issues

**`ntn-sagin` not registered after configure** — the sibling `ntn-slice`
module is missing; the library will not register without `contrib/ntn-slice/`.

**Examples missing after configure** — the real-stack examples need
`ntn-traffic`, `ntn-cho`, `ntn-constellation` and `mmwave` in `contrib/`
(section 2); the library builds without them, those examples do not.

---

## 8. Uninstall

```bash
rm -rf contrib/ntn-sagin
./ns3 configure --enable-examples
./ns3 build
```
