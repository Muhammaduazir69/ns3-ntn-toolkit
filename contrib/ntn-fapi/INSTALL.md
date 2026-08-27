# Install & run — ntn-fapi

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

`ntn-fapi` is an ns-3.43 contributed module. It provides the **SCF-222 FAPI
L1↔L2 message ABI** (DL_TTI / TX_DATA / RX_DATA / CRC.indication) for NR-NTN —
a header-only set of structs in `namespace ns3::fapi`, plus the
`fapi-helpers` conversions. The recommended way to use it is inside the
[ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit) tree
(branch `ntn-integration-v2`), where every dependency below is already
present. `ntn-fapi` ships **bundled inside that tree** — there is no
standalone repository to clone.

The library itself only needs `core`; it builds on a vanilla ns-3.43 tree with
no siblings. The **examples** are the part that needs the rest of the toolkit:
they ride a real mmwave NR NTN cell, so they additionally require
`ntn-traffic`, `ntn-constellation`, `ntn-cho`, `ntn-observability`, and
`mmwave` (see section 2).

---

## 1. System requirements

| Component | Version |
|---|---|
| OS | Linux (Ubuntu 22.04+ / Fedora 39+ recommended) |
| C++ compiler | gcc ≥ 11 or clang ≥ 14 (C++17: `std::variant`) |
| CMake | ≥ 3.24 |
| Python | ≥ 3.10 |
| ns-3 | **3.43** |
| Disk | ~6 GB after build (the examples pull in SNS3 TLE data) |

---

## 2. Dependencies

### 2a. Library — none beyond `core`

`ntn-fapi`'s `LIBRARIES_TO_LINK` is just `${libcore}` (see `CMakeLists.txt`).
The structs and helpers compile on a bare ns-3.43 tree, and the
`./test.py --suite=ntn-fapi` unit suite runs without any sibling module.

### 2b. Toolkit modules `ntn-traffic` + `ntn-constellation` + `ntn-cho` + `ntn-observability` (REQUIRED for the examples)

All three examples link the toolkit's `ntn-traffic`
(`NtnRealStackHelper` — the real mmwave NR NTN cell), `ntn-constellation`
(`Sgp4MobilityModel`, `WalkerConstellation`), and `ntn-cho`
(`NtnTr38811MobilityHelper`, the TR 38.811 UE mobility classes). The
`ntn-fapi-real-stack` example additionally links `ntn-observability`
(`NtnSceneHelper` for the optional NetSimulyzer / Cesium output). These are
**bundled in the toolkit**, so inside `ns3-ntn-toolkit` they are already in
`contrib/`. On a vanilla ns-3.43 tree, obtain them from the toolkit tree
(they have no per-module repositories).

### 2c. SNS3 `satellite` (REQUIRED for the examples, transitively)

The toolkit modules above pull in the SNS3 `satellite` module. On a vanilla
tree:

```bash
cd contrib/
git clone https://github.com/sns3/sns3-satellite.git satellite
cd ..
```

> Size note: SNS3 + bundled TLE data is ~3.7 GB.

### 2d. mmWave NR PHY (REQUIRED for the examples)

The examples run real mmwave NR cells, so `contrib/mmwave` (and its bundled
`lte` dependency) must be present:

```bash
cd contrib/
git clone https://github.com/nyuwireless-unipd/ns3-mmwave.git mmwave
cd ..
```

---

## 3. Install the module

`ntn-fapi` is **bundled-only** — it has no standalone GitHub repository. Clone
the toolkit and it is already in `contrib/ntn-fapi`:

```bash
git clone -b ntn-integration-v2 https://github.com/Muhammaduazir69/ns3-ntn-toolkit.git
cd ns3-ntn-toolkit
# contrib/ntn-fapi is already present
```

GitLab mirror: `https://gitlab.com/ns3-ntn-toolkit/ns3-ntn-toolkit`.
Or skip the build entirely with the prebuilt image
`uzairdocker69/ns3-ntn-toolkit:latest`.

---

## 4. Configure & build

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build ntn-fapi
./ns3 show profile | grep ntn-fapi   # expect: ... ntn-fapi ...
```

---

## 5. Run the examples

Every example writes an honest `sim_health.csv` (measured-KPI fidelity gates,
via `NtnRealStackHelper::WriteHealthReport()`) to `--outputDir` in addition to
its stdout summary.

### 5a. ntn-fapi-real-stack — flagship: SCF-222 ABI on a measured radio

```bash
./ns3 run "ntn-fapi-real-stack --duration=20 --numUes=4 --scsKhz=30"
```
The SCF-222 data ABI exercised slot-by-slot on a real mmwave NR NTN cell; the
serving satellite is an SGP4 Walker satellite, TR 38.811 UEs sit under its t=0
sub-point, and CRC pass/fail + HARQ feedback follow the measured TBLER read off
the mmwave `RxPacketTraceUe` trace. Prints a `--- FAPI Summary (SCF-222 ABI on
MEASURED radio) ---` block (measured mean SINR, mean DL TBLER, radio
throughput, FAPI slots sent/crcOk/retx, delivered KB, goodput).
Also writes **`fapi_sap.csv`** to `--outputDir` (alongside `sim_health.csv`)
persisting the measured SCF-222 SAP latency — slot/DL_TTI/CRC counts plus
`sap_latency_mean_us`/`_min_us`/`_max_us` and `sched_pipeline_mean_us`. This
`DL_TTI.request → CRC.indication` latency is the **CI gate-15 KPI**, measured
SFN/slot-aligned off the real mmwave MAC↔PHY SAP.
Args: `duration` (s), `numUes`, `scsKhz` (15/30/60/120), `tbBytes`,
`altitude` (km), `satEirpDbm`, `outputDir`, `netSim` (NetSimulyzer JSON path),
`czml` (Cesium CZML path).

### 5b. ntn-fapi-dl-data-slotloop — original slot loop on the measured radio

```bash
./ns3 run "ntn-fapi-dl-data-slotloop --simSeconds=20 --numUes=4 --scsKhz=30 --tbBytes=1500"
```
Per-slot `TX_DATA.request` → `RX_DATA.indication` / `CRC.indication` with the
CRC outcome drawn from the measured TBLER of UE 0; the elevation descent drops
the measured SINR so CRC failures and HARQ retransmissions appear. Same FAPI
summary block as above.
Args: `simSeconds`, `numUes`, `scsKhz`, `tbBytes`, `altitude` (km),
`satEirpDbm`, `outputDir`.

### 5c. ntn-fapi-leo-pass-slotloop — same FAPI path over a real SGP4 TLE pass

```bash
./ns3 run "ntn-fapi-leo-pass-slotloop --simSeconds=20 --scsKhz=30 --tle=contrib/ntn-rrc/data/iss-zarya.tle"
```
The serving satellite is propagated from a TLE by `ntn-constellation`'s
`Sgp4MobilityModel`; the ground station is auto-placed at the satellite's t=0
sub-point so a real rise→zenith→set pass occurs regardless of TLE epoch. Prints
a `# === ntn-fapi-leo-pass-slotloop summary ===` block.
Args: `simSeconds`, `scsKhz`, `tbBytes`, `numUes`, `satEirpDbm`,
`tle` (path to a 3-line TLE; defaults to `contrib/ntn-rrc/data/iss-zarya.tle`),
`outputDir`.

---

## 6. Run the unit tests

```bash
./test.py --suite=ntn-fapi
```
The `ntn-fapi` suite (5 unit tests) covers SCF 222.10.02/.04 message-ID
stability and string names, Numerology→SCS mapping against TS 38.211
Table 4.2-1, DMRS bitmap round-trips, DL_TTI PDCCH+PDSCH PDU ordering, and the
UL indication shapes (CRC/SRS/RACH). The suite needs no sibling modules.

---

## 7. Common issues

**`ntn-fapi` examples missing after configure** — the examples need
`ntn-traffic`, `ntn-constellation`, `ntn-cho`, `ntn-observability`, and
`mmwave` in `contrib/` (section 2). The **library and the unit suite build
without any of them**; only the examples need the real-stack siblings.

**`could not read 3-line TLE from ...`** (ntn-fapi-leo-pass-slotloop) — pass an
explicit `--tle=<path>` to a 3-line TLE; the default
`contrib/ntn-rrc/data/iss-zarya.tle` ships with the `ntn-rrc` toolkit module.

**Header-only confusion** — all FAPI types are headers under `model/`
(`fapi-common.h`, `fapi-messages.h`, `fapi-pdu-types.h`, `fapi-helpers.h`); the
only compiled unit is `fapi-helpers.cc`. Include them via
`ns3/fapi-messages.h` etc. after `./ns3 build` installs the headers.

---

## 8. Uninstall

```bash
rm -rf contrib/ntn-fapi
./ns3 configure --enable-examples
./ns3 build
```