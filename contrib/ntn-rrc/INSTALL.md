# Install & run — ntn-rrc

`ntn-rrc` is an ns-3.43 contributed module (3GPP Rel-17 NR-NTN RRC procedures:
SIB19 ephemeris broadcast, timing advance, pass-aware DRX, UE location
reporting). The recommended way to run it is inside the
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

The library (`build_lib` in `CMakeLists.txt`) links only ns-3 `core`, `network`,
and `mobility`. **All five examples** link the sibling toolkit modules below;
install whichever you need under `contrib/` before configuring.

### 2a. Toolkit modules `ntn-traffic` + `ntn-cho` + `ntn-constellation` (REQUIRED for the examples)

Every example builds the radio through `NtnRealStackHelper` (from `ntn-traffic`),
uses TR 38.811 class UE mobility (`NtnTr38811MobilityModel` from `ntn-cho`), and
flies a Walker-Delta `Sgp4MobilityModel` from `ntn-constellation`. Inside
`ns3-ntn-toolkit` all three are already in `contrib/`; on a vanilla tree they ship
with the toolkit checkout.

### 2b. mmWave NR PHY (REQUIRED for the examples)

The examples run real mmwave NR NTN cells, so `contrib/mmwave` (and its bundled
`lte` dependency) must be present:

```bash
cd contrib/
git clone https://github.com/nyuwireless-unipd/ns3-mmwave.git mmwave
cd ..
```

### 2c. SNS3 `satellite` (REQUIRED for the examples)

The examples link the `satellite` module, and `ntn-rrc-from-tle` drives an SNS3
`SatSGP4MobilityModel`:

```bash
cd contrib/
git clone https://github.com/sns3/sns3-satellite.git satellite
cd ..
```

> Size note: SNS3 + bundled TLE data is ~3.7 GB.

---

## 3. Install the module

### Inside the toolkit (recommended)

Already present in `ns3-ntn-toolkit/contrib/ntn-rrc`. Clone the toolkit (branch
`ntn-integration-v2`):

```bash
git clone -b ntn-integration-v2 \
  https://github.com/Muhammaduazir69/ns3-ntn-toolkit.git
# GitLab mirror: https://gitlab.com/ns3-ntn-toolkit/ns3-ntn-toolkit
```

Docker image (everything pre-built): `uzairdocker69/ns3-ntn-toolkit:2.2.1`
(or `:latest`).

### Standalone repo

Clone the standalone module into `contrib/ntn-rrc`, pinning its current branch:

```bash
cd contrib/
git clone -b ntn-rrc-v2 \
  https://github.com/Muhammaduazir69/ntn-rrc.git ntn-rrc
cd ..
```

You still need the section 2 dependencies (`ntn-traffic`, `ntn-cho`,
`ntn-constellation`, `mmwave`, `satellite`) in `contrib/`.

---

## 4. Configure & build

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build ntn-rrc
./ns3 show profile | grep ntn-rrc   # expect: ... ntn-rrc ...
```

---

## 5. Run the examples

All five examples run real UDP traffic over a real mmwave NR NTN cell and write a
`sim_health.csv` to `--outputDir`. Binaries live under
`build/contrib/ntn-rrc/examples/` as `ns3.43-<name>-default`.

### 5a. ntn-rrc-leo-pass — TA + SIB19 over a real SGP4 pass

```bash
./ns3 run "ntn-rrc-leo-pass --simTime=20 --transparent=true --outputDir=ntn-rrc-leo-pass-output"
```
Ephemeris-driven Timing Advance + SIB19 on a real mmwave NR cell over a genuine
SGP4 LEO pass; an RRC measurement report fires when the measured DL SINR drops
below threshold. Writes `ntn-rrc-leo-pass-ta.csv` + `sim_health.csv`. Args:
`simTime`, `numUes`, `altitude`, `satEirpDbm`, `freqGhz`, `transparent`,
`outputDir`.

### 5b. ntn-rrc-full-stack — all four NTN-RRC components at once

```bash
./ns3 run "ntn-rrc-full-stack --simTime=20 --transparent=true --passAwareDrx=true --prefix=ntn-rrc-full"
```
TA pre-comp, SIB19 broadcast, UE GNSS reporting, and the DRX state machine over a
real SGP4 Walker orbit with TR 38.811 UEs. Writes `<prefix>-ta.csv`,
`<prefix>-sib19.csv`, `<prefix>-ue.csv`, `<prefix>-drx.csv`, `sim_health.csv`.
Args: `simTime`, `numUes`, `altitude`, `satEirpDbm`, `transparent`,
`passAwareDrx`, `prefix`, `outputDir`.

### 5c. ntn-rrc-from-tle — SNS3 SatSGP4MobilityModel from a TLE

```bash
# runs with no args (bundled ISS TLE)
./ns3 run "ntn-rrc-from-tle"
# or with an explicit TLE + scenario start
./ns3 run "ntn-rrc-from-tle --tle=contrib/ntn-rrc/data/iss-zarya.tle --start=2024-01-01T12:00:00"
```
Drives an SNS3 `SatSGP4MobilityModel` from a real 3-line TLE under the mmwave
cell; UEs auto-placed at the t=0 sub-point. Ships with a bundled ISS TLE
(`data/iss-zarya.tle`). Writes `ntn-rrc-from-tle.csv` + `sim_health.csv`. Args:
`tle` (default bundled ISS TLE), `start`, `simTime`, `numUes`, `satEirpDbm`,
`step`, `transparent`, `outputDir`.

### 5d. ntn-rrc-drx-data-traffic — DRX power/throughput trade-off

```bash
./ns3 run "ntn-rrc-drx-data-traffic --simSeconds=20 --numUes=4 --drxOn=true"
```
Real UDP downlink with SIB19, live TA, and the connected-mode DRX cycle; reports
measured KPIs with DRX on-duty fraction and effective goodput. Compare
`--drxOn=true` vs `--drxOn=false`. Writes `sim_health.csv`. Args: `simSeconds`,
`numUes`, `leoAltKm`, `freqGHz`, `satEirpDbm`, `drxOn`, `drxLongCycleMs`,
`drxOnDurationMs`, `outputDir`.

### 5e. ntn-rrc-real-stack — real-stack flagship

```bash
./ns3 run "ntn-rrc-real-stack --duration=20 --numUes=4"
```
`NtnTimingAdvance` + `NtnSib19Broadcaster` (TS 38.331 NTN-Config) on the real
mmwave NR cell, SIB19 re-broadcast every si-period from live ephemeris; RRC
measurement report triggers on the measured DL SINR. Writes
`ntn-rrc-real-stack-ta.csv` + `sim_health.csv`. Args: `duration`, `numUes`,
`altitude`, `satEirpDbm`, `freqGhz`, `transparent`, `outputDir`.

---

## 6. Run the unit tests

```bash
./test.py --suite=ntn-rrc
```
The suite (`Type::UNIT`) has 16 test cases covering the closed-form TA
(transparent and regenerative), the common/UE-specific decomposition, the LEO
drift-rate bound, SIB19 codec round-trip and truncation rejection, broadcaster
cadence, ECEF↔WGS-84 round-trip, all three location-report modes, the standard
DRX cycle and data-activity override, pass-aware deep sleep, and invalid-config
rejection.

---

## 7. Common issues

**Examples missing after configure** — the examples need `ntn-traffic`,
`ntn-cho`, `ntn-constellation`, `mmwave`, and `satellite` in `contrib/`
(section 2); the library itself builds without them.

**`ntn-rrc-from-tle` fails** — it requires the SNS3 `satellite` module
(`SatSGP4MobilityModel`); see step 2c.

---

## 8. Uninstall

```bash
rm -rf contrib/ntn-rrc
./ns3 configure --enable-examples
./ns3 build
```
