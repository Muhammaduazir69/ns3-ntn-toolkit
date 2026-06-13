# Install & run — ntn-cho

`ntn-cho` is an ns-3.43 contributed module. The recommended way to run it is
inside the [ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit)
tree, where every dependency below is already present. It also builds on a
vanilla ns-3.43 tree, provided you add the sibling toolkit modules listed in
section 2 (the library itself needs `satellite`; the examples additionally
need `ntn-traffic`, `ntn-constellation`, and `mmwave`).

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

### 2a. SNS3 `satellite` (REQUIRED)

`NtnOrbitPredictor` / `NtnTteEstimator` use the SNS3 `satellite` module's
`SatSGP4MobilityModel` and antenna-gain patterns:

```bash
cd contrib/
git clone https://github.com/sns3/sns3-satellite.git satellite
cd ..
```

> Size note: SNS3 + bundled TLE data is ~3.7 GB.

### 2b. Toolkit modules `ntn-traffic` + `ntn-constellation` (REQUIRED for the examples)

Since v2.1 the examples (`ntn-cho-leo-basic`, `ntn-cho-full-constellation`,
`ntn-cho-handover-traffic`, `ntn-cho-real-stack`) link the toolkit's
`ntn-traffic` module (`NtnRealStackHelper`, the standards traffic
applications) and `ntn-constellation` (`Sgp4MobilityModel`,
`WalkerConstellation`). The traffic helper is **no longer vendored** into
this module. Inside `ns3-ntn-toolkit` both modules are already in
`contrib/`; on a vanilla ns-3.43 tree, clone them from the toolkit into
`contrib/ntn-traffic` and `contrib/ntn-constellation`.

### 2c. mmWave NR PHY (REQUIRED for the examples)

The same examples run real mmwave NR cells, so `contrib/mmwave` (and its
bundled `lte` dependency) must be present:

```bash
cd contrib/
git clone https://github.com/nyuwireless-unipd/ns3-mmwave.git mmwave
cd ..
```

### 2d. `ns3-ai` (OPTIONAL — learning-based path only)

`NtnAiInterface` bridges to a Python agent via `ns3-ai`. The C++ CHO triggers
(a3 / location / time / tte-aware) build and run **without** it. To enable it:

```bash
cd contrib/
git clone https://github.com/hust-diangroup/ns3-ai.git
cd ..
```

The build auto-detects `ns3-ai` (or the toolkit's `ns3-ai-ntn` fork) and only
then compiles the bridge (CMake defines `NTN_CHO_HAS_NS3AI`).

---

## 3. Install the module

```bash
cd contrib/
git clone https://github.com/Muhammaduazir69/ntn-cho-framework.git ntn-cho
cd ..
```

---

## 4. Configure & build

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build ntn-cho
./ns3 show profile | grep ntn-cho   # expect: ... ntn-cho ...
```

---

## 5. Run the examples

### 5a. ntn-cho-leo-basic — event-driven smoke test (real UDP traffic)

```bash
./ns3 run "ntn-cho-leo-basic --trigger=tte-aware --trafficProfile=mixed --simTime=120 --outputDir=/tmp/leo"
```
Writes `sim_health.csv` to `--outputDir`. Args: `simTime`, `scenario`
(dense-urban|urban|suburban|rural), `trigger` (a3|location|tte-aware),
`d1Threshold`, `qualityTh`, `tteMinimum`, `numUes`, `outputDir`,
`trafficProfile` (nb-iot|embb|urllc|dt|mixed), `strict`.

### 5b. ntn-cho-full-constellation — Walker constellation, 4 algorithms

```bash
./ns3 run "ntn-cho-full-constellation --algorithm=tte-aware --numUes=50 --rngRun=1 --outputDir=/tmp/full"
```
Writes `handover_events.csv`, `measurements.csv`, `tte_computations.csv`,
`satellite_tracks.csv`, `ue_tracks.csv`, `kpi_timeseries.csv`,
`kpi_summary.txt`, the GeoJSON layers, and `sim_health.csv`.
Args include `algorithm` (a3|location|time|tte-aware), `numUes`, `scenario`,
`numPlanes`, `satsPerPlane`, `rngRun`.

### 5c. ntn-cho-handover-traffic — UDP flow that survives a handover

```bash
./ns3 run "ntn-cho-handover-traffic --simSeconds=120 --tteMinSec=20 --dataRateMbps=10"
```
Prints per-second serving cell / SINR / goodput and a FlowMonitor summary; the
PointToPoint link follows the CHO decision so the UDP flow continues across the
handover. No CSV.

### 5d. ntn-realistic-mobility-demo — seven-class UE mobility

```bash
./ns3 run "ntn-realistic-mobility-demo --outputDir=/tmp/mob --simTime=600 --dt=1"
```
Writes `mobility_trace.csv`.

---

## 6. Run the unit tests

```bash
./test.py --suite=ntn-cho
```
Covers the CHO algorithm, the CHO state machine, and the NTN measurement model.

The aggregate KPIs in any associated manuscript come from sweeping
`ntn-cho-full-constellation` over several `--rngRun` seeds and the four
`--algorithm` settings; aggregate them with `tools/analyze_results.py`. They
are not asserted by the unit suite.

---

## 7. Common issues

**`ntn-cho` not registered after configure** — the SNS3 `satellite` module is
missing; `ntn-cho` will not register without `contrib/satellite/`.

**Examples missing after configure** — the examples need `ntn-traffic`,
`ntn-constellation`, and `mmwave` in `contrib/` (steps 2b–2c); the library
builds without them, the examples do not.

**`ns3-ai` build errors** — only `NtnAiInterface` needs it (step 2d). Without
`ns3-ai`, the module still builds and the C++ triggers run.

---

## 8. Uninstall

```bash
rm -rf contrib/ntn-cho
./ns3 configure --enable-examples
./ns3 build
```
