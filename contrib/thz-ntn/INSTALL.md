# Install & run — thz-ntn

This guide installs the `thz-ntn` module on top of the
[ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit)
or any vanilla ns-3.43 tree.

---

## 1. System requirements

| Component | Version |
|---|---|
| OS | Linux (Ubuntu 22.04+ / Fedora 39+) |
| C++ compiler | gcc ≥ 11 or clang ≥ 14 |
| CMake | ≥ 3.24 |
| Python | ≥ 3.10 (for figure-regen scripts only) |
| ns-3 | **3.43** |
| Disk | ~2 GB after build |

No additional Python or HITRAN-database installation is required —
the absorption-line table ships with the module under
`model/data/hitran_subset.csv`.

---

## 2. Prerequisites

### 2a. ns-3.43 tree

Easiest:

```bash
git clone https://github.com/Muhammaduazir69/ns3-ntn-toolkit.git
cd ns3-ntn-toolkit
```

### 2b. SNS3 satellite (REQUIRED — for `SatFreeSpaceLoss` parent class)

```bash
cd contrib/
git clone https://github.com/sns3/sns3-satellite.git satellite
cd ..
```

### 2c. mmWave (REQUIRED — for `MmWaveAmc` MCS selection + `SpectrumValue`)

```bash
cd contrib/
git clone https://gitlab.com/cttc-lena/nr.git mmwave
cd ..
```

---

## 3. Install the thz-ntn module

```bash
cd contrib/
git clone https://github.com/Muhammaduazir69/ns3-thz-ntn.git thz-ntn
cd ..
```

---

## 4. Configure & build

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build thz-ntn
```

Verify:

```bash
./ns3 show profile | grep thz-ntn
```

---

## 5. Run examples

The `thz-ntn-demo` umbrella binary exposes 9 example scenarios via `--example=N`:

| N | Scenario | What it produces |
|---|---|---|
| 1 | LEO-ground slant link budget @ 225 GHz | per-elevation FSPL/abs/scint/pointing CSV |
| 2 | ISL capacity @ 225 GHz | range vs Shannon-capacity sweep |
| 3 | UM-MIMO array gain | gain vs N²×N² element count |
| 4 | RIS sweep | 64–4 096 element gain table |
| 5 | ISAC CRLB on space debris | 2-cm / 10-cm / 1-m detection ranges |
| 6 | EKF beam tracking | per-step pointing-error trace |
| 7 | Atmospheric-window detection | 140 / 220 / 340 / 410 / 460 GHz |
| 8 | 600-s LEO beam-tracking pass | full-pass timeseries |
| 9 | Cross-reference validation | residuals vs ITU-R P.676 / *am* |

Run all 9:

```bash
./ns3 run "thz-ntn-demo --outputDir=thz-ntn-output/"
```

Or one specific:

```bash
./ns3 run "thz-ntn-demo --example=7 --outputDir=/tmp/windows/"
```

### Test suite

```bash
./ns3 run "test-runner --suite=thz-ntn --verbose"
```

Expected: **12 / 12 passing**.

---

## 6. Reproduce the paper figures

```bash
cd papers/sim_runs/
./run_thz_ntn.sh
python3 build_figures_thz_oran.py
```

---

## 7. Common issues

**`ThzNtnFreeSpaceLoss inherits unknown class SatFreeSpaceLoss`**
You're missing SNS3 `satellite`. Clone it under `contrib/satellite/`.

**`MmWaveAmc not found`**
Missing mmWave dependency. Clone it under `contrib/mmwave/`.

**HITRAN line file missing**
Make sure `model/data/hitran_subset.csv` exists in your clone — it ships with the repo at ~250 KB.

**Build cache filtering modules**
`./ns3 configure --enable-modules='' --enable-tests --enable-examples`.

---

## 8. Citing

See [README](README.md#cite-this-work).
