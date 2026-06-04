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
the HITRAN-2024 absorption table ships with the module under
`data/hitran2024-lut-subthz.csv`.

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

The module ships 13 built examples under `examples/`. A few to start with:

```bash
./ns3 run thz-ntn-leo-ground
./ns3 run "thz-ntn-leo-ground --freq=300e9 --altitude=600"
./ns3 run thz-ntn-isl
./ns3 run thz-ntn-ris-assisted
```

See [doc/EXAMPLES.md](doc/EXAMPLES.md) for the full list and their arguments.

### Test suite

```bash
./test.py --suite=thz-ntn
```

Expected: **38 / 38 passing**.

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
Make sure `data/hitran2024-lut-subthz.csv` exists in your clone — it ships with the repo.

**Build cache filtering modules**
`./ns3 configure --enable-modules='' --enable-tests --enable-examples`.

---

## 8. Citing

See [README](README.md#cite-this-work).
