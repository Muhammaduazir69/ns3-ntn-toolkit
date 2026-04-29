# Install & run — ns3-ntn-toolkit

This guide walks you through getting `ns3-ntn-toolkit` from a fresh checkout to a working multi-module simulation.

---

## 1. System requirements

| Component | Version |
|---|---|
| OS | Linux (Ubuntu 22.04+ / Fedora 39+ recommended) |
| C++ compiler | gcc ≥ 11 or clang ≥ 14 |
| CMake | ≥ 3.24 |
| Python | ≥ 3.10 (3.13 supported) |
| Boost | ≥ 1.74 (interprocess) |
| Disk | **≥ 10 GB** after build (toolkit + SNS3 TLE data + outputs) |
| RAM | 8 GB build, 4 GB runtime |

### Distro packages (Ubuntu 22.04)

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3 python3-pip \
    libboost-all-dev libgsl-dev libxml2-dev libsqlite3-dev \
    g++-11 gcc-11
```

### Python deps (only needed if you'll use the RL bridge or rebuild figures)

```bash
pip install "numpy>=2.0" "gymnasium>=1.0" "torch>=2.0" matplotlib pandas
```

---

## 2. Clone the toolkit

```bash
git clone https://github.com/Muhammaduazir69/ns3-ntn-toolkit.git
cd ns3-ntn-toolkit
```

Branch `ntn-integration` is the default and contains the patched LTE,
mmWave, and the five custom modules.

---

## 3. Pull the upstream `satellite` module (REQUIRED)

The SNS3 satellite module is **not bundled** because of its size (~3.7 GB
with TLE data). You must clone it under `contrib/`:

```bash
cd contrib/
git clone https://github.com/sns3/sns3-satellite.git satellite
cd ..
```

Without this, `ntn-cho`, `oran-ntn`, and `thz-ntn` will silently fail to register.

---

## 4. (Optional) clone the standalone module repos

The five custom modules are all already inside this toolkit's `contrib/` —
but if you want to get fresher commits or contribute back upstream:

```bash
cd contrib/
git clone https://github.com/Muhammaduazir69/ntn-cho-framework.git ntn-cho-fresh
git clone https://github.com/Muhammaduazir69/oran-ntn.git oran-ntn-fresh
git clone https://github.com/Muhammaduazir69/ns3-thz-ntn.git thz-ntn-fresh
git clone -b fix/ns3-43-compatibility-and-critical-bugs \
  https://github.com/Muhammaduazir69/ns3-ai.git ai-fresh
cd ..
```

---

## 5. Configure & build

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build
```

Verify all modules registered:

```bash
./ns3 show profile | grep -E 'ntn-cho|oran-ntn|thz-ntn|ai|satellite|mmwave'
```

Expected: all six lines present.

> ⚠ If you see only `thz-ntn` (or any single module) due to a stale build cache:
> `./ns3 configure --enable-modules='' --enable-tests --enable-examples`
> An empty `--enable-modules=''` re-enables all autoloaded modules.

---

## 6. Run the integrated example

```bash
./ns3 run "ntn-tn-integrated-analysis \
  --algorithm=tte-aware \
  --simTime=10 \
  --numTnUes=4 \
  --scenario=suburban"
```

This exercises mmWave terrestrial PHY, the SNS3 satellite module, the 3GPP NTN propagation models, and the `ntn-cho` handover engine in one binary.

Output files land in the working directory:
`mmwave_dl_sinr_trace.csv` · `ntn_measurements.csv` ·
`ntn_handover_events.csv` · `tte_computations.csv` ·
`satellite_tracks.csv` · `cho_state_log.csv` · `kpi_summary.txt`.

---

## 7. Run a per-module example

| Module | Command |
|---|---|
| `ntn-cho` | `./ns3 run "ntn-cho-full-constellation --algorithm=tte-aware --simTime=600"` |
| `oran-ntn` | `./ns3 run "oran-ntn-full-scenario --simTime=600 --xapps=ho,beamhop,slice,doppler,tnntn"` |
| `thz-ntn` | `./ns3 run "thz-ntn-demo --example=8"` |
| `ai` | `cd contrib/ai/examples/a-plus-b/use-gym/ && python3 a-plus-b.py` |

Per-module install/run details: see each module's own `INSTALL.md`.

---

## 8. Reproduce the paper Monte-Carlo

```bash
cd papers/sim_runs/
./run_mc_sweep.sh                      # 10 seeds × 600 s × 4 algorithms (~5 min)
python3 build_figures.py               # publication-ready PDFs
python3 build_figures_thz_oran.py      # THz + O-RAN extra panels
```

Outputs land in `papers/figures/`.

---

## 9. Run the test suites

```bash
./ns3 run "test-runner --suite=ntn-cho --verbose"     # 3 / 3 passing
./ns3 run "test-runner --suite=oran-ntn --verbose"
./ns3 run "test-runner --suite=thz-ntn --verbose"     # 12 / 12 passing
```

---

## 10. Common issues

**`SatMobilityModel not found`**
You skipped step 3 (SNS3 satellite clone). It's required.

**`MmWaveAmc not found`**
mmWave isn't built. Verify `contrib/mmwave/` exists and re-configure.

**ns3-ai `ImportError: dynamic module does not define module export function`**
LTO bug — make sure you're on the modernised fork (this toolkit's bundled `contrib/ai/` is already on the right branch).

**Build cache filtering modules**
Run a clean configure: `./ns3 configure --enable-modules=''`.

**`MmWaveComponentCarrierConf already defined`**
You have two copies of mmWave. Delete one.

---

## 11. Constellation TLE data

The `satellite/data/constellations/` folder ships with real TLE for:
- Iridium-NEXT (66 satellites, 780 km, 86.4°)
- Starlink shell-1 (1 584 satellites, 550 km, 53°)
- Kuiper-1 (1 156 satellites, 590 km, 33°)

Refresh against current operational TLE:

```bash
cd contrib/satellite/data/constellations/
./refresh_tle.sh
```

---

## 12. Per-module follow-ups

| Module | Detailed install guide |
|---|---|
| ntn-cho | [contrib/ntn-cho/INSTALL.md](contrib/ntn-cho/INSTALL.md) |
| oran-ntn | [contrib/oran-ntn/INSTALL.md](contrib/oran-ntn/INSTALL.md) |
| thz-ntn | [contrib/thz-ntn/INSTALL.md](contrib/thz-ntn/INSTALL.md) |
| ai (fork) | [contrib/ai/INSTALL.md](contrib/ai/INSTALL.md) |

---

## 13. Citing

See the [README](README.md#cite-this-work) for the BibTeX entry.
