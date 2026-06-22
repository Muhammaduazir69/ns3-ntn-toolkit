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
| Disk | **≥ 20 GB** after build (toolkit + SNS3 satellite data + build tree + outputs) |
| RAM | 8 GB build, 4 GB runtime |

### Distro packages (Ubuntu 22.04)

```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build git python3 python3-pip \
    libboost-all-dev libgsl-dev libxml2-dev libsqlite3-dev libpcap-dev \
    pybind11-dev libprotobuf-dev protobuf-compiler \
    g++-11 gcc-11
```

(`pybind11-dev` and the protobuf packages are needed by the `ns3-ai-ntn`
bridge; everything else is the standard ns-3 toolchain.)

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

Branch `ntn-integration-v2` is the current release line and contains the
patched LTE, mmWave, and all 14 custom contrib modules.

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

After pulling the upstream `satellite` module, add the following libraries to `contrib/satellite/CMakeLists.txt`:

```diff
     ${libconfig-store}
+    ${libmagister-stats}
+    ${libntn-traffic}
+    ${libpoint-to-point}
   TEST_SOURCES ${test_sources}
```

---

## 4. (Optional) clone the standalone module repos

All custom modules are already inside this toolkit's `contrib/` — but most
also live in standalone repos if you want fresher commits or to contribute
back upstream (`ntn-cho-framework`, `oran-ntn`, `ns3-thz-ntn`, `ns3-ai`,
`ntn-constellation`, `ntn-rrc`, `ntn-observability`, `ntn-sagin`,
`ntn-slice`, `ntn-v2x`, `ntn-sionna`, `ntn-digital-twin`, `flexric-bridge`
under <https://github.com/Muhammaduazir69>). For example:

```bash
cd contrib/
git clone https://github.com/Muhammaduazir69/ntn-cho-framework.git ntn-cho-fresh
git clone https://github.com/Muhammaduazir69/oran-ntn.git oran-ntn-fresh
git clone https://github.com/Muhammaduazir69/ns3-thz-ntn.git thz-ntn-fresh
git clone -b fix/ns3-43-compatibility-and-critical-bugs \
  https://github.com/Muhammaduazir69/ns3-ai.git ai-fresh
cd ..
```

`ntn-traffic` and `ntn-fapi` ship only inside this toolkit tree.

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
| `ntn-cho` (trigger classes) | `./ns3 run "ntn-cho-handover-traffic --trigger=t1"` (a3 / d1 / t1 / elevation / ta) |
| `oran-ntn` | `./ns3 run "oran-ntn-full-scenario --simTime=600 --xapps=ho,beamhop,slice,doppler,tnntn"` |
| `oran-ntn` (RIC placement A/B) | `./ns3 run "oran-ntn-ric-placement-ab"` |
| `ntn-traffic` | `./ns3 run "ntn-oran-qos-flows"` — 4 5QI flows + C&C on a real NR NTN cell |
| `ntn-traffic` (calibration) | `./ns3 run "ntn-tr38821-calibration"` — TR 38.821 Set-1 LEO-600 gate |
| `thz-ntn` | `./ns3 run "thz-ntn-demo --example=8"` |
| `ns3-ai-ntn` | `cd contrib/ns3-ai-ntn/examples/a-plus-b/use-gym/ && python3 a-plus-b.py` |

Per-module run details: see each module's own `README.md`.

---

## 8. Run the repo-wide validation gates

Two aggregate gates assert that every example runs on a measured radio and
that the radio matches 3GPP study-case numbers:

```bash
python3 tools/check_protocol_fidelity.py    # 36 checks — measured-KPI fidelity per example
python3 tools/check_ntn_standards.py        # 12 checks — orbital theory, Doppler, TR 38.821
                                            #             calibration, platform latency bands,
                                            #             5 NTN handover trigger classes
```

Both exit non-zero on any failure, so they are CI-friendly.

---

## 9. Run the test suites

```bash
./test.py -s ntn-cho
./test.py -s oran-ntn
./test.py -s oran-ntn-multi-tier-ric
./test.py -s oran-ntn-ws4
./test.py -s ntn-oran-application
./test.py -s ntn-oran-ai-flow-monitor
./test.py -s ntn-standards-validation
./test.py -s thz-ntn
```

(`./test.py` with no arguments runs everything, including the upstream ns-3
suites — expect a long run.)

---

## 10. Common issues

**`SatMobilityModel not found`**
You skipped step 3 (SNS3 satellite clone). It's required.

**`MmWaveAmc not found`**
mmWave isn't built. Verify `contrib/mmwave/` exists and re-configure.

**ns3-ai `ImportError: dynamic module does not define module export function`**
LTO bug — make sure you're on the modernised fork (this toolkit's bundled `contrib/ns3-ai-ntn/` is already on the right branch).

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
| thz-ntn | [contrib/thz-ntn/INSTALL.md](contrib/thz-ntn/INSTALL.md) |
| ns3-ai-ntn (fork) | [contrib/ns3-ai-ntn/INSTALL.md](contrib/ns3-ai-ntn/INSTALL.md) |

All other modules document their install/run details in their own
`contrib/<module>/README.md`.

---

## 13. Citing

See the [README](README.md#cite-this-work) for the BibTeX entry.
