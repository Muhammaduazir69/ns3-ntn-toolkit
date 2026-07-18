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

Branch **`ntn-integration-v2`** is the current release line. It contains the
patched LTE and mmWave modules, the vendored **NetSimulyzer v1.0.13** 3D
visualization module, and **all 14 custom NTN contrib modules** (see the
[Module guide](#12-module-guide--all-modules) below). The GitLab mirror is
<https://gitlab.com/ns3-ntn-toolkit/ns3-ntn-toolkit> (same branch).

> Prefer not to build from source? A prebuilt image ships everything:
> `docker pull uzairdocker69/ns3-ntn-toolkit:2.2.1` (also `:latest`).

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

Every custom module is **already inside this toolkit's `contrib/`** — you do
not need this step to build. But 12 of the 14 modules also live in their own
standalone repos under <https://github.com/Muhammaduazir69> if you want fresher
commits or to contribute upstream. **Each has its own current branch** — always
clone with the explicit `-b <branch>` below:

| Module | Standalone repo | Current branch |
|---|---|---|
| `ntn-cho` | `ntn-cho-framework` | `main` |
| `oran-ntn` | `oran-ntn` | `oran-ntn-v2` |
| `thz-ntn` | `ns3-thz-ntn` | `thz-ntn-v2` |
| `ns3-ai-ntn` | `ns3-ai` | `fix/ns3-43-compatibility-and-critical-bugs` |
| `ntn-constellation` | `ntn-constellation` | `ntn-constellation-v2` |
| `ntn-rrc` | `ntn-rrc` | `ntn-rrc-v2` |
| `ntn-observability` | `ntn-observability` | `ntn-observability-v2` |
| `ntn-sagin` | `ntn-sagin` | `ntn-sagin-v2` |
| `ntn-slice` | `ntn-slice` | `ntn-slice-v2` |
| `ntn-v2x` | `ntn-v2x` | `ntn-v2x-v2` |
| `ntn-sionna` | `ntn-sionna` | `ntn-sionna-v2` |
| `ntn-digital-twin` | `ntn-digital-twin` | `ntn-digital-twin-v2` |
| `ntn-fapi` | — (bundled-only) | — |
| `ntn-traffic` | — (bundled-only) | — |

For example, to pull a fresh `oran-ntn` on its current branch:

```bash
cd contrib/
git clone -b oran-ntn-v2 https://github.com/Muhammaduazir69/oran-ntn.git oran-ntn-fresh
git clone -b ntn-constellation-v2 https://github.com/Muhammaduazir69/ntn-constellation.git nc-fresh
git clone -b fix/ns3-43-compatibility-and-critical-bugs \
  https://github.com/Muhammaduazir69/ns3-ai.git ai-fresh
cd ..
```

`ntn-traffic` and `ntn-fapi` ship **only** inside this toolkit tree (no
standalone repo).

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
./ns3 run "ntn-e2e-full-stack --duration=60 --numUes=8 --altitude=600 --outputDir=/tmp/e2e"
```

`ntn-e2e-full-stack` is the cross-module integration flagship: **one** real
mmWave NR NTN cell under SGP4 mobility whose **measured** PHY SINR
simultaneously feeds the O-RAN E2SM-KPM flow monitor, the Near-RT RIC xApps,
and the in-band QoS sink — exercising `ntn-traffic`, `oran-ntn`,
`ntn-constellation`, and the `mmwave` / `satellite` stack in one binary.

Output lands in `--outputDir`: `sim_health.csv` (the realism gate),
`kpm_dataset.csv` / `kpm_canonical.csv` (E2SM-KPM canonical names),
`action_log.csv`, `xapp_metrics.csv`, `measured_kpi_log.csv`.

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
| `ntn-v2x` (PC5 sidelink) | `./ns3 run "ntn-v2x-pc5-sidelink-bsm --numVehicles=20 --numSubchannels=5 --duration=4"` — NR PC5 Mode-2 J2735 BSM broadcast, PRR vs distance (TS 38.885) |
| `ns3-ai-ntn` | `cd contrib/ns3-ai-ntn/examples/a-plus-b/use-gym/ && python3 a-plus-b.py` |

Per-module run details: see each module's own `README.md`.

---

## 8. Run the repo-wide validation gates

Two aggregate gates assert that every example runs on a measured radio and
that the radio matches 3GPP study-case numbers:

```bash
python3 tools/check_protocol_fidelity.py    # measured-KPI fidelity gate — asserts every
                                            # registered example runs on a real radio plane
                                            # (sim_health.csv provenance, not synthetic data)
python3 tools/check_ntn_standards.py        # 3GPP conformance gate — TR 38.821 link-budget
                                            # calibration, Table-3 platform latency bands,
                                            # and the NTN handover trigger classes
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

## 12. Module guide — all modules

The toolkit ships **14 custom NTN modules** plus 4 vendored/patched upstream
modules. Every custom module has its own `INSTALL.md` (build + dependencies +
how to run its examples) and `README.md` (design + capabilities) in
`contrib/<module>/`. Branches are the standalone-repo branches from
[section 4](#4-optional-clone-the-standalone-module-repos).

### Custom NTN modules

| Module | What it does | Per-module guide |
|---|---|---|
| **ntn-cho** | TTE-aware Rel-17/18 conditional handover for LEO NTN (full D1/D2/T1 + A3/elevation/TA trigger set) | [INSTALL](contrib/ntn-cho/INSTALL.md) · [README](contrib/ntn-cho/README.md) |
| **oran-ntn** | O-RAN Near-RT + Space RIC, 16 xApps, real E2AP/E2SM over SCTP, KPM/RC/CCC/Ephemeris service models | [INSTALL](contrib/oran-ntn/INSTALL.md) · [README](contrib/oran-ntn/README.md) |
| **thz-ntn** | 100 GHz–1 THz physics: ITU-R P.676-13 line-by-line absorption, UM-MIMO, RIS, ISAC, beam tracking | [INSTALL](contrib/thz-ntn/INSTALL.md) · [README](contrib/thz-ntn/README.md) |
| **ntn-constellation** | Vallado SGP4 Walker mega-constellations, live CelesTrak/Space-Track TLE, C/N0→BLER link, contact-graph routing | [INSTALL](contrib/ntn-constellation/INSTALL.md) · [README](contrib/ntn-constellation/README.md) |
| **ntn-rrc** | NR-NTN control plane: SIB19 ephemeris over the air, timing advance, NTN-DRX | [INSTALL](contrib/ntn-rrc/INSTALL.md) · [README](contrib/ntn-rrc/README.md) |
| **ntn-sagin** | Space-air-ground integration: HAPS, UAV, maritime (AIS), aviation (ADS-B), high-speed-train mobility + multi-layer routing | [INSTALL](contrib/ntn-sagin/INSTALL.md) · [README](contrib/ntn-sagin/README.md) |
| **ntn-sionna** | NVIDIA Sionna RT ray-traced channel bridged into ns-3 (cascade channel, calibration vs TR 38.811) | [INSTALL](contrib/ntn-sionna/INSTALL.md) · [README](contrib/ntn-sionna/README.md) |
| **ntn-slice** | 5G network slicing over NTN (eMBB / URLLC / mMTC) with 5QI/S-NSSAI selection and an orchestrator xApp | [INSTALL](contrib/ntn-slice/INSTALL.md) · [README](contrib/ntn-slice/README.md) |
| **ntn-v2x** | Vehicle-to-everything over LEO: SUMO/AIS trace replay, `V2xLeoRelay` routing, maritime mobility | [INSTALL](contrib/ntn-v2x/INSTALL.md) · [README](contrib/ntn-v2x/README.md) |
| **ntn-observability** | Measured-KPI observability: InfluxDB/Grafana, NetSimulyzer JSON, Cesium CZML, `NtnSceneRecorder`, repro manifest | [INSTALL](contrib/ntn-observability/INSTALL.md) · [README](contrib/ntn-observability/README.md) |
| **ntn-digital-twin** | Live constellation digital twin + FastAPI prediction service (TLE→SGP4→geometry) | [INSTALL](contrib/ntn-digital-twin/INSTALL.md) · [README](contrib/ntn-digital-twin/README.md) |
| **ns3-ai-ntn** | ns3-ai RL/gym shared-memory bridge (modernised fork). *The bundled NTN RL envs are clearly-labeled synthetic policy-search placeholders — they do not boot ns-3.* | [INSTALL](contrib/ns3-ai-ntn/INSTALL.md) · [README](contrib/ns3-ai-ntn/README.md) |
| **ntn-fapi** | 3GPP SCF-222 FAPI P5/P7 L1↔L2 message ABI (bundled-only) | [INSTALL](contrib/ntn-fapi/INSTALL.md) · [README](contrib/ntn-fapi/README.md) |
| **ntn-traffic** | `NtnRealStackHelper` (real mmWave NR NTN cell) + `NtnOranApplication`/`Sink` + 3GPP CBR/NRTV/HTTP traffic. **Required by most other modules' examples.** (bundled-only) | [INSTALL](contrib/ntn-traffic/INSTALL.md) · [README](contrib/ntn-traffic/README.md) |

### Vendored / patched upstream modules (bundled in `contrib/`)

| Module | Role | Source |
|---|---|---|
| `mmwave` | 5G NR mmWave PHY/MAC (patched for NTN); the real radio under `NtnRealStackHelper` | nyuwireless-unipd/ns3-mmwave |
| `nr` | 5G-LENA NR (CTTC) — FR1 numerology (15/30 kHz), BWP, TR 38.821-calibratable PHY; integrated for the FR1-NTN migration path (`git clone --branch 5g-lena-v3.3.y https://gitlab.com/cttc-lena/nr.git contrib/nr`). One ns-3.43 compat patch: `#undef MIN_NO_CC/MAX_NO_CC` guard in `nr-common.h`. | cttc-lena/nr |
| `satellite` | SNS3 — SatSGP4 mobility, DVB-S2/RCS2, antenna patterns, TLE corpus | sns3/sns3-satellite (**clone separately**, see [§3](#3-pull-the-upstream-satellite-module-required)) |
| `netsimulyzer` | NetSimulyzer v1.0.13 — 3D scene visualization sink (now bundled in-tree) | usnistgov/NetSimulyzer-ns3-module |
| `magister-stats` | Statistics framework used by SNS3 and the observability exporters | Magister / SNS3 |

> Every example writes a standardized output bundle to `<example>-output/`:
> `sim_health.csv` (the realism gate — every KPI carries a `provenance` tag),
> `<example>_kpm_series.csv` (O-RAN E2SM-KPM canonical names), plus per-module
> CSV/GeoJSON. See `STANDARDS_CONFORMANCE_AUDIT_2026-06-24.md` for the
> standards mapping.

---

## 13. Citing

See the [README](README.md#cite-this-work) for the BibTeX entry.
