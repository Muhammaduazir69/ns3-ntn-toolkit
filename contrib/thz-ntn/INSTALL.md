# Install & run — thz-ntn

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

`thz-ntn` is an ns-3.43 contributed module: a physics-grounded Sub-THz /
D-band (100 GHz – 1 THz) PHY for non-terrestrial links — molecular-absorption-
gated LEO-ground and inter-satellite links, RIS relays, ultra-massive MIMO,
EKF beam tracking, and ISAC. The recommended way to run it is inside the
[ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit) tree
(branch `ntn-integration-v2`), where every dependency below is already
present. It also builds on a vanilla ns-3.43 tree once you add the sibling
modules in section 2 — the library links `satellite` and `mmwave`; the
examples additionally pull in `ntn-cho`, `ntn-constellation`, `ntn-traffic`,
`oran-ntn`, and `ntn-observability`.

---

## 1. System requirements

| Component | Version |
|---|---|
| OS | Linux (Ubuntu 22.04+ / Fedora 39+) |
| C++ compiler | gcc ≥ 11 or clang ≥ 14 |
| CMake | ≥ 3.24 |
| Python | ≥ 3.10 (for figure-regen scripts only) |
| ns-3 | **3.43** |
| Disk | ~6 GB after build (incl. SNS3 TLE data) |

No additional Python or spectroscopic-database installation is required — the
absorption lookup table ships with the module under
`data/hitran2024-lut-subthz.csv` (an ITU-R P.676-13 grid sample kept under a
legacy filename; regenerate it with `tools/p676-lut-gen.py`, which is
self-contained and needs no external HITRAN `.par` file).

---

## 2. Dependencies

### 2a. SNS3 `satellite` (REQUIRED — for the `SatFreeSpaceLoss` parent class)

```bash
cd contrib/
git clone https://github.com/sns3/sns3-satellite.git satellite
cd ..
```

> Size note: SNS3 + bundled TLE data is ~3.7 GB.

### 2b. mmWave NR PHY (REQUIRED — for `MmWaveAmc` MCS selection + `SpectrumValue`)

The toolkit uses the **NYU/UNIPD** ns-3-mmwave module — **not** the CTTC NR
module. The traffic examples chain the THz channel onto a real mmwave NR
spectrum channel, and the library links mmwave directly:

```bash
cd contrib/
git clone https://github.com/nyuwireless-unipd/ns3-mmwave.git mmwave
cd ..
```

### 2c. Example-only siblings (REQUIRED for the examples)

The examples additionally link `ntn-cho` and `ntn-constellation` (SGP4 /
Walker mobility), `ntn-traffic` (`NtnRealStackHelper`, the standards traffic
apps), `oran-ntn` (the closed-loop RIC example), and `ntn-observability`
(the real-stack example). All are bundled in the toolkit under `contrib/`;
on a vanilla tree, copy them from the toolkit. The `thz-ntn` library itself
builds without them — the examples do not.

---

## 3. Install the thz-ntn module

```bash
cd contrib/
git clone -b thz-ntn-v2 https://github.com/Muhammaduazir69/ns3-thz-ntn.git thz-ntn
cd ..
```

---

## 4. Configure & build

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build thz-ntn
./ns3 show profile | grep thz-ntn   # expect: ... thz-ntn ...
```

---

## 5. Run the examples

The module ships 16 built examples under `examples/` (the legacy
`thz-ntn-isac` is intentionally left out of the build pending the Q4 2026
ISAC scheduler rework). A few to start with:

### Analytic / physics examples (no traffic stack)

```bash
./ns3 run thz-ntn-leo-ground
./ns3 run "thz-ntn-leo-ground --freq=300e9 --altitude=600 --txPower=10"
./ns3 run thz-ntn-isl
./ns3 run "thz-ntn-ris-assisted --freq=140e9 --risSize=256 --elevation=40"
```
`thz-ntn-leo-ground` / `thz-ntn-isl` args: `freq`, `altitude` (leo-ground
only), `txPower`, `bandwidth`, `txGain`, `rxGain`, `simTime`, `outputDir`.
`thz-ntn-ris-assisted` args: `freq`, `risSize`, `elevation`, `txPower`.

### Measured packet-plane examples (real mmwave NR stack)

```bash
./ns3 run "thz-ntn-leo-ground-downlink-traffic --simSeconds=60 --freqGHz=140 --outputDir=/tmp/thz"
./ns3 run "thz-ntn-weather-traffic --simSeconds=60 --freqGHz=140 --rainMmH=10 --outputDir=/tmp/thz"
./ns3 run "thz-ntn-real-stack --duration=60 --numUes=4 --freqGhz=140 --rainMmH=5 --outputDir=/tmp/thz"
./ns3 run "thz-ntn-ric-controlled-traffic --simSeconds=60 --xapp=ris --sinrThreshDb=5 --outputDir=/tmp/thz"
```
These chain the THz channel onto a real mmwave NR NTN cell, so the
atmospheric/weather loss attenuates actual packets and shows up in measured
SINR / TBLER / goodput. `thz-ntn-ric-controlled-traffic` closes a loop with
`oran-ntn`: THz KPIs feed a KPM report and the xApp toggles the RIS.

The remaining built examples are `thz-ntn-isl-traffic`, `thz-ntn-beam-tracking`,
`thz-ntn-um-mimo`, `thz-ntn-ris-relay-traffic`, `thz-ntn-isac-coexist-traffic`,
`thz-ntn-dband-constellation`, `thz-ntn-demo`, and `thz-ntn-full-stack`.

See [doc/EXAMPLES.md](doc/EXAMPLES.md) for the full list and their arguments.

---

## 6. Run the unit tests

`thz-ntn` registers a single test suite:

```bash
./test.py --suite=thz-ntn
```
Expected: **38 / 38 passing** (link budget, molecular absorption, ISL channel,
RIS, antenna array, beam tracking, spectrum, alpha-mu fading, NYUSIM-140).

---

## 7. Common issues

**`ThzNtnFreeSpaceLoss inherits unknown class SatFreeSpaceLoss`** — SNS3
`satellite` is missing; clone it under `contrib/satellite/` (step 2a).

**`MmWaveAmc not found` / unresolved `SpectrumValue` symbols** — the mmWave
dependency is missing. Clone the **NYU/UNIPD** module under `contrib/mmwave/`
(step 2b); do **not** substitute the CTTC NR module.

**Examples missing after configure** — the traffic examples need `ntn-cho`,
`ntn-constellation`, `ntn-traffic`, `oran-ntn`, and `ntn-observability` in
`contrib/` (step 2c); the library builds without them, the examples do not.

**Absorption LUT missing** — make sure `data/hitran2024-lut-subthz.csv`
(the ITU-R P.676-13 grid sample, under its legacy filename) exists in your
clone; it ships with the repo and can be regenerated with `tools/p676-lut-gen.py`.

**Build cache filtering modules** —
`./ns3 configure --enable-modules='' --enable-tests --enable-examples`.

---

## 8. Uninstall

```bash
rm -rf contrib/thz-ntn
./ns3 configure --enable-examples
./ns3 build
```