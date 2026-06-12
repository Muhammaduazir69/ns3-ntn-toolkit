<h1 align="center">thz-ntn</h1>

<p align="center"><strong>Sub-THz / D-band non-terrestrial PHY for ns-3.43 — LEO-ground & ISL links, RIS, ultra-massive MIMO, ISAC, alpha-mu fading, NYUSIM-140 calibration, HITRAN-2020-baseline atmospheric absorption</strong></p>

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0--only-green.svg"/></a>
  <img src="https://img.shields.io/badge/HITRAN-2020%20baseline-orange.svg"/>
  <img src="https://img.shields.io/badge/UM--MIMO-up%20to%20128×128-purple.svg"/>
  <img src="https://img.shields.io/badge/tests-38%2F38%20passing-success.svg"/>
</p>

> 100 GHz – 1 THz NTN physics for ns-3: molecular-absorption-gated LEO-ground and inter-satellite links, RIS relays, ultra-massive MIMO, and ISAC.
> Part of the [ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit). See [INSTALL.md](INSTALL.md) and [CHANGELOG.md](CHANGELOG.md).

---

## Overview

`thz-ntn` is a physics-grounded PHY layer for non-terrestrial links operating in the
**100 GHz – 1 THz** band (D-band and sub-THz). The composite channel cascades:

- **Free-space path loss** (`ThzNtnFreeSpaceLoss`) with correct frequency/distance scaling
- **Molecular absorption** from a HITRAN line-by-line model
  (`ThzNtnMolecularAbsorption`) over an ITU-R P.835 stratified atmosphere.
  **HITRAN baseline honesty:** the in-process line parameters (14 H2O + 9 O2
  rotational lines) are drawn from the **HITRAN-2020** release. The optional
  lookup-table path (`HitranLut` in namespace `ns3::thzntn`,
  `data/hitran2024-lut-subthz.csv`) is *tagged* HITRAN-2024 but is generated
  by `tools/hitran2024-lut-gen.py` from the same 23-line model (per-line
  2020→2024 deltas are < 0.5 %); native ingestion of a HITRAN-2024 `.par`
  file remains roadmapped (Roadmap §4.3.1 follow-up)
- **Weather attenuation** (rain / fog / snow) per ITU-R P.838 / P.840 extended to THz
- **ITU-R P.618 scintillation** (amplitude + phase, AR(1) time series)
- **alpha-mu small-scale fading** (`ThzNtnAlphaMuFading`)
- **Composite pointing error** (vibration, J2 perturbation, atmospheric refraction,
  tracking latency)
- **Hardware impairments** (PA, phase noise, ADC SQNR, I/Q imbalance)

On top of the channel it provides **ultra-massive MIMO** arrays (UPA / UCA / Cassegrain)
with DFT codebooks and beam-squint analysis, **RIS** (space / aerial / ground), an
**ISAC** sensing subsystem for space-debris ranging, an **EKF beam tracker** for LEO
passes, **atmospheric transmission windows** (140 / 220 / 340 / 410 / 460 GHz), and a
**NYUSIM-140** calibration reference. The model is cross-validated against ITU-R
P.676-13, P.618-13, and S. Paine's *am* atmospheric model.

## What's new in v2

See [CHANGELOG.md](CHANGELOG.md) for the full history.

- **THz physics is now a real channel plug-in.** `ThzNtnPropagationLossModel`
  re-homes the molecular-absorption and weather calculators as an ns-3
  `PropagationLossModel` that is chained onto a real mmwave NR NTN spectrum
  channel (via `NtnRealStackHelper::AddExtraPropagationLoss` from
  `contrib/ntn-traffic`), so the atmospheric loss attenuates actual packets and
  shows up in the **measured** SINR / TBLER / goodput.
- **All traffic examples were converted to the real radio.** They run a full
  mmwave NR NTN stack (SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC) with
  SGP4/Walker satellite mobility and TR 38.811 ground terminals, and carry
  `NtnOranApplication` QoS flows (in-band 24-byte payload headers with
  5QI / S-NSSAI / sequence / timestamp) whose KPIs are measured at
  `NtnOranSink`. Each writes an honest `sim_health.csv`.
- **Atmospheric-window fields are documented as INDEPENDENT inputs.**
  `AtmosphericWindow::peakTransmittance` (best-case in-band factor at band centre)
  and `AtmosphericWindow::maxZenithAttenuation_dB` (worst-case one-way zenith gaseous
  loss) are consumed **multiplicatively** in `ComputeTransmittance`
  (`T_eff = peakTransmittance · 10^(−atten/10) · freqFactor`). They are **not inverses
  of each other** — both are required, distinct inputs.
- **Physics verified numerically correct:** FSPL frequency/distance scaling, radar
  range^4 SNR, RCS dBsm ↔ m^2 conversion, RIS `20·log10(N)` array gain, UM-MIMO
  `10·log10(N)` gain with `1/√N` beamwidth narrowing, Fraunhofer near-field boundary,
  and Shannon capacity.
- **alpha-mu fading**, **HITRAN-2024-tagged LUT** (generated from the
  HITRAN-2020-baseline line model, see Overview), **NYUSIM-140** calibration
  reference, and a **RIS service model + xApp** for O-RAN-style closed-loop
  control.
- **Geometry fix (2026-06 audit).** `thz-ntn-dband-constellation` used a
  broken hand-rolled spherical conversion (geocentric x-y ring of radius
  Re+h with a flat-Earth z = h against local-frame terminals) that placed
  every "550 km" satellite ~6900 km from the terminals at ~4° elevation; its
  outputs were wrong. It now uses real `Sgp4MobilityModel` orbits projected
  into a local ENU frame (`|ECEF position| = Re + altitude`). The
  `thz-ntn-ris-assisted` satellite is now placed on the actual line of sight
  at the requested elevation instead of at zenith.
- **Mobility maturation (2026-06 audit).** The analytic examples that model a
  pass or a constellation (`leo-ground`, `isl`, `full-stack`,
  `dband-constellation`) now derive their time-varying geometry from real
  SGP4 Walker elements instead of triangular/sinusoidal profiles; the
  remaining parametric geometries (`um-mimo`, `ris-assisted`, `isac`) are the
  point of those sweeps and are explicitly labelled analysis-only.
- **AI flow monitor.** Every measured-radio example calls
  `rs.EnableAiFlowMonitor("<example-name>")` after traffic install, producing
  the toolkit-standard KPM flow series alongside `sim_health.csv`.

## Models, helpers & key classes

Derived from `model/*.h`:

| Class / file | Role |
|---|---|
| `ThzNtnSpectrum` (`thz-ntn-spectrum`) | Atmospheric transmission windows + THz band classification; `ComputeTransmittance`, `GetStandardWindows`, `GetBestWindow` |
| `ThzNtnMolecularAbsorption` (`thz-ntn-molecular-absorption`) | HITRAN line-by-line gaseous absorption over P.835 layers |
| `HitranLut` (`thz-ntn-hitran-lut`, namespace `ns3::thzntn`) | Bundled specific-attenuation lookup table (`data/hitran2024-lut-subthz.csv`) — tagged HITRAN-2024 but generated from the HITRAN-2020-baseline line model (see Overview) |
| `Itu838RainModel`, `Itu618LossModel`, `Itu676AbsorptionModel`, `Itu681LmsModel` (`thz-ntn-itu-recommendations`) | ITU-R P.838 / P.618 / P.676 / P.681 reference implementations |
| `ThzNtnAlphaMuFading` (`thz-ntn-alpha-mu-fading`) | alpha-mu small-scale fading distribution |
| `ThzNtnFreeSpaceLoss`, `ThzNtnChannelModel` | FSPL + composite cascade propagation loss |
| `ThzNtnPropagationLossModel` (`thz-ntn-propagation-loss-model`) | Atmospheric excess loss (gaseous absorption + rain/fog/snow) as a real `PropagationLossModel`, chainable onto a live spectrum channel |
| `ThzNtnWeatherAttenuation`, `ThzNtnScintillation`, `ThzNtnPointingError`, `ThzNtnHardwareImpairments` | Weather, scintillation, pointing, and RF-chain impairments |
| `ThzNtnAntennaArray`, `ThzNtnBeamforming` (`um-mimo`) | UPA / UCA / Cassegrain arrays, DFT codebook, `ComputeBeamSquintLoss_dB` |
| `ThzNtnBeamTracking` (`thz-ntn-beam-tracking`) | EKF / position-based satellite-ephemeris beam tracker |
| `ThzNtnRis` (`thz-ntn-ris`), `ThzNtnRisController` | Reconfigurable intelligent surface, N^2 scaling + phase quantisation loss |
| `ThzNtnRisServiceModel` (`thz-ntn-ris-service-model`), `ThzNtnRisXapp` (`thz-ntn-ris-xapp`) | O-RAN service model + xApp for closed-loop RIS control |
| `ThzNtnIsac`, `ThzNtnIsacProcessor`, `ThzNtnIsacScheduler` (`thz-ntn-isac*`) | Integrated sensing & communication (debris CRLB ranging, comm/sense scheduling) |
| `ThzNtnNyusimReference`, `ThzNtnNyusimCalibrator` (`thz-ntn-nyusim-*`) | NYUSIM-140 GHz calibration reference + calibrator |
| `ThzNtnIslChannel`, `ThzNtnIslLink` | Inter-satellite link channel + link budget / routing |
| `ThzNtnPhy`, `ThzNtnPhySat`, `ThzNtnPhyGround`, `ThzNtnMac`, `ThzNtnMacScheduler` | PHY/MAC stack (SpectrumValue + SINR, MCS selection) |
| `ThzNtnLinkBudget`, `ThzNtnWaveform` | Link-budget utility; 5 candidate waveforms (OFDM / DFT-s-OFDM / OTFS / AFDM / SC-FDE) |
| `ThzNtnHelper` (`helper/`) | Top-level helper: build channels, ISL channels, attach mobility |

## Examples

Each example has two equivalent run forms:

```bash
./ns3 run thz-ntn-<name>                                   # ns3 wrapper
./ns3 run "thz-ntn-<name> --arg=value"                     # with arguments
# or run the built binary directly:
./build/contrib/thz-ntn/examples/ns3.43-thz-ntn-<name>-default
```

### Analytic examples (link budgets / sweeps — analysis-only, no measured radio)

All examples in this group are **analysis-only**: they drive the module's
physics APIs (link budgets, scaling laws, sensing equations) and print/CSV the
results — there is **no measured packet data plane** (each prints an
`[analytic-tool]` banner and carries an "analysis-only" header comment).
Where a pass or constellation is part of the scenario (`leo-ground`, `isl`,
`full-stack`, `dband-constellation`), the geometry comes from **real SGP4
Walker orbits** projected into a local ENU frame; where a single parametric
sweep IS the experiment (`um-mimo`, `ris-assisted`, the unbuilt `isac`), the
parametric geometry is kept and physically consistent.

#### thz-ntn-leo-ground
LEO-to-ground sub-THz downlink link budget over a pass (gated by molecular
absorption). The elevation-sweep table is parametric; the per-second pass time
series is driven by a real SGP4 element (zenith at t=0, receding).
- **Key args:** `--freq` (Hz, def 225e9), `--altitude` (km, def 550), `--txPower` (dBm,
  def 34.77), `--bandwidth` (Hz, def 10e9), `--txGain` (dBi, def 40), `--rxGain` (dBi,
  def 45), `--simTime` (s, def 60), `--outputDir` (def `thz-leo-ground-out`).
- **Outputs:** `thz-leo-ground-out/pass_timeseries.csv` + `sim_health.csv`.

```bash
./ns3 run thz-ntn-leo-ground
./ns3 run "thz-ntn-leo-ground --freq=300e9 --altitude=600"
```

#### thz-ntn-isl
Inter-satellite link SNR / capacity / Doppler vs separation (vacuum, 300 GHz).
The distance-sweep table is parametric; the per-second time series uses two
real SGP4 cross-plane neighbours of a Starlink-class shell (separation and
range-rate Doppler from the live orbits).
- **Key args:** `--freq` (Hz, def 300e9), `--txPower` (dBm, def 30), `--bandwidth` (Hz,
  def 20e9), `--txGain` (dBi, def 40), `--rxGain` (dBi, def 40), `--simTime` (s),
  `--outputDir` (def `thz-isl-out`).
- **Outputs:** `thz-isl-out/isl_timeseries.csv` + `sim_health.csv`.

```bash
./ns3 run thz-ntn-isl
./ns3 run "thz-ntn-isl --freq=300e9 --bandwidth=20e9"
```

#### thz-ntn-um-mimo
Ultra-massive MIMO array + beam-squint analysis: configures frequency / bandwidth /
element count, builds a DFT codebook, and reads worst-case wideband squint loss from
`ComputeBeamSquintLoss_dB()`.
- **Key args:** `--freq` (Hz, def 300e9), `--numElements` (per side, def 32),
  `--cassegrainDiam` (m, def 0.45).
- **Outputs:** console report (no CSV).

```bash
./ns3 run thz-ntn-um-mimo
./ns3 run "thz-ntn-um-mimo --numElements=128"
```

#### thz-ntn-ris-assisted
Direct vs RIS-assisted SNR, N^2 scaling law, and quantisation loss; cascaded path loss
from actual Sat / RIS / GT mobility positions. The satellite is placed
parametrically on the line of sight at the requested `--elevation` (the
configurable elevation IS the experiment), consistent with the parametric
slant range.
- **Key args:** `--freq` (Hz, def 300e9), `--risSize` (elements per side, def 64),
  `--elevation` (deg, def 30), `--txPower` (dBm, def 34.77).
- **Outputs:** console report (no CSV).

```bash
./ns3 run thz-ntn-ris-assisted
./ns3 run "thz-ntn-ris-assisted --risSize=64 --elevation=30"
```

#### thz-ntn-dband-constellation
D-band constellation UT-to-satellite association and link budgets on real SGP4
geometry: slot-0 satellites of `--numSats` adjacent Walker planes (72 × 22,
53°), ENU-projected. **Geometry fix (2026-06):** the previous hand-rolled
placement put every satellite ~6900 km from the terminals at ~4° elevation;
pre-fix outputs of this example should be discarded.
- **Key args:** `--numSats` (def 4), `--numUts` (def 10), `--freq` (Hz, def 140e9),
  `--altitude` (km, def 550), `--txPower` (dBm, def 30).
- **Outputs:** console report (no CSV).

```bash
./ns3 run thz-ntn-dband-constellation
./ns3 run "thz-ntn-dband-constellation --numSats=8 --freq=140e9"
```

#### thz-ntn-full-stack
Integration demo: downlink link budget + RIS assist, ISL SNR/capacity from satellite
mobility, ISAC debris sensing, EKF beam tracking, and achievable spectral efficiency.
Both satellites fly real SGP4 orbits (adjacent Walker planes, common ENU frame):
the downlink elevation profile and the ISL separation come from genuine orbital
dynamics, evaluated in scheduled per-second simulator events.
- **Key args:** `--duration` (s, def 30), `--preset` (def `TeraLink-225GHz`),
  `--debrisRangeKm` (km, def 0.5).
- **Outputs:** console report (no CSV).

```bash
./ns3 run thz-ntn-full-stack
./ns3 run "thz-ntn-full-stack --preset=TeraLink-225GHz --debrisRangeKm=0.5"
```

### Real-radio examples (mmwave NR NTN stack, measured KPIs)

These run a full mmwave NR NTN cell via `NtnRealStackHelper` (SpectrumPhy + MAC +
HARQ + RLC/PDCP + RRC + EPC) with **SGP4/Walker satellite mobility** and TR 38.811
ground terminals. The THz physics enter the packet path as live channel plug-ins
(`ThzNtnPropagationLossModel`, `NtnStaticExtraLossModel`), traffic is carried by
`NtnOranApplication` QoS flows measured at `NtnOranSink`, and each example writes
`<outputDir>/sim_health.csv` plus a console KPI summary (measured SINR / TBLER /
goodput). Every example in this group also enables the toolkit's AI flow
monitor (`rs.EnableAiFlowMonitor("<example-name>")`) right after traffic
install, exporting the standard per-flow KPM series. The carrier is capped at
**100 GHz** (sub-THz / W-band) by the 3GPP spectrum model; the higher-band
studies stay in the analytic examples above.

#### thz-ntn-real-stack
Flagship channel-plugin demo: gaseous absorption + rain chained onto the real mmwave
channel; toggling rain mid-run visibly degrades the measured link.
- **Key args:** `--duration` (s, def 16), `--numUes` (def 4), `--altitude` (km, def 550),
  `--freqGhz` (def 100), `--satEirpDbm` (def 92), `--rainMmH` (def 4, applied in the
  2nd half), `--outputDir` (def `thz-ntn-real-stack-output`).

```bash
./ns3 run thz-ntn-real-stack
./ns3 run "thz-ntn-real-stack --duration=16 --freqGhz=100 --rainMmH=25"
```

#### thz-ntn-leo-ground-downlink-traffic
End-to-end downlink over a receding SGP4 satellite: molecular absorption
(`ThzNtnPropagationLossModel`) attenuates the real packets; FSPL comes from the
stack's own Friis model over the live geometry.
- **Key args:** `--simSeconds` (def 60), `--freqGHz` (def 100, capped at 100),
  `--satEirpDbm` (def 115), `--outputDir` (def `thz-ntn-leo-ground-downlink-output`).

```bash
./ns3 run thz-ntn-leo-ground-downlink-traffic
./ns3 run "thz-ntn-leo-ground-downlink-traffic --simSeconds=40"
```

#### thz-ntn-isl-traffic
Real mmwave NR ISL between two cross-plane SGP4 satellites of a Starlink-class
Walker shell; `ThzNtnIslChannel`'s analytic budget is printed beside the measured SINR.
- **Key args:** `--simSeconds`, `--freqGHz` (capped at 100), `--islEirpDbm`,
  `--numPlanes`, `--satsPerPlane`, `--outputDir` (def `thz-ntn-isl-traffic-output`).

```bash
./ns3 run thz-ntn-isl-traffic
./ns3 run "thz-ntn-isl-traffic --numPlanes=6 --satsPerPlane=10"
```

#### thz-ntn-beam-tracking
EKF beam tracking of a real SGP4 pass, closed over the real radio: the EKF is fed
the measured DL SINR, and its prediction error maps through the `ThzNtnAntennaArray`
3-dB beamwidth to a pointing loss applied as a live channel reconfiguration —
`EKF` and `POSITION_BASED` modes produce measurably different links.
- **Key args:** `--trackingMode` (`EKF` | `POSITION_BASED`), `--updateRate` (Hz, def 10),
  `--simSeconds` (def 40), `--freqGHz` (capped at 100), `--satEirpDbm`, `--arraySize`
  (NxN side, def 32), `--measNoiseDeg` (def 0.05), `--outputDir` (def `thz-beam-track-out`).

```bash
./ns3 run thz-ntn-beam-tracking
./ns3 run "thz-ntn-beam-tracking --trackingMode=POSITION_BASED --updateRate=10"
```

#### thz-ntn-ris-relay-traffic
RIS recovers a blocked THz link mid-simulation: blockage and RIS engagement are
live channel events on the real cell.
- **Key args:** `--simSeconds`, `--freqGHz` (capped at 100), `--satEirpDbm`,
  `--blockageDb`, `--risX`, `--risY`, `--reflEff` (0–1), `--blockFraction`,
  `--risOnFraction`, `--outputDir` (def `thz-ntn-ris-relay-output`).

```bash
./ns3 run thz-ntn-ris-relay-traffic
./ns3 run "thz-ntn-ris-relay-traffic --risX=32 --risY=32 --blockageDb=30"
```

#### thz-ntn-isac-coexist-traffic
ISAC comm/sense coexistence: the real `ThzNtnIsacScheduler` partitions the sub-band
grid per ISAC mode (COMM_ONLY → … → SENSING_ONLY) and its comm share gates the live
downlink TDM-style, so measured goodput tracks the scheduler's decisions.
- **Key args:** `--simSeconds`, `--freqGHz` (capped at 100), `--satEirpDbm`,
  `--numSubBands`, `--numUes`, `--outputDir` (def `thz-ntn-isac-coexist-output`).

```bash
./ns3 run thz-ntn-isac-coexist-traffic
./ns3 run "thz-ntn-isac-coexist-traffic --simSeconds=50 --numSubBands=20"
```

#### thz-ntn-weather-traffic
Weather front (fog / rain / snow phases) as a live channel plug-in on the real cell:
the weather attenuates real packets.
- **Key args:** `--simSeconds`, `--freqGHz`, `--satEirpDbm`, `--rainMmH`,
  `--fogLwc` (g/m^3), `--snowMmH`, `--outputDir` (def `thz-ntn-weather-traffic-output`).

```bash
./ns3 run thz-ntn-weather-traffic
./ns3 run "thz-ntn-weather-traffic --rainMmH=25"
```

#### thz-ntn-ric-controlled-traffic
Closed-loop thz-ntn × oran-ntn demo on the real radio: a KPM tick reads the measured
DL SINR off the mmwave PHY trace and submits it via `OranNtnE2Node`; an xApp engages
the ground RIS (its `ComputeSnrGain_dB()` applied as a live channel reconfiguration)
when intrinsic SINR falls below the threshold. Goodput collapses on a mid-run
urban-canyon blockage and recovers when the xApp engages the RIS.
- **Key args:** `--simSeconds`, `--freqGHz` (capped at 100), `--satEirpDbm`,
  `--blockageDb`, `--sinrThreshDb`, `--xapp` (0/1), `--risN`, `--humidityProfile`,
  `--outputDir` (def `thz-ntn-ric-controlled-output`).

```bash
./ns3 run thz-ntn-ric-controlled-traffic
./ns3 run "thz-ntn-ric-controlled-traffic --simSeconds=40 --xapp=1"
```

> Note: two sources in `examples/` are not built: `thz-ntn-isac.cc` (legacy ISAC API,
> excluded pending the Q4 2026 ISAC scheduler redesign) and `thz-ntn-demo.cc`
> (not registered in `examples/CMakeLists.txt`). Both carry analysis-only
> header labels.

## Experimental components

The following exported classes ship with the module but are **not yet
exercised by any example** (2026-06 orphan audit). Their headers carry a
matching `\warning`. Nothing here is scheduled for deletion; each item is
either example-pending or test-validated.

| Class | Status |
|---|---|
| `ThzNtnIsacProcessor` (`thz-ntn-isac-processor`) | **Experimental — no example or test.** Range-Doppler/CFAR/tracking processor; example pending the Q4 2026 ISAC scheduler redesign |
| `ThzNtnIslLink` (`thz-ntn-isl-link`) | **Experimental — no example or test.** ISL lifecycle/routing manager; the measured ISL path uses `NtnRealStackHelper` instead (`thz-ntn-isl-traffic`) |
| `ThzNtnWaveform` (`thz-ntn-waveform`) | **Experimental — no example or test.** 5-candidate waveform selector (OFDM / DFT-s-OFDM / OTFS / AFDM / SC-FDE) |
| `ThzNtnAlphaMuFading` | Unit-tested (`test/thz-ntn-test-suite.cc`); no example yet |
| `ThzNtnRisController` | Unit-tested; consumed by `ThzNtnRisServiceModel`; no example yet |
| `ThzNtnRisServiceModel` | Unit-tested; no example yet (the measured RIS loop in `thz-ntn-ric-controlled-traffic` drives `ThzNtnRis` directly) |
| `ThzNtnRisXapp` | Unit-tested; no example yet |
| `ThzNtnNyusimCalibrator` / `ThzNtnNyusimReference` | Unit-tested against `data/nyusim-140-reference.csv`; no example yet |
| `Itu838RainModel` / `Itu618LossModel` / `Itu676AbsorptionModel` / `Itu681LmsModel` | Unit-tested against ITU-R reference values; the example channel cascade uses the module's own weather/absorption/scintillation classes |

Cross-module consumers worth knowing about: `HitranLut` is consumed in-module
by `ThzNtnMolecularAbsorption`; `ThzNtnPhy` is the base class of
`ThzNtnPhySat` / `ThzNtnPhyGround`; `ThzNtnSpectrum` is unit-tested and used
by the (unbuilt) `thz-ntn-demo` dataset generator.

## Build, run & test

```bash
# from the ns-3-dev root
./ns3 configure --enable-examples --enable-tests
./ns3 build thz-ntn

# run any example
./ns3 run thz-ntn-leo-ground

# run the test suite (38 unit tests)
./test.py --suite=thz-ntn
```

For full setup and dependency notes see [INSTALL.md](INSTALL.md).

## License & author

**GPL-2.0-only** — see [LICENSE](LICENSE).

**Muhammad Uzair**, Independent Researcher — `muhammaduzairr69@gmail.com`
ORCID: [0009-0002-4104-2680](https://orcid.org/0009-0002-4104-2680)

```bibtex
@misc{uzair2026thzntn,
  author = {Uzair, Muhammad},
  title  = {thz-ntn: 100 GHz -- 1 THz Sub-THz/D-band NTN Physics Module for ns-3.43},
  year   = {2026}
}
```

### Acknowledgements

ns-3 core team · SNS3 maintainers · HITRAN team (Gordon et al. 2022, CFA Harvard) ·
S. Paine's *am* atmospheric model (SAO) · ITU-R P.676 / P.618 / P.835 / P.838 / P.840.
