<h1 align="center">thz-ntn</h1>

<p align="center"><strong>Sub-THz / D-band non-terrestrial PHY for ns-3.43 — LEO-ground & ISL links, RIS, ultra-massive MIMO, ISAC, alpha-mu fading, NYUSIM-140 calibration, HITRAN-2024 atmospheric LUT</strong></p>

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0--only-green.svg"/></a>
  <img src="https://img.shields.io/badge/HITRAN-2024-orange.svg"/>
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
- **Molecular absorption** from a HITRAN line-by-line model and a bundled
  **HITRAN-2024** lookup table (`ThzNtnMolecularAbsorption`, `HitranLut` in
  namespace `ns3::thzntn`) over an ITU-R P.835 stratified atmosphere
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
- **alpha-mu fading**, **HITRAN-2024 LUT**, **NYUSIM-140** calibration reference, and a
  **RIS service model + xApp** for O-RAN-style closed-loop control.
- New **real-data-plane traffic examples** (NetDevice + IP + apps + FlowMonitor) for
  LEO downlink, ISL, RIS relay, ISAC coexistence, weather, and RIC-controlled scenarios.

## Models, helpers & key classes

Derived from `model/*.h`:

| Class / file | Role |
|---|---|
| `ThzNtnSpectrum` (`thz-ntn-spectrum`) | Atmospheric transmission windows + THz band classification; `ComputeTransmittance`, `GetStandardWindows`, `GetBestWindow` |
| `ThzNtnMolecularAbsorption` (`thz-ntn-molecular-absorption`) | HITRAN line-by-line gaseous absorption over P.835 layers |
| `HitranLut` (`thz-ntn-hitran-lut`, namespace `ns3::thzntn`) | Bundled HITRAN-2024 specific-attenuation lookup table (`data/hitran2024-lut-subthz.csv`) |
| `Itu838RainModel`, `Itu618LossModel`, `Itu676AbsorptionModel`, `Itu681LmsModel` (`thz-ntn-itu-recommendations`) | ITU-R P.838 / P.618 / P.676 / P.681 reference implementations |
| `ThzNtnAlphaMuFading` (`thz-ntn-alpha-mu-fading`) | alpha-mu small-scale fading distribution |
| `ThzNtnFreeSpaceLoss`, `ThzNtnChannelModel` | FSPL + composite cascade propagation loss |
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

### Physics-only examples (analytical link budgets / sweeps, CSV output)

#### thz-ntn-leo-ground
LEO-to-ground sub-THz downlink link budget over a pass (gated by molecular absorption).
- **Key args:** `--freq` (Hz, def 225e9), `--altitude` (km, def 550), `--txPower` (dBm,
  def 34.77), `--bandwidth` (Hz, def 10e9), `--txGain` (dBi, def 40), `--rxGain` (dBi,
  def 45), `--simTime` (s, def 60), `--outputDir` (def `thz-leo-ground-out`).
- **Outputs:** `thz-leo-ground-out/pass_timeseries.csv`.

```bash
./ns3 run thz-ntn-leo-ground
./ns3 run "thz-ntn-leo-ground --freq=300e9 --altitude=600"
```

#### thz-ntn-isl
Inter-satellite link SNR / capacity / Doppler vs separation (vacuum, 300 GHz).
- **Key args:** `--freq` (Hz, def 300e9), `--txPower` (dBm, def 30), `--bandwidth` (Hz,
  def 20e9), `--txGain` (dBi, def 40), `--rxGain` (dBi, def 40), `--simTime` (s),
  `--outputDir` (def `thz-isl-out`).
- **Outputs:** `thz-isl-out/isl_timeseries.csv`.

```bash
./ns3 run thz-ntn-isl
./ns3 run "thz-ntn-isl --freq=300e9 --bandwidth=20e9"
```

#### thz-ntn-beam-tracking
EKF / position-based beam tracking through a LEO pass; drives a traffic helper for the
health report.
- **Key args:** `--trackingMode` (`EKF` | `POSITION_BASED`), `--updateRate` (Hz),
  `--passDuration` (s), `--maxElevation` (deg), `--outputDir` (def `thz-beam-track-out`).
- **Outputs:** `thz-beam-track-out/sim_health.csv` (traffic-helper health report).

```bash
./ns3 run thz-ntn-beam-tracking
./ns3 run "thz-ntn-beam-tracking --trackingMode=EKF --updateRate=20"
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
from actual Sat / RIS / GT mobility positions.
- **Key args:** `--freq` (Hz, def 300e9), `--risSize` (elements per side, def 64),
  `--elevation` (deg, def 30), `--txPower` (dBm, def 34.77).
- **Outputs:** console report (no CSV).

```bash
./ns3 run thz-ntn-ris-assisted
./ns3 run "thz-ntn-ris-assisted --risSize=64 --elevation=30"
```

#### thz-ntn-dband-constellation
D-band constellation UT-to-satellite association and link budgets.
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
- **Key args:** `--duration` (s, def 30), `--preset` (def `TeraLink-225GHz`),
  `--debrisRangeKm` (km, def 0.5).
- **Outputs:** console report (no CSV).

```bash
./ns3 run thz-ntn-full-stack
./ns3 run "thz-ntn-full-stack --preset=TeraLink-225GHz --debrisRangeKm=0.5"
```

### Real-data-plane traffic examples (NetDevice + IP + apps + FlowMonitor)

These build a point-to-point data plane gated by the THz channel and report
FlowMonitor statistics; those using the `ntn-traffic` helper also emit
`<outputDir>/sim_health.csv`.

#### thz-ntn-leo-ground-downlink-traffic
Real UDP downlink over a LEO sub-THz link gated by molecular absorption.
- **Key args:** `--simSeconds`, `--altKm`, `--satSpeed` (m/s), `--freqGHz`,
  `--dataRateMbps`, `--packetBytes`, `--txPowerDbm`, `--antennaGainDb`, `--noiseDbm`,
  `--minElevDeg`, `--linkCapacityMbps`.
- **Outputs:** FlowMonitor stats to console.

```bash
./ns3 run thz-ntn-leo-ground-downlink-traffic
./ns3 run "thz-ntn-leo-ground-downlink-traffic --freqGHz=225 --dataRateMbps=500"
```

#### thz-ntn-isl-traffic
Real packet transmission over a 300 GHz ISL gated by `ThzNtnIslChannel` SNR.
- **Key args:** `--simSeconds`, `--freqGHz`, `--txPowerDbm`, `--txGainDb`, `--rxGainDb`,
  `--bandwidthGHz`, `--startSepKm`, `--maxSepKm`, `--dataRateMbps`, `--packetBytes`,
  `--minSnrDb`, `--linkCapacityMbps`.
- **Outputs:** FlowMonitor stats to console.

```bash
./ns3 run thz-ntn-isl-traffic
./ns3 run "thz-ntn-isl-traffic --freqGHz=300 --maxSepKm=2000"
```

#### thz-ntn-ris-relay-traffic
RIS recovers a blocked THz link mid-simulation (real data plane).
- **Key args:** `--simSeconds`, `--altKm`, `--satSpeed`, `--freqGHz`, `--dataRateMbps`,
  `--packetBytes`, `--txPowerDbm`, `--antennaGainDb`, `--blockageDb`, `--risX`, `--risY`,
  `--phaseBits` (1–4), `--reflEff` (0–1), `--risOnFraction`, `--linkCapacityMbps`.
- **Outputs:** FlowMonitor stats to console.

```bash
./ns3 run thz-ntn-ris-relay-traffic
./ns3 run "thz-ntn-ris-relay-traffic --risX=32 --risY=32 --blockageDb=30"
```

#### thz-ntn-isac-coexist-traffic
ISAC comm/sense coexistence via `ThzNtnIsacScheduler` over a real data plane.
- **Key args:** `--simSeconds`, `--offeredMbps`, `--numSubBands`, `--numUes`,
  `--packetBytes`.
- **Outputs:** FlowMonitor stats to console.

```bash
./ns3 run thz-ntn-isac-coexist-traffic
./ns3 run "thz-ntn-isac-coexist-traffic --numSubBands=8 --offeredMbps=400"
```

#### thz-ntn-weather-traffic
Weather front (fog / rain / snow) over a THz downlink with a real data plane.
- **Key args:** `--simSeconds`, `--altKm`, `--satSpeed`, `--freqGHz`, `--dataRateMbps`,
  `--packetBytes`, `--txPowerDbm`, `--antennaGainDb`, `--rainMmH`, `--fogLwc` (g/m^3),
  `--snowMmH`, `--linkCapacityMbps`.
- **Outputs:** FlowMonitor stats to console.

```bash
./ns3 run thz-ntn-weather-traffic
./ns3 run "thz-ntn-weather-traffic --rainMmH=25 --freqGHz=225"
```

#### thz-ntn-ric-controlled-traffic
Closed-loop thz-ntn × oran-ntn demo: THz KPIs feed an `OranNtnE2Node` KPM report; an
xApp toggles the RIS when SINR crosses a threshold; data-plane goodput tracks the loop.
- **Key args:** `--simSeconds`, `--leoAltKm`, `--satSpeed`, `--freqGHz`, `--dataRateMbps`,
  `--packetBytes`, `--satEirpDbm`, `--rxGainDb`, `--sinrThreshDb`, `--xapp` (0/1),
  `--risN`, `--humidityProfile` (def `mid_latitude_summer`).
- **Outputs:** FlowMonitor stats to console.

```bash
./ns3 run thz-ntn-ric-controlled-traffic
./ns3 run "thz-ntn-ric-controlled-traffic --xapp=1 --sinrThreshDb=5"
```

> Note: `thz-ntn-isac.cc` (legacy ISAC API) is currently excluded from the build pending
> the Q4 2026 ISAC scheduler redesign and is not produced as a binary.

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
