<h1 align="center">thz-ntn</h1>

<p align="center"><strong>Sub-THz / D-band non-terrestrial PHY for ns-3.43 — LEO-ground & ISL links, RIS, ultra-massive MIMO, ISAC, alpha-mu fading, NYUSIM-140 calibration, ITU-R P.676-13 gaseous absorption</strong></p>

> ## Two examples now couple their physics to their packets, and the link does not close (audit WF-06)
>
> `thz-ntn-leo-ground` and `thz-ntn-isl` computed a full link budget — FSPL, molecular absorption,
> weather, scintillation, pointing — printed it to CSV, and then carried their packets over a
> point-to-point star with a **hardcoded 15 ms delay and a `RateErrorModel` pinned at 0.0**. There
> was no SpectrumPhy, no MAC, no SINR and no TBLER in the data path, so nothing the physics
> computed could affect a single packet. The helper's coupling hook,
> `UpdateUeLink(ueIndex, oneWayDelay, per)`, had **zero callers across all 94 example files**.
>
> Both now apply it: the delay is the real slant range over *c*, and the error rate comes from the
> SNR the budget just produced, mapped through the vendored SNS3 **DVB-S2 forward-link tables** —
> a measured per-MODCOD curve rather than a sigmoid. (THz links do not use DVB-S2 MODCODs; it is a
> stand-in for a THz-native waterfall and is labelled as one, but it has real thresholds, which a
> PER of exactly zero did not.)
>
> **The measured consequence is that these configurations do not close.** At the shipped defaults
> the LEO-ground pass computes an SNR of **−18.3 to −15.1 dB** across its whole 60 s, so the PER is
> 1.0 throughout and `rx/tx` goes from 1 to 0. That is the correct answer, and it was completely
> hidden while every packet arrived regardless of the physics. The dominant term is the noise
> bandwidth: `--bandwidth=10e9` puts the thermal floor at −174 + 100 dB. Measured sweep:
> 10 GHz and 1 GHz do not close, **100 MHz does** (PER 0, `rx/tx` 1).
>
> The defaults are left as they are rather than quietly narrowed — changing them to make the demo
> deliver would manufacture a result. Both examples now print the verdict and name the knob.

> ## Which results carry packets, and at what frequency (audit THZ-07)
>
> This module has two kinds of example and they are not interchangeable.
>
> **Above 100 GHz: link-budget analysis only, no packets.** `thz-ntn-leo-ground` (225 GHz),
> `thz-ntn-demo` (300 GHz), `thz-ntn-isac`, `thz-ntn-um-mimo`, `thz-ntn-isl`,
> `thz-ntn-ris-assisted` (all 300 GHz) and `thz-ntn-dband-constellation` (140 GHz) contain **no**
> `InternetStackHelper`, `NetDeviceContainer`, `PointToPointHelper`, `FlowMonitor`, `OnOffHelper`
> or `PacketSink` — verified by grep, zero occurrences in all seven. `thz-ntn-demo` has no
> `Simulator::Run` at all. They compute SNR, capacity, absorption and beam geometry into CSV. Any
> throughput they report is a Shannon bound, not a measured goodput.
>
> **Packets: capped at 100 GHz.** Every example that actually carries traffic —
> `thz-ntn-beam-tracking`, `thz-ntn-ris-relay-traffic`, `thz-ntn-ric-controlled-traffic`,
> `thz-ntn-isac-coexist-traffic`, `thz-ntn-isl-traffic`, `thz-ntn-weather-traffic`,
> `thz-ntn-leo-ground-downlink-traffic` — forces `freqGHz = 100.0`. That is not a choice: the
> in-tree 3GPP spectrum model asserts `500 MHz ≤ f ≤ 100 GHz`
> (`three-gpp-propagation-loss-model.cc:358`, and the same bound in `three-gpp-channel-model.cc`),
> so the NR/mmwave bridge cannot be driven above it.
>
> **THZ-07 UPDATE — there is now one.** `thz-ntn-native-300ghz-traffic` carries real IP traffic
> at **300 GHz** by not using the 3GPP model at all: the propagation chain is
> `FriisPropagationLossModel` → `ThzNtnPropagationLossModel` (free space plus this module's own
> gaseous, weather, scintillation and pointing terms), neither of which has a frequency cap, and
> `ThzNtnLinkErrorModel` puts that chain in the packet path — asking it per packet what the
> received power is for the current geometry. Measured on the default run: 1.27 M packets,
> 484.5 Mbps goodput, SNR falling 10.18 → 8.77 dB across the pass. The physics is load-bearing:
> 25 mm/h of rain takes the SNR to **−32.8 dB and the PDR to 0%**, a 43 dB penalty.
>
> **Honest scope:** that example is not a SpectrumPhy — no per-subcarrier processing, no MAC, no
> HARQ, no scheduler. Read its throughput as a link-limited transport figure, not as an
> NR-at-THz result. The paragraph below still describes every OTHER packet-carrying example in
> this module, which remain capped at 100 GHz.
>
> **Previously, and still true of the rest:** the module's headline band (200–400 GHz) has no example in which a packet is
> ever transmitted, and every measured-plane THz result in this repo is a W-band result.**
> Closing that needs a THz-native spectrum channel that bypasses `ThreeGppPropagationLossModel`;
> it is not done, and the title above should be read with that in mind.

> ## Small-scale fading: what is and is not modelled (audit BOTH-01)
>
> **There is no TR 38.811 §6.9 NTN-TDL or CDL multipath in this module.** A grep for
> `ntn-tdl|NtnTdl|CDL` across its model and helper sources returns only bibliography lines. The
> small-scale processes that do exist are the ITU-R P.681-11 Lutz two-state model (environment-
> keyed, **not** elevation-dependent), the alpha-mu model, and a hand-written four-tap snapshot
> inside one example. None is parameterised by elevation angle.
>
> Concretely, this module produces **no frequency-selective fading, no delay spread, and no
> elevation-dependent Rician K-factor.** Any BLER or throughput number from it reflects a flatter,
> smoother channel than a real NTN link. `SCOPE_AND_LIMITATIONS.md` A1 records that
> `ntn-traffic`'s excess-loss chain *does* carry the §6.7.2 elevation-dependent K-factor; that
> statement does **not** extend here, and this note exists because it previously was not repeated
> anywhere a reader of this module would look.
>
> The fix is an `NtnTdlSpectrumPropagationLossModel` carrying the Table 6.9.2-x tap powers and
> delays with an elevation-interpolated K-factor, chainable through
> `NtnRealStackHelper::AddExtraPropagationLoss`. It is not implemented.

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0--only-green.svg"/></a>
  <img src="https://img.shields.io/badge/ITU--R-P.676--13-orange.svg"/>
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
- **Molecular absorption** from the **ITU-R P.676-13 Annex 1** line-by-line model
  (`ThzNtnMolecularAbsorption`) over the ITU-R P.835-6 Section 1.1 reference atmosphere.
  The in-process line database is the full P.676-13 set (**44 O2 lines from
  Table 1 + 35 H2O lines from Table 2**) with the standard resonant line shape
  and the dry/Debye continuum; it supersedes the earlier hand-scaled "HITRAN"
  intensities and reproduces the ITU-Rpy P.676-13 validation set to within ~2 %.
  Gaseous absorption is therefore **non-monotonic across 2–90 GHz, peaking at
  the 60 GHz O2 complex** (not a monotone rise with frequency). The optional
  lookup-table path (`HitranLut` in namespace `ns3::thzntn`,
  `data/hitran2024-lut-subthz.csv` — the file keeps its legacy name) is an exact
  grid sample of the same P.676-13 kernel, regenerated by `tools/p676-lut-gen.py`
  (which replaced the old `hitran2024-lut-gen.py`); no external HITRAN `.par`
  file is needed, as P.676-13 Annex 1 is self-contained
- **Weather attenuation** (rain / fog / snow) per ITU-R P.838-3 (rain) / P.840 (fog) extended to THz
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
- **alpha-mu fading**, an **ITU-R P.676-13 gaseous-absorption LUT**
  (`tools/p676-lut-gen.py`, an exact grid sample of the in-process P.676-13
  kernel; see Overview), **NYUSIM-140** calibration reference, and a **RIS
  service model + xApp** for O-RAN-style closed-loop control.
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
| `ThzNtnMolecularAbsorption` (`thz-ntn-molecular-absorption`) | ITU-R P.676-13 Annex 1 line-by-line gaseous absorption (44 O2 + 35 H2O lines) over the ITU-R P.835-6 Section 1.1 reference atmosphere |
| `HitranLut` (`thz-ntn-hitran-lut`, namespace `ns3::thzntn`) | Bundled specific-attenuation lookup table (`data/hitran2024-lut-subthz.csv`, legacy filename) — an exact grid sample of the ITU-R P.676-13 kernel, regenerated by `tools/p676-lut-gen.py` (see Overview) |
| `Itu838RainModel`, `Itu618LossModel`, `Itu676AbsorptionModel`, `Itu681LmsModel` (`thz-ntn-itu-recommendations`) | ITU-R P.838-3 / P.618-13 / P.676-13 / P.681-11 reference implementations |
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

## Scope & limitations (toolkit boundaries)

**A2** — the THz array/beamforming/pointing physics is computed in the offline link-budget calculator; the *measured* `*-traffic` examples carry FSPL + atmosphere only (no pointing/beam-squint loss in the measured KPIs). See the toolkit-wide [`SCOPE_AND_LIMITATIONS.md`](../../SCOPE_AND_LIMITATIONS.md) for the authoritative statement of what is and is not modelled.
