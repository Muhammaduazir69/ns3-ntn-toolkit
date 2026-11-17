# Changelog

## [Unreleased]

### Wave 7C (Nov 2026)
- ITU-R P.840-9 cloud attenuation (`ThzNtnCloudAttenuation`) on the default
  packet path of `ThzNtnPropagationLossModel` and
  `ThzNtnSpectrumAtmosphereModel`: K_L(f) of eq. (13), site log-normal liquid
  water statistics from the Recommendation's maps (1 degree nodes), one
  liquid water state per run, reported per run. `Mode=Off` reproduces older runs.
- ITU-R P.834-9 free-space to apparent elevation (eqs. 8 to 14) at the
  slant-path input of both models (`ApparentElevation=false` reproduces
  older runs). `ThzNtnPropagationLossModel` now uses `ThzNtnLinkGeometry`, so
  ECEF endpoints (SGP4) get WGS-84 altitudes instead of z.
- Liquid-water K_l uses the P.840-9 constants (was P.840-4/5 with an
  unsourced 146.4).
- `ThzNtnLinkErrorModel`: DVB-S2 BLER decode rule, sky-noise antenna
  temperature, 1 ms SNR reuse window.
- `ThzNtnPacketLink` helper: live slant-range delay, DVB-S2 ACM with one-RTT
  CSI age, pointing and scintillation, per-update CSV.
- Examples `thz-ntn-isac` (ported to the current API and built again),
  `-absorption-sweep`, `-ris-assisted`, `-altitude-relay`, `-demo`,
  `-native-300ghz-traffic`, `-um-mimo` run a measured packet companion
  (`examples/thz-ntn-pass-kit.h`): SGP4 satellite, TR 38.811 terminal,
  ntn-traffic 5QI flow, KPM series, FlowMonitor, run manifest.

### Fixed
- THZ-13: the bundled table `data/hitran2024-lut-subthz.csv` and its generator
  still carried the pre-THZ-01 six-layer atmosphere. Regenerated from the
  ITU-R P.835-6 Sec. 1.1 closed form on a 1 GHz x 1 km grid, 100-1000 GHz,
  with log-linear altitude interpolation; the table declares its atmosphere
  and is bypassed when the configured profile or water-vapour scale differs.
- THZ-14: `HumidityProfile` names selected invented altitude-band multipliers;
  they now select the ITU-R P.835-6 Sec. 2 seasonal atmospheres. Default
  `global-annual` is unchanged bit for bit.

### Added
- `WaterVapourScale` attribute and `GetWaterVapourColumn()`.
- Example `thz-ntn-absorption-sweep` (frequency x elevation x column CSV).
- `HitranLut` (namespace `ns3::thzntn`): bundled HITRAN-2024 specific-attenuation
  lookup table + generator script (`tools/hitran2024-lut-gen.py`)
- ITU-R reference wrappers: `Itu838RainModel`, `Itu618LossModel`,
  `Itu676AbsorptionModel`, `Itu681LmsModel` (`thz-ntn-itu-recommendations`)
- `ThzNtnAlphaMuFading` small-scale fading
- `ThzNtnIsacScheduler` comm/sense sub-band partitioning
- `ThzNtnNyusimReference` / `ThzNtnNyusimCalibrator` (NYUSIM-140 calibration,
  `data/nyusim-140-reference.csv`)
- `ThzNtnRisServiceModel` + `ThzNtnRisXapp` for O-RAN-style closed-loop RIS control
- `ThzNtnPropagationLossModel`: molecular absorption + weather as a real
  `PropagationLossModel`, chainable onto a live spectrum channel

### Changed
- All traffic examples converted from P2P/error-model data planes to a real
  mmwave NR NTN stack (`NtnRealStackHelper`) with SGP4/Walker satellite mobility,
  TR 38.811 ground terminals, `NtnOranApplication` QoS flows measured at
  `NtnOranSink`, and honest `sim_health.csv` reports; new flagship example
  `thz-ntn-real-stack`

## [1.0.0] - 2026-04-20

Initial public release.

### Added
- 24 research-grade THz-NTN model classes covering channel, PHY, MAC, antenna, beamforming, ISL, RIS, ISAC
- HITRAN-based altitude-stratified molecular absorption (100 GHz - 10 THz); the
  bundled lookup table is pinned to HITRAN-2024 (`data/hitran2024-lut-subthz.csv`,
  `kHitranRelease = "HITRAN-2024"`)
- ITU-R P.838 / P.840 weather attenuation extended to THz
- ITU-R P.618 scintillation (amplitude + phase) with AR(1) time series
- Hardware impairments: PA (Rapp/Saleh), phase noise (Lorentzian), ADC SQNR, I/Q imbalance
- Ultra-Massive MIMO arrays (UPA/UCA/Cassegrain) up to 128x128 elements
- Hierarchical DFT codebook with beam squint compensation
- Extended Kalman Filter beam tracking with satellite ephemeris
- Inter-satellite link channel with Dijkstra routing
- Reconfigurable Intelligent Surfaces (space-borne / aerial / ground)
- ISAC for space debris detection (cm-resolution)
- 5 THz waveforms: OFDM / DFT-s-OFDM / OTFS / AFDM / SC-FDE
- 5 standard atmospheric windows (140, 220, 340, 410, 460 GHz)
- O-RAN integration with 4 new THz xApps
- NTN-CHO extensions: THz beam quality trigger + THz TTE estimation
- Demo with 8 scenarios and CSV dataset generation
- 38 unit tests -- all passing
- Full documentation (EXAMPLES, MODULE_REFERENCE, INTEGRATION, VALIDATION)

### Integration
- `ThzNtnFreeSpaceLoss` extends `SatFreeSpaceLoss` -- drop-in for satellite pipeline
- `ThzNtnPhy` uses `SpectrumValue` + `mmWaveInterference` for real SINR
- `ThzNtnPhySat` delegates MCS via `SatWaveformConf::GetBestWaveformId()`
- `ThzNtnMacScheduler` delegates MCS via `MmWaveAmc::GetMcsFromSpectralEfficiency()`
- Distance / Doppler / altitude all derived from `MobilityModel`
