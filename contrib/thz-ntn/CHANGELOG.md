# Changelog

## [Unreleased]

### Added
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
