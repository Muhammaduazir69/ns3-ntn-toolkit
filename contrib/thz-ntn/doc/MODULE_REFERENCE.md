# THz-NTN Module Reference

Per-class reference documentation for the module's core classes.
Each section covers: purpose, public API, ns-3 attributes, and key formulas.
Classes added after the first release are summarised in
[Newer classes (v2)](#newer-classes-v2) at the end; their headers in `model/`
are the authoritative reference.

---

## 1. ThzNtnMolecularAbsorption

**Header:** `thz-ntn-molecular-absorption.h`
**Base:** `ns3::Object`
**Purpose:** HITRAN-based molecular absorption through altitude-stratified atmosphere (ITU-R P.835).

### Key API
```cpp
double ComputeAbsorptionLoss_dB(
    double freqHz,          // 100 GHz - 10 THz
    double distanceM,
    double altitudeTx_km,
    double altitudeRx_km,
    double elevationDeg) const;

double ComputeAbsorptionCoefficient(
    double freqHz, double tempK,
    double pressureHPa, double humidityPct) const;

double ComputeSlantPathAbsorption(
    double freqHz, double elevationDeg,
    double groundAlt_km, double satAlt_km) const;

void SetHumidityProfile(const std::string& profile);  // tropical / midlatitude-summer / winter / subarctic
double GetTransmittance(double freqHz, double distanceM, double altitudeAvg_km) const;
```

### Attributes
| Attribute | Default | Description |
|---|---|---|
| `HumidityProfile` | `"midlatitude-summer"` | Atmospheric humidity profile |
| `IntegrationSteps` | `50` | Number of atmospheric layer slices |

### Key formula (Van Vleck-Weisskopf line shape)
```
k(f, T, P, q) = sum_i [S_i(T) * g_VVW(f, f_0i, gamma_i)] + k_continuum
```
where `S_i` is line intensity, `gamma_i` is pressure-broadened half-width, `f_0i` is line centre.

### ISL behaviour
Returns **exactly 0 dB** when both altitudes > 100 km (vacuum).

---

## 2. ThzNtnWeatherAttenuation

**Header:** `thz-ntn-weather-attenuation.h`
**Base:** `ns3::Object`
**Purpose:** Rain (ITU-R P.838), fog/cloud (P.840), snow, sand/dust attenuation extended to THz.

### Key API
```cpp
double ComputeRainAttenuation_dB(double freqHz, double elevationDeg, double rainRate_mm_h) const;
double ComputeFogAttenuation_dB(double freqHz, double elevationDeg, double lwc_g_m3) const;
double ComputeSnowAttenuation_dB(double freqHz, double elevationDeg, double snowRate_mm_h, bool isWet) const;
double ComputeDustAttenuation_dB(double freqHz, double elevationDeg, double visibility_km) const;
double ComputeTotalWeatherLoss_dB(double freqHz, double elevationDeg) const;
```

### Attributes
| Attribute | Default | Description |
|---|---|---|
| `RainRate` | 0 | mm/h |
| `LiquidWaterContent` | 0 | g/m^3 (clouds/fog) |
| `SnowRate` | 0 | mm/h |
| `DustVisibility` | 100 | km |
| `EnableRain` / `EnableFog` / `EnableSnow` / `EnableDust` | false | Per-effect enables |
| `ClimateRegion` | `MIDLAT_SUMMER` | tropical / midlat-summer / midlat-winter |

### Key formula (ITU-R P.838)
```
gamma_R = k * R^alpha   (dB/km)
```
Coefficients table covers 1 GHz - 1000 GHz with log-log interpolation.
At 100 GHz: k≈1.31, alpha≈0.79. At 500 GHz: k≈3.20, alpha≈0.64.

---

## 3. ThzNtnScintillation

**Header:** `thz-ntn-scintillation.h`
**Base:** `ns3::Object`
**Purpose:** Amplitude + phase scintillation time series (ITU-R P.618 extension).

### Key API
```cpp
double ComputeAmplitudeScintillation_dB(double freqHz, double elevationDeg) const;
double ComputePhaseScintillation_rad(double freqHz, double elevationDeg) const;
double GetScintillationSample_dB(double freqHz, double elevationDeg) const;  // AR(1) sample
double ComputeScintillationFadeDepth_dB(double freqHz, double elevationDeg, double percentage) const;
int64_t AssignStreams(int64_t stream);
```

### Attributes
| Attribute | Default | Description |
|---|---|---|
| `TurbulenceStrength` | `"moderate"` | weak / moderate / strong |
| `WindSpeed_m_s` | 10 | Tropospheric wind speed |
| `GroundTemperature_K` | 293 | Ground temperature |
| `SamplingPeriod_s` | 0.1 | Time between AR(1) samples |

### Key formula
```
sigma^2 ~ f^(7/6) * csc(theta)^(11/6) * Cn2_integral
```
Hufnagel-Valley turbulence profile integrated from ground to 20 km.

---

## 4. ThzNtnPointingError

**Header:** `thz-ntn-pointing-error.h`
**Base:** `ns3::Object`
**Purpose:** RSS of vibration, J2 perturbation, atmospheric refraction, and tracking latency errors.

### Key API
```cpp
double ComputePointingError_deg(double elevationDeg, double satVelocity_km_s) const;
double ComputePointingLoss_dB(double pointingError_deg, double beamwidth3dB_deg) const;
double ComputeTotalPointingLoss_dB(double elevationDeg, double satVelocity_km_s, double beamwidth3dB_deg) const;
double GetPointingErrorSample_deg() const;
```

### Attributes
| Attribute | Default | Description |
|---|---|---|
| `VibrationRms_deg` | 0.02 | Platform vibration RMS |
| `EphemerisUpdateInterval_s` | 10.0 | J2 drift accumulation window |
| `TrackingUpdateRate_Hz` | 100.0 | Beam update rate |
| `TrackingLatency_ms` | 1.0 | Beam-steering latency |
| `EnableAtmosphericRefraction` | true | ITU-R P.834 refraction |

### Key formulas
```
total_err = sqrt(vib^2 + J2^2 + refr^2 + track^2)   # RSS combination
loss_dB  = 12 * (err / BW_3dB)^2                    # parabolic
```

---

## 5. ThzNtnHardwareImpairments

**Header:** `thz-ntn-hardware-impairments.h`
**Base:** `ns3::Object`
**Purpose:** Realistic capacity ceiling from PA, phase noise, ADC, I/Q imbalance.

### Key API
```cpp
double ComputeEvmTotal() const;
double ComputePaNonlinearityEvm(double iboDb) const;
double ComputePhaseNoiseEvm(double freqHz, double subcarrierSpacingHz) const;
double ComputeAdcQuantizationNoise_dBm(double signalPower_dBm) const;
double ComputeIqImbalanceEvm() const;
double ComputeHardwareLimitedCapacity(double snr_linear) const;  // bits/s/Hz ceiling
double ComputeEffectiveSnr_dB(double idealSnr_dB) const;
```

### Attributes
| Attribute | Default | Description |
|---|---|---|
| `PaModel` | `"rapp"` | rapp / saleh |
| `PaSaturation_dBm` | 30 | PA saturation point |
| `PhaseNoiseLinewidth_Hz` | 1000 | Lorentzian 3 dB linewidth |
| `AdcBits` | 6 | THz-rate ADC bits |
| `IqAmplitudeImbalance_dB` | 1.0 | |
| `IqPhaseImbalance_deg` | 3.0 | |
| `EnableDpd` | false | Digital pre-distortion |
| `InputBackoff_dB` | 6.0 | |

### Key formulas
```
SQNR  = 6.02*b + 1.76 dB           # ADC quantization
C_hw  = log2(1 + SNR/(1+SNR*k^2))  # hardware-limited capacity
```
Ceiling ~7.4 bits/s/Hz with DPD, ~2.8 bits/s/Hz without.

---

## 6. ThzNtnSpectrum

**Header:** `thz-ntn-spectrum.h`
**Base:** `ns3::Object`
**Purpose:** Atmospheric window database and band classification.

### Key API
```cpp
static std::vector<AtmosphericWindow> GetStandardWindows();
std::vector<AtmosphericWindow> GetAvailableWindows(double minBwGHz, double maxAtten_dB) const;
AtmosphericWindow GetBestWindow(double requiredBwGHz) const;
double GetWindowTransmittance(double freqGHz, double elevationDeg) const;
bool IsInAtmosphericWindow(double freqGHz) const;
std::string GetBandName(double freqGHz) const;
```

### Standard windows
| # | Centre | BW | Peak Transmittance | Zenith Atten |
|---|--------|----|--------------------|--------------|
| 1 | 140 GHz | 20 GHz | 0.85 | 1.2 dB |
| 2 | 220 GHz | 30 GHz | 0.75 | 2.5 dB |
| 3 | 340 GHz | 25 GHz | 0.55 | 4.8 dB |
| 4 | 410 GHz | 15 GHz | 0.45 | 6.5 dB |
| 5 | 460 GHz | 10 GHz | 0.35 | 8.2 dB |

### Band enums
`D_BAND_LOW` (110-140), `D_BAND_HIGH` (140-170), `G_BAND` (170-220), `H_BAND` (220-325), `SUB_THZ` (100-300), `THZ_LOW` (300-500), `THZ_MID` (500 GHz-1 THz), `THZ_HIGH` (1-10 THz), `ISL_ANY`.

---

## 7. ThzNtnLinkBudget

**Header:** `thz-ntn-link-budget.h`
**Base:** `ns3::Object`
**Purpose:** End-to-end link budget with all losses, hardware ceiling, and capacity.

### Key API
```cpp
LinkBudgetResult ComputeLinkBudget(Ptr<MobilityModel> tx, Ptr<MobilityModel> rx,
                                   double txPowerDbm, double txGainDbi, double rxGainDbi,
                                   double bandwidthHz, double noiseFigureDb) const;

LinkBudgetResult ComputeTeraLinkBudget(Ptr<MobilityModel> sat, Ptr<MobilityModel> ground) const;
LinkBudgetResult ComputeDbandLeoBudget(Ptr<MobilityModel> sat, Ptr<MobilityModel> ground) const;
LinkBudgetResult ComputeIslBudget(Ptr<MobilityModel> sat1, Ptr<MobilityModel> sat2, double freqHz) const;

// Setters to wire the full stack:
void SetFreeSpaceLossModel(Ptr<ThzNtnFreeSpaceLoss> fsl);
void SetHardwareModel(Ptr<ThzNtnHardwareImpairments> hw);
void SetMolecularAbsorptionModel(Ptr<ThzNtnMolecularAbsorption> model);
void SetWeatherModel(Ptr<ThzNtnWeatherAttenuation> model);
void SetScintillationModel(Ptr<ThzNtnScintillation> model);
void SetPointingErrorModel(Ptr<ThzNtnPointingError> model);
```

### LinkBudgetResult struct
```
double txPower_dBm, txAntennaGain_dBi, eirp_dBm;
double fspl_dB, molecularAbsorption_dB, weatherLoss_dB, scintillationLoss_dB, pointingLoss_dB;
double totalPathLoss_dB;
double rxAntennaGain_dBi, rxPower_dBm;
double noiseFigure_dB, thermalNoise_dBm, noisePower_dBm;
double snr_dB, hardwareLimitedSnr_dB;
double shannonCapacity_Gbps, hardwareLimitedCapacity_Gbps;
double linkMargin_dB;
double distanceM, elevationDeg;
```

---

## 8. ThzNtnFreeSpaceLoss (deep integration point)

**Header:** `thz-ntn-free-space-loss.h`
**Base:** `ns3::SatFreeSpaceLoss` (from satellite module)
**Purpose:** Drop-in replacement for the satellite module's FSPL calculator -- auto-applies all THz losses when `SatChannel` pipeline calls `GetFsl()`.

### Key API
```cpp
double GetFsl(Ptr<MobilityModel> a, Ptr<MobilityModel> b, double freqHz) const override;
double GetFsldB(Ptr<MobilityModel> a, Ptr<MobilityModel> b, double freqHz) const override;

void SetMolecularAbsorptionModel(Ptr<ThzNtnMolecularAbsorption> model);
void SetWeatherModel(Ptr<ThzNtnWeatherAttenuation> model);
void SetScintillationModel(Ptr<ThzNtnScintillation> model);
void SetPointingErrorModel(Ptr<ThzNtnPointingError> model);
void EnableMolecularAbsorption(bool on);
void EnableWeatherEffects(bool on);
void EnableScintillation(bool on);
void EnablePointingError(bool on);

double GetLastBaseFspl_dB() const;
double GetLastMolecularAbsorption_dB() const;
double GetLastWeatherLoss_dB() const;
double GetLastScintillationLoss_dB() const;
double GetLastPointingLoss_dB() const;

TracedCallback<double, double, double, double, double> m_thzLossBreakdown;
```

---

## 9. ThzNtnChannelModel

**Header:** `thz-ntn-channel-model.h`
**Base:** `ns3::PropagationLossModel` (for mmWave pipeline)
**Purpose:** Composite channel model orchestrator for mmWave `SpectrumChannel` integration.

### Key API
```cpp
Ptr<ThzNtnFreeSpaceLoss> GetThzFreeSpaceLoss() const;  // expose for satellite pipeline

void SetLinkType(LinkType);  // GROUND_TO_SAT / INTER_SATELLITE / AUTO_DETECT
void SetBand(const std::string& band);  // "TeraLink-225GHz", "D-band-140GHz", "ISL-300GHz"

ThzNtnSignalInfo GetLinkSignalInfo(uint32_t nodeA, uint32_t nodeB) const;

protected:
  double DoCalcRxPower(double txPowerDbm, Ptr<MobilityModel> a, Ptr<MobilityModel> b) const override;
  int64_t DoAssignStreams(int64_t) override;
```

---

## 10. ThzNtnPhy / ThzNtnPhySat / ThzNtnPhyGround

**Purpose:** PHY layer with actual `SpectrumValue`-based SINR.

### Key shared API
```cpp
Ptr<SpectrumModel> CreateThzSpectrumModel(double centerFreqHz, double bwHz, uint32_t numSubBands) const;
Ptr<SpectrumValue> CreateTxPsd(double txPowerDbm) const;
Ptr<SpectrumValue> CreateNoisePsd() const;

double ComputeSinrFromSpectrum(Ptr<const SpectrumValue> rxPsd,
                                Ptr<const SpectrumValue> noisePsd,
                                Ptr<const SpectrumValue> interferencePsd) const;

double ComputeDopplerFromMobility(Ptr<MobilityModel> satMob, Ptr<MobilityModel> groundMob) const;
uint32_t SelectSubcarrierSpacing_Hz(Ptr<MobilityModel> satMob, Ptr<MobilityModel> groundMob) const;

void SetAmcModel(Ptr<mmwave::MmWaveAmc> amc);
uint8_t GetMcsFromSinr(double sinrDb) const;
double GetSpectralEfficiencyFromMcs(uint8_t mcs) const;

void SetInterferenceModel(Ptr<mmwave::mmWaveInterference> iface);
void AddInterference(Ptr<const SpectrumValue> psd, Time duration);

void SetWaveform(WaveformType wf);
```

### ThzNtnPhySat extensions
```cpp
void SetSatellitePhy(Ptr<SatPhy>);
void SetWaveformConf(Ptr<SatWaveformConf>);
SatEnums::SatModcod_t SelectModCod(double cnoDb) const;
void SetSatelliteMobility(Ptr<MobilityModel>);  // for Doppler pre-comp
double ComputeDopplerPreCompensation(Ptr<MobilityModel> ground) const;
Time ComputeOnBoardProcessingDelay() const;
void SetPayloadMode(PayloadMode);  // TRANSPARENT / REGENERATIVE
void SetSatelliteClass(SatelliteClass);  // CUBESAT / SMALLSAT / FULLSAT
void ProcessRegenerativePayload(Ptr<SatSignalParameters>);
```

### ThzNtnPhyGround extensions
```cpp
void SetUtPhy(Ptr<SatPhy>);
void SetReceiverType(ReceiverType);  // HETERODYNE / DIRECT_DETECTION / HOMODYNE
void SetPolarization(Polarization);
double EstimateDopplerFromSignal(Ptr<SatSignalParameters>) const;
void GenerateCqiFeedback(Ptr<const SpectrumValue> sinr, uint8_t& wbCqi, uint8_t& wbMcs);
double ComputeAgcGain(double instantRxPowerW, double targetRxPowerW) const;
```

---

## 11. ThzNtnWaveform

**Header:** `thz-ntn-waveform.h`
**Purpose:** Waveform-specific parameters and auto-selection for THz LEO.

### Waveforms supported
| Enum | PAPR | Doppler Rob | SE factor | Complexity | ISAC | Use case |
|---|---|---|---|---|---|---|
| `OFDM` | 11 dB | 0.30 | 1.00 | 1.0 | No | Default, downlink |
| `DFT_S_OFDM` | 7 dB | 0.40 | 0.95 | 1.2 | No | Uplink 3GPP NR |
| `OTFS` | 9 dB | **0.95** | 0.98 | 3.0 | No | **LEO high-Doppler** |
| `AFDM` | 8 dB | 0.85 | 0.92 | 2.5 | **Yes** | ISAC sensing-centric |
| `SC_FDE` | 2 dB | 0.70 | 0.90 | 0.8 | No | ISL (low-PAPR) |

---

## 12. ThzNtnMac / ThzNtnMacScheduler

**Purpose:** Ultra-wideband MAC with distance-aware multi-carrier (DAMC) and QoS-aware scheduling.

### ThzNtnMac key API
```cpp
void ConfigureResourceGrid(double totalBwHz, double subBandWidthHz, double centerFreqHz);
void SetMolecularAbsorptionModel(Ptr<ThzNtnMolecularAbsorption>);
void UpdateSubBandAvailability(Ptr<MobilityModel> tx, Ptr<MobilityModel> rx);
std::vector<SubBand> GetAvailableSubBands(double maxAbsorption_dB) const;
void AllocateSubBands(uint32_t ueId, uint32_t num, Ptr<MobilityModel> mob);
void SetSuperframeConf(Ptr<SatSuperframeConf>);
void SetWaveformConf(Ptr<SatWaveformConf>);
double ComputeAggregateCapacity_Gbps(const std::vector<double>& sinrs, Ptr<SatWaveformConf>) const;
```

### ThzNtnMacScheduler key API
```cpp
std::vector<SchedulingDecision> Schedule(const std::vector<UeContext>& ues,
                                          const std::vector<SubBand>& avail);
void SetAmc(Ptr<mmwave::MmWaveAmc>);
uint8_t SelectMcs(double sinrDb) const;  // uses actual AMC
uint32_t SelectScs(Ptr<MobilityModel> sat, Ptr<MobilityModel> ue) const;  // Doppler-aware
mmwave::DciInfoElementTdma CreateDci(const SchedulingDecision&) const;
```

---

## 13. ThzNtnAntennaArray

**Purpose:** UM-MIMO array with UPA / UCA / Cassegrain geometries.

### Key API
```cpp
void Configure(uint32_t nx, uint32_t ny, double freqHz, ThzArrayType type);
double ComputeArrayGain_dBi(double theta, double phi, double steerTheta, double steerPhi) const;
double ComputeMaxGain_dBi() const;
double ComputeBeamwidth3dB_deg() const;
double ComputeArrayFactor(double theta, double phi, double steerTheta, double steerPhi) const;
double ComputeElementPattern_dBi(double theta) const;
double ComputeCassegrainGain_dBi(double theta, double D, double freqHz) const;
double ComputeMutualCouplingLoss_dB() const;
double ComputeNearFieldDistance_m() const;
bool IsInNearField(double distance) const;
double ComputePhysicalSize_m() const;

// Integration with satellite module:
void SetSatAntennaGainPattern(Ptr<SatAntennaGainPattern>);
double GetGainFromPosition(Ptr<MobilityModel> sat, Ptr<MobilityModel> target) const;
double ComputeArrayGainToward(Ptr<MobilityModel> target, Ptr<MobilityModel> self) const;
```

### Key formulas
```
G_max    = 10*log10(N) + G_element     # broadside directivity
BW_3dB   = 0.886 * lambda / (N*d) rad  # at d = lambda/2
d_F      = 2*D^2/lambda                # Fraunhofer near-field
```

---

## 14. ThzNtnBeamforming

**Purpose:** Hierarchical DFT codebook + hybrid analog/digital beamforming with beam-squint.

### Key API
```cpp
void GenerateCodebook(uint32_t numBeams, ThzCodebookLevel level);
uint32_t SelectBeam(double theta, double phi, ThzCodebookLevel level) const;
BeamState PerformHierarchicalSearch(double theta, double phi) const;
uint32_t SelectBeamForTarget(Ptr<MobilityModel> target, Ptr<MobilityModel> self, ThzCodebookLevel) const;
BeamState PerformHierarchicalSearch(Ptr<MobilityModel> target, Ptr<MobilityModel> self) const;

double ComputeBeamSquintAngle_deg(double f, double f_c, double steerAngle) const;
double ComputeBeamSquintLoss_dB() const;
double ComputeBeamSwitchingDelay_us() const;
void SetBeamformingMode(ThzBeamformingMode);  // ANALOG_ONLY / HYBRID / DIGITAL_ONLY
void SetMmWaveBeamformingModel(Ptr<mmwave::MmWaveBeamformingModel>);
```

### Codebook levels
| Level | Number of beams |
|---|---|
| `WIDE` | 4 |
| `MEDIUM` | 16 |
| `NARROW` | 64 |
| `ULTRA_NARROW` | 256 |

---

## 15. ThzNtnBeamTracking

**Purpose:** Extended Kalman Filter beam prediction using satellite ephemeris.

### State vector
```
x = [theta, phi, d_theta/dt, d_phi/dt]^T
```

### Key API
```cpp
void Initialize(double theta, double phi, double d_theta, double d_phi);
void UpdateMeasurement(double theta_meas, double phi_meas, double sinrDb, double t);
std::pair<double,double> PredictBeamDirection(double futureTime_s) const;
std::pair<double,double> PredictFromEphemeris(double satLat, double satLon, double satAlt,
                                               double ueLat, double ueLon) const;
std::pair<double,double> PredictFromMobility() const;  // uses stored MobilityModel

void SetSatelliteMobility(Ptr<MobilityModel>);
void SetUeMobility(Ptr<MobilityModel>);
void OnSinrMeasurement(double sinr_dB);  // connect to PHY TracedCallback
bool DetectBeamFailure() const;
void TriggerBeamRecovery();
TrackingMetrics GetTrackingMetrics() const;
```

### Attributes
| Attribute | Default | Description |
|---|---|---|
| `TrackingMode` | `"EKF"` | EKF / POSITION_BASED / ML_ASSISTED |
| `BeamFailureThreshold_dB` | -10 | SINR below which failure is declared |
| `ConsecutiveFailures` | 5 | Measurements for failure decision |
| `ProcessNoise` | 0.01 | Q matrix scale |
| `MeasurementNoise` | 0.1 | R matrix scale |
| `UpdateRate_Hz` | 100 | EKF update rate |

---

## 16. ThzNtnIslChannel

**Purpose:** Inter-satellite link channel in vacuum (FSPL only + hardware impairments).

### Key API
```cpp
double ComputeIslSnr_dB(Ptr<MobilityModel> satA, Ptr<MobilityModel> satB) const;
double ComputeIslCapacity_Gbps(double snrDb) const;
double ComputeRelativeDoppler_Hz(Ptr<MobilityModel> satA, Ptr<MobilityModel> satB) const;
double ComputeSpaceNoiseTemperature_K() const;  // cosmic + receiver
IslLinkState ComputeLinkState(Ptr<MobilityModel> satA, Ptr<MobilityModel> satB) const;
bool IsLinkFeasible(Ptr<MobilityModel>, Ptr<MobilityModel>, double minSnr) const;
double ComputeMaxLinkDistance_km(double minSnr) const;

void SetFreeSpaceLossModel(Ptr<SatFreeSpaceLoss>);  // integration point
void SetHardwareModel(Ptr<ThzNtnHardwareImpairments>);
```

### Attributes
| Attribute | Default |
|---|---|
| `Frequency` | 300 GHz |
| `TxPower` | 30 dBm |
| `TxGain` / `RxGain` | 40 / 40 dBi |
| `Bandwidth` | 20 GHz |
| `ReceiverNoiseTemp_K` | 500 |

---

## 17. ThzNtnIslLink

**Purpose:** ISL management, routing, and adaptive rate.

### Key API
```cpp
bool EstablishLink(uint32_t srcSatId, uint32_t dstSatId, double distKm);
void TeardownLink(uint32_t linkId);
IslLinkStatus GetLinkStatus(uint32_t linkId) const;
double GetCurrentDataRate_Gbps(uint32_t linkId) const;
void AdaptDataRate(uint32_t linkId, double snrDb);  // 1.5 - 17.3 Gbps rate table

IslRoute ComputeRoute(uint32_t src, uint32_t dst, RoutingAlgorithm algo) const;  // Dijkstra
double ComputeEndToEndDelay_ms(const IslRoute&) const;
void UpdateTopology(const std::map<uint32_t, Ptr<MobilityModel>>& satMobilities);
void SetIslChannel(Ptr<ThzNtnIslChannel>);
void SetIslNetDevice(uint32_t linkId, Ptr<SatPointToPointIslNetDevice>);
```

### Routing algorithms
`SHORTEST_PATH` (unit weights), `MINIMUM_DELAY` (propagation delay), `LOAD_BALANCED` (inverse capacity).

---

## 18. ThzNtnRis / ThzNtnRisController

**Purpose:** Reconfigurable Intelligent Surface with N^2 gain and controller for phase optimization.

### ThzNtnRis API
```cpp
void Configure(uint32_t nx, uint32_t ny, double freqHz, RisDeployment deploy);
double ComputeRisGain_dB(double thIn, double phiIn, double thOut, double phiOut) const;
double ComputeRisGainForLink(Ptr<MobilityModel> tx, Ptr<MobilityModel> ris, Ptr<MobilityModel> rx) const;
double ComputeMaxGain_dB() const;
void ComputeOptimalPhases(Ptr<MobilityModel> tx, Ptr<MobilityModel> ris, Ptr<MobilityModel> rx);
double ComputeQuantizationLoss_dB() const;
double ComputeCascadedPathLoss_dB(Ptr<MobilityModel>, Ptr<MobilityModel>, Ptr<MobilityModel>) const;
double ComputeSnrGain_dB(bool perfectCsi) const;
bool IsInNearField(double d) const;
double ComputeAerialRisCoverage_deg() const;  // 360 aerial, 180 ground
double GetWavelength() const;
std::pair<uint32_t,uint32_t> GetPanelDimensions() const;
```

### Deployments
`SPACE_BORNE`, `AERIAL`, `GROUND`.

### Phase models
`CONTINUOUS`, `DISCRETE_1BIT`, `DISCRETE_2BIT`, `DISCRETE_3BIT`, `DISCRETE_4BIT`.

### ThzNtnRisController API
```cpp
void GenerateCodebook(uint32_t numEntries);
void SelectCodebookEntry(Ptr<MobilityModel> tx, Ptr<MobilityModel> ris, Ptr<MobilityModel> rx);
void OptimizePhases(Ptr<MobilityModel>, Ptr<MobilityModel>, Ptr<MobilityModel>, OptimizationMethod);
double EstimateChannel(const std::vector<double>& pilots);
void CoordinateMultiPanel(const std::vector<Ptr<ThzNtnRis>>& panels);

typedef Callback<std::vector<double>, double, double, double, double> RisPhasePredictionCallback;
void SetDrlCallback(RisPhasePredictionCallback);
```

---

## 19. ThzNtnIsac / ThzNtnIsacProcessor

**Purpose:** Joint radar-communication at THz for cm-resolution sensing.

### ThzNtnIsac key API
```cpp
SensingResult PerformSensing(double freqHz, double txPowerDbm, double txGain, double rxGain,
                              double bw, double rangeM, double rcsM2, double intTime) const;

double ComputeRadarSnr_dB(double freqHz, double txPowerDbm, double totalGain,
                           double range, double rcs, double bw) const;
double ComputeRangeResolution_m(double bw) const;                    // c/(2*BW)
double ComputeVelocityResolution_m_s(double freqHz, double T) const; // lambda/(2*T)
double ComputeAngularResolution_deg(double freqHz, double aperture) const;

double ComputeCramerRaoBound_range(double snrLin, double bw) const;
double ComputeCramerRaoBound_velocity(double snrLin, double freq, double T) const;
double ComputeDetectionProbability(double snrDb, double pfa) const;   // Swerling-I
double ComputeMaxDetectionRange_km(double rcsM2, double minSnr) const;

IsacPerformance EvaluatePerformance(double rangeM) const;
DebrisModel GetDebrisModel(const std::string& sizeCategory) const;

void SetPhy(Ptr<ThzNtnPhy>);         // pull TX/BW/freq from actual PHY
void SetAntennaArray(Ptr<ThzNtnAntennaArray>);  // pull gain
```

### Debris RCS table (300 GHz)
| Size | RCS (dBsm) |
|---|---|
| small_1cm | -40 |
| medium_10cm | -20 |
| large_1m | 0 |

### Modes
`COMMUNICATION_ONLY`, `SENSING_ONLY`, `JOINT_ISAC`, `COMMUNICATION_CENTRIC`, `SENSING_CENTRIC`.

### ThzNtnIsacProcessor API
```cpp
std::vector<std::vector<double>> ComputeRangeDopplerMap(double range, double velocity, double snr) const;
CfarResult PerformCfar(const std::vector<std::vector<double>>& rdMap) const;
std::pair<double,double> ComputeJointBeamformingGain(double commA, double senseA, uint32_t N) const;
double ComputeBeamPatternMismatch_dB(double commA, double senseA) const;
void UpdateTargetTrack(uint32_t id, double range, double velocity, double t);
std::pair<double,double> PredictTargetPosition(uint32_t id, double futureT) const;
```

---

## 20. ThzNtnHelper

**Purpose:** Fluent factory that wires the full stack together.

### Key API
```cpp
Ptr<ThzNtnChannelModel> CreateChannelModel(const std::string& preset) const;
Ptr<ThzNtnPhySat> CreateSatellitePhy(const std::string& preset, Ptr<SatPhy> satPhy = nullptr) const;
Ptr<ThzNtnPhyGround> CreateGroundPhy(const std::string& preset, Ptr<SatPhy> utPhy = nullptr) const;
Ptr<ThzNtnAntennaArray> CreateAntennaArray(const std::string& preset) const;
Ptr<ThzNtnBeamforming> CreateBeamforming(uint32_t numBeams) const;
Ptr<ThzNtnBeamTracking> CreateBeamTracking() const;
Ptr<ThzNtnMac> CreateMac(const std::string& preset, Ptr<ThzNtnMolecularAbsorption> abs = nullptr) const;
Ptr<ThzNtnIslChannel> CreateIslChannel(const std::string& preset) const;
Ptr<ThzNtnRis> CreateRis(uint32_t nx, uint32_t ny, const std::string& deploy) const;
Ptr<ThzNtnIsac> CreateIsac(const std::string& mode) const;
Ptr<ThzNtnLinkBudget> CreateLinkBudget() const;

void InstallOnSatellite(Ptr<Node> sat, const std::string& preset);
void InstallOnGroundTerminal(Ptr<Node> gt, const std::string& preset);
void PrintConfiguration() const;
```

### Presets
`"TeraLink-225GHz"`, `"D-band-140GHz"`, `"ISL-300GHz"`, `"FullStack"`.

---

## O-RAN Integration (extensions to `contrib/oran-ntn`)

### New xApps
- **`OranNtnXappThzBeamMgmt`** - triggers beam search on pointing error
- **`OranNtnXappThzSpectrum`** - selects atmospheric windows based on weather
- **`OranNtnXappThzRis`** - reconfigures RIS phase profile
- **`OranNtnXappIsac`** - manages sensing/comm resource split

### Extended E2-KPM fields
`thzCenterFreq_GHz, molecularAbsorption_dB, pointingError_deg, thzSnr_dB, umMimoElements, risGain_dB, hardwareImpairmentLoss_dB, atmosphericWindow_GHz, isacSensingActive, debrisDetectionRange_km, beamSquintLoss_dB, activeWaveform`

### New E2-RC actions
`ACTION_THZ_FREQ_SELECT, ACTION_THZ_BEAM_CODEBOOK, ACTION_THZ_RIS_CONFIG, ACTION_THZ_WAVEFORM_SELECT, ACTION_THZ_ISAC_MODE, ACTION_THZ_POWER_BACKOFF, ACTION_THZ_WINDOW_HOP`

---

## NTN-CHO Integration (extensions to `contrib/ntn-cho`)

### New trigger type in `NtnChoAlgorithm`
`TRIGGER_THZ_BEAM_QUALITY` -- fires when THz pointing error exceeds threshold.

### New attributes
`ThzBeamTrackingThreshold`, `ThzSnrThreshold`, `EnableMultiBandCho`, `ThzBeamwidth`.

### New methods in `NtnChoAlgorithm`
```cpp
bool EvaluateThzBeamQuality(uint32_t candidateIdx) const;
double ComputeThzBeamTte(uint32_t candidateIdx) const;
void PrepareMultiBandCandidates();  // Ka-band fallback + THz primary
```

### New methods in `NtnTteEstimator`
```cpp
double ComputeThzBeamTte(double satAlt_km, double satVelocity_km_s,
                         double beamwidth_deg, double pointingError_deg,
                         double elevationDeg) const;
double ComputeThzEffectiveCoverage_km(double satAlt_km, double beamwidth_deg,
                                       double pointingError_deg) const;
```

---

## Newer classes (v2)

Added after the sections above were written; see the `model/*.h` headers for
the full API.

| Class | Header | Role |
|---|---|---|
| `ThzNtnAlphaMuFading` | `thz-ntn-alpha-mu-fading.h` | alpha-mu small-scale fading (Rayleigh / Nakagami special cases) |
| `HitranLut` (namespace `ns3::thzntn`) | `thz-ntn-hitran-lut.h` | Bundled HITRAN-2024 specific-attenuation lookup table (`data/hitran2024-lut-subthz.csv`) |
| `Itu838RainModel`, `Itu618LossModel`, `Itu676AbsorptionModel`, `Itu681LmsModel` | `thz-ntn-itu-recommendations.h` | ITU-R P.838 / P.618 / P.676 / P.681 reference implementations |
| `ThzNtnNyusimReference`, `ThzNtnNyusimCalibrator` | `thz-ntn-nyusim-reference.h`, `thz-ntn-nyusim-calibrator.h` | NYUSIM-140 GHz calibration reference (`data/nyusim-140-reference.csv`) + calibrator |
| `ThzNtnIsacScheduler` | `thz-ntn-isac-scheduler.h` | Comm/sense sub-band partitioning per ISAC mode |
| `ThzNtnRisServiceModel`, `ThzNtnRisXapp` | `thz-ntn-ris-service-model.h`, `thz-ntn-ris-xapp.h` | O-RAN service model + xApp for closed-loop RIS control |
| `ThzNtnPropagationLossModel` | `thz-ntn-propagation-loss-model.h` | Atmospheric excess loss (gaseous absorption + rain/fog/snow) as a real `PropagationLossModel`, chainable onto a live spectrum channel |
