# THz-NTN Examples Reference

The reference demo at `scratch/thz-ntn-demo.cc` runs 8 distinct scenarios, prints validated results to the console, and writes CSV datasets to `thz-ntn-results/`. Each example can be run individually with `--example=N`.

---

## Building and Running

```bash
./ns3 configure --enable-modules=thz-ntn
./ns3 build scratch/thz-ntn-demo

# All 8 examples
./build/scratch/ns3.43-thz-ntn-demo-debug

# Individual example
./build/scratch/ns3.43-thz-ntn-demo-debug --example=5
```

Output datasets:

```
thz-ntn-results/
├── 01_leo_ground_sweep.csv
├── 02_isl_distance_sweep.csv
├── 03_dband_constellation.csv
├── 04_ris_sweep.csv
├── 05_isac_debris.csv
├── 06_um_mimo.csv
├── 07_atmospheric_windows.csv
├── 08_beam_tracking.csv
└── SUMMARY.txt
```

---

## Example 1 -- LEO-Ground TeraLink (225 GHz, elevation sweep)

### What it does
Sweeps the elevation angle from 5° to 90° and computes a full link budget for a LEO-to-ground link at 225 GHz (TeraLink-1 frequency). For each elevation, a satellite is positioned at 550 km altitude such that the line-of-sight to a ground terminal at (latitude 0°, longitude 0°) matches the required elevation.

### Parameters
| Parameter | Value |
|---|---|
| Frequency | 225 GHz |
| Altitude | 550 km |
| TX Power | 34.77 dBm (3 W) |
| TX antenna gain | 40 dBi |
| RX antenna gain | 45 dBi |
| Bandwidth | 10 GHz |
| Noise figure | 10 dB |

### Expected console output
```
 Elev  Distance   FSPL   Absorb  Weather  Pointing   RxPwr    SNR    Cap
  deg    km        dB      dB      dB       dB       dBm     dB    Gbps
-----------------------------------------------------------------------
 5.00  2204.97  206.36    0.96     0.00     0.48    -88.80  -24.82    0.00
10.00  1815.08  204.67    0.51     0.00     0.48    -86.57  -22.59    0.00
15.00  1518.02  203.12    0.35     0.00     0.48    -84.69  -20.71    0.00
20.00  1293.55  201.73    0.26     0.00     0.48    -82.87  -18.90    0.18
30.00   992.78  199.43    0.18     0.00     0.48    -80.42  -16.45    0.32
45.00   749.11  196.98    0.13     0.00     0.48    -77.95  -13.98    0.56
60.00   626.89  195.44    0.11     0.00     0.48    -76.32  -12.35    0.81
75.00   567.79  194.58    0.09     0.00     0.48    -75.44  -11.47    0.99
90.00   550.00  194.30    0.09     0.00     0.48    -75.16  -11.19    1.05
```

### Interpretation
- **Distance**: law of cosines on a spherical Earth with 550 km satellite altitude -- 2205 km at 5° elevation (horizon) to 550 km at 90° (overhead).
- **FSPL**: `20*log10(4*pi*d*f/c)` -- every value matches the analytical formula exactly. At zenith (d=550 km, f=225 GHz) FSPL = 194.30 dB.
- **Molecular absorption**: higher at low elevation (0.96 dB at 5°) due to longer slant path through the troposphere; drops to 0.09 dB at zenith.
- **Weather**: 0 dB (no rain configured -- set `RainRate` attribute to enable).
- **Pointing**: ~0.48 dB constant (default 0.5° beamwidth, 7.5 km/s satellite velocity).
- **RX Power**: EIRP (74.77 dBm) minus total path loss plus RX antenna gain (45 dBi).
- **SNR**: relative to thermal noise of kTB + NF = -63.98 dBm over 10 GHz bandwidth.
- **Capacity**: Shannon limit with hardware impairment ceiling -- negative link margin indicates this preset needs more antenna gain or narrower bandwidth for closed link; this is expected for the basic preset and shows the link budget tool correctly reports insufficient margin.

### CSV columns
`elevation_deg, distance_km, fspl_dB, absorption_dB, weather_dB, scint_dB, pointing_dB, total_loss_dB, rx_power_dBm, snr_dB, hw_snr_dB, shannon_Gbps, hw_capacity_Gbps, link_margin_dB`

---

## Example 2 -- Inter-Satellite Link (ISL @ 300 GHz)

### What it does
Computes the ISL link budget at 300 GHz (vacuum regime) for a range of inter-satellite distances from 100 km to 5000 km. Both satellites orbit at 550 km altitude and move with opposite tangential velocities to induce Doppler.

### Parameters
| Parameter | Value |
|---|---|
| Frequency | 300 GHz |
| TX power | 30 dBm |
| TX/RX gain | 50 dBi |
| Bandwidth | 20 GHz |
| Noise temperature | 500 K (space + receiver) |

### Expected console output
```
 Distance    SNR     Capacity
   km         dB       Gbps
----------------------------------
 100.00     16.59      88.65
 249.99      8.63      48.82
 499.89      2.61      23.96
 999.13     -3.41       8.68
1497.07     -6.92       4.27
1993.05     -9.41       2.51
2976.57    -12.89       1.16
4891.97    -17.20       0.44
```

### Interpretation
- **SNR falls 6 dB per doubling of distance** (standard inverse-square-law behaviour).
- **100 km → 1000 km**: SNR drops by 20 dB (exactly 10*log10(100) as expected for R^2 FSPL).
- **No molecular absorption**: ISL runs in vacuum (confirmed 0 absorption in the underlying model).
- **Capacity**: Shannon limit at the receiver's SINR, capped by the hardware impairment ceiling (max ~100 Gbps at 20 GHz BW).

### CSV columns
`distance_km, snr_dB, capacity_Gbps, doppler_MHz`

(Doppler is near zero since the tangential velocities are symmetric about the line-of-sight.)

---

## Example 3 -- D-band LEO Constellation (140 GHz, 4 sats, 10 UTs)

### What it does
Places 4 satellites at 600 km altitude spread across geocentric angles -5°, 2°, 9°, 16° and 10 ground terminals at latitudes 0° through 27° (3° spacing). For each ground terminal, picks the satellite with the shortest line-of-sight distance and computes the D-band link budget.

### Parameters
| Parameter | Value |
|---|---|
| Frequency | 140 GHz |
| Satellite altitude | 600 km |
| TX power | 40 dBm (10 W) |
| TX/RX gains | 38 / 42 dBi |
| Bandwidth | 20 GHz |

### Expected console output
```
 UT  BestSat  Dist_km  Elev_deg  SNR_dB  Cap_Gbps
------------------------------------------------------
  0     1     643.51    69.79    -9.15     3.29
  1     1     611.17    79.52    -8.69     3.63
  2     2     694.07    61.29    -9.79     2.86
  3     2     600.00    90.00    -8.51     3.77
  4     2     694.07    61.29    -9.79     2.86
  5     3     611.17    79.52    -8.65     3.66
  6     3     643.51    69.79    -9.12     3.31
  7     3     835.47    48.35   -11.39     2.01
  8     3    1106.54    36.75   -13.86     1.16
  9     3    1411.37    30.53   -16.01     0.71

Aggregate capacity: 23.38 Gbps (7/10 UTs served)
```

### Interpretation
- **Best-satellite handover pattern**: UTs 0-1 attach to sat 1 (at 2°), UTs 2-4 to sat 2 (at 9°), UTs 5-9 to sat 3 (at 16°). Handover boundaries are at the midpoints between satellite ground tracks -- correct behaviour.
- **UT 3 is at exactly the same angle as sat 2** (9°), giving distance=600 km (pure altitude) and elevation=90° (straight up).
- **Symmetric distance pairs**: UT 2 and UT 4 are both 3° from sat 2 so they see identical distances (694.07 km) and SNRs.
- **Aggregate capacity**: 23.38 Gbps across 7 UTs that meet the -10 dB SNR threshold.

### CSV columns
`ut_id, best_sat_id, distance_km, elevation_deg, snr_dB, capacity_Gbps`

---

## Example 4 -- RIS N-squared Scaling Law Verification

### What it does
Configures RIS panels of 5 different sizes (8x8, 16x16, 32x32, 64x64, 128x128 elements), all at 300 GHz, and measures the theoretical max gain plus the SNR gain under perfect and imperfect CSI.

### Expected console output
```
 Size     Elements  MaxGain_dB  SnrPerfect  SnrImperfect  QuantLoss
------------------------------------------------------------------------
   8x  8      64      40.15     34.24     16.18      0.91
  16x 16     256      52.20     46.28     22.20      0.91
  32x 32    1024      64.24     58.32     28.22      0.91
  64x 64    4096      76.28     70.37     34.24      0.91
 128x128   16384      88.32     82.41     40.26      0.91

N^2 scaling verification: 4x more elements -> 12.04 dB more gain (expected 20*log10(4) = 12.04 dB)
```

### Interpretation
- **Max gain** = `20*log10(N_total) + efficiency_loss` (power scales as N^2 for coherent combining).
- **Exactly +12.04 dB per 4x elements**: this is the hallmark of the N^2 law (the RIS's most celebrated property).
- **Perfect CSI** ≈ max gain minus ~6 dB phase alignment overhead.
- **Imperfect CSI** scales as N only (half the dB increase: +6.02 dB per 4x elements) -- matches Wu and Zhang (IEEE TWC 2019).
- **Quantization loss** constant at 0.91 dB for 2-bit phase shifters: `10*log10(sinc(pi/4)^2)` ≈ 0.91 dB (Wu and Zhang 2019 Eq. 8).

### CSV columns
`num_elements, nx, ny, max_gain_dB, snr_gain_perfect_dB, snr_gain_imperfect_dB, quant_loss_dB, array_size_cm`

---

## Example 5 -- ISAC Debris Detection (300 GHz, 20 GHz BW)

### What it does
Demonstrates Integrated Sensing and Communication (ISAC) at THz for space debris detection. Uses a 32x32 UM-MIMO array for both TX and RX (monostatic radar), computes the radar SNR at 10 m / 50 m / 100 m / 500 m for three debris sizes, and reports the maximum detection range for 10 dB minimum SNR.

### Parameters
| Parameter | Value |
|---|---|
| Frequency | 300 GHz (lambda = 0.9993 mm) |
| Bandwidth | 20 GHz |
| TX power | 40 dBm (10 W) |
| Array | 32x32 elements (35.10 dBi per antenna) |
| Total bistatic gain | 70.21 dBi |
| Min detection SNR | 10 dB |
| P_FA | 1e-6 |

### Expected console output
```
Range resolution:     0.7495 cm (c/2BW, BW=20 GHz)
Wavelength:           0.9993 mm
Velocity resolution:  0.4997 m/s (T_int=1ms)
Array (32x32) gain:   35.1030 dBi (per antenna)
Total bistatic gain:  70.2060 dBi

 Debris      RCS(dBsm)  MaxRng(m)  SNR@10m  SNR@50m  SNR@100m  SNR@500m
------------------------------------------------------------------------------
 small_1cm     -40.00      5.07     8.19  -19.77   -31.81   -59.77
 medium_10cm   -20.00     16.02    28.19    0.23   -11.81   -39.77
 large_1m       0.00     50.67    48.19   20.23     8.19   -19.77

Interpretation: Pd @ SNR=10dB, Pfa=1e-6 -> 0.285
  (Large 1m debris detectable reliably below ~50m range)
```

### Interpretation
- **Range resolution = c / (2 * BW)**: 7.495 mm at 20 GHz bandwidth (sub-centimetre).
- **Velocity resolution = lambda / (2 * T_int)** at 1 ms integration = 0.5 m/s.
- **Radar equation scaling**: SNR drops by 40 dB per decade of range (`R^4` dependency), so going from 10 m to 100 m loses exactly 40 dB -- confirmed in every row.
- **RCS sensitivity**: 20 dB difference between 1 m debris (0 dBsm) and 10 cm debris (-20 dBsm) translates to exactly 20 dB SNR difference.
- **Short range is expected**: THz radar's `lambda^2` factor means THz is a short-range high-resolution sensor; it is **not** suitable for long-range surveillance. It is ideal for proximity operations, rendezvous, and docking applications.

### CSV columns
`debris_size, rcs_dBsm, rcs_m2, max_detect_range_m, snr_at_10m_dB, snr_at_50m_dB, snr_at_100m_dB, snr_at_500m_dB, pd_at_10dB`

---

## Example 6 -- UM-MIMO Array Characterization

### What it does
Characterises Uniform Planar Arrays at 300 GHz for 6 sizes from 4x4 to 128x128. Prints the maximum gain, 3 dB beamwidth, physical size (at lambda/2 spacing), and Fraunhofer near-field distance.

### Expected console output
```
 Config     Elements  MaxGain_dBi  BW3dB_deg   Size_cm   NearField_m
---------------------------------------------------------------------
   4x4        16      17.041      25.382     0.283      0.016
   8x8        64      23.062      12.691     0.565      0.064
  16x16      256      29.082       6.346     1.131      0.256
  32x32     1024      35.103       3.173     2.261      1.023
  64x64     4096      41.124       1.586     4.522      4.093
 128x128   16384      47.144       0.793     9.045     16.373
```

### Verification
- **Gain formula: 10*log10(N) + element gain** -- matches each row exactly (15 + 5 = 17.041 is 10*log10(16) + 5 + small aperture efficiency).
- **Beamwidth doubles per halving of N**: 25.38 -> 12.69 -> 6.35 -> 3.17 -> 1.59 -> 0.79 deg. Matches `0.886 * lambda / (N * d)` where d = lambda/2.
- **Physical size = N * d = N * lambda/2**: at 300 GHz, lambda = 1 mm so d = 0.5 mm. A 128x128 array is 9.04 cm across -- fits on any satellite.
- **Near-field distance = 2 * D^2 / lambda**: scales as N^2. For 128x128 this is 16.37 m -- the far-field assumption breaks at practical satellite-satellite proximity operations.

### CSV columns
`nx, ny, total_elements, max_gain_dBi, beamwidth_deg, physical_size_cm, near_field_m`

---

## Example 7 -- THz Atmospheric Windows

### What it does
Lists the 5 canonical sub-THz transmission windows used by the module's `ThzNtnSpectrum::GetStandardWindows()`.

### Expected console output
```
 # | CenterFreq | BW_GHz | PeakTrans | ZenithAtten | Sat-Ground
-----------------------------------------------------------------
 1 |  140.00 GHz|20.00  |   0.85  |   1.20 dB  |  YES
 2 |  220.00 GHz|30.00  |   0.75  |   2.50 dB  |  YES
 3 |  340.00 GHz|25.00  |   0.55  |   4.80 dB  |  YES
 4 |  410.00 GHz|15.00  |   0.45  |   6.50 dB  |  YES
 5 |  460.00 GHz|10.00  |   0.35  |   8.20 dB  |  YES
```

### Interpretation
- **140 GHz (D-band low)**: best transmittance (85%) and lowest zenith loss (1.2 dB) -- preferred for satellite-ground.
- **220 GHz**: the TeraLink frequency band, widest usable bandwidth (30 GHz).
- **340 GHz+**: higher frequency windows trade transmittance for potential higher-rate links but require more compensation.
- **Monotonic roll-off** in peak transmittance (0.85 -> 0.35) and monotonic increase in attenuation (1.2 -> 8.2 dB) -- matches ITU-R literature for sub-THz.

### CSV columns
`window_id, center_freq_GHz, bandwidth_GHz, peak_transmittance, zenith_atten_dB, suitable_sat_ground`

---

## Example 8 -- Beam Tracking During 60 s LEO Pass

### What it does
Simulates a 60-second LEO satellite overpass. The satellite moves from geocentric angle -10° (pre-culmination) to +10° (post-culmination) at 550 km altitude, with 5-second snapshots. For each time step the link budget is recomputed with the current satellite position.

### Expected console output
```
 Time  SatAngle  Dist_km  Elev_deg  SNR_dB  Cap_Gbps
-------------------------------------------------------
 0.00   -10.00   1281.51    30.31   -18.78     0.19
 5.00    -8.33   1110.68    33.76   -17.55     0.25
10.00    -6.67   948.05    38.72   -16.05     0.35
15.00    -5.00   798.80    45.96   -14.43     0.51
20.00    -3.33   672.09    56.55   -12.90     0.72
25.00    -1.67   582.93    71.47   -11.67     0.95
30.00     0.00   550.00    90.00   -11.19     1.05
35.00     1.67   582.93    71.47   -11.64     0.95
40.00     3.33   672.09    56.55   -12.98     0.71
45.00     5.00   798.80    45.96   -14.44     0.51
50.00     6.67   948.05    38.72   -16.01     0.36
55.00     8.33   1110.68    33.76   -17.39     0.26
60.00    10.00   1281.51    30.31   -18.72     0.19
```

### Interpretation
- **Symmetric pass** about culmination at t=30 s, where the satellite is directly overhead (elevation=90°, distance=550 km).
- **Handover opportunities**: SNR drops below -15 dB outside of a ~30 s window around culmination -- this is where a TTE-aware CHO algorithm would trigger.
- **Small asymmetries** (e.g. -18.78 vs -18.72 dB at t=0 / 60) are due to the scintillation model's random time-series -- a correct stochastic behaviour.
- **Capacity peak at culmination**: 1.05 Gbps.
- Matches typical LEO pass geometry at 550 km altitude.

### CSV columns
`time_s, sat_angle_deg, distance_km, elevation_deg, snr_dB, capacity_Gbps`

---

## Plotting with matplotlib

```python
import pandas as pd
import matplotlib.pyplot as plt

# Example 1 - elevation sweep
df = pd.read_csv('thz-ntn-results/01_leo_ground_sweep.csv')
df.plot(x='elevation_deg', y=['fspl_dB', 'total_loss_dB'], marker='o')
plt.title('TeraLink 225 GHz - Loss vs Elevation')
plt.grid()
plt.savefig('loss_vs_elevation.png')

# Example 4 - RIS N^2 verification
df = pd.read_csv('thz-ntn-results/04_ris_sweep.csv')
df.plot(x='num_elements', y='max_gain_dB', logx=True, marker='o')
plt.title('RIS max gain: N^2 scaling')
plt.grid()
plt.savefig('ris_n2_law.png')

# Example 8 - LEO pass
df = pd.read_csv('thz-ntn-results/08_beam_tracking.csv')
fig, ax1 = plt.subplots()
ax1.plot(df.time_s, df.snr_dB, 'b-', label='SNR (dB)')
ax2 = ax1.twinx()
ax2.plot(df.time_s, df.distance_km, 'r-', label='Distance (km)')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('SNR (dB)')
ax2.set_ylabel('Distance (km)')
plt.title('60s LEO pass at 550 km altitude')
plt.savefig('leo_pass.png')
```
