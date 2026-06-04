<h1 align="center">ntn-rrc</h1>

<p align="center"><strong>3GPP Rel-17 NR-NTN RRC procedures for ns-3.43: SIB19 ephemeris broadcast, timing advance, pass-aware DRX, and UE location reporting.</strong></p>

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0--only-green.svg"/></a>
  <img src="https://img.shields.io/badge/3GPP-TS%2038.213%20%2F%20TS%2038.331%20%2F%20TS%2038.321-orange.svg"/>
  <img src="https://img.shields.io/badge/unit_tests-suite%20ntn--rrc-success.svg"/>
</p>

> Part of **ns3-ntn-toolkit** — see the [toolkit repository](https://github.com/Muhammaduazir69/ns3-ntn-toolkit) for the full build, dependency, and module map, and [INSTALL.md](INSTALL.md) for this module's setup.

---

## Overview

LEO non-terrestrial networks introduce one-way delays and Doppler trajectories that the terrestrial 5G RRC stack was never specified for: a single-leg propagation can exceed 17 ms at low elevation, common timing-advance values drift by tens of microseconds-per-second, and the assistance information broadcast in SIB1 says nothing about satellite ephemerides. The 3GPP NTN work item (Release-17 onward) closes these gaps with new IEs, new SIB types, and new MAC behaviours — but most ns-3 distributions still carry only the terrestrial RRC.

`ntn-rrc` adds the NTN-specific RRC procedures as a clean, optional contrib module:

- **SIB19 ephemeris broadcast** — periodic broadcaster (default 160 ms) snapshots fresh satellite ephemeris (ECEF state vector), the common timing advance, and its drift rate into a fixed-layout 124-byte codec frame (TS 38.331 §6.3.2).
- **Timing advance** — ephemeris-driven TA pre-compensation decomposed into a SIB19-broadcast **common** term plus a per-UE **UE-specific** residual; `2·d/c` for transparent payload, `d/c` for regenerative (TS 38.213 §4.2.2, TR 38.821 §6.3.3).
- **Pass-aware DRX** — NR connected-mode DRX state machine (`Active / OnDuration / ShortSleep / LongSleep`) extended with an NTN `AwaitingPass` deep-sleep state between visibility windows (TS 38.321 + TR 38.821 §6.3.4).
- **UE location report** — GNSS-assisted reporting in periodic / event-triggered / on-demand modes, with closed-form ECEF↔WGS-84 conversion (TS 38.331 §5.7.4).

## What's new in v2

See [CHANGELOG.md](CHANGELOG.md) for this module's history.

- **`ntn-rrc-full-stack` now flies an orbital `SatSGP4MobilityModel`** (driven from the bundled ISS TLE) instead of the old straight-line constant-velocity model that climbed out of the orbital shell over the pass. The UE now sits at ground level (Islamabad, lat 33.6844°, lon 73.0479°), and the beam-centre reference position is offset ~50 km north of the UE so the UE-specific TA residual (`ta_ue`) is genuinely non-zero rather than collapsing to 0.
- **`ta_drift_rate` is now emitted in µs/s** — all CSV columns are renamed `*_us_per_s`. The underlying timing-advance model (`ComputeTaDriftRate()`) and the SIB19 codec ABI keep their native s/s convention; only the CSV presentation is scaled.
- **`ntn-rrc-from-tle` ships a default ISS TLE** (`data/iss-zarya.tle`) and runs with zero arguments — it auto-discovers the bundled TLE from the build root, `contrib/`, or install layout and defaults the scenario start to the TLE epoch.

## Models, helpers & key classes

| Class / API | Header | Role |
|---|---|---|
| `NtnTimingAdvance` | `model/ntn-timing-advance.h` | `ComputeTotalTa()`, `ComputeCommonTa()`, `ComputeUeSpecificTa()`, `ComputeTaDriftRate()` (s/s), `GetSlantRangeMetres()`. Consumes a UE + satellite `MobilityModel` pair and a reference (beam-centre) position. |
| `EphemerisInfo`, `Sib19Content` | `model/ntn-sib19.h` | NTN-Config-r17 assistance data: ECEF ephemeris state vector, `taCommon`, drift rate/variation, UL-sync validity, K-offsets, payload mode, cell id. |
| `Sib19Codec` | `model/ntn-sib19.h` | Fixed-layout little-endian serialise/parse; `kSerialisedBytes = 124`; rejects under-size buffers. |
| `NtnSib19Broadcaster` | `model/ntn-sib19.h` | Periodic broadcaster (default 160 ms); re-derives content from mobility + TA each tick; `RefreshNow()`, `GetLatest()`, `Broadcast` trace. |
| `NtnDrxStateMachine`, `NtnDrxConfig`, `DrxState` | `model/ntn-drx.h` | NR DRX cycles plus NTN `AwaitingPass`; `NotifyDataActivity()`, `NotifyNextPass()`, `GetTimeInState()`, `StateChange` trace; `NtnDrxConfig::IsValid()` rejects malformed cycles. |
| `NtnUeLocationReporter`, `UeLocationReport`, `LocationReportMode` | `model/ntn-ue-location-report.h` | Periodic / event-triggered / on-demand GNSS reporting; `EcefToGeodeticWgs84()` / `GeodeticWgs84ToEcef()` (Heikkinen closed-form); `Report` trace; `ReportNow()`. |
| `PayloadMode`, `TaReferenceFrame` | `model/ntn-rrc-types.h` | Transparent vs regenerative payload; TA reference frame. |
| `NtnRrcHelper` | `helper/ntn-rrc-helper.h` | Façade: `SetPayloadMode()`, `SetReferencePosition()`, `InstallTimingAdvance()`, `InstallSib19Broadcaster()`, `InstallUeLocationReporter()`, `InstallDrx()`. |

## Examples

All four examples are built when `--enable-examples` is configured. Each block gives the `./ns3 run` form and the direct-binary form (binaries live under `build/contrib/ntn-rrc/examples/` as `ns3.43-<name>-default`).

### ntn-rrc-leo-pass

Single-component demo: ephemeris-driven Timing Advance over a 600 s LEO pass (straight-line fly-over geometry), sampled once per second. Produces the classic NTN "smile" curve. A realistic UDP traffic plane is auto-injected so a `sim_health.csv` is also written.

```bash
./ns3 run "ntn-rrc-leo-pass --simTime=600 --transparent=true --csv=ntn-rrc-leo-pass.csv --outputDir=."
```
```bash
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-rrc/examples/ns3.43-ntn-rrc-leo-pass-default \
    --simTime=600 --transparent=true --csv=ntn-rrc-leo-pass.csv --outputDir=.
```

**Outputs:** `ntn-rrc-leo-pass.csv` (columns `time_s,ta_total_us,ta_common_us,ta_ue_us,ta_drift_rate_us_per_s`); `sim_health.csv` in `--outputDir`.
**Key args:** `--simTime` (s), `--transparent` (true=transparent / false=regenerative), `--csv` (output path), `--outputDir` (sim_health.csv directory).

### ntn-rrc-full-stack

End-to-end pass exercising all four NTN-RRC components at once — TA pre-comp, SIB19 broadcast, UE GNSS reporting, and the DRX state machine — over a real SGP4 orbit (bundled ISS TLE) with the UE on the ground in Islamabad. A realistic UDP traffic plane is auto-injected.

```bash
./ns3 run "ntn-rrc-full-stack --simTime=600 --transparent=true --passAwareDrx=true --prefix=ntn-rrc-full --outputDir=."
```
```bash
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-rrc/examples/ns3.43-ntn-rrc-full-stack-default \
    --simTime=600 --transparent=true --passAwareDrx=true --prefix=ntn-rrc-full --outputDir=.
```

**Outputs:** `<prefix>-ta.csv` (`time_s,ta_total_us,ta_common_us,ta_ue_us,ta_drift_rate_us_per_s`), `<prefix>-sib19.csv` (`time_s,broadcast_seq,cell_id,sat_x,sat_y,sat_z,ta_common_us,drift_rate_us_per_s`), `<prefix>-ue.csv` (`time_s,sequence,lat_deg,lon_deg,alt_m`), `<prefix>-drx.csv` (`time_s,state,active_ms,onDuration_ms,shortSleep_ms,longSleep_ms,awaitingPass_ms`); `sim_health.csv` in `--outputDir`.
**Key args:** `--simTime` (s), `--transparent` (payload mode), `--passAwareDrx` (enable `AwaitingPass` deep sleep), `--prefix` (CSV filename prefix), `--outputDir` (sim_health.csv directory).

### ntn-rrc-from-tle

Reads a real 3-line TLE, drives a SNS3 `SatSGP4MobilityModel` from it, and logs Timing Advance across a pass. Ships with a bundled ISS TLE (`data/iss-zarya.tle`) and **runs with no arguments** — `--tle` is optional. A realistic UDP traffic plane is auto-injected.

```bash
./ns3 run "ntn-rrc-from-tle"
```
```bash
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-rrc/examples/ns3.43-ntn-rrc-from-tle-default
```

With an explicit TLE and scenario start:

```bash
./ns3 run "ntn-rrc-from-tle --tle=contrib/ntn-rrc/data/iss-zarya.tle --start=2024-01-01T12:00:00 --csv=ntn-rrc-from-tle.csv --outputDir=."
```
```bash
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-rrc/examples/ns3.43-ntn-rrc-from-tle-default \
    --tle=contrib/ntn-rrc/data/iss-zarya.tle --start=2024-01-01T12:00:00 --csv=ntn-rrc-from-tle.csv --outputDir=.
```

**Outputs:** `ntn-rrc-from-tle.csv` (columns `time_s,sat_x_m,sat_y_m,sat_z_m,slant_km,ta_total_us,ta_drift_rate_us_per_s`); `sim_health.csv` in `--outputDir`.
**Key args:** `--tle` (3-line TLE file; defaults to bundled ISS TLE), `--start` (scenario start UTC, `YYYY-MM-DDTHH:MM:SS`; the `T` separator avoids whitespace truncation), `--ueLat` / `--ueLon` / `--ueAlt` (UE geodetic position), `--simTime` (s), `--step` (sample period s), `--transparent` (payload mode), `--csv` (output path), `--outputDir`.

### ntn-rrc-drx-data-traffic

Real UDP downlink to a UE whose receiver is gated by the NTN RRC procedures: SIB19 broadcast + live timing advance + connected-mode DRX. On every DRX `StateChange` the receive link opens (awake) or closes (asleep), so delivered goodput tracks the DRX on-fraction — the power-saving vs throughput trade-off, measured on a real data plane (P2P link, `RateErrorModel`, FlowMonitor). Compare `--drxOn=true` vs `--drxOn=false`.

```bash
./ns3 run "ntn-rrc-drx-data-traffic --simSeconds=60 --dataRateMbps=5 --drxOn=true"
```
```bash
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-rrc/examples/ns3.43-ntn-rrc-drx-data-traffic-default \
    --simSeconds=60 --dataRateMbps=5 --drxOn=true
```

**Outputs:** per-second console trace (elevation, slant range, TA, DRX state, goodput) and a summary line (PDR, average goodput, on-duty %). No CSV.
**Key args:** `--simSeconds` (s), `--leoAltKm`, `--satSpeed` (m/s), `--freqGHz`, `--dataRateMbps` (offered load), `--packetBytes`, `--txPowerDbm`, `--antennaGainDb`, `--drxOn` (enable DRX gating), `--drxLongCycleMs`, `--drxOnDurationMs`, `--linkCapacityMbps`.

## Build, run & test

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build
```

Run the unit-test suite:

```bash
./build/utils/ns3.43-test-runner-default --suite=ntn-rrc
```

The suite (`Type::UNIT`) covers the closed-form TA (transparent and regenerative), the common/UE-specific decomposition, the LEO drift-rate bound, SIB19 codec round-trip and truncation rejection, broadcaster cadence, ECEF↔WGS-84 round-trip, all three location-report modes, the standard DRX cycle and data-activity override, pass-aware deep sleep, and invalid-config rejection.

For full setup notes (SNS3 / satellite-module dependency, traffic helper, toolkit layout) see [INSTALL.md](INSTALL.md).

## License & author

GPL-2.0-only — see [LICENSE](LICENSE).

**Muhammad Uzair**, Independent Researcher.
