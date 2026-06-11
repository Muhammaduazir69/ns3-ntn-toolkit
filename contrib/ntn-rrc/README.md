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

## What's new

See [CHANGELOG.md](CHANGELOG.md) for this module's history.

- **All examples now run on a real mmwave NR NTN cell.** Every example builds the radio through `NtnRealStackHelper` (from the sibling `ntn-traffic` module): SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC, with real UDP traffic over the radio and the DL SINR/TBLER/throughput **measured off the mmwave PHY trace** — no closed-form SINR, no synthetic link models. Each example also writes a `sim_health.csv` with phy-trace provenance.
- **Real mobility everywhere.** Serving satellites fly genuine SGP4 orbits — a Walker-Delta element from `ntn-constellation` (`Sgp4MobilityModel`) in `ntn-rrc-leo-pass`, `ntn-rrc-full-stack`, `ntn-rrc-drx-data-traffic`, and `ntn-rrc-real-stack`, or a real TLE through the SNS3 `SatSGP4MobilityModel` in `ntn-rrc-from-tle`. UEs move under 3GPP TR 38.811 §6.1.1.1 class mobility (`NtnTr38811MobilityModel` from `ntn-cho`), placed under the satellite's t=0 sub-point so a real overhead pass occurs.
- **RRC measurement reports on measured radio** — `ntn-rrc-leo-pass` and `ntn-rrc-real-stack` fire a connection-quality measurement report when the **measured** DL SINR crosses a threshold, alongside the live TA/SIB19 machinery.
- **`ntn-rrc-from-tle` ships a default ISS TLE** (`data/iss-zarya.tle`) and runs with zero arguments — it auto-discovers the bundled TLE relative to the working directory and defaults the scenario start.

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

All five examples are built when `--enable-examples` is configured. Each block gives the `./ns3 run` form and the direct-binary form (binaries live under `build/contrib/ntn-rrc/examples/` as `ns3.43-<name>-default`). Every example runs real UDP traffic over a real mmwave NR NTN cell and writes a `sim_health.csv` to `--outputDir`.

### ntn-rrc-leo-pass

Ephemeris-driven Timing Advance + SIB19 on a real mmwave NR NTN cell over a genuine SGP4 LEO pass, sampled twice per second. Produces the classic NTN "smile" curve from the live slant-range geometry; an RRC measurement report fires whenever the measured DL SINR drops below the threshold.

```bash
./ns3 run "ntn-rrc-leo-pass --simTime=20 --transparent=true --outputDir=ntn-rrc-leo-pass-output"
```
```bash
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-rrc/examples/ns3.43-ntn-rrc-leo-pass-default \
    --simTime=20 --transparent=true --outputDir=ntn-rrc-leo-pass-output
```

**Outputs (in `--outputDir`):** `ntn-rrc-leo-pass-ta.csv` (columns `time_s,slant_km,ta_total_us,ta_common_us,ta_ue_us,ta_drift_us_per_s,measured_sinr_db`) and `sim_health.csv`; summary on stdout (measured mean SINR, measured DL throughput, final slant/TA, SIB19 refreshes, measurement reports).
**Key args:** `--simTime` (s), `--numUes`, `--altitude` (km), `--satEirpDbm`, `--freqGhz`, `--transparent` (true=transparent / false=regenerative), `--outputDir`.

### ntn-rrc-full-stack

End-to-end pass exercising all four NTN-RRC components at once — TA pre-comp, SIB19 broadcast, UE GNSS reporting, and the DRX state machine — over a real SGP4 Walker orbit with TR 38.811 class UEs under the sub-point and real traffic on the mmwave cell. The beam-centre reference is offset ~50 km north of the sub-point so the UE-specific TA residual (`ta_ue`) is genuinely non-zero.

```bash
./ns3 run "ntn-rrc-full-stack --simTime=20 --transparent=true --passAwareDrx=true --prefix=ntn-rrc-full"
```
```bash
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-rrc/examples/ns3.43-ntn-rrc-full-stack-default \
    --simTime=20 --transparent=true --passAwareDrx=true --prefix=ntn-rrc-full
```

**Outputs (in `--outputDir`):** `<prefix>-ta.csv` (`time_s,ta_total_us,ta_common_us,ta_ue_us,measured_sinr_db`), `<prefix>-sib19.csv` (`time_s,broadcast_seq,cell_id,sat_x,sat_y,sat_z,ta_common_us,drift_rate_us_per_s`), `<prefix>-ue.csv` (`time_s,sequence,lat_deg,lon_deg,alt_m`), `<prefix>-drx.csv` (`time_s,state,active_ms,onDuration_ms,shortSleep_ms,longSleep_ms,awaitingPass_ms`); `sim_health.csv`.
**Key args:** `--simTime` (s), `--numUes`, `--altitude` (km), `--satEirpDbm`, `--transparent` (payload mode), `--passAwareDrx` (enable `AwaitingPass` deep sleep), `--prefix` (CSV filename prefix), `--outputDir`.

### ntn-rrc-from-tle

Reads a real 3-line TLE, drives a SNS3 `SatSGP4MobilityModel` from it, and runs the mmwave NR cell under the satellite. The UEs are auto-placed at the satellite's t=0 sub-point so a real overhead pass occurs; Timing Advance is logged from the live SGP4 geometry next to the measured DL SINR. Ships with a bundled ISS TLE (`data/iss-zarya.tle`) and **runs with no arguments** — `--tle` is optional.

```bash
./ns3 run "ntn-rrc-from-tle"
```
```bash
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-rrc/examples/ns3.43-ntn-rrc-from-tle-default
```

With an explicit TLE and scenario start:

```bash
./ns3 run "ntn-rrc-from-tle --tle=contrib/ntn-rrc/data/iss-zarya.tle --start=2024-01-01T12:00:00"
```
```bash
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-rrc/examples/ns3.43-ntn-rrc-from-tle-default \
    --tle=contrib/ntn-rrc/data/iss-zarya.tle --start=2024-01-01T12:00:00
```

**Outputs (in `--outputDir`):** `ntn-rrc-from-tle.csv` (columns `time_s,sat_x_m,sat_y_m,sat_z_m,slant_km,ta_total_us,ta_drift_rate_us_per_s,measured_sinr_db`) and `sim_health.csv`; summary on stdout (sub-point, measured mean SINR, measured throughput, final slant).
**Key args:** `--tle` (3-line TLE file; defaults to the bundled ISS TLE), `--start` (scenario start UTC, `YYYY-MM-DDTHH:MM:SS`; the `T` separator avoids whitespace truncation), `--simTime` (s), `--numUes`, `--satEirpDbm`, `--step` (sample period s), `--transparent` (payload mode), `--outputDir`.

### ntn-rrc-drx-data-traffic

Real UDP downlink to UEs on the mmwave NR cell with the NTN RRC procedures running on the live geometry: SIB19 broadcast, live timing advance, and the connected-mode DRX cycle. The summary reports the measured link KPIs together with the DRX on-duty fraction and the resulting effective goodput (measured goodput × on-duty) — the power-saving vs throughput trade-off. Compare `--drxOn=true` vs `--drxOn=false`.

```bash
./ns3 run "ntn-rrc-drx-data-traffic --simSeconds=20 --numUes=4 --drxOn=true"
```
```bash
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-rrc/examples/ns3.43-ntn-rrc-drx-data-traffic-default \
    --simSeconds=20 --numUes=4 --drxOn=true
```

**Outputs:** per-second console trace (slant range, TA, DRX state, measured SINR), a summary line (measured SINR/TBLER/goodput, DRX on-duty %, effective goodput), and `sim_health.csv` in `--outputDir`.
**Key args:** `--simSeconds` (s), `--numUes`, `--leoAltKm`, `--freqGHz`, `--satEirpDbm`, `--drxOn` (enable DRX gating), `--drxLongCycleMs`, `--drxOnDurationMs`, `--outputDir`.

### ntn-rrc-real-stack

Real-stack flagship: `NtnTimingAdvance` + `NtnSib19Broadcaster` (TS 38.331 NTN-Config) on the real mmwave NR cell, with SIB19 re-broadcast every si-period from the live ephemeris while packets traverse the radio. The TA total/common/UE-specific decomposition and drift come from the real slant-range geometry, and the RRC connection-quality measurement report triggers on the measured DL SINR.

```bash
./ns3 run "ntn-rrc-real-stack --duration=20 --numUes=4"
```
```bash
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-rrc/examples/ns3.43-ntn-rrc-real-stack-default \
    --duration=20 --numUes=4
```

**Outputs (in `--outputDir`):** `ntn-rrc-real-stack-ta.csv` (columns `time_s,slant_km,ta_total_us,ta_common_us,ta_ue_us,ta_drift_us_per_s,measured_sinr_db`) and `sim_health.csv`; summary on stdout (measured mean SINR, measured DL throughput, final slant/TA, SIB19 refreshes, measurement reports).
**Key args:** `--duration` (s), `--numUes`, `--altitude` (km), `--satEirpDbm`, `--freqGhz`, `--transparent` (payload mode), `--outputDir`.

## Build, run & test

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build
```

Run the unit-test suite:

```bash
./test.py -s ntn-rrc
```

The suite (`Type::UNIT`) covers the closed-form TA (transparent and regenerative), the common/UE-specific decomposition, the LEO drift-rate bound, SIB19 codec round-trip and truncation rejection, broadcaster cadence, ECEF↔WGS-84 round-trip, all three location-report modes, the standard DRX cycle and data-activity override, pass-aware deep sleep, and invalid-config rejection.

For full setup notes (SNS3 / satellite-module dependency, the `ntn-traffic` / `ntn-cho` / `ntn-constellation` sibling modules used by the examples, toolkit layout) see [INSTALL.md](INSTALL.md).

## License & author

GPL-2.0-only — see [LICENSE](LICENSE).

**Muhammad Uzair**, Independent Researcher.
