# ntn-fapi

> SCF-222 FAPI L1↔L2 message **model** — C++ structs mirroring the DL_TTI / TX_DATA / RX_DATA / CRC.indication field names for NR-NTN.
> Part of **ns3-ntn-toolkit** — [README](../../README.md) / [INSTALL](../../INSTALL.md).

## Overview

`ntn-fapi` provides the **SCF-222 functional API between the MAC (L2) and the PHY (L1)** as a clean, header-only **C++ struct model** for NR-NTN simulation. It mirrors the message and PDU **field names** of the Small Cell Forum FAPI specification (SCF FAPI 222.10.02 + the 222.10.04 addendum), giving scheduler-side code in ns-3 (mmwave, oran-ntn) a common, spec-named vocabulary. **This is a source-level C++ model, not a binary/wire ABI:** the structs use `std::vector`/`std::variant` and have no fixed byte layout, so they are not a drop-in link target for NVIDIA Aerial cuPHY or OAI's nfapi and are not serialized over a wire — they are a reference shape a translator could target.

The message and PDU types carry **real transport-block bytes slot-by-slot over NR-NTN timing**: the MAC builds a `DL_TTI.request` plus a `TX_DATA.request` carrying the actual TB byte buffer each scheduled slot, the PHY returns an `RX_DATA.indication` (received bytes) plus a `CRC.indication` (pass/fail + UL CQI), and the MAC drives **HARQ retransmission** on a CRC NACK. In the shipped examples the L1 outcome that fills the `CRC.indication` is **measured** off a real mmwave NR NTN cell (the recent DL SINR/TBLER from the PHY trace), so genuine data crosses the FAPI with real CRC and HARQ feedback and goodput tracks the measured radio.

The structs are intentionally free of algorithmic logic — the role of this module is to provide a stable, SCF-222-named C++ struct shape, plus a couple of conversion helpers, that other ns-3 modules can share (a wire/binary ABI for external PHYs would require adding real serialization, which this module does not yet do).

## What's new in v2

See the toolkit [CHANGELOG](../../CHANGELOG.md).

- **All three examples now ride a REAL mmwave NR NTN cell** (`NtnRealStackHelper` from `ntn-traffic`: SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC). The per-slot FAPI `CRC.indication` is decided by the **measured** PHY outcome — recent DL SINR/TBLER read off the mmwave `RxPacketTraceUe` trace (the real SINR→BLER error model) — not a closed-form `SinrToBler()` sigmoid or a coin flip.
- **Real NTN mobility everywhere**: the serving satellite is an SGP4/Walker `Sgp4MobilityModel` (from `ntn-constellation`) and the ground UEs use TR 38.811 mobility classes (`NtnTr38811MobilityHelper`, from `ntn-cho`), so the pass geometry is genuine.
- **NEW flagship example `ntn-fapi-real-stack`** — the canonical "FAPI ABI on a measured radio" scenario.
- `ntn-fapi-leo-pass-slotloop` drives the FAPI data ABI from a **real SGP4 LEO pass from a TLE**, with the ground station auto-placed at the satellite's t=0 sub-point so a real rise→zenith→set pass occurs.
- The earlier closed-form variants (analytical triangular SINR, `ElevationToSinrDb`) are gone.

## Models, helpers & key classes

All types live in `namespace ns3::fapi`. Headers under `model/`:

- **`fapi-common.h`** — shared enums and config TLVs: `MessageId` (message-type codes: `kDlTtiRequest`, `kTxDataRequest`, `kRxDataIndication`, `kCrcIndication`, `kUlTtiRequest`, `kSlotIndication`, `kUlDciRequest`, `kUciIndication`, `kSrsIndication`, `kRachIndication`), `MessageIdName()`, `CyclicPrefix`, `Numerology` (mu 0–4) + `SubCarrierSpacingKhz()`, and the `CarrierConfig` / `CellConfig` / `BwpInfo` / `CceRegMapping` / `DmrsConfig` structs. `kScfFapiBase` / `kScfFapiAddendum` record the tracked spec versions.
- **`fapi-messages.h`** — message-level typedefs, each with a `static constexpr MessageId kId`: `DlTtiRequest` (with `DlTtiPdu` variant over PDCCH/PDSCH/CSI-RS), `UlTtiRequest`, `SlotIndication`, `UlDciRequest`, `TxDataRequest` (carries `PduPayload::tbBytes` — the real TB), `RxDataIndication` (carries `PduRx::tbBytes`), `CrcIndication` (per-TB/per-CBG CRC status + `ul_cqi`), `SrsIndication`, `RachIndication`.
- **`fapi-pdu-types.h`** — per-PDU descriptors carried inside the scheduling messages, with SCF wire-format field names: `PdcchPdu` (+ `Dci`), `PdschPdu` (+ `Codeword`), `CsiRsPdu`, `PuschPdu`, `PucchPdu`, `SrsPdu`, `PrachPdu`.
- **`fapi-helpers.h` / `.cc`** — `DmrsFapiToBitArray(uint16_t)` expands the 14-bit `dmrsSymbPos` bitmap into OFDM symbol indices; `DmrsBitArrayToFapi(...)` is its inverse (lenient, drops indices ≥ 14); `GetMessageId<Msg>()` returns the `kId` of any typed message struct.

## Examples

All three examples are listed in `examples/CMakeLists.txt` and build to `build/contrib/ntn-fapi/examples/`. They depend on the toolkit's `mmwave`, `ntn-traffic`, `ntn-constellation`, and `ntn-cho` modules. Each writes an honest `sim_health.csv` (measured-KPI fidelity gates, via `NtnRealStackHelper::WriteHealthReport()`) to `--outputDir` in addition to its stdout summary.

### ntn-fapi-real-stack

Real-stack flagship: the SCF-222 data ABI (DL_TTI / TX_DATA / RX_DATA / CRC.indication) exercised slot-by-slot on a real mmwave NR NTN cell. The serving satellite is an SGP4 Walker satellite; TR 38.811 UEs sit under its t=0 sub-point; CRC pass/fail and HARQ feedback follow the measured TBLER.

```sh
./ns3 run "ntn-fapi-real-stack --duration=20 --numUes=4 --scsKhz=30"
```

- **Outputs:** stdout — a header block, a `--- FAPI SAP (CI gate 15: real request->indication latency) ---` block, and a final `--- FAPI Summary (SCF-222 ABI on MEASURED radio) ---` with measured mean SINR, measured mean DL TBLER, measured radio throughput, FAPI slots sent/crcOk/retx, delivered KB, and FAPI goodput (Mbps); plus `sim_health.csv` and **`fapi_sap.csv`** in `--outputDir`. `fapi_sap.csv` persists the measured SCF-222 SAP latency (previously console-only): slot / DL_TTI / CRC counts (`slot_indication_count`, `dl_tti_request_count`, `dl_tti_with_data_count`, `crc_indication_count`, `matched_latency_count`) plus `sap_latency_mean_us` / `_min_us` / `_max_us` and `sched_pipeline_mean_us`. The SAP latency is the **CI gate-15 KPI** and is genuinely **measured** — the `DL_TTI.request → CRC.indication` interval read off the real mmwave MAC↔PHY SAP, SFN/slot-aligned (a representative run: mean ≈ 627 µs over ≈ 21 000 matched samples), not a synthesised figure.
- **Key args:** `--duration` (s), `--numUes`, `--scsKhz` (15/30/60/120 → slots per ms), `--tbBytes`, `--altitude` (km), `--satEirpDbm`, `--outputDir`.

### ntn-fapi-dl-data-slotloop

The original slot-loop demo, now on the same measured radio: per-slot `TX_DATA.request` → `RX_DATA.indication` / `CRC.indication` with the CRC outcome drawn from the measured TBLER of UE 0; the natural elevation descent of the pass drops the measured SINR so CRC failures and HARQ retransmissions appear.

```sh
./ns3 run "ntn-fapi-dl-data-slotloop --simSeconds=20 --numUes=4 --scsKhz=30 --tbBytes=1500"
```

- **Outputs:** stdout header + `--- FAPI Summary (SCF-222 ABI on MEASURED radio) ---` (same fields as above); `sim_health.csv` in `--outputDir`.
- **Key args:** `--simSeconds`, `--numUes`, `--scsKhz`, `--tbBytes`, `--altitude` (km), `--satEirpDbm`, `--outputDir`.

### ntn-fapi-leo-pass-slotloop

Same FAPI data path, but the serving satellite is propagated from a **TLE** by `ntn-constellation`'s `Sgp4MobilityModel`, and the ground station is auto-placed at the satellite's t=0 sub-point so a real rise→zenith→set pass occurs regardless of TLE epoch.

```sh
./ns3 run "ntn-fapi-leo-pass-slotloop --simSeconds=20 --scsKhz=30 --tle=contrib/ntn-rrc/data/iss-zarya.tle"
```

- **Outputs:** stdout — a header (TLE path, GS sub-point, sim config) and a final `# === ntn-fapi-leo-pass-slotloop summary ===` block with measured SINR/TBLER/throughput, FAPI slots sent/crcOk/retx, delivered KB, and goodput (Mbps); `sim_health.csv` in `--outputDir`.
- **Key args:** `--simSeconds`, `--scsKhz`, `--tbBytes`, `--numUes`, `--satEirpDbm`, `--tle` (path to a 3-line TLE; defaults to `contrib/ntn-rrc/data/iss-zarya.tle`), `--outputDir`.

## Build, run & test

```sh
./ns3 configure --enable-examples --enable-tests
./ns3 build
./test.py --suite=ntn-fapi
```

The `ntn-fapi` test suite (5 unit tests) covers SCF 222.10.02/.04 message-ID stability and names, Numerology→SCS mapping against TS 38.211 Table 4.2-1, DMRS bitmap round-trips, DL_TTI PDCCH+PDSCH PDU ordering, and the UL indication shapes (CRC/SRS/RACH). See [INSTALL](../../INSTALL.md) for full toolkit setup.

## License & author

GPL-2.0-only. Muhammad Uzair, Independent Researcher.
