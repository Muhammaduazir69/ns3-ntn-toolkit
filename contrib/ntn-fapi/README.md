# ntn-fapi

> SCF-222 FAPI L1↔L2 message ABI (DL_TTI / TX_DATA / RX_DATA / CRC.indication) for NR-NTN.
> Part of **ns3-ntn-toolkit** — [README](../../README.md) / [INSTALL](../../INSTALL.md).

## Overview

`ntn-fapi` provides the **SCF-222 functional API between the MAC (L2) and the PHY (L1)** as a clean, header-only C++ ABI for NR-NTN simulation. It mirrors the message and PDU layout of the Small Cell Forum FAPI specification (SCF FAPI 222.10.02 + the 222.10.04 addendum) so that scheduler-side code in ns-3 (mmwave, oran-ntn), NVIDIA Aerial cuPHY, and OAI's nfapi can all link against the same struct shapes without touching scheduler logic.

The message and PDU types carry **real transport-block bytes slot-by-slot over NR-NTN timing**: the MAC builds a `DL_TTI.request` plus a `TX_DATA.request` carrying the actual TB byte buffer each scheduled slot, the PHY "transmits" it over a satellite link and returns an `RX_DATA.indication` (received bytes) plus a `CRC.indication` (pass/fail + UL CQI), and the MAC drives **HARQ retransmission** on a CRC NACK. So genuine data crosses the FAPI with real CRC and HARQ feedback, with goodput tracking the geometry-driven SINR.

The structs are intentionally free of algorithmic logic — the role of this module (Realism-Adoption-Roadmap-2026 §3 T1) is to provide a stable ABI shape, plus a couple of conversion helpers, against which other modules and external PHYs can interoperate.

## What's new in v2

See the toolkit [CHANGELOG](../../CHANGELOG.md).

- **NEW example `ntn-fapi-leo-pass-slotloop`** drives the FAPI data ABI from a **real SGP4 LEO pass** (`ntn-constellation`'s `Sgp4MobilityModel`): per-slot SINR follows the **live satellite elevation** over a ground station (slant-range → free-space path loss → SINR), with HARQ retransmission and Shannon-tracked goodput. Out-of-contact slots (elevation ≤ 0°) correctly fall to **BLER = 1.0** with no delivery.
- The original `ntn-fapi-dl-data-slotloop` continues to exercise the same data path with an **analytical triangular SINR** (edges → zenith → edges) for a deterministic, dependency-free demo.

## Models, helpers & key classes

All types live in `namespace ns3::fapi`. Headers under `model/`:

- **`fapi-common.h`** — shared enums and config TLVs: `MessageId` (message-type codes: `kDlTtiRequest`, `kTxDataRequest`, `kRxDataIndication`, `kCrcIndication`, `kUlTtiRequest`, `kSlotIndication`, `kUlDciRequest`, `kUciIndication`, `kSrsIndication`, `kRachIndication`), `MessageIdName()`, `CyclicPrefix`, `Numerology` (mu 0–4) + `SubCarrierSpacingKhz()`, and the `CarrierConfig` / `CellConfig` / `BwpInfo` / `CceRegMapping` / `DmrsConfig` structs. `kScfFapiBase` / `kScfFapiAddendum` record the tracked spec versions.
- **`fapi-messages.h`** — message-level typedefs, each with a `static constexpr MessageId kId`: `DlTtiRequest` (with `DlTtiPdu` variant over PDCCH/PDSCH/CSI-RS), `UlTtiRequest`, `SlotIndication`, `UlDciRequest`, `TxDataRequest` (carries `PduPayload::tbBytes` — the real TB), `RxDataIndication` (carries `PduRx::tbBytes`), `CrcIndication` (per-TB/per-CBG CRC status + `ul_cqi`), `SrsIndication`, `RachIndication`.
- **`fapi-pdu-types.h`** — per-PDU descriptors carried inside the scheduling messages, with SCF wire-format field names: `PdcchPdu` (+ `Dci`), `PdschPdu` (+ `Codeword`), `CsiRsPdu`, `PuschPdu`, `PucchPdu`, `SrsPdu`, `PrachPdu`.
- **`fapi-helpers.h` / `.cc`** — `DmrsFapiToBitArray(uint16_t)` expands the 14-bit `dmrsSymbPos` bitmap into OFDM symbol indices; `DmrsBitArrayToFapi(...)` is its inverse (lenient, drops indices ≥ 14); `GetMessageId<Msg>()` returns the `kId` of any typed message struct.

## Examples

Both examples are listed in `examples/CMakeLists.txt` and build to `build/contrib/ntn-fapi/examples/`.

### ntn-fapi-dl-data-slotloop

DL FAPI data path over an **analytical triangular** pass SINR with CRC/HARQ.

```sh
./ns3 run "ntn-fapi-dl-data-slotloop --simSeconds=10 --scsKhz=30 --tbBytes=1500"
```
```sh
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-fapi/examples/ns3.43-ntn-fapi-dl-data-slotloop-default --simSeconds=10 --scsKhz=30 --tbBytes=1500
```

- **Outputs:** stdout — a header line, periodic per-slot progress lines (`t`, `sfn/slot`, `sinr`, `bler`, `tbOk/tbSent`, `deliveredKB`), and a final `# === summary ===` line with TBs sent/ok, HARQ-retx count, average BLER, delivered MB, and goodput (Mbps).
- **Key args:** `--simSeconds` (sim duration, s), `--scsKhz` (sub-carrier spacing 15/30/60/120 → slots per ms), `--tbBytes` (transport-block size), `--minSinrDb` (SINR at pass edges), `--maxSinrDb` (SINR at zenith), `--rngSeed` (RNG run number).

### ntn-fapi-leo-pass-slotloop

Same FAPI data path, but per-slot SINR is derived from a **real SGP4 LEO pass** (`ntn-constellation`).

```sh
./ns3 run "ntn-fapi-leo-pass-slotloop --simSeconds=600 --scsKhz=30 --tle=contrib/ntn-rrc/data/iss-zarya.tle"
```
```sh
LD_LIBRARY_PATH=build/lib ./build/contrib/ntn-fapi/examples/ns3.43-ntn-fapi-leo-pass-slotloop-default --simSeconds=600 --scsKhz=30 --tle=contrib/ntn-rrc/data/iss-zarya.tle
```

- **Outputs:** stdout — a header (TLE path, GS lat/lon, sim config), periodic per-slot progress lines (`t`, `elev`, `sinr`, `bler`, `tbOk/tbSent`, `deliveredKB`), and a final `# === ntn-fapi-leo-pass-slotloop summary ===` block with slots sent/ok/retx, slots in contact, max elevation, delivered KB, and goodput (Mbps).
- **Key args:** `--simSeconds`, `--scsKhz`, `--tbBytes`, `--rngSeed`, `--tle` (path to a 3-line TLE; defaults to `contrib/ntn-rrc/data/iss-zarya.tle`), `--gsLat` / `--gsLon` (ground-station coordinates in degrees; default to the satellite's t=0 sub-point so a real rise→zenith→set pass occurs).

## Build, run & test

```sh
./ns3 configure --enable-examples --enable-tests
./ns3 build
./test.py --suite=ntn-fapi
```

The `ntn-fapi` test suite covers the message/PDU ABI and the DMRS bitmap helpers. See [INSTALL](../../INSTALL.md) for full toolkit setup.

## License & author

GPL-2.0-only. Muhammad Uzair, Independent Researcher.
