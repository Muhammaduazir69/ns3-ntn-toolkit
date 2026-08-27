<h1 align="center">ntn-fapi</h1>

<p align="center"><strong>An SCF-222 FAPI MAC-PHY adapter on a live NR SAP, with a latency gate anchored to the link it runs on</strong></p>

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg" alt="ns-3.43"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0-green.svg" alt="GPL-2.0"/></a>
  <img src="https://img.shields.io/badge/SCF-FAPI%20222.10.02-orange.svg" alt="Small Cell Forum FAPI 222.10.02"/>
  <img src="https://img.shields.io/badge/messages-P5%20%C2%B7%20P7-purple.svg" alt="P5 and P7 message sets"/>
  <img src="https://img.shields.io/badge/examples-3-informational.svg" alt="3 examples"/>
</p>

<p align="center">
  <a href="https://github.com/Muhammaduazir69/ns3-ntn-toolkit">Toolkit</a>
  &nbsp;·&nbsp;
  <a href="INSTALL.md">Install</a>
  &nbsp;·&nbsp;
  <a href="#examples">Examples</a>
  &nbsp;·&nbsp;
  <a href="https://muhammaduazir69.github.io/ns3-ntn-toolkit/modules/ntn-fapi/">Docs</a>
</p>

---

FAPI is the interface between MAC and PHY that real base stations are built on, and the reason to model it in simulation is to ask what an NTN round trip does to a slot pipeline designed for a terrestrial one.

The bridge decorates a live NR MAC-PHY service access point, so SLOT.indications fire at the radio's own cadence and DL_TTI.requests come from real downlink allocations rather than from a timer. HARQ state is keyed by frame, subframe, slot and RNTI, because keying by slot alone let two terminals in the same slot overwrite each other.

One thing this module is careful to say rather than imply: the backend it decorates carries no propagation delay model, so the measured request-to-indication interval is a slot-pipeline turnaround and not an NR-NTN one. The gate asserts the mean stays *below* the geometric floor of the configured link while propagation is absent, so the number cannot silently acquire a delay its label does not admit, and the bridge reports that qualification alongside the value.

## Quick start

Inside the toolkit, where the module is already present and built:

```bash
./ns3 run ntn-fapi-real-stack
./ns3 run ntn-fapi-leo-pass-slotloop
```

This module ships **only** inside the toolkit tree; there is no standalone
repository for it. It decorates the NR MAC-PHY SAP that `ntn-traffic` builds,
so it needs that spine rather than a bare ns-3. `INSTALL.md` in this directory
carries the full dependency list.

## Overview

`ntn-fapi` provides the **SCF-222 functional API between the MAC (L2) and the PHY (L1)** as a clean, header-only **C++ struct model** for NR-NTN simulation. It mirrors the message and PDU **field names** of the Small Cell Forum FAPI specification (SCF FAPI 222.10.02 + the 222.10.04 addendum), giving scheduler-side code in ns-3 (mmwave, oran-ntn) a common, spec-named vocabulary. **This is a source-level C++ model, not a binary/wire ABI:** the structs use `std::vector`/`std::variant` and have no fixed byte layout, so they are not a drop-in link target for NVIDIA Aerial cuPHY or OAI's nfapi and are not serialized over a wire — they are a reference shape a translator could target.

The message and PDU types carry **real transport-block bytes slot-by-slot over NR-NTN timing**: the MAC builds a `DL_TTI.request` plus a `TX_DATA.request` carrying the actual TB byte buffer each scheduled slot, the PHY returns an `RX_DATA.indication` (received bytes) plus a `CRC.indication` (pass/fail + UL CQI), and the MAC drives **HARQ retransmission** on a CRC NACK. In the shipped examples the L1 outcome that fills the `CRC.indication` is **measured** off a real mmwave NR NTN cell (the recent DL SINR/TBLER from the PHY trace), so genuine data crosses the FAPI with real CRC and HARQ feedback and goodput tracks the measured radio.

The structs are intentionally free of algorithmic logic — the role of this module is to provide a stable, SCF-222-named C++ struct shape, plus a couple of conversion helpers, that other ns-3 modules can share (a wire/binary ABI for external PHYs would require adding real serialization, which this module does not yet do).

## What changed in v2.5

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

- **Outputs:** stdout — a header block, a `--- FAPI SAP (CI gate 15: real request->indication latency) ---` block, and a final `--- FAPI Summary (SCF-222 ABI on MEASURED radio) ---` with measured mean SINR, measured mean DL TBLER, measured radio throughput, FAPI slots sent/crcOk/retx, delivered KB, and FAPI goodput (Mbps); plus `sim_health.csv` and **`fapi_sap.csv`** in `--outputDir`. `fapi_sap.csv` persists the measured SCF-222 SAP latency (previously console-only): slot / DL_TTI / CRC counts (`slot_indication_count`, `dl_tti_request_count`, `dl_tti_with_data_count`, `crc_indication_count`, `matched_latency_count`) plus `sap_latency_mean_us` / `_min_us` / `_max_us` and `sched_pipeline_mean_us`. The SAP latency is the **CI gate-15 KPI** and is genuinely **measured** — the `DL_TTI.request → CRC.indication` interval read off the real mmwave MAC↔PHY SAP, SFN/slot-aligned (a representative run: mean ≈ 627 µs over ≈ 21 000 matched samples), not a synthesised figure. **What it does not contain:** this bridge runs on the mmwave backend, whose SpectrumChannel carries no `PropagationDelayModel`, so the interval is the slot pipeline alone — no part of the NTN round trip is in it. 627 µs is below the 2.00 ms one-way floor a 600 km link imposes, which is the arithmetic proof of that rather than a caveat taken on trust. Read it as an L1/L2 *slot-pipeline* turnaround, not as an NR-NTN turnaround. The bridge probes the live channel and reports the qualification alongside the number (`GetSapLatencyProvenance()` → `measured-no-air-propagation`), and the gate-15 test asserts the mean stays **below** the geometric floor while propagation is absent, so the figure cannot silently acquire a delay its label does not admit.
- **Key args:** `--duration` (s), `--numUes`, `--scsKhz` (15/30/60/120 → slots per ms), `--tbBytes`, `--altitude` (km), `--satEirpDbm`, `--outputDir`.

### ntn-fapi-dl-data-slotloop

The original slot-loop demo, now on the same measured radio: per-slot `TX_DATA.request` → `RX_DATA.indication` / `CRC.indication` with the CRC outcome drawn from the measured TBLER of UE 0; the natural elevation descent of the pass drops the measured SINR so CRC failures and HARQ retransmissions appear. **The slot loop is gone.** Until v2.5.0 this example (and `ntn-fapi-leo-pass-slotloop`) assembled its DL_TTI/TX_DATA structs on a self-scheduled timer with its own SFN/slot counters, uncorrelated with the radio's — printed beside real measured SINR, which made it look driven. Both now install `NtnFapiSapBridge`, so every message is emitted by a real MAC↔PHY SAP call and the SFN/slot are the radio's own. The cross-check is in the output: `TX_DATA.request` equals the health line's `phy_rx_tb` exactly (29 871 on the default run), which a free-running loop could not do.

```sh
./ns3 run "ntn-fapi-dl-data-slotloop --simSeconds=20 --numUes=4 --scsKhz=30 --tbBytes=1500"
```

- **Outputs:** stdout header + `--- FAPI Summary (SCF-222 ABI on MEASURED radio) ---` (same fields as above), then `--- FAPI messages off the REAL MAC<->PHY SAP (no shadow loop) ---` with the SLOT.indication / DL_TTI / UL_TTI / TX_DATA / RACH.indication counts the bridge actually emitted and the SAP latency with its provenance; `sim_health.csv` in `--outputDir`.
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

---

## Standards implemented

Small Cell Forum FAPI 222.10.02 (P5 configuration and P7 slot-timing message sets, DL_TTI.request, TX_DATA.request, RX_DATA.indication, CRC.indication, SLOT.indication, RACH.indication). 3GPP TS 38.211 and TS 38.213 for the slot and frame timing the messages carry, TR 38.821 for the NTN delay budget the gate is anchored to.

## Keywords

FAPI, Small Cell Forum, SCF-222, MAC-PHY interface, L1 L2 split, DL_TTI, TX_DATA, CRC indication, slot indication, HARQ, functional split, open RAN fronthaul, slot pipeline latency, NR numerology, satellite base station, non-terrestrial network, ns-3.

## Author

**Muhammad Uzair**, Independent Researcher
[ORCID 0009-0002-4104-2680](https://orcid.org/0009-0002-4104-2680)

Part of the [ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit),
a pre-integrated ns-3.43 platform for 6G non-terrestrial network research.
Mirrored on [GitLab](https://gitlab.com/ns3-ntn-toolkit).

## License

GPL-2.0-only, matching ns-3.
