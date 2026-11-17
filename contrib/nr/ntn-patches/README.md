<!--
Copyright (c) 2026 Muhammad Uzair
SPDX-License-Identifier: GPL-2.0-only
-->

# Vendored 5G-LENA (nr) provenance and NTN patch series

## What this module is

`contrib/nr` is **upstream 5G-LENA `nr` v3.3.1** (the latest release of the
`5g-lena-v3.3.y` line, 15 Oct 2024) carried on the toolkit's **ns-3.43** base,
with **fourteen** local adaptation patches applied on top. Nothing else in
the tree diverges from pristine upstream v3.3.1: a full-tree content diff of
`model/` and `helper/` against pristine v3.3.1 reports only the files of the
eleven patches below, and no added or removed files.

v3.3.y is the newest 5G-LENA line that targets an ns-3.42/3.43-class base;
therefore the vendored module already tracks the latest upstream release that
is compatible with this toolkit's ns-3 base. See
`5G_LENA_UPGRADE_ASSESSMENT_2026-07-18.md` (repo root) for why v4.0 (ns-3.44)
and v5.0 (ns-3.48, the only line with native NTN) are ns-3-base migrations
rather than in-place `nr` bumps.

## The fourteen patches (apply in order from `contrib/nr/`)

    for p in ntn-patches/[0-9][0-9]-*.patch; do patch -p1 < "$p"; done

The eleven (apply in order; the two-digit glob sorts 10 and 11 after 09):

| # | File | Why |
|---|------|-----|
| 01 | `model/nr-common.h` | `#undef` ns-3.43 LTE's `MIN_NO_CC`/`MAX_NO_CC` *macros* before nr's `constexpr` redeclaration: without this the two collide and nr fails to compile on ns-3.43. |
| 02 | `model/nr-mac-scheduler-ns3.cc` | Tolerate a stale/zero CQI across an X2 handover reconfiguration instead of wedging the scheduler: required for NTN inter-satellite handover to complete. |
| 03 | `model/nr-pdcp-header.cc` | TS 38.323 §6.2.2/§6.2.3: deserialize a Control PDU (D/C=0) instead of `NS_ASSERT`-aborting. A real X2/Xn handover with non-zero link delay (any NTN feeder/ISL leg) delivers Control PDUs to PDCP; asserting turned a spec-mandated PDU into a crash. |
| 04 | `model/nr-pdcp.cc` | TS 38.323 §6.2.3: `DoReceivePdu` consumes and discards a Control PDU (status report / RoHC feedback: not modelled here) rather than delivering it upward as an SDU or advancing the RX sequence number. |
| 05 | `model/nr-gnb-phy.cc` | TS 38.213 §4.2 NTN K_offset: raise the `N2Delay` attribute checker upper bound from 4 to 320 slots. The terrestrial cap of 4 slots cannot hold the NTN cell-specific K_offset (~9–10 slots for a 600 km LEO round trip, far more for MEO/GEO), so `NtnRealStackHelper::SetKOffsetConsumption(true)` can program the geometry-derived K_offset into the UL DCI→PUSCH gap. |
| 06 | `model/nr-mac-scheduler-tdma.cc` | `AssignRBGTDMA`: with as many active UEs as data symbols the allocator gave every UE one symbol (an 8-byte TB at MCS 0, below the DCI floor), so nothing was ever sent and the cell starved (UE12 livelock). Symbols are now shared so each scheduled UE gets a decodable TB. |
| 07 | `model/nr-spectrum-phy.cc/.h` | Half-duplex DL control/data conflicts (a UE asked to receive while transmitting, which a ms-scale air delay without UE timing advance produces) are dropped and counted (`GetHalfDuplex*Drops`) instead of aborting the run; the counters feed the boundary A6/A9 diagnosis. |
| 08 | `model/nr-ue-rrc.cc`, `model/nr-ue-phy.cc` | TS 38.331 5.5.4.1 `reportType periodical`: NrUeRrc asserted that every report configuration is event-triggered; a PERIODICAL configuration now creates a reporting entry with all measured cells and re-arms every `reportInterval` (used by `NtnRealStackHelper::SetPeriodicMeasurementReports`, so a CHO/RIC/xApp decision owner gets per-UE RSRP/RSRQ with A3 off). Opt-in (GlobalValue `NrUeRsrqFromRssi`, default false): SS-RSRQ = N x RSRP / RSSI (TS 38.215 5.1.3) instead of the constant 0 dB; opt-in (GlobalValue `NrUeRsrpLinearAverage`, default false): L1 RSRP averaged in mW (TS 38.215 5.1.1) with a saturating count, because the default dB average divides by a `uint8_t` count that wraps above 255 PSS samples per 200 ms, which produced -1600 dBm L1 samples on ntn-nr-handover-pass. |
| 09 | `model/nr-gnb-phy.cc`, `helper/nr-helper.cc` | Control, including the PSS/SSB the UE measures, is sent on the gNB's configured cell beam (BeamManager predefined beam) and falls back to quasi-omni exactly as before when none is set (TS 38.215 5.1.1 SS-RSRP is a cell-level quantity). `NrHelper::DoAssignStreamsToChannelObjects` no longer dereferences a null `ThreeGppChannelModel` when another `MatrixBasedChannelModel` is plugged in. |
| 10 | `model/nr-spectrum-phy.cc`, `model/nr-ue-net-device.cc/.h` | The UE serving cell follows handovers. `NrSpectrumPhy` labelled every UE DL TB in `RxPacketTraceUe` with `GetTargetGnb()->GetCellId()`, and `NrUeNetDevice::GetCellId()` returned the same; `m_targetGnb` is written once by `NrHelper::AttachToGnb` and no X2 handover (TS 38.331 5.3.5.5.2 reconfigurationWithSync) updates it. After the first handover every TB therefore carried the attach-time cell id, so any (cellId, RNTI) consumer lost the UE: `NtnRealStackHelper` per-UE SINR/TBLER went NaN, its per-UE RAN TB counters froze, and per-cell DL stats were credited to the old cell. `NrUeNetDevice::GetCellId()` now returns the cell the UE RRC is synchronized to (`NrUeRrc::GetCellId()`, the `targetPhysCellId` of the last reconfiguration), falling back to the attach-time gNB while the RRC has no cell, and the trace uses it. Identical to the old value until the first handover, so runs without handovers reproduce bit for bit. `GetTargetGnb()` itself is unchanged (setup-time DRB activation and beamforming pairing use it). |
| 11 | `model/nr-gnb-phy.cc` | Run-to-run determinism. `NrGnbPhy` ordered the DL control messages of a slot with comparator-less `std::list::sort()` / `merge()`, which use `ns3::operator<(Ptr, Ptr)` and therefore compare raw heap addresses: the order in which a UE received, e.g., SIB1 and an UL DCI in the same slot depended on the allocator, so outputs changed with ASLR, the output-path length or any unrelated code change (the ntn-slice paper examples alternated between outputs on one binary). The messages are now ordered by message type with the same (stable) `sort`/`merge`, so messages of one type keep their insertion order. This can change the output of any run (the old order was arbitrary, not standard); it makes every run reproducible. Found by Wave 2 M2. |
| 12 | `model/nr-gnb-mac.cc/.h`, `model/nr-ue-mac.cc` | RACH at scale. `NrGnbMac::m_rapIdRntiMap` (RNTI to RA preamble ID) was keyed by `uint8_t`, so every RNTI above 255 folded onto a lower one and a RAR could carry another UE's preamble ID once a cell had allocated 256 temporary C-RNTIs (a few dozen UEs with RACH retries); it is keyed by the 16-bit RNTI (TS 38.321 7.1). `DoBuildRarList` now forgets only the RNTIs answered in the RAR it sends instead of clearing the map, which dropped RNTIs whose Msg3 was granted in a later slot. `NrUeMac` accepts only the first RAR subPDU whose RAPID matches its preamble (TS 38.321 5.1.4); a second match made `NrUeRrc` abort with "unexpected event in state IDLE_CONNECTING". This was the "about 16 UEs" connection-setup limit: ntn-v2x-rural-highway now attaches all 40 vehicles of its trace, and ntn-scalability-load-regimes runs 60 nr UEs. Runs that never reached the bug (every paper gate) are unchanged. Wave 4. |
| 13 | `model/nr-mac-scheduler-ns3.cc/.h`, `model/nr-mac-scheduler-cqi-management.cc`, `model/nr-mac-scheduler-ue-info.h` | DL outer-loop link adaptation (OLLA). The scheduler set the DL MCS from the last wideband CQI only, so interference the CQI had not seen (a second co-channel satellite cell turning active, a handover into a loaded cell) left the first-transmission BLER near 0.34 for the rest of the run and the HARQ retransmissions inflated the delay (ntn-cho-real-stack: 29.2 ms mean). Deployed gNBs close this loop. Each UE now carries an offset in MCS indices subtracted from the CQI mapping: a NACK of a first transmission (RV 0) raises it by `OllaStep`, an ACK lowers it by `OllaStep` x target / (1 - target), so the initial BLER converges to `OllaBlerTarget` (default 0.1, the TS 38.214 5.2.2.1 CQI reference). Each feedback counts once (a NACKed process is re-evaluated every slot until its retransmission), the loop starts after the first wideband CQI, the offset is bounded to [-4, +8], and the MCS is capped at the table maximum. Same run: BLER 0.07, delay 6.2 ms, full offered load. `NrMacSchedulerNs3::OllaEnabled=false` restores the previous behavior bit for bit. Wave 5. |
| 14 | `model/nr-ue-mac.cc/.h`, `model/nr-mac-scheduler-ns3.cc/.h` | Rel-17 NTN MAC timing. (a) TS 38.321 5.1.4: `NrUeMac::RaResponseWindowStartOffset` (Time, default 0) delays both edges of ra-ResponseWindow by T_TA + k_mac, the UE-gNB RTT. nr opened the window one slot after the preamble whatever the geometry, so with a real air-interface delay at LEO-600 the first RAR (6.5 ms after the preamble) overshot the terrestrial window and the first access attempt timed out. A RAR arriving before the window opens is ignored and reported by the new `RarOutsideWindow` trace. `NtnRealStackHelper` programs the per-UE RTT when the air-interface delay is on. (b) TS 38.331 downlinkHARQ-FeedbackDisabled-r17: `NrMacSchedulerNs3::DlHarqFeedbackDisabled` (uint32 bitmap over DL HARQ process IDs, default 0) releases a feedback-disabled process when its TB is scheduled instead of holding it one HARQ round trip for an ACK/NACK, so it is never retransmitted by HARQ; late feedback for it is discarded by the existing INACTIVE-process path. The UE still sends HARQ-ACK on PUCCH for those processes (not suppressed on the UE side), and OLLA gets no updates from them. Both defaults reproduce the 01-13 behavior bit for bit. Wave 6A. |

Patch 10 changes outputs only after a handover (the UE DL RX trace cell id,
the per-UE and per-cell statistics built on it, and the UE-side cell id in the
nr-phy-rx-trace interference files). Patches 08 and 09 change nothing unless a scenario opts in (a PERIODICAL report
configuration, a predefined beam, or one of the two GlobalValues), so every
existing run reproduces bit for bit. The `uint8_t` RSRP sample counter wrap in
the default path is left in place for the same reason and is documented here.


## Reproducing the vendored tree from scratch

    # 1. fetch pristine upstream
    git clone --branch 5g-lena-v3.3.y https://gitlab.com/cttc-lena/nr.git nr-v331
    # 2. copy into contrib/nr on the ns-3.43 tree, then:
    cd contrib/nr && for p in ntn-patches/[0-9][0-9]-*.patch; do patch -p1 < "$p"; done

Verified 2026-07-18: pristine v3.3.1 + patches 01-05 reproduces the vendored
`model/` files byte-for-byte (0 mismatches). Verified 2026-11-16 for 01-09:
the pristine tree (the 2026-07-18 vendored state with 01-05 reverse-applied)
plus the nine patches reproduces `model/` and `helper/` byte-for-byte
(`diff -rq`: 0 mismatches). Verified 2026-11-16 (Wave 2 core) for 01-10: the
same pristine tree plus the ten patches reproduces `model/` and `helper/`
byte-for-byte (`diff -rq`: 0 mismatches). Verified again for 01-11 the same
day after patch 11: 0 mismatches. Verified 2026-11-17 (Wave 4) for 01-12: 0 mismatches. Verified 2026-11-17 (Wave 5) for 13 against the 01-12 state: 0 mismatches. Verified 2026-11-18 (Wave 6A) for 14 against the 01-13 state: 0 mismatches.

## Upgrading later

When the toolkit's ns-3 base moves to 3.44 (→ nr v4.0) or 3.48 (→ nr v5.0,
native NTN), drop in the pristine upstream `nr` for that base and re-apply
whichever of these patches upstream has not itself fixed. Patches 03/04
(PDCP Control-PDU handling) are the ones most likely still required, since they
reflect a modelling scope choice, not an ns-3-version quirk.
