<!--
Copyright (c) 2026 Muhammad Uzair
SPDX-License-Identifier: GPL-2.0-only
-->

# Vendored 5G-LENA (nr) provenance and NTN patch series

## What this module is

`contrib/nr` is **upstream 5G-LENA `nr` v3.3.1** (the latest release of the
`5g-lena-v3.3.y` line, 15 Oct 2024) carried on the toolkit's **ns-3.43** base,
with **five** local adaptation patches applied on top. Nothing else in
the tree diverges from pristine upstream v3.3.1: a full-tree content diff of
`model/`, `helper/` and `utils/` against pristine v3.3.1 reports only the five
files below, and no added or removed files.

v3.3.y is the newest 5G-LENA line that targets an ns-3.42/3.43-class base;
therefore the vendored module already tracks the latest upstream release that
is compatible with this toolkit's ns-3 base. See
`5G_LENA_UPGRADE_ASSESSMENT_2026-07-18.md` (repo root) for why v4.0 (ns-3.44)
and v5.0 (ns-3.48, the only line with native NTN) are ns-3-base migrations
rather than in-place `nr` bumps.

## The four patches (apply in order from `contrib/nr/`)

    for p in ntn-patches/0*.patch; do patch -p1 < "$p"; done

The five (apply in order):

| # | File | Why |
|---|------|-----|
| 01 | `model/nr-common.h` | `#undef` ns-3.43 LTE's `MIN_NO_CC`/`MAX_NO_CC` *macros* before nr's `constexpr` redeclaration — without this the two collide and nr fails to compile on ns-3.43. |
| 02 | `model/nr-mac-scheduler-ns3.cc` | Tolerate a stale/zero CQI across an X2 handover reconfiguration instead of wedging the scheduler — required for NTN inter-satellite handover to complete. |
| 03 | `model/nr-pdcp-header.cc` | TS 38.323 §6.2.2/§6.2.3: deserialize a Control PDU (D/C=0) instead of `NS_ASSERT`-aborting. A real X2/Xn handover with non-zero link delay (any NTN feeder/ISL leg) delivers Control PDUs to PDCP; asserting turned a spec-mandated PDU into a crash. |
| 04 | `model/nr-pdcp.cc` | TS 38.323 §6.2.3: `DoReceivePdu` consumes and discards a Control PDU (status report / RoHC feedback — not modelled here) rather than delivering it upward as an SDU or advancing the RX sequence number. |
| 05 | `model/nr-gnb-phy.cc` | TS 38.213 §4.2 NTN K_offset: raise the `N2Delay` attribute checker upper bound from 4 to 320 slots. The terrestrial cap of 4 slots cannot hold the NTN cell-specific K_offset (~9–10 slots for a 600 km LEO round trip, far more for MEO/GEO), so `NtnRealStackHelper::SetKOffsetConsumption(true)` can program the geometry-derived K_offset into the UL DCI→PUSCH gap. |

## Reproducing the vendored tree from scratch

    # 1. fetch pristine upstream
    git clone --branch 5g-lena-v3.3.y https://gitlab.com/cttc-lena/nr.git nr-v331
    # 2. copy into contrib/nr on the ns-3.43 tree, then:
    cd contrib/nr && for p in ntn-patches/0*.patch; do patch -p1 < "$p"; done

Verified 2026-07-18: pristine v3.3.1 + these four patches reproduces the
vendored `model/` files byte-for-byte (0 mismatches).

## Upgrading later

When the toolkit's ns-3 base moves to 3.44 (→ nr v4.0) or 3.48 (→ nr v5.0,
native NTN), drop in the pristine upstream `nr` for that base and re-apply
whichever of these four patches upstream has not itself fixed. Patches 03/04
(PDCP Control-PDU handling) are the ones most likely still required, since they
reflect a modelling scope choice, not an ns-3-version quirk.
