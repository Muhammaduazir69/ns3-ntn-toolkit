<h1 align="center">ntn-rrc</h1>

<p align="center"><strong>3GPP Rel-17/18/19 NTN-specific RRC procedures for the <a href="https://github.com/Muhammaduazir69/ns3-ntn-toolkit">ns3-ntn-toolkit</a>.</strong></p>

<p align="center">
  <em>Part of the v2.0 roadmap (<a href="../../ROADMAP_EXECUTION.md">Workstream W2</a>) — Phase 1.2 NTN protocol compliance.</em>
</p>

---

## Why this module exists

The toolkit's `ntn-cho` module handles handover at the application layer. The 3GPP RRC procedures *underneath* CHO — Timing Advance pre-compensation, GNSS-assisted RRC, SIB19 (NTN assistance information), regenerative-vs-transparent payload mode, NTN-DRX, UE location reporting — were unmodelled. Without those, the RACH preambles in any NTN scenario arrive far outside the receiver window, and CHO timing decisions sit on top of an impossible RRC layer.

`ntn-rrc` adds those procedures as a clean, optional contrib module.

## Specs implemented

| Procedure | Spec reference | This module |
|---|---|---|
| Timing Advance pre-compensation | TS 38.213 §4.2.2 + TR 38.821 §6.3.3 | `model/ntn-timing-advance` ✅ |
| Common TA / UE-specific TA decomposition | TS 38.331 NTN-Config IE | `model/ntn-timing-advance` ✅ |
| TA drift rate signalling | TR 38.821 §6.3.3 | `model/ntn-timing-advance` ✅ |
| Payload modes (transparent / regenerative) | TR 38.821 §4.2 | `model/ntn-rrc-types.h` ✅ |
| SIB19 broadcast (NTN assistance info) | TS 38.331 §6.3.2 | `model/ntn-sib19` ✅ |
| GNSS-assisted RRC + UE location reporting | TS 38.331 §5.7.4 | `model/ntn-ue-location-report` ✅ |
| NTN-DRX (with pass-aware deep sleep) | TS 38.321 + TR 38.821 §6.3.4 | `model/ntn-drx` ✅ |

## Quick start

```cpp
#include "ns3/ntn-rrc-helper.h"
#include "ns3/ntn-timing-advance.h"

using namespace ns3;
using namespace ns3::ntnrrc;

NtnRrcHelper helper;
helper.SetPayloadMode(PayloadMode::Transparent);
helper.SetReferencePosition(Vector{0, 0, 0}); // beam centre on ground

Ptr<NtnTimingAdvance> ta = helper.InstallTimingAdvance(ueMobility, satelliteMobility);

Time taTotal     = ta->ComputeTotalTa();         // 2 * d / c (transparent)
Time taCommon    = ta->ComputeCommonTa();        // SIB19-broadcast value
Time taResidual  = ta->ComputeUeSpecificTa();    // total - common
double driftRate = ta->ComputeTaDriftRate();     // s/s
```

The TA model consumes any `MobilityModel` — typically `SatelliteSGP4MobilityModel` (SNS3) fed by the [W1 ntn-constellation](../ntn-constellation/) module.

## Example: full LEO pass

```bash
./ns3 build ntn-rrc-leo-pass
./build/contrib/ntn-rrc/examples/ns3.43-ntn-rrc-leo-pass-debug \
    --simTime=600 --csv=/tmp/ntn-rrc-pass.csv
```

The CSV captures the classic NTN "smile" curve: TA peaks at ~17 ms when the satellite is far on the horizon, drops to ~3.7 ms at zenith (the exact `2 × 550 km / c = 3.668 ms` closed form), and rises again as the satellite passes.

## Tests

```bash
./test.py --suite=ntn-rrc -v
```

16 unit tests in `test/ntn-rrc-test-suite.cc` plus a Python W1+W2 integration test in `tests-py/test_w1_w2_integration.py`:

**Unit tests:**

| Test | Asserts |
|---|---|
| `NtnTimingAdvanceClosedFormTest` | `Total TA = 2·d/c` for transparent payload (10 ns tolerance). |
| `NtnTimingAdvanceRegenerativeTest` | Regenerative payload halves TA (single-leg). |
| `NtnTimingAdvance38821ReferenceTest` | TA at 600 km nadir matches TR 38.821 reference within 5%. |
| `NtnTimingAdvanceCommonAndUeSpecificTest` | `total = common + ue-specific` decomposition holds; off-centre UE has non-zero residual. |
| `NtnTimingAdvanceDriftRateTest` | LEO drift rate < 50 µs/s (TR 38.821 bound). |
| `Sib19CodecRoundTripTest` | Serialise → parse round-trips every SIB19 field (124 bytes). |
| `Sib19CodecRejectsTruncatedTest` | Codec returns `false` on undersized buffer. |
| `Sib19BroadcasterTickTest` | Broadcaster ticks every 160 ms snapshotting fresh ephemeris. |
| `GeodeticConversionRoundTripTest` | ECEF↔WGS-84 round-trips for 5 sample points to <1 mm. |
| `PeriodicLocationReporterTest` | Periodic mode emits one report per period. |
| `EventTriggeredReporterTest` | Event-triggered mode respects move-distance threshold (50 m/s × 100 m → ~7 reports/15 s). |
| `OnDemandReporterTest` | OnDemand mode emits exactly when `ReportNow()` is called. |
| `DrxStandardCycleTest` | DRX visits Active / OnDuration / ShortSleep / LongSleep correctly; 1.56% on-duty over 1 s. |
| `DrxDataActivityTest` | `NotifyDataActivity()` forces the SM into `Active`. |
| `DrxPassAwareTest` | Pass-aware mode enters `AwaitingPass` deep sleep when next pass is far. |
| `DrxInvalidConfigTest` | Malformed configs (zero `onDuration`, `shortCycle < onDuration`) are rejected. |

**Integration test (`tests-py/test_w1_w2_integration.py`):**

Pulls a Starlink TLE through W1 (CelesTrak with embedded fallback), spawns the C++ `ntn-rrc-from-tle` example over a 600 s pass with `SatSGP4MobilityModel` (SNS3), and compares each TA sample against an independent Skyfield reference. Pass criterion: max |error| < 200 µs and drift bias < 0.5 µs/s. Last run: 121 samples, mean 6.6 µs / max 12.8 µs error, drift bias 0.02 µs/s.

```bash
cd contrib/ntn-constellation
.venv/bin/python ../ntn-rrc/tests-py/test_w1_w2_integration.py
```

## Examples shipped

| Binary | Purpose |
|---|---|
| `ntn-rrc-leo-pass` | Single-component demo: TA only over a 600 s LEO pass; emits the classic "smile" CSV. |
| `ntn-rrc-full-stack` | All four W2 components running together over a pass; 4 CSVs (ta / sib19 / ue / drx). |
| `ntn-rrc-from-tle` | Reads a real 3-line TLE, drives `SatSGP4MobilityModel` (SNS3), runs TA. Used by the W1+W2 integration test. |

## Audit results (2026-05-04)

Stress-tested at production scenario length (1800 s = 30 min) as part of the
W1–W4 integration audit (`AUDIT_W1_W4.md`):

**`ntn-rrc-leo-pass --simTime=1800`:**

| t (s) | TA (µs) | drift (µs/s) | meaning |
|---:|---:|---:|---|
| 0 | 13 837 | −48.8 | sat 2 Mm west, approaching |
| 263 | 3 669 | −0.3 | **zenith — drift sign flips here** |
| 598 | 17 330 | +49.5 | departing, near asymptote |
| 1198 | 47 460 | +50.5 | approaching analytic limit |
| 1798 | 77 785 | **+50.6** | = **2·v/c** at v=7590 m/s (4-sig-fig match) |

The drift saturating at exactly +50.60 µs/s confirms `ComputeTaDriftRate()`
uses a **closed-form velocity projection**, not a Simulator-step finite
difference — the documented behaviour holds at long-run scale.

**`ntn-rrc-full-stack --simTime=1800`:**

| Cadence | Measured | Expected | Match |
|---|---:|---:|:---:|
| SIB19 broadcasts (160 ms) | 11 250 | 1800 / 0.160 = 11 250 | exact |
| UE reports (5 s) | 360 | 1800 / 5 = 360 | exact |
| TA samples (1 Hz) | 1 800 | 1800 | exact |

**W1→W2 live CelesTrak integration:** 121 samples over a 600 s pass of
STARLINK-1008, mean &#124;err&#124; vs Skyfield = **2.6 µs**, max 5.8 µs,
drift 0.009 µs/s.

**W2→W3 KPI fidelity:** 1800 / 1800 TA values landing in InfluxDB line
protocol match the analytic ground truth within ≤ 1 µs (the unavoidable
`Time::GetMicroSeconds()` int truncation). Bit-exact.

## What's next

W2 is complete and audit-verified. Next workstream per
`ROADMAP_EXECUTION.md`: **W5 — SAGIN** (HAPS + UAV + A2G channel).

## License

GPL-2.0-only.

## Maintainer

Muhammad Uzair — `muhammaduzairr69@gmail.com` (ORCID: 0009-0002-4104-2680)
