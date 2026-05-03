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
| Payload modes (transparent / regenerative) | TR 38.821 §4.2 | `model/ntn-rrc-types.h` ✅ (enum + behaviour switch in TA model) |
| SIB19 broadcast (NTN assistance info) | TS 38.331 NTN-SIB19 | `model/ntn-sib19` (next commit) |
| GNSS-assisted RRC + UE location reporting | TS 38.331 §5.7.4 | `model/ntn-ue-location-report` (next commit) |
| NTN-DRX | TS 38.321 NTN extensions | `model/ntn-drx` (next commit) |

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

5 unit tests in `test/ntn-rrc-test-suite.cc`:

| Test | Asserts |
|---|---|
| `NtnTimingAdvanceClosedFormTest` | `Total TA = 2·d/c` for transparent payload (10 ns tolerance). |
| `NtnTimingAdvanceRegenerativeTest` | Regenerative payload halves TA (single-leg). |
| `NtnTimingAdvance38821ReferenceTest` | TA at 600 km nadir matches TR 38.821 reference within 5%. |
| `NtnTimingAdvanceCommonAndUeSpecificTest` | `total = common + ue-specific` decomposition holds; off-centre UE has non-zero residual. |
| `NtnTimingAdvanceDriftRateTest` | LEO drift rate < 50 µs/s (TR 38.821 bound). |

## What's next on this workstream

The remaining four W2 components ship in follow-up commits within the same module:

- `ntn-sib19` — broadcast structure + serialiser
- `ntn-ue-location-report` — periodic GNSS report timer + payload
- `ntn-drx` — NTN-extended DRX cycle (sleep across non-pass windows)
- `ntn-payload-mode` — explicit regenerative-vs-transparent behaviour switch hookups in upper layers

Each lands with its own test cases and updates `ROADMAP_EXECUTION.md`'s W2 status badge.

## License

GPL-2.0-only.

## Maintainer

Muhammad Uzair — `muhammaduzairr69@gmail.com` (ORCID: 0009-0002-4104-2680)
