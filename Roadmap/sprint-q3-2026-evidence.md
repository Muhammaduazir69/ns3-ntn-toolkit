# Q3 2026 Sprint Evidence — Realism Adoption Roadmap

**Author:** Muhammad Uzair, Independent Researcher
**Date:** 2026-05-23
**Scope:** Opening four workstreams of `Realism-Adoption-Roadmap-2026.md`
  — T9, T8, T1, 4.1.1.

## Commits landed (branch `ntn-integration`)

| SHA | Workstream | Subject |
|---|---|---|
| `a50c0557f` | **T9** (§3 T9)   | `ntn-observability`: reproducibility manifest module |
| `a5ca04b2e` | **T8** (§3 T8)   | `oran-ntn`: align KPM metric IDs to WG3 canonical names |
| `26b71b181` | **T1** (§3 T1)   | `ntn-fapi`: new module with SCF FAPI 222.10.02/.04 typedefs |
| `e927e1c34` | **4.1.1** (§4.1.1) | `oran-ntn`: FlexRIC field-name parity types |

All commits use `Muhammad Uzair <muhammaduzairr69@gmail.com>`. No co-authors.
Not pushed (per standing instruction).

## Per-suite test results

```
ntn-observability  PASS  10/10 cases  (5 pre-existing + 5 manifest)
oran-ntn           PASS  23/23 cases  (18 pre-existing + 3 T8 + 2 4.1.1)
ntn-fapi           PASS   5/5  cases  (all new T1 typedef coverage)
                  ----  ------------
TOTAL                    38/38 cases
```

Suites run via `build/utils/ns3.43-test-runner-default --suite=<name> --verbose`.

## Cross-module integration check

The manifest emitted by the C++ writer (`NtnReproManifest::WriteJson`) round-trips
through an external Python consumer:

```text
cross-module manifest validation: PASS
  schema = ns3-ntn-toolkit/manifest v1
  scenario = oran-ntn-full-scenario 600 s
  constellation = Walker-Star planes=6 sats=11 alt=550 km
  service models = KPM=v3.00, RC=v1.03
```

The KPM service-model name `KPM v3.00` recorded in the manifest is the
same string the canonical metric IDs (`oran-ntn-kpm-canonical-ids.h`)
align to, and the same string the FlexRIC parity layer's
`kpm_v3::meas_type_t::meas_name` carries verbatim. All four workstreams
agree on this single text identity.

## Coverage of the §3 / §4 roadmap items

| Roadmap | Status | Files |
|---|---|---|
| §3 T9 Reproducibility manifest                | DONE | `contrib/ntn-observability/model/ntn-repro-manifest.{h,cc}` |
| §3 T8 KPM metric-ID alignment (10 IDs + 3 dims) | DONE | `contrib/oran-ntn/model/oran-ntn-kpm-canonical-ids.{h,cc}` |
| §3 T1 SCF FAPI typedefs (9 msgs + 7 PDUs)     | DONE | `contrib/ntn-fapi/model/fapi-{common,pdu-types,messages,helpers}.{h,cc}` |
| §4.1.1 FlexRIC e2ap_msg_t + kpm_ind_msg_format_1_t parity | DONE | `contrib/oran-ntn/model/oran-ntn-flexric-types.h` |

## Knock-on items deferred to later workstreams

- `OranNtnServiceModelKpm` plugin class — T4 (Q4 2026); for v2.1 the
  canonical IDs live as toolkit-wide constants.
- `OranNtnDataRepository` SQLite logger — 4.1.4 (Q3 2026 sprint, not yet
  started); will replace the throughput→volume stand-in mapping in
  `BuildCanonicalKpmMeasurements`.
- `thz-ntn-isac` example — gated with `if(FALSE)` until §4.3.5 rewrites
  the ISAC scheduler (Q4 2026); the example call-sites reference an
  outdated `ThzNtnIsac` API.
- UL throughput / UL PDCP volume / UL PRBs — emitted with the
  `present=false` sentinel until the CU/DU/RU split (§4.1.9, Q1 2027)
  plumbs the UL counters end-to-end.

## Next sprint candidates

Per §7 Q3 2026 the remaining must-finish items are:

- **4.1.2–4.1.4** — wire T8 canonical IDs through the actual extractor
  output, retag HO commands as E2SM-RC Style 3, add the SQLite
  `OranNtnDataRepository` logger.
- **4.2.1–4.2.2** — `ntn-sionna` pybind11 transport + bump to Sionna RT 2.0.1
  with MIMO.
- **4.3.1–4.3.2** — `thz-ntn` HITRAN-2024 LUT + ITU-Rpy pybind11 wrapper.
- **4.4.1–4.4.3** — `ntn-sagin` OpenSky + AIS + HST trace importers.

All four cover ~6 weeks of effort, parallelisable; closes the v2.1
"realism baseline" public release per §7.
