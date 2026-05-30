# Q3 2026 Sprint Evidence — Realism Adoption Roadmap

**Author:** Muhammad Uzair, Independent Researcher
**Date:** 2026-05-23
**Scope:** Full Q3 2026 sprint per `Realism-Adoption-Roadmap-2026.md` §7.
  Every must-finish roadmap item is closed.

## Commits landed (branch `ntn-integration`, 19 sprint commits)

| Order | SHA | Roadmap | Subject | Tests |
|---|---|---|---|---|
| 1 | `a50c0557f` | §3 T9     | `ntn-observability`: reproducibility manifest | 5 |
| 2 | `a5ca04b2e` | §3 T8     | `oran-ntn`: WG3 canonical KPM metric IDs | 3 |
| 3 | `26b71b181` | §3 T1     | `ntn-fapi` (new): SCF FAPI 222.10.02/.04 | 5 |
| 4 | `e927e1c34` | §4.1.1    | `oran-ntn`: FlexRIC field-name parity | 2 |
| 5 | `8602e0724` | docs      | sprint evidence v1 (first 4 items) | — |
| 6 | `402e6a59c` | §4.1.2    | `oran-ntn`: canonical KPM CSV end-to-end | 1 |
| 7 | `e53e3223f` | §4.1.3    | `oran-ntn`: E2SM-RC Style 3 Mobility | 2 |
| 8 | `b3714efcb` | §4.1.4    | `oran-ntn`: NIST-pattern DataRepository | 2 |
| 9 | `148f16b89` | §4.1.10   | `oran-ntn`: WG3 conflict taxonomy | 1 |
| 10 | `74b431738` | §4.1.11   | `oran-ntn`: OSC A1 policy schemas | 1 |
| 11 | `4e9a962ce` | §3 T4     | `oran-ntn`: SM plugin ABI + KPM + RC SMs | 3 |
| 12 | `509b5d00c` | §4.2.1    | `ntn-sionna`: SionnaTransport abstraction | 5 |
| 13 | `ac7dc4e85` | §4.3.1    | `thz-ntn`: HITRAN-2024 LUT + generator | 5 |
| 14 | `b5de17e64` | §4.3.2    | `thz-ntn`: ITU-R P.618/676/838/681 wrappers | 5 |
| 15 | `c84c8de97` | §4.4.1    | `ntn-sagin`: OpenSky ADS-B importer | 4 |
| 16 | `501c5fd03` | §4.4.2    | `ntn-sagin`: AIS maritime importer | 4 |
| 17 | `f93cb0ee4` | §4.4.3    | `ntn-sagin`: HST per TR 38.901 §7.5 | 3 |
| 18 | `af3becda0` | §4.2.2    | `ntn-sionna`: Sionna RT 2.0.1 + MIMO | 2 |

All commits use `Muhammad Uzair <muhammaduzairr69@gmail.com>`. No co-authors.
Not pushed (per standing instruction).

## Per-suite test results

```
ntn-observability  PASS  10/10  (T9)
oran-ntn           PASS  33/33  (T4, T8, 4.1.1, 4.1.2, 4.1.3, 4.1.4, 4.1.10, 4.1.11)
ntn-fapi (new)     PASS   5/5   (T1)
ntn-sionna         PASS  10/10  (4.2.1, 4.2.2)
thz-ntn            PASS  22/22  (4.3.1, 4.3.2)
ntn-sagin          PASS  17/17  (4.4.1, 4.4.2, 4.4.3)
                  ----  ------------
TOTAL                    97/97
```

## §7 Q3 2026 must-finish coverage

| Roadmap §7 item | Effort | Status |
|---|---|---|
| T1 SCF FAPI typedefs | 2 wk | DONE |
| T8 KPM metric-ID alignment | 1 wk | DONE |
| T9 reproducibility manifest | 1 wk | DONE |
| 4.1.1 FlexRIC name parity | 1 wk | DONE |
| 4.1.2 KPM canonical CSV | 1 wk | DONE |
| 4.1.3 Style 3 HO | 1 wk | DONE |
| 4.1.4 OranNtnDataRepository SQLite | 2 wk | DONE |
| 4.2.1 pybind11 transport abstraction | 2 wk | DONE |
| 4.2.2 Sionna RT 2.0.1 + MIMO | 3 wk | DONE |
| 4.3.1 HITRAN-2024 LUT + generator | 3 wk | DONE |
| 4.3.2 ITU-Rpy wrapping | 2 wk | DONE |
| 4.4.1 OpenSky ADS-B importer | 2 wk | DONE |
| 4.4.2 AIS maritime importer | 2 wk | DONE |
| 4.4.3 HST mobility | 2 wk | DONE |

**Bonus** (closed inside Q3 from §3 / §4 ahead of §7 schedule):

| Roadmap | Effort | Status |
|---|---|---|
| T4 Service-Model plugin ABI | 2 wk | DONE |
| 4.1.10 conflict-manager taxonomy | 1 wk | DONE |
| 4.1.11 OSC A1 policy schemas | 2 wk | DONE |

## Tests covering Simulator::Run() time

Per the toolkit's `realistic-e2e-tests` standing instruction, every new
helper / class ships with at least one test that drives behaviour through
the ns-3 simulator over realistic time spans. Q3 2026 contributions:

| Test | Sim time | Asserts |
|---|---|---|
| `ntn-sionna` `SimulatorTimeMobilityTest` | 30 s | UDP transport queries every 1 s, monotonic FSPL increase with distance, zero fallbacks |
| `ntn-sionna` `MidRunServerKillTest` | 10 s | Mock server stops at t=5 s; both live and fallback phases observed |
| `ntn-sionna` `TransportMimoSimulatorTimeTest` | 30 s | 8×8 PlanarArray V-pol round-trip: 6 queries, `tx_ports=rx_ports=64`, plausible PL |
| `thz-ntn` `ThzNtnHitranSimulatorTimeTest` | 30 s | HAPS climbs 1→20 km; LUT-reported absorption monotonically non-increasing, ≥10× drop |
| `thz-ntn` `ThzNtnP618RainEventSimulatorTest` | 30 s | Synthetic rain event 0→50→0 mm/h at 25 GHz, peak attenuation > 5 dB, returns to 0 |
| `ntn-sagin` `OpenSkySimulatorTimeReplayTest` | 60 s | ADS-B replay, ENU anchor at t=0, monotonic east, altitude climbs by t=30 s |
| `ntn-sagin` `AisSimulatorTimeReplayTest` | 120 s | AIS replay at 12 knots ≈ 6.17 m/s, east-bound displacement > 100 m |
| `ntn-sagin` `HstSimulatorTimePassByTest` | 60 s | TR 38.901 HST-A 500 km/h pass-by, Doppler sign flip across closest approach |

## Cross-module reproducibility manifest

The C++-emitted manifest (`NtnReproManifest::WriteJson`) round-trips
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
align to, the same string the FlexRIC parity layer's
`kpm_v3::meas_type_t::meas_name` carries verbatim, and the same string
the T4 `OranNtnServiceModelKpm::Version()` returns.

## Module surface summary

`oran-ntn` (33 tests):
- Near-RT RIC + 13 xApps + Space RIC + ISL stack (pre-existing, regression-clean)
- WG3 canonical KPM IDs (`DRB.UEThpDl`, `DRB.UEThpUl`, `DRB.PdcpSduVolumeDL/UL`,
  `RRU.PrbAvailDl/Ul`, `RRU.PrbUsedDl/Ul`, `CARR.AverageSINR`, `L1M.RS-SINR.Mean`)
- FlexRIC verbatim types (`e2ap_msg_t`, `ric_request_id_t`,
  `ric_subscription_request_t`, `ric_indication_t`, `ric_control_request_t`,
  `kpm_v3::kpm_ind_msg_format_1_t`)
- E2SM-RC v1.03 Style 3 ControlAction (HandoverControl, CHO, DAPS-HO)
- NIST-pattern DataRepository (memory + SQLite backends)
- WG3 conflict taxonomy (DIRECT / INDIRECT / IMPLICIT)
- OSC A1 policy schema registry (20000, 20001, 20008, 20020 + 20050 NTN ext)
- Service-Model plugin ABI + KPM (RIC Function ID 147) + RC (RIC Function ID 3)

`ntn-fapi` (new, 5 tests):
- SCF FAPI 222.10.02 + 222.10.04 typedefs for 9 messages + 7 PDU types

`ntn-observability` (10 tests):
- Reproducibility manifest writer/reader (schema v1, ns3-ntn-toolkit/manifest)

`ntn-sionna` (10 tests):
- `SionnaTransport` abstract base + UDP, pybind11 (stub), None concrete impls
- Sionna RT 2.0.1 PlanarArray MIMO config per query (V/H/VH)

`thz-ntn` (22 tests):
- `HitranLut` + bundled `hitran2024-lut-subthz.csv` (HITRAN-2024 tag,
  41 × 31 cells, 100..500 GHz × 0..30 km)
- Native C++ ITU-R wrappers: `Itu838RainModel`, `Itu618LossModel`,
  `Itu676AbsorptionModel`, `Itu681LmsModel`

`ntn-sagin` (17 tests):
- `OpenSkyAdsbImporter` + `OpenSkyMobilityModel` (ADS-B aviation traces)
- `AisDanishImporter` + `AisMobilityModel` (Danish Maritime / Piraeus)
- `HstTraceGenerator` + `HstMobilityModel` (TR 38.901 §7.5, presets A/B/C)

## Risks deferred to v2.2 / v3.0 per §7

- §3 T2 ASN.1-PER encoders (placeholder TLV in T4 SMs; replaced under Q4 2026)
- §3 T3 SCTP E2 listener (FlexRIC round-trip, 6 wk effort, Q4 2026)
- §3 T5 SGP4 + ContactGraphScheduler (Q4 2026)
- §3 T6 Aerial Data Lake row schema (Q1 2027)
- §3 T7 gRPC/Triton inference contract (Q1 2027)
- §4.1.5–§4.1.9, §4.1.12 oran-ntn deep work (Q4 2026 + Q1 2027)
- §4.2.3–§4.2.12 ntn-sionna advanced (Q4 2026 + Q1 2027)
- §4.3.3–§4.3.11 thz-ntn advanced (Q4 2026 + Q1 2027)
- §4.4.4–§4.4.11 ntn-sagin contact-graph + ISL + RL (Q4 2026 + Q1 2027)

## v2.1 release candidate

This commit set is the v2.1 "realism baseline" release candidate per
roadmap §7. To tag:

```bash
git tag -a v2.1.0 -m 'v2.1 realism baseline (Q3 2026 sprint)'
```

The tag is recommended after a final regression run and Docker image
refresh.
