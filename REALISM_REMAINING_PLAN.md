# Realism roadmap — remaining pure-ns-3 backlog (post-audit, May 2026)

Derived from `Roadmap/Realism-Adoption-Roadmap-2026.md` after a full T1–T9 + §4.1–§4.4
implementation audit. As of **2026-05-28** the realism roadmap is **complete** for every
pure-ns-3 item; externally-blocked work (FlexRIC binary, live Triton/gRPC, libpython3
embed, Sionna GPU, Blender 4.x) is parked in the roadmap and out-of-scope here.

Status: ⏳ pending · 🟡 in-progress · ✅ done (built + run-verified)

## Tier A — integration ("modules compose as real measured sims")

- ✅ **A1 §4.2.8** — Composed `NtnSionnaCascadeChannel → NakagamiPropagationLossModel`
- ✅ **A2 §4.4.4+4.4.5** — SGP4-driven SAGIN routing
- ✅ **A3 §4.3.7** — thz-ntn × oran-ntn closed-loop full-stack example

## Tier B — standalone features

- ✅ **B1 §4.4.6** — Regenerative payload split (`SatRf` + `SatGnb` node aggregates).
- ✅ **B2 §4.1.10** — Conflict-manager taxonomy aligned with WG3 direct/indirect/implicit.
- ✅ **B3 §4.2.5** — Async batched Sionna query API (`SionnaBatchClient`).
- ✅ **B4 §4.2.11** — `apply_doppler()` C++ equivalent (`CirDopplerSynthesizer`).
- ✅ **B5 §4.4.8** — HAPS trajectory CSV ingest (`HapsTrajectoryTrace` + mobility model).
- ✅ **B6 §4.1.9** — CU/CU-UP/DU/RU split entity + F1 + OFH transports.
- ✅ **B7 §4.1.12** — Two-stage mMIMO precoder xApp (NN + 64-entry codebook).
- ✅ **B8 §T7** — gRPC/Triton AI-RAN inference contract + ns-3 client/server.

## Tier C — data-lake / tooling

- ✅ **C1 T6** — Data-Lake trace sink (covered by ns-3-ai-ntn shared-memory tables).
- ✅ **C2 §4.2.6** — Sionna precompute/replay backend (`SionnaReplayTransport` `.ntnbin`).
- ✅ **C3 §4.2.9** — OSM scene CLI tool (`tools/osm_to_sionna_scene.py`).
- ✅ **C4 §4.2.10** — LiDAR + AW3D30 DEM ingest (`tools/lidar_dem_ingest.py`).
- ✅ **C5 §4.3.8** — Mitsuba 3.x / 4.x unpin + CI probe (`tools/probe_sionna_env.py`).
- ✅ **C6 §4.4.11** — TR 38.821 + Starlink EU calibration corpus + harness.

## Externally-blocked (out of scope for this file)

- §3 T7 live edge — needs grpc++ + Triton server bin (proto + ns-3 client + mock runtime
  are shipped; flipping to live just wires `GrpcInferenceChannel` when the dep is built).
- §3 T9 — Reproducibility manifest example pinning (data-source IDs depend on which
  external corpora the user actually has).
- §4.2.10 live mode — needs `rasterio` + `laspy` Python deps; tool degrades gracefully.
- §4.3.8 Mitsuba 4.x with Blender → requires Blender 4.4 native build.

## Test count (2026-05-28)

| Module                       | Tests |
|------------------------------|-------|
| oran-ntn                     | 58    |
| ntn-sionna                   | 38    |
| thz-ntn                      | 38    |
| ntn-sagin                    | 32    |
| ntn-constellation            | 20    |
| ntn-observability            | 10    |
| oran-ntn-airan-inference     | 9     |
| ntn-fapi                     | 5     |
| ntn-sionna/tools (pytest)    | 14    |
| **Total**                    | **224** |

All suites green; per realism instructions every helper/class ships with
Simulator::Run() coverage where applicable, plus failure-mode tests.
