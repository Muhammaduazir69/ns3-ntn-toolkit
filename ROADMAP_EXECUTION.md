# Roadmap execution plan — ns3-ntn-toolkit v2.0

This is the canonical execution plan derived from `Roadmap/NTN-Toolkit-World-Best-Roadmap.md.pdf` (Apr 2026). Every future modification to the toolkit must reference a workstream ID below or extend this plan. Do not start a workstream out of order without recording the reason here.

**Source of truth:** the roadmap PDF is strategy; this file is execution.

---

## Status legend

| Symbol | Meaning |
|---|---|
| ✅ DONE | merged, validation gates passed, in main branch |
| 🟡 IN-PROGRESS | actively being implemented |
| ⏳ PENDING | next-up; design ready, code not started |
| 🚧 BLOCKED | external collaboration / hardware blocker |
| ⏸ PARKED | de-prioritised; not blocking anything downstream |

---

## Dependency graph

```
                        ┌──────────────────────────────┐
                        │  W1 ntn-constellation  ✅    │
                        │  (live TLE → SGP4 → SNS3)    │
                        └──────────────┬───────────────┘
                                       │
                ┌──────────────────────┼─────────────────────┐
                │                      │                     │
                ▼                      ▼                     ▼
       ┌────────────────┐   ┌──────────────────┐    ┌────────────────┐
       │ W2 ntn-rrc     │   │ W3 obs stack     │    │ W9 sionna-rt   │
       │ (3GPP Rel-18/19)│   │ Grafana+Influx   │    │ (GPU optional) │
       │   ⏳ NEXT      │   │ +NetSimulyzer ⏳ │    │   🚧 GPU       │
       └────────┬───────┘   └────────┬─────────┘    └────────────────┘
                │                    │
                └────────┬───────────┘
                         ▼
              ┌─────────────────────────┐
              │ W4 ai-ml extensions     │
              │ SB3 + PyG + ns3-gym     │
              │   ⏳                    │
              └────────┬────────────────┘
                       │
            ┌──────────┼──────────────┐
            ▼          ▼              ▼
       ┌─────────┐ ┌─────────┐  ┌──────────┐
       │ W5 SAGIN│ │ W6 slice│  │ W7 V2X   │
       │ HAPS+UAV│ │ orchestr│  │ SUMO     │
       │   ⏳    │ │   ⏳    │  │   ⏳     │
       └────┬────┘ └────┬────┘  └────┬─────┘
            │           │            │
            └───────────┴────────────┘
                        │
                        ▼
              ┌─────────────────────┐
              │ W8 FlexRIC E2 wire  │
              │   🚧 build env       │
              └──────────┬──────────┘
                         │
                         ▼
              ┌─────────────────────┐
              │ W10 digital twin    │
              │   ⏳                │
              └─────────────────────┘

         ⏸ Parked (external collab):
            W11 TeraSim — needs Prof. Jornet
            W12 SimRIS  — needs Koç University
```

---

## Pipeline integration

End-to-end data flow once the executable workstreams are in place:

```
[CelesTrak/Space-Track]   live TLEs
        │
        ▼
[W1 ntn-constellation]    SGP4 propagator + ISL graph + presets
        │
        ▼
[contrib/satellite (SNS3)] SatelliteSGP4MobilityModel reads positions/*.txt
        │
        ▼
[W2 ntn-rrc]              Timing Advance pre-comp (from ephemeris)
                          SIB19 + UE location reporting + NTN-DRX
        │
        ▼
[contrib/mmwave + lte]    NR PHY/MAC + RRC layer
        │
        ▼
[contrib/ntn-cho]         TTE-aware CHO algorithm (becomes xApp under W8)
        │
        ▼
[contrib/oran-ntn]        13 xApps, A1/E2 stubs (real wire under W8)
        │
        ▼ (control)        ▼ (data plane)
[W4 RL/GNN xApps]      [W6 slices: eMBB / URLLC / mMTC]
        │                  │
        └────────┬─────────┘
                 ▼
[W5 SAGIN: HAPS / UAV]    cross-layer routing
        │
        ▼
[W7 V2X-NTN bridge]       SUMO TraCI → ns-3 vehicular nodes
        │
        ▼
[W3 metrics]              ns-3 traces → InfluxDB → Grafana
                          ns-3 traces → NetSimulyzer 3D
[W1 czml]                 → CesiumJS live globe
        │
        ▼
[W10 digital twin]        cron loop closes the feedback to live TLE
```

Every stage must pass its own validation gate before the next stage's tests run.

---

## Cross-cutting validation strategy

Every workstream produces three test layers. CI does not merge a workstream branch until all three pass.

| Layer | What it asserts | Where it lives |
|---|---|---|
| **Unit** | one function, one assertion (TLE checksum, TA formula, CZML structure) | `tests/` (Python) or `test/` (C++ ns-3 test suite) |
| **Integration** | two-or-more workstreams cooperate (e.g. W1 → W2: TA pre-comp uses live Starlink ephemeris) | `tests/integration/` |
| **End-to-end smoke** | a one-line scenario that exercises every shipped workstream from input to dashboard | `tests/e2e/` + a top-level `make smoke` |

Validation gates per workstream are listed in each section below.

---

## W1 — ntn-constellation ✅ DONE

**Roadmap ref:** Phase 1.3 — Constellation Database + Live TLE Feeds
**Status:** shipped 2026-05-04 in commit `eb7d05487`, `contrib/ntn-constellation/`.
**Packages integrated:** `sgp4`, `skyfield`, `pyorbital`, `requests` (CelesTrak public + Space-Track credentialed), CesiumJS (CZML feed).
**Depends on:** —
**Blocks:** W2, W3 (sat positions for KPI dashboards), W4 (env state), W5 (mobility extension base), W9 (geometry), W10 (live feed loop).

**Validation gates passed:**

- [x] 10/10 unit tests green (`pytest -v`)
- [x] Live demo: 10299 Starlink TLEs fetched, 50-sat sample propagated, 118 ISL edges, SNS3 + 1.3 MB CZML emitted
- [x] Walker preset: 1156-sat Kuiper with 2278 +grid ISLs offline-generated
- [x] CLI smoke: `ntn-fetch iridium-next --out … --isl-walker --czml` exit 0

**Outstanding tightening (not blocking):**

- JPL Horizons cross-validation (assert SGP4 < 1 km position error vs Horizons over a 24-h pass)
- Wire CZML auto-load into the existing CesiumJS viewer at `contrib/ntn-cho/visualization/public/index.html`

These two are tightenings, not gates. Will be done as part of W10.

---

## W2 — ntn-rrc (3GPP Rel-18/19 NTN compliance) 🟡 IN-PROGRESS

**Roadmap ref:** Phase 1.2 — 3GPP Rel-18/19 NTN Protocol Compliance
**Status:** 🟡 IN-PROGRESS — TA pre-comp model + tests + LEO-pass example landed; SIB19 / NTN-DRX / UE-location-report / payload-mode hooks pending
**Packages integrated:** none new — pure ns-3 C++ extension built on top of `mmwave`/`lte` and consuming W1 ephemeris.
**Depends on:** W1
**Blocks:** W4, W5, W6, W8, W10 (every higher-layer workstream needs valid RRC under it).

### Goal

Add the 3GPP NTN-specific RRC procedures the audit found missing. Today the toolkit handles CHO at the application layer (`contrib/ntn-cho`); the actual TS 38.331 procedures (TA pre-comp, SIB19, GNSS-assisted timing, NTN DRX, UE location reporting, regenerative-vs-transparent payload) are unmodelled. This workstream fills that gap.

### Deliverables

```
contrib/ntn-rrc/
├── CMakeLists.txt
├── README.md
├── model/
│   ├── ntn-timing-advance.{h,cc}     # ephemeris-driven TA pre-compensation
│   ├── ntn-sib19.{h,cc}              # SIB19 (NTN assistance) structure + serialiser
│   ├── ntn-drx.{h,cc}                # NTN-extended DRX (long sleep / wake on pass)
│   ├── ntn-ue-location-report.{h,cc} # periodic GNSS location report
│   ├── ntn-payload-mode.{h,cc}       # regenerative vs transparent enum + behaviour
│   └── ntn-rrc-types.h               # shared types
├── helper/
│   └── ntn-rrc-helper.{h,cc}         # wires into LteEnbRrc / mmWaveEnbRrc
├── examples/
│   ├── ntn-rrc-leo-pass.cc           # 600 s LEO pass with TA pre-comp on/off
│   └── ntn-rrc-sib19-broadcast.cc    # SIB19 broadcast & UE consumption
└── test/
    └── ntn-rrc-test-suite.cc          # ns-3 test suite
```

### Test plan

- **Unit (C++ ns-3 tests):**
  - `NtnTimingAdvanceTest` — assert TA value matches closed-form `2·d/c` for known sat-ground geometry (within 1 µs).
  - `NtnSib19SerialiseTest` — round-trip serialise → parse → fields match.
  - `NtnDrxCycleTest` — sleep cycle = expected for given LEO orbital period.
  - `NtnPayloadModeTest` — regenerative gNB-on-sat path differs from transparent bent-pipe.
- **Integration (Python harness, runs after build):**
  - W1+W2: feed live Starlink TLE via W1 → enb computes per-second TA → assert TA derivative < 50 µs/s (LEO Doppler bound from TR 38.821).
- **End-to-end smoke:**
  - `ntn-rrc-leo-pass` example runs to completion, prints PASS, logs show `TA pre-comp ON` reduces RACH failures vs OFF.

### Validation gates

- [ ] `./test.py --suite=ntn-rrc` — 5/5 unit tests pass (full rebuild in progress)
- [x] LEO-pass example runs end-to-end and produces the expected TA "smile" curve: peak ~17 ms at far edges, ~3.668 ms at zenith (matches `2·550 km/c`), drift rate ~0 at zenith and ±50 µs/s at edges
- [ ] Integration: live-Starlink → TA pre-comp pipeline (deferred until W1+W2 hookup commit)
- [x] TA values match TR 38.821 reference table 6.1.1.1-1 within ±5% (validated in test `NtnTimingAdvance38821ReferenceTest`)

### Effort estimate
~5 days of focused work (4 core models + helper + tests + example).

### External blockers
None.

---

## W3 — Observability stack (Grafana + InfluxDB + NetSimulyzer) ⏳

**Roadmap ref:** Phase 2.4 (visualization piece) + Part 6 (Grafana, InfluxDB, NetSimulyzer)
**Status:** ⏳ pending; can run parallel with W2.
**Packages integrated:** Grafana, InfluxDB (line-protocol push), NetSimulyzer (NIST `JsonHandler` trace exporter).
**Depends on:** W1 (sat position trace source).
**Blocks:** W4 (RL needs reward signals from metrics), W6 (per-slice KPI), W10 (live dashboard).

### Goal

Stream every existing ns-3 trace source into a time-series database (InfluxDB) with pre-built Grafana dashboards, plus emit NetSimulyzer JSON for 3D playback. This makes regressions visible and gives RL agents (W4) a structured reward channel.

### Deliverables

```
contrib/ntn-observability/
├── CMakeLists.txt
├── README.md
├── model/
│   ├── ntn-influx-sink.{h,cc}        # ns-3 trace → InfluxDB line protocol over UDP
│   ├── ntn-netsimulyzer-helper.{h,cc}# wraps NIST netsimulyzer JsonHandler for NTN nodes
│   └── ntn-metric-schema.h            # canonical KPI names (RSRP, SINR, TA, HOcount, ISL load)
├── helper/
│   └── ntn-observability-helper.{h,cc}
├── dashboards/                        # Grafana JSON exports (committed)
│   ├── ntn-overview.json
│   ├── ntn-handover.json
│   ├── ntn-radio.json
│   └── ntn-isl.json
├── docker/
│   ├── docker-compose.yml             # influxdb 2.x + grafana 10 + provisioning
│   └── grafana-provisioning/
└── examples/
    └── ntn-observability-demo.cc      # 60 s scenario emitting full KPI set
```

### Test plan

- **Unit:** `NtnInfluxSinkTest` — line-protocol output parses correctly; `NtnMetricSchemaTest` — KPI names match Grafana dashboard JSON.
- **Integration:** docker compose up → run `ntn-observability-demo` → assert InfluxDB has > 1000 points and dashboards render without "No data" panels.
- **End-to-end smoke:** add metric emission to W1 demo + W2 example; both surface as time-series in Grafana.

### Validation gates

- [ ] `./ns3 test --suite=ntn-observability` pass
- [ ] `docker compose up` brings up InfluxDB+Grafana with provisioned dashboards
- [ ] All 4 dashboards render real data from `ntn-observability-demo`
- [ ] Per-second push latency < 100 ms

### Effort estimate
~3 days.

### External blockers
None. NetSimulyzer is a public NIST package; Grafana/InfluxDB are public Docker images.

---

## W4 — AI/ML extensions (SB3 + PyTorch Geometric + ns3-gym) ⏳

**Roadmap ref:** Phase 2.4
**Status:** ⏳ pending
**Packages integrated:** `stable-baselines3`, `torch_geometric`, canonical `ns3-gym` (in addition to the existing `ns3-ai` shared-mem bridge), `pyorbital` for env state.
**Depends on:** W1 (state inputs), W3 (reward signal from KPIs).
**Blocks:** W6 (RL-driven slice orchestration), W8 (RL xApps over real E2), W10 (predictive prefetch).

### Goal

Extend the existing `contrib/ns3-ai-ntn` fork with: (a) a Stable-Baselines3 wrapper so users can run `PPO("MlpPolicy", env)` against ns-3 NTN environments out-of-the-box; (b) PyTorch Geometric models for constellation-graph learning (GNN on satellite topology); (c) the canonical `ns3-gym` interface alongside the existing custom Gymnasium 1.0 wrapper, so external researchers' existing scripts work without modification.

### Deliverables

```
contrib/ns3-ai-ntn/python_utils/
├── ns3_ai_ntn/
│   ├── envs/
│   │   ├── handover_env.py           # Gymnasium env wrapping ntn-cho CHO
│   │   ├── beam_mgmt_env.py          # multi-beam selection
│   │   ├── slice_env.py              # eMBB/URLLC/mMTC slice orchestration (W6 stub)
│   │   └── power_ctrl_env.py         # uplink power control over LEO
│   ├── sb3/
│   │   ├── __init__.py
│   │   └── train_ppo_handover.py     # canonical SB3 training script
│   ├── gnn/
│   │   ├── __init__.py
│   │   ├── constellation_graph.py    # build PyG Data from W1 ISL graph
│   │   └── gat_topology.py           # GAT for next-hop prediction
│   ├── marl/
│   │   ├── mappo_handover.py         # multi-UE MAPPO baseline
│   │   └── masac_beam.py
│   └── ns3gym_compat.py               # adaptor to canonical ns3-gym API
├── pyproject.toml                     # adds sb3, torch-geometric, ns3-gym
└── tests/
    ├── test_envs.py                   # gym checker passes for all 4 envs
    ├── test_sb3.py                    # PPO trains 1k steps without error
    └── test_gnn.py                    # forward pass on Starlink graph
```

### Test plan

- **Unit:** Gymnasium `check_env(env)` passes for every env; SB3 `learn(total_timesteps=1024)` exit 0; PyG forward pass yields correct shape on a 50-node ISL graph.
- **Integration:** train PPO 10k steps on `handover_env`, assert mean episode reward > random baseline.
- **End-to-end smoke:** `train_ppo_handover.py` runs end-to-end with InfluxDB metric streaming.

### Validation gates

- [ ] All 4 envs pass `gymnasium.utils.env_checker.check_env`
- [ ] `pytest contrib/ns3-ai-ntn/python_utils/tests/` green
- [ ] PPO baseline beats random on `handover_env` (reward > random + 1 σ)
- [ ] GNN converges to handover prediction accuracy ≥ 70% on Starlink subset

### Effort estimate
~5 days.

### External blockers
None.

---

## W5 — SAGIN (HAPS + UAV + A2G) ⏳

**Roadmap ref:** Phase 3.2
**Status:** ⏳ pending
**Packages integrated:** none new; relies on W1 mobility infrastructure.
**Depends on:** W1, W2 (RRC for cross-layer handover).
**Blocks:** W7 (V2X-NTN), supports the strategic SAGIN rebrand discussion.

### Goal

Extend the toolkit from "satellite-only" to "Space-Air-Ground" (SAGIN) by adding HAPS (20 km station-keeping), UAV mobility (random-waypoint, patrol, search patterns), and 3GPP TR 36.777 A2G channel models.

### Deliverables

```
contrib/ntn-sagin/
├── CMakeLists.txt
├── README.md
├── model/
│   ├── haps-mobility-model.{h,cc}         # 20 km altitude station-keeping
│   ├── uav-mobility-models.{h,cc}         # waypoint / patrol / search
│   ├── a2g-channel-tr36777.{h,cc}         # 3GPP TR 36.777 LOS/NLOS
│   ├── multi-layer-router.{h,cc}          # ground → UAV → HAPS → LEO routing
│   └── aeronautical-scenario.{h,cc}       # passenger connectivity
├── helper/
│   └── sagin-helper.{h,cc}
├── examples/
│   ├── sagin-haps-leo-relay.cc
│   ├── sagin-uav-swarm.cc
│   └── sagin-aeronautical.cc
└── test/
    └── ntn-sagin-test-suite.cc
```

### Test plan

- **Unit:** HAPS altitude held within ±50 m for 1 h; UAV patrol-pattern returns to start; A2G PL matches TR 36.777 reference within ±2 dB.
- **Integration:** ground-UE → UAV-relay → HAPS → LEO end-to-end ICMP echo round-trip.
- **End-to-end smoke:** `sagin-haps-leo-relay` example completes; metrics in Grafana.

### Validation gates

- [ ] Test suite green
- [ ] PL spot-checks within TR 36.777 ±2 dB
- [ ] Multi-layer routing converges in <5 s

### Effort estimate
~5 days.

### External blockers
None.

---

## W6 — Network slicing for NTN ⏳

**Roadmap ref:** Phase 3.4
**Status:** ⏳ pending
**Packages integrated:** none new.
**Depends on:** W2 (RRC for slice association SST/SD), W4 (RL slice orchestrator), W3 (per-slice KPIs).
**Blocks:** —

### Goal

Implement 3GPP-compliant network slicing with eMBB / URLLC / mMTC slices spread across LEO and GEO, plus a slice orchestrator xApp.

### Deliverables

```
contrib/ntn-slice/
├── model/
│   ├── ntn-slice-types.h                  # SST/SD definitions
│   ├── ntn-slice-selector.{h,cc}          # per-flow slice association
│   ├── slice-orchestrator-xapp.{h,cc}     # plugs into oran-ntn
│   └── slice-isolation-monitor.{h,cc}
├── helper/
│   └── ntn-slice-helper.{h,cc}
├── examples/
│   └── ntn-three-slice-leo-geo.cc
└── test/
```

### Validation gates

- [ ] eMBB/URLLC/mMTC slices co-exist on same satellite without isolation breach
- [ ] URLLC E2E latency < 50 ms via GEO-mode-skip routing
- [ ] Per-slice KPI panels in Grafana populated

### Effort estimate
~4 days.

---

## W7 — V2X-NTN bridge (SUMO TraCI) ⏳

**Roadmap ref:** Phase 3.3
**Status:** ⏳ pending
**Packages integrated:** SUMO (TraCI), CelesTrak feed (via W1).
**Note:** Veins is OMNeT++-only and **cannot** be imported into ns-3. We build a SUMO TraCI → ns-3 bridge that mirrors what Veins offers in spirit.
**Depends on:** W1, W2, W5.
**Blocks:** —

### Goal

Vehicular nodes driven by SUMO consume LEO/GEO connectivity for rural / oceanic / emergency scenarios.

### Deliverables

```
contrib/ntn-v2x/
├── model/
│   ├── sumo-traci-bridge.{h,cc}           # TraCI client → ns-3 mobility
│   ├── v2x-leo-direct.{h,cc}              # vehicle-to-satellite direct
│   ├── v2x-leo-relay.{h,cc}               # vehicle-to-vehicle via LEO
│   └── maritime-scenario.{h,cc}
├── examples/
│   └── ntn-v2x-rural-highway.cc
└── test/
```

### Validation gates

- [ ] TraCI bridge stays in sync with SUMO clock (jitter < 100 ms)
- [ ] 100-vehicle highway scenario over LEO completes 5-min sim in CI
- [ ] Metrics in Grafana

### Effort estimate
~4 days.

---

## W8 — FlexRIC E2 real-wire integration 🚧

**Roadmap ref:** Phase 1.1
**Status:** 🚧 needs FlexRIC build environment + ASN.1 toolchain.
**Packages integrated:** FlexRIC (EURECOM), real ASN.1-compiled E2AP v1.01 / E2SM-KPM v3 / E2SM-RC v1.03 over SCTP.
**Depends on:** W1, W2, W4 (xApps).
**Blocks:** —

### Goal

Replace `oran-ntn`'s in-memory E2 stubs with a real FlexRIC Near-RT RIC. The CHO algorithm graduates from a built-in to an externally-running xApp speaking real E2.

### Deliverables

```
contrib/oran-ntn/flexric-bridge/
├── flexric/                                 # git submodule of EURECOM FlexRIC
├── e2-agent-ns3/                            # E2 Agent in ns-3 gNB-DU/CU
│   ├── e2-agent.{h,cc}
│   ├── e2sm-kpm-ntn.{h,cc}                  # NTN-specific KPM (elevation, Doppler, beam ID, TTE)
│   └── e2sm-rc-ntn.{h,cc}                   # NTN-specific RC actions (CHO trigger)
├── xapps/
│   ├── cho-xapp/                            # the existing TTE-aware CHO ported to xApp
│   ├── beam-mgmt-xapp/
│   └── slice-orch-xapp/
└── docs/
    └── BUILD_FLEXRIC.md                     # how to build the toolchain
```

### Test plan

- **Unit:** ASN.1 codec round-trip; SCTP bringup; subscription/indication state machine.
- **Integration:** xApp registers with Near-RT RIC, subscribes to KPM, receives 100 IndicationMessages over a LEO pass.
- **End-to-end smoke:** CHO xApp triggers handover via E2SM-RC ControlRequest; UE sees handover; metrics flow.

### Validation gates

- [ ] FlexRIC builds in a Docker image we ship under `flexric-bridge/docker/`
- [ ] xApp CHO produces handovers indistinguishable in result from built-in CHO
- [ ] Real wire trace captured with `wireshark -d sctp.port==36421,e2ap`

### Effort estimate
~15 days.

### External blockers
- FlexRIC build env (ASN.1 toolchain, Docker)
- Possibly a separate VM for the RIC process

---

## W9 — NVIDIA Sionna RT (NTN extension) 🚧

**Roadmap ref:** Phase 2.1
**Status:** 🚧 needs NVIDIA GPU.
**Packages integrated:** Sionna RT, Mitsuba 3.
**Depends on:** W1.
**Blocks:** —

### Goal

GPU-accelerated ray-traced channel for satellite-to-ground links. The current TR 38.811 implementation remains the simulation default; Sionna becomes an *opt-in* high-fidelity channel.

### Deliverables

```
contrib/ntn-sionna/
├── README.md
├── bridge/
│   ├── sionna-server.py                    # Python process running Sionna RT
│   └── ns3-sionna-channel.{h,cc}           # ns-3 channel model talking to it via UDP
├── examples/
│   └── leo-pass-sionna-vs-tr38811.cc
└── test/
```

### Validation gates

- [ ] Sionna server starts on GPU host
- [ ] ns-3 channel queries Sionna and gets responses with < 50 ms RTT
- [ ] Path loss within ±3 dB of TR 38.811 reference under matched scenario

### Effort estimate
~8 days (after GPU available).

### External blockers
- NVIDIA GPU + CUDA env
- Sionna 0.18+ install

---

## W10 — Digital Twin Mode ⏳

**Roadmap ref:** Phase 3.1
**Status:** ⏳ pending; mostly polish over W1 + W3 + the existing CesiumJS viewer.
**Packages integrated:** systemd timer / cron, FastAPI (for the prediction API).
**Depends on:** W1, W2, W3.
**Blocks:** —

### Goal

A live mode where the simulator mirrors a real constellation in near-real time: cron pulls TLEs, propagator regenerates state, CesiumJS shows current positions, REST API answers "predict next handover for UE X over the next 10 min".

### Deliverables

```
contrib/ntn-digital-twin/
├── twin_loop.py                          # cron entry; refresh TLE + push to InfluxDB + emit CZML
├── api/
│   ├── server.py                         # FastAPI: /predict/handover, /constellation/state
│   └── schemas.py
├── systemd/
│   └── ntn-twin.service
├── viewer-patches/
│   └── index.html.patch                  # adds "Live" toggle to CesiumJS viewer
└── tests/
    └── test_twin_loop.py
```

### Validation gates

- [ ] 24-h continuous loop without crash; position error vs current TLE < 1 km
- [ ] API answers `/predict/handover` in <500 ms
- [ ] CesiumJS "Live" toggle works against running loop

### Effort estimate
~3 days.

---

## W11 — TeraSim (Northeastern) ⏸ PARKED

**Roadmap ref:** Phase 2.2
**Status:** ⏸ blocked on Prof. Jornet collaboration.
**Reason for parking:** the existing `contrib/thz-ntn` already provides custom THz physics (HITRAN-2020, ITU-R P.676/618/838, UM-MIMO, RIS, ISAC). TeraSim adds value only if integrated *with* the Northeastern team. User to reply to Jornet first; this workstream un-parks once we have a yes.

---

## W12 — SimRIS (Koç University) ⏸ PARKED

**Roadmap ref:** Phase 2.3 (RIS-NTN module)
**Status:** ⏸ blocked on Koç University (Prof. Basar) collaboration / licensing.
**Reason for parking:** `contrib/thz-ntn` already implements RIS quantisation sweep + ISAC CRB. SimRIS would supplement, not replace.

---

## Implementation order (sequential plan)

| Order | ID | Workstream | Reason for position |
|---|---|---|---|
| 1 | W1 | ntn-constellation | foundation; everything reads from here ✅ |
| 2 | W2 | ntn-rrc | every higher layer needs valid RRC |
| 3 | W3 | observability | observability before complexity (parallel-able with W2) |
| 4 | W4 | AI/ML extensions | enables RL xApps for W6, W8 |
| 5 | W5 | SAGIN | strategic positioning + extends mobility cleanly |
| 6 | W6 | network slicing | needs W2+W4; orthogonal to W5 |
| 7 | W7 | V2X-NTN | needs W2+W5 |
| 8 | W8 | FlexRIC E2 wire | heavyweight; better with W4 ML xApps already proven |
| 9 | W10 | digital twin | polish on W1+W3 |
| 10 | W9 | Sionna RT | GPU-gated; opt-in channel |
| — | W11 | TeraSim | parked |
| — | W12 | SimRIS | parked |

Total estimated effort for executable workstreams (W2–W10, excluding GPU-gated W9): ~46 days of focused engineering. W9 adds ~8 days when GPU is available.

---

## Working agreement

These rules govern *how* this plan is executed:

1. **Never start a workstream out of order** without recording the reason at the top of this file. Out-of-order work creates integration debt.
2. **Each workstream must pass all three test layers** (unit + integration + e2e smoke) before its branch merges. No "we'll fix it later."
3. **Every commit references the workstream ID** in the message: `W2: add Timing Advance pre-comp model`.
4. **Update the status badge** at the top of the workstream section the moment status changes (⏳ → 🟡 → ✅).
5. **External-blocker workstreams** (W8, W9, W11, W12) stay 🚧/⏸ until the blocker resolves; do not invent stubs that won't work without the real package.
6. **Do not auto-rebrand** the toolkit (`SkyNet-Sim` / `SAGIN-Sim`) or migrate to a GitHub Organization without explicit user approval — those are one-way doors recorded in `project_roadmap_v2.md` memory.
7. **Both remotes always.** Every feature branch pushes to GitHub `origin` and GitLab `gitlab` once tests pass.

---

## Quick reference

- Strategy: `Roadmap/NTN-Toolkit-World-Best-Roadmap.md.pdf`
- Execution (this file): `ROADMAP_EXECUTION.md`
- Auto-memory: `~/.claude/projects/-home-uzair-6g-ntn-ns3/memory/project_roadmap_v2.md`
- W1 module: `contrib/ntn-constellation/`
- Mirrors: <https://github.com/Muhammaduazir69/ns3-ntn-toolkit> · <https://gitlab.com/ha5050/ns3-ntn-toolkit>

Last updated: 2026-05-04
