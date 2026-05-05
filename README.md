<h1 align="center">ns3-ntn-toolkit</h1>

<p align="center"><strong>A Pre-Integrated ns-3.43 Simulation Platform for 6G Non-Terrestrial Network Research — Clone, Build, Run.</strong></p>

<p align="center">
  <em>Mirrors:&nbsp;</em>
  <a href="https://github.com/Muhammaduazir69/ns3-ntn-toolkit">GitHub</a>
  &nbsp;·&nbsp;
  <a href="https://gitlab.com/ha5050/ns3-ntn-toolkit">GitLab</a>
</p>

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0-green.svg"/></a>
  <img src="https://img.shields.io/badge/3GPP-Rel--17%2F18%2F19%20NTN-orange.svg"/>
  <img src="https://img.shields.io/badge/O--RAN-WG3%20%2F%20WG2%20%2B%20FlexRIC-purple.svg"/>
  <img src="https://img.shields.io/badge/THz-100%20GHz%20%E2%80%93%201%20THz-success.svg"/>
  <img src="https://img.shields.io/badge/Sionna%20RT-2.0%20GPU-red.svg"/>
  <img src="https://img.shields.io/badge/RL-Gymnasium%201.0-yellow.svg"/>
  <img src="https://img.shields.io/badge/modules-13%20custom-informational.svg"/>
</p>

<p align="center">
  <img src="docs/ns3_ntn_toolkit_architecture.png" alt="ns3-ntn-toolkit architecture" width="950"/>
</p>

---

## Why this toolkit

Open research on 6G non-terrestrial networks is held back by **tool fragmentation**: orbit propagation, 3GPP NTN protocol procedures, 5G/NR PHY/MAC, Open-RAN E2/A1 wires, network slicing, V2X mobility, ray-traced channels, and reinforcement-learning bridges typically live in eight or nine packages that rarely build together on a recent ns-3 release. `ns3-ntn-toolkit` consolidates them into a single buildable distribution. A single `./ns3 build` yields an end-to-end NTN simulator in roughly fifteen minutes — turning what would be a multi-week integration effort into an hour-long onboarding task — and every contributed module ships with a numerical verification harness (closed-form ground truth, parameterised tests, long-run audits) so a reviewer can re-run and trust the published numbers.

## At a glance

| Capability | Numbers |
|---|---|
| ns-3 base | **3.43** (patched LTE for dual connectivity) |
| Custom modules contributed by this work | **13** (see *Bundled modules* below) |
| Combined unit + integration tests | **80+** across 13 repos, all passing |
| 3GPP NTN procedures implemented | TS 38.213 TA · TS 38.331 SIB19 + UE Location Report · TS 38.321 NTN-DRX · TR 36.777 A2G |
| 3GPP slicing | TS 23.501 + TS 22.261 default profiles, eMBB / URLLC / mMTC / V2X |
| O-RAN xApps shipped | 16 (13 in `oran-ntn` + 3 NTN-aware in `flexric-bridge`) |
| O-RAN E2 wire | live FlexRIC SCTP/E2AP via Docker; CI-friendly TCP/JSON stub for the same xApp logic |
| Reinforcement-learning bridge | Gymnasium 1.0 over patched ns3-ai (Py 3.13 + NumPy 2 ready); SB3 PPO + PyG GAT |
| Channel models | TR 38.811 closed-form (default) · NVIDIA Sionna RT GPU ray-tracing (opt-in) |
| Vehicular | SUMO TraCI v20+ live + FCD-trace replay |
| Constellation feeds | live CelesTrak + Space-Track + 5 named presets (Starlink / OneWeb / Kuiper / Telesat / Iridium) |
| Observability | InfluxDB 2.7 (UDP + file) + Grafana 10.4 (4 dashboards) + NetSimulyzer 1.0 JSON |
| Live digital twin | FastAPI prediction API at p99 ≤ 30 ms; CesiumJS Live mode |

## Bundled modules

| # | Module | Repo | Purpose |
|---:|---|---|---|
| 1 | `ntn-constellation` | [ntn-constellation](https://github.com/Muhammaduazir69/ntn-constellation) | Live TLE feeds, SGP4/SDP4 propagation, ISL topology, SNS3 + CesiumJS exporters |
| 2 | `ntn-rrc` | [ntn-rrc](https://github.com/Muhammaduazir69/ntn-rrc) | TS 38.213 TA pre-comp, TS 38.331 SIB19 + UE Location Report, TS 38.321 NTN-DRX |
| 3 | `ntn-observability` | [ntn-observability](https://github.com/Muhammaduazir69/ntn-observability) | InfluxDB sinks, NetSimulyzer JSON, Grafana stack with 4 dashboards |
| 4 | `ns3-ai` (fork) | [ns3-ai](https://github.com/Muhammaduazir69/ns3-ai) | Modernised shared-memory bridge — ns-3.43 + Py 3.13 + NumPy 2 + Gymnasium 1.0 + SB3 + PyG |
| 5 | `ntn-sagin` | [ntn-sagin](https://github.com/Muhammaduazir69/ntn-sagin) | HAPS / UAV mobility + TR 36.777 A2G + multi-layer Ground→UAV→HAPS→LEO router |
| 6 | `ntn-slice` | [ntn-slice](https://github.com/Muhammaduazir69/ntn-slice) | TS 23.501 slicing (eMBB/URLLC/mMTC/V2X) + isolation monitor + GEO mode-skip |
| 7 | `ntn-v2x` | [ntn-v2x](https://github.com/Muhammaduazir69/ntn-v2x) | SUMO TraCI bridge + V2X-LEO direct/relay channels + maritime scenario |
| 8 | `flexric-bridge` | [flexric-bridge](https://github.com/Muhammaduazir69/flexric-bridge) | FlexRIC E2 real-wire integration: NTN E2 agent + 3 xApps + Docker stack |
| 9 | `ntn-sionna` | [ntn-sionna](https://github.com/Muhammaduazir69/ntn-sionna) | NVIDIA Sionna RT bridge: GPU-accelerated ray-traced sat-to-ground channel |
| 10 | `ntn-digital-twin` | [ntn-digital-twin](https://github.com/Muhammaduazir69/ntn-digital-twin) | Live TLE refresher + FastAPI predict-handover + CesiumJS Live mode |
| 11 | `ntn-cho` | [ntn-cho-framework](https://github.com/Muhammaduazir69/ntn-cho-framework) | TTE-aware 3GPP Rel-17 conditional handover + 7-class realistic UE mobility |
| 12 | `oran-ntn` | [oran-ntn](https://github.com/Muhammaduazir69/oran-ntn) | Space O-RAN: 13 xApps, dual Near-RT/Space RIC, conflict mgr, 4 FL aggregators |
| 13 | `thz-ntn` | [ns3-thz-ntn](https://github.com/Muhammaduazir69/ns3-thz-ntn) | 100 GHz – 1 THz physics: HITRAN-2020, UM-MIMO ≤ 128×128, RIS, ISAC, EKF beam tracking |

Plus the upstream packages this distribution patches and integrates:

| Module | Source | Role |
|---|---|---|
| `satellite` (SNS3) | [SNS3/sns3-satellite](https://github.com/sns3/sns3-satellite) | SGP4 propagator + TR 38.811 NTN channel + Loo / Markov fading |
| `mmwave` | [NYU/CTTC](https://github.com/nyuwireless-unipd/ns3-mmwave) | 5G NR PHY/MAC + dual-connectivity LTE patches |
| `ns-3.43` | [nsnam/ns-3-dev](https://gitlab.com/nsnam/ns-3-dev) | core simulation kernel |

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                            ns3-ntn-toolkit                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│   ┌─────────────────┐   ┌──────────────────┐   ┌────────────────────┐    │
│   │ ntn-constellation│  │   ntn-rrc        │   │ ntn-observability  │    │
│   │  CelesTrak/SGP4  ├─►│ TA · SIB19 · DRX ├──►│  InfluxDB · Grafana │    │
│   │  presets · ISLs  │  │ UE Loc Report    │   │  NetSimulyzer JSON  │    │
│   └────────┬─────────┘  └─────────┬────────┘   └──────────┬─────────┘    │
│            │                       │                       │              │
│   ┌────────▼─────────┐   ┌─────────▼────────┐   ┌─────────▼─────────┐    │
│   │   ntn-sagin      │   │   ntn-slice      │   │   ntn-v2x          │    │
│   │ HAPS · UAV · A2G │   │ eMBB/URLLC/mMTC  │   │ SUMO TraCI · LEO   │    │
│   │ multi-layer rtr  │   │ GEO mode-skip    │   │ direct + relay     │    │
│   └────────┬─────────┘   └─────────┬────────┘   └─────────┬──────────┘    │
│            │                       │                       │              │
│   ┌────────▼─────────┐   ┌─────────▼────────┐   ┌─────────▼─────────┐    │
│   │ flexric-bridge   │   │  ntn-sionna      │   │ ntn-digital-twin   │    │
│   │ E2/SCTP · 3 xApps│   │ Sionna RT (GPU)  │   │ FastAPI predict    │    │
│   │ Docker stack     │   │ ±3 dB matched-PL │   │ CesiumJS Live      │    │
│   └────────┬─────────┘   └─────────┬────────┘   └─────────┬──────────┘    │
│            │                       │                       │              │
│   ┌────────▼─────────┐   ┌─────────▼────────┐   ┌─────────▼─────────┐    │
│   │   ntn-cho        │   │   oran-ntn       │   │   thz-ntn          │    │
│   │ TTE-aware Rel-17 │   │ 13 xApps · dual  │   │ 100 GHz – 1 THz    │    │
│   │ + 7-class UE mob │   │ RIC · FL · A1    │   │ UM-MIMO · RIS · ISAC│   │
│   └──────────────────┘   └──────────────────┘   └────────────────────┘    │
│                                                                          │
│                ┌────────────────────────────────────────┐                │
│                │   ns3-ai (Gymnasium 1.0 / SB3 / PyG)   │                │
│                └────────────────────────────────────────┘                │
│                                   │                                      │
│         ┌─────────────────────────▼─────────────────────────┐           │
│         │   ns-3.43  +  SNS3 satellite  +  mmwave (5G NR)    │           │
│         └─────────────────────────────────────────────────────┘           │
└─────────────────────────────────────────────────────────────────────────┘
```

## Live demos

### Realistic NTN UE mobility — 14 UEs across all 7 TR 38.811 classes

<p align="center">
  <img src="docs/ntn_realistic_mobility.gif" alt="Realistic NTN UE mobility" width="900"/>
</p>

### Per-class TTE-aware handover behaviour over the same scenario

<p align="center">
  <img src="docs/ntn_handover_realistic.gif" alt="Per-class HO behaviour" width="900"/>
</p>

### O-RAN NTN — 66-satellite Walker-Star with Space RICs

<p align="center">
  <img src="docs/oran_ntn_constellation_ric.gif" alt="O-RAN constellation + Space RIC" width="850"/>
</p>

### Per-module animated demos

Every contributed module ships its own animated demo inside its repo —
follow the [bundled-modules](#bundled-modules) links above to see each one in
context (e.g. `ntn-sionna/docs/ntn_sionna_demo.gif`).

### Module-output snapshots

| O-RAN xApps showcase | NTN-CHO algorithm comparison |
|---|---|
| <img src="docs/oran_ntn_showcase.png" width="430"/> | <img src="docs/ntn_cho_showcase.png" width="430"/> |

## Install & run

See [**INSTALL.md**](INSTALL.md) for the full step-by-step guide (system requirements, SNS3 satellite clone, build flags, Docker stacks, GPU prerequisites, troubleshooting).

Quick taste:

```bash
# 1. Clone the toolkit
git clone https://github.com/Muhammaduazir69/ns3-ntn-toolkit.git
cd ns3-ntn-toolkit

# 2. Pull SNS3 satellite (REQUIRED — not bundled, ~3.7 GB with TLE data)
cd contrib/ && git clone https://github.com/sns3/sns3-satellite.git satellite && cd ..

# 3. Configure & build
./ns3 configure --enable-examples --enable-tests
./ns3 build

# 4. Run the integrated multi-module example
./ns3 run "ntn-tn-integrated-analysis --algorithm=tte-aware --simTime=10 --numTnUes=4"

# 5. (Optional) Bring up the observability stack
cd contrib/ntn-observability/docker && docker compose up -d
# Grafana now available at http://localhost:3000 (admin/admin)
```

## Reference scenarios shipped

| Scenario | Module | Wallclock | Outputs |
|---|---|---|---|
| `ntn-realistic-mobility-demo` | ntn-cho | ~5 s | 14-UE 600-s trajectory CSV |
| `ntn-cho-full-constellation` | ntn-cho | ~30 s | per-seed CHO KPM CSVs |
| `oran-ntn-full-scenario` | oran-ntn | ~60 s | 5-xApp 600-s action / KPM logs |
| `thz-ntn-demo` (9 sub-scenarios) | thz-ntn | ~30 s | atm-windows / link budget / ISAC CSVs |
| `ntn-rrc-leo-pass` | ntn-rrc | ~5 s | classic NTN "smile" TA curve |
| `ntn-rrc-full-stack` | ntn-rrc | ~30 s | TA + SIB19 + UE-report + DRX CSVs |
| `ntn-observability-demo` | ntn-observability | ~10 s | InfluxDB LP + NetSimulyzer JSON |
| `sagin-haps-leo-relay` | ntn-sagin | ~60 s | 1 h 4-layer routing CSV |
| `sagin-uav-swarm` | ntn-sagin | ~30 s | 8-UAV TR 36.777 PL spot-check |
| `ntn-three-slice-leo-geo` | ntn-slice | ~60 s | 3-slice 1 h KPI CSV (URLLC mode-skip) |
| `ntn-v2x-rural-highway` | ntn-v2x | ~30 s | 100-vehicle 5-min trace |
| `leo-pass-sionna-vs-tr38811` | ntn-sionna | ~5 s* | 30-step Sionna vs TR 38.811 PL log |
| `ntn-tn-integrated-analysis` | toolkit | ~20 s | TN+NTN integrated traces |

\* needs `python3 contrib/ntn-sionna/bridge/sionna-server.py --port 8765` running on a CUDA host.

## Verification highlights

Every contributed module ships with a numerical verification harness. Headline numbers, all measured locally:

| Module | Verification result |
|---|---|
| `ntn-constellation` | 24 h propagation 0 NaN · period 96.00 min · vs Skyfield max 23.5 µs over 1800 s · drift 0.006 µs/s |
| `ntn-rrc` | 16 / 16 unit tests · TA drift saturates at +50.6 µs/s = 2·v/c (4-sig-fig) · vs Skyfield max 12.8 µs |
| `ntn-observability` | 5 / 5 unit tests · 1800 s scenario: every cadence exact (1800·1, 11250·1, 360·1) · TA fidelity ≤ 1 µs |
| `ns3-ai` (fork) | 15 / 15 tests · PPO 50 k × 3 seeds beats random by 4–7 σ · GAT 80-sat × 5 seeds × 1000 epochs **95.2 %** mean |
| `ntn-sagin` | 6 / 6 unit tests · TR 36.777 RMa-AV LOS spot-check 75.43 dB matches spec to **0.02 dB** |
| `ntn-slice` | 7 / 7 unit tests · URLLC p99 = **47.02 ms** (mode-skip ON) vs 295.52 ms (forced GEO, 6.3×) |
| `ntn-v2x` | 5 / 5 unit tests · 100 vehicles × 5 min, 30 100 samples, jitter **0 ms** · V2X-LEO direct PL within 0.1 dB |
| `flexric-bridge` | 7 / 7 tests · 30 k IND/s loopback, 0 % loss · CHO xApp **bit-identical** to in-memory oracle |
| `ntn-sionna` | 3 C++ + 6 Py = 9 tests · 30-step LEO pass max \|Δ\| = **0.002 dB** vs TR 38.811 · steady-state RTT ~9 ms |
| `ntn-digital-twin` | 6 / 6 tests · 144 / 144 iters, 0 errors · `/predict/handover` p99 = **29.9 ms** (16× under 500 ms gate) |
| `ntn-cho` | 10-seed × 600-s × 66-sat Walker-Star: HOs **135 ± 12** vs A3 463 ± 48; ping-pong 57 % → **0 %**; Wilcoxon p < 0.005 |
| `oran-ntn` | 600-s scenario, 5 live xApps: **85 074** actions, 0 reported conflicts |
| `thz-ntn` | atm windows match ITU-R P.676/618; UM-MIMO ≤ 128×128 demonstrated; ISAC CRB tracked over LEO pass |

## Documentation

- [INSTALL.md](INSTALL.md) — full setup, dependencies, GPU + Docker prerequisites, troubleshooting.
- [docs/ns3_ntn_toolkit_architecture.png](docs/ns3_ntn_toolkit_architecture.png) — high-level architecture diagram.
- Per-module READMEs — see each repository in the *Bundled modules* table above.
- [docs/UPSTREAM_NS3_README.md](docs/UPSTREAM_NS3_README.md) — original ns-3 README (build / test / run / app-store / contributing instructions for upstream ns-3).
- Reference papers (in submission):
  - SoftwareX — *ns3-ntn-toolkit: A Pre-Integrated ns-3 Platform for 6G NTN*
  - IEEE TAES — *Time-to-Exit Conditional Handover for 6G LEO Satellite Networks*
  - IEEE TNSM — *A Space O-RAN Architecture and E2 Service Model for Handover Prediction*
  - IEEE T-TST — *A Physics-Grounded 300 GHz – 1 THz LEO-NTN Model*

## Cite this work

```bibtex
@software{ns3_ntn_toolkit_2026,
  author = {Uzair, Muhammad},
  title  = {{ns3-ntn-toolkit}: An Integrated ns-3.43 Platform for 6G Non-Terrestrial Network Simulation},
  year   = {2026},
  url    = {https://github.com/Muhammaduazir69/ns3-ntn-toolkit}
}
```

## Credits & upstream sources

- **ns-3.43** — [nsnam/ns-3-dev](https://gitlab.com/nsnam/ns-3-dev) (GPL-2.0)
- **mmWave module** — [NYU Wireless / CTTC](https://github.com/nyuwireless-unipd/ns3-mmwave) (GPL-2.0)
- **SNS3 satellite** — [SNS3/sns3-satellite](https://github.com/sns3/sns3-satellite) (GPL-2.0)
- **ns3-ai upstream** — [hust-diangroup/ns3-ai](https://github.com/hust-diangroup/ns3-ai) (GPL-2.0)
- **NVIDIA Sionna RT** — [NVlabs/sionna](https://github.com/NVlabs/sionna) (Apache-2.0)
- **EURECOM FlexRIC** — [Mosaic5G / FlexRIC](https://gitlab.eurecom.fr/mosaic5g/flexric) (BSD-3)
- **Eclipse SUMO** — [eclipse-sumo/sumo](https://github.com/eclipse-sumo/sumo) (EPL-2.0)
- **Brandon Rhodes** — `sgp4`, Skyfield (MIT)
- **InfluxData / Grafana Labs / NIST NetSimulyzer** — observability stack
- **Integration, NTN modules, FlexRIC bridge, Sionna bridge, RL bridge patches** — Muhammad Uzair

## License

GPL-2.0-only — see [LICENSE](LICENSE). Each bundled module retains its own license file (all GPL-2.0-compatible). Upstream `Sionna` (Apache-2.0) and `FlexRIC` (BSD-3) are integrated as separate processes via UDP / SCTP wire protocols and ship in the toolkit only via Docker recipes — no source files are vendored under GPL terms.

---

> The original ns-3 README (build / test / run / app-store / contributing instructions for upstream ns-3) is preserved at [docs/UPSTREAM_NS3_README.md](docs/UPSTREAM_NS3_README.md).
