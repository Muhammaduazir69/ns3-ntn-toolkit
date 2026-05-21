# oran-ntn — Near-RT RIC + Space RIC for Non-Terrestrial Networks

`oran-ntn` is an ns-3.43 contributed module that implements an O-RAN
**Near-Real-Time RIC** with 13 xApps, an A1 policy engine, an
E2SM-RC action runtime, and a co-orbiting **Space RIC** that operates
autonomously through feeder-link outages. The module ships with five
Gymnasium environments wired into [`ns3-ai`](../ns3-ai-ntn/), an
mmWave NR-NTN PHY layer, an ISL transport header, and a federated-learning
runtime for cross-satellite policy aggregation.

<p align="center">
  <img src="visualization/oran_ntn_architecture.png"
       alt="oran-ntn architecture" width="900"/>
</p>

| | |
|---|---|
| ns-3 version          | `release ns-3.43`              |
| Module version        | `1.0.0`                        |
| License               | GPL-2.0-only                   |
| Maintainer            | Muhammad Uzair (ORCID 0009-0002-4104-2680) |
| Source size           | 33 `.cc` / 35 `.h` files       |
| Test cases            | 18 (single suite, all QUICK)   |
| Default scenario      | `examples/oran-ntn-full-scenario.cc` |

## What's in the box

### Near-RT RIC platform
- `OranRic` (`oran-ntn-near-rt-ric.{cc,h}`) — RIC kernel with
  xApp lifecycle, E2 termination, SDL, and a conflict manager.
- `OranNtnE2Interface` — E2AP-style subscription / indication path.
- `OranNtnA1Interface` — A1 policy ingest from the Non-RT RIC.
- `OranNtnConflictManager` — five resolution strategies (priority,
  temporal, merge, drop, escalate).

### 13 xApps (`model/oran-ntn-xapp-*`)
| # | xApp                  | Algorithm class | Source file |
|---|-----------------------|-----------------|-------------|
| 1 | HO Predict            | DQN             | `oran-ntn-xapp-ho-predict` |
| 2 | Beam Hop              | PPO             | `oran-ntn-xapp-beam-hop` |
| 3 | Slice Manager         | MAPPO           | `oran-ntn-xapp-slice-manager` |
| 4 | Doppler Comp.         | Kalman          | `oran-ntn-xapp-doppler-comp` |
| 5 | TN-NTN Steering       | rule-based      | `oran-ntn-xapp-tn-ntn-steering` |
| 6 | Interference Mgmt     | ICIC            | `oran-ntn-xapp-interference-mgmt` |
| 7 | Energy Harvest        | RL              | `oran-ntn-xapp-energy-harvest` |
| 8 | Predictive Alloc.     | LSTM            | `oran-ntn-xapp-predictive-alloc` |
| 9 | Multi-Connectivity    | DC / MC         | `oran-ntn-xapp-multi-conn` |
| 10 | ISAC                 | joint comm./sense | `oran-ntn-xapp-isac` |
| 11 | THz Beam Mgmt        | EKF             | `oran-ntn-xapp-thz-beam-mgmt` |
| 12 | THz RIS              | phase-config    | `oran-ntn-xapp-thz-ris` |
| 13 | THz Spectrum         | sensing         | `oran-ntn-xapp-thz-spectrum` |

All xApps derive from `OranNtnXappBase` and expose a uniform
`Decide(KpmReport) → RcAction` interface.

### Space RIC
- `OranNtnSpaceRic` — on-board RIC stub with autonomous mode
  triggered by feeder-link outage.
- `OranNtnSpaceRicInference` — local inference path for
  KPM-driven decisions while the ground RIC is unreachable.
- `OranNtnIslHeader` — ISL transport header for intra-/inter-plane
  policy and gradient exchange.

### NR-NTN physical layer
- `OranNtnMmwaveBeamforming` — mmWave NR PHY hooks.
- `OranNtnChannelModel` — NTN channel composition.
- `OranNtnNtnScheduler` — NTN-aware scheduler.
- `OranNtnPhyKpmExtractor` — per-symbol KPM extraction.
- `OranNtnDualConnectivity` — TN ↔ NTN dual connectivity.

### Satellite bridge
- `OranNtnSatBridge` — SGP4 orbit propagation, Markov 3-state fading,
  DVB-S2X ModCod table (28 entries), inter-beam interference,
  ISL topology, C/N₀ + link-budget computation.

### AI/ML integration (`ns3-ai`)
| Gym env file               | Wired xApp / pipeline |
|----------------------------|-----------------------|
| `oran-ntn-gym-handover`    | HO Predict            |
| `oran-ntn-gym-beam-hop`    | Beam Hop              |
| `oran-ntn-gym-slice`       | Slice Manager         |
| `oran-ntn-gym-steering`    | TN-NTN Steering       |
| `oran-ntn-gym-predictive`  | Predictive Alloc.     |

Python agents live in `tools/` (`oran_ntn_ai_agent.py`,
`oran_ntn_gym_agents.py`, `oran_ntn_space_ric_agent.py`).

### Federated learning
`OranNtnFederatedLearning` exposes hooks for the four aggregator
families used in the toolkit (FedAvg, FedProx, FedNova, SCAFFOLD)
over ISL gradients.

## Build & run

The module is built automatically when the parent `ns3-ntn-toolkit`
is configured. Standalone:

```bash
cd ns-3-dev
./ns3 configure --enable-tests --build-profile=optimized
./ns3 build oran-ntn -j$(nproc)
```

Run the bundled scenario:

```bash
./ns3 run "oran-ntn-full-scenario --duration=600 --numUes=100"
```

CLI options (from `oran-ntn-full-scenario.cc`):

| Flag                 | Default | Meaning |
|----------------------|---------|---------|
| `--duration`         | 600 s   | simulation time |
| `--numPlanes`        | 6       | orbital planes |
| `--satsPerPlane`     | 11      | satellites per plane |
| `--altitude`         | 550 km  | orbit altitude |
| `--inclination`      | 53°     | orbital inclination |
| `--numTnGnbs`        | 5       | terrestrial gNBs |
| `--numUes`           | 100     | UEs (mixed mobility) |
| `--kpmInterval`      | 1.0 s   | KPM reporting interval |
| `--outputDir`        | `./`    | CSV output directory |
| `--conflictStrategy` | priority | `priority`, `temporal`, or `merge` |
| `--enableSpaceRic`   | true    | enable on-board Space RICs |
| `--enableFL`         | false   | enable federated learning |

## Tests

```bash
./ns3 test --suite=oran-ntn
```

The suite (`test/oran-ntn-test-suite.cc`) contains 18 QUICK test
cases covering the RIC core, E2/A1 interfaces, conflict resolution,
Space RIC, the full pipeline, the satellite bridge, KPM extraction,
the channel model, scheduler, dual connectivity, federated learning,
the advanced xApps, ISL header, inference path, and the Space RIC + ISL
integration.

## Outputs

A 600-s `oran-ntn-full-scenario` run produces, in `outputDir`:

| File                | Content |
|---------------------|---------|
| `action_log.csv`    | per-action E2SM-RC log |
| `conflict_log.csv`  | conflict-manager decisions |
| `xapp_metrics.csv`  | per-xApp activation + decision metrics |
| `kpm_dataset.csv`   | flat KPM record stream |

Regenerate the architecture diagram:

```bash
python3 tools/generate_architecture.py
```

## Module dependencies

`oran-ntn` links against ns-3 core / network / mobility / spectrum /
propagation / internet / applications, the in-tree `satellite`,
`mmwave`, and `lte` modules, and the `ns3-ai-ntn` fork shipped in
this toolkit.

## Citing

```bibtex
@misc{uzair2026oranntn,
  author = {Muhammad Uzair},
  title  = {oran-ntn: Near-RT RIC + Space RIC and 13 xApps for
            Non-Terrestrial Networks in ns-3.43},
  year   = {2026},
  note   = {ns-3 contributed module, v1.0.0. ORCID 0009-0002-4104-2680},
  url    = {https://github.com/Muhammaduazir69/oran-ntn}
}
```

## Part of the ns3-ntn-toolkit

| Module | Repo |
|---|---|
| Toolkit (umbrella) | [ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit) |
| ntn-constellation | [ntn-constellation](https://github.com/Muhammaduazir69/ntn-constellation) |
| ntn-rrc | [ntn-rrc](https://github.com/Muhammaduazir69/ntn-rrc) |
| ntn-observability | [ntn-observability](https://github.com/Muhammaduazir69/ntn-observability) |
| ns3-ai (fork) | [ns3-ai](https://github.com/Muhammaduazir69/ns3-ai) |
| ntn-sagin | [ntn-sagin](https://github.com/Muhammaduazir69/ntn-sagin) |
| ntn-slice | [ntn-slice](https://github.com/Muhammaduazir69/ntn-slice) |
| ntn-v2x | [ntn-v2x](https://github.com/Muhammaduazir69/ntn-v2x) |
| ntn-traffic | [ntn-traffic](https://github.com/Muhammaduazir69/ns3-ntn-toolkit/tree/main/ns-3-dev/contrib/ntn-traffic) |
| ntn-sionna | [ntn-sionna](https://github.com/Muhammaduazir69/ntn-sionna) |
| ntn-digital-twin | [ntn-digital-twin](https://github.com/Muhammaduazir69/ntn-digital-twin) |
| ntn-cho | [ntn-cho-framework](https://github.com/Muhammaduazir69/ntn-cho-framework) |
| **oran-ntn** | this repo |
| thz-ntn | [ns3-thz-ntn](https://github.com/Muhammaduazir69/ns3-thz-ntn) |

## License

GPL-2.0-only — see [`LICENSE`](LICENSE).
