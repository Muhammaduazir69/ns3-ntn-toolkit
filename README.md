<h1 align="center">ns3-ntn-toolkit</h1>

<p align="center"><strong>A pre-integrated ns-3.43 simulation platform for 6G Non-Terrestrial Network research — clone, build, run.</strong></p>

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0-green.svg"/></a>
  <img src="https://img.shields.io/badge/3GPP-Rel--17%20NTN-orange.svg"/>
  <img src="https://img.shields.io/badge/O--RAN-WG3%20%2F%20WG2-purple.svg"/>
  <img src="https://img.shields.io/badge/THz-100%20GHz%20–%201%20THz-success.svg"/>
  <img src="https://img.shields.io/badge/RL-Gymnasium%201.0-red.svg"/>
</p>

<p align="center">
  <img src="docs/ns3_ntn_toolkit_architecture.png" alt="ns3-ntn-toolkit architecture" width="950"/>
</p>

---

## Why this toolkit

Open research on 6G non-terrestrial networks is held back by **tool fragmentation**: orbit propagation, 3GPP NTN channels, 5G/NR protocol stacks, Open-RAN interfaces, and reinforcement-learning bridges live in four or five packages that rarely build together on recent ns-3 releases. `ns3-ntn-toolkit` is one buildable distribution that bundles them — a single `./ns3 build` yields an end-to-end NTN simulator in under fifteen minutes, turning an otherwise multi-week integration effort into a one-hour onboarding task.

## At a glance

| Capability | Numbers |
|---|---|
| ns-3 base | **3.43** (patched LTE for dual connectivity) |
| Custom modules contributed by this work | **5** (`ntn-cho`, `oran-ntn`, `thz-ntn`, `traffic`, `ai` fork) |
| O-RAN xApps shipped | 13 |
| E2SM-RC actions / A1 policies | 28 / 11 |
| THz physics | HITRAN-2020 line-by-line, ITU-R P.835/676/618/838, UM-MIMO ≤ 128×128 |
| Conditional handover | 3GPP Rel-17 with novel TTE trigger |
| Reference Monte-Carlo | **10-seed × 600-s** Walker-Star, 4 algorithms |
| RL bridge | Gymnasium 1.0 over patched ns3-ai (Py 3.13 + NumPy 2 ready) |
| 3D dashboard | CesiumJS web viewer |

## Bundled modules

| Module | Repo | Purpose |
|---|---|---|
| `ntn-cho` | [ntn-cho-framework](https://github.com/Muhammaduazir69/ntn-cho-framework) | TTE-aware 3GPP Rel-17 conditional handover + 7-class realistic UE mobility |
| `oran-ntn` | [oran-ntn](https://github.com/Muhammaduazir69/oran-ntn) | Space O-RAN: 13 xApps, dual Near-RT/Space RIC, conflict mgr, 4 FL aggregators |
| `thz-ntn` | [ns3-thz-ntn](https://github.com/Muhammaduazir69/ns3-thz-ntn) | 100 GHz – 1 THz physics: HITRAN, UM-MIMO, RIS, ISAC, EKF beam tracking |
| `ai` (fork) | [ns3-ai](https://github.com/Muhammaduazir69/ns3-ai) | Modernised shared-memory bridge — ns-3.43 + Py 3.13 + NumPy 2 + Gymnasium 1.0 |
| `satellite` | [SNS3](https://github.com/sns3/sns3-satellite) | SGP4 propagator + TR 38.811 NTN channel + Loo/Markov fading (upstream) |
| `mmwave` | [NYU/CTTC](https://gitlab.com/cttc-lena/nr) | 5G NR PHY/MAC + dual connectivity (upstream + LTE patches) |

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

### Module-output snapshots

| O-RAN xApps showcase | NTN-CHO algorithm comparison |
|---|---|
| <img src="docs/oran_ntn_showcase.png" width="430"/> | <img src="docs/ntn_cho_showcase.png" width="430"/> |

## Install & run

See [**INSTALL.md**](INSTALL.md) for the full step-by-step guide (system requirements, SNS3 satellite clone, build flags, troubleshooting).

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
```

## Reference scenarios shipped

| Scenario | Module | Wall-clock | Outputs |
|---|---|---|---|
| `ntn-realistic-mobility-demo` | ntn-cho | ~5 s | 14-UE 600-s trajectory CSV |
| `ntn-cho-full-constellation` | ntn-cho | ~30 s | per-seed CHO KPM CSVs |
| `oran-ntn-full-scenario` | oran-ntn | ~60 s | 5-xApp 600-s action / KPM logs |
| `thz-ntn-demo` (9 sub-scenarios) | thz-ntn | ~30 s | atm-windows / link budget / ISAC CSVs |
| `ntn-tn-integrated-analysis` | toolkit | ~20 s | TN+NTN integrated traces |

## Documentation

- [INSTALL.md](INSTALL.md) — full setup, dependencies, troubleshooting
- [docs/ns3_ntn_toolkit_architecture.png](docs/ns3_ntn_toolkit_architecture.png) — toolkit architecture
- Per-module READMEs — see each repo above
- Reference papers (in submission):
  - SoftwareX — *ns3-ntn-toolkit: A Pre-Integrated ns-3 Platform for 6G NTN*
  - IEEE TAES — *Time-to-Exit Conditional Handover for 6G LEO Satellite Networks*
  - IEEE TNSM — *A Space O-RAN Architecture and E2 Service Model for Handover Prediction*
  - IEEE T-TST — *A Physics-Grounded 300 GHz – 1 THz LEO-NTN Model*

## Cite this work

```bibtex
@software{ns3_ntn_toolkit_2026,
  author = {Muhammad Uzair},
  title  = {ns3-ntn-toolkit: An Integrated ns-3.43 Platform for 6G Non-Terrestrial Network Simulation},
  year   = {2026},
  url    = {https://github.com/Muhammaduazir69/ns3-ntn-toolkit}
}
```

## Credits & upstream sources

- **ns-3.43** — [nsnam/ns-3-dev](https://gitlab.com/nsnam/ns-3-dev) (GPL-2.0)
- **mmWave module** — [NYU Wireless / CTTC](https://github.com/nyuwireless-unipd/ns3-mmwave) (GPL-2.0)
- **SNS3 satellite** — [SNS3/sns3-satellite](https://github.com/sns3/sns3-satellite) (GPL-2.0)
- **ns3-ai upstream** — [hust-diangroup/ns3-ai](https://github.com/hust-diangroup/ns3-ai) (GPL-2.0)
- **Integration, NTN modules, RL bridge patches** — Muhammad Uzair

## License

GPL-2.0-only — see [LICENSE](LICENSE).

---

> ℹ The original ns-3 README (build / test / run / app-store / contributing instructions for upstream ns-3) is preserved at [docs/UPSTREAM_NS3_README.md](docs/UPSTREAM_NS3_README.md).
