# oran-ntn — Near-RT RIC + Space-RIC + 13 xApps for NTN in ns-3

`oran-ntn` is an ns-3 contributed module that implements the **O-RAN
Near-Real-Time RIC** and a new **Space RIC** on top of the `ntn-cho`
module. It ships 13 xApps, an A1 policy engine with 11 policies, an
E2SM-RC action runtime with 28 actions, and a conflict-resolution
matrix for co-located xApps.

<p align="center">
  <img src="visualization/oran_ntn_architecture.png" alt="oran-ntn architecture" width="900"/>
</p>

- ns-3 version: `release ns-3.43`
- Version: `1.0.0`
- License: GPL-2.0-only
- Maintainer: Muhammad Uzair (ORCID 0009-0002-4104-2680)

## Highlights

- **13 xApps** covering traffic steering, handover, QoS, beam
  management, load balancing, anomaly detection, energy saving,
  slicing, RLM, cell on/off, admission control, interference
  coordination, and Space RIC.
- **A1 policy engine** (11 policy types) + **E2SM-RC action table**
  (28 actions) with a conflict-resolution matrix.
- **60-s full-xApp run**: 7369 actions, 0 conflicts
  (`action_log.csv`, `conflict_log.csv`, `xapp_metrics.csv`).
- **Space RIC** with stressed-feeder autonomous-mode metrics.
- **Federated-extension hooks** (A1 cost bound based on Shen 2025
  TNSM transfer accounting).

## Quick start

```bash
cd ns-3-dev
git clone https://github.com/Muhammaduazir69/oran-ntn contrib/oran-ntn
./ns3 configure --enable-examples --enable-tests \
    --enable-modules=oran-ntn
./ns3 build
./ns3 run "oran-ntn-scenario-b-full-xapps --duration=60"
./ns3 test --suite=oran-ntn
```

## What's in the module

```
model/       — OranRic, 13 xApps, A1 engine, E2SM-KPM/RC runtime,
               SpaceRic, conflict resolver
helper/      — RIC helper, xApp helper, policy installer
examples/    — 6 scenarios including oran-ntn-scenario-b-full-xapps
test/        — 5 suites: e2ap, a1, kpm, conflict, space-ric
tools/       — action_log → CSV pipeline, xApp metric aggregator
visualization/ — Python timeline & activity-matrix plotters
```

## Reproducing the paper results

```bash
cd papers/sim_runs/oran-ntn/run1
# CSVs already present, regenerate figures:
cd ../..
python3 build_figures_thz_oran.py
```

Full methodology, xApp inventory, policy table, and action table are
in **Paper 3** (`papers/paper3_tnsm_oran_ntn/main.tex`), targeted at
IEEE TNSM.

## Citing

```bibtex
@misc{uzair2026oranntn,
  author = {Muhammad Uzair},
  title  = {oran-ntn: Near-RT RIC, Space RIC and 13 xApps for
            Non-Terrestrial Networks in ns-3},
  year   = {2026},
  note   = {ns-3 App Store module, v1.0.0. ORCID 0009-0002-4104-2680}
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

GPL-2.0-only. See `LICENSE`.
