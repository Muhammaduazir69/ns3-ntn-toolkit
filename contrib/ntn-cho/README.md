# ntn-cho — Conditional Handover for NTN in ns-3

`ntn-cho` is an ns-3 contributed module that implements
**3GPP Release-17 Conditional Handover (CHO)** for **Non-Terrestrial
Networks (NTN)**, with a focus on LEO satellite constellations. It
ships four CHO algorithms (including a novel **Time-to-Exit
(TTE)-aware** variant), an SGP4 orbit propagator, a Walker-Star
constellation generator, and an mmWave+satellite channel adaptor built
on top of the public mmWave-ns3 module.

- ns-3 version: `release ns-3.43`
- Version: `1.0.0`
- License: GPL-2.0-only
- Maintainer: Muhammad Uzair (ORCID 0009-0002-4104-2680)

## Highlights

- **TTE-aware CHO** reduces ping-pong handovers to **0 %** in a
  5-seed Monte-Carlo campaign versus **47.64 %** for the event-A3
  baseline and **56.51 %** for location-only CHO, while maintaining
  **81.75 % ± 14.83 %** handover success (Paper 2, IEEE TAES, in
  submission).
- **3GPP-aligned state machine**: `CHO_CONFIGURED → CHO_EVALUATING →
  CHO_EXECUTING → CHO_COMPLETED` with TS 38.331 §5.3.5 timers.
- **SGP4 orbit propagation** validated against a Vallado reference
  vector (RMSE < 150 m over 1 orbit, see `test/ntn-sgp4-test.cc`).
- **Walker-Star constellation generator** with arbitrary
  `(T, P, F)` configuration.
- **Monte-Carlo harness** with Student's t 95 % CIs.
- **12 example scripts** and **6 unit test suites**.

## Quick start

```bash
cd ns-3-dev
git clone https://github.com/Muhammaduazir69/ntn-cho-framework contrib/ntn-cho
./ns3 configure --enable-examples --enable-tests --enable-modules=ntn-cho
./ns3 build
./ns3 run "ntn-cho-scenario-a --tteThreshold=5.0 --RngRun=1"
./ns3 test --suite=ntn-cho
```

## Programmatic use

```cpp
#include "ns3/ntn-cho-module.h"

Ptr<NtnConstellationHelper> walker =
    CreateObject<NtnConstellationHelper> ();
walker->SetAttribute ("TotalSats", UintegerValue (66));
walker->SetAttribute ("Planes",    UintegerValue (6));
walker->SetAttribute ("Phasing",   UintegerValue (2));
walker->SetAttribute ("AltitudeKm",DoubleValue (550));
walker->Install ();

Ptr<NtnTteCondHandoverAlgorithm> cho =
    CreateObject<NtnTteCondHandoverAlgorithm> ();
cho->SetAttribute ("TteThresholdS", DoubleValue (5.0));
cho->SetAttribute ("HysteresisDb",  DoubleValue (2.0));
```

## What's in the module

```
model/       — TTE estimator, 4 CHO algos, SGP4, Walker-Star,
               mmWave+sat channel, geometry helpers
helper/      — constellation helper, CHO helper, mobility helper
examples/    — 12 scenarios (scenario-a, scenario-b-mc, walker-star …)
test/        — 6 suites: geometry, sgp4, cho-state, tte-monotonic …
visualization/ — Python plotters for dwell/track/HO traces
tools/       — Monte-Carlo runner (mc_runner.cc), CSV helpers
```

## Reproducing the paper results

```bash
cd papers/sim_runs
./run_mc_sweep.sh          # 4 algos × 5 seeds = 20 ns-3 runs
python3 build_figures.py   # regenerate all figures/*.pdf
```

Full methodology, equations, and result tables are in
**Paper 2** (`papers/paper2_taes_tte_cho/main.tex`), targeted at
IEEE TAES.

## Citing

If you use this module, please cite:

```bibtex
@misc{uzair2026ntncho,
  author = {Muhammad Uzair},
  title  = {ntn-cho: Time-to-Exit-Aware Conditional Handover for
            Non-Terrestrial Networks in ns-3},
  year   = {2026},
  note   = {ns-3 App Store module, v1.0.0. ORCID 0009-0002-4104-2680}
}
```

## License

GPL-2.0-only. See `LICENSE`.
