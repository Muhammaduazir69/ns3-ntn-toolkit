# Route breaks and lifetime-aware routing

Code and inputs behind the article "Route Breaks and Lifetime-Aware Routing in Low Earth Orbit Constellations
With Range-Limited +Grid Laser Links" (Muhammad Uzair, under review).

## What is in the module

- `examples/ntn-constellation-route-lifetime.cc`: ground-to-ground routing over a +Grid shell on a real IPv4
  data plane. Every inter-satellite link and every ground link that can come into view is a point-to-point link
  whose delay follows the live range and whose error model drops everything while the contact is down, so the
  cost of a stale route is measured. Policies (`--policy`): `snap`, `ondemand`, `asr`, `asrk`, `life`, `elev`,
  `elevb`, `track`, `alpr` (average-latency persistent routing) and `isasr` (stability and activeness slotted
  routing). Other switches used in the study: `--bbm` (one terminal per gateway), `--noData` (control plane
  only), `--fallbackEps`, `--tAcq`, `--elevNoiseDeg`, `--trackSamples`, `--floorGrid`, and `--tle`/`--edges`/
  `--epochUnix` for real shells propagated with Vallado SGP4.
- `model/contact-graph-scheduler.{h,cc}`: candidate ground links (`SetGslCandidates`), ISL refresh every k-th
  tick (`SetIslRefreshEvery`) and contact-end prediction.
- `model/contact-graph-router.{h,cc}`: flat adjacency Dijkstra and `ShortestPathCost` (Dijkstra with a
  caller-supplied edge cost).
- `model/sgp4-mobility-model.{h,cc}`: `SetSimulationEpochUnix` (one common time origin for satellites read from
  real element sets), `GetEcefStateAt` (state at a future instant), the Julian-date time-precision fix and the
  propagator counters.
- `test/ntn-constellation-test-suite.cc`: tests for `LifetimeAwarePath`, `PredictContactEndS`, the ISL candidate
  list and SGP4 against Kepler+J2 on a real LEO element set.

## Files here

| File | Purpose |
|---|---|
| `run_p7.py` | main campaign (Walker shells and real shells, every policy), resumable, one JSON line per run in `out/p7_v8.jsonl` |
| `run_p7_part2.py` | reproduction sample, ISASR grid, held-out shells and the elevation-rank sweep, into `out/p7_v9.jsonl` |
| `gen_real_shell.py`, `isl_core.py` | build `data/real_*.tle` and `data/real_*.edges` from CelesTrak catalogs |
| `data/real_{starlink,starlink16,oneweb}.{tle,edges}` | the real shells used in the runs (element sets of 2 and 16 Oct 2026) |

## How to reproduce

```bash
cd ns-3-dev
./ns3 configure --build-profile=optimized --enable-examples --enable-tests
./ns3 build ntn-constellation-route-lifetime
./test.py -s ntn-constellation

# one run
./ns3 run "ntn-constellation-route-lifetime --policy=life --eps=0.05 --tAcq=1"

# the campaign: point the scripts at the built example (or set P7_BIN), then
cd contrib/ntn-constellation/papers/route-lifetime
mkdir -p bin out
ln -sf "$(ls ../../../../build/contrib/ntn-constellation/examples/*ntn-constellation-route-lifetime*)" bin/p7-route-lifetime
python3 run_p7.py 4          # jobs in parallel; optional second argument: comma-separated parts
python3 run_p7_part2.py 4
```

The scripts run the binary from the ns-3 root (`NS3_DIR`, by default four levels up) and need Python 3 with NumPy.
`gen_real_shell.py` also needs `sgp4` and the CelesTrak catalogs it names, which are not stored here; the shells it
produced are in `data/`. Run outputs are not stored in the repository.
