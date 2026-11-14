# Sidelink relaying in urban street canyons

Code and inputs behind the article "Which Neighbor Should Forward? Sidelink Relaying of Vehicle Status to
Direct-to-Cell Satellites in Urban Street Canyons" (Muhammad Uzair, under review).

## What is in the module

- `examples/ntn-v2x-canyon-relay.cc`: vehicles in a street canyon report status messages to a server through LEO
  satellites propagated with SGP4 from real element sets (`data/starlink_dtc_20261002.tle`). Line of sight to each
  satellite is decided by the facade geometry. A blocked vehicle can name a neighbor in the message it broadcasts on
  the NR Mode-2 sidelink, and the neighbor forwards it over its own gated IPv4 uplink. Output is one CSV row per
  delivered message, from which the scripts compute the age of information at the server.
  Forwarding rules (`--policy`): `direct`, `multisat`, `relay` (nearest neighbor that announced an uplink),
  `relayk`, `relayrand`, `relayfar` (farthest announcer in range), `relaylb` (nearest announcer at least one
  building length away), `blind`, `blindfar`, `blindlb`, `blindbest`, `random`, `randpersist`, `rand2`, `rand3`,
  `randk`, `roundrobin`, `hybrid`, `anyfwd`, `anyp`, `anypn`. For `relayk` and `relayrand` with `--relayK` up to 3
  the named neighbors travel in the message tag.
  Other switches: shared per-satellite uplink (`--ulKbps`, `--grantRtt`, `--ulMaxQueueS`), relay link setup
  (`--relaySetupMs`), SUMO mobility (`--mobTrace`), facades from a file or with cross-street gaps (`--facadeFile`,
  `--gapEveryM`, `--gapLenM`), TR 37.885 urban sidelink channel (`--slUrban`, `--slNlosP`) and reselection aligned
  with the message period (`--alignResel`).
- `model/ntn-nr-sidelink.{h,cc}`: `NtnSlUeMac::SetAlignReselection` and `SelectResourceAt` (selection window after
  the arrival of the next periodic message), `NtnSlChannel::SetUrbanChannel` (TR 37.885 urban LOS, NLOSv and NLOS
  per vehicle pair, redrawn after 10 m of relative motion), and PRR with half-duplex misses counted
  (`GetPrrWithinRange`) or excluded (`GetPrrWithinRangeExcludingHalfDuplex`).
- `test/ntn-v2x-test-suite.cc`: the Mode-2 test now checks both PRR definitions.

## Files here

| File | Purpose |
|---|---|
| `run_ns3_canyon5.py` | main campaign: every rule on three canyons and twenty starts, plus the sensitivity studies (period, capacity, handover, setup, mobility, channel, geometry, real streets) |
| `run_ns3_canyon6.py` | further runs on the final rules (`relayfar`, `relaylb`) and the server-side position error |
| `make_traces.py` | SUMO stop-and-go traces of the street, written to `traces/` (needs SUMO's `netconvert` and `sumo`) |
| `realcity/fetch_nyc.py`, `realcity/make_facades.py` | facade profiles of five Manhattan streets from NYC Open Data building footprints |
| `realcity/facade_*.csv`, `realcity/stretches*.json`, `realcity/facade_stats.json` | the facade profiles used in the runs |

## How to reproduce

```bash
cd ns-3-dev
./ns3 configure --build-profile=optimized --enable-examples --enable-tests
./ns3 build ntn-v2x-canyon-relay
./test.py -s ntn-v2x

# one run: 60 vehicles, urban canyon, nearest announcer
./ns3 run "ntn-v2x-canyon-relay --policy=relay --nVeh=60 --width=20 --h50=18 --out=/tmp/relay.csv"

# the campaign
cd contrib/ntn-v2x/papers/canyon-relay
mkdir -p bin out
ln -sf "$(ls ../../../../build/contrib/ntn-v2x/examples/*ntn-v2x-canyon-relay*)" bin/p8-canyon-relay   # or set P8_BIN
python3 make_traces.py
python3 run_ns3_canyon5.py 4 primary period cap ho setup mob chan geo real
python3 run_ns3_canyon6.py 4 r4pos r4ann r4cap
```

The scripts run the binary from the ns-3 root (`NS3_DIR`, by default four levels up) and need Python 3 with NumPy;
`make_facades.py` also needs Shapely. The study was first run on four successive builds of this example. The source
here is the last one, and the earlier studies are reproduced with `--grantRtt=0` because the grant round trip had no
effect in the builds they were first run on; the scripts pass it. SUMO traces and run outputs are not stored in the
repository.
