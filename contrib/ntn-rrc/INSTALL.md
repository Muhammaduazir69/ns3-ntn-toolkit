# Installing ntn-rrc

`ntn-rrc` is an optional ns-3.43 contrib module. Place this directory under
`contrib/ntn-rrc/` in an ns-3.43 tree.

## Dependencies

The library (`build_lib` in `CMakeLists.txt`) links ns-3 `core`, `network`, and
`mobility`.

The examples additionally need other modules (see `examples/CMakeLists.txt`):

- `internet`, `applications`, `point-to-point`, and the `ntn-traffic` contrib
  module (used by `ntn-rrc-leo-pass`, `ntn-rrc-full-stack`, `ntn-rrc-from-tle`).
- The **`satellite` module (SNS3)** — required by `ntn-rrc-full-stack` and
  `ntn-rrc-from-tle`, which drive a `SatSGP4MobilityModel`.
- `flow-monitor` — used by `ntn-rrc-drx-data-traffic`.

## Build

From the repository root:

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build
```

## Run the examples

There are four examples:

```bash
# Ephemeris-driven Timing Advance over a 600 s LEO pass (straight-line geometry)
./ns3 run "ntn-rrc-leo-pass --simTime=600 --transparent=true"

# All four NTN-RRC components over a real SGP4 orbit (bundled ISS TLE), UE in Islamabad
./ns3 run "ntn-rrc-full-stack --simTime=600 --transparent=true --passAwareDrx=true"

# Drive a SatSGP4MobilityModel from a TLE; runs with no args (bundled ISS TLE)
./ns3 run "ntn-rrc-from-tle"
# ...or with an explicit TLE:
./ns3 run "ntn-rrc-from-tle --tle=contrib/ntn-rrc/data/iss-zarya.tle"

# Real UDP downlink gated by SIB19 + timing advance + connected-mode DRX
./ns3 run "ntn-rrc-drx-data-traffic --simSeconds=60 --dataRateMbps=5 --drxOn=true"
```

## Run the test suite

```bash
./test.py --suite=ntn-rrc
```

The suite (`Type::UNIT`) has 16 test cases covering the closed-form TA
(transparent and regenerative), the common/UE-specific decomposition, the LEO
drift-rate bound, SIB19 codec round-trip and truncation rejection, broadcaster
cadence, ECEF<->WGS-84 round-trip, all three location-report modes, the standard
DRX cycle and data-activity override, pass-aware deep sleep, and invalid-config
rejection.
