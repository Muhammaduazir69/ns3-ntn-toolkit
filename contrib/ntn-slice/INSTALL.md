# Install & run — ntn-slice

`ntn-slice` is an ns-3.43 contributed module. It builds on top of a vanilla
ns-3.43 tree, or as part of the
[ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit).

---

## 1. System requirements

| Component | Version |
|---|---|
| OS | Linux (Ubuntu 22.04+ / Fedora 39+ recommended) |
| C++ compiler | gcc >= 11 or clang >= 14 |
| CMake | >= 3.24 |
| Python | >= 3.10 |
| ns-3 | **3.43** |

---

## 2. Dependencies

The module library (`CMakeLists.txt`) links only the ns-3 **core** module.
There are no other module dependencies for the library itself.

The examples link extra sibling modules through their own example
`CMakeLists.txt`:

- `ntn-three-slice-leo-geo` links **`ntn-traffic`**
  (`NtnRealisticTrafficHelper`) plus `internet`, `applications` and
  `point-to-point`.
- `ntn-slice-isolation-traffic` links `internet`, `applications`,
  `point-to-point`, `mobility` and `flow-monitor`.

Install whichever siblings you need under `contrib/` before configuring.

The orchestrator's `(observation, action)` shape matches the `SliceEnv`
Gymnasium wrapper in `ns3-ai`; that is only needed for the optional RL path,
not for the C++ allocator.

---

## 3. Configure & build

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build ntn-slice
./ns3 show profile | grep ntn-slice   # expect: ... ntn-slice ...
```

---

## 4. Run the examples

```bash
# Three co-existing slices over LEO + GEO; GEO mode-skip + per-slice KPIs.
./ns3 run "ntn-three-slice-leo-geo --simTime=3600 --csv=/tmp/three-slice.csv"

# Real per-slice traffic with a live ns-3 data plane (FlowMonitor).
./ns3 run "ntn-slice-isolation-traffic --simSeconds=30 --dataRateMbps=20"
```

See the README for the full per-example argument list.

---

## 5. Run the unit tests

```bash
./test.py --suite=ntn-slice
```

The suite has 7 unit tests (S-NSSAI pack/unpack round-trip, selector
first-match-wins, URLLC min-throughput preservation under contention,
isolation-monitor breach detection, URLLC GEO mode-skip, three-slice
co-existence, RL-shares honoured).

---

## 6. Uninstall

```bash
rm -rf contrib/ntn-slice
./ns3 configure --enable-examples
./ns3 build
```
