# Installing ntn-v2x

`ntn-v2x` is an ns-3 contrib module. Drop it into the `contrib/` directory of
an ns-3.43 tree and build it with the normal ns-3 workflow.

## Dependencies

The module library links only against core ns-3 modules (see `CMakeLists.txt`):

- `core`
- `network`
- `mobility`

No external libraries are required. The live SUMO TraCI path uses only the
standard POSIX sockets already available; SUMO itself is **not** required —
FCD replay is the supported path (see the README).

## Build

Place the module under `contrib/ntn-v2x`, then from the ns-3 root:

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build
```

## Examples

The examples pull in a few more ns-3 modules (`internet`, `applications`,
`point-to-point`, `flow-monitor`). In addition, the `ntn-v2x-rural-highway`
example also depends on the **`ntn-traffic`** module (`libntn-traffic`), so that
module must be present under `contrib/` for the example to build. The
`ntn-v2x-leo-relay-traffic` example does not need `ntn-traffic`.

## Test

Run the module's unit-test suite:

```bash
./test.py --suite=ntn-v2x
```

Or run the test runner directly:

```bash
./build/utils/ns3.43-test-runner-default --suite=ntn-v2x
```

## Part of the toolkit

`ntn-v2x` is one module of the
[ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit). See the
toolkit repository for the full multi-module setup and build notes.
