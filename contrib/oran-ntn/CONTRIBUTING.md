# Contributing

Thanks for considering a contribution to this ns-3 App Store module. All
modules in the `ns3-ntn-toolkit` family follow the same workflow.

## How to file an issue

1. Check the existing issues on the repo.
2. Open a new issue with:
   - ns-3 release you are on (e.g., `release ns-3.43`)
   - Minimal `.cc` file or command line that reproduces the problem
   - Full stack trace / `NS_LOG` output

## How to submit a patch

1. Fork the repository.
2. Create a feature branch from `main`:
   ```bash
   git checkout -b fix/short-description
   ```
3. Follow the ns-3 coding style:
   - Run `clang-format -i` using the root `.clang-format` shipped with
     `ns-3-dev`.
   - Keep public APIs documented with Doxygen.
4. Add or extend a unit test under `test/` for any behavioural change.
5. Verify `./ns3 test --suite=<module>` passes locally.
6. Commit with a short, imperative subject line; reference issues with
   `Fixes #NN` where applicable.
7. Open a pull request against `main`.

## Coding style essentials

- C++17, 4-space indentation, 100-column soft limit.
- `CamelCase` for types, `m_camelCase` for members, `camelCase` for
  variables.
- One public class per header file when practical.
- No `using namespace` in headers.
- Every new class gets an `ObjectBase` + `GetTypeId` pairing unless it
  is a pure helper.

## Licensing

All contributions are accepted under **GPL-2.0-only**. By submitting a
patch you certify the contents of your patch under the Developer
Certificate of Origin (DCO) v1.1 (https://developercertificate.org).

Add an SPDX header to every new source file:

```cpp
// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Muhammad Uzair and contributors
```

## Running tests

```bash
cd ns-3-dev
./ns3 configure --enable-examples --enable-tests
./ns3 build
./ns3 test --suite=<module-name>
```

## Getting help

- Mailing list: `ns-developers@googlegroups.com`
- App Store discussion: https://www.nsnam.org/support/
- Module maintainer: Muhammad Uzair <muhammaduzairr69@gmail.com>
  (ORCID 0009-0002-4104-2680)
