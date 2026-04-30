# Authors

## Upstream (credited and preserved)

This module is a **hard fork** of the upstream `ns3-ai` project.
All of the upstream copyright notices, SPDX headers, and license text
are preserved in the forked source tree.  Please cite the upstream
authors if you publish results based on the shared-memory interface,
ProtoBuf binding, or Gym-style environment wrapper:

- **Pengyu Liu**, Nanjing University of Posts and Telecommunications
- **Hao Yin**, University of Washington
- **Liu Cao**
- **Huazhi Wang**
- All other upstream `ns3-ai` contributors listed at
  https://github.com/hust-diangroup/ns3-ai

Upstream project homepage:
https://github.com/hust-diangroup/ns3-ai
Upstream license: GPL-2.0-only.

## Fork maintainer (NTN extensions)

- **Muhammad Uzair** — `muhammaduzairr69@gmail.com`
  ORCID: 0009-0002-4104-2680
  Added: NTN-specific Gym wrappers, federated-learning harness,
  Flower AI adaptor, NTN handover RL environment, ns-3.43 build fix
  pack, examples coupling `ns3-ai-ntn` with `ntn-cho` and
  `oran-ntn`.

## Attribution policy

If you contribute a change that extends functionality shared with
upstream `ns3-ai`, please also submit the patch upstream to
`hust-diangroup/ns3-ai`.  Keeping the two codebases in sync benefits
the whole ns-3 community.
