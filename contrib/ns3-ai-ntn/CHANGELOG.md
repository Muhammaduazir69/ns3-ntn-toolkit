# Changelog

## [1.0.0] — 2026-04-23

Fork of upstream `ns3-ai` with the following NTN-oriented additions:

### Added
- NTN-specific Gym environment templates
  (`examples/ntn-handover-gym.cc`) that wrap `ntn-cho` state into an
  observation/action/reward loop.
- Flower AI adaptor for federated learning across multiple ns-3
  instances (`python_utils/flower_adaptor.py`).
- Federated-learning benchmarks: FedAvg, FedProx, FedNova,
  SCAFFOLD on the NTN CHO decision task.
- Build fixes for ns-3.43 (`release ns-3.43`).
- Examples bridging `ns3-ai-ntn` ↔ `ntn-cho` and
  `ns3-ai-ntn` ↔ `oran-ntn` (AI-driven xApps).

### Preserved
- Shared-memory IPC, ProtoBuf interface, and Gym-style environment
  wrapper from upstream `ns3-ai`.
- All upstream LICENSE, SPDX headers and copyright notices.
