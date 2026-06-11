# Changelog

## [Unreleased]

### Added
- AI-RAN inference contract (`grpc/`): `AiranInferenceClient` /
  `AiranInferenceServer` exchanging length-prefixed protobuf
  (`grpc/proto/airan_inference.proto`) over pluggable in-process and
  TCP `InferenceChannel` transports; Triton `config.pbtxt` parser;
  two shipped Triton model-repository skeletons
  (`beam_index_classifier`, `precoder_csi_to_weights`); deterministic
  mock runtimes (`AiranMockRuntime`); test suite
  `oran-ntn-airan-inference` (9 cases).

## [1.0.0] — 2026-04-23

Fork of upstream `ns3-ai` with the following additions:

### Added
- Compatibility fixes for ns-3.43 / Python 3.13 / NumPy 2.0 /
  Gymnasium 1.0: per-target LTO disable
  (`ns3ai_add_pybind_module()`), shared-memory RAII,
  `Simulator::Stop()` instead of `std::exit(0)` in library code.
- `ns3_ai_ntn` Python package (`python_utils/`): four NTN Gymnasium
  environments (`HandoverEnv`, `BeamMgmtEnv`, `SliceEnv`,
  `PowerCtrlEnv`), Stable-Baselines3 PPO training, PyTorch Geometric
  GAT constellation-graph models, MAPPO / MASAC multi-agent
  baselines, and an `ns3gym` compatibility shim.

### Preserved
- Shared-memory IPC, ProtoBuf interface, and Gym-style environment
  wrapper from upstream `ns3-ai`.
- All upstream examples (`a-plus-b`, `rl-tcp`, `rate-control`,
  `lte-cqi`, `multi-bss`), updated to the modern stack.
- All upstream LICENSE, SPDX headers and copyright notices.
