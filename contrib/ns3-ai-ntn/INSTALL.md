# Install & run — ns3-ai (modernised fork)

<p align="center">
  <a href="README.md">Module README</a>
  &nbsp;·&nbsp;
  <a href="https://github.com/Muhammaduazir69/ns3-ntn-toolkit">Toolkit</a>
  &nbsp;·&nbsp;
  <a href="https://github.com/Muhammaduazir69/ns3-ntn-toolkit/blob/ntn-integration-v2/INSTALL.md">Toolkit install guide</a>
  &nbsp;·&nbsp;
  <a href="https://muhammaduazir69.github.io/ns3-ntn-toolkit/">Docs site</a>
</p>

> **The fastest path is the container.** `docker pull uzairdocker69/ns3-ntn-toolkit:latest`
> ships this module already built alongside the other thirteen and the vendored
> stacks, so nothing below is needed to simply run the examples. Build from source
> when you intend to change the module.

---

This guide installs the **fork** of `ns3-ai` that is patched for
ns-3.43, Python 3.13, NumPy 2.0, and Gymnasium 1.0.

> **Branch:** `fix/ns3-43-compatibility-and-critical-bugs`
> **Upstream:** https://github.com/hust-diangroup/ns3-ai

---

## 1. System requirements

| Component | Version |
|---|---|
| OS | Linux (Ubuntu 22.04+ / Fedora 39+) |
| C++ compiler | gcc ≥ 11 or clang ≥ 14 |
| CMake | ≥ 3.24 |
| Python | **3.10–3.13** (3.13 explicitly tested) |
| NumPy | **≥ 2.0** |
| Boost | ≥ 1.74 (interprocess) |
| pybind11 | **≥ 2.13** (bundled in `vendor/`) |
| ns-3 | **3.43** |

### Python deps

```bash
pip install "numpy>=2.0" "gymnasium>=1.0" "torch>=2.0"
```

---

## 2. Prerequisites

### 2a. ns-3.43

```bash
git clone -b ntn-integration-v2 https://github.com/Muhammaduazir69/ns3-ntn-toolkit.git
cd ns3-ntn-toolkit
```

> GitLab mirror: `git clone -b ntn-integration-v2 https://gitlab.com/ns3-ntn-toolkit/ns3-ntn-toolkit.git`
> Or skip the build entirely with the Docker image (this fork preinstalled):
> `docker pull uzairdocker69/ns3-ntn-toolkit:latest` (or `:latest`), then
> `docker run -it uzairdocker69/ns3-ntn-toolkit:latest`.

### 2b. (Optional) other contrib modules

If you want to drive a satellite scenario from RL:

```bash
cd contrib/
git clone https://github.com/sns3/sns3-satellite.git satellite
git clone -b main https://github.com/Muhammaduazir69/ntn-cho-framework.git ntn-cho
cd ..
```

---

## 3. Install the fork

The module's CMake `LIBNAME` is `ns3-ai-ntn`, so it must be cloned into
`contrib/ns3-ai-ntn`:

```bash
cd contrib/
git clone -b fix/ns3-43-compatibility-and-critical-bugs \
  https://github.com/Muhammaduazir69/ns3-ai.git ns3-ai-ntn
cd ..
```

---

## 4. Configure & build

The fork's CMake helper handles the per-target LTO disable that ns-3.43 needs:

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build ns3-ai-ntn
```

Verify the bridge module is built:

```bash
./ns3 show profile | grep ns3-ai-ntn
ls build/contrib/ns3-ai-ntn/  # build artefacts + the per-example ns3ai_*.so modules
```

---

## 5. Run examples

Each example pairs a C++ ns-3 binary with a Python driver; the Python script
spawns the matching ns-3 binary itself. Build the example's CMake target first
(target names are listed in each example's `README.md`).

### 5a. Hello-world (`a-plus-b`)

```bash
./ns3 build ns3ai_apb_gym
cd contrib/ns3-ai-ntn/examples/a-plus-b/use-gym/
python3 apb.py     # Python launches the ns-3 binary internally
```

Expected output: a stream of `(a, b, c=a+b)` triples. The message-interface
variants live in `use-msg-stru/` and `use-msg-vec/` (also `apb.py`).

### 5b. LTE CQI prediction (online LSTM)

```bash
./ns3 build ns3ai_ltecqi_msg
cd contrib/ns3-ai-ntn/examples/lte-cqi/use-msg/
python3 run_online_lstm.py
```

### 5c. Multi-BSS Wi-Fi RL

```bash
./ns3 build ns3ai_multibss
cd contrib/ns3-ai-ntn/examples/multi-bss/
python3 run_multi_bss.py
```

### 5d. RL-TCP

```bash
./ns3 build ns3ai_rltcp_gym
cd contrib/ns3-ai-ntn/examples/rl-tcp/use-gym/
python3 run_rl_tcp.py
```

### 5e. Rate control (Wi-Fi, message interface)

```bash
./ns3 build ns3ai_ratecontrol_ts
cd contrib/ns3-ai-ntn/examples/rate-control/thompson-sampling/
python3 ai_thompson_sampling.py
```

---

## 6. NTN RL environments (synthetic — NOT ns-3-backed)

The `ns3_ai_ntn` Python package ships four Gymnasium environments
(`HandoverEnv`, `BeamMgmtEnv`, `SliceEnv`, `PowerCtrlEnv`) for fast policy
search. **These are synthetic placeholders: their RSRP/SINR are closed-form
proxies, they do NOT boot `contrib/ntn-cho` or read any ns-3 PHY trace, and they
must not be reported as measured results.** The source files carry that warning
in their docstrings and raise `RuntimeError` honesty guards rather than
silently fabricating measured KPIs. There is no real ns-3 C++ environment
binary behind them.

```bash
cd contrib/ns3-ai-ntn/python_utils
pip install -e .[test]
python3 -c "from ns3_ai_ntn.envs import HandoverEnv; e=HandoverEnv(); e.reset(); print(e.step(0))"
```

For a *measured* satellite RL/inference loop, drive the ns-3 data plane through
the AI-RAN inference contract (`grpc/`) or the shared-memory bridge against a
real scenario in `contrib/ntn-cho` / `contrib/oran-ntn`.

---

## 7. Run the tests

```bash
# C++ AI-RAN inference contract (suite name: oran-ntn-airan-inference, 9 cases —
# codec round-trips, in-proc/TCP channels, Triton config parsing, failure
# modes, in-simulator workload)
./test.py -s oran-ntn-airan-inference

# Python NTN RL extensions
cd contrib/ns3-ai-ntn/python_utils
pip install -e .[test] && pytest tests/ -v
```

---

## 8. Common issues

**`ImportError: dynamic module does not define module export function (PyInit_ns3ai_X)`**
You're hitting the LTO bug. Make sure you're on this fork — upstream ns3-ai still has it. The fix is `ns3ai_add_pybind_module()` in `cmake/`.

**`AttributeError: module 'numpy' has no attribute 'int'`**
Old NumPy 1.x type-aliases. Make sure NumPy ≥ 2.0 is installed and you're on this fork.

**`OSError: [Errno 38] Function not implemented` from `os.setpgrp`**
Python 3.13 removed `preexec_fn=os.setpgrp`. This fork uses `start_new_session=True` instead — make sure you're on the latest commit.

**Stale shared-memory segment after a crash**
Run `ipcs -m` to find ns3ai_* segments and `ipcrm -m <id>` to clear them. The fork's `OpenGymInterface` destructor cleans these up on graceful exit.

**Build hangs with `flto` linker errors**
Verify `ns3ai_add_pybind_module()` was applied to the offending target — grep for it in `build/CMakeCache.txt`.

---

## 9. Citing

See [README](README.md#cite-this-work) — please cite both the original ns3-ai paper and this fork.