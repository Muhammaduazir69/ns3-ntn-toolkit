# Install & run — ns3-ai (modernised fork)

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
git clone https://github.com/Muhammaduazir69/ns3-ntn-toolkit.git
cd ns3-ntn-toolkit
```

### 2b. (Optional) other contrib modules

If you want to drive a satellite scenario from RL:

```bash
cd contrib/
git clone https://github.com/sns3/sns3-satellite.git satellite
git clone https://github.com/Muhammaduazir69/ntn-cho-framework.git ntn-cho
cd ..
```

---

## 3. Install the fork

```bash
cd contrib/
git clone -b fix/ns3-43-compatibility-and-critical-bugs \
  https://github.com/Muhammaduazir69/ns3-ai.git ai
cd ..
```

---

## 4. Configure & build

The fork's CMake helper handles the per-target LTO disable that ns-3.43 needs:

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build ai
```

Verify the bridge module is built:

```bash
./ns3 show profile | grep ai
ls build/contrib/ns3-ai-ntn/python/  # should show ns3ai_*.so files
```

---

## 5. Run examples

Each example pairs a C++ ns-3 binary with a Python driver.

### 5a. Hello-world (`a-plus-b`)

```bash
cd contrib/ns3-ai-ntn/examples/a-plus-b/use-gym/
python3 a-plus-b.py     # Python launches the ns-3 binary internally
```

Expected output: a stream of `(a, b, c=a+b)` triples.

### 5b. LTE CQI prediction

```bash
cd contrib/ns3-ai-ntn/examples/lte-cqi/
python3 run_baseline.py
python3 run_dqn.py --episodes=100
```

### 5c. Multi-BSS Wi-Fi RL

```bash
cd contrib/ns3-ai-ntn/examples/multi-bss/
python3 multi_bss.py --episodes=200
```

### 5d. RL-TCP

```bash
cd contrib/ns3-ai-ntn/examples/rl-tcp/
python3 run_rl_tcp.py
```

---

## 6. Drive a satellite RL workflow

Once `ntn-cho` and `oran-ntn` are also installed:

```bash
cd contrib/oran-ntn/python/
python3 train_ho_xapp.py --algo=dqn --episodes=200
```

This trains the HO-prediction xApp using the 68-feature observation vector that `ntn-cho` exposes via the ns3-ai shared-memory bridge.

---

## 7. Common issues

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

## 8. Citing

See [README](README.md#cite-this-work) — please cite both the original ns3-ai paper and this fork.
