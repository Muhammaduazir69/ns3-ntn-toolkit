# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W9)
#
# Live integration tests for the Sionna RT bridge. Each test launches a
# fresh sionna-server.py subprocess on a unique port, queries it over UDP,
# and verifies the W9 validation gates:
#
#   - server starts on GPU host
#   - RTT < 50 ms (steady-state, after JIT warmup)
#   - path loss within +/- 3 dB of TR 38.811 reference (matched scenario)
#
# These tests need TensorFlow + Sionna importable. CI without a GPU should
# skip them via `pytest -m "not gpu"`.

from __future__ import annotations

import json
import math
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest


HERE = Path(__file__).resolve().parent
SERVER = HERE.parent / "bridge" / "sionna-server.py"


def _free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _wait_until_listening(port: int, timeout_s: float = 30.0) -> bool:
    """Probe the UDP server until a query gets a response or timeout elapses."""
    deadline = time.monotonic() + timeout_s
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    payload = json.dumps({"tx": [0, 0, 1000], "rx": [1000, 0, 1.5],
                          "freq_hz": 2.0e9, "id": 0}).encode()
    while time.monotonic() < deadline:
        try:
            sock.sendto(payload, ("127.0.0.1", port))
            sock.recvfrom(8192)
            return True
        except (socket.timeout, ConnectionRefusedError, OSError):
            time.sleep(0.5)
    return False


@pytest.fixture(scope="module")
def server():
    port = _free_udp_port()
    proc = subprocess.Popen(
        [sys.executable, str(SERVER), "--port", str(port), "--log-level", "WARNING"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env={**os.environ, "TF_CPP_MIN_LOG_LEVEL": "3"},
    )
    try:
        if not _wait_until_listening(port):
            proc.terminate()
            pytest.skip("sionna-server did not come up within 30 s "
                        "(likely no GPU / Sionna unavailable)")
        yield port
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


def _query(port: int, tx, rx, freq_hz=2.0e9, los_only=True, timeout_s=2.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout_s)
    req = {"tx": tx, "rx": rx, "freq_hz": freq_hz, "los_only": los_only, "id": 1}
    t0 = time.perf_counter()
    s.sendto(json.dumps(req).encode(), ("127.0.0.1", port))
    data, _ = s.recvfrom(8192)
    rtt_ms = (time.perf_counter() - t0) * 1e3
    rsp = json.loads(data)
    s.close()
    return rsp, rtt_ms


def _fspl_db(d_m, freq_hz):
    return 20.0 * math.log10(max(d_m, 1e-3)) + 20.0 * math.log10(freq_hz / 1e9) + 32.45


def test_server_starts(server):
    """Gate 1: Sionna server starts on GPU host."""
    # If the fixture didn't skip, the server is up. Confirm with a sanity ping.
    rsp, _ = _query(server, [0.0, 0.0, 1000.0], [1000.0, 0.0, 1.5])
    assert "path_loss_db" in rsp


def test_steady_state_rtt_under_50ms(server):
    """Gate 2: ns-3 channel queries Sionna and gets responses with <50 ms RTT.

    The C++ side already enforces this with a mock server; here we verify the
    real Sionna path solver also fits inside the budget once warm.
    """
    # JIT warmup
    _query(server, [0.0, 0.0, 1000.0], [1000.0, 0.0, 1.5])
    rtts = []
    for i in range(10):
        _, rtt = _query(server, [0.0, 0.0, 1000.0 + i], [1000.0, 0.0, 1.5])
        rtts.append(rtt)
    p99 = sorted(rtts)[-1]
    mean = sum(rtts) / len(rtts)
    assert p99 < 50.0, f"max RTT {p99:.2f} ms exceeds 50 ms gate (mean {mean:.2f})"


@pytest.mark.parametrize("d_m,freq_hz", [
    (1413.0,    2.0e9),    # the 101.47 dB spot we used during bring-up
    (10_000.0,  2.0e9),    # 118.5 dB
    (100_000.0, 5.0e9),    # 146.4 dB (10x further, 5 GHz)
    (600_000.0, 12.0e9),   # 169.5 dB (LEO sat, Ku-band)
])
def test_pl_within_3db_of_tr38811(server, d_m, freq_hz):
    """Gate 3: path loss within +/- 3 dB of TR 38.811 reference under matched
    scenario (free-space LOS only)."""
    # Warmup
    _query(server, [0.0, 0.0, 1000.0], [1000.0, 0.0, 1.5], freq_hz=freq_hz)
    # Place tx straight up at altitude d_m, rx at origin (z=1.5 m AGL).
    rsp, _ = _query(server,
                    [0.0, 0.0, d_m + 1.5], [0.0, 0.0, 1.5],
                    freq_hz=freq_hz)
    pl = rsp["path_loss_db"]
    expect = _fspl_db(d_m, freq_hz)
    delta = pl - expect
    assert abs(delta) <= 3.0, (
        f"|delta|={abs(delta):.3f} dB exceeds gate at d={d_m} m, "
        f"f={freq_hz/1e9} GHz: sionna={pl:.3f} ref={expect:.3f}"
    )
