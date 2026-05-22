#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2026 Muhammad Uzair
"""Lightweight UDP stub for ntn-sionna tests (Roadmap §4.2.1).

Mimics sionna-server.py's wire protocol but uses the closed-form FSPL so
the C++ tests don't need Sionna RT installed locally. The C++ side
verifies path-loss returned by this server matches NtnSionnaChannel's
FreeSpacePathLossDb() within 1e-6 dB, which is the same identity Sionna
RT converges to in an empty scene.
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import socket
import sys


def fspl_db(d: float, f_hz: float) -> float:
    f_ghz = f_hz / 1e9
    return 20.0 * math.log10(max(d, 1e-3)) + 20.0 * math.log10(f_ghz) + 32.45


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=0,
                   help="0 = bind to an ephemeral port")
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.host, args.port))
    bound = sock.getsockname()
    # The C++ test reads this line off stdout to learn the port.
    print(f"PORT={bound[1]}", flush=True)

    def _shutdown(_signo, _frame):
        try:
            sock.close()
        except OSError:
            pass
        sys.exit(0)
    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    while True:
        try:
            data, peer = sock.recvfrom(65536)
        except OSError:
            return 0
        try:
            req = json.loads(data.decode("utf-8"))
            tx = req["tx"]
            rx = req["rx"]
            d = math.dist(tx, rx)
            pl = fspl_db(d, float(req.get("freq_hz", 2.0e9)))
            rsp = {"id": req.get("id", 0), "path_loss_db": pl,
                   "n_paths": 1, "compute_ms": 0.05}
        except Exception as exc:  # malformed request etc.
            rsp = {"id": -1, "error": str(exc)}
        sock.sendto(json.dumps(rsp).encode("utf-8"), peer)


if __name__ == "__main__":
    sys.exit(main())
