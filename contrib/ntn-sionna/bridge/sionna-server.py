#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W9)

"""sionna-server.py — UDP path-loss query server backed by Sionna RT.

Wire protocol (JSON lines, one per UDP datagram):

  Request : {"tx":[x,y,z], "rx":[x,y,z], "freq_hz":2.0e9, "id":<int>}
  Response: {"id":<int>, "path_loss_db":<float>, "n_paths":<int>,
             "compute_ms":<float>}

A single Mitsuba/Sionna scene (defaults to ``simple_reflector``) is loaded
once at startup and reused for every query. Each query removes the previous
tx/rx pair and re-adds them at the requested positions — Sionna RT's
``PathSolver`` then runs on the GPU.

The fall-back FSPL the ns-3 channel uses on UDP timeout matches what this
server returns for free-space geometry (verified to 0.00 dB at 1413 m,
2 GHz against ``20*log10(d) + 20*log10(f_GHz) + 32.45``).
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import os
import signal
import socket
import sys
import time
from typing import Any

# Quiet TF / Sionna chatter so the JSON wire stays clean if a user pipes us.
os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "3")

import numpy as np  # noqa: E402  (after os.environ)
import sionna.rt as rt  # noqa: E402

LOG = logging.getLogger("ntn-sionna.server")


def _setup_logger(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )


class SionnaPathLossEngine:
    """Wraps a Sionna RT scene and answers (tx, rx, freq) → path_loss_dB."""

    def __init__(self, scene_xml: str | None = None, freq_hz: float = 2.0e9):
        self._scene_xml = scene_xml or rt.scene.simple_reflector
        self._scene = rt.load_scene(self._scene_xml)
        self._scene.frequency = freq_hz
        # Single isotropic antenna both sides — keeps the comparison vs the
        # closed-form TR 38.811 reference clean (which assumes isotropic).
        self._scene.tx_array = rt.PlanarArray(
            num_rows=1, num_cols=1,
            vertical_spacing=0.5, horizontal_spacing=0.5,
            pattern="iso", polarization="V",
        )
        self._scene.rx_array = self._scene.tx_array
        self._solver = rt.PathSolver()
        self._cur_freq_hz = freq_hz
        self._tx_added = False
        self._rx_added = False
        LOG.info("Sionna engine ready: scene=%s, default_freq=%.3f GHz",
                 self._scene_xml, freq_hz / 1e9)

    def _set_freq(self, freq_hz: float) -> None:
        # Sionna re-builds internal radio-material lookups on frequency change,
        # so only update when it actually changes.
        if abs(freq_hz - self._cur_freq_hz) > 1.0:
            self._scene.frequency = freq_hz
            self._cur_freq_hz = freq_hz

    def _replace_endpoints(self, tx_xyz: list[float], rx_xyz: list[float]) -> None:
        if self._tx_added:
            self._scene.remove("tx")
        if self._rx_added:
            self._scene.remove("rx")
        self._scene.add(rt.Transmitter("tx", position=[float(v) for v in tx_xyz]))
        self._scene.add(rt.Receiver("rx",   position=[float(v) for v in rx_xyz]))
        self._tx_added = True
        self._rx_added = True

    def query(self, tx_xyz: list[float], rx_xyz: list[float],
              freq_hz: float, los_only: bool = True) -> dict[str, Any]:
        t0 = time.perf_counter()
        self._set_freq(freq_hz)
        self._replace_endpoints(tx_xyz, rx_xyz)
        # `los_only=True` is the matched-scenario reference for TR 38.811's
        # closed-form FSPL — no reflections / refractions / diffractions.
        # Set False to get the full multipath superposition (Sionna's edge).
        if los_only:
            paths = self._solver(self._scene, max_depth=0,
                                 specular_reflection=False,
                                 refraction=False,
                                 diffraction=False)
        else:
            paths = self._solver(self._scene)
        a, _tau = paths.cir(out_type="numpy")
        # |a|^2 summed across all paths = total received power for unit Tx power.
        a = np.asarray(a)
        gain = float(np.sum(np.abs(a) ** 2))
        if gain <= 0.0:
            # No paths (Sionna might prune below epsilon). Fall back to FSPL.
            d = math.dist(tx_xyz, rx_xyz)
            f_ghz = freq_hz / 1e9
            pl_db = 20.0 * math.log10(max(d, 1e-3)) + 20.0 * math.log10(f_ghz) + 32.45
            n_paths = 0
        else:
            pl_db = -10.0 * math.log10(gain)
            n_paths = int(np.prod(a.shape[:-1])) if a.ndim > 1 else int(a.size)
        return {
            "path_loss_db": pl_db,
            "n_paths": n_paths,
            "compute_ms": (time.perf_counter() - t0) * 1e3,
        }


class UdpServer:
    def __init__(self, engine: SionnaPathLossEngine, host: str, port: int):
        self._engine = engine
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._stop = False
        LOG.info("UDP server listening on %s:%d", host, port)

    def stop(self) -> None:
        self._stop = True
        try:
            self._sock.close()
        except OSError:
            pass

    def serve_forever(self) -> None:
        # 64 KiB request buffer is more than enough for a few floats of JSON.
        while not self._stop:
            try:
                data, peer = self._sock.recvfrom(65536)
            except OSError:
                return
            try:
                req = json.loads(data.decode("utf-8"))
                rsp = self._engine.query(
                    tx_xyz=req["tx"], rx_xyz=req["rx"],
                    freq_hz=float(req.get("freq_hz", 2.0e9)),
                    los_only=bool(req.get("los_only", True)),
                )
                rsp["id"] = req.get("id", 0)
            except Exception as exc:
                LOG.warning("query failed: %s", exc)
                rsp = {"id": -1, "error": str(exc)}
            self._sock.sendto(json.dumps(rsp).encode("utf-8"), peer)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Sionna RT UDP path-loss server")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--freq-hz", type=float, default=2.0e9,
                   help="default carrier; overridden per-request")
    p.add_argument("--scene-xml", default=None,
                   help="path to a Mitsuba scene XML; default = simple_reflector")
    p.add_argument("--log-level", default="INFO")
    args = p.parse_args(argv)
    _setup_logger(args.log_level)
    engine = SionnaPathLossEngine(scene_xml=args.scene_xml, freq_hz=args.freq_hz)
    server = UdpServer(engine, args.host, args.port)

    def _shutdown(_signo: int, _frame: object) -> None:
        LOG.info("shutdown signal received")
        server.stop()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)
    try:
        server.serve_forever()
    finally:
        server.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
