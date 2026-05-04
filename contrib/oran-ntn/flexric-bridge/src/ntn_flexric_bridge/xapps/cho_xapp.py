"""TTE-aware Conditional Handover xApp.

Subscribes to ``elevation_deg`` + ``rsrp_dbm`` + ``tte_total_us`` KPMs and
issues a `CHO trigger` when the serving cell's elevation falls below a
threshold AND a candidate cell's elevation rises above it. Mirrors the
in-memory CHO algorithm in ``contrib/oran-ntn/model/oran-ntn-near-rt-ric.h``
so the W8 equivalence test passes.

Algorithm (TTE-aware):
  1. Maintain per-UE last-known elevation per (ue_imsi, sat_norad).
  2. When a serving sat's elevation < ``handover_below_deg``, scan for a
     candidate sat whose elevation > ``handover_above_deg``.
  3. If found, emit a `cho_trigger` with `expected_tta_ms` derived from
     the candidate's TTE measurement.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from dataclasses import dataclass, field
from typing import Any

from ntn_flexric_bridge.xapps.xapp_base import XappBase

LOG = logging.getLogger("ntn-flexric-bridge.cho-xapp")


@dataclass
class ChoConfig:
    handover_below_deg: float = 30.0    # serving sat below this → consider HO
    handover_above_deg: float = 35.0    # candidate must be above this
    hysteresis_db: float = 3.0          # candidate RSRP must beat serving by this
    gnb_id: int = 1


@dataclass
class _UeState:
    serving_sat: int | None = None
    serving_cell: str | None = None
    last_elevation: dict[int, float] = field(default_factory=dict)
    last_rsrp: dict[int, float] = field(default_factory=dict)
    last_tte_us: dict[int, float] = field(default_factory=dict)


class ChoXapp(XappBase):
    def __init__(self, cfg: ChoConfig | None = None):
        super().__init__(name="cho-xapp")
        self.cfg = cfg or ChoConfig()
        self._ues: dict[str, _UeState] = {}
        self.handovers: list[dict] = []
        self.on_indication(self._on_kpm)

    def _ue(self, imsi: str) -> _UeState:
        if imsi not in self._ues:
            self._ues[imsi] = _UeState()
        return self._ues[imsi]

    def _on_kpm(self, header: dict, message: dict, _full: dict) -> None:
        sat = header.get("sat_norad")
        if sat is None:
            return
        # Record all measurements first, then evaluate handover ONCE at the
        # end. Otherwise a TTE measurement arriving after RSRP/elevation in
        # the same indication would be missed (handover already fired).
        touched_ues: set[str] = set()
        for m in message.get("measurements", []):
            imsi = m.get("ue_imsi")
            if not imsi:
                continue
            st = self._ue(imsi)
            mt = m.get("measurement_type")
            if mt == "elevation_deg":
                st.last_elevation[int(sat)] = float(m["value"])
            elif mt == "rsrp_dbm":
                st.last_rsrp[int(sat)] = float(m["value"])
            elif mt == "tte_total_us":
                st.last_tte_us[int(sat)] = float(m["value"])
            cell = m.get("cell_id")
            if cell and st.serving_sat == int(sat):
                st.serving_cell = cell
            touched_ues.add(imsi)
        for imsi in touched_ues:
            self._maybe_handover(imsi, self._ue(imsi),
                                 sat_norad_observed=int(sat))

    def _maybe_handover(self, imsi: str, st: _UeState, sat_norad_observed: int) -> None:
        # Initial attach: take first observed sat as serving.
        if st.serving_sat is None:
            st.serving_sat = sat_norad_observed
            return
        serv = st.serving_sat
        serv_el = st.last_elevation.get(serv, -90.0)
        serv_rsrp = st.last_rsrp.get(serv, -200.0)
        if serv_el >= self.cfg.handover_below_deg:
            return
        # Find candidate sat with high elevation AND better RSRP.
        best = None
        for cand_sat, cand_el in st.last_elevation.items():
            if cand_sat == serv:
                continue
            if cand_el < self.cfg.handover_above_deg:
                continue
            cand_rsrp = st.last_rsrp.get(cand_sat, -200.0)
            if cand_rsrp <= serv_rsrp + self.cfg.hysteresis_db:
                continue
            if best is None or cand_rsrp > best[1]:
                best = (cand_sat, cand_rsrp, cand_el)
        if best is None:
            return
        cand_sat, _cand_rsrp, _cand_el = best
        cand_tte_us = st.last_tte_us.get(cand_sat, 0.0)
        target_cell = f"cell-{cand_sat}"
        self.send_cho_trigger(
            gnb_id=self.cfg.gnb_id, ue_imsi=imsi,
            target_cell_id=target_cell, target_sat_norad=cand_sat,
            expected_tta_ms=cand_tte_us / 1000.0,
        )
        self.handovers.append({
            "ue_imsi": imsi,
            "from_sat": serv,
            "to_sat": cand_sat,
            "target_cell": target_cell,
            "expected_tta_ms": cand_tte_us / 1000.0,
            "at_seconds": time.time(),
        })
        st.serving_sat = cand_sat
        st.serving_cell = target_cell


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=36421)
    parser.add_argument("--below", type=float, default=30.0)
    parser.add_argument("--above", type=float, default=35.0)
    parser.add_argument("--hysteresis-db", type=float, default=3.0)
    parser.add_argument("--gnb-id", type=int, default=1)
    parser.add_argument("--seconds", type=float, default=60.0,
                        help="how long to run before exiting")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    cfg = ChoConfig(
        handover_below_deg=args.below,
        handover_above_deg=args.above,
        hysteresis_db=args.hysteresis_db,
        gnb_id=args.gnb_id,
    )
    xapp = ChoXapp(cfg)
    xapp.connect(args.host, args.port)
    xapp.subscribe_kpm(["elevation_deg", "rsrp_dbm", "tte_total_us"],
                       period_ms=1000)
    LOG.info("cho-xapp listening %.1fs ...", args.seconds)
    time.sleep(args.seconds)
    LOG.info("cho-xapp stats=%s handovers=%d",
             xapp.stats, len(xapp.handovers))
    xapp.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
