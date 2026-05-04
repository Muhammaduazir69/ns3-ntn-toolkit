"""Slice-orchestrator xApp — drives W6's `SliceOrchestratorXapp` over E2.

Subscribes to per-slice load KPMs (``slice_demand_mbps``) and emits
`slice_prb_share` RC ControlRequests once per orchestrator tick. The
allocation policy here is the same min-throughput-first-then-priority
allocator as W6's built-in default; an RL policy trained against W4's
`SliceEnv` can replace `_decide_shares()` by reading a SB3 model.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from dataclasses import dataclass, field

from ntn_flexric_bridge import e2ap_codec, e2sm_rc_ntn
from ntn_flexric_bridge.xapps.xapp_base import XappBase

LOG = logging.getLogger("ntn-flexric-bridge.slice-orch-xapp")


@dataclass
class _SliceProfile:
    sst: int
    sd_hex: str
    min_mbps: float
    priority: int


# Default profiles match W6's TS 22.261-derived eMBB/URLLC/mMTC.
DEFAULT_PROFILES = [
    _SliceProfile(sst=1, sd_hex="ffffff", min_mbps=50.0, priority=5),
    _SliceProfile(sst=2, sd_hex="ffffff", min_mbps=5.0,  priority=9),
    _SliceProfile(sst=3, sd_hex="ffffff", min_mbps=1.0,  priority=1),
]


class SliceOrchXapp(XappBase):
    def __init__(self, gnb_id: int = 1, total_prb: int = 273,
                 mbps_per_prb: float = 0.6,
                 profiles: list[_SliceProfile] | None = None):
        super().__init__(name="slice-orch-xapp")
        self.gnb_id = gnb_id
        self.total_prb = total_prb
        self.mbps_per_prb = mbps_per_prb
        self.profiles = profiles or DEFAULT_PROFILES
        self._demand: dict[int, float] = {p.sst: 0.0 for p in self.profiles}
        self.allocations: list[dict] = []
        self.on_indication(self._on_kpm)

    def _on_kpm(self, header: dict, message: dict, _full: dict) -> None:
        # Expect measurements with ``measurement_type == "slice_demand_mbps"``
        # and ``slice_sst`` field carried via ue_imsi (workaround for the
        # sparse stub schema). Real FlexRIC carries SST as a tag.
        for m in message.get("measurements", []):
            if m.get("measurement_type") != "slice_demand_mbps":
                continue
            sst_str = m.get("ue_imsi") or "1"   # sst piggybacked on ue_imsi
            try:
                self._demand[int(sst_str)] = float(m["value"])
            except (KeyError, ValueError):
                continue
        self._tick()

    def _tick(self) -> None:
        shares = self._decide_shares()
        for prof, share in zip(self.profiles, shares):
            self._send_slice_prb(prof, share)
            self.allocations.append({
                "sst": prof.sst, "share": share, "at_seconds": time.time(),
            })

    def _decide_shares(self) -> list[float]:
        # Reserve mins, then distribute remainder by priority × unmet demand.
        cap_mbps = self.total_prb * self.mbps_per_prb
        reserved = []
        for prof in self.profiles:
            r = min(prof.min_mbps, self._demand.get(prof.sst, 0.0))
            reserved.append(r)
        remainder = max(0.0, cap_mbps - sum(reserved))
        weights = []
        for prof, r in zip(self.profiles, reserved):
            unmet = max(0.0, self._demand.get(prof.sst, 0.0) - r)
            weights.append(unmet * (1.0 + prof.priority))
        total_w = sum(weights)
        extras = [(remainder * w / total_w) if total_w > 1e-9 else 0.0
                  for w in weights]
        served = [r + e for r, e in zip(reserved, extras)]
        # Convert to PRB shares 0..1
        shares = [s / cap_mbps if cap_mbps > 0 else 0.0 for s in served]
        return shares

    def _send_slice_prb(self, prof: _SliceProfile, share: float) -> None:
        if self.sock is None:
            return
        self._instance_seq += 1
        ric_request_id = {"requestor_id": self.requestor_id,
                          "instance_id": self._instance_seq}
        header, message = e2sm_rc_ntn.build_slice_prb_share(
            e2sm_rc_ntn.RcControlHeader(gnb_id=self.gnb_id, ue_imsi="orch"),
            e2sm_rc_ntn.SlicePrbShareMessage(
                sst=prof.sst, sd_hex=prof.sd_hex, prb_share=share,
            ),
        )
        self.sock.sendall(e2ap_codec.encode_ric_control_request(
            ric_request_id, e2sm_rc_ntn.RAN_FUNCTION_ID,
            call_process_id=None, header=header, message=message, ack_request=False,
        ))
        self.stats.controls_sent += 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=36421)
    parser.add_argument("--seconds", type=float, default=60.0)
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    xapp = SliceOrchXapp()
    xapp.connect(args.host, args.port)
    xapp.subscribe_kpm(["slice_demand_mbps"], period_ms=1000)
    time.sleep(args.seconds)
    LOG.info("slice-orch-xapp stats=%s allocations=%d",
             xapp.stats, len(xapp.allocations))
    xapp.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
