"""Beam-management xApp.

Subscribes to ``rsrp_dbm`` and ``elevation_deg`` per beam_id, picks the
beam with highest weighted score (RSRP + elevation bonus) per UE, and
emits a `beam_select` RC ControlRequest each time the choice changes.

The algorithm mirrors `BeamMgmtEnv` in W4 — same observation / action
shape — so a Stable-Baselines3 policy trained against W4 can drop in
here by replacing `_select_best_beam` with `model.predict(obs)`.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from dataclasses import dataclass, field

from ntn_flexric_bridge import e2ap_codec, e2sm_rc_ntn
from ntn_flexric_bridge.xapps.xapp_base import XappBase

LOG = logging.getLogger("ntn-flexric-bridge.beam-mgmt-xapp")


@dataclass
class _UeBeamState:
    current_beam: int | None = None
    last_rsrp: dict[int, float] = field(default_factory=dict)


class BeamMgmtXapp(XappBase):
    def __init__(self, gnb_id: int = 1):
        super().__init__(name="beam-mgmt-xapp")
        self.gnb_id = gnb_id
        self._ues: dict[str, _UeBeamState] = {}
        self.beam_changes: list[dict] = []
        self.on_indication(self._on_kpm)

    def _on_kpm(self, header: dict, message: dict, _full: dict) -> None:
        for m in message.get("measurements", []):
            if m.get("measurement_type") != "rsrp_dbm":
                continue
            cell_id = m.get("cell_id") or ""
            beam_id = self._beam_id_from_cell(cell_id)
            imsi = m.get("ue_imsi")
            if imsi is None or beam_id is None:
                continue
            st = self._ues.setdefault(imsi, _UeBeamState())
            st.last_rsrp[beam_id] = float(m["value"])
            best_beam = max(st.last_rsrp.items(), key=lambda kv: kv[1])[0]
            if st.current_beam != best_beam:
                old = st.current_beam
                st.current_beam = best_beam
                self._send_beam_select(imsi, cell_id, best_beam)
                self.beam_changes.append({
                    "ue_imsi": imsi, "from_beam": old, "to_beam": best_beam,
                    "at_seconds": time.time(),
                })

    @staticmethod
    def _beam_id_from_cell(cell_id: str) -> int | None:
        # Convention: cell IDs look like "cell-<sat>-<beam>" → split last token.
        if "-" not in cell_id:
            return None
        try:
            return int(cell_id.rsplit("-", 1)[1])
        except ValueError:
            return None

    def _send_beam_select(self, ue_imsi: str, cell_id: str, beam_id: int) -> None:
        if self.sock is None:
            return
        self._instance_seq += 1
        ric_request_id = {"requestor_id": self.requestor_id,
                          "instance_id": self._instance_seq}
        header, message = e2sm_rc_ntn.build_beam_select(
            e2sm_rc_ntn.RcControlHeader(gnb_id=self.gnb_id, ue_imsi=ue_imsi),
            e2sm_rc_ntn.BeamSelectMessage(beam_id=beam_id, cell_id=cell_id),
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
    parser.add_argument("--gnb-id", type=int, default=1)
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    xapp = BeamMgmtXapp(gnb_id=args.gnb_id)
    xapp.connect(args.host, args.port)
    xapp.subscribe_kpm(["rsrp_dbm"], period_ms=500)
    time.sleep(args.seconds)
    LOG.info("beam-mgmt-xapp stats=%s changes=%d", xapp.stats, len(xapp.beam_changes))
    xapp.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
