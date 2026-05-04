"""Common scaffold shared by every shipped xApp.

The base class:
  - opens an SCTP / TCP connection to the RIC,
  - performs E2 Setup,
  - keeps a request-id counter so subsequent subscriptions/controls don't collide,
  - dispatches RIC Indications to a user-supplied callback,
  - exposes a `send_control()` helper.
"""

from __future__ import annotations

import logging
import socket
import threading
from dataclasses import dataclass, field
from typing import Callable

from ntn_flexric_bridge import e2ap_codec, e2sm_kpm_ntn, e2sm_rc_ntn

LOG = logging.getLogger("ntn-flexric-bridge.xapp")


@dataclass
class XappStats:
    setup_done: bool = False
    indications_received: int = 0
    controls_sent: int = 0
    controls_acked: int = 0


class XappBase:
    def __init__(self, name: str, requestor_id: int = 0xC0DE):
        self.name = name
        self.requestor_id = requestor_id
        self._instance_seq = 0
        self.sock: socket.socket | None = None
        self.stats = XappStats()
        self._on_indication: Callable[[dict, dict, dict], None] = lambda *_: None
        self._control_acks: list[dict] = []
        self._reader_thread: threading.Thread | None = None
        self._stop = threading.Event()

    def connect(self, host: str = "127.0.0.1", port: int = 36421) -> None:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((host, port))
        self.sock = s
        # E2 Setup: advertise both RAN functions we use.
        ran_functions = [
            {"ran_function_id": e2sm_kpm_ntn.RAN_FUNCTION_ID,
             "ran_function_revision": 3,
             "ran_function_definition": "E2SM-KPM-NTN/v3"},
            {"ran_function_id": e2sm_rc_ntn.RAN_FUNCTION_ID,
             "ran_function_revision": 1,
             "ran_function_definition": "E2SM-RC-NTN/v1.03"},
        ]
        s.sendall(e2ap_codec.encode_e2_setup_request(
            global_e2_node_id=1, plmn_id="001/01",
            ran_function_list=ran_functions,
        ))
        # Wait for setup response then start reader.
        msg = e2ap_codec.read_frame(s)
        if msg is None or msg["procedure_code"] != int(e2ap_codec.ProcedureCode.E2_SETUP):
            raise RuntimeError("E2 setup failed")
        self.stats.setup_done = True
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def subscribe_kpm(self, measurement_types: list[str], period_ms: int = 1000) -> None:
        if self.sock is None:
            raise RuntimeError("xApp not connected")
        self._instance_seq += 1
        ric_request_id = {"requestor_id": self.requestor_id,
                          "instance_id": self._instance_seq}
        action = e2sm_kpm_ntn.action_report(action_id=1,
                                             measurement_types=measurement_types)
        trigger = e2sm_kpm_ntn.event_trigger_periodic(period_ms)
        self.sock.sendall(e2ap_codec.encode_subscription_request(
            ric_request_id, e2sm_kpm_ntn.RAN_FUNCTION_ID,
            event_trigger=trigger, actions=[action],
        ))

    def send_cho_trigger(self, gnb_id: int, ue_imsi: str, target_cell_id: str,
                          target_sat_norad: int | None = None,
                          expected_tta_ms: float = 0.0) -> None:
        if self.sock is None:
            raise RuntimeError("xApp not connected")
        self._instance_seq += 1
        ric_request_id = {"requestor_id": self.requestor_id,
                          "instance_id": self._instance_seq}
        header, message = e2sm_rc_ntn.build_cho_trigger(
            e2sm_rc_ntn.RcControlHeader(gnb_id=gnb_id, ue_imsi=ue_imsi),
            e2sm_rc_ntn.ChoTriggerMessage(
                target_cell_id=target_cell_id,
                target_sat_norad=target_sat_norad,
                expected_tta_ms=expected_tta_ms,
            ),
        )
        self.sock.sendall(e2ap_codec.encode_ric_control_request(
            ric_request_id, e2sm_rc_ntn.RAN_FUNCTION_ID,
            call_process_id=None, header=header, message=message, ack_request=True,
        ))
        self.stats.controls_sent += 1

    def on_indication(self, callback: Callable[[dict, dict, dict], None]) -> None:
        self._on_indication = callback

    def stop(self) -> None:
        self._stop.set()
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.sock.close()

    # ------------------------------------------------------------------ reader
    def _reader_loop(self) -> None:
        while not self._stop.is_set() and self.sock is not None:
            try:
                msg = e2ap_codec.read_frame(self.sock)
            except OSError:
                return
            if msg is None:
                return
            proc = e2ap_codec.ProcedureCode(msg["procedure_code"])
            kind = e2ap_codec.MessageType(msg["type"])
            v = msg["value"]
            if proc == e2ap_codec.ProcedureCode.RIC_INDICATION:
                self.stats.indications_received += 1
                self._on_indication(
                    v.get("ric_indication_header", {}),
                    v.get("ric_indication_message", {}),
                    v,
                )
            elif (proc == e2ap_codec.ProcedureCode.RIC_CONTROL
                  and kind == e2ap_codec.MessageType.SUCCESSFUL_OUTCOME):
                self.stats.controls_acked += 1
                self._control_acks.append(v.get("ric_control_outcome", {}))
