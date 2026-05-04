"""E2 Agent process running alongside ns-3.

Connects to the Near-RT RIC (FlexRIC in live mode, this process's own
loopback in stub mode), performs E2 Setup, accepts RIC Subscriptions,
and forwards RIC Indications. Receives RIC Control Requests from xApps
and emits NTN-specific actions (CHO trigger / beam select / slice PRB).

In **stub mode** the agent listens on a TCP socket; xApps connect and
the wire is JSON-framed (see `e2ap_codec`). In **live mode** the agent
opens an SCTP association to FlexRIC port 36421 and uses the asn1c
codec (require the Docker image — see `docs/BUILD_FLEXRIC.md`).

The agent exposes a small "data plane" that callers (e.g. ns-3 example
scripts) push KPM measurements into via `submit_kpm_measurement()`.
"""

from __future__ import annotations

import argparse
import logging
import socket
import socketserver
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable

from ntn_flexric_bridge import e2ap_codec, e2sm_kpm_ntn, e2sm_rc_ntn

LOG = logging.getLogger("ntn-flexric-bridge.e2-agent")


@dataclass
class AgentStats:
    setup_done: bool = False
    subscriptions: int = 0
    indications_sent: int = 0
    controls_received: int = 0
    controls_executed: int = 0
    started_iso: str = ""


@dataclass
class _Subscription:
    ric_request_id: dict
    ran_function_id: int
    actions: list[dict]
    event_trigger: dict
    indication_sn: int = 0


class E2AgentNs3:
    """E2 agent state machine + TCP server for stub-mode wire exchange."""

    def __init__(self, gnb_id: int, plmn: str = "001/01"):
        self.gnb_id = gnb_id
        self.plmn = plmn
        self.stats = AgentStats()
        self._subs: dict[tuple[int, int, int], _Subscription] = {}
        self._control_handlers: dict[str, Callable[[dict, dict], dict]] = {}
        self._client: socket.socket | None = None
        self._lock = threading.Lock()
        self._kpm_buffer: deque[dict] = deque(maxlen=10000)
        self._handover_log: list[dict] = []
        # Default control handler: log + record handovers for the equivalence test.
        self.register_control_handler("cho_trigger", self._default_cho_handler)

    # ------------------------------------------------------------------ public
    def register_control_handler(self, action: str,
                                 handler: Callable[[dict, dict], dict]) -> None:
        self._control_handlers[action] = handler

    def submit_kpm_measurement(self, header: e2sm_kpm_ntn.KpmIndicationHeader,
                               message: e2sm_kpm_ntn.KpmIndicationMessage) -> None:
        """Push a KPM measurement that all matching subscribers will receive."""
        with self._lock:
            for sub in self._subs.values():
                if sub.ran_function_id != e2sm_kpm_ntn.RAN_FUNCTION_ID:
                    continue
                sub.indication_sn += 1
                hdr, msg = e2sm_kpm_ntn.build_indication(header, message)
                frame = e2ap_codec.encode_ric_indication(
                    sub.ric_request_id, sub.ran_function_id,
                    action_id=sub.actions[0]["action_id"] if sub.actions else 0,
                    indication_sn=sub.indication_sn,
                    indication_type="report",
                    header=hdr, message=msg,
                )
                self._send_frame(frame)
                self.stats.indications_sent += 1

    def get_handover_log(self) -> list[dict]:
        with self._lock:
            return list(self._handover_log)

    # ------------------------------------------------------------------ wire
    def _send_frame(self, frame: bytes) -> None:
        if self._client is None:
            return
        try:
            self._client.sendall(frame)
        except OSError as exc:
            LOG.warning("xApp socket gone (%s); dropping frame", exc)
            self._client = None

    def _default_cho_handler(self, header: dict, message: dict) -> dict:
        """Built-in handler — logs the handover for the equivalence test."""
        with self._lock:
            self._handover_log.append({
                "ue_imsi": header.get("ue_imsi"),
                "target_cell_id": message.get("target_cell_id"),
                "target_sat_norad": message.get("target_sat_norad"),
                "expected_tta_ms": message.get("expected_tta_ms"),
                "executed_at": time.time(),
            })
        return {"outcome": "executed"}

    # ------------------------------------------------------------------ server
    def run_stub_server(self, host: str = "127.0.0.1", port: int = 36421,
                        forever: bool = True) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, port))
        srv.listen(4)
        self.stats.started_iso = time.strftime("%Y-%m-%dT%H:%M:%S")
        LOG.info("E2 agent listening on %s:%d (stub TCP, JSON wire)", host, port)
        while True:
            client, addr = srv.accept()
            LOG.info("xApp connected: %s", addr)
            self._client = client
            try:
                self._serve_one(client)
            except Exception as exc:  # noqa: BLE001
                LOG.warning("client %s errored: %s", addr, exc)
            finally:
                self._client = None
                client.close()
                if not forever:
                    srv.close()
                    return

    def _serve_one(self, client: socket.socket) -> None:
        while True:
            msg = e2ap_codec.read_frame(client)
            if msg is None:
                return
            self._handle_message(msg)

    def _handle_message(self, msg: dict) -> None:
        proc = e2ap_codec.ProcedureCode(msg["procedure_code"])
        kind = e2ap_codec.MessageType(msg["type"])
        v = msg["value"]
        if proc == e2ap_codec.ProcedureCode.E2_SETUP:
            self._handle_setup(v)
        elif proc == e2ap_codec.ProcedureCode.RIC_SUBSCRIPTION:
            self._handle_subscription(v)
        elif proc == e2ap_codec.ProcedureCode.RIC_CONTROL:
            self._handle_control(v)
        else:
            LOG.warning("unhandled procedure %s", proc)

    def _handle_setup(self, v: dict) -> None:
        # Respond with E2SetupResponse accepting all advertised RAN functions.
        self.stats.setup_done = True
        accepted = [int(rf["ran_function_id"]) for rf in v.get("ran_function_list", [])]
        resp = e2ap_codec.encode_e2_setup_response(transaction_id=1,
                                                    accepted_ran_function_ids=accepted)
        self._send_frame(resp)

    def _handle_subscription(self, v: dict) -> None:
        ric_request_id = v["ric_request_id"]
        rf = v["ran_function_id"]
        sub = _Subscription(ric_request_id=ric_request_id, ran_function_id=rf,
                            actions=v.get("actions", []),
                            event_trigger=v.get("event_trigger_definition", {}))
        key = (ric_request_id["requestor_id"], ric_request_id["instance_id"], rf)
        self._subs[key] = sub
        self.stats.subscriptions += 1
        admitted = [{"action_id": a["action_id"]} for a in sub.actions]
        resp = e2ap_codec.encode_subscription_response(ric_request_id, rf, admitted)
        self._send_frame(resp)

    def _handle_control(self, v: dict) -> None:
        self.stats.controls_received += 1
        header = v["ric_control_header"]
        message = v["ric_control_message"]
        action = message.get("action")
        h = self._control_handlers.get(action)
        if h is None:
            LOG.warning("no control handler registered for action=%s", action)
            return
        outcome = h(header, message)
        self.stats.controls_executed += 1
        if v.get("ric_control_ack_request") == "ack":
            ack = e2ap_codec.encode_ric_control_acknowledge(
                v["ric_request_id"], v["ran_function_id"],
                bytes.fromhex(v["ric_call_process_id"]) if v.get("ric_call_process_id") else None,
                outcome,
            )
            self._send_frame(ack)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gnb-id", type=int, default=1)
    parser.add_argument("--port", type=int, default=36421)
    parser.add_argument("--host", default="127.0.0.1")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    agent = E2AgentNs3(gnb_id=args.gnb_id)
    agent.run_stub_server(host=args.host, port=args.port)
    return 0


if __name__ == "__main__":
    sys.exit(main())
