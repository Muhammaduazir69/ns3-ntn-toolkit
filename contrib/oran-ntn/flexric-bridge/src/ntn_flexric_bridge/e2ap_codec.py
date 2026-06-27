"""Minimal E2AP message codec.

Only **stub mode** is implemented here: the wire format is JSON framed by a
4-byte big-endian length prefix. Each message has the same *structural*
fields as asn1c-compiled E2AP (procedure code, criticality, value), so
round-trip tests and state machines line up at the field level — but the
JSON values are NOT ASN.1-PER bits, so this does not interoperate with a
real FlexRIC/asn1c peer.

The **live mode** referenced throughout is roadmapped, not built: there is
no asn1c-generated encoder and no `cffi` shim in this repo. The intent is
for live mode to hand bytes to FlexRIC's asn1c codec (to be provided by the
Docker image); until that exists, only the JSON loopback runs.
"""

from __future__ import annotations

import enum
import json
import struct
from typing import Any

# E2AP procedure codes per O-RAN.WG3.E2AP-v01.01, table 9.1.4-1.
class ProcedureCode(enum.IntEnum):
    E2_SETUP = 1
    RIC_SUBSCRIPTION = 8
    RIC_SUBSCRIPTION_DELETE = 9
    RIC_INDICATION = 5
    RIC_CONTROL = 4
    E2_CONNECTION_UPDATE = 10
    RESET = 7


class Criticality(enum.IntEnum):
    REJECT = 0
    IGNORE = 1
    NOTIFY = 2


class MessageType(enum.IntEnum):
    INITIATING_MESSAGE = 0
    SUCCESSFUL_OUTCOME = 1
    UNSUCCESSFUL_OUTCOME = 2


def encode_message(procedure: ProcedureCode, kind: MessageType,
                   payload: dict[str, Any]) -> bytes:
    """Frame a message as <length:u32 BE><JSON>."""
    body = {
        "procedure_code": int(procedure),
        "type": int(kind),
        "criticality": int(Criticality.REJECT if procedure in (
            ProcedureCode.E2_SETUP, ProcedureCode.RIC_SUBSCRIPTION,
            ProcedureCode.RIC_CONTROL,
        ) else Criticality.IGNORE),
        "value": payload,
    }
    raw = json.dumps(body, separators=(",", ":"), sort_keys=True).encode("utf-8")
    return struct.pack(">I", len(raw)) + raw


def decode_message(buf: bytes) -> dict[str, Any]:
    """Inverse of `encode_message`. Raises on truncation / malformed frames."""
    if len(buf) < 4:
        raise ValueError("frame too short for length prefix")
    n = struct.unpack(">I", buf[:4])[0]
    if len(buf) < 4 + n:
        raise ValueError(f"frame truncated: header says {n} bytes, got {len(buf)-4}")
    return json.loads(buf[4 : 4 + n].decode("utf-8"))


def read_frame(reader) -> dict[str, Any] | None:
    """Read one frame from a blocking socket-like object. None on EOF."""
    header = b""
    while len(header) < 4:
        chunk = reader.recv(4 - len(header))
        if not chunk:
            return None
        header += chunk
    n = struct.unpack(">I", header)[0]
    body = b""
    while len(body) < n:
        chunk = reader.recv(n - len(body))
        if not chunk:
            return None
        body += chunk
    return json.loads(body.decode("utf-8"))


# ============================================================================
# Message constructors — one per procedure we care about.
# ============================================================================

def encode_e2_setup_request(global_e2_node_id: int, plmn_id: str,
                             ran_function_list: list[dict]) -> bytes:
    """E2AP §9.1.1 E2SetupRequest."""
    return encode_message(
        ProcedureCode.E2_SETUP, MessageType.INITIATING_MESSAGE,
        {
            "global_e2_node_id": global_e2_node_id,
            "plmn_identity": plmn_id,
            "ran_function_list": ran_function_list,
        },
    )


def encode_e2_setup_response(transaction_id: int,
                              accepted_ran_function_ids: list[int]) -> bytes:
    return encode_message(
        ProcedureCode.E2_SETUP, MessageType.SUCCESSFUL_OUTCOME,
        {
            "transaction_id": transaction_id,
            "accepted_ran_function_ids": accepted_ran_function_ids,
        },
    )


def encode_subscription_request(ric_request_id: dict, ran_function_id: int,
                                event_trigger: dict, actions: list[dict]) -> bytes:
    """E2AP §9.1.5 RICSubscriptionRequest."""
    return encode_message(
        ProcedureCode.RIC_SUBSCRIPTION, MessageType.INITIATING_MESSAGE,
        {
            "ric_request_id": ric_request_id,
            "ran_function_id": ran_function_id,
            "event_trigger_definition": event_trigger,
            "actions": actions,
        },
    )


def encode_subscription_response(ric_request_id: dict, ran_function_id: int,
                                  admitted: list[dict],
                                  not_admitted: list[dict] | None = None) -> bytes:
    return encode_message(
        ProcedureCode.RIC_SUBSCRIPTION, MessageType.SUCCESSFUL_OUTCOME,
        {
            "ric_request_id": ric_request_id,
            "ran_function_id": ran_function_id,
            "admitted_actions": admitted,
            "not_admitted_actions": not_admitted or [],
        },
    )


def encode_ric_indication(ric_request_id: dict, ran_function_id: int,
                          action_id: int, indication_sn: int,
                          indication_type: str,
                          header: dict, message: dict) -> bytes:
    """E2AP §9.1.7 RICIndication.

    `indication_type` is "report" (mode 1) or "insert" (mode 2).
    """
    return encode_message(
        ProcedureCode.RIC_INDICATION, MessageType.INITIATING_MESSAGE,
        {
            "ric_request_id": ric_request_id,
            "ran_function_id": ran_function_id,
            "ric_action_id": action_id,
            "ric_indication_sn": indication_sn,
            "ric_indication_type": indication_type,
            "ric_indication_header": header,
            "ric_indication_message": message,
        },
    )


def encode_ric_control_request(ric_request_id: dict, ran_function_id: int,
                                call_process_id: bytes | None,
                                header: dict, message: dict,
                                ack_request: bool = True) -> bytes:
    """E2AP §9.1.10 RICControlRequest."""
    return encode_message(
        ProcedureCode.RIC_CONTROL, MessageType.INITIATING_MESSAGE,
        {
            "ric_request_id": ric_request_id,
            "ran_function_id": ran_function_id,
            "ric_call_process_id": call_process_id.hex() if call_process_id else None,
            "ric_control_header": header,
            "ric_control_message": message,
            "ric_control_ack_request": "ack" if ack_request else "no_ack",
        },
    )


def encode_ric_control_acknowledge(ric_request_id: dict, ran_function_id: int,
                                    call_process_id: bytes | None,
                                    outcome: dict) -> bytes:
    return encode_message(
        ProcedureCode.RIC_CONTROL, MessageType.SUCCESSFUL_OUTCOME,
        {
            "ric_request_id": ric_request_id,
            "ran_function_id": ran_function_id,
            "ric_call_process_id": call_process_id.hex() if call_process_id else None,
            "ric_control_outcome": outcome,
        },
    )
