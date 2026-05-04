"""E2SM-RC v1.03 service-model with NTN-specific control actions.

Standard E2SM-RC carries RAN Control actions like handover trigger or QoS
modification. The NTN-specific actions we register on top:

  - ``cho_trigger``         — execute a Conditional Handover for a UE
  - ``beam_select``         — direct a beam for a UE (multi-beam constellation)
  - ``slice_prb_share``     — set per-slice PRB share on a cell (W6 hook)

Each control header identifies the target gNB + UE; each control message
carries the action-specific parameters.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

RAN_FUNCTION_ID = 148   # E2SM-RC-NTN, toolkit-local


@dataclass
class RcControlHeader:
    gnb_id: int
    ue_imsi: str
    style_type: int = 1   # "Radio Bearer And QoS Flow Control" — used by all 3 actions
    action_id: int = 1    # action chosen from style_type's action set


@dataclass
class ChoTriggerMessage:
    target_cell_id: str
    target_sat_norad: int | None = None
    expected_tta_ms: float = 0.0


@dataclass
class BeamSelectMessage:
    beam_id: int
    cell_id: str


@dataclass
class SlicePrbShareMessage:
    sst: int               # 1=eMBB 2=URLLC 3=mMTC
    sd_hex: str            # 24-bit slice differentiator
    prb_share: float       # 0..1


def build_cho_trigger(header: RcControlHeader, msg: ChoTriggerMessage) -> tuple[dict, dict]:
    return asdict(header), {"action": "cho_trigger", **asdict(msg)}


def build_beam_select(header: RcControlHeader, msg: BeamSelectMessage) -> tuple[dict, dict]:
    return asdict(header), {"action": "beam_select", **asdict(msg)}


def build_slice_prb_share(header: RcControlHeader, msg: SlicePrbShareMessage) -> tuple[dict, dict]:
    return asdict(header), {"action": "slice_prb_share", **asdict(msg)}


def parse_cho_trigger(header_dict: dict[str, Any], message_dict: dict[str, Any]) -> tuple[RcControlHeader, ChoTriggerMessage]:
    h = RcControlHeader(**{k: v for k, v in header_dict.items() if k in RcControlHeader.__dataclass_fields__})
    if message_dict.get("action") != "cho_trigger":
        raise ValueError(f"expected action=cho_trigger, got {message_dict.get('action')!r}")
    body = {k: v for k, v in message_dict.items() if k != "action"}
    return h, ChoTriggerMessage(**body)
