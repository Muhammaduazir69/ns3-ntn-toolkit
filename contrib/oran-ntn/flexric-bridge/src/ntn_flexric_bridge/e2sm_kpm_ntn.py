"""E2SM-KPM v3 service-model with NTN-specific extensions.

Standard E2SM-KPM (O-RAN.WG3.E2SM-KPM) carries cell-level and UE-level
performance counters. We add four NTN-specific measurements to the
`measurement_type_list`:

  - ``elevation_deg``       — UE→sat elevation (deg), maps to SatNorad tag
  - ``doppler_hz``          — instantaneous Doppler shift on the UE link
  - ``tte_total_us``        — total Timing Advance (TTE) in µs
  - ``feeder_link_status``  — 0 = down, 1 = up (transparent payload only)

Indication header carries the gNB ID + epoch, indication message carries
a list of (measurement_type, value) tuples per UE/cell.
"""

from __future__ import annotations

import datetime as dt
from dataclasses import asdict, dataclass, field
from typing import Any

# RAN function ID for E2SM-KPM-NTN (toolkit-local, must match the agent + xApp).
RAN_FUNCTION_ID = 147


@dataclass
class KpmIndicationHeader:
    gnb_id: int
    sat_norad: int | None = None
    timestamp_iso: str = ""
    plmn_id: str = "001/01"

    @classmethod
    def make(cls, gnb_id: int, sat_norad: int | None = None) -> "KpmIndicationHeader":
        return cls(gnb_id=gnb_id, sat_norad=sat_norad,
                   timestamp_iso=dt.datetime.now(tz=dt.timezone.utc).isoformat())


@dataclass
class KpmMeasurement:
    measurement_type: str   # "rsrp_dbm", "sinr_db", "elevation_deg", "doppler_hz", "tte_total_us", "feeder_link_status"
    value: float
    ue_imsi: str | None = None
    cell_id: str | None = None


@dataclass
class KpmIndicationMessage:
    measurements: list[KpmMeasurement] = field(default_factory=list)
    granularity_period_ms: int = 1000


def build_indication(header: KpmIndicationHeader,
                     message: KpmIndicationMessage) -> tuple[dict, dict]:
    """Pack to (header_dict, message_dict) ready for `e2ap_codec.encode_ric_indication`."""
    return asdict(header), {
        "granularity_period_ms": message.granularity_period_ms,
        "measurements": [asdict(m) for m in message.measurements],
    }


def parse_indication(header_dict: dict[str, Any],
                     message_dict: dict[str, Any]) -> tuple[KpmIndicationHeader, KpmIndicationMessage]:
    h = KpmIndicationHeader(**header_dict)
    measurements = [KpmMeasurement(**m) for m in message_dict.get("measurements", [])]
    m = KpmIndicationMessage(
        measurements=measurements,
        granularity_period_ms=message_dict.get("granularity_period_ms", 1000),
    )
    return h, m


# Subscription event triggers — periodic at `report_period_ms`, or
# on-event when a measurement crosses a threshold.
def event_trigger_periodic(report_period_ms: int) -> dict:
    return {"trigger_type": "periodic", "period_ms": report_period_ms}


def event_trigger_threshold(measurement: str, op: str, value: float) -> dict:
    if op not in ("<", "<=", ">", ">=", "=="):
        raise ValueError(f"unsupported op {op!r}")
    return {
        "trigger_type": "threshold",
        "measurement_type": measurement,
        "op": op,
        "value": value,
    }


# Action definitions — REPORT mode (read-only) and INSERT mode (callback).
def action_report(action_id: int, measurement_types: list[str]) -> dict:
    return {
        "action_id": action_id,
        "action_type": "report",
        "measurement_types": measurement_types,
    }
