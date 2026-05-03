"""CZML exporter for the existing CesiumJS viewer in `contrib/ntn-cho/visualization/`.

Each satellite becomes a CZML packet with a position SAMPLED_POSITION track in
ECEF metres (Cesium's CARTESIAN reference frame). Times are ISO-8601 UTC.
"""

from __future__ import annotations

import json
import math
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Sequence

from skyfield.api import load, wgs84

from ntn_constellation.propagator import Constellation


def _ecef_from_eci_geodetic(lat_deg: float, lon_deg: float, alt_km: float) -> tuple[float, float, float]:
    """Geodetic (WGS-84) to ECEF metres. Cesium expects ECEF/Fixed."""
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    h = alt_km * 1000.0
    n = a / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
    x = (n + h) * math.cos(lat) * math.cos(lon)
    y = (n + h) * math.cos(lat) * math.sin(lon)
    z = (n * (1.0 - e2) + h) * math.sin(lat)
    return x, y, z


def write_czml(
    *,
    constellation: Constellation,
    start: datetime,
    duration: timedelta,
    sample_step: timedelta,
    out_path: Path | str,
    document_name: str = "ntn-constellation",
    show_orbit_path: bool = True,
    point_color_rgba: tuple[int, int, int, int] = (0, 200, 255, 255),
) -> Path:
    """Generate a CZML document with a SAMPLED_POSITION track per satellite.

    Cesium's `referenceFrame: "FIXED"` expects ECEF; we convert via WGS-84.
    """
    if sample_step <= timedelta(0):
        raise ValueError("sample_step must be positive")

    start_utc = start.astimezone(timezone.utc) if start.tzinfo else start.replace(tzinfo=timezone.utc)
    stop_utc = start_utc + duration

    iso_start = start_utc.strftime("%Y-%m-%dT%H:%M:%SZ")
    iso_stop = stop_utc.strftime("%Y-%m-%dT%H:%M:%SZ")
    interval = f"{iso_start}/{iso_stop}"

    document_packet = {
        "id": "document",
        "name": document_name,
        "version": "1.0",
        "clock": {
            "interval": interval,
            "currentTime": iso_start,
            "multiplier": 60,
            "range": "LOOP_STOP",
            "step": "SYSTEM_CLOCK_MULTIPLIER",
        },
    }
    czml: list[dict] = [document_packet]

    ts = load.timescale(builtin=True)

    for sat in constellation:
        samples: list[float] = []
        t = start_utc
        while t <= stop_utc:
            sky_t = ts.from_datetime(t)
            geo = wgs84.subpoint_of(sat._es.at(sky_t))
            alt_km = wgs84.height_of(sat._es.at(sky_t)).km
            x, y, z = _ecef_from_eci_geodetic(geo.latitude.degrees, geo.longitude.degrees, alt_km)
            samples.extend([(t - start_utc).total_seconds(), x, y, z])
            t = t + sample_step

        packet = {
            "id": f"sat-{sat.norad_id}-{sat.name}",
            "name": sat.name,
            "availability": interval,
            "position": {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "FIXED",
                "epoch": iso_start,
                "cartesian": samples,
            },
            "point": {
                "color": {"rgba": list(point_color_rgba)},
                "pixelSize": 6,
                "outlineColor": {"rgba": [0, 0, 0, 255]},
                "outlineWidth": 1,
            },
            "label": {
                "text": sat.name,
                "scale": 0.5,
                "fillColor": {"rgba": [255, 255, 255, 200]},
                "showBackground": True,
                "backgroundColor": {"rgba": [0, 0, 0, 120]},
                "pixelOffset": {"cartesian2": [10, 0]},
                "show": False,
            },
        }
        if show_orbit_path:
            packet["path"] = {
                "leadTime": int(duration.total_seconds() / 4),
                "trailTime": int(duration.total_seconds() / 4),
                "width": 1,
                "material": {"solidColor": {"color": {"rgba": [255, 255, 255, 60]}}},
                "resolution": int(sample_step.total_seconds()),
            }
        czml.append(packet)

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(czml, indent=2), encoding="utf-8")
    return out_path
