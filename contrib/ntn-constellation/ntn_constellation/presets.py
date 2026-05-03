"""Constellation presets: Walker-Delta / Walker-Star generators and named templates.

Numbers reflect public filings as of 2026 — they are presets for simulation, not
operational reality. Always overlay live TLEs where actual positioning matters.
"""

from __future__ import annotations

import dataclasses
import math
from datetime import datetime, timezone
from typing import Literal

from ntn_constellation.feeds import TleRecord


PatternType = Literal["star", "delta"]


@dataclasses.dataclass(frozen=True)
class ConstellationPreset:
    name: str
    altitude_km: float
    inclination_deg: float
    num_planes: int
    sats_per_plane: int
    pattern: PatternType
    description: str = ""

    @property
    def total_satellites(self) -> int:
        return self.num_planes * self.sats_per_plane


PRESETS: dict[str, ConstellationPreset] = {
    "starlink-v1-shell1": ConstellationPreset(
        name="starlink-v1-shell1",
        altitude_km=550.0,
        inclination_deg=53.0,
        num_planes=72,
        sats_per_plane=22,
        pattern="delta",
        description="SpaceX Starlink Gen1 shell 1 (53.0 deg, 550 km, 1584 sats)",
    ),
    "starlink-v1-shell2": ConstellationPreset(
        name="starlink-v1-shell2",
        altitude_km=540.0,
        inclination_deg=53.2,
        num_planes=72,
        sats_per_plane=22,
        pattern="delta",
        description="SpaceX Starlink Gen1 shell 2 (53.2 deg, 540 km, 1584 sats)",
    ),
    "starlink-polar": ConstellationPreset(
        name="starlink-polar",
        altitude_km=560.0,
        inclination_deg=97.6,
        num_planes=6,
        sats_per_plane=58,
        pattern="star",
        description="Starlink polar shell (97.6 deg, 560 km, 348 sats)",
    ),
    "oneweb": ConstellationPreset(
        name="oneweb",
        altitude_km=1200.0,
        inclination_deg=87.9,
        num_planes=18,
        sats_per_plane=36,
        pattern="star",
        description="OneWeb Phase 1 (87.9 deg, 1200 km, 648 sats)",
    ),
    "kuiper": ConstellationPreset(
        name="kuiper",
        altitude_km=630.0,
        inclination_deg=51.9,
        num_planes=34,
        sats_per_plane=34,
        pattern="delta",
        description="Amazon Kuiper Phase 1 (51.9 deg, 630 km, 1156 sats; full plan 3236)",
    ),
    "telesat-lightspeed": ConstellationPreset(
        name="telesat-lightspeed",
        altitude_km=1015.0,
        inclination_deg=98.98,
        num_planes=27,
        sats_per_plane=13,
        pattern="star",
        description="Telesat Lightspeed (98.98 deg polar + inclined, 298 sats planned)",
    ),
    "iridium-next": ConstellationPreset(
        name="iridium-next",
        altitude_km=780.0,
        inclination_deg=86.4,
        num_planes=6,
        sats_per_plane=11,
        pattern="star",
        description="Iridium NEXT operational (86.4 deg, 780 km, 66 sats)",
    ),
}


_MU_EARTH = 398600.4418  # km^3 / s^2 — WGS-72


def _mean_motion_rev_per_day(altitude_km: float, earth_radius_km: float = 6378.135) -> float:
    """Mean motion in rev/day for a circular orbit of given altitude (WGS-72 Earth radius)."""
    a_km = earth_radius_km + altitude_km
    period_s = 2.0 * math.pi * math.sqrt(a_km**3 / _MU_EARTH)
    return 86400.0 / period_s


def _format_tle_line1(catnr: int, epoch_year_yy: int, epoch_doy: float) -> str:
    classification = "U"
    intl_designator = "00000ABC"
    bstar = " 00000-0"
    line = (
        f"1 {catnr:05d}{classification} "
        f"{intl_designator:<8} "
        f"{epoch_year_yy:02d}{epoch_doy:012.8f} "
        f" .00000000  00000-0 {bstar} 0  0000"
    )
    line = line[:68]
    line = line + str(_tle_checksum(line))
    return line


def _format_tle_line2(
    catnr: int,
    inclination_deg: float,
    raan_deg: float,
    eccentricity: float,
    arg_perigee_deg: float,
    mean_anomaly_deg: float,
    mean_motion_rev_per_day: float,
    rev_number: int = 0,
) -> str:
    ecc_str = f"{int(round(eccentricity * 1e7)):07d}"
    line = (
        f"2 {catnr:05d} "
        f"{inclination_deg:8.4f} "
        f"{raan_deg:8.4f} "
        f"{ecc_str} "
        f"{arg_perigee_deg:8.4f} "
        f"{mean_anomaly_deg:8.4f} "
        f"{mean_motion_rev_per_day:11.8f}"
        f"{rev_number:5d}"
    )
    line = line[:68]
    line = line + str(_tle_checksum(line))
    return line


def _tle_checksum(line: str) -> int:
    s = 0
    for ch in line[:68]:
        if ch.isdigit():
            s += int(ch)
        elif ch == "-":
            s += 1
    return s % 10


def _epoch_yyddd(epoch: datetime) -> tuple[int, float]:
    epoch_utc = epoch.astimezone(timezone.utc)
    yy = epoch_utc.year % 100
    start = datetime(epoch_utc.year, 1, 1, tzinfo=timezone.utc)
    doy = (epoch_utc - start).total_seconds() / 86400.0 + 1.0
    return yy, doy


def walker_delta(
    *,
    name_prefix: str,
    altitude_km: float,
    inclination_deg: float,
    num_planes: int,
    sats_per_plane: int,
    phasing_factor: int = 1,
    epoch: datetime | None = None,
    catnr_start: int = 90000,
) -> list[TleRecord]:
    """Walker-Delta T/P/F constellation. Returns synthetic TLEs ready for SGP4."""
    return _walker(
        pattern="delta",
        name_prefix=name_prefix,
        altitude_km=altitude_km,
        inclination_deg=inclination_deg,
        num_planes=num_planes,
        sats_per_plane=sats_per_plane,
        phasing_factor=phasing_factor,
        epoch=epoch,
        catnr_start=catnr_start,
    )


def walker_star(
    *,
    name_prefix: str,
    altitude_km: float,
    inclination_deg: float,
    num_planes: int,
    sats_per_plane: int,
    epoch: datetime | None = None,
    catnr_start: int = 90000,
) -> list[TleRecord]:
    """Walker-Star (RAAN spread over 180 deg, polar/near-polar)."""
    return _walker(
        pattern="star",
        name_prefix=name_prefix,
        altitude_km=altitude_km,
        inclination_deg=inclination_deg,
        num_planes=num_planes,
        sats_per_plane=sats_per_plane,
        phasing_factor=1,
        epoch=epoch,
        catnr_start=catnr_start,
    )


def _walker(
    *,
    pattern: PatternType,
    name_prefix: str,
    altitude_km: float,
    inclination_deg: float,
    num_planes: int,
    sats_per_plane: int,
    phasing_factor: int,
    epoch: datetime | None,
    catnr_start: int,
) -> list[TleRecord]:
    if num_planes <= 0 or sats_per_plane <= 0:
        raise ValueError("num_planes and sats_per_plane must be positive")
    if pattern == "star" and num_planes > 0:
        raan_span_deg = 180.0
    else:
        raan_span_deg = 360.0

    epoch = epoch or datetime.now(tz=timezone.utc)
    yy, doy = _epoch_yyddd(epoch)

    mean_motion = _mean_motion_rev_per_day(altitude_km)
    records: list[TleRecord] = []
    catnr = catnr_start
    total = num_planes * sats_per_plane

    for p in range(num_planes):
        raan = (p * raan_span_deg / num_planes) % 360.0
        for s in range(sats_per_plane):
            in_plane_anomaly = s * 360.0 / sats_per_plane
            inter_plane_phase = phasing_factor * p * 360.0 / total
            mean_anomaly = (in_plane_anomaly + inter_plane_phase) % 360.0
            l1 = _format_tle_line1(catnr, yy, doy)
            l2 = _format_tle_line2(
                catnr=catnr,
                inclination_deg=inclination_deg,
                raan_deg=raan,
                eccentricity=0.0,
                arg_perigee_deg=0.0,
                mean_anomaly_deg=mean_anomaly,
                mean_motion_rev_per_day=mean_motion,
            )
            records.append(TleRecord(name=f"{name_prefix} {p:02d}-{s:02d}", line1=l1, line2=l2))
            catnr += 1

    return records


def from_preset(preset: ConstellationPreset, *, epoch: datetime | None = None) -> list[TleRecord]:
    fn = walker_star if preset.pattern == "star" else walker_delta
    return fn(
        name_prefix=preset.name,
        altitude_km=preset.altitude_km,
        inclination_deg=preset.inclination_deg,
        num_planes=preset.num_planes,
        sats_per_plane=preset.sats_per_plane,
        epoch=epoch,
    )
