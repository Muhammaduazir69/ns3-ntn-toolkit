"""Propagation: thin layer over Skyfield (precision) and sgp4 (speed).

Skyfield is preferred for coordinates because it handles the WGS-72/JPL frames
correctly out of the box. The raw `sgp4` library is exposed too for batch
state-vector queries that don't need geodetic conversion.
"""

from __future__ import annotations

import dataclasses
import math
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Iterable, Iterator, Sequence

from sgp4.api import SGP4_ERRORS, Satrec, jday
from skyfield.api import EarthSatellite, load, wgs84

from ntn_constellation.feeds import TleRecord


@dataclasses.dataclass(frozen=True)
class StateVector:
    """ECI position (km) and velocity (km/s) at a given UTC epoch."""

    epoch: datetime
    r_eci_km: tuple[float, float, float]
    v_eci_km_s: tuple[float, float, float]

    @property
    def altitude_km(self) -> float:
        return math.sqrt(sum(c * c for c in self.r_eci_km)) - 6378.135


@dataclasses.dataclass(frozen=True)
class GeodeticPosition:
    """Latitude/longitude/altitude (WGS-84) at a given UTC epoch."""

    epoch: datetime
    lat_deg: float
    lon_deg: float
    alt_km: float


class Satellite:
    """One satellite, propagatable at any UTC epoch.

    The Skyfield `EarthSatellite` is the canonical object — it caches the
    Satrec and handles time scale conversions. The bare `Satrec` is also
    cached for fast batch state vectors when geodesy isn't needed.
    """

    __slots__ = ("name", "tle", "_es", "_satrec", "_ts")

    def __init__(self, tle: TleRecord, ts=None) -> None:
        self.name = tle.name
        self.tle = tle
        self._ts = ts or load.timescale(builtin=True)
        self._es = EarthSatellite(tle.line1, tle.line2, tle.name, self._ts)
        self._satrec = Satrec.twoline2rv(tle.line1, tle.line2)

    @property
    def norad_id(self) -> int:
        return self.tle.norad_id

    def state(self, when: datetime) -> StateVector:
        when = _ensure_utc(when)
        jd, fr = jday(when.year, when.month, when.day, when.hour, when.minute,
                      when.second + when.microsecond / 1e6)
        err, r, v = self._satrec.sgp4(jd, fr)
        if err != 0:
            raise RuntimeError(f"SGP4 error for {self.name}: {SGP4_ERRORS.get(err, err)}")
        return StateVector(epoch=when, r_eci_km=tuple(r), v_eci_km_s=tuple(v))

    def geodetic(self, when: datetime) -> GeodeticPosition:
        when = _ensure_utc(when)
        t = self._ts.from_datetime(when)
        sub = wgs84.subpoint_of(self._es.at(t))
        return GeodeticPosition(
            epoch=when,
            lat_deg=sub.latitude.degrees,
            lon_deg=sub.longitude.degrees,
            alt_km=wgs84.height_of(self._es.at(t)).km,
        )

    def trajectory(
        self, start: datetime, stop: datetime, step: timedelta
    ) -> Iterator[StateVector]:
        if step <= timedelta(0):
            raise ValueError("step must be positive")
        t = _ensure_utc(start)
        stop = _ensure_utc(stop)
        while t <= stop:
            yield self.state(t)
            t = t + step

    def elevation_deg(
        self, when: datetime, *, observer_lat_deg: float, observer_lon_deg: float,
        observer_alt_m: float = 0.0,
    ) -> float:
        """Elevation angle (deg) of this satellite from a ground observer."""
        when = _ensure_utc(when)
        observer = wgs84.latlon(observer_lat_deg, observer_lon_deg, observer_alt_m)
        t = self._ts.from_datetime(when)
        topocentric = (self._es - observer).at(t)
        alt, _az, _dist = topocentric.altaz()
        return alt.degrees


class Constellation:
    """A collection of `Satellite`s sharing one Skyfield timescale."""

    def __init__(self, satellites: Sequence[Satellite]) -> None:
        if not satellites:
            raise ValueError("constellation must have at least one satellite")
        self._sats: list[Satellite] = list(satellites)
        self._ts = self._sats[0]._ts

    @classmethod
    def from_tles(cls, tles: Iterable[TleRecord]) -> "Constellation":
        ts = load.timescale(builtin=True)
        return cls([Satellite(t, ts=ts) for t in tles])

    @classmethod
    def from_tle_file(cls, path: Path | str) -> "Constellation":
        from ntn_constellation.feeds import iter_tle_file
        return cls.from_tles(iter_tle_file(Path(path)))

    def __len__(self) -> int:
        return len(self._sats)

    def __iter__(self):
        return iter(self._sats)

    def __getitem__(self, idx):
        return self._sats[idx]

    def names(self) -> list[str]:
        return [s.name for s in self._sats]

    def positions(self, when: datetime) -> list[GeodeticPosition]:
        return [s.geodetic(when) for s in self._sats]

    def state_vectors(self, when: datetime) -> list[StateVector]:
        return [s.state(when) for s in self._sats]

    def visible_from(
        self, when: datetime, *, observer_lat_deg: float, observer_lon_deg: float,
        observer_alt_m: float = 0.0, min_elevation_deg: float = 10.0,
    ) -> list[tuple[Satellite, float]]:
        out: list[tuple[Satellite, float]] = []
        for sat in self._sats:
            el = sat.elevation_deg(
                when,
                observer_lat_deg=observer_lat_deg,
                observer_lon_deg=observer_lon_deg,
                observer_alt_m=observer_alt_m,
            )
            if el >= min_elevation_deg:
                out.append((sat, el))
        out.sort(key=lambda pair: pair[1], reverse=True)
        return out


def _ensure_utc(when: datetime) -> datetime:
    if when.tzinfo is None:
        return when.replace(tzinfo=timezone.utc)
    return when.astimezone(timezone.utc)
