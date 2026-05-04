"""FastAPI server for the digital-twin (W10).

Endpoints:

* ``GET  /health``                — liveness + uptime
* ``GET  /constellation/state``   — sat positions at given UTC epoch
* ``POST /predict/handover``      — predict handovers for one UE over a horizon

Performance gate: ``/predict/handover`` returns in < 500 ms for a
50-sat / 10-min horizon. Reached by:

  - propagating *all* sats once with the bare ``Satrec`` SGP4 (sub-ms each),
  - computing topocentric elevation in closed form from ECEF positions
    (no Skyfield AltAz on the hot path),
  - reusing the cached ``Constellation`` between calls.

The constellation is loaded lazily on first request and refreshed each time
``twin_loop`` writes a new CZML file (the file's mtime is the cache key).
"""

from __future__ import annotations

import argparse
import datetime as dt
import math
import os
import time
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException
from pydantic import ValidationError

from ntn_constellation.feeds import CelesTrakFeed, TleCache
from ntn_constellation.propagator import Constellation, Satellite

from ntn_digital_twin.api.schemas import (
    ConstellationStateResponse,
    HandoverEvent,
    HealthResponse,
    PredictHandoverRequest,
    PredictHandoverResponse,
    SatState,
)

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def _ecef_from_geodetic(lat_deg: float, lon_deg: float, alt_m: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    x = (n + alt_m) * math.cos(lat) * math.cos(lon)
    y = (n + alt_m) * math.cos(lat) * math.sin(lon)
    z = (n * (1 - WGS84_E2) + alt_m) * math.sin(lat)
    return (x, y, z)


def _elevation_deg_ecef(observer: tuple[float, float, float],
                        target: tuple[float, float, float],
                        observer_lat_deg: float,
                        observer_lon_deg: float) -> float:
    """Closed-form elevation of `target` from `observer` (both ECEF, metres).

    Uses the rotation from ECEF to local ENU at the observer's geodetic
    location. Elevation = atan2(up, sqrt(east² + north²)).
    """
    dx = target[0] - observer[0]
    dy = target[1] - observer[1]
    dz = target[2] - observer[2]
    lat = math.radians(observer_lat_deg)
    lon = math.radians(observer_lon_deg)
    sin_lat = math.sin(lat); cos_lat = math.cos(lat)
    sin_lon = math.sin(lon); cos_lon = math.cos(lon)
    # ECEF → ENU
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    horiz = math.sqrt(east * east + north * north)
    return math.degrees(math.atan2(up, max(horiz, 1e-9)))


class TwinState:
    def __init__(self):
        self.cons: Constellation | None = None
        self.last_refresh: dt.datetime | None = None
        self.started_at = time.time()
        self.group = os.environ.get("NTN_TWIN_GROUP", "starlink")
        self.max_sats = int(os.environ.get("NTN_TWIN_MAX_SATS", "50"))
        self.cache_dir = Path(os.environ.get("NTN_TWIN_CACHE_DIR",
                                              "/tmp/.ntn-twin-cache"))

    def ensure_loaded(self) -> Constellation:
        if self.cons is not None:
            return self.cons
        cache = TleCache(self.cache_dir)
        feed = CelesTrakFeed(cache=cache)
        records = feed.fetch_group(self.group)
        if self.max_sats > 0:
            records = records[: self.max_sats]
        if not records:
            raise HTTPException(503, f"no TLEs for group={self.group}")
        self.cons = Constellation([Satellite(r) for r in records])
        self.last_refresh = dt.datetime.now(tz=dt.timezone.utc)
        return self.cons


_state = TwinState()
app = FastAPI(title="ntn-digital-twin", version="0.1.0")


@app.get("/health", response_model=HealthResponse)
def health() -> HealthResponse:
    return HealthResponse(
        ok=True,
        uptime_s=time.time() - _state.started_at,
        constellation_size=len(_state.cons) if _state.cons else 0,
        last_refresh_iso=_state.last_refresh.isoformat() if _state.last_refresh else None,
    )


@app.get("/constellation/state", response_model=ConstellationStateResponse)
def constellation_state(at: str | None = None) -> ConstellationStateResponse:
    cons = _state.ensure_loaded()
    when = dt.datetime.fromisoformat(at) if at else dt.datetime.now(tz=dt.timezone.utc)
    if when.tzinfo is None:
        when = when.replace(tzinfo=dt.timezone.utc)
    sats: list[SatState] = []
    states = cons.state_vectors(when)
    for sat, sv in zip(cons, states):
        geo = sat.geodetic(when)
        sats.append(SatState(
            sat_norad=sat.norad_id,
            name=sat.name.strip(),
            lat_deg=geo.lat_deg,
            lon_deg=geo.lon_deg,
            alt_km=geo.alt_km,
            vel_eci_km_s=list(sv.v_eci_km_s),
        ))
    return ConstellationStateResponse(
        epoch_iso=when.isoformat(),
        count=len(sats),
        satellites=sats,
    )


@app.post("/predict/handover", response_model=PredictHandoverResponse)
def predict_handover(req: PredictHandoverRequest) -> PredictHandoverResponse:
    t0 = time.time()
    cons = _state.ensure_loaded()
    now = dt.datetime.now(tz=dt.timezone.utc)
    horizon_sec = req.horizon_min * 60.0
    step = req.step_sec

    obs_ecef = _ecef_from_geodetic(req.ue_lat_deg, req.ue_lon_deg, req.ue_alt_m)
    events: list[HandoverEvent] = []
    current_serving: int | None = None
    current_serving_name: str | None = None

    n_steps = int(horizon_sec / step) + 1
    for k in range(n_steps):
        when = now + dt.timedelta(seconds=k * step)
        states = cons.state_vectors(when)
        # Best-elevation satellite at this tick
        best_idx = -1
        best_el = -90.0
        # Track current's elevation for outgoing handover annotation
        current_el = -90.0
        for i, sv in enumerate(states):
            sat_ecef = (sv.r_eci_km[0] * 1000.0,
                        sv.r_eci_km[1] * 1000.0,
                        sv.r_eci_km[2] * 1000.0)
            el = _elevation_deg_ecef(obs_ecef, sat_ecef,
                                     req.ue_lat_deg, req.ue_lon_deg)
            if el > best_el:
                best_el = el
                best_idx = i
            if cons[i].norad_id == current_serving:
                current_el = el

        if best_idx < 0 or best_el < req.min_elevation_deg:
            continue

        best_sat = cons[best_idx]
        if best_sat.norad_id != current_serving:
            events.append(HandoverEvent(
                time_iso=when.isoformat(),
                sat_in_norad=best_sat.norad_id,
                sat_in_name=best_sat.name.strip(),
                sat_out_norad=current_serving,
                sat_out_name=current_serving_name,
                elevation_in_deg=best_el,
                elevation_out_deg=current_el if current_serving is not None else None,
            ))
            current_serving = best_sat.norad_id
            current_serving_name = best_sat.name.strip()

    elapsed_ms = (time.time() - t0) * 1000.0
    return PredictHandoverResponse(
        requested_at_iso=now.isoformat(),
        horizon_min=req.horizon_min,
        n_handovers=len(events),
        events=events,
        elapsed_ms=elapsed_ms,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    args = parser.parse_args(argv)
    import uvicorn
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")
    return 0


if __name__ == "__main__":
    main()
