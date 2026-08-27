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

from fastapi import FastAPI, HTTPException

from ntn_digital_twin import a3_guard
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
        # The twin_loop rewrites this CZML file every iteration. Its mtime is
        # the cache key: when it changes, the API reloads the constellation.
        self.czml_path = Path(os.environ.get("NTN_TWIN_CZML",
                                             "/tmp/ntn-twin.czml"))
        self.czml_mtime: float | None = None

    def _czml_mtime(self) -> float | None:
        """Current mtime of the CZML file, or None if it does not exist."""
        try:
            return self.czml_path.stat().st_mtime
        except OSError:
            return None

    def ensure_loaded(self) -> Constellation:
        mtime = self._czml_mtime()
        # Serve the cached constellation unless the loop has written a newer
        # CZML file. If the file is absent, we cannot tell — keep the cache.
        if self.cons is not None and (mtime is None or mtime == self.czml_mtime):
            return self.cons
        cache = TleCache(self.cache_dir)
        feed = CelesTrakFeed(cache=cache)
        records = feed.fetch_group(self.group)
        if self.max_sats > 0:
            records = records[: self.max_sats]
        if not records:
            raise HTTPException(503, f"no TLEs for group={self.group}")
        self.cons = Constellation([Satellite(r) for r in records])
        self.czml_mtime = mtime
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
    try:
        when = dt.datetime.fromisoformat(at) if at else dt.datetime.now(tz=dt.timezone.utc)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=f"invalid 'at' timestamp: {exc}") from exc
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



def _a3_params_from_request(req) -> a3_guard.A3Params:
    """TWIN-04: the request's guard settings as the shared A3 parameters.

    The endpoint used to carry its own copy of the link budget and its own copy
    of the A3 state machine. Both now live in ntn_digital_twin.a3_guard, which
    the exporter that ns-3 actuates on also calls, so the two paths cannot drift
    apart again.
    """
    return a3_guard.A3Params(
        margin_quantity=req.margin_quantity,
        a3_offset_db=req.a3_offset_db,
        hysteresis_deg=req.hysteresis_deg,
        time_to_trigger_s=req.time_to_trigger_sec,
        min_service_s=req.min_service_sec,
        min_elevation_deg=req.min_elevation_deg,
        link_eirp_dbm=req.link_eirp_dbm,
        link_gt_db_per_k=req.link_gt_db_per_k,
        link_bandwidth_hz=req.link_bandwidth_hz,
        link_frequency_hz=req.link_frequency_hz,
    )


@app.post("/predict/handover", response_model=PredictHandoverResponse)
def predict_handover(req: PredictHandoverRequest) -> PredictHandoverResponse:
    t0 = time.time()
    cons = _state.ensure_loaded()
    if req.start_iso:
        try:
            now = dt.datetime.fromisoformat(req.start_iso)
        except ValueError as exc:
            raise HTTPException(status_code=422,
                                detail=f"start_iso is not ISO-8601: {exc}") from exc
        if now.tzinfo is None:
            now = now.replace(tzinfo=dt.timezone.utc)
    else:
        now = dt.datetime.now(tz=dt.timezone.utc)
    horizon_sec = req.horizon_min * 60.0
    step = req.step_sec

    obs_ecef = _ecef_from_geodetic(req.ue_lat_deg, req.ue_lon_deg, req.ue_alt_m)
    params = _a3_params_from_request(req)
    ev = a3_guard.A3Evaluator(params)
    events: list[HandoverEvent] = []

    # Hard cap on the loop length (defense in depth on top of the schema
    # bounds): keeps a large horizon / fine step from pinning a CPU.
    MAX_STEPS = 20000
    n_steps = min(int(horizon_sec / step) + 1, MAX_STEPS)
    times = [now + dt.timedelta(seconds=k * step) for k in range(n_steps)]

    # Propagate each satellite across the WHOLE grid in one call. Looping
    # ecef_m per satellite per tick spent its time in Skyfield's per-call setup
    # rather than in SGP4, and it is what put a 10 minute horizon over the
    # endpoint's own 500 ms budget.
    tracks = [sat.ecef_m_series(times) for sat in cons]
    names = [sat.name.strip() for sat in cons]
    norads = [sat.norad_id for sat in cons]

    for k in range(n_steps):
        t_s = k * step
        when = times[k]
        cands = []
        for i in range(len(cons)):
            # gap B3: state vectors are inertial (TEME); ecef_m_series returns
            # the true Earth-fixed position, so the topocentric elevation is not
            # off by the Earth-rotation angle.
            sat_ecef = tracks[i][k]
            el = _elevation_deg_ecef(obs_ecef, sat_ecef,
                                     req.ue_lat_deg, req.ue_lon_deg)
            cands.append(a3_guard.Candidate(
                key=norads[i], elev_deg=el,
                alt_km=a3_guard.altitude_km_from_ecef(sat_ecef), name=names[i]))

        # TWIN-04: the guard itself lives in ntn_digital_twin.a3_guard and is
        # shared with the exporter the C++ side actuates on. This endpoint used
        # to compare a margin in topocentric elevation DEGREES while claiming it
        # enforced "exactly the conditions the sim's A3 algorithm enforces"; the
        # simulator's A3 is a dB margin on a signal level, and the two are not
        # monotonically related once pattern, scan loss and P.618/P.676 enter.
        hit = ev.step(t_s, cands)
        if hit is None:
            continue
        events.append(HandoverEvent(
            time_iso=when.isoformat(),
            sat_in_norad=hit.key_in,
            sat_in_name=hit.name_in,
            sat_out_norad=hit.key_out,
            sat_out_name=hit.name_out,
            elevation_in_deg=hit.elev_in_deg,
            elevation_out_deg=hit.elev_out_deg,
        ))

    elapsed_ms = (time.time() - t0) * 1000.0
    return PredictHandoverResponse(
        requested_at_iso=now.isoformat(),
        horizon_min=req.horizon_min,
        margin_quantity=params.margin_quantity,
        effective_margin=params.effective_margin,
        n_handovers=len(events),
        events=events,
        elapsed_ms=elapsed_ms,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    # Default to loopback: the service is unauthenticated, so it must not bind
    # all interfaces unless the operator explicitly opts in (--host 0.0.0.0
    # behind an authenticating reverse proxy).
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8090)
    args = parser.parse_args(argv)
    import uvicorn
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")
    return 0


if __name__ == "__main__":
    main()
