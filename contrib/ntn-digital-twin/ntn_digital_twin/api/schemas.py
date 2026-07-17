"""Pydantic schemas for the digital-twin REST API."""

from __future__ import annotations

import datetime as dt

from pydantic import BaseModel, Field


class HealthResponse(BaseModel):
    ok: bool = True
    uptime_s: float
    constellation_size: int
    last_refresh_iso: str | None = None


class SatState(BaseModel):
    sat_norad: int
    name: str
    lat_deg: float
    lon_deg: float
    alt_km: float
    vel_eci_km_s: list[float] = Field(min_length=3, max_length=3)


class ConstellationStateResponse(BaseModel):
    epoch_iso: str
    count: int
    satellites: list[SatState]


class HandoverEvent(BaseModel):
    time_iso: str
    sat_in_norad: int
    sat_in_name: str
    sat_out_norad: int | None
    sat_out_name: str | None
    elevation_in_deg: float
    elevation_out_deg: float | None


class PredictHandoverRequest(BaseModel):
    # Bounds are load-bearing: step_sec > 0 removes the ZeroDivisionError, and
    # bounding horizon/step keeps the O(n_steps x n_sats) prediction loop from
    # being driven into an unbounded CPU hang by a hostile request.
    ue_lat_deg: float = Field(ge=-90.0, le=90.0)
    ue_lon_deg: float = Field(ge=-180.0, le=180.0)
    ue_alt_m: float = Field(default=0.0, ge=-500.0, le=100_000.0)
    horizon_min: float = Field(default=10.0, gt=0.0, le=1440.0)  # up to 24 h
    step_sec: float = Field(default=5.0, ge=0.1, le=3600.0)      # [0.1 s, 1 h]
    min_elevation_deg: float = Field(default=10.0, ge=0.0, le=90.0)
    # W1: the sim executes A3-style handovers (hysteresis + time-to-trigger),
    # not a bare per-tick best-elevation argmax. Mirror that here so the twin's
    # predicted sequence can actually match the sim's executed one instead of
    # ping-ponging at every elevation crossover.
    hysteresis_deg: float = Field(default=3.0, ge=0.0, le=45.0)
    time_to_trigger_sec: float = Field(default=0.0, ge=0.0, le=60.0)
    min_service_sec: float = Field(default=0.0, ge=0.0, le=600.0)


class PredictHandoverResponse(BaseModel):
    requested_at_iso: str
    horizon_min: float
    n_handovers: int
    events: list[HandoverEvent]
    elapsed_ms: float
