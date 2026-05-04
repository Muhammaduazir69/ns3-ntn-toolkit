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
    ue_lat_deg: float
    ue_lon_deg: float
    ue_alt_m: float = 0.0
    horizon_min: float = 10.0
    step_sec: float = 5.0
    min_elevation_deg: float = 10.0


class PredictHandoverResponse(BaseModel):
    requested_at_iso: str
    horizon_min: float
    n_handovers: int
    events: list[HandoverEvent]
    elapsed_ms: float
