"""Pydantic schemas for the digital-twin REST API."""

from __future__ import annotations

import datetime as dt

from typing import Literal

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

    # TWIN-04: which QUANTITY the A3 margin is measured in.
    #
    # This endpoint compared elevations and the code claimed it enforced
    # "exactly the conditions the sim's A3 algorithm enforces". It did not.
    # TS 38.331 Event A3 is a margin on a measurement quantity (RSRP, RSRQ or
    # SINR) expressed in dB, and the simulator's A3 is sinr_dB > servingSinr_dB
    # + a3Offset_dB. Elevation and SINR are not monotonically related once
    # antenna pattern, scan loss and ITU-R P.618/P.676 attenuation enter, so an
    # elevation margin in DEGREES admits and rejects different handovers than a
    # dB margin. A 3 degree threshold and a 3 dB threshold are not the same
    # test.
    #
    # Default "db" so the endpoint agrees with the simulator. "elevation_deg"
    # keeps the previous behaviour for a caller that wants it, and is labelled
    # rather than implied.
    margin_quantity: Literal["db", "elevation_deg"] = "db"
    a3_offset_db: float = Field(default=3.0, ge=0.0, le=40.0)
    # Link-budget terms for the dB quantity. The twin has ephemeris but no
    # radio, so it forms an SNR from free-space loss over the slant range.
    link_eirp_dbm: float = Field(default=62.0, ge=-30.0, le=120.0)
    link_gt_db_per_k: float = Field(default=1.1, ge=-40.0, le=40.0)
    link_bandwidth_hz: float = Field(default=20.0e6, gt=0.0)
    link_frequency_hz: float = Field(default=2.0e9, gt=0.0)

    # TWIN-04: the epoch the prediction starts from, ISO-8601 UTC.
    #
    # The endpoint always started at wall-clock now(), so no answer it gave
    # could be reproduced, compared against a recorded run, or checked against
    # the exporter that ns-3 actuates on. Leave it unset for the live behaviour.
    start_iso: str | None = None


class PredictHandoverResponse(BaseModel):
    requested_at_iso: str
    horizon_min: float
    # TWIN-04: echo the guard that was actually applied.
    #
    # The margin quantity now defaults to dB so the endpoint agrees with the
    # simulator, which means a caller who sets only hysteresis_deg is no longer
    # setting the knob in force. Silently ignoring a supplied parameter is the
    # worst kind of breaking change, so the response says which quantity was
    # used and what margin it carried.
    margin_quantity: str = "db"
    effective_margin: float = 0.0
    n_handovers: int
    events: list[HandoverEvent]
    elapsed_ms: float
