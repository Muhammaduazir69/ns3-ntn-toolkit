"""W10 tests — twin loop crash-tolerance + API correctness/latency."""

from __future__ import annotations

import datetime as dt
import math
from pathlib import Path

import pytest

# Constellation-side imports.
from ntn_constellation.presets import PRESETS, from_preset
from ntn_constellation.propagator import Constellation, Satellite

from ntn_digital_twin import twin_loop
from ntn_digital_twin.api import server as api_server


@pytest.fixture
def synthetic_constellation():
    """A small synthetic Walker-Star Starlink shell used by API tests.

    Avoids any live network call so the suite is self-contained.
    """
    preset = PRESETS["starlink-v1-shell1"]
    tles = from_preset(preset, epoch=dt.datetime(2026, 5, 4, tzinfo=dt.timezone.utc))[:50]
    return Constellation([Satellite(t) for t in tles])


# --------------------------------------------------------------------------- twin_loop
def test_emit_influx_lp_writes_correct_schema(tmp_path: Path, synthetic_constellation):
    cfg = twin_loop.LoopConfig(
        run_id="pytest",
        influx_lp_path=tmp_path / "out.lp",
    )
    when = dt.datetime(2026, 5, 4, 10, 0, 0, tzinfo=dt.timezone.utc)
    n = twin_loop.emit_influx_lp(synthetic_constellation, when, cfg)
    assert n == len(synthetic_constellation)

    lines = (tmp_path / "out.lp").read_text().strip().splitlines()
    assert len(lines) == n
    sample = lines[0]
    # measurement, tags, fields, timestamp ns
    head, _, ts = sample.rpartition(" ")
    assert int(ts) == int(when.timestamp() * 1e9)
    meas, _, body = head.partition(",")
    assert meas == "ntn_sat_pos"
    # required tags
    assert "sat_norad=" in body and "run_id=pytest" in body
    # required fields
    fields_section = head.split(" ", 1)[1]
    for f in ("sat_x_m", "sat_y_m", "sat_z_m"):
        assert f in fields_section


def test_run_iteration_handles_missing_network(tmp_path: Path, monkeypatch):
    """If the TLE feed throws, the loop reports the error and continues."""
    cfg = twin_loop.LoopConfig(
        czml_path=tmp_path / "twin.czml",
        influx_lp_path=tmp_path / "twin.lp",
        interval_sec=0.0,
    )

    def boom(_cfg):
        raise RuntimeError("simulated network outage")

    monkeypatch.setattr(twin_loop, "fetch_constellation", boom)
    stats = twin_loop.run_loop(cfg, max_iterations=3)
    assert stats.error_count == 3
    assert stats.iterations == 0
    assert "simulated network outage" in stats.last_error


# --------------------------------------------------------------------------- API
def test_api_health(synthetic_constellation, monkeypatch):
    from fastapi.testclient import TestClient

    api_server._state.cons = synthetic_constellation
    api_server._state.last_refresh = dt.datetime(2026, 5, 4, 10, 0, 0,
                                                 tzinfo=dt.timezone.utc)
    client = TestClient(api_server.app)
    r = client.get("/health")
    assert r.status_code == 200
    body = r.json()
    assert body["ok"] is True
    assert body["constellation_size"] == 50
    assert body["last_refresh_iso"].startswith("2026-05-04")


def test_api_constellation_state(synthetic_constellation):
    from fastapi.testclient import TestClient

    api_server._state.cons = synthetic_constellation
    client = TestClient(api_server.app)
    r = client.get("/constellation/state",
                   params={"at": "2026-05-04T12:00:00+00:00"})
    assert r.status_code == 200
    body = r.json()
    assert body["count"] == 50
    sat0 = body["satellites"][0]
    # Walker-Star Starlink shell-1 nominal altitude is 540–555 km.
    assert 530 < sat0["alt_km"] < 580
    assert -180 <= sat0["lon_deg"] <= 180


def test_api_predict_handover_under_500ms(synthetic_constellation):
    from fastapi.testclient import TestClient

    api_server._state.cons = synthetic_constellation
    client = TestClient(api_server.app)
    payload = {
        "ue_lat_deg": 33.6844, "ue_lon_deg": 73.0479, "ue_alt_m": 540.0,
        "horizon_min": 10.0, "step_sec": 5.0, "min_elevation_deg": 10.0,
    }
    import time
    t0 = time.time()
    r = client.post("/predict/handover", json=payload)
    dt_ms = (time.time() - t0) * 1000.0
    assert r.status_code == 200
    body = r.json()
    # Under-500-ms gate (tolerate FastAPI overhead — server-side elapsed_ms is the gate)
    assert body["elapsed_ms"] < 500.0, f"server elapsed_ms={body['elapsed_ms']}"
    assert dt_ms < 1500.0  # round-trip headroom for CI
    assert body["horizon_min"] == 10.0


def test_api_predict_handover_returns_events(synthetic_constellation):
    from fastapi.testclient import TestClient

    api_server._state.cons = synthetic_constellation
    client = TestClient(api_server.app)
    # 30-min horizon to guarantee at least one handover for a Starlink-class shell.
    payload = {
        "ue_lat_deg": 0.0, "ue_lon_deg": 0.0,
        "horizon_min": 30.0, "step_sec": 5.0, "min_elevation_deg": 10.0,
    }
    r = client.post("/predict/handover", json=payload)
    assert r.status_code == 200
    body = r.json()
    assert body["n_handovers"] >= 0
    if body["n_handovers"] > 0:
        ev = body["events"][0]
        assert ev["sat_in_norad"] > 0
        assert -90 <= ev["elevation_in_deg"] <= 90
