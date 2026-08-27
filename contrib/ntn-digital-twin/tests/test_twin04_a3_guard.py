"""TWIN-03/TWIN-04: the actuating path is reachable and guarded.

TWIN-03: emit_predictions_file() had exactly one caller - a test - while
run_iteration() never called it and main() exposed no flag for it. So the file
the C++ OranNtnTwinPredictionConsumer reads could not be produced by any shipped
command, and the twin->sim loop both READMEs advertise as closed could not be
executed.

TWIN-04: the exporter took a bare argmax over elevation and appended a
prediction on every serving change - no hysteresis, no time-to-trigger, no
minimum service time - and it is the path the C++ side actuates on. The guarded
REST variant, which nothing consumes, triggered on elevation DEGREES against a
simulator whose A3 uses SINR in dB. Elevation and SINR are not monotonically
related once antenna pattern, scan loss and P.618/P.676 attenuation enter, so
the two admit different handovers.
"""

from __future__ import annotations

import datetime as dt
import math
from pathlib import Path

from ntn_digital_twin.twin_loop import (
    LoopConfig,
    _slant_range_m,
    _snr_db,
    emit_predictions_file,
    run_iteration,
    LoopStats,
)


def _walker_cfg(**kw) -> LoopConfig:
    base = dict(
        source="walker",
        walker_planes=6,
        walker_sats_per_plane=12,
        walker_altitude_km=600.0,
        walker_inclination_deg=53.0,
        walker_phasing_factor=1,
        epoch_unix_s=1735689600.0,
        max_sats=72,
    )
    base.update(kw)
    return LoopConfig(**base)


def test_slant_range_closed_form():
    """The slant range must match the spherical closed form."""
    # Straight overhead: the slant range IS the altitude.
    assert abs(_slant_range_m(90.0, 600.0) - 600e3) < 1.0
    # At the horizon it is the tangent length, sqrt((R+h)^2 - R^2).
    R, h = 6371e3, 600e3
    horizon = math.sqrt((R + h) ** 2 - R * R)
    assert abs(_slant_range_m(0.0, 600.0) - horizon) < 1.0
    # And it decreases monotonically with elevation.
    prev = _slant_range_m(0.0, 600.0)
    for el in range(5, 91, 5):
        cur = _slant_range_m(float(el), 600.0)
        assert cur < prev, f"slant range must fall as elevation rises (at {el} deg)"
        prev = cur


def test_snr_is_a_db_quantity_that_tracks_geometry():
    """TWIN-04: the trigger quantity is an SNR in dB, not elevation degrees."""
    cfg = _walker_cfg()
    lo = _snr_db(cfg, 10.0, 600.0)
    hi = _snr_db(cfg, 80.0, 600.0)
    assert hi > lo, "a higher elevation gives a shorter slant and a better SNR"
    # The gain is the free-space term over the range ratio, so it is a few dB -
    # not the tens of degrees an elevation margin would report. That difference
    # in UNITS is the whole point: a 3 dB A3 offset and a 3 degree elevation
    # margin are not the same threshold.
    assert 1.0 < (hi - lo) < 20.0, f"unexpected SNR span {hi - lo:.1f} dB"
    # Below the horizon there is no link.
    assert _snr_db(cfg, 0.0, 600.0) < -100.0


def test_a3_offset_suppresses_marginal_switches(tmp_path: Path):
    """A larger A3 offset must emit no MORE predictions than a smaller one.

    This is the property hysteresis has: raising the bar cannot admit a
    handover that a lower bar rejected.
    """
    counts = []
    for offset in (0.0, 3.0, 12.0):
        out = tmp_path / f"pred-{offset}.txt"
        emit_predictions_file(
            _walker_cfg(a3_offset_db=offset),
            observer_lat_deg=33.6844,
            observer_lon_deg=73.0479,
            observer_alt_m=540.0,
            out_path=out,
            horizon_s=45 * 60.0,
            step_s=15.0,
        )
        counts.append(sum(1 for ln in out.read_text().splitlines()
                          if ln and not ln.startswith("#")))
    assert counts[0] >= counts[1] >= counts[2], (
        f"a larger A3 offset must not admit more handovers: {counts}")
    assert counts[0] > 0, "the permissive case must still produce handovers"
    # Monotonicity alone is too weak: it holds trivially if the offset is
    # ignored and every count is identical, which is exactly what the
    # pre-fix bare argmax produced. The offset must STRICTLY reduce the
    # count, or it is not filtering anything. Measured: 14 at 0 dB, 11 at
    # 1 dB, 7 at 3 dB, saturating at 7 because the remaining switches are
    # genuine pass-driven ones where the new satellite is overwhelmingly
    # better.
    assert counts[2] < counts[0], (
        f"a 12 dB A3 offset must admit strictly fewer handovers than 0 dB; "
        f"equal counts {counts} mean the offset is being ignored and the "
        "exporter is still taking a bare argmax")


def test_min_service_time_enforces_a_dwell(tmp_path: Path):
    """No two emitted predictions may be closer than the minimum service time."""
    out = tmp_path / "pred-dwell.txt"
    dwell = 600.0
    emit_predictions_file(
        _walker_cfg(min_service_time_s=dwell),
        observer_lat_deg=33.6844,
        observer_lon_deg=73.0479,
        observer_alt_m=540.0,
        out_path=out,
        horizon_s=90 * 60.0,
        step_s=15.0,
    )
    times = [float(ln.split(",")[0]) for ln in out.read_text().splitlines()
             if ln and not ln.startswith("#")]
    for a, b in zip(times, times[1:]):
        assert b - a >= dwell - 1e-6, (
            f"switches {a} and {b} are closer than the {dwell} s minimum service time; "
            "without this the twin's foresight reaches ns-3 in its most "
            "ping-pong-prone form")


def test_run_iteration_emits_predictions(tmp_path: Path):
    """TWIN-03: the loop itself must produce the file, not only a test."""
    out = tmp_path / "loop-pred.txt"
    cfg = _walker_cfg(
        predictions_path=out,
        predictions_horizon_s=45 * 60.0,
        predictions_step_s=15.0,
        observer_lat_deg=33.6844,
        observer_lon_deg=73.0479,
        observer_alt_m=540.0,
        czml_path=tmp_path / "t.czml",
    )
    stats = LoopStats()
    run_iteration(cfg, stats)
    assert out.exists(), (
        "run_iteration must emit the prediction file when it is configured; "
        "before TWIN-03 it never called the exporter, so no shipped command "
        "could produce the file the C++ consumer reads")
    assert stats.last_prediction_count > 0, "and report how many it wrote"
    # The format is the C++ LoadPredictionsFromFile contract.
    lines = [ln for ln in out.read_text().splitlines() if ln and not ln.startswith("#")]
    for ln in lines:
        parts = ln.split(",")
        assert len(parts) == 4, f"expected t_s,ueId,gnbId,confidence; got {ln!r}"
        float(parts[0]); int(parts[1]); int(parts[2]); float(parts[3])


def test_predictions_path_unset_emits_nothing(tmp_path: Path):
    """Default off: an existing scenario keeps its behaviour."""
    cfg = _walker_cfg(czml_path=tmp_path / "t.czml")
    stats = LoopStats()
    run_iteration(cfg, stats)
    assert stats.last_prediction_count == 0


# ---------------------------------------------------------------------------
# TWIN-04, the part that closes the finding: ONE guard, not two that agree by
# coincidence. The audit's fix asks for a test "asserting the exported file and
# the REST endpoint produce the same event list for the same config".
# ---------------------------------------------------------------------------

def test_exporter_and_rest_agree_event_for_event(tmp_path: Path):
    """The exported file and /predict/handover must produce the SAME sequence.

    This is the assertion the two implementations could never have passed. They
    differed in three ways that no single-path test could see: the exporter
    ranked candidates by SNR while the endpoint ranked by elevation, the
    exporter had no minimum-elevation filter, and the exporter counted initial
    acquisition as a handover so its list was one longer at the front.
    """
    from fastapi.testclient import TestClient
    from ntn_digital_twin.api import server as api_server
    from ntn_digital_twin.twin_loop import fetch_constellation

    epoch_unix = 1735689600.0
    epoch = dt.datetime.fromtimestamp(epoch_unix, tz=dt.timezone.utc)
    lat, lon = 12.0, 34.0
    horizon_s, step_s = 45 * 60.0, 10.0
    offset_db, ttt_s, min_svc_s = 3.0, 10.0, 30.0
    alt_km = 600.0

    cfg = _walker_cfg(
        walker_altitude_km=alt_km,
        a3_offset_db=offset_db,
        a3_time_to_trigger_s=ttt_s,
        min_service_time_s=min_svc_s,
    )
    out = tmp_path / "twin-predictions.txt"
    emit_predictions_file(
        cfg,
        observer_lat_deg=lat, observer_lon_deg=lon, observer_alt_m=0.0,
        out_path=out, horizon_s=horizon_s, step_s=step_s, ue_id=0,
    )
    # (t_s, e2_node_id) from the file the C++ consumer reads.
    file_events = []
    for line in out.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split(",")
        file_events.append((float(parts[0]), int(parts[2])))

    # The endpoint, over the SAME constellation, epoch, observer and guard.
    # The SAME constellation object the exporter built, not a reconstruction:
    # a reconstruction that differed would make the comparison vacuous.
    cons, _ = fetch_constellation(cfg)
    api_server._state.cons = cons
    norad_of_index = {s.norad_id: i for i, s in enumerate(api_server._state.cons)}
    client = TestClient(api_server.app)
    resp = client.post("/predict/handover", json={
        "ue_lat_deg": lat, "ue_lon_deg": lon,
        "horizon_min": horizon_s / 60.0, "step_sec": step_s,
        # The exporter admits any satellite above the horizon; match it, or the
        # two are simply being asked different questions.
        "min_elevation_deg": 0.0,
        "margin_quantity": "db", "a3_offset_db": offset_db,
        "time_to_trigger_sec": ttt_s, "min_service_sec": min_svc_s,
        "link_eirp_dbm": cfg.link_eirp_dbm,
        "link_gt_db_per_k": cfg.link_gt_db_per_k,
        "link_bandwidth_hz": cfg.link_bandwidth_hz,
        "link_frequency_hz": cfg.link_frequency_hz,
        "start_iso": epoch.isoformat(),
    })
    assert resp.status_code == 200, resp.text
    rest_events = []
    for ev in resp.json()["events"]:
        when = dt.datetime.fromisoformat(ev["time_iso"])
        rest_events.append(((when - epoch).total_seconds(),
                            norad_of_index[ev["sat_in_norad"]] + 1))

    # A shell that never hands over would let a broken guard pass this.
    assert len(file_events) >= 2, (
        f"scenario must exercise handovers to be a real comparison, "
        f"got {len(file_events)}")
    assert file_events == rest_events, (
        f"the actuating path and the REST path disagree.\n"
        f"  file: {file_events}\n"
        f"  rest: {rest_events}")


def test_shared_guard_is_the_only_guard():
    """Neither caller may reintroduce a private copy of the A3 comparison.

    The finding was not that the rule was wrong in one place; it was that the
    rule existed twice and the copies drifted. A grep is a blunt instrument, but
    it fails loudly the moment someone inlines the comparison again.
    """
    import inspect
    from ntn_digital_twin import twin_loop
    from ntn_digital_twin.api import server as api_server

    for mod in (twin_loop, api_server):
        src = inspect.getsource(mod)
        assert "a3_guard" in src, f"{mod.__name__} must use the shared guard"
        for banned in ("+ req.a3_offset_db", "+ cfg.a3_offset_db",
                       "+ req.hysteresis_deg", "+ cfg.hysteresis_deg"):
            assert banned not in src, (
                f"{mod.__name__} reintroduced a private A3 comparison "
                f"({banned!r}); the guard belongs in a3_guard.py")
