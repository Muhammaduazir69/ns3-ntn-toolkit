"""Smoke tests: TLE parsing, presets generate valid TLEs, SGP4 propagates,
SNS3 writer emits the expected file tree.
"""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

from ntn_constellation.feeds import TleRecord, parse_tle_text, write_tle_file, iter_tle_file
from ntn_constellation.isl import build_isl_topology, grid_isl_topology_walker
from ntn_constellation.ns3_export import write_sns3_scenario
from ntn_constellation.presets import PRESETS, from_preset, walker_delta, walker_star
from ntn_constellation.propagator import Constellation


ISS_TLE = (
    "ISS (ZARYA)",
    "1 25544U 98067A   24001.50000000  .00010000  00000-0  18000-3 0  9999",
    "2 25544  51.6400 100.0000 0001000   0.0000  60.0000 15.50000000  0009",
)


def test_tle_record_roundtrip():
    r = TleRecord(name=ISS_TLE[0], line1=ISS_TLE[1], line2=ISS_TLE[2])
    assert r.norad_id == 25544
    assert r.epoch.year == 2024


def test_parse_tle_text_with_blank_lines():
    text = "\n".join([ISS_TLE[0], ISS_TLE[1], ISS_TLE[2], "", ""])
    records = parse_tle_text(text)
    assert len(records) == 1
    assert records[0].norad_id == 25544


def test_walker_star_generates_expected_count():
    tles = walker_star(
        name_prefix="OneWeb",
        altitude_km=1200.0,
        inclination_deg=87.9,
        num_planes=18,
        sats_per_plane=36,
    )
    assert len(tles) == 18 * 36
    for t in tles:
        assert len(t.line1) == 69
        assert len(t.line2) == 69
        assert t.line1.startswith("1 ")
        assert t.line2.startswith("2 ")


def test_walker_delta_phasing():
    tles = walker_delta(
        name_prefix="Kuiper",
        altitude_km=630.0,
        inclination_deg=51.9,
        num_planes=2,
        sats_per_plane=4,
        phasing_factor=1,
    )
    assert len(tles) == 8
    raans = sorted({float(t.line2[17:25]) for t in tles})
    assert len(raans) == 2


def test_preset_iridium_propagates():
    tles = from_preset(PRESETS["iridium-next"])
    constellation = Constellation.from_tles(tles[:6])
    when = datetime.now(tz=timezone.utc)
    states = constellation.state_vectors(when)
    assert len(states) == 6
    for s in states:
        r = (s.r_eci_km[0] ** 2 + s.r_eci_km[1] ** 2 + s.r_eci_km[2] ** 2) ** 0.5
        assert 6700 < r < 7400


def test_iss_visible_above_pakistan(tmp_path):
    constellation = Constellation.from_tles(
        [TleRecord(name=ISS_TLE[0], line1=ISS_TLE[1], line2=ISS_TLE[2])]
    )
    found_visible = False
    t0 = datetime(2024, 1, 1, 0, 0, 0, tzinfo=timezone.utc)
    for minute in range(0, 24 * 60, 5):
        when = t0 + timedelta(minutes=minute)
        visible = constellation.visible_from(
            when, observer_lat_deg=33.6844, observer_lon_deg=73.0479,
            min_elevation_deg=5.0,
        )
        if visible:
            found_visible = True
            break
    assert found_visible, "ISS should pass over Islamabad at least once in 24h"


def test_isl_grid_walker_topology_has_expected_edges():
    edges = grid_isl_topology_walker(num_planes=4, sats_per_plane=8)
    assert len(edges) == 4 * 8 + 3 * 8


def test_isl_knn_respects_max_range():
    tles = from_preset(PRESETS["iridium-next"])[:10]
    constellation = Constellation.from_tles(tles)
    edges = build_isl_topology(constellation, when=datetime.now(tz=timezone.utc),
                               k_nearest=2, max_range_km=20000.0)
    assert all(i < j for i, j in edges)
    assert len(edges) <= 10 * 2


def test_write_sns3_scenario_emits_correct_layout(tmp_path: Path):
    tles = from_preset(PRESETS["iridium-next"])[:6]
    isls = grid_isl_topology_walker(num_planes=1, sats_per_plane=6)
    pos = write_sns3_scenario(
        scenario_dir=tmp_path / "scn",
        tles=tles,
        start_date=datetime(2026, 5, 4, 0, 0, 0, tzinfo=timezone.utc),
        isls=isls,
        gw_positions_lla_m=[(0.0, 0.0, 0.0)],
        ut_positions_lla_m=[(45.0, 90.0, 100.0)],
    )
    assert (pos / "start_date.txt").read_text().startswith("2026-05-04 00:00:00")
    body = (pos / "tles.txt").read_text().splitlines()
    assert body[0] == "6"
    assert len(body) == 1 + 6 * 3
    isl_body = (pos / "isls.txt").read_text().splitlines()
    assert isl_body[0] == str(len(isls))
    assert (pos / "gw_positions.txt").exists()
    assert (pos / "ut_positions.txt").exists()


def test_write_tle_file_roundtrip(tmp_path: Path):
    tles = from_preset(PRESETS["iridium-next"])[:3]
    p = write_tle_file(tles, tmp_path / "raw.tle")
    back = list(iter_tle_file(p))
    assert len(back) == 3
    assert back[0].name == tles[0].name
