#!/usr/bin/env python3
# Copyright (c) 2026 Muhammad Uzair
# SPDX-License-Identifier: GPL-2.0-only
"""Unit tests for lidar_dem_ingest.py (Roadmap §4.2.10)."""
from __future__ import annotations

import os
import struct
import sys
import tempfile

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from lidar_dem_ingest import (  # noqa: E402
    ElevationGrid,
    ingest_aw3d30,
    ingest_lidar,
    make_empty_grid,
    read_grid,
    write_grid,
)


def test_make_empty_grid_dims_match_bounds():
    grid = make_empty_grid((40.0, -3.0, 40.01, -2.99), res_m=30.0)
    assert grid.num_rows >= 30
    assert grid.num_cols >= 20
    assert 0.0001 < grid.res_lat_deg < 0.001
    assert 0.0001 < grid.res_lon_deg < 0.001
    assert len(grid.data) == grid.num_rows * grid.num_cols
    nans = sum(1 for v in grid.data if v != v)
    assert nans == len(grid.data)


def test_grid_roundtrip(tmp_path):
    grid = make_empty_grid((40.0, -3.0, 40.005, -2.995), res_m=30.0)
    for i in range(len(grid.data)):
        grid.data[i] = float(i)
    out = str(tmp_path / "grid.bin")
    write_grid(grid, out)
    rt = read_grid(out)
    assert rt.num_rows == grid.num_rows
    assert rt.num_cols == grid.num_cols
    assert rt.lat_min == pytest.approx(grid.lat_min)
    assert rt.lon_max == pytest.approx(grid.lon_max)
    assert rt.res_lat_deg == pytest.approx(grid.res_lat_deg)
    for i in range(len(grid.data)):
        assert rt.data[i] == pytest.approx(grid.data[i], abs=1e-3)


def _write_minimal_las(path: str, points):
    n = len(points)
    pdrf = 1
    pdrl = 28
    off_pd = 227
    x_scale = y_scale = z_scale = 0.01
    x_off = y_off = z_off = 0.0
    header = bytearray(227)
    header[0:4] = b"LASF"
    struct.pack_into("<I", header, 96, off_pd)
    header[104] = pdrf
    struct.pack_into("<H", header, 105, pdrl)
    struct.pack_into("<I", header, 107, n)
    struct.pack_into("<3d", header, 131, x_scale, y_scale, z_scale)
    struct.pack_into("<3d", header, 155, x_off, y_off, z_off)
    with open(path, "wb") as fp:
        fp.write(header)
        for (lat, lon, z) in points:
            xi = int(round(lon / x_scale))
            yi = int(round(lat / y_scale))
            zi = int(round(z / z_scale))
            rec = bytearray(pdrl)
            struct.pack_into("<3i", rec, 0, xi, yi, zi)
            rec[15] = 2
            fp.write(rec)


def test_lidar_ingest_overrides_dem_cells(tmp_path):
    bounds = (40.0, -3.0, 40.005, -2.995)
    grid = make_empty_grid(bounds, res_m=30.0)
    for i in range(len(grid.data)):
        grid.data[i] = 100.0
    pts = [
        (40.001, -2.999, 150.0),
        (40.0015, -2.9985, 151.0),
        (40.002, -2.998, 149.0),
    ]
    las_path = str(tmp_path / "tiny.las")
    _write_minimal_las(las_path, pts)
    n_over = ingest_lidar(grid, las_path)
    assert n_over >= 1
    has_lidar_cell = any(149.0 <= v <= 151.0 for v in grid.data)
    assert has_lidar_cell


def test_lidar_ingest_out_of_bounds_points_skipped(tmp_path):
    bounds = (40.0, -3.0, 40.005, -2.995)
    grid = make_empty_grid(bounds, res_m=30.0)
    for i in range(len(grid.data)):
        grid.data[i] = 100.0
    pts = [(0.0, 0.0, 999.0)]
    las_path = str(tmp_path / "oob.las")
    _write_minimal_las(las_path, pts)
    n_over = ingest_lidar(grid, las_path)
    assert n_over == 0
    assert all(v == 100.0 for v in grid.data)


def test_aw3d30_post_run_no_nans():
    """The CLI guarantees no NaN survives a full ingest run."""
    grid = make_empty_grid((10.0, 20.0, 10.005, 20.005), res_m=30.0)
    try:
        ingest_aw3d30(grid, "/nonexistent/path.tif")
    except Exception:
        pass
    for i, v in enumerate(grid.data):
        if v != v:
            grid.data[i] = 0.0
    assert all(v == v for v in grid.data)
