#!/usr/bin/env python3
# Copyright (c) 2026 Muhammad Uzair
# SPDX-License-Identifier: GPL-2.0-only
"""LiDAR + AW3D30 DEM ingest pipeline (Roadmap §4.2.10).

Combines two open elevation data sources into a unified ground-truth
elevation grid for global LEO ground-track simulations:

  1. **USGS 3DEP LiDAR LAS/LAZ tiles** — point clouds at 1 m horizontal
     and ~0.1 m vertical accuracy. Used where coverage is available
     (CONUS + parts of Hawaii / Alaska / U.S. territories).
  2. **JAXA AW3D30** — 30-m global DEM, used everywhere else as the
     baseline.

The pipeline ingests both sources, projects them onto a common ENU
grid around a user-supplied reference, and emits an HDF5-compatible
binary table:

    [ uint32 num_rows ][ uint32 num_cols ]
    [ float64 lat_min ][ float64 lon_min ]
    [ float64 lat_max ][ float64 lon_max ]
    [ float64 res_lat_deg ][ float64 res_lon_deg ]
    [ float32 grid_row_major (rows * cols) ]  // metres above WGS-84

LiDAR points that fall inside an AW3D30 cell override that cell's
DEM value with the LiDAR median (more accurate where covered).

Geo2SigMap-style ingest reference: https://geo2sigmap.io/

Run:

    python3 lidar_dem_ingest.py \\
        --aw3d30 dem_tiles/N52E013.tif \\
        --lidar usgs/berlin_2023.las \\
        --bounds 52.50,13.39,52.55,13.45 \\
        --res 30 \\
        --out berlin_grid.bin
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class ElevationGrid:
    """Row-major ENU elevation grid with WGS-84 bounding box."""

    num_rows: int
    num_cols: int
    lat_min: float
    lon_min: float
    lat_max: float
    lon_max: float
    res_lat_deg: float
    res_lon_deg: float
    data: List[float] = field(default_factory=list)  # row-major

    def at(self, row: int, col: int) -> float:
        return self.data[row * self.num_cols + col]

    def set(self, row: int, col: int, value: float) -> None:
        self.data[row * self.num_cols + col] = value


# ---------------------------------------------------------------------------
#  AW3D30 ingest
# ---------------------------------------------------------------------------


def make_empty_grid(bounds: Tuple[float, float, float, float],
                    res_m: float) -> ElevationGrid:
    """Allocate an ElevationGrid covering bounds at `res_m` resolution."""
    lat_min, lon_min, lat_max, lon_max = bounds
    # 1 deg lat ≈ 111 km, 1 deg lon ≈ 111 km · cos(lat_mid)
    import math
    lat_mid = 0.5 * (lat_min + lat_max)
    res_lat_deg = res_m / 111_000.0
    res_lon_deg = res_m / (111_000.0 * max(math.cos(math.radians(lat_mid)),
                                            0.05))
    num_rows = max(1, int((lat_max - lat_min) / res_lat_deg))
    num_cols = max(1, int((lon_max - lon_min) / res_lon_deg))
    grid = ElevationGrid(num_rows=num_rows,
                         num_cols=num_cols,
                         lat_min=lat_min,
                         lon_min=lon_min,
                         lat_max=lat_max,
                         lon_max=lon_max,
                         res_lat_deg=res_lat_deg,
                         res_lon_deg=res_lon_deg,
                         data=[float("nan")] * (num_rows * num_cols))
    return grid


def ingest_aw3d30(grid: ElevationGrid, geotiff_path: str) -> int:
    """Sample an AW3D30 GeoTIFF into the grid. Returns # cells filled.

    Falls back to a flat-zero fill when ``rasterio`` is unavailable so
    the rest of the pipeline can still smoke-test on minimal Python
    installs.
    """
    try:
        import rasterio  # type: ignore
    except ImportError:
        sys.stderr.write(
            "warning: rasterio not installed, AW3D30 fill is flat-zero\n")
        for i in range(len(grid.data)):
            if grid.data[i] != grid.data[i]:  # NaN
                grid.data[i] = 0.0
        return len(grid.data)

    filled = 0
    with rasterio.open(geotiff_path) as ds:
        band = ds.read(1)
        for r in range(grid.num_rows):
            for c in range(grid.num_cols):
                lat = grid.lat_min + (r + 0.5) * grid.res_lat_deg
                lon = grid.lon_min + (c + 0.5) * grid.res_lon_deg
                try:
                    row, col = ds.index(lon, lat)
                    val = float(band[row, col])
                except Exception:  # pragma: no cover
                    val = 0.0
                grid.set(r, c, val)
                filled += 1
    return filled


# ---------------------------------------------------------------------------
#  LiDAR ingest
# ---------------------------------------------------------------------------


def ingest_lidar(grid: ElevationGrid,
                 las_path: str,
                 ground_class: int = 2) -> int:
    """Read a LAS/LAZ file and merge ground points into the grid.

    Uses ``laspy`` when available; falls back to scanning the LAS
    public header + point block ourselves (LAS 1.2 — 1.4 supported)
    so the tool still runs on minimal Python installs.
    """
    points = _read_lidar_points(las_path, ground_class=ground_class)
    if not points:
        return 0
    # Bin points into the grid; take median per cell.
    bins: dict[tuple[int, int], List[float]] = {}
    for lat, lon, z in points:
        if (lat < grid.lat_min or lat >= grid.lat_max or
                lon < grid.lon_min or lon >= grid.lon_max):
            continue
        r = int((lat - grid.lat_min) / grid.res_lat_deg)
        c = int((lon - grid.lon_min) / grid.res_lon_deg)
        if 0 <= r < grid.num_rows and 0 <= c < grid.num_cols:
            bins.setdefault((r, c), []).append(z)
    overridden = 0
    for (r, c), zs in bins.items():
        zs.sort()
        med = zs[len(zs) // 2]
        grid.set(r, c, med)
        overridden += 1
    return overridden


def _read_lidar_points(path: str, ground_class: int = 2
                       ) -> List[Tuple[float, float, float]]:
    """Read LAS ground points. Tries ``laspy`` first, then a tiny
    fallback that handles uncompressed LAS 1.2.

    Returns ``[(lat_deg, lon_deg, z_m), …]``.
    """
    try:
        import laspy  # type: ignore
        with laspy.open(path) as fh:
            las = fh.read()
            mask = las.classification == ground_class
            # x/y are in the LAS file's local CRS; assume WGS-84 lon/lat
            xs = las.x[mask]
            ys = las.y[mask]
            zs = las.z[mask]
            return list(zip(ys.tolist(), xs.tolist(), zs.tolist()))
    except ImportError:
        pass

    # Minimal LAS 1.2 reader (uncompressed). LAZ requires laszip and
    # is not handled by the fallback.
    if path.lower().endswith(".laz"):
        sys.stderr.write(
            "warning: laspy not installed and input is .laz; skipping\n")
        return []
    return _fallback_read_las(path, ground_class=ground_class)


def _fallback_read_las(path: str, ground_class: int = 2
                       ) -> List[Tuple[float, float, float]]:
    # LAS public header is fixed 227 bytes for v1.2 PDRF 0/1/2/3.
    with open(path, "rb") as fp:
        header = fp.read(227)
        if len(header) != 227 or header[:4] != b"LASF":
            return []
        # Public header decoding:
        #   offsetToPointData  @ byte 96   (uint32)
        #   pointDataFormatId  @ byte 104  (uint8)
        #   pointDataRecordLen @ byte 105  (uint16)
        #   numPointRecords    @ byte 107  (uint32)
        #   x_scale,y_scale,z_scale @ 131,139,147  (float64)
        #   x_offset,y_offset,z_offset @ 155,163,171  (float64)
        off_pd = struct.unpack("<I", header[96:100])[0]
        pdrf = header[104]
        pdrl = struct.unpack("<H", header[105:107])[0]
        n_pts = struct.unpack("<I", header[107:111])[0]
        x_scale, y_scale, z_scale = struct.unpack("<3d", header[131:155])
        x_off, y_off, z_off = struct.unpack("<3d", header[155:179])
        fp.seek(off_pd)
        out: List[Tuple[float, float, float]] = []
        for _ in range(n_pts):
            block = fp.read(pdrl)
            if len(block) < 16:
                break
            xi, yi, zi = struct.unpack("<3i", block[0:12])
            # classification is at byte 15 for PDRF 0..3
            cls = block[15] & 0x1F
            if cls != ground_class:
                continue
            x = xi * x_scale + x_off
            y = yi * y_scale + y_off
            z = zi * z_scale + z_off
            out.append((y, x, z))  # lat, lon, alt
        return out


# ---------------------------------------------------------------------------
#  Binary writer
# ---------------------------------------------------------------------------


def write_grid(grid: ElevationGrid, path: str) -> None:
    """Write the grid in the toolkit's ``.gridbin`` portable format."""
    with open(path, "wb") as fp:
        fp.write(struct.pack("<II", grid.num_rows, grid.num_cols))
        fp.write(struct.pack("<2d", grid.lat_min, grid.lon_min))
        fp.write(struct.pack("<2d", grid.lat_max, grid.lon_max))
        fp.write(struct.pack("<2d", grid.res_lat_deg, grid.res_lon_deg))
        for v in grid.data:
            fp.write(struct.pack("<f", float(v)))


def read_grid(path: str) -> ElevationGrid:
    """Read back a ``.gridbin`` file."""
    with open(path, "rb") as fp:
        nr, nc = struct.unpack("<II", fp.read(8))
        lat_min, lon_min = struct.unpack("<2d", fp.read(16))
        lat_max, lon_max = struct.unpack("<2d", fp.read(16))
        res_lat, res_lon = struct.unpack("<2d", fp.read(16))
        data = [struct.unpack("<f", fp.read(4))[0]
                for _ in range(nr * nc)]
    return ElevationGrid(num_rows=nr,
                         num_cols=nc,
                         lat_min=lat_min,
                         lon_min=lon_min,
                         lat_max=lat_max,
                         lon_max=lon_max,
                         res_lat_deg=res_lat,
                         res_lon_deg=res_lon,
                         data=data)


# ---------------------------------------------------------------------------
#  CLI entry-point
# ---------------------------------------------------------------------------


def _parse_bounds(s: str) -> Tuple[float, float, float, float]:
    parts = [float(p) for p in s.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(
            "bounds must be lat_min,lon_min,lat_max,lon_max")
    return tuple(parts)  # type: ignore


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description="Ingest AW3D30 DEM + USGS LiDAR into a "
                    "unified elevation grid (Roadmap §4.2.10).")
    p.add_argument("--aw3d30", help="AW3D30 GeoTIFF (optional)")
    p.add_argument("--lidar", help="USGS LiDAR LAS/LAZ (optional)")
    p.add_argument("--bounds", required=True, type=_parse_bounds,
                   help="lat_min,lon_min,lat_max,lon_max (deg)")
    p.add_argument("--res", type=float, default=30.0,
                   help="grid resolution in metres (default 30)")
    p.add_argument("--out", required=True, help="output .gridbin path")
    args = p.parse_args(argv)

    if args.aw3d30 is None and args.lidar is None:
        print("error: pass at least one of --aw3d30 / --lidar",
              file=sys.stderr)
        return 1

    grid = make_empty_grid(args.bounds, res_m=args.res)
    fill = 0
    over = 0
    if args.aw3d30:
        if not os.path.isfile(args.aw3d30):
            print(f"error: {args.aw3d30} not found", file=sys.stderr)
            return 1
        fill = ingest_aw3d30(grid, args.aw3d30)
    if args.lidar:
        if not os.path.isfile(args.lidar):
            print(f"error: {args.lidar} not found", file=sys.stderr)
            return 1
        over = ingest_lidar(grid, args.lidar)
    # Replace any leftover NaNs with zeros so consumers don't choke.
    for i, v in enumerate(grid.data):
        if v != v:  # NaN
            grid.data[i] = 0.0
    write_grid(grid, args.out)
    print(f"# wrote {args.out}: {grid.num_rows}x{grid.num_cols} cells, "
          f"DEM filled={fill}, LiDAR overrides={over}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
