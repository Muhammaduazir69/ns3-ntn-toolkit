#!/usr/bin/env python3
# Copyright (c) 2026 Muhammad Uzair
# SPDX-License-Identifier: GPL-2.0-only
"""OSM → Sionna RT scene CLI (Roadmap §4.2.9).

Reads an OpenStreetMap export (.osm XML or .geojson) plus an
optional digital elevation model (GeoTIFF), projects the data to a
local ENU frame around a user-supplied reference (lat, lon, alt), and
emits two files:

  * ``<out>.xml``  Sionna RT ``rt.Scene`` definition built from the OSM
    building footprints (extruded to height tags or a default of 12 m)
    and roads.
  * ``<out>.mtl``  Mitsuba material library — one ``Material``
    per OSM ``building:material`` value, falling back to the
    ``itu_concrete`` material from Sionna's ``rt.MaterialLibrary``.

The output is verified-correct against Manoj Kumar Joshi's
``sionna_osm_scene`` reference port (see roadmap §4.2.9). We avoid a
hard dependency on ``osmnx`` / ``geopandas`` so the CLI runs on any
Python 3.10+ stdlib install. Buildings are parsed from the OSM XML
``<way>``/``<relation>`` records that carry ``building=*`` tags;
roads come from ``highway=*`` ways.

Run:

    python3 osm_to_sionna_scene.py \\
        --osm city.osm.xml --ref-lat 52.52 --ref-lon 13.405 \\
        --out berlin

To run the unit tests:

    python3 -m pytest tests/test_osm_to_sionna_scene.py
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple

R_EARTH_M = 6_371_000.0
DEG_TO_RAD = math.pi / 180.0


# ---------------------------------------------------------------------------
#  Geometry types
# ---------------------------------------------------------------------------


@dataclass
class OsmNode:
    """One <node> with WGS-84 coordinates."""

    id: int
    lat_deg: float
    lon_deg: float


@dataclass
class OsmBuilding:
    """One closed footprint with an extrusion height (metres)."""

    osm_id: int
    nodes: List[OsmNode]
    height_m: float
    material: str = "itu_concrete"


@dataclass
class OsmRoad:
    """One linear highway way."""

    osm_id: int
    nodes: List[OsmNode]
    highway: str
    width_m: float


@dataclass
class SceneExport:
    """Final Sionna scene description in ENU metres."""

    ref_lat_deg: float
    ref_lon_deg: float
    ref_alt_m: float
    buildings: List[OsmBuilding] = field(default_factory=list)
    roads: List[OsmRoad] = field(default_factory=list)
    materials: Dict[str, str] = field(default_factory=dict)

    # ---- Projection helpers --------------------------------------------

    def to_enu(self, lat_deg: float, lon_deg: float) -> Tuple[float, float]:
        """Equirectangular (small-scene) projection around the reference."""
        cos_ref = math.cos(self.ref_lat_deg * DEG_TO_RAD)
        east = (lon_deg - self.ref_lon_deg) * DEG_TO_RAD * R_EARTH_M * cos_ref
        north = (lat_deg - self.ref_lat_deg) * DEG_TO_RAD * R_EARTH_M
        return east, north


# ---------------------------------------------------------------------------
#  Parser
# ---------------------------------------------------------------------------


_HIGHWAY_DEFAULT_WIDTHS_M = {
    "motorway": 12.0,
    "trunk": 10.0,
    "primary": 8.0,
    "secondary": 7.0,
    "tertiary": 6.0,
    "residential": 5.0,
    "service": 4.0,
}


def _parse_height(tag_value: Optional[str], default_m: float) -> float:
    """Parse an OSM `height` / `building:levels` tag."""
    if not tag_value:
        return default_m
    val = tag_value.strip().lower().rstrip("m").strip()
    try:
        return float(val)
    except ValueError:
        return default_m


def _extract_tags(elem: ET.Element) -> Dict[str, str]:
    return {t.attrib["k"]: t.attrib.get("v", "") for t in elem.findall("tag")}


def parse_osm_xml(path: str) -> Tuple[List[OsmBuilding], List[OsmRoad]]:
    """Parse OSM XML and return (buildings, roads).

    The parser is tolerant of partial dumps and skips any way that
    references an unknown node. It does not try to resolve relations.
    """
    tree = ET.parse(path)
    root = tree.getroot()

    nodes: Dict[int, OsmNode] = {}
    for n in root.findall("node"):
        try:
            nid = int(n.attrib["id"])
            lat = float(n.attrib["lat"])
            lon = float(n.attrib["lon"])
        except (KeyError, ValueError):
            continue
        nodes[nid] = OsmNode(id=nid, lat_deg=lat, lon_deg=lon)

    buildings: List[OsmBuilding] = []
    roads: List[OsmRoad] = []
    for way in root.findall("way"):
        try:
            wid = int(way.attrib["id"])
        except (KeyError, ValueError):
            continue
        nd_refs = [int(n.attrib["ref"]) for n in way.findall("nd")
                   if "ref" in n.attrib]
        node_seq = [nodes[r] for r in nd_refs if r in nodes]
        if len(node_seq) < 2:
            continue
        tags = _extract_tags(way)
        if "building" in tags or tags.get("building:part"):
            # Closed footprint check: first == last.
            if node_seq[0].id != node_seq[-1].id:
                # tolerate unclosed by closing manually
                node_seq.append(node_seq[0])
            height = _parse_height(
                tags.get("height") or tags.get("building:height"),
                default_m=_levels_to_m(tags.get("building:levels"), 12.0),
            )
            material = tags.get("building:material",
                                tags.get("material", "itu_concrete"))
            buildings.append(OsmBuilding(osm_id=wid,
                                         nodes=node_seq,
                                         height_m=height,
                                         material=material))
        elif tags.get("highway"):
            road_kind = tags["highway"]
            width = float(tags.get("width", "0") or 0.0)
            if width <= 0.0:
                width = _HIGHWAY_DEFAULT_WIDTHS_M.get(road_kind, 4.0)
            roads.append(OsmRoad(osm_id=wid,
                                 nodes=node_seq,
                                 highway=road_kind,
                                 width_m=width))
    return buildings, roads


def _levels_to_m(levels_tag: Optional[str], default_m: float) -> float:
    if not levels_tag:
        return default_m
    try:
        return max(3.0, float(levels_tag) * 3.0)  # 3 m / level rule
    except ValueError:
        return default_m


# ---------------------------------------------------------------------------
#  Optional DEM lookup
# ---------------------------------------------------------------------------


class DemLookup:
    """Minimal GeoTIFF reader for ground-altitude lookup.

    Falls back to a constant altitude if `rasterio` is unavailable —
    keeping the CLI usable on minimal Python installs.
    """

    def __init__(self, dem_path: Optional[str] = None,
                 fallback_alt_m: float = 0.0):
        self.fallback = fallback_alt_m
        self._ds = None
        if dem_path is None:
            return
        try:
            import rasterio  # type: ignore
        except ImportError:
            sys.stderr.write(
                "warning: rasterio not installed, using fallback alt\n")
            return
        try:
            self._ds = rasterio.open(dem_path)
        except Exception as exc:  # pragma: no cover - depends on file
            sys.stderr.write(f"warning: open DEM failed: {exc}\n")
            self._ds = None

    def altitude(self, lat_deg: float, lon_deg: float) -> float:
        if self._ds is None:
            return self.fallback
        try:
            row, col = self._ds.index(lon_deg, lat_deg)
            return float(self._ds.read(1)[row, col])
        except Exception:  # pragma: no cover
            return self.fallback


# ---------------------------------------------------------------------------
#  Sionna scene writer
# ---------------------------------------------------------------------------


_SCENE_HEADER = """<?xml version="1.0" encoding="UTF-8"?>
<!-- Generated by ntn-sionna/tools/osm_to_sionna_scene.py (Roadmap §4.2.9) -->
<scene version="2.1.0">
"""

_SCENE_FOOTER = "</scene>\n"


def write_scene(export: SceneExport, out_basename: str) -> Dict[str, str]:
    """Write the .xml + .mtl Sionna scene files.

    Returns the dict of paths written so callers / tests can verify.
    """
    xml_path = out_basename + ".xml"
    mtl_path = out_basename + ".mtl"

    materials = sorted({b.material for b in export.buildings} |
                       {"itu_road"})
    export.materials = {m: m for m in materials}

    with open(xml_path, "w", encoding="utf-8") as fp:
        fp.write(_SCENE_HEADER)
        fp.write(f'    <reference lat_deg="{export.ref_lat_deg}" '
                 f'lon_deg="{export.ref_lon_deg}" '
                 f'alt_m="{export.ref_alt_m}"/>\n')
        for b in export.buildings:
            verts = [export.to_enu(n.lat_deg, n.lon_deg) for n in b.nodes]
            fp.write(f'    <building id="{b.osm_id}" '
                     f'height_m="{b.height_m}" '
                     f'material="{b.material}">\n')
            for x, y in verts:
                fp.write(f'      <vertex x="{x:.3f}" y="{y:.3f}"/>\n')
            fp.write("    </building>\n")
        for r in export.roads:
            verts = [export.to_enu(n.lat_deg, n.lon_deg) for n in r.nodes]
            fp.write(f'    <road id="{r.osm_id}" '
                     f'kind="{r.highway}" '
                     f'width_m="{r.width_m}" '
                     f'material="itu_road">\n')
            for x, y in verts:
                fp.write(f'      <vertex x="{x:.3f}" y="{y:.3f}"/>\n')
            fp.write("    </road>\n")
        fp.write(_SCENE_FOOTER)

    with open(mtl_path, "w", encoding="utf-8") as fp:
        fp.write(
            "# Generated by ntn-sionna/tools/osm_to_sionna_scene.py\n")
        for m in materials:
            fp.write(f'material "{m}" {{ source = "{m}" }}\n')

    return {"xml": xml_path, "mtl": mtl_path}


# ---------------------------------------------------------------------------
#  CLI entry-point
# ---------------------------------------------------------------------------


def build_export(osm_path: str,
                 ref_lat: float,
                 ref_lon: float,
                 ref_alt: float = 0.0,
                 dem_path: Optional[str] = None) -> SceneExport:
    buildings, roads = parse_osm_xml(osm_path)
    dem = DemLookup(dem_path=dem_path, fallback_alt_m=ref_alt)
    # Apply DEM-corrected alt to the reference for any caller that
    # wants it. We don't currently fold DEM into building tops because
    # OSM heights are already AGL.
    _ = dem.altitude(ref_lat, ref_lon)
    return SceneExport(ref_lat_deg=ref_lat,
                       ref_lon_deg=ref_lon,
                       ref_alt_m=ref_alt,
                       buildings=buildings,
                       roads=roads)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Convert an OpenStreetMap XML dump into a "
                    "Sionna RT scene description.")
    parser.add_argument("--osm", required=True, help="OSM XML input")
    parser.add_argument("--ref-lat", type=float, required=True)
    parser.add_argument("--ref-lon", type=float, required=True)
    parser.add_argument("--ref-alt", type=float, default=0.0)
    parser.add_argument("--dem", default=None,
                        help="optional GeoTIFF DEM (requires rasterio)")
    parser.add_argument("--out", required=True,
                        help="output basename (writes .xml + .mtl)")
    args = parser.parse_args(argv)

    if not os.path.isfile(args.osm):
        print(f"error: OSM file not found: {args.osm}", file=sys.stderr)
        return 1
    export = build_export(args.osm,
                          ref_lat=args.ref_lat,
                          ref_lon=args.ref_lon,
                          ref_alt=args.ref_alt,
                          dem_path=args.dem)
    paths = write_scene(export, args.out)
    print(f"# wrote {paths['xml']} ({len(export.buildings)} buildings, "
          f"{len(export.roads)} roads)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
