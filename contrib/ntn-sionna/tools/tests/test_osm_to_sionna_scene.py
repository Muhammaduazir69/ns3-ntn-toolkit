#!/usr/bin/env python3
# Copyright (c) 2026 Muhammad Uzair
# SPDX-License-Identifier: GPL-2.0-only
"""Unit tests for osm_to_sionna_scene.py (Roadmap §4.2.9)."""
from __future__ import annotations

import os
import sys
import tempfile
import textwrap
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from osm_to_sionna_scene import (  # noqa: E402
    SceneExport,
    build_export,
    parse_osm_xml,
    write_scene,
)


_TINY_OSM = textwrap.dedent("""\
    <?xml version="1.0" encoding="UTF-8"?>
    <osm version="0.6">
      <node id="1" lat="52.520000" lon="13.405000"/>
      <node id="2" lat="52.520050" lon="13.405000"/>
      <node id="3" lat="52.520050" lon="13.405050"/>
      <node id="4" lat="52.520000" lon="13.405050"/>
      <node id="5" lat="52.519900" lon="13.404950"/>
      <node id="6" lat="52.519950" lon="13.405010"/>
      <way id="100">
        <nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/><nd ref="1"/>
        <tag k="building" v="yes"/>
        <tag k="height" v="35"/>
        <tag k="building:material" v="concrete"/>
      </way>
      <way id="101">
        <nd ref="5"/><nd ref="6"/>
        <tag k="highway" v="residential"/>
      </way>
    </osm>
""")


def _write_tmp(content: str, suffix: str = ".osm") -> str:
    fd, path = tempfile.mkstemp(suffix=suffix)
    with os.fdopen(fd, "w") as fp:
        fp.write(content)
    return path


def test_parse_extracts_building_and_road():
    path = _write_tmp(_TINY_OSM)
    try:
        buildings, roads = parse_osm_xml(path)
    finally:
        os.remove(path)
    assert len(buildings) == 1
    b = buildings[0]
    assert b.osm_id == 100
    assert b.height_m == pytest.approx(35.0)
    assert b.material == "concrete"
    # First and last node should be the same (closed footprint).
    assert b.nodes[0].id == b.nodes[-1].id == 1
    assert len(roads) == 1
    r = roads[0]
    assert r.highway == "residential"
    assert r.width_m == pytest.approx(5.0)  # default for residential


def test_building_levels_fallback():
    osm = _TINY_OSM.replace(
        '<tag k="height" v="35"/>',
        '<tag k="building:levels" v="6"/>'
    )
    path = _write_tmp(osm)
    try:
        buildings, _ = parse_osm_xml(path)
    finally:
        os.remove(path)
    assert buildings[0].height_m == pytest.approx(18.0)  # 6 * 3 m


def test_build_export_and_write_scene(tmp_path):
    path = _write_tmp(_TINY_OSM)
    try:
        export = build_export(path,
                              ref_lat=52.520,
                              ref_lon=13.405,
                              ref_alt=37.0)
    finally:
        os.remove(path)
    assert isinstance(export, SceneExport)
    assert len(export.buildings) == 1
    out_base = str(tmp_path / "scene")
    paths = write_scene(export, out_base)
    assert os.path.isfile(paths["xml"])
    assert os.path.isfile(paths["mtl"])
    # Validate the XML
    tree = ET.parse(paths["xml"])
    root = tree.getroot()
    assert root.tag == "scene"
    ref = root.find("reference")
    assert ref is not None
    assert float(ref.attrib["lat_deg"]) == pytest.approx(52.520)
    assert float(ref.attrib["alt_m"]) == pytest.approx(37.0)
    bldg = root.find("building")
    assert bldg is not None
    assert int(bldg.attrib["id"]) == 100
    assert float(bldg.attrib["height_m"]) == pytest.approx(35.0)
    assert bldg.attrib["material"] == "concrete"
    verts = bldg.findall("vertex")
    assert len(verts) == 5  # closed polygon
    # MTL file should contain a "concrete" entry plus itu_road.
    with open(paths["mtl"]) as fp:
        mtl_text = fp.read()
    assert 'material "concrete"' in mtl_text
    assert 'material "itu_road"' in mtl_text


def test_enu_projection_reference_origin():
    """The reference (lat, lon) should map to ENU (0, 0)."""
    export = SceneExport(ref_lat_deg=10.0, ref_lon_deg=20.0, ref_alt_m=0.0)
    e, n = export.to_enu(10.0, 20.0)
    assert e == pytest.approx(0.0, abs=1e-3)
    assert n == pytest.approx(0.0, abs=1e-3)
    # Moving 0.001 deg north should be ~111 m * 0.001 = 111 m
    _, north = export.to_enu(10.001, 20.0)
    assert north == pytest.approx(111.0, abs=0.5)


def test_unclosed_building_auto_closes():
    """Way without a closing node should be auto-closed."""
    osm = _TINY_OSM.replace(
        '<nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/><nd ref="1"/>',
        '<nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/>'
    )
    path = _write_tmp(osm)
    try:
        buildings, _ = parse_osm_xml(path)
    finally:
        os.remove(path)
    assert buildings[0].nodes[0].id == buildings[0].nodes[-1].id == 1
