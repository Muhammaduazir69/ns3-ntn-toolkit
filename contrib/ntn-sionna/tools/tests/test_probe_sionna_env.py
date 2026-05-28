#!/usr/bin/env python3
# Copyright (c) 2026 Muhammad Uzair
# SPDX-License-Identifier: GPL-2.0-only
"""Unit tests for probe_sionna_env.py (Roadmap §4.3.8)."""
from __future__ import annotations

import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from probe_sionna_env import _major_minor, gather, verdict  # noqa: E402


def test_major_minor_handles_strings():
    assert _major_minor("2.0.1") == (2, 0)
    assert _major_minor("3.6.2") == (3, 6)
    assert _major_minor("4.0") == (4, 0)
    assert _major_minor("1.0.5-rc.3") == (1, 0)


def test_major_minor_invalid_returns_none():
    assert _major_minor(None) is None
    assert _major_minor("") is None
    assert _major_minor("notaversion") is None


def test_verdict_classifies_sionna():
    v = verdict({"sionna": "2.0.1", "mitsuba": "3.6.2",
                 "blender": "4.2.0"})
    assert v["sionna"] == "ok"
    assert v["mitsuba"] == "ok (3.x)"
    assert v["blender"] == "ok"

    v2 = verdict({"sionna": "1.0.0", "mitsuba": "4.0.0",
                  "blender": None})
    assert v2["sionna"] == "outdated"
    assert v2["mitsuba"] == "ok (4.x)"
    assert "missing" in v2["blender"]

    v3 = verdict({"sionna": None, "mitsuba": "3.5.0",
                  "blender": "3.6.0"})
    assert v3["sionna"] == "missing"
    assert v3["mitsuba"] == "outdated"
    assert v3["blender"] == "outdated"


def test_gather_returns_dict_with_keys():
    env = gather()
    for k in ("sionna", "mitsuba", "drjit", "tensorflow",
              "blender", "numpy"):
        assert k in env
