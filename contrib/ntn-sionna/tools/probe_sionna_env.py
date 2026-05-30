#!/usr/bin/env python3
# Copyright (c) 2026 Muhammad Uzair
# SPDX-License-Identifier: GPL-2.0-only
"""Sionna RT / Mitsuba environment probe (Roadmap §4.3.8).

Reports the installed Sionna RT, Mitsuba 3.x or 4.x, Dr.Jit, TensorFlow
and Blender versions, plus an availability + version-compat verdict
that the docs link to.

Toolkit support matrix (verified May 2026):

  | Sionna RT | Mitsuba       | Dr.Jit  | TensorFlow | Blender    |
  |-----------|---------------|---------|------------|------------|
  | <  1.0    | 3.5.0..3.6.2  | 1.0.5+  | 2.14..2.16 | 4.0..4.2   |
  | 1.0..1.2  | 3.6.0..3.6.2  | 1.0.5+  | 2.15..2.17 | 4.1..4.3   |
  | 2.0.x     | 3.6.x or 4.x  | 1.0.5+  | 2.17+      | 4.2..4.5   |

The `--ci-gate` flag exits non-zero when the environment is below the
toolkit's minimum (Sionna 2.0.1 / Mitsuba 3.6 or 4.0+). Used by CI
jobs to fail loudly instead of silently falling back to FSPL.
"""
from __future__ import annotations

import argparse
import importlib
import sys
from typing import Dict, Optional


def _probe(module_name: str) -> Optional[str]:
    """Return module version or None if not importable."""
    try:
        mod = importlib.import_module(module_name)
    except Exception:
        return None
    for attr in ("__version__", "VERSION", "version"):
        if hasattr(mod, attr):
            v = getattr(mod, attr)
            if callable(v):
                try:
                    v = v()
                except Exception:
                    continue
            return str(v)
    return "installed"


def gather() -> Dict[str, Optional[str]]:
    return {
        "sionna": _probe("sionna"),
        "sionna_rt": _probe("sionna.rt"),
        "mitsuba": _probe("mitsuba"),
        "drjit": _probe("drjit"),
        "tensorflow": _probe("tensorflow"),
        "blender": _probe("bpy"),
        "numpy": _probe("numpy"),
        "scipy": _probe("scipy"),
        "h5py": _probe("h5py"),
        "rasterio": _probe("rasterio"),
        "laspy": _probe("laspy"),
    }


def _major_minor(v: Optional[str]) -> Optional[tuple]:
    if not v:
        return None
    parts = v.replace("-", ".").split(".")
    try:
        return tuple(int(parts[i]) for i in range(min(2, len(parts))))
    except ValueError:
        return None


def verdict(env: Dict[str, Optional[str]],
            min_sionna: tuple = (2, 0),
            min_mitsuba: tuple = (3, 6)) -> Dict[str, str]:
    """Translate raw versions into pass/warn/fail per component."""
    out: Dict[str, str] = {}
    sionna_mm = _major_minor(env.get("sionna"))
    out["sionna"] = ("missing" if sionna_mm is None
                     else "ok" if sionna_mm >= min_sionna
                     else "outdated")
    mit = _major_minor(env.get("mitsuba"))
    if mit is None:
        out["mitsuba"] = "missing"
    elif mit[0] >= 4:
        out["mitsuba"] = "ok (4.x)"
    elif mit >= min_mitsuba:
        out["mitsuba"] = "ok (3.x)"
    else:
        out["mitsuba"] = "outdated"
    blender_mm = _major_minor(env.get("blender"))
    if blender_mm is None:
        out["blender"] = "missing (optional for OSM/scene authoring)"
    elif blender_mm[0] >= 4:
        out["blender"] = "ok"
    else:
        out["blender"] = "outdated"
    return out


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ci-gate", action="store_true",
                   help="exit non-zero when env is below toolkit minimum")
    args = p.parse_args(argv)

    env = gather()
    v = verdict(env)
    print("# Sionna RT environment probe (Roadmap §4.3.8)")
    print("")
    for k in ("sionna", "sionna_rt", "mitsuba", "drjit",
              "tensorflow", "blender", "numpy", "scipy", "h5py",
              "rasterio", "laspy"):
        ver = env.get(k) or "—"
        print(f"  {k:14s} {ver}")
    print("")
    print("# Verdict:")
    for k, s in v.items():
        print(f"  {k:14s} {s}")
    if args.ci_gate:
        rc = 0
        if v.get("sionna") not in ("ok", "ok (4.x)", "ok (3.x)"):
            print("CI gate: Sionna RT below 2.0", file=sys.stderr)
            rc = 1
        if v.get("mitsuba") == "outdated":
            print("CI gate: Mitsuba below 3.6", file=sys.stderr)
            rc = 1
        if v.get("mitsuba") == "missing":
            print("CI gate: Mitsuba not installed", file=sys.stderr)
            rc = 1
        return rc
    return 0


if __name__ == "__main__":
    sys.exit(main())
