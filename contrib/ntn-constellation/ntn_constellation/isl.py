"""Inter-satellite link (ISL) topology generation.

For Walker-pattern constellations the canonical "+grid" topology is:
- 2 in-plane neighbours (predecessor / successor in the same orbit plane)
- 2 cross-plane neighbours (nearest sat in the adjacent orbit plane)

For arbitrary populations, fall back to k-nearest by 3D distance.
"""

from __future__ import annotations

import math
from datetime import datetime
from typing import Sequence

from ntn_constellation.propagator import Constellation, Satellite


def _euclid(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


_EARTH_RADIUS_KM = 6371.0


def line_of_sight(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    earth_radius_km: float = _EARTH_RADIUS_KM,
    margin_km: float = 80.0,
) -> bool:
    """True iff the straight segment a->b does NOT pass through the Earth.

    Both endpoints are satellites (outside the sphere); the link is blocked only
    when its closest approach to Earth's centre falls inside the segment and
    below the (radius + atmosphere-margin) shell. Without this, an ISL edge
    chosen purely by range can be drawn straight through the planet.
    """
    r = earth_radius_km + margin_km
    ax, ay, az = a
    dx, dy, dz = b[0] - ax, b[1] - ay, b[2] - az
    dd = dx * dx + dy * dy + dz * dz
    if dd == 0.0:
        return True
    t = -(ax * dx + ay * dy + az * dz) / dd
    if t <= 0.0 or t >= 1.0:
        return True
    cx, cy, cz = ax + t * dx, ay + t * dy, az + t * dz
    return (cx * cx + cy * cy + cz * cz) >= r * r


def build_isl_topology(
    constellation: Constellation,
    when: datetime,
    *,
    k_nearest: int = 4,
    max_range_km: float | None = 5000.0,
) -> list[tuple[int, int]]:
    """Return a deduplicated edge list (i, j) with i < j of inter-satellite links.

    The default 5000 km cap reflects realistic optical/RF ISL budgets at LEO
    altitudes; pass `max_range_km=None` to disable.
    """
    if k_nearest < 1:
        raise ValueError("k_nearest must be >= 1")

    sats: Sequence[Satellite] = list(constellation)
    states = constellation.state_vectors(when)
    edges: set[tuple[int, int]] = set()

    for i, si in enumerate(sats):
        ranged = []
        for j, sj in enumerate(sats):
            if i == j:
                continue
            d = _euclid(states[i].r_eci_km, states[j].r_eci_km)
            if max_range_km is not None and d > max_range_km:
                continue
            # Reject edges whose beam would pass through the Earth.
            if not line_of_sight(states[i].r_eci_km, states[j].r_eci_km):
                continue
            ranged.append((d, j))
        ranged.sort()
        for _, j in ranged[:k_nearest]:
            a, b = (i, j) if i < j else (j, i)
            edges.add((a, b))

    return sorted(edges)


def grid_isl_topology_walker(
    *,
    num_planes: int,
    sats_per_plane: int,
) -> list[tuple[int, int]]:
    """Closed-form +grid topology for Walker constellations laid out plane-by-plane.

    Indices follow the order produced by `presets.walker_*`: plane-major,
    in-plane index minor (i.e. global index = plane * sats_per_plane + slot).
    """
    edges: set[tuple[int, int]] = set()
    for p in range(num_planes):
        for s in range(sats_per_plane):
            idx = p * sats_per_plane + s
            next_in_plane = p * sats_per_plane + (s + 1) % sats_per_plane
            a, b = (idx, next_in_plane) if idx < next_in_plane else (next_in_plane, idx)
            edges.add((a, b))
            if p < num_planes - 1:
                cross = (p + 1) * sats_per_plane + s
                edges.add((idx, cross))
    return sorted(edges)
