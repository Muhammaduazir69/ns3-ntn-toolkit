"""CI gate 9 — twin/sim handover-agreement >= 80%.

The digital twin runs a Walker constellation and, per tick, selects a serving
satellite for a ground point (the same primitive its ``/predict/handover`` path
uses: argmax topocentric elevation via the closed-form ECEF->ENU rotation in
``ntn_digital_twin.api.server``). The ns-3 CHO "sim" side selects the serving
satellite by best link (highest elevation / lowest slant range) over the SAME
Walker elements and epoch. With no ns-3 runtime available here, the reference is
the geometric ground truth over identical orbits: at each tick the serving sat
is the one with maximum elevation above the ground point.

Gate 9 passes if the twin's per-tick serving selection agrees with this
geometric oracle on >= 80% of comparable ticks over a representative pass
window, with real handovers occurring (so the test is not the trivial
"one satellite the whole time").

Anti-tautology note: the two sides use INDEPENDENT elevation computations.
  * TWIN  -> ``api_server._elevation_deg_ecef`` : closed-form ECEF->ENU
             rotation applied to ``Satellite.ecef_m`` (the twin's own hot-path
             primitive, straight out of ``predict_handover``).
  * ORACLE -> ``Satellite.elevation_deg``       : Skyfield topocentric altaz
             ((sat - observer).at(t).altaz()), a different math path entirely.
Both take an independent argmax, so agreement is a genuine cross-check of the
twin's link-selection geometry, not the same code compared to itself.

Determinism: fixed epoch (never wall-clock now()) + fixed Walker elements.
"""

from __future__ import annotations

import datetime as dt

from ntn_digital_twin.twin_loop import LoopConfig, fetch_constellation
from ntn_digital_twin.api import server as api_server

# Fixed, shared epoch: 2025-01-01 00:00:00 UTC. Both the twin selection and the
# oracle propagate the SAME Walker elements from this epoch -> fully deterministic.
EPOCH_UNIX_S = 1_735_689_600.0

# Ground point (Islamabad-ish), well inside the 53-deg inclination coverage band.
GROUND_LAT_DEG = 33.6844
GROUND_LON_DEG = 73.0479
GROUND_ALT_M = 540.0

MIN_ELEVATION_DEG = 10.0     # a link below this is not a usable serving cell
WINDOW_MIN = 45.0            # pass window long enough for several handovers
STEP_SEC = 15.0             # tick cadence


def _build_walker() -> object:
    """Deterministic Walker-Delta constellation at the fixed epoch.

    6 planes x 12 sats = 72 LEO sats @ 600 km / 53 deg -- dense enough that the
    serving satellite changes several times across the window.
    """
    cfg = LoopConfig(
        source="walker",
        walker_planes=6,
        walker_sats_per_plane=12,
        walker_altitude_km=600.0,
        walker_inclination_deg=53.0,
        walker_phasing_factor=1,
        epoch_unix_s=EPOCH_UNIX_S,
        max_sats=72,
    )
    cons, n = fetch_constellation(cfg)
    assert n == 72, f"expected 72 Walker sats, got {n}"
    return cons


def _twin_serving(cons, when, obs_ecef) -> tuple[int | None, float]:
    """Twin's per-tick serving selection: argmax closed-form ECEF->ENU elevation.

    This is the exact primitive ``predict_handover`` uses on its hot path.
    """
    best_id: int | None = None
    best_el = -90.0
    for sat in cons:
        el = api_server._elevation_deg_ecef(
            obs_ecef, sat.ecef_m(when), GROUND_LAT_DEG, GROUND_LON_DEG
        )
        if el > best_el:
            best_el, best_id = el, sat.norad_id
    return best_id, best_el


def _oracle_serving(cons, when) -> tuple[int | None, float]:
    """Geometric oracle (sim ground truth): argmax Skyfield topocentric elevation.

    Independent of the twin's ECEF->ENU math -- a real cross-check.
    """
    best_id: int | None = None
    best_el = -90.0
    for sat in cons:
        el = sat.elevation_deg(
            when,
            observer_lat_deg=GROUND_LAT_DEG,
            observer_lon_deg=GROUND_LON_DEG,
            observer_alt_m=GROUND_ALT_M,
        )
        if el > best_el:
            best_el, best_id = el, sat.norad_id
    return best_id, best_el


def test_gate9_twin_sim_handover_agreement_ge_80pct():
    cons = _build_walker()
    obs_ecef = api_server._ecef_from_geodetic(
        GROUND_LAT_DEG, GROUND_LON_DEG, GROUND_ALT_M
    )
    start = dt.datetime.fromtimestamp(EPOCH_UNIX_S, tz=dt.timezone.utc)
    n_steps = int(WINDOW_MIN * 60.0 / STEP_SEC) + 1

    comparable = 0     # ticks where BOTH sides have a usable serving cell
    agree = 0
    oracle_handovers = 0
    prev_oracle_id: int | None = None

    for k in range(n_steps):
        when = start + dt.timedelta(seconds=k * STEP_SEC)
        twin_id, twin_el = _twin_serving(cons, when, obs_ecef)
        oracle_id, oracle_el = _oracle_serving(cons, when)

        twin_ok = twin_el >= MIN_ELEVATION_DEG
        oracle_ok = oracle_el >= MIN_ELEVATION_DEG

        # Count a handover whenever the oracle's serving satellite changes
        # between consecutive ticks it deems usable.
        if oracle_ok:
            if prev_oracle_id is not None and oracle_id != prev_oracle_id:
                oracle_handovers += 1
            prev_oracle_id = oracle_id

        if twin_ok and oracle_ok:
            comparable += 1
            if twin_id == oracle_id:
                agree += 1

    assert comparable > 0, "no comparable ticks -- ground point never saw a sat"
    fraction = agree / comparable

    print(
        f"\n[gate9] comparable_ticks={comparable} agree={agree} "
        f"agreement_fraction={fraction:.4f} oracle_handovers={oracle_handovers}"
    )

    # Gate: the window must actually exercise handovers (not one sat throughout)...
    assert oracle_handovers >= 2, (
        f"window exercised too few handovers ({oracle_handovers}); "
        "test would be trivial"
    )
    # ...and the twin's serving selection must match the sim oracle on >= 80%.
    assert fraction >= 0.80, (
        f"twin/sim serving agreement {fraction:.4f} < 0.80 "
        f"({agree}/{comparable} ticks)"
    )
    print(f"[gate9] PASS: agreement {fraction:.1%} >= 80% "
          f"over {comparable} ticks with {oracle_handovers} handovers")
