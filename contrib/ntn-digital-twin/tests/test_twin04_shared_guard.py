"""TWIN-04: the semantics of the one shared A3 guard.

The equivalence test in test_twin04_a3_guard.py proves the exporter and the
REST endpoint run the SAME rule. It cannot prove the rule is right: both paths
call ntn_digital_twin.a3_guard, so any change to the rule changes both
identically and equivalence survives by construction. These tests pin the rule
itself, one decision per test, against the three ways the two old copies
actually disagreed plus the state-machine behavior A3 requires.
"""

from __future__ import annotations

import pytest

from ntn_digital_twin.a3_guard import (
    A3Evaluator,
    A3Params,
    Candidate,
    altitude_km_from_ecef,
    run_a3,
    slant_range_m,
    snr_db,
)


def test_ranking_quantity_is_the_comparison_quantity():
    """In dB mode the best satellite is the best in dB, not the highest one.

    The old REST path ranked candidates by elevation and then compared them in
    dB. On a single-altitude shell those pick the same satellite, which is why
    every Walker test missed it; on a mixed-altitude shell they do not. Here A
    is 5 degrees higher than B but 8.8 dB worse, because it is at 2000 km.
    """
    p = A3Params(margin_quantity="db", a3_offset_db=0.0)
    hi_far = Candidate(key="A", elev_deg=30.0, alt_km=2000.0)
    lo_near = Candidate(key="B", elev_deg=25.0, alt_km=550.0)
    assert hi_far.elev_deg > lo_near.elev_deg
    assert snr_db(p, hi_far.elev_deg, hi_far.alt_km) < snr_db(
        p, lo_near.elev_deg, lo_near.alt_km)

    serving = Candidate(key="S", elev_deg=20.0, alt_km=600.0)
    ev = A3Evaluator(p)
    ev.step(0.0, [serving])                       # latch S
    hit = ev.step(10.0, [serving, hi_far, lo_near])
    assert hit is not None, "a better cell exists; the guard must hand over"
    assert hit.key_in == "B", (
        "the guard picked the higher-elevation satellite over the stronger one, "
        "so it is ranking in degrees and comparing in dB")

    # And the elevation mode must still rank by elevation, or it is not the
    # behaviour it claims to preserve.
    ev2 = A3Evaluator(A3Params(margin_quantity="elevation_deg", hysteresis_deg=0.0))
    ev2.step(0.0, [serving])
    hit2 = ev2.step(10.0, [serving, hi_far, lo_near])
    assert hit2 is not None and hit2.key_in == "A"


def test_initial_acquisition_is_not_a_handover():
    """There is no cell to hand over FROM at the first tick.

    The exporter counted it, so every predicted sequence it wrote was one event
    longer at the front than the endpoint's, and the extra event named a source
    satellite that had never served.
    """
    p = A3Params(margin_quantity="db", a3_offset_db=0.0)
    ev = A3Evaluator(p)
    assert ev.step(0.0, [Candidate(key="S", elev_deg=40.0)]) is None
    assert ev.serving is not None and ev.serving.key == "S"


def test_time_to_trigger_restarts_when_the_target_changes():
    """A3's TTT is per-target: a new best candidate starts a new window.

    Letting the window carry over lets a handover to B fire on time that A
    accrued, which is the ping-pong the timer exists to suppress.
    """
    p = A3Params(margin_quantity="db", a3_offset_db=0.0, time_to_trigger_s=10.0)
    serving = Candidate(key="S", elev_deg=10.0, alt_km=600.0)
    a = Candidate(key="A", elev_deg=40.0, alt_km=600.0)
    b = Candidate(key="B", elev_deg=50.0, alt_km=600.0)

    ev = A3Evaluator(p)
    ev.step(0.0, [serving])
    assert ev.step(10.0, [serving, a]) is None       # window opens on A
    assert ev.step(20.0, [serving, a, b]) is None    # target flips to B, restart
    assert ev.step(25.0, [serving, a, b]) is None, (
        "held 5 s on B but fired anyway: the window inherited A's credit")
    hit = ev.step(30.0, [serving, a, b])
    assert hit is not None and hit.key_in == "B"


def test_a_lapsed_condition_resets_the_window():
    """Re-entry starts over rather than resuming where it left off."""
    p = A3Params(margin_quantity="db", a3_offset_db=0.0, time_to_trigger_s=10.0)
    serving = Candidate(key="S", elev_deg=10.0, alt_km=600.0)
    strong = Candidate(key="A", elev_deg=40.0, alt_km=600.0)
    weak = Candidate(key="A", elev_deg=5.0, alt_km=600.0)   # same cell, faded

    ev = A3Evaluator(p)
    ev.step(0.0, [serving])
    assert ev.step(10.0, [serving, strong]) is None   # window opens
    assert ev.step(15.0, [serving, weak]) is None     # condition lapses
    assert ev.step(20.0, [serving, strong]) is None, (
        "fired 10 s after the window first opened, but it lapsed in between")
    assert ev.step(30.0, [serving, strong]) is not None

    # The lapse above happens because the faded cell stops being the best, which
    # takes the best-is-serving branch. Exercise the OTHER lapse route too: the
    # candidate stays the best but its margin falls under the offset. That is
    # the branch a guard is most likely to return from without clearing the
    # timer, and then a re-entry inherits time the condition was not met.
    q = A3Params(margin_quantity="db", a3_offset_db=5.0, time_to_trigger_s=10.0)
    base = Candidate(key="S", elev_deg=20.0, alt_km=600.0)
    over = Candidate(key="C", elev_deg=60.0, alt_km=600.0)     # clears 5 dB
    under = Candidate(key="C", elev_deg=24.0, alt_km=600.0)    # best, under 5 dB
    assert 0.0 < (snr_db(q, under.elev_deg, 600.0)
                  - snr_db(q, base.elev_deg, 600.0)) < 5.0
    ev5 = A3Evaluator(q)
    ev5.step(0.0, [base])
    assert ev5.step(10.0, [base, over]) is None      # window opens
    assert ev5.step(15.0, [base, under]) is None     # margin lapses, timer clears
    assert ev5.step(20.0, [base, over]) is None, (
        "an under-margin tick left the time-to-trigger window running")
    assert ev5.step(30.0, [base, over]) is not None


def test_minimum_service_time_is_enforced_between_handovers():
    p = A3Params(margin_quantity="db", a3_offset_db=0.0, min_service_s=100.0)
    s = Candidate(key="S", elev_deg=10.0, alt_km=600.0)
    a = Candidate(key="A", elev_deg=40.0, alt_km=600.0)
    b = Candidate(key="B", elev_deg=60.0, alt_km=600.0)
    ev = A3Evaluator(p)
    ev.step(0.0, [s])
    assert ev.step(10.0, [s, a]) is not None          # first handover
    assert ev.step(60.0, [s, a, b]) is None, "handed over 50 s into a 100 s hold"
    assert ev.step(120.0, [s, a, b]) is not None


def test_candidates_below_minimum_elevation_cannot_serve():
    """The exporter had no minimum-elevation filter at all."""
    p = A3Params(margin_quantity="db", a3_offset_db=0.0, min_elevation_deg=10.0)
    s = Candidate(key="S", elev_deg=40.0, alt_km=600.0)
    low = Candidate(key="L", elev_deg=5.0, alt_km=600.0)
    ev = A3Evaluator(p)
    ev.step(0.0, [s])
    assert ev.step(10.0, [s, low]) is None
    # It is not merely weaker: it is not a candidate. Give it a budget that
    # would beat the serving cell if it were admitted.
    assert snr_db(p, 5.0, 60.0) > snr_db(p, 40.0, 600.0)
    assert ev.step(20.0, [s, Candidate(key="L", elev_deg=5.0, alt_km=60.0)]) is None


def test_margin_must_be_strictly_exceeded_and_the_offset_bites():
    """A candidate exactly at the offset does not trigger; above it does."""
    p = A3Params(margin_quantity="db", a3_offset_db=3.0)
    s = Candidate(key="S", elev_deg=20.0, alt_km=600.0)
    base = snr_db(p, 20.0, 600.0)

    # Find an elevation whose budget sits just under, and one just over, +3 dB.
    def snr_at(el):
        return snr_db(p, el, 600.0)

    below = next(el for el in [x / 10 for x in range(200, 900)]
                 if snr_at(el) - base > 2.0 and snr_at(el) - base < 2.9)
    above = next(el for el in [x / 10 for x in range(200, 900)]
                 if snr_at(el) - base > 3.1)

    ev = A3Evaluator(p)
    ev.step(0.0, [s])
    assert ev.step(10.0, [s, Candidate(key="C", elev_deg=below, alt_km=600.0)]) is None
    ev2 = A3Evaluator(p)
    ev2.step(0.0, [s])
    hit = ev2.step(10.0, [s, Candidate(key="C", elev_deg=above, alt_km=600.0)])
    assert hit is not None and hit.margin > 3.0

    # And the boundary itself: a candidate whose margin EQUALS the offset does
    # not trigger. TS 38.331 Event A3 enters on Mn + Ofn + Ocn - Hys > Mp + Ofp
    # + Ocp + Off, a strict inequality, so equality is the non-triggering side.
    # Without this the comparison could be <= or < and nothing would notice.
    exact = snr_db(p, 35.0, 600.0) - snr_db(p, 20.0, 600.0)
    p_exact = A3Params(margin_quantity="db", a3_offset_db=exact)
    ev3 = A3Evaluator(p_exact)
    ev3.step(0.0, [s])
    assert ev3.step(10.0, [s, Candidate(key="C", elev_deg=35.0, alt_km=600.0)]) is None, (
        "a margin exactly equal to the offset triggered; A3 requires strictly greater")
    p_eps = A3Params(margin_quantity="db", a3_offset_db=exact - 1e-9)
    ev4 = A3Evaluator(p_eps)
    ev4.step(0.0, [s])
    assert ev4.step(10.0, [s, Candidate(key="C", elev_deg=35.0, alt_km=600.0)]) is not None, (
        "the boundary case is not actually near the boundary")


def test_effective_margin_reports_the_units_in_use():
    assert A3Params(margin_quantity="db", a3_offset_db=7.0,
                    hysteresis_deg=2.0).effective_margin == 7.0
    assert A3Params(margin_quantity="elevation_deg", a3_offset_db=7.0,
                    hysteresis_deg=2.0).effective_margin == 2.0
    with pytest.raises(ValueError):
        A3Params(margin_quantity="rsrp")


def test_slant_range_and_altitude_round_trip():
    assert abs(slant_range_m(90.0, 600.0) - 600e3) < 1.0
    # Straight up from the equator at 600 km over the WGS-84 equatorial radius.
    assert abs(altitude_km_from_ecef((6378135.0 + 600e3, 0.0, 0.0)) - 600.0) < 1e-6


def test_run_a3_matches_stepping_by_hand():
    p = A3Params(margin_quantity="db", a3_offset_db=0.0)
    s = Candidate(key="S", elev_deg=10.0, alt_km=600.0)
    a = Candidate(key="A", elev_deg=40.0, alt_km=600.0)
    ticks = [(0.0, [s]), (10.0, [s, a]), (20.0, [s, a])]
    ev = A3Evaluator(p)
    by_hand = [e for e in (ev.step(t, c) for t, c in ticks) if e is not None]
    assert [(e.t_s, e.key_in) for e in run_a3(p, ticks)] == \
           [(e.t_s, e.key_in) for e in by_hand]
