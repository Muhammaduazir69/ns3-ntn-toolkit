"""Wave 7C: the twin's link budget is the TR 38.821 Set-1 handheld, and the
change of absolute level does not move a single A3 decision."""

from __future__ import annotations

from ntn_digital_twin.a3_guard import A3Params, Candidate, run_a3, snr_db


def test_set1_handheld_snr_is_realistic():
    p = A3Params()
    assert abs(p.link_eirp_dbm - 77.0) < 0.05
    assert abs(p.link_gt_db_per_k + 31.6) < 0.05
    zenith = snr_db(p, 90.0, 600.0)
    low = snr_db(p, 10.0, 600.0)
    # Free-space budget at zenith is about 17 dB (TR 38.821 Table 6.1.3.3-1
    # adds a further ~8 dB of margins); never the >25 dB of the old pair.
    assert 10.0 < zenith < 25.0, zenith
    assert low < zenith


def test_old_budget_was_unrealistic():
    old = A3Params(link_eirp_dbm=62.0, link_gt_db_per_k=1.1)
    assert snr_db(old, 90.0, 600.0) > 25.0


def _pass_ticks():
    ticks = []
    for i in range(0, 600, 5):
        t = float(i)
        a = 80.0 - 0.25 * t  # setting satellite
        b = 5.0 + 0.2 * t    # rising satellite, slightly lower shell
        ticks.append((t, [Candidate("a", a, 550.0), Candidate("b", b, 540.0)]))
    return ticks


def test_a3_decisions_invariant_to_absolute_level():
    new = run_a3(A3Params(time_to_trigger_s=10.0), _pass_ticks())
    old = run_a3(A3Params(link_eirp_dbm=62.0, link_gt_db_per_k=1.1, time_to_trigger_s=10.0),
                 _pass_ticks())
    assert [(e.t_s, e.key_in, e.key_out) for e in new] == [
        (e.t_s, e.key_in, e.key_out) for e in old]
    assert len(new) == 1
