

def test_ecef_m_series_matches_the_per_call_path():
    """The batched propagation must return exactly what ecef_m returns.

    ecef_m_series exists because looping ecef_m over a horizon was spending its
    time in Skyfield's per-call setup (it made the twin's own 500 ms prediction
    budget unreachable). A faster path that answers differently is worse than a
    slow one, so this pins the two together rather than trusting them to agree.
    """
    import datetime as dt
    from ntn_constellation.presets import PRESETS, from_preset
    from ntn_constellation.propagator import Satellite

    epoch = dt.datetime(2026, 5, 4, tzinfo=dt.timezone.utc)
    tle = from_preset(PRESETS["starlink-v1-shell1"], epoch=epoch)[0]
    sat = Satellite(tle)
    times = [epoch + dt.timedelta(seconds=17.0 * k) for k in range(40)]

    batched = sat.ecef_m_series(times)
    one_at_a_time = [sat.ecef_m(w) for w in times]
    assert len(batched) == len(times)
    for w, b, o in zip(times, batched, one_at_a_time):
        for axis, (bv, ov) in enumerate(zip(b, o)):
            # Sub-millimetre: the two differ only in float accumulation order.
            assert abs(bv - ov) < 1e-3, (
                f"batched and per-call ECEF disagree at {w} axis {axis}: "
                f"{bv} vs {ov}")

    # The positions must actually move, or the comparison is between constants.
    assert abs(batched[0][0] - batched[-1][0]) > 1e5

    assert sat.ecef_m_series([]) == []


def test_preset_fixtures_are_present_and_parseable():
    """WF-14: the preset constellation data must ship with the module.

    `data/` was gitignored wholesale, so the 3.7 MB of preset TLE, CZML and ISL
    fixtures existed only on the author's machine and in this module's own
    nested repo. A fresh clone of the parent got ntn-constellation WITHOUT the
    data its own CLI presets write and read, so `live_starlink_demo.py` and
    `walker_kuiper.py` could not reproduce. The paper pins a release artifact,
    which makes "works on the machine it was written on" the failure mode that
    matters.

    This asserts the fixtures exist and parse, not merely that a path exists:
    an empty file at the right path is the same defect with a green check.
    """
    from pathlib import Path

    root = Path(__file__).resolve().parent.parent / "data"
    assert root.is_dir(), f"preset data directory is missing at {root}"

    expected = [
        "cli-iridium/positions/tles.txt",
        "cli-iridium/positions/isls.txt",
        "preset-kuiper/positions/tles.txt",
        "preset-kuiper/positions/gw_positions.txt",
        "live-starlink/scenario/positions/tles.txt",
    ]
    for rel in expected:
        p = root / rel
        assert p.is_file(), f"preset fixture missing: {rel}"
        assert p.stat().st_size > 0, f"preset fixture is empty: {rel}"

    # The TLE files must actually be TLEs: two 69-character lines starting
    # with '1 ' and '2 '. A truncated or placeholder file passes an existence
    # check and fails a propagator.
    for rel in ("cli-iridium/positions/tles.txt",
                "preset-kuiper/positions/tles.txt",
                "live-starlink/scenario/positions/tles.txt"):
        lines = [ln.rstrip("\r\n") for ln in (root / rel).read_text().splitlines() if ln.strip()]
        assert len(lines) >= 2, f"{rel} holds no TLE"
        l1 = next((i for i, ln in enumerate(lines) if ln.startswith("1 ")), None)
        assert l1 is not None, f"{rel} has no TLE line 1"
        assert lines[l1 + 1].startswith("2 "), f"{rel} line 1 is not followed by line 2"
        assert len(lines[l1]) >= 69, f"{rel} TLE line 1 is truncated ({len(lines[l1])} chars)"

    # And at least one must propagate, which is the only check that proves the
    # data is usable rather than merely well-shaped.
    import datetime as dt

    from ntn_constellation.propagator import Satellite
    from ntn_constellation.feeds import TleRecord

    lines = [ln.rstrip("\r\n") for ln in
             (root / "preset-kuiper/positions/tles.txt").read_text().splitlines() if ln.strip()]
    i1 = next(i for i, ln in enumerate(lines) if ln.startswith("1 "))
    name = lines[i1 - 1].strip() if i1 > 0 else "PRESET-0"
    sat = Satellite(TleRecord(name=name, line1=lines[i1], line2=lines[i1 + 1]))
    pos = sat.ecef_m(dt.datetime(2026, 5, 4, tzinfo=dt.timezone.utc))
    r = sum(c * c for c in pos) ** 0.5
    assert 6.5e6 < r < 5.0e7, f"preset satellite propagated to an implausible radius {r:.0f} m"
