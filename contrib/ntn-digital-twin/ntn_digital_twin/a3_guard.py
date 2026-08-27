"""TWIN-04: the one A3 guard, shared by every path that predicts a handover.

There used to be two. The exporter that ns-3 actually actuates on took a bare
argmax over elevation and appended a prediction on every serving change, and
the REST endpoint applied a margin in topocentric elevation DEGREES while its
comment claimed it enforced "exactly the conditions the sim's A3 algorithm
enforces". The simulator's A3 (contrib/ntn-cho/model/ntn-cho-algorithm.cc) is

    info.sinr_dB > servingSinr_dB + a3Offset_dB

a margin in dB on a signal level. Elevation and SINR are not monotonically
related once antenna pattern, scan loss and ITU-R P.618/P.676 attenuation
enter, so an elevation-margin guard admits and rejects different handovers than
a dB-margin one. Two implementations of the same rule also drift, and these two
had: they disagreed on the tie-break quantity, on the minimum-elevation filter,
and on whether initial acquisition counts as a handover.

So both callers now feed this evaluator instead of carrying their own copy.

What the budget is and is not: free-space loss over the slant range implied by
the elevation and the satellite altitude, against a configured EIRP and G/T. No
fading, no atmospheric term, no antenna pattern. It is a link budget, not a
channel model. What it buys is that a 3 dB threshold means 3 dB, in the same
units the simulator compares.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Hashable, Optional, Sequence

# Speed-of-light constant in the FSPL form: 20log10(4*pi/c) with d in m and f
# in Hz. Matches the C++ side.
_FSPL_CONST_DB = 147.55
# Boltzmann in dBW/K/Hz.
_K_DBW = 228.6
_EARTH_RADIUS_M = 6371e3


@dataclass
class A3Params:
    """Everything the guard needs, in one place both callers can pass."""

    # "db" compares a link-budget SNR, matching the simulator. "elevation_deg"
    # keeps the original topocentric-angle comparison for callers that want it;
    # it is NOT the simulator's rule and is retained only for continuity.
    margin_quantity: str = "db"
    a3_offset_db: float = 3.0
    hysteresis_deg: float = 0.0
    time_to_trigger_s: float = 0.0
    min_service_s: float = 0.0
    min_elevation_deg: float = 0.0
    # Link budget. Defaults are the toolkit's S-band reference point; a
    # scenario should set them to whatever it is actually modelling.
    link_eirp_dbm: float = 62.0
    link_gt_db_per_k: float = 1.1
    link_bandwidth_hz: float = 20.0e6
    link_frequency_hz: float = 2.0e9

    def __post_init__(self) -> None:
        if self.margin_quantity not in ("db", "elevation_deg"):
            raise ValueError(
                f"margin_quantity must be 'db' or 'elevation_deg', "
                f"got {self.margin_quantity!r}")

    @property
    def effective_margin(self) -> float:
        """The margin that is actually applied, in the units actually used.

        Callers echo this so a parameter that is being ignored cannot be
        mistaken for one that is being enforced.
        """
        return (self.a3_offset_db if self.margin_quantity == "db"
                else self.hysteresis_deg)


@dataclass(slots=True)
class Candidate:
    """One satellite as seen from the observer at one instant."""

    key: Hashable          # whatever the caller identifies satellites by
    elev_deg: float
    alt_km: float = 600.0
    name: str = ""


@dataclass(slots=True)
class A3Event:
    """A handover the guard admitted."""

    t_s: float
    key_in: Hashable
    key_out: Hashable
    elev_in_deg: float
    elev_out_deg: Optional[float]
    name_in: str = ""
    name_out: str = ""
    # The margin the candidate actually beat the serving cell by, in the units
    # the guard compared. Reported so a downstream reader can see how decisive
    # the crossover was rather than inferring it.
    margin: float = 0.0


def slant_range_m(elev_deg: float, alt_km: float,
                  earth_radius_m: float = _EARTH_RADIUS_M) -> float:
    """Slant range to a satellite at `alt_km` seen at `elev_deg`.

    Spherical geometry, the same relation the C++ side uses:
        d = -R sin(el) + sqrt((R sin el)^2 + 2 R h + h^2)
    """
    el = math.radians(max(0.0, min(90.0, elev_deg)))
    R = earth_radius_m
    h = alt_km * 1000.0
    rs = R * math.sin(el)
    return -rs + math.sqrt(rs * rs + 2.0 * R * h + h * h)


def altitude_km_from_ecef(ecef_m) -> float:
    """Geocentric altitude from an ECEF position vector, in km.

    Shared because the two prediction paths disagreed on it: the exporter
    passed the Walker preset's nominal altitude as a constant for every
    satellite, while the endpoint measured it per satellite. On a shell whose
    orbital radius is not exactly nominal that shifts the budget, and it moved
    one handover by a tick.
    """
    r = math.sqrt(sum(c * c for c in ecef_m))
    return max(0.0, (r - 6378135.0) / 1000.0)


def snr_db(params: A3Params, elev_deg: float, alt_km: float) -> float:
    """Link-budget SNR in dB for a satellite at `alt_km` seen at `elev_deg`."""
    if elev_deg <= 0.0:
        return -999.0
    d = slant_range_m(elev_deg, alt_km)
    fspl_db = (20.0 * math.log10(max(d, 1.0))
               + 20.0 * math.log10(max(params.link_frequency_hz, 1.0))
               - _FSPL_CONST_DB)
    # C/N0 = EIRP - FSPL + G/T - k. EIRP is dBm here, hence the -30.
    cn0_dbhz = ((params.link_eirp_dbm - 30.0) - fspl_db
                + params.link_gt_db_per_k + _K_DBW)
    return cn0_dbhz - 10.0 * math.log10(max(params.link_bandwidth_hz, 1.0))


def _quantity(params: A3Params, c: Candidate) -> float:
    """The value the guard ranks and compares, in the configured units."""
    if params.margin_quantity == "db":
        return snr_db(params, c.elev_deg, c.alt_km)
    return c.elev_deg


class A3Evaluator:
    """Stateful A3 across a series of ticks. Feed it ticks in time order.

    Semantics, fixed here once so the two callers cannot disagree again:
      * The ranking quantity is the COMPARISON quantity. Ranking by elevation
        and then comparing in dB would pick a different best satellite than the
        dB comparison implies whenever the shell has mixed altitudes.
      * Candidates below `min_elevation_deg` are not visible and cannot serve.
      * Initial acquisition is NOT a handover. There is no cell to hand over
        from, and calling it one inflates every count by exactly one.
      * The time-to-trigger window tracks one target. If the best candidate
        changes, the window restarts rather than inheriting credit.
      * A lapsed condition resets the window, so a re-entry starts over.
    """

    def __init__(self, params: A3Params) -> None:
        self.p = params
        self.serving: Optional[Candidate] = None
        self._since: Optional[float] = None
        self._target: Optional[Hashable] = None
        self._last_ho_t: Optional[float] = None

    def step(self, t_s: float,
             candidates: Sequence[Candidate]) -> Optional[A3Event]:
        """Advance to time `t_s`. Returns an event if a handover fires."""
        visible = [c for c in candidates
                   if c.elev_deg >= self.p.min_elevation_deg and c.elev_deg > -90.0]
        if not visible:
            # Nothing in view: the condition cannot hold through an outage.
            self._since = None
            self._target = None
            return None

        best = max(visible, key=lambda c: _quantity(self.p, c))

        if self.serving is None:
            self.serving = best
            return None

        # Refresh the serving cell's geometry at this tick. If it has dropped
        # out of view we keep its last known elevation, which is the worst case
        # for the candidate and so cannot manufacture a handover.
        serving_now = next((c for c in visible if c.key == self.serving.key), None)
        if serving_now is not None:
            self.serving = serving_now

        if best.key == self.serving.key:
            self._since = None
            self._target = None
            return None

        margin = _quantity(self.p, best) - _quantity(self.p, self.serving)
        if margin <= self.p.effective_margin:
            self._since = None
            self._target = None
            return None

        if self._since is None or self._target != best.key:
            self._since = t_s
            self._target = best.key

        held = t_s - self._since
        served = (t_s - self._last_ho_t) if self._last_ho_t is not None else 1e9
        if held < self.p.time_to_trigger_s or served < self.p.min_service_s:
            return None

        ev = A3Event(
            t_s=t_s,
            key_in=best.key, key_out=self.serving.key,
            elev_in_deg=best.elev_deg, elev_out_deg=self.serving.elev_deg,
            name_in=best.name, name_out=self.serving.name,
            margin=margin,
        )
        self.serving = best
        self._last_ho_t = t_s
        self._since = None
        self._target = None
        return ev


def run_a3(params: A3Params,
           ticks: Sequence[tuple[float, Sequence[Candidate]]]) -> list[A3Event]:
    """Convenience: run a whole prepared series through one evaluator."""
    ev = A3Evaluator(params)
    out: list[A3Event] = []
    for t_s, cands in ticks:
        e = ev.step(t_s, cands)
        if e is not None:
            out.append(e)
    return out
