"""Closed-form ISL geometry and churn on Walker shells, with a direct numerical reference.

Every satellite of a shell flies a circular orbit of radius r and inclination i. Satellite (p, s) has
right ascension Omega_p and argument of latitude u_ps + w t. For two satellites with node difference dO
and phase difference D = u2 - u1, the cosine of the central angle is

    cos g(u1) = M - N cos(2 u1 + D),
    M = cos D [cos dO (1 + cos^2 i) + sin^2 i] / 2 - cos i sin dO sin D,   N = sin^2 i (1 - cos dO) / 2,

so the squared range d^2 = 2 r^2 (1 - cos g) is sinusoidal in 2 u1 and the link is up while
cos(2u1 + D) <= c = (M - kappa) / N, kappa = 1 - Rcap^2 / (2 r^2).
"""
import numpy as np

RE = 6378.137e3
MU = 3.986004418e14


def walker(P, S, F, inc_deg, h, star=False):
    """Node and phase of every satellite, arrays of shape (P*S,), plus plane index."""
    p = np.repeat(np.arange(P), S)
    s = np.tile(np.arange(S), P)
    span = np.pi if star else 2 * np.pi
    Om = span * p / P
    u0 = 2 * np.pi * s / S + 2 * np.pi * F * p / (P * S)
    return dict(P=P, S=S, F=F, i=np.radians(inc_deg), r=RE + h, Om=Om, u0=u0, plane=p, star=star)


def pos(sh, u_shift):
    """ECI positions (n,3) at common argument-of-latitude advance u_shift."""
    u = sh["u0"] + u_shift
    O, i, r = sh["Om"], sh["i"], sh["r"]
    x = r * (np.cos(O) * np.cos(u) - np.sin(O) * np.sin(u) * np.cos(i))
    y = r * (np.sin(O) * np.cos(u) + np.cos(O) * np.sin(u) * np.cos(i))
    z = r * np.sin(u) * np.sin(i)
    return np.stack([x, y, z], -1)


def MN(dO, D, inc):
    ci, si = np.cos(inc), np.sin(inc)
    M = np.cos(D) * (np.cos(dO) * (1 + ci ** 2) + si ** 2) / 2 - ci * np.sin(dO) * np.sin(D)
    N = si ** 2 * (1 - np.cos(dO)) / 2
    return M, N


def rcap_eff(r, rmax, h_atm):
    dl = 2 * np.sqrt(r ** 2 - (RE + h_atm) ** 2)
    return min(rmax, dl)


def pair_law(dO, D, inc, r, rcap):
    """Return (c, up_fraction, events_per_orbit) for a pair."""
    M, N = MN(dO, D, inc)
    kappa = 1 - rcap ** 2 / (2 * r ** 2)
    with np.errstate(divide="ignore", invalid="ignore"):
        c = np.where(N > 1e-15, (M - kappa) / np.maximum(N, 1e-300), np.where(M >= kappa, np.inf, -np.inf))
    phi = np.where(c >= 1, 1.0, np.where(c <= -1, 0.0, 1 - np.arccos(np.clip(c, -1, 1)) / np.pi))
    ev = np.where(np.abs(c) < 1, 4, 0)
    return c, phi, ev


def refs(sh):
    """Reference satellites: one per plane for a star shell (no transitivity), else satellite 0."""
    return [p * sh["S"] for p in range(sh["P"])] if sh["star"] else [0]


def shell_law(sh, rcap):
    """Mean degree and ISL events per orbit for the all-feasible range-capped policy."""
    n = sh["P"] * sh["S"]
    deg, ev = 0.0, 0.0
    R = refs(sh)
    for a in R:
        o = np.arange(n) != a
        dO = sh["Om"][o] - sh["Om"][a]
        D = sh["u0"][o] - sh["u0"][a]
        _, phi, e = pair_law(dO, D, sh["i"], sh["r"], rcap)
        deg += phi.sum()
        ev += e.sum()
    w = n / len(R)
    return dict(mean_degree=deg / len(R), events_per_orbit=ev * w / 2, links_mean=deg * w / 2)


def event_times(sh, rcap):
    """All ISL up/down epochs in one orbit, as fractions of the period, with the pair and sign."""
    n = sh["P"] * sh["S"]
    out = []
    a_idx, b_idx = np.triu_indices(n, 1)
    dO = sh["Om"][b_idx] - sh["Om"][a_idx]
    D = sh["u0"][b_idx] - sh["u0"][a_idx]
    c, _, e = pair_law(dO, D, sh["i"], sh["r"], rcap)
    m = e > 0
    a_idx, b_idx, D, c = a_idx[m], b_idx[m], D[m], c[m]
    ac = np.arccos(c)
    for sgn, up in ((+1, True), (-1, False)):
        for k in (0, 1):
            u1 = (sgn * ac - D) / 2 + np.pi * k
            frac = np.mod((u1 - sh["u0"][a_idx]) / (2 * np.pi), 1.0)
            out.append((frac, a_idx, b_idx, np.full(frac.size, up)))
    return out
