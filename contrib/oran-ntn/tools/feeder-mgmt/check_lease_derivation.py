#!/usr/bin/env python3
# Copy of papers/paper3_tnsm_oran_ntn/sim/ from the author's tree; paths below are absolute to that tree.
"""T10 check of Eq. (cost) and Eq. (tau): numeric argmin of J(tau_L) vs the
closed form tau* = -T ln(1 - eps), eps = co*Delta/(cv*pv), for random inputs
and the edge cases; also checks that the minimizer does not depend on D."""
import math, random
def J(tl, D, T, a, b):  # a = cv*pv, b = co*Delta
    u = min(tl, D)
    hold = a * (u - T * (1 - math.exp(-u / T)))   # closed integral of a(1-e^{-t/T})
    return hold + b * max(D - tl, 0.0)
random.seed(1)
worst = 0
for _ in range(2000):
    T = random.uniform(10, 300); a = random.uniform(0.1, 5); b = random.uniform(0.01, 5)
    D = random.uniform(1, 2000)
    grid = [D * i / 20000 for i in range(20001)] + [D * 1.5]
    tl_num = min(grid, key=lambda x: J(x, D, T, a, b))
    eps = b / a
    tstar = -T * math.log(1 - eps) if eps < 1 else float('inf')
    # optimal cost via closed form must equal numeric min cost
    jstar = J(min(tstar, D * 1.5), D, T, a, b)
    jnum = J(tl_num, D, T, a, b)
    worst = max(worst, (jstar - jnum) / max(jnum, 1e-9))
print(f"max relative excess cost of tau* over numeric argmin: {worst:.2e}")
# numeric integral check of the closed form used above
T, a = 60.0, 1.0
for u in [0.5, 10, 100]:
    n = 200000; h = u / n
    num = sum(a * (1 - math.exp(-(i + 0.5) * h / T)) * h for i in range(n))
    print(u, num, a * (u - T * (1 - math.exp(-u / T))))
print("tau*(T=60, eps=0.2) =", -60 * math.log(0.8))
