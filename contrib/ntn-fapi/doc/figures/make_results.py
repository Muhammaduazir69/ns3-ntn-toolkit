#!/usr/bin/env python3
"""Results figure of ntn-fapi from stored example runs (no new simulation).

Source: the toolkit audit of 6 Oct 2026, ns-3-dev/toolkit-audit-2026-10-06/runs/A1-cho-rrc-fapi/
  fapi-rs-def    ntn-fapi-real-stack --duration=5                (fapi_sap.csv + run.log)
  fapi-dl-def    ntn-fapi-dl-data-slotloop --simSeconds=5         (run.log)
  fapi-dl-seed2  ntn-fapi-dl-data-slotloop --simSeconds=5 --RngRun=2
  fapi-leo-def   ntn-fapi-leo-pass-slotloop --simSeconds=5        (ISS TLE)
One run per configuration. Writes ntn-fapi-results.png (200 dpi) and .svg.

    nice -n 19 python3 make_results.py [path/to/A1-cho-rrc-fapi]
"""
import csv
import os
import re
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = sys.argv[1] if len(sys.argv) > 1 else os.path.abspath(
    os.path.join(HERE, "..", "..", "..", "..", "toolkit-audit-2026-10-06", "runs", "A1-cho-rrc-fapi"))


def sap_csv():
    with open(os.path.join(RUNS, "fapi-rs-def", "ntn-fapi-real-stack-output", "fapi_sap.csv")) as f:
        return {r["metric"]: float(r["value"]) for r in csv.DictReader(f)}


def log_stats(run):
    txt = open(os.path.join(RUNS, run, "run.log")).read()
    m = re.search(r"sent=(\d+)\s+crcOk=(\d+)\s+retx=(\d+)", txt)
    if m is None:  # real-stack prints the SAP counts instead
        crc = int(re.search(r"CRC\.indication emitted:\s+(\d+)", txt).group(1))
        rx = int(re.search(r"RX_DATA\.indication emitted:\s+(\d+)", txt).group(1))
        m = re.match(r"(\d+) (\d+) (\d+)", "%d %d %d" % (crc, rx, crc - rx))
    tb = re.search(r"phy_rx_tb=(\d+)", txt)
    tx = re.search(r"TX_DATA(?:\.request)?[:=]\s*(\d+)", txt) or tb
    tbler = re.search(r"tbler=([0-9.]+)", txt)
    sinr = re.search(r"dl_sinr=([0-9.]+)dB", txt)
    return dict(sent=int(m.group(1)), ok=int(m.group(2)), retx=int(m.group(3)), phy_rx_tb=int(tb.group(1)),
                tx_data=int(tx.group(1)), tbler=float(tbler.group(1)), sinr=float(sinr.group(1)),
                has_tx=tx is not tb)


def main():
    plt.rcParams.update({"font.family": "sans-serif",
                         "font.sans-serif": ["Arial", "Nimbus Sans", "Liberation Sans", "DejaVu Sans"],
                         "font.size": 9, "axes.linewidth": 0.8, "svg.fonttype": "none"})
    s = sap_csv()
    runs = [("fapi-dl-def", "dl-data-slotloop\nseed 1"), ("fapi-dl-seed2", "dl-data-slotloop\nseed 2"),
            ("fapi-leo-def", "leo-pass-slotloop\nISS TLE"), ("fapi-rs-def", "real-stack\nseed 1")]
    st = [log_stats(r) for r, _ in runs]

    fig, (a, b) = plt.subplots(1, 2, figsize=(10.0, 3.7), gridspec_kw={"width_ratios": [1.0, 1.25]})
    # (a) latency against the geometric floor
    labels = ["Scheduler pipeline\nDL_TTI.request to SLOT.ind.", "SAP turnaround\nDL_TTI.request to CRC.ind.",
              "One-way propagation\nfloor of the geometry"]
    vals = [s["sched_pipeline_mean_us"], s["sap_latency_mean_us"], s["one_way_propagation_floor_us"]]
    cols = ["#91ace0", "#3d7199", "#c9c9c9"]
    y = range(len(vals))
    a.barh(y, vals, color=cols, edgecolor="#2e2e2e", lw=0.6, height=0.55)
    a.errorbar([s["sap_latency_mean_us"]], [1], xerr=[[s["sap_latency_mean_us"] - s["sap_latency_min_us"]],
                                                      [s["sap_latency_max_us"] - s["sap_latency_mean_us"]]],
               fmt="none", ecolor="#1a1a1a", capsize=4, lw=1.0)
    for i, v in enumerate(vals):
        xx = s["sap_latency_max_us"] if i == 1 else v
        a.text(xx + 50, i, "%.0f us" % v + (" (min %.0f, max %.0f)" % (s["sap_latency_min_us"], s["sap_latency_max_us"])
                                         if i == 1 else ""), va="center", fontsize=8.3)
    a.set_yticks(list(y)); a.set_yticklabels(labels, fontsize=8.2); a.invert_yaxis()
    a.set_xlim(0, 2400); a.set_xlabel("Time (us)")
    a.set_title("(a) Latency off the MAC-PHY SAP, %d matched pairs" % s["matched_latency_count"], fontsize=9.5)
    a.grid(axis="x", ls=":", lw=0.6)
    # (b) per-TB accounting
    x = range(len(runs))
    ok = [r["ok"] for r in st]; re_ = [r["retx"] for r in st]
    b.bar(x, ok, color="#75bd42", edgecolor="#2e2e2e", lw=0.6, width=0.55, label="CRC.indication pass")
    b.bar(x, re_, bottom=ok, color="#e54c5e", edgecolor="#2e2e2e", lw=0.6, width=0.55,
          label="CRC fail, HARQ retransmission")
    for i, r in enumerate(st):
        b.text(i, r["sent"] + 80, "SINR %.1f dB\nTBLER %.3f" % (r["sinr"], r["tbler"]),
               ha="center", va="bottom", fontsize=7.6)
    b.set_xticks(list(x)); b.set_xticklabels([l for _, l in runs], fontsize=8)
    b.set_ylim(0, 7400); b.set_ylabel("FAPI data slots in 5 s")
    b.set_title("(b) CRC outcome decided by the measured PHY decode", fontsize=9.5)
    b.legend(loc="upper left", fontsize=7.8, frameon=False, ncol=2)
    b.grid(axis="y", ls=":", lw=0.6)
    for ax_ in (a, b):
        ax_.spines["top"].set_visible(False); ax_.spines["right"].set_visible(False)
    fig.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(HERE, "ntn-fapi-results." + ext), dpi=200, facecolor="white")
    for (r, _), v in zip(runs, st):
        print(r, v)
    print(s)


if __name__ == "__main__":
    main()
