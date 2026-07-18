"""WS-C: the twin exports a handover-prediction file that the ns-3 C++ consumer
(ns3::OranNtnTwinPredictionConsumer) can read.

This checks the Python side of the twin<->sim loop contract: a deterministic
Walker constellation over a fixed epoch produces at least one handover over a
pass window, and the emitted file matches the exact line format the C++
LoadPredictionsFromFile() parses (`t_s,ueId,recommendedGnbId,confidence`, with
'#' comment lines and a shared-epoch header). Deterministic: fixed epoch, fixed
elements, no wall-clock now().
"""

from __future__ import annotations

import datetime as dt
from pathlib import Path

from ntn_digital_twin.twin_loop import LoopConfig, emit_predictions_file


def test_wsc_predictions_export(tmp_path: Path) -> None:
    out = tmp_path / "twin-predictions.txt"
    cfg = LoopConfig(
        source="walker",
        walker_planes=6,
        walker_sats_per_plane=12,
        walker_altitude_km=600.0,
        walker_inclination_deg=53.0,
        walker_phasing_factor=1,
        epoch_unix_s=1735689600.0,  # 2025-01-01 UTC, fixed
        max_sats=72,
    )

    n = emit_predictions_file(
        cfg,
        observer_lat_deg=33.6844,
        observer_lon_deg=73.0479,
        observer_alt_m=540.0,
        out_path=out,
        horizon_s=45 * 60.0,  # 45-min window -> multiple handovers
        step_s=15.0,
        ue_id=0,
        min_confidence=0.0,
    )

    assert out.exists(), "emitter must write the predictions file"
    assert n >= 2, f"a 45-min pass window must contain multiple handovers, got {n}"

    lines = out.read_text().splitlines()
    header = [ln for ln in lines if ln.startswith("#")]
    data = [ln for ln in lines if ln and not ln.startswith("#")]

    assert any("epoch_unix=" in h for h in header), "must carry the shared-epoch header"
    assert len(data) == n, "one data line per prediction"

    # Every data line matches the C++ contract: t_s,ueId,recommendedGnbId,confidence
    prev_t = -1.0
    for ln in data:
        parts = ln.split(",")
        assert len(parts) == 4, f"line must have 4 comma fields: '{ln}'"
        t_s = float(parts[0])
        ue_id = int(parts[1])
        gnb = int(parts[2])
        conf = float(parts[3])
        assert t_s >= prev_t, "predictions must be time-ordered"
        prev_t = t_s
        assert ue_id == 0
        assert gnb >= 1, "recommendedGnbId is 1-indexed (matches ns-3 E2 nodes)"
        assert 0.0 <= conf <= 1.0

    print(f"[wsc] PASS: wrote {n} time-ordered handover predictions readable by the C++ consumer")
