"""W8 — end-to-end loopback test: agent ↔ RIC stub ↔ xApps over TCP.

Exercises the full subscription / indication / control state machine and
asserts:
  - E2 setup completes for every xApp
  - 100 RIC Indications delivered without loss across a synthetic 'pass'
  - CHO xApp produces a handover sequence matching a hand-computed reference
"""

from __future__ import annotations

import threading
import time

import pytest

from ntn_flexric_bridge import e2sm_kpm_ntn
from ntn_flexric_bridge.e2_agent_ns3 import E2AgentNs3
from ntn_flexric_bridge.xapps.cho_xapp import ChoConfig, ChoXapp


PORT_BASE = 39000  # tests pick distinct ports so they can run in parallel


def _start_agent(port: int) -> tuple[E2AgentNs3, threading.Thread]:
    agent = E2AgentNs3(gnb_id=1)
    th = threading.Thread(target=agent.run_stub_server,
                          kwargs={"port": port, "forever": False},
                          daemon=True)
    th.start()
    # Give the listening socket a moment to bind.
    time.sleep(0.05)
    return agent, th


def test_setup_subscribe_indicate_flow():
    agent, _ = _start_agent(PORT_BASE)
    xapp = ChoXapp(ChoConfig())
    xapp.connect("127.0.0.1", PORT_BASE)
    try:
        assert xapp.stats.setup_done
        assert agent.stats.setup_done
        xapp.subscribe_kpm(["rsrp_dbm", "elevation_deg", "tte_total_us"], period_ms=100)
        # Wait for subscription response to land.
        time.sleep(0.2)
        assert agent.stats.subscriptions == 1

        # Push 100 KPM samples into the agent — they should reach the xApp.
        for k in range(100):
            agent.submit_kpm_measurement(
                e2sm_kpm_ntn.KpmIndicationHeader.make(gnb_id=1, sat_norad=44714),
                e2sm_kpm_ntn.KpmIndicationMessage(measurements=[
                    e2sm_kpm_ntn.KpmMeasurement("rsrp_dbm", -90.0 + k * 0.1, ue_imsi="100001"),
                ]),
            )
        time.sleep(0.5)
        assert agent.stats.indications_sent == 100
        assert xapp.stats.indications_received == 100
    finally:
        xapp.stop()


def test_cho_xapp_triggers_on_low_serving_elevation():
    """Synthetic pass: serving sat A drops below 30°, candidate B rises above 35°.

    Expected: cho-xapp emits a CHO trigger for UE 100001 from sat A → B.
    """
    agent, _ = _start_agent(PORT_BASE + 1)
    xapp = ChoXapp(ChoConfig(handover_below_deg=30.0, handover_above_deg=35.0,
                              hysteresis_db=2.0))
    xapp.connect("127.0.0.1", PORT_BASE + 1)
    try:
        xapp.subscribe_kpm(["rsrp_dbm", "elevation_deg", "tte_total_us"], period_ms=50)
        time.sleep(0.1)

        SAT_A, SAT_B = 44714, 44715
        # Tick 0: only sat A is observed → becomes serving.
        agent.submit_kpm_measurement(
            e2sm_kpm_ntn.KpmIndicationHeader.make(gnb_id=1, sat_norad=SAT_A),
            e2sm_kpm_ntn.KpmIndicationMessage(measurements=[
                e2sm_kpm_ntn.KpmMeasurement("elevation_deg", 60.0, ue_imsi="100001"),
                e2sm_kpm_ntn.KpmMeasurement("rsrp_dbm", -85.0, ue_imsi="100001"),
                e2sm_kpm_ntn.KpmMeasurement("tte_total_us", 4000.0, ue_imsi="100001"),
            ]),
        )
        time.sleep(0.05)
        # Tick 1: A drops below 30°, B rises above 35° with stronger RSRP.
        agent.submit_kpm_measurement(
            e2sm_kpm_ntn.KpmIndicationHeader.make(gnb_id=1, sat_norad=SAT_A),
            e2sm_kpm_ntn.KpmIndicationMessage(measurements=[
                e2sm_kpm_ntn.KpmMeasurement("elevation_deg", 25.0, ue_imsi="100001"),
                e2sm_kpm_ntn.KpmMeasurement("rsrp_dbm", -98.0, ue_imsi="100001"),
            ]),
        )
        agent.submit_kpm_measurement(
            e2sm_kpm_ntn.KpmIndicationHeader.make(gnb_id=1, sat_norad=SAT_B),
            e2sm_kpm_ntn.KpmIndicationMessage(measurements=[
                e2sm_kpm_ntn.KpmMeasurement("elevation_deg", 40.0, ue_imsi="100001"),
                e2sm_kpm_ntn.KpmMeasurement("rsrp_dbm", -88.0, ue_imsi="100001"),
                e2sm_kpm_ntn.KpmMeasurement("tte_total_us", 3300.0, ue_imsi="100001"),
            ]),
        )
        time.sleep(0.5)
        # The xApp should have emitted exactly one CHO trigger A→B.
        assert len(xapp.handovers) == 1
        ho = xapp.handovers[0]
        assert ho["from_sat"] == SAT_A
        assert ho["to_sat"] == SAT_B
        assert ho["target_cell"] == f"cell-{SAT_B}"
        assert abs(ho["expected_tta_ms"] - 3.3) < 0.01
        # And the agent should have logged the same handover (gate-2 equivalence).
        time.sleep(0.1)
        log = agent.get_handover_log()
        assert len(log) == 1
        assert log[0]["target_sat_norad"] == SAT_B
        assert log[0]["target_cell_id"] == f"cell-{SAT_B}"
    finally:
        xapp.stop()


def test_cho_xapp_matches_inmemory_oracle():
    """Equivalence gate: replay a 30-step synthetic pass; xApp HO sequence ==
    the in-process reference computed with the same thresholds.
    """
    agent, _ = _start_agent(PORT_BASE + 2)
    cfg = ChoConfig(handover_below_deg=30.0, handover_above_deg=35.0,
                    hysteresis_db=3.0)
    xapp = ChoXapp(cfg)
    xapp.connect("127.0.0.1", PORT_BASE + 2)
    try:
        xapp.subscribe_kpm(["rsrp_dbm", "elevation_deg", "tte_total_us"], period_ms=20)
        time.sleep(0.05)

        # Three sats following sinusoidal elevation curves with phase offsets.
        # The expected handover sequence is computed below by the same rule.
        import math
        sats = [44714, 44715, 44716]
        phases = [0.0, 1.0, 2.0]
        rsrp_at_el = lambda el: -120.0 + el * 0.5  # better RSRP at higher el

        # Reference handover oracle: a *direct* implementation of the rule.
        serving = None
        ref_handovers: list[tuple[int, int, int]] = []  # (step, from, to)

        for step in range(30):
            elevations = {}
            for s, p in zip(sats, phases):
                el = max(0.0, 60.0 * math.sin(0.2 * step + p))
                elevations[s] = el
                agent.submit_kpm_measurement(
                    e2sm_kpm_ntn.KpmIndicationHeader.make(gnb_id=1, sat_norad=s),
                    e2sm_kpm_ntn.KpmIndicationMessage(measurements=[
                        e2sm_kpm_ntn.KpmMeasurement("elevation_deg", el, ue_imsi="ue1"),
                        e2sm_kpm_ntn.KpmMeasurement("rsrp_dbm", rsrp_at_el(el),
                                                    ue_imsi="ue1"),
                        e2sm_kpm_ntn.KpmMeasurement("tte_total_us", 3000.0 + (60 - el) * 100.0,
                                                    ue_imsi="ue1"),
                    ]),
                )
                # Update reference state machine right after each ind.
                if serving is None:
                    serving = s
                    continue
                serv_el = elevations.get(serving, -90)
                if serv_el >= cfg.handover_below_deg:
                    continue
                # find best candidate above threshold with rsrp better by hysteresis
                best = None
                for cand_s, cand_el in elevations.items():
                    if cand_s == serving:
                        continue
                    if cand_el < cfg.handover_above_deg:
                        continue
                    cand_r = rsrp_at_el(cand_el)
                    serv_r = rsrp_at_el(serv_el)
                    if cand_r <= serv_r + cfg.hysteresis_db:
                        continue
                    if best is None or cand_r > best[1]:
                        best = (cand_s, cand_r)
                if best is not None:
                    ref_handovers.append((step, serving, best[0]))
                    serving = best[0]
            time.sleep(0.005)
        time.sleep(0.5)
        xapp_hos = [(h["from_sat"], h["to_sat"]) for h in xapp.handovers]
        ref_hos = [(f, t) for _step, f, t in ref_handovers]
        assert xapp_hos == ref_hos, (
            f"xApp HO sequence != reference\n"
            f"  xapp: {xapp_hos}\n"
            f"  ref:  {ref_hos}"
        )
    finally:
        xapp.stop()
