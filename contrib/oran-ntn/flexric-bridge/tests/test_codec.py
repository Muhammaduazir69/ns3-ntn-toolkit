"""W8 — codec round-trip tests."""

from __future__ import annotations

from ntn_flexric_bridge import e2ap_codec, e2sm_kpm_ntn, e2sm_rc_ntn


def test_e2_setup_request_round_trip():
    blob = e2ap_codec.encode_e2_setup_request(
        global_e2_node_id=1, plmn_id="001/01",
        ran_function_list=[
            {"ran_function_id": e2sm_kpm_ntn.RAN_FUNCTION_ID,
             "ran_function_revision": 3,
             "ran_function_definition": "E2SM-KPM-NTN/v3"},
        ],
    )
    msg = e2ap_codec.decode_message(blob)
    assert msg["procedure_code"] == int(e2ap_codec.ProcedureCode.E2_SETUP)
    assert msg["type"] == int(e2ap_codec.MessageType.INITIATING_MESSAGE)
    assert msg["value"]["plmn_identity"] == "001/01"


def test_subscription_request_round_trip():
    blob = e2ap_codec.encode_subscription_request(
        ric_request_id={"requestor_id": 0xC0DE, "instance_id": 1},
        ran_function_id=e2sm_kpm_ntn.RAN_FUNCTION_ID,
        event_trigger=e2sm_kpm_ntn.event_trigger_periodic(1000),
        actions=[e2sm_kpm_ntn.action_report(action_id=1,
                                             measurement_types=["rsrp_dbm", "elevation_deg"])],
    )
    msg = e2ap_codec.decode_message(blob)
    assert msg["procedure_code"] == int(e2ap_codec.ProcedureCode.RIC_SUBSCRIPTION)
    assert msg["value"]["actions"][0]["measurement_types"] == ["rsrp_dbm", "elevation_deg"]


def test_kpm_indication_payload_round_trip():
    h = e2sm_kpm_ntn.KpmIndicationHeader(gnb_id=1, sat_norad=44714,
                                         timestamp_iso="2026-05-04T12:00:00+00:00")
    m = e2sm_kpm_ntn.KpmIndicationMessage(
        measurements=[
            e2sm_kpm_ntn.KpmMeasurement("rsrp_dbm", -97.5, ue_imsi="100001"),
            e2sm_kpm_ntn.KpmMeasurement("elevation_deg", 47.3, ue_imsi="100001"),
            e2sm_kpm_ntn.KpmMeasurement("tte_total_us", 3669.0, ue_imsi="100001"),
        ],
    )
    hdr_d, msg_d = e2sm_kpm_ntn.build_indication(h, m)
    h2, m2 = e2sm_kpm_ntn.parse_indication(hdr_d, msg_d)
    assert h2 == h
    assert len(m2.measurements) == 3
    assert m2.measurements[0].measurement_type == "rsrp_dbm"
    assert abs(m2.measurements[1].value - 47.3) < 1e-9


def test_rc_cho_trigger_round_trip():
    h = e2sm_rc_ntn.RcControlHeader(gnb_id=1, ue_imsi="100001")
    m = e2sm_rc_ntn.ChoTriggerMessage(target_cell_id="cell-44715",
                                       target_sat_norad=44715,
                                       expected_tta_ms=3.67)
    hd, md = e2sm_rc_ntn.build_cho_trigger(h, m)
    h2, m2 = e2sm_rc_ntn.parse_cho_trigger(hd, md)
    assert h2.ue_imsi == "100001"
    assert m2.target_sat_norad == 44715
    assert abs(m2.expected_tta_ms - 3.67) < 1e-9
