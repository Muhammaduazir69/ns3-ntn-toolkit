"""ntn-flexric-bridge — E2 real-wire integration for the ns3-ntn-toolkit (W8).

Two operating modes:

  * **Stub mode**  — `Wire.JSON_LOOPBACK` over TCP localhost.
    Lets CI exercise the full agent ↔ xApp message flow with no external
    dependencies. The wire bytes are JSON-on-the-wire framed by a 4-byte
    big-endian length, but the **semantics** mirror E2AP exactly so a
    drop-in switch to live mode requires only the codec change.

  * **Live mode**  — `Wire.E2AP_SCTP` over SCTP (port 36421/36422).
    Uses FlexRIC's asn1c-generated E2AP codec. Provided by the Docker
    image in `flexric-bridge/docker/`. See `docs/BUILD_FLEXRIC.md` for
    instructions on building FlexRIC and running the live stack.

The package mirrors the message taxonomy of E2AP v1.01 / E2SM-KPM v3 /
E2SM-RC v1.03:

    E2 Setup Request          → e2ap_codec.encode_e2_setup_request
    RIC Subscription Request  → e2ap_codec.encode_subscription_request
    RIC Indication            → e2sm_kpm_ntn.build_indication
    RIC Control Request       → e2sm_rc_ntn.build_control_request
"""

__version__ = "0.1.0"
__all__ = ["__version__"]
