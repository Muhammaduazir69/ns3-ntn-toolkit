"""ntn-flexric-bridge — E2 real-wire integration for the ns3-ntn-toolkit (W8).

One implemented mode plus one roadmapped mode:

  * **Stub mode (implemented)**  — `Wire.JSON_LOOPBACK` over TCP localhost.
    Lets CI exercise the full agent ↔ xApp message flow with no external
    dependencies. The wire bytes are JSON-on-the-wire framed by a 4-byte
    big-endian length; the message **semantics** mirror E2AP's field
    structure, but the JSON values are not ASN.1-PER bits, so this does not
    interoperate with a real asn1c/FlexRIC peer.

  * **Live mode (roadmapped, NOT built)**  — `Wire.E2AP_SCTP` over SCTP
    (port 36421/36422). Intended to use FlexRIC's asn1c-generated E2AP codec
    provided by the Docker image in `flexric-bridge/docker/`. That codec,
    the cffi shim, and the live association do not exist in this repo yet;
    `docs/BUILD_FLEXRIC.md` is an unverified bring-up recipe, not a working
    path. Reaching live mode is more than "only the codec change".

The package mirrors the message taxonomy of E2AP v1.01 / E2SM-KPM v3 /
E2SM-RC v1.03:

    E2 Setup Request          → e2ap_codec.encode_e2_setup_request
    RIC Subscription Request  → e2ap_codec.encode_subscription_request
    RIC Indication            → e2sm_kpm_ntn.build_indication
    RIC Control Request       → e2sm_rc_ntn.build_control_request
"""

__version__ = "0.1.0"
__all__ = ["__version__"]
