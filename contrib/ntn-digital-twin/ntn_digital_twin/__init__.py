"""ntn-digital-twin — live digital-twin loop and REST API for ns3-ntn-toolkit.

Two entry points:

* ``ntn-twin-loop``   — long-running cron-style refresher: every
                        ``REFRESH_INTERVAL_SEC`` it pulls TLEs via W1's
                        ``CelesTrakFeed``, propagates the constellation,
                        emits CZML for the CesiumJS viewer, and pushes
                        line-protocol points to InfluxDB (W3).

* ``ntn-twin-api``    — FastAPI server with ``/health``, ``/constellation/state``,
                        and ``/predict/handover`` endpoints — answers under
                        500 ms.
"""

__version__ = "0.1.0"
__all__ = ["__version__"]
