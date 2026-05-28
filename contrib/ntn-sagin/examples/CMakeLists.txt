build_lib_example(
  NAME sagin-haps-leo-relay
  SOURCE_FILES sagin-haps-leo-relay.cc
  LIBRARIES_TO_LINK
    ${libntn-sagin}
    ${libcore}
    ${libmobility}
    ${libntn-traffic}
    ${libnetwork}
    ${libinternet}
    ${libapplications}
    ${libpoint-to-point}
)
build_lib_example(
  NAME sagin-uav-swarm
  SOURCE_FILES sagin-uav-swarm.cc
  LIBRARIES_TO_LINK
    ${libntn-sagin}
    ${libcore}
    ${libmobility}
    ${libntn-traffic}
    ${libnetwork}
    ${libinternet}
    ${libapplications}
    ${libpoint-to-point}
)
build_lib_example(
  NAME sagin-aeronautical
  SOURCE_FILES sagin-aeronautical.cc
  LIBRARIES_TO_LINK
    ${libntn-sagin}
    ${libcore}
    ${libmobility}
    ${libntn-traffic}
    ${libnetwork}
    ${libinternet}
    ${libapplications}
    ${libpoint-to-point}
)

# Roadmap §4.4.9 — commercial-flight + LEO with O-RAN E2-KPM indications.
build_lib_example(
  NAME sagin-flight-leo-e2
  SOURCE_FILES sagin-flight-leo-e2.cc
  LIBRARIES_TO_LINK
    ${libntn-sagin}
    ${liboran-ntn}
    ${libcore}
    ${libmobility}
    ${libnetwork}
)

# Real end-to-end multi-hop forwarding Ground→UAV→HAPS→LEO with per-hop
# geometry-driven error models and IPv4 global routing.
build_lib_example(
  NAME sagin-multihop-traffic
  SOURCE_FILES sagin-multihop-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-sagin}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)

# Cross-layer network-slice steering (SaginSliceRouter) with real per-slice traffic.
build_lib_example(
  NAME sagin-slice-traffic
  SOURCE_FILES sagin-slice-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-sagin}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)

# Maritime AIS vessel terminal + LEO downlink (AisMobilityModel), real traffic.
build_lib_example(
  NAME sagin-maritime-leo-traffic
  SOURCE_FILES sagin-maritime-leo-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-sagin}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)

# High-speed-train terminal + LEO downlink (HstMobilityModel), real traffic.
build_lib_example(
  NAME sagin-hst-leo-traffic
  SOURCE_FILES sagin-hst-leo-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-sagin}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)

# ADS-B aircraft terminal + LEO downlink (OpenSkyMobilityModel), real traffic.
build_lib_example(
  NAME sagin-adsb-flight-leo-traffic
  SOURCE_FILES sagin-adsb-flight-leo-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-sagin}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)

# Roadmap §4.4.4/§4.4.5 — SGP4-driven SAGIN with graph shortest-path ISL routing.
# Composes ntn-constellation (Sgp4MobilityModel + ContactGraphScheduler +
# ContactGraphRouter) with a real ns-3 IP data plane whose link states follow
# the live contact graph.
build_lib_example(
  NAME sagin-sgp4-routed-traffic
  SOURCE_FILES sagin-sgp4-routed-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-sagin}
    ${libntn-constellation}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)
