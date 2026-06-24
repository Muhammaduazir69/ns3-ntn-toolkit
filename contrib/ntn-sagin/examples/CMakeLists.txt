# SAGIN terminals (HAPS / UAV / aircraft / train / vessel) served by a LEO over a
# REAL mmwave NR NTN cell (NtnRealStackHelper), with measured SINR/TBLER/throughput.
set(_sagin_real_libs
  ${libntn-sagin}
  ${libntn-traffic}
  ${libntn-cho}
  ${libntn-constellation}
  ${libcore}
  ${libnetwork}
  ${libmobility}
  ${libspectrum}
  ${libpropagation}
  ${libinternet}
  ${libapplications}
  ${libmmwave}
  ${liblte}
)

build_lib_example(
  NAME sagin-haps-leo-relay
  SOURCE_FILES sagin-haps-leo-relay.cc
  LIBRARIES_TO_LINK ${_sagin_real_libs}
)
build_lib_example(
  NAME sagin-uav-swarm
  SOURCE_FILES sagin-uav-swarm.cc
  LIBRARIES_TO_LINK ${_sagin_real_libs}
)

# Phase-2 real-stack (channel-plugin recipe): TR 36.777 A2G path loss re-homed as
# a real PropagationLossModel and chained on a real mmwave NR link; SINR measured.
build_lib_example(
  NAME sagin-a2g-real-stack
  SOURCE_FILES sagin-a2g-real-stack.cc
  LIBRARIES_TO_LINK ${_sagin_real_libs}
)
build_lib_example(
  NAME sagin-aeronautical
  SOURCE_FILES sagin-aeronautical.cc
  LIBRARIES_TO_LINK ${_sagin_real_libs}
)

# Roadmap §4.4.9 — commercial-flight + LEO with O-RAN E2-KPM indications.
build_lib_example(
  NAME sagin-flight-leo-e2
  SOURCE_FILES sagin-flight-leo-e2.cc
  LIBRARIES_TO_LINK ${_sagin_real_libs} ${liboran-ntn}
)

# Real end-to-end multi-hop forwarding Ground→UAV→HAPS→LEO; the air-to-ground hop
# uses the TR 36.777 A2G PropagationLossModel, the rest real IPv4 global routing.
build_lib_example(
  NAME sagin-multihop-traffic
  SOURCE_FILES sagin-multihop-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-sagin}
    ${libntn-cho}
    ${libntn-constellation}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libntn-traffic}
)

# Cross-layer network-slice steering (SaginSliceRouter) with real per-slice traffic.
build_lib_example(
  NAME sagin-slice-traffic
  SOURCE_FILES sagin-slice-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-sagin}
    ${libntn-cho}
    ${libntn-constellation}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libntn-traffic}
)

# Maritime AIS vessel terminal + LEO downlink (AisMobilityModel), real mmwave cell.
build_lib_example(
  NAME sagin-maritime-leo-traffic
  SOURCE_FILES sagin-maritime-leo-traffic.cc
  LIBRARIES_TO_LINK ${_sagin_real_libs}
)

# High-speed-train terminal + LEO downlink (HstMobilityModel), real mmwave cell.
build_lib_example(
  NAME sagin-hst-leo-traffic
  SOURCE_FILES sagin-hst-leo-traffic.cc
  LIBRARIES_TO_LINK ${_sagin_real_libs}
)

# ADS-B aircraft terminal + LEO downlink (OpenSkyMobilityModel), real mmwave cell.
build_lib_example(
  NAME sagin-adsb-flight-leo-traffic
  SOURCE_FILES sagin-adsb-flight-leo-traffic.cc
  LIBRARIES_TO_LINK ${_sagin_real_libs}
)

# Roadmap §4.4.4/§4.4.5 — SGP4-driven SAGIN with graph shortest-path ISL routing.
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
    ${libntn-traffic}
)

# WS6 — remote coverage with multi-MNO infrastructure sharing.
build_lib_example(
  NAME ntn-sagin-remote-coverage
  SOURCE_FILES ntn-sagin-remote-coverage.cc
  LIBRARIES_TO_LINK ${_sagin_real_libs}
)

# Gives the two orphan mobility models (HAPS-trajectory + maritime) a home,
# with a 3D scene trace.
build_lib_example(
  NAME ntn-sagin-orphan-mobility-showcase
  SOURCE_FILES ntn-sagin-orphan-mobility-showcase.cc
  LIBRARIES_TO_LINK
    ${libntn-sagin}
    ${libntn-v2x}
    ${libntn-constellation}
    ${libntn-cho}
    ${libntn-observability}
    ${libntn-traffic}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)
