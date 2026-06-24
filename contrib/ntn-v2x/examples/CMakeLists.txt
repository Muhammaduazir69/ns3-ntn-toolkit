build_lib_example(
  NAME ntn-v2x-rural-highway
  SOURCE_FILES ntn-v2x-rural-highway.cc
  LIBRARIES_TO_LINK
    ${libntn-v2x}
    ${libntn-cho}
    ${libntn-constellation}
    ${libntn-traffic}
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

# Phase-2 real-stack (RAN recipe): direct-vs-relay decided on the MEASURED per-UE
# DL SINR of a real mmwave NR cell, not V2xLeoDirect's closed-form budget.
build_lib_example(
  NAME ntn-v2x-real-stack
  SOURCE_FILES ntn-v2x-real-stack.cc
  LIBRARIES_TO_LINK
    ${libntn-v2x}
    ${libntn-cho}
    ${libntn-constellation}
    ${libntn-traffic}
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

# Real 2-hop V2X relay: shadowed vehicle -> relay vehicle -> LEO -> server.
build_lib_example(
  NAME ntn-v2x-leo-relay-traffic
  SOURCE_FILES ntn-v2x-leo-relay-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-v2x}
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

# Real maritime NTN access: an AIS-replayed vessel UE (ntn-sagin AisMobilityModel
# driven by AisDanishImporter on a real Danish Maritime CSV) on a real mmwave NR
# cell. Replaces the dead box-bounce MaritimeMobilityModel with measured SINR.
build_lib_example(
  NAME ntn-v2x-maritime-ais
  SOURCE_FILES ntn-v2x-maritime-ais.cc
  LIBRARIES_TO_LINK
    ${libntn-v2x}
    ${libntn-sagin}
    ${libntn-cho}
    ${libntn-constellation}
    ${libntn-traffic}
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

# WS6 — V2X edge URLLC with sat-vs-ground inference placement.
build_lib_example(
  NAME ntn-v2x-edge-urllc
  SOURCE_FILES ntn-v2x-edge-urllc.cc
  LIBRARIES_TO_LINK
    ${libntn-v2x}
    ${liboran-ntn}
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
