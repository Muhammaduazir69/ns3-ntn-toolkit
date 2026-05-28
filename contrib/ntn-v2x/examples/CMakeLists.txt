build_lib_example(
  NAME ntn-v2x-rural-highway
  SOURCE_FILES ntn-v2x-rural-highway.cc
  LIBRARIES_TO_LINK
    ${libntn-v2x}
    ${libcore}
    ${libmobility}
    ${libntn-traffic}
    ${libnetwork}
    ${libinternet}
    ${libapplications}
    ${libpoint-to-point}
)

# Real 2-hop V2X relay: shadowed vehicle -> relay vehicle -> LEO -> server.
build_lib_example(
  NAME ntn-v2x-leo-relay-traffic
  SOURCE_FILES ntn-v2x-leo-relay-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-v2x}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)
