build_lib_example(
  NAME ntn-rrc-leo-pass
  SOURCE_FILES ntn-rrc-leo-pass.cc
  LIBRARIES_TO_LINK
    ${libntn-rrc}
    ${libcore}
    ${libmobility}
    ${libntn-traffic}
    ${libnetwork}
    ${libinternet}
    ${libapplications}
    ${libpoint-to-point}
)
build_lib_example(
  NAME ntn-rrc-full-stack
  SOURCE_FILES ntn-rrc-full-stack.cc
  LIBRARIES_TO_LINK
    ${libntn-rrc}
    ${libsatellite}
    ${libcore}
    ${libmobility}
    ${libntn-traffic}
    ${libnetwork}
    ${libinternet}
    ${libapplications}
    ${libpoint-to-point}
)
build_lib_example(
  NAME ntn-rrc-from-tle
  SOURCE_FILES ntn-rrc-from-tle.cc
  LIBRARIES_TO_LINK
    ${libntn-rrc}
    ${libsatellite}
    ${libcore}
    ${libmobility}
    ${libntn-traffic}
    ${libnetwork}
    ${libinternet}
    ${libapplications}
    ${libpoint-to-point}
)

# Real downlink traffic gated by SIB19 + timing-advance + connected-mode DRX.
build_lib_example(
  NAME ntn-rrc-drx-data-traffic
  SOURCE_FILES ntn-rrc-drx-data-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-rrc}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)
