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
