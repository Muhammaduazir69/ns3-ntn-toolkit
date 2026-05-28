build_lib_example(
  NAME ntn-cho-leo-basic
  SOURCE_FILES ntn-cho-leo-basic.cc
  LIBRARIES_TO_LINK
    ${libntn-cho}
    ${libntn-traffic}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libinternet}
    ${libapplications}
    ${libpoint-to-point}
)

build_lib_example(
  NAME ntn-cho-full-constellation
  SOURCE_FILES ntn-cho-full-constellation.cc
  LIBRARIES_TO_LINK
    ${libntn-cho}
    ${libntn-traffic}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libinternet}
    ${libapplications}
    ${libpoint-to-point}
)

build_lib_example(
  NAME ntn-realistic-mobility-demo
  SOURCE_FILES ntn-realistic-mobility-demo.cc
  LIBRARIES_TO_LINK
    ${libntn-cho}
    ${libcore}
)

# Real UDP traffic handed over between two LEO sats by the TTE-aware CHO.
build_lib_example(
  NAME ntn-cho-handover-traffic
  SOURCE_FILES ntn-cho-handover-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-cho}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)
