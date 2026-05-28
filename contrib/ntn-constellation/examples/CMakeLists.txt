# Real ISL-routed packet forwarding GS1 -> satA -> [ISL] -> satB -> GS2.
build_lib_example(
  NAME ntn-constellation-isl-routed-traffic
  SOURCE_FILES ntn-constellation-isl-routed-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-constellation}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)
