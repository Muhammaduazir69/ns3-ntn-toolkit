build_lib_example(
  NAME ntn-three-slice-leo-geo
  SOURCE_FILES ntn-three-slice-leo-geo.cc
  LIBRARIES_TO_LINK
    ${libntn-slice}
    ${libcore}
    ${libnetwork}
    ${libntn-traffic}
    ${libnetwork}
    ${libinternet}
    ${libapplications}
    ${libpoint-to-point}
)

# Real per-slice traffic with NtnSliceSelector + SliceIsolationMonitor.
build_lib_example(
  NAME ntn-slice-isolation-traffic
  SOURCE_FILES ntn-slice-isolation-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-slice}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)
