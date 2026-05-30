build_lib_example(
  NAME ntn-observability-demo
  SOURCE_FILES ntn-observability-demo.cc
  LIBRARIES_TO_LINK
    ${libntn-observability}
    ${libntn-rrc}
    ${libntn-traffic}
    ${libcore}
    ${libmobility}
    ${libnetwork}
    ${libinternet}
    ${libapplications}
    ${libpoint-to-point}
)

# Real LEO downlink packet sim with InfluxDB KPI export (NtnInfluxSink).
build_lib_example(
  NAME ntn-observability-traffic
  SOURCE_FILES ntn-observability-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-observability}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)
