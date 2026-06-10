build_lib_example(
  NAME ntn-observability-demo
  SOURCE_FILES ntn-observability-demo.cc
  LIBRARIES_TO_LINK
    ${libntn-observability}
    ${libntn-rrc}
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

# Real LEO downlink packet sim with InfluxDB KPI export (NtnInfluxSink).
build_lib_example(
  NAME ntn-observability-traffic
  SOURCE_FILES ntn-observability-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-observability}
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
