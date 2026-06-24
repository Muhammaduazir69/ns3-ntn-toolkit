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

# Path B (optional): drive the OFFICIAL usnistgov NetSimulyzer module from the
# toolkit's real SGP4 + TR 38.811 mobility. Built only when the vendored
# netsimulyzer module is present in contrib/.
if(TARGET ${libnetsimulyzer})
  build_lib_example(
    NAME ntn-netsimulyzer-official-demo
    SOURCE_FILES ntn-netsimulyzer-official-demo.cc
    LIBRARIES_TO_LINK
      ${libnetsimulyzer}
      ${libntn-observability}
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
endif()
