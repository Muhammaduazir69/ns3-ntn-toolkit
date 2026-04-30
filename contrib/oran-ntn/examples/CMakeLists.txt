build_lib_example(
  NAME oran-ntn-full-scenario
  SOURCE_FILES oran-ntn-full-scenario.cc
  LIBRARIES_TO_LINK
    ${liboran-ntn}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libspectrum}
    ${libpropagation}
    ${libinternet}
    ${libapplications}
    ${libsatellite}
    ${libmmwave}
    ${liblte}
)
