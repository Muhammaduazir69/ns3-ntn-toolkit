build_lib_example(
  NAME oran-ntn-full-scenario
  SOURCE_FILES oran-ntn-full-scenario.cc
  LIBRARIES_TO_LINK
    ${liboran-ntn}
    ${libntn-traffic}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libspectrum}
    ${libpropagation}
    ${libinternet}
    ${libapplications}
    ${libpoint-to-point}
    ${libsatellite}
    ${libmmwave}
)

# Phase-1 real-stack flagship: KPM from MEASURED mmwave PHY SINR drives the RIC.
build_lib_example(
  NAME oran-ntn-real-stack-scenario
  SOURCE_FILES oran-ntn-real-stack-scenario.cc
  LIBRARIES_TO_LINK
    ${liboran-ntn}
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

# Closed RIC control loop: E2-KPM -> mMIMO precoder xApp -> beam -> real link.
build_lib_example(
  NAME oran-ntn-ric-controlled-traffic
  SOURCE_FILES oran-ntn-ric-controlled-traffic.cc
  LIBRARIES_TO_LINK
    ${liboran-ntn}
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

# Phase-3 composition: one shared mmwave cell feeds RIC + slice + observability.
build_lib_example(
  NAME ntn-e2e-full-stack
  SOURCE_FILES ntn-e2e-full-stack.cc
  LIBRARIES_TO_LINK
    ${liboran-ntn}
    ${libntn-slice}
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
