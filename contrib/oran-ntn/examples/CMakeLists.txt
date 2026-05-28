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
    ${liblte}
)

# Closed RIC control loop: E2-KPM -> mMIMO precoder xApp -> beam -> real link.
build_lib_example(
  NAME oran-ntn-ric-controlled-traffic
  SOURCE_FILES oran-ntn-ric-controlled-traffic.cc
  LIBRARIES_TO_LINK
    ${liboran-ntn}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)
