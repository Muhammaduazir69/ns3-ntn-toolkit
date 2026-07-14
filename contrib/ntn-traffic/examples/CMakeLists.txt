set(base_examples
    nrtv-p2p-example
    nrtv-variables-plot
)

foreach(
  example
  ${base_examples}
)
  build_lib_example(
    NAME ${example}
    SOURCE_FILES ${example}.cc
    LIBRARIES_TO_LINK
    ${libntn-traffic}
    ${libntn-cho}
    ${libntn-constellation}
    ${libapplications}
    ${libpoint-to-point}
    ${libinternet}
    ${libnetwork}
  )
endforeach()

build_lib_example(
  NAME ntn-cbr-leo-link
  SOURCE_FILES ntn-cbr-leo-link.cc
  LIBRARIES_TO_LINK
    ${libntn-traffic}
    ${libapplications}
    ${libpoint-to-point}
    ${libinternet}
    ${libnetwork}
    ${libflow-monitor}
    ${libcore}
)

build_lib_example(
  NAME ntn-real-stack-smoke
  SOURCE_FILES ntn-real-stack-smoke.cc
  LIBRARIES_TO_LINK
    ${libntn-traffic}
    ${libntn-cho}
    ${libntn-constellation}
    ${libmmwave}
    ${liblte}
    ${libmobility}
    ${libinternet}
    ${libflow-monitor}
    ${libcore}
)

build_lib_example(
  NAME ntn-nr-fr1-demo
  SOURCE_FILES ntn-nr-fr1-demo.cc
  LIBRARIES_TO_LINK
    ${libntn-traffic}
    ${libntn-constellation}
    ${libntn-cho}
    ${libnr}
    ${liblte}
    ${libmobility}
    ${libinternet}
    ${libflow-monitor}
    ${libnetwork}
    ${libcore}
)

build_lib_example(
  NAME ntn-nr-deep-integration-demo
  SOURCE_FILES ntn-nr-deep-integration-demo.cc
  LIBRARIES_TO_LINK
    ${libntn-traffic}
    ${libntn-constellation}
    ${libntn-cho}
    ${libnr}
    ${liblte}
    ${libmobility}
    ${libinternet}
    ${libflow-monitor}
    ${libnetwork}
    ${libcore}
)

build_lib_example(
  NAME ntn-nr-handover-pass
  SOURCE_FILES ntn-nr-handover-pass.cc
  LIBRARIES_TO_LINK
    ${libntn-traffic}
    ${libntn-constellation}
    ${libntn-cho}
    ${libnr}
    ${liblte}
    ${libmobility}
    ${libinternet}
    ${libflow-monitor}
    ${libnetwork}
    ${libcore}
)

build_lib_example(
  NAME ntn-oran-qos-flows
  SOURCE_FILES ntn-oran-qos-flows.cc
  LIBRARIES_TO_LINK
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
    ${libpoint-to-point}
    ${libmmwave}
    ${liblte}
)

build_lib_example(
  NAME ntn-tr38821-calibration
  SOURCE_FILES ntn-tr38821-calibration.cc
  LIBRARIES_TO_LINK
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
