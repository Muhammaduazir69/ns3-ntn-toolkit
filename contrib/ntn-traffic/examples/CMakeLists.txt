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
