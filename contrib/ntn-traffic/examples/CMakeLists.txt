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
