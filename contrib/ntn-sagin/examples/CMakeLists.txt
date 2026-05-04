build_lib_example(
  NAME sagin-haps-leo-relay
  SOURCE_FILES sagin-haps-leo-relay.cc
  LIBRARIES_TO_LINK
    ${libntn-sagin}
    ${libcore}
    ${libmobility}
)

build_lib_example(
  NAME sagin-uav-swarm
  SOURCE_FILES sagin-uav-swarm.cc
  LIBRARIES_TO_LINK
    ${libntn-sagin}
    ${libcore}
    ${libmobility}
)

build_lib_example(
  NAME sagin-aeronautical
  SOURCE_FILES sagin-aeronautical.cc
  LIBRARIES_TO_LINK
    ${libntn-sagin}
    ${libcore}
    ${libmobility}
)
