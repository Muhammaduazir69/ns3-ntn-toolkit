build_lib_example(
  NAME ntn-cho-leo-basic
  SOURCE_FILES ntn-cho-leo-basic.cc
  LIBRARIES_TO_LINK
    ${libntn-cho}
    ${libcore}
    ${libnetwork}
    ${libmobility}
)

build_lib_example(
  NAME ntn-cho-full-constellation
  SOURCE_FILES ntn-cho-full-constellation.cc
  LIBRARIES_TO_LINK
    ${libntn-cho}
    ${libcore}
    ${libnetwork}
    ${libmobility}
)

build_lib_example(
  NAME ntn-realistic-mobility-demo
  SOURCE_FILES ntn-realistic-mobility-demo.cc
  LIBRARIES_TO_LINK
    ${libntn-cho}
    ${libcore}
)
