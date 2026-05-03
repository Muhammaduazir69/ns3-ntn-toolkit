build_lib_example(
  NAME ntn-observability-demo
  SOURCE_FILES ntn-observability-demo.cc
  LIBRARIES_TO_LINK
    ${libntn-observability}
    ${libntn-rrc}
    ${libcore}
    ${libmobility}
)
