build_lib_example(
  NAME ntn-three-slice-leo-geo
  SOURCE_FILES ntn-three-slice-leo-geo.cc
  LIBRARIES_TO_LINK
    ${libntn-slice}
    ${libcore}
    ${libnetwork}
)
