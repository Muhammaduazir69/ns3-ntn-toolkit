build_lib_example(
  NAME ntn-v2x-rural-highway
  SOURCE_FILES ntn-v2x-rural-highway.cc
  LIBRARIES_TO_LINK
    ${libntn-v2x}
    ${libcore}
    ${libmobility}
    ${libntn-traffic}
    ${libnetwork}
    ${libinternet}
    ${libapplications}
    ${libpoint-to-point}
)
