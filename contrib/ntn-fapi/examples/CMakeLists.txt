# FAPI SCF-222 L1<->L2 data ABI exercised slot-by-slot over a REAL mmwave NR NTN
# cell; the per-slot CRC.indication is decided by the MEASURED PHY SINR/TBLER.
set(_fapi_real_libs
  ${libntn-fapi}
  ${libntn-traffic}
  ${libntn-constellation}
  ${libntn-cho}
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

build_lib_example(
  NAME ntn-fapi-dl-data-slotloop
  SOURCE_FILES ntn-fapi-dl-data-slotloop.cc
  LIBRARIES_TO_LINK ${_fapi_real_libs}
)

# Same FAPI data path, but driven by a real SGP4 LEO pass (ntn-constellation).
build_lib_example(
  NAME ntn-fapi-leo-pass-slotloop
  SOURCE_FILES ntn-fapi-leo-pass-slotloop.cc
  LIBRARIES_TO_LINK ${_fapi_real_libs}
)

# Real-stack flagship: FAPI L1 CRC.indication decided by MEASURED SINR/TBLER.
build_lib_example(
  NAME ntn-fapi-real-stack
  SOURCE_FILES ntn-fapi-real-stack.cc
  LIBRARIES_TO_LINK ${_fapi_real_libs}
)
