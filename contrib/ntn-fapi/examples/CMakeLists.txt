# FAPI L1<->L2 data ABI exercised as a slot-by-slot DL transfer with CRC/HARQ.
build_lib_example(
  NAME ntn-fapi-dl-data-slotloop
  SOURCE_FILES ntn-fapi-dl-data-slotloop.cc
  LIBRARIES_TO_LINK
    ${libntn-fapi}
    ${libcore}
)

# Same FAPI L1<->L2 data path, but the per-slot SINR is derived from a real
# SGP4 LEO pass (ntn-constellation) instead of an analytical triangle.
build_lib_example(
  NAME ntn-fapi-leo-pass-slotloop
  SOURCE_FILES ntn-fapi-leo-pass-slotloop.cc
  LIBRARIES_TO_LINK
    ${libntn-fapi}
    ${libntn-constellation}
    ${libcore}
    ${libmobility}
)
