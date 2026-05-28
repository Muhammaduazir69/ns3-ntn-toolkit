# FAPI L1<->L2 data ABI exercised as a slot-by-slot DL transfer with CRC/HARQ.
build_lib_example(
  NAME ntn-fapi-dl-data-slotloop
  SOURCE_FILES ntn-fapi-dl-data-slotloop.cc
  LIBRARIES_TO_LINK
    ${libntn-fapi}
    ${libcore}
)
