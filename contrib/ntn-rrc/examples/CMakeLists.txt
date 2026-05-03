build_lib_example(
  NAME ntn-rrc-leo-pass
  SOURCE_FILES ntn-rrc-leo-pass.cc
  LIBRARIES_TO_LINK
    ${libntn-rrc}
    ${libcore}
    ${libmobility}
)
