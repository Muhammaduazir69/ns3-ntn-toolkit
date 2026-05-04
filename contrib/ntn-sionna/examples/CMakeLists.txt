build_lib_example(
  NAME leo-pass-sionna-vs-tr38811
  SOURCE_FILES leo-pass-sionna-vs-tr38811.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
    ${libcore}
    ${libmobility}
    ${libpropagation}
)
