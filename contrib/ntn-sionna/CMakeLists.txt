set(source_files
    bridge/ns3-sionna-channel.cc
)

set(header_files
    bridge/ns3-sionna-channel.h
)

set(test_sources
    test/ntn-sionna-test-suite.cc
)

build_lib(
  LIBNAME ntn-sionna
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpropagation}
  TEST_SOURCES ${test_sources}
)
