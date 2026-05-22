set(source_files
    bridge/sionna-transport.cc
    bridge/sionna-udp-transport.cc
    bridge/sionna-pybind-transport.cc
    bridge/ns3-sionna-channel.cc
)

set(header_files
    bridge/sionna-transport.h
    bridge/sionna-udp-transport.h
    bridge/sionna-pybind-transport.h
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
