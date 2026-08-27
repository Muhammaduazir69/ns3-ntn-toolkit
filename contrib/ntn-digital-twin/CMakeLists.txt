# TWIN-02: ntn-digital-twin had NO CMakeLists.txt, so it was not an ns-3 module
# and its tests never ran under ./test.py. The "twin/sim agreement" gate was
# therefore Python-only and never invoked the simulator it claimed to agree
# with. This makes the C++ half buildable and testable.
set(source_files
    model/ntn-twin-agreement.cc
)

set(header_files
    model/ntn-twin-agreement.h
)

set(test_sources
    test/ntn-digital-twin-test-suite.cc
)

build_lib(
  LIBNAME ntn-digital-twin
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
  LIBRARIES_TO_LINK
    ${libcore}
    ${libmobility}
    ${libntn-constellation}
  TEST_SOURCES ${test_sources}
)
