# TWIN-02: ntn-digital-twin had NO CMakeLists.txt, so it was not an ns-3 module
# and its tests never ran under ./test.py. The "twin/sim agreement" gate was
# therefore Python-only and never invoked the simulator it claimed to agree
# with. This makes the C++ half buildable and testable.
set(source_files
    model/ntn-twin-agreement.cc
    model/ntn-twin-replay.cc
)

set(header_files
    model/ntn-twin-agreement.h
    model/ntn-twin-replay.h
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

# The cross-module case drives a real NR handover (ntn-traffic) from the twin's
# predictions through the O-RAN consumer (oran-ntn). Only the test library links
# them; the twin library keeps its own small dependency set.
if(TARGET ntn-digital-twin-test)
  target_link_libraries(ntn-digital-twin-test ntn-traffic oran-ntn)
endif()
