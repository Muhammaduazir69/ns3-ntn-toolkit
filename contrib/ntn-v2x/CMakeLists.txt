set(source_files
    model/sumo-traci-bridge.cc
    model/ntn-v2x-bsm-header.cc
    model/ntn-nr-sidelink.cc
    model/v2x-leo-direct.cc
    model/v2x-leo-relay.cc
    model/maritime-scenario.cc
    helper/ntn-v2x-helper.cc
)

set(header_files
    model/sumo-traci-bridge.h
    model/ntn-v2x-bsm-header.h
    model/ntn-nr-sidelink.h
    model/v2x-leo-direct.h
    model/v2x-leo-relay.h
    model/maritime-scenario.h
    helper/ntn-v2x-helper.h
)

set(test_sources
    test/ntn-v2x-test-suite.cc
    test/ntn-v2x-indepth-test.cc
)

build_lib(
  LIBNAME ntn-v2x
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libmobility}
  TEST_SOURCES ${test_sources}
)

# The in-depth suite (ntn-v2x-indepth) runs the PC5 sidelink over a real IPv4
# satellite hop timed by an SGP4 orbit, so its test library also needs the
# internet, point-to-point and flow-monitor stacks, the SGP4 propagator
# (ntn-constellation) and the ENU projection (ntn-cho). Only the test library
# links them; the module library keeps its own small dependency set.
if(TARGET ntn-v2x-test)
  target_link_libraries(
    ntn-v2x-test
    ${libinternet}
    ${libpoint-to-point}
    ${libflow-monitor}
    ntn-constellation
    ntn-cho
  )
endif()
