# OBS-07: stamp the build's own provenance into a generated header, so the
# reproducibility manifest records what actually produced a result instead of
# an empty string and a hardcoded version literal.
find_package(Git QUIET)
set(NTN_TOOLKIT_GIT_SHA "unknown")
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE NTN_GIT_SHA_RAW
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  if(NTN_GIT_SHA_RAW)
    set(NTN_TOOLKIT_GIT_SHA "${NTN_GIT_SHA_RAW}")
    execute_process(
      COMMAND ${GIT_EXECUTABLE} status --porcelain
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      OUTPUT_VARIABLE NTN_GIT_DIRTY
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET)
    if(NTN_GIT_DIRTY)
      set(NTN_TOOLKIT_GIT_SHA "${NTN_TOOLKIT_GIT_SHA}-dirty")
    endif()
  endif()
endif()
set(NTN_NS3_VERSION "unknown")
if(EXISTS "${PROJECT_SOURCE_DIR}/VERSION")
  file(READ "${PROJECT_SOURCE_DIR}/VERSION" NTN_NS3_VERSION_RAW)
  string(STRIP "${NTN_NS3_VERSION_RAW}" NTN_NS3_VERSION)
endif()
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/model/ntn-build-provenance.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/ntn-build-provenance.h
  @ONLY)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(source_files
    model/ntn-influx-sink.cc
    model/ntn-netsimulyzer-exporter.cc
    model/ntn-repro-manifest.cc
    # The scene-trace spine: one tap on real mobility + measured KPIs, fanned
    # out to NetSimulyzer JSON + Cesium CZML (+ Influx, planned).
    model/ntn-scene-recorder.cc
    helper/ntn-observability-helper.cc
    helper/ntn-scene-helper.cc
)

set(header_files
    model/ntn-metric-schema.h
    model/ntn-influx-sink.h
    model/ntn-netsimulyzer-exporter.h
    model/ntn-repro-manifest.h
    model/ntn-dataset-writer.h
    model/ntn-scene-recorder.h
    helper/ntn-observability-helper.h
    helper/ntn-scene-helper.h
)

set(test_sources
    test/ntn-observability-test-suite.cc
    test/ntn-observability-indepth-test.cc
)

build_lib(
  LIBNAME ntn-observability
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libmobility}
  TEST_SOURCES ${test_sources}
)

# The in-depth suite (ntn-observability-indepth) feeds FlowMonitor KPIs from a
# real IPv4/P2P data plane into the sinks and tracks SGP4 satellites, so its
# test library links those stacks and ntn-constellation. The module library
# keeps its own small dependency set.
if(TARGET ntn-observability-test)
  target_link_libraries(
    ntn-observability-test
    ${libinternet}
    ${libpoint-to-point}
    ${libapplications}
    ${libflow-monitor}
    ntn-constellation
  )
endif()
