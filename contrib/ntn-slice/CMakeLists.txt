set(source_files
    model/ntn-slice-selector.cc
    model/slice-orchestrator-xapp.cc
    model/slice-isolation-monitor.cc
    helper/ntn-slice-helper.cc
)

set(header_files
    model/ntn-slice-types.h
    model/ntn-slice-selector.h
    model/slice-orchestrator-xapp.h
    model/slice-isolation-monitor.h
    helper/ntn-slice-helper.h
)

set(test_sources
    test/ntn-slice-test-suite.cc
)

build_lib(
  LIBNAME ntn-slice
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
  LIBRARIES_TO_LINK
    ${libcore}
  TEST_SOURCES ${test_sources}
)
