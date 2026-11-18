set(source_files
    model/ntn-slice-selector.cc
    model/slice-orchestrator-xapp.cc
    model/slice-isolation-monitor.cc
    helper/ntn-slice-helper.cc
    model/ntn-slice-quota-scheduler.cc
)

set(header_files
    model/ntn-slice-types.h
    model/ntn-slice-selector.h
    model/slice-orchestrator-xapp.h
    model/slice-isolation-monitor.h
    helper/ntn-slice-helper.h
    model/ntn-slice-quota-scheduler.h
    model/ntn-slice-contract-trim.h
)

set(test_sources
    test/ntn-slice-test-suite.cc
    test/ntn-slice-quota-scheduler-test.cc
    test/ntn-slice-cell-realism-test.cc
    test/ntn-slice-depth-test.cc
)

build_lib(
  LIBNAME ntn-slice
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnr}
    ${libntn-traffic}
    ${libntn-constellation}
    ${libmobility}
    ${libnetwork}
  TEST_SOURCES ${test_sources}
)
