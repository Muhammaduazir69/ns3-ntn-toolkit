set(source_files
    model/ntn-timing-advance.cc
    model/ntn-sib19.cc
    model/ntn-ue-location-report.cc
    model/ntn-drx.cc
    helper/ntn-rrc-helper.cc
)

set(header_files
    model/ntn-rrc-types.h
    model/ntn-timing-advance.h
    model/ntn-sib19.h
    model/ntn-ue-location-report.h
    model/ntn-drx.h
    helper/ntn-rrc-helper.h
)

set(test_sources
    test/ntn-rrc-test-suite.cc
)

build_lib(
  LIBNAME ntn-rrc
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libmobility}
  TEST_SOURCES ${test_sources}
)
