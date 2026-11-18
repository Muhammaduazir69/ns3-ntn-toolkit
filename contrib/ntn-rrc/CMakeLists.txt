set(source_files
    model/ntn-timing-advance.cc
    model/ntn-rach-window.cc
    model/ntn-meas-report.cc
    model/ntn-sib19.cc
    model/ntn-ue-location-report.cc
    model/ntn-drx.cc
    model/ntn-ul-timing-precompensator.cc
    model/ntn-ue-sib19-consumer.cc
    helper/ntn-rrc-helper.cc
)

set(header_files
    model/ntn-rrc-types.h
    model/ntn-timing-advance.h
    model/ntn-rach-window.h
    model/ntn-meas-report.h
    model/ntn-sib19.h
    model/ntn-ue-location-report.h
    model/ntn-drx.h
    model/ntn-ul-timing-precompensator.h
    model/ntn-ue-sib19-consumer.h
    helper/ntn-rrc-helper.h
)

set(test_sources
    test/ntn-rrc-test-suite.cc
    test/ntn-rrc-depth-test.cc
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

# The depth tests carry SIB19 over a point-to-point IP link. Only the test
# library links internet and point-to-point.
if(TARGET ntn-rrc-test)
  target_link_libraries(ntn-rrc-test ${libinternet} ${libpoint-to-point})
endif()
