set(source_files
    model/orbital-elements.cc
    model/sgp4-mobility-model.cc
    model/walker-constellation.cc
    model/contact-graph-scheduler.cc
)

set(header_files
    model/orbital-elements.h
    model/sgp4-mobility-model.h
    model/walker-constellation.h
    model/contact-graph-scheduler.h
)

set(test_sources
    test/ntn-constellation-test-suite.cc
)

build_lib(
  LIBNAME ntn-constellation
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libmobility}
  TEST_SOURCES ${test_sources}
)
