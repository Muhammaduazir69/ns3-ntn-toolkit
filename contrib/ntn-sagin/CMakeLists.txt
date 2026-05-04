set(source_files
    model/haps-mobility-model.cc
    model/uav-mobility-models.cc
    model/a2g-channel-tr36777.cc
    model/multi-layer-router.cc
    model/aeronautical-scenario.cc
    helper/sagin-helper.cc
)

set(header_files
    model/haps-mobility-model.h
    model/uav-mobility-models.h
    model/a2g-channel-tr36777.h
    model/multi-layer-router.h
    model/aeronautical-scenario.h
    helper/sagin-helper.h
)

set(test_sources
    test/ntn-sagin-test-suite.cc
)

build_lib(
  LIBNAME ntn-sagin
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpropagation}
  TEST_SOURCES ${test_sources}
)
