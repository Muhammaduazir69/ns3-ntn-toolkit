set(source_files
    model/fapi-helpers.cc
)

set(header_files
    model/fapi-common.h
    model/fapi-pdu-types.h
    model/fapi-messages.h
    model/fapi-helpers.h
)

set(test_sources
    test/ntn-fapi-test-suite.cc
)

build_lib(
    LIBNAME ntn-fapi
    SOURCE_FILES ${source_files}
    HEADER_FILES ${header_files}
    LIBRARIES_TO_LINK
        ${libcore}
    TEST_SOURCES ${test_sources}
)
