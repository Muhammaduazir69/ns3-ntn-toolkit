set(source_files
    model/fapi-helpers.cc
    model/ntn-fapi-sap-bridge.cc
)

set(header_files
    model/fapi-common.h
    model/fapi-pdu-types.h
    model/fapi-messages.h
    model/fapi-helpers.h
    model/ntn-fapi-sap-bridge.h
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
        ${libnetwork}
        ${libspectrum}
        ${liblte}
        ${libmmwave}
        # ntn-traffic supplies NtnRealStackHelper, used by the real-stack test
        # to stand up a live mmwave cell the FAPI SAP bridge decorates.
        ${libntn-traffic}
    TEST_SOURCES ${test_sources}
)
