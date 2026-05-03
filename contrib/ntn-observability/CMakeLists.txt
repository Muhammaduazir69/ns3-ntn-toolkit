set(source_files
    model/ntn-influx-sink.cc
    model/ntn-netsimulyzer-exporter.cc
    helper/ntn-observability-helper.cc
)

set(header_files
    model/ntn-metric-schema.h
    model/ntn-influx-sink.h
    model/ntn-netsimulyzer-exporter.h
    helper/ntn-observability-helper.h
)

set(test_sources
    test/ntn-observability-test-suite.cc
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
