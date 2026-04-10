set(source_files
    helper/stats-delay-helper.cc
    helper/stats-helper.cc
    helper/stats-throughput-helper.cc
    model/collector-map.cc
    model/distribution-collector.cc
    model/interval-rate-collector.cc
    model/address-boolean-probe.cc
    model/address-double-probe.cc
    model/address-tag.cc
    model/address-time-probe.cc
    model/address-uinteger-probe.cc
    model/application-delay-probe.cc
    model/bytes-probe.cc
    model/magister-gnuplot-aggregator.cc
    model/multi-file-aggregator.cc
    model/scalar-collector.cc
    model/time-tag.cc
    model/uinteger-32-single-probe.cc
    model/unit-conversion-collector.cc
)

set(header_files
    helper/stats-delay-helper.h
    helper/stats-helper.h
    helper/stats-throughput-helper.h
    model/collector-map.h
    model/distribution-collector.h
    model/interval-rate-collector.h
    model/address-boolean-probe.h
    model/address-double-probe.h
    model/address-tag.h
    model/address-time-probe.h
    model/address-uinteger-probe.h
    model/application-delay-probe.h
    model/bytes-probe.h
    model/magister-gnuplot-aggregator.h
    model/magister-stats.h
    model/multi-file-aggregator.h
    model/scalar-collector.h
    model/time-tag.h
    model/uinteger-32-single-probe.h
    model/unit-conversion-collector.h
)

set(test_sources
    test/distribution-collector-test-suite.cc
)

build_lib(
  LIBNAME magister-stats
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libstats}
    ${libinternet}
    ${libapplications}
  TEST_SOURCES ${test_sources}
)
