set(source_files
    helper/cbr-helper.cc
    helper/client-rx-trace-plot.cc
    helper/nrtv-helper.cc
    helper/three-gpp-http-satellite-helper.cc
    model/cbr-application.cc
    model/nrtv-header.cc
    model/nrtv-tcp-client.cc
    model/nrtv-tcp-server.cc
    model/nrtv-udp-server.cc
    model/nrtv-variables.cc
    model/nrtv-video-worker.cc
    model/traffic-time-tag.cc
    model/three-gpp-http-satellite-client.cc
    stats/application-stats-helper.cc
    stats/application-stats-delay-helper.cc
    stats/application-stats-throughput-helper.cc
    stats/application-stats-helper-container.cc
)

set(header_files
    helper/cbr-helper.h
    helper/client-rx-trace-plot.h
    helper/histogram-plot-helper.h
    helper/nrtv-helper.h
    helper/three-gpp-http-satellite-helper.h
    model/traffic.h
    model/cbr-application.h
    model/nrtv-header.h
    model/nrtv-tcp-client.h
    model/nrtv-tcp-server.h
    model/nrtv-udp-server.h
    model/nrtv-variables.h
    model/nrtv-video-worker.h
    model/traffic-time-tag.h
    model/three-gpp-http-satellite-client.h
    stats/application-stats-helper.h
    stats/application-stats-delay-helper.h
    stats/application-stats-throughput-helper.h
    stats/application-stats-helper-container.h
)

set(test_sources
    test/cbr-test.cc
    test/nrtv-test.cc
)

build_lib(
  LIBNAME traffic
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
  LIBRARIES_TO_LINK
    ${libcore}
    ${libapplications}
    ${libflow-monitor}
    ${libmagister-stats}
    ${libpoint-to-point}
  TEST_SOURCES ${test_sources}
)
