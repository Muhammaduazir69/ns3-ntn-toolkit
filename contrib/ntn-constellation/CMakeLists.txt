set(source_files
    model/orbital-elements.cc
    model/sgp4-mobility-model.cc
    model/walker-constellation.cc
    model/contact-graph-scheduler.cc
    # 4.4.4 — Contact-graph router integration (Roadmap §4.4 deep work)
    model/contact-graph-router.cc
    # 4.4.11 — TR 38.821 + Starlink calibration corpus (Roadmap §4.4.11)
    model/tr38821-corpus.cc
)

set(header_files
    model/orbital-elements.h
    model/sgp4-mobility-model.h
    model/walker-constellation.h
    model/contact-graph-scheduler.h
    # 4.4.4 — Contact-graph router integration (Roadmap §4.4 deep work)
    model/contact-graph-router.h
    # 4.4.11 — TR 38.821 + Starlink calibration corpus (Roadmap §4.4.11)
    model/tr38821-corpus.h
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
