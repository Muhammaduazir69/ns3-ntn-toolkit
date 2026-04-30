set(source_files
    model/ntn-orbit-predictor.cc
    model/ntn-tte-estimator.cc
    model/ntn-cho-algorithm.cc
    model/ntn-measurement-model.cc
    model/ntn-ai-interface.cc
    helper/ntn-cho-helper.cc
    helper/ntn-realistic-mobility.cc
)

set(header_files
    model/ntn-orbit-predictor.h
    model/ntn-tte-estimator.h
    model/ntn-cho-algorithm.h
    model/ntn-measurement-model.h
    model/ntn-ai-interface.h
    helper/ntn-cho-helper.h
    helper/ntn-realistic-mobility.h
)

set(test_sources
    test/ntn-cho-test-suite.cc
)

build_lib(
  LIBNAME ntn-cho
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpropagation}
    ${libspectrum}
    ${liblte}
    ${libmmwave}
    ${libsatellite}
    ${libinternet}
    ${libapplications}
    ${libpoint-to-point}
    ${libns3-ai-ntn}
  TEST_SOURCES ${test_sources}
)
