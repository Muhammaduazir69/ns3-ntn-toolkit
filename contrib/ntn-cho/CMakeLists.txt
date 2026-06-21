set(source_files
    model/ntn-orbit-predictor.cc
    model/ntn-tte-estimator.cc
    model/ntn-cho-algorithm.cc
    model/ntn-measurement-model.cc
    model/ntn-ai-interface.cc
    # Real 3GPP TR 38.811 UE-class mobility as an ns-3 MobilityModel (ECEF,
    # SGP4-compatible) -- the toolkit's core NTN UE mobility architecture.
    model/ntn-tr38811-mobility-model.cc
    # Ground terminal anchored at a true WGS84 geodetic location (ECEF),
    # for globe-correct placement and physically-anchored sat geometry.
    model/ntn-geodetic-fixed-mobility-model.cc
    helper/ntn-cho-helper.cc
    helper/ntn-realistic-mobility.cc
)

set(header_files
    model/ntn-orbit-predictor.h
    model/ntn-tte-estimator.h
    model/ntn-cho-algorithm.h
    model/ntn-measurement-model.h
    model/ntn-ai-interface.h
    model/ntn-tr38811-mobility-model.h
    model/ntn-geodetic-fixed-mobility-model.h
    helper/ntn-cho-helper.h
    helper/ntn-realistic-mobility.h
)

set(test_sources
    test/ntn-cho-test-suite.cc
    test/ntn-standards-validation-test.cc
)

# ns3-ai is an OPTIONAL dependency: only NtnAiInterface (the learning-based
# HO path) uses it. Detect whichever ns3-ai target is present — the upstream
# module (libns3-ai) or the ns3-ntn-toolkit fork (libns3-ai-ntn) — and only
# then link it and define NTN_CHO_HAS_NS3AI so the bridge is compiled in.
# Without it, the C++ CHO triggers (a3 / location / time / tte-aware) build
# and run unchanged.
set(ntn_cho_optional_libs "")
if(TARGET ${libns3-ai})
  set(ntn_cho_optional_libs ${libns3-ai})
elseif(TARGET ${libns3-ai-ntn})
  set(ntn_cho_optional_libs ${libns3-ai-ntn})
endif()

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
    ${libsatellite}
    ${libinternet}
    ${libapplications}
    ${libpoint-to-point}
    ${ntn_cho_optional_libs}
  TEST_SOURCES ${test_sources}
)

if(ntn_cho_optional_libs)
  target_compile_definitions(${libntn-cho} PRIVATE NTN_CHO_HAS_NS3AI)
endif()
