build_lib_example(
  NAME thz-ntn-leo-ground
  SOURCE_FILES thz-ntn-leo-ground.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libntn-traffic}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
)

build_lib_example(
  NAME thz-ntn-isl
  SOURCE_FILES thz-ntn-isl.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libntn-traffic}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
)

build_lib_example(
  NAME thz-ntn-beam-tracking
  SOURCE_FILES thz-ntn-beam-tracking.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libntn-traffic}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
)

build_lib_example(
  NAME thz-ntn-um-mimo
  SOURCE_FILES thz-ntn-um-mimo.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libcore}
    ${libmobility}
)

build_lib_example(
  NAME thz-ntn-ris-assisted
  SOURCE_FILES thz-ntn-ris-assisted.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libcore}
    ${libmobility}
)

# TODO Roadmap §4.3.5: ISAC scheduler is being redesigned; thz-ntn-isac.cc
# references the legacy ThzNtnIsac API (CreateIsac/PerformSensing/...). The
# example will be rewritten alongside the Q4 2026 ISAC scheduler work. Until
# then leave the example out of the build so test-runner can link.
if(FALSE)
build_lib_example(
  NAME thz-ntn-isac
  SOURCE_FILES thz-ntn-isac.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libcore}
    ${libmobility}
)
endif()

build_lib_example(
  NAME thz-ntn-dband-constellation
  SOURCE_FILES thz-ntn-dband-constellation.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libcore}
    ${libmobility}
)

build_lib_example(
  NAME thz-ntn-full-stack
  SOURCE_FILES thz-ntn-full-stack.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libcore}
    ${libmobility}
)
