set(source_files
    model/haps-mobility-model.cc
    model/uav-mobility-models.cc
    model/a2g-channel-tr36777.cc
    model/sagin-a2g-propagation-loss-model.cc
    model/multi-layer-router.cc
    model/aeronautical-scenario.cc
    # 4.4.1 — OpenSky ADS-B trace importer + mobility model (Roadmap §4.4.1)
    model/opensky-adsb-trace.cc
    model/opensky-mobility-model.cc
    # 4.4.2 — AIS maritime trace importer + mobility model (Roadmap §4.4.2)
    model/ais-maritime-trace.cc
    model/ais-mobility-model.cc
    # 4.4.3 — HST high-speed train (TR 38.901 §7.5) mobility (Roadmap §4.4.3)
    model/hst-trace.cc
    model/hst-mobility-model.cc
    # 4.4.7 — cross-layer slice-aware routing (Roadmap §4.4.7)
    model/sagin-slice-router.cc
    # 4.4.8 — HAPS trajectory CSV ingest (Roadmap §4.4.8)
    model/haps-trajectory-trace.cc
    model/haps-trajectory-mobility-model.cc
    helper/sagin-helper.cc
)

set(header_files
    model/haps-mobility-model.h
    model/uav-mobility-models.h
    model/a2g-channel-tr36777.h
    model/sagin-a2g-propagation-loss-model.h
    model/multi-layer-router.h
    model/aeronautical-scenario.h
    # 4.4.1 — OpenSky ADS-B trace importer + mobility model (Roadmap §4.4.1)
    model/opensky-adsb-trace.h
    model/opensky-mobility-model.h
    # 4.4.2 — AIS maritime trace importer + mobility model (Roadmap §4.4.2)
    model/ais-maritime-trace.h
    model/ais-mobility-model.h
    # 4.4.3 — HST high-speed train (TR 38.901 §7.5) mobility (Roadmap §4.4.3)
    model/hst-trace.h
    model/hst-mobility-model.h
    # 4.4.7 — cross-layer slice-aware routing (Roadmap §4.4.7)
    model/sagin-slice-router.h
    # 4.4.8 — HAPS trajectory CSV ingest (Roadmap §4.4.8)
    model/haps-trajectory-trace.h
    model/haps-trajectory-mobility-model.h
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
    ${libntn-slice}
    # point-to-point + internet + applications are needed by the router
    # data-plane-actuation test (RouterGatesDataPlaneTest), which builds a
    # real IPv4 forwarding topology and gates it from the route decision.
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
  TEST_SOURCES ${test_sources}
)
