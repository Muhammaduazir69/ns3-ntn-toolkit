set(source_files
    # Core O-RAN framework
    model/oran-ntn-e2-interface.cc
    model/oran-ntn-a1-interface.cc
    model/oran-ntn-conflict-manager.cc
    model/oran-ntn-xapp-base.cc
    model/oran-ntn-near-rt-ric.cc
    model/oran-ntn-space-ric.cc
    model/oran-ntn-sat-bridge.cc
    # T8 — canonical WG3 KPM metric-ID alignment (Roadmap §3 T8)
    model/oran-ntn-kpm-canonical-ids.cc
    # 4.1.3 — E2SM-RC v1.03 Style 3 Connected-Mode Mobility (Roadmap §4.1.3)
    model/oran-ntn-rc-style3.cc
    # 4.1.4 — NIST-pattern SQLite data repository (Roadmap §4.1.4)
    model/oran-ntn-data-repository.cc
    # 4.1.11 — OSC-aligned A1 policy schema registry (Roadmap §4.1.11)
    model/oran-ntn-a1-policy-schema.cc
    # T4 — Service-Model plugin ABI + first two concrete SMs (Roadmap §3 T4)
    model/oran-ntn-service-model.cc
    model/oran-ntn-service-model-kpm.cc
    model/oran-ntn-service-model-rc.cc
    # 4.1.6 — E2SM-CCC v1.00 SM plugin (Roadmap §4.1.6)
    model/oran-ntn-service-model-ccc.cc
    # T2 — ASN.1-PER codec for E2AP / E2SM-KPM / E2SM-RC (Roadmap §3 T2)
    asn1/asn1-per-codec.cc
    # T3 — SCTP / TCP E2 listener for FlexRIC bridge (Roadmap §3 T3)
    flexric-bridge/e2-transport.cc
    flexric-bridge/e2-listener.cc
    # Original xApps
    model/oran-ntn-xapp-ho-predict.cc
    model/oran-ntn-xapp-beam-hop.cc
    model/oran-ntn-xapp-slice-manager.cc
    model/oran-ntn-xapp-doppler-comp.cc
    model/oran-ntn-xapp-tn-ntn-steering.cc
    # Phase 2: mmWave NTN physical layer
    model/oran-ntn-mmwave-beamforming.cc
    model/oran-ntn-channel-model.cc
    model/oran-ntn-ntn-scheduler.cc
    model/oran-ntn-phy-kpm-extractor.cc
    model/oran-ntn-dual-connectivity.cc
    # Phase 3: AI/ML integration via ns3-ai
    model/oran-ntn-gym-handover.cc
    model/oran-ntn-gym-beam-hop.cc
    model/oran-ntn-gym-slice.cc
    model/oran-ntn-gym-steering.cc
    model/oran-ntn-gym-predictive.cc
    model/oran-ntn-federated-learning.cc
    # Phase 4: Advanced xApps
    model/oran-ntn-xapp-interference-mgmt.cc
    model/oran-ntn-xapp-energy-harvest.cc
    model/oran-ntn-xapp-predictive-alloc.cc
    model/oran-ntn-xapp-multi-conn.cc
    # Phase 5: Enhanced Space RIC
    model/oran-ntn-space-ric-inference.cc
    model/oran-ntn-isl-header.cc
    # Phase 5: THz-NTN xApps
    model/oran-ntn-xapp-thz-beam-mgmt.cc
    model/oran-ntn-xapp-thz-spectrum.cc
    model/oran-ntn-xapp-thz-ris.cc
    model/oran-ntn-xapp-isac.cc
    # Helper
    helper/oran-ntn-helper.cc
)

set(header_files
    # Core O-RAN framework
    model/oran-ntn-types.h
    model/oran-ntn-e2-interface.h
    model/oran-ntn-a1-interface.h
    model/oran-ntn-conflict-manager.h
    model/oran-ntn-xapp-base.h
    model/oran-ntn-near-rt-ric.h
    model/oran-ntn-space-ric.h
    model/oran-ntn-sat-bridge.h
    model/oran-ntn-kpm-canonical-ids.h
    # 4.1.1 — FlexRIC field-name parity (Roadmap §4.1.1)
    model/oran-ntn-flexric-types.h
    # 4.1.3 — E2SM-RC v1.03 Style 3 Connected-Mode Mobility (Roadmap §4.1.3)
    model/oran-ntn-rc-style3.h
    # 4.1.4 — NIST-pattern SQLite data repository (Roadmap §4.1.4)
    model/oran-ntn-data-repository.h
    # 4.1.11 — OSC-aligned A1 policy schema registry (Roadmap §4.1.11)
    model/oran-ntn-a1-policy-schema.h
    # T4 — Service-Model plugin ABI + first two concrete SMs (Roadmap §3 T4)
    model/oran-ntn-service-model.h
    model/oran-ntn-service-model-kpm.h
    model/oran-ntn-service-model-rc.h
    # 4.1.6 — E2SM-CCC v1.00 SM plugin (Roadmap §4.1.6)
    model/oran-ntn-service-model-ccc.h
    # T2 — ASN.1-PER codec for E2AP / E2SM-KPM / E2SM-RC (Roadmap §3 T2)
    asn1/asn1-per-codec.h
    # T3 — SCTP / TCP E2 listener for FlexRIC bridge (Roadmap §3 T3)
    flexric-bridge/e2-message-types.h
    flexric-bridge/e2-transport.h
    flexric-bridge/e2-listener.h
    # Original xApps
    model/oran-ntn-xapp-ho-predict.h
    model/oran-ntn-xapp-beam-hop.h
    model/oran-ntn-xapp-slice-manager.h
    model/oran-ntn-xapp-doppler-comp.h
    model/oran-ntn-xapp-tn-ntn-steering.h
    # Phase 2: mmWave NTN physical layer
    model/oran-ntn-mmwave-beamforming.h
    model/oran-ntn-channel-model.h
    model/oran-ntn-ntn-scheduler.h
    model/oran-ntn-phy-kpm-extractor.h
    model/oran-ntn-dual-connectivity.h
    # Phase 3: AI/ML integration
    model/oran-ntn-gym-handover.h
    model/oran-ntn-gym-beam-hop.h
    model/oran-ntn-gym-slice.h
    model/oran-ntn-gym-steering.h
    model/oran-ntn-gym-predictive.h
    model/oran-ntn-msg-interface.h
    model/oran-ntn-federated-learning.h
    # Phase 4: Advanced xApps
    model/oran-ntn-xapp-interference-mgmt.h
    model/oran-ntn-xapp-energy-harvest.h
    model/oran-ntn-xapp-predictive-alloc.h
    model/oran-ntn-xapp-multi-conn.h
    # Phase 5: Enhanced Space RIC
    model/oran-ntn-space-ric-inference.h
    model/oran-ntn-isl-header.h
    # Phase 5: THz-NTN xApps
    model/oran-ntn-xapp-thz-beam-mgmt.h
    model/oran-ntn-xapp-thz-spectrum.h
    model/oran-ntn-xapp-thz-ris.h
    model/oran-ntn-xapp-isac.h
    # Helper
    helper/oran-ntn-helper.h
)

set(test_sources
    test/oran-ntn-test-suite.cc
)

set(oran_ntn_extra_libs "")
if(${ENABLE_SQLITE})
    list(APPEND oran_ntn_extra_libs ${SQLite3_LIBRARIES})
endif()
# T3 — libsctp for SCTP E2 transport (always linked; runtime fallback to TCP
# when the kernel SCTP module is unavailable).
find_library(SCTP_LIB sctp)
if(SCTP_LIB)
    list(APPEND oran_ntn_extra_libs ${SCTP_LIB})
endif()

build_lib(
    LIBNAME oran-ntn
    SOURCE_FILES ${source_files}
    HEADER_FILES ${header_files}
    LIBRARIES_TO_LINK
        ${libcore}
        ${libnetwork}
        ${libmobility}
        ${libspectrum}
        ${libpropagation}
        ${libinternet}
        ${libapplications}
        ${libsatellite}
        ${libmmwave}
        ${liblte}
        ${libns3-ai-ntn}
        ${oran_ntn_extra_libs}
    TEST_SOURCES ${test_sources}
)
