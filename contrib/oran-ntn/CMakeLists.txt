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
    TEST_SOURCES ${test_sources}
)
