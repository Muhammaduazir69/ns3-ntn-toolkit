# Constellation-scale RIC on real SGP4 geometry: anchored cells measured on a
# real mmwave NR NTN link, scale-out KPM from a TR 38.821-style budget over the
# same live geometry (per-row provenance in kpm_feed.csv).
build_lib_example(
  NAME oran-ntn-gym-handover-example
  SOURCE_FILES oran-ntn-gym-handover-example.cc
  LIBRARIES_TO_LINK
    ${liboran-ntn}
    ${libntn-traffic}
    ${libntn-cho}
    ${libntn-constellation}
    ${libns3-ai-ntn}
)

build_lib_example(
  NAME oran-ntn-full-scenario
  SOURCE_FILES oran-ntn-full-scenario.cc
  LIBRARIES_TO_LINK
    ${liboran-ntn}
    ${libntn-traffic}
    ${libntn-cho}
    ${libntn-constellation}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libspectrum}
    ${libpropagation}
    ${libinternet}
    ${libapplications}
    ${libmmwave}
    ${liblte}
)

# Phase-1 real-stack flagship: KPM from MEASURED mmwave PHY SINR drives the RIC.
build_lib_example(
  NAME oran-ntn-real-stack-scenario
  SOURCE_FILES oran-ntn-real-stack-scenario.cc
  LIBRARIES_TO_LINK
    ${libntn-observability}
    ${liboran-ntn}
    ${libntn-traffic}
    ${libntn-cho}
    ${libntn-constellation}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libspectrum}
    ${libpropagation}
    ${libinternet}
    ${libapplications}
    ${libmmwave}
    ${liblte}
)

# Closed RIC control loop: E2-KPM -> mMIMO precoder xApp -> beam -> real link.
build_lib_example(
  NAME oran-ntn-ric-controlled-traffic
  SOURCE_FILES oran-ntn-ric-controlled-traffic.cc
  LIBRARIES_TO_LINK
    ${liboran-ntn}
    ${libntn-cho}
    ${libntn-constellation}
    ${libntn-traffic}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libspectrum}
    ${libpropagation}
    ${libinternet}
    ${libapplications}
    ${libmmwave}
    ${liblte}
)

# Phase-3 composition: one shared mmwave cell feeds RIC + slice + observability.
build_lib_example(
  NAME ntn-e2e-full-stack
  SOURCE_FILES ntn-e2e-full-stack.cc
  LIBRARIES_TO_LINK
    ${liboran-ntn}
    ${libntn-slice}
    ${libntn-traffic}
    ${libntn-cho}
    ${libntn-constellation}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libspectrum}
    ${libpropagation}
    ${libinternet}
    ${libapplications}
    ${libmmwave}
    ${liblte}
)

# WS3 — RIC placement A/B on the real cell: on-board vs gateway vs cloud,
# measured fade-recovery time per placement.
build_lib_example(
  NAME oran-ntn-ric-placement-ab
  SOURCE_FILES oran-ntn-ric-placement-ab.cc
  LIBRARIES_TO_LINK
    ${liboran-ntn}
    ${libntn-traffic}
    ${libntn-cho}
    ${libntn-constellation}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libspectrum}
    ${libpropagation}
    ${libinternet}
    ${libapplications}
    ${libmmwave}
    ${liblte}
)

# WS3 — cross-domain slice adaptation: NWDAF + SMO + TPN + ONNX xApp over two
# real SGP4 transport paths.
build_lib_example(
  NAME oran-ntn-cross-domain-slice
  SOURCE_FILES oran-ntn-cross-domain-slice.cc
  LIBRARIES_TO_LINK
    ${liboran-ntn}
    ${libntn-traffic}
    ${libntn-cho}
    ${libntn-constellation}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libinternet}
    ${libapplications}
    ${libpoint-to-point}
)

# WS4 — four payload architectures measured on one real cell.
build_lib_example(
  NAME oran-ntn-payload-options-ab
  SOURCE_FILES oran-ntn-payload-options-ab.cc
  LIBRARIES_TO_LINK
    ${liboran-ntn}
    ${libntn-traffic}
    ${libntn-cho}
    ${libntn-constellation}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libspectrum}
    ${libpropagation}
    ${libinternet}
    ${libapplications}
    ${libmmwave}
    ${liblte}
)

# WS5 — Table-3 per-platform user-plane latency validation (real packets).
build_lib_example(
  NAME ntn-platform-latency-validation
  SOURCE_FILES ntn-platform-latency-validation.cc
  LIBRARIES_TO_LINK
    ${liboran-ntn}
    ${libntn-traffic}
    ${libcore}
    ${libnetwork}
    ${libinternet}
    ${libapplications}
    ${libpoint-to-point}
)

# WS6 — post-disaster emergency communication flagship.
build_lib_example(
  NAME oran-ntn-emergency-communication
  SOURCE_FILES oran-ntn-emergency-communication.cc
  LIBRARIES_TO_LINK
    ${liboran-ntn}
    ${libntn-traffic}
    ${libntn-cho}
    ${libntn-constellation}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libspectrum}
    ${libpropagation}
    ${libinternet}
    ${libapplications}
    ${libmmwave}
    ${liblte}
)

# Standalone two-process E2 end-to-end driver (RIC + agent roles) over real SCTP.
build_lib_example(
  NAME oran-ntn-e2-termination
  SOURCE_FILES oran-ntn-e2-termination.cc
  LIBRARIES_TO_LINK
    ${liboran-ntn}
    ${libcore}
    ${libnetwork}
)
