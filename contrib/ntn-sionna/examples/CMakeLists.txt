build_lib_example(
  NAME leo-pass-sionna-vs-tr38811
  SOURCE_FILES leo-pass-sionna-vs-tr38811.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
    ${libcore}
    ${libmobility}
    ${libpropagation}
)

# Roadmap §4.2.12 — mMIMO precoder vs codebook beamformer comparison.
build_lib_example(
  NAME mmimo-vs-codebook-leo
  SOURCE_FILES mmimo-vs-codebook-leo.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
    ${libcore}
    ${libmobility}
    ${libpropagation}
)

# Roadmap §4.2.12 — RIS-assisted LEO ground link before/after.
build_lib_example(
  NAME ris-assisted-leo-link
  SOURCE_FILES ris-assisted-leo-link.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
    ${libcore}
    ${libmobility}
    ${libpropagation}
)

# Roadmap §4.2.12 — AODT-style 4-UE city-block with caching transport.
build_lib_example(
  NAME city-block-4ue-cache
  SOURCE_FILES city-block-4ue-cache.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
    ${libcore}
    ${libmobility}
    ${libpropagation}
)

# Roadmap §4.3.3 — Sionna RT calibration harness driver.
build_lib_example(
  NAME sionna-calibration-harness
  SOURCE_FILES sionna-calibration-harness.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
    ${libcore}
    ${libmobility}
    ${libpropagation}
)

# End-to-end packet transmission: LEO downlink whose Wi-Fi PHY link is gated
# by the NTN cascade channel; real UDP traffic + FlowMonitor.
build_lib_example(
  NAME ntn-sionna-leo-downlink-traffic
  SOURCE_FILES ntn-sionna-leo-downlink-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
    ${libcore}
    ${libmobility}
    ${libpropagation}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)

# Dynamic rain cell sweeping over the gateway during a LEO pass; real traffic.
build_lib_example(
  NAME ntn-sionna-rain-event-traffic
  SOURCE_FILES ntn-sionna-rain-event-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
    ${libcore}
    ${libmobility}
    ${libpropagation}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)

# RIS recovers a blocked NLOS LEO link mid-sim; real traffic before/after.
build_lib_example(
  NAME ntn-sionna-ris-relay-traffic
  SOURCE_FILES ntn-sionna-ris-relay-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
    ${libcore}
    ${libmobility}
    ${libpropagation}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)

# SISO vs N×N MIMO terminals on the same LEO pass; real traffic to both.
build_lib_example(
  NAME ntn-sionna-mimo-traffic
  SOURCE_FILES ntn-sionna-mimo-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
    ${libcore}
    ${libmobility}
    ${libpropagation}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)

# Multi-satellite hand-over: UE follows the best LEO; traffic spans hand-overs.
build_lib_example(
  NAME ntn-sionna-constellation-handover-traffic
  SOURCE_FILES ntn-sionna-constellation-handover-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
    ${libcore}
    ${libmobility}
    ${libpropagation}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)

# Roadmap §4.2.8 — compose the ntn-sionna cascade channel with the oran-ntn
# TR 38.811 channel via ns-3 PropagationLossModel chaining; drive a real data
# plane from the composed CalcRxPower(). Links oran-ntn for OranNtnChannelModel.
build_lib_example(
  NAME ntn-sionna-composed-channel-traffic
  SOURCE_FILES ntn-sionna-composed-channel-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
    ${liboran-ntn}
    ${libcore}
    ${libmobility}
    ${libpropagation}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)
