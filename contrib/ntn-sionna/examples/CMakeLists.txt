# MEASURED real-plane LEO pass: Sionna multipath+Doppler EXCESS fades real
# packets on a real mmwave NR link (NtnRealStackHelper); TR 38.811 free space
# is the reference column. Also exercises the §4.2.6 offline replay backend.
build_lib_example(
  NAME leo-pass-sionna-vs-tr38811
  SOURCE_FILES leo-pass-sionna-vs-tr38811.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
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

# Phase-2 real-stack (channel-plugin recipe): the FULL Sionna RT CIR (multipath +
# Doppler) fades real packets on a real mmwave NR link; SINR is measured.
build_lib_example(
  NAME ntn-sionna-cir-real-stack
  SOURCE_FILES ntn-sionna-cir-real-stack.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
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

# Roadmap §4.2.12 — mMIMO precoder vs codebook beamformer comparison.
build_lib_example(
  NAME mmimo-vs-codebook-leo
  SOURCE_FILES mmimo-vs-codebook-leo.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
    ${libntn-cho}
    ${libntn-constellation}
    ${libcore}
    ${libmobility}
    ${libpropagation}
)

# Roadmap §4.2.12 — RIS-assisted LEO ground link: MEASURED DL SINR/goodput on
# a real mmwave NR cell (NtnRealStackHelper) with ITU-R atmospheric excess in
# the packet path; RIS reflection gain is an analytic Sionna-cascade probe.
build_lib_example(
  NAME ris-assisted-leo-link
  SOURCE_FILES ris-assisted-leo-link.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
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

# Roadmap §4.2.12 — AODT-style 4-UE city-block with caching transport.
build_lib_example(
  NAME city-block-4ue-cache
  SOURCE_FILES city-block-4ue-cache.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
    ${libntn-cho}
    ${libntn-constellation}
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
    ${libntn-cho}
    ${libntn-constellation}
    ${libcore}
    ${libmobility}
    ${libpropagation}
)

# LEO downlink on a real mmwave NR cell; ITU-R atmospheric cascade (P.676 +
# P.618/P.838 + optional P.681 LMS) as a live channel plug-in (measured KPIs).
build_lib_example(
  NAME ntn-sionna-leo-downlink-traffic
  SOURCE_FILES ntn-sionna-leo-downlink-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-observability}
    ${libntn-sionna}
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

# Dynamic rain cell sweeping over the gateway during a LEO pass — the ITU-R
# rain physics is a LIVE channel plug-in on a real mmwave NR cell (measured).
build_lib_example(
  NAME ntn-sionna-rain-event-traffic
  SOURCE_FILES ntn-sionna-rain-event-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
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

# RIS recovers a blocked NLOS LEO link mid-sim — blockage + RIS engagement
# are LIVE channel events on a real mmwave NR cell (measured KPIs).
build_lib_example(
  NAME ntn-sionna-ris-relay-traffic
  SOURCE_FILES ntn-sionna-ris-relay-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
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

# SISO vs NxN MIMO terminals on ONE real mmwave NR cell; the array gain is a
# per-UE channel plug-in, so the SISO/MIMO gap is measured, not asserted.
build_lib_example(
  NAME ntn-sionna-mimo-traffic
  SOURCE_FILES ntn-sionna-mimo-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
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

# Multi-satellite hand-over on measured radio + real SGP4 Walker geometry;
# candidates projected from the one real measurement via the Friis ratio.
build_lib_example(
  NAME ntn-sionna-constellation-handover-traffic
  SOURCE_FILES ntn-sionna-constellation-handover-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
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

# Roadmap §4.2.8 — compose the ntn-sionna cascade channel with the oran-ntn
# TR 38.811 channel via ns-3 PropagationLossModel chaining; drive a real data
# plane from the composed CalcRxPower(). Links oran-ntn for OranNtnChannelModel.
build_lib_example(
  NAME ntn-sionna-composed-channel-traffic
  SOURCE_FILES ntn-sionna-composed-channel-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-sionna}
    ${libntn-cho}
    ${libntn-constellation}
    ${liboran-ntn}
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
