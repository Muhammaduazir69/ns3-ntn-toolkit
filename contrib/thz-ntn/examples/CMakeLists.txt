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

# Real packet transmission: sub-THz LEO downlink gated by molecular absorption.
build_lib_example(
  NAME thz-ntn-leo-ground-downlink-traffic
  SOURCE_FILES thz-ntn-leo-ground-downlink-traffic.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
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

# Real packet transmission over a 300 GHz ISL gated by ThzNtnIslChannel SNR.
build_lib_example(
  NAME thz-ntn-isl-traffic
  SOURCE_FILES thz-ntn-isl-traffic.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
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

# UM-MIMO array + beam-squint analysis. Drives the real beamforming engine:
# configures frequency/bandwidth/element count, generates a DFT codebook, and
# reads the worst-case wideband squint loss from ComputeBeamSquintLoss_dB().
build_lib_example(
  NAME thz-ntn-um-mimo
  SOURCE_FILES thz-ntn-um-mimo.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libcore}
    ${libmobility}
)

# Real packet transmission: RIS recovers a blocked THz link mid-sim.
build_lib_example(
  NAME thz-ntn-ris-relay-traffic
  SOURCE_FILES thz-ntn-ris-relay-traffic.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)

# Real packet transmission: ISAC comm/sense coexistence via ThzNtnIsacScheduler.
build_lib_example(
  NAME thz-ntn-isac-coexist-traffic
  SOURCE_FILES thz-ntn-isac-coexist-traffic.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)

# Real packet transmission: weather front (fog/rain/snow) over a THz downlink.
build_lib_example(
  NAME thz-ntn-weather-traffic
  SOURCE_FILES thz-ntn-weather-traffic.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)

# RIS-assisted THz downlink analysis: direct vs RIS-assisted SNR, N^2 scaling
# law, and quantization loss, using the current ThzNtnRis API (cascaded path
# loss computed from the actual Sat/RIS/GT MobilityModel positions).
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

# Full-stack integration demo: downlink link budget + RIS assist, ISL SNR/
# capacity from satellite MobilityModels, ISAC debris sensing, EKF beam
# tracking, and achievable spectral efficiency, all on the current API.
build_lib_example(
  NAME thz-ntn-full-stack
  SOURCE_FILES thz-ntn-full-stack.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libcore}
    ${libmobility}
)

# Roadmap §4.3.7 — thz-ntn × oran-ntn closed-loop full-stack example. THz channel
# KPIs (ThzNtnMolecularAbsorption + ThzNtnRis) feed an OranNtnE2Node KPM report;
# the xApp toggles the RIS when SINR crosses a threshold; the data plane goodput
# tracks the loop.
build_lib_example(
  NAME thz-ntn-ric-controlled-traffic
  SOURCE_FILES thz-ntn-ric-controlled-traffic.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${liboran-ntn}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)
