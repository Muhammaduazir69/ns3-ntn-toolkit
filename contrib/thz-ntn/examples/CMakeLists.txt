build_lib_example(
  NAME thz-ntn-leo-ground
  SOURCE_FILES thz-ntn-leo-ground.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libntn-cho}
    ${libntn-constellation}
    ${libntn-traffic}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
)

# Sub-THz LEO downlink on a real mmwave NR cell; molecular absorption is a
# channel plug-in attenuating real packets (measured SINR/TBLER/goodput).
build_lib_example(
  NAME thz-ntn-leo-ground-downlink-traffic
  SOURCE_FILES thz-ntn-leo-ground-downlink-traffic.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
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

build_lib_example(
  NAME thz-ntn-isl
  SOURCE_FILES thz-ntn-isl.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libntn-cho}
    ${libntn-constellation}
    ${libntn-traffic}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
)

# Real mmwave NR ISL between two cross-plane SGP4 satellites of a Starlink-
# class shell; ThzNtnIslChannel's analytic budget printed beside measured SINR.
build_lib_example(
  NAME thz-ntn-isl-traffic
  SOURCE_FILES thz-ntn-isl-traffic.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
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

# EKF beam tracking of a real SGP4 pass; the tracker's pointing loss is a
# live channel reconfiguration on a real mmwave NR cell (measured KPIs).
build_lib_example(
  NAME thz-ntn-beam-tracking
  SOURCE_FILES thz-ntn-beam-tracking.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
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

# UM-MIMO array + beam-squint analysis. Drives the real beamforming engine:
# configures frequency/bandwidth/element count, generates a DFT codebook, and
# reads the worst-case wideband squint loss from ComputeBeamSquintLoss_dB().
build_lib_example(
  NAME thz-ntn-um-mimo
  SOURCE_FILES thz-ntn-um-mimo.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libntn-cho}
    ${libntn-constellation}
    ${libcore}
    ${libmobility}
)

# RIS recovers a blocked THz link mid-sim — blockage + RIS engagement are
# LIVE channel events on a real mmwave NR cell (measured SINR/TBLER/goodput).
build_lib_example(
  NAME thz-ntn-ris-relay-traffic
  SOURCE_FILES thz-ntn-ris-relay-traffic.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
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

# ISAC comm/sense coexistence on a real mmwave NR cell: the real
# ThzNtnIsacScheduler's comm share gates the live channel TDM-style (measured).
build_lib_example(
  NAME thz-ntn-isac-coexist-traffic
  SOURCE_FILES thz-ntn-isac-coexist-traffic.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
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

# Weather front (fog/rain/snow) as a LIVE channel plug-in on a real mmwave NR
# NTN cell — the weather attenuates real packets (measured SINR/TBLER/goodput).
build_lib_example(
  NAME thz-ntn-weather-traffic
  SOURCE_FILES thz-ntn-weather-traffic.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
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

# RIS-assisted THz downlink analysis: direct vs RIS-assisted SNR, N^2 scaling
# law, and quantization loss, using the current ThzNtnRis API (cascaded path
# loss computed from the actual Sat/RIS/GT MobilityModel positions).
build_lib_example(
  NAME thz-ntn-ris-assisted
  SOURCE_FILES thz-ntn-ris-assisted.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libntn-cho}
    ${libntn-constellation}
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
    ${libntn-cho}
    ${libntn-constellation}
    ${libcore}
    ${libmobility}
)
endif()

build_lib_example(
  NAME thz-ntn-dband-constellation
  SOURCE_FILES thz-ntn-dband-constellation.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libntn-cho}
    ${libntn-constellation}
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
    ${libntn-cho}
    ${libntn-constellation}
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

# Phase-1 real-stack flagship: THz physics as a channel plug-in on a real radio.
build_lib_example(
  NAME thz-ntn-real-stack
  SOURCE_FILES thz-ntn-real-stack.cc
  LIBRARIES_TO_LINK
    ${libthz-ntn}
    ${libntn-cho}
    ${libntn-constellation}
    ${libntn-traffic}
    ${libntn-observability}
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
