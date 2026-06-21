# RRC NTN examples — all run real UDP traffic over a real mmwave NR NTN cell
# (NtnRealStackHelper) with TA/SIB19/DRX/location bound to the live geometry and
# the radio KPIs measured off the mmwave PHY trace.
set(_rrc_real_libs
  ${libntn-rrc}
  ${libntn-traffic}
  ${libntn-cho}
  ${libntn-constellation}
  ${libcore}
  ${libnetwork}
  ${libmobility}
  ${libinternet}
  ${libapplications}
  ${libmmwave}
  ${liblte}
  ${libsatellite}
)

build_lib_example(
  NAME ntn-rrc-leo-pass
  SOURCE_FILES ntn-rrc-leo-pass.cc
  LIBRARIES_TO_LINK ${_rrc_real_libs}
)
build_lib_example(
  NAME ntn-rrc-full-stack
  SOURCE_FILES ntn-rrc-full-stack.cc
  LIBRARIES_TO_LINK ${_rrc_real_libs}
)
build_lib_example(
  NAME ntn-rrc-from-tle
  SOURCE_FILES ntn-rrc-from-tle.cc
  LIBRARIES_TO_LINK ${_rrc_real_libs}
)
build_lib_example(
  NAME ntn-rrc-drx-data-traffic
  SOURCE_FILES ntn-rrc-drx-data-traffic.cc
  LIBRARIES_TO_LINK ${_rrc_real_libs}
)
# Real-stack flagship: SIB19 + ephemeris TA on a real mmwave NR cell.
build_lib_example(
  NAME ntn-rrc-real-stack
  SOURCE_FILES ntn-rrc-real-stack.cc
  LIBRARIES_TO_LINK ${_rrc_real_libs}
    ${libntn-observability}
)
