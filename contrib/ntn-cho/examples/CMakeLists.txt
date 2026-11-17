build_lib_example(
  NAME ntn-cho-leo-basic
  SOURCE_FILES ntn-cho-leo-basic.cc
  LIBRARIES_TO_LINK
    ${libntn-cho}
    ${libntn-traffic}
    ${libntn-constellation}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libinternet}
    ${libapplications}
    ${libmmwave}
    ${liblte}
    ${libnr}
)

build_lib_example(
  NAME ntn-cho-full-constellation
  SOURCE_FILES ntn-cho-full-constellation.cc
  LIBRARIES_TO_LINK
    ${libntn-cho}
    ${libntn-traffic}
    ${libntn-constellation}
    ${libntn-observability}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libinternet}
    ${libapplications}
    ${libmmwave}
    ${liblte}
    ${libnr}
)

build_lib_example(
  NAME ntn-realistic-mobility-demo
  SOURCE_FILES ntn-realistic-mobility-demo.cc
  LIBRARIES_TO_LINK
    ${libntn-cho}
    ${libntn-traffic}
    ${libntn-constellation}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libinternet}
    ${libapplications}
    ${libmmwave}
    ${liblte}
)

# Real UDP traffic to a ground UE on a real mmwave NR cell, handed over by the
# TTE-aware CHO on MEASURED serving SINR.
build_lib_example(
  NAME ntn-cho-handover-traffic
  SOURCE_FILES ntn-cho-handover-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-cho}
    ${libntn-traffic}
    ${libntn-constellation}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libinternet}
    ${libapplications}
    ${libmmwave}
    ${liblte}
    ${libnr}
)

# Real-stack flagship: TTE-aware CHO decides on MEASURED mmwave SINR.
build_lib_example(
  NAME ntn-cho-real-stack
  SOURCE_FILES ntn-cho-real-stack.cc
  LIBRARIES_TO_LINK
    ${libntn-cho}
    ${libntn-traffic}
    ${libntn-constellation}
    ${libntn-observability}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libinternet}
    ${libapplications}
    ${libmmwave}
    ${liblte}
    ${libnr}
)

build_lib_example(
  NAME ntn-cho-tte-campaign
  SOURCE_FILES ntn-cho-tte-campaign.cc
  LIBRARIES_TO_LINK
    ${libntn-cho}
    ${libntn-constellation}
    ${libntn-traffic}
    ${libnr}
    ${libinternet}
    ${libapplications}
    ${libpropagation}
    ${libcore}
    ${libnetwork}
    ${libmobility}
)
