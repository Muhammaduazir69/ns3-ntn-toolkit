# Real ISL-routed packet forwarding GS1 -> satA -> [ISL] -> satB -> GS2.
build_lib_example(
  NAME ntn-constellation-isl-routed-traffic
  SOURCE_FILES ntn-constellation-isl-routed-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-constellation}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)

# Roadmap §4.4.5 — Walker constellation driving a real UDP data plane
# through NtnRealisticTrafficHelper, sampling the ContactGraphScheduler
# every second to log live ISL connectivity changes alongside the flow.
build_lib_example(
  NAME ntn-constellation-walker-traffic
  SOURCE_FILES ntn-constellation-walker-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-constellation}
    ${libntn-traffic}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)

# Roadmap §4.4.4 — Single SGP4 LEO pass over a configurable ground station
# with realistic data plane and GSL up/down sampling.
build_lib_example(
  NAME ntn-constellation-sgp4-mobility-traffic
  SOURCE_FILES ntn-constellation-sgp4-mobility-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-constellation}
    ${libntn-traffic}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)
