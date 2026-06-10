# Real ISL-routed packet forwarding GS1 -> satA -> [ISL] -> satB -> GS2: real
# Ipv4 forwarding through the sat nodes, real range delays, geometry contact gates.
build_lib_example(
  NAME ntn-constellation-isl-routed-traffic
  SOURCE_FILES ntn-constellation-isl-routed-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-constellation}
    ${libntn-cho}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libntn-traffic}
)

# Walker constellation as ephemeris/contact-graph context; ONE serving satellite
# carries a real mmwave NR cell to a ground terminal (measured SINR/throughput).
build_lib_example(
  NAME ntn-constellation-walker-traffic
  SOURCE_FILES ntn-constellation-walker-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-constellation}
    ${libntn-traffic}
    ${libntn-cho}
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

# Single SGP4 LEO pass driving a real mmwave NR cell over a ground station at the
# sub-point, with GSL up/down sampling (measured SINR/throughput).
build_lib_example(
  NAME ntn-constellation-sgp4-mobility-traffic
  SOURCE_FILES ntn-constellation-sgp4-mobility-traffic.cc
  LIBRARIES_TO_LINK
    ${libntn-constellation}
    ${libntn-traffic}
    ${libntn-cho}
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

# Real-stack flagship: ContactGraphRouter INSTALLS routes; real packets traverse
# the satellite nodes and adaptively reroute as the constellation moves.
build_lib_example(
  NAME ntn-constellation-real-routed
  SOURCE_FILES ntn-constellation-real-routed.cc
  LIBRARIES_TO_LINK
    ${libntn-constellation}
    ${libntn-cho}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpoint-to-point}
    ${libinternet}
    ${libapplications}
    ${libntn-traffic}
)
