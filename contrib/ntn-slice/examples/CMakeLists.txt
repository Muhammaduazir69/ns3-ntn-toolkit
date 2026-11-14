build_lib_example(
  NAME ntn-three-slice-leo-geo
  SOURCE_FILES ntn-three-slice-leo-geo.cc
  LIBRARIES_TO_LINK
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

# Real per-slice traffic with NtnSliceSelector + SliceIsolationMonitor.
build_lib_example(
  NAME ntn-slice-isolation-traffic
  SOURCE_FILES ntn-slice-isolation-traffic.cc
  LIBRARIES_TO_LINK
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

# Phase-2 (RAN recipe): slices share ONE real mmwave cell; per-slice KPIs measured.
build_lib_example(
  NAME ntn-slice-real-stack
  SOURCE_FILES ntn-slice-real-stack.cc
  LIBRARIES_TO_LINK
    ${libntn-observability}
    ${libntn-slice}
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
)


# Slice partition enforced by the MAC: NtnSliceQuotaScheduler, static or closed-loop quotas, on a LEO pass.
build_lib_example(
  NAME ntn-slice-quota-leo
  SOURCE_FILES ntn-slice-quota-leo.cc
  LIBRARIES_TO_LINK
    ${libntn-slice}
    ${libntn-traffic}
    ${libntn-constellation}
    ${libnr}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libntn-cho}
)

build_lib_example(
  NAME ntn-slice-pass-leo
  SOURCE_FILES ntn-slice-pass-leo.cc
  LIBRARIES_TO_LINK
    ${libntn-slice}
    ${libntn-traffic}
    ${libntn-constellation}
    ${libnr}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libntn-cho}
)

build_lib_example(
  NAME ntn-slice-tenants-leo
  SOURCE_FILES ntn-slice-tenants-leo.cc
  LIBRARIES_TO_LINK
    ${libntn-slice}
    ${libntn-traffic}
    ${libntn-constellation}
    ${libnr}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libntn-cho}
)
