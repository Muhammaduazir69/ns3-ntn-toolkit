set(source_files
    bridge/sionna-transport.cc
    bridge/sionna-udp-transport.cc
    bridge/sionna-pybind-transport.cc
    bridge/ns3-sionna-channel.cc
    # Roadmap §4.2.4 — 4-D LRU caching transport
    bridge/sionna-caching-transport.cc
    # Roadmap §4.2.7 — composite atmospheric cascade
    bridge/ntn-atmospheric-loss-chain.cc
    bridge/ntn-sionna-cascade-channel.cc
    # Roadmap §4.2.5 — async batched query API
    bridge/sionna-batch-client.cc
    # Roadmap §4.2.6 — precompute/replay backend
    bridge/sionna-replay-transport.cc
    # Roadmap §4.2.11 — paths.apply_doppler() C++ equivalent
    bridge/cir-doppler-synth.cc
    # Roadmap §4.3.3 — Sionna vs propagation-model calibrator
    bridge/sionna-calibrator.cc
)

set(header_files
    bridge/sionna-transport.h
    bridge/sionna-udp-transport.h
    bridge/sionna-pybind-transport.h
    bridge/ns3-sionna-channel.h
    # Roadmap §4.2.4 — 4-D LRU caching transport
    bridge/sionna-caching-transport.h
    # Roadmap §4.2.7 — composite atmospheric cascade
    bridge/ntn-atmospheric-loss-chain.h
    bridge/ntn-sionna-cascade-channel.h
    # Roadmap §4.2.5 — async batched query API
    bridge/sionna-batch-client.h
    # Roadmap §4.2.6 — precompute/replay backend
    bridge/sionna-replay-transport.h
    # Roadmap §4.2.11 — paths.apply_doppler() C++ equivalent
    bridge/cir-doppler-synth.h
    # Roadmap §4.3.3 — Sionna vs propagation-model calibrator
    bridge/sionna-calibrator.h
)

set(test_sources
    test/ntn-sionna-test-suite.cc
)

build_lib(
  LIBNAME ntn-sionna
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpropagation}
    ${libthz-ntn}
  TEST_SOURCES ${test_sources}
)
