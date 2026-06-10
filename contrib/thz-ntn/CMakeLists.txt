set(source_files
    # Phase 1: Foundation - Channel models and spectrum
    model/thz-ntn-molecular-absorption.cc
    model/thz-ntn-propagation-loss-model.cc
    # 4.3.1 — HITRAN-2024 LUT loader (Roadmap §4.3.1)
    model/thz-ntn-hitran-lut.cc
    # 4.3.2 — ITU-R P.618/676/838/681 wrappers (Roadmap §4.3.2)
    model/thz-ntn-itu-recommendations.cc
    model/thz-ntn-weather-attenuation.cc
    model/thz-ntn-scintillation.cc
    model/thz-ntn-pointing-error.cc
    model/thz-ntn-hardware-impairments.cc
    model/thz-ntn-spectrum.cc
    model/thz-ntn-link-budget.cc
    model/thz-ntn-channel-model.cc
    model/thz-ntn-free-space-loss.cc
    # Phase 2: PHY + Antenna
    model/thz-ntn-phy.cc
    model/thz-ntn-phy-sat.cc
    model/thz-ntn-phy-ground.cc
    model/thz-ntn-waveform.cc
    model/thz-ntn-antenna-array.cc
    model/thz-ntn-beamforming.cc
    model/thz-ntn-beam-tracking.cc
    # Phase 3: MAC + ISL + RIS
    model/thz-ntn-mac.cc
    model/thz-ntn-mac-scheduler.cc
    model/thz-ntn-isl-channel.cc
    model/thz-ntn-isl-link.cc
    model/thz-ntn-ris.cc
    model/thz-ntn-ris-controller.cc
    # Phase 4: ISAC
    model/thz-ntn-isac.cc
    model/thz-ntn-isac-processor.cc
    # Roadmap §4.3.4 — NYUSIM-140 reference loader + calibrator
    model/thz-ntn-nyusim-reference.cc
    model/thz-ntn-nyusim-calibrator.cc
    # Roadmap §4.3.5 — ISAC scheduler + α-μ fading
    model/thz-ntn-isac-scheduler.cc
    model/thz-ntn-alpha-mu-fading.cc
    # Roadmap §4.3.6 — THz-RIS-SM service model + xApp
    model/thz-ntn-ris-service-model.cc
    model/thz-ntn-ris-xapp.cc
    # Helper
    helper/thz-ntn-helper.cc
)

set(header_files
    # Phase 1: Foundation - Channel models and spectrum
    model/thz-ntn-molecular-absorption.h
    model/thz-ntn-propagation-loss-model.h
    # 4.3.1 — HITRAN-2024 LUT loader (Roadmap §4.3.1)
    model/thz-ntn-hitran-lut.h
    # 4.3.2 — ITU-R P.618/676/838/681 wrappers (Roadmap §4.3.2)
    model/thz-ntn-itu-recommendations.h
    model/thz-ntn-weather-attenuation.h
    model/thz-ntn-scintillation.h
    model/thz-ntn-pointing-error.h
    model/thz-ntn-hardware-impairments.h
    model/thz-ntn-spectrum.h
    model/thz-ntn-link-budget.h
    model/thz-ntn-channel-model.h
    model/thz-ntn-free-space-loss.h
    # Phase 2: PHY + Antenna
    model/thz-ntn-phy.h
    model/thz-ntn-phy-sat.h
    model/thz-ntn-phy-ground.h
    model/thz-ntn-waveform.h
    model/thz-ntn-antenna-array.h
    model/thz-ntn-beamforming.h
    model/thz-ntn-beam-tracking.h
    # Phase 3: MAC + ISL + RIS
    model/thz-ntn-mac.h
    model/thz-ntn-mac-scheduler.h
    model/thz-ntn-isl-channel.h
    model/thz-ntn-isl-link.h
    model/thz-ntn-ris.h
    model/thz-ntn-ris-controller.h
    # Phase 4: ISAC
    model/thz-ntn-isac.h
    model/thz-ntn-isac-processor.h
    # Roadmap §4.3.4 — NYUSIM-140 reference loader + calibrator
    model/thz-ntn-nyusim-reference.h
    model/thz-ntn-nyusim-calibrator.h
    # Roadmap §4.3.5 — ISAC scheduler + α-μ fading
    model/thz-ntn-isac-scheduler.h
    model/thz-ntn-alpha-mu-fading.h
    # Roadmap §4.3.6 — THz-RIS-SM service model + xApp
    model/thz-ntn-ris-service-model.h
    model/thz-ntn-ris-xapp.h
    # Helper
    helper/thz-ntn-helper.h
)

set(test_sources
    test/thz-ntn-test-suite.cc
)

build_lib(
    LIBNAME thz-ntn
    SOURCE_FILES ${source_files}
    HEADER_FILES ${header_files}
    LIBRARIES_TO_LINK
        ${libcore}
        ${libnetwork}
        ${libmobility}
        ${libspectrum}
        ${libpropagation}
        ${libinternet}
        ${libapplications}
        ${libsatellite}
        ${libmmwave}
    TEST_SOURCES ${test_sources}
)
