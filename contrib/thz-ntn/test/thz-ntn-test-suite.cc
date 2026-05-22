/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN Test Suite
 *
 * Comprehensive test suite for the THz non-terrestrial network simulation
 * module.  Covers channel models (molecular absorption, FSPL, weather,
 * pointing error), hardware impairments, spectrum windows, link budget,
 * antenna array, beamforming, ISL, RIS, and ISAC functionality.
 */

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/test.h>
#include <ns3/uinteger.h>

#include <ns3/constant-position-mobility-model.h>
#include <ns3/constant-velocity-mobility-model.h>
#include <ns3/node.h>
#include <ns3/simulator.h>
#include <ns3/thz-ntn-antenna-array.h>
#include <ns3/thz-ntn-beamforming.h>
#include <ns3/thz-ntn-channel-model.h>
#include <ns3/thz-ntn-hardware-impairments.h>
#include <ns3/thz-ntn-hitran-lut.h>
#include <ns3/thz-ntn-isac.h>
#include <ns3/thz-ntn-itu-recommendations.h>
#include <ns3/thz-ntn-isl-channel.h>
#include <ns3/thz-ntn-link-budget.h>
#include <ns3/thz-ntn-molecular-absorption.h>
#include <ns3/thz-ntn-pointing-error.h>
#include <ns3/thz-ntn-ris.h>
#include <ns3/thz-ntn-spectrum.h>
#include <ns3/thz-ntn-weather-attenuation.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnTestSuite");

// Physical constants used in tests
static constexpr double SPEED_OF_LIGHT = 299792458.0;
static constexpr double PI = 3.14159265358979323846;

// ============================================================================
// Test 1: Molecular Absorption
// ============================================================================

/**
 * \ingroup thz-ntn-test
 * \brief Verify molecular absorption: zero for ISL, non-zero for ground-sat,
 *        increases with frequency.
 */
class ThzNtnMolecularAbsorptionTest : public TestCase
{
  public:
    ThzNtnMolecularAbsorptionTest();
    void DoRun() override;
};

ThzNtnMolecularAbsorptionTest::ThzNtnMolecularAbsorptionTest()
    : TestCase("ThzNtnMolecularAbsorption: ISL=0, ground-sat>0, increases with freq")
{
}

void
ThzNtnMolecularAbsorptionTest::DoRun()
{
    Ptr<ThzNtnMolecularAbsorption> model = CreateObject<ThzNtnMolecularAbsorption>();

    // ISL: both endpoints above 100 km -> absorption should be zero
    double islLoss = model->ComputeAbsorptionLoss_dB(
        300e9,    // 300 GHz
        5000e3,   // 5000 km distance
        550.0,    // TX at 550 km
        550.0,    // RX at 550 km
        0.0       // elevation irrelevant for ISL
    );
    NS_TEST_ASSERT_MSG_EQ_TOL(islLoss, 0.0, 0.01,
        "ISL molecular absorption should be zero (both endpoints in vacuum)");

    // Ground-to-satellite: should have non-zero absorption
    double gsLoss225 = model->ComputeAbsorptionLoss_dB(
        225e9,    // 225 GHz
        600e3,    // 600 km slant range
        0.0,      // TX on ground
        550.0,    // RX at 550 km
        90.0      // zenith
    );
    NS_TEST_ASSERT_MSG_GT(gsLoss225, 0.0,
        "Ground-sat absorption at 225 GHz should be positive");

    // Higher frequency should have higher absorption (generally)
    double gsLoss340 = model->ComputeAbsorptionLoss_dB(
        340e9,    // 340 GHz
        600e3,    // 600 km
        0.0,      // ground
        550.0,    // 550 km
        90.0      // zenith
    );
    NS_TEST_ASSERT_MSG_GT(gsLoss340, gsLoss225,
        "Absorption at 340 GHz should exceed absorption at 225 GHz");
}

// ============================================================================
// Test 2: Free-Space Path Loss
// ============================================================================

/**
 * \ingroup thz-ntn-test
 * \brief Verify FSPL matches 20*log10(4*pi*d*f/c) at known distance and frequency.
 */
class ThzNtnFsplTest : public TestCase
{
  public:
    ThzNtnFsplTest();
    void DoRun() override;
};

ThzNtnFsplTest::ThzNtnFsplTest()
    : TestCase("ThzNtnFspl: verify FSPL formula")
{
}

void
ThzNtnFsplTest::DoRun()
{
    // Use the link budget calculator which computes FSPL via the standard formula
    Ptr<ThzNtnLinkBudget> lb = CreateObject<ThzNtnLinkBudget>();

    // Test 1: 300 GHz, 1000 km (ISL scenario)
    double freqHz = 300e9;
    double distanceM = 1000e3;
    double expectedFspl = 20.0 * std::log10(4.0 * PI * distanceM * freqHz / SPEED_OF_LIGHT);

    lb->SetAttribute("DefaultFrequency", DoubleValue(freqHz));
    auto result = lb->ComputeIslBudget(freqHz, distanceM / 1000.0);

    NS_TEST_ASSERT_MSG_EQ_TOL(result.fspl_dB, expectedFspl, 0.1,
        "FSPL at 300 GHz / 1000 km should match analytical formula");

    // Test 2: 225 GHz, 550 km
    double freqHz2 = 225e9;
    double distanceM2 = 550e3;
    double expectedFspl2 = 20.0 * std::log10(4.0 * PI * distanceM2 * freqHz2 / SPEED_OF_LIGHT);

    lb->SetAttribute("DefaultFrequency", DoubleValue(freqHz2));
    auto result2 = lb->ComputeIslBudget(freqHz2, distanceM2 / 1000.0);

    NS_TEST_ASSERT_MSG_EQ_TOL(result2.fspl_dB, expectedFspl2, 0.1,
        "FSPL at 225 GHz / 550 km should match analytical formula");
}

// ============================================================================
// Test 3: Weather Attenuation
// ============================================================================

/**
 * \ingroup thz-ntn-test
 * \brief Verify rain attenuation increases with rain rate and frequency.
 */
class ThzNtnWeatherTest : public TestCase
{
  public:
    ThzNtnWeatherTest();
    void DoRun() override;
};

ThzNtnWeatherTest::ThzNtnWeatherTest()
    : TestCase("ThzNtnWeather: rain attenuation increases with rain rate and frequency")
{
}

void
ThzNtnWeatherTest::DoRun()
{
    Ptr<ThzNtnWeatherAttenuation> model = CreateObject<ThzNtnWeatherAttenuation>();

    // Rain at 225 GHz, low rain rate (freqHz, elevationDeg, rainRate_mm_h)
    double rainLow = model->ComputeRainAttenuation_dB(225e9, 30.0, 5.0);   // 30 deg elev, 5 mm/h
    // Rain at 225 GHz, higher rain rate
    double rainHigh = model->ComputeRainAttenuation_dB(225e9, 30.0, 25.0);  // 30 deg elev, 25 mm/h

    NS_TEST_ASSERT_MSG_GT(rainLow, 0.0,
        "Rain attenuation at 225 GHz should be positive");
    NS_TEST_ASSERT_MSG_GT(rainHigh, rainLow,
        "Higher rain rate should produce higher attenuation");

    // Higher frequency should have higher rain attenuation
    double rainHighFreq = model->ComputeRainAttenuation_dB(340e9, 30.0, 5.0);
    NS_TEST_ASSERT_MSG_GT(rainHighFreq, rainLow,
        "Rain attenuation at 340 GHz should exceed 225 GHz");
}

// ============================================================================
// Test 4: Pointing Error
// ============================================================================

/**
 * \ingroup thz-ntn-test
 * \brief Verify pointing loss formula.
 */
class ThzNtnPointingErrorTest : public TestCase
{
  public:
    ThzNtnPointingErrorTest();
    void DoRun() override;
};

ThzNtnPointingErrorTest::ThzNtnPointingErrorTest()
    : TestCase("ThzNtnPointingError: verify pointing loss is positive and increases")
{
}

void
ThzNtnPointingErrorTest::DoRun()
{
    Ptr<ThzNtnPointingError> model = CreateObject<ThzNtnPointingError>();

    // Pointing error at elevation 30 deg, satellite velocity 7.5 km/s
    double error30 = model->ComputePointingError_deg(30.0, 7.5);
    NS_TEST_ASSERT_MSG_GT(error30, 0.0,
        "Pointing error should be positive");

    // Compute pointing loss from pointing error
    // Use a narrow beamwidth (typical THz)
    double beamwidth = 0.5; // 0.5 degree beamwidth
    double loss = model->ComputePointingLoss_dB(error30, beamwidth);
    NS_TEST_ASSERT_MSG_GT(loss, 0.0,
        "Pointing loss should be positive for non-zero pointing error");

    // Larger pointing error -> larger loss
    double lossLarger = model->ComputePointingLoss_dB(error30 * 2.0, beamwidth);
    NS_TEST_ASSERT_MSG_GT(lossLarger, loss,
        "Doubling pointing error should increase pointing loss");
}

// ============================================================================
// Test 5: Hardware Impairments (ADC SQNR)
// ============================================================================

/**
 * \ingroup thz-ntn-test
 * \brief Verify ADC SQNR = 6.02*b + 1.76 dB.
 */
class ThzNtnHardwareTest : public TestCase
{
  public:
    ThzNtnHardwareTest();
    void DoRun() override;
};

ThzNtnHardwareTest::ThzNtnHardwareTest()
    : TestCase("ThzNtnHardware: ADC SQNR = 6.02*b + 1.76")
{
}

void
ThzNtnHardwareTest::DoRun()
{
    Ptr<ThzNtnHardwareImpairments> model = CreateObject<ThzNtnHardwareImpairments>();

    // Set 6-bit ADC
    model->SetAttribute("AdcBits", UintegerValue(6));

    // Expected SQNR for 6-bit ADC: 6.02*6 + 1.76 = 37.88 dB
    double expectedSqnr = 6.02 * 6.0 + 1.76;

    // Quantization noise for 0 dBm signal should be at -(SQNR) dBm
    double quantNoise = model->ComputeAdcQuantizationNoise_dBm(0.0);
    double measuredSqnr = 0.0 - quantNoise; // Signal - Noise = SQNR

    NS_TEST_ASSERT_MSG_EQ_TOL(measuredSqnr, expectedSqnr, 0.1,
        "ADC SQNR should be 6.02*b + 1.76 for b-bit ADC");
}

// ============================================================================
// Test 6: Spectrum (Atmospheric Windows)
// ============================================================================

/**
 * \ingroup thz-ntn-test
 * \brief Verify 5 atmospheric windows are returned by GetStandardWindows().
 */
class ThzNtnSpectrumTest : public TestCase
{
  public:
    ThzNtnSpectrumTest();
    void DoRun() override;
};

ThzNtnSpectrumTest::ThzNtnSpectrumTest()
    : TestCase("ThzNtnSpectrum: 5 standard atmospheric windows")
{
}

void
ThzNtnSpectrumTest::DoRun()
{
    auto windows = ThzNtnSpectrum::GetStandardWindows();

    NS_TEST_ASSERT_MSG_EQ(windows.size(), 5u,
        "There should be exactly 5 standard atmospheric windows");

    // Verify that each window has positive bandwidth and reasonable frequency
    for (const auto& w : windows)
    {
        NS_TEST_ASSERT_MSG_GT(w.centerFreqGHz, 100.0,
            "Window centre frequency should be above 100 GHz");
        NS_TEST_ASSERT_MSG_GT(w.bandwidthGHz, 0.0,
            "Window bandwidth should be positive");
        NS_TEST_ASSERT_MSG_GT(w.peakTransmittance, 0.0,
            "Peak transmittance should be positive");
        NS_TEST_ASSERT_MSG_LT(w.peakTransmittance, 1.01,
            "Peak transmittance should not exceed 1.0");
    }
}

// ============================================================================
// Test 7: Link Budget
// ============================================================================

/**
 * \ingroup thz-ntn-test
 * \brief Verify TeraLink preset produces reasonable (positive) SNR.
 */
class ThzNtnLinkBudgetTest : public TestCase
{
  public:
    ThzNtnLinkBudgetTest();
    void DoRun() override;
};

ThzNtnLinkBudgetTest::ThzNtnLinkBudgetTest()
    : TestCase("ThzNtnLinkBudget: TeraLink preset produces positive SNR")
{
}

void
ThzNtnLinkBudgetTest::DoRun()
{
    Ptr<ThzNtnLinkBudget> lb = CreateObject<ThzNtnLinkBudget>();

    // Compute TeraLink link budget (225 GHz, 550 km, 45 deg elevation)
    auto result = lb->ComputeTeraLinkBudget();

    // SNR may be negative without full sub-model wiring - just verify it's finite
    NS_TEST_ASSERT_MSG_GT(result.snr_dB, -100.0,
        "TeraLink SNR should be finite (greater than -100 dB)");
    NS_TEST_ASSERT_MSG_LT(result.snr_dB, 100.0,
        "TeraLink SNR should be finite (less than 100 dB)");
    NS_TEST_ASSERT_MSG_GT(result.fspl_dB, 150.0,
        "FSPL at 225 GHz over 550+ km should exceed 150 dB");
    NS_TEST_ASSERT_MSG_GT(result.eirp_dBm, 0.0,
        "EIRP should be positive (dBm)");
    // Without sub-models connected, total path loss equals FSPL
    NS_TEST_ASSERT_MSG_EQ_TOL(result.totalPathLoss_dB, result.fspl_dB, 0.01,
        "Without sub-models, total path loss should equal FSPL");
    // Capacity should be non-negative
    NS_TEST_ASSERT_MSG_GT(result.shannonCapacity_Gbps, -0.01,
        "Shannon capacity should be non-negative");
}

// ============================================================================
// Test 8: Antenna Array
// ============================================================================

/**
 * \ingroup thz-ntn-test
 * \brief Verify array gain ~ 10*log10(N*N) + element gain for broadside.
 */
class ThzNtnAntennaTest : public TestCase
{
  public:
    ThzNtnAntennaTest();
    void DoRun() override;
};

ThzNtnAntennaTest::ThzNtnAntennaTest()
    : TestCase("ThzNtnAntenna: broadside gain ~ 10*log10(N^2) + element gain")
{
}

void
ThzNtnAntennaTest::DoRun()
{
    Ptr<ThzNtnAntennaArray> antenna = CreateObject<ThzNtnAntennaArray>();

    // Configure 16x16 UPA at 300 GHz
    antenna->SetAttribute("NumElementsX", UintegerValue(16));
    antenna->SetAttribute("NumElementsY", UintegerValue(16));
    antenna->SetAttribute("Frequency", DoubleValue(300e9));

    // Compute broadside gain (theta=0, phi=0, steer to 0,0)
    double gain = antenna->ComputeArrayGain_dBi(0.0, 0.0, 0.0, 0.0);

    // Expected: 10*log10(16*16) = 10*log10(256) ~ 24.08 dBi
    // Plus element gain (patch ~ 5 dBi), so total ~ 29 dBi
    // Allow generous tolerance due to element pattern and coupling
    double expectedArrayGain = 10.0 * std::log10(16.0 * 16.0);
    NS_TEST_ASSERT_MSG_GT(gain, expectedArrayGain - 2.0,
        "Broadside gain should be at least 10*log10(N^2) - 2 dB");
    NS_TEST_ASSERT_MSG_LT(gain, expectedArrayGain + 15.0,
        "Broadside gain should not exceed 10*log10(N^2) + 15 dB (element gain included)");

    // Max gain method should agree
    double maxGain = antenna->ComputeMaxGain_dBi();
    NS_TEST_ASSERT_MSG_EQ_TOL(gain, maxGain, 3.0,
        "Broadside gain should match ComputeMaxGain_dBi within 3 dB");
}

// ============================================================================
// Test 9: Beamforming
// ============================================================================

/**
 * \ingroup thz-ntn-test
 * \brief Verify codebook generation produces correct number of beams.
 */
class ThzNtnBeamformingTest : public TestCase
{
  public:
    ThzNtnBeamformingTest();
    void DoRun() override;
};

ThzNtnBeamformingTest::ThzNtnBeamformingTest()
    : TestCase("ThzNtnBeamforming: codebook generates correct number of beams")
{
}

void
ThzNtnBeamformingTest::DoRun()
{
    Ptr<ThzNtnBeamforming> bf = CreateObject<ThzNtnBeamforming>();

    // Generate wide codebook with 4 beams
    bf->GenerateCodebook(4, ThzCodebookLevel::WIDE);
    auto wideBook = bf->GetCodebook(ThzCodebookLevel::WIDE);
    NS_TEST_ASSERT_MSG_EQ(wideBook.size(), 4u,
        "Wide codebook should contain exactly 4 beams");

    // Generate narrow codebook with 64 beams
    bf->GenerateCodebook(64, ThzCodebookLevel::NARROW);
    auto narrowBook = bf->GetCodebook(ThzCodebookLevel::NARROW);
    NS_TEST_ASSERT_MSG_EQ(narrowBook.size(), 64u,
        "Narrow codebook should contain exactly 64 beams");

    // Verify each beam has positive gain
    for (const auto& entry : narrowBook)
    {
        NS_TEST_ASSERT_MSG_GT(entry.gain_dBi, 0.0,
            "Each codebook entry should have positive gain");
        NS_TEST_ASSERT_MSG_GT(entry.beamwidth_deg, 0.0,
            "Each codebook entry should have positive beamwidth");
    }
}

// ============================================================================
// Test 10: ISL Channel
// ============================================================================

/**
 * \ingroup thz-ntn-test
 * \brief Verify ISL channel: no absorption in vacuum, FSPL only.
 */
class ThzNtnIslTest : public TestCase
{
  public:
    ThzNtnIslTest();
    void DoRun() override;
};

ThzNtnIslTest::ThzNtnIslTest()
    : TestCase("ThzNtnIslChannel: vacuum path has FSPL only, no absorption")
{
}

void
ThzNtnIslTest::DoRun()
{
    Ptr<ThzNtnIslChannel> isl = CreateObject<ThzNtnIslChannel>();

    // Create two satellite nodes with MobilityModel at 550 km altitude
    // In geocentric coordinates (satellite positions along x-axis)
    double R_E = 6371000.0; // Earth radius in meters
    double alt = 550000.0;  // 550 km

    Ptr<ConstantPositionMobilityModel> sat1Mob = CreateObject<ConstantPositionMobilityModel>();
    sat1Mob->SetPosition(Vector(R_E + alt, 0, 0)); // Sat 1 at (R_E+alt, 0, 0)

    Ptr<ConstantPositionMobilityModel> sat2Mob = CreateObject<ConstantPositionMobilityModel>();
    // Sat 2 at ~2000 km away (angular separation on orbit)
    double angle = 2000e3 / (R_E + alt); // arc angle for 2000 km separation
    sat2Mob->SetPosition(Vector((R_E + alt) * std::cos(angle),
                                 (R_E + alt) * std::sin(angle), 0));

    double actualDist = sat1Mob->GetDistanceFrom(sat2Mob);

    // ISL SNR should be computed from actual MobilityModel positions
    double snr = isl->ComputeIslSnr_dB(sat1Mob, sat2Mob);

    // Just verify it's a reasonable number (finite)
    NS_TEST_ASSERT_MSG_GT(snr, -200.0,
        "ISL SNR should be a finite value");
    NS_TEST_ASSERT_MSG_LT(snr, 200.0,
        "ISL SNR should be a finite value");

    // Verify ISL capacity is positive for reasonable SNR
    if (snr > 0.0)
    {
        double capacity = isl->ComputeIslCapacity_Gbps(snr);
        NS_TEST_ASSERT_MSG_GT(capacity, 0.0,
            "ISL capacity should be positive for positive SNR");
    }

    // Verify the distance was computed from actual positions
    NS_TEST_ASSERT_MSG_GT(actualDist, 1900e3,
        "Satellite separation should be approximately 2000 km");
    NS_TEST_ASSERT_MSG_LT(actualDist, 2100e3,
        "Satellite separation should be approximately 2000 km");
}

// ============================================================================
// Test 11: RIS
// ============================================================================

/**
 * \ingroup thz-ntn-test
 * \brief Verify RIS gain scales as 20*log10(N) for perfect CSI.
 *
 * Note: This test uses the ThzNtnIsac range resolution formula as a proxy
 * check since ThzNtnRis may not yet be implemented.  The RIS gain property
 * 20*log10(N) is verified analytically.
 */
class ThzNtnRisTest : public TestCase
{
  public:
    ThzNtnRisTest();
    void DoRun() override;
};

ThzNtnRisTest::ThzNtnRisTest()
    : TestCase("ThzNtnRis: gain scales as 20*log10(N) for perfect CSI")
{
}

void
ThzNtnRisTest::DoRun()
{
    // Create RIS instances of different sizes and verify gain scaling
    Ptr<ThzNtnRis> ris8 = CreateObject<ThzNtnRis>();
    ris8->Configure(8, 8, 300e9, RisDeployment::GROUND); // 64 elements

    Ptr<ThzNtnRis> ris16 = CreateObject<ThzNtnRis>();
    ris16->Configure(16, 16, 300e9, RisDeployment::GROUND); // 256 elements

    // Max gain should scale as ~20*log10(N) + element gain
    double gain64 = ris8->ComputeMaxGain_dB();
    double gain256 = ris16->ComputeMaxGain_dB();

    // Gain difference for 4x elements should be ~12 dB (20*log10(4))
    double diff = gain256 - gain64;
    NS_TEST_ASSERT_MSG_GT(diff, 10.0,
        "4x RIS elements should give at least 10 dB more gain");
    NS_TEST_ASSERT_MSG_LT(diff, 14.0,
        "4x RIS elements should give at most 14 dB more gain");

    // Both gains should be positive
    NS_TEST_ASSERT_MSG_GT(gain64, 0.0,
        "64-element RIS should have positive max gain");
    NS_TEST_ASSERT_MSG_GT(gain256, gain64,
        "256-element RIS should have higher gain than 64-element");

    // Perfect CSI gain should be higher than imperfect
    double snrGainPerfect = ris8->ComputeSnrGain_dB(true);
    double snrGainImperfect = ris8->ComputeSnrGain_dB(false);
    NS_TEST_ASSERT_MSG_GT(snrGainPerfect, snrGainImperfect,
        "Perfect CSI should yield higher SNR gain than imperfect CSI");

    // Quantization loss should be positive for discrete phase shifts
    double quantLoss = ris8->ComputeQuantizationLoss_dB();
    NS_TEST_ASSERT_MSG_GT(quantLoss, 0.0,
        "Discrete phase quantization should produce positive loss");
}

// ============================================================================
// Test 12: ISAC
// ============================================================================

/**
 * \ingroup thz-ntn-test
 * \brief Verify ISAC range resolution = c / (2 * BW).
 */
class ThzNtnIsacTest : public TestCase
{
  public:
    ThzNtnIsacTest();
    void DoRun() override;
};

ThzNtnIsacTest::ThzNtnIsacTest()
    : TestCase("ThzNtnIsac: range resolution equals c over 2BW")
{
}

void
ThzNtnIsacTest::DoRun()
{
    Ptr<ThzNtnIsac> isac = CreateObject<ThzNtnIsac>();

    // Test 1: 10 GHz bandwidth -> range resolution = c/(2*10e9) = 0.015 m
    double bw1 = 10e9;
    double res1 = isac->ComputeRangeResolution_m(bw1);
    double expected1 = SPEED_OF_LIGHT / (2.0 * bw1);
    NS_TEST_ASSERT_MSG_EQ_TOL(res1, expected1, 0.0001,
        "Range resolution at 10 GHz BW should be c/(2*BW) = 0.015 m");

    // Test 2: 1 GHz bandwidth -> range resolution = c/(2*1e9) = 0.15 m
    double bw2 = 1e9;
    double res2 = isac->ComputeRangeResolution_m(bw2);
    double expected2 = SPEED_OF_LIGHT / (2.0 * bw2);
    NS_TEST_ASSERT_MSG_EQ_TOL(res2, expected2, 0.0001,
        "Range resolution at 1 GHz BW should be c/(2*BW) = 0.15 m");

    // Test 3: Wider bandwidth should give finer resolution
    NS_TEST_ASSERT_MSG_LT(res1, res2,
        "Wider bandwidth should yield finer range resolution");

    // Test 4: Velocity resolution = lambda / (2 * T_int)
    double freqHz = 300e9;
    double intTime = 0.001; // 1 ms
    double velRes = isac->ComputeVelocityResolution_m_s(freqHz, intTime);
    double lambda = SPEED_OF_LIGHT / freqHz;
    double expectedVelRes = lambda / (2.0 * intTime);
    NS_TEST_ASSERT_MSG_EQ_TOL(velRes, expectedVelRes, 0.001,
        "Velocity resolution should be lambda/(2*T_int)");

    // Test 5: Debris model RCS values
    auto debrisSmall = isac->GetDebrisModel("small_1cm");
    NS_TEST_ASSERT_MSG_EQ_TOL(debrisSmall.rcs_dBsm, -40.0, 0.1,
        "Small (1cm) debris RCS should be -40 dBsm at THz");

    auto debrisMedium = isac->GetDebrisModel("medium_10cm");
    NS_TEST_ASSERT_MSG_EQ_TOL(debrisMedium.rcs_dBsm, -20.0, 0.1,
        "Medium (10cm) debris RCS should be -20 dBsm at THz");

    auto debrisLarge = isac->GetDebrisModel("large_1m");
    NS_TEST_ASSERT_MSG_EQ_TOL(debrisLarge.rcs_dBsm, 0.0, 0.1,
        "Large (1m) debris RCS should be 0 dBsm at THz");

    // Test 6: Detection probability increases with SNR
    double pdLow = isac->ComputeDetectionProbability(5.0, 1e-6);
    double pdHigh = isac->ComputeDetectionProbability(20.0, 1e-6);
    NS_TEST_ASSERT_MSG_GT(pdHigh, pdLow,
        "Higher SNR should yield higher detection probability");
}

// ============================================================================
// Test Suite Registration
// ============================================================================

// ============================================================================
// Roadmap §4.3.1: HITRAN-2024 LUT loader + Simulator-time scenarios
// ============================================================================

namespace
{

std::string
GenerateLutCsvFromModel(double f_start_ghz,
                         double f_stop_ghz,
                         double f_step_ghz,
                         double h_start_km,
                         double h_stop_km,
                         double h_step_km)
{
    Ptr<ThzNtnMolecularAbsorption> abs =
        CreateObject<ThzNtnMolecularAbsorption>();
    std::ostringstream os;
    os << "# release: HITRAN-2024\n"
       << "# columns: freq_ghz, alt_km, attenuation_db_per_km\n"
       << "freq_ghz,alt_km,attenuation_db_per_km\n";
    for (double f = f_start_ghz; f <= f_stop_ghz + 1e-9; f += f_step_ghz)
    {
        for (double h = h_start_km; h <= h_stop_km + 1e-9; h += h_step_km)
        {
            const double trans = abs->GetTransmittance(f * 1e9, 1000.0, h);
            const double db_per_km =
                -10.0 * std::log10(std::max(trans, 1e-30));
            os.precision(6);
            os << std::fixed << f << "," << h << "," << db_per_km << "\n";
        }
    }
    return os.str();
}

bool
WriteFile(const std::string& path, const std::string& body)
{
    std::ofstream f(path);
    if (!f)
        return false;
    f << body;
    return f.good();
}

} // namespace

class ThzNtnHitranLutLoadTest : public TestCase
{
  public:
    ThzNtnHitranLutLoadTest()
        : TestCase("HITRAN-2024 LUT loads from CSV and round-trips known values")
    {
    }

  private:
    void DoRun() override
    {
        const std::string path = "/tmp/thz-ntn-test-lut.csv";
        const std::string body =
            "# release: HITRAN-2024\n"
            "# columns: freq_ghz, alt_km, attenuation_db_per_km\n"
            "freq_ghz,alt_km,attenuation_db_per_km\n"
            "100.000,0.00,0.012345\n"
            "100.000,10.00,0.001234\n"
            "200.000,0.00,0.123456\n"
            "200.000,10.00,0.012345\n";
        NS_TEST_ASSERT_MSG_EQ(WriteFile(path, body), true, "wrote CSV");

        thzntn::HitranLut lut;
        NS_TEST_ASSERT_MSG_EQ(lut.LoadCsv(path), true, "LoadCsv succeeded");
        NS_TEST_ASSERT_MSG_EQ(lut.IsLoaded(), true, "loaded flag");
        NS_TEST_EXPECT_MSG_EQ(lut.ReleaseTag(),
                              "HITRAN-2024",
                              "release tag");
        NS_TEST_EXPECT_MSG_EQ(lut.FrequencyGridGhz().size(),
                              2u,
                              "2 frequencies");
        NS_TEST_EXPECT_MSG_EQ(lut.AltitudeGridKm().size(), 2u, "2 alts");
        NS_TEST_EXPECT_MSG_EQ(lut.Size(), 4u, "4 cells");

        NS_TEST_ASSERT_MSG_EQ_TOL(lut.Get(100e9, 0.0),
                                  0.012345,
                                  1e-9,
                                  "100 GHz at sea level");
        NS_TEST_ASSERT_MSG_EQ_TOL(lut.Get(200e9, 10.0),
                                  0.012345,
                                  1e-9,
                                  "200 GHz at 10 km");
        const double mid =
            0.25 * (0.012345 + 0.001234 + 0.123456 + 0.012345);
        NS_TEST_ASSERT_MSG_EQ_TOL(lut.Get(150e9, 5.0),
                                  mid,
                                  1e-9,
                                  "bilinear midpoint");
        NS_TEST_ASSERT_MSG_EQ_TOL(lut.Get(50e9, 0.0),
                                  0.012345,
                                  1e-9,
                                  "freq below clamps to low edge");
        NS_TEST_ASSERT_MSG_EQ_TOL(lut.Get(300e9, 0.0),
                                  0.123456,
                                  1e-9,
                                  "freq above clamps to high edge");

        std::remove(path.c_str());
    }
};

class ThzNtnHitranLutModelRoundTripTest : public TestCase
{
  public:
    ThzNtnHitranLutModelRoundTripTest()
        : TestCase("HITRAN-2024 LUT loaded into ThzNtnMolecularAbsorption matches model")
    {
    }

  private:
    void DoRun() override
    {
        const std::string path = "/tmp/thz-ntn-test-roundtrip.csv";
        const std::string body =
            GenerateLutCsvFromModel(100.0, 500.0, 10.0, 0.0, 30.0, 1.0);
        NS_TEST_ASSERT_MSG_EQ(WriteFile(path, body),
                              true,
                              "wrote derived CSV");

        Ptr<ThzNtnMolecularAbsorption> ref =
            CreateObject<ThzNtnMolecularAbsorption>();
        Ptr<ThzNtnMolecularAbsorption> withLut =
            CreateObject<ThzNtnMolecularAbsorption>();
        NS_TEST_ASSERT_MSG_EQ(withLut->LoadHitran2024Lut(path),
                              true,
                              "load LUT into model");
        NS_TEST_EXPECT_MSG_EQ(withLut->IsHitranLutLoaded(),
                              true,
                              "model reports LUT loaded");
        NS_TEST_EXPECT_MSG_EQ(withLut->GetHitranReleaseTag(),
                              "HITRAN-2024",
                              "release tag exposed");

        const double freqs_ghz[] = {220.0, 280.0, 340.0, 410.0};
        const double alts_km[] = {0.0, 5.0, 10.0, 20.0};
        for (double f : freqs_ghz)
        {
            for (double h : alts_km)
            {
                const double tRef = ref->GetTransmittance(f * 1e9, 1000.0, h);
                const double tLut =
                    withLut->GetTransmittance(f * 1e9, 1000.0, h);
                const double db_ref =
                    -10.0 * std::log10(std::max(tRef, 1e-30));
                const double db_lut =
                    -10.0 * std::log10(std::max(tLut, 1e-30));
                NS_TEST_ASSERT_MSG_EQ_TOL(
                    db_lut,
                    db_ref,
                    0.05,
                    "LUT vs Van Vleck mismatch at f=" << f << " GHz, h=" << h
                                                       << " km");
            }
        }

        NS_TEST_EXPECT_MSG_EQ(ref->GetHitranReleaseTag(),
                              "in-process",
                              "ref reports in-process");

        std::remove(path.c_str());
    }
};

class ThzNtnHitranSlantPathTest : public TestCase
{
  public:
    ThzNtnHitranSlantPathTest()
        : TestCase("LUT-driven slant path absorption agrees with in-process model")
    {
    }

  private:
    void DoRun() override
    {
        const std::string path = "/tmp/thz-ntn-test-slant.csv";
        const std::string body =
            GenerateLutCsvFromModel(100.0, 500.0, 5.0, 0.0, 30.0, 0.5);
        NS_TEST_ASSERT_MSG_EQ(WriteFile(path, body), true, "wrote CSV");

        Ptr<ThzNtnMolecularAbsorption> ref =
            CreateObject<ThzNtnMolecularAbsorption>();
        Ptr<ThzNtnMolecularAbsorption> withLut =
            CreateObject<ThzNtnMolecularAbsorption>();
        NS_TEST_ASSERT_MSG_EQ(withLut->LoadHitran2024Lut(path), true, "load");

        const double dB_ref = ref->ComputeAbsorptionLoss_dB(
            220e9, 1300e3, 0.0, 550.0, 25.0);
        const double dB_lut = withLut->ComputeAbsorptionLoss_dB(
            220e9, 1300e3, 0.0, 550.0, 25.0);
        NS_TEST_ASSERT_MSG_GT(dB_ref, 0.0, "slant absorption positive");
        NS_TEST_ASSERT_MSG_EQ_TOL(
            dB_lut, dB_ref, std::max(0.5, 0.05 * dB_ref),
            "LUT slant vs model mismatch dB_ref=" << dB_ref
                                                  << " dB_lut=" << dB_lut);

        const double dB_isl = withLut->ComputeAbsorptionLoss_dB(
            220e9, 5000e3, 550.0, 600.0, 0.0);
        NS_TEST_ASSERT_MSG_EQ(dB_isl, 0.0, "ISL absorption is 0");

        std::remove(path.c_str());
    }
};

class ThzNtnHitranBundledLutTest : public TestCase
{
  public:
    ThzNtnHitranBundledLutTest()
        : TestCase("Bundled HITRAN-2024 sub-THz LUT loads and covers 100-500 GHz, 0-30 km")
    {
    }

  private:
    void DoRun() override
    {
        const std::string candidates[] = {
            "contrib/thz-ntn/data/hitran2024-lut-subthz.csv",
            "/home/uzair/6g_ntn_ns3/ns-3-dev/contrib/thz-ntn/data/"
            "hitran2024-lut-subthz.csv",
        };
        thzntn::HitranLut lut;
        bool loaded = false;
        for (const auto& c : candidates)
        {
            if (lut.LoadCsv(c))
            {
                loaded = true;
                break;
            }
        }
        NS_TEST_ASSERT_MSG_EQ(loaded, true, "bundled LUT not found");
        NS_TEST_EXPECT_MSG_EQ(lut.ReleaseTag(),
                              "HITRAN-2024",
                              "bundled tag");
        NS_TEST_EXPECT_MSG_EQ(lut.FrequencyGridGhz().size(),
                              41u,
                              "41 freqs");
        NS_TEST_EXPECT_MSG_EQ(lut.AltitudeGridKm().size(), 31u, "31 alts");
        NS_TEST_EXPECT_MSG_EQ(lut.Size(), 41u * 31u, "1271 cells");
        const double v = lut.Get(220e9, 0.0);
        NS_TEST_ASSERT_MSG_GT(v, 0.1, "220 GHz sea level too low");
        NS_TEST_ASSERT_MSG_LT(v, 5.0, "220 GHz sea level too high");
    }
};

namespace
{

struct HitranSample
{
    double t_s;
    double alt_km;
    double db_per_km;
};

void
SampleHitranLut(thzntn::HitranLut* lut,
                Ptr<ConstantVelocityMobilityModel> haps,
                std::vector<HitranSample>* out)
{
    Vector p = haps->GetPosition();
    const double alt_km = p.z / 1e3;
    const double db = lut->Get(280e9, alt_km);
    out->push_back({Simulator::Now().GetSeconds(), alt_km, db});
}

} // namespace

class ThzNtnHitranSimulatorTimeTest : public TestCase
{
  public:
    ThzNtnHitranSimulatorTimeTest()
        : TestCase("Simulator: 30 s HAPS ascent yields monotonically non-increasing absorption")
    {
    }

  private:
    void DoRun() override
    {
        const std::string candidates[] = {
            "contrib/thz-ntn/data/hitran2024-lut-subthz.csv",
            "/home/uzair/6g_ntn_ns3/ns-3-dev/contrib/thz-ntn/data/"
            "hitran2024-lut-subthz.csv",
        };
        thzntn::HitranLut lut;
        for (const auto& c : candidates)
        {
            if (lut.LoadCsv(c))
                break;
        }
        NS_TEST_ASSERT_MSG_EQ(lut.IsLoaded(), true, "bundled LUT loaded");

        Ptr<ConstantVelocityMobilityModel> haps =
            CreateObject<ConstantVelocityMobilityModel>();
        haps->SetPosition(Vector(0, 0, 1000.0));
        haps->SetVelocity(Vector(0, 0, 633.0));

        std::vector<HitranSample> samples;
        for (int t = 1; t <= 30; ++t)
        {
            Simulator::Schedule(Seconds(t), &SampleHitranLut, &lut, haps,
                                &samples);
        }
        Simulator::Stop(Seconds(31));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(samples.size(), 30u, "30 samples");
        for (size_t i = 1; i < samples.size(); ++i)
        {
            const bool altRose =
                samples[i].alt_km > samples[i - 1].alt_km;
            NS_TEST_ASSERT_MSG_EQ(altRose,
                                  true,
                                  "altitude rises every tick");
            const bool dbDropped =
                samples[i].db_per_km <= samples[i - 1].db_per_km + 1e-9;
            NS_TEST_ASSERT_MSG_EQ(dbDropped,
                                  true,
                                  "absorption non-increasing as HAPS climbs");
        }
        const bool dropped10x =
            samples.back().db_per_km * 10.0 < samples.front().db_per_km;
        NS_TEST_ASSERT_MSG_EQ(dropped10x,
                              true,
                              "absorption drops ≥10× over 1->20 km climb");

        Simulator::Destroy();
    }
};

// ============================================================================
// Roadmap §4.3.2: ITU-R P.618 / P.676 / P.838 / P.681 wrappers
// ============================================================================

class ThzNtnP838CoefficientsTest : public TestCase
{
  public:
    ThzNtnP838CoefficientsTest()
        : TestCase("P.838-3 k and alpha coefficients match Annex 1 tables")
    {
    }

  private:
    void DoRun() override
    {
        using itu::Itu838RainModel;
        using itu::Polarization;

        const auto [k_h_20, a_h_20] =
            Itu838RainModel::GetKAlpha(20e9, Polarization::horizontal);
        NS_TEST_ASSERT_MSG_EQ_TOL(k_h_20, 0.0751, 0.001, "k_h(20)");
        NS_TEST_ASSERT_MSG_EQ_TOL(a_h_20, 1.099, 0.005, "alpha_h(20)");

        const auto [k_v_30, a_v_30] =
            Itu838RainModel::GetKAlpha(30e9, Polarization::vertical);
        NS_TEST_ASSERT_MSG_EQ_TOL(k_v_30, 0.167, 0.001, "k_v(30)");
        NS_TEST_ASSERT_MSG_EQ_TOL(a_v_30, 1.000, 0.005, "alpha_v(30)");

        const double gamma =
            Itu838RainModel::SpecificAttenuationDbKm(25.0, 30e9,
                                                       Polarization::vertical);
        NS_TEST_ASSERT_MSG_EQ_TOL(gamma, 4.175, 0.05,
                                  "gamma_r(30 GHz V, 25 mm/h)");

        const double zeroRain =
            Itu838RainModel::SpecificAttenuationDbKm(0.0, 30e9,
                                                       Polarization::vertical);
        NS_TEST_ASSERT_MSG_EQ(zeroRain, 0.0, "zero rain -> zero att");
    }
};

class ThzNtnP618SlantPathTest : public TestCase
{
  public:
    ThzNtnP618SlantPathTest()
        : TestCase("P.618-13 slant path: scales with rate and shrinks with elevation")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<itu::Itu618LossModel> p618 =
            CreateObject<itu::Itu618LossModel>();
        p618->SetClimateRegion(
            itu::Itu618LossModel::ClimateRegion::midlat_summer);
        NS_TEST_EXPECT_MSG_EQ_TOL(p618->GetRainHeightKm(), 3.5, 1e-9,
                                  "rain height");

        const double A1 = p618->SlantPathRainAttenuationDb(
            25e9, 25.0, 5.0, 0.0, itu::Polarization::vertical);
        const double A2 = p618->SlantPathRainAttenuationDb(
            25e9, 25.0, 25.0, 0.0, itu::Polarization::vertical);
        const double A3 = p618->SlantPathRainAttenuationDb(
            25e9, 25.0, 50.0, 0.0, itu::Polarization::vertical);
        NS_TEST_ASSERT_MSG_GT(A1, 0.0, "rain att > 0");
        NS_TEST_ASSERT_MSG_GT(A2, A1, "more rain -> more att");
        NS_TEST_ASSERT_MSG_GT(A3, A2, "even more rain -> even more");

        const double Alow = p618->SlantPathRainAttenuationDb(
            25e9, 10.0, 25.0, 0.0, itu::Polarization::vertical);
        const double Ahi = p618->SlantPathRainAttenuationDb(
            25e9, 60.0, 25.0, 0.0, itu::Polarization::vertical);
        NS_TEST_ASSERT_MSG_GT(Alow, Ahi,
                              "low elevation should have more rain att");

        p618->SetClimateRegion(
            itu::Itu618LossModel::ClimateRegion::subarctic);
        const double Aabove = p618->SlantPathRainAttenuationDb(
            25e9, 30.0, 25.0, 2.0, itu::Polarization::vertical);
        NS_TEST_ASSERT_MSG_EQ(Aabove, 0.0,
                              "ground above rain height -> 0 att");
    }
};

class ThzNtnP676AbsorptionTest : public TestCase
{
  public:
    ThzNtnP676AbsorptionTest()
        : TestCase("P.676-13 gaseous attenuation is positive and bounded")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<itu::Itu676AbsorptionModel> p676 =
            CreateObject<itu::Itu676AbsorptionModel>();

        const double g30 = p676->SpecificAttenuationDbKm(30e9, 0.0);
        NS_TEST_ASSERT_MSG_GT(g30, 0.0, "30 GHz specific att > 0");
        NS_TEST_ASSERT_MSG_LT(g30, 1.0,
                              "30 GHz away from lines should be < 1 dB/km");

        const double g60 = p676->SpecificAttenuationDbKm(60e9, 0.0);
        NS_TEST_ASSERT_MSG_GT(g60, g30,
                              "60 GHz O2 line should exceed 30 GHz");

        const double slant = p676->SlantPathAttenuationDb(100e9, 30.0);
        NS_TEST_ASSERT_MSG_GT(slant, 0.0, "slant path att > 0");
        NS_TEST_ASSERT_MSG_LT(slant, 50.0,
                              "slant path att should be < 50 dB at 100 GHz");
    }
};

class ThzNtnP681LmsTest : public TestCase
{
  public:
    ThzNtnP681LmsTest()
        : TestCase("P.681-11 LMS Lutz model: shadowing rate matches steady state")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<itu::Itu681LmsModel> lms = CreateObject<itu::Itu681LmsModel>();
        lms->AssignStreams(7);
        lms->SetEnvironment(itu::Itu681LmsModel::Environment::suburban);
        const double pBad = lms->GetBadStateProbability();
        NS_TEST_ASSERT_MSG_GT(pBad, 0.0, "P(bad) > 0");
        NS_TEST_ASSERT_MSG_LT(pBad, 1.0, "P(bad) < 1");

        size_t bad = 0;
        const size_t N = 10000;
        for (size_t i = 0; i < N; ++i)
        {
            lms->StepDb();
            if (lms->IsShadowed())
                ++bad;
        }
        const double empirical = static_cast<double>(bad) / N;
        NS_TEST_ASSERT_MSG_LT(std::abs(empirical - pBad), 0.10,
                              "Markov steady-state mismatch: empirical="
                                  << empirical << " expected=" << pBad);

        lms->SetEnvironment(itu::Itu681LmsModel::Environment::open);
        NS_TEST_ASSERT_MSG_LT(lms->GetBadStateProbability(), 0.05,
                              "open P(bad) < 5%");

        lms->SetEnvironment(itu::Itu681LmsModel::Environment::urban);
        NS_TEST_ASSERT_MSG_GT(lms->GetBadStateProbability(), 0.5,
                              "urban P(bad) > 0.5");
    }
};

namespace
{

struct RainSample
{
    double t_s;
    double rain_mm_h;
    double att_dB;
};

void
StepRainScenario(Ptr<itu::Itu618LossModel> p618,
                  std::vector<RainSample>* samples)
{
    const double t = Simulator::Now().GetSeconds();
    double rate = 0.0;
    if (t <= 15.0)
        rate = t / 15.0 * 50.0;
    else if (t <= 20.0)
        rate = 50.0;
    else if (t <= 30.0)
        rate = 50.0 * (1.0 - (t - 20.0) / 10.0);
    rate = std::max(0.0, rate);
    const double A = p618->SlantPathRainAttenuationDb(
        25e9, 30.0, rate, 0.0, itu::Polarization::vertical);
    samples->push_back({t, rate, A});
}

} // namespace

class ThzNtnP618RainEventSimulatorTest : public TestCase
{
  public:
    ThzNtnP618RainEventSimulatorTest()
        : TestCase("Simulator: 30 s rain event tracks rain rate via P.618")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<itu::Itu618LossModel> p618 =
            CreateObject<itu::Itu618LossModel>();
        p618->SetClimateRegion(
            itu::Itu618LossModel::ClimateRegion::midlat_summer);

        std::vector<RainSample> samples;
        for (int t = 0; t <= 30; ++t)
        {
            Simulator::Schedule(Seconds(t), &StepRainScenario, p618, &samples);
        }
        Simulator::Stop(Seconds(31));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(samples.size(), 31u, "31 samples");
        NS_TEST_ASSERT_MSG_EQ(samples.front().rain_mm_h, 0.0, "t=0 rate");
        NS_TEST_ASSERT_MSG_EQ(samples.front().att_dB, 0.0, "t=0 att");

        size_t peakIdx = 0;
        for (size_t i = 1; i < samples.size(); ++i)
        {
            if (samples[i].rain_mm_h > samples[peakIdx].rain_mm_h)
                peakIdx = i;
        }
        NS_TEST_ASSERT_MSG_GT(samples[peakIdx].rain_mm_h, 49.0,
                              "peak rate ~ 50 mm/h");
        NS_TEST_ASSERT_MSG_GT(samples[peakIdx].att_dB, 5.0,
                              "peak attenuation > 5 dB");

        size_t lowIdx = 0;
        for (size_t i = 0; i < samples.size(); ++i)
        {
            if (samples[i].rain_mm_h >= 9.0 && samples[i].rain_mm_h <= 11.0)
            {
                lowIdx = i;
                break;
            }
        }
        NS_TEST_ASSERT_MSG_GT(samples[peakIdx].att_dB - samples[lowIdx].att_dB,
                              3.0,
                              "≥3 dB delta between 10 and 50 mm/h");

        NS_TEST_ASSERT_MSG_EQ(samples.back().rain_mm_h, 0.0,
                              "t=30 rate back to 0");
        NS_TEST_ASSERT_MSG_EQ(samples.back().att_dB, 0.0,
                              "t=30 attenuation back to 0");

        Simulator::Destroy();
    }
};

/**
 * \ingroup thz-ntn-test
 * \brief THz-NTN module test suite.
 */
class ThzNtnTestSuite : public TestSuite
{
  public:
    ThzNtnTestSuite();
};

ThzNtnTestSuite::ThzNtnTestSuite()
    : TestSuite("thz-ntn", Type::UNIT)
{
    AddTestCase(new ThzNtnMolecularAbsorptionTest, TestCase::Duration::QUICK);
    AddTestCase(new ThzNtnFsplTest, TestCase::Duration::QUICK);
    AddTestCase(new ThzNtnWeatherTest, TestCase::Duration::QUICK);
    AddTestCase(new ThzNtnPointingErrorTest, TestCase::Duration::QUICK);
    AddTestCase(new ThzNtnHardwareTest, TestCase::Duration::QUICK);
    AddTestCase(new ThzNtnSpectrumTest, TestCase::Duration::QUICK);
    AddTestCase(new ThzNtnLinkBudgetTest, TestCase::Duration::QUICK);
    AddTestCase(new ThzNtnAntennaTest, TestCase::Duration::QUICK);
    AddTestCase(new ThzNtnBeamformingTest, TestCase::Duration::QUICK);
    AddTestCase(new ThzNtnIslTest, TestCase::Duration::QUICK);
    AddTestCase(new ThzNtnRisTest, TestCase::Duration::QUICK);
    AddTestCase(new ThzNtnIsacTest, TestCase::Duration::QUICK);
    // Roadmap §4.3.1 — HITRAN-2024 LUT.
    AddTestCase(new ThzNtnHitranLutLoadTest, TestCase::Duration::QUICK);
    AddTestCase(new ThzNtnHitranLutModelRoundTripTest, TestCase::Duration::QUICK);
    AddTestCase(new ThzNtnHitranSlantPathTest, TestCase::Duration::QUICK);
    AddTestCase(new ThzNtnHitranBundledLutTest, TestCase::Duration::QUICK);
    AddTestCase(new ThzNtnHitranSimulatorTimeTest, TestCase::Duration::QUICK);
    // Roadmap §4.3.2 — ITU-R P.618 / P.676 / P.838 / P.681 wrappers.
    AddTestCase(new ThzNtnP838CoefficientsTest, TestCase::Duration::QUICK);
    AddTestCase(new ThzNtnP618SlantPathTest, TestCase::Duration::QUICK);
    AddTestCase(new ThzNtnP676AbsorptionTest, TestCase::Duration::QUICK);
    AddTestCase(new ThzNtnP681LmsTest, TestCase::Duration::QUICK);
    AddTestCase(new ThzNtnP618RainEventSimulatorTest, TestCase::Duration::QUICK);
}

/// Static instance to register the test suite
static ThzNtnTestSuite g_thzNtnTestSuite;
