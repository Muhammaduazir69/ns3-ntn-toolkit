/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * THz-NTN Comprehensive Demo + Dataset Generation
 *
 * Runs 8 scenarios, prints results to console, and writes CSV datasets
 * to ./thz-ntn-results/ for further analysis and plotting.
 *
 * All values computed from actual sub-models with proper wiring.
 */

#include <ns3/command-line.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/constant-velocity-mobility-model.h>
#include <ns3/core-module.h>
#include <ns3/double.h>
#include <ns3/mobility-model.h>
#include <ns3/simulator.h>

#include <ns3/thz-ntn-antenna-array.h>
#include <ns3/thz-ntn-beam-tracking.h>
#include <ns3/thz-ntn-beamforming.h>
#include <ns3/thz-ntn-channel-model.h>
#include <ns3/thz-ntn-free-space-loss.h>
#include <ns3/thz-ntn-hardware-impairments.h>
#include <ns3/thz-ntn-isac.h>
#include <ns3/thz-ntn-isl-channel.h>
#include <ns3/thz-ntn-link-budget.h>
#include <ns3/thz-ntn-molecular-absorption.h>
#include <ns3/thz-ntn-pointing-error.h>
#include <ns3/thz-ntn-ris.h>
#include <ns3/thz-ntn-scintillation.h>
#include <ns3/thz-ntn-spectrum.h>
#include <ns3/thz-ntn-weather-attenuation.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace ns3;

static constexpr double R_E = 6371000.0;
static constexpr double SPEED_OF_LIGHT = 299792458.0;
static const std::string OUT_DIR = "thz-ntn-results";

// ----------------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------------

static Ptr<MobilityModel>
CreateGroundNode(double lat_deg = 0.0, double lon_deg = 0.0)
{
    Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel>();
    double lat = lat_deg * M_PI / 180.0;
    double lon = lon_deg * M_PI / 180.0;
    mob->SetPosition(Vector(R_E * std::cos(lat) * std::cos(lon),
                             R_E * std::cos(lat) * std::sin(lon),
                             R_E * std::sin(lat)));
    return mob;
}

static Ptr<MobilityModel>
CreateSatellite(double alt_km, double theta_deg, double phi_deg = 0.0)
{
    Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel>();
    double r = R_E + alt_km * 1000.0;
    double theta = theta_deg * M_PI / 180.0;
    double phi = phi_deg * M_PI / 180.0;
    mob->SetPosition(Vector(r * std::cos(theta) * std::cos(phi),
                             r * std::cos(theta) * std::sin(phi),
                             r * std::sin(theta)));
    return mob;
}

static Ptr<ThzNtnLinkBudget>
CreateFullLinkBudget(double freqHz)
{
    Ptr<ThzNtnMolecularAbsorption> abs = CreateObject<ThzNtnMolecularAbsorption>();
    Ptr<ThzNtnWeatherAttenuation> weather = CreateObject<ThzNtnWeatherAttenuation>();
    Ptr<ThzNtnScintillation> scint = CreateObject<ThzNtnScintillation>();
    Ptr<ThzNtnPointingError> pointing = CreateObject<ThzNtnPointingError>();
    Ptr<ThzNtnHardwareImpairments> hw = CreateObject<ThzNtnHardwareImpairments>();

    Ptr<ThzNtnFreeSpaceLoss> fsl = CreateObject<ThzNtnFreeSpaceLoss>();
    fsl->SetMolecularAbsorptionModel(abs);
    fsl->SetWeatherModel(weather);
    fsl->SetScintillationModel(scint);
    fsl->SetPointingErrorModel(pointing);
    fsl->EnableMolecularAbsorption(true);
    fsl->EnableWeatherEffects(true);
    fsl->EnableScintillation(true);
    fsl->EnablePointingError(true);

    Ptr<ThzNtnLinkBudget> lb = CreateObject<ThzNtnLinkBudget>();
    lb->SetAttribute("DefaultFrequency", DoubleValue(freqHz));
    lb->SetFreeSpaceLossModel(fsl);
    lb->SetHardwareModel(hw);
    lb->SetMolecularAbsorptionModel(abs);
    lb->SetWeatherModel(weather);
    lb->SetScintillationModel(scint);
    lb->SetPointingErrorModel(pointing);
    return lb;
}

static void
EnsureOutDir()
{
    std::filesystem::create_directories(OUT_DIR);
}

// ============================================================================
// Example 1: LEO-Ground TeraLink @ 225 GHz (elevation sweep)
// ============================================================================
static void
Example1_LeoGround()
{
    std::cout << "\n================================================================\n";
    std::cout << "  Example 1: LEO-Ground TeraLink @ 225 GHz (elevation sweep)\n";
    std::cout << "================================================================\n";

    Ptr<ThzNtnLinkBudget> lb = CreateFullLinkBudget(225e9);
    Ptr<MobilityModel> ground = CreateGroundNode();

    std::ofstream csv(OUT_DIR + "/01_leo_ground_sweep.csv");
    csv << "elevation_deg,distance_km,fspl_dB,absorption_dB,weather_dB,scint_dB,"
        << "pointing_dB,total_loss_dB,rx_power_dBm,snr_dB,hw_snr_dB,"
        << "shannon_Gbps,hw_capacity_Gbps,link_margin_dB\n";

    std::cout << std::fixed << std::setprecision(2);
    std::cout << " Elev  Distance   FSPL   Absorb  Weather  Pointing   RxPwr    SNR    Cap\n";
    std::cout << "  deg    km        dB      dB      dB       dB       dBm     dB    Gbps\n";
    std::cout << "-----------------------------------------------------------------------\n";

    for (double elev : {5.0, 10.0, 15.0, 20.0, 30.0, 45.0, 60.0, 75.0, 90.0})
    {
        double altKm = 550.0;
        double r = R_E + altKm * 1000.0;
        double cosGamma = (R_E / r) * std::cos(elev * M_PI / 180.0);
        double gamma = std::acos(cosGamma) - elev * M_PI / 180.0;

        Ptr<MobilityModel> sat = CreateSatellite(altKm, gamma * 180.0 / M_PI);
        auto r2 = lb->ComputeTeraLinkBudget(sat, ground);

        std::cout << std::setw(5) << elev << "  "
                  << std::setw(7) << r2.distanceM / 1000.0 << "  "
                  << std::setw(6) << r2.fspl_dB << "  "
                  << std::setw(6) << r2.molecularAbsorption_dB << "  "
                  << std::setw(7) << r2.weatherLoss_dB << "  "
                  << std::setw(7) << r2.pointingLoss_dB << "   "
                  << std::setw(7) << r2.rxPower_dBm << "  "
                  << std::setw(6) << r2.snr_dB << "  "
                  << std::setw(6) << r2.hardwareLimitedCapacity_Gbps << "\n";

        csv << elev << "," << r2.distanceM / 1000.0 << "," << r2.fspl_dB << ","
            << r2.molecularAbsorption_dB << "," << r2.weatherLoss_dB << ","
            << r2.scintillationLoss_dB << "," << r2.pointingLoss_dB << ","
            << r2.totalPathLoss_dB << "," << r2.rxPower_dBm << ","
            << r2.snr_dB << "," << r2.hardwareLimitedSnr_dB << ","
            << r2.shannonCapacity_Gbps << "," << r2.hardwareLimitedCapacity_Gbps << ","
            << r2.linkMargin_dB << "\n";
    }
    std::cout << "[CSV] " << OUT_DIR << "/01_leo_ground_sweep.csv\n";
}

// ============================================================================
// Example 2: ISL @ 300 GHz (distance sweep)
// ============================================================================
static void
Example2_ISL()
{
    std::cout << "\n================================================================\n";
    std::cout << "  Example 2: Inter-Satellite Link @ 300 GHz (vacuum)\n";
    std::cout << "================================================================\n";

    Ptr<ThzNtnIslChannel> isl = CreateObject<ThzNtnIslChannel>();
    isl->SetAttribute("Frequency", DoubleValue(300e9));
    isl->SetAttribute("TxPower", DoubleValue(30.0));
    isl->SetAttribute("TxGain", DoubleValue(50.0));
    isl->SetAttribute("RxGain", DoubleValue(50.0));
    isl->SetAttribute("Bandwidth", DoubleValue(20e9));
    Ptr<ThzNtnFreeSpaceLoss> fsl = CreateObject<ThzNtnFreeSpaceLoss>();
    isl->SetFreeSpaceLossModel(fsl);

    std::ofstream csv(OUT_DIR + "/02_isl_distance_sweep.csv");
    csv << "distance_km,snr_dB,capacity_Gbps,doppler_MHz\n";

    std::cout << std::fixed << std::setprecision(2);
    std::cout << " Distance    SNR     Capacity\n";
    std::cout << "   km         dB       Gbps\n";
    std::cout << "----------------------------------\n";

    for (double distKm : {100.0, 250.0, 500.0, 1000.0, 1500.0, 2000.0, 3000.0, 5000.0})
    {
        Ptr<ConstantVelocityMobilityModel> sat1 =
            CreateObject<ConstantVelocityMobilityModel>();
        sat1->SetPosition(Vector(R_E + 550e3, 0, 0));
        sat1->SetVelocity(Vector(0, 7500, 0));

        Ptr<ConstantVelocityMobilityModel> sat2 =
            CreateObject<ConstantVelocityMobilityModel>();
        double angle = (distKm * 1000.0) / (R_E + 550e3);
        sat2->SetPosition(Vector((R_E + 550e3) * std::cos(angle),
                                  (R_E + 550e3) * std::sin(angle), 0));
        sat2->SetVelocity(Vector(-7500 * std::sin(angle), 7500 * std::cos(angle), 0));

        double actualDist = sat1->GetDistanceFrom(sat2);
        double snr = isl->ComputeIslSnr_dB(sat1, sat2);
        double cap = (snr > -20.0) ? isl->ComputeIslCapacity_Gbps(snr) : 0.0;
        double doppler = isl->ComputeRelativeDoppler_Hz(sat1, sat2);

        std::cout << std::setw(7) << actualDist / 1000.0 << "   "
                  << std::setw(7) << snr << "    "
                  << std::setw(7) << cap << "\n";

        csv << actualDist / 1000.0 << "," << snr << "," << cap << ","
            << doppler / 1e6 << "\n";
    }
    std::cout << "[CSV] " << OUT_DIR << "/02_isl_distance_sweep.csv\n";
}

// ============================================================================
// Example 3: D-band Constellation
// ============================================================================
static void
Example3_DbandConstellation()
{
    std::cout << "\n================================================================\n";
    std::cout << "  Example 3: D-band Constellation @ 140 GHz (4 sats, 10 UTs)\n";
    std::cout << "================================================================\n";

    Ptr<ThzNtnLinkBudget> lb = CreateFullLinkBudget(140e9);

    // 4 satellites at 600 km altitude, close to the UT region (theta -5 to +10 deg)
    std::vector<Ptr<MobilityModel>> sats;
    std::vector<double> satThetas = {-5.0, 2.0, 9.0, 16.0};
    for (int i = 0; i < 4; i++)
    {
        sats.push_back(CreateSatellite(600.0, satThetas[i]));
    }

    std::ofstream csv(OUT_DIR + "/03_dband_constellation.csv");
    csv << "ut_id,best_sat_id,distance_km,elevation_deg,snr_dB,capacity_Gbps\n";

    double totalCapacity = 0.0;
    int servedUts = 0;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << " UT  BestSat  Dist_km  Elev_deg  SNR_dB  Cap_Gbps\n";
    std::cout << "------------------------------------------------------\n";
    for (int u = 0; u < 10; u++)
    {
        double utLat = u * 3.0;
        Ptr<MobilityModel> ut = CreateGroundNode(utLat, 0.0);

        int bestIdx = -1;
        double bestDist = 1e20;
        for (int i = 0; i < 4; i++)
        {
            double d = sats[i]->GetDistanceFrom(ut);
            if (d < bestDist)
            {
                bestDist = d;
                bestIdx = i;
            }
        }

        auto r2 = lb->ComputeDbandLeoBudget(sats[bestIdx], ut);
        if (r2.snr_dB > -10.0)
        {
            totalCapacity += r2.hardwareLimitedCapacity_Gbps;
            servedUts++;
        }
        std::cout << std::setw(3) << u << "     "
                  << std::setw(1) << bestIdx << "    "
                  << std::setw(7) << r2.distanceM / 1000.0 << "   "
                  << std::setw(6) << r2.elevationDeg << "   "
                  << std::setw(6) << r2.snr_dB << "  "
                  << std::setw(7) << r2.hardwareLimitedCapacity_Gbps << "\n";

        csv << u << "," << bestIdx << "," << r2.distanceM / 1000.0 << ","
            << r2.elevationDeg << "," << r2.snr_dB << ","
            << r2.hardwareLimitedCapacity_Gbps << "\n";
    }
    std::cout << "\nAggregate capacity: " << totalCapacity << " Gbps ("
              << servedUts << "/10 UTs served)\n";
    std::cout << "[CSV] " << OUT_DIR << "/03_dband_constellation.csv\n";
}

// ============================================================================
// Example 4: RIS Analysis
// ============================================================================
static void
Example4_Ris()
{
    std::cout << "\n================================================================\n";
    std::cout << "  Example 4: RIS-Assisted Link Analysis @ 300 GHz\n";
    std::cout << "================================================================\n";

    std::ofstream csv(OUT_DIR + "/04_ris_sweep.csv");
    csv << "num_elements,nx,ny,max_gain_dB,snr_gain_perfect_dB,snr_gain_imperfect_dB,"
        << "quant_loss_dB,array_size_cm\n";

    std::cout << std::fixed << std::setprecision(2);
    std::cout << " Size     Elements  MaxGain_dB  SnrPerfect  SnrImperfect  QuantLoss\n";
    std::cout << "------------------------------------------------------------------------\n";
    for (uint32_t n : {8u, 16u, 32u, 64u, 128u})
    {
        Ptr<ThzNtnRis> ris = CreateObject<ThzNtnRis>();
        ris->Configure(n, n, 300e9, RisDeployment::GROUND);

        double maxG = ris->ComputeMaxGain_dB();
        double snrGPerf = ris->ComputeSnrGain_dB(true);
        double snrGImp = ris->ComputeSnrGain_dB(false);
        double qLoss = ris->ComputeQuantizationLoss_dB();
        double sizeC = (n * 0.5 / (300.0)) * 10.0; // cm

        std::cout << " " << std::setw(3) << n << "x" << std::setw(3) << n
                  << "  " << std::setw(6) << n * n << "   "
                  << std::setw(8) << maxG << "  "
                  << std::setw(8) << snrGPerf << "  "
                  << std::setw(8) << snrGImp << "    "
                  << std::setw(6) << qLoss << "\n";

        csv << (n * n) << "," << n << "," << n << "," << maxG << "," << snrGPerf << ","
            << snrGImp << "," << qLoss << "," << sizeC << "\n";
    }

    Ptr<ThzNtnRis> r8 = CreateObject<ThzNtnRis>();
    r8->Configure(8, 8, 300e9, RisDeployment::GROUND);
    Ptr<ThzNtnRis> r16 = CreateObject<ThzNtnRis>();
    r16->Configure(16, 16, 300e9, RisDeployment::GROUND);
    double diff = r16->ComputeMaxGain_dB() - r8->ComputeMaxGain_dB();
    std::cout << "\nN^2 scaling verification: 4x more elements -> "
              << diff << " dB more gain (expected 20*log10(4) = 12.04 dB)\n";
    std::cout << "[CSV] " << OUT_DIR << "/04_ris_sweep.csv\n";
}

// ============================================================================
// Example 5: ISAC Debris Detection (with proper antenna)
// ============================================================================
static void
Example5_Isac()
{
    std::cout << "\n================================================================\n";
    std::cout << "  Example 5: ISAC Debris Detection @ 300 GHz, 20 GHz BW\n";
    std::cout << "================================================================\n";

    // Wire up a full ISAC stack with proper antenna
    Ptr<ThzNtnAntennaArray> array = CreateObject<ThzNtnAntennaArray>();
    array->SetAttribute("NumElementsX", UintegerValue(32));
    array->SetAttribute("NumElementsY", UintegerValue(32));
    array->SetAttribute("Frequency", DoubleValue(300e9));
    array->SetAttribute("ElementGain", DoubleValue(5.0));

    Ptr<ThzNtnIsac> isac = CreateObject<ThzNtnIsac>();
    isac->SetAntennaArray(array);

    double freqHz = 300e9;
    double bandwidthHz = 20e9;
    double txPowerDbm = 40.0;    // 10 W (regenerative satellite)
    double totalGainDbi = 2.0 * array->ComputeMaxGain_dBi(); // monostatic

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Range resolution:     " << isac->ComputeRangeResolution_m(bandwidthHz) * 100.0
              << " cm (c/2BW, BW=20 GHz)\n";
    double lambda = SPEED_OF_LIGHT / freqHz;
    std::cout << "Wavelength:           " << lambda * 1000.0 << " mm\n";
    std::cout << "Velocity resolution:  "
              << isac->ComputeVelocityResolution_m_s(freqHz, 0.001) << " m/s (T_int=1ms)\n";
    std::cout << "Array (32x32) gain:   " << array->ComputeMaxGain_dBi()
              << " dBi (per antenna)\n";
    std::cout << "Total bistatic gain:  " << totalGainDbi << " dBi\n\n";

    std::ofstream csv(OUT_DIR + "/05_isac_debris.csv");
    csv << "debris_size,rcs_dBsm,rcs_m2,max_detect_range_m,"
        << "snr_at_10m_dB,snr_at_50m_dB,snr_at_100m_dB,snr_at_500m_dB,pd_at_10dB\n";

    std::cout << std::setprecision(2);
    std::cout << "NOTE: THz radar is inherently SHORT-range (lambda^2 term in radar equation\n"
              << "      limits range; ideal for rendezvous / proximity operations at m-scale).\n\n";
    std::cout << " Debris      RCS(dBsm)  MaxRng(m)  SNR@10m  SNR@50m  SNR@100m  SNR@500m\n";
    std::cout << "------------------------------------------------------------------------------\n";

    for (auto cat : std::vector<std::string>{"small_1cm", "medium_10cm", "large_1m"})
    {
        auto dbr = isac->GetDebrisModel(cat);
        double rcsLinear = std::pow(10.0, dbr.rcs_dBsm / 10.0);
        double maxRange_m = isac->ComputeMaxDetectionRange_km(rcsLinear, 10.0) * 1000.0;
        double pd10 = isac->ComputeDetectionProbability(10.0, 1e-6);
        double snr10m = isac->ComputeRadarSnr_dB(freqHz, txPowerDbm, totalGainDbi,
                                                   10.0, rcsLinear, bandwidthHz);
        double snr50m = isac->ComputeRadarSnr_dB(freqHz, txPowerDbm, totalGainDbi,
                                                   50.0, rcsLinear, bandwidthHz);
        double snr100m = isac->ComputeRadarSnr_dB(freqHz, txPowerDbm, totalGainDbi,
                                                    100.0, rcsLinear, bandwidthHz);
        double snr500m = isac->ComputeRadarSnr_dB(freqHz, txPowerDbm, totalGainDbi,
                                                    500.0, rcsLinear, bandwidthHz);

        std::cout << " " << std::setw(12) << std::left << cat << std::right
                  << "  " << std::setw(5) << dbr.rcs_dBsm
                  << "    " << std::setw(6) << maxRange_m
                  << "   " << std::setw(6) << snr10m
                  << "  " << std::setw(6) << snr50m
                  << "   " << std::setw(6) << snr100m
                  << "   " << std::setw(6) << snr500m << "\n";

        csv << cat << "," << dbr.rcs_dBsm << "," << rcsLinear << "," << maxRange_m << ","
            << snr10m << "," << snr50m << "," << snr100m << "," << snr500m << "," << pd10 << "\n";
    }
    std::cout << "\nInterpretation: Pd @ SNR=10dB, Pfa=1e-6 -> " << std::setprecision(3);
    std::cout << isac->ComputeDetectionProbability(10.0, 1e-6) << "\n";
    std::cout << "  (Large 1m debris detectable reliably below ~50m range)\n";
    std::cout << "[CSV] " << OUT_DIR << "/05_isac_debris.csv\n";
}

// ============================================================================
// Example 6: UM-MIMO Array Characterization
// ============================================================================
static void
Example6_UmMimo()
{
    std::cout << "\n================================================================\n";
    std::cout << "  Example 6: UM-MIMO Array Characterization @ 300 GHz\n";
    std::cout << "================================================================\n";

    std::ofstream csv(OUT_DIR + "/06_um_mimo.csv");
    csv << "nx,ny,total_elements,max_gain_dBi,beamwidth_deg,physical_size_cm,near_field_m\n";

    std::cout << std::fixed << std::setprecision(3);
    std::cout << " Config     Elements  MaxGain_dBi  BW3dB_deg   Size_cm   NearField_m\n";
    std::cout << "---------------------------------------------------------------------\n";

    for (uint32_t n : {4u, 8u, 16u, 32u, 64u, 128u})
    {
        Ptr<ThzNtnAntennaArray> arr = CreateObject<ThzNtnAntennaArray>();
        arr->SetAttribute("NumElementsX", UintegerValue(n));
        arr->SetAttribute("NumElementsY", UintegerValue(n));
        arr->SetAttribute("Frequency", DoubleValue(300e9));

        double g = arr->ComputeMaxGain_dBi();
        double bw = arr->ComputeBeamwidth3dB_deg();
        double sz = arr->ComputePhysicalSize_m() * 100.0;
        double nf = arr->ComputeNearFieldDistance_m();

        std::cout << " " << std::setw(3) << n << "x" << std::setw(3) << std::left << n
                  << std::right << "  " << std::setw(6) << n * n << "    "
                  << std::setw(8) << g << "    "
                  << std::setw(8) << bw << "   "
                  << std::setw(7) << sz << "    "
                  << std::setw(7) << nf << "\n";

        csv << n << "," << n << "," << (n * n) << "," << g << "," << bw << ","
            << sz << "," << nf << "\n";
    }
    std::cout << "[CSV] " << OUT_DIR << "/06_um_mimo.csv\n";
}

// ============================================================================
// Example 7: Atmospheric Windows
// ============================================================================
static void
Example7_AtmosphericWindows()
{
    std::cout << "\n================================================================\n";
    std::cout << "  Example 7: THz Atmospheric Windows\n";
    std::cout << "================================================================\n";

    auto windows = ThzNtnSpectrum::GetStandardWindows();

    std::ofstream csv(OUT_DIR + "/07_atmospheric_windows.csv");
    csv << "window_id,center_freq_GHz,bandwidth_GHz,peak_transmittance,"
        << "zenith_atten_dB,suitable_sat_ground\n";

    std::cout << std::fixed << std::setprecision(2);
    std::cout << " # | CenterFreq | BW_GHz | PeakTrans | ZenithAtten | Sat-Ground\n";
    std::cout << "-----------------------------------------------------------------\n";
    int idx = 1;
    for (auto const& w : windows)
    {
        std::cout << " " << idx << " | " << std::setw(7) << w.centerFreqGHz
                  << " GHz|" << std::setw(5) << w.bandwidthGHz
                  << "  |  " << std::setw(5) << w.peakTransmittance
                  << "  |  " << std::setw(5) << w.maxZenithAttenuation_dB
                  << " dB  |  " << (w.suitableForSatGround ? "YES" : "NO") << "\n";
        csv << idx << "," << w.centerFreqGHz << "," << w.bandwidthGHz << ","
            << w.peakTransmittance << "," << w.maxZenithAttenuation_dB << ","
            << (w.suitableForSatGround ? 1 : 0) << "\n";
        idx++;
    }
    std::cout << "[CSV] " << OUT_DIR << "/07_atmospheric_windows.csv\n";
}

// ============================================================================
// Example 8: Beam Tracking During LEO Pass
// ============================================================================
static void
Example8_BeamTracking()
{
    std::cout << "\n================================================================\n";
    std::cout << "  Example 8: Beam Tracking During 600-sec LEO Pass\n";
    std::cout << "================================================================\n";

    Ptr<ThzNtnBeamTracking> bt = CreateObject<ThzNtnBeamTracking>();
    bt->Initialize(45.0, 0.0, 0.0, 0.0);

    Ptr<ThzNtnLinkBudget> lb = CreateFullLinkBudget(225e9);
    Ptr<MobilityModel> ground = CreateGroundNode();

    std::ofstream csv(OUT_DIR + "/08_beam_tracking.csv");
    csv << "time_s,sat_angle_deg,distance_km,elevation_deg,snr_dB,capacity_Gbps\n";

    std::cout << std::fixed << std::setprecision(2);
    std::cout << " Time  SatAngle  Dist_km  Elev_deg  SNR_dB  Cap_Gbps\n";
    std::cout << "-------------------------------------------------------\n";

    // Realistic 600-s LEO pass: satellite traverses -10 deg to +10 deg
    // off-zenith at 550 km altitude, sampled every 10 s (61 samples).
    const double T_PASS = 600.0;
    const double T_STEP = 10.0;
    for (double t = 0.0; t <= T_PASS + 1e-9; t += T_STEP)
    {
        double satAngle = -10.0 + (20.0 * t / T_PASS);
        Ptr<MobilityModel> sat = CreateSatellite(550.0, satAngle);
        auto r = lb->ComputeTeraLinkBudget(sat, ground);

        std::cout << " " << std::setw(4) << t << "   "
                  << std::setw(6) << satAngle << "   "
                  << std::setw(6) << r.distanceM / 1000.0 << "    "
                  << std::setw(5) << r.elevationDeg << "   "
                  << std::setw(6) << r.snr_dB << "   "
                  << std::setw(6) << r.hardwareLimitedCapacity_Gbps << "\n";

        csv << t << "," << satAngle << "," << r.distanceM / 1000.0 << ","
            << r.elevationDeg << "," << r.snr_dB << ","
            << r.hardwareLimitedCapacity_Gbps << "\n";
    }
    std::cout << "[CSV] " << OUT_DIR << "/08_beam_tracking.csv\n";
}

// ============================================================================
// Summary report
// ============================================================================
static void
WriteSummary()
{
    std::ofstream sum(OUT_DIR + "/SUMMARY.txt");
    sum << "=========================================================\n"
        << "  THz-NTN Module Comprehensive Demo Results Summary\n"
        << "=========================================================\n\n"
        << "Generated CSV datasets in " << OUT_DIR << "/:\n\n"
        << "  01_leo_ground_sweep.csv      - LEO-ground link budget vs elevation\n"
        << "  02_isl_distance_sweep.csv    - ISL SNR/capacity/Doppler vs distance\n"
        << "  03_dband_constellation.csv   - Constellation UT-sat associations\n"
        << "  04_ris_sweep.csv             - RIS gain scaling law verification\n"
        << "  05_isac_debris.csv           - ISAC debris detection performance\n"
        << "  06_um_mimo.csv               - UM-MIMO array characterization\n"
        << "  07_atmospheric_windows.csv   - THz atmospheric window table\n"
        << "  08_beam_tracking.csv         - LEO pass time series\n\n"
        << "Each CSV has a header row and can be loaded with pandas/matplotlib:\n"
        << "  import pandas as pd\n"
        << "  df = pd.read_csv('" << OUT_DIR << "/01_leo_ground_sweep.csv')\n"
        << "\n";
    sum.close();
    std::cout << "[SUMMARY] " << OUT_DIR << "/SUMMARY.txt\n";
}

// ============================================================================
// Main
// ============================================================================
int
main(int argc, char* argv[])
{
    int example = 0;  // 0 = all
    CommandLine cmd;
    cmd.AddValue("example", "Example number to run (1-8), 0=all", example);
    cmd.Parse(argc, argv);

    EnsureOutDir();

    std::cout << "\n###############################################################\n";
    std::cout << "#      THz-NTN Comprehensive Demo + Dataset Generation         #\n";
    std::cout << "#      Output directory: " << OUT_DIR << "/\n";
    std::cout << "###############################################################\n";

    if (example == 0 || example == 1) Example1_LeoGround();
    if (example == 0 || example == 2) Example2_ISL();
    if (example == 0 || example == 3) Example3_DbandConstellation();
    if (example == 0 || example == 4) Example4_Ris();
    if (example == 0 || example == 5) Example5_Isac();
    if (example == 0 || example == 6) Example6_UmMimo();
    if (example == 0 || example == 7) Example7_AtmosphericWindows();
    if (example == 0 || example == 8) Example8_BeamTracking();

    if (example == 0) WriteSummary();

    std::cout << "\n###############################################################\n";
    std::cout << "#      All 8 examples completed; 8 CSV datasets + summary      #\n";
    std::cout << "###############################################################\n\n";

    return 0;
}
