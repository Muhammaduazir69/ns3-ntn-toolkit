/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN Setup Helper -- Integrated with satellite and mmWave module APIs
 */

#include "thz-ntn-helper.h"

#include "ns3/thz-ntn-antenna-array.h"
#include "ns3/thz-ntn-beam-tracking.h"
#include "ns3/thz-ntn-beamforming.h"
#include "ns3/thz-ntn-channel-model.h"
#include "ns3/thz-ntn-free-space-loss.h"
#include "ns3/thz-ntn-hardware-impairments.h"
#include "ns3/thz-ntn-isl-channel.h"
#include "ns3/thz-ntn-link-budget.h"
#include "ns3/thz-ntn-mac.h"
#include "ns3/thz-ntn-molecular-absorption.h"
#include "ns3/thz-ntn-phy-ground.h"
#include "ns3/thz-ntn-phy-sat.h"
#include "ns3/thz-ntn-pointing-error.h"
#include "ns3/thz-ntn-scintillation.h"
#include "ns3/thz-ntn-weather-attenuation.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/mobility-model.h>
#include <ns3/satellite-phy.h>
#include <ns3/string.h>
#include <ns3/uinteger.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnHelper");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnHelper);

/// Speed of light for lambda/2 element spacing computation
static constexpr double SPEED_OF_LIGHT = 299792458.0;

TypeId
ThzNtnHelper::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnHelper")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnHelper>();
    return tid;
}

ThzNtnHelper::ThzNtnHelper()
{
    NS_LOG_FUNCTION(this);
}

ThzNtnHelper::~ThzNtnHelper()
{
    NS_LOG_FUNCTION(this);
}

ThzNtnHelper::PresetConfig
ThzNtnHelper::GetPresetConfig(const std::string& preset) const
{
    PresetConfig cfg;

    if (preset == "TeraLink-225GHz")
    {
        cfg.frequencyHz = 225e9;
        cfg.bandwidthHz = 10e9;
        cfg.txPowerDbm = 34.77;  // 3 W
        cfg.numElementsX = 32;
        cfg.numElementsY = 32;
        cfg.txGainDbi = 40.0;
        cfg.rxGainDbi = 45.0;
        cfg.noiseFigureDb = 10.0;
        cfg.isIsl = false;
    }
    else if (preset == "D-band-140GHz")
    {
        cfg.frequencyHz = 140e9;
        cfg.bandwidthHz = 20e9;
        cfg.txPowerDbm = 40.0;  // 10 W
        cfg.numElementsX = 16;
        cfg.numElementsY = 16;
        cfg.txGainDbi = 38.0;
        cfg.rxGainDbi = 42.0;
        cfg.noiseFigureDb = 8.0;
        cfg.isIsl = false;
    }
    else if (preset == "ISL-300GHz")
    {
        cfg.frequencyHz = 300e9;
        cfg.bandwidthHz = 20e9;
        cfg.txPowerDbm = 30.0;  // 1 W
        cfg.numElementsX = 32;
        cfg.numElementsY = 32;
        cfg.txGainDbi = 40.0;
        cfg.rxGainDbi = 40.0;
        cfg.noiseFigureDb = 8.0;
        cfg.isIsl = true;
    }
    else
    {
        NS_LOG_WARN("Unknown preset '" << preset
                    << "', using TeraLink-225GHz defaults");
        cfg.frequencyHz = 225e9;
        cfg.bandwidthHz = 10e9;
        cfg.txPowerDbm = 34.77;
        cfg.numElementsX = 32;
        cfg.numElementsY = 32;
        cfg.txGainDbi = 40.0;
        cfg.rxGainDbi = 45.0;
        cfg.noiseFigureDb = 10.0;
        cfg.isIsl = false;
    }

    return cfg;
}

// ---- Individual sub-model creation ----

Ptr<ThzNtnMolecularAbsorption>
ThzNtnHelper::CreateMolecularAbsorption() const
{
    NS_LOG_FUNCTION(this);
    Ptr<ThzNtnMolecularAbsorption> model =
        CreateObject<ThzNtnMolecularAbsorption>();
    model->SetHumidityProfile("midlatitude-summer");
    return model;
}

Ptr<ThzNtnWeatherAttenuation>
ThzNtnHelper::CreateWeatherAttenuation() const
{
    NS_LOG_FUNCTION(this);
    return CreateObject<ThzNtnWeatherAttenuation>();
}

Ptr<ThzNtnScintillation>
ThzNtnHelper::CreateScintillation() const
{
    NS_LOG_FUNCTION(this);
    return CreateObject<ThzNtnScintillation>();
}

Ptr<ThzNtnPointingError>
ThzNtnHelper::CreatePointingError() const
{
    NS_LOG_FUNCTION(this);
    return CreateObject<ThzNtnPointingError>();
}

Ptr<ThzNtnHardwareImpairments>
ThzNtnHelper::CreateHardwareImpairments() const
{
    NS_LOG_FUNCTION(this);
    return CreateObject<ThzNtnHardwareImpairments>();
}

// ---- ThzNtnFreeSpaceLoss with all sub-models wired ----

Ptr<ThzNtnFreeSpaceLoss>
ThzNtnHelper::CreateFreeSpaceLoss(const std::string& preset) const
{
    NS_LOG_FUNCTION(this << preset);

    PresetConfig cfg = GetPresetConfig(preset);

    Ptr<ThzNtnFreeSpaceLoss> fsl = CreateObject<ThzNtnFreeSpaceLoss>();

    // Create and wire all sub-models
    Ptr<ThzNtnMolecularAbsorption> absorption = CreateMolecularAbsorption();
    fsl->SetMolecularAbsorptionModel(absorption);

    Ptr<ThzNtnWeatherAttenuation> weather = CreateWeatherAttenuation();
    fsl->SetWeatherModel(weather);

    Ptr<ThzNtnScintillation> scint = CreateScintillation();
    fsl->SetScintillationModel(scint);

    Ptr<ThzNtnPointingError> pointing = CreatePointingError();
    fsl->SetPointingErrorModel(pointing);

    // Enable/disable effects based on preset
    if (cfg.isIsl)
    {
        // ISL: no atmospheric effects, only pointing error
        fsl->EnableMolecularAbsorption(false);
        fsl->EnableWeatherEffects(false);
        fsl->EnableScintillation(false);
        fsl->EnablePointingError(true);
    }
    else
    {
        fsl->EnableMolecularAbsorption(true);
        fsl->EnableWeatherEffects(true);
        fsl->EnableScintillation(true);
        fsl->EnablePointingError(true);
    }

    NS_LOG_INFO("Created ThzNtnFreeSpaceLoss with all sub-models for preset: "
                << preset);
    return fsl;
}

// ---- Channel model with fully wired sub-model stack ----

Ptr<ThzNtnChannelModel>
ThzNtnHelper::CreateChannelModel(const std::string& preset) const
{
    NS_LOG_FUNCTION(this << preset);

    PresetConfig cfg = GetPresetConfig(preset);

    Ptr<ThzNtnChannelModel> channel = CreateObject<ThzNtnChannelModel>();

    // Set band from preset
    channel->SetBand(preset);

    // Create all sub-models
    Ptr<ThzNtnMolecularAbsorption> absorption = CreateMolecularAbsorption();
    Ptr<ThzNtnWeatherAttenuation> weather = CreateWeatherAttenuation();
    Ptr<ThzNtnScintillation> scint = CreateScintillation();
    Ptr<ThzNtnPointingError> pointing = CreatePointingError();
    Ptr<ThzNtnHardwareImpairments> hw = CreateHardwareImpairments();

    // Wire sub-models into channel model
    channel->SetMolecularAbsorptionModel(absorption);
    channel->SetWeatherModel(weather);
    channel->SetScintillationModel(scint);
    channel->SetPointingErrorModel(pointing);
    channel->SetHardwareModel(hw);

    // Enable effects based on preset
    if (cfg.isIsl)
    {
        channel->SetLinkType(ThzNtnChannelModel::INTER_SATELLITE);
        channel->EnableMolecularAbsorption(false);
        channel->EnableWeatherEffects(false);
        channel->EnableScintillation(false);
        channel->EnablePointingError(true);
        channel->EnableHardwareImpairments(true);
    }
    else
    {
        channel->SetLinkType(ThzNtnChannelModel::AUTO_DETECT);
        channel->EnableMolecularAbsorption(true);
        channel->EnableWeatherEffects(true);
        channel->EnableScintillation(true);
        channel->EnablePointingError(true);
        channel->EnableHardwareImpairments(true);
    }

    NS_LOG_INFO("Created ThzNtnChannelModel with all sub-models for preset: "
                << preset);
    return channel;
}

// ---- PHY creation with satellite module wiring ----

Ptr<ThzNtnPhySat>
ThzNtnHelper::CreateSatellitePhy(const std::string& preset,
                                  Ptr<SatPhy> satPhy) const
{
    NS_LOG_FUNCTION(this << preset);

    PresetConfig cfg = GetPresetConfig(preset);

    Ptr<ThzNtnPhySat> phy = CreateObject<ThzNtnPhySat>();

    // Set preset parameters
    phy->SetAttributeFailSafe("TxPowerDbm", DoubleValue(cfg.txPowerDbm));
    phy->SetAttributeFailSafe("FrequencyHz", DoubleValue(cfg.frequencyHz));
    phy->SetAttributeFailSafe("BandwidthHz", DoubleValue(cfg.bandwidthHz));

    // Configure satellite class based on power
    if (cfg.txPowerDbm <= 35.0)
    {
        phy->SetSatelliteClass(SAT_CUBESAT);
    }
    else if (cfg.txPowerDbm <= 41.0)
    {
        phy->SetSatelliteClass(SAT_SMALLSAT);
    }
    else
    {
        phy->SetSatelliteClass(SAT_FULLSAT);
    }

    // Default to regenerative payload
    phy->SetPayloadMode(PAYLOAD_REGENERATIVE);
    phy->SetEnableDopplerPreComp(true);

    // Wire satellite PHY if provided
    if (satPhy)
    {
        phy->SetSatellitePhy(satPhy);
        NS_LOG_INFO("Wired SatPhy into ThzNtnPhySat");
    }

    NS_LOG_INFO("Created ThzNtnPhySat with preset: " << preset);
    return phy;
}

Ptr<ThzNtnPhyGround>
ThzNtnHelper::CreateGroundPhy(const std::string& preset,
                               Ptr<SatPhy> utPhy) const
{
    NS_LOG_FUNCTION(this << preset);

    PresetConfig cfg = GetPresetConfig(preset);

    Ptr<ThzNtnPhyGround> phy = CreateObject<ThzNtnPhyGround>();

    // Set preset parameters
    phy->SetAttributeFailSafe("FrequencyHz", DoubleValue(cfg.frequencyHz));
    phy->SetAttributeFailSafe("BandwidthHz", DoubleValue(cfg.bandwidthHz));

    // Default receiver configuration
    phy->SetReceiverType(RX_HETERODYNE);
    phy->SetPolarization(POL_DUAL_CIRCULAR);

    // Wire UT PHY if provided
    if (utPhy)
    {
        phy->SetUtPhy(utPhy);
        NS_LOG_INFO("Wired SatUtPhy into ThzNtnPhyGround");
    }

    NS_LOG_INFO("Created ThzNtnPhyGround with preset: " << preset);
    return phy;
}

// ---- Antenna array with frequency-derived element spacing ----

Ptr<ThzNtnAntennaArray>
ThzNtnHelper::CreateAntennaArray(const std::string& preset) const
{
    NS_LOG_FUNCTION(this << preset);

    PresetConfig cfg = GetPresetConfig(preset);

    Ptr<ThzNtnAntennaArray> antenna = CreateObject<ThzNtnAntennaArray>();

    // Configure array geometry; element spacing = lambda/2 is computed
    // internally by Configure() from the frequency
    antenna->Configure(cfg.numElementsX,
                       cfg.numElementsY,
                       cfg.frequencyHz,
                       ThzArrayType::UPA);

    NS_LOG_INFO("Created ThzNtnAntennaArray " << cfg.numElementsX << "x"
                << cfg.numElementsY << " at " << cfg.frequencyHz / 1e9
                << " GHz, lambda/2 = "
                << SPEED_OF_LIGHT / (2.0 * cfg.frequencyHz) * 1e6
                << " um");
    return antenna;
}

// ---- MAC with absorption model connected ----

Ptr<ThzNtnMac>
ThzNtnHelper::CreateMac(const std::string& preset,
                         Ptr<ThzNtnMolecularAbsorption> absorption) const
{
    NS_LOG_FUNCTION(this << preset);

    PresetConfig cfg = GetPresetConfig(preset);

    Ptr<ThzNtnMac> mac = CreateObject<ThzNtnMac>();

    // Configure resource grid: 1 GHz sub-band width
    double subBandWidthHz = 1.0e9;
    mac->ConfigureResourceGrid(cfg.bandwidthHz, subBandWidthHz, cfg.frequencyHz);

    // Configure frame structure with overhead parameters
    mac->ConfigureFrame(
        100,   // numSlots
        10.0,  // slotDuration_us
        0.5,   // guardInterval_us
        1.0,   // preambleDuration_us
        0.1    // pilotOverheadRatio
    );

    // Set access mode
    mac->SetAccessMode(ThzNtnAccessMode::HYBRID_TDMA_FDMA);

    // Wire molecular absorption model for DAMC filtering
    if (absorption)
    {
        mac->SetMolecularAbsorptionModel(absorption);
        NS_LOG_INFO("Wired provided absorption model into ThzNtnMac");
    }
    else if (!cfg.isIsl)
    {
        // Create a new one for atmospheric links
        Ptr<ThzNtnMolecularAbsorption> newAbsorption = CreateMolecularAbsorption();
        mac->SetMolecularAbsorptionModel(newAbsorption);
        NS_LOG_INFO("Created and wired new absorption model into ThzNtnMac");
    }

    NS_LOG_INFO("Created ThzNtnMac with preset: " << preset
                << ", BW=" << cfg.bandwidthHz / 1e9 << " GHz");
    return mac;
}

// ---- Other component creation ----

Ptr<ThzNtnBeamforming>
ThzNtnHelper::CreateBeamforming(uint32_t numBeams) const
{
    NS_LOG_FUNCTION(this << numBeams);

    Ptr<ThzNtnBeamforming> bf = CreateObject<ThzNtnBeamforming>();
    // The configurable analog-beam count is exposed as the "NumAnalogBeams"
    // attribute (there is no "NumBeams" attribute); setting the wrong name
    // here used to abort at runtime for every caller.
    bf->SetAttribute("NumAnalogBeams", UintegerValue(numBeams));

    NS_LOG_INFO("Created ThzNtnBeamforming with " << numBeams << " analog beams");
    return bf;
}

Ptr<ThzNtnBeamTracking>
ThzNtnHelper::CreateBeamTracking() const
{
    NS_LOG_FUNCTION(this);

    Ptr<ThzNtnBeamTracking> bt = CreateObject<ThzNtnBeamTracking>();

    NS_LOG_INFO("Created ThzNtnBeamTracking (EKF mode)");
    return bt;
}

Ptr<ThzNtnIslChannel>
ThzNtnHelper::CreateIslChannel(const std::string& preset) const
{
    NS_LOG_FUNCTION(this << preset);

    PresetConfig cfg = GetPresetConfig(preset);

    Ptr<ThzNtnIslChannel> isl = CreateObject<ThzNtnIslChannel>();
    isl->SetAttributeFailSafe("FrequencyHz", DoubleValue(cfg.frequencyHz));
    isl->SetAttributeFailSafe("BandwidthHz", DoubleValue(cfg.bandwidthHz));

    NS_LOG_INFO("Created ThzNtnIslChannel at " << cfg.frequencyHz / 1e9
                << " GHz, BW=" << cfg.bandwidthHz / 1e9 << " GHz");
    return isl;
}

Ptr<ThzNtnLinkBudget>
ThzNtnHelper::CreateLinkBudget(const std::string& preset) const
{
    NS_LOG_FUNCTION(this << preset);

    Ptr<ThzNtnLinkBudget> lb = CreateObject<ThzNtnLinkBudget>();

    // Create and wire sub-models into the link budget
    Ptr<ThzNtnFreeSpaceLoss> fsl = CreateFreeSpaceLoss(preset);
    lb->SetFreeSpaceLossModel(fsl);

    Ptr<ThzNtnHardwareImpairments> hw = CreateHardwareImpairments();
    lb->SetHardwareModel(hw);

    // Also wire individual atmospheric models for the link budget's
    // own preset computations
    Ptr<ThzNtnMolecularAbsorption> absorption = CreateMolecularAbsorption();
    lb->SetMolecularAbsorptionModel(absorption);

    Ptr<ThzNtnWeatherAttenuation> weather = CreateWeatherAttenuation();
    lb->SetWeatherModel(weather);

    Ptr<ThzNtnScintillation> scint = CreateScintillation();
    lb->SetScintillationModel(scint);

    Ptr<ThzNtnPointingError> pointing = CreatePointingError();
    lb->SetPointingErrorModel(pointing);

    NS_LOG_INFO("Created ThzNtnLinkBudget with all sub-models for preset: "
                << preset);
    return lb;
}

// ---- Full stack installation ----

void
ThzNtnHelper::InstallOnSatellite(Ptr<Node> satNode, const std::string& preset)
{
    NS_LOG_FUNCTION(this << satNode << preset);

    [[maybe_unused]] PresetConfig cfg = GetPresetConfig(preset);

    // Get MobilityModel from node
    Ptr<MobilityModel> mobility = satNode->GetObject<MobilityModel>();
    NS_ABORT_MSG_IF(!mobility,
                    "ThzNtnHelper::InstallOnSatellite: node "
                    << satNode->GetId() << " has no MobilityModel");

    // Create satellite PHY and wire MobilityModel
    Ptr<ThzNtnPhySat> phy = CreateSatellitePhy(preset);
    phy->SetSatelliteMobility(mobility);
    satNode->AggregateObject(phy);

    // Create antenna array with frequency-derived spacing
    Ptr<ThzNtnAntennaArray> antenna = CreateAntennaArray(preset);
    satNode->AggregateObject(antenna);

    // Create beamforming
    Ptr<ThzNtnBeamforming> bf = CreateBeamforming(64);
    satNode->AggregateObject(bf);

    // Create beam tracking and wire satellite MobilityModel
    Ptr<ThzNtnBeamTracking> bt = CreateBeamTracking();
    bt->SetSatelliteMobility(mobility);
    satNode->AggregateObject(bt);

    // Create MAC with absorption model for DAMC
    Ptr<ThzNtnMolecularAbsorption> absorption = CreateMolecularAbsorption();
    Ptr<ThzNtnMac> mac = CreateMac(preset, absorption);
    satNode->AggregateObject(mac);

    NS_LOG_INFO("Installed THz-NTN satellite stack on node " << satNode->GetId()
                << " with preset: " << preset
                << ", MobilityModel wired to PHY and beam tracker");
}

void
ThzNtnHelper::InstallOnGroundTerminal(Ptr<Node> gtNode,
                                       const std::string& preset)
{
    NS_LOG_FUNCTION(this << gtNode << preset);

    // Get MobilityModel from node
    Ptr<MobilityModel> mobility = gtNode->GetObject<MobilityModel>();
    NS_ABORT_MSG_IF(!mobility,
                    "ThzNtnHelper::InstallOnGroundTerminal: node "
                    << gtNode->GetId() << " has no MobilityModel");

    // Create ground PHY
    Ptr<ThzNtnPhyGround> phy = CreateGroundPhy(preset);
    gtNode->AggregateObject(phy);

    // Create antenna array with frequency-derived spacing
    Ptr<ThzNtnAntennaArray> antenna = CreateAntennaArray(preset);
    gtNode->AggregateObject(antenna);

    // Create MAC with absorption model
    Ptr<ThzNtnMolecularAbsorption> absorption = CreateMolecularAbsorption();
    Ptr<ThzNtnMac> mac = CreateMac(preset, absorption);
    gtNode->AggregateObject(mac);

    NS_LOG_INFO("Installed THz-NTN ground terminal stack on node "
                << gtNode->GetId() << " with preset: " << preset);
}

void
ThzNtnHelper::PrintConfiguration() const
{
    NS_LOG_FUNCTION(this);

    std::ostringstream oss;
    oss << "\n"
        << "============================================================\n"
        << "  THz-NTN Helper -- Available Presets (Integrated APIs)\n"
        << "============================================================\n"
        << "\n"
        << "  TeraLink-225GHz:\n"
        << "    Frequency:      225 GHz\n"
        << "    Bandwidth:      10 GHz\n"
        << "    TX Power:       3 W (34.77 dBm) -- CubeSat\n"
        << "    Antenna:        32x32 UPA (1024 elements)\n"
        << "    Element spacing: "
        << std::fixed << std::setprecision(1)
        << SPEED_OF_LIGHT / (2.0 * 225e9) * 1e6 << " um (lambda/2)\n"
        << "    TX Gain:        40 dBi\n"
        << "    RX Gain:        45 dBi (Cassegrain 0.45m)\n"
        << "    Noise Figure:   10 dB\n"
        << "    Sub-models:     absorption + weather + scintillation\n"
        << "                    + pointing error + HW impairments\n"
        << "\n"
        << "  D-band-140GHz:\n"
        << "    Frequency:      140 GHz\n"
        << "    Bandwidth:      20 GHz\n"
        << "    TX Power:       10 W (40 dBm) -- SmallSat\n"
        << "    Antenna:        16x16 UPA (256 elements)\n"
        << "    Element spacing: "
        << SPEED_OF_LIGHT / (2.0 * 140e9) * 1e6 << " um (lambda/2)\n"
        << "    TX Gain:        38 dBi\n"
        << "    RX Gain:        42 dBi\n"
        << "    Noise Figure:   8 dB\n"
        << "\n"
        << "  ISL-300GHz:\n"
        << "    Frequency:      300 GHz\n"
        << "    Bandwidth:      20 GHz\n"
        << "    TX Power:       1 W (30 dBm)\n"
        << "    Antenna:        32x32 UPA (1024 elements)\n"
        << "    Element spacing: "
        << SPEED_OF_LIGHT / (2.0 * 300e9) * 1e6 << " um (lambda/2)\n"
        << "    Path:           Vacuum (no atmospheric effects)\n"
        << "\n"
        << "  Integration points:\n"
        << "    ThzNtnFreeSpaceLoss -> SatFreeSpaceLoss (satellite pipeline)\n"
        << "    ThzNtnChannelModel  -> PropagationLossModel (mmWave pipeline)\n"
        << "    ThzNtnPhySat        -> SatPhy, SatWaveformConf, MobilityModel\n"
        << "    ThzNtnPhyGround     -> SatUtPhy, SpectrumValue, DlCqiInfo\n"
        << "    ThzNtnMac           -> ThzNtnMolecularAbsorption, MobilityModel\n"
        << "    ThzNtnAntennaArray  -> SatAntennaGainPattern, MobilityModel\n"
        << "    ThzNtnBeamTracking  -> MobilityModel (satellite ephemeris)\n"
        << "============================================================\n";

    NS_LOG_INFO(oss.str());
    std::cout << oss.str();
}

} // namespace ns3
