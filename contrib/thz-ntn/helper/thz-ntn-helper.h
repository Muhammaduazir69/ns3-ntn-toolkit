/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN Setup Helper -- Integrated with satellite and mmWave module APIs
 *
 * Convenience helper that creates and wires the complete THz-NTN simulation
 * stack using actual sub-model pointers, MobilityModel positions, and
 * satellite/mmWave module integration points.
 *
 * Each factory method creates a properly configured and wired model object.
 * InstallOnSatellite() and InstallOnGroundTerminal() retrieve the node's
 * MobilityModel, create all components, and wire them together.
 *
 * Built-in presets with actual physical parameters:
 *   - "TeraLink-225GHz": freq=225e9, BW=10e9, txPower=34.77dBm(3W),
 *                        CubeSat, 32x32 array
 *   - "D-band-140GHz":   freq=140e9, BW=20e9, txPower=40dBm(10W),
 *                        SmallSat, 16x16 array
 *   - "ISL-300GHz":      freq=300e9, BW=20e9, txPower=30dBm(1W),
 *                        32x32 array, ISL mode (no atmospheric effects)
 *
 * References:
 *   [1] 3GPP TR 38.811, "Study on New Radio to support non-terrestrial
 *       networks," v15.4.0, 2020.
 *   [2] S. Koenig et al., "Wireless sub-THz communication system with
 *       high data rate," Nature Photonics, vol. 7, 2013.
 */

#ifndef THZ_NTN_HELPER_H
#define THZ_NTN_HELPER_H

#include <ns3/node.h>
#include <ns3/object.h>
#include <ns3/ptr.h>
#include <ns3/satellite-phy.h>

#include <cstdint>
#include <string>

namespace ns3
{

// Forward declarations of model classes
class ThzNtnChannelModel;
class ThzNtnFreeSpaceLoss;
class ThzNtnPhySat;
class ThzNtnPhyGround;
class ThzNtnAntennaArray;
class ThzNtnBeamforming;
class ThzNtnBeamTracking;
class ThzNtnMac;
class ThzNtnIslChannel;
class ThzNtnLinkBudget;
class ThzNtnMolecularAbsorption;
class ThzNtnWeatherAttenuation;
class ThzNtnScintillation;
class ThzNtnPointingError;
class ThzNtnHardwareImpairments;

/**
 * \ingroup thz-ntn
 * \brief Setup helper for THz non-terrestrial network simulations
 *
 * This utility class provides factory methods that create and wire
 * THz-NTN model objects using actual sub-model pointers, MobilityModel
 * positions, and satellite/mmWave module integration points.
 *
 * All components are properly wired together:
 *   - ThzNtnFreeSpaceLoss receives all sub-models (absorption, weather,
 *     scintillation, pointing error) and is wired into ThzNtnChannelModel
 *   - ThzNtnPhySat connects to SatPhy and SatWaveformConf when provided
 *   - ThzNtnMac connects to ThzNtnMolecularAbsorption for DAMC filtering
 *   - ThzNtnAntennaArray element spacing is computed from lambda/2
 *   - ThzNtnBeamTracking connects to satellite MobilityModel
 */
class ThzNtnHelper : public Object
{
  public:
    /**
     * \brief Get the TypeId for this class.
     * \return the ns-3 TypeId
     */
    static TypeId GetTypeId();

    ThzNtnHelper();
    ~ThzNtnHelper() override;

    /**
     * \brief Preset configuration parameters.
     */
    struct PresetConfig
    {
        double frequencyHz;     ///< carrier frequency in Hz
        double bandwidthHz;     ///< channel bandwidth in Hz
        double txPowerDbm;      ///< transmit power in dBm
        uint32_t numElementsX;  ///< antenna array columns
        uint32_t numElementsY;  ///< antenna array rows
        double txGainDbi;       ///< transmit antenna gain in dBi
        double rxGainDbi;       ///< receive antenna gain in dBi
        double noiseFigureDb;   ///< receiver noise figure in dB
        bool isIsl;             ///< true if ISL (no atmospheric effects)
    };

    /**
     * \brief Get preset configuration parameters by name.
     * \param preset preset name
     * \return preset configuration structure
     */
    PresetConfig GetPresetConfig(const std::string& preset) const;

    // ---- Channel model creation with fully wired sub-models ----

    /**
     * \brief Create a composite channel model with all sub-models wired.
     *
     * Creates ThzNtnMolecularAbsorption, ThzNtnWeatherAttenuation,
     * ThzNtnScintillation, ThzNtnPointingError, ThzNtnHardwareImpairments,
     * wires them into ThzNtnFreeSpaceLoss, and wires that into
     * ThzNtnChannelModel.
     *
     * \param preset one of "TeraLink-225GHz", "D-band-140GHz", "ISL-300GHz"
     * \return configured ThzNtnChannelModel with all sub-models connected
     */
    Ptr<ThzNtnChannelModel> CreateChannelModel(const std::string& preset) const;

    // ---- PHY creation with satellite module connections ----

    /**
     * \brief Create a satellite PHY with optional SatPhy wiring.
     *
     * When satPhy is provided, it is wired in via SetSatellitePhy().
     *
     * \param preset preset name
     * \param satPhy optional pointer to satellite module's SatPhy
     * \return configured ThzNtnPhySat
     */
    Ptr<ThzNtnPhySat> CreateSatellitePhy(
        const std::string& preset,
        Ptr<SatPhy> satPhy = nullptr) const;

    /**
     * \brief Create a ground terminal PHY with optional SatUtPhy wiring.
     *
     * When utPhy is provided, it is wired in via SetUtPhy().
     *
     * \param preset preset name
     * \param utPhy optional pointer to satellite module's SatPhy (UT)
     * \return configured ThzNtnPhyGround
     */
    Ptr<ThzNtnPhyGround> CreateGroundPhy(
        const std::string& preset,
        Ptr<SatPhy> utPhy = nullptr) const;

    // ---- Antenna array with frequency-derived geometry ----

    /**
     * \brief Create an antenna array with element spacing = lambda/2.
     *
     * Element spacing is computed from the preset frequency:
     *   spacing = c / (2 * freq)
     *
     * \param preset preset name
     * \return configured ThzNtnAntennaArray
     */
    Ptr<ThzNtnAntennaArray> CreateAntennaArray(const std::string& preset) const;

    // ---- MAC with absorption model connected ----

    /**
     * \brief Create a MAC layer with molecular absorption model connected.
     *
     * When absorption model is provided, it is wired into the MAC for
     * DAMC sub-band filtering.  If not provided, a new one is created
     * from the preset.
     *
     * \param preset preset name
     * \param absorption optional pointer to ThzNtnMolecularAbsorption
     * \return configured ThzNtnMac
     */
    Ptr<ThzNtnMac> CreateMac(
        const std::string& preset,
        Ptr<ThzNtnMolecularAbsorption> absorption = nullptr) const;

    // ---- Individual sub-model creation ----

    /**
     * \brief Create a molecular absorption model.
     * \return configured ThzNtnMolecularAbsorption
     */
    Ptr<ThzNtnMolecularAbsorption> CreateMolecularAbsorption() const;

    /**
     * \brief Create a weather attenuation model.
     * \return configured ThzNtnWeatherAttenuation
     */
    Ptr<ThzNtnWeatherAttenuation> CreateWeatherAttenuation() const;

    /**
     * \brief Create a scintillation model.
     * \return configured ThzNtnScintillation
     */
    Ptr<ThzNtnScintillation> CreateScintillation() const;

    /**
     * \brief Create a pointing error model.
     * \return configured ThzNtnPointingError
     */
    Ptr<ThzNtnPointingError> CreatePointingError() const;

    /**
     * \brief Create a hardware impairments model.
     * \return configured ThzNtnHardwareImpairments
     */
    Ptr<ThzNtnHardwareImpairments> CreateHardwareImpairments() const;

    // ---- ThzNtnFreeSpaceLoss with all sub-models ----

    /**
     * \brief Create a ThzNtnFreeSpaceLoss with all sub-models wired in.
     *
     * This is the primary satellite module integration point: the
     * returned object extends SatFreeSpaceLoss and can replace the
     * default in the satellite channel pipeline.
     *
     * \param preset preset name
     * \return configured ThzNtnFreeSpaceLoss with all sub-models
     */
    Ptr<ThzNtnFreeSpaceLoss> CreateFreeSpaceLoss(const std::string& preset) const;

    // ---- Other component creation ----

    /**
     * \brief Create a beamforming engine for the specified number of beams.
     * \param numBeams number of beams in the codebook
     * \return configured ThzNtnBeamforming
     */
    Ptr<ThzNtnBeamforming> CreateBeamforming(uint32_t numBeams) const;

    /**
     * \brief Create a beam tracking engine.
     * \return configured ThzNtnBeamTracking
     */
    Ptr<ThzNtnBeamTracking> CreateBeamTracking() const;

    /**
     * \brief Create an ISL channel model with preset parameters.
     * \param preset preset name
     * \return configured ThzNtnIslChannel
     */
    Ptr<ThzNtnIslChannel> CreateIslChannel(const std::string& preset) const;

    /**
     * \brief Create a link budget calculator with all sub-models connected.
     *
     * Wires ThzNtnFreeSpaceLoss and ThzNtnHardwareImpairments into the
     * link budget so that ComputeLinkBudget() uses actual sub-models.
     *
     * \param preset preset name (default "TeraLink-225GHz")
     * \return configured ThzNtnLinkBudget with connected sub-models
     */
    Ptr<ThzNtnLinkBudget> CreateLinkBudget(
        const std::string& preset = "TeraLink-225GHz") const;

    // ---- Full stack installation ----

    /**
     * \brief Install the full THz-NTN stack on a satellite node.
     *
     * Retrieves MobilityModel from the node, creates all components
     * (PHY, antenna, beamforming, beam tracking, MAC, channel), and
     * wires them together.  The satellite's MobilityModel is connected
     * to beam tracking and PHY Doppler pre-compensation.
     *
     * \param satNode the satellite ns-3 Node (must have MobilityModel)
     * \param preset preset name for configuration
     */
    void InstallOnSatellite(Ptr<Node> satNode, const std::string& preset);

    /**
     * \brief Install the full THz-NTN stack on a ground terminal node.
     *
     * Retrieves MobilityModel from the node, creates all components
     * (PHY, antenna, MAC), and wires them together.
     *
     * \param gtNode the ground terminal ns-3 Node (must have MobilityModel)
     * \param preset preset name for configuration
     */
    void InstallOnGroundTerminal(Ptr<Node> gtNode, const std::string& preset);

    /**
     * \brief Print the current configuration to the log.
     */
    void PrintConfiguration() const;
};

} // namespace ns3

#endif // THZ_NTN_HELPER_H
