/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Master Composite THz-NTN Channel Model
 *
 * Unified propagation loss model for terahertz non-terrestrial network links
 * that aggregates all channel impairment components into a single ns-3
 * PropagationLossModel.  This serves as the mmWave-side integration point,
 * while ThzNtnFreeSpaceLoss serves as the satellite-module integration point.
 *
 * The model internally delegates to ThzNtnFreeSpaceLoss, which extends
 * SatFreeSpaceLoss with THz atmospheric effects.  This dual-interface design
 * allows THz channel effects to be applied through either:
 *   (a) The satellite module's SatChannel -> SatFreeSpaceLoss::GetFsl() path
 *   (b) The mmWave/PropagationLossModel -> DoCalcRxPower() path
 *
 * Link types are automatically detected from node positions:
 *   - ISL: both nodes > 100 km altitude (no atmospheric effects)
 *   - Satellite-ground: one node in space, one on ground
 *   - HAP relay: intermediate altitude nodes
 *
 * Configurable bands with built-in frequency presets:
 *   - "D-band-140GHz"   -> 140 GHz
 *   - "TeraLink-225GHz" -> 225 GHz (default)
 *   - "H-band-300GHz"   -> 300 GHz
 *   - "ISL-300GHz"      -> 300 GHz, ISL mode
 *
 * References:
 *   [1] J. M. Jornet and I. F. Akyildiz, "Channel modeling and capacity
 *       analysis for electromagnetic wireless nanonetworks in the THz band,"
 *       IEEE Trans. Wireless Commun., vol. 10, no. 10, 2011.
 *   [2] 3GPP TR 38.811, "Study on New Radio to support non-terrestrial
 *       networks," v15.4.0, 2020.
 *   [3] ITU-R P.619-5, "Propagation data for interference evaluation"
 */

#ifndef THZ_NTN_CHANNEL_MODEL_H
#define THZ_NTN_CHANNEL_MODEL_H

#include <ns3/propagation-loss-model.h>
#include <ns3/ptr.h>
#include <ns3/traced-callback.h>

#include <map>
#include <string>

namespace ns3
{

// Forward declarations of component models
class ThzNtnFreeSpaceLoss;
class ThzNtnMolecularAbsorption;
class ThzNtnWeatherAttenuation;
class ThzNtnScintillation;
class ThzNtnPointingError;
class ThzNtnHardwareImpairments;
class ThzNtnSpectrum;

/**
 * \ingroup thz-ntn
 * \brief Per-link THz signal metadata for KPM extraction.
 *
 * Stores the breakdown of channel impairments from the most recent
 * computation for a given transmitter-receiver pair, enabling O-RAN
 * KPM reporting of individual THz channel components.
 */
struct ThzNtnSignalInfo
{
    double totalLoss_dB;           ///< total path loss (FSPL + all effects)
    double baseFspl_dB;            ///< free-space path loss component
    double molecularAbsorption_dB; ///< molecular absorption component
    double weatherLoss_dB;         ///< weather attenuation component
    double scintillationLoss_dB;   ///< tropospheric scintillation component
    double pointingLoss_dB;        ///< beam pointing error component
    double elevationDeg;           ///< elevation angle of the link
    double distanceM;              ///< 3-D link distance in metres
    double frequencyHz;            ///< carrier frequency used
    bool isIsl;                    ///< true if inter-satellite link
};

/**
 * \ingroup thz-ntn
 * \brief Master composite channel model for THz non-terrestrial network links
 *
 * This PropagationLossModel implementation combines all THz-NTN channel
 * impairment components into a single model.  Individual components can be
 * enabled or disabled independently.  The model automatically detects ISL
 * vs. atmospheric links from node altitudes and applies the appropriate
 * subset of impairments.
 *
 * Internally, all THz loss computations are delegated to ThzNtnFreeSpaceLoss,
 * which can also be retrieved via GetThzFreeSpaceLoss() and installed directly
 * into the satellite module as a SatFreeSpaceLoss replacement.
 *
 * Usage (mmWave path):
 * \code
 *   auto channel = CreateObject<ThzNtnChannelModel>();
 *   channel->SetBand("TeraLink-225GHz");
 *   channel->EnableWeatherEffects(true);
 *   lossModel->SetNext(channel);
 * \endcode
 *
 * Usage (satellite module path):
 * \code
 *   auto channel = CreateObject<ThzNtnChannelModel>();
 *   channel->SetBand("TeraLink-225GHz");
 *   auto thzFsl = channel->GetThzFreeSpaceLoss();
 *   satHelper->SetFreeSpaceLoss(thzFsl);
 * \endcode
 */
class ThzNtnChannelModel : public PropagationLossModel
{
  public:
    /**
     * \brief Get the TypeId for this class.
     * \return the ns-3 TypeId
     */
    static TypeId GetTypeId();

    ThzNtnChannelModel();
    ~ThzNtnChannelModel() override;

    /**
     * \brief Link type enumeration.
     */
    enum LinkType
    {
        GROUND_TO_SAT = 0,  ///< Ground station to satellite
        SAT_TO_GROUND,      ///< Satellite to ground station
        INTER_SATELLITE,    ///< Inter-satellite link (vacuum)
        SAT_TO_HAP,         ///< Satellite to high-altitude platform
        HAP_TO_GROUND,      ///< High-altitude platform to ground
        AUTO_DETECT          ///< Automatically detect from node positions
    };

    // ---- Satellite module integration ----

    /**
     * \brief Get the ThzNtnFreeSpaceLoss object for satellite module integration.
     *
     * The returned object extends SatFreeSpaceLoss and can be used as a
     * drop-in replacement in the satellite channel pipeline.
     *
     * \return Ptr to ThzNtnFreeSpaceLoss
     */
    Ptr<ThzNtnFreeSpaceLoss> GetThzFreeSpaceLoss() const;

    // ---- Configuration ----

    /**
     * \brief Set the link type explicitly.
     *
     * If set to AUTO_DETECT (default), the link type is inferred from
     * node altitudes in DoCalcRxPower.
     *
     * \param type link type enumeration value
     */
    void SetLinkType(LinkType type);

    /**
     * \brief Set the operating band by name.
     *
     * Supported bands:
     *   "D-band-140GHz"   -> 140 GHz
     *   "TeraLink-225GHz" -> 225 GHz
     *   "H-band-300GHz"   -> 300 GHz
     *   "ISL-300GHz"      -> 300 GHz, forced ISL mode
     *
     * \param band band name string
     */
    void SetBand(const std::string& band);

    /**
     * \brief Enable or disable molecular absorption.
     * \param enable true to enable
     */
    void EnableMolecularAbsorption(bool enable);

    /**
     * \brief Enable or disable weather effects (rain, fog, snow, dust).
     * \param enable true to enable
     */
    void EnableWeatherEffects(bool enable);

    /**
     * \brief Enable or disable tropospheric scintillation.
     * \param enable true to enable
     */
    void EnableScintillation(bool enable);

    /**
     * \brief Enable or disable pointing error modelling.
     * \param enable true to enable
     */
    void EnablePointingError(bool enable);

    /**
     * \brief Enable or disable hardware impairment modelling.
     * \param enable true to enable
     */
    void EnableHardwareImpairments(bool enable);

    // ---- Component model setters ----

    /**
     * \brief Set the molecular absorption model.
     * \param model pointer to ThzNtnMolecularAbsorption
     */
    void SetMolecularAbsorptionModel(Ptr<ThzNtnMolecularAbsorption> model);

    /**
     * \brief Set the weather attenuation model.
     * \param model pointer to ThzNtnWeatherAttenuation
     */
    void SetWeatherModel(Ptr<ThzNtnWeatherAttenuation> model);

    /**
     * \brief Set the scintillation model.
     * \param model pointer to ThzNtnScintillation
     */
    void SetScintillationModel(Ptr<ThzNtnScintillation> model);

    /**
     * \brief Set the pointing error model.
     * \param model pointer to ThzNtnPointingError
     */
    void SetPointingErrorModel(Ptr<ThzNtnPointingError> model);

    /**
     * \brief Set the hardware impairments model.
     * \param model pointer to ThzNtnHardwareImpairments
     */
    void SetHardwareModel(Ptr<ThzNtnHardwareImpairments> model);

    /**
     * \brief Set the spectrum model for atmospheric window data.
     * \param model pointer to ThzNtnSpectrum
     */
    void SetSpectrumModel(Ptr<ThzNtnSpectrum> model);

    // ---- Result accessors (cached from last computation) ----

    /**
     * \brief Get the total path loss from the last DoCalcRxPower call.
     * \return total path loss in dB
     */
    double GetLastPathLoss_dB() const;

    /**
     * \brief Get the molecular absorption component from the last computation.
     * \return molecular absorption loss in dB
     */
    double GetLastMolecularAbsorption_dB() const;

    /**
     * \brief Get the per-link signal info for a given link key.
     *
     * The key is formed as "txNodeId-rxNodeId" and entries are stored
     * from the most recent DoCalcRxPower call for each unique link.
     *
     * \param linkKey string key "txNodeId-rxNodeId"
     * \param[out] info the ThzNtnSignalInfo struct (populated if found)
     * \return true if the link key was found
     */
    bool GetLinkSignalInfo(const std::string& linkKey,
                           ThzNtnSignalInfo& info) const;

    // ---- Trace sources ----

    /**
     * \brief Traced callback for total path loss: (pathLoss_dB)
     */
    TracedCallback<double> m_pathLossTrace;

    /**
     * \brief Traced callback for molecular absorption: (absorption_dB)
     */
    TracedCallback<double> m_molecularAbsorptionTrace;

    /**
     * \brief Traced callback for weather loss: (weatherLoss_dB)
     */
    TracedCallback<double> m_weatherLossTrace;

    /**
     * \brief Traced callback for pointing loss: (pointingLoss_dB)
     */
    TracedCallback<double> m_pointingLossTrace;

    /**
     * \brief Traced callback for total received power: (rxPower_dBm)
     */
    TracedCallback<double> m_totalRxPowerTrace;

  protected:
    void DoDispose() override;

    /**
     * \brief Compute received power combining all channel components.
     *
     * Delegates THz loss computation to the internal ThzNtnFreeSpaceLoss
     * object, which applies base FSPL + molecular absorption + weather +
     * scintillation + pointing error.  Hardware impairment degradation is
     * applied afterwards.
     *
     * \param txPowerDbm transmit power in dBm
     * \param a          transmitter mobility model
     * \param b          receiver mobility model
     * \return received power in dBm
     */
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;

    /**
     * \brief Assign random variable streams.
     * \param stream first stream index to use
     * \return number of streams consumed
     */
    int64_t DoAssignStreams(int64_t stream) override;

  private:
    /**
     * \brief Determine the link type from node altitudes.
     *
     * Both nodes > 100 km -> INTER_SATELLITE
     * One node > 100 km, other < 20 km -> GROUND_TO_SAT or SAT_TO_GROUND
     * Otherwise -> SAT_TO_HAP or HAP_TO_GROUND
     *
     * \param altA_km altitude of node A in km
     * \param altB_km altitude of node B in km
     * \return detected LinkType
     */
    LinkType DetectLinkType(double altA_km, double altB_km) const;

    /**
     * \brief Parse a band name string to set the centre frequency.
     * \param band band name string
     */
    void ParseBandName(const std::string& band);

    /**
     * \brief Build a link key string from two mobility models.
     * \param a first node
     * \param b second node
     * \return "nodeIdA-nodeIdB" string
     */
    std::string MakeLinkKey(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const;

    // Configuration
    LinkType m_linkType;            ///< configured or auto-detected link type
    std::string m_bandName;         ///< operating band name
    double m_centerFreqHz;          ///< centre frequency in Hz

    // Enable flags
    bool m_enableMolecular;         ///< enable molecular absorption
    bool m_enableWeather;           ///< enable weather effects
    bool m_enableScintillation;     ///< enable scintillation
    bool m_enablePointing;          ///< enable pointing error
    bool m_enableHardware;          ///< enable hardware impairments

    // Internal ThzNtnFreeSpaceLoss (the integration bridge)
    Ptr<ThzNtnFreeSpaceLoss> m_thzFsl;  ///< THz free-space loss extension

    // Component model pointers (also installed into m_thzFsl)
    Ptr<ThzNtnMolecularAbsorption> m_molecularModel;  ///< molecular absorption model
    Ptr<ThzNtnWeatherAttenuation> m_weatherModel;      ///< weather attenuation model
    Ptr<ThzNtnScintillation> m_scintillationModel;     ///< scintillation model
    Ptr<ThzNtnPointingError> m_pointingModel;          ///< pointing error model
    Ptr<ThzNtnHardwareImpairments> m_hardwareModel;    ///< hardware impairments model
    Ptr<ThzNtnSpectrum> m_spectrumModel;               ///< spectrum/window model

    // Cached results from last computation
    mutable double m_lastPathLoss_dB;             ///< cached total path loss
    mutable double m_lastMolecularAbsorption_dB;  ///< cached molecular absorption
    mutable double m_lastWeatherLoss_dB;          ///< cached weather loss
    mutable double m_lastPointingLoss_dB;         ///< cached pointing loss
    mutable double m_lastScintillationLoss_dB;    ///< cached scintillation loss

    // Per-link signal info map for KPM reporting
    mutable std::map<std::string, ThzNtnSignalInfo> m_linkInfoMap;

    // Altitude thresholds
    static constexpr double ISL_ALTITUDE_THRESHOLD_KM = 100.0;
    static constexpr double HAP_ALTITUDE_THRESHOLD_KM = 20.0;
};

} // namespace ns3

#endif // THZ_NTN_CHANNEL_MODEL_H
