/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN Inter-Satellite Link (ISL) Channel Model
 *
 * Vacuum propagation model for inter-satellite links at THz frequencies.
 * Integrates with the satellite module's SatFreeSpaceLoss for FSPL and
 * MobilityModel for actual distances / velocities.
 *
 * Features:
 *   - FSPL via SatFreeSpaceLoss (or ThzNtnFreeSpaceLoss extension)
 *   - Distance from actual MobilityModel positions
 *   - Doppler from actual satellite velocity vectors projected along LOS
 *   - Space thermal noise: cosmic microwave background (2.7 K) + receiver
 *   - Hardware impairment-limited capacity via ThzNtnHardwareImpairments
 *   - Intra-plane, inter-plane, and cross-link ISL distinctions
 *
 * References:
 *   [1] ITU-R S.1591, "Sharing between inter-satellite links and other
 *       services," 2002.
 *   [2] B. Di et al., "Ultra-dense LEO: Integration of satellite access
 *       networks into 5G and beyond," IEEE Wireless Commun., 2019.
 */

#ifndef THZ_NTN_ISL_CHANNEL_H
#define THZ_NTN_ISL_CHANNEL_H

#include <ns3/mobility-model.h>
#include <ns3/object.h>
#include <ns3/ptr.h>

#include <cstdint>
#include <string>

namespace ns3
{

// Forward declarations
class SatFreeSpaceLoss;
class ThzNtnHardwareImpairments;

/**
 * \brief Type of inter-satellite link.
 */
enum class ThzNtnIslType : uint8_t
{
    INTRA_PLANE = 0,   ///< Same orbital plane, adjacent satellite
    INTER_PLANE,       ///< Different orbital planes
    CROSS_LINK         ///< Cross-seam or long-range inter-plane
};

/**
 * \brief State of an individual ISL.
 */
struct ThzNtnIslLinkState
{
    uint32_t linkId;                ///< Link identifier
    uint32_t srcSatId;              ///< Source satellite ID
    uint32_t dstSatId;              ///< Destination satellite ID
    ThzNtnIslType islType;          ///< ISL type
    double distance_km;             ///< Current link distance (km)
    double relativeVelocity_m_s;    ///< Relative radial velocity (m/s)
    double doppler_Hz;              ///< Doppler shift (Hz)
    double fspl_dB;                 ///< Free-space path loss (dB)
    double rxPower_dBm;             ///< Received power (dBm)
    double snr_dB;                  ///< Signal-to-noise ratio (dB)
    double capacity_Gbps;           ///< Achievable capacity (Gbps)
    bool isEstablished;             ///< Whether the link is currently active
    double linkUpTime_s;            ///< Duration for which the link has been up (s)
};

/**
 * \ingroup thz-ntn
 * \brief ISL-specific THz channel model for vacuum propagation.
 *
 * Computes FSPL, Doppler, noise, SNR, and capacity for inter-satellite
 * links operating at THz frequencies.  All distances are derived from
 * actual MobilityModel positions, Doppler from velocity vectors, FSPL
 * from SatFreeSpaceLoss, and capacity is limited by hardware impairments.
 */
class ThzNtnIslChannel : public Object
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    ThzNtnIslChannel();
    ~ThzNtnIslChannel() override;

    // ---- Sub-model setters ----

    /**
     * \brief Set the free-space loss model from the satellite module.
     * \param fsl pointer to SatFreeSpaceLoss (or ThzNtnFreeSpaceLoss)
     */
    void SetFreeSpaceLossModel(Ptr<SatFreeSpaceLoss> fsl);

    /**
     * \brief Set the hardware impairments model.
     * \param hw pointer to ThzNtnHardwareImpairments
     */
    void SetHardwareModel(Ptr<ThzNtnHardwareImpairments> hw);

    // ---- MobilityModel-based computations ----

    /**
     * \brief Compute ISL signal-to-noise ratio from actual satellite positions.
     *
     * Uses MobilityModel::GetPosition() for distance, SatFreeSpaceLoss for
     * FSPL, and configured TX power / antenna gains.
     *
     * \param satA mobility model of satellite A
     * \param satB mobility model of satellite B
     * \return SNR in dB
     */
    double ComputeIslSnr_dB(Ptr<MobilityModel> satA,
                            Ptr<MobilityModel> satB) const;

    /**
     * \brief Compute complete link state from actual satellite mobility models.
     *
     * Distance from positions, Doppler from velocity vectors, FSPL from
     * SatFreeSpaceLoss, SNR and capacity from configured parameters.
     *
     * \param satA mobility model of satellite A
     * \param satB mobility model of satellite B
     * \return complete link state structure
     */
    ThzNtnIslLinkState ComputeLinkState(Ptr<MobilityModel> satA,
                                        Ptr<MobilityModel> satB) const;

    /**
     * \brief Compute relative Doppler shift from actual satellite velocity vectors.
     *
     * Gets velocity vectors from MobilityModel, projects relative velocity
     * along the line-of-sight direction.
     *
     * \param satA mobility model of satellite A
     * \param satB mobility model of satellite B
     * \return Doppler shift in Hz
     */
    double ComputeRelativeDoppler_Hz(Ptr<MobilityModel> satA,
                                     Ptr<MobilityModel> satB) const;

    /**
     * \brief Compute effective noise temperature in space.
     * \return noise temperature in Kelvin (cosmic background + receiver)
     */
    double ComputeSpaceNoiseTemperature_K() const;

    /**
     * \brief Compute ISL capacity with hardware impairment limitation.
     * \param snrDb SNR in dB
     * \return capacity in Gbps
     */
    double ComputeIslCapacity_Gbps(double snrDb) const;

    /**
     * \brief Check whether a link is feasible at given satellite positions.
     * \param satA mobility model of satellite A
     * \param satB mobility model of satellite B
     * \param minSnrDb minimum required SNR in dB
     * \return true if the link can be established
     */
    bool IsLinkFeasible(Ptr<MobilityModel> satA,
                        Ptr<MobilityModel> satB,
                        double minSnrDb) const;

    /**
     * \brief Compute maximum feasible ISL distance.
     * \param minSnrDb minimum required SNR in dB
     * \return maximum distance in km
     */
    double ComputeMaxLinkDistance_km(double minSnrDb) const;

  protected:
    void DoDispose() override;

  private:
    double m_frequency;           ///< Operating frequency (Hz)
    double m_txPower;             ///< Transmit power (dBm)
    double m_txGain;              ///< Transmit antenna gain (dBi)
    double m_rxGain;              ///< Receive antenna gain (dBi)
    double m_bandwidth;           ///< Channel bandwidth (Hz)
    double m_receiverNoiseTemp_K; ///< Receiver noise temperature (K)

    Ptr<SatFreeSpaceLoss> m_fsl;               ///< Free-space loss model
    Ptr<ThzNtnHardwareImpairments> m_hwModel;  ///< Hardware impairments model

    /// Physical constants
    static constexpr double SPEED_OF_LIGHT = 299792458.0;
    static constexpr double BOLTZMANN_K = 1.380649e-23;
    static constexpr double COSMIC_BACKGROUND_K = 2.725;
    static constexpr double PI_VAL = 3.14159265358979323846;
};

} // namespace ns3

#endif /* THZ_NTN_ISL_CHANNEL_H */
