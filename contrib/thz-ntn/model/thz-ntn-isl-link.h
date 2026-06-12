/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN ISL Link Management
 *
 * Link establishment, teardown, routing, and adaptive rate control for
 * inter-satellite links in LEO/MEO constellations operating at THz.
 * Integrates with the satellite module's PointToPointIslNetDevice and
 * actual MobilityModel positions for topology management.
 *
 * Features:
 *   - Integration with SatPointToPointIslNetDevice
 *   - Topology from actual MobilityModel positions
 *   - Route computation using ThzNtnIslChannel link states
 *   - Data rate from actual SNR computation
 *   - Multi-hop ISL path computation
 *
 * References:
 *   [1] M. Handley, "Delay is not an option: low latency routing in space,"
 *       ACM HotNets, 2018.
 *   [2] B. Di et al., "Ultra-dense LEO: Integration of satellite access
 *       networks into 5G and beyond," IEEE Wireless Commun., 2019.
 */

#ifndef THZ_NTN_ISL_LINK_H
#define THZ_NTN_ISL_LINK_H

#include "thz-ntn-isl-channel.h"

#include <ns3/mobility-model.h>
#include <ns3/object.h>
#include <ns3/ptr.h>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ns3
{

// Forward declarations from satellite module
class PointToPointIslNetDevice;

/**
 * \brief Routing algorithm for multi-hop ISL paths.
 */
enum class ThzNtnRoutingAlgorithm : uint8_t
{
    SHORTEST_PATH = 0,
    MINIMUM_DELAY,
    LOAD_BALANCED
};

/**
 * \brief ISL link operational status.
 */
enum class ThzNtnIslLinkStatus : uint8_t
{
    DOWN = 0,
    ESTABLISHING,
    UP,
    DEGRADED,
    TEARING_DOWN
};

/**
 * \brief A computed multi-hop ISL route.
 */
struct ThzNtnIslRoute
{
    std::vector<uint32_t> hops;    ///< Ordered list of satellite IDs
    double totalDelay_ms;          ///< End-to-end propagation delay (ms)
    double totalCapacity_Gbps;     ///< Bottleneck capacity of the route (Gbps)
    uint32_t numHops;              ///< Number of hops
};

/**
 * \ingroup thz-ntn
 * \brief ISL link management for THz-NTN constellations.
 *
 * Manages the lifecycle of inter-satellite links: establishment based
 * on visibility constraints, adaptive data rate, and multi-hop routing.
 * Integrates with actual MobilityModel positions and PointToPointIslNetDevice.
 *
 * \warning Experimental: not yet exercised by any example or test.
 */
class ThzNtnIslLink : public Object
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    ThzNtnIslLink();
    ~ThzNtnIslLink() override;

    // ---- Sub-model setters ----

    /**
     * \brief Set the ISL channel model for link budget computation.
     * \param channel pointer to ThzNtnIslChannel
     */
    void SetIslChannel(Ptr<ThzNtnIslChannel> channel);

    /**
     * \brief Register an actual ISL net device from the satellite module.
     * \param linkId link identifier
     * \param device pointer to PointToPointIslNetDevice
     */
    void SetIslNetDevice(uint32_t linkId, Ptr<PointToPointIslNetDevice> device);

    // ---- Topology management from actual MobilityModels ----

    /**
     * \brief Update constellation topology from actual satellite MobilityModels.
     *
     * Computes all distances from MobilityModel positions, evaluates link
     * states via ThzNtnIslChannel, and tears down links exceeding max distance.
     *
     * \param satMobilities map of satellite ID to MobilityModel pointer
     */
    void UpdateTopology(const std::map<uint32_t, Ptr<MobilityModel>>& satMobilities);

    // ---- Link lifecycle ----

    /**
     * \brief Establish an ISL between two satellites using actual MobilityModels.
     * \param srcSatId source satellite ID
     * \param dstSatId destination satellite ID
     * \return true if the link was successfully established
     */
    bool EstablishLink(uint32_t srcSatId, uint32_t dstSatId);

    /**
     * \brief Tear down an existing ISL.
     * \param linkId link identifier
     */
    void TeardownLink(uint32_t linkId);

    /**
     * \brief Get the operational status of a link.
     * \param linkId link identifier
     * \return link status
     */
    ThzNtnIslLinkStatus GetLinkStatus(uint32_t linkId) const;

    /**
     * \brief Get the current data rate of a link.
     * \param linkId link identifier
     * \return data rate in Gbps
     */
    double GetCurrentDataRate_Gbps(uint32_t linkId) const;

    /**
     * \brief Adapt the data rate of a link based on actual SNR from MobilityModels.
     * \param linkId link identifier
     */
    void AdaptDataRate(uint32_t linkId);

    // ---- Routing ----

    /**
     * \brief Compute a multi-hop route between two satellites.
     * \param srcSatId source satellite ID
     * \param dstSatId destination satellite ID
     * \param algo routing algorithm to use
     * \return computed route
     */
    ThzNtnIslRoute ComputeRoute(uint32_t srcSatId,
                                uint32_t dstSatId,
                                ThzNtnRoutingAlgorithm algo) const;

    /**
     * \brief Compute end-to-end delay for a given route.
     * \param route the ISL route
     * \return delay in milliseconds
     */
    double ComputeEndToEndDelay_ms(const ThzNtnIslRoute& route) const;

    /**
     * \brief Set the routing algorithm.
     * \param algo routing algorithm
     */
    void SetRoutingAlgorithm(ThzNtnRoutingAlgorithm algo);

    /**
     * \brief Get IDs of all active links.
     * \return vector of active link IDs
     */
    std::vector<uint32_t> GetActiveLinkIds() const;

  protected:
    void DoDispose() override;

  private:
    /**
     * \brief Internal representation of a managed ISL.
     */
    struct ManagedLink
    {
        uint32_t linkId;
        uint32_t srcSatId;
        uint32_t dstSatId;
        ThzNtnIslLinkStatus status;
        double distance_km;
        double dataRate_Gbps;
        double snr_dB;
    };

    double m_maxLinkDistance_km;        ///< Maximum ISL distance (km)
    double m_minSnr_dB;                ///< Minimum SNR for link viability (dB)
    double m_linkSetupTime_ms;         ///< Link setup latency (ms)
    double m_maxDataRate_Gbps;         ///< Maximum ISL data rate (Gbps)
    std::string m_routingAlgoStr;      ///< Routing algorithm string for TypeId
    ThzNtnRoutingAlgorithm m_routingAlgo; ///< Active routing algorithm

    Ptr<ThzNtnIslChannel> m_islChannel;    ///< ISL channel model

    std::map<uint32_t, ManagedLink> m_links;   ///< All managed links by ID
    uint32_t m_nextLinkId;                     ///< Next available link ID

    /// Satellite MobilityModels: ID -> MobilityModel
    std::map<uint32_t, Ptr<MobilityModel>> m_satMobilities;

    /// Registered ISL net devices: linkId -> device
    std::map<uint32_t, Ptr<PointToPointIslNetDevice>> m_islDevices;

    /// Speed of light (m/s)
    static constexpr double SPEED_OF_LIGHT_M_S = 299792458.0;

    /// Minimum data rate for ISL link (Gbps)
    static constexpr double MIN_ISL_DATA_RATE_GBPS = 1.5;

    /**
     * \brief Map SNR to an achievable data rate.
     */
    double SnrToDataRate_Gbps(double snrDb) const;

    /**
     * \brief Parse routing algorithm string to enum.
     */
    static ThzNtnRoutingAlgorithm ParseRoutingAlgorithm(const std::string& s);
};

} // namespace ns3

#endif /* THZ_NTN_ISL_LINK_H */
