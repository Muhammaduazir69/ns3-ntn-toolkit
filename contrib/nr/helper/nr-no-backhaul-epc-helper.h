// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Manuel Requena <manuel.requena@cttc.es>
//         (based on the original point-to-point-epc-helper.h)

#ifndef NR_NO_BACKHAUL_EPC_HELPER_H
#define NR_NO_BACKHAUL_EPC_HELPER_H

#include "nr-ran-epc-helper.h"

#include "ns3/ipv4-interface-address.h"
#include "ns3/ipv4.h"
#include "ns3/ipv6-interface-address.h"
#include "ns3/ipv6.h"
#include "ns3/nr-export.h"

#include <map>

namespace ns3
{

class NrEpcSgwApplication;
class NrEpcPgwApplication;
class NrEpcMmeApplication;

/**
 * @ingroup nr
 * @brief Create an EPC network with PointToPoint links between the core network nodes.
 *
 * This Helper will create an EPC network topology comprising of
 * three nodes: SGW, PGW and MME.
 * The X2-U, X2-C, S5 and S11 interfaces are realized over PointToPoint links.
 *
 * The S1 interface is not created. So, no backhaul network is built.
 * You have to build your own backhaul network in the simulation program.
 * Or you can use NrPointToPointEpcHelper or CsmaNrEpcHelper
 * (instead of this NrNoBackhaulEpcHelper) to use reference backhaul networks.
 */
class NR_EXPORT NrNoBackhaulEpcHelper : public NrRanEpcHelper
{
  public:
    /**
     * Constructor
     */
    NrNoBackhaulEpcHelper();

    /**
     * Destructor
     */
    ~NrNoBackhaulEpcHelper() override;

    // inherited from Object
    /**
     * Register this type.
     * @return The object TypeId.
     */
    static TypeId GetTypeId();
    void DoDispose() override;

    // inherited from NrEpcHelper
    void AddUe(Ptr<NetDevice> ueNrDevice, uint64_t imsi) override;
    uint8_t ActivateQosFlow(Ptr<NetDevice> ueNrDevice,
                            uint64_t imsi,
                            Ptr<NrQosRule> rule,
                            NrQosFlow flow) override;
    uint8_t ActivateUnstructuredQosFlow(Ptr<NetDevice> ueNrDevice,
                                        uint64_t imsi,
                                        uint16_t protocolNumber,
                                        NrQosFlow flow) override;
    Ptr<VirtualNetDevice> GetUnstructuredSessionDevice(uint64_t imsi, uint8_t qfi) const override;
    Ptr<Node> GetSgwNode() const override;
    Ptr<Node> GetPgwNode() const override;
    Ipv4InterfaceContainer AssignUeIpv4Address(NetDeviceContainer ueDevices) override;
    Ipv6InterfaceContainer AssignUeIpv6Address(NetDeviceContainer ueDevices) override;
    Ipv4Address GetUeDefaultGatewayAddress() override;
    Ipv6Address GetUeDefaultGatewayAddress6() override;
    int64_t AssignStreams(int64_t stream) override;
    std::pair<Ptr<Node>, Ipv4Address> SetupRemoteHost(std::optional<std::string> dataRate,
                                                      std::optional<uint16_t> mtu,
                                                      std::optional<Time> delay) override;
    std::pair<Ptr<Node>, Ipv6Address> SetupRemoteHost6(std::optional<std::string> dataRate,
                                                       std::optional<uint16_t> mtu,
                                                       std::optional<Time> delay) override;

    /**
     * Make the networks of this helper and of another one roaming partners: each one attaches
     * the subscribers of the other one, and the session of a UE in the network that served it
     * before is deleted.
     *
     * With local breakout, the session of a roaming UE is in the PGW of the visited network,
     * and the UE gets an IPv4 address of that network. With home-routed roaming, the SGW of
     * each network is connected to the PGW of the other one with an S8 link (S8Link*
     * attributes of this helper): the session of a roaming UE is in the PGW of its home
     * network, and the UE keeps its home address.
     *
     * @param partner the helper of the other network
     * @param homeRouted whether the roaming is home-routed, rather than with local breakout
     */
    void AddRoamingPartner(Ptr<NrNoBackhaulEpcHelper> partner, bool homeRouted = false);

    /**
     * @return the MME application of the network
     */
    Ptr<NrEpcMmeApplication> GetMmeApplication() const;

  protected:
    void NotifyConstructionCompleted() override;

    // inherited from NrRanEpcHelper
    void ConnectGnbToCore(Ptr<NrEpcGnbApplication> gnbApp,
                          uint16_t cellId,
                          Ipv4Address gnbAddress,
                          Ipv4Address sgwAddress) override;

    /**
     * @brief DoActivateQosFlowForUe: Schedule ActivateQosFlow on the UE
     * @param ueDevice NR device for the UE
     * @param rule QoS rule
     * @param flow QoS flow
     */
    virtual void DoActivateQosFlowForUe(const Ptr<NetDevice>& ueDevice,
                                        const Ptr<NrQosRule>& rule,
                                        const NrQosFlow& flow) const;

  private:
    /**
     * Select the PGW of a UE that attaches in the network, and set its IPv4 and IPv6 addresses
     * (for the families it has addresses of): its home addresses with home-routed roaming, or
     * else its addresses in this network, assigned on its first attachment if it has none.
     *
     * @param imsi the IMSI of the UE
     * @param home the MME of the home network of the UE, nullptr for a subscriber
     * @return the S5/S8 address of the PGW of the home network with home-routed roaming, or
     * the default address for the PGW of this network
     */
    Ipv4Address UeAttached(uint64_t imsi, NrEpcMmeApplication* home);

    /**
     * Create the egress device of an unstructured PDU session on the PGW, and declare it to the
     * PGW application. The device is kept for the session, if it already exists.
     *
     * @param imsi the IMSI of the UE
     * @param qfi the QFI of the session
     * @param protocolNumber the network layer protocol carried by the session
     */
    void CreateUnstructuredSessionDevice(uint64_t imsi, uint8_t qfi, uint16_t protocolNumber);

    /**
     * Set the IPv4 address of a UE, replacing its address and default route of another network.
     *
     * @param ueIpv4 the IPv4 stack of the UE
     * @param interface the interface of the NR device of the UE
     * @param address the address of the UE
     * @param gateway the default gateway of the UE
     */
    static void SetUeAddress(Ptr<Ipv4> ueIpv4,
                             int32_t interface,
                             Ipv4InterfaceAddress address,
                             Ipv4Address gateway);

    /**
     * Set the global IPv6 address of a UE, replacing its address and default route of another
     * network (its link-local address is kept).
     *
     * @param ueIpv6 the IPv6 stack of the UE
     * @param interface the interface of the NR device of the UE
     * @param address the address of the UE
     * @param gateway the default gateway of the UE
     */
    static void SetUeAddress6(Ptr<Ipv6> ueIpv6,
                              int32_t interface,
                              Ipv6InterfaceAddress address,
                              Ipv6Address gateway);

    /**
     * Create an S8 link between the SGW of a visited network and the PGW of a home network.
     *
     * @param visited the helper of the visited network
     * @param home the helper of the home network
     */
    void AddS8Link(Ptr<NrNoBackhaulEpcHelper> visited, Ptr<NrNoBackhaulEpcHelper> home);

    Ipv4Address m_pgwS5Address; ///< S5 address of the PGW
    Ipv4Address m_sgwS5Address; ///< S5 address of the SGW
    DataRate m_s8LinkDataRate;  ///< data rate of the S8 links
    Time m_s8LinkDelay;         ///< delay of the S8 links
    uint16_t m_s8LinkMtu;       ///< MTU of the S8 links
    /// helpers of the home networks with home-routed roaming, by their MME
    std::map<NrEpcMmeApplication*, NrNoBackhaulEpcHelper*> m_homeRoutedPartners;

    /**
     * helper to assign IPv4 addresses to UE devices as well as to the TUN device of the SGW/PGW
     */
    Ipv4AddressHelper m_uePgwAddressHelper;
    Ipv4Address m_ueIpv4Network; ///< IPv4 network of the UEs
    Ipv4Mask m_ueIpv4Mask;       ///< IPv4 mask of the network of the UEs
    /**
     * helper to assign IPv6 addresses to UE devices as well as to the TUN device of the SGW/PGW
     */
    Ipv6AddressHelper m_uePgwAddressHelper6;

    /**
     * PGW network element
     */
    Ptr<Node> m_pgw;

    /**
     * SGW network element
     */
    Ptr<Node> m_sgw;

    /**
     * MME network element
     */
    Ptr<Node> m_mme;

    /**
     * SGW application
     */
    Ptr<NrEpcSgwApplication> m_sgwApp;

    /**
     * PGW application
     */
    Ptr<NrEpcPgwApplication> m_pgwApp;

    /**
     * MME application
     */
    Ptr<NrEpcMmeApplication> m_mmeApp;

    /**
     * TUN device implementing tunneling of user data over GTP-U/UDP/IP
     */
    Ptr<VirtualNetDevice> m_tunDevice;

    /**
     * Egress device of each unstructured PDU session, stored by (IMSI, QFI)
     */
    std::map<std::pair<uint64_t, uint8_t>, Ptr<VirtualNetDevice>> m_unstructuredSessionDevices;

    /**
     * UDP port where the GTP-U Socket is bound, fixed by the standard as 2152
     */
    uint16_t m_gtpuUdpPort;

    /**
     * Helper to assign addresses to S11 NetDevices
     */
    Ipv4AddressHelper m_s11Ipv4AddressHelper;

    /**
     * The data rate to be used for the next S11 link to be created
     */
    DataRate m_s11LinkDataRate;

    /**
     * The delay to be used for the next S11 link to be created
     */
    Time m_s11LinkDelay;

    /**
     * The MTU of the next S11 link to be created
     */
    uint16_t m_s11LinkMtu;

    /**
     * UDP port where the GTPv2-C Socket is bound, fixed by the standard as 2123
     */
    uint16_t m_gtpcUdpPort;

    /**
     * S5 interfaces
     */

    /**
     * Helper to assign addresses to S5 NetDevices
     */
    Ipv4AddressHelper m_s5Ipv4AddressHelper;

    /**
     * The data rate to be used for the next S5 link to be created
     */
    DataRate m_s5LinkDataRate;

    /**
     * The delay to be used for the next S5 link to be created
     */
    Time m_s5LinkDelay;

    /**
     * The MTU of the next S5 link to be created
     */
    uint16_t m_s5LinkMtu;

    /**
     * Map storing for each IMSI the corresponding gNB NetDevice
     */
    std::map<uint64_t, Ptr<NetDevice>> m_imsiGnbDeviceMap;
};

} // namespace ns3

#endif // NR_NO_BACKHAUL_EPC_HELPER_H
