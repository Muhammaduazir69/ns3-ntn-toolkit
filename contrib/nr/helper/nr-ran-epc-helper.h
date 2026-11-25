// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_RAN_EPC_HELPER_H
#define NR_RAN_EPC_HELPER_H

#include "nr-epc-helper.h"

#include "ns3/data-rate.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/nr-export.h"

namespace ns3
{

class NrEpcGnbApplication;

/**
 * @ingroup nr
 * @brief Base class of the EPC helpers, installing the RAN side of the RAN-core interfaces.
 *
 * This helper installs on each gNB node the NrEpcGnbApplication, which terminates the S1
 * interface (S1-AP control plane and GTP-U user plane) on the gNB side, and the NrEpcX2
 * entity, and connects them to the RRC of the gNB. It also creates the X2 links between gNBs.
 *
 * It does not create any core network node. Subclasses provide the core network, connecting
 * each gNB to it in ConnectGnbToCore(), and implement the rest of the NrEpcHelper interface.
 * NrNoBackhaulEpcHelper provides the built-in core network (MME, SGW and PGW).
 */
class NR_EXPORT NrRanEpcHelper : public NrEpcHelper
{
  public:
    /**
     * Register this type.
     * @return The object TypeId.
     */
    static TypeId GetTypeId();

    NrRanEpcHelper();

    // inherited from NrEpcHelper
    void AddGnb(Ptr<Node> gnbNode, Ptr<NetDevice> nrGnbNetDevice, uint16_t cellId) override;
    void AddX2Interface(Ptr<Node> gnbNode1, Ptr<Node> gnbNode2) override;
    void AddS1Interface(Ptr<Node> gnb,
                        Ipv4Address gnbAddress,
                        Ipv4Address sgwAddress,
                        uint16_t cellId) override;

  protected:
    /**
     * @return the index of this core network among the core networks of the simulation (the
     * smallest index not used by another one), which selects the address ranges of its links
     */
    uint32_t GetCoreIndex() const;

    /**
     * @param firstOctet the first octet of an IPv4 network used by the core networks
     * @return the IPv4 network of this core network: firstOctet.index.0.0
     */
    Ipv4Address GetCoreSubnet(uint8_t firstOctet) const;

    /**
     * @brief Connect a gNB to the core network, once its S1-U endpoint has been created.
     *
     * Implementations must connect the S1-AP SAPs of the gNB application to the core network
     * and make the core network aware of the gNB user-plane address.
     *
     * @param gnbApp the gNB application terminating the S1 interface on the gNB side
     * @param cellId the cell ID of the gNB
     * @param gnbAddress the S1-U address of the gNB
     * @param sgwAddress the S1-U address of the core network user-plane node
     */
    virtual void ConnectGnbToCore(Ptr<NrEpcGnbApplication> gnbApp,
                                  uint16_t cellId,
                                  Ipv4Address gnbAddress,
                                  Ipv4Address sgwAddress) = 0;

    /**
     * @brief DoAddX2Interface: Call AddX2Interface on top of the Gnb device pointers
     *
     * @param gnb1X2 EPCX2 of gNB1
     * @param gnb1NrDev NR device of gNB1
     * @param gnb1X2Address Address for gNB1
     * @param gnb2X2 EPCX2 of gNB2
     * @param gnb2NrDev NR device of gNB2
     * @param gnb2X2Address Address for gNB2
     */
    virtual void DoAddX2Interface(const Ptr<NrEpcX2>& gnb1X2,
                                  const Ptr<NetDevice>& gnb1NrDev,
                                  const Ipv4Address& gnb1X2Address,
                                  const Ptr<NrEpcX2>& gnb2X2,
                                  const Ptr<NetDevice>& gnb2NrDev,
                                  const Ipv4Address& gnb2X2Address) const;

  private:
    static constexpr uint16_t GTPU_UDP_PORT{2152}; ///< UDP port of GTP-U, fixed by the standard

    uint32_t m_coreIndex; ///< index of this core network among the core networks

    /**
     * helper to assign addresses to X2 NetDevices
     */
    Ipv4AddressHelper m_x2Ipv4AddressHelper;

    /**
     * The data rate to be used for the next X2 link to be created
     */
    DataRate m_x2LinkDataRate;

    /**
     * The delay to be used for the next X2 link to be created
     */
    Time m_x2LinkDelay;

    /**
     * The MTU of the next X2 link to be created. Note that,
     * because of some big X2 messages, you need a big MTU.
     */
    uint16_t m_x2LinkMtu;

    /**
     * Enable PCAP generation for X2 link
     */
    bool m_x2LinkEnablePcap;

    /**
     * Prefix for the PCAP file for the X2 link
     */
    std::string m_x2LinkPcapPrefix;
};

} // namespace ns3

#endif // NR_RAN_EPC_HELPER_H
