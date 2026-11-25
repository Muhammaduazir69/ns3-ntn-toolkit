// Copyright (c) 2018 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Manuel Requena <manuel.requena@cttc.es>
//         (based on epc-sgw-pgw-application.h)

#ifndef NR_EPC_PGW_APPLICATION_H
#define NR_EPC_PGW_APPLICATION_H

#include "nr-epc-gtpc-header.h"
#include "nr-qos-rule-classifier.h"

#include "ns3/application.h"
#include "ns3/nr-export.h"
#include "ns3/socket.h"
#include "ns3/virtual-net-device.h"

#include <set>

namespace ns3
{

/**
 * @ingroup nr
 *
 * This application implements the Packet Data Network (PDN) Gateway Entity (PGW)
 * according to the 3GPP TS 23.401 document.
 *
 * This Application implements the PGW side of the S5 interface between
 * the PGW node and the SGW nodes and the PGW side of the SGi interface between
 * the PGW node and the internet hosts. It supports the following functions and messages:
 *
 *  - S5 connectivity (i.e. GTPv2-C signalling and GTP-U data plane)
 *  - Bearer management functions including dedicated bearer establishment
 *  - Per-user based packet filtering
 *  - UL and DL bearer binding
 *  - Tunnel Management messages
 *
 * Others functions enumerated in section 4.4.3.3 of 3GPP TS 23.401 are not supported.
 */
class NR_EXPORT NrEpcPgwApplication : public Application
{
  public:
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();
    void DoDispose() override;

    /**
     * Constructor that binds the tap device to the callback methods.
     *
     * @param tunDevice TUN VirtualNetDevice used to tunnel IP packets from
     * the SGi interface of the PGW in the internet
     * over GTP-U/UDP/IP on the S5 interface
     * @param s5Addr IP address of the PGW S5 interface
     * @param s5uSocket socket used to send GTP-U packets to the peer SGW
     * @param s5cSocket socket used to send GTP-C packets to the peer SGW
     */
    NrEpcPgwApplication(const Ptr<VirtualNetDevice> tunDevice,
                        Ipv4Address s5Addr,
                        const Ptr<Socket> s5uSocket,
                        const Ptr<Socket> s5cSocket);

    /** Destructor */
    ~NrEpcPgwApplication() override;

    /**
     * Method to be assigned to the callback of the SGi TUN VirtualNetDevice.
     * It is called when the PGW receives a data packet from the
     * internet (including IP headers) that is to be sent to the UE via
     * its associated SGW and gNB, tunneling IP over GTP-U/UDP/IP.
     *
     * @param packet
     * @param source
     * @param dest
     * @param protocolNumber
     * @return true always
     */
    bool RecvFromTunDevice(Ptr<Packet> packet,
                           const Address& source,
                           const Address& dest,
                           uint16_t protocolNumber);

    /**
     * Method to be assigned to the callback of the egress device of an
     * unstructured PDU session. It is called when the PGW receives a data packet
     * to be sent to the UE of that session via its associated SGW and gNB. The
     * payload is unstructured, so the session it arrived on names its destination and
     * nothing is read from the packet itself.
     *
     * @param imsi the unique identifier of the UE of the session
     * @param qfi the QFI of the session
     * @param packet the packet
     * @param source unused
     * @param dest unused
     * @param protocolNumber unused
     * @return true always
     */
    bool RecvFromUnstructuredDevice(uint64_t imsi,
                                    uint8_t qfi,
                                    Ptr<Packet> packet,
                                    const Address& source,
                                    const Address& dest,
                                    uint16_t protocolNumber);

    /**
     * Method to be assigned to the receiver callback of the S5-U socket.
     * It is called when the PGW receives a data packet from the SGW
     * that is to be forwarded to the internet.
     *
     * @param socket pointer to the S5-U socket
     */
    void RecvFromS5uSocket(Ptr<Socket> socket);

    /**
     * Method to be assigned to the receiver callback of the S5-C socket.
     * It is called when the PGW receives a control packet from the SGW.
     *
     * @param socket pointer to the S5-C socket
     */
    void RecvFromS5cSocket(Ptr<Socket> socket);

    /**
     * Send a data packet to the internet via the SGi interface of the PGW
     *
     * @param packet packet to be sent
     * @param teid the Tunnel Endpoint Identifier
     */
    void SendToTunDevice(Ptr<Packet> packet, uint32_t teid);

    /**
     * Send a data packet to the SGW via the S5-U interface
     *
     * @param packet packet to be sent
     * @param sgwS5uAddress the address of the SGW
     * @param teid the Tunnel Endpoint Identifier
     */
    void SendToS5uSocket(Ptr<Packet> packet, Ipv4Address sgwS5uAddress, uint32_t teid);

    /**
     * Let the PGW be aware of a new SGW
     *
     * @param sgwS5Addr the address of the SGW S5 interface
     */
    void AddSgw(Ipv4Address sgwS5Addr);

    /**
     * Let the PGW be aware of a new UE
     *
     * @param imsi the unique identifier of the UE
     */
    void AddUe(uint64_t imsi);

    /**
     * Set the address of a previously added UE
     *
     * @param imsi the unique identifier of the UE
     * @param ueAddr the IPv4 address of the UE
     */
    void SetUeAddress(uint64_t imsi, Ipv4Address ueAddr);

    /**
     * @param imsi the unique identifier of a UE
     * @return whether the UE was added
     */
    bool HasUe(uint64_t imsi) const;

    /**
     * @param imsi the unique identifier of a previously added UE
     * @return the IPv4 address of the UE, or the default address if it has none
     */
    Ipv4Address GetUeAddress(uint64_t imsi) const;

    /**
     * @param imsi the unique identifier of a previously added UE
     * @return the IPv6 address of the UE, or the default address if it has none
     */
    Ipv6Address GetUeAddress6(uint64_t imsi) const;

    /**
     * set the address of a previously added UE
     *
     * @param imsi the unique identifier of the UE
     * @param ueAddr the IPv6 address of the UE
     */
    void SetUeAddress6(uint64_t imsi, Ipv6Address ueAddr);

    /**
     * Let the PGW be aware of an unstructured PDU session of a previously added UE
     * and of the device through which the session reaches the external network.
     *
     * An unstructured session carries a payload the network does not parse, so it
     * cannot be addressed the way an IP session is: the session has a device of its
     * own, and whatever is sent through that device goes to the UE of the session.
     *
     * @param imsi the unique identifier of the UE
     * @param qfi the QFI of the session
     * @param protocolNumber the network layer protocol carried by the session
     * @param device the egress device of the session
     */
    void AddUnstructuredSession(uint64_t imsi,
                                uint8_t qfi,
                                uint16_t protocolNumber,
                                Ptr<VirtualNetDevice> device);

    /**
     * Set the callback invoked when a session is created for an unstructured flow that has no
     * egress device on this PGW yet (e.g., the flow of a UE roaming with local breakout, whose
     * session is in the PGW of the visited network), so that its owner creates the device and
     * declares it with AddUnstructuredSession().
     *
     * @param cb the callback, with the IMSI of the UE, the QFI and the network layer protocol of
     * the session
     */
    void SetUnstructuredSessionRequestCallback(Callback<void, uint64_t, uint8_t, uint16_t> cb);

    /**
     * TracedCallback signature for data Packet reception event.
     *
     * @param [in] packet The data packet sent from the internet.
     */
    typedef void (*RxTracedCallback)(Ptr<Packet> packet);

  private:
    /**
     * Process Create Session Request message
     * @param packet GTPv2-C Create Session Request message
     */
    void DoRecvCreateSessionRequest(Ptr<Packet> packet);

    /**
     * Process Modify Flow Request message
     * @param packet GTPv2-C Modify Flow Request message
     */
    void DoRecvModifyFlowRequest(Ptr<Packet> packet);

    /**
     * Process Delete Flow Command message
     * @param packet GTPv2-C Delete Flow Command message
     */
    void DoRecvDeleteFlowCommand(Ptr<Packet> packet);

    /**
     * Process Delete Flow Response message
     * @param packet GTPv2-C Delete Flow Response message
     */
    void DoRecvDeleteFlowResponse(Ptr<Packet> packet);

    /**
     * Process Delete Session Request message: delete all the flows of the UE, if its session is
     * still the one of the SGW
     * @param packet GTPv2-C Delete Session Request message
     * @param sgwAddr the address of the SGW that sent the message
     */
    void DoRecvDeleteSessionRequest(Ptr<Packet> packet, Ipv4Address sgwAddr);

    /**
     * Tear down the state of an unstructured PDU session, undoing
     * AddUnstructuredSession(). It does nothing when the flow is not an
     * unstructured session, so it can be called for any flow being released.
     *
     * @param imsi the unique identifier of the UE of the session
     * @param qfi the QFI of the session
     * @param teid the TEID the session was bound to
     */
    void RemoveUnstructuredSession(uint64_t imsi, uint8_t qfi, uint32_t teid);

    /**
     * store info for each UE connected to this PGW
     */
    class NR_EXPORT NrUeInfo : public SimpleRefCount<NrUeInfo>
    {
      public:
        NrUeInfo();

        /**
         * Add a flow for this UE on PGW side
         *
         * @param qfi the ID of the QoS flow to be activated
         * @param teid  the TEID of the new flow
         * @param rule the QoS rule of the new flow to be added
         */
        void AddFlow(uint8_t qfi, uint32_t teid, Ptr<NrQosRule> rule);

        /**
         * Delete context of flow for this UE on PGW side
         *
         * @param qfi the ID of the QoS flow whose contexts is to be removed
         */
        void RemoveFlow(uint8_t qfi);

        /**
         * Delete the contexts of all the flows of this UE on PGW side
         */
        void RemoveFlows();

        /**
         * Classify the packet according to QoS rules of this UE
         *
         * This method performs QoS rule classification on downlink packets arriving from the
         * internet. The classifier matches the packet against the QoS rules registered for
         * this UE. Once a matching rule is found, the QFI is determined and used to look up
         * the corresponding TEID for GTP-U tunneling via m_teidByFlowIdMap.
         *
         * Data flow (downlink - Internet to UE):
         * 1. Internet packet arrives at PGW TUN device
         * 2. PGW::RecvFromTunDevice() calls this Classify() method
         * 3. Classifier matches packet to QoS rule -> determines QFI
         * 4. Method looks up TEID: m_teidByFlowIdMap[qfi]
         * 5. Method returns TEID (or std::nullopt if no match)
         * 6. PGW encapsulates packet in GTP-U with TEID for S5-U tunnel
         * 7. SGW receives GTP-U packet and forwards to gNB
         * 8. gNB receives GTP-U, extracts TEID, looks up (RNTI, QFI) in m_teidRqfiMap
         * 9. gNB routes to correct PDCP bearer
         *
         * @param p the IPv4 or IPv6 packet from the internet to be classified
         * @param protocolNumber identifies the type of packet.
         *        Only IPv4 and IPv6 packets are allowed.
         *
         * @return the TEID of the matching flow for GTP-U tunneling;
         * returns std::nullopt if no flow matches with the previously declared QoS rules
         */
        std::optional<uint32_t> Classify(Ptr<Packet> p, uint16_t protocolNumber);

        /**
         * Get the TEID of a flow of this UE by its QFI
         *
         * @param qfi the QFI of the flow
         *
         * @return the TEID of the flow, or std::nullopt if the flow is not established
         */
        std::optional<uint32_t> GetTeidByQfi(uint8_t qfi) const;

        /**
         * Get the address of the SGW to which the UE is connected
         *
         * @return the address of the SGW
         */
        Ipv4Address GetSgwAddr();

        /**
         * Set the address of the gNB to which the UE is connected
         *
         * @param addr the address of the SGW
         */
        void SetSgwAddr(Ipv4Address addr);

        /**
         * Get the IPv4 address of the UE
         *
         * @return the IPv4 address of the UE
         */
        Ipv4Address GetUeAddr();

        /**
         * Set the IPv4 address of the UE
         *
         * @param addr the IPv4 address of the UE
         */
        void SetUeAddr(Ipv4Address addr);

        /**
         * Get the IPv6 address of the UE
         *
         * @return the IPv6 address of the UE
         */
        Ipv6Address GetUeAddr6();

        /**
         * Set the IPv6 address of the UE
         *
         * @param addr the IPv6 address of the UE
         */
        void SetUeAddr6(Ipv6Address addr);

      private:
        Ipv4Address m_ueAddr;                    ///< UE IPv4 address
        Ipv6Address m_ueAddr6;                   ///< UE IPv6 address
        Ipv4Address m_sgwAddr;                   ///< SGW IPv4 address
        NrQosRuleClassifier m_qosRuleClassifier; ///< QoS rule classifier
        /**
         * Maps QFI to TEID for downlink packet routing.
         *
         * Usage (downlink - Internet to UE):
         * After QoS rule classification returns a QFI, this map is used to find the
         * corresponding TEID (Tunnel Endpoint Identifier) that was allocated by the SGW
         * and received during bearer setup in DoRecvCreateSessionRequest().
         *
         * The TEID is then used to encapsulate the packet in a GTP-U header for tunneling
         * through the S5-U interface to the SGW.
         *
         * Relationship to other identifiers:
         * - QFI (QoS Flow Identifier, 0-63): Allocated by MME, identifies a logical flow
         * - TEID (Tunnel Endpoint Identifier, 32-bit): Allocated by SGW, identifies tunnel
         * - At PGW: Both exist simultaneously, linked by this map
         * - At gNB: Only TEID exists on S1-U; gNB uses m_teidRqfiMap to recover (RNTI, QFI)
         *
         * Key: QFI (value: 0-63)
         * Value: TEID (value: 1-0xFFFFFFFF)
         *
         * Populated by: DoRecvCreateSessionRequest() when bearer is established
         * Updated by: DoRecvModifyFlowRequest() when bearer parameters change
         * Cleared by: DoRecvDeleteFlowResponse() when bearer is released
         */
        std::map<uint8_t, uint32_t> m_teidByFlowIdMap;

        /**
         * QFIs of the unstructured flows of this UE. Such a flow has no entry in
         * the classifier, so its removal has no classifier rule to delete.
         */
        std::set<uint8_t> m_unstructuredQfis;
    };

    /**
     * PGW address of the S5 interface
     */
    Ipv4Address m_pgwS5Addr;

    /**
     * UDP socket to send/receive GTP-U packets to/from the S5 interface
     */
    Ptr<Socket> m_s5uSocket;

    /**
     * UDP socket to send/receive GTPv2-C packets to/from the S5 interface
     */
    Ptr<Socket> m_s5cSocket;

    /**
     * TUN VirtualNetDevice used for tunneling/detunneling IP packets
     * from/to the internet over GTP-U/UDP/IP on the S5 interface
     */
    Ptr<VirtualNetDevice> m_tunDevice;

    /**
     * NrUeInfo stored by UE IPv4 address
     */
    std::map<Ipv4Address, Ptr<NrUeInfo>> m_ueInfoByAddrMap;

    /**
     * NrUeInfo stored by UE IPv6 address
     */
    std::map<Ipv6Address, Ptr<NrUeInfo>> m_ueInfoByAddrMap6;

    /**
     * NrUeInfo stored by IMSI
     */
    std::map<uint64_t, Ptr<NrUeInfo>> m_ueInfoByImsiMap;

    /**
     * The egress of an unstructured PDU session towards the external network
     */
    struct UnstructuredSession
    {
        Ptr<VirtualNetDevice> device; ///< device through which the session reaches the network
        uint16_t protocolNumber;      ///< network layer protocol carried by the session
    };

    /**
     * Unstructured session stored by (IMSI, QFI), as declared by AddUnstructuredSession()
     */
    std::map<std::pair<uint64_t, uint8_t>, UnstructuredSession> m_unstructuredSessionByUe;

    /**
     * Unstructured session stored by TEID, bound when the session is established.
     * An uplink packet is delivered to the device of its tunnel, with no inspection.
     */
    std::map<uint32_t, UnstructuredSession> m_unstructuredSessionByTeid;

    /// callback creating the egress device of an unstructured session that has none
    Callback<void, uint64_t, uint8_t, uint16_t> m_unstructuredSessionRequestCallback;

    /**
     * UDP port to be used for GTP-U
     */
    uint16_t m_gtpuUdpPort;

    /**
     * UDP port to be used for GTPv2-C
     */
    uint16_t m_gtpcUdpPort;

    /**
     * SGW address of the S5 interface
     */
    Ipv4Address m_sgwS5Addr;

    /**
     * @brief Callback to trace received data packets at Tun NetDevice from internet.
     */
    TracedCallback<Ptr<Packet>> m_rxTunPktTrace;

    /**
     * @brief Callback to trace received data packets from S5 socket.
     */
    TracedCallback<Ptr<Packet>> m_rxS5PktTrace;
};

} // namespace ns3

#endif // NR_EPC_PGW_APPLICATION_H
