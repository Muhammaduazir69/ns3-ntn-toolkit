// Copyright (c) 2017-2018 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Manuel Requena <manuel.requena@cttc.es>

#ifndef NR_EPC_MME_APPLICATION_H
#define NR_EPC_MME_APPLICATION_H

#include "nr-epc-gtpc-header.h"
#include "nr-epc-s1ap-sap.h"
#include "nr-qos-flow.h"

#include "ns3/application.h"
#include "ns3/nr-export.h"
#include "ns3/socket.h"
#include "ns3/traced-callback.h"

#include <list>
#include <map>
#include <optional>

namespace ns3
{

/**
 * @ingroup nr
 *
 * This application implements the Mobility Management Entity (MME) according to
 * the 3GPP TS 23.401 document.
 *
 * This Application implements the MME side of the S1-MME interface between
 * the MME node and the gNB nodes and the MME side of the S11 interface between
 * the MME node and the SGW node. It supports the following functions and messages:
 *
 *  - QoS flow management functions including dedicated flow establishment
 *  - NAS signalling
 *  - Tunnel Management messages
 *  - Roaming with local breakout: a UE that is not a subscriber of the network of the MME is
 *    attached if a roaming partner is its home network, which provides its subscription
 *    (Update Location, as on S6a), and releases its session in the network that served it
 *    before (Cancel Location)
 *
 * Others functions enumerated in section 4.4.2 of 3GPP TS 23.401 are not supported.
 */
class NR_EXPORT NrEpcMmeApplication : public Application
{
    /// allow NrMemberEpcS1apSapMme<EpcMme> class friend access
    friend class NrMemberEpcS1apSapMme<NrEpcMmeApplication>;

  public:
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();
    void DoDispose() override;

    /** Constructor */
    NrEpcMmeApplication();

    /** Destructor */
    ~NrEpcMmeApplication() override;

    /**
     *
     * @return the MME side of the S1-AP SAP
     */
    NrEpcS1apSapMme* GetS1apSapMme();

    /**
     * Add a new SGW to the MME
     *
     * @param sgwS11Addr IPv4 address of the SGW S11 interface
     * @param mmeS11Addr IPv4 address of the MME S11 interface
     * @param mmeS11Socket socket of the MME S11 interface
     */
    void AddSgw(Ipv4Address sgwS11Addr, Ipv4Address mmeS11Addr, Ptr<Socket> mmeS11Socket);

    /**
     * Add a new gNB to the MME
     *
     * @param ecgi E-UTRAN Cell Global ID, the unique identifier of the gNB
     * @param gnbS1UAddr IPv4 address of the gNB for S1-U communications
     * @param gnbS1apSap the gNB side of the S1-AP SAP
     */
    void AddGnb(uint16_t ecgi, Ipv4Address gnbS1UAddr, NrEpcS1apSapGnb* gnbS1apSap);

    /**
     * Add a new UE to the MME. This is the equivalent of storing the UE
     * credentials before the UE is ever turned on.
     *
     * @param imsi the unique identifier of the UE
     */
    void AddUe(uint64_t imsi);

    /**
     * Add a QosFlow to the list of flows to be activated for this UE.
     * The flow will be activated when the UE enters the ECM
     * connected state.
     *
     * @param imsi UE identifier
     * @param rule QoS rule of the flow
     * @param flow QoS flow (QoS characteristics)
     * @returns QoS flow ID
     */
    uint8_t AddFlow(uint64_t imsi, Ptr<NrQosRule> rule, NrQosFlow flow);

    /**
     * Hold info on an QoS flow to be activated
     */
    struct NR_EXPORT FlowInfo
    {
        Ptr<NrQosRule> rule; ///< QoS rule
        NrQosFlow flow;      ///< QoS flow (QOS characteristics)
        uint8_t qfi;         ///< QoS flow ID
    };

    /**
     * Add a roaming partner: the MME of another network, which is asked for the subscription of
     * the UEs that are not subscribers of the network of this MME.
     *
     * @param partner the MME of the other network
     */
    void AddRoamingPartner(Ptr<NrEpcMmeApplication> partner);

    /**
     * Update Location (as on S6a): another MME serves a UE, which releases its session in the
     * network that served it before.
     *
     * @param imsi the IMSI of the UE
     * @param mme the MME that serves the UE
     * @return the flows of the subscription of the UE, if it is a subscriber of the network of
     * this MME
     */
    std::optional<std::list<FlowInfo>> UpdateLocation(uint64_t imsi, NrEpcMmeApplication* mme);

    /**
     * Cancel Location (as on S6a): the UE is served by another MME, so its session is deleted.
     *
     * @param imsi the IMSI of the UE
     */
    void CancelLocation(uint64_t imsi);

    /**
     * Update Location from a network outside of the simulation (e.g., an external 5G core
     * network that roams with the network of this MME through a SEPP): it serves a subscriber,
     * which releases its session in the network of the simulation that served it before.
     *
     * @param imsi the IMSI of the UE
     * @return whether the UE is a subscriber of the network of this MME
     */
    bool ExternalUpdateLocation(uint64_t imsi);

    /**
     * Set the callback invoked when a network of the simulation serves a subscriber that was
     * served by a network outside of the simulation, so that the latter releases it.
     *
     * @param cb the callback, with the IMSI of the UE
     */
    void SetExternalCancelLocationCallback(Callback<void, uint64_t> cb);

    /**
     * Set the callback invoked when a UE attaches, before its session is created, so that the
     * core network sets the address of the UE and selects its PGW.
     *
     * @param cb the callback, with the IMSI of the UE and the MME of its home network (nullptr
     * for a subscriber), which returns the address of the PGW of the session (the default
     * address for the PGW of the network of the MME)
     */
    void SetUeAttachCallback(Callback<Ipv4Address, uint64_t, NrEpcMmeApplication*> cb);

    /**
     * TracedCallback signature for the attachment of a UE.
     *
     * @param [in] imsi the IMSI of the UE
     * @param [in] cellId the cell ID of the gNB
     * @param [in] roaming whether the UE is not a subscriber of the network of the MME
     */
    typedef void (*AttachTracedCallback)(uint64_t imsi, uint16_t cellId, bool roaming);

    /**
     * TracedCallback signature for an event about a UE.
     *
     * @param [in] imsi the IMSI of the UE
     */
    typedef void (*ImsiTracedCallback)(uint64_t imsi);

  private:
    // S1-AP SAP MME forwarded methods

    /**
     * Process the S1 Initial UE Message received from an gNB
     * @param mmeUeS1Id the MME UE S1 ID
     * @param gnbUeS1Id the gNB UE S1 ID
     * @param imsi the IMSI
     * @param ecgi the ECGI
     */
    void DoInitialUeMessage(uint64_t mmeUeS1Id, uint16_t gnbUeS1Id, uint64_t imsi, uint16_t ecgi);

    /**
     * Process the S1 Initial Context Setup Response received from an gNB
     * @param mmeUeS1Id the MME UE S1 ID
     * @param gnbUeS1Id the gNB UE S1 ID
     * @param erabSetupList the ERAB setup list
     */
    void DoInitialContextSetupResponse(uint64_t mmeUeS1Id,
                                       uint16_t gnbUeS1Id,
                                       std::list<NrEpcS1apSapMme::ErabSetupItem> erabSetupList);

    /**
     * Process the S1 Path Switch Request received from an gNB
     * @param mmeUeS1Id the MME UE S1 ID
     * @param gnbUeS1Id the gNB UE S1 ID
     * @param cgi the CGI
     * @param erabToBeSwitchedInDownlinkList the ERAB to be switched in downlink list
     */
    void DoPathSwitchRequest(
        uint64_t gnbUeS1Id,
        uint64_t mmeUeS1Id,
        uint16_t cgi,
        std::list<NrEpcS1apSapMme::ErabSwitchedInDownlinkItem> erabToBeSwitchedInDownlinkList);

    /**
     * Process ERAB Release Indication received from an gNB
     * @param mmeUeS1Id the MME UE S1 ID
     * @param gnbUeS1Id the gNB UE S1 ID
     * @param erabToBeReleaseIndication the ERAB to be release indication list
     */
    void DoErabReleaseIndication(
        uint64_t mmeUeS1Id,
        uint16_t gnbUeS1Id,
        std::list<NrEpcS1apSapMme::ErabToBeReleasedIndication> erabToBeReleaseIndication);

    // Methods to read/process GTP-C messages of the S11 interface

    /**
     * Reads the S11 messages from a socket
     * @param socket the socket
     */
    void RecvFromS11Socket(Ptr<Socket> socket);

    /**
     * Process GTP-C Create Session Response message
     * @param header the GTP-C header
     * @param packet the packet containing the message
     */
    void DoRecvCreateSessionResponse(NrGtpcHeader& header, Ptr<Packet> packet);

    /**
     * Process GTP-C Modify Flow Response message
     * @param header the GTP-C header
     * @param packet the packet containing the message
     */
    void DoRecvModifyFlowResponse(NrGtpcHeader& header, Ptr<Packet> packet);

    /**
     * Process GTP-C Delete Flow Request message
     * @param header the GTP-C header
     * @param packet the packet containing the message
     */
    void DoRecvDeleteFlowRequest(NrGtpcHeader& header, Ptr<Packet> packet);

    /**
     * Process the S1 UE Context Release Complete received from a gNB
     * @param mmeUeS1Id the MME UE S1 ID
     * @param gnbUeS1Id the gNB UE S1 ID
     */
    void DoUeContextReleaseComplete(uint64_t mmeUeS1Id, uint16_t gnbUeS1Id);

    /**
     * Process GTP-C Delete Session Response message
     * @param header the GTP-C header
     * @param packet the packet containing the message
     */
    void DoRecvDeleteSessionResponse(NrGtpcHeader& header, Ptr<Packet> packet);

    /**
     * Hold info on a UE
     */
    struct NR_EXPORT NrUeInfo : public SimpleRefCount<NrUeInfo>
    {
        uint64_t imsi;                          ///< UE identifier
        uint64_t mmeUeS1Id;                     ///< mmeUeS1Id
        uint16_t gnbUeS1Id;                     ///< gnbUeS1Id
        uint16_t cellId;                        ///< cell ID
        uint16_t flowCounter;                   ///< flow counter
        std::list<FlowInfo> flowsToBeActivated; ///< list of flows to be activated
        /// the MME of the home network of a roaming UE, nullptr for a subscriber
        NrEpcMmeApplication* home{nullptr};
        /// the MME that serves a subscriber, nullptr if none
        NrEpcMmeApplication* servingMme{nullptr};
        /// whether a network outside of the simulation serves the subscriber
        bool servingExternal{false};
    };

    /**
     * NrUeInfo stored by IMSI
     */
    std::map<uint64_t, Ptr<NrUeInfo>> m_ueInfoMap;

    /**
     * @brief This Function erases all contexts of flow from MME side
     * @param ueInfo UE information pointer
     * @param qosFlowId Flow Id which needs to be removed corresponding to UE
     */
    void RemoveFlow(Ptr<NrUeInfo> ueInfo, uint8_t qosFlowId);

    /**
     * Hold info on an gNB
     */
    struct NR_EXPORT GnbInfo : public SimpleRefCount<GnbInfo>
    {
        uint16_t gci;                ///< GCI
        Ipv4Address s1uAddr;         ///< IP address of the S1-U interface
        NrEpcS1apSapGnb* s1apSapGnb; ///< NrEpcS1apSapGnb
    };

    /**
     * GnbInfo stored by EGCI
     */
    std::map<uint16_t, Ptr<GnbInfo>> m_gnbInfoMap;

    NrEpcS1apSapMme* m_s1apSapMme; ///< NrEpcS1apSapMme

    Ptr<Socket> m_s11Socket;  ///< Socket to send/receive messages in the S11 interface
    Ipv4Address m_mmeS11Addr; ///< IPv4 address of the MME S11 interface
    Ipv4Address m_sgwS11Addr; ///< IPv4 address of the SGW S11 interface
    uint16_t m_gtpcUdpPort;   ///< UDP port for GTP-C protocol. Fixed by the standard to port 2123

    std::list<Ptr<NrEpcMmeApplication>> m_roamingPartners; ///< MMEs of the roaming partners
    /// callback on the attachment of a UE
    Callback<Ipv4Address, uint64_t, NrEpcMmeApplication*> m_ueAttachCallback;
    /// callback on the Cancel Location of a network outside of the simulation
    Callback<void, uint64_t> m_externalCancelLocationCallback;

    /**
     * A network of the simulation serves a subscriber: the network that served it before
     * releases it.
     *
     * @param ueInfo the subscriber
     * @param mme the MME of the network that serves it
     */
    void SetServingMme(Ptr<NrUeInfo> ueInfo, NrEpcMmeApplication* mme);

    /// Trace fired on the attachment of a UE
    TracedCallback<uint64_t, uint16_t, bool> m_attachTrace;
    /// Trace fired when the session of a UE is deleted, since another network serves it
    TracedCallback<uint64_t> m_cancelLocationTrace;
};

} // namespace ns3

#endif // NR_EPC_MME_APPLICATION_H
