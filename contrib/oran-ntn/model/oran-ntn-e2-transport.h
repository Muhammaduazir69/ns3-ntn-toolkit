/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * OranNtnE2Transport (Wave 2 M2, P0-4): E2 as packets over the feeder.
 *
 * Opt-in replacement for the event-delay E2 delivery of OranNtnE2Node
 * (Simulator::Schedule(FeederLinkDelay)). Each attached E2 node gets its own
 * ns-3 node (the E2 agent on the satellite / gNB side) and a point-to-point
 * link to one Near-RT RIC node. The link delay is the LIVE satellite-gateway
 * slant / c plus a terrestrial gateway-to-RIC delay, refreshed periodically
 * from the mobility models, and the link rate is the control bearer rate, so
 * an E2 message sees propagation, serialization, queueing and loss.
 *
 * E2AP procedures carried (O-RAN.WG3.E2AP v03.00 procedure codes):
 *   E2 Setup (1)            agent -> RIC request, RIC -> agent response,
 *                           with the RAN function list (KPM, RC)
 *   RIC Subscription (8)    RIC -> agent request, agent -> RIC response
 *   RIC Indication (5)      agent -> RIC, E2SM-KPM v03.00 Indication
 *                           Message Format 1 (TS 28.552 measurement names,
 *                           noValue for anything not measured)
 *   RIC Control (4)         RIC -> agent request, agent -> RIC acknowledge
 * No indication is sent before the E2 Setup and the RIC Subscription of its
 * subscription have completed on the link.
 *
 * Fidelity statement: messages travel as UDP datagrams to port 36421 (the
 * IANA E2AP SCTP port) and are padded by the SCTP common header (12 B) plus
 * one DATA chunk header (16 B), as oran-ntn-feeder-mgmt does; SCTP itself
 * (association, retransmission, PPID 70) is not simulated, so a datagram lost
 * on the feeder is a lost E2 message. The E2AP envelope is a compact
 * toolkit framing (procedure code, message type, node id, sequence number,
 * creation time), not ASN.1 APER (deferred item A4); the E2SM-KPM payload is
 * the in-tree OranNtnServiceModelKpm codec and is DECODED at the RIC, so the
 * xApps act on the bytes that crossed the link.
 *
 * Wave 7B: reliable transport (SetProtocol(Protocol::Tcp), attribute
 * "Protocol"). ns-3 has no SCTP (O-RAN.WG3.E2GAP and E2AP v03.00 Section 7
 * mandate SCTP, RFC 9260, with PPID 70 on port 36421). With Tcp, each E2 node
 * opens ONE ns-3 TCP connection from its agent to the RIC (the analog of the
 * one SCTP association per E2 node), and every E2AP message is written to
 * the byte stream with a 4-byte length prefix; the receiver reassembles the
 * stream and dispatches each complete message, so a message larger than one
 * segment, or one that straddles segments, arrives whole or not at all. The
 * E2AP message bytes are the same as in Udp mode; the 28 B SCTP common plus
 * DATA chunk header padding is replaced by the 4 B length prefix, and TCP's
 * own header (20 B plus 12 B timestamps option) stands in for SCTP's on the
 * wire. E2 Setup is sent only once the connection is established, the RIC
 * subscribes after the setup, and no indication leaves before the
 * subscription is accepted, as in Udp mode.
 *
 * Reproduced from SCTP: reliable, ordered delivery per association with
 * retransmission on loss (RTO.Min 1 s as in RFC 9260 Section 16, ns-3 TCP
 * MinRto default), a bounded retransmission count after which the
 * association is declared lost (attribute "MaxRetrans", default 10 =
 * RFC 9260 Association.Max.Retrans; "MaxInitRetrans", default 8 =
 * Max.Init.Retransmits for the connection setup), and re-establishment: on
 * loss of the connection the agent re-connects after "ReconnectDelay" and
 * repeats E2 Setup, and the RIC re-subscribes, as an E2 node does after an
 * SCTP restart. Every E2 message therefore sees propagation, serialization,
 * queueing, retransmission and head-of-line waiting.
 *
 * Not reproduced: SCTP multi-streaming (E2AP uses one stream for global
 * procedures and others per RAN function; over one TCP byte stream a lost
 * segment holds back every later message of the node, so head-of-line
 * blocking is stronger than in SCTP), message-oriented framing (emulated by
 * the length prefix), the PPID 70 field, the 4-way INIT/INIT-ACK/COOKIE
 * handshake (TCP uses 3 ways, one round trip less before E2 Setup),
 * HEARTBEAT path supervision (HB.interval 30 s; a silent dead path is found
 * only when data is sent), multi-homing and SACK-based fast retransmit
 * differences between TCP NewReno and SCTP. Udp mode is still the default so
 * published runs reproduce.
 */

#ifndef ORAN_NTN_E2_PACKET_TRANSPORT_H
#define ORAN_NTN_E2_PACKET_TRANSPORT_H

#include "oran-ntn-types.h"

#include "ns3/data-rate.h"
#include "ns3/event-id.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/ptr.h"

#include <cstdint>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace ns3
{

class OranNtnE2Node;
struct E2Indication;
class Socket;
class Packet;
class Address;
class PointToPointChannel;
class RateErrorModel;

/**
 * \ingroup oran-ntn
 * \brief E2AP messages as packets over a per-node feeder link (opt-in).
 */
class OranNtnE2Transport : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnE2Transport();
    ~OranNtnE2Transport() override;

    /// Transport under E2AP (see the file comment).
    enum class Protocol : uint8_t
    {
        Udp, //!< one datagram per message, SCTP header size as padding (default)
        Tcp, //!< one reliable connection per E2 node, length-prefixed messages
    };
    /// Select the transport; call before Attach().
    void SetProtocol(Protocol p);
    Protocol GetProtocol() const { return m_proto; }
    /// Packet error rate on both directions of every feeder link (also the
    /// rate restored after an outage). Call before or after Attach().
    void SetFeederPacketErrorRate(double per);

    /// E2AP v03.00 procedure codes used here.
    enum ProcedureCode : uint8_t
    {
        kE2Setup = 1,
        kRicControl = 4,
        kRicIndication = 5,
        kRicSubscription = 8,
    };
    /// E2AP-PDU CHOICE.
    enum MessageType : uint8_t
    {
        kInitiating = 0,
        kSuccessful = 1,
        kUnsuccessful = 2,
    };

    void SetBearerRate(DataRate r) { m_rate = r; }
    void SetGatewayToRicDelay(Time d) { m_gwToRic = d; }
    void SetRefreshPeriod(Time p) { m_refresh = p; }
    /// CSV trace of every E2 message sent and received (empty = off).
    void SetTraceFile(const std::string& path);

    /**
     * \brief Attach an E2 node. The feeder delay is |sat - gw| / c + the
     * gateway-to-RIC delay, refreshed every SetRefreshPeriod. Either mobility
     * may be null: the node's configured FeederLinkDelay is then used as the
     * (constant) link delay.
     */
    void Attach(Ptr<OranNtnE2Node> node, Ptr<MobilityModel> sat, Ptr<MobilityModel> gw);

    /// Start the E2 Setup of every attached node at `at`.
    void Start(Time at);

    /// Feeder outage on one node's link: every packet is lost in [start, start + dur).
    void ScheduleOutage(uint32_t gnbId, Time start, Time duration);

    // ---- Called by OranNtnE2Node when a transport is bound ----
    void SendIndication(Ptr<OranNtnE2Node> node, const E2Indication& ind);
    void SendControl(Ptr<OranNtnE2Node> node, const E2RcAction& action);

    // ---- Counters ----
    struct Counters
    {
        uint64_t setupDone{0};
        uint64_t subscriptionsDone{0};
        uint64_t indicationsSent{0};
        uint64_t indicationsReceived{0};
        uint64_t indicationsBeforeSubscription{0}; //!< suppressed: no E2AP context yet
        uint64_t controlsSent{0};
        uint64_t controlsBeforeSetup{0}; //!< not sent: no E2 Setup on the link yet
        uint64_t controlsReceived{0};
        uint64_t controlAcks{0};
        uint64_t bytesUp{0};
        uint64_t bytesDown{0};
        uint64_t decodeFailures{0};
        // Tcp mode (Wave 7B)
        uint64_t connectionsEstablished{0}; //!< agent-side connect successes
        uint64_t connectFailures{0};        //!< connect gave up (MaxInitRetrans)
        uint64_t associationsLost{0};       //!< established connection lost (MaxRetrans)
        uint64_t indicationsNoAssociation{0}; //!< indications while the connection was down
        uint64_t sendBlocked{0};            //!< messages queued because the TX buffer was full
        uint64_t outOfOrder{0};             //!< messages that arrived with an older sequence
    };
    const Counters& GetCounters() const { return m_c; }
    /// Mean / sd of the measured indication one-way delay (ms).
    double GetIndicationOwdMeanMs() const;
    double GetIndicationOwdSdMs() const;
    /// Mean configured link delay (ms) sampled at every indication sent.
    double GetMeanLinkDelayMs() const;
    /// Simulated instant the first indication reached the RIC (-1 s if none).
    Time GetFirstIndicationTime() const { return m_firstIndication; }
    /// Instant the last E2 Setup Response reached its agent (-1 s if none).
    Time GetLastSetupDoneTime() const { return m_lastSetupDone; }
    /// Largest gap between consecutive indications received from one node (s).
    double GetMaxIndicationGapS() const;

    /// Encode / decode the measured report as an E2SM-KPM v03.00 Format 1 body
    /// (public for the unit test).
    static std::vector<uint8_t> EncodeKpm(const E2KpmReport& r);
    static bool DecodeKpm(const std::vector<uint8_t>& bytes, E2KpmReport& out);
    static std::vector<uint8_t> EncodeControl(const E2RcAction& a);
    static bool DecodeControl(const std::vector<uint8_t>& bytes, E2RcAction& out);

  protected:
    void DoDispose() override;

  private:
    struct Link
    {
        Ptr<OranNtnE2Node> e2;
        Ptr<Node> agent;
        Ptr<Socket> agentSock;
        Ptr<PointToPointChannel> ch;
        Ptr<RateErrorModel> emAgent;
        Ptr<RateErrorModel> emRic;
        Ptr<MobilityModel> sat;
        Ptr<MobilityModel> gw;
        uint32_t agentAddr{0};
        uint32_t ricAddr{0};
        bool setupDone{false};
        std::set<uint32_t> subsRequested; //!< at the RIC
        std::set<uint32_t> subsActive;    //!< at the agent
        uint32_t seqUp{0};
        uint32_t seqDown{0};
        Time lastIndicationRx{Seconds(-1)};
        double maxGapS{0.0};
        Ptr<Socket> ricSock; //!< RIC-side socket bound to this link's address
        // Tcp mode
        Ptr<Socket> ricListen;          //!< RIC listening socket for this link
        bool assocUp{false};            //!< agent connection established
        std::vector<uint8_t> agentRxBuf; //!< stream reassembly at the agent
        std::vector<uint8_t> ricRxBuf;   //!< stream reassembly at the RIC
        std::vector<Ptr<Packet>> agentTxQ; //!< waiting for TX buffer space
        std::vector<Ptr<Packet>> ricTxQ;
        int64_t lastUpSeq{-1};          //!< highest uplink sequence seen at the RIC
        int64_t lastDownSeq{-1};        //!< highest downlink sequence seen at the agent
    };

    void Refresh();
    void SendFrame(Link& l,
                   bool uplink,
                   uint8_t proc,
                   uint8_t type,
                   const std::vector<uint8_t>& body,
                   Time created);
    void AgentRx(Ptr<Socket> s);
    void RicRx(Ptr<Socket> s);
    void HandleAtRic(Link& l, uint8_t proc, uint8_t type, uint32_t seq, Time created,
                     const std::vector<uint8_t>& body, uint32_t bytes);
    void HandleAtAgent(Link& l, uint8_t proc, uint8_t type, uint32_t seq, Time created,
                       const std::vector<uint8_t>& body, uint32_t bytes);
    void SendSubscriptions(Link& l);
    void Trace(const char* dir, uint8_t proc, uint8_t type, uint32_t gnb, uint32_t seq,
               uint32_t bytes, double owdMs);
    Time LinkDelay(const Link& l) const;
    void SetOutage(uint32_t gnbId, bool on);
    // Shared by the datagram and stream receive paths.
    void Dispatch(uint32_t gnb, const uint8_t* buf, uint32_t bytes, bool atRic);
    // Tcp mode
    void TcpConnect(uint32_t gnb);
    void TcpAgentRx(uint32_t gnb, Ptr<Socket> s);
    void TcpRicRx(uint32_t gnb, Ptr<Socket> s);
    void TcpDrain(std::vector<uint8_t>& buf, uint32_t gnb, bool atRic);
    void TcpSend(Ptr<Socket> s, std::vector<Ptr<Packet>>& q, Ptr<Packet> p);
    void TcpFlush(Ptr<Socket> s, std::vector<Ptr<Packet>>& q);
    void TcpAssociationLost(uint32_t gnb);

    Ptr<Node> m_ric;
    std::map<uint32_t, Link> m_links; //!< by gnbId
    DataRate m_rate{"10Mbps"};
    Time m_gwToRic{MilliSeconds(1)};
    Time m_refresh{MilliSeconds(100)};
    EventId m_refreshEvent;
    uint32_t m_sctpPad{28};
    Protocol m_proto{Protocol::Udp};
    double m_per{0.0};
    uint32_t m_maxRetrans{10};
    uint32_t m_maxInitRetrans{8};
    Time m_reconnect{Seconds(1)};
    uint32_t m_nextNet{0};
    Counters m_c;
    double m_owdSum{0.0};
    double m_owdSq{0.0};
    uint64_t m_owdN{0};
    double m_linkDelaySum{0.0};
    uint64_t m_linkDelayN{0};
    Time m_firstIndication{Seconds(-1)};
    Time m_lastSetupDone{Seconds(-1)};
    std::shared_ptr<std::ofstream> m_trace;
};

} // namespace ns3

#endif // ORAN_NTN_E2_PACKET_TRANSPORT_H
