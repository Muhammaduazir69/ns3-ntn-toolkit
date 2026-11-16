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

    Ptr<Node> m_ric;
    std::map<uint32_t, Link> m_links; //!< by gnbId
    DataRate m_rate{"10Mbps"};
    Time m_gwToRic{MilliSeconds(1)};
    Time m_refresh{MilliSeconds(100)};
    EventId m_refreshEvent;
    uint32_t m_sctpPad{28};
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
