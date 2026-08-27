/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * oran-ntn-e2-termination — a standalone, TWO-PROCESS E2 end-to-end driver over
 * a REAL SCTP association (the transport the O-RAN E2 interface mandates).
 *
 * Run it twice, as separate OS processes:
 *
 *   # terminal 1 — the RIC / E2 termination side
 *   ./ns3 run "oran-ntn-e2-termination --role=ric --proto=sctp --port=36421 --duration=8"
 *
 *   # terminal 2 — the gNB / E2 node side
 *   ./ns3 run "oran-ntn-e2-termination --role=agent --proto=sctp --port=36421 --indications=5"
 *
 * WHAT THIS IS, PRECISELY (audit ORAN-11).
 *
 * This is a two-process demo of a TOOLKIT-PRIVATE framing over a real SCTP (or
 * TCP) association, carrying real E2SM payloads. It is not E2AP.
 *
 *   - The framing is 2-byte type + 4-byte length, defined by this repo. Its
 *     message-type values are toolkit-internal and do not map onto E2AP
 *     codepoints; e2-message-types.h says so itself. No SCTP PPID 70 is
 *     negotiated, and the procedure codes are not ASN.1 InitiatingMessage tags.
 *   - The E2SM payloads ARE real: the Indications carry an E2SM-KPM
 *     IndicationMessage and the Control Request carries an E2SM-RC Style 3
 *     ControlMessage, both produced by the toolkit's own codecs. Those codecs
 *     round-trip against themselves but are NOT bit-conformant Aligned-PER
 *     (full-byte CHOICE index, octet-aligned preambles, no extension markers),
 *     exactly as README.md states.
 *
 * It therefore does NOT interoperate with FlexRIC or any asn1c peer. An earlier
 * version of this header invited the reader to "point --host/--port at a running
 * FlexRIC nearRT-RIC to drive it"; a reader who did would get an association
 * that desynchronises on the first frame. That sentence is removed, and it is
 * worth being blunt about why it mattered: every other honesty statement in this
 * module is careful, and this was the one file a user opens to find out whether
 * E2 is real.
 *
 * What the demo genuinely establishes is that the codecs survive a process
 * boundary. The RIC side DECODES what it receives rather than counting frames:
 * every Indication is run through OranNtnServiceModelKpm::DecodeIndication and
 * the Control Request through OranNtnServiceModelRc::DecodeControl, and a
 * decode failure fails the run. Before this, the control payload was the three
 * bytes {0x77} and the RIC acknowledged it without looking, so the "full E2
 * procedure" it reported would have been reported just the same by a peer that
 * sent nothing meaningful at all.
 */

#include "ns3/core-module.h"
#include "ns3/e2-listener.h"
#include "ns3/e2-message-types.h"
#include "ns3/e2-transport.h"
#include "ns3/oran-ntn-kpm-canonical-ids.h"
#include "ns3/oran-ntn-service-model-kpm.h"
#include "ns3/oran-ntn-rc-style3.h"
#include "ns3/oran-ntn-service-model-rc.h"
#include "ns3/oran-ntn-types.h"

#include <chrono>
#include <iostream>
#include <thread>

using namespace ns3;
using namespace oranntn::flexric;

namespace
{

/// Build a real PER-encoded E2SM-KPM indication payload.
std::vector<uint8_t>
MakeKpmIndicationBlob(double thpDl)
{
    using namespace oranntn::flexric::kpm_v3;
    kpm_ind_msg_format_1_t body{};
    body.gran_period_ms = 1000;
    meas_info_format_1_lst_t row{};
    row.meas_type.form = meas_type_form_t::name;
    row.meas_type.meas_name = oranntn::kpm::kDrbUeThpDl;
    meas_record_item_t r{};
    r.form = meas_value_form_t::real;
    r.real_val = thpDl;
    row.meas_record_lst.push_back(r);
    label_info_t lbl{};
    lbl.five_qi = 9;
    lbl.s_nssai = "1-000001";
    lbl.plmn_id = "00101";
    row.label_info_lst.push_back(lbl);
    body.meas_info_lst.push_back(row);

    OranNtnServiceModelKpm sm;
    return sm.EncodeIndication(&body);
}

int
RunRic(E2Transport::Protocol proto, const std::string& host, uint16_t port, double durationSec)
{
    Ptr<OranNtnE2Listener> ric = CreateObject<OranNtnE2Listener>();
    ric->SetTransportKind(proto);
    if (!ric->Start(host, port))
    {
        std::cerr << "[ric] FAILED to bind " << host << ":" << port << "\n";
        return 2;
    }
    std::cout << "[ric] E2 termination listening on " << host << ":" << port << " ("
              << (proto == E2Transport::Protocol::sctp ? "SCTP" : "TCP") << ")\n";

    // ORAN-11: decode, do not merely count. A listener that acknowledges a
    // control it never parsed cannot tell a real peer from a peer sending
    // three magic bytes, which is exactly what it used to be handed.
    uint32_t indOk = 0;
    uint32_t indBad = 0;
    uint32_t ctrlOk = 0;
    uint32_t ctrlBad = 0;

    ric->SetIndicationSink(MakeCallback(
        +[](uint32_t* okp, uint32_t* badp, std::vector<uint8_t> payload) {
            OranNtnServiceModelKpm sm;
            oranntn::flexric::kpm_v3::kpm_ind_msg_format_1_t out{};
            (sm.DecodeIndication(payload, out) ? *okp : *badp)++;
        }).Bind(&indOk).Bind(&indBad));
    ric->SetControlSink(MakeCallback(
        +[](uint32_t* okp, uint32_t* badp, std::vector<uint8_t> payload) {
            OranNtnServiceModelRc sm;
            oranntn::rc_v103::style3::ControlMessage out{};
            (sm.DecodeControl(payload, &out) ? *okp : *badp)++;
        }).Bind(&ctrlOk).Bind(&ctrlBad));

    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() <
           durationSec)
    {
        ric->Poll(200);
    }

    std::cout << "[ric] clients=" << ric->NumClients()
              << " setups=" << ric->SetupRequestsHandled()
              << " indications=" << ric->IndicationsForwarded()
              << " (KPM decoded ok=" << indOk << " bad=" << indBad << ")"
              << " (RC control decoded ok=" << ctrlOk << " bad=" << ctrlBad << ")\n";
    const bool ok = ric->SetupRequestsHandled() >= 1 && ric->IndicationsForwarded() >= 1 &&
                    indOk >= 1 && indBad == 0 && ctrlOk >= 1 && ctrlBad == 0;
    ric->Stop();
    std::cout << "[ric] " << (ok ? "PASS" : "no traffic seen") << "\n";
    return ok ? 0 : 1;
}

bool
Exchange(E2Transport* c, const E2Message& req, E2Message& reply, E2MessageType want)
{
    if (c->Send(req) < 0)
    {
        return false;
    }
    if (!c->Recv(reply, 1000))
    {
        return false;
    }
    return reply.type == want;
}

/// ORAN-11: a real E2SM-RC Style 3 ControlMessage, not {0x77}.
///
/// Built the way the in-sim path would build it: an E2RcAction goes through
/// ConvertE2RcToStyle3 and then through the RC codec. If the conversion or the
/// encode is wrong, the RIC's decode fails and the run fails with it - which is
/// the point of carrying real bytes rather than a magic number.
std::vector<uint8_t>
MakeRcControlBlob(uint32_t targetGnbId)
{
    E2RcAction act{};
    act.timestamp = 0.0;
    act.xappId = 1;
    act.xappName = "e2-termination-demo";
    act.actionType = E2RcActionType::HANDOVER_TRIGGER;
    act.targetGnbId = targetGnbId;
    act.targetUeId = 42;
    act.confidence = 1.0;

    auto action = oranntn::rc_v103::style3::ConvertE2RcToStyle3(act);
    if (!action)
    {
        return {};
    }
    oranntn::rc_v103::style3::ControlMessage msg{};
    msg.action = *action;

    OranNtnServiceModelRc sm;
    return sm.EncodeControl(msg);
}

int
RunAgent(E2Transport::Protocol proto,
         const std::string& host,
         uint16_t port,
         uint32_t indications)
{
    auto c = MakeE2Transport(proto);
    if (!c->Connect(host, port))
    {
        std::cerr << "[agent] connect to " << host << ":" << port << " failed\n";
        return 2;
    }
    std::cout << "[agent] connected (" << (proto == E2Transport::Protocol::sctp ? "SCTP" : "TCP")
              << ")\n";

    E2Message reply{};
    if (!Exchange(c.get(), {E2MessageType::e2_setup_request, {0x01}}, reply,
                  E2MessageType::e2_setup_response))
    {
        std::cerr << "[agent] E2 Setup failed\n";
        return 1;
    }
    std::cout << "[agent] E2 Setup -> Response OK\n";

    if (!Exchange(c.get(), {E2MessageType::ric_subscription_request, {0xAA, 0xBB}}, reply,
                  E2MessageType::ric_subscription_response))
    {
        std::cerr << "[agent] RIC Subscription failed\n";
        return 1;
    }
    std::cout << "[agent] RIC Subscription -> Response OK\n";

    for (uint32_t i = 0; i < indications; ++i)
    {
        const std::vector<uint8_t> blob = MakeKpmIndicationBlob(10.0 + i);
        if (c->Send({E2MessageType::ric_indication, blob}) < 0)
        {
            std::cerr << "[agent] indication " << i << " send failed\n";
            return 1;
        }
    }
    std::cout << "[agent] sent " << indications << " PER-encoded KPM indications\n";

    const std::vector<uint8_t> ctrl = MakeRcControlBlob(/*targetGnbId=*/9);
    if (ctrl.empty())
    {
        std::cerr << "[agent] could not build an E2SM-RC Style 3 control\n";
        return 1;
    }
    if (!Exchange(c.get(), {E2MessageType::ric_control_request, ctrl}, reply,
                  E2MessageType::ric_control_acknowledge))
    {
        std::cerr << "[agent] RIC Control failed\n";
        return 1;
    }
    std::cout << "[agent] RIC Control (" << ctrl.size()
              << " B E2SM-RC Style 3 ControlMessage) -> Acknowledge OK\n";

    c->Close();
    // Deliberately NOT "full E2 procedure": the framing is this repo's own.
    std::cout << "[agent] PASS (toolkit framing over real "
              << (proto == E2Transport::Protocol::sctp ? "SCTP" : "TCP")
              << ", carrying real E2SM-KPM and E2SM-RC payloads)\n";
    return 0;
}

} // namespace

int
main(int argc, char* argv[])
{
    std::string role = "ric";
    std::string proto = "sctp";
    std::string host = "127.0.0.1";
    uint32_t port = 36421;
    double duration = 8.0;
    uint32_t indications = 5;

    CommandLine cmd(__FILE__);
    cmd.AddValue("role", "ric | agent", role);
    cmd.AddValue("proto", "sctp | tcp", proto);
    cmd.AddValue("host", "bind/connect host", host);
    cmd.AddValue("port", "E2 port", port);
    cmd.AddValue("duration", "RIC listen seconds", duration);
    cmd.AddValue("indications", "agent: number of RIC indications to send", indications);
    cmd.Parse(argc, argv);

    const E2Transport::Protocol p =
        (proto == "tcp") ? E2Transport::Protocol::tcp : E2Transport::Protocol::sctp;

    if (role == "agent")
    {
        return RunAgent(p, host, static_cast<uint16_t>(port), indications);
    }
    return RunRic(p, host, static_cast<uint16_t>(port), duration);
}
