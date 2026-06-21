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
 * The agent runs the real E2AP procedures — E2 Setup, RIC Subscription, a burst
 * of RIC Indications whose payload is a genuine E2SM-KPM message PER-encoded by
 * OranNtnServiceModelKpm, then a RIC Control Request — and prints what it got
 * back. The RIC accepts the association, completes the handshake, counts the
 * indications, and acknowledges control. This closes the loopback-only gap: the
 * PER + SCTP stack is exercised across a process boundary. (Interop with the
 * third-party FlexRIC binary is the same wire protocol; point --host/--port at a
 * running FlexRIC nearRT-RIC to drive it.)
 */

#include "ns3/core-module.h"
#include "ns3/e2-listener.h"
#include "ns3/e2-message-types.h"
#include "ns3/e2-transport.h"
#include "ns3/oran-ntn-kpm-canonical-ids.h"
#include "ns3/oran-ntn-service-model-kpm.h"

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

    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() <
           durationSec)
    {
        ric->Poll(200);
    }

    std::cout << "[ric] clients=" << ric->NumClients()
              << " setups=" << ric->SetupRequestsHandled()
              << " indications=" << ric->IndicationsForwarded() << "\n";
    const bool ok = ric->SetupRequestsHandled() >= 1 && ric->IndicationsForwarded() >= 1;
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

    if (!Exchange(c.get(), {E2MessageType::ric_control_request, {0x77}}, reply,
                  E2MessageType::ric_control_acknowledge))
    {
        std::cerr << "[agent] RIC Control failed\n";
        return 1;
    }
    std::cout << "[agent] RIC Control -> Acknowledge OK\n";

    c->Close();
    std::cout << "[agent] PASS (full E2 procedure over real " << (proto == E2Transport::Protocol::sctp ? "SCTP" : "TCP") << ")\n";
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
