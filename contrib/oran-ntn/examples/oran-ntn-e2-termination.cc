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
 *
 * Wave 7B: the KPM values are MEASURED, not constants. Before the agent
 * connects, it runs one in-process ns-3 pass (one Walker LEO satellite at
 * 600 km propagated by ntn-constellation's Sgp4MobilityModel, --simUes TR
 * 38.811 UEs from ntn-cho under its sub-point, a 5G-LENA NR cell built by
 * ntn-traffic's NtnRealStackHelper with the TR 38.821 Set-1 S-band budget and
 * a feeder gateway, the MixedBouquet standards flows, and the
 * NtnOranAiFlowMonitor over FlowMonitor). Indication k carries every metric the
 * monitor measured in granularity period k (1 s), one E2SM-KPM
 * measInfo row per (TS 28.552 measurement name, flow) with the flow's 5QI and
 * S-NSSAI as labels; a metric the period did not measure is left out. The pass
 * runs for --indications periods plus 2 s, so --indications=5 is a 7 s
 * simulation (about 15 to 30 s wall clock with the NR stack). In the
 * two-process mode start the RIC with a --duration that covers that; in
 * --role=both the pass runs before the listener starts. The pass also writes
 * <outputDir>/e2term_kpm_series.csv and sim_health.csv. The RIC side decodes
 * every indication and prints the first ones, so the values that crossed the
 * socket can be compared with the CSV.
 */

#include "ns3/core-module.h"
#include "ns3/ntn-oran-ai-flow-monitor.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"
#include "ns3/e2-listener.h"
#include "ns3/e2-message-types.h"
#include "ns3/e2-transport.h"
#include "ns3/oran-ntn-kpm-canonical-ids.h"
#include "ns3/oran-ntn-service-model-kpm.h"
#include "ns3/oran-ntn-rc-style3.h"
#include "ns3/oran-ntn-service-model-rc.h"
#include "ns3/oran-ntn-types.h"

#include <chrono>
#include <cstdio>
#include <map>
#include <iostream>
#include <thread>

using namespace ns3;
using namespace oranntn::flexric;

namespace
{

/// One measured KPM row: TS 28.552 name, flow labels, value.
struct MeasuredRow
{
    std::string name;
    uint8_t fiveQi{0};
    std::string snssai;
    double value{0.0};
};

/// One 1 s granularity period of measured KPM rows.
struct MeasuredPeriod
{
    double tS{0.0};
    std::vector<MeasuredRow> rows;
};

/// The measured pass behind the indications (see the file comment).
std::vector<MeasuredPeriod>
RunMeasuredPass(uint32_t periods, uint32_t numUes, const std::string& outDir)
{
    using namespace ns3::ntncon;
    const double simS = periods + 2.0;
    WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 1;
    wcfg.altitude_km = 600.0;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0; // 2025-01-01
    const auto elements = WalkerConstellation::BuildDelta(wcfg);
    auto sat = CreateObject<Sgp4MobilityModel>();
    sat->SetElementsSgp4(elements.front());
    double subLat = 0;
    double subLon = 0;
    double subAlt = 0;
    sat->GetGeodetic(subLat, subLon, subAlt);
    NodeContainer satNodes;
    satNodes.Create(1);
    satNodes.Get(0)->AggregateObject(sat);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    NtnTr38811MobilityHelper ueMobility(1);
    ueMobility.Install(ueNodes, NtnMobilityScenarios::MixedContinental(), subLat - 0.03,
                       subLat + 0.03, subLon - 0.03, subLon + 0.03);

    NtnRealStackHelper rs;
    rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
    rs.SetNumerology(1);
    rs.SetSimTime(Seconds(simS));
    rs.SetOutputDir(outDir);
    rs.SetRunTag("oran-ntn-e2-termination");
    rs.ApplyTr38821Set1Defaults();
    rs.Build(satNodes, ueNodes);
    rs.SetFeederGatewayGeodetic(subLat + 5.0, subLon, 0.0);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet, Seconds(1.0),
                      Seconds(simS - 0.5));
    rs.EnableAiFlowMonitor(outDir + "/e2term");
    Simulator::Stop(Seconds(simS));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::map<int64_t, MeasuredPeriod> byTime;
    Ptr<NtnOranAiFlowMonitor> mon = rs.GetAiFlowMonitor();
    for (const auto& [flowId, series] : mon->GetKpmSeries())
    {
        OranFlowKey key;
        mon->GetClassifier()->FindFlow(flowId, key);
        char snssai[16];
        std::snprintf(snssai, sizeof(snssai), "%u-%06x", key.sst, key.sd & 0xFFFFFFu);
        for (const auto& smp : series)
        {
            MeasuredPeriod& per = byTime[smp.time.GetNanoSeconds()];
            per.tS = smp.time.GetSeconds();
            for (const auto& [name, v] : smp.metrics)
            {
                if (std::isfinite(v))
                {
                    per.rows.push_back({name, key.fiveQi, snssai, v});
                }
            }
        }
    }
    std::printf("[agent] measured pass: SGP4 LEO 600 km, %u TR 38.811 UEs, NR SINR %.2f dB, "
                "DL %.3f Mb/s, owd %.2f ms, jitter %.3f ms, loss %.4f, %zu KPM periods\n",
                numUes, rs.GetMeanDlSinrDb(), rs.GetRxThroughputMbps(), rs.GetMeanDelayMs(),
                rs.GetMeanJitterMs(), rs.GetAppLossRatio(), byTime.size());
    Simulator::Destroy();
    std::vector<MeasuredPeriod> out;
    for (auto& [t, per] : byTime)
    {
        if (!per.rows.empty())
        {
            out.push_back(std::move(per));
        }
    }
    return out;
}

/// Build a real PER-encoded E2SM-KPM indication payload from one measured period.
std::vector<uint8_t>
MakeKpmIndicationBlob(const MeasuredPeriod& period)
{
    using namespace oranntn::flexric::kpm_v3;
    kpm_ind_msg_format_1_t body{};
    body.gran_period_ms = 1000;
    for (const auto& m : period.rows)
    {
        meas_info_format_1_lst_t row{};
        row.meas_type.form = meas_type_form_t::name;
        row.meas_type.meas_name = m.name;
        meas_record_item_t r{};
        r.form = meas_value_form_t::real;
        r.real_val = m.value;
        row.meas_record_lst.push_back(r);
        label_info_t lbl{};
        lbl.five_qi = m.fiveQi;
        lbl.s_nssai = m.snssai;
        lbl.plmn_id = "00101";
        row.label_info_lst.push_back(lbl);
        body.meas_info_lst.push_back(row);
    }
    OranNtnServiceModelKpm sm;
    return sm.EncodeIndication(&body);
}

/// RIC-side decode tally; the first indications are printed in full.
struct RicDecode
{
    uint32_t ok{0};
    uint32_t bad{0};
    uint64_t rows{0};
};

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
    RicDecode ind;
    uint32_t ctrlOk = 0;
    uint32_t ctrlBad = 0;

    ric->SetIndicationSink(MakeCallback(
        +[](RicDecode* d, std::vector<uint8_t> payload) {
            OranNtnServiceModelKpm sm;
            oranntn::flexric::kpm_v3::kpm_ind_msg_format_1_t out{};
            if (!sm.DecodeIndication(payload, out))
            {
                ++d->bad;
                return;
            }
            ++d->ok;
            d->rows += out.meas_info_lst.size();
            if (d->ok <= 3)
            {
                std::cout << "[ric] indication " << d->ok << ": " << out.meas_info_lst.size()
                          << " rows;";
                for (const auto& r : out.meas_info_lst)
                {
                    if (r.meas_type.meas_name == "DRB.UEThpDl" && !r.meas_record_lst.empty() &&
                        !r.label_info_lst.empty() && r.label_info_lst[0].five_qi)
                    {
                        std::cout << " 5QI " << unsigned(*r.label_info_lst[0].five_qi)
                                  << " DRB.UEThpDl=" << r.meas_record_lst[0].real_val;
                    }
                }
                std::cout << "\n";
            }
        }).Bind(&ind));
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
              << " (KPM decoded ok=" << ind.ok << " bad=" << ind.bad << ", " << ind.rows
              << " measured rows)"
              << " (RC control decoded ok=" << ctrlOk << " bad=" << ctrlBad << ")\n";
    const bool ok = ric->SetupRequestsHandled() >= 1 && ric->IndicationsForwarded() >= 1 &&
                    ind.ok >= 1 && ind.bad == 0 && ctrlOk >= 1 && ctrlBad == 0;
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
         uint32_t indications,
         const std::vector<MeasuredPeriod>& measured)
{
    if (measured.size() < indications)
    {
        std::cerr << "[agent] the measured pass produced " << measured.size()
                  << " KPM periods, fewer than --indications=" << indications << "\n";
        return 1;
    }
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
        const std::vector<uint8_t> blob = MakeKpmIndicationBlob(measured[i]);
        if (c->Send({E2MessageType::ric_indication, blob}) < 0)
        {
            std::cerr << "[agent] indication " << i << " send failed\n";
            return 1;
        }
    }
    std::cout << "[agent] sent " << indications
              << " PER-encoded KPM indications of measured periods\n";

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
    std::string role = "both";
    std::string proto = "sctp";
    std::string host = "127.0.0.1";
    uint32_t port = 36421;
    double duration = 8.0;
    uint32_t indications = 5;

    CommandLine cmd(__FILE__);
    cmd.AddValue("role",
                 "both (default: RIC plus an agent on a thread, self-contained) | "
                 "ric | agent",
                 role);
    cmd.AddValue("proto", "sctp | tcp", proto);
    cmd.AddValue("host", "bind/connect host", host);
    cmd.AddValue("port", "E2 port", port);
    cmd.AddValue("duration", "RIC listen seconds", duration);
    cmd.AddValue("indications", "agent: number of RIC indications to send", indications);
    uint32_t simUes = 2;
    std::string outputDir = "oran-ntn-e2-termination-output";
    cmd.AddValue("simUes", "agent: TR 38.811 UEs in the measured NR pass", simUes);
    cmd.AddValue("outputDir", "agent: output directory of the measured pass", outputDir);
    cmd.Parse(argc, argv);

    const E2Transport::Protocol p =
        (proto == "tcp") ? E2Transport::Protocol::tcp : E2Transport::Protocol::sctp;

    if (role == "agent")
    {
        const auto measured = RunMeasuredPass(indications, simUes, outputDir);
        return RunAgent(p, host, static_cast<uint16_t>(port), indications, measured);
    }

    if (role == "both")
    {
        // Self-contained mode, and the default.
        //
        // This example is a two-process demo: a RIC in one terminal, an agent in
        // another. Run on its own it defaulted to role=ric, sat waiting for a
        // peer that was never started, and exited 1 reporting "no traffic seen".
        // That is correct behaviour for a listener with no client and useless as
        // a check: any sweep or CI run over the examples recorded it as a
        // permanent failure, and the E2 exchange it exists to demonstrate was
        // never actually exercised by anything.
        //
        // Here the agent runs on a thread against this process's own listener,
        // so the association, the PER-encoded KPM indications and the decode all
        // happen for real over a loopback SCTP (or TCP) socket. The two-process
        // roles are unchanged for anyone who wants them.
        // The measured pass runs here, on the main thread and before the
        // listener starts: the ns-3 simulator is not thread-safe.
        const auto measured = RunMeasuredPass(indications, simUes, outputDir);
        int agentRc = 0;
        std::thread agent([&]() {
            // Let the listener bind before connecting; a connect that races the
            // bind fails for a reason that has nothing to do with E2.
            std::this_thread::sleep_for(std::chrono::milliseconds(400));
            agentRc = RunAgent(p, host, static_cast<uint16_t>(port), indications, measured);
        });
        const int ricRc = RunRic(p, host, static_cast<uint16_t>(port), duration);
        agent.join();
        if (ricRc != 0 || agentRc != 0)
        {
            std::cout << "[both] ric=" << ricRc << " agent=" << agentRc << "\n";
        }
        return (ricRc == 0 && agentRc == 0) ? 0 : 1;
    }

    return RunRic(p, host, static_cast<uint16_t>(port), duration);
}
