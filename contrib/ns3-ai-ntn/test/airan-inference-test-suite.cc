/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

// AI-RAN inference contract tests (Roadmap §3 T7).
//
// Coverage:
//   1. TritonModelConfigParser — pbtxt round-trip + edge cases
//   2. AiranMessageCodec      — request / response encode-decode for
//                                CSI and RSRP input flavours, plus
//                                malformed-frame rejection
//   3. InProcInferenceChannel — paired Send/Recv with deferred reads
//   4. TcpInferenceChannel    — round-trip over a real loopback
//                                socket (skipped if bind fails)
//   5. AiranInferenceServer + Client — round-trip via in-proc and
//      via TCP, under Simulator::Run() with periodic Poll() ticks
//   6. Failure modes — unknown model, mismatched input kind
//   7. Simulated workload — 60 s scenario, 100 alternating requests,
//      latency aggregation, no dropped frames

#include "ns3/airan-inference-client.h"
#include "ns3/airan-inference-server.h"
#include "ns3/airan-messages.h"
#include "ns3/airan-mock-runtime.h"
#include "ns3/inference-channel-inproc.h"
#include "ns3/inference-channel-tcp.h"
#include "ns3/triton-model-config.h"
#include <complex>

#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <atomic>
#include <fstream>
#include <memory>
#include <sstream>

using namespace ns3;
using namespace ns3::oranntn::airan;

namespace
{

const char* kPrecoderPbtxt = R"PBTXT(
# precoder model
name: "precoder_csi_to_weights"
platform: "onnxruntime_onnx"
max_batch_size: 32
default_model_filename: "model.onnx"
dynamic_batching {
  preferred_batch_size: [ 4, 8, 16 ]
  max_queue_delay_microseconds: 200
}
parameters {
  key: "INFERENCE_BUDGET_US"
  value: { string_value: "1000" }
}
parameters {
  key: "TOOLKIT_OUTPUT_FIELD"
  value: { string_value: "precoder" }
}
)PBTXT";

const char* kBeamPbtxt = R"PBTXT(
name: "beam_index_classifier"
platform: "tensorrt_plan"
max_batch_size: 64
parameters {
  key: "INFERENCE_BUDGET_US"
  value: { string_value: "500" }
}
parameters {
  key: "TOOLKIT_OUTPUT_FIELD"
  value: { string_value: "beam" }
}
)PBTXT";

TritonModelConfig
LoadPrecoderConfig()
{
    auto cfg = TritonModelConfigParser::Parse(kPrecoderPbtxt);
    return *cfg;
}

TritonModelConfig
LoadBeamConfig()
{
    auto cfg = TritonModelConfigParser::Parse(kBeamPbtxt);
    return *cfg;
}

CsiTensor
MakeFakeCsi(uint64_t seed)
{
    CsiTensor t;
    t.num_tx = 4;
    t.num_rx = 2;
    t.num_subcarriers = 8;
    t.doppler_hz = 100.0 + 0.5 * static_cast<double>(seed);
    t.values.reserve(2 * t.num_tx * t.num_rx * t.num_subcarriers);
    for (uint32_t i = 0; i < 2 * t.num_tx * t.num_rx * t.num_subcarriers;
         ++i)
    {
        t.values.push_back(static_cast<float>(0.01 * (i + seed)));
    }
    return t;
}

RsrpVector
MakeFakeRsrp(uint64_t seed)
{
    RsrpVector v;
    v.rsrp_dbm.reserve(8);
    v.beam_az_deg.reserve(8);
    v.beam_el_deg.reserve(8);
    for (uint32_t i = 0; i < 8; ++i)
    {
        v.rsrp_dbm.push_back(
            -90.0f + static_cast<float>((i * 7 + seed) % 30));
        v.beam_az_deg.push_back(static_cast<float>(i * 45));
        v.beam_el_deg.push_back(static_cast<float>(15 + i * 5));
    }
    return v;
}

} // namespace

// ----------------------------------------------------------------------------

class TritonModelConfigParserTest : public TestCase
{
  public:
    TritonModelConfigParserTest()
        : TestCase("Parse Triton model config (T7)")
    {
    }
    void DoRun() override
    {
        const auto p = TritonModelConfigParser::Parse(kPrecoderPbtxt);
        NS_TEST_ASSERT_MSG_EQ(p.has_value(), true, "parse precoder");
        NS_TEST_EXPECT_MSG_EQ(p->name,
                               "precoder_csi_to_weights",
                               "model name");
        NS_TEST_EXPECT_MSG_EQ(p->platform,
                               "onnxruntime_onnx",
                               "platform");
        NS_TEST_EXPECT_MSG_EQ(p->max_batch_size, 32u, "max batch");
        NS_TEST_EXPECT_MSG_EQ(p->default_model_filename,
                               "model.onnx",
                               "default model file");
        NS_TEST_ASSERT_MSG_EQ(p->max_queue_delay_us.has_value(),
                               true,
                               "queue delay parsed");
        NS_TEST_EXPECT_MSG_EQ(*p->max_queue_delay_us,
                               200u,
                               "queue delay value");
        const bool precoder_field =
            p->output_field == OutputField::precoder;
        NS_TEST_EXPECT_MSG_EQ(precoder_field,
                               true,
                               "precoder output field");
        NS_TEST_ASSERT_MSG_EQ(p->inference_budget_us.has_value(),
                               true,
                               "inference budget parsed");
        NS_TEST_EXPECT_MSG_EQ(*p->inference_budget_us,
                               1000u,
                               "inference budget value");

        const auto b = TritonModelConfigParser::Parse(kBeamPbtxt);
        NS_TEST_ASSERT_MSG_EQ(b.has_value(), true, "parse beam");
        const bool beam_field = b->output_field == OutputField::beam;
        NS_TEST_EXPECT_MSG_EQ(beam_field,
                               true,
                               "beam output field");

        // Edge cases: empty input, missing name → reject.
        NS_TEST_EXPECT_MSG_EQ(
            TritonModelConfigParser::Parse("").has_value(),
            false,
            "empty rejected");
        NS_TEST_EXPECT_MSG_EQ(TritonModelConfigParser::Parse(
                                   "platform: \"foo\"")
                                   .has_value(),
                               false,
                               "no-name rejected");
    }
};

// ----------------------------------------------------------------------------

class TritonModelConfigShippedFilesTest : public TestCase
{
  public:
    TritonModelConfigShippedFilesTest()
        : TestCase("Parse shipped Triton config.pbtxt files (T7)")
    {
    }
    void DoRun() override
    {
        // Resolve the shipped pbtxt files relative to this source.
        // We try a couple of standard relative paths so the test works
        // both during in-tree builds and from the install tree.
        const std::vector<std::string> roots = {
            "contrib/ns3-ai-ntn/grpc/triton/",
            "../contrib/ns3-ai-ntn/grpc/triton/",
            "../../contrib/ns3-ai-ntn/grpc/triton/",
        };
        bool found_precoder = false;
        bool found_beam = false;
        for (const auto& root : roots)
        {
            const auto p = TritonModelConfigParser::LoadFile(
                root + "precoder_csi_to_weights/config.pbtxt");
            if (p && p->name == "precoder_csi_to_weights")
            {
                found_precoder = true;
                const bool ok =
                    p->output_field == OutputField::precoder;
                NS_TEST_EXPECT_MSG_EQ(
                    ok,
                    true,
                    "shipped precoder pbtxt has precoder field");
            }
            const auto b = TritonModelConfigParser::LoadFile(
                root + "beam_index_classifier/config.pbtxt");
            if (b && b->name == "beam_index_classifier")
            {
                found_beam = true;
                const bool ok =
                    b->output_field == OutputField::beam;
                NS_TEST_EXPECT_MSG_EQ(
                    ok,
                    true,
                    "shipped beam pbtxt has beam field");
            }
        }
        // Soft-pass: shipped files are nice-to-have under in-tree
        // builds. The parser logic is exercised by the previous test
        // with embedded literals, so we don't fail the suite if the
        // files aren't reachable from this cwd.
        if (!found_precoder && !found_beam)
        {
            NS_LOG_UNCOND("Triton pbtxt files not in cwd, skipping");
        }
    }
};

// ----------------------------------------------------------------------------

class AiranMessageCodecRoundTripTest : public TestCase
{
  public:
    AiranMessageCodecRoundTripTest()
        : TestCase("Airan request-response codec round-trip (T7)")
    {
    }
    void DoRun() override
    {
        // CSI request round-trip
        InferenceRequest req;
        req.request_id = 42;
        req.ue_id = 0xdeadbeefcafebabeULL;
        req.nr_cgi = 7;
        req.model_name = "precoder_csi_to_weights";
        req.sim_time_s = 1.234;
        req.input_kind = InferenceRequest::InputKind::csi;
        req.csi = MakeFakeCsi(11);

        const auto bytes = AiranMessageCodec::EncodeRequest(req);
        NS_TEST_EXPECT_MSG_EQ(
            AiranMessageCodec::PeekKind(bytes), 0u, "kind=request");

        InferenceRequest decoded;
        NS_TEST_ASSERT_MSG_EQ(
            AiranMessageCodec::DecodeRequest(bytes, decoded),
            true,
            "decode csi request");
        NS_TEST_EXPECT_MSG_EQ(decoded.request_id, 42u, "rid");
        NS_TEST_EXPECT_MSG_EQ(decoded.ue_id,
                               0xdeadbeefcafebabeULL,
                               "ue id");
        const bool csi_kind =
            decoded.input_kind == InferenceRequest::InputKind::csi;
        NS_TEST_EXPECT_MSG_EQ(csi_kind,
                               true,
                               "kind is csi");
        NS_TEST_EXPECT_MSG_EQ(decoded.csi.num_tx, req.csi.num_tx, "tx");
        NS_TEST_EXPECT_MSG_EQ(decoded.csi.num_rx, req.csi.num_rx, "rx");
        NS_TEST_EXPECT_MSG_EQ(decoded.csi.num_subcarriers,
                               req.csi.num_subcarriers,
                               "sc");
        NS_TEST_ASSERT_MSG_EQ(decoded.csi.values.size(),
                               req.csi.values.size(),
                               "csi length");
        NS_TEST_EXPECT_MSG_EQ_TOL(decoded.csi.values.front(),
                                    req.csi.values.front(),
                                    1e-5,
                                    "csi front");
        NS_TEST_EXPECT_MSG_EQ_TOL(decoded.csi.values.back(),
                                    req.csi.values.back(),
                                    1e-5,
                                    "csi back");
        NS_TEST_EXPECT_MSG_EQ_TOL(decoded.csi.doppler_hz,
                                    req.csi.doppler_hz,
                                    1e-9,
                                    "doppler");

        // RSRP request round-trip
        InferenceRequest req2;
        req2.request_id = 99;
        req2.ue_id = 5;
        req2.nr_cgi = 12;
        req2.model_name = "beam_index_classifier";
        req2.input_kind = InferenceRequest::InputKind::rsrp;
        req2.rsrp = MakeFakeRsrp(3);
        const auto bytes2 = AiranMessageCodec::EncodeRequest(req2);
        InferenceRequest decoded2;
        NS_TEST_ASSERT_MSG_EQ(
            AiranMessageCodec::DecodeRequest(bytes2, decoded2),
            true,
            "decode rsrp request");
        const bool rsrp_kind =
            decoded2.input_kind == InferenceRequest::InputKind::rsrp;
        NS_TEST_EXPECT_MSG_EQ(rsrp_kind,
                               true,
                               "rsrp kind");
        NS_TEST_ASSERT_MSG_EQ(decoded2.rsrp.rsrp_dbm.size(),
                               req2.rsrp.rsrp_dbm.size(),
                               "rsrp size");
        NS_TEST_EXPECT_MSG_EQ_TOL(decoded2.rsrp.beam_el_deg.back(),
                                    req2.rsrp.beam_el_deg.back(),
                                    1e-5,
                                    "rsrp el last");

        // Response round-trip (precoder kind)
        InferenceResponse resp;
        resp.request_id = 42;
        resp.inference_latency_ms = 0.83;
        resp.status = 0;
        resp.status_message = "ok";
        resp.output_kind = InferenceResponse::OutputKind::precoder;
        resp.precoder.num_tx = 4;
        resp.precoder.num_layers = 4;
        resp.precoder.values = {0.1f,
                                  -0.2f,
                                  0.3f,
                                  -0.4f,
                                  0.5f,
                                  -0.6f,
                                  0.7f,
                                  -0.8f};
        const auto rbytes = AiranMessageCodec::EncodeResponse(resp);
        InferenceResponse rdec;
        NS_TEST_ASSERT_MSG_EQ(
            AiranMessageCodec::DecodeResponse(rbytes, rdec),
            true,
            "decode precoder response");
        const bool resp_precoder_kind =
            rdec.output_kind ==
            InferenceResponse::OutputKind::precoder;
        NS_TEST_EXPECT_MSG_EQ(resp_precoder_kind,
                               true,
                               "precoder kind round-trips");
        NS_TEST_EXPECT_MSG_EQ_TOL(rdec.inference_latency_ms,
                                    0.83,
                                    1e-9,
                                    "latency");
        NS_TEST_EXPECT_MSG_EQ_TOL(rdec.precoder.values[3],
                                    -0.4f,
                                    1e-5,
                                    "precoder value");

        // Malformed: wrong leading kind byte
        std::vector<uint8_t> bogus = bytes;
        bogus[0] = 0x77;
        InferenceRequest junk;
        NS_TEST_EXPECT_MSG_EQ(
            AiranMessageCodec::DecodeRequest(bogus, junk),
            false,
            "bogus kind rejected");
    }
};

// ----------------------------------------------------------------------------

class InProcChannelRoundTripTest : public TestCase
{
  public:
    InProcChannelRoundTripTest()
        : TestCase("InProc channel send-recv (T7)")
    {
    }
    void DoRun() override
    {
        auto pair = InProcInferenceChannel::CreatePair();
        auto& a = pair.first;
        auto& b = pair.second;
        NS_TEST_ASSERT_MSG_EQ(a->IsOpen(), true, "a open");
        NS_TEST_ASSERT_MSG_EQ(b->IsOpen(), true, "b open");

        const std::vector<uint8_t> hello = {1, 2, 3, 4, 5};
        NS_TEST_EXPECT_MSG_EQ(a->Send(hello), true, "a sent");
        std::vector<uint8_t> got;
        NS_TEST_EXPECT_MSG_EQ(b->TryRecv(got, 10),
                               true,
                               "b received");
        NS_TEST_EXPECT_MSG_EQ(got.size(), 5u, "size matches");
        NS_TEST_EXPECT_MSG_EQ(got[4], 5u, "last byte");

        // Receive on empty side should time out.
        NS_TEST_EXPECT_MSG_EQ(b->TryRecv(got, 2),
                               false,
                               "no more inbound");

        // Close on one side propagates: TryRecv returns false.
        a->Close();
        NS_TEST_EXPECT_MSG_EQ(b->TryRecv(got, 5),
                               false,
                               "closed side stops");
        const bool a_sent = a->BytesSent() >= 5;
        const bool b_recv = b->BytesRecv() >= 5;
        NS_TEST_EXPECT_MSG_EQ(a_sent, true, "counter");
        NS_TEST_EXPECT_MSG_EQ(b_recv, true, "counter b");
    }
};

// ----------------------------------------------------------------------------

class TcpChannelRoundTripTest : public TestCase
{
  public:
    TcpChannelRoundTripTest()
        : TestCase("TCP channel send-recv on loopback (T7)")
    {
    }
    void DoRun() override
    {
        TcpInferenceListener listener;
        const bool listening = listener.Listen("127.0.0.1", 0);
        if (!listening)
        {
            NS_LOG_UNCOND(
                "TCP listener could not bind, skipping test");
            return;
        }
        const uint16_t port = listener.LocalPort();
        NS_TEST_ASSERT_MSG_GT(port, 0u, "ephemeral port allocated");

        TcpInferenceChannel client;
        NS_TEST_ASSERT_MSG_EQ(client.ConnectTo("127.0.0.1", port),
                               true,
                               "connect");
        auto srv = listener.AcceptOne(200);
        NS_TEST_ASSERT_MSG_NE(srv.get(),
                               nullptr,
                               "server accepts");

        const std::vector<uint8_t> frame = {0xA0,
                                            0x01,
                                            0x02,
                                            0x03,
                                            0xFF};
        NS_TEST_EXPECT_MSG_EQ(client.Send(frame),
                               true,
                               "client sent");
        std::vector<uint8_t> got;
        NS_TEST_EXPECT_MSG_EQ(srv->TryRecv(got, 200),
                               true,
                               "server received");
        NS_TEST_ASSERT_MSG_EQ(got.size(), frame.size(), "size");
        NS_TEST_EXPECT_MSG_EQ(got.front(), 0xA0u, "first byte");
        NS_TEST_EXPECT_MSG_EQ(got.back(), 0xFFu, "last byte");

        // Round-trip in the other direction.
        const std::vector<uint8_t> rframe(33, 0x42);
        NS_TEST_EXPECT_MSG_EQ(srv->Send(rframe), true, "server sent");
        std::vector<uint8_t> rgot;
        NS_TEST_EXPECT_MSG_EQ(client.TryRecv(rgot, 200),
                               true,
                               "client received");
        NS_TEST_EXPECT_MSG_EQ(rgot.size(), 33u, "size 33");

        client.Close();
        srv->Close();
        listener.Close();
    }
};

// ----------------------------------------------------------------------------

class AiranInProcRoundTripTest : public TestCase
{
  public:
    AiranInProcRoundTripTest()
        : TestCase("Client+Server round-trip via in-proc channel (T7)")
    {
    }
    void DoRun() override
    {
        AiranInferenceServer server;
        NS_TEST_ASSERT_MSG_EQ(
            server.RegisterModel(LoadPrecoderConfig(),
                                  AiranMockRuntime::MakePrecoderHandler(
                                      /*num_layers=*/2,
                                      /*base_latency_ms=*/0.5)),
            true,
            "register precoder");
        NS_TEST_ASSERT_MSG_EQ(
            server.RegisterModel(LoadBeamConfig(),
                                  AiranMockRuntime::MakeBeamHandler(
                                      /*top_k=*/3,
                                      /*latency_ms=*/0.3)),
            true,
            "register beam");

        auto pair = InProcInferenceChannel::CreatePair();
        AiranInferenceClient client;
        client.Attach(std::move(pair.first));
        server.AddChannel(std::move(pair.second));

        bool got_precoder_resp = false;
        bool got_beam_resp = false;

        const auto rid1 = client.SubmitPrecoderRequest(
            "precoder_csi_to_weights",
            /*ue_id=*/100,
            /*nr_cgi=*/1,
            /*sim_time_s=*/0.0,
            MakeFakeCsi(5),
            [&](const InferenceResponse& resp) {
                got_precoder_resp = true;
                const bool ok = resp.output_kind ==
                                InferenceResponse::OutputKind::precoder;
                NS_TEST_EXPECT_MSG_EQ(
                    ok,
                    true,
                    "precoder response kind");
                NS_TEST_EXPECT_MSG_EQ(resp.status, 0u, "status ok");
                NS_TEST_EXPECT_MSG_EQ(resp.precoder.num_layers,
                                       2u,
                                       "two layers");
                NS_TEST_EXPECT_MSG_EQ(resp.precoder.values.size(),
                                       static_cast<size_t>(
                                           2 *
                                           resp.precoder.num_tx *
                                           resp.precoder.num_layers),
                                       "precoder value count");
            });
        NS_TEST_ASSERT_MSG_GT(rid1, 0u, "precoder rid");

        const auto rid2 = client.SubmitBeamRequest(
            "beam_index_classifier",
            /*ue_id=*/101,
            /*nr_cgi=*/2,
            /*sim_time_s=*/0.0,
            MakeFakeRsrp(13),
            [&](const InferenceResponse& resp) {
                got_beam_resp = true;
                const bool ok = resp.output_kind ==
                                InferenceResponse::OutputKind::beam;
                NS_TEST_EXPECT_MSG_EQ(
                    ok,
                    true,
                    "beam response kind");
                NS_TEST_EXPECT_MSG_EQ(resp.beam.scores_top_k.size(),
                                       3u,
                                       "top-3");
            });
        NS_TEST_ASSERT_MSG_GT(rid2, 0u, "beam rid");

        server.Poll(5);
        client.Poll(5);
        NS_TEST_EXPECT_MSG_EQ(got_precoder_resp,
                               true,
                               "precoder cb fired");
        NS_TEST_EXPECT_MSG_EQ(got_beam_resp,
                               true,
                               "beam cb fired");
        NS_TEST_EXPECT_MSG_EQ(client.PendingCount(),
                               0u,
                               "no pending");
        NS_TEST_EXPECT_MSG_EQ(server.RequestsHandled(),
                               2u,
                               "two handled");
        NS_TEST_EXPECT_MSG_EQ(server.RequestsRejected(),
                               0u,
                               "none rejected");
    }
};

// ----------------------------------------------------------------------------

class AiranTcpRoundTripTest : public TestCase
{
  public:
    AiranTcpRoundTripTest()
        : TestCase("Client+Server round-trip over TCP (T7)")
    {
    }
    void DoRun() override
    {
        AiranInferenceServer server;
        NS_TEST_ASSERT_MSG_EQ(
            server.RegisterModel(LoadBeamConfig(),
                                  AiranMockRuntime::MakeBeamHandler()),
            true,
            "register beam");
        if (!server.StartTcpListener("127.0.0.1", 0))
        {
            NS_LOG_UNCOND("TCP listener failed to bind, skip");
            return;
        }
        const uint16_t port = server.LocalTcpPort();
        NS_TEST_ASSERT_MSG_GT(port, 0u, "port assigned");

        auto cli_chan = std::make_unique<TcpInferenceChannel>();
        NS_TEST_ASSERT_MSG_EQ(
            cli_chan->ConnectTo("127.0.0.1", port),
            true,
            "client connect");

        AiranInferenceClient client;
        client.Attach(std::move(cli_chan));

        // Server pumps once to accept.
        server.Poll(10);
        NS_TEST_EXPECT_MSG_EQ(server.NumChannels(),
                               1u,
                               "one client");

        bool fired = false;
        InferenceResponse out;
        client.SubmitBeamRequest(
            "beam_index_classifier",
            7,
            3,
            0.5,
            MakeFakeRsrp(9),
            [&](const InferenceResponse& r) {
                fired = true;
                out = r;
            });

        server.Poll(50);
        client.Poll(50);
        NS_TEST_EXPECT_MSG_EQ(fired, true, "callback fired");
        NS_TEST_EXPECT_MSG_EQ(out.status, 0u, "status");
        const bool tcp_beam_kind =
            out.output_kind == InferenceResponse::OutputKind::beam;
        NS_TEST_EXPECT_MSG_EQ(
            tcp_beam_kind,
            true,
            "beam response");

        server.Stop();
        client.Close();
    }
};

// ----------------------------------------------------------------------------

class AiranFailureModesTest : public TestCase
{
  public:
    AiranFailureModesTest()
        : TestCase("Failure modes: unknown model + mismatched input (T7)")
    {
    }
    void DoRun() override
    {
        AiranInferenceServer server;
        server.RegisterModel(LoadPrecoderConfig(),
                              AiranMockRuntime::MakePrecoderHandler());

        auto pair = InProcInferenceChannel::CreatePair();
        AiranInferenceClient client;
        client.Attach(std::move(pair.first));
        server.AddChannel(std::move(pair.second));

        // 1. Unknown model
        InferenceResponse rA;
        bool cbA = false;
        client.SubmitPrecoderRequest(
            "no_such_model",
            1,
            1,
            0.0,
            MakeFakeCsi(1),
            [&](const InferenceResponse& r) {
                cbA = true;
                rA = r;
            });
        server.Poll(5);
        client.Poll(5);
        NS_TEST_EXPECT_MSG_EQ(cbA, true, "unknown-model cb fired");
        NS_TEST_EXPECT_MSG_EQ(rA.status, 1u, "unknown model status=1");

        // 2. Mismatched input kind (beam request to a precoder model
        //    — we send RSRP to the registered precoder model name)
        bool cbB = false;
        InferenceResponse rB;
        client.SubmitBeamRequest(
            "precoder_csi_to_weights",
            2,
            2,
            0.0,
            MakeFakeRsrp(1),
            [&](const InferenceResponse& r) {
                cbB = true;
                rB = r;
            });
        server.Poll(5);
        client.Poll(5);
        NS_TEST_EXPECT_MSG_EQ(cbB, true, "mismatch cb fired");
        NS_TEST_EXPECT_MSG_EQ(rB.status,
                               2u,
                               "mismatched-input status=2");

        NS_TEST_EXPECT_MSG_EQ(server.RequestsHandled(),
                               0u,
                               "none handled");
        NS_TEST_EXPECT_MSG_EQ(server.RequestsRejected(),
                               2u,
                               "two rejected");
    }
};

// ----------------------------------------------------------------------------
// End-to-end Simulator::Run() workload — 60 simulated seconds, periodic
// inference requests, drains via scheduled Poll() ticks.

class AiranSimulatorWorkloadTest : public TestCase
{
  public:
    AiranSimulatorWorkloadTest()
        : TestCase("End-to-end inference workload under Simulator::Run (T7)")
    {
    }
    void DoRun() override
    {
        auto server = std::make_shared<AiranInferenceServer>();
        server->RegisterModel(LoadPrecoderConfig(),
                               AiranMockRuntime::MakePrecoderHandler(
                                   /*num_layers=*/4,
                                   /*base_latency_ms=*/0.4));
        server->RegisterModel(LoadBeamConfig(),
                               AiranMockRuntime::MakeBeamHandler(
                                   /*top_k=*/4,
                                   /*latency_ms=*/0.25));

        auto pair = InProcInferenceChannel::CreatePair();
        auto client = std::make_shared<AiranInferenceClient>();
        client->Attach(std::move(pair.first));
        server->AddChannel(std::move(pair.second));

        std::atomic<uint64_t> precoder_responses{0};
        std::atomic<uint64_t> beam_responses{0};
        std::atomic<double> sum_latency_ms{0.0};

        const uint32_t kNumRequests = 100;
        // Schedule requests across t = [1 s, 50 s].
        for (uint32_t i = 0; i < kNumRequests; ++i)
        {
            const double when_s =
                1.0 +
                49.0 * (static_cast<double>(i) / kNumRequests);
            Simulator::Schedule(
                Seconds(when_s),
                [client, &precoder_responses, &beam_responses,
                 &sum_latency_ms, i, when_s] {
                    if (i % 2 == 0)
                    {
                        client->SubmitPrecoderRequest(
                            "precoder_csi_to_weights",
                            /*ue_id=*/i,
                            /*nr_cgi=*/1 + (i % 4),
                            when_s,
                            MakeFakeCsi(i),
                            [&](const InferenceResponse& resp) {
                                if (resp.status == 0)
                                {
                                    ++precoder_responses;
                                    sum_latency_ms.store(
                                        sum_latency_ms.load() +
                                        resp.inference_latency_ms);
                                }
                            });
                    }
                    else
                    {
                        client->SubmitBeamRequest(
                            "beam_index_classifier",
                            /*ue_id=*/i,
                            /*nr_cgi=*/1 + (i % 4),
                            when_s,
                            MakeFakeRsrp(i),
                            [&](const InferenceResponse& resp) {
                                if (resp.status == 0)
                                {
                                    ++beam_responses;
                                    sum_latency_ms.store(
                                        sum_latency_ms.load() +
                                        resp.inference_latency_ms);
                                }
                            });
                    }
                });
        }

        // Periodic pump ticks at 50 ms — server then client.
        for (uint32_t t_ms = 50; t_ms <= 60'000; t_ms += 50)
        {
            Simulator::Schedule(
                MilliSeconds(t_ms),
                [server, client] {
                    server->Poll(0, /*max=*/64);
                    client->Poll(0, /*max=*/64);
                });
        }
        Simulator::Stop(Seconds(60.0));
        Simulator::Run();
        // One final drain post-Run to flush in-flight frames the
        // periodic pump may have missed at the boundary.
        server->Poll(5, 64);
        client->Poll(5, 64);
        Simulator::Destroy();

        NS_TEST_EXPECT_MSG_EQ(precoder_responses.load(),
                               static_cast<uint64_t>(kNumRequests / 2),
                               "50 precoder responses");
        NS_TEST_EXPECT_MSG_EQ(beam_responses.load(),
                               static_cast<uint64_t>(kNumRequests / 2),
                               "50 beam responses");
        NS_TEST_EXPECT_MSG_EQ(client->PendingCount(),
                               0u,
                               "no pending at end");
        NS_TEST_EXPECT_MSG_EQ(client->TotalOrphan(),
                               0u,
                               "no orphans");
        const double avg_ms =
            sum_latency_ms.load() / kNumRequests;
        NS_TEST_EXPECT_MSG_GT(avg_ms, 0.0, "latency > 0");
        NS_TEST_EXPECT_MSG_LT(avg_ms, 5.0, "latency < 5 ms");
    }
};

// ----------------------------------------------------------------------------


/// AI-06: the precoder must be a function of the CHANNEL.
///
/// The mock computed cos/sin of `seed = req.request_id`, reading only
/// csi.num_tx. Two consequences, both bad: the same channel produced a
/// different beam on every call, and two completely different channels produced
/// the same beam if the counters lined up. No CSI influenced any output
/// anywhere in the toolkit, so a transposed or stale tensor was undetectable.
class MockPrecoderIsAFunctionOfCsiTest : public TestCase
{
  public:
    MockPrecoderIsAFunctionOfCsiTest()
        : TestCase("AI-06: the mock precoder depends on the CSI, not the request counter")
    {
    }

  private:
    static oranntn::airan::CsiTensor MakeCsi(uint32_t nTx, uint32_t nRx, uint32_t nSc,
                                            double phase)
    {
        oranntn::airan::CsiTensor csi;
        csi.num_tx = nTx;
        csi.num_rx = nRx;
        csi.num_subcarriers = nSc;
        csi.values.assign(csi.ExpectedSize(), 0.0f);
        for (uint32_t sc = 0; sc < nSc; ++sc)
        {
            for (uint32_t rx = 0; rx < nRx; ++rx)
            {
                for (uint32_t tx = 0; tx < nTx; ++tx)
                {
                    const double a = phase + 0.7 * tx + 0.31 * rx + 0.03 * sc;
                    const std::size_t o = csi.Index(sc, rx, tx);
                    csi.values[o] = static_cast<float>(std::cos(a));
                    csi.values[o + 1] = static_cast<float>(std::sin(a));
                }
            }
        }
        return csi;
    }

    static oranntn::airan::InferenceResponse Run(
        const oranntn::airan::CsiTensor& csi, uint64_t requestId, uint32_t layers,
        uint32_t expectedNumTx = 0)
    {
        auto handler = AiranMockRuntime::MakePrecoderHandler(layers, 0.4, expectedNumTx);
        oranntn::airan::InferenceRequest req;
        req.request_id = requestId;
        req.csi = csi;
        oranntn::airan::InferenceResponse out;
        handler(req, out);
        return out;
    }

    void DoRun() override
    {
        const uint32_t nTx = 4;
        const auto csiA = MakeCsi(nTx, 2, 8, 0.0);
        const auto csiB = MakeCsi(nTx, 2, 8, 1.1);

        // 1. SAME channel, DIFFERENT request id -> same precoder.
        //    This is the assertion the old kernel could never satisfy: its
        //    entire output was a function of the counter.
        const auto r1 = Run(csiA, 1, 2);
        const auto r2 = Run(csiA, 99999, 2);
        NS_TEST_ASSERT_MSG_EQ(r1.status, 0, "a valid tensor is accepted");
        NS_TEST_ASSERT_MSG_EQ(r1.precoder.values.size(), r2.precoder.values.size(),
                              "same shape");
        double maxDiff = 0.0;
        for (std::size_t i = 0; i < r1.precoder.values.size(); ++i)
        {
            maxDiff = std::max(maxDiff,
                               std::abs(static_cast<double>(r1.precoder.values[i]) -
                                        static_cast<double>(r2.precoder.values[i])));
        }
        NS_TEST_ASSERT_MSG_LT(maxDiff, 1e-9,
                              "the same channel must give the same beam whatever the request "
                              "counter says; a difference here means the counter is still in "
                              "the kernel");

        // 2. DIFFERENT channel, same request id -> different precoder.
        const auto r3 = Run(csiB, 1, 2);
        double maxChange = 0.0;
        for (std::size_t i = 0; i < r1.precoder.values.size(); ++i)
        {
            maxChange = std::max(maxChange,
                                 std::abs(static_cast<double>(r1.precoder.values[i]) -
                                          static_cast<double>(r3.precoder.values[i])));
        }
        NS_TEST_ASSERT_MSG_GT(maxChange, 1e-3,
                              "a different channel must give a different beam, or the CSI is "
                              "still being ignored");

        // 3. MRT property, checked against the closed form.
        //    Single layer, single rx: w = conj(hbar)/||hbar||, so h . w is real
        //    and equals ||hbar||. That is the definition of maximum-ratio
        //    transmission and a beam that does not satisfy it is not one.
        const auto csiS = MakeCsi(nTx, 1, 4, 0.4);
        const auto rs = Run(csiS, 7, 1);
        NS_TEST_ASSERT_MSG_EQ(rs.status, 0, "single-layer request accepted");

        std::vector<std::complex<double>> hbar(nTx, {0.0, 0.0});
        for (uint32_t sc = 0; sc < csiS.num_subcarriers; ++sc)
        {
            for (uint32_t tx = 0; tx < nTx; ++tx)
            {
                const std::size_t o = csiS.Index(sc, 0, tx);
                hbar[tx] += std::complex<double>(csiS.values[o], csiS.values[o + 1]);
            }
        }
        double hnorm = 0.0;
        for (auto& v : hbar)
        {
            v /= static_cast<double>(csiS.num_subcarriers);
            hnorm += std::norm(v);
        }
        hnorm = std::sqrt(hnorm);

        std::complex<double> gain{0.0, 0.0};
        double wnorm = 0.0;
        for (uint32_t tx = 0; tx < nTx; ++tx)
        {
            const std::complex<double> w(rs.precoder.values[2 * tx],
                                         rs.precoder.values[2 * tx + 1]);
            gain += hbar[tx] * w;
            wnorm += std::norm(w);
        }
        NS_TEST_ASSERT_MSG_EQ_TOL(std::sqrt(wnorm), 1.0, 1e-5,
                                  "the precoder is unit norm, so it does not invent power");
        NS_TEST_ASSERT_MSG_EQ_TOL(std::abs(gain), hnorm, 1e-5,
                                  "MRT achieves the full channel norm as array gain; a lower "
                                  "value means the weights are not matched to the channel");
        NS_TEST_ASSERT_MSG_LT(std::abs(gain.imag()), 1e-5,
                              "and the combined gain is real, which is what conjugating the "
                              "channel buys");

        // 4. A tensor that does not match its declared dimensions must be
        //    REFUSED. Silently beamforming on a mis-shaped buffer is how a
        //    layout error survives to the PHY.
        auto bad = csiA;
        bad.values.pop_back();
        const auto rbad = Run(bad, 3, 2);
        NS_TEST_ASSERT_MSG_NE(rbad.status, 0,
                              "a CSI buffer whose size disagrees with num_tx/num_rx/num_sc is "
                              "not usable and must be reported, not beamformed on");

        // Transposing num_tx with num_subcarriers leaves the total element
        // count unchanged, so a size check alone cannot see it. What catches it
        // is the model's DECLARED input shape, which is what a Triton
        // config.pbtxt carries and what a real deployment would enforce.
        auto swapped = csiA;
        std::swap(swapped.num_tx, swapped.num_subcarriers);
        NS_TEST_ASSERT_MSG_EQ(swapped.values.size(), swapped.ExpectedSize(),
                              "the transposed tensor is the same SIZE, which is precisely why "
                              "a size check cannot catch it");
        const auto rswapUnchecked = Run(swapped, 4, 2);
        NS_TEST_ASSERT_MSG_EQ(rswapUnchecked.status, 0,
                              "with no declared shape the transposition is accepted, and this "
                              "is recorded rather than papered over");
        const auto rswap = Run(swapped, 4, 2, /*expectedNumTx=*/nTx);
        NS_TEST_ASSERT_MSG_NE(rswap.status, 0,
                              "a model that declares its input shape rejects the transposition");
        // And the declared shape must not reject the CORRECT tensor.
        NS_TEST_ASSERT_MSG_EQ(Run(csiA, 5, 2, nTx).status, 0,
                              "the declared shape accepts a conforming tensor");
    }
};

class AiranInferenceTestSuite : public TestSuite
{
  public:
    AiranInferenceTestSuite()
        : TestSuite("oran-ntn-airan-inference", Type::UNIT)
    {
        AddTestCase(new MockPrecoderIsAFunctionOfCsiTest, TestCase::Duration::QUICK);
        AddTestCase(new TritonModelConfigParserTest(),
                     TestCase::Duration::QUICK);
        AddTestCase(new TritonModelConfigShippedFilesTest(),
                     TestCase::Duration::QUICK);
        AddTestCase(new AiranMessageCodecRoundTripTest(),
                     TestCase::Duration::QUICK);
        AddTestCase(new InProcChannelRoundTripTest(),
                     TestCase::Duration::QUICK);
        AddTestCase(new TcpChannelRoundTripTest(),
                     TestCase::Duration::QUICK);
        AddTestCase(new AiranInProcRoundTripTest(),
                     TestCase::Duration::QUICK);
        AddTestCase(new AiranTcpRoundTripTest(),
                     TestCase::Duration::QUICK);
        AddTestCase(new AiranFailureModesTest(),
                     TestCase::Duration::QUICK);
        AddTestCase(new AiranSimulatorWorkloadTest(),
                     TestCase::Duration::QUICK);
    }
};

static AiranInferenceTestSuite g_airanInferenceTestSuite;
