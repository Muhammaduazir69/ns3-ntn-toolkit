/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * O-RAN NTN control-loop latency probe (reviewer response R2.7).
 *
 * WHY THIS EXISTS
 * ---------------
 * An earlier version of the toolkit paper reported "the controller spends
 * under a third of a second of computation over the entire run". That number
 * was the accumulated CPU cost of the xApp decision function ALONE. It said
 * nothing about the cost a reader actually cares about: the end-to-end
 * control-loop latency from the instant a KPM measurement is taken on the
 * RAN to the instant the resulting RC action changes the radio.
 *
 * This class replaces that single number with a per-stage breakdown of the
 * whole loop, separated into two physically different quantities that must
 * never be summed together:
 *
 *   Kind::CPU        wall-clock host CPU time (std::chrono::steady_clock)
 *                    spent executing a piece of controller code. This is
 *                    simulator overhead: it does NOT advance the simulated
 *                    clock and would be replaced by the real implementation's
 *                    cost on a deployed RIC.
 *
 *   Kind::SIMULATED  ns-3 simulated time (Simulator::Now() deltas) elapsed
 *                    across a modeled transport or scheduling wait — the
 *                    feeder-link legs and the Near-RT RIC tick alignment.
 *                    This is what a deployed system would experience as
 *                    latency.
 *
 * HONESTY CONSTRAINTS (read before citing any row this class emits)
 * -----------------------------------------------------------------
 *  1. E2AP-over-SCTP is NOT simulated in this toolkit (see
 *     oran-ntn-e2-interface.h). There is therefore NO wire encode/decode on
 *     the in-simulation E2 path, and no row here may be presented as
 *     "E2AP serialization cost". The on-path stand-in is
 *     `e2_indication_construct`, which measures exactly what its name says:
 *     E2Indication message construction plus enqueueing of the delayed
 *     delivery event inside OranNtnE2Node::SubmitKpmMeasurement.
 *  2. The toolkit does own a real E2SM-KPM encoder
 *     (OranNtnServiceModelKpm::EncodeIndication over the Aligned-PER subset
 *     writer in asn1/asn1-per-codec.h). Callers may time it, but it is not
 *     on the in-sim delivery path and it is not bit-conformant PER, so its
 *     stage MUST be named
 *     `kpm_serialize_e2sm_kpm_per_subset_offpath` (constant
 *     oranntn::loopstage::kKpmSerializeOffPath) so the "_offpath" suffix
 *     travels with the number into any table.
 *  3. `loop_total` is NOT the sum of the stages. It is recorded by
 *     RecordLoop() from the actual measurement instant to the actual
 *     actuation instant, so it captures every wait the decomposition misses.
 *     Summing the per-stage rows and comparing to loop_total is a legitimate
 *     residual check, not a substitute.
 *  4. Instrument cost is itself measurable: MeasureInstrumentOverhead()
 *     records empty Start/EndStage pairs under `probe_self_overhead` so a
 *     reader can see how much of a CPU row is the timer and not the work.
 */

#ifndef ORAN_NTN_LOOP_LATENCY_PROBE_H
#define ORAN_NTN_LOOP_LATENCY_PROBE_H

#include <ns3/nstime.h>
#include <ns3/object.h>

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace ns3
{

/**
 * \ingroup oran-ntn
 * \brief Canonical stage names for the O-RAN NTN closed control loop.
 *
 * Using these constants (rather than string literals at the call site) keeps
 * the CSV column vocabulary stable across examples and tests.
 */
namespace oranntn
{
namespace loopstage
{

// ---- CPU stages (host compute; do not advance simulated time) ----

/// S1: building the E2KpmReport from MEASURED radio state (PHY trace read,
/// geometry). Real code, on the loop path.
inline constexpr const char* kKpmBuild = "kpm_build";

/// S2 (on-path): E2Indication construction + subscription match + enqueue of
/// the delayed delivery event, inside OranNtnE2Node::SubmitKpmMeasurement.
/// This is NOT a wire serializer — E2AP/SCTP is not simulated. Named for
/// exactly what it measures.
inline constexpr const char* kE2IndicationConstruct = "e2_indication_construct";

/// S2 (off-path, optional): real E2SM-KPM Indication Format 1 encode through
/// OranNtnServiceModelKpm over the Aligned-PER SUBSET writer. Not bit-
/// conformant PER and NOT on the in-sim delivery path — hence "_offpath".
inline constexpr const char* kKpmSerializeOffPath =
    "kpm_serialize_e2sm_kpm_per_subset_offpath";

/// S4: the xApp decision logic. This — and only this — is what the retracted
/// "under a third of a second" claim measured.
inline constexpr const char* kXappCompute = "xapp_compute";

/// S5a: A1 policy compliance check inside OranNtnNearRtRic::ProcessXappAction.
/// NOTE: the shipped A1 HO_THRESHOLD policies govern HANDOVER_TRIGGER actions
/// only; for other action types the check short-circuits, so this row is a
/// lower bound for policy-governed action types.
inline constexpr const char* kRicPolicyCheck = "ric_a1_policy_check";

/// S5b: conflict manager CheckAndResolve inside ProcessXappAction.
inline constexpr const char* kRicConflictCheck = "ric_conflict_check";

/// S6: OranNtnE2Termination::RouteRcAction — RIC to E2 termination routing
/// (target lookup / fan-out + enqueue of the downlink delivery event).
inline constexpr const char* kRcActionRoute = "rc_action_route";

/// S8: applying the action to the radio (the E2 node's RC action callback).
inline constexpr const char* kActuation = "actuation";

/// Instrument self-cost: empty Start/EndStage pairs.
inline constexpr const char* kProbeSelfOverhead = "probe_self_overhead";

// ---- SIMULATED stages (modeled transport / scheduling waits) ----

/// S3a: measurement instant -> arrival at the RIC side of the feeder link
/// (OranNtnE2Node::DeliverReport). One uplink FeederLinkDelay.
inline constexpr const char* kIndicationFeederUplink =
    "e2_indication_feeder_uplink";

/// S3b: feeder arrival -> dispatch to the xApp callback. Nonzero only when
/// OranNtnE2Node's AlignToControlLoop is enabled (wait for the next Near-RT
/// RIC tick); exactly zero otherwise.
inline constexpr const char* kRicTickAlignWait = "ric_tick_align_wait";

/// S3: measurement instant -> xApp callback entry (feeder uplink + tick wait),
/// measured directly, not summed.
inline constexpr const char* kIndicationTransport = "e2_indication_transport";

/// S7: OranNtnE2Termination::RouteRcAction / OranNtnE2Node::ReceiveRcAction
/// -> actuation on the E2 node. The downlink FeederLinkDelay leg.
inline constexpr const char* kRcActionTransport = "rc_action_transport";

/// End-to-end: KPM measurement instant -> actuation instant, per closed-loop
/// iteration. Recorded by RecordLoop(); never summed from stages.
inline constexpr const char* kLoopTotal = "loop_total";

} // namespace loopstage
} // namespace oranntn

/**
 * \ingroup oran-ntn
 * \brief Per-stage latency accumulator for the O-RAN NTN control loop.
 *
 * Typical wiring (see examples/oran-ntn-ric-controlled-traffic.cc):
 * \code
 *   Ptr<OranNtnLoopLatencyProbe> probe = CreateObject<OranNtnLoopLatencyProbe>();
 *   e2Node->SetLoopLatencyProbe(probe);   // S2, S3, S7, S8
 *   ric->SetLoopLatencyProbe(probe);      // S5, S6
 *   // S1 / S4 are timed at the measurement and decision call sites:
 *   { OranNtnLoopLatencyProbe::ScopedCpuTimer t(probe, loopstage::kKpmBuild); ... }
 *   // once the action actuates:
 *   probe->RecordLoop(measurementTime, Simulator::Now());
 *   probe->WriteCsv("control_loop_latency.csv");
 *   probe->WriteSamplesCsv("control_loop_samples.csv");
 * \endcode
 *
 * All accumulated samples are stored in microseconds. Percentiles use the
 * nearest-rank definition (index = ceil(p*n) - 1 on the sorted sample list),
 * so every reported percentile is an OBSERVED sample, never an interpolation.
 */
class OranNtnLoopLatencyProbe : public Object
{
  public:
    /// Sample kind. CPU and SIMULATED samples are never mixed in one row.
    enum class Kind : uint8_t
    {
        CPU = 0,      //!< host compute time (std::chrono::steady_clock)
        SIMULATED = 1 //!< ns-3 simulated time (Simulator::Now() delta)
    };

    /// Summary of one (stage, kind) sample set. All fields in microseconds.
    struct StageStats
    {
        uint64_t count{0};
        double mean_us{0.0};
        double p50_us{0.0};
        double p95_us{0.0};
        double p99_us{0.0};
        double min_us{0.0};
        double max_us{0.0};
    };

    /// One closed-loop iteration: measurement instant -> actuation instant.
    struct LoopSample
    {
        uint32_t iter{0};
        double measure_t_s{0.0};
        double actuation_t_s{0.0};
        double loop_latency_ms{0.0};
    };

    static TypeId GetTypeId();
    OranNtnLoopLatencyProbe() = default;
    ~OranNtnLoopLatencyProbe() override = default;

    // ---- CPU stage timing ----

    /**
     * \brief Open a CPU stage. A second Start on an already-open stage
     *        restarts it (the earlier open is discarded, not double counted).
     */
    void StartStage(const std::string& stage);

    /**
     * \brief Close a CPU stage opened by StartStage() and record the elapsed
     *        host time. A close without a matching open is ignored.
     */
    void EndStage(const std::string& stage);

    /// Record an already-measured CPU sample (microseconds).
    void RecordCpuSample(const std::string& stage, double us);

    /// RAII form of StartStage()/EndStage(). Safe with a null probe.
    class ScopedCpuTimer
    {
      public:
        ScopedCpuTimer(Ptr<OranNtnLoopLatencyProbe> probe, std::string stage);
        ~ScopedCpuTimer();
        ScopedCpuTimer(const ScopedCpuTimer&) = delete;
        ScopedCpuTimer& operator=(const ScopedCpuTimer&) = delete;

      private:
        Ptr<OranNtnLoopLatencyProbe> m_probe;
        std::string m_stage;
        std::chrono::steady_clock::time_point m_start;
    };

    // ---- Simulated stage timing ----

    /// Record a simulated-time span for a modeled transport / scheduling wait.
    void RecordSimulatedStage(const std::string& stage, Time delta);

    /**
     * \brief Record one closed-loop iteration end to end.
     *
     * \param measureTime   simulated instant the KPM measurement was taken
     * \param actuationTime simulated instant the action changed the radio
     *
     * Adds a `loop_total` SIMULATED sample AND a raw per-iteration row for
     * WriteSamplesCsv(). The value is measured, never summed from stages.
     */
    void RecordLoop(Time measureTime, Time actuationTime);

    // ---- Instrument self-cost ----

    /**
     * \brief Time \p iterations empty Start/EndStage pairs and record them
     *        under `probe_self_overhead`, so a reader can judge how much of a
     *        CPU row is the timer itself.
     */
    void MeasureInstrumentOverhead(uint32_t iterations);

    // ---- Query ----

    bool HasStage(const std::string& stage, Kind kind) const;
    uint64_t GetStageCount(const std::string& stage, Kind kind) const;
    /// \return false (and leaves \p out untouched) if the stage has no samples.
    bool GetStats(const std::string& stage, Kind kind, StageStats& out) const;
    const std::vector<LoopSample>& GetLoopSamples() const { return m_loopSamples; }
    /// Every (stage, kind) key that holds at least one sample, in insertion order.
    std::vector<std::pair<std::string, Kind>> GetStageKeys() const;

    void Clear();

    // ---- Output ----

    /**
     * \brief Write the per-stage summary.
     *
     * Schema: stage,kind,count,mean_us,p50_us,p95_us,p99_us,min_us,max_us
     * with kind in {cpu, simulated}. Rows are emitted in the order the stages
     * were first observed, so the file reads in loop order; `loop_total`
     * (simulated) is the end-to-end row.
     *
     * \return false if the file could not be opened.
     */
    bool WriteCsv(const std::string& path) const;

    /**
     * \brief Write the raw per-iteration loop samples so the distribution can
     *        be plotted.
     *
     * Schema: iter,measure_t_s,actuation_t_s,loop_latency_ms
     *
     * \return false if the file could not be opened.
     */
    bool WriteSamplesCsv(const std::string& path) const;

    static const char* KindToString(Kind k);

  protected:
    void DoDispose() override;

  private:
    using Key = std::pair<std::string, Kind>;

    static double Percentile(const std::vector<double>& sorted, double p);

    std::map<Key, std::vector<double>> m_samples;      //!< microseconds
    std::vector<Key> m_order;                          //!< first-seen order
    std::map<std::string, std::chrono::steady_clock::time_point> m_open;
    std::vector<LoopSample> m_loopSamples;
};

} // namespace ns3

#endif // ORAN_NTN_LOOP_LATENCY_PROBE_H
