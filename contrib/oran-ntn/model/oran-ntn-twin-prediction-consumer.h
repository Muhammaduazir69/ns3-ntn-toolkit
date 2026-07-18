/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_TWIN_PREDICTION_CONSUMER_H
#define ORAN_NTN_TWIN_PREDICTION_CONSUMER_H

#include "ns3/callback.h"
#include "ns3/nstime.h"
#include "ns3/object.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ns3
{

/**
 * \ingroup oran-ntn
 * \brief Consumes the digital twin's handover predictions and drives them into
 *        the running ns-3 O-RAN E2/CHO loop — the C++ side of the twin<->sim loop.
 *
 * The Python digital twin (contrib/ntn-digital-twin) predicts, ahead of time,
 * which satellite each UE should be served by, using the SAME Walker elements and
 * SHARED EPOCH the C++ simulation runs (the W1 fix). Until now nothing consumed
 * those predictions inside ns-3: the twin was a Non-RT-RIC / rApp with no actuator.
 *
 * This class closes that loop. The twin exports its handover schedule to a plain
 * newline-delimited file (the `emit_predictions_file` helper in the twin package
 * writes the identical format):
 *
 *     # ntn-twin handover predictions  epoch_unix=<...>
 *     # t_s,ueId,recommendedGnbId,confidence
 *     12.5,0,2,0.94
 *     37.0,0,3,0.88
 *
 * `t_s` is seconds from the shared epoch (i.e. simulation time). The consumer
 * loads the file and, on Start(), schedules each prediction at its `t_s` to fire a
 * user-supplied submit callback — wire that to an xApp's SubmitAction
 * (HANDOVER_TRIGGER) or directly to NtnRealStackHelper::TriggerHandover, and the
 * twin's foresight actuates a real handover. A CHO algorithm can instead poll
 * GetRecommendation() to rank candidates by the twin's guidance (Non-RT policy).
 */
class OranNtnTwinPredictionConsumer : public Object
{
  public:
    /// One predicted handover from the twin.
    struct TwinPrediction
    {
        double tSec;                //!< time from shared epoch (== sim time), seconds
        uint32_t ueId;              //!< UE id (matches the sim's UE indexing)
        uint32_t recommendedGnbId;  //!< twin-recommended serving cell / satellite E2 id
        double confidence;          //!< 0..1
    };

    /// submit(ueId, recommendedGnbId, confidence) — wire to an xApp / real stack.
    typedef Callback<void, uint32_t, uint32_t, double> HandoverSubmitCallback;

    static TypeId GetTypeId();
    OranNtnTwinPredictionConsumer();
    ~OranNtnTwinPredictionConsumer() override;

    /**
     * \brief Load predictions from a twin-exported file. Returns the count loaded.
     * Lines starting with '#' or blank are ignored; malformed lines are skipped
     * with a warning (no silent fabrication). Predictions are kept sorted by time.
     */
    uint32_t LoadPredictionsFromFile(const std::string& path);

    /// Add one prediction programmatically (kept sorted by time).
    void AddPrediction(double tSec, uint32_t ueId, uint32_t recommendedGnbId, double confidence);

    /// Number of predictions currently held.
    uint32_t GetNumPredictions() const;

    /**
     * \brief Non-RT policy query: the twin's latest recommendation for \p ueId at
     * or before \p now. Returns false if none applies yet.
     */
    bool GetRecommendation(uint32_t ueId, Time now, uint32_t& recommendedGnbId, double& confidence) const;

    /**
     * \brief Schedule every prediction (confidence >= \p minConfidence) to fire
     * \p submit at its time. This is what closes the loop into the running sim.
     */
    void Start(HandoverSubmitCallback submit, double minConfidence = 0.0);

    /// How many predictions were actuated (fired the submit callback) so far.
    uint32_t GetActuatedCount() const;

  private:
    void Fire(uint32_t index);

    std::vector<TwinPrediction> m_predictions; //!< sorted by tSec
    HandoverSubmitCallback m_submit;
    double m_minConfidence{0.0};
    uint32_t m_actuated{0};
    bool m_started{false};
};

} // namespace ns3

#endif // ORAN_NTN_TWIN_PREDICTION_CONSUMER_H
