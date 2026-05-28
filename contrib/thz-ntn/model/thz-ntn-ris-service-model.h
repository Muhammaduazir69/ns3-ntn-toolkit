/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

// thz-ntn-ris-service-model — Roadmap §4.3.6 THz-RIS-SM service model.
//
// Defines an O-RAN-style service model around `ThzNtnRisController`:
//
//   - IndicationMessage: current RIS phase profile + the most recent
//     estimated channel gain. Emitted periodically.
//   - ControlMessage: target phase profile + optimisation hint
//     (codebook / random / DRL). Issued by an xApp.
//   - Policy: phase-profile strategy enum (one of CODEBOOK, RANDOM_SEARCH,
//     ALTERNATING_OPT, DRL_BASED). Issued by an A1-style policy emitter
//     that wishes to coarsely steer the xApp's behaviour.
//
// The classes are intentionally generic; they integrate with the toolkit's
// existing `oran-ntn` E2 / A1 surface once W8 lands the full SM plugin ABI
// (Roadmap T4). The service-model body here uses ns-3 callbacks
// (`SubscribeIndications`, `SendControl`, `ApplyPolicy`) so it can be
// driven from either a Python xApp via the FlexRIC bridge or a pure-ns-3
// xApp stub like `ThzNtnRisXapp`.

#ifndef THZ_NTN_RIS_SERVICE_MODEL_H
#define THZ_NTN_RIS_SERVICE_MODEL_H

#include "thz-ntn-ris.h"
#include "thz-ntn-ris-controller.h"

#include <ns3/callback.h>
#include <ns3/event-id.h>
#include <ns3/nstime.h>
#include <ns3/object.h>

#include <cstdint>
#include <vector>

namespace ns3
{

/**
 * \ingroup thz-ntn
 *
 * \brief Service model around ThzNtnRisController (Roadmap §4.3.6).
 *
 * Indications + controls + policy. The class doesn't tie to a specific
 * transport; an external xApp calls `OnControl()` to issue control
 * directives, calls `OnPolicy()` to install an A1 policy, and subscribes
 * to indications via `SubscribeIndications()`.
 */
class ThzNtnRisServiceModel : public Object
{
  public:
    /// Periodic Indication contents.
    struct IndicationMessage
    {
        Time timestamp;
        uint32_t risId{0};
        std::vector<double> phaseProfile_rad; //!< current phase profile
        double estimatedChannelGain_dB{0.0};   //!< RIS reflection gain
        double snr_dB{0.0};                    //!< most recent measured SNR
    };

    /// Control directive issued by an xApp.
    struct ControlMessage
    {
        uint32_t risId{0};
        std::vector<double> targetPhaseProfile_rad; //!< desired phases
        OptimizationMethod method{
            OptimizationMethod::CODEBOOK};
    };

    /// A1-style policy. The xApp may consult this to choose `method`.
    enum class Policy : uint8_t
    {
        CODEBOOK = 0,
        RANDOM_SEARCH = 1,
        ALTERNATING_OPT = 2,
        DRL_BASED = 3,
    };

    using IndicationCallback = Callback<void, const IndicationMessage&>;

    static TypeId GetTypeId();

    ThzNtnRisServiceModel();
    ~ThzNtnRisServiceModel() override;

    void SetRis(Ptr<ThzNtnRis> ris) { m_ris = ris; }
    Ptr<ThzNtnRis> GetRis() const { return m_ris; }

    void SetController(Ptr<ThzNtnRisController> ctl) { m_controller = ctl; }
    Ptr<ThzNtnRisController> GetController() const { return m_controller; }

    /// Set / get the RIS instance ID surfaced on every message.
    void SetRisId(uint32_t id) { m_risId = id; }
    uint32_t GetRisId() const { return m_risId; }

    /// Period of the periodic Indication emission. Default: 100 ms.
    void SetIndicationPeriod(Time t) { m_indicationPeriod = t; }
    Time GetIndicationPeriod() const { return m_indicationPeriod; }

    /// Start / stop the periodic Indication emitter.
    void StartIndications();
    void StopIndications();

    /// Register a callback that fires for every Indication. Multiple
    /// subscribers are supported.
    void SubscribeIndications(IndicationCallback cb);

    /// Inject a control directive (xApp → service-model).
    void OnControl(const ControlMessage& ctl);

    /// Install a coarse A1 policy. The xApp consults `GetActivePolicy()`
    /// to bias its method choice.
    void OnPolicy(Policy p);
    Policy GetActivePolicy() const { return m_policy; }

    /// Stats for tests/observability.
    uint64_t GetIndicationsEmitted() const { return m_indEmitted; }
    uint64_t GetControlsAccepted() const { return m_ctlAccepted; }
    uint64_t GetPolicyUpdates() const { return m_polUpdates; }

  private:
    void EmitIndication();

    Ptr<ThzNtnRis> m_ris;
    Ptr<ThzNtnRisController> m_controller;
    uint32_t m_risId{0};
    Time m_indicationPeriod{MilliSeconds(100)};
    EventId m_indicationEvent;
    Policy m_policy{Policy::CODEBOOK};

    std::vector<IndicationCallback> m_subscribers;
    std::vector<double> m_currentPhases;
    double m_lastChannelGain_dB{0.0};

    uint64_t m_indEmitted{0};
    uint64_t m_ctlAccepted{0};
    uint64_t m_polUpdates{0};
};

} // namespace ns3

#endif // THZ_NTN_RIS_SERVICE_MODEL_H
