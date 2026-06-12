/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

// thz-ntn-ris-xapp — minimal in-process xApp that subscribes to a
// ThzNtnRisServiceModel's indications and emits codebook-driven
// ControlMessages back. The xApp's method is biased by the active A1
// policy (CODEBOOK / RANDOM_SEARCH / ALTERNATING_OPT / DRL_BASED) but the
// v2.x baseline always emits CODEBOOK directives. Replace with a richer
// implementation once W8 lands the SM plugin ABI.

#ifndef THZ_NTN_RIS_XAPP_H
#define THZ_NTN_RIS_XAPP_H

#include "thz-ntn-ris-service-model.h"

#include <ns3/object.h>

namespace ns3
{

/**
 * \ingroup thz-ntn
 * \brief Minimal in-process RIS xApp (Roadmap §4.3.6).
 *
 * \warning Experimental: validated by the unit test suite
 * (test/thz-ntn-test-suite.cc) but not yet exercised by any example;
 * the measured-radio RIS xApp loop lives in thz-ntn-ric-controlled-traffic.
 */
class ThzNtnRisXapp : public Object
{
  public:
    static TypeId GetTypeId();

    ThzNtnRisXapp();
    ~ThzNtnRisXapp() override;

    /// Bind to a service model. Subscribes to indications + arms the
    /// control reply pipeline.
    void Attach(Ptr<ThzNtnRisServiceModel> sm);
    Ptr<ThzNtnRisServiceModel> GetServiceModel() const { return m_sm; }

    /// Stats.
    uint64_t GetIndicationsObserved() const { return m_indSeen; }
    uint64_t GetControlsSent() const { return m_ctlSent; }

  private:
    void OnIndication(const ThzNtnRisServiceModel::IndicationMessage& msg);

    Ptr<ThzNtnRisServiceModel> m_sm;
    uint64_t m_indSeen{0};
    uint64_t m_ctlSent{0};
};

} // namespace ns3

#endif // THZ_NTN_RIS_XAPP_H
