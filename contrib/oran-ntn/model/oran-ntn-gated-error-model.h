/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * OranNtnGatedErrorModel (Wave 7B): a receive error model that drops every
 * packet while a gate predicate holds and otherwise defers to an inner model
 * (for example ntn-constellation's NtnSatLinkErrorModel on the live
 * geometry). It lets a user-plane link follow the up/down state the control
 * plane already keeps for the same feeder or service link (contact windows,
 * acquisition gaps, rain outages, the serving satellite) without drawing
 * random numbers from that control plane: the predicate only reads state.
 * Drops are counted by cause.
 */

#ifndef ORAN_NTN_GATED_ERROR_MODEL_H
#define ORAN_NTN_GATED_ERROR_MODEL_H

#include "ns3/error-model.h"
#include "ns3/packet.h"
#include "ns3/ptr.h"

#include <cstdint>
#include <functional>

namespace ns3
{

/**
 * \ingroup oran-ntn
 * \brief Drops while a gate predicate holds, else applies an inner model.
 */
class OranNtnGatedErrorModel : public ErrorModel
{
  public:
    static TypeId GetTypeId();
    OranNtnGatedErrorModel();

    /// Gate: true means the link is down and the packet is dropped.
    void SetGate(std::function<bool()> down) { m_down = std::move(down); }
    /// Inner model applied while the gate is open (null: no further loss).
    void SetInner(Ptr<ErrorModel> inner) { m_inner = inner; }
    /// Gate that mirrors a control-plane RateErrorModel used as an on/off
    /// switch: down while its rate is 1.
    static std::function<bool()> MirrorOutage(Ptr<RateErrorModel> control);

    uint64_t GetGatedDrops() const { return m_gated; }
    uint64_t GetInnerDrops() const { return m_innerDrops; }
    uint64_t GetPassed() const { return m_passed; }

  private:
    bool DoCorrupt(Ptr<Packet> p) override;
    void DoReset() override;

    std::function<bool()> m_down;
    Ptr<ErrorModel> m_inner;
    uint64_t m_gated{0};
    uint64_t m_innerDrops{0};
    uint64_t m_passed{0};
};

} // namespace ns3

#endif // ORAN_NTN_GATED_ERROR_MODEL_H
