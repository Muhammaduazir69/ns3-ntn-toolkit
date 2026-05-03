/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_RRC_HELPER_H
#define NTN_RRC_HELPER_H

#include "ns3/ntn-rrc-types.h"
#include "ns3/ntn-timing-advance.h"

#include <ns3/object-factory.h>
#include <ns3/ptr.h>

namespace ns3
{

class MobilityModel;

namespace ntnrrc
{

/**
 * \ingroup ntn-rrc
 *
 * Helper that wires NtnTimingAdvance (and, in later commits, SIB19 / NTN-DRX
 * / UE location reporting) onto a UE/satellite pair.
 */
class NtnRrcHelper
{
  public:
    NtnRrcHelper();

    void SetPayloadMode(PayloadMode mode);
    void SetReferencePosition(const Vector& earthFixedRefPosition);

    /// Build a NtnTimingAdvance instance bound to the given UE and satellite
    /// mobility models. The returned object retains TraceSource access.
    Ptr<NtnTimingAdvance> InstallTimingAdvance(Ptr<MobilityModel> ueMob,
                                               Ptr<MobilityModel> satMob) const;

  private:
    PayloadMode m_payloadMode{PayloadMode::Transparent};
    Vector m_referencePos{0.0, 0.0, 0.0};
};

} // namespace ntnrrc
} // namespace ns3

#endif // NTN_RRC_HELPER_H
