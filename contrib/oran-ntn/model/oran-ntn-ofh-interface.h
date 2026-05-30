/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_OFH_INTERFACE_H
#define ORAN_NTN_OFH_INTERFACE_H

// Open Fronthaul (OFH) lite transport between O-DU and O-RU
// (Roadmap §4.1.9).
//
// Real OFH is split into four planes (TS 38.872):
//
//   * C-Plane — slot-level scheduling commands DU → RU
//   * U-Plane — IQ samples in both directions
//   * S-Plane — synchronisation (PTP / IEEE 1588 / SyncE)
//   * M-Plane — NETCONF-driven management
//
// The toolkit's lite transport conveys typed messages per plane with
// configurable plane-specific delays (defaults: C 0.1 ms, U 0.05 ms,
// S 0.001 ms, M 5 ms). Tests use these knobs to verify the DU/RU
// pipeline degrades sensibly on plane-specific outages.

#include "oran-ntn-split-gnb.h"

#include <ns3/event-id.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/ptr.h>
#include <ns3/traced-callback.h>

#include <cstdint>
#include <functional>
#include <vector>

namespace ns3
{

enum class OfhPlane : uint8_t
{
    c_plane = 0,
    u_plane = 1,
    s_plane = 2,
    m_plane = 3,
};

inline const char*
OfhPlaneName(OfhPlane p)
{
    switch (p)
    {
    case OfhPlane::c_plane: return "C-Plane";
    case OfhPlane::u_plane: return "U-Plane";
    case OfhPlane::s_plane: return "S-Plane";
    case OfhPlane::m_plane: return "M-Plane";
    }
    return "?";
}

struct OfhMessage
{
    OfhPlane plane;
    /// Implementation-defined opcode (e.g. C-plane "section type").
    uint16_t opcode{0};
    /// SFN, slot, symbol for C/U-plane scheduling messages.
    uint16_t sfn{0};
    uint8_t slot{0};
    uint8_t symbol{0};
    /// Toolkit-internal: serialised RC ControlAction or RF-retune
    /// command from DU → RU.
    std::vector<uint8_t> payload;
};

/**
 * \ingroup oran-ntn
 * \brief OFH-lite interface between DU and RU.
 */
class OranNtnOfhInterface : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnOfhInterface();
    ~OranNtnOfhInterface() override;

    void Attach(Ptr<OranNtnSplitGnbEntity> du,
                Ptr<OranNtnSplitGnbEntity> ru);

    /// Override per-plane delays (defaults set in constructor).
    void SetPlaneDelay(OfhPlane p, Time t);
    Time GetPlaneDelay(OfhPlane p) const;

    using OfhCallback = std::function<void(const OfhMessage&)>;
    void SetDuReceiveCallback(OfhCallback cb) { m_duRecv = std::move(cb); }
    void SetRuReceiveCallback(OfhCallback cb) { m_ruRecv = std::move(cb); }

    bool SendFromDu(const OfhMessage& msg);
    bool SendFromRu(const OfhMessage& msg);

    /// Force a per-plane outage (e.g. simulate C-Plane fronthaul link
    /// failure). Pending events for that plane are cancelled.
    void SetPlaneUp(OfhPlane p, bool up);
    bool IsPlaneUp(OfhPlane p) const;

    uint64_t MessagesDuToRu() const { return m_duToRu; }
    uint64_t MessagesRuToDu() const { return m_ruToDu; }
    uint64_t MessagesDropped() const { return m_dropped; }
    uint64_t MessagesDroppedPerPlane(OfhPlane p) const;

    TracedCallback<OfhPlane> m_planeUsed;

  protected:
    void DoDispose() override;

  private:
    bool DispatchToRu(OfhMessage msg);
    bool DispatchToDu(OfhMessage msg);

    Ptr<OranNtnSplitGnbEntity> m_du;
    Ptr<OranNtnSplitGnbEntity> m_ru;

    Time m_planeDelays[4];
    bool m_planeUp[4]{true, true, true, true};
    uint64_t m_planeDroppedCounters[4]{0, 0, 0, 0};

    OfhCallback m_duRecv;
    OfhCallback m_ruRecv;

    uint64_t m_duToRu{0};
    uint64_t m_ruToDu{0};
    uint64_t m_dropped{0};

    std::vector<EventId> m_pending;
};

} // namespace ns3

#endif // ORAN_NTN_OFH_INTERFACE_H
