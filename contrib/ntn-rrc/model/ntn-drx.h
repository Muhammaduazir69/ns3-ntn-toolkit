/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_DRX_H
#define NTN_DRX_H

#include <ns3/event-id.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/traced-callback.h>

#include <array>
#include <cstdint>

namespace ns3
{
namespace ntnrrc
{

/// DRX state (TS 38.321 §5.7 + NTN extensions per TR 38.821 §6.3.4).
enum class DrxState : uint8_t
{
    Active = 0,        //!< monitoring PDCCH each slot
    OnDuration = 1,    //!< inside drx-onDurationTimer
    ShortSleep = 2,    //!< inside drx-ShortCycle
    LongSleep = 3,     //!< inside drx-LongCycle
    AwaitingPass = 4,  //!< NTN: deep sleep until next satellite visibility window
};

/// DRX cycle parameters (TS 38.321 §5.7 + NTN augmentations).
struct NtnDrxConfig
{
    Time longCycle{MilliSeconds(320)};     //!< drx-LongCycle
    Time shortCycle{MilliSeconds(20)};     //!< drx-ShortCycle (0 disables)
    Time onDuration{MilliSeconds(5)};      //!< drx-onDurationTimer
    Time inactivityTimer{MilliSeconds(10)};//!< drx-InactivityTimer
    uint32_t shortCycleCount{2};           //!< drx-ShortCycleTimer (in multiples of drx-ShortCycle, TS 38.321 5.7)

    /// NTN: when true, allow the state machine to enter `AwaitingPass` between
    /// the current pass-end and the next pass-start.
    bool passAware{false};
    Time passDuration{Seconds(600)};       //!< how long a pass typically is

    // ---- NTN HARQ round-trip timers (TS 38.321 §5.7) ----
    // Baseline NR drx-HARQ-RTT-TimerDL/UL: the minimum gap the UE waits, after
    // a DL/UL transmission, before it need monitor PDCCH for the corresponding
    // retransmission. In NTN this must be EXTENDED by the latest UE-gNB
    // round-trip time (TS 38.321 §5.7 defines HARQ-RTT-TimerDL-NTN /
    // HARQ-RTT-TimerUL-NTN = drx-HARQ-RTT-TimerDL/UL + UE-gNB RTT), because the
    // retransmission cannot arrive sooner than one RTT after the (N)ACK. Feed
    // `ntnRtt` from NtnTimingAdvance::ComputeTotalTa() (2*slant/c). The
    // NTN-extended values are exposed via GetHarqRttTimerDl/UlNtn().
    Time harqRttTimerDl{MilliSeconds(0)};  //!< drx-HARQ-RTT-TimerDL (baseline NR)
    Time harqRttTimerUl{MilliSeconds(0)};  //!< drx-HARQ-RTT-TimerUL (baseline NR)
    Time ntnRtt{MilliSeconds(0)};          //!< latest UE-gNB RTT (TS 38.321 §5.7)

    // ---- Wave 6A: the timers above now drive Active Time ----
    /// drx-RetransmissionTimerDL / UL: started when the HARQ RTT timer of a
    /// process expires and its data was not decoded (DL) / always (UL); while
    /// it runs the UE is in Active Time. Zero disables the retransmission
    /// window (the UE then never wakes for a HARQ retransmission).
    Time retransmissionTimerDl{MilliSeconds(0)};
    Time retransmissionTimerUl{MilliSeconds(0)};
    /// downlinkHARQ-FeedbackDisabled-r17 (TS 38.331), bit i = DL HARQ process
    /// i has feedback disabled. For such a process the UE does not wait the
    /// UE-gNB RTT: drx-HARQ-RTT-TimerDL (no RTT extension) starts at the end
    /// of the PDSCH instead of after the HARQ feedback.
    uint32_t dlHarqFeedbackDisabled{0};
    /// uplinkHARQ-Mode-r17 (TS 38.331), bit i = UL HARQ process i in mode B.
    /// HARQ-RTT-TimerUL-NTN (with the RTT) applies to mode A only.
    uint32_t ulHarqModeB{0};

    /// True when `longCycle >= shortCycle >= onDuration > 0`.
    bool IsValid() const;
};

/**
 * \ingroup ntn-rrc
 *
 * NTN-aware DRX state machine.
 *
 * Drives the canonical NR DRX cycles (TS 38.321) plus an NTN-specific deep
 * sleep state (`AwaitingPass`). External code:
 *   1. Calls `Start()` to begin ticking.
 *   2. Calls `NotifyDataActivity()` whenever there is downlink/uplink data,
 *      which restarts the inactivity timer and forces `Active`.
 *   3. (Pass-aware mode) Calls `NotifyNextPass(start, duration)` whenever the
 *      next satellite-visibility window is predicted, allowing the SM to
 *      sleep until then.
 *
 * The state machine reports awake/asleep via `IsAwake()` and emits a
 * `StateChange` trace on every transition.
 */
class NtnDrxStateMachine : public Object
{
  public:
    static TypeId GetTypeId();

    NtnDrxStateMachine();
    ~NtnDrxStateMachine() override;

    void SetConfig(const NtnDrxConfig& cfg);
    const NtnDrxConfig& GetConfig() const;

    /// Notify the state machine that data has arrived; restarts inactivity timer.
    void NotifyDataActivity();
    /// (Pass-aware mode) Update the next satellite visibility window.
    void NotifyNextPass(Time start, Time duration);

    /// Update the UE-gNB round-trip time used to offset the NTN HARQ-RTT
    /// timers (TS 38.321 §5.7). Typically fed from
    /// NtnTimingAdvance::ComputeTotalTa() each time the TA is refreshed.
    void SetNtnRoundTripTime(Time rtt);

    /// NTN-extended DL HARQ round-trip timer (TS 38.321 §5.7):
    /// HARQ-RTT-TimerDL-NTN = drx-HARQ-RTT-TimerDL + UE-gNB RTT. The HARQ
    /// entity uses this (not the bare drx-HARQ-RTT-TimerDL) to decide when a
    /// DL retransmission grant may be monitored.
    Time GetHarqRttTimerDlNtn() const;
    /// NTN-extended UL HARQ round-trip timer (TS 38.321 §5.7):
    /// HARQ-RTT-TimerUL-NTN = drx-HARQ-RTT-TimerUL + UE-gNB RTT.
    Time GetHarqRttTimerUlNtn() const;

    // ---- Wave 6A: HARQ RTT / retransmission timers and SR (TS 38.321 §5.7) ----

    /// The DL HARQ RTT timer this process actually runs: HARQ-RTT-TimerDL-NTN
    /// (drx-HARQ-RTT-TimerDL + UE-gNB RTT) with feedback enabled, the bare
    /// drx-HARQ-RTT-TimerDL with feedback disabled (no RTT wait).
    Time GetHarqRttTimerDlFor(uint8_t harqId) const;
    /// UL counterpart: + UE-gNB RTT for HARQ mode A, bare timer for mode B.
    Time GetHarqRttTimerUlFor(uint8_t harqId) const;

    /// A PDSCH for DL HARQ process \p harqId ended now. \p decoded is the
    /// UE's decoding outcome. With feedback enabled the HARQ RTT timer starts
    /// after the feedback transmission, \p feedbackDelay from now (PDSCH end
    /// to end of the PUCCH carrying the ACK/NACK, i.e. K1 plus K_offset);
    /// with feedback disabled it starts now. A running
    /// drx-RetransmissionTimerDL of the process stops (TS 38.321 §5.7).
    void NotifyDlTransmission(uint8_t harqId, bool decoded, Time feedbackDelay = Time(0));
    /// A PUSCH for UL HARQ process \p harqId ended now.
    void NotifyUlTransmission(uint8_t harqId);
    /// An SR was sent on PUCCH and is pending. In NTN the Active Time for the
    /// pending SR starts one UE-gNB RTT after the SR (TS 38.321 §5.7, Rel-17):
    /// no grant can arrive sooner, so the UE need not monitor PDCCH meanwhile.
    void NotifySrSent();
    /// A UL grant arrived: the pending SR is cancelled (TS 38.321 §5.4.4) and
    /// the PDCCH counts as activity (inactivity timer restarts).
    void NotifyUlGrant();

    bool IsHarqRttTimerDlRunning(uint8_t harqId) const;
    bool IsRetransmissionTimerDlRunning(uint8_t harqId) const;
    bool IsHarqRttTimerUlRunning(uint8_t harqId) const;
    bool IsRetransmissionTimerUlRunning(uint8_t harqId) const;
    /// True while an SR is pending and its RTT offset has elapsed.
    bool IsSrActive() const;

    /// Begin ticking the state machine. Initial state is `Active`.
    void Start();
    void Stop();

    DrxState GetState() const;
    /// DRX Active Time (TS 38.321 §5.7): on-duration or inactivity (the cycle
    /// state), a running drx-RetransmissionTimerDL/UL, or a pending SR past its
    /// RTT offset.
    bool IsAwake() const;

    /// Total simulated time spent in each state since `Start()`.
    Time GetTimeInState(DrxState s) const;

  protected:
    void DoDispose() override;

  private:
    void Transition(DrxState next);
    void TickActive();
    void TickOnDuration();
    void TickShortSleep();
    void TickLongSleep();
    void TickAwaitingPass();
    void AccumulateTimeInState();

    NtnDrxConfig m_cfg;
    DrxState m_state{DrxState::Active};
    Time m_stateEnteredAt{Seconds(0)};
    Time m_inactivityDeadline{Seconds(0)};
    Time m_nextPassStart{Seconds(0)};
    Time m_nextPassEnd{Seconds(0)};
    uint32_t m_shortCycleRemaining{0};
    bool m_running{false};
    EventId m_event;

    Time m_timeInState[5]{};

    // Wave 6A per-process timers (32 processes, TS 38.331 n32).
    static constexpr uint8_t kMaxHarq = 32;
    std::array<Time, kMaxHarq> m_dlHarqRttEnd{};   //!< DL HARQ RTT timer expiry
    std::array<Time, kMaxHarq> m_dlRetxEnd{};      //!< drx-RetransmissionTimerDL expiry
    std::array<Time, kMaxHarq> m_ulHarqRttEnd{};
    std::array<Time, kMaxHarq> m_ulRetxEnd{};
    std::array<EventId, kMaxHarq> m_dlHarqRttEvent;
    std::array<EventId, kMaxHarq> m_ulHarqRttEvent;
    bool m_srPending{false};
    Time m_srSentAt{Seconds(0)};
    void DlHarqRttExpired(uint8_t harqId, bool decoded);
    void UlHarqRttExpired(uint8_t harqId);

    TracedCallback<DrxState, DrxState, Time> m_stateChangeTrace;
};

} // namespace ntnrrc
} // namespace ns3

#endif // NTN_DRX_H
