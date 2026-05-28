/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

// thz-ntn-isac-scheduler — Roadmap §4.3.5 joint comm + sense allocator.
//
// Wraps the existing `ThzNtnMacScheduler` and partitions the resource
// grid into communication / sensing / shared sub-bands per the configured
// `IsacMode`. Six modes are honoured:
//
//   COMMUNICATION_ONLY     comm slice = 100 % subbands; sensing = 0
//   COMMUNICATION_CENTRIC  comm 80 % ; sensing 20 %
//   JOINT_ISAC             comm 50 % ; sensing 50 % (split per Set*)
//   SENSING_CENTRIC        comm 20 % ; sensing 80 %
//   SENSING_ONLY           comm 0 ; sensing 100 %
//
// Plus an optional `JOINT_WAVEFORM` overlay where a chosen number of
// subbands carry both data and sensing pilots simultaneously (the
// resource is counted toward both partitions). The waveform overlay is
// the more advanced setting and is opt-in via SetJointSubbands.
//
// The sensing-side companion `ThzNtnIsac` object is consulted for the
// link budget side (range, velocity resolution) so the scheduler can
// avoid allocating subbands too narrow to meet the sensing resolution
// requirement.

#ifndef THZ_NTN_ISAC_SCHEDULER_H
#define THZ_NTN_ISAC_SCHEDULER_H

#include "thz-ntn-isac.h"
#include "thz-ntn-mac-scheduler.h"

#include <ns3/object.h>

#include <cstdint>
#include <vector>

namespace ns3
{

/**
 * \ingroup thz-ntn
 *
 * \brief Joint comm + sense resource scheduler.
 *
 * Output struct extends the existing per-UE decisions with explicit
 * sensing + shared subband lists and the resulting time-share ratio.
 */
struct ThzNtnIsacSchedulingDecision
{
    std::vector<ThzNtnSchedulingDecision> commDecisions;
    std::vector<uint32_t> commSubBandIndices;      //!< all comm-only subbands
    std::vector<uint32_t> sensingSubBandIndices;   //!< all sensing-only subbands
    std::vector<uint32_t> sharedSubBandIndices;    //!< joint comm+sense subbands
    IsacMode mode{IsacMode::JOINT_ISAC};
    double sensingTimeShareRatio{0.0};             //!< [0,1] sensing's fraction
    uint32_t numSubBandsTotal{0};                  //!< for accounting / tests
};

class ThzNtnIsacScheduler : public Object
{
  public:
    static TypeId GetTypeId();

    ThzNtnIsacScheduler();
    ~ThzNtnIsacScheduler() override;

    void SetIsac(Ptr<ThzNtnIsac> isac) { m_isac = isac; }
    Ptr<ThzNtnIsac> GetIsac() const { return m_isac; }

    void SetCommScheduler(Ptr<ThzNtnMacScheduler> sched) { m_comm = sched; }
    Ptr<ThzNtnMacScheduler> GetCommScheduler() const { return m_comm; }

    /// Overlay J subbands carry both comm and sensing pilots. These
    /// subbands appear in both `commSubBandIndices` and
    /// `sharedSubBandIndices`. The default 0 means no overlay (strict TDM).
    void SetJointSubbands(uint32_t j) { m_jointSubbands = j; }
    uint32_t GetJointSubbands() const { return m_jointSubbands; }

    /// Run one scheduling decision for the current TTI.
    ThzNtnIsacSchedulingDecision Schedule(
        const std::vector<ThzNtnUeContext>& ues,
        const std::vector<ThzNtnSubBand>& availableSubBands);

    /// Time-share ratio for sensing given the current ISAC mode. Public so
    /// tests/examples can audit the partition independently of Schedule().
    static double ResolveSensingRatio(IsacMode mode);

  private:
    /// Pick `count` sub-bands from the head of `pool`. Returns the slice
    /// and removes the picked entries from `pool` (passed by reference).
    static std::vector<uint32_t> PickFront(
        std::vector<ThzNtnSubBand>& pool,
        uint32_t count);

    Ptr<ThzNtnIsac> m_isac;
    Ptr<ThzNtnMacScheduler> m_comm;
    uint32_t m_jointSubbands{0};
};

} // namespace ns3

#endif // THZ_NTN_ISAC_SCHEDULER_H
