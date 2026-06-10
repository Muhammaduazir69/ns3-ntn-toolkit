/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// NtnFhSplitModel — dynamic fronthaul functional split (Deng 2026 Sec. IV-B
// enabler 1; adoption plan WS4). Encodes the 3GPP TR 38.801 / O-RAN WG4
// split options the paper discusses for NTN, with their one-way latency
// requirements and fronthaul data-rate multipliers (relative to the user-
// plane rate):
//
//   option   latency bound    FH rate vs user rate     functions on the RU side
//   Opt 2    1.5 - 10 ms      ~1.05x (PDCP PDUs)       RLC/MAC/PHY/RF
//   Opt 7.2a <= 0.25 ms       ~10x (freq-domain IQ)    low-PHY + RF
//   Opt 7.2b <= 0.25 ms       ~7x (beamformed IQ)      low-PHY(+BF) + RF
//   Opt 8    <= 0.25 ms       ~16x (time-domain IQ)    RF only
//
// IsFeasible() checks a split against the MEASURED feeder one-way delay and
// the available FH capacity — at LEO slant the 2.1-18 ms NTN fronthaul rules
// out every lower-PHY split, exactly the paper's argument for regenerative
// payloads. ChooseBestSplit() returns the most centralized feasible option
// (the dynamic-split decision the SMO executes); the GNN+DRL refinement the
// paper proposes plugs in via the existing gym environments.

#ifndef ORAN_NTN_FH_SPLIT_H
#define ORAN_NTN_FH_SPLIT_H

#include <ns3/nstime.h>
#include <ns3/object.h>

#include <cstdint>
#include <string>
#include <vector>

namespace ns3
{

class NtnFhSplitModel : public Object
{
  public:
    enum class Split : uint8_t
    {
        Opt2 = 0,
        Opt7_2a = 1,
        Opt7_2b = 2,
        Opt8 = 3,
    };

    struct SplitSpec
    {
        Split split;
        std::string name;
        Time maxOneWayLatency;
        double fhRateMultiplier; ///< FH bit rate / user-plane bit rate
    };

    static TypeId GetTypeId();
    NtnFhSplitModel() = default;

    static const SplitSpec& GetSpec(Split s);
    static const std::vector<SplitSpec>& GetAllSpecs();

    /// Feasibility against MEASURED transport characteristics.
    static bool IsFeasible(Split s,
                           Time measuredFhOneWay,
                           double userPlaneMbps,
                           double fhCapacityMbps);

    /**
     * \brief Most centralized feasible split (Opt8 > 7.2b > 7.2a > 2 in
     *        centralization order) for the measured FH delay/capacity.
     *        Falls back to Opt2 when nothing lower is feasible (the NTN
     *        regenerative-payload case). \p ok reports whether even Opt2 fit.
     */
    static Split ChooseBestSplit(Time measuredFhOneWay,
                                 double userPlaneMbps,
                                 double fhCapacityMbps,
                                 bool& ok);
};

} // namespace ns3

#endif // ORAN_NTN_FH_SPLIT_H
