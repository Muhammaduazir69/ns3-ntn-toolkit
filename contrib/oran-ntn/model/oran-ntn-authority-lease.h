/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// OranNtnAuthorityLease: placement, lease sizing and reconciliation rules for
// control authority that moves between a ground Near-RT RIC and an on-board
// Space RIC as the feeder link comes and goes.
//
// The three rules are kept free of simulator state so they can be unit tested
// and reused by any scenario:
//
//   Place()        a function class runs on the ground only if the PREDICTED
//                  ground loop time fits its deadline with slack AND the
//                  predicted remaining feeder contact is long enough to be
//                  worth holding authority on the ground;
//   LeaseExpiry()  authority handed to the Space RIC expires at the predicted
//                  feeder restoration plus a guard margin, capped, and is
//                  carried as the validUntil of the A1 policy snapshot that
//                  travels with it;
//   Merge()        at reconnection, reconcile the ground and on-board write
//                  logs of each controlled parameter. GroundWins is the usual
//                  reactive behaviour (the ground controller resumes and
//                  overwrites). LeaseVersioned keeps, for every parameter, the
//                  latest write made by whichever side held the lease at the
//                  time of the write, and flags writes made under a superseded
//                  A1 policy version for recomputation.

#ifndef ORAN_NTN_AUTHORITY_LEASE_H
#define ORAN_NTN_AUTHORITY_LEASE_H

#include <ns3/nstime.h>
#include <ns3/object.h>

#include <cstdint>
#include <map>
#include <vector>

namespace ns3
{

class OranNtnAuthorityLease : public Object
{
  public:
    enum class Host : uint8_t
    {
        Ground = 0,
        Onboard = 1,
    };

    enum class MergeRule : uint8_t
    {
        GroundWins = 0,
        LeaseVersioned = 1,
    };

    /// One write of a controlled parameter by either controller.
    struct Write
    {
        uint32_t paramId{0};
        double value{0.0};
        uint32_t policyVersion{0}; //!< A1 policy version the decision used
        Time at;                   //!< when the write was decided
        Host writer{Host::Ground};
    };

    struct MergeResult
    {
        std::map<uint32_t, double> state;  //!< reconciled value per parameter
        std::map<uint32_t, Host> winner;   //!< which side's write was kept
        uint32_t directConflicts{0};       //!< both sides wrote, values differ
        uint32_t policyStaleWrites{0};     //!< kept writes under an old version
        std::vector<uint32_t> recompute;   //!< parameters to recompute
    };

    static TypeId GetTypeId();
    OranNtnAuthorityLease() = default;

    /**
     * \brief Host for one function class.
     * \param predictedGroundLoop predicted measure-to-actuation time on the
     *        ground path (both feeder legs, backhaul, processing)
     * \param deadline the class deadline
     * \param slackFrac fraction of the deadline kept as slack (0..1)
     * \param remainingContact predicted time until the feeder is lost
     * \param minContact shortest remaining contact for which ground hosting
     *        is chosen
     */
    static Host Place(Time predictedGroundLoop,
                      Time deadline,
                      double slackFrac,
                      Time remainingContact,
                      Time minContact);

    /**
     * \brief Lease horizon from the supersession bound.
     *
     * If A1 policy updates arrive as a Poisson process with mean spacing
     * \p meanUpdate, a snapshot taken at the last contact is still current
     * after a time tau with probability exp(-tau / meanUpdate). The horizon
     * is the largest tau for which the probability that the snapshot has
     * been superseded stays at or below \p eps:
     *     tau* = -meanUpdate * ln(1 - eps),   0 < eps < 1.
     * eps is the ratio of the marginal over-provisioning cost of the safe
     * fallback to the marginal cost of enforcing a superseded intent, so the
     * horizon is where the two marginal costs meet.
     */
    static Time LeaseHorizon(Time meanUpdate, double eps);

    /// Lease expiry: predicted restoration + margin, never later than
    /// now + maxLease, never earlier than now.
    static Time LeaseExpiry(Time now, Time predictedRestore, Time margin, Time maxLease);

    /**
     * \brief Reconcile the two write logs at reconnection.
     * \param leaseStart start of the on-board authority interval
     * \param leaseEnd end of the on-board authority interval (the instant the
     *        ground took authority back)
     * \param currentVersion newest A1 policy version known at merge time
     * \param tol two values closer than this are the same decision
     */
    static MergeResult Merge(MergeRule rule,
                             const std::vector<Write>& ground,
                             const std::vector<Write>& onboard,
                             Time leaseStart,
                             Time leaseEnd,
                             uint32_t currentVersion,
                             double tol);
};

} // namespace ns3

#endif // ORAN_NTN_AUTHORITY_LEASE_H
