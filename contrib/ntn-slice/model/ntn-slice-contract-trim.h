/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * Delivered-rate trim of a tenant's slice quota: the quota is scaled by the ratio of the bytes the tenant should have
 * received in a control period to those it did, so that RBGs it wins beyond its quota do not add to its rate.
 */
#ifndef NTN_SLICE_CONTRACT_TRIM_H
#define NTN_SLICE_CONTRACT_TRIM_H

#include <algorithm>
#include <cmath>

namespace ns3
{
namespace ntnslice
{

/**
 * \param quota the current quota, as a share of the RBGs of a slot
 * \param targetBytes the transport-block bytes the tenant should receive per control period
 * \param deliveredBytes the transport-block bytes it did receive; must be positive
 * \param gain exponent of the correction, in (0, 1]
 * \param lowest smallest quota allowed
 * \param highest largest quota allowed
 * \return the next quota
 */
inline double
TrimmedQuota(double quota, double targetBytes, double deliveredBytes, double gain, double lowest, double highest)
{
    return std::min(highest, std::max(lowest, quota * std::pow(targetBytes / deliveredBytes, gain)));
}

} // namespace ntnslice
} // namespace ns3

#endif /* NTN_SLICE_CONTRACT_TRIM_H */
