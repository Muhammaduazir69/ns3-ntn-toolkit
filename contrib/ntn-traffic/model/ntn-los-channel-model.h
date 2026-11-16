// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Muhammad Uzair
//
// NtnLosChannelModel: a deterministic line-of-sight MatrixBasedChannelModel.
//
// One plane wave between the two arrays, built exactly like the LOS ray of
// TR 38.901 eq. 7.5-29 (the ns-3 ThreeGppChannelModel LOS term): element field
// patterns of both arrays, the array phase of every element along the LOS
// direction, and the distance phase. No clusters, no delay spread, no K-factor
// draw. It is what TR 38.821 assumes for inter-satellite links (free space)
// and what a closed-form link budget assumes, so a measured SINR can be
// checked against that budget term by term. Plugged into the stock
// ThreeGppSpectrumPropagationLossModel ("ChannelModel" attribute), so array
// gain, beamforming vectors, the spectrum channel matrix and the nr MIMO
// chunk processors work unchanged.

#ifndef NTN_LOS_CHANNEL_MODEL_H
#define NTN_LOS_CHANNEL_MODEL_H

#include "ns3/matrix-based-channel-model.h"
#include "ns3/nstime.h"

#include <unordered_map>

namespace ns3
{

/**
 * \ingroup ntn-traffic
 * \brief Deterministic single-ray LOS channel (no small-scale fading).
 */
class NtnLosChannelModel : public MatrixBasedChannelModel
{
  public:
    static TypeId GetTypeId();
    NtnLosChannelModel();
    ~NtnLosChannelModel() override;

    Ptr<const ChannelMatrix> GetChannel(Ptr<const MobilityModel> aMob,
                                        Ptr<const MobilityModel> bMob,
                                        Ptr<const PhasedArrayModel> aAntenna,
                                        Ptr<const PhasedArrayModel> bAntenna) override;
    Ptr<const ChannelParams> GetParams(Ptr<const MobilityModel> aMob,
                                       Ptr<const MobilityModel> bMob) const override;

  private:
    struct Entry
    {
        Ptr<ChannelMatrix> matrix;
        Ptr<ChannelParams> params;
    };
    double m_frequency{2.0e9};
    Time m_updatePeriod{MilliSeconds(100)};
    std::unordered_map<uint64_t, Entry> m_cache;
};

} // namespace ns3

#endif // NTN_LOS_CHANNEL_MODEL_H
