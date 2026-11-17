/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ThzNtnPacketLink - a measured THz Earth-space packet link (Wave 7C).
//
// The THZ-07 data plane (ThzNtnLinkErrorModel on a point-to-point device)
// carried packets at true THz, but with three fixed quantities: a constant
// propagation delay set once at setup, a constant 10 Gb/s line rate, and an
// SNR threshold that delivers either every packet or none. Over a LEO pass
// none of those is constant. This helper keeps the same propagation chain
// (Friis -> ThzNtnPropagationLossModel: P.676-13 Annex 1 gas, P.840-9 cloud,
// P.838/P.618 rain, P.834 apparent elevation) and adds, per update period:
//   - the point-to-point channel delay set to the live slant range / c, in
//     both directions;
//   - DVB-S2 ACM (ETSI EN 302 307-1 normal FECFRAME MODCOD set): the MODCOD
//     is chosen from the SNR the receiver measured one round trip earlier
//     (the return-channel feedback age), with a margin, and the device line
//     rate is set to R_s * log2(M) * code rate;
//   - per-packet decode from the DVB-S2 BLER(Es/N0) curve of the MODCOD in
//     use (SNS3 link results), so loss rises gradually as the margin erodes;
//   - receiver sky noise from the slant-path atmospheric loss (ITU-R
//     P.618-14 Section 3, T_mr = 275 K) for the ground receiver and a 290 K
//     Earth-viewing antenna for the satellite receiver.
// It is still not a SpectrumPhy: no per-subcarrier processing, no HARQ.
#ifndef THZ_NTN_PACKET_LINK_H
#define THZ_NTN_PACKET_LINK_H

#include "ns3/thz-ntn-link-error-model.h"
#include "ns3/thz-ntn-pointing-loss-model.h"
#include "ns3/thz-ntn-propagation-loss-model.h"
#include "ns3/thz-ntn-scintillation.h"

#include "ns3/mobility-model.h"
#include "ns3/net-device-container.h"
#include "ns3/node.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/satellite-enums.h"

#include <deque>
#include <limits>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace ns3
{

class PointToPointChannel;
class PointToPointNetDevice;

namespace thzntn
{

class ThzNtnPacketLink : public Object
{
  public:
    struct Config
    {
        double freqHz{300e9};
        double txPowerDbm{30.0};    //!< conducted power, both ends
        double spaceGainDbi{55.0};  //!< satellite (or relay) aperture gain
        double groundGainDbi{55.0}; //!< ground terminal aperture gain
        double bandwidthHz{1e9};    //!< occupied channel bandwidth
        double rollOff{0.05};       //!< DVB-S2X roll-off; R_s = B / (1 + alpha)
        double noiseFigureDb{8.0};
        double rainRateMmH{0.0};
        bool skyNoise{true};        //!< antenna temperature from the path loss
        /// Pointing loss at both apertures (ThzNtnPointingLossModel: vibration,
        /// J2 ephemeris, P.834 refraction and tracking-latency terms with a
        /// per-evaluation Rayleigh jitter sample), beamwidth 70 lambda / D of
        /// the aperture that gives the declared gain at efficiency 0.6.
        bool pointing{true};
        /// Tropospheric amplitude scintillation at the ground aperture
        /// (ThzNtnScintillation, P.618-14 2.4.1 with aperture averaging, AR(1)
        /// in simulated time), sampled per update. P.618 states 4 to 55 GHz;
        /// above that it is an extrapolation and the report counts it.
        bool scintillation{true};
        bool acm{true};             //!< false: fixed MODCOD
        SatEnums::SatModcod_t fixedModcod{SatEnums::SAT_MODCOD_QPSK_1_TO_2};
        double acmMarginDb{1.0};
        double blerTarget{1e-4};    //!< BLER the ACM selection is sized for
        Time updatePeriod{MilliSeconds(100)}; //!< delay, noise and ACM refresh
        bool thresholdDecode{false}; //!< legacy THZ-07 step decode instead of BLER
        /// Step-decode threshold (dB); NaN = the fixed MODCOD's requirement.
        double decodeSnrDb{std::numeric_limits<double>::quiet_NaN()};
    };

    static TypeId GetTypeId();
    ThzNtnPacketLink();
    ~ThzNtnPacketLink() override;

    void SetConfig(const Config& c) { m_cfg = c; }
    const Config& GetConfig() const { return m_cfg; }

    /// Build the point-to-point link between a space node and a ground node.
    /// Device 0 is on the space node, device 1 on the ground node.
    NetDeviceContainer Install(Ptr<Node> space,
                               Ptr<Node> ground,
                               Ptr<MobilityModel> spaceMob,
                               Ptr<MobilityModel> groundMob);

    /// Per-update CSV (time, geometry, loss terms, SNR, MODCOD, rate).
    void EnableTimeSeries(const std::string& path);
    /// Extra loss (dB) added to both directions, e.g. a beam-squint or
    /// pointing term computed by an analysis component. Applied as a gain
    /// reduction at the receive aperture.
    void SetExtraLossDb(double db);
    double GetExtraLossDb() const { return m_extraLossDb; }

    Ptr<ThzNtnPropagationLossModel> GetAtmosphere() const { return m_atm; }
    /// Aperture diameter (m) that gives \p gainDbi at \p freqHz (efficiency 0.6).
    static double ApertureDiameterM(double gainDbi, double freqHz);
    Ptr<ThzNtnLinkErrorModel> GetDownlinkErrorModel() const { return m_emDown; }
    Ptr<ThzNtnLinkErrorModel> GetUplinkErrorModel() const { return m_emUp; }
    double GetSymbolRateHz() const;

    /// Spectral efficiency (bit/symbol) of a DVB-S2 MODCOD: log2(M) * rate.
    static double SpectralEfficiency(SatEnums::SatModcod_t m);
    /// The DVB-S2 normal-frame MODCOD set with link results, ascending by
    /// spectral efficiency.
    static const std::vector<SatEnums::SatModcod_t>& DvbS2ModcodSet();
    /// The ACM decision for an Es/N0 (dB): highest MODCOD whose required
    /// Es/N0 at the BLER target is at most esNo - margin; false if none.
    bool SelectModcod(double esNoDb, SatEnums::SatModcod_t& out) const;

    // ---- live / end-of-run statistics ----
    double GetLastSnrDb() const { return m_lastSnrDown; }
    double GetLastRangeKm() const { return m_lastRangeKm; }
    double GetLastDelayMs() const { return m_lastDelayMs; }
    double GetLastRateMbps() const { return m_lastRateMbps; }
    SatEnums::SatModcod_t GetCurrentModcod() const { return m_modcod; }
    uint64_t GetUpdates() const { return m_updates; }
    uint64_t GetModcodChanges() const { return m_modcodChanges; }
    /// Time (s) with no MODCOD that closes (link held at the lowest MODCOD).
    double GetOutageSeconds() const { return m_outageS; }
    double GetMinDelayMs() const { return m_minDelayMs; }
    double GetMaxDelayMs() const { return m_maxDelayMs; }
    double GetMinSnrDb() const { return m_minSnr; }
    double GetMaxSnrDb() const { return m_maxSnr; }
    double GetMeanRateMbps() const { return m_updates ? m_sumRate / m_updates : 0.0; }
    double GetMinElevationDeg() const { return m_minElev; }
    double GetMaxElevationDeg() const { return m_maxElev; }
    /// One-paragraph end-of-run report (geometry, loss terms, cloud
    /// statistics, ACM, decode counters).
    std::string GetReport() const;

    int64_t AssignStreams(int64_t stream);

  private:
    void Update();
    void ApplyRate(double bps);

    Config m_cfg;
    Ptr<ThzNtnPropagationLossModel> m_atm;
    Ptr<PropagationLossModel> m_chain;
    Ptr<ThzNtnLinkErrorModel> m_emDown; //!< on the ground device (space -> ground)
    Ptr<ThzNtnLinkErrorModel> m_emUp;   //!< on the space device (ground -> space)
    Ptr<MobilityModel> m_spaceMob;
    Ptr<MobilityModel> m_groundMob;
    Ptr<PointToPointChannel> m_channel;
    Ptr<PointToPointNetDevice> m_devSpace;
    Ptr<PointToPointNetDevice> m_devGround;
    double m_extraLossDb{0.0};
    double m_scintDb{0.0};
    double m_sumScintSq{0.0};
    double m_sumPointing{0.0};
    uint64_t m_scintExtrap{0};
    Ptr<ThzNtnPointingLossModel> m_pointSpace;
    Ptr<ThzNtnPointingLossModel> m_pointGround;
    Ptr<ThzNtnScintillation> m_scint;
    void ApplyGains();
    EventId m_event;
    std::deque<std::pair<Time, double>> m_csi; //!< (time, measured Es/N0)
    SatEnums::SatModcod_t m_modcod{SatEnums::SAT_MODCOD_QPSK_1_TO_2};
    bool m_haveModcod{false};
    std::unique_ptr<std::ofstream> m_ts;
    double m_lastSnrDown{0.0};
    double m_lastRangeKm{0.0};
    double m_lastDelayMs{0.0};
    double m_lastRateMbps{0.0};
    uint64_t m_updates{0};
    uint64_t m_modcodChanges{0};
    double m_outageS{0.0};
    double m_minDelayMs{1e300};
    double m_maxDelayMs{0.0};
    double m_minSnr{1e300};
    double m_maxSnr{-1e300};
    double m_minElev{1e300};
    double m_maxElev{-1e300};
    double m_sumRate{0.0};
    double m_sumGas{0.0};
    double m_sumCloud{0.0};
    double m_sumRain{0.0};
};

} // namespace thzntn
} // namespace ns3

#endif // THZ_NTN_PACKET_LINK_H
