// SPDX-License-Identifier: GPL-2.0-only
//
// NtnNrStackHelper — the 5G-LENA (`nr`) NTN radio spine. This is the
// ALTERNATIVE spine that closes architectural boundary A5(i): a real NR data
// plane running with FR1 numerology (15 kHz / 30 kHz SCS) at an S-band
// carrier (default 2.0 GHz, 20 MHz — the legal NTN-FR1 BW ceiling), which the
// existing mmwave-based NtnRealStackHelper CANNOT produce (mmwave is locked to
// FR2 numerologies, 60/120 kHz SCS). It lives ALONGSIDE NtnRealStackHelper and
// mirrors its core public surface (SetSimTime/SetOutputDir/SetCarrier.../Build/
// InstallTraffic/Collect/GetMeanDlSinrDb/GetMeanDlTbler/GetRxThroughputMbps/
// GetNumUes) so a module can swap spines without touching its own logic.
//
// What it installs (5G-LENA stack, cttc-nr-demo recipe):
//   * a single operational band -> single CC -> single FR1 BWP at m_freqHz/m_bwHz,
//     built with CcBwpCreator::SimpleOperationBandConf ->
//     CreateOperationBandContiguousCc -> NrHelper::InitializeOperationBand ->
//     CcBwpCreator::GetAllBwps;
//   * NrPointToPointEpcHelper core + IdealBeamformingHelper (DirectPath);
//   * gNB (4x8 UPA) and UE (1x2 UPA) NR devices, shadowing disabled for a clean
//     first link; the gNB PHY Numerology is set to m_numerology (0 = 15 kHz,
//     1 = 30 kHz FR1) and TxPower to m_satEirpDbm (the satellite EIRP scalar);
//     the UE PHY TxPower to m_ueTxDbm;
//   * a remote host wired to the PGW over a P2P backhaul (delay = m_backhaulDelay)
//     with static routing, exactly like the demo; UEs AttachToClosestGnb.
//
// Every headline KPI is MEASURED, not closed-form:
//   * mean DL SINR (dB) and DL TBLER come from the nr SpectrumPhy
//     "RxPacketTraceUe" trace (struct ns3::RxPacketTraceParams — the same
//     m_sinr / m_tbler / m_cellId / m_rnti / m_tbSize family as mmwave), summed
//     in DlRxTrace() over the live run;
//   * RX throughput (Mbps) comes from a FlowMonitor over the real UDP DL flows.
//
// Per-5QI dedicated EPS bearers are available through nr (NrEpsBearer +
// NrEpcTft) for future network-slice actuation (boundary A4); this first cut
// installs a single saturating best-effort DL flow per UE. The TR 38.811
// large-scale channel chaining onto the nr BWP SpectrumChannel and the Rel-17
// DRX / K_offset NTN timing binding are deliberate FOLLOW-ONS (not in this
// keystone) — here the channel is the in-tree 3GPP spatial model with
// shadowing disabled, which already exercises the full FR1 PHY/MAC/HARQ/AMC/
// RLC/PDCP/RRC/EPC chain end to end.
//
// Typical use:
//   NtnNrStackHelper nr;
//   nr.SetSimTime(Seconds(simTime));
//   nr.SetOutputDir(outputDir);
//   nr.SetNumerology(1);                 // 30 kHz FR1
//   nr.SetCarrierFrequencyHz(2.0e9);     // S-band
//   nr.SetBandwidthHz(20e6);             // NTN-FR1 max
//   nr.Build(gnbNodes /*sat, MobilityModel*/, ueNodes /*ground, MobilityModel*/);
//   nr.InstallTraffic(Seconds(0.4), Seconds(simTime));
//   Simulator::Stop(Seconds(simTime));
//   Simulator::Run();
//   nr.Collect();                        // pull measured KPIs
//   Simulator::Destroy();

#ifndef NTN_NR_STACK_HELPER_H
#define NTN_NR_STACK_HELPER_H

#include "ns3/application-container.h"
#include "ns3/ipv4-address.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/nstime.h"
#include "ns3/ptr.h"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ns3
{

class Node;
class NrHelper;
class NrPointToPointEpcHelper;
class IdealBeamformingHelper;
class FlowMonitor;
class FlowMonitorHelper;
struct RxPacketTraceParams;

/**
 * \brief Installs a real 5G-LENA (nr) FR1 NTN air interface between caller-
 *        supplied satellite (gNB) and ground (UE) nodes and measures KPIs from
 *        the live data plane. The FR1 counterpart of NtnRealStackHelper.
 *        See file header for rationale (closes boundary A5(i)).
 */
class NtnNrStackHelper
{
  public:
    NtnNrStackHelper();
    ~NtnNrStackHelper();

    // ---- Configuration (call before Build) ------------------------------
    void SetSimTime(Time t) { m_simTime = t; }
    void SetOutputDir(std::string d) { m_outputDir = std::move(d); }
    /// Optional tag appended to output filenames (mirror of NtnRealStackHelper).
    void SetRunTag(std::string t) { m_runTag = std::move(t); }
    /// S-band NTN carrier. Default 2.0 GHz.
    void SetCarrierFrequencyHz(double f) { m_freqHz = f; }
    double GetCarrierFrequencyHz() const { return m_freqHz; }
    /// Channel bandwidth. Default 20 MHz (the legal NTN-FR1 maximum).
    void SetBandwidthHz(double b) { m_bwHz = b; }
    double GetBandwidthHz() const { return m_bwHz; }
    /// FR1 numerology: 0 = 15 kHz SCS, 1 = 30 kHz SCS (default). Values > 1 are
    /// accepted by nr but leave the NTN-FR1 SCS set.
    void SetNumerology(uint16_t n) { m_numerology = n; }
    uint16_t GetNumerology() const { return m_numerology; }
    /// SCS in kHz for the configured numerology (15 * 2^numerology).
    double GetScsKhz() const { return 15.0 * (1u << m_numerology); }
    /// Satellite (gNB) EIRP scalar written verbatim into NrGnbPhy::TxPower (dBm).
    void SetSatEirpDbm(double p) { m_satEirpDbm = p; }
    double GetSatEirpDbm() const { return m_satEirpDbm; }
    void SetUeTxPowerDbm(double p) { m_ueTxDbm = p; }
    /// Feeder + core one-way delay on the PGW<->remote-host backhaul.
    void SetBackhaulDelay(Time t) { m_backhaulDelay = t; }

    // ---- Build the real radio stack -------------------------------------
    /**
     * \brief Wire nr gNB devices on \p gnbNodes and UE devices on \p ueNodes,
     *        attach UEs to the closest gNB, stand up the EPC + remote host, and
     *        connect the measured-KPI PHY sink (RxPacketTraceUe). Both node sets
     *        MUST already carry a MobilityModel.
     */
    void Build(NodeContainer gnbNodes, NodeContainer ueNodes);

    /// Install a saturating UDP DL flow remote-host -> each UE plus a
    /// FlowMonitor to measure throughput.
    void InstallTraffic(Time start, Time stop);

    // ---- Post-run measurement (call after Simulator::Run) ----------------
    /// Aggregate FlowMonitor throughput + the PHY-sink SINR/TBLER samples.
    void Collect();

    // ---- Measured KPI accessors -----------------------------------------
    // SINR/TBLER return the LIVE running mean while the sim is running (so a
    // CHO/RIC tick reads a real measured serving SINR), and the same value
    // after Collect(). 0 until the first transport block is decoded.
    double GetMeanDlSinrDb() const
    {
        return m_dlGlobal.n > 0 ? m_dlGlobal.sumSinrDb / m_dlGlobal.n : m_dlSinrDbMean;
    }
    double GetMeanDlTbler() const
    {
        return m_dlGlobal.n > 0 ? m_dlGlobal.sumTbler / m_dlGlobal.n : m_dlTblerMean;
    }
    double GetRxThroughputMbps() const { return m_rxThroughputMbps; }
    uint64_t GetPhyRxTb() const { return m_phyRxTb; }
    uint32_t GetNumUes() const { return m_ue.GetN(); }

    // ---- Handles for module-specific wiring ------------------------------
    NetDeviceContainer GetUeDevices() const { return m_ueDevs; }
    NetDeviceContainer GetGnbDevices() const { return m_gnbDevs; }
    Ptr<Node> GetRemoteHost() const { return m_remoteHost; }

  private:
    // PHY measured-KPI sink (connected to RxPacketTraceUe on each UE SpectrumPhy).
    void DlRxTrace(RxPacketTraceParams params);

    struct SinrAccum
    {
        double sumSinrDb = 0.0;
        double sumTbler = 0.0;
        uint64_t n = 0;
    };

    // Config
    Time m_simTime{Seconds(2.0)};
    std::string m_outputDir{"."};
    std::string m_runTag{"run"};
    double m_freqHz{2.0e9};     ///< S-band carrier
    double m_bwHz{20.0e6};      ///< NTN-FR1 max bandwidth
    uint16_t m_numerology{1};   ///< 0 = 15 kHz, 1 = 30 kHz FR1
    double m_satEirpDbm{40.0};  ///< gNB TxPower / satellite EIRP scalar
    double m_ueTxDbm{23.0};     ///< UE TxPower
    Time m_backhaulDelay{MilliSeconds(5)};

    // ns-3 objects
    Ptr<NrHelper> m_nr;
    Ptr<NrPointToPointEpcHelper> m_epc;
    Ptr<IdealBeamformingHelper> m_beamforming;
    NodeContainer m_gnb;
    NodeContainer m_ue;
    NetDeviceContainer m_gnbDevs;
    NetDeviceContainer m_ueDevs;
    Ptr<Node> m_remoteHost;
    std::vector<Ipv4Address> m_ueAddrs;
    ApplicationContainer m_clientApps;
    ApplicationContainer m_serverApps;
    Ptr<FlowMonitor> m_monitor;
    FlowMonitorHelper* m_flowmonHelper{nullptr};

    // Measured-KPI sink state
    SinrAccum m_dlGlobal;

    // Collected results
    double m_dlSinrDbMean{0.0};
    double m_dlTblerMean{0.0};
    uint64_t m_phyRxTb{0};
    double m_rxThroughputMbps{0.0};
    Time m_trafficStart{Seconds(0)};
    Time m_trafficStop{Seconds(0)};

    bool m_built{false};
};

} // namespace ns3

#endif // NTN_NR_STACK_HELPER_H
