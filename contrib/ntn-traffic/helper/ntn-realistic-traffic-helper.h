// SPDX-License-Identifier: GPL-2.0-only
// NtnRealisticTrafficHelper — drops a real ns-3 traffic plane into any
// scenario so that Simulator::Run() actually exercises packets, schedulers,
// and the protocol stack instead of returning instantly.
//
// Each example calls Install(...) once and gets:
//   - one "remote host" node (PacketSink, simulates the data center / CN)
//   - one "feeder gateway" node (PointToPoint to remote host)
//   - per-UE PointToPoint links (gateway <-> UE)
//   - InternetStack everywhere, /16 + /24 IPv4 addressing
//   - one OnOff app per UE, ramped traffic profile
//   - a periodic SimHealth heartbeat that counts events / packets
//   - end-of-run sim_health.csv that asserts realism gates (Phase 4 in
//     SIMULATION_REALITY_FIX_PLAN.md)
//
// Usage (in any example):
//   NtnRealisticTrafficHelper traffic;
//   traffic.SetSimTime(Seconds(simTime));
//   traffic.SetOutputDir(outputDir);
//   traffic.SetProfile(NtnRealisticTrafficHelper::TrafficProfile::MixedBouquet);
//   NodeContainer ues = traffic.InstallUes(numUes);
//   traffic.Wire();             // installs stack + apps + periodic gates
//   Simulator::Stop(Seconds(simTime));
//   Simulator::Run();
//   traffic.WriteHealthReport(); // dumps sim_health.csv + asserts gates
//   Simulator::Destroy();

#ifndef NTN_REALISTIC_TRAFFIC_HELPER_H
#define NTN_REALISTIC_TRAFFIC_HELPER_H

#include "ns3/application-container.h"
#include "ns3/ipv4-address.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/ptr.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ns3
{

/**
 * \brief Installs a realistic packet-driven traffic plane on top of any
 *        analytical ns-3 NTN scenario. Emits sim_health.csv at end of run.
 *
 * The helper intentionally lives in the `ntn-traffic` module so that every
 * other NTN-specific module (ntn-cho, oran-ntn, thz-ntn, ntn-rrc, sagin,
 * slice, v2x, observability, sionna) can pull it in via a single
 * include without circular dependencies.
 */
class NtnRealisticTrafficHelper
{
  public:
    /// Pre-canned traffic mixes. Each profile spreads packet sizes,
    /// intervals, and on/off duty cycles to keep the event queue busy
    /// even with a small UE count.
    enum class TrafficProfile : uint8_t
    {
        NbIotPeriodic,        ///< 128 B every 1 s — IoT telemetry
        EmbbStreaming,        ///< 1400 B at 2 Mbps — video / file
        UrllcPings,           ///< 256 B every 10 ms — control plane
        DigitalTwinTelemetry, ///< 512 B every 100 ms — DT synchronization
        MixedBouquet,         ///< 1/3 NB-IoT + 1/3 eMBB + 1/3 URLLC
    };

    /// Minimum realism the helper enforces at end of run. If any floor is
    /// missed and `m_strictGates` is true, the run aborts with
    /// NS_FATAL_ERROR; otherwise it just records the failure in the CSV.
    /// Floors scale linearly with sim time.  The defaults are calibrated
    /// so a 60 s run with ≥6 UEs and the MixedBouquet profile passes.
    struct HealthGates
    {
        double minWallClockPerSec = 0.015;   ///< wall sec per 1 s of sim
        uint64_t minPacketsPerSimSec = 100;  ///< total tx/sec (across all UEs)
        double minRxOverTxRatio = 0.85;      ///< delivery floor
        uint64_t minAnalyticalCallbacks = 0; ///< example-specific ticks
    };

    NtnRealisticTrafficHelper();
    ~NtnRealisticTrafficHelper() = default;

    // ---- Configuration --------------------------------------------------
    void SetSimTime(Time t) { m_simTime = t; }
    void SetOutputDir(std::string dir) { m_outputDir = std::move(dir); }
    void SetRunTag(std::string tag) { m_runTag = std::move(tag); }
    void SetProfile(TrafficProfile p) { m_profile = p; }
    void SetUeBackboneRate(std::string r) { m_ueRate = std::move(r); }
    void SetGatewayRate(std::string r) { m_gwRate = std::move(r); }
    void SetGates(HealthGates g) { m_gates = g; }
    void SetStrictGates(bool s) { m_strictGates = s; }
    void SetHeartbeatPeriod(Time t) { m_heartbeat = t; }
    void SetServerPort(uint16_t p) { m_port = p; }

    // ---- Node setup -----------------------------------------------------
    /**
     * \brief Create `numUes` UE nodes wired through a gateway to a remote
     *        host. Returns the UE container.
     */
    NodeContainer InstallUes(uint32_t numUes);

    /// Caller-supplied UE container variant. Used when the example already
    /// owns its node set (e.g. nodes carrying a custom MobilityModel).
    void AttachExistingUes(NodeContainer ues);

    /**
     * \brief Wire the protocol stack, links, addressing, and traffic apps.
     *        Call after configuration setters and before Simulator::Run().
     */
    void Wire();

    /**
     * \brief Schedule a user callback periodically. Use this to register
     *        the example's analytical step (CHO eval, KPM emit, beam-hop
     *        decision) so they fire on the real ns-3 event queue.
     */
    void RegisterPeriodicCallback(Time period, std::function<void(Time)> cb);

    // ---- End-of-run reporting -------------------------------------------
    /**
     * \brief Write sim_health.csv, print a one-line summary, and (if
     *        strict gates are on) NS_FATAL_ERROR if any floor was missed.
     */
    void WriteHealthReport();

    // ---- Public counters (xApps / analytical modules can increment) -----
    void NotePacketTx() { ++m_txPackets; }
    void NotePacketRx() { ++m_rxPackets; }
    void NoteAnalyticalEvent() { ++m_analyticalEvents; }

    uint64_t GetTxPackets() const { return m_txPackets; }
    uint64_t GetRxPackets() const { return m_rxPackets; }

  private:
    // Internal scheduling helpers
    void RunPeriodic(uint32_t cbIdx);
    void RunHeartbeat();
    void TraceTx(Ptr<const Packet>);
    void TraceRx(Ptr<const Packet>, const Address&);

    // Configuration
    Time m_simTime{Seconds(60.0)};
    std::string m_outputDir{"."};
    std::string m_runTag{"run"};
    TrafficProfile m_profile{TrafficProfile::MixedBouquet};
    std::string m_ueRate{"2Mbps"};
    std::string m_gwRate{"1Gbps"};
    HealthGates m_gates{};
    bool m_strictGates{false};
    Time m_heartbeat{Seconds(1.0)};
    uint16_t m_port{49000};

    // Internal containers
    NodeContainer m_ues;
    NodeContainer m_gateway;
    NodeContainer m_remoteHost;
    NetDeviceContainer m_gwToUeDevices;
    NetDeviceContainer m_gwToRemoteDevices;
    ApplicationContainer m_clientApps;
    ApplicationContainer m_serverApps;

    // Per-period callbacks the example registers
    struct PeriodicEntry
    {
        Time period;
        std::function<void(Time)> cb;
    };
    std::vector<PeriodicEntry> m_periodics;

    // Counters
    uint64_t m_txPackets{0};
    uint64_t m_rxPackets{0};
    uint64_t m_analyticalEvents{0};
    uint64_t m_heartbeatTicks{0};

    // Wall-clock timing
    int64_t m_wallStartNs{0};
    bool m_wired{false};
};

} // namespace ns3

#endif
