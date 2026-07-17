/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W7)
 *
 * SUMO -> ns-3 mobility bridge (offline FCD-replay only).
 *
 * **Trace-replay mode** reads a SUMO FCD output exported to the simple CSV
 * `time,vehid,x,y,z,speed` dialect and drives each registered vehicle's
 * MobilityModel along the recorded track. This lets CI run without a live
 * SUMO and lets researchers reproduce scenarios bit-for-bit. Road traces
 * must be supplied as files (e.g. via SUMO `fcd-export`); this module does
 * NOT fabricate vehicle motion.
 *
 * **Live TraCI co-simulation is explicitly OUT OF SCOPE for W7.** No live
 * TraCI client is implemented in-tree: `ConnectTcp` opens a real socket but
 * does not implement the `CMD_SIMSTEP` / `CMD_GET_VEHICLE_VARIABLE` wire
 * protocol, so it refuses to enter a live mode (callers must use
 * `LoadFcdTrace`). Implementing the TraCI wire protocol would be net-new
 * functionality.
 *
 * The bridge tracks per-step jitter — the MEASURED absolute offset between
 * `Simulator::Now()` and the trace timestamp at each sync point, with no
 * synthetic component — so the W7 validation gate (`jitter < 100 ms`) is
 * measured directly from the real ns-3 scheduler.
 */
#ifndef NTN_V2X_SUMO_TRACI_BRIDGE_H
#define NTN_V2X_SUMO_TRACI_BRIDGE_H

#include "ns3/mobility-model.h"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/traced-callback.h"

#include <map>
#include <string>
#include <vector>

namespace ns3
{
namespace ntnv2x
{

struct VehicleSample
{
    std::string vehId;
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double speedMps{0.0};
    double simulationTimeSec{0.0};
};

class SumoTraciBridge : public Object
{
  public:
    static TypeId GetTypeId();
    SumoTraciBridge();
    ~SumoTraciBridge() override;

    /// Open a TCP socket to `host:port`. NOTE: live TraCI co-simulation is
    /// unimplemented (out of scope for W7) — the TraCI stepping protocol is
    /// not driven, so this always returns false after warning, and the bridge
    /// stays in Disconnected mode. Use `LoadFcdTrace` for offline FCD replay.
    bool ConnectTcp(const std::string& host, uint16_t port);

    /// Load a CSV FCD trace: header `time,vehid,x,y,z,speed`.
    bool LoadFcdTrace(const std::string& path);

    /// Bind a vehicle ID to a mobility model. Position is updated each Step.
    void RegisterVehicle(const std::string& vehId, Ptr<MobilityModel> mob);

    /// Advance simulation; emit positions for all registered vehicles.
    /// Returns the number of vehicles updated this step.
    std::size_t Step();

    /// Run the bridge for `steps` ticks at `intervalSec` cadence using
    /// ns-3's Simulator (schedules events).
    void RunReplay(double intervalSec, std::size_t steps);

    /// Seconds elapsed since the bridge was constructed (replay timer).
    double GetSumoClockSec() const;

    /// MEASURED |Simulator::Now() - trace timestamp| at the most recent Step
    /// (seconds); the genuine replay clock offset, no synthetic component.
    double GetLastJitterSec() const;

    /// Worst observed jitter so far.
    double GetMaxJitterSec() const;

    /// Number of FCD trace rows currently loaded (0 if live).
    std::size_t LoadedSampleCount() const;

    /// Trace fired for every per-vehicle position emitted.
    typedef TracedCallback<const VehicleSample&> SampleTrace;

  private:
    enum class Mode
    {
        Disconnected,
        LiveTraci,
        Replay,
    };

    void DoStep();
    void EmitSample(const VehicleSample& s);
    bool ParseFcdCsv(const std::string& path);

    Mode m_mode{Mode::Disconnected};
    int m_socketFd{-1};

    // Replay state
    std::vector<VehicleSample> m_samples;     ///< sorted by simulationTimeSec
    std::size_t m_replayCursor{0};
    double m_currentSumoTime{0.0};
    double m_lastStepNowSec{0.0};   // V2: sim time at the previous Step()
    bool m_cadenceChecked{false};   // V2: trace-vs-tick cadence warned once
    /// V2: detected trace step (s) = smallest positive gap between distinct
    /// SUMO timestamps in the loaded FCD samples.
    double DetectTraceCadence() const;

    std::map<std::string, Ptr<MobilityModel>> m_vehicles;
    double m_lastJitterSec{0.0};
    double m_maxJitterSec{0.0};
    SampleTrace m_traceSample;
};

} // namespace ntnv2x
} // namespace ns3

#endif // NTN_V2X_SUMO_TRACI_BRIDGE_H
