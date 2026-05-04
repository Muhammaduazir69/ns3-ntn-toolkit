/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W7)
 *
 * SUMO ↔ ns-3 mobility bridge.
 *
 * Two operation modes are supported behind a single API:
 *
 *   1. **Live TraCI mode** — opens a TCP socket to a running SUMO process,
 *      issues `CMD_SIMSTEP` per step, and reads back vehicle positions via
 *      `CMD_GET_VEHICLE_VARIABLE` (var ID 0x42 = position3D). Use when
 *      SUMO is available; the wire protocol is TraCI v20+ as documented in
 *      the SUMO user manual (sumo.dlr.de).
 *
 *   2. **Trace-replay mode** — reads a SUMO FCD output file (XML-ish or
 *      CSV) and feeds the same callback. Lets CI run without a live SUMO,
 *      and lets researchers reproduce scenarios bit-for-bit. The dialect
 *      is the simple CSV `time,vehid,x,y,z,v` form.
 *
 * The bridge tracks per-step jitter — the absolute offset between
 * `Simulator::Now()` and the SUMO timestamp at each sync point — so the
 * W7 validation gate (`jitter < 100 ms`) can be measured directly.
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

    /// Open a connection to a live SUMO process (TraCI v20+).
    /// Returns false if the socket fails to connect; the bridge then
    /// silently degrades — call `LoadFcdTrace` afterwards to fall back.
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

    /// |Simulator::Now() − SUMO clock| at the most recent Step (seconds).
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

    std::map<std::string, Ptr<MobilityModel>> m_vehicles;
    double m_lastJitterSec{0.0};
    double m_maxJitterSec{0.0};
    SampleTrace m_traceSample;
};

} // namespace ntnv2x
} // namespace ns3

#endif // NTN_V2X_SUMO_TRACI_BRIDGE_H
