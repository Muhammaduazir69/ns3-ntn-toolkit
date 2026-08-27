/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W7)
 */
#include "sumo-traci-bridge.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include "ns3/constant-velocity-mobility-model.h"
#include <cmath>
#include <cstring>
#include <fstream>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace ns3
{
namespace ntnv2x
{

NS_LOG_COMPONENT_DEFINE("SumoTraciBridge");
NS_OBJECT_ENSURE_REGISTERED(SumoTraciBridge);

TypeId
SumoTraciBridge::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ntnv2x::SumoTraciBridge")
            .SetParent<Object>()
            .SetGroupName("NtnV2x")
            .AddConstructor<SumoTraciBridge>()
            .AddTraceSource("Sample",
                            "Per-vehicle position sample emitted at each Step",
                            MakeTraceSourceAccessor(&SumoTraciBridge::m_traceSample),
                            "ns3::ntnv2x::SumoTraciBridge::SampleTrace");
    return tid;
}

SumoTraciBridge::SumoTraciBridge() = default;

SumoTraciBridge::~SumoTraciBridge()
{
    if (m_socketFd >= 0)
    {
        ::close(m_socketFd);
        m_socketFd = -1;
    }
}

bool
SumoTraciBridge::ConnectTcp(const std::string& host, uint16_t port)
{
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
    {
        NS_LOG_WARN("socket() failed: " << std::strerror(errno));
        return false;
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) <= 0)
    {
        NS_LOG_WARN("invalid host " << host);
        ::close(fd);
        return false;
    }
    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        NS_LOG_WARN("connect() to " << host << ":" << port
                                   << " failed: " << std::strerror(errno));
        ::close(fd);
        return false;
    }
    // The socket open/connect above is real, but the TraCI stepping wire
    // protocol (CMD_SIMSTEP / CMD_GET_VEHICLE_VARIABLE) is NOT implemented in
    // this module — live co-simulation is out of scope for W7. We therefore do
    // NOT switch to a live mode here: leaving m_mode as Disconnected prevents a
    // caller from accidentally driving a no-op live loop that silently holds
    // positions. The caller must use LoadFcdTrace() for offline FCD replay.
    NS_LOG_WARN("ConnectTcp: socket connected to "
                << host << ":" << port
                << " but the TraCI stepping protocol is unimplemented; "
                   "this module is offline FCD-replay only — call LoadFcdTrace().");
    ::close(fd);
    m_socketFd = -1;
    return false;
}

bool
SumoTraciBridge::LoadFcdTrace(const std::string& path)
{
    if (!ParseFcdCsv(path))
    {
        return false;
    }
    std::sort(m_samples.begin(), m_samples.end(),
              [](const VehicleSample& a, const VehicleSample& b) {
                  return a.simulationTimeSec < b.simulationTimeSec;
              });
    m_replayCursor = 0;
    m_currentSumoTime = m_samples.empty() ? 0.0 : m_samples.front().simulationTimeSec;
    m_mode = Mode::Replay;
    return true;
}

bool
SumoTraciBridge::ParseFcdCsv(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
    {
        NS_LOG_WARN("cannot open trace " << path);
        return false;
    }
    std::string line;
    bool sawHeader = false;
    while (std::getline(f, line))
    {
        if (line.empty())
            continue;
        if (!sawHeader)
        {
            sawHeader = true;
            if (line.find("time") != std::string::npos)
                continue; // header row
        }
        std::stringstream ss(line);
        std::string tok;
        std::vector<std::string> tokens;
        while (std::getline(ss, tok, ','))
            tokens.push_back(tok);
        if (tokens.size() < 5)
            continue;
        VehicleSample s;
        try
        {
            s.simulationTimeSec = std::stod(tokens[0]);
            s.vehId = tokens[1];
            s.x = std::stod(tokens[2]);
            s.y = std::stod(tokens[3]);
            s.z = std::stod(tokens[4]);
            if (tokens.size() >= 6)
                s.speedMps = std::stod(tokens[5]);
        }
        catch (const std::exception&)
        {
            // Malformed row (non-numeric field): skip it rather than abort the run.
            continue;
        }
        m_samples.push_back(s);
    }
    return !m_samples.empty();
}

void
SumoTraciBridge::RegisterVehicle(const std::string& vehId, Ptr<MobilityModel> mob)
{
    m_vehicles[vehId] = mob;
}

double
SumoTraciBridge::GetSumoClockSec() const
{
    return m_currentSumoTime;
}

double
SumoTraciBridge::GetLastJitterSec() const
{
    return m_lastJitterSec;
}

double
SumoTraciBridge::GetMaxJitterSec() const
{
    return m_maxJitterSec;
}

std::size_t
SumoTraciBridge::LoadedSampleCount() const
{
    return m_samples.size();
}

void
SumoTraciBridge::EmitSample(const VehicleSample& s)
{
    auto it = m_vehicles.find(s.vehId);
    if (it != m_vehicles.end())
    {
        const Vector pos{s.x, s.y, s.z};

        // V2X-6: push VELOCITY too, not only position.
        //
        // This used to call SetPosition and nothing else, so the speedMps
        // parsed out of the FCD trace was read and discarded. Every consumer
        // that asked the mobility model how fast a vehicle was going got zero:
        // the BSM header's speed and heading fields (SAE J2735 Part I) were
        // computed from GetVelocity() and so were 0 and atan2(0,0)=0 on every
        // packet, and any Doppler or velocity-dependent channel attached to a
        // SUMO-driven vehicle saw a stationary car.
        //
        // Direction comes from the displacement between consecutive samples,
        // magnitude from SUMO's own speed column when it has one - SUMO is
        // authoritative about speed, and differencing positions across a coarse
        // trace cadence is not. With no speed column the displacement supplies
        // both.
        Vector vel{0.0, 0.0, 0.0};
        auto prev = m_lastSample.find(s.vehId);
        if (prev != m_lastSample.end())
        {
            const double dt = s.simulationTimeSec - prev->second.timeSec;
            if (dt > 0.0)
            {
                const Vector d{pos.x - prev->second.pos.x,
                               pos.y - prev->second.pos.y,
                               pos.z - prev->second.pos.z};
                const double dist = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
                if (dist > 1e-9)
                {
                    const double speed = (s.speedMps > 0.0) ? s.speedMps : (dist / dt);
                    vel = Vector{d.x / dist * speed, d.y / dist * speed, d.z / dist * speed};
                }
            }
        }

        it->second->SetPosition(pos);
        Ptr<ConstantVelocityMobilityModel> cv =
            DynamicCast<ConstantVelocityMobilityModel>(it->second);
        if (cv)
        {
            cv->SetVelocity(vel);
            if (vel.x != 0.0 || vel.y != 0.0 || vel.z != 0.0)
            {
                ++m_velocitySamples;
            }
        }
        else if (s.speedMps > 0.0)
        {
            // Say it once. A ConstantPositionMobilityModel cannot carry a
            // velocity, so a scenario that registers one silently throws away
            // every speed SUMO reported - which is the defect this fixes, one
            // layer up.
            if (!m_warnedNoVelocity)
            {
                m_warnedNoVelocity = true;
                NS_LOG_WARN("SumoTraciBridge: vehicle "
                            << s.vehId << " is registered against a mobility model that cannot "
                            << "hold a velocity, so SUMO's speed is discarded. Register a "
                            << "ConstantVelocityMobilityModel if BSM speed/heading or any "
                            << "velocity-dependent channel matters.");
            }
            ++m_velocityDropped;
        }

        m_lastSample[s.vehId] = LastSample{pos, s.simulationTimeSec};
    }
    m_traceSample(s);
}

double
SumoTraciBridge::DetectTraceCadence() const
{
    // Smallest positive difference between consecutive distinct SUMO timestamps.
    double best = 0.0;
    double prev = -1.0;
    for (const auto& smp : m_samples)
    {
        if (prev >= 0.0)
        {
            const double d = smp.simulationTimeSec - prev;
            if (d > 1e-6 && (best == 0.0 || d < best))
            {
                best = d;
            }
        }
        prev = smp.simulationTimeSec;
    }
    return best;
}

std::size_t
SumoTraciBridge::Step()
{
    if (m_mode == Mode::Replay)
    {
        // GAP V2 FIX: TIME-LOCK the replay cursor to the simulation clock.
        //
        // This used to advance exactly one SUMO timestamp-group per Step() call,
        // ignoring Simulator::Now() entirely. So if the caller's tick interval
        // did not equal the trace cadence (e.g. Step() every 0.1 s on a 1 Hz
        // FCD trace) the vehicles fast-forwarded 10x through their track while
        // wall time crawled — positions applied at the wrong times, and the
        // jitter metric ballooned with nothing to stop the desync.
        //
        // Now emit every sample whose trace timestamp is <= the current sim
        // time, so playback tracks the simulation clock at whatever rate Step()
        // is called. Warn once if the trace cadence and the observed tick
        // interval disagree, since that is a scenario-configuration mistake.
        if (m_replayCursor >= m_samples.size())
        {
            return 0;
        }
        const double nowSec = Simulator::Now().GetSeconds();
        std::size_t emitted = 0;
        double lastTime = m_currentSumoTime;
        while (m_replayCursor < m_samples.size() &&
               m_samples[m_replayCursor].simulationTimeSec <= nowSec + 1e-6)
        {
            EmitSample(m_samples[m_replayCursor]);
            lastTime = m_samples[m_replayCursor].simulationTimeSec;
            ++m_replayCursor;
            ++emitted;
        }
        // One-time cadence sanity check: compare the trace's own step (first two
        // distinct timestamps) against the tick interval this Step() was called
        // at. A mismatch means the caller's --dt does not match the FCD rate.
        if (!m_cadenceChecked && m_replayCursor >= 2)
        {
            m_cadenceChecked = true;
            const double traceStep =
                m_samples.size() > 1 ? DetectTraceCadence() : 0.0;
            const double tickStep = nowSec - m_lastStepNowSec;
            if (traceStep > 0.0 && tickStep > 0.0 &&
                std::abs(traceStep - tickStep) > 0.25 * traceStep)
            {
                NS_LOG_WARN("SUMO FCD trace cadence ("
                            << traceStep << " s) differs from the Step() tick interval ("
                            << tickStep << " s). Replay is time-locked to Simulator::Now(), so "
                            "positions stay correct, but set the tick interval to the trace "
                            "cadence to avoid emitting many groups per tick or stalling.");
            }
        }
        m_lastStepNowSec = nowSec;
        m_currentSumoTime = lastTime;
        // Genuine replay clock offset: the absolute difference between the real
        // ns-3 scheduler clock and the trace timestamp at this sync point. In
        // locked trace-replay this is ~0 by construction; it is MEASURED from
        // Simulator::Now(), with no synthetic/IPC component injected.
        m_lastJitterSec = std::abs(nowSec - m_currentSumoTime);
        if (m_lastJitterSec > m_maxJitterSec)
            m_maxJitterSec = m_lastJitterSec;
        return emitted;
    }
    if (m_mode == Mode::LiveTraci)
    {
        // Unreachable in practice: ConnectTcp never sets LiveTraci because the
        // TraCI stepping protocol is unimplemented. Guard honestly anyway —
        // warn once and return 0 WITHOUT advancing m_currentSumoTime, so no
        // vehicle position is silently held or fabricated.
        static bool warned = false;
        if (!warned)
        {
            NS_LOG_WARN("Step(): live TraCI is not implemented; this module is "
                        "offline FCD-replay only. Use LoadFcdTrace().");
            warned = true;
        }
        return 0;
    }
    return 0;
}

void
SumoTraciBridge::DoStep()
{
    Step();
}

void
SumoTraciBridge::RunReplay(double intervalSec, std::size_t steps)
{
    Time interval = Seconds(intervalSec);
    for (std::size_t i = 0; i < steps; ++i)
    {
        Simulator::Schedule(interval * static_cast<int>(i),
                            &SumoTraciBridge::DoStep, this);
    }
}

} // namespace ntnv2x
} // namespace ns3
