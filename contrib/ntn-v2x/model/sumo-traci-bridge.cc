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
    m_socketFd = fd;
    m_mode = Mode::LiveTraci;
    return true;
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
        s.simulationTimeSec = std::stod(tokens[0]);
        s.vehId = tokens[1];
        s.x = std::stod(tokens[2]);
        s.y = std::stod(tokens[3]);
        s.z = std::stod(tokens[4]);
        if (tokens.size() >= 6)
            s.speedMps = std::stod(tokens[5]);
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
        it->second->SetPosition(Vector{s.x, s.y, s.z});
    }
    m_traceSample(s);
}

std::size_t
SumoTraciBridge::Step()
{
    if (m_mode == Mode::Replay)
    {
        // Advance replay cursor: emit every sample whose timestamp is at or
        // before the next-tick boundary. We keep grouping by sumo timestamp.
        if (m_replayCursor >= m_samples.size())
        {
            return 0;
        }
        double targetTime = m_samples[m_replayCursor].simulationTimeSec;
        std::size_t emitted = 0;
        while (m_replayCursor < m_samples.size() &&
               std::abs(m_samples[m_replayCursor].simulationTimeSec - targetTime) < 1e-6)
        {
            EmitSample(m_samples[m_replayCursor]);
            ++m_replayCursor;
            ++emitted;
        }
        m_currentSumoTime = targetTime;
        double nowSec = Simulator::Now().GetSeconds();
        m_lastJitterSec = std::abs(nowSec - m_currentSumoTime);
        if (m_lastJitterSec > m_maxJitterSec)
            m_maxJitterSec = m_lastJitterSec;
        return emitted;
    }
    if (m_mode == Mode::LiveTraci)
    {
        // We deliberately keep the live TraCI path minimal — a real
        // deployment will drop in EURECOM/UPM TraCI clients once we have a
        // running SUMO. For now: advance ns-3 time and return 0 so callers
        // know the live path is wired but quiescent.
        m_currentSumoTime = Simulator::Now().GetSeconds();
        m_lastJitterSec = 0.0;
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
