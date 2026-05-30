/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "sionna-replay-transport.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SionnaReplayTransport");

namespace
{

constexpr char kMagic[9] = "NTNRPLAY"; // 8 chars + NUL = 9
constexpr uint8_t kVersion = 1;

} // namespace

// ----------------------------------------------------------------------------
// SionnaReplayWriter
// ----------------------------------------------------------------------------

bool
SionnaReplayWriter::Open(const std::string& path,
                            uint64_t freq_hz_default)
{
    Close();
    m_path = path;
    std::FILE* fp = std::fopen(path.c_str(), "wb");
    if (!fp)
    {
        return false;
    }
    // Magic + version
    if (std::fwrite(kMagic, 1, 8, fp) != 8)
    {
        std::fclose(fp);
        return false;
    }
    if (std::fwrite(&kVersion, 1, 1, fp) != 1)
    {
        std::fclose(fp);
        return false;
    }
    // record_count placeholder (patched in Close)
    const uint64_t zero64 = 0;
    if (std::fwrite(&zero64, sizeof(zero64), 1, fp) != 1)
    {
        std::fclose(fp);
        return false;
    }
    if (std::fwrite(&freq_hz_default, sizeof(freq_hz_default), 1, fp)
         != 1)
    {
        std::fclose(fp);
        return false;
    }
    m_fp = fp;
    m_count = 0;
    return true;
}

bool
SionnaReplayWriter::Write(const ReplayRecord& r)
{
    if (!m_fp)
    {
        return false;
    }
    auto* fp = static_cast<std::FILE*>(m_fp);
    if (std::fwrite(&r.t_s, sizeof(r.t_s), 1, fp) != 1)
    {
        return false;
    }
    if (std::fwrite(r.sat_pos, sizeof(r.sat_pos), 1, fp) != 1)
    {
        return false;
    }
    if (std::fwrite(r.ue_pos, sizeof(r.ue_pos), 1, fp) != 1)
    {
        return false;
    }
    if (std::fwrite(&r.freq_hz, sizeof(r.freq_hz), 1, fp) != 1)
    {
        return false;
    }
    if (std::fwrite(&r.path_loss_db, sizeof(r.path_loss_db), 1, fp)
         != 1)
    {
        return false;
    }
    if (std::fwrite(&r.n_paths, sizeof(r.n_paths), 1, fp) != 1)
    {
        return false;
    }
    if (std::fwrite(&r.compute_ms, sizeof(r.compute_ms), 1, fp) != 1)
    {
        return false;
    }
    ++m_count;
    return true;
}

bool
SionnaReplayWriter::Close()
{
    if (!m_fp)
    {
        return true;
    }
    auto* fp = static_cast<std::FILE*>(m_fp);
    // Patch record_count at offset 9.
    if (std::fseek(fp, 9, SEEK_SET) != 0)
    {
        std::fclose(fp);
        m_fp = nullptr;
        return false;
    }
    if (std::fwrite(&m_count, sizeof(m_count), 1, fp) != 1)
    {
        std::fclose(fp);
        m_fp = nullptr;
        return false;
    }
    std::fclose(fp);
    m_fp = nullptr;
    return true;
}

// ----------------------------------------------------------------------------
// SionnaReplayReader
// ----------------------------------------------------------------------------

bool
SionnaReplayReader::Load(const std::string& path)
{
    m_records.clear();
    m_freqDefault = 0;
    std::FILE* fp = std::fopen(path.c_str(), "rb");
    if (!fp)
    {
        return false;
    }
    char magic[8];
    uint8_t version = 0;
    if (std::fread(magic, 1, 8, fp) != 8 ||
        std::memcmp(magic, kMagic, 8) != 0 ||
        std::fread(&version, 1, 1, fp) != 1 ||
        version != kVersion)
    {
        std::fclose(fp);
        return false;
    }
    uint64_t count = 0;
    if (std::fread(&count, sizeof(count), 1, fp) != 1)
    {
        std::fclose(fp);
        return false;
    }
    if (std::fread(&m_freqDefault, sizeof(m_freqDefault), 1, fp) != 1)
    {
        std::fclose(fp);
        return false;
    }
    m_records.reserve(count);
    for (uint64_t i = 0; i < count; ++i)
    {
        ReplayRecord r;
        if (std::fread(&r.t_s, sizeof(r.t_s), 1, fp) != 1 ||
            std::fread(r.sat_pos, sizeof(r.sat_pos), 1, fp) != 1 ||
            std::fread(r.ue_pos, sizeof(r.ue_pos), 1, fp) != 1 ||
            std::fread(&r.freq_hz, sizeof(r.freq_hz), 1, fp) != 1 ||
            std::fread(&r.path_loss_db, sizeof(r.path_loss_db), 1, fp)
                != 1 ||
            std::fread(&r.n_paths, sizeof(r.n_paths), 1, fp) != 1 ||
            std::fread(&r.compute_ms, sizeof(r.compute_ms), 1, fp) != 1)
        {
            m_records.clear();
            std::fclose(fp);
            return false;
        }
        m_records.push_back(r);
    }
    std::fclose(fp);
    return true;
}

void
SionnaReplayReader::SetDistanceWeights(double w_time_s,
                                          double w_space_m)
{
    m_wTime = w_time_s;
    m_wSpace = w_space_m;
}

size_t
SionnaReplayReader::NearestIndex(double t_s,
                                    const double sat_pos[3],
                                    const double ue_pos[3]) const
{
    if (m_records.empty())
    {
        return std::numeric_limits<size_t>::max();
    }
    size_t best = 0;
    double best_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < m_records.size(); ++i)
    {
        const auto& r = m_records[i];
        const double dt = (r.t_s - t_s);
        double dsat = 0.0;
        double due = 0.0;
        for (int k = 0; k < 3; ++k)
        {
            const double a = r.sat_pos[k] - sat_pos[k];
            const double b = r.ue_pos[k] - ue_pos[k];
            dsat += a * a;
            due += b * b;
        }
        const double d = m_wTime * dt * dt +
                          m_wSpace * (dsat + due);
        if (d < best_dist)
        {
            best_dist = d;
            best = i;
        }
    }
    return best;
}

// ----------------------------------------------------------------------------
// SionnaReplayTransport
// ----------------------------------------------------------------------------

TypeId
SionnaReplayTransport::GetTypeId()
{
    static TypeId tid = TypeId("ns3::SionnaReplayTransport")
                            .SetParent<SionnaTransport>()
                            .SetGroupName("NtnSionna")
                            .AddConstructor<SionnaReplayTransport>();
    return tid;
}

SionnaReplayTransport::SionnaReplayTransport() = default;

bool
SionnaReplayTransport::LoadFile(const std::string& path)
{
    return m_reader.Load(path);
}

void
SionnaReplayTransport::SetClockOverride(double t_s)
{
    m_useClockOverride = true;
    m_clockOverride = t_s;
}

void
SionnaReplayTransport::ClearClockOverride()
{
    m_useClockOverride = false;
}

SionnaTransport::Response
SionnaReplayTransport::Query(const Request& req) const
{
    ++m_queriesSent;
    Response out;
    if (m_reader.Size() == 0)
    {
        out.path_loss_db = std::numeric_limits<double>::infinity();
        out.ok = false;
        ++m_failures;
        return out;
    }
    const double t_s = m_useClockOverride
                             ? m_clockOverride
                             : Simulator::Now().GetSeconds();
    const double sat[3] = {req.tx_x, req.tx_y, req.tx_z};
    const double ue[3] = {req.rx_x, req.rx_y, req.rx_z};
    const size_t idx = m_reader.NearestIndex(t_s, sat, ue);
    const auto& r = m_reader.At(idx);
    out.path_loss_db = r.path_loss_db;
    out.n_paths = r.n_paths;
    out.compute_ms = r.compute_ms;
    out.ok = true;
    return out;
}

} // namespace ns3
