/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "haps-trajectory-trace.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace ns3
{
namespace sagin
{

namespace
{

std::vector<std::string>
SplitCsv(const std::string& line)
{
    std::vector<std::string> out;
    std::string cur;
    for (char c : line)
    {
        if (c == ',')
        {
            out.push_back(cur);
            cur.clear();
        }
        else if (c == '\r')
        {
            // ignored
        }
        else
        {
            cur.push_back(c);
        }
    }
    out.push_back(cur);
    return out;
}

std::string
ToLower(std::string s)
{
    for (char& c : s)
    {
        if (c >= 'A' && c <= 'Z')
        {
            c = static_cast<char>(c + 32);
        }
    }
    return s;
}

bool
ParseDouble(const std::string& s, double& out)
{
    if (s.empty())
    {
        return false;
    }
    try
    {
        out = std::stod(s);
        return std::isfinite(out);
    }
    catch (...)
    {
        return false;
    }
}

double
LerpDouble(double a, double b, double t)
{
    return a + (b - a) * t;
}

/// Heading lerp on the circle in [0, 360) — picks the shorter arc.
double
LerpHeading(double a, double b, double t)
{
    double d = b - a;
    if (d > 180.0)
    {
        d -= 360.0;
    }
    else if (d < -180.0)
    {
        d += 360.0;
    }
    double r = a + d * t;
    if (r < 0.0)
    {
        r += 360.0;
    }
    else if (r >= 360.0)
    {
        r -= 360.0;
    }
    return r;
}

} // namespace

double
HapsTrajectoryTrace::TStart() const
{
    if (samples.empty())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return samples.front().time_s;
}

double
HapsTrajectoryTrace::TEnd() const
{
    if (samples.empty())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return samples.back().time_s;
}

HapsTrajectorySample
HapsTrajectoryTrace::InterpolateAt(double time_s) const
{
    if (samples.empty())
    {
        return HapsTrajectorySample{};
    }
    if (time_s <= samples.front().time_s)
    {
        return samples.front();
    }
    if (time_s >= samples.back().time_s)
    {
        return samples.back();
    }
    // Binary search for upper bound.
    auto it = std::upper_bound(
        samples.begin(),
        samples.end(),
        time_s,
        [](double t, const HapsTrajectorySample& s) {
            return t < s.time_s;
        });
    const auto& hi = *it;
    const auto& lo = *(it - 1);
    const double span = hi.time_s - lo.time_s;
    const double t =
        (span > 0.0) ? (time_s - lo.time_s) / span : 0.0;
    HapsTrajectorySample out;
    out.time_s = time_s;
    out.lat_deg = LerpDouble(lo.lat_deg, hi.lat_deg, t);
    out.lon_deg = LerpDouble(lo.lon_deg, hi.lon_deg, t);
    out.alt_m = LerpDouble(lo.alt_m, hi.alt_m, t);
    out.has_heading = lo.has_heading && hi.has_heading;
    if (out.has_heading)
    {
        out.heading_deg =
            LerpHeading(lo.heading_deg, hi.heading_deg, t);
    }
    out.has_speed = lo.has_speed && hi.has_speed;
    if (out.has_speed)
    {
        out.speed_mps = LerpDouble(lo.speed_mps, hi.speed_mps, t);
    }
    return out;
}

std::map<std::string, HapsTrajectoryTrace>
HapsTrajectoryImporter::LoadCsv(const std::string& path,
                                  double tStart_s,
                                  double tEnd_s)
{
    m_lastRowsRead = 0;
    m_lastRowsSkipped = 0;
    std::map<std::string, HapsTrajectoryTrace> out;
    std::ifstream in(path);
    if (!in)
    {
        return out;
    }
    std::string header_line;
    if (!std::getline(in, header_line))
    {
        return out;
    }
    const auto header = SplitCsv(header_line);
    int col_t = -1, col_lat = -1, col_lon = -1, col_alt = -1;
    int col_heading = -1, col_speed = -1, col_id = -1;
    for (size_t i = 0; i < header.size(); ++i)
    {
        const auto h = ToLower(header[i]);
        if (h == "time_s" || h == "time" || h == "t")
        {
            col_t = static_cast<int>(i);
        }
        else if (h == "lat_deg" || h == "lat" || h == "latitude")
        {
            col_lat = static_cast<int>(i);
        }
        else if (h == "lon_deg" || h == "lon" || h == "lng" ||
                  h == "longitude")
        {
            col_lon = static_cast<int>(i);
        }
        else if (h == "alt_m" || h == "alt" || h == "altitude")
        {
            col_alt = static_cast<int>(i);
        }
        else if (h == "heading_deg" || h == "heading" ||
                  h == "course_deg")
        {
            col_heading = static_cast<int>(i);
        }
        else if (h == "speed_mps" || h == "speed" || h == "vel_mps")
        {
            col_speed = static_cast<int>(i);
        }
        else if (h == "platform_id" || h == "platform" || h == "id")
        {
            col_id = static_cast<int>(i);
        }
    }
    if (col_t < 0 || col_lat < 0 || col_lon < 0 || col_alt < 0)
    {
        return out;
    }
    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty())
        {
            continue;
        }
        ++m_lastRowsRead;
        const auto cols = SplitCsv(line);
        if (cols.size() <
            static_cast<size_t>(
                std::max({col_t, col_lat, col_lon, col_alt}) + 1))
        {
            ++m_lastRowsSkipped;
            continue;
        }
        double t = 0.0, lat = 0.0, lon = 0.0, alt = 0.0;
        if (!ParseDouble(cols[col_t], t) ||
            !ParseDouble(cols[col_lat], lat) ||
            !ParseDouble(cols[col_lon], lon) ||
            !ParseDouble(cols[col_alt], alt))
        {
            ++m_lastRowsSkipped;
            continue;
        }
        if (t < tStart_s || t > tEnd_s)
        {
            ++m_lastRowsSkipped;
            continue;
        }
        HapsTrajectorySample s;
        s.time_s = t;
        s.lat_deg = lat;
        s.lon_deg = lon;
        s.alt_m = alt;
        if (col_heading >= 0 &&
            col_heading < static_cast<int>(cols.size()))
        {
            double h = 0.0;
            if (ParseDouble(cols[col_heading], h))
            {
                s.heading_deg = h;
                s.has_heading = true;
            }
        }
        if (col_speed >= 0 &&
            col_speed < static_cast<int>(cols.size()))
        {
            double v = 0.0;
            if (ParseDouble(cols[col_speed], v))
            {
                s.speed_mps = v;
                s.has_speed = true;
            }
        }
        std::string id = "haps";
        if (col_id >= 0 &&
            col_id < static_cast<int>(cols.size()) &&
            !cols[col_id].empty())
        {
            id = cols[col_id];
        }
        auto& tr = out[id];
        tr.platform_id = id;
        tr.samples.push_back(s);
    }
    // Sort each trace by time.
    for (auto& kv : out)
    {
        std::sort(kv.second.samples.begin(),
                   kv.second.samples.end(),
                   [](const HapsTrajectorySample& a,
                       const HapsTrajectorySample& b) {
                       return a.time_s < b.time_s;
                   });
    }
    return out;
}

} // namespace sagin
} // namespace ns3
