/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ais-maritime-trace.h"

#include "ns3/log.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <limits>
#include <sstream>

namespace ns3
{
namespace sagin
{

NS_LOG_COMPONENT_DEFINE("AisMaritimeTrace");

double
AisMaritimeTrace::TStart() const
{
    return samples.empty() ? std::numeric_limits<double>::quiet_NaN()
                             : samples.front().time_s;
}

double
AisMaritimeTrace::TEnd() const
{
    return samples.empty() ? std::numeric_limits<double>::quiet_NaN()
                             : samples.back().time_s;
}

AisSample
AisMaritimeTrace::InterpolateAt(double time_s) const
{
    if (samples.empty())
    {
        return {};
    }
    if (time_s <= samples.front().time_s)
    {
        return samples.front();
    }
    if (time_s >= samples.back().time_s)
    {
        return samples.back();
    }
    auto it = std::upper_bound(samples.begin(),
                                 samples.end(),
                                 time_s,
                                 [](double t, const AisSample& s) {
                                     return t < s.time_s;
                                 });
    const AisSample& hi = *it;
    const AisSample& lo = *(it - 1);
    const double dt = hi.time_s - lo.time_s;
    if (dt <= 0.0)
    {
        return lo;
    }
    const double f = (time_s - lo.time_s) / dt;
    AisSample out{};
    out.time_s = time_s;
    out.lat_deg = lo.lat_deg + f * (hi.lat_deg - lo.lat_deg);
    out.lon_deg = lo.lon_deg + f * (hi.lon_deg - lo.lon_deg);
    out.sog_knots = lo.sog_knots + f * (hi.sog_knots - lo.sog_knots);
    // Short-arc COG / heading interpolation.
    auto shortArc = [&](double a, double b) {
        double d = b - a;
        if (d > 180.0)
            d -= 360.0;
        else if (d < -180.0)
            d += 360.0;
        double v = a + f * d;
        if (v < 0.0)
            v += 360.0;
        if (v >= 360.0)
            v -= 360.0;
        return v;
    };
    out.cog_deg = shortArc(lo.cog_deg, hi.cog_deg);
    out.heading_deg = (lo.heading_deg < 360.0 && hi.heading_deg < 360.0)
                         ? shortArc(lo.heading_deg, hi.heading_deg)
                         : lo.heading_deg;
    out.ship_type = lo.ship_type;
    out.callsign = lo.callsign;
    return out;
}

namespace
{

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

bool
ParseUint(const std::string& s, uint32_t& out)
{
    if (s.empty())
    {
        return false;
    }
    try
    {
        out = static_cast<uint32_t>(std::stoul(s));
        return true;
    }
    catch (...)
    {
        return false;
    }
}

/// Danish Maritime timestamps are 'DD/MM/YYYY HH:MM:SS' (24-hour, UTC).
bool
ParseDanishTimestamp(const std::string& s, double& out_unix_s)
{
    if (s.empty())
    {
        return false;
    }
    int dd, mo, yy, hh, mm, ss;
    if (std::sscanf(s.c_str(),
                     "%d/%d/%d %d:%d:%d",
                     &dd,
                     &mo,
                     &yy,
                     &hh,
                     &mm,
                     &ss) != 6)
    {
        return false;
    }
    std::tm t{};
    t.tm_mday = dd;
    t.tm_mon = mo - 1;
    t.tm_year = yy - 1900;
    t.tm_hour = hh;
    t.tm_min = mm;
    t.tm_sec = ss;
    t.tm_isdst = 0;
    time_t ts = timegm(&t); // UTC
    if (ts < 0)
    {
        return false;
    }
    out_unix_s = static_cast<double>(ts);
    return true;
}

std::vector<std::string>
SplitCsv(const std::string& line)
{
    std::vector<std::string> out;
    std::string cur;
    bool inQuote = false;
    for (char c : line)
    {
        if (c == '"')
        {
            inQuote = !inQuote;
            continue;
        }
        if (c == ',' && !inQuote)
        {
            out.push_back(cur);
            cur.clear();
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
Trim(std::string s)
{
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t' ||
                          s.front() == '#'))
    {
        s.erase(s.begin());
    }
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t' ||
                          s.back() == '\r'))
    {
        s.pop_back();
    }
    return s;
}

} // namespace

std::map<uint32_t, AisMaritimeTrace>
AisDanishImporter::LoadCsv(const std::string& path,
                            double tStartAbs_s,
                            double tEndAbs_s)
{
    std::map<uint32_t, AisMaritimeTrace> out;
    m_lastRowsRead = 0;
    m_lastRowsSkipped = 0;

    std::ifstream f(path);
    if (!f)
    {
        NS_LOG_WARN("AisDanishImporter: cannot open " << path);
        return out;
    }

    std::string header;
    if (!std::getline(f, header))
    {
        return out;
    }
    // Header may start with '#' in the Danish Maritime dump.
    std::vector<std::string> cols = SplitCsv(header);
    for (auto& c : cols)
    {
        c = Trim(c);
    }
    auto idx = [&](const std::string& name) -> int {
        for (size_t i = 0; i < cols.size(); ++i)
        {
            if (cols[i] == name)
                return static_cast<int>(i);
        }
        return -1;
    };
    const int iTime = idx("Timestamp");
    const int iMmsi = idx("MMSI");
    const int iLat = idx("Latitude");
    const int iLon = idx("Longitude");
    const int iSog = idx("SOG");
    const int iCog = idx("COG");
    const int iHdg = idx("Heading");
    const int iType = idx("Ship type");
    const int iCall = idx("Callsign");
    if (iTime < 0 || iMmsi < 0 || iLat < 0 || iLon < 0)
    {
        NS_LOG_WARN("AisDanishImporter: required column missing");
        return out;
    }

    std::string line;
    while (std::getline(f, line))
    {
        if (line.empty())
        {
            continue;
        }
        ++m_lastRowsRead;
        std::vector<std::string> v = SplitCsv(line);
        if (static_cast<int>(v.size()) <=
            std::max({iTime, iMmsi, iLat, iLon}))
        {
            ++m_lastRowsSkipped;
            continue;
        }
        double t = 0.0;
        if (!ParseDanishTimestamp(Trim(v[iTime]), t))
        {
            // Fall back: numeric Unix-epoch timestamp.
            if (!ParseDouble(Trim(v[iTime]), t))
            {
                ++m_lastRowsSkipped;
                continue;
            }
        }
        if (t < tStartAbs_s || t > tEndAbs_s)
        {
            ++m_lastRowsSkipped;
            continue;
        }
        uint32_t mmsi = 0;
        if (!ParseUint(Trim(v[iMmsi]), mmsi) || mmsi == 0)
        {
            ++m_lastRowsSkipped;
            continue;
        }
        double lat;
        double lon;
        if (!ParseDouble(Trim(v[iLat]), lat) ||
            !ParseDouble(Trim(v[iLon]), lon))
        {
            ++m_lastRowsSkipped;
            continue;
        }
        AisSample s{};
        s.time_s = t;
        s.lat_deg = lat;
        s.lon_deg = lon;
        if (iSog >= 0 && static_cast<int>(v.size()) > iSog)
        {
            ParseDouble(Trim(v[iSog]), s.sog_knots);
        }
        if (iCog >= 0 && static_cast<int>(v.size()) > iCog)
        {
            ParseDouble(Trim(v[iCog]), s.cog_deg);
        }
        if (iHdg >= 0 && static_cast<int>(v.size()) > iHdg)
        {
            ParseDouble(Trim(v[iHdg]), s.heading_deg);
        }
        if (iType >= 0 && static_cast<int>(v.size()) > iType)
        {
            s.ship_type = Trim(v[iType]);
        }
        if (iCall >= 0 && static_cast<int>(v.size()) > iCall)
        {
            s.callsign = Trim(v[iCall]);
        }
        auto& tr = out[mmsi];
        tr.mmsi = mmsi;
        if (tr.ship_type.empty() && !s.ship_type.empty())
        {
            tr.ship_type = s.ship_type;
        }
        tr.samples.push_back(s);
    }
    for (auto& kv : out)
    {
        std::sort(kv.second.samples.begin(),
                   kv.second.samples.end(),
                   [](const AisSample& a, const AisSample& b) {
                       return a.time_s < b.time_s;
                   });
    }
    NS_LOG_INFO("AisDanishImporter: " << path << " => " << out.size()
                                        << " vessels, " << m_lastRowsRead
                                        << " rows read, "
                                        << m_lastRowsSkipped << " skipped");
    return out;
}

} // namespace sagin
} // namespace ns3
