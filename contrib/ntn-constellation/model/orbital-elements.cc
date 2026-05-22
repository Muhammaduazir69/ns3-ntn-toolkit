/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "orbital-elements.h"

#include <cmath>
#include <ctime>
#include <sstream>
#include <stdexcept>

namespace ns3
{
namespace ntncon
{

namespace
{

constexpr double kPi = M_PI;
constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kDegToRad = kPi / 180.0;

bool
ParseDoubleField(const std::string& s, double& out)
{
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
ParseIntField(const std::string& s, int& out)
{
    try
    {
        out = std::stoi(s);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

/// Decodes TLE's compact-mantissa field "+12345-3" => +0.12345e-3.
double
DecodeImpliedDecimal(const std::string& field)
{
    if (field.empty())
        return 0.0;
    std::string s = field;
    while (!s.empty() && s.front() == ' ')
        s.erase(s.begin());
    if (s.empty())
        return 0.0;
    char sign = '+';
    if (s[0] == '+' || s[0] == '-')
    {
        sign = s[0];
        s.erase(s.begin());
    }
    // Split mantissa from the trailing signed exponent.
    int expSign = +1;
    int expVal = 0;
    for (size_t i = s.size(); i-- > 0;)
    {
        if (s[i] == '+' || s[i] == '-')
        {
            expSign = (s[i] == '-') ? -1 : +1;
            int eRaw = 0;
            ParseIntField(s.substr(i + 1), eRaw);
            expVal = expSign * eRaw;
            s = s.substr(0, i);
            break;
        }
    }
    double mantissa = 0.0;
    if (!s.empty())
    {
        try
        {
            mantissa = std::stod("0." + s);
        }
        catch (...)
        {
            mantissa = 0.0;
        }
    }
    double v = mantissa * std::pow(10.0, expVal);
    return (sign == '-') ? -v : v;
}

/// TLE epoch field "YYDDD.dddddddd" -> Unix seconds (UTC).
bool
DecodeTleEpoch(const std::string& field, double& outUnixS)
{
    if (field.size() < 5)
        return false;
    int yy = 0;
    if (!ParseIntField(field.substr(0, 2), yy))
        return false;
    const int year = (yy < 57) ? 2000 + yy : 1900 + yy;
    double doy = 0.0;
    if (!ParseDoubleField(field.substr(2), doy))
        return false;
    std::tm tm{};
    tm.tm_year = year - 1900;
    tm.tm_mon = 0;
    tm.tm_mday = 1;
    tm.tm_hour = 0;
    tm.tm_min = 0;
    tm.tm_sec = 0;
    time_t base = timegm(&tm);
    outUnixS = static_cast<double>(base) + (doy - 1.0) * 86400.0;
    return true;
}

/// Compute the TLE modulo-10 checksum digit for a 68-char body.
int
TleChecksum(const std::string& line68)
{
    int sum = 0;
    for (char c : line68)
    {
        if (c >= '0' && c <= '9')
            sum += (c - '0');
        else if (c == '-')
            sum += 1;
    }
    return sum % 10;
}

} // namespace

double
KeplerianElements::MeanMotionRadS() const
{
    if (semi_major_axis_m <= 0.0)
        return 0.0;
    return std::sqrt(kEarthMuM3S2 /
                      (semi_major_axis_m * semi_major_axis_m *
                       semi_major_axis_m));
}

double
KeplerianElements::PeriodSeconds() const
{
    double n = MeanMotionRadS();
    return (n > 0.0) ? (kTwoPi / n) : 0.0;
}

bool
TleRecord::ChecksumOk() const
{
    auto check = [](const std::string& l) {
        if (l.size() < 69)
            return false;
        int expected = l[68] - '0';
        if (expected < 0 || expected > 9)
            return false;
        return TleChecksum(l.substr(0, 68)) == expected;
    };
    return check(line1) && check(line2);
}

bool
TleRecord::ToKeplerian(KeplerianElements& out) const
{
    // TLE columns are 1-indexed in the spec; here we use 0-indexed C++.
    if (line1.size() < 69 || line2.size() < 69)
        return false;
    if (line1[0] != '1' || line2[0] != '2')
        return false;

    // Line 1: cols 19-32 epoch, cols 54-61 Bstar mantissa+exp.
    double epoch_s = 0.0;
    if (!DecodeTleEpoch(line1.substr(18, 14), epoch_s))
        return false;
    const double bstar = DecodeImpliedDecimal(line1.substr(53, 8));

    // Line 2: cols 9-16 inc deg, 17-25 raan deg, 26-33 ecc, 34-42 argp deg,
    // 43-51 mean anomaly deg, 52-63 mean motion (rev/day), 2-7 catalog no.
    int norad = 0;
    if (!ParseIntField(line2.substr(2, 5), norad))
        return false;
    double inc_deg, raan_deg, ecc_raw, argp_deg, ma_deg, mm_revday;
    if (!ParseDoubleField(line2.substr(8, 8), inc_deg) ||
        !ParseDoubleField(line2.substr(17, 8), raan_deg) ||
        !ParseDoubleField("0." + line2.substr(26, 7), ecc_raw) ||
        !ParseDoubleField(line2.substr(34, 8), argp_deg) ||
        !ParseDoubleField(line2.substr(43, 8), ma_deg) ||
        !ParseDoubleField(line2.substr(52, 11), mm_revday))
    {
        return false;
    }

    // Mean motion (rev/day) -> rad/s -> semi-major axis from n^2 a^3 = GM.
    const double n_rad_s = mm_revday * kTwoPi / 86400.0;
    if (n_rad_s <= 0.0)
        return false;
    const double a = std::cbrt(kEarthMuM3S2 / (n_rad_s * n_rad_s));

    out.semi_major_axis_m = a;
    out.eccentricity = ecc_raw;
    out.inclination_rad = inc_deg * kDegToRad;
    out.raan_rad = raan_deg * kDegToRad;
    out.arg_perigee_rad = argp_deg * kDegToRad;
    out.mean_anomaly_rad = ma_deg * kDegToRad;
    out.epoch_unix_s = epoch_s;
    out.bstar = bstar;
    out.norad_id = static_cast<uint32_t>(norad);
    out.name = name;
    return true;
}

std::size_t
ParseTleStream(const std::string& body, std::vector<TleRecord>& out)
{
    out.clear();
    std::istringstream is(body);
    std::string line;
    std::vector<std::string> staged;
    while (std::getline(is, line))
    {
        // Strip trailing \r.
        while (!line.empty() && (line.back() == '\r' || line.back() == ' '))
        {
            line.pop_back();
        }
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        staged.push_back(line);
        // A TLE record is line1 (starts with '1 ') + line2 (starts with '2 '),
        // optionally prefixed by a 0-line satellite name.
        if (staged.size() == 1)
        {
            if (staged.front().size() >= 1 && staged.front()[0] == '1')
            {
                continue;
            }
            // First line isn't a TLE line 1 — assume name line.
            continue;
        }
        if (staged.size() == 2)
        {
            // Could be (name, line1) or (line1, line2).
            if (staged[0].size() >= 1 && staged[0][0] == '1' &&
                staged[1].size() >= 1 && staged[1][0] == '2')
            {
                TleRecord r;
                r.line1 = staged[0];
                r.line2 = staged[1];
                out.push_back(r);
                staged.clear();
            }
            continue;
        }
        if (staged.size() == 3)
        {
            if (staged[1].size() >= 1 && staged[1][0] == '1' &&
                staged[2].size() >= 1 && staged[2][0] == '2')
            {
                TleRecord r;
                r.name = staged[0];
                r.line1 = staged[1];
                r.line2 = staged[2];
                out.push_back(r);
            }
            staged.clear();
        }
    }
    return out.size();
}

} // namespace ntncon
} // namespace ns3
