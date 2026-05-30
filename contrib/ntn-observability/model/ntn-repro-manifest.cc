/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-repro-manifest.h"

#include "ns3/log.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sstream>

namespace ns3
{
namespace ntnobs
{

NS_LOG_COMPONENT_DEFINE("NtnReproManifest");

namespace
{

std::string
NowIso8601Utc()
{
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    gmtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
    return buf;
}

void
JsonEscape(std::ostream& os, const std::string& s)
{
    os << '"';
    for (char c : s)
    {
        switch (c)
        {
        case '"':  os << "\\\""; break;
        case '\\': os << "\\\\"; break;
        case '\b': os << "\\b"; break;
        case '\f': os << "\\f"; break;
        case '\n': os << "\\n"; break;
        case '\r': os << "\\r"; break;
        case '\t': os << "\\t"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20)
            {
                char hex[8];
                std::snprintf(hex, sizeof(hex), "\\u%04x",
                              static_cast<unsigned char>(c));
                os << hex;
            }
            else
            {
                os << c;
            }
        }
    }
    os << '"';
}

// Minimal JSON parser, just enough for the manifest schema we emit.
// Tokens: '{' '}' '[' ']' ':' ',' string, number, true/false/null.
class JsonReader
{
  public:
    explicit JsonReader(const std::string& s)
        : m_s(s),
          m_i(0)
    {
    }

    void Skip()
    {
        while (m_i < m_s.size() &&
               (m_s[m_i] == ' ' || m_s[m_i] == '\t' || m_s[m_i] == '\n' ||
                m_s[m_i] == '\r'))
        {
            ++m_i;
        }
    }

    bool Eat(char c)
    {
        Skip();
        if (m_i < m_s.size() && m_s[m_i] == c)
        {
            ++m_i;
            return true;
        }
        return false;
    }

    bool Expect(char c)
    {
        if (!Eat(c))
        {
            Fail(std::string("expected '") + c + "'");
            return false;
        }
        return true;
    }

    bool ReadString(std::string& out)
    {
        Skip();
        if (m_i >= m_s.size() || m_s[m_i] != '"')
        {
            Fail("expected string");
            return false;
        }
        ++m_i;
        out.clear();
        while (m_i < m_s.size())
        {
            char c = m_s[m_i++];
            if (c == '"')
            {
                return true;
            }
            if (c == '\\')
            {
                if (m_i >= m_s.size())
                {
                    Fail("trailing backslash");
                    return false;
                }
                char e = m_s[m_i++];
                switch (e)
                {
                case '"':  out.push_back('"'); break;
                case '\\': out.push_back('\\'); break;
                case '/':  out.push_back('/'); break;
                case 'b':  out.push_back('\b'); break;
                case 'f':  out.push_back('\f'); break;
                case 'n':  out.push_back('\n'); break;
                case 'r':  out.push_back('\r'); break;
                case 't':  out.push_back('\t'); break;
                case 'u': {
                    if (m_i + 4 > m_s.size())
                    {
                        Fail("short \\u escape");
                        return false;
                    }
                    unsigned code = 0;
                    for (int k = 0; k < 4; ++k)
                    {
                        char h = m_s[m_i++];
                        code <<= 4;
                        if (h >= '0' && h <= '9')
                            code |= static_cast<unsigned>(h - '0');
                        else if (h >= 'a' && h <= 'f')
                            code |= static_cast<unsigned>(h - 'a' + 10);
                        else if (h >= 'A' && h <= 'F')
                            code |= static_cast<unsigned>(h - 'A' + 10);
                        else
                        {
                            Fail("bad \\u digit");
                            return false;
                        }
                    }
                    if (code < 0x80)
                    {
                        out.push_back(static_cast<char>(code));
                    }
                    else if (code < 0x800)
                    {
                        out.push_back(static_cast<char>(0xC0 | (code >> 6)));
                        out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
                    }
                    else
                    {
                        out.push_back(static_cast<char>(0xE0 | (code >> 12)));
                        out.push_back(
                            static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
                        out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
                    }
                    break;
                }
                default:
                    Fail("bad escape");
                    return false;
                }
            }
            else
            {
                out.push_back(c);
            }
        }
        Fail("unterminated string");
        return false;
    }

    bool ReadNumber(double& out)
    {
        Skip();
        size_t start = m_i;
        if (m_i < m_s.size() && (m_s[m_i] == '-' || m_s[m_i] == '+'))
        {
            ++m_i;
        }
        while (m_i < m_s.size() &&
               ((m_s[m_i] >= '0' && m_s[m_i] <= '9') || m_s[m_i] == '.' ||
                m_s[m_i] == 'e' || m_s[m_i] == 'E' || m_s[m_i] == '-' ||
                m_s[m_i] == '+'))
        {
            ++m_i;
        }
        if (m_i == start)
        {
            Fail("expected number");
            return false;
        }
        try
        {
            out = std::stod(m_s.substr(start, m_i - start));
            return true;
        }
        catch (...)
        {
            Fail("bad number");
            return false;
        }
    }

    bool SkipValue()
    {
        Skip();
        if (m_i >= m_s.size())
        {
            Fail("EOF in value");
            return false;
        }
        char c = m_s[m_i];
        if (c == '"')
        {
            std::string s;
            return ReadString(s);
        }
        if (c == '{')
        {
            return SkipObject();
        }
        if (c == '[')
        {
            return SkipArray();
        }
        if (c == 't' || c == 'f' || c == 'n')
        {
            // true / false / null literal
            while (m_i < m_s.size() && m_s[m_i] >= 'a' && m_s[m_i] <= 'z')
            {
                ++m_i;
            }
            return true;
        }
        double d;
        return ReadNumber(d);
    }

    bool SkipObject()
    {
        if (!Expect('{'))
        {
            return false;
        }
        if (Eat('}'))
        {
            return true;
        }
        do
        {
            std::string k;
            if (!ReadString(k))
            {
                return false;
            }
            if (!Expect(':'))
            {
                return false;
            }
            if (!SkipValue())
            {
                return false;
            }
        } while (Eat(','));
        return Expect('}');
    }

    bool SkipArray()
    {
        if (!Expect('['))
        {
            return false;
        }
        if (Eat(']'))
        {
            return true;
        }
        do
        {
            if (!SkipValue())
            {
                return false;
            }
        } while (Eat(','));
        return Expect(']');
    }

    bool HasError() const { return !m_err.empty(); }

    const std::string& Error() const { return m_err; }

  private:
    void Fail(const std::string& msg)
    {
        if (m_err.empty())
        {
            std::ostringstream os;
            os << msg << " at offset " << m_i;
            m_err = os.str();
        }
    }

    const std::string& m_s;
    size_t m_i;
    std::string m_err;
};

} // namespace

NtnReproManifest::NtnReproManifest()
    : m_createdUtc(NowIso8601Utc())
{
}

NtnReproManifest&
NtnReproManifest::SetToolkitGitSha(const std::string& s)
{
    m_gitSha = s;
    return *this;
}

NtnReproManifest&
NtnReproManifest::SetNs3Version(const std::string& v)
{
    m_ns3Version = v;
    return *this;
}

NtnReproManifest&
NtnReproManifest::SetScenarioName(const std::string& n)
{
    m_scenarioName = n;
    return *this;
}

NtnReproManifest&
NtnReproManifest::SetScenarioDuration(double s)
{
    m_scenarioDuration = s;
    return *this;
}

NtnReproManifest&
NtnReproManifest::SetRng(uint32_t seed, uint32_t run)
{
    m_rngSeed = seed;
    m_rngRun = run;
    return *this;
}

NtnReproManifest&
NtnReproManifest::SetTleEpoch(const std::string& utc)
{
    m_tleEpoch = utc;
    return *this;
}

NtnReproManifest&
NtnReproManifest::AddNoradId(uint32_t id)
{
    m_noradIds.push_back(id);
    return *this;
}

NtnReproManifest&
NtnReproManifest::SetConstellation(const std::string& name,
                                   uint32_t planes,
                                   uint32_t satsPerPlane,
                                   double altKm,
                                   double incDeg)
{
    m_hasConstellation = true;
    m_constName = name;
    m_planes = planes;
    m_satsPerPlane = satsPerPlane;
    m_altKm = altKm;
    m_incDeg = incDeg;
    return *this;
}

NtnReproManifest&
NtnReproManifest::SetSionna(const std::string& rt, const std::string& sceneSha)
{
    m_sionnaRt = rt;
    m_sionnaSceneSha = sceneSha;
    return *this;
}

NtnReproManifest&
NtnReproManifest::SetHitranRelease(const std::string& t)
{
    m_hitran = t;
    return *this;
}

NtnReproManifest&
NtnReproManifest::SetItuRpyVersion(const std::string& v)
{
    m_ituRpy = v;
    return *this;
}

NtnReproManifest&
NtnReproManifest::AddServiceModel(const std::string& n, const std::string& v)
{
    m_serviceModels.emplace_back(n, v);
    return *this;
}

NtnReproManifest&
NtnReproManifest::SetCliArgv(int argc, char** argv)
{
    m_cliArgv.clear();
    if (argc > 0)
    {
        m_cliArgv.reserve(static_cast<size_t>(argc));
    }
    for (int i = 0; i < argc; ++i)
    {
        m_cliArgv.emplace_back(argv[i] ? argv[i] : "");
    }
    return *this;
}

NtnReproManifest&
NtnReproManifest::AddExtra(const std::string& k, const std::string& v)
{
    m_extras.emplace_back(k, v);
    return *this;
}

std::string
NtnReproManifest::ToJson() const
{
    std::ostringstream os;
    os.precision(12);
    os << "{\n";
    os << "  \"schema\": ";
    JsonEscape(os, kSchemaName);
    os << ",\n";
    os << "  \"schema_version\": " << kSchemaVersion << ",\n";
    os << "  \"toolkit_git_sha\": ";
    JsonEscape(os, m_gitSha);
    os << ",\n";
    os << "  \"ns3_version\": ";
    JsonEscape(os, m_ns3Version);
    os << ",\n";
    os << "  \"created_utc\": ";
    JsonEscape(os, m_createdUtc);
    os << ",\n";
    os << "  \"scenario\": {\n";
    os << "    \"name\": ";
    JsonEscape(os, m_scenarioName);
    os << ",\n";
    os << "    \"duration_s\": " << m_scenarioDuration << ",\n";
    os << "    \"rng_seed\": " << m_rngSeed << ",\n";
    os << "    \"rng_run\": " << m_rngRun << "\n";
    os << "  },\n";
    os << "  \"tle\": {\n";
    os << "    \"epoch_utc\": ";
    JsonEscape(os, m_tleEpoch);
    os << ",\n";
    os << "    \"norad_ids\": [";
    for (size_t i = 0; i < m_noradIds.size(); ++i)
    {
        if (i)
        {
            os << ", ";
        }
        os << m_noradIds[i];
    }
    os << "]\n";
    os << "  },\n";
    if (m_hasConstellation)
    {
        os << "  \"constellation\": {\n";
        os << "    \"name\": ";
        JsonEscape(os, m_constName);
        os << ",\n";
        os << "    \"planes\": " << m_planes << ",\n";
        os << "    \"sats_per_plane\": " << m_satsPerPlane << ",\n";
        os << "    \"altitude_km\": " << m_altKm << ",\n";
        os << "    \"inclination_deg\": " << m_incDeg << "\n";
        os << "  },\n";
    }
    os << "  \"sionna\": {\n";
    os << "    \"rt_version\": ";
    JsonEscape(os, m_sionnaRt);
    os << ",\n";
    os << "    \"scene_sha256\": ";
    JsonEscape(os, m_sionnaSceneSha);
    os << "\n";
    os << "  },\n";
    os << "  \"hitran\": { \"release\": ";
    JsonEscape(os, m_hitran);
    os << " },\n";
    os << "  \"itu_rpy\": { \"version\": ";
    JsonEscape(os, m_ituRpy);
    os << " },\n";
    os << "  \"service_models\": [";
    for (size_t i = 0; i < m_serviceModels.size(); ++i)
    {
        if (i)
        {
            os << ",";
        }
        os << "\n    { \"name\": ";
        JsonEscape(os, m_serviceModels[i].first);
        os << ", \"version\": ";
        JsonEscape(os, m_serviceModels[i].second);
        os << " }";
    }
    if (!m_serviceModels.empty())
    {
        os << "\n  ";
    }
    os << "],\n";
    os << "  \"cli_argv\": [";
    for (size_t i = 0; i < m_cliArgv.size(); ++i)
    {
        if (i)
        {
            os << ",";
        }
        os << "\n    ";
        JsonEscape(os, m_cliArgv[i]);
    }
    if (!m_cliArgv.empty())
    {
        os << "\n  ";
    }
    os << "],\n";
    os << "  \"extras\": {";
    for (size_t i = 0; i < m_extras.size(); ++i)
    {
        if (i)
        {
            os << ",";
        }
        os << "\n    ";
        JsonEscape(os, m_extras[i].first);
        os << ": ";
        JsonEscape(os, m_extras[i].second);
    }
    if (!m_extras.empty())
    {
        os << "\n  ";
    }
    os << "}\n";
    os << "}\n";
    return os.str();
}

bool
NtnReproManifest::WriteJson(const std::string& path) const
{
    const std::string body = ToJson();
    const std::string tmp = path + ".tmp";
    {
        std::ofstream f(tmp);
        if (!f)
        {
            return false;
        }
        f << body;
        if (!f)
        {
            return false;
        }
    }
    if (std::rename(tmp.c_str(), path.c_str()) != 0)
    {
        std::remove(tmp.c_str());
        return false;
    }
    return true;
}

bool
NtnReproManifest::LoadJson(const std::string& path, NtnReproManifest& out)
{
    std::ifstream f(path);
    if (!f)
    {
        return false;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return ParseJson(ss.str(), out);
}

bool
NtnReproManifest::ParseJson(const std::string& body, NtnReproManifest& out)
{
    JsonReader r(body);
    if (!r.Expect('{'))
    {
        return false;
    }
    if (r.Eat('}'))
    {
        return true;
    }
    do
    {
        std::string key;
        if (!r.ReadString(key))
        {
            return false;
        }
        if (!r.Expect(':'))
        {
            return false;
        }

        if (key == "schema")
        {
            std::string s;
            if (!r.ReadString(s))
            {
                return false;
            }
        }
        else if (key == "schema_version")
        {
            double d;
            if (!r.ReadNumber(d))
            {
                return false;
            }
        }
        else if (key == "toolkit_git_sha")
        {
            if (!r.ReadString(out.m_gitSha))
            {
                return false;
            }
        }
        else if (key == "ns3_version")
        {
            if (!r.ReadString(out.m_ns3Version))
            {
                return false;
            }
        }
        else if (key == "created_utc")
        {
            if (!r.ReadString(out.m_createdUtc))
            {
                return false;
            }
        }
        else if (key == "scenario")
        {
            if (!r.Expect('{'))
            {
                return false;
            }
            if (!r.Eat('}'))
            {
                do
                {
                    std::string k2;
                    if (!r.ReadString(k2))
                    {
                        return false;
                    }
                    if (!r.Expect(':'))
                    {
                        return false;
                    }
                    if (k2 == "name")
                    {
                        if (!r.ReadString(out.m_scenarioName))
                        {
                            return false;
                        }
                    }
                    else if (k2 == "duration_s")
                    {
                        if (!r.ReadNumber(out.m_scenarioDuration))
                        {
                            return false;
                        }
                    }
                    else if (k2 == "rng_seed")
                    {
                        double d;
                        if (!r.ReadNumber(d))
                        {
                            return false;
                        }
                        out.m_rngSeed = static_cast<uint32_t>(d);
                    }
                    else if (k2 == "rng_run")
                    {
                        double d;
                        if (!r.ReadNumber(d))
                        {
                            return false;
                        }
                        out.m_rngRun = static_cast<uint32_t>(d);
                    }
                    else
                    {
                        if (!r.SkipValue())
                        {
                            return false;
                        }
                    }
                } while (r.Eat(','));
                if (!r.Expect('}'))
                {
                    return false;
                }
            }
        }
        else if (key == "tle")
        {
            if (!r.Expect('{'))
            {
                return false;
            }
            if (!r.Eat('}'))
            {
                do
                {
                    std::string k2;
                    if (!r.ReadString(k2))
                    {
                        return false;
                    }
                    if (!r.Expect(':'))
                    {
                        return false;
                    }
                    if (k2 == "epoch_utc")
                    {
                        if (!r.ReadString(out.m_tleEpoch))
                        {
                            return false;
                        }
                    }
                    else if (k2 == "norad_ids")
                    {
                        if (!r.Expect('['))
                        {
                            return false;
                        }
                        if (!r.Eat(']'))
                        {
                            do
                            {
                                double d;
                                if (!r.ReadNumber(d))
                                {
                                    return false;
                                }
                                out.m_noradIds.push_back(static_cast<uint32_t>(d));
                            } while (r.Eat(','));
                            if (!r.Expect(']'))
                            {
                                return false;
                            }
                        }
                    }
                    else
                    {
                        if (!r.SkipValue())
                        {
                            return false;
                        }
                    }
                } while (r.Eat(','));
                if (!r.Expect('}'))
                {
                    return false;
                }
            }
        }
        else if (key == "constellation")
        {
            out.m_hasConstellation = true;
            if (!r.Expect('{'))
            {
                return false;
            }
            if (!r.Eat('}'))
            {
                do
                {
                    std::string k2;
                    if (!r.ReadString(k2))
                    {
                        return false;
                    }
                    if (!r.Expect(':'))
                    {
                        return false;
                    }
                    if (k2 == "name")
                    {
                        if (!r.ReadString(out.m_constName))
                        {
                            return false;
                        }
                    }
                    else if (k2 == "planes")
                    {
                        double d;
                        if (!r.ReadNumber(d))
                        {
                            return false;
                        }
                        out.m_planes = static_cast<uint32_t>(d);
                    }
                    else if (k2 == "sats_per_plane")
                    {
                        double d;
                        if (!r.ReadNumber(d))
                        {
                            return false;
                        }
                        out.m_satsPerPlane = static_cast<uint32_t>(d);
                    }
                    else if (k2 == "altitude_km")
                    {
                        if (!r.ReadNumber(out.m_altKm))
                        {
                            return false;
                        }
                    }
                    else if (k2 == "inclination_deg")
                    {
                        if (!r.ReadNumber(out.m_incDeg))
                        {
                            return false;
                        }
                    }
                    else
                    {
                        if (!r.SkipValue())
                        {
                            return false;
                        }
                    }
                } while (r.Eat(','));
                if (!r.Expect('}'))
                {
                    return false;
                }
            }
        }
        else if (key == "sionna")
        {
            if (!r.Expect('{'))
            {
                return false;
            }
            if (!r.Eat('}'))
            {
                do
                {
                    std::string k2;
                    if (!r.ReadString(k2))
                    {
                        return false;
                    }
                    if (!r.Expect(':'))
                    {
                        return false;
                    }
                    if (k2 == "rt_version")
                    {
                        if (!r.ReadString(out.m_sionnaRt))
                        {
                            return false;
                        }
                    }
                    else if (k2 == "scene_sha256")
                    {
                        if (!r.ReadString(out.m_sionnaSceneSha))
                        {
                            return false;
                        }
                    }
                    else
                    {
                        if (!r.SkipValue())
                        {
                            return false;
                        }
                    }
                } while (r.Eat(','));
                if (!r.Expect('}'))
                {
                    return false;
                }
            }
        }
        else if (key == "hitran")
        {
            if (!r.Expect('{'))
            {
                return false;
            }
            if (!r.Eat('}'))
            {
                do
                {
                    std::string k2;
                    if (!r.ReadString(k2))
                    {
                        return false;
                    }
                    if (!r.Expect(':'))
                    {
                        return false;
                    }
                    if (k2 == "release")
                    {
                        if (!r.ReadString(out.m_hitran))
                        {
                            return false;
                        }
                    }
                    else
                    {
                        if (!r.SkipValue())
                        {
                            return false;
                        }
                    }
                } while (r.Eat(','));
                if (!r.Expect('}'))
                {
                    return false;
                }
            }
        }
        else if (key == "itu_rpy")
        {
            if (!r.Expect('{'))
            {
                return false;
            }
            if (!r.Eat('}'))
            {
                do
                {
                    std::string k2;
                    if (!r.ReadString(k2))
                    {
                        return false;
                    }
                    if (!r.Expect(':'))
                    {
                        return false;
                    }
                    if (k2 == "version")
                    {
                        if (!r.ReadString(out.m_ituRpy))
                        {
                            return false;
                        }
                    }
                    else
                    {
                        if (!r.SkipValue())
                        {
                            return false;
                        }
                    }
                } while (r.Eat(','));
                if (!r.Expect('}'))
                {
                    return false;
                }
            }
        }
        else if (key == "service_models")
        {
            if (!r.Expect('['))
            {
                return false;
            }
            if (!r.Eat(']'))
            {
                do
                {
                    if (!r.Expect('{'))
                    {
                        return false;
                    }
                    std::string n;
                    std::string v;
                    if (!r.Eat('}'))
                    {
                        do
                        {
                            std::string k2;
                            if (!r.ReadString(k2))
                            {
                                return false;
                            }
                            if (!r.Expect(':'))
                            {
                                return false;
                            }
                            if (k2 == "name")
                            {
                                if (!r.ReadString(n))
                                {
                                    return false;
                                }
                            }
                            else if (k2 == "version")
                            {
                                if (!r.ReadString(v))
                                {
                                    return false;
                                }
                            }
                            else
                            {
                                if (!r.SkipValue())
                                {
                                    return false;
                                }
                            }
                        } while (r.Eat(','));
                        if (!r.Expect('}'))
                        {
                            return false;
                        }
                    }
                    out.m_serviceModels.emplace_back(n, v);
                } while (r.Eat(','));
                if (!r.Expect(']'))
                {
                    return false;
                }
            }
        }
        else if (key == "cli_argv")
        {
            if (!r.Expect('['))
            {
                return false;
            }
            if (!r.Eat(']'))
            {
                do
                {
                    std::string a;
                    if (!r.ReadString(a))
                    {
                        return false;
                    }
                    out.m_cliArgv.push_back(std::move(a));
                } while (r.Eat(','));
                if (!r.Expect(']'))
                {
                    return false;
                }
            }
        }
        else if (key == "extras")
        {
            if (!r.Expect('{'))
            {
                return false;
            }
            if (!r.Eat('}'))
            {
                do
                {
                    std::string k2;
                    std::string val;
                    if (!r.ReadString(k2))
                    {
                        return false;
                    }
                    if (!r.Expect(':'))
                    {
                        return false;
                    }
                    if (!r.ReadString(val))
                    {
                        return false;
                    }
                    out.m_extras.emplace_back(std::move(k2), std::move(val));
                } while (r.Eat(','));
                if (!r.Expect('}'))
                {
                    return false;
                }
            }
        }
        else
        {
            // Unknown top-level key; skip value for forward compatibility.
            if (!r.SkipValue())
            {
                return false;
            }
        }
    } while (r.Eat(','));

    if (!r.Expect('}'))
    {
        return false;
    }
    if (r.HasError())
    {
        NS_LOG_WARN("Manifest parse error: " << r.Error());
        return false;
    }
    return true;
}

} // namespace ntnobs
} // namespace ns3
