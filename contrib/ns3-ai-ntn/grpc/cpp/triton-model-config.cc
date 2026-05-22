/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "triton-model-config.h"

#include <cctype>
#include <fstream>
#include <sstream>

namespace ns3
{
namespace oranntn
{
namespace airan
{

namespace
{

/// Strip line comments (`# ...`) and surrounding whitespace from each
/// line. Returns the joined "code-only" form.
std::string
StripComments(const std::string& src)
{
    std::ostringstream out;
    std::istringstream is(src);
    std::string line;
    while (std::getline(is, line))
    {
        const size_t hash = line.find('#');
        if (hash != std::string::npos)
        {
            line = line.substr(0, hash);
        }
        out << line << '\n';
    }
    return out.str();
}

/// Cursor over the pbtxt body. Returns logical tokens.
struct Lexer
{
    const std::string& s;
    size_t i = 0;

    explicit Lexer(const std::string& src) : s(src) {}

    void SkipWs()
    {
        while (i < s.size() &&
               (std::isspace(static_cast<unsigned char>(s[i])) ||
                s[i] == ','))
        {
            ++i;
        }
    }

    bool Eof()
    {
        SkipWs();
        return i >= s.size();
    }

    char Peek()
    {
        SkipWs();
        return (i < s.size()) ? s[i] : '\0';
    }

    /// Read an identifier (letters/digits/underscore).
    std::string ReadIdent()
    {
        SkipWs();
        const size_t start = i;
        while (i < s.size() &&
               (std::isalnum(static_cast<unsigned char>(s[i])) ||
                s[i] == '_'))
        {
            ++i;
        }
        return s.substr(start, i - start);
    }

    /// Read a quoted string. The opening quote must be next.
    bool ReadString(std::string& out)
    {
        SkipWs();
        if (i >= s.size() || s[i] != '"')
        {
            return false;
        }
        ++i;
        out.clear();
        while (i < s.size() && s[i] != '"')
        {
            if (s[i] == '\\' && i + 1 < s.size())
            {
                out.push_back(s[i + 1]);
                i += 2;
            }
            else
            {
                out.push_back(s[i++]);
            }
        }
        if (i >= s.size())
        {
            return false; // unterminated
        }
        ++i; // closing quote
        return true;
    }

    /// Read a bare number (integer / float, possibly negative).
    std::string ReadNumber()
    {
        SkipWs();
        const size_t start = i;
        if (i < s.size() && (s[i] == '-' || s[i] == '+'))
        {
            ++i;
        }
        while (i < s.size() &&
               (std::isdigit(static_cast<unsigned char>(s[i])) ||
                s[i] == '.' || s[i] == 'e' || s[i] == 'E' ||
                s[i] == '-' || s[i] == '+'))
        {
            ++i;
        }
        return s.substr(start, i - start);
    }

    /// Consume a single character literal if present.
    bool Consume(char ch)
    {
        SkipWs();
        if (i < s.size() && s[i] == ch)
        {
            ++i;
            return true;
        }
        return false;
    }

    /// Skip a balanced block starting with the next '{'. Used when we
    /// encounter a section we don't model (e.g. `input { ... }`).
    bool SkipBlock()
    {
        SkipWs();
        if (i >= s.size() || s[i] != '{')
        {
            return false;
        }
        int depth = 0;
        while (i < s.size())
        {
            const char c = s[i];
            if (c == '"')
            {
                std::string ignored;
                if (!ReadString(ignored))
                {
                    return false;
                }
                continue;
            }
            if (c == '{')
            {
                ++depth;
            }
            else if (c == '}')
            {
                --depth;
                if (depth == 0)
                {
                    ++i;
                    return true;
                }
            }
            ++i;
        }
        return false;
    }

    /// Skip a list literal `[ ... ]`.
    bool SkipList()
    {
        SkipWs();
        if (i >= s.size() || s[i] != '[')
        {
            return false;
        }
        ++i;
        int depth = 1;
        while (i < s.size() && depth > 0)
        {
            const char c = s[i];
            if (c == '"')
            {
                std::string ignored;
                if (!ReadString(ignored))
                {
                    return false;
                }
                continue;
            }
            if (c == '[')
            {
                ++depth;
            }
            else if (c == ']')
            {
                --depth;
                if (depth == 0)
                {
                    ++i;
                    return true;
                }
            }
            ++i;
        }
        return false;
    }
};

/// Parse a single `parameters { key: "..." value { ... } }` block,
/// stashing the discovered key/value into the config.
bool
ParseParameter(Lexer& lex, TritonModelConfig& cfg)
{
    if (!lex.Consume('{'))
    {
        return false;
    }
    std::string key;
    std::string val;
    while (!lex.Eof() && lex.Peek() != '}')
    {
        const std::string field = lex.ReadIdent();
        if (field.empty())
        {
            return false;
        }
        lex.Consume(':');
        if (field == "key")
        {
            if (!lex.ReadString(key))
            {
                return false;
            }
        }
        else if (field == "value")
        {
            if (!lex.Consume('{'))
            {
                return false;
            }
            while (!lex.Eof() && lex.Peek() != '}')
            {
                const std::string vfield = lex.ReadIdent();
                if (vfield.empty())
                {
                    return false;
                }
                lex.Consume(':');
                if (vfield == "string_value")
                {
                    if (!lex.ReadString(val))
                    {
                        return false;
                    }
                }
                else
                {
                    // numeric value / enum
                    val = lex.ReadNumber();
                }
            }
            if (!lex.Consume('}'))
            {
                return false;
            }
        }
        else
        {
            // unknown field — skip token
            (void)lex.ReadNumber();
        }
    }
    if (!lex.Consume('}'))
    {
        return false;
    }
    if (key.empty())
    {
        return true;
    }
    cfg.params[key] = val;
    if (key == "TOOLKIT_OUTPUT_FIELD")
    {
        if (val == "precoder")
        {
            cfg.output_field = OutputField::precoder;
        }
        else if (val == "beam")
        {
            cfg.output_field = OutputField::beam;
        }
    }
    else if (key == "INFERENCE_BUDGET_US")
    {
        try
        {
            cfg.inference_budget_us =
                static_cast<uint64_t>(std::stoull(val));
        }
        catch (...)
        {
        }
    }
    return true;
}

/// Recursively descend into a `dynamic_batching { ... }` block,
/// extracting just `max_queue_delay_microseconds`.
bool
ParseDynamicBatching(Lexer& lex, TritonModelConfig& cfg)
{
    if (!lex.Consume('{'))
    {
        return false;
    }
    while (!lex.Eof() && lex.Peek() != '}')
    {
        const std::string field = lex.ReadIdent();
        if (field.empty())
        {
            return false;
        }
        lex.Consume(':');
        if (field == "max_queue_delay_microseconds")
        {
            const std::string num = lex.ReadNumber();
            try
            {
                cfg.max_queue_delay_us =
                    static_cast<uint64_t>(std::stoull(num));
            }
            catch (...)
            {
            }
        }
        else if (lex.Peek() == '{')
        {
            if (!lex.SkipBlock())
            {
                return false;
            }
        }
        else if (lex.Peek() == '[')
        {
            if (!lex.SkipList())
            {
                return false;
            }
        }
        else if (lex.Peek() == '"')
        {
            std::string ignored;
            if (!lex.ReadString(ignored))
            {
                return false;
            }
        }
        else
        {
            (void)lex.ReadNumber();
        }
    }
    return lex.Consume('}');
}

} // namespace

std::optional<TritonModelConfig>
TritonModelConfigParser::Parse(const std::string& pbtxt)
{
    const std::string body = StripComments(pbtxt);
    Lexer lex(body);
    TritonModelConfig cfg;
    while (!lex.Eof())
    {
        const std::string field = lex.ReadIdent();
        if (field.empty())
        {
            break;
        }
        lex.Consume(':');
        if (field == "name")
        {
            if (!lex.ReadString(cfg.name))
            {
                return std::nullopt;
            }
        }
        else if (field == "platform")
        {
            if (!lex.ReadString(cfg.platform))
            {
                return std::nullopt;
            }
        }
        else if (field == "default_model_filename")
        {
            if (!lex.ReadString(cfg.default_model_filename))
            {
                return std::nullopt;
            }
        }
        else if (field == "max_batch_size")
        {
            const std::string num = lex.ReadNumber();
            try
            {
                cfg.max_batch_size =
                    static_cast<uint32_t>(std::stoul(num));
            }
            catch (...)
            {
                return std::nullopt;
            }
        }
        else if (field == "parameters")
        {
            if (!ParseParameter(lex, cfg))
            {
                return std::nullopt;
            }
        }
        else if (field == "dynamic_batching")
        {
            if (!ParseDynamicBatching(lex, cfg))
            {
                return std::nullopt;
            }
        }
        else
        {
            // Unknown top-level field: skip its body shape.
            const char next = lex.Peek();
            if (next == '{')
            {
                if (!lex.SkipBlock())
                {
                    return std::nullopt;
                }
            }
            else if (next == '[')
            {
                if (!lex.SkipList())
                {
                    return std::nullopt;
                }
            }
            else if (next == '"')
            {
                std::string ignored;
                if (!lex.ReadString(ignored))
                {
                    return std::nullopt;
                }
            }
            else
            {
                (void)lex.ReadNumber();
            }
        }
    }
    if (cfg.name.empty())
    {
        return std::nullopt;
    }
    return cfg;
}

std::optional<TritonModelConfig>
TritonModelConfigParser::LoadFile(const std::string& path)
{
    std::ifstream in(path);
    if (!in)
    {
        return std::nullopt;
    }
    std::stringstream buf;
    buf << in.rdbuf();
    return Parse(buf.str());
}

} // namespace airan
} // namespace oranntn
} // namespace ns3
