/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "fapi-helpers.h"

namespace ns3
{
namespace fapi
{

std::vector<uint8_t>
DmrsFapiToBitArray(uint16_t dmrsSymbPos)
{
    std::vector<uint8_t> out;
    out.reserve(14);
    for (uint8_t s = 0; s < 14; ++s)
    {
        if (dmrsSymbPos & (uint16_t{1} << s))
        {
            out.push_back(s);
        }
    }
    return out;
}

uint16_t
DmrsBitArrayToFapi(const std::vector<uint8_t>& symbolIndices)
{
    uint16_t mask = 0;
    for (uint8_t s : symbolIndices)
    {
        if (s < 14)
        {
            mask |= static_cast<uint16_t>(uint16_t{1} << s);
        }
    }
    return mask;
}

} // namespace fapi
} // namespace ns3
