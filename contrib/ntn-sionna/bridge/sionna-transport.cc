/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "sionna-transport.h"

namespace ns3
{

TypeId
SionnaTransport::GetTypeId()
{
    static TypeId tid = TypeId("ns3::SionnaTransport")
                            .SetParent<Object>()
                            .SetGroupName("NtnSionna");
    return tid;
}

TypeId
SionnaNoneTransport::GetTypeId()
{
    static TypeId tid = TypeId("ns3::SionnaNoneTransport")
                            .SetParent<SionnaTransport>()
                            .SetGroupName("NtnSionna")
                            .AddConstructor<SionnaNoneTransport>();
    return tid;
}

} // namespace ns3
