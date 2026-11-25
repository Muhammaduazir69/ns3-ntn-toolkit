// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-device-registry.h"

#include "ns3/node-list.h"
#include "ns3/node.h"

using namespace ns3;

Ptr<NrGnbNetDevice>
NrDeviceRegistry::GetGnbNetDevFromCellId(uint16_t cellId)
{
    // Search nodes for gNB PHY/MAC with corresponding cellId
    Ptr<NrGnbNetDevice> gnbNet;

    for (std::size_t nodeI = 0; nodeI < NodeList::GetNNodes(); nodeI++)
    {
        auto node = NodeList::GetNode(nodeI);
        for (std::size_t deviceI = 0; deviceI < node->GetNDevices(); deviceI++)
        {
            auto device = node->GetDevice(deviceI);
            gnbNet = DynamicCast<NrGnbNetDevice>(device);
            if (gnbNet && gnbNet->GetCellId() == cellId)
            {
                return gnbNet;
            }
        }
    }
    NS_ABORT_MSG_IF(!gnbNet, "No gNB with cellId " << cellId);
    return nullptr;
}

Ptr<NrUeNetDevice>
NrDeviceRegistry::GetUeNetDevFromImsi(uint64_t imsi)
{
    // Search nodes for UE PHY/MAC with corresponding cellId
    Ptr<NrUeNetDevice> ueNet;

    for (std::size_t nodeI = 0; nodeI < NodeList::GetNNodes(); nodeI++)
    {
        auto node = NodeList::GetNode(nodeI);
        for (std::size_t deviceI = 0; deviceI < node->GetNDevices(); deviceI++)
        {
            auto device = node->GetDevice(deviceI);
            ueNet = DynamicCast<NrUeNetDevice>(device);
            if (ueNet && ueNet->GetImsi() == imsi)
            {
                return ueNet;
            }
        }
    }
    NS_ABORT_MSG_IF(!ueNet, "No UE with IMSI " << imsi);
    return nullptr;
}

void
NrDeviceRegistry::SetUeTargetCell(uint16_t cellId, uint64_t imsi)
{
    auto gnbNet = GetGnbNetDevFromCellId(cellId);
    auto ueNet = GetUeNetDevFromImsi(imsi);
    ueNet->SetTargetGnb(gnbNet); // not standard
}
