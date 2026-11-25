// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_DEVICE_REGISTRY_H
#define NR_DEVICE_REGISTRY_H

#include "nr-gnb-net-device.h"
#include "nr-ue-net-device.h"

#include "ns3/nr-export.h"

namespace ns3
{

class NR_EXPORT NrDeviceRegistry
{
  public:
    /**
     * Configures the NrUeNetDevice::m_targetGnb field with a pointer to the attached gNB,
     * which can be used for logging, RRC, beamforming management, etc.
     *
     * @param cellId The cell ID identifying the specific gNB.
     * @param imsi The IMSI/Node ID of the UE to set the target gNB.
     */
    static void SetUeTargetCell(uint16_t cellId, uint64_t imsi);
    /**
     * Retrieve the gNB NetDevice associated with the given cellId.
     *
     * This function searches through all nodes and their devices to locate
     * a gNB Net Device with a corresponding cell identity. If a match is
     * found, the gNB Net Device is returned. If no matching device is found,
     * a null pointer is returned.
     *
     * @param cellId The cell ID to search for.
     * @return A smart pointer to the matching NrGnbNetDevice.
     */
    static Ptr<NrGnbNetDevice> GetGnbNetDevFromCellId(uint16_t cellId);

    /**
     * @ingroup helper
     * @brief Retrieves the network device corresponding to the given IMSI from the UE list.
     *
     * This method allows users to find the User Equipment (UE) network device
     * associated with a specific International Mobile Subscriber Identity (IMSI).
     * IMSI is a unique identifier tied to each subscriber in the network.
     *
     * The function searches through the stored UE network devices and matching IMSI
     * values to return the corresponding network device.
     *
     * It is expected that the input IMSI already exists in the list of UE devices;
     * otherwise, the result will not be valid. Ensure that UEs have been properly
     * initialized and configured before calling this method.
     *
     * @param imsi The International Mobile Subscriber Identity of the UE.
     * @return A pointer to the network device corresponding to the provided IMSI.
     */
    static Ptr<NrUeNetDevice> GetUeNetDevFromImsi(uint64_t imsi);
};

} // namespace ns3
#endif // NR_DEVICE_REGISTRY_H
