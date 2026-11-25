/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Manuel Requena <manuel.requena@cttc.es>
 */

/**
 * @ingroup test
 * @file nr-simple-net-device.h
 *
 * @brief Test support net device, not a test suite. NrSimpleNetDevice subclasses SimpleNetDevice
 * to stand in for the real NR net device where only a packet delivery service between protocol
 * stacks is needed: it forwards Send() to SimpleNetDevice so PDUs travel over a SimpleChannel
 * (optionally dropped by a receive error model) instead of the NR PHY/MAC. It is instantiated by
 * NrSimpleHelper and used by the RLC UM and AM end-to-end test suites (nr-test-rlc-um-e2e.cc and
 * nr-test-rlc-am-e2e.cc).
 */

#ifndef NR_SIMPLE_NET_DEVICE_H
#define NR_SIMPLE_NET_DEVICE_H

#include "ns3/error-model.h"
#include "ns3/event-id.h"
#include "ns3/node.h"
#include "ns3/nr-rlc.h"
#include "ns3/simple-channel.h"
#include "ns3/simple-net-device.h"

namespace ns3
{

/**
 * @ingroup nr
 * The NrSimpleNetDevice class implements the NR simple net device.
 * This class is used to provide a limited NrNetDevice functionalities that
 * are necessary for testing purposes.
 */
class NrSimpleNetDevice : public SimpleNetDevice
{
  public:
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();

    NrSimpleNetDevice();
    /**
     * Constructor
     *
     * @param node the Node
     */
    NrSimpleNetDevice(Ptr<Node> node);

    ~NrSimpleNetDevice() override;
    void DoDispose() override;

    // inherited from NetDevice
    bool Send(Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber) override;

  protected:
    // inherited from Object
    void DoInitialize() override;
};

} // namespace ns3

#endif // NR_SIMPLE_NET_DEVICE_H
