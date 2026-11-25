// Copyright (c) 2011-2013 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors:
//   Jaume Nin <jnin@cttc.es>
//   Nicola Baldo <nbaldo@cttc.es>
//   Manuel Requena <manuel.requena@cttc.es>

#include "nr-epc-helper.h"

#include "ns3/abort.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv6-address.h"
#include "ns3/log.h"
#include "ns3/node.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("nrEpcHelper");

NS_OBJECT_ENSURE_REGISTERED(NrEpcHelper);

NrEpcHelper::NrEpcHelper()
{
    NS_LOG_FUNCTION(this);
}

NrEpcHelper::~NrEpcHelper()
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrEpcHelper::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrEpcHelper").SetParent<Object>().SetGroupName("Nr");
    return tid;
}

void
NrEpcHelper::DoDispose()
{
    NS_LOG_FUNCTION(this);
    Object::DoDispose();
}

uint8_t
NrEpcHelper::ActivateUnstructuredQosFlow(Ptr<NetDevice> ueNrDevice,
                                         uint64_t imsi,
                                         uint16_t protocolNumber,
                                         NrQosFlow flow)
{
    NS_LOG_FUNCTION(this << ueNrDevice << imsi << protocolNumber);
    NS_ABORT_MSG("Unstructured PDU sessions are not supported by this EPC helper");
}

Ptr<VirtualNetDevice>
NrEpcHelper::GetUnstructuredSessionDevice(uint64_t imsi, uint8_t qfi) const
{
    NS_LOG_FUNCTION(this << imsi << qfi);
    NS_ABORT_MSG("Unstructured PDU sessions are not supported by this EPC helper");
}

} // namespace ns3
