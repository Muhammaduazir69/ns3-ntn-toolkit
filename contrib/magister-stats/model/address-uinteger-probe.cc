/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Bucknell University
 * Copyright (c) 2014 Magister Solutions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors of original work (application-packet-probe.cc) which this work
 * derives from:
 * - L. Felipe Perrone (perrone@bucknell.edu)
 * - Tiago G. Rodrigues (tgr002@bucknell.edu)
 * - Mitch Watrous (watrous@u.washington.edu)
 *
 * Modified for DrxRxMiss and RxError trace sources by:
 * - Frans Laakso (frans.laakso@magister.fi)
 */

#include "address-uinteger-probe.h"

#include "ns3/callback.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/simulator.h"

#include <string>

NS_LOG_COMPONENT_DEFINE("AddressUintegerProbe");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(AddressUintegerProbe);

TypeId
AddressUintegerProbe::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::AddressUintegerProbe")
            .SetParent<Probe>()
            .AddConstructor<AddressUintegerProbe>()
            .AddTraceSource(
                "Output",
                "The delay plus its socket address that serve as the output for this probe",
                MakeTraceSourceAccessor(&AddressUintegerProbe::m_output),
                "ns3::AddressUintegerProbe::UintegerAddressCallback")
            .AddTraceSource("OutputUinteger",
                            "The Uinteger of the traced packet",
                            MakeTraceSourceAccessor(&AddressUintegerProbe::m_outputUinteger),
                            "ns3::AddressUintegerProbe::UintegerCallback");
    return tid;
}

AddressUintegerProbe::AddressUintegerProbe()
{
    NS_LOG_FUNCTION(this);
    m_uintegerValue = 0;
}

AddressUintegerProbe::~AddressUintegerProbe()
{
    NS_LOG_FUNCTION(this);
}

void
AddressUintegerProbe::SetValue(uint32_t uintegerValue, const Address& address)
{
    NS_LOG_FUNCTION(this << uintegerValue << address);

    m_output(uintegerValue, address);
    m_outputUinteger(m_uintegerValue, uintegerValue);
    m_uintegerValue = uintegerValue;
    m_address = address;
}

void
AddressUintegerProbe::SetValueByPath(std::string path,
                                     uint32_t uintegerValue,
                                     const Address& address)
{
    NS_LOG_FUNCTION(path << uintegerValue << address);
    Ptr<AddressUintegerProbe> probe = Names::Find<AddressUintegerProbe>(path);
    NS_ASSERT_MSG(probe, "Error:  Can't find probe for path " << path);
    probe->SetValue(uintegerValue, address);
}

bool
AddressUintegerProbe::ConnectByObject(std::string traceSource, Ptr<Object> obj)
{
    NS_LOG_FUNCTION(this << traceSource << obj);
    NS_LOG_DEBUG("Name of probe (if any) in names database: " << Names::FindPath(obj));
    bool connected =
        obj->TraceConnectWithoutContext(traceSource,
                                        MakeCallback(&ns3::AddressUintegerProbe::TraceSink, this));
    return connected;
}

void
AddressUintegerProbe::ConnectByPath(std::string path)
{
    NS_LOG_FUNCTION(this << path);
    NS_LOG_DEBUG("Name of probe to search for in config database: " << path);
    Config::ConnectWithoutContext(path, MakeCallback(&ns3::AddressUintegerProbe::TraceSink, this));
}

void
AddressUintegerProbe::TraceSink(uint32_t uintegerValue, const Address& address)
{
    NS_LOG_FUNCTION(this << uintegerValue << address);
    if (IsEnabled())
    {
        m_output(uintegerValue, address);
        m_outputUinteger(m_uintegerValue, uintegerValue);
        m_uintegerValue = uintegerValue;
        m_address = address;
    }
}

} // namespace ns3
