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

#include "address-double-probe.h"

#include "ns3/callback.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/simulator.h"

#include <string>

NS_LOG_COMPONENT_DEFINE("AddressDoubleProbe");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(AddressDoubleProbe);

TypeId
AddressDoubleProbe::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::AddressDoubleProbe")
            .SetParent<Probe>()
            .AddConstructor<AddressDoubleProbe>()
            .AddTraceSource(
                "Output",
                "The delay plus its socket address that serve as the output for this probe",
                MakeTraceSourceAccessor(&AddressDoubleProbe::m_output),
                "ns3::AddressDoubleProbe::DoubleAddressCallback")
            .AddTraceSource("OutputDouble",
                            "The double of the traced packet",
                            MakeTraceSourceAccessor(&AddressDoubleProbe::m_outputDouble),
                            "ns3::AddressDoubleProbe::DoubleCallback");
    return tid;
}

AddressDoubleProbe::AddressDoubleProbe()
{
    NS_LOG_FUNCTION(this);
    m_doubleValue = 0.0;
}

AddressDoubleProbe::~AddressDoubleProbe()
{
    NS_LOG_FUNCTION(this);
}

void
AddressDoubleProbe::SetValue(double doubleValue, const Address& address)
{
    NS_LOG_FUNCTION(this << doubleValue << address);

    m_output(doubleValue, address);
    m_outputDouble(m_doubleValue, doubleValue);
    m_doubleValue = doubleValue;
    m_address = address;
}

void
AddressDoubleProbe::SetValueByPath(std::string path, double doubleValue, const Address& address)
{
    NS_LOG_FUNCTION(path << doubleValue << address);
    Ptr<AddressDoubleProbe> probe = Names::Find<AddressDoubleProbe>(path);
    NS_ASSERT_MSG(probe, "Error:  Can't find probe for path " << path);
    probe->SetValue(doubleValue, address);
}

bool
AddressDoubleProbe::ConnectByObject(std::string traceSource, Ptr<Object> obj)
{
    NS_LOG_FUNCTION(this << traceSource << obj);
    NS_LOG_DEBUG("Name of probe (if any) in names database: " << Names::FindPath(obj));
    bool connected =
        obj->TraceConnectWithoutContext(traceSource,
                                        MakeCallback(&ns3::AddressDoubleProbe::TraceSink, this));
    return connected;
}

void
AddressDoubleProbe::ConnectByPath(std::string path)
{
    NS_LOG_FUNCTION(this << path);
    NS_LOG_DEBUG("Name of probe to search for in config database: " << path);
    Config::ConnectWithoutContext(path, MakeCallback(&ns3::AddressDoubleProbe::TraceSink, this));
}

void
AddressDoubleProbe::TraceSink(double doubleValue, const Address& address)
{
    NS_LOG_FUNCTION(this << doubleValue << address);
    if (IsEnabled())
    {
        m_output(doubleValue, address);
        m_outputDouble(m_doubleValue, doubleValue);
        m_doubleValue = doubleValue;
        m_address = address;
    }
}

} // namespace ns3
