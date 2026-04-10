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
 * Modified by:
 * - Budiarto Herman (budiarto.herman@magister.fi)
 */

#include "address-time-probe.h"

#include "ns3/callback.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/simulator.h"

#include <string>

NS_LOG_COMPONENT_DEFINE("AddressTimeProbe");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(AddressTimeProbe);

TypeId
AddressTimeProbe::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::AddressTimeProbe")
            .SetParent<Probe>()
            .AddConstructor<AddressTimeProbe>()
            .AddTraceSource(
                "Output",
                "The time value plus its socket address that serve as the output for this probe",
                MakeTraceSourceAccessor(&AddressTimeProbe::m_output),
                "ns3::AddressTimeProbe::TimeAddressCallback")
            .AddTraceSource("OutputSeconds",
                            "The time value of the trace",
                            MakeTraceSourceAccessor(&AddressTimeProbe::m_outputSeconds),
                            "ns3::AddressTimeProbe::TimeCallback");
    return tid;
}

AddressTimeProbe::AddressTimeProbe()
{
    NS_LOG_FUNCTION(this);
    m_timeValue = MilliSeconds(0);
}

AddressTimeProbe::~AddressTimeProbe()
{
    NS_LOG_FUNCTION(this);
}

void
AddressTimeProbe::SetValue(Time timeValue, const Address& address)
{
    NS_LOG_FUNCTION(this << timeValue.GetSeconds() << address);

    m_output(timeValue, address);
    m_outputSeconds(m_timeValue.GetSeconds(), timeValue.GetSeconds());
    m_timeValue = timeValue;
    m_address = address;
}

void
AddressTimeProbe::SetValueByPath(std::string path, Time timeValue, const Address& address)
{
    NS_LOG_FUNCTION(path << timeValue.GetSeconds() << address);
    Ptr<AddressTimeProbe> probe = Names::Find<AddressTimeProbe>(path);
    NS_ASSERT_MSG(probe, "Error:  Can't find probe for path " << path);
    probe->SetValue(timeValue, address);
}

bool
AddressTimeProbe::ConnectByObject(std::string traceSource, Ptr<Object> obj)
{
    NS_LOG_FUNCTION(this << traceSource << obj);
    NS_LOG_DEBUG("Name of probe (if any) in names database: " << Names::FindPath(obj));
    bool connected =
        obj->TraceConnectWithoutContext(traceSource,
                                        MakeCallback(&ns3::AddressTimeProbe::TraceSink, this));
    return connected;
}

void
AddressTimeProbe::ConnectByPath(std::string path)
{
    NS_LOG_FUNCTION(this << path);
    NS_LOG_DEBUG("Name of probe to search for in config database: " << path);
    Config::ConnectWithoutContext(path, MakeCallback(&ns3::AddressTimeProbe::TraceSink, this));
}

void
AddressTimeProbe::TraceSink(Time timeValue, const Address& address)
{
    NS_LOG_FUNCTION(this << timeValue.GetSeconds() << address);

    if (IsEnabled())
    {
        m_output(timeValue, address);
        m_outputSeconds(m_timeValue.GetSeconds(), timeValue.GetSeconds());
        m_timeValue = timeValue;
        m_address = address;
    }
}

} // namespace ns3
