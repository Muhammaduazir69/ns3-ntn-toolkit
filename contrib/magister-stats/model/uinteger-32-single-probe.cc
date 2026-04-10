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
 * Author of original work (uinteger-32-probe.cc):
 * - L. Felipe Perrone (perrone@bucknell.edu)
 * - Tiago G. Rodrigues (tgr002@bucknell.edu)
 * - Mitch Watrous (watrous@u.washington.edu)
 *
 * Modified to support trace sources with a single uint32_t argument by:
 * - Budiarto Herman (budiarto.herman@magister.fi)
 */

#include "uinteger-32-single-probe.h"

#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/packet.h"
#include "ns3/trace-source-accessor.h"

#include <string>

NS_LOG_COMPONENT_DEFINE("Uinteger32SingleProbe");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(Uinteger32SingleProbe);

TypeId
Uinteger32SingleProbe::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::Uinteger32SingleProbe")
            .SetParent<Probe>()
            .AddConstructor<Uinteger32SingleProbe>()
            .AddTraceSource("Output",
                            "The uint32_t that serves as output for this probe",
                            MakeTraceSourceAccessor(&Uinteger32SingleProbe::m_output),
                            "ns3::Packet::PacketSizeTracedCallback");
    return tid;
}

Uinteger32SingleProbe::Uinteger32SingleProbe()
    : m_uintegerValue(0)
{
    NS_LOG_FUNCTION(this);
}

Uinteger32SingleProbe::~Uinteger32SingleProbe()
{
    NS_LOG_FUNCTION(this);
}

uint32_t
Uinteger32SingleProbe::GetValue(void) const
{
    NS_LOG_FUNCTION(this);
    return m_uintegerValue;
}

void
Uinteger32SingleProbe::SetValue(uint32_t uintegerValue)
{
    NS_LOG_FUNCTION(this << uintegerValue);
    m_output(m_uintegerValue, uintegerValue);
    m_uintegerValue = uintegerValue;
}

void
Uinteger32SingleProbe::SetValueByPath(std::string path, uint32_t uintegerValue)
{
    NS_LOG_FUNCTION(path << uintegerValue);
    Ptr<Uinteger32SingleProbe> probe = Names::Find<Uinteger32SingleProbe>(path);
    NS_ASSERT_MSG(probe, "Error:  Can't find probe for path " << path);
    probe->SetValue(uintegerValue);
}

bool
Uinteger32SingleProbe::ConnectByObject(std::string traceSource, Ptr<Object> obj)
{
    NS_LOG_FUNCTION(this << traceSource << obj);
    NS_LOG_DEBUG("Name of probe (if any) in names database: " << Names::FindPath(obj));
    bool connected =
        obj->TraceConnectWithoutContext(traceSource,
                                        MakeCallback(&ns3::Uinteger32SingleProbe::TraceSink, this));
    return connected;
}

void
Uinteger32SingleProbe::ConnectByPath(std::string path)
{
    NS_LOG_FUNCTION(this << path);
    NS_LOG_DEBUG("Name of probe to search for in config database: " << path);
    Config::ConnectWithoutContext(path, MakeCallback(&ns3::Uinteger32SingleProbe::TraceSink, this));
}

void
Uinteger32SingleProbe::TraceSink(uint32_t uintegerValue)
{
    NS_LOG_FUNCTION(this << uintegerValue);
    if (IsEnabled())
    {
        m_output(m_uintegerValue, uintegerValue);
        m_uintegerValue = uintegerValue;
    }
}

} // namespace ns3
