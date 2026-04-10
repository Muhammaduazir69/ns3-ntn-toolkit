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

#include "bytes-probe.h"

#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/packet.h"
#include "ns3/trace-source-accessor.h"

#include <string>

NS_LOG_COMPONENT_DEFINE("BytesProbe");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(BytesProbe);

TypeId
BytesProbe::GetTypeId()
{
    static TypeId tid = TypeId("ns3::BytesProbe")
                            .SetParent<Probe>()
                            .AddConstructor<BytesProbe>()
                            .AddTraceSource("Output",
                                            "The uint32_t that serves as output for this probe",
                                            MakeTraceSourceAccessor(&BytesProbe::m_output),
                                            "ns3::Packet::PacketSizeTracedCallback");
    return tid;
}

BytesProbe::BytesProbe()
{
    NS_LOG_FUNCTION(this);
}

BytesProbe::~BytesProbe()
{
    NS_LOG_FUNCTION(this);
}

uint32_t
BytesProbe::GetValue(void) const
{
    NS_LOG_FUNCTION(this);
    return m_bytesOld;
}

void
BytesProbe::SetValue(uint32_t newVal)
{
    NS_LOG_FUNCTION(this << newVal);
    m_output(m_bytesOld, newVal);
    m_bytesOld = newVal;
}

void
BytesProbe::SetValueByPath(std::string path, uint32_t newVal)
{
    NS_LOG_FUNCTION(path << newVal);
    Ptr<BytesProbe> probe = Names::Find<BytesProbe>(path);
    NS_ASSERT_MSG(probe, "Error:  Can't find probe for path " << path);
    probe->SetValue(newVal);
}

bool
BytesProbe::ConnectByObject(std::string traceSource, Ptr<Object> obj)
{
    NS_LOG_FUNCTION(this << traceSource << obj);
    NS_LOG_DEBUG("Name of probe (if any) in names database: " << Names::FindPath(obj));
    bool connected =
        obj->TraceConnectWithoutContext(traceSource,
                                        MakeCallback(&ns3::BytesProbe::TraceSink, this));
    return connected;
}

void
BytesProbe::ConnectByPath(std::string path)
{
    NS_LOG_FUNCTION(this << path);
    NS_LOG_DEBUG("Name of probe to search for in config database: " << path);
    Config::ConnectWithoutContext(path, MakeCallback(&ns3::BytesProbe::TraceSink, this));
}

void
BytesProbe::TraceSink(uint32_t bytes)
{
    NS_LOG_FUNCTION(this << bytes);
    if (IsEnabled())
    {
        m_output(m_bytesOld, bytes);
        m_bytesOld = bytes;
    }
}

} // namespace ns3
