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
 * Modified for RxDelay trace source by:
 * - Budiarto Herman (budiarto.herman@magister.fi)
 */

#include "application-delay-probe.h"

#include "ns3/callback.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/simulator.h"

#include <string>

NS_LOG_COMPONENT_DEFINE("ApplicationDelayProbe");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(ApplicationDelayProbe);

TypeId
ApplicationDelayProbe::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ApplicationDelayProbe")
            .SetParent<Probe>()
            .AddConstructor<ApplicationDelayProbe>()
            .AddTraceSource(
                "Output",
                "The delay plus its socket address that serve as the output for this probe",
                MakeTraceSourceAccessor(&ApplicationDelayProbe::m_output),
                "ns3::ApplicationDelayProbe::PacketDelayAddressCallback")
            .AddTraceSource("OutputSeconds",
                            "The delay of the traced packet",
                            MakeTraceSourceAccessor(&ApplicationDelayProbe::m_outputSeconds),
                            "ns3::ApplicationDelayProbe::PacketDelayCallback");
    return tid;
}

ApplicationDelayProbe::ApplicationDelayProbe()
{
    NS_LOG_FUNCTION(this);
    m_delay = MilliSeconds(0);
    m_delayOld = 0.0;
}

ApplicationDelayProbe::~ApplicationDelayProbe()
{
    NS_LOG_FUNCTION(this);
}

void
ApplicationDelayProbe::SetValue(Time delay, const Address& address)
{
    NS_LOG_FUNCTION(this << delay.GetSeconds() << address);
    m_delay = delay;
    m_address = address;
    m_output(delay, address);

    double delayNew = delay.GetSeconds();
    m_outputSeconds(m_delayOld, delayNew);
    m_delayOld = delayNew;
}

void
ApplicationDelayProbe::SetValueByPath(std::string path, Time delay, const Address& address)
{
    NS_LOG_FUNCTION(path << delay.GetSeconds() << address);
    Ptr<ApplicationDelayProbe> probe = Names::Find<ApplicationDelayProbe>(path);
    NS_ASSERT_MSG(probe, "Error:  Can't find probe for path " << path);
    probe->SetValue(delay, address);
}

bool
ApplicationDelayProbe::ConnectByObject(std::string traceSource, Ptr<Object> obj)
{
    NS_LOG_FUNCTION(this << traceSource << obj);
    NS_LOG_DEBUG("Name of probe (if any) in names database: " << Names::FindPath(obj));
    bool connected =
        obj->TraceConnectWithoutContext(traceSource,
                                        MakeCallback(&ApplicationDelayProbe::TraceSink, this));
    return connected;
}

void
ApplicationDelayProbe::ConnectByPath(std::string path)
{
    NS_LOG_FUNCTION(this << path);
    NS_LOG_DEBUG("Name of probe to search for in config database: " << path);
    Config::ConnectWithoutContext(path, MakeCallback(&ApplicationDelayProbe::TraceSink, this));
}

void
ApplicationDelayProbe::TraceSink(const Time& delay, const Address& address)
{
    NS_LOG_FUNCTION(this << delay.GetSeconds() << address);
    if (IsEnabled())
    {
        m_delay = delay;
        m_address = address;
        m_output(delay, address);

        double delayNew = delay.GetSeconds();
        m_outputSeconds(m_delayOld, delayNew);
        m_delayOld = delayNew;
    }
}

} // namespace ns3
