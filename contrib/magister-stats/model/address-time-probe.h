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
 * Authors of original work (application-packet-probe.h) which this work
 * derives from:
 * - L. Felipe Perrone (perrone@bucknell.edu)
 * - Tiago G. Rodrigues (tgr002@bucknell.edu)
 * - Mitch Watrous (watrous@u.washington.edu)
 *
 * Modified by:
 * - Budiarto Herman (budiarto.herman@magister.fi)
 */

#ifndef ADDRESS_TIME_PROBE_H
#define ADDRESS_TIME_PROBE_H

#include "ns3/address.h"
#include "ns3/nstime.h"
#include "ns3/probe.h"
#include "ns3/traced-callback.h"

#include <string>

namespace ns3
{

/**
 * @brief Probe to translate from a TraceSource to two more easily parsed TraceSources.
 *
 * This class is designed to probe an underlying ns3 TraceSource exporting a
 * time information and a socket address.  This probe exports a trace
 * source "Output" with arguments of type Time and const Address&.  This probe
 * exports another trace source "OutputSeconds" with arguments of type double,
 * which is the time in seconds.  The trace sources emit values
 * when either the probed trace source emits a new value, or when SetValue ()
 * is called.
 */
class AddressTimeProbe : public Probe
{
  public:
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();
    AddressTimeProbe();
    virtual ~AddressTimeProbe();

    /**
     * @brief Set a probe value
     *
     * @param timeValue set the traced time value equal to this
     * @param address set the socket address for the traced packet equal to this
     */
    void SetValue(Time timeValue, const Address& address);

    /**
     * @brief Set a probe value by its name in the Config system
     *
     * @param path config path to access the probe
     * @param timeValue set the traced time value equal to this
     * @param address set the socket address for the traced packet equal to this
     */
    static void SetValueByPath(std::string path, Time timeValue, const Address& address);

    /**
     * @brief connect to a trace source attribute provided by a given object
     *
     * @param traceSource the name of the attribute TraceSource to connect to
     * @param obj ns3::Object to connect to
     * @return true if the trace source was successfully connected
     */
    virtual bool ConnectByObject(std::string traceSource, Ptr<Object> obj);

    /**
     * @brief connect to a trace source provided by a config path
     *
     * @param path Config path to bind to
     *
     * Note, if an invalid path is provided, the probe will not be connected
     * to anything.
     */
    virtual void ConnectByPath(std::string path);

    /**
     * @brief Callback signature for time value and address.
     *
     * @param timeValue the time value
     * @param address the socket address for the packet
     *
     * @todo Optimize by using const-reference of Time.
     */
    typedef void (*TimeAddressCallback)(Time timeValue, const Address& address);

    /**
     * @brief Callback signature for changes in time value.
     *
     * @param oldValue the previous time value in seconds
     * @param newValue the actual time value in seconds
     */
    typedef void (*TimeCallback)(double oldValue, double newValue);

  private:
    /**
     * @brief Method to connect to an underlying ns3::TraceSource with
     * arguments of type Time value and const Address&
     *
     * @param timeValue the traced time value
     * @param address the socket address for the traced packet
     *
     */
    void TraceSink(Time timeValue, const Address& address);

    /// Output trace, thetime value and source address
    TracedCallback<Time, const Address&> m_output;
    /// Output trace, previous time value and current time value
    TracedCallback<double, double> m_outputSeconds;

    /// The traced time value.
    Time m_timeValue;

    /// The socket address.
    Address m_address;
};

} // namespace ns3

#endif // ADDRESS_TIME_PROBE_H
