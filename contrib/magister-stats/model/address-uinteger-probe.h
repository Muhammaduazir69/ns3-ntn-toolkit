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
 * - Frans Laakso (frans.laakso@magister.fi)
 */

#ifndef ADDRESS_UINTEGER_PROBE_H
#define ADDRESS_UINTEGER_PROBE_H

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
 * Uinteger information and a socket address.  This probe exports a trace
 * source "Output" with arguments of type Uinteger and const Address&.  This probe
 * exports another trace source "OutputUinteger" with arguments of type Uinteger.
 * The trace sources emit values when either the probed trace source emits a
 * new value, or when SetValue () is called.
 */
class AddressUintegerProbe : public Probe
{
  public:
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();
    AddressUintegerProbe();
    virtual ~AddressUintegerProbe();

    /**
     * @brief Set a probe value
     *
     * @param uintegerValue set the traced Uinteger equal to this
     * @param address set the socket address for the traced packet equal to this
     */
    void SetValue(uint32_t uintegerValue, const Address& address);

    /**
     * @brief Set a probe value by its name in the Config system
     *
     * @param path config path to access the probe
     * @param uintegerValue set the traced Uinteger equal to this
     * @param address set the socket address for the traced packet equal to this
     */
    static void SetValueByPath(std::string path, uint32_t uintegerValue, const Address& address);

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
     * @brief Callback signature for Uinteger and address.
     *
     * @param uintegerValue the Uinteger
     * @param address the socket address for the packet
     *
     */
    typedef void (*UintegerAddressCallback)(uint32_t uintegerValue, const Address& address);

    /**
     * @brief Callback signature for changes in packet boolean.
     *
     * @param oldValue the previous Uinteger
     * @param newValue the actual Uinteger
     */
    typedef void (*UintegerCallback)(uint32_t oldValue, uint32_t newValue);

  private:
    /**
     * @brief Method to connect to an underlying ns3::TraceSource with
     * arguments of type Uinteger value and const Address&
     *
     * @param uintegerValue the traced Uinteger
     * @param address the socket address for the traced packet
     *
     */
    void TraceSink(uint32_t uintegerValue, const Address& address);

    /// Output trace, the Uinteger and source address
    TracedCallback<uint32_t, const Address&> m_output;
    /// Output trace, previous value and current value
    TracedCallback<uint32_t, uint32_t> m_outputUinteger;

    /// The traced Uinteger.
    uint32_t m_uintegerValue;

    /// The socket address for the traced packet.
    Address m_address;
};

} // namespace ns3

#endif // MAG_ADDRESS_UINTEGER_PROBE_H
