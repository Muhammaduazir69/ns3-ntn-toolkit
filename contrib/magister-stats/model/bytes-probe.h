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
 * Author of original work (uinteger-32-probe.h):
 * - L. Felipe Perrone (perrone@bucknell.edu)
 * - Tiago G. Rodrigues (tgr002@bucknell.edu)
 * - Mitch Watrous (watrous@u.washington.edu)
 *
 * Modified to support trace sources with a single uint32_t argument by:
 * - Budiarto Herman (budiarto.herman@magister.fi)
 */

#ifndef BYTES_PROBE_H
#define BYTES_PROBE_H

#include "ns3/callback.h"
#include "ns3/probe.h"
#include "ns3/traced-callback.h"

#include <string>

namespace ns3
{

/**
 * @ingroup probes
 *
 * This class is designed to probe an underlying ns3 TraceSource exporting
 * a uint32_t which represents a size in bytes.  This probe exports a trace
 * source "Output" of type uint32_t. The Output trace source emits a value when
 * either the trace source emits a new value, or when SetValue () is called.
 *
 * This class differs with Uinteger32Probe in the following way:
 * - Uinteger32Probe expects input from a TracedValue (i.e., an old uint32_t
 *   value and a new uint32_t value) or a similar TracedCallback with two
 *   arguments.
 * - BytesProbe expects input from a TracedCallback with one argument.
 * - Uinteger32Probe is *not* fired when the new value is the same with the old
 *   value, but BytesProbe is.
 *
 * The current value of the probe can be polled with the GetValue () method.
 */
class BytesProbe : public Probe
{
  public:
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();
    BytesProbe();
    virtual ~BytesProbe();

    /**
     * @return the most recent value
     */
    uint32_t GetValue(void) const;

    /**
     * @param value set the traced uint32_t to a new value
     */
    void SetValue(uint32_t value);

    /**
     * @brief Set a probe value by its name in the Config system
     *
     * @param path Config path to access the probe
     * @param value set the traced uint32_t to a new value
     */
    static void SetValueByPath(std::string path, uint32_t value);

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

  private:
    /**
     * @brief Method to connect to an underlying ns3::TraceSource of type uint32_t
     *
     * @param bytes value of the uint32_t
     */
    void TraceSink(uint32_t bytes);

    TracedCallback<uint32_t, uint32_t> m_output; //!< Output trace source.

    /// The size of the traced bytes.
    uint32_t m_bytesOld;
};

} // namespace ns3

#endif // BYTES_PROBE_H
