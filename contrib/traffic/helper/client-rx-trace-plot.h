/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 Magister Solutions
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
 * Author: Budiarto Herman <budiarto.herman@magister.fi>
 *
 */

#ifndef CLIENT_RX_TRACE_PLOT_H
#define CLIENT_RX_TRACE_PLOT_H

#include "ns3/application.h"
#include "ns3/gnuplot.h"
#include "ns3/object.h"
#include "ns3/ptr.h"

#include <string>

namespace ns3
{

class Packet;
class Address;
class NrtvTcpClient;
class PacketSink;

/**
 * @ingroup applications
 * @brief Installed on an application with "Rx" trace source, this helper class will
 *        generate a Gnuplot file out of the Rx traffic experienced by the
 *        application.
 *
 * One helper is intended only for one application and will generate one Gnuplot
 * file at the end of the simulation. Usage example:
 *
 *     Ptr<PacketSink> packetSink = apps.Get (0);
 *     Ptr<ClientTracePlot> plot =
 *         CreateObject<ClientTracePlot> (packetSink);
 *
 * By default, the Gnuplot file name is "client-trace.plt". This can be
 * modified by calling the SetOutputName() method, or by using the extended
 * constructor.
 *
 * The Gnuplot file can be converted to a PNG file, for example by using this
 * command:
 *
 *     $ gnuplot client-trace.plt
 *
 * The above command generates a new file "client-trace.png" in the same
 * directory.
 */
class ClientRxTracePlot : public Object
{
  public:
    /**
     * @brief Creates a new object instance which generates a plot file named
     *        "client-trace.plt".
     *
     * @param clientApp the client application from which the traffic data is
     *                  taken and generated as a plot
     */
    ClientRxTracePlot(Ptr<Application> clientApp);

    /**
     * @brief Creates a new object instance which generates a plot file with the
     *        specified name.
     *
     * @param clientApp  the client application from which the traffic data is
     *                   taken and generated as a plot
     * @param outputName the name of the plot file, e.g., specifying the value
     *                   "output" will generate "output.plt" file, which then can
     *                   be converted to "output.png"
     */
    ClientRxTracePlot(Ptr<Application> clientApp, std::string outputName);

    /// Object destructor, which will generate the output.
    ~ClientRxTracePlot();

    // Inherited from ObjectBase base class
    static TypeId GetTypeId();

    /**
     * @param outputName the name of the plot file, e.g., specifying the value
     *                   "output" will generate "output.plt" file, which then can
     *                   be converted to "output.png"
     */
    void SetOutputName(std::string outputName);

    /**
     * @return the name of the plot file
     */
    std::string GetOutputName() const;

  private:
    /**
     * @internal
     * @brief Connecting the object to trace sources in the client application and
     *        creating the Gnuplot datasets for storing the gathered data.
     */
    void Initialize();

    /// Generating the plot.
    void Plot();

    // TRACE CALLBACK FUNCTIONS

    void RxCallback(Ptr<const Packet> packet, const Address& from);

    Ptr<Application> m_client; ///< The currently active client application.
    std::string m_outputName;  ///< The name of the plot file.
    Gnuplot2dDataset m_packet; ///< Size of every packet received.
    u_int32_t m_counter;

}; // end of `class ClientRxTracePlot`

} // namespace ns3

#endif /* CLIENT_RX_TRACE_PLOT_H */
