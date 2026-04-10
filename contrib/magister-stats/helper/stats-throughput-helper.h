/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 Magister Solutions
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
 * Modified: Lauri Sormunen <lauri.sormunen@magister.fi>
 *
 */

#ifndef STATS_THROUGHPUT_HELPER_H
#define STATS_THROUGHPUT_HELPER_H

#include "stats-helper.h"

#include "ns3/address.h"
#include "ns3/collector-map.h"
#include "ns3/ptr.h"

#include <list>
#include <map>

namespace ns3
{

// BASE CLASS /////////////////////////////////////////////////////////////////

class Node;
class Packet;
class DataCollectionObject;
class DistributionCollector;

/**
 * @ingroup stats
 * @brief Base class for throughput statistics helpers of different levels.
 */
class StatsThroughputHelper : public StatsHelper
{
  public:
    // inherited from StatsHelper base class
    StatsThroughputHelper();

    /**
     * / Destructor.
     */
    virtual ~StatsThroughputHelper();

    /**
     * inherited from ObjectBase base class
     */
    static TypeId GetTypeId();

    /**
     * @param averagingMode average all samples before passing them to aggregator.
     */
    void SetAveragingMode(bool averagingMode);

    /**
     * @return the currently active averaging mode.
     */
    bool GetAveragingMode() const;

    /**
     * @brief Set up several probes or other means of listeners and connect them
     *        to the first-level collectors.
     */
    void InstallProbes();

    /**
     * @brief Receive inputs from trace sources and determine the right collector
     *        to forward the inputs to.
     * @param packet received packet data.
     * @param from the address of the sender of the packet.
     *
     * Used in received traffic statistics. DoInstallProbes() is expected to connect
     * the right trace sources to this method.
     */
    void RxCallback(Ptr<const Packet> packet, const Address& from);

  protected:
    // inherited from StatsHelper base class
    void DoInstall();

    /**
     * @brief Install probes to trace sources. Implemented by child classes.
     */
    virtual void DoInstallProbes() = 0;

    /// Maintains a list of first-level collectors created by this helper.
    CollectorMap m_conversionCollectors;

    /// Maintains a list of second-level collectors created by this helper.
    CollectorMap m_terminalCollectors;

    /// The final collector utilized in averaged output (histogram, PDF, and CDF).
    Ptr<DistributionCollector> m_averagingCollector;

    /// The aggregator created by this helper.
    Ptr<DataCollectionObject> m_aggregator;

    /// Map of address and the identifier associated with it (for return link).
    std::map<const Address, uint32_t> m_identifierMap;

  private:
    bool m_averagingMode; ///< `AveragingMode` attribute.

}; // end of class StatsThroughputHelper

// APPLICATION-LEVEL /////////////////////////////////////////////

class Probe;

/**
 * @ingroup stats
 * @brief Produce application-level throughput statistics from a simulation.
 *
 * The following example can be used:
 * @code
 * Ptr<StatsAppThroughputHelper> s = Create<StatsAppThroughputHelper> (satHelper);
 * s->SetName ("name");
 * s->SetIdentifierType (StatsHelper::IDENTIFIER_GLOBAL);
 * s->SetOutputType (StatsHelper::OUTPUT_SCATTER_FILE);
 * s->InstallNodes (nodes);
 * s->Install ();
 * @endcode
 */
class StatsAppThroughputHelper : public StatsThroughputHelper
{
  public:
    // inherited from StatsHelper base class
    StatsAppThroughputHelper();

    /**
     * / Destructor.
     */
    virtual ~StatsAppThroughputHelper();

    /**
     * inherited from ObjectBase base class
     */
    static TypeId GetTypeId();

  protected:
    // inherited from StatsThroughputHelper base class
    void DoInstallProbes();

  private:
    /// Maintains a list of probes created by this helper.
    std::list<Ptr<Probe>> m_probes;

}; // end of class StatsAppThroughputHelper

} // end of namespace ns3

#endif /* STATS_THROUGHPUT_HELPER_H */
