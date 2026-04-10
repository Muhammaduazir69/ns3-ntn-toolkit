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
 */

#ifndef STATS_DELAY_HELPER_H
#define STATS_DELAY_HELPER_H

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
class Time;
class DataCollectionObject;
class DistributionCollector;

/**
 * @ingroup stats
 * @brief Base class for delay statistics helpers.
 */
class StatsDelayHelper : public StatsHelper
{
  public:
    // inherited from StatsHelper base class
    StatsDelayHelper();

    /**
     * / Destructor.
     */
    virtual ~StatsDelayHelper();

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
     *        to the collectors.
     */
    void InstallProbes();

  protected:
    // inherited from StatsHelper base class
    void DoInstall();

    /**
     * @brief Install callbacks and probes to application trace sources,
     * if needed. Implemented by child classes.
     */
    virtual void DoInstallProbes() = 0;

    /**
     * @brief Connect the probe to the right collector.
     * @param probe
     * @param identifier
     */
    bool ConnectProbeToCollector(Ptr<Probe> probe, uint32_t identifier);

    /**
     * @brief Find a collector with the right identifier and pass a sample data
     *        to it.
     * @param delay
     * @param identifier
     */
    void PassSampleToCollector(Time delay, uint32_t identifier);

    /// Maintains a list of collectors created by this helper.
    CollectorMap m_terminalCollectors;

    /// The final collector utilized in averaged output (histogram, PDF, and CDF).
    Ptr<DistributionCollector> m_averagingCollector;

    /// The aggregator created by this helper.
    Ptr<DataCollectionObject> m_aggregator;

    /// Map of address and the identifier associated with it (for return link).
    std::map<const Address, uint32_t> m_identifierMap;

  private:
    bool m_averagingMode; ///< `AveragingMode` attribute.

}; // end of class StatsDelayHelper

// APPLICATION-LEVEL /////////////////////////////////////////////

class Probe;

/**
 * @ingroup stats
 * @brief Produce application-level delay statistics from a simulation.
 *
 * The following example can be used:
 * @code
 * Ptr<StatsFwdAppDelayHelper> s = Create<StatsFwdAppDelayHelper> ();
 * s->SetName ("name");
 * s->SetIdentifierType (StatsHelper::IDENTIFIER_GLOBAL);
 * s->SetOutputType (StatsHelper::OUTPUT_SCATTER_FILE);
 * s->InstallNodes (nodes);
 * s->Install ();
 * @endcode
 */
class StatsAppDelayHelper : public StatsDelayHelper
{
  public:
    // inherited from StatsHelper base class
    StatsAppDelayHelper();

    /**
     * Destructor for StatsFwdAppDelayHelper.
     */
    virtual ~StatsAppDelayHelper();

    /**
     * inherited from ObjectBase base class
     */
    static TypeId GetTypeId();

    /**
     * @brief Receive inputs from trace sources and determine the right collector
     *        to forward the inputs to.
     * @param helper Pointer to the delay statistics collector helper
     * @param identifier Identifier used to group statistics.
     * @param packet the received packet, expected to have been tagged with
     *               TrafficTimeTag.
     * @param from the InetSocketAddress of the sender of the packet.
     */
    static void RxCallback(Ptr<StatsAppDelayHelper> helper,
                           uint32_t identifier,
                           Ptr<const Packet> packet,
                           const Address& from);

    /**
     * @brief Receive inputs from trace sources and determine the right collector
     *        to forward the inputs to.
     * @param helper Pointer to the delay statistics collector helper
     * @param identifier Identifier used to group statistics.
     * @param packet the sent packet, yo which TrafficTimeTag will be attached.
     */
    static void TxCallback(Ptr<StatsAppDelayHelper> helper, Ptr<const Packet> packet);

  protected:
    // inherited from StatsDelayHelper base class
    void DoInstallProbes();

  private:
    /// Maintains a list of probes created by this helper.
    std::list<Ptr<Probe>> m_probes;

}; // end of class StatsAppDelayHelper

} // namespace ns3

#endif /* STATS_DELAY_HELPER_H */
