/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
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
 * Author: Budiarto Herman <budiarto.herman@magister.fi>
 *
 */

#ifndef COLLECTOR_MAP_H
#define COLLECTOR_MAP_H

#include "ns3/assert.h"
#include "ns3/attribute.h"
#include "ns3/data-collection-object.h"
#include "ns3/object-factory.h"
#include "ns3/probe.h"
#include "ns3/ptr.h"
#include "ns3/type-id.h"

#include <iostream>
#include <map>
#include <string>

namespace ns3
{

/**
 * @ingroup aggregator
 * @brief Keep track and manipulate a set of statistics collectors within the
 *        data collection framework.
 *
 * The map contains a set of collectors of the same type. Each of them is
 * uniquely identified by a non-negative integer called an *identifier*.
 *
 * The class provides several methods for connecting the collectors with
 * probes, aggregators, and other collectors.
 *
 * The constructor creates an empty map. The following example demonstrates
 * adding two collectors into a new map.
 * @code
 *   // Create a new map with zero collector.
 *   CollectorMap c;
 *
 *   // set the type of collectors to be created.
 *   c.SetType ("ns3::ScalarCollector");
 *
 *   // Set the attributes of collectors to be created.
 *   c.SetAttribute ("Name",
 *                   StringValue ("collector-1"));
 *   c.SetAttribute ("InputDataType",
 *                   EnumValue (ScalarCollector::INPUT_DATA_TYPE_DOUBLE));
 *   c.SetAttribute ("OutputType",
 *                   EnumValue (ScalarCollector::OUTPUT_TYPE_SUM));
 *
 *   // Create a collector with identifier `1`.
 *   c.Create (1);
 *
 *   // Change the attribute of collectors to be created.
 *   c.SetAttribute ("Name",
 *                   StringValue ("collector-3"));
 *
 *   // Create a collector with identifier `3`.
 *   c.Create (3);
 * @endcode
 */
class CollectorMap
{
  public:
    /// Creates an empty map.
    CollectorMap();

    /**
     * @param type the type of collectors to be created.
     *
     * @warning Will raise an error if the TypeId referred by the `type` argument
     *          does not refer to a class derived from DataCollectionObject class.
     */
    void SetType(std::string type);

    /**
     * @return the type information used for creating collectors.
     */
    TypeId GetType() const;

    /**
     * @param n the name of the attribute to be set on each collector created.
     * @param v the value of the attribute to be set on each collector created.
     */
    void SetAttribute(std::string n, const AttributeValue& v);

    /**
     * @brief Create a single collector and append it to this map.
     * @param identifier the identifier to be associated with the new collector.
     *
     * The collector will be created based on the type information previously set
     * using SetType() and then configured using attribute values previously
     * declared using SetAttribute().
     *
     * @warning If a collector with the same identifier has already existed in
     *          the map, it will be replaced by the newly created collector.
     *          Because of this, the destructor of the previous collector might
     *          be triggered. In most cases, this will trigger the previous
     *          collector to prematurely emit outputs.
     */
    void Create(uint32_t identifier);

    /**
     * @brief Append a single collector to this map.
     * @param identifier the identifier to be associated with the new collector.
     * @param the new collector.
     */
    void Insert(uint32_t identifier, Ptr<DataCollectionObject> dataCollectionObject);

    /**
     * @return true if the map contains zero collector, or false otherwise.
     */
    bool IsEmpty() const;

    /**
     * @param identifier the identifier of the collector to be queried.
     * @return true if a collector with the specified identifier exists in the
     *         map, or false otherwise.
     */
    bool IsExists(uint32_t identifier) const;

    /**
     * @return the number of collectors stored in this map.
     */
    uint32_t GetN() const;

    typedef std::map<uint32_t, Ptr<DataCollectionObject>>::const_iterator Iterator;

    /**
     * @brief Get an iterator which refers to the first collector in the map.
     * @return an iterator which refers to the first collector in the map.
     *
     * Collectors can be retrieved from the map in two ways.  First, directly
     * using the identifier of the container, and second, using an iterator.
     * This method is used in the iterator method and is typically used in a
     * for-loop to run through the collectors.
     *
     * @code
     *   CollectorMap::Iterator i;
     *   for (i = container.Begin (); i != container.End (); ++i)
     *     {
     *       std::cout << i->first << std::endl;  // the collector's identifier
     *       i->second->method ();  // some collector method
     *     }
     * @endcode
     */
    Iterator Begin() const;

    /**
     * @brief Get an iterator which indicates past-the-last collector in the map.
     * @return an iterator which indicates an ending condition for a loop.
     *
     * Collectors can be retrieved from the map in two ways.  First, directly
     * using the identifier of the container, and second, using an iterator.
     * This method is used in the iterator method and is typically used in a
     * for-loop to run through the collectors.
     *
     * @code
     *   CollectorMap::Iterator i;
     *   for (i = container.Begin (); i != container.End (); ++i)
     *     {
     *       std::cout << i->first << std::endl;  // the collector's identifier
     *       i->second->method ();  // some collector method
     *     }
     * @endcode
     */
    Iterator End() const;

    /**
     * @brief Get the collector stored in this map.
     * @param identifier the identifier of the requested collector.
     * @return a pointer to the requested collector, or zero if the requested
     *         collector is not found.
     *
     * Collectors can be retrieved from the map in two ways.  First, directly
     * using the identifier of the container, and second, using an iterator.
     * This method is used in the direct method and is used to retrieve the
     * collector by its identifier.
     */
    Ptr<DataCollectionObject> Get(uint32_t identifier) const;

    /**
     * @brief Connect a probe with one of the collectors inside the map.
     * @param probe a pointer to the probe.
     * @param probeTraceSourceName the name of the trace source of the probe to
     *                             be connected with the collector.
     * @param collectorIdentifier the identifier of the collector.
     * @param collectorTraceSink a pointer to a function of the collector which
     *                           acts as a trace sink.
     * @return true if a connection is successfully made, or false otherwise
     *         (e.g., because of invalid probe trace source name).
     *
     * @warning May cause undefined behaviour if the collector with the given
     *          identifier is not found within the map.
     *
     * Upon connection, the probe's output will become the input of the collector.
     *
     * The collector's trace sink function must be an accessible (e.g., public)
     * class method which accepts two input arguments of the same type. For
     * example: `&IntervalRateCollector::TraceSinkDouble`.
     */
    template <typename R, typename C, typename P>
    bool ConnectWithProbe(Ptr<Probe> probe,
                          std::string probeTraceSourceName,
                          uint32_t collectorIdentifier,
                          R (C::*collectorTraceSink)(P, P)) const;

    /**
     * @brief Disconnect a probe from one of the collectors inside the map.
     * @param probe a pointer to the probe.
     * @param probeTraceSourceName the name of the trace source of the probe to
     *                             be connected with the collector.
     * @param collectorIdentifier the identifier of the collector.
     * @param collectorTraceSink a pointer to a function of the collector which
     *                           acts as a trace sink.
     * @return true if a connection is successfully made, or false otherwise
     *         (e.g., invalid probe trace source name).
     *
     * The collector's trace sink function must be an accessible (e.g., public)
     * class method which accepts two input arguments of the same type. For
     * example: `&IntervalRateCollector::TraceSinkDouble`.
     *
     * @todo To remember the collector which a probe is currently connected to,
     *       so that disconnecting can be done without mentioning the identifier.
     */
    template <typename R, typename C, typename P>
    bool DisconnectWithProbe(Ptr<Probe> probe,
                             std::string probeTraceSourceName,
                             uint32_t collectorIdentifier,
                             R (C::*collectorTraceSink)(P, P)) const;

    /**
     * @brief Connect each collector in this map with a corresponding collector
     *        in the target map.
     * @param traceSourceName the name of the trace source of the collectors in
     *                        this map to be connected with the target map.
     * @param targetMap map of downstream collectors.
     * @param traceSink a pointer to a function of the collectors in the target
     *                  map which acts as a trace sink.
     * @return true if connections are created successfully, or false otherwise.
     *
     * The connections created are one-to-one, where collectors with the same
     * identifier are connected. Upon connected, statistics data will flow from
     * the collectors in this map (upstream map) to the corresponding collectors
     * in the target map (downstream map).
     *
     * The collector's trace sink function must be an accessible (e.g., public)
     * class method which accepts two input arguments of the same type. For
     * example: `&IntervalRateCollector::TraceSinkDouble`.
     *
     * @warning May cause undefined behaviour if the target CollectorMap has
     *          different number of collectors or different set of identifiers.
     */
    template <typename R, typename C, typename P>
    bool ConnectToCollector(std::string traceSourceName,
                            CollectorMap& targetMap,
                            R (C::*traceSink)(P, P));

    /**
     * @brief Connect each collector in the map to an aggregator.
     * @param traceSourceName the name of the trace source of the collectors in
     *                        this map to be connected with the aggregator.
     * @param aggregator a pointer to the aggregator.
     * @param aggregatorTraceSink a pointer to a function of the aggregator which
     *                            which acts as a trace sink.
     *
     * Upon connection, the collectors' output will become the input of the
     * aggregator.
     *
     * The aggregator's trace sink function must be an accessible (e.g., public)
     * class method which accepts one input argument. The collector's name will
     * be passed as the argument. The trace sink function can be specified as,
     * for example, `&MultiFileAggregator::EnableContextWarning`.
     */
    template <typename R, typename C, typename P1>
    bool ConnectToAggregator(std::string traceSourceName,
                             Ptr<DataCollectionObject> aggregator,
                             R (C::*aggregatorTraceSink)(P1)) const;

    /**
     * @brief Connect each collector in the map to an aggregator.
     * @param traceSourceName the name of the trace source of the collectors in
     *                        this map to be connected with the aggregator.
     * @param aggregator a pointer to the aggregator.
     * @param aggregatorTraceSink a pointer to a function of the aggregator which
     *                            which acts as a trace sink.
     *
     * Upon connection, the collectors' output will become the input of the
     * aggregator.
     *
     * The aggregator's trace sink function must be an accessible (e.g., public)
     * class method which accepts two input arguments. The collector's name will
     * be passed to the first argument. Then the value produced by the trace
     * source will be passed to the second argument. The trace sink function can
     * be specified as, for example, `&MultiFileAggregator::Write1d`.
     */
    template <typename R, typename C, typename P1, typename V1>
    bool ConnectToAggregator(std::string traceSourceName,
                             Ptr<DataCollectionObject> aggregator,
                             R (C::*aggregatorTraceSink)(P1, V1)) const;

    /**
     * @brief Connect each collector in the map to an aggregator.
     * @param traceSourceName the name of the trace source of the collectors in
     *                        this map to be connected with the aggregator.
     * @param aggregator a pointer to the aggregator.
     * @param aggregatorTraceSink a pointer to a function of the aggregator which
     *                            which acts as a trace sink.
     *
     * Upon connection, the collectors' output will become the input of the
     * aggregator.
     *
     * The aggregator's trace sink function must be an accessible (e.g., public)
     * class method which accepts three input arguments. The collector's name
     * will be passed to the first argument. Then two values produced by the
     * collector's trace source will be passed to the second and third argument.
     * The trace sink function can be specified as, for example,
     * `&MultiFileAggregator::Write2d`.
     */
    template <typename R, typename C, typename P1, typename V1, typename V2>
    bool ConnectToAggregator(std::string traceSourceName,
                             Ptr<DataCollectionObject> aggregator,
                             R (C::*aggregatorTraceSink)(P1, V1, V2)) const;

  private:
    /// Utilized to automate creating instances of collectors.
    ObjectFactory m_factory;

    /// Internal map of identifiers (as the key) and collectors (as the value).
    std::map<uint32_t, Ptr<DataCollectionObject>> m_map;

}; // end of class CollectorMap

// TEMPLATE METHOD DEFINITIONS ////////////////////////////////////////////////

template <typename R, typename C, typename P>
bool
CollectorMap::ConnectWithProbe(Ptr<Probe> probe,
                               std::string probeTraceSourceName,
                               uint32_t collectorIdentifier,
                               R (C::*collectorTraceSink)(P, P)) const
{
    Ptr<DataCollectionObject> collector = Get(collectorIdentifier);
    NS_ASSERT_MSG(collector != nullptr,
                  "Error finding collector with identifier " << collectorIdentifier);
    Ptr<C> c = collector->GetObject<C>();
    NS_ASSERT_MSG(c != nullptr,
                  "Collector type " << collector->GetInstanceTypeId().GetName()
                                    << " is incompatible with the specified trace sink");
    return probe->TraceConnectWithoutContext(probeTraceSourceName,
                                             MakeCallback(collectorTraceSink, c));
}

template <typename R, typename C, typename P>
bool
CollectorMap::DisconnectWithProbe(Ptr<Probe> probe,
                                  std::string probeTraceSourceName,
                                  uint32_t collectorIdentifier,
                                  R (C::*collectorTraceSink)(P, P)) const
{
    Ptr<DataCollectionObject> collector = Get(collectorIdentifier);
    NS_ASSERT_MSG(collector != nullptr,
                  "Error finding collector with identifier " << collectorIdentifier);
    Ptr<C> c = collector->GetObject<C>();
    NS_ASSERT_MSG(c != nullptr,
                  "Collector type " << collector->GetInstanceTypeId().GetName()
                                    << " is incompatible with the specified trace sink");
    return probe->TraceDisconnectWithoutContext(probeTraceSourceName,
                                                MakeCallback(collectorTraceSink, c));
}

template <typename R, typename C, typename P>
bool
CollectorMap::ConnectToCollector(std::string traceSourceName,
                                 CollectorMap& targetMap,
                                 R (C::*traceSink)(P, P))
{
    NS_ASSERT_MSG(m_map.size() == targetMap.GetN(), "Error connecting maps of different size");

    for (CollectorMap::Iterator it = m_map.begin(); it != m_map.end(); ++it)
    {
        const uint32_t identifier = it->first;
        const Ptr<DataCollectionObject> source = it->second;
        NS_ASSERT(source != nullptr);
        Ptr<DataCollectionObject> target = targetMap.Get(identifier);
        NS_ASSERT_MSG(target != nullptr,
                      "Unable to find target collector with identifier " << identifier);
        Ptr<C> c = target->GetObject<C>();
        NS_ASSERT_MSG(c != nullptr,
                      "Collector type " << target->GetInstanceTypeId().GetName()
                                        << " is incompatible with the specified trace sink");
        if (!source->TraceConnectWithoutContext(traceSourceName, MakeCallback(traceSink, c)))
        {
            return false;
        }
    }

    return true;
}

template <typename R, typename C, typename P1>
bool
CollectorMap::ConnectToAggregator(std::string traceSourceName,
                                  Ptr<DataCollectionObject> aggregator,
                                  R (C::*aggregatorTraceSink)(P1)) const
{
    for (CollectorMap::Iterator it = m_map.begin(); it != m_map.end(); ++it)
    {
        Ptr<DataCollectionObject> collector = it->second;
        NS_ASSERT(collector != nullptr);
        const std::string context = collector->GetName();
        Ptr<C> c = aggregator->GetObject<C>();
        NS_ASSERT_MSG(c != nullptr,
                      "Aggregator type " << aggregator->GetInstanceTypeId().GetName()
                                         << " is incompatible with the specified trace sink");
        if (!collector->TraceConnect(traceSourceName,
                                     context,
                                     MakeCallback(aggregatorTraceSink, c)))
        {
            return false;
        }
    }

    return true;
}

template <typename R, typename C, typename P1, typename V1>
bool
CollectorMap::ConnectToAggregator(std::string traceSourceName,
                                  Ptr<DataCollectionObject> aggregator,
                                  R (C::*aggregatorTraceSink)(P1, V1)) const
{
    for (CollectorMap::Iterator it = m_map.begin(); it != m_map.end(); ++it)
    {
        Ptr<DataCollectionObject> collector = it->second;
        NS_ASSERT(collector != nullptr);
        const std::string context = collector->GetName();
        Ptr<C> c = aggregator->GetObject<C>();
        NS_ASSERT_MSG(c != nullptr,
                      "Aggregator type " << aggregator->GetInstanceTypeId().GetName()
                                         << " is incompatible with the specified trace sink");
        if (!collector->TraceConnect(traceSourceName,
                                     context,
                                     MakeCallback(aggregatorTraceSink, c)))
        {
            return false;
        }
    }

    return true;
}

template <typename R, typename C, typename P1, typename V1, typename V2>
bool
CollectorMap::ConnectToAggregator(std::string traceSourceName,
                                  Ptr<DataCollectionObject> aggregator,
                                  R (C::*aggregatorTraceSink)(P1, V1, V2)) const
{
    // The following code is exactly the same as the previous function's code.
    for (CollectorMap::Iterator it = m_map.begin(); it != m_map.end(); ++it)
    {
        Ptr<DataCollectionObject> collector = it->second;
        NS_ASSERT(collector != nullptr);
        const std::string context = collector->GetName();
        Ptr<C> c = aggregator->GetObject<C>();
        NS_ASSERT_MSG(c != nullptr,
                      "Aggregator type " << aggregator->GetInstanceTypeId().GetName()
                                         << " is incompatible with the specified trace sink");
        if (!collector->TraceConnect(traceSourceName,
                                     context,
                                     MakeCallback(aggregatorTraceSink, c)))
        {
            return false;
        }
    }

    return true;
}

} // end of namespace ns3

#endif /* COLLECTOR_MAP_H */
