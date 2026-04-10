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

#ifndef APPLICATION_STATS_HELPER_H
#define APPLICATION_STATS_HELPER_H

#include "ns3/application-container.h"
#include "ns3/callback.h"
#include "ns3/collector-map.h"
#include "ns3/object.h"
#include "ns3/probe.h"
#include "ns3/ptr.h"
#include "ns3/type-id.h"

#include <list>
#include <map>
#include <string>

namespace ns3
{

class DataCollectionObject;
class Address;

/**
 * @ingroup traffic
 * @defgroup applicationstats Application Statistics
 *
 * Data Collection Framework (DCF) implementation on Application module. For
 * usage in simulation script, see ApplicationStatsHelperContainer.
 *
 * @warning ApplicationStatsHelperContainer takes care of setting the attributes
 *          `Name`, `IdentifierType`, and `OutputType`. Thus it's *not*
 *          recommended to manually set the values of these attributes while
 *          using ApplicationStatsHelperContainer.
 */

/**
 * @ingroup applicationstats
 * @brief Parent abstract class of all application statistics helpers.
 *
 * A helper is responsible to locate source objects, create probes, collectors,
 * and aggregators, and connect them together in a proper way to produce the
 * required statistics.
 *
 * As shown in the example code below, the helper requires several inputs.
 * After all the necessary inputs have been set, the statistics can be started
 * into action by invoking Install().
 * @code
 *     ApplicationContainer txApps;
 *     ApplicationContainer rxApps;
 *     // ... (snip) ...
 *     std::map<std::string, ApplicationContainer> txInfo;
 *     txInfo["sender-1"] = txApps;
 *     std::map<std::string, ApplicationContainer> rxInfo;
 *     rxInfo["receiver-1"] = rxApps;
 *     Ptr<ApplicationStatsThroughputHelper> stat
 *         = CreateObject<ApplicationStatsThroughputHelper> ();
 *     stat->SetName ("name");
 *     stat->SetTraceSourceName ("Rx");
 *     stat->SetSenderInformation (txApps);
 *     stat->SetReceiverInformation (rxApps);
 *     stat->SetIdentifierType (ApplicationStatsHelper::IDENTIFIER_SENDER);
 *     stat->SetOutputType (ApplicationStatsHelper::OUTPUT_SCALAR_FILE);
 *     stat->Install ();
 *     m_stats.push_back (stat);
 * @endcode
 *
 * However, it's recommended to use the ApplicationStatsHelperContainer class
 * to automatically handle the above.
 *
 * This parent abstract class hosts several protected methods which are
 * intended to simplify the development of child classes by sharing common
 * functions.
 *
 * @see ApplicationStatsHelperContainer
 */
class ApplicationStatsHelper : public Object
{
  public:
    // COMMON ENUM DATA TYPES ///////////////////////////////////////////////////

    /**
     * @enum IdentifierType_t
     * @brief Possible categorization of statistics output.
     */
    typedef enum
    {
        IDENTIFIER_GLOBAL = 0,
        IDENTIFIER_SENDER,
        IDENTIFIER_RECEIVER
    } IdentifierType_t;

    /**
     * @param identifierType an arbitrary identifier type.
     * @return representation of the identifier type in string.
     */
    static std::string GetIdentifierTypeName(IdentifierType_t identifierType);

    /**
     * @enum OutputType_t
     * @brief Possible types and formats of statistics output.
     */
    typedef enum
    {
        OUTPUT_NONE = 0,
        OUTPUT_SCALAR_FILE,
        OUTPUT_SCATTER_FILE,
        OUTPUT_HISTOGRAM_FILE,
        OUTPUT_PDF_FILE, // probability distribution function
        OUTPUT_CDF_FILE, // cumulative distribution function
        OUTPUT_SCALAR_PLOT,
        OUTPUT_SCATTER_PLOT,
        OUTPUT_HISTOGRAM_PLOT,
        OUTPUT_PDF_PLOT, // probability distribution function
        OUTPUT_CDF_PLOT, // cumulative distribution function
    } OutputType_t;

    /**
     * @param outputType an arbitrary output type.
     * @return representation of the output type in string.
     */
    static std::string GetOutputTypeName(OutputType_t outputType);

    // CONSTRUCTOR AND DESTRUCTOR ///////////////////////////////////////////////

    /// Creates a new helper instance.
    ApplicationStatsHelper();

    /// Destructor.
    virtual ~ApplicationStatsHelper();

    // inherited from ObjectBase base class
    static TypeId GetTypeId();

    // PUBLIC METHODS ///////////////////////////////////////////////////////////

    /**
     * @brief Provide the helper pointers to applications who will act as the
     *        senders.
     * @param info pairs of a name and a group of applications.
     *
     * Subsequent calls will replace any existing sender information that have
     * been provided before.
     *
     * Pointers to the same application instance may be added as both sender and
     * receiver.
     */
    void SetSenderInformation(std::map<std::string, ApplicationContainer> info);

    /**
     * @brief Provide the helper pointers to applications who will act as the
     *        receivers.
     * @param info pairs of a name and a group of applications.
     *
     * Subsequent calls will replace any existing receiver information that have
     * been provided before.
     *
     * Pointers to the same application instance may be added as both sender and
     * receiver.
     */
    void SetReceiverInformation(std::map<std::string, ApplicationContainer> info);

    /**
     * @brief Install probes, collectors, and aggregators.
     *
     * Behaviour should be implemented by child class in DoInstall().
     */
    void Install();

    // SETTER AND GETTER METHODS ////////////////////////////////////////////////

    /**
     * @param name string to be prepended on every output file name.
     */
    void SetName(std::string name);

    /**
     * @return the name of this helper instance.
     */
    std::string GetName() const;

    /**
     * @param traceSourceName the name of the application's trace source
     *                        which produces the required data.
     */
    void SetTraceSourceName(std::string traceSourceName);

    /**
     * @return the name of the application's trace source from whom this helper
     *         instance will receive data.
     */
    std::string GetTraceSourceName() const;

    /**
     * @param identifierType categorization of statistics output.
     * @warning Does not have any effect if invoked after Install().
     */
    void SetIdentifierType(IdentifierType_t identifierType);

    /**
     * @return the currently active categorization of statistics output.
     */
    IdentifierType_t GetIdentifierType() const;

    /**
     * @param outputType types and formats of statistics output.
     * @warning Does not have any effect if invoked after Install().
     */
    void SetOutputType(OutputType_t outputType);

    /**
     * @return the currently active types and formats of statistics output.
     */
    OutputType_t GetOutputType() const;

    /**
     * @return true if Install() has been invoked, otherwise false.
     */
    bool IsInstalled() const;

  protected:
    /**
     * @brief Install the probes, collectors, and aggregators necessary to
     *        produce the statistics output.
     *
     * An abstract method of ApplicationStatsHelper which must be implemented by
     * child classes. It will be invoked by Install().
     */
    virtual void DoInstall() = 0;

    /**
     * @brief Create the aggregator according to the output type.
     * @param aggregatorTypeId the type of aggregator to be created.
     * @param n1 the name of the attribute to be set on the aggregator created.
     * @param v1 the value of the attribute to be set on the aggregator created.
     * @param n2 the name of the attribute to be set on the aggregator created.
     * @param v2 the value of the attribute to be set on the aggregator created.
     * @param n3 the name of the attribute to be set on the aggregator created.
     * @param v3 the value of the attribute to be set on the aggregator created.
     * @param n4 the name of the attribute to be set on the aggregator created.
     * @param v4 the value of the attribute to be set on the aggregator created.
     * @param n5 the name of the attribute to be set on the aggregator created.
     * @param v5 the value of the attribute to be set on the aggregator created.
     * @return the created aggregator.
     *
     * The created aggregator is stored in #m_aggregator. It can be retrieved
     * from outside using GetAggregator().
     */
    Ptr<DataCollectionObject> CreateAggregator(std::string aggregatorTypeId,
                                               std::string n1 = "",
                                               const AttributeValue& v1 = EmptyAttributeValue(),
                                               std::string n2 = "",
                                               const AttributeValue& v2 = EmptyAttributeValue(),
                                               std::string n3 = "",
                                               const AttributeValue& v3 = EmptyAttributeValue(),
                                               std::string n4 = "",
                                               const AttributeValue& v4 = EmptyAttributeValue(),
                                               std::string n5 = "",
                                               const AttributeValue& v5 = EmptyAttributeValue());

    /**
     * @brief Create one collector instance for each identifier in the simulation.
     * @param collectorMap the CollectorMap where the collectors will be created.
     * @return number of collector instances created.
     *
     * The identifier is determined by the currently active identifier type, as
     * previously selected by SetIdentifierType() method or `IdentifierType`
     * attribute. Then the method searches the sender and receiver information
     * provided by the SetSenderInformation() and SetReceiverInformation()
     * methods for such identifier. For each of the found identifiers, the method
     * creates a collector instance for it and put the collector instance into
     * the CollectorMap.
     *
     * The collector instances in the map are simply labelled using running
     * integers starting from 0.
     */
    uint32_t CreateCollectorPerIdentifier(CollectorMap& collectorMap) const;

    /**
     * @brief Create a probe attached to every receiver application and connected
     *        to a collector.
     * @param probeOutputName the name of the trace source of the probe to be
     *                        connected with the collector.
     * @param collectorMap a map containing the collectors.
     * @param collectorTraceSink a pointer to a function of the collectors in the
     *                           target map which acts as a trace sink.
     * @param probeList an output argument of this function, which is a list of
     *                  probes where the newly created probes will be pushed.
     * @return number of probes created.
     *
     * The type of probe to be created (must be a child class of Probe) is
     * specified as a template argument to the method call. For example, below we
     * create an ApplicationPacketProbe for each receiver:
     * @code
     *     uint32_t n = SetupProbesAtReceiver<ApplicationPacketProbe> (
     *                      "OutputBytes",
     *                      collectorMap,
     *                      &ScalarCollector::TraceSinkUinteger32,
     *                      m_probes);
     * @endcode
     *
     * The probe will listen to each Application previously specified by
     * SetReceiverInformation(). The trace source which the probe listens to is
     * specified using SetTraceSourceName().
     *
     * @warning This method is only applicable for `GLOBAL` and `RECEIVER`
     *          identifiers. In addition, the number of collectors in the
     *          `collectorMap` argument must match the number of identifiers,
     *          i.e., the CreateCollectorPerIdentifier() method should be
     *          called beforehand.
     */
    template <typename P, typename Q, typename R, typename C>
    uint32_t SetupProbesAtReceiver(std::string probeOutputName,
                                   CollectorMap& collectorMap,
                                   R (C::*collectorTraceSink)(Q, Q),
                                   std::list<Ptr<Probe>>& probeList);

    /**
     * @brief Connect the trace source of every receiver application to a given
     *        callback function.
     * @param cb a callback function whose second argument is the sender address.
     * @return number of trace sources connected with the callback.
     *
     * The callback will listen to each Application previously specified by
     * SetReceiverInformation(). The trace source which the callback listens to
     * is specified using SetTraceSourceName().
     *
     * The second argument of the callback is the address of the sender, which
     * is usually utilized by the callback to determine the sender of the packet,
     * so that it can pass data samples to the right collector according to the
     * sender, i.e., when `IDENTIFIER_SENDER` is selected as the identifier type.
     */
    template <typename Q>
    uint32_t SetupListenersAtReceiver(Callback<void, Q, const Address&> cb);

    /// Internal map of sender applications, indexed by their names.
    std::map<std::string, ApplicationContainer> m_senderInfo;

    /// Internal map of receiver applications, indexed by their names.
    std::map<std::string, ApplicationContainer> m_receiverInfo;

  private:
    std::string m_name;                ///<
    IdentifierType_t m_identifierType; ///<
    OutputType_t m_outputType;         ///<
    std::string m_traceSourceName;     ///<
    bool m_isInstalled;                ///<

}; // end of class ApplicationStatsHelper

// TEMPLATE METHOD DEFINITIONS ////////////////////////////////////////////////

template <typename P, typename Q, typename R, typename C>
uint32_t
ApplicationStatsHelper::SetupProbesAtReceiver(std::string probeOutputName,
                                              CollectorMap& collectorMap,
                                              R (C::*collectorTraceSink)(Q, Q),
                                              std::list<Ptr<Probe>>& probeList)
{
    if (P::GetTypeId().GetParent() != TypeId::LookupByName("ns3::Probe"))
    {
        NS_FATAL_ERROR("Invalid probe type");
    }

    /*
     * If we use GLOBAL as identifier, then there's only 1 collector in the map.
     * If we use RECEIVER as identifier, then there are the same number of
     * collectors as the number of receiver applications.
     */
    NS_ASSERT(((m_identifierType == ApplicationStatsHelper::IDENTIFIER_GLOBAL) &&
               (collectorMap.GetN() == 1)) ||
              ((m_identifierType == ApplicationStatsHelper::IDENTIFIER_RECEIVER) &&
               (collectorMap.GetN() == m_receiverInfo.size())));

    uint32_t n = 0;
    uint32_t identifier = 0;

    std::map<std::string, ApplicationContainer>::const_iterator it1;
    for (it1 = m_receiverInfo.begin(); it1 != m_receiverInfo.end(); ++it1)
    {
        for (ApplicationContainer::Iterator it2 = it1->second.Begin(); it2 != it1->second.End();
             ++it2)
        {
            if ((*it2)->GetInstanceTypeId().LookupTraceSourceByName(m_traceSourceName) != nullptr)
            {
                // Create the probe.
                Ptr<P> probe = CreateObject<P>();
                probe->SetName(it1->first);

                // Connect the object to the probe.
                if (probe->ConnectByObject(m_traceSourceName, (*it2)) &&
                    collectorMap.ConnectWithProbe(probe,
                                                  probeOutputName,
                                                  identifier,
                                                  collectorTraceSink))
                {
                    probeList.push_back(probe->template GetObject<Probe>());
                    n++;
                }
            }

        } // end of `for (it2 = it1->second)`

        if (m_identifierType == ApplicationStatsHelper::IDENTIFIER_RECEIVER)
        {
            identifier++; // Move to the next collector.
        }

    } // end of `for (it1 = m_receiverInfo)`

    return n;
}

template <typename Q>
uint32_t
ApplicationStatsHelper::SetupListenersAtReceiver(Callback<void, Q, const Address&> cb)
{
    uint32_t n = 0;

    std::map<std::string, ApplicationContainer>::const_iterator it1;
    for (it1 = m_receiverInfo.begin(); it1 != m_receiverInfo.end(); ++it1)
    {
        for (ApplicationContainer::Iterator it2 = it1->second.Begin(); it2 != it1->second.End();
             ++it2)
        {
            if ((*it2)->GetInstanceTypeId().LookupTraceSourceByName(m_traceSourceName) != nullptr &&
                (*it2)->TraceConnectWithoutContext(m_traceSourceName, cb))
            {
                n++;
            }
        }
    }

    return n;
}

} // end of namespace ns3

#endif /* APPLICATION_STATS_HELPER_H */
