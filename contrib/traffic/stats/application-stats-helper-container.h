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

#ifndef APPLICATION_STATS_HELPER_CONTAINER_H
#define APPLICATION_STATS_HELPER_CONTAINER_H

#include "application-stats-helper.h"

#include "ns3/application-container.h"
#include "ns3/node-container.h"
#include "ns3/object.h"
#include "ns3/ptr.h"

#include <list>
#include <map>
#include <string>

namespace ns3
{

/*
 * The macro definitions following this comment block are used to declare the
 * majority of methods in this class. Below is the list of the class methods
 * created using this C++ pre-processing approach.
 *
 * - Add [Global,PerReceiver,PerSender] Throughput
 * - Add [Global,PerReceiver,PerSender] Delay
 *
 * Also check the Doxygen documentation of this class for more information.
 */

#define APPLICATION_STATS_METHOD_DECLARATION(id)                                                   \
    void AddGlobal##id(ApplicationStatsHelper::OutputType_t outputType);                           \
    void AddPerReceiver##id(ApplicationStatsHelper::OutputType_t outputType);                      \
    void AddPerSender##id(ApplicationStatsHelper::OutputType_t outputType);

/**
 * @ingroup applicationstats
 * @brief Container of ApplicationStatsHelper instances.
 *
 * The container is initially empty upon creation. ApplicationStatsHelper
 * instances can be added into the container using attributes or class methods.
 * The following example creates a new container and adds two different
 * statistics instances:
 * @code
 *     ApplicationContainer txApps;
 *     ApplicationContainer rxApps;
 *     // ... (snip) ...
 *     Ptr<ApplicationStatsHelperContainer> stat
 *         = CreateObject<ApplicationStatsHelperContainer> ();
 *     stat->SetTraceSourceName ("Rx");
 *     stat->AddSenderApplication (app);
 *     stat->AddReceiverApplications (sinkApps);
 *     stat->AddPerSenderThroughput (ApplicationStatsHelper::OUTPUT_SCATTER_FILE);
 *     stat->AddPerReceiverThroughput (ApplicationStatsHelper::OUTPUT_SCATTER_FILE);
 * @endcode
 *
 * The value of the attributes and the arguments of the class methods are the
 * desired output type (e.g., scalar, scatter, histogram, files, plots, etc.).
 *
 * The output files will be named in a certain pattern using the name set in
 * the `Name` attribute or SetName() method. The default name is "stat", e.g.,
 * which will produce output files with the names such as
 * `stat-per-receiver-throughput-scalar.txt`,
 * `stat-per-receiver-delay-cdf-receiver-1.txt`, etc.
 */
class ApplicationStatsHelperContainer : public Object
{
  public:
    /**
     * @brief Creates a new instance of container.
     */
    ApplicationStatsHelperContainer();

    // inherited from ObjectBase base class
    static TypeId GetTypeId();

    /**
     * @param name a string prefix to be prepended on every output file name.
     */
    void SetName(std::string name);

    /**
     * @return a string prefix prepended on every output file name.
     */
    std::string GetName() const;

    /**
     * @param traceSourceName the name of the application's trace source
     *                        which produces the required data.
     *
     * There is no standard trace source name between different applications, so
     * the name that must be provided to this method depends on the application
     * type. Please refer to the intended statistics class (i.e., child class of
     * ApplicationStatsHelper) for the expected trace source signature and
     * semantic, and then seek (or implement) the trace source in the
     * application class.
     *
     * For example, throughput statistics (see ApplicationStatsThroughputHelper)
     * require a trace source which exports a pointer to received packet and a
     * reference to the address of the packet sender. When used together with
     * PacketSink application, the right trace source to be used is `Rx`, because
     * it matches the expected signature and semantic.
     */
    void SetTraceSourceName(std::string traceSourceName);

    /**
     * @return the name of the application's trace source from whom this helper
     *         instance will receive data.
     */
    std::string GetTraceSourceName() const;

    // Throughput statistics.
    APPLICATION_STATS_METHOD_DECLARATION(Throughput)
    void AddAverageSenderThroughput(ApplicationStatsHelper::OutputType_t outputType);
    void AddAverageReceiverThroughput(ApplicationStatsHelper::OutputType_t outputType);

    // Delay statistics.
    APPLICATION_STATS_METHOD_DECLARATION(Delay)
    //  void AddAverageSenderDelay (ApplicationStatsHelper::OutputType_t outputType);
    //  void AddAverageReceiverDelay (ApplicationStatsHelper::OutputType_t outputType);

    /**
     * @param outputType an arbitrary output type.
     * @return a string suffix to be appended at the end of the corresponding
     *         output file for this output type.
     */
    static std::string GetOutputTypeSuffix(ApplicationStatsHelper::OutputType_t outputType);

    // SENDER APPLICATIONS //////////////////////////////////////////////////////

    /**
     * @brief Register the provided application as a sender.
     * @param application pointer to the application.
     * @param identifier name associated with this application (if the argument
     *                   is omitted, a unique name will be computed).
     *
     * The computed identifier name is a combination of the node ID and the
     * index of the application inside the node.
     */
    void AddSenderApplication(Ptr<Application> application, std::string identifier = "");

    /**
     * @brief Register the provided applications as senders.
     * @param container group of applications.
     * @param isGroup not supported yet, please use `false`.
     * @param groupIdentifier not supported yet, please leave as blank.
     */
    void AddSenderApplications(ApplicationContainer container,
                               bool isGroup = false,
                               std::string groupIdentifier = "");

    /**
     * @brief Register the applications in the provided node as senders.
     * @param node pointer to the node.
     * @param isGroup not supported yet, please use `false`.
     * @param groupIdentifier not supported yet, please leave as blank.
     */
    void AddSenderNode(Ptr<Node> node, bool isGroup = false, std::string groupIdentifier = "");

    /**
     * @brief Register the applications in the provided nodes as senders.
     * @param container group of nodes.
     * @param isGroup not supported yet, please use `false`.
     * @param groupIdentifier not supported yet, please leave as blank.
     */
    void AddSenderNodes(NodeContainer container,
                        bool isGroup = false,
                        std::string groupIdentifier = "");

    // RECEIVER APPLICATIONS ////////////////////////////////////////////////////

    /**
     * @brief Register the provided application as a receiver.
     * @param application pointer to the application.
     * @param identifier name associated with this application (if the argument
     *                   is omitted, a unique name will be computed).
     *
     * The computed identifier name is a combination of the node ID and the
     * index of the application inside the node.
     */
    void AddReceiverApplication(Ptr<Application> application, std::string identifier = "");

    /**
     * @brief Register the provided applications as receivers.
     * @param container group of applications.
     * @param isGroup not supported yet, please use `false`.
     * @param groupIdentifier not supported yet, please leave as blank.
     */
    void AddReceiverApplications(ApplicationContainer container,
                                 bool isGroup = false,
                                 std::string groupIdentifier = "");

    /**
     * @brief Register the applications in the provided node as receivers.
     * @param node pointer to the node.
     * @param isGroup not supported yet, please use `false`.
     * @param groupIdentifier not supported yet, please leave as blank.
     */
    void AddReceiverNode(Ptr<Node> node, bool isGroup = false, std::string groupIdentifier = "");

    /**
     * @brief Register the applications in the provided nodes as receivers.
     * @param container group of nodes.
     * @param isGroup not supported yet, please use `false`.
     * @param groupIdentifier not supported yet, please leave as blank.
     */
    void AddReceiverNodes(NodeContainer container,
                          bool isGroup = false,
                          std::string groupIdentifier = "");

  protected:
    // Inherited from Object base class
    virtual void DoDispose();

  private:
    /// Prefix of every ApplicationStatsHelper instance names and every output file.
    std::string m_name;

    /// The name of the application's trace source which produce the required information.
    std::string m_traceSourceName;

    /// Maintains the active ApplicationStatsHelper instances which have created.
    std::list<Ptr<const ApplicationStatsHelper>> m_stats;

    /// Internal map of sender applications, indexed by their names.
    std::map<std::string, ApplicationContainer> m_senderInfo;

    /// Internal map of receiver applications, indexed by their names.
    std::map<std::string, ApplicationContainer> m_receiverInfo;

}; // end of class ApplicationStatsHelperContainer

} // end of namespace ns3

#endif /* APPLICATION_STATS_HELPER_CONTAINER_H */
