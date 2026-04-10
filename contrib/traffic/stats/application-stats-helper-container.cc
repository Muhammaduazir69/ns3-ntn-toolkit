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

#include "application-stats-helper-container.h"

#include "application-stats-delay-helper.h"
#include "application-stats-helper.h"
#include "application-stats-throughput-helper.h"

#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/string.h"

#include <map>
#include <sstream>
#include <string>

NS_LOG_COMPONENT_DEFINE("ApplicationStatsHelperContainer");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(ApplicationStatsHelperContainer);

ApplicationStatsHelperContainer::ApplicationStatsHelperContainer()
{
    NS_LOG_FUNCTION(this);
}

void
ApplicationStatsHelperContainer::DoDispose()
{
    NS_LOG_FUNCTION(this);
}

/*
 * The macro definitions following this comment block are used to define most
 * attributes of this class. Below is the list of attributes created using this
 * C++ pre-processing approach.
 *
 * - [Global,PerReceiver,PerSender] Throughput
 * - [Global,PerReceiver,PerSender] Delay
 * - Average [PerReceiver,PerSender] Throughput
 * - Average [PerReceiver,PerSender] Delay
 *
 * Also check the Doxygen documentation of this class for more information.
 */

#define ST_HE_CL ApplicationStatsHelper
#define ST_HE_CO_CL ApplicationStatsHelperContainer

#define ADD_APPLICATION_STATS_BASIC_OUTPUT_CHECKER                                                 \
  MakeEnumChecker (ST_HE_CL::OUTPUT_NONE,           "NONE",                   \
                   ST_HE_CL::OUTPUT_SCALAR_FILE,    "SCALAR_FILE",            \
                   ST_HE_CL::OUTPUT_SCATTER_FILE,   "SCATTER_FILE",           \
                   ST_HE_CL::OUTPUT_SCATTER_PLOT,   "SCATTER_PLOT"))

#define ADD_APPLICATION_STATS_DISTRIBUTION_OUTPUT_CHECKER                                          \
  MakeEnumChecker (ST_HE_CL::OUTPUT_NONE,           "NONE",                   \
                   ST_HE_CL::OUTPUT_SCALAR_FILE,    "SCALAR_FILE",            \
                   ST_HE_CL::OUTPUT_SCATTER_FILE,   "SCATTER_FILE",           \
                   ST_HE_CL::OUTPUT_HISTOGRAM_FILE, "HISTOGRAM_FILE",         \
                   ST_HE_CL::OUTPUT_PDF_FILE,       "PDF_FILE",               \
                   ST_HE_CL::OUTPUT_CDF_FILE,       "CDF_FILE",               \
                   ST_HE_CL::OUTPUT_SCATTER_PLOT,   "SCATTER_PLOT",           \
                   ST_HE_CL::OUTPUT_HISTOGRAM_PLOT, "HISTOGRAM_PLOT",         \
                   ST_HE_CL::OUTPUT_PDF_PLOT,       "PDF_PLOT",               \
                   ST_HE_CL::OUTPUT_CDF_PLOT,       "CDF_PLOT"))

#define ADD_APPLICATION_STATS_AVERAGED_DISTRIBUTION_OUTPUT_CHECKER                                 \
  MakeEnumChecker (ST_HE_CL::OUTPUT_NONE,           "NONE",                   \
                   ST_HE_CL::OUTPUT_HISTOGRAM_FILE, "HISTOGRAM_FILE",         \
                   ST_HE_CL::OUTPUT_PDF_FILE,       "PDF_FILE",               \
                   ST_HE_CL::OUTPUT_CDF_FILE,       "CDF_FILE",               \
                   ST_HE_CL::OUTPUT_HISTOGRAM_PLOT, "HISTOGRAM_PLOT",         \
                   ST_HE_CL::OUTPUT_PDF_PLOT,       "PDF_PLOT",               \
                   ST_HE_CL::OUTPUT_CDF_PLOT,       "CDF_PLOT"))

#define ADD_APPLICATION_STATS_ATTRIBUTE_HEAD(id, desc)                                             \
  .AddAttribute (# id,                                                         \
                 std::string ("Enable the output of ") + desc,                \
                 EnumValue (ST_HE_CL::OUTPUT_NONE),                           \
                 MakeEnumAccessor<ApplicationStatsHelper::OutputType_t> (&ST_HE_CO_CL::Add ## id),

#define ADD_APPLICATION_STATS_ATTRIBUTES_BASIC_SET(id, desc)                                       \
    ADD_APPLICATION_STATS_ATTRIBUTE_HEAD(Global##id, std::string("global ") + desc)                \
    ADD_APPLICATION_STATS_BASIC_OUTPUT_CHECKER                                                     \
    ADD_APPLICATION_STATS_ATTRIBUTE_HEAD(PerSender##id, std::string("per sender ") + desc)         \
    ADD_APPLICATION_STATS_BASIC_OUTPUT_CHECKER                                                     \
    ADD_APPLICATION_STATS_ATTRIBUTE_HEAD(PerReceiver##id, std::string("per receiver ") + desc)     \
    ADD_APPLICATION_STATS_BASIC_OUTPUT_CHECKER

#define ADD_APPLICATION_STATS_ATTRIBUTES_DISTRIBUTION_SET(id, desc)                                \
    ADD_APPLICATION_STATS_ATTRIBUTE_HEAD(Global##id, std::string("global ") + desc)                \
    ADD_APPLICATION_STATS_DISTRIBUTION_OUTPUT_CHECKER                                              \
    ADD_APPLICATION_STATS_ATTRIBUTE_HEAD(PerSender##id, std::string("per sender ") + desc)         \
    ADD_APPLICATION_STATS_DISTRIBUTION_OUTPUT_CHECKER                                              \
    ADD_APPLICATION_STATS_ATTRIBUTE_HEAD(PerReceiver##id, std::string("per receiver ") + desc)     \
    ADD_APPLICATION_STATS_DISTRIBUTION_OUTPUT_CHECKER

#define ADD_SAT_STATS_ATTRIBUTES_AVERAGED_DISTRIBUTION_SET(id, desc)                               \
    ADD_APPLICATION_STATS_ATTRIBUTE_HEAD(AverageSender##id, std::string("average sender ") + desc) \
    ADD_APPLICATION_STATS_AVERAGED_DISTRIBUTION_OUTPUT_CHECKER                                     \
    ADD_APPLICATION_STATS_ATTRIBUTE_HEAD(AverageReceiver##id,                                      \
                                         std::string("average receiver ") + desc)                  \
    ADD_APPLICATION_STATS_AVERAGED_DISTRIBUTION_OUTPUT_CHECKER

TypeId // static
ApplicationStatsHelperContainer::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ApplicationStatsHelperContainer")
            .SetParent<Object>()
            .AddAttribute("Name",
                          "String to be prepended on every output file name",
                          StringValue("stat"),
                          MakeStringAccessor(&ApplicationStatsHelperContainer::SetName,
                                             &ApplicationStatsHelperContainer::GetName),
                          MakeStringChecker())
            .AddAttribute(
                "TraceSourceName",
                "The name of the application's trace source which produce the required information",
                StringValue(""),
                MakeStringAccessor(&ApplicationStatsHelperContainer::SetTraceSourceName,
                                   &ApplicationStatsHelperContainer::GetTraceSourceName),
                MakeStringChecker())

        // Throughput statistics.
        ADD_APPLICATION_STATS_ATTRIBUTES_BASIC_SET(Throughput, "throughput statistics")
            ADD_SAT_STATS_ATTRIBUTES_AVERAGED_DISTRIBUTION_SET(Throughput, "throughput statistics")

        // Delay statistics.
        ADD_APPLICATION_STATS_ATTRIBUTES_DISTRIBUTION_SET(Delay, "packet delay statistics")
        //    ADD_SAT_STATS_ATTRIBUTES_AVERAGED_DISTRIBUTION_SET (Delay,
        //                                                        "packet delay statistics")

        ;
    return tid;
}

void
ApplicationStatsHelperContainer::SetName(std::string name)
{
    NS_LOG_FUNCTION(this << name);

    // convert all spaces and slashes in the name to underscores
    for (size_t pos = name.find_first_of(" /"); pos != std::string::npos;
         pos = name.find_first_of(" /", pos + 1, 1))
    {
        name[pos] = '_';
    }

    m_name = name;
}

std::string
ApplicationStatsHelperContainer::GetName() const
{
    return m_name;
}

void
ApplicationStatsHelperContainer::SetTraceSourceName(std::string traceSourceName)
{
    NS_LOG_FUNCTION(this << traceSourceName);
    m_traceSourceName = traceSourceName;
}

std::string
ApplicationStatsHelperContainer::GetTraceSourceName() const
{
    return m_traceSourceName;
}

/*
 * The macro definitions following this comment block are used to declare the
 * majority of methods in this class. Below is the list of the class methods
 * created using this C++ pre-processing approach.
 *
 * - Add [Global,PerReceiver,PerSender] Throughput
 * - AddAverage [Receiver,Sender] Throughput
 * - Add [Global,PerReceiver,PerSender] Delay
 * - AddAverage [Receiver,Sender] Delay
 *
 * Also check the Doxygen documentation of this class for more information.
 */

#define APPLICATION_STATS_METHOD_DEFINITION(id, name)                                              \
    void ApplicationStatsHelperContainer::AddGlobal##id(ApplicationStatsHelper::OutputType_t type) \
    {                                                                                              \
        NS_LOG_FUNCTION(this << ApplicationStatsHelper::GetOutputTypeName(type));                  \
        if (type != ApplicationStatsHelper::OUTPUT_NONE)                                           \
        {                                                                                          \
            Ptr<ApplicationStats##id##Helper> stat = CreateObject<ApplicationStats##id##Helper>(); \
            stat->SetName(m_name + "-global-" + name + GetOutputTypeSuffix(type));                 \
            stat->SetTraceSourceName(m_traceSourceName);                                           \
            stat->SetIdentifierType(ApplicationStatsHelper::IDENTIFIER_GLOBAL);                    \
            stat->SetOutputType(type);                                                             \
            stat->SetSenderInformation(m_senderInfo);                                              \
            stat->SetReceiverInformation(m_receiverInfo);                                          \
            stat->Install();                                                                       \
            m_stats.push_back(stat);                                                               \
        }                                                                                          \
    }                                                                                              \
    void ApplicationStatsHelperContainer::AddPerReceiver##id(                                      \
        ApplicationStatsHelper::OutputType_t type)                                                 \
    {                                                                                              \
        NS_LOG_FUNCTION(this << ApplicationStatsHelper::GetOutputTypeName(type));                  \
        if (type != ApplicationStatsHelper::OUTPUT_NONE)                                           \
        {                                                                                          \
            Ptr<ApplicationStats##id##Helper> stat = CreateObject<ApplicationStats##id##Helper>(); \
            stat->SetName(m_name + "-per-receiver-" + name + GetOutputTypeSuffix(type));           \
            stat->SetTraceSourceName(m_traceSourceName);                                           \
            stat->SetIdentifierType(ApplicationStatsHelper::IDENTIFIER_RECEIVER);                  \
            stat->SetOutputType(type);                                                             \
            stat->SetSenderInformation(m_senderInfo);                                              \
            stat->SetReceiverInformation(m_receiverInfo);                                          \
            stat->Install();                                                                       \
            m_stats.push_back(stat);                                                               \
        }                                                                                          \
    }                                                                                              \
    void ApplicationStatsHelperContainer::AddPerSender##id(                                        \
        ApplicationStatsHelper::OutputType_t type)                                                 \
    {                                                                                              \
        NS_LOG_FUNCTION(this << ApplicationStatsHelper::GetOutputTypeName(type));                  \
        if (type != ApplicationStatsHelper::OUTPUT_NONE)                                           \
        {                                                                                          \
            Ptr<ApplicationStats##id##Helper> stat = CreateObject<ApplicationStats##id##Helper>(); \
            stat->SetName(m_name + "-per-sender-" + name + GetOutputTypeSuffix(type));             \
            stat->SetTraceSourceName(m_traceSourceName);                                           \
            stat->SetIdentifierType(ApplicationStatsHelper::IDENTIFIER_SENDER);                    \
            stat->SetOutputType(type);                                                             \
            stat->SetSenderInformation(m_senderInfo);                                              \
            stat->SetReceiverInformation(m_receiverInfo);                                          \
            stat->Install();                                                                       \
            m_stats.push_back(stat);                                                               \
        }                                                                                          \
    }

#define APPLICATION_STATS_AVERAGE_METHOD_DEFINITION(id, name)                                      \
    void ApplicationStatsHelperContainer::AddAverageReceiver##id(                                  \
        ApplicationStatsHelper::OutputType_t type)                                                 \
    {                                                                                              \
        NS_LOG_FUNCTION(this << ApplicationStatsHelper::GetOutputTypeName(type));                  \
        if (type != ApplicationStatsHelper::OUTPUT_NONE)                                           \
        {                                                                                          \
            Ptr<ApplicationStats##id##Helper> stat = CreateObject<ApplicationStats##id##Helper>(); \
            stat->SetName(m_name + "-average-receiver-" + name + GetOutputTypeSuffix(type));       \
            stat->SetTraceSourceName(m_traceSourceName);                                           \
            stat->SetIdentifierType(ApplicationStatsHelper::IDENTIFIER_RECEIVER);                  \
            stat->SetOutputType(type);                                                             \
            stat->SetAveragingMode(true);                                                          \
            stat->SetSenderInformation(m_senderInfo);                                              \
            stat->SetReceiverInformation(m_receiverInfo);                                          \
            stat->Install();                                                                       \
            m_stats.push_back(stat);                                                               \
        }                                                                                          \
    }                                                                                              \
    void ApplicationStatsHelperContainer::AddAverageSender##id(                                    \
        ApplicationStatsHelper::OutputType_t type)                                                 \
    {                                                                                              \
        NS_LOG_FUNCTION(this << ApplicationStatsHelper::GetOutputTypeName(type));                  \
        if (type != ApplicationStatsHelper::OUTPUT_NONE)                                           \
        {                                                                                          \
            Ptr<ApplicationStats##id##Helper> stat = CreateObject<ApplicationStats##id##Helper>(); \
            stat->SetName(m_name + "-average-sender-" + name + GetOutputTypeSuffix(type));         \
            stat->SetTraceSourceName(m_traceSourceName);                                           \
            stat->SetIdentifierType(ApplicationStatsHelper::IDENTIFIER_SENDER);                    \
            stat->SetOutputType(type);                                                             \
            stat->SetAveragingMode(true);                                                          \
            stat->SetSenderInformation(m_senderInfo);                                              \
            stat->SetReceiverInformation(m_receiverInfo);                                          \
            stat->Install();                                                                       \
            m_stats.push_back(stat);                                                               \
        }                                                                                          \
    }

// Throughput statistics.
APPLICATION_STATS_METHOD_DEFINITION(Throughput, "throughput")
APPLICATION_STATS_AVERAGE_METHOD_DEFINITION(Throughput, "throughput")

// Delay statistics.
APPLICATION_STATS_METHOD_DEFINITION(Delay, "delay")

// APPLICATION_STATS_AVERAGE_METHOD_DEFINITION (Delay, "delay")

std::string // static
ApplicationStatsHelperContainer::GetOutputTypeSuffix(
    ApplicationStatsHelper::OutputType_t outputType)
{
    switch (outputType)
    {
    case ApplicationStatsHelper::OUTPUT_NONE:
        return "";

    case ApplicationStatsHelper::OUTPUT_SCALAR_FILE:
    case ApplicationStatsHelper::OUTPUT_SCALAR_PLOT:
        return "-scalar";

    case ApplicationStatsHelper::OUTPUT_SCATTER_FILE:
    case ApplicationStatsHelper::OUTPUT_SCATTER_PLOT:
        return "-scatter";

    case ApplicationStatsHelper::OUTPUT_HISTOGRAM_FILE:
    case ApplicationStatsHelper::OUTPUT_HISTOGRAM_PLOT:
        return "-histogram";

    case ApplicationStatsHelper::OUTPUT_PDF_FILE:
    case ApplicationStatsHelper::OUTPUT_PDF_PLOT:
        return "-pdf";

    case ApplicationStatsHelper::OUTPUT_CDF_FILE:
    case ApplicationStatsHelper::OUTPUT_CDF_PLOT:
        return "-cdf";

    default:
        NS_FATAL_ERROR("ApplicationStatsHelperContainer - Invalid output type");
        break;
    }

    NS_FATAL_ERROR("ApplicationStatsHelperContainer - Invalid output type");
    return "";
}

// SENDER APPLICATIONS ////////////////////////////////////////////////////////

void
ApplicationStatsHelperContainer::AddSenderApplication(Ptr<Application> application,
                                                      std::string identifier)
// Default value(s): identifier = ""
{
    NS_LOG_FUNCTION(this << application << identifier);

    if (identifier.empty())
    {
        // Assign a default identifier: node ID.
        Ptr<Node> node = application->GetNode();
        NS_ASSERT_MSG(node != nullptr, "Application is not attached to any Node");
        std::ostringstream oss;
        oss << node->GetId();
        identifier = oss.str();
    }

    NS_ASSERT(!identifier.empty());
    std::map<std::string, ApplicationContainer>::iterator it = m_senderInfo.find(identifier);

    if (it == m_senderInfo.end())
    {
        m_senderInfo[identifier] = ApplicationContainer(application);
    }
    else
    {
        it->second.Add(application);
    }
}

void
ApplicationStatsHelperContainer::AddSenderApplications(ApplicationContainer container,
                                                       bool isGroup,
                                                       std::string groupIdentifier)
// Default value(s): isGroup = false, groupIdentifier = ""
{
    NS_LOG_FUNCTION(this << container.GetN() << isGroup << groupIdentifier);

    if (isGroup)
    {
        NS_FATAL_ERROR("Group identifier is not supported at the moment");
    }

    for (ApplicationContainer::Iterator it = container.Begin(); it != container.End(); ++it)
    {
        AddSenderApplication(*it); // using default identifier
    }
}

void
ApplicationStatsHelperContainer::AddSenderNode(Ptr<Node> node,
                                               bool isGroup,
                                               std::string groupIdentifier)
// Default value(s): isGroup = false, groupIdentifier = ""
{
    NS_LOG_FUNCTION(this << node << node->GetId() << groupIdentifier);

    if (isGroup)
    {
        NS_FATAL_ERROR("Group identifier is not supported at the moment");
    }

    for (uint32_t i = 0; i < node->GetNApplications(); i++)
    {
        std::ostringstream oss;
        oss << node->GetId() << "-" << i;
        AddSenderApplication(node->GetApplication(i), oss.str());
    }
}

void
ApplicationStatsHelperContainer::AddSenderNodes(NodeContainer container,
                                                bool isGroup,
                                                std::string groupIdentifier)
// Default value(s): isGroup = false, groupIdentifier = ""
{
    NS_LOG_FUNCTION(this << container.GetN() << isGroup << groupIdentifier);

    if (isGroup)
    {
        NS_FATAL_ERROR("Group identifier is not supported at the moment");
    }

    for (NodeContainer::Iterator it = container.Begin(); it != container.End(); ++it)
    {
        AddSenderNode(*it); // using default identifier
    }
}

// RECEIVER APPLICATIONS //////////////////////////////////////////////////////

void
ApplicationStatsHelperContainer::AddReceiverApplication(Ptr<Application> application,
                                                        std::string identifier)
// Default value(s): identifier = ""
{
    NS_LOG_FUNCTION(this << application << identifier);

    if (identifier.empty())
    {
        // Assign a default identifier: node ID and application ID.
        Ptr<Node> node = application->GetNode();
        NS_ASSERT_MSG(node != nullptr, "Application is not attached to any Node");
        for (uint32_t i = 0; i < node->GetNApplications(); i++)
        {
            if (node->GetApplication(i) == application)
            {
                std::ostringstream oss;
                oss << node->GetId() << "-" << i;
                identifier = oss.str();
                i = node->GetNApplications(); // this exits the loop
            }
        }
    }

    NS_ASSERT(!identifier.empty());
    std::map<std::string, ApplicationContainer>::iterator it = m_receiverInfo.find(identifier);

    if (it == m_receiverInfo.end())
    {
        m_receiverInfo[identifier] = ApplicationContainer(application);
    }
    else
    {
        it->second.Add(application);
    }
}

void
ApplicationStatsHelperContainer::AddReceiverApplications(ApplicationContainer container,
                                                         bool isGroup,
                                                         std::string groupIdentifier)
// Default value(s): isGroup = false, groupIdentifier = ""
{
    NS_LOG_FUNCTION(this << container.GetN() << isGroup << groupIdentifier);

    if (isGroup)
    {
        NS_FATAL_ERROR("Group identifier is not supported at the moment");
    }

    for (ApplicationContainer::Iterator it = container.Begin(); it != container.End(); ++it)
    {
        AddReceiverApplication(*it); // using default identifier
    }
}

void
ApplicationStatsHelperContainer::AddReceiverNode(Ptr<Node> node,
                                                 bool isGroup,
                                                 std::string groupIdentifier)
// Default value(s): isGroup = false, groupIdentifier = ""
{
    NS_LOG_FUNCTION(this << node << node->GetId() << groupIdentifier);

    if (isGroup)
    {
        NS_FATAL_ERROR("Group identifier is not supported at the moment");
    }

    for (uint32_t i = 0; i < node->GetNApplications(); i++)
    {
        std::ostringstream oss;
        oss << node->GetId() << "-" << i;
        AddReceiverApplication(node->GetApplication(i), oss.str());
    }
}

void
ApplicationStatsHelperContainer::AddReceiverNodes(NodeContainer container,
                                                  bool isGroup,
                                                  std::string groupIdentifier)
// Default value(s): isGroup = false, groupIdentifier = ""
{
    NS_LOG_FUNCTION(this << container.GetN() << isGroup << groupIdentifier);

    if (isGroup)
    {
        NS_FATAL_ERROR("Group identifier is not supported at the moment");
    }

    for (NodeContainer::Iterator it = container.Begin(); it != container.End(); ++it)
    {
        AddReceiverNode(*it); // using default identifier
    }
}

} // end of namespace ns3
