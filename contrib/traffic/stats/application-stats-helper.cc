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

#include "application-stats-helper.h"

#include "ns3/address.h"
#include "ns3/data-collection-object.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/object-factory.h"
#include "ns3/string.h"

#include <map>
#include <sstream>
#include <string>

NS_LOG_COMPONENT_DEFINE("ApplicationStatsHelper");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(ApplicationStatsHelper);

std::string // static
ApplicationStatsHelper::GetIdentifierTypeName(
    ApplicationStatsHelper::IdentifierType_t identifierType)
{
    switch (identifierType)
    {
    case ApplicationStatsHelper::IDENTIFIER_GLOBAL:
        return "IDENTIFIER_GLOBAL";
    case ApplicationStatsHelper::IDENTIFIER_SENDER:
        return "IDENTIFIER_SENDER";
    case ApplicationStatsHelper::IDENTIFIER_RECEIVER:
        return "IDENTIFIER_RECEIVER";
    default:
        NS_FATAL_ERROR("ApplicationStatsHelper - Invalid identifier type");
        break;
    }

    NS_FATAL_ERROR("ApplicationStatsHelper - Invalid identifier type");
    return "";
}

std::string // static
ApplicationStatsHelper::GetOutputTypeName(ApplicationStatsHelper::OutputType_t outputType)
{
    switch (outputType)
    {
    case ApplicationStatsHelper::OUTPUT_NONE:
        return "OUTPUT_NONE";
    case ApplicationStatsHelper::OUTPUT_SCALAR_FILE:
        return "OUTPUT_SCALAR_FILE";
    case ApplicationStatsHelper::OUTPUT_SCATTER_FILE:
        return "OUTPUT_SCATTER_FILE";
    case ApplicationStatsHelper::OUTPUT_HISTOGRAM_FILE:
        return "OUTPUT_HISTOGRAM_FILE";
    case ApplicationStatsHelper::OUTPUT_PDF_FILE:
        return "OUTPUT_PDF_FILE";
    case ApplicationStatsHelper::OUTPUT_CDF_FILE:
        return "OUTPUT_CDF_FILE";
    case ApplicationStatsHelper::OUTPUT_SCALAR_PLOT:
        return "OUTPUT_SCALAR_PLOT";
    case ApplicationStatsHelper::OUTPUT_SCATTER_PLOT:
        return "OUTPUT_SCATTER_PLOT";
    case ApplicationStatsHelper::OUTPUT_HISTOGRAM_PLOT:
        return "OUTPUT_HISTOGRAM_PLOT";
    case ApplicationStatsHelper::OUTPUT_PDF_PLOT:
        return "OUTPUT_PDF_PLOT";
    case ApplicationStatsHelper::OUTPUT_CDF_PLOT:
        return "OUTPUT_CDF_PLOT";
    default:
        NS_FATAL_ERROR("ApplicationStatsHelper - Invalid output type");
        break;
    }

    NS_FATAL_ERROR("ApplicationStatsHelper - Invalid output type");
    return "";
}

ApplicationStatsHelper::ApplicationStatsHelper()
    : m_name("stat"),
      m_identifierType(ApplicationStatsHelper::IDENTIFIER_GLOBAL),
      m_outputType(ApplicationStatsHelper::OUTPUT_SCATTER_FILE),
      m_traceSourceName(""),
      m_isInstalled(false)
{
    NS_LOG_FUNCTION(this);
}

ApplicationStatsHelper::~ApplicationStatsHelper()
{
    NS_LOG_FUNCTION(this);
}

TypeId // static
ApplicationStatsHelper::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ApplicationStatsHelper")
            .SetParent<Object>()
            .AddAttribute("Name",
                          "String to be prepended on every output file name.",
                          StringValue("stat"),
                          MakeStringAccessor(&ApplicationStatsHelper::SetName,
                                             &ApplicationStatsHelper::GetName),
                          MakeStringChecker())
            .AddAttribute("TraceSourceName",
                          "The name of the application's trace source "
                          "which produce the required data.",
                          StringValue(""),
                          MakeStringAccessor(&ApplicationStatsHelper::SetTraceSourceName,
                                             &ApplicationStatsHelper::GetTraceSourceName),
                          MakeStringChecker())
            .AddAttribute("IdentifierType",
                          "Determines how the statistics are categorized.",
                          EnumValue(ApplicationStatsHelper::IDENTIFIER_GLOBAL),
                          MakeEnumAccessor<ApplicationStatsHelper::IdentifierType_t>(
                              &ApplicationStatsHelper::SetIdentifierType,
                              &ApplicationStatsHelper::GetIdentifierType),
                          MakeEnumChecker(ApplicationStatsHelper::IDENTIFIER_GLOBAL,
                                          "GLOBAL",
                                          ApplicationStatsHelper::IDENTIFIER_SENDER,
                                          "SENDER",
                                          ApplicationStatsHelper::IDENTIFIER_RECEIVER,
                                          "RECEIVER"))
            .AddAttribute("OutputType",
                          "Determines the type and format of the output.",
                          EnumValue(ApplicationStatsHelper::OUTPUT_SCATTER_FILE),
                          MakeEnumAccessor<ApplicationStatsHelper::OutputType_t>(
                              &ApplicationStatsHelper::SetOutputType,
                              &ApplicationStatsHelper::GetOutputType),
                          MakeEnumChecker(ApplicationStatsHelper::OUTPUT_NONE,
                                          "NONE",
                                          ApplicationStatsHelper::OUTPUT_SCALAR_FILE,
                                          "SCALAR_FILE",
                                          ApplicationStatsHelper::OUTPUT_SCATTER_FILE,
                                          "SCATTER_FILE",
                                          ApplicationStatsHelper::OUTPUT_HISTOGRAM_FILE,
                                          "HISTOGRAM_FILE",
                                          ApplicationStatsHelper::OUTPUT_PDF_FILE,
                                          "PDF_FILE",
                                          ApplicationStatsHelper::OUTPUT_CDF_FILE,
                                          "CDF_FILE",
                                          ApplicationStatsHelper::OUTPUT_SCATTER_PLOT,
                                          "SCATTER_PLOT",
                                          ApplicationStatsHelper::OUTPUT_HISTOGRAM_PLOT,
                                          "HISTOGRAM_PLOT",
                                          ApplicationStatsHelper::OUTPUT_PDF_PLOT,
                                          "PDF_PLOT",
                                          ApplicationStatsHelper::OUTPUT_CDF_PLOT,
                                          "CDF_PLOT"));
    return tid;
}

void
ApplicationStatsHelper::SetSenderInformation(std::map<std::string, ApplicationContainer> info)
{
    NS_LOG_FUNCTION(this << info.size());
    m_senderInfo = info;
}

void
ApplicationStatsHelper::SetReceiverInformation(std::map<std::string, ApplicationContainer> info)
{
    NS_LOG_FUNCTION(this << info.size());
    m_receiverInfo = info;
}

void
ApplicationStatsHelper::Install()
{
    NS_LOG_FUNCTION(this);

    if (m_traceSourceName.empty())
    {
        NS_FATAL_ERROR("Trace source name must not be blank.");
    }
    else if (m_outputType == ApplicationStatsHelper::OUTPUT_NONE)
    {
        NS_LOG_WARN(this << " Skipping statistics installation"
                         << " because OUTPUT_NONE output type is selected.");
    }
    else
    {
        DoInstall(); // this method is supposed to be implemented by the child class
        m_isInstalled = true;
    }
}

void
ApplicationStatsHelper::SetName(std::string name)
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
ApplicationStatsHelper::GetName() const
{
    return m_name;
}

void
ApplicationStatsHelper::SetTraceSourceName(std::string traceSourceName)
{
    NS_LOG_FUNCTION(this << traceSourceName);
    m_traceSourceName = traceSourceName;
}

std::string
ApplicationStatsHelper::GetTraceSourceName() const
{
    return m_traceSourceName;
}

void
ApplicationStatsHelper::SetIdentifierType(ApplicationStatsHelper::IdentifierType_t identifierType)
{
    NS_LOG_FUNCTION(this << GetIdentifierTypeName(identifierType));

    if (m_isInstalled && (m_identifierType != identifierType))
    {
        NS_LOG_WARN(this << " cannot modify the current identifier type" << " ("
                         << GetIdentifierTypeName(m_identifierType) << ")"
                         << " because this instance have already been installed");
    }
    else
    {
        m_identifierType = identifierType;
    }
}

ApplicationStatsHelper::IdentifierType_t
ApplicationStatsHelper::GetIdentifierType() const
{
    return m_identifierType;
}

void
ApplicationStatsHelper::SetOutputType(ApplicationStatsHelper::OutputType_t outputType)
{
    NS_LOG_FUNCTION(this << GetOutputTypeName(outputType));

    if (m_isInstalled && (m_outputType != outputType))
    {
        NS_LOG_WARN(this << " cannot modify the current output type" << " ("
                         << GetIdentifierTypeName(m_identifierType) << ")"
                         << " because this instance have already been installed");
    }
    else
    {
        m_outputType = outputType;
    }
}

ApplicationStatsHelper::OutputType_t
ApplicationStatsHelper::GetOutputType() const
{
    return m_outputType;
}

bool
ApplicationStatsHelper::IsInstalled() const
{
    return m_isInstalled;
}

Ptr<DataCollectionObject>
ApplicationStatsHelper::CreateAggregator(std::string aggregatorTypeId,
                                         std::string n1,
                                         const AttributeValue& v1,
                                         std::string n2,
                                         const AttributeValue& v2,
                                         std::string n3,
                                         const AttributeValue& v3,
                                         std::string n4,
                                         const AttributeValue& v4,
                                         std::string n5,
                                         const AttributeValue& v5)
{
    NS_LOG_FUNCTION(this << aggregatorTypeId);

    TypeId tid = TypeId::LookupByName(aggregatorTypeId);
    ObjectFactory factory;
    factory.SetTypeId(tid);
    factory.Set(n1, v1);
    factory.Set(n2, v2);
    factory.Set(n3, v3);
    factory.Set(n4, v4);
    factory.Set(n5, v5);
    return factory.Create()->GetObject<DataCollectionObject>();
}

uint32_t
ApplicationStatsHelper::CreateCollectorPerIdentifier(CollectorMap& collectorMap) const
{
    NS_LOG_FUNCTION(this);
    uint32_t n = 0;

    switch (GetIdentifierType())
    {
    case ApplicationStatsHelper::IDENTIFIER_GLOBAL: {
        collectorMap.SetAttribute("Name", StringValue("global"));
        collectorMap.Create(0);
        n++;
        break;
    }

    case ApplicationStatsHelper::IDENTIFIER_RECEIVER: {
        std::map<std::string, ApplicationContainer>::const_iterator it;
        for (it = m_receiverInfo.begin(); it != m_receiverInfo.end(); ++it)
        {
            collectorMap.SetAttribute("Name", StringValue(it->first));
            collectorMap.Create(n);
            n++;
        }
        break;
    }

    case ApplicationStatsHelper::IDENTIFIER_SENDER: {
        std::map<std::string, ApplicationContainer>::const_iterator it;
        for (it = m_senderInfo.begin(); it != m_senderInfo.end(); ++it)
        {
            collectorMap.SetAttribute("Name", StringValue(it->first));
            collectorMap.Create(n);
            n++;
        }
        break;
    }

    default:
        NS_FATAL_ERROR("ApplicationStatsHelper - Invalid identifier type");
        break;
    }

    NS_LOG_INFO(this << " created " << n << " instance(s)" << " of "
                     << collectorMap.GetType().GetName() << " for "
                     << GetIdentifierTypeName(GetIdentifierType()));

    return n;

} // end of `uint32_t CreateCollectorPerIdentifier (CollectorMap &);`

} // end of namespace ns3
