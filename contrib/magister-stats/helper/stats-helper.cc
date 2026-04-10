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

#include "stats-helper.h"

#include "ns3/address.h"
#include "ns3/collector-map.h"
#include "ns3/data-collection-object.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/mac48-address.h"
#include "ns3/node-container.h"
#include "ns3/object-factory.h"
#include "ns3/singleton.h"
#include "ns3/string.h"
#include "ns3/type-id.h"

#include <sstream>
#include <stdio.h>
#include <string>
#include <sys/stat.h>

NS_LOG_COMPONENT_DEFINE("StatsHelper");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(StatsHelper);

std::string // static
StatsHelper::GetIdentifierTypeName(StatsHelper::IdentifierType_t identifierType)
{
    switch (identifierType)
    {
    case StatsHelper::IDENTIFIER_GLOBAL:
        return "IDENTIFIER_GLOBAL";
    case StatsHelper::IDENTIFIER_NODE:
        return "IDENTIFIER_NODE";
    default:
        NS_FATAL_ERROR("StatsHelper - Invalid identifier type");
        break;
    }

    NS_FATAL_ERROR("StatsHelper - Invalid identifier type");
    return "";
}

std::string // static
StatsHelper::GetOutputTypeName(StatsHelper::OutputType_t outputType)
{
    switch (outputType)
    {
    case StatsHelper::OUTPUT_NONE:
        return "OUTPUT_NONE";
    case StatsHelper::OUTPUT_SCALAR_FILE:
        return "OUTPUT_SCALAR_FILE";
    case StatsHelper::OUTPUT_SCATTER_FILE:
        return "OUTPUT_SCATTER_FILE";
    case StatsHelper::OUTPUT_HISTOGRAM_FILE:
        return "OUTPUT_HISTOGRAM_FILE";
    case StatsHelper::OUTPUT_PDF_FILE:
        return "OUTPUT_PDF_FILE";
    case StatsHelper::OUTPUT_CDF_FILE:
        return "OUTPUT_CDF_FILE";
    case StatsHelper::OUTPUT_SCALAR_PLOT:
        return "OUTPUT_SCALAR_PLOT";
    case StatsHelper::OUTPUT_SCATTER_PLOT:
        return "OUTPUT_SCATTER_PLOT";
    case StatsHelper::OUTPUT_HISTOGRAM_PLOT:
        return "OUTPUT_HISTOGRAM_PLOT";
    case StatsHelper::OUTPUT_PDF_PLOT:
        return "OUTPUT_PDF_PLOT";
    case StatsHelper::OUTPUT_CDF_PLOT:
        return "OUTPUT_CDF_PLOT";
    default:
        NS_FATAL_ERROR("StatsHelper - Invalid output type");
        break;
    }

    NS_FATAL_ERROR("StatsHelper - Invalid output type");
    return "";
}

StatsHelper::StatsHelper()
    : m_name("stat"),
      m_outputPath("output"),
      m_identifierType(StatsHelper::IDENTIFIER_GLOBAL),
      m_outputType(StatsHelper::OUTPUT_SCATTER_FILE),
      m_isInstalled(false),
      m_nodes(NodeContainer())
{
    NS_LOG_FUNCTION(this);
}

StatsHelper::~StatsHelper()
{
    NS_LOG_FUNCTION(this);
}

TypeId // static
StatsHelper::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::StatsHelper")
            .SetParent<Object>()
            .AddAttribute("Name",
                          "String to be prepended on every output file name.",
                          StringValue("stat"),
                          MakeStringAccessor(&StatsHelper::SetName, &StatsHelper::GetName),
                          MakeStringChecker())
            .AddAttribute(
                "OutputPath",
                "Default output path to the output files, *without* "
                "the last slash (/) separator. If the given "
                "path does not exist, it will be created.",
                StringValue("output"),
                MakeStringAccessor(&StatsHelper::SetOutputPath, &StatsHelper::GetOutputPath),
                MakeStringChecker())
            .AddAttribute(
                "IdentifierType",
                "Determines how the statistics are categorized.",
                EnumValue(StatsHelper::IDENTIFIER_GLOBAL),
                MakeEnumAccessor<StatsHelper::IdentifierType_t>(&StatsHelper::SetIdentifierType,
                                                                &StatsHelper::GetIdentifierType),
                MakeEnumChecker(StatsHelper::IDENTIFIER_GLOBAL,
                                "GLOBAL",
                                StatsHelper::IDENTIFIER_NODE,
                                "NODE"))
            .AddAttribute("OutputType",
                          "Determines the type and format of the output.",
                          EnumValue(StatsHelper::OUTPUT_SCATTER_FILE),
                          MakeEnumAccessor<StatsHelper::OutputType_t>(&StatsHelper::SetOutputType,
                                                                      &StatsHelper::GetOutputType),
                          MakeEnumChecker(StatsHelper::OUTPUT_NONE,
                                          "NONE",
                                          StatsHelper::OUTPUT_SCALAR_FILE,
                                          "SCALAR_FILE",
                                          StatsHelper::OUTPUT_SCATTER_FILE,
                                          "SCATTER_FILE",
                                          StatsHelper::OUTPUT_HISTOGRAM_FILE,
                                          "HISTOGRAM_FILE",
                                          StatsHelper::OUTPUT_PDF_FILE,
                                          "PDF_FILE",
                                          StatsHelper::OUTPUT_CDF_FILE,
                                          "CDF_FILE",
                                          StatsHelper::OUTPUT_SCATTER_PLOT,
                                          "SCATTER_PLOT",
                                          StatsHelper::OUTPUT_HISTOGRAM_PLOT,
                                          "HISTOGRAM_PLOT",
                                          StatsHelper::OUTPUT_PDF_PLOT,
                                          "PDF_PLOT",
                                          StatsHelper::OUTPUT_CDF_PLOT,
                                          "CDF_PLOT"));
    return tid;
}

void
StatsHelper::Install()
{
    NS_LOG_FUNCTION(this);

    if (m_outputType == StatsHelper::OUTPUT_NONE)
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
StatsHelper::InstallNodes(NodeContainer nodes)
{
    NS_LOG_FUNCTION(this);
    m_nodes.Add(nodes);
}

void
StatsHelper::SetName(std::string name)
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
StatsHelper::GetName() const
{
    return m_name;
}

void
StatsHelper::SetIdentifierType(StatsHelper::IdentifierType_t identifierType)
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

StatsHelper::IdentifierType_t
StatsHelper::GetIdentifierType() const
{
    return m_identifierType;
}

void
StatsHelper::SetOutputType(StatsHelper::OutputType_t outputType)
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

StatsHelper::OutputType_t
StatsHelper::GetOutputType() const
{
    return m_outputType;
}

bool
StatsHelper::IsInstalled() const
{
    return m_isInstalled;
}

Ptr<DataCollectionObject>
StatsHelper::CreateAggregator(std::string aggregatorTypeId,
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
StatsHelper::CreateCollectorPerIdentifier(CollectorMap& collectorMap) const
{
    NS_LOG_FUNCTION(this);
    uint32_t n = 0;

    switch (GetIdentifierType())
    {
    case StatsHelper::IDENTIFIER_GLOBAL: {
        collectorMap.SetAttribute("Name", StringValue("0"));
        collectorMap.Create(0);
        n++;
        break;
    }

    case StatsHelper::IDENTIFIER_NODE: {
        NodeContainer allNodes = m_nodes;
        for (NodeContainer::Iterator it = allNodes.Begin(); it != allNodes.End(); ++it)
        {
            const uint32_t nodeId = (*it)->GetId();
            std::ostringstream name;
            name << nodeId;
            collectorMap.SetAttribute("Name", StringValue(name.str()));
            collectorMap.Create(nodeId);
            n++;
        }
        break;
    }

    default:
        NS_FATAL_ERROR("StatsHelper - Invalid identifier type");
        break;
    }

    NS_LOG_INFO(this << " created " << n << " instance(s)" << " of "
                     << collectorMap.GetType().GetName() << " for "
                     << GetIdentifierTypeName(GetIdentifierType()));

    return n;

} // end of `uint32_t CreateCollectorPerIdentifier (CollectorMap &);`

std::string
StatsHelper::GetOutputPath() const
{
    return m_outputPath;
}

bool // static
StatsHelper::IsValidDirectory(std::string path)
{
    struct stat st;
    bool validDirectory = false;

    if (stat(path.c_str(), &st) == 0)
    {
        if (st.st_mode && S_IFDIR != 0)
        {
            validDirectory = true;
        }
    }

    NS_LOG_INFO("StatsHelper::IsValidDirectory - " << path << " validity: " << validDirectory);

    return validDirectory;
}

void
StatsHelper::SetOutputPath(std::string outputPath)
{
    if (!IsValidDirectory(outputPath))
    {
        mode_t nMode = 0777; // UNIX permissions
        int nError = 0;
#if defined(_WIN32)
        nError = _mkdir(outputPath.c_str()); // Windows
#else
        nError = mkdir(outputPath.c_str(), nMode); // non-Windows
#endif
        if (nError != 0)
        {
            NS_FATAL_ERROR("Directory " << outputPath << " could not be created.");
        }
    }
    m_outputPath = outputPath;
}

std::string
StatsHelper::GetOutputFileName() const
{
    return GetOutputPath() + "/" + GetName();
}

std::string
StatsHelper::GetIdentifierHeading(std::string dataLabel) const
{
    switch (GetIdentifierType())
    {
    case StatsHelper::IDENTIFIER_GLOBAL:
        return "% global " + dataLabel;

    case StatsHelper::IDENTIFIER_NODE:
        return "% node_id " + dataLabel;

    default:
        NS_FATAL_ERROR("StatsHelper - Invalid identifier type");
        break;
    }
    return "";
}

std::string
StatsHelper::GetTimeHeading(std::string dataLabel) const
{
    return "% time_sec " + dataLabel;
}

std::string
StatsHelper::GetDistributionHeading(std::string dataLabel) const
{
    return "% " + dataLabel + " freq";
}

} // end of namespace ns3
