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

#include "application-stats-delay-helper.h"

#include "ns3/application-container.h"
#include "ns3/application-delay-probe.h"
#include "ns3/boolean.h"
#include "ns3/data-collection-object.h"
#include "ns3/distribution-collector.h"
#include "ns3/enum.h"
#include "ns3/gnuplot-aggregator.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4.h"
#include "ns3/log.h"
#include "ns3/multi-file-aggregator.h"
#include "ns3/node.h"
#include "ns3/nstime.h"
#include "ns3/probe.h"
#include "ns3/scalar-collector.h"
#include "ns3/string.h"
#include "ns3/unit-conversion-collector.h"

#include <map>
#include <sstream>
#include <string>

NS_LOG_COMPONENT_DEFINE("ApplicationStatsDelayHelper");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(ApplicationStatsDelayHelper);

ApplicationStatsDelayHelper::ApplicationStatsDelayHelper()
{
    NS_LOG_FUNCTION(this);
}

ApplicationStatsDelayHelper::~ApplicationStatsDelayHelper()
{
    NS_LOG_FUNCTION(this);
}

TypeId // static
ApplicationStatsDelayHelper::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ApplicationStatsDelayHelper").SetParent<ApplicationStatsHelper>();
    return tid;
}

void
ApplicationStatsDelayHelper::DoInstall()
{
    NS_LOG_FUNCTION(this);

    // Setup aggregators and collectors.

    switch (GetOutputType())
    {
    case ApplicationStatsHelper::OUTPUT_NONE:
        NS_FATAL_ERROR(GetOutputTypeName(GetOutputType())
                       << " is not a valid output type for this statistics.");
        break;

    case ApplicationStatsHelper::OUTPUT_SCALAR_FILE: {
        // Setup aggregator.
        m_aggregator = CreateAggregator("ns3::MultiFileAggregator",
                                        "OutputFileName",
                                        StringValue(GetName()),
                                        "MultiFileMode",
                                        BooleanValue(false),
                                        "EnableContextPrinting",
                                        BooleanValue(true),
                                        "GeneralHeading",
                                        StringValue("% identifier delay_sec"));

        // Setup collectors.
        m_terminalCollectors.SetType("ns3::ScalarCollector");
        m_terminalCollectors.SetAttribute("InputDataType",
                                          EnumValue(ScalarCollector::INPUT_DATA_TYPE_DOUBLE));
        m_terminalCollectors.SetAttribute(
            "OutputType",
            EnumValue(ScalarCollector::OUTPUT_TYPE_AVERAGE_PER_SAMPLE));
        CreateCollectorPerIdentifier(m_terminalCollectors);
        m_terminalCollectors.ConnectToAggregator("Output",
                                                 m_aggregator,
                                                 &MultiFileAggregator::Write1d);
        break;
    }

    case ApplicationStatsHelper::OUTPUT_SCATTER_FILE: {
        // Setup aggregator.
        m_aggregator = CreateAggregator("ns3::MultiFileAggregator",
                                        "OutputFileName",
                                        StringValue(GetName()),
                                        "GeneralHeading",
                                        StringValue("% time_sec delay_sec"));

        // Setup collectors.
        m_terminalCollectors.SetType("ns3::UnitConversionCollector");
        m_terminalCollectors.SetAttribute("ConversionType",
                                          EnumValue(UnitConversionCollector::TRANSPARENT));
        CreateCollectorPerIdentifier(m_terminalCollectors);
        m_terminalCollectors.ConnectToAggregator("OutputTimeValue",
                                                 m_aggregator,
                                                 &MultiFileAggregator::Write2d);
        break;
    }

    case ApplicationStatsHelper::OUTPUT_HISTOGRAM_FILE:
    case ApplicationStatsHelper::OUTPUT_PDF_FILE:
    case ApplicationStatsHelper::OUTPUT_CDF_FILE: {
        // Setup aggregator.
        m_aggregator = CreateAggregator("ns3::MultiFileAggregator",
                                        "OutputFileName",
                                        StringValue(GetName()),
                                        "GeneralHeading",
                                        StringValue("% delay_sec freq"));

        // Setup collectors.
        m_terminalCollectors.SetType("ns3::DistributionCollector");
        DistributionCollector::OutputType_t outputType =
            DistributionCollector::OUTPUT_TYPE_HISTOGRAM;
        if (GetOutputType() == ApplicationStatsHelper::OUTPUT_PDF_FILE)
        {
            outputType = DistributionCollector::OUTPUT_TYPE_PROBABILITY;
        }
        else if (GetOutputType() == ApplicationStatsHelper::OUTPUT_CDF_FILE)
        {
            outputType = DistributionCollector::OUTPUT_TYPE_CUMULATIVE;
        }
        m_terminalCollectors.SetAttribute("OutputType", EnumValue(outputType));
        CreateCollectorPerIdentifier(m_terminalCollectors);
        m_terminalCollectors.ConnectToAggregator("Output",
                                                 m_aggregator,
                                                 &MultiFileAggregator::Write2d);
        m_terminalCollectors.ConnectToAggregator("OutputString",
                                                 m_aggregator,
                                                 &MultiFileAggregator::AddContextHeading);
        m_terminalCollectors.ConnectToAggregator("Warning",
                                                 m_aggregator,
                                                 &MultiFileAggregator::EnableContextWarning);
        break;
    }

    case ApplicationStatsHelper::OUTPUT_SCALAR_PLOT:
        /// @todo Add support for boxes in Gnuplot.
        NS_FATAL_ERROR(GetOutputTypeName(GetOutputType())
                       << " is not a valid output type for this statistics.");
        break;

    case ApplicationStatsHelper::OUTPUT_SCATTER_PLOT: {
        // Setup aggregator.
        Ptr<GnuplotAggregator> plotAggregator = CreateObject<GnuplotAggregator>(GetName());
        // plot->SetTitle ("");
        plotAggregator->SetLegend("Time (in seconds)", "Packet delay (in seconds)");
        plotAggregator->Set2dDatasetDefaultStyle(Gnuplot2dDataset::LINES);
        m_aggregator = plotAggregator;

        // Setup collectors.
        m_terminalCollectors.SetType("ns3::UnitConversionCollector");
        m_terminalCollectors.SetAttribute("ConversionType",
                                          EnumValue(UnitConversionCollector::TRANSPARENT));
        CreateCollectorPerIdentifier(m_terminalCollectors);
        for (CollectorMap::Iterator it = m_terminalCollectors.Begin();
             it != m_terminalCollectors.End();
             ++it)
        {
            const std::string context = it->second->GetName();
            plotAggregator->Add2dDataset(context, context);
        }
        m_terminalCollectors.ConnectToAggregator("OutputTimeValue",
                                                 m_aggregator,
                                                 &GnuplotAggregator::Write2d);
        break;
    }

    case ApplicationStatsHelper::OUTPUT_HISTOGRAM_PLOT:
    case ApplicationStatsHelper::OUTPUT_PDF_PLOT:
    case ApplicationStatsHelper::OUTPUT_CDF_PLOT: {
        // Setup aggregator.
        Ptr<GnuplotAggregator> plotAggregator = CreateObject<GnuplotAggregator>(GetName());
        // plot->SetTitle ("");
        plotAggregator->SetLegend("Packet delay (in seconds)", "Frequency");
        plotAggregator->Set2dDatasetDefaultStyle(Gnuplot2dDataset::LINES);
        m_aggregator = plotAggregator;

        // Setup collectors.
        m_terminalCollectors.SetType("ns3::DistributionCollector");
        DistributionCollector::OutputType_t outputType =
            DistributionCollector::OUTPUT_TYPE_HISTOGRAM;
        if (GetOutputType() == ApplicationStatsHelper::OUTPUT_PDF_PLOT)
        {
            outputType = DistributionCollector::OUTPUT_TYPE_PROBABILITY;
        }
        else if (GetOutputType() == ApplicationStatsHelper::OUTPUT_CDF_PLOT)
        {
            outputType = DistributionCollector::OUTPUT_TYPE_CUMULATIVE;
        }
        m_terminalCollectors.SetAttribute("OutputType", EnumValue(outputType));
        CreateCollectorPerIdentifier(m_terminalCollectors);
        for (CollectorMap::Iterator it = m_terminalCollectors.Begin();
             it != m_terminalCollectors.End();
             ++it)
        {
            const std::string context = it->second->GetName();
            plotAggregator->Add2dDataset(context, context);
        }
        m_terminalCollectors.ConnectToAggregator("Output",
                                                 m_aggregator,
                                                 &GnuplotAggregator::Write2d);
        break;
    }

    default:
        NS_FATAL_ERROR("ApplicationStatsDelayHelper - Invalid output type");
        break;

    } // end of `switch (GetOutputType ())`

    // Setup probes and connect them to the collectors.

    switch (GetIdentifierType())
    {
    case ApplicationStatsHelper::IDENTIFIER_GLOBAL:
    case ApplicationStatsHelper::IDENTIFIER_RECEIVER: {
        /*
         * Install a probe on each receiver and connect them to the
         * first-level collectors.
         */
        uint32_t n = 0;
        switch (GetOutputType())
        {
        case ApplicationStatsHelper::OUTPUT_SCALAR_FILE:
        case ApplicationStatsHelper::OUTPUT_SCALAR_PLOT:
            n = SetupProbesAtReceiver<ApplicationDelayProbe>("OutputSeconds",
                                                             m_terminalCollectors,
                                                             &ScalarCollector::TraceSinkDouble,
                                                             m_probes);
            break;

        case ApplicationStatsHelper::OUTPUT_SCATTER_FILE:
        case ApplicationStatsHelper::OUTPUT_SCATTER_PLOT:
            n = SetupProbesAtReceiver<ApplicationDelayProbe>(
                "OutputSeconds",
                m_terminalCollectors,
                &UnitConversionCollector::TraceSinkDouble,
                m_probes);
            break;

        case ApplicationStatsHelper::OUTPUT_HISTOGRAM_FILE:
        case ApplicationStatsHelper::OUTPUT_HISTOGRAM_PLOT:
        case ApplicationStatsHelper::OUTPUT_PDF_FILE:
        case ApplicationStatsHelper::OUTPUT_PDF_PLOT:
        case ApplicationStatsHelper::OUTPUT_CDF_FILE:
        case ApplicationStatsHelper::OUTPUT_CDF_PLOT:
            n = SetupProbesAtReceiver<ApplicationDelayProbe>(
                "OutputSeconds",
                m_terminalCollectors,
                &DistributionCollector::TraceSinkDouble,
                m_probes);
            break;

        default:
            NS_FATAL_ERROR(GetOutputTypeName(GetOutputType())
                           << " is not a valid output type for this statistics.");
            break;
        }

        NS_LOG_INFO(this << " created " << n << " instance(s)" << " of ApplicationDelayProbe");
        break;
    }

    case ApplicationStatsHelper::IDENTIFIER_SENDER: {
        // Create a look-up table of sender addresses and collector identifiers.
        uint32_t identifier = 0;
        std::map<std::string, ApplicationContainer>::const_iterator it1;
        for (it1 = m_senderInfo.begin(); it1 != m_senderInfo.end(); ++it1)
        {
            for (ApplicationContainer::Iterator it2 = it1->second.Begin(); it2 != it1->second.End();
                 ++it2)
            {
                SaveAddressAndIdentifier(*it2, identifier);
            }

            identifier++;
        }

        // Connect with trace sources in receiver applications.
        const uint32_t n = SetupListenersAtReceiver(
            MakeCallback(&ApplicationStatsDelayHelper::RxDelayCallback, this));
        NS_LOG_INFO(this << " connected to " << n << " trace sources");
        break;
    }

    default:
        NS_FATAL_ERROR("ApplicationStatsDelayHelper - Invalid identifier type");
        break;

    } // end of `switch (GetIdentifierType ())`

} // end of `void DoInstall ();`

void
ApplicationStatsDelayHelper::RxDelayCallback(Time delay, const Address& from)
{
    // NS_LOG_FUNCTION (this << delay.GetSeconds () << from);

    if (InetSocketAddress::IsMatchingType(from))
    {
        // Determine the identifier associated with the sender address.
        const Address ipv4Addr = InetSocketAddress::ConvertFrom(from).GetIpv4();
        std::map<const Address, uint32_t>::const_iterator it1 = m_identifierMap.find(ipv4Addr);

        if (it1 == m_identifierMap.end())
        {
            NS_LOG_WARN(this << " discarding a packet delay of " << delay.GetSeconds()
                             << " from statistics collection because of"
                             << " unknown sender IPv4 address " << ipv4Addr);
        }
        else
        {
            PassSampleToCollector(delay, it1->second);
        }
    }
    else
    {
        NS_LOG_WARN(this << " discarding a packet delay of " << delay.GetSeconds()
                         << " from statistics collection" << " because it comes from sender "
                         << from << " without valid InetSocketAddress");
    }

} // end of `void RxDelayCallback (Time, const Address &)`

void
ApplicationStatsDelayHelper::SaveAddressAndIdentifier(Ptr<Application> application,
                                                      uint32_t identifier)
{
    NS_LOG_FUNCTION(this << application << identifier);

    Ptr<Node> node = application->GetNode();
    NS_ASSERT_MSG(node != nullptr, "Application is not attached to any Node");
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();

    if (ipv4 == nullptr)
    {
        NS_LOG_INFO(this << " Node " << node->GetId() << " does not support IPv4 protocol");
    }
    else
    {
        NS_LOG_DEBUG(this << " found " << ipv4->GetNInterfaces() << " interface(s)" << " in Node "
                          << node->GetId());

        // Skipping interface #0 because it is assumed to be a loopback interface.
        for (uint32_t i = 1; i < ipv4->GetNInterfaces(); i++)
        {
            NS_LOG_DEBUG(this << " found " << ipv4->GetNAddresses(i) << " address(es)"
                              << " in Node " << node->GetId() << " interface #" << i);

            for (uint32_t j = 0; j < ipv4->GetNAddresses(i); j++)
            {
                const Address addr = ipv4->GetAddress(i, j).GetLocal();
                m_identifierMap[addr] = identifier;
                NS_LOG_INFO(this << " associated address " << addr << " with identifier "
                                 << identifier);
            }
        }
    }

} // end of `void SaveAddressAndIdentifier (Ptr<Application>, uint32_t)`

void
ApplicationStatsDelayHelper::PassSampleToCollector(Time delay, uint32_t identifier)
{
    // NS_LOG_FUNCTION (this << delay.GetSeconds () << identifier);

    Ptr<DataCollectionObject> collector = m_terminalCollectors.Get(identifier);
    NS_ASSERT_MSG(collector != nullptr, "Unable to find collector with identifier " << identifier);

    switch (GetOutputType())
    {
    case ApplicationStatsHelper::OUTPUT_SCALAR_FILE:
    case ApplicationStatsHelper::OUTPUT_SCALAR_PLOT: {
        Ptr<ScalarCollector> c = collector->GetObject<ScalarCollector>();
        NS_ASSERT(c != nullptr);
        c->TraceSinkDouble(0.0, delay.GetSeconds());
        break;
    }

    case ApplicationStatsHelper::OUTPUT_SCATTER_FILE:
    case ApplicationStatsHelper::OUTPUT_SCATTER_PLOT: {
        Ptr<UnitConversionCollector> c = collector->GetObject<UnitConversionCollector>();
        NS_ASSERT(c != nullptr);
        c->TraceSinkDouble(0.0, delay.GetSeconds());
        break;
    }

    case ApplicationStatsHelper::OUTPUT_HISTOGRAM_FILE:
    case ApplicationStatsHelper::OUTPUT_HISTOGRAM_PLOT:
    case ApplicationStatsHelper::OUTPUT_PDF_FILE:
    case ApplicationStatsHelper::OUTPUT_PDF_PLOT:
    case ApplicationStatsHelper::OUTPUT_CDF_FILE:
    case ApplicationStatsHelper::OUTPUT_CDF_PLOT: {
        Ptr<DistributionCollector> c = collector->GetObject<DistributionCollector>();
        NS_ASSERT(c != nullptr);
        c->TraceSinkDouble(0.0, delay.GetSeconds());
        break;
    }

    default:
        NS_FATAL_ERROR(GetOutputTypeName(GetOutputType())
                       << " is not a valid output type for this statistics.");
        break;

    } // end of `switch (GetOutputType ())`

} // end of `void PassSampleToCollector (Time, uint32_t)`

} // end of namespace ns3
