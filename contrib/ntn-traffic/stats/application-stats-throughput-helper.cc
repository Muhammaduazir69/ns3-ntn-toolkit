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

#include "application-stats-throughput-helper.h"

#include "ns3/application-container.h"
#include "ns3/application-packet-probe.h"
#include "ns3/boolean.h"
#include "ns3/data-collection-object.h"
#include "ns3/distribution-collector.h"
#include "ns3/enum.h"
#include "ns3/gnuplot-aggregator.h"
#include "ns3/inet-socket-address.h"
#include "ns3/interval-rate-collector.h"
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

NS_LOG_COMPONENT_DEFINE("ApplicationStatsThroughputHelper");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(ApplicationStatsThroughputHelper);

ApplicationStatsThroughputHelper::ApplicationStatsThroughputHelper()
    : m_averagingMode(false)

{
    NS_LOG_FUNCTION(this);
}

ApplicationStatsThroughputHelper::~ApplicationStatsThroughputHelper()
{
    NS_LOG_FUNCTION(this);
}

TypeId // static
ApplicationStatsThroughputHelper::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ApplicationStatsThroughputHelper")
            .SetParent<ApplicationStatsHelper>()
            .AddAttribute("AveragingMode",
                          "If true, all samples will be averaged before passed to aggregator. "
                          "Only affects histogram, PDF, and CDF output types.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&ApplicationStatsThroughputHelper::SetAveragingMode,
                                              &ApplicationStatsThroughputHelper::GetAveragingMode),
                          MakeBooleanChecker());
    return tid;
}

void
ApplicationStatsThroughputHelper::SetAveragingMode(bool averagingMode)
{
    NS_LOG_FUNCTION(this << averagingMode);
    m_averagingMode = averagingMode;
}

bool
ApplicationStatsThroughputHelper::GetAveragingMode() const
{
    return m_averagingMode;
}

void
ApplicationStatsThroughputHelper::DoInstall()
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
                                        StringValue("% identifier throughput_kbps"));

        // Setup second-level collectors.
        m_terminalCollectors.SetType("ns3::ScalarCollector");
        m_terminalCollectors.SetAttribute("InputDataType",
                                          EnumValue(ScalarCollector::INPUT_DATA_TYPE_DOUBLE));
        m_terminalCollectors.SetAttribute(
            "OutputType",
            EnumValue(ScalarCollector::OUTPUT_TYPE_AVERAGE_PER_SECOND));
        CreateCollectorPerIdentifier(m_terminalCollectors);
        m_terminalCollectors.ConnectToAggregator("Output",
                                                 m_aggregator,
                                                 &MultiFileAggregator::Write1d);

        // Setup first-level collectors.
        m_conversionCollectors.SetType("ns3::UnitConversionCollector");
        m_conversionCollectors.SetAttribute("ConversionType",
                                            EnumValue(UnitConversionCollector::FROM_BYTES_TO_KBIT));
        CreateCollectorPerIdentifier(m_conversionCollectors);
        m_conversionCollectors.ConnectToCollector("Output",
                                                  m_terminalCollectors,
                                                  &ScalarCollector::TraceSinkDouble);
        break;
    }

    case ApplicationStatsHelper::OUTPUT_SCATTER_FILE: {
        // Setup aggregator.
        m_aggregator = CreateAggregator("ns3::MultiFileAggregator",
                                        "OutputFileName",
                                        StringValue(GetName()),
                                        "GeneralHeading",
                                        StringValue("% time_sec throughput_kbps"));

        // Setup second-level collectors.
        m_terminalCollectors.SetType("ns3::IntervalRateCollector");
        m_terminalCollectors.SetAttribute("InputDataType",
                                          EnumValue(IntervalRateCollector::INPUT_DATA_TYPE_DOUBLE));
        CreateCollectorPerIdentifier(m_terminalCollectors);
        m_terminalCollectors.ConnectToAggregator("OutputWithTime",
                                                 m_aggregator,
                                                 &MultiFileAggregator::Write2d);
        m_terminalCollectors.ConnectToAggregator("OutputString",
                                                 m_aggregator,
                                                 &MultiFileAggregator::AddContextHeading);

        // Setup first-level collectors.
        m_conversionCollectors.SetType("ns3::UnitConversionCollector");
        m_conversionCollectors.SetAttribute("ConversionType",
                                            EnumValue(UnitConversionCollector::FROM_BYTES_TO_KBIT));
        CreateCollectorPerIdentifier(m_conversionCollectors);
        m_conversionCollectors.ConnectToCollector("Output",
                                                  m_terminalCollectors,
                                                  &IntervalRateCollector::TraceSinkDouble);
        break;
    }

    case ApplicationStatsHelper::OUTPUT_HISTOGRAM_FILE:
    case ApplicationStatsHelper::OUTPUT_PDF_FILE:
    case ApplicationStatsHelper::OUTPUT_CDF_FILE: {
        if (!m_averagingMode)
        {
            NS_FATAL_ERROR("This statistics require AveragingMode to be enabled");
        }

        // Setup aggregator.
        m_aggregator = CreateAggregator("ns3::MultiFileAggregator",
                                        "OutputFileName",
                                        StringValue(GetName()),
                                        "MultiFileMode",
                                        BooleanValue(false),
                                        "EnableContextPrinting",
                                        BooleanValue(false),
                                        "GeneralHeading",
                                        StringValue("% throughput_kbps freq"));
        Ptr<MultiFileAggregator> fileAggregator = m_aggregator->GetObject<MultiFileAggregator>();
        NS_ASSERT(fileAggregator != nullptr);

        // Setup the final-level collector.
        m_averagingCollector = CreateObject<DistributionCollector>();
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
        m_averagingCollector->SetOutputType(outputType);
        m_averagingCollector->SetName("0");
        m_averagingCollector->TraceConnect(
            "Output",
            "0",
            MakeCallback(&MultiFileAggregator::Write2d, fileAggregator));
        m_averagingCollector->TraceConnect(
            "OutputString",
            "0",
            MakeCallback(&MultiFileAggregator::AddContextHeading, fileAggregator));
        m_averagingCollector->TraceConnect(
            "Warning",
            "0",
            MakeCallback(&MultiFileAggregator::EnableContextWarning, fileAggregator));

        // Setup second-level collectors.
        m_terminalCollectors.SetType("ns3::ScalarCollector");
        m_terminalCollectors.SetAttribute("InputDataType",
                                          EnumValue(ScalarCollector::INPUT_DATA_TYPE_DOUBLE));
        m_terminalCollectors.SetAttribute(
            "OutputType",
            EnumValue(ScalarCollector::OUTPUT_TYPE_AVERAGE_PER_SECOND));
        CreateCollectorPerIdentifier(m_terminalCollectors);
        Callback<void, double> callback =
            MakeCallback(&DistributionCollector::TraceSinkDouble1, m_averagingCollector);
        for (CollectorMap::Iterator it = m_terminalCollectors.Begin();
             it != m_terminalCollectors.End();
             ++it)
        {
            it->second->TraceConnectWithoutContext("Output", callback);
        }

        // Setup first-level collectors.
        m_conversionCollectors.SetType("ns3::UnitConversionCollector");
        m_conversionCollectors.SetAttribute("ConversionType",
                                            EnumValue(UnitConversionCollector::FROM_BYTES_TO_KBIT));
        CreateCollectorPerIdentifier(m_conversionCollectors);
        m_conversionCollectors.ConnectToCollector("Output",
                                                  m_terminalCollectors,
                                                  &ScalarCollector::TraceSinkDouble);
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
        plotAggregator->SetLegend("Time (in seconds)",
                                  "Received throughput (in kilobits per second)");
        plotAggregator->Set2dDatasetDefaultStyle(Gnuplot2dDataset::LINES);
        m_aggregator = plotAggregator;

        // Setup second-level collectors.
        m_terminalCollectors.SetType("ns3::IntervalRateCollector");
        m_terminalCollectors.SetAttribute("InputDataType",
                                          EnumValue(IntervalRateCollector::INPUT_DATA_TYPE_DOUBLE));
        CreateCollectorPerIdentifier(m_terminalCollectors);
        for (CollectorMap::Iterator it = m_terminalCollectors.Begin();
             it != m_terminalCollectors.End();
             ++it)
        {
            const std::string context = it->second->GetName();
            plotAggregator->Add2dDataset(context, context);
        }
        m_terminalCollectors.ConnectToAggregator("OutputWithTime",
                                                 m_aggregator,
                                                 &GnuplotAggregator::Write2d);

        // Setup first-level collectors.
        m_conversionCollectors.SetType("ns3::UnitConversionCollector");
        m_conversionCollectors.SetAttribute("ConversionType",
                                            EnumValue(UnitConversionCollector::FROM_BYTES_TO_KBIT));
        CreateCollectorPerIdentifier(m_conversionCollectors);
        m_conversionCollectors.ConnectToCollector("Output",
                                                  m_terminalCollectors,
                                                  &IntervalRateCollector::TraceSinkDouble);
        break;
    }

    case ApplicationStatsHelper::OUTPUT_HISTOGRAM_PLOT:
    case ApplicationStatsHelper::OUTPUT_PDF_PLOT:
    case ApplicationStatsHelper::OUTPUT_CDF_PLOT: {
        if (!m_averagingMode)
        {
            NS_FATAL_ERROR("This statistics require AveragingMode to be enabled");
        }

        // Setup aggregator.
        Ptr<GnuplotAggregator> plotAggregator = CreateObject<GnuplotAggregator>(GetName());
        // plot->SetTitle ("");
        plotAggregator->SetLegend("Received throughput (in kilobits per second)", "Frequency");
        plotAggregator->Set2dDatasetDefaultStyle(Gnuplot2dDataset::LINES);
        plotAggregator->Add2dDataset(GetName(), GetName());
        m_aggregator = plotAggregator;

        // Setup the final-level collector.
        m_averagingCollector = CreateObject<DistributionCollector>();
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
        m_averagingCollector->SetOutputType(outputType);
        m_averagingCollector->SetName("0");
        m_averagingCollector->TraceConnect(
            "Output",
            GetName(),
            MakeCallback(&GnuplotAggregator::Write2d, plotAggregator));

        // Setup second-level collectors.
        m_terminalCollectors.SetType("ns3::ScalarCollector");
        m_terminalCollectors.SetAttribute("InputDataType",
                                          EnumValue(ScalarCollector::INPUT_DATA_TYPE_DOUBLE));
        m_terminalCollectors.SetAttribute(
            "OutputType",
            EnumValue(ScalarCollector::OUTPUT_TYPE_AVERAGE_PER_SECOND));
        CreateCollectorPerIdentifier(m_terminalCollectors);
        Callback<void, double> callback =
            MakeCallback(&DistributionCollector::TraceSinkDouble1, m_averagingCollector);
        for (CollectorMap::Iterator it = m_terminalCollectors.Begin();
             it != m_terminalCollectors.End();
             ++it)
        {
            it->second->TraceConnectWithoutContext("Output", callback);
        }

        // Setup first-level collectors.
        m_conversionCollectors.SetType("ns3::UnitConversionCollector");
        m_conversionCollectors.SetAttribute("ConversionType",
                                            EnumValue(UnitConversionCollector::FROM_BYTES_TO_KBIT));
        CreateCollectorPerIdentifier(m_conversionCollectors);
        m_conversionCollectors.ConnectToCollector("Output",
                                                  m_terminalCollectors,
                                                  &ScalarCollector::TraceSinkDouble);
        break;
    }

    default:
        NS_FATAL_ERROR("ApplicationStatsThroughputHelper - Invalid output type");
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
        const uint32_t n = SetupProbesAtReceiver<ApplicationPacketProbe>(
            "OutputBytes",
            m_conversionCollectors,
            &UnitConversionCollector::TraceSinkUinteger32,
            m_probes);
        NS_LOG_INFO(this << " created " << n << " instance(s)" << " of ApplicationPacketProbe");
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
            MakeCallback(&ApplicationStatsThroughputHelper::RxCallback, this));
        NS_LOG_INFO(this << " connected to " << n << " trace sources");
        break;
    }

    default:
        NS_FATAL_ERROR("ApplicationStatsThroughputHelper - Invalid identifier type");
        break;

    } // end of `switch (GetIdentifierType ())`

} // end of `void DoInstall ();`

void
ApplicationStatsThroughputHelper::RxCallback(Ptr<const Packet> packet, const Address& from)
{
    // NS_LOG_FUNCTION (this << packet->GetSize () << from);

    if (InetSocketAddress::IsMatchingType(from))
    {
        // Determine the identifier associated with the sender address.
        const Address ipv4Addr = InetSocketAddress::ConvertFrom(from).GetIpv4();
        std::map<const Address, uint32_t>::const_iterator it1 = m_identifierMap.find(ipv4Addr);

        if (it1 == m_identifierMap.end())
        {
            NS_LOG_WARN(this << " discarding packet " << packet << " (" << packet->GetSize()
                             << " bytes)" << " from statistics collection because of"
                             << " unknown sender IPv4 address " << ipv4Addr);
        }
        else
        {
            // Find the collector with the right identifier.
            Ptr<DataCollectionObject> collector = m_conversionCollectors.Get(it1->second);
            NS_ASSERT_MSG(collector != nullptr,
                          "Unable to find collector with identifier " << it1->second);
            Ptr<UnitConversionCollector> c = collector->GetObject<UnitConversionCollector>();
            NS_ASSERT(c != nullptr);

            // Pass the sample to the collector.
            c->TraceSinkUinteger32(0, packet->GetSize());
        }
    }
    else
    {
        NS_LOG_WARN(this << " discarding packet " << packet << " (" << packet->GetSize()
                         << " bytes)" << " from statistics collection"
                         << " because it comes from sender " << from
                         << " without valid InetSocketAddress");
    }

} // end of `void RxCallback (Ptr<const Packet>, const Address &)`

void
ApplicationStatsThroughputHelper::SaveAddressAndIdentifier(Ptr<Application> application,
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

} // end of namespace ns3
