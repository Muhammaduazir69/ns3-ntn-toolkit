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

#include "stats-throughput-helper.h"

#include "ns3/application-packet-probe.h"
#include "ns3/application.h"
#include "ns3/boolean.h"
#include "ns3/callback.h"
#include "ns3/data-collection-object.h"
#include "ns3/distribution-collector.h"
#include "ns3/enum.h"
#include "ns3/inet-socket-address.h"
#include "ns3/interval-rate-collector.h"
#include "ns3/ipv4.h"
#include "ns3/log.h"
#include "ns3/mac48-address.h"
#include "ns3/magister-gnuplot-aggregator.h"
#include "ns3/multi-file-aggregator.h"
#include "ns3/net-device.h"
#include "ns3/node-container.h"
#include "ns3/packet.h"
#include "ns3/probe.h"
#include "ns3/scalar-collector.h"
#include "ns3/singleton.h"
#include "ns3/string.h"
#include "ns3/unit-conversion-collector.h"

#include <map>
#include <sstream>
#include <string>

NS_LOG_COMPONENT_DEFINE("StatsThroughputHelper");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(StatsThroughputHelper);

StatsThroughputHelper::StatsThroughputHelper()
    : StatsHelper(),
      m_averagingMode(false)
{
    NS_LOG_FUNCTION(this);
}

StatsThroughputHelper::~StatsThroughputHelper()
{
    NS_LOG_FUNCTION(this);
}

TypeId // static
StatsThroughputHelper::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::StatsThroughputHelper")
            .SetParent<StatsHelper>()
            .AddAttribute("AveragingMode",
                          "If true, all samples will be averaged before passed to aggregator. "
                          "Only affects histogram, PDF, and CDF output types.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&StatsThroughputHelper::SetAveragingMode,
                                              &StatsThroughputHelper::GetAveragingMode),
                          MakeBooleanChecker());
    return tid;
}

void
StatsThroughputHelper::SetAveragingMode(bool averagingMode)
{
    NS_LOG_FUNCTION(this << averagingMode);
    m_averagingMode = averagingMode;
}

bool
StatsThroughputHelper::GetAveragingMode() const
{
    return m_averagingMode;
}

void
StatsThroughputHelper::DoInstall()
{
    NS_LOG_FUNCTION(this);

    switch (GetOutputType())
    {
    case StatsHelper::OUTPUT_NONE:
        NS_FATAL_ERROR(GetOutputTypeName(GetOutputType())
                       << " is not a valid output type for this statistics.");
        break;

    case StatsHelper::OUTPUT_SCALAR_FILE: {
        // Setup aggregator.
        m_aggregator = CreateAggregator("ns3::MultiFileAggregator",
                                        "OutputFileName",
                                        StringValue(GetOutputFileName()),
                                        "MultiFileMode",
                                        BooleanValue(false),
                                        "EnableContextPrinting",
                                        BooleanValue(true),
                                        "GeneralHeading",
                                        StringValue(GetIdentifierHeading("throughput_kbps")));

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

    case StatsHelper::OUTPUT_SCATTER_FILE: {
        // Setup aggregator.
        m_aggregator = CreateAggregator("ns3::MultiFileAggregator",
                                        "OutputFileName",
                                        StringValue(GetOutputFileName()),
                                        "GeneralHeading",
                                        StringValue(GetTimeHeading("throughput_kbps")));

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

    case StatsHelper::OUTPUT_HISTOGRAM_FILE:
    case StatsHelper::OUTPUT_PDF_FILE:
    case StatsHelper::OUTPUT_CDF_FILE: {
        if (!m_averagingMode)
        {
            NS_FATAL_ERROR("This statistics require AveragingMode to be enabled");
        }

        // Setup aggregator.
        m_aggregator = CreateAggregator("ns3::MultiFileAggregator",
                                        "OutputFileName",
                                        StringValue(GetOutputFileName()),
                                        "MultiFileMode",
                                        BooleanValue(false),
                                        "EnableContextPrinting",
                                        BooleanValue(false),
                                        "GeneralHeading",
                                        StringValue(GetDistributionHeading("throughput_kbps")));
        Ptr<MultiFileAggregator> fileAggregator = m_aggregator->GetObject<MultiFileAggregator>();
        NS_ASSERT(fileAggregator != nullptr);

        // Setup the final-level collector.
        m_averagingCollector = CreateObject<DistributionCollector>();
        DistributionCollector::OutputType_t outputType =
            DistributionCollector::OUTPUT_TYPE_HISTOGRAM;
        if (GetOutputType() == StatsHelper::OUTPUT_PDF_FILE)
        {
            outputType = DistributionCollector::OUTPUT_TYPE_PROBABILITY;
        }
        else if (GetOutputType() == StatsHelper::OUTPUT_CDF_FILE)
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

    case StatsHelper::OUTPUT_SCALAR_PLOT:
        /// @todo Add support for boxes in Gnuplot.
        NS_FATAL_ERROR(GetOutputTypeName(GetOutputType())
                       << " is not a valid output type for this statistics.");
        break;

    case StatsHelper::OUTPUT_SCATTER_PLOT: {
        // Setup aggregator.
        m_aggregator = CreateAggregator("ns3::MagisterGnuplotAggregator",
                                        "OutputPath",
                                        StringValue(GetOutputPath()),
                                        "OutputFileName",
                                        StringValue(GetName()));
        Ptr<MagisterGnuplotAggregator> plotAggregator =
            m_aggregator->GetObject<MagisterGnuplotAggregator>();
        NS_ASSERT(plotAggregator != nullptr);
        // plot->SetTitle ("");
        plotAggregator->SetLegend("Time (in seconds)",
                                  "Received throughput (in kilobits per second)");
        plotAggregator->Set2dDatasetDefaultStyle(Gnuplot2dDataset::LINES);

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
                                                 &MagisterGnuplotAggregator::Write2d);

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

    case StatsHelper::OUTPUT_HISTOGRAM_PLOT:
    case StatsHelper::OUTPUT_PDF_PLOT:
    case StatsHelper::OUTPUT_CDF_PLOT: {
        if (!m_averagingMode)
        {
            NS_FATAL_ERROR("This statistics requires AveragingMode to be enabled");
        }

        // Setup aggregator.
        m_aggregator = CreateAggregator("ns3::MagisterGnuplotAggregator",
                                        "OutputPath",
                                        StringValue(GetOutputPath()),
                                        "OutputFileName",
                                        StringValue(GetName()));
        Ptr<MagisterGnuplotAggregator> plotAggregator =
            m_aggregator->GetObject<MagisterGnuplotAggregator>();
        NS_ASSERT(plotAggregator != nullptr);
        // plot->SetTitle ("");
        plotAggregator->SetLegend("Received throughput (in kilobits per second)", "Frequency");
        plotAggregator->Set2dDatasetDefaultStyle(Gnuplot2dDataset::LINES);
        plotAggregator->Add2dDataset(GetName(), GetName());
        /// @todo Find a better dataset name.

        // Setup the final-level collector.
        m_averagingCollector = CreateObject<DistributionCollector>();
        DistributionCollector::OutputType_t outputType =
            DistributionCollector::OUTPUT_TYPE_HISTOGRAM;
        if (GetOutputType() == StatsHelper::OUTPUT_PDF_PLOT)
        {
            outputType = DistributionCollector::OUTPUT_TYPE_PROBABILITY;
        }
        else if (GetOutputType() == StatsHelper::OUTPUT_CDF_PLOT)
        {
            outputType = DistributionCollector::OUTPUT_TYPE_CUMULATIVE;
        }
        m_averagingCollector->SetOutputType(outputType);
        m_averagingCollector->SetName("0");
        m_averagingCollector->TraceConnect(
            "Output",
            GetName(),
            MakeCallback(&MagisterGnuplotAggregator::Write2d, plotAggregator));
        /// @todo Find a better dataset name.

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
        NS_FATAL_ERROR("StatsThroughputHelper - Invalid output type");
        break;
    }

    // Setup probes and connect them to conversion collectors.
    InstallProbes();

} // end of `void DoInstall ();`

void
StatsThroughputHelper::InstallProbes()
{
    NS_LOG_FUNCTION(this);

    // The method below is supposed to be implemented by the child class.
    DoInstallProbes();
}

void
StatsThroughputHelper::RxCallback(Ptr<const Packet> packet, const Address& from)
{
    NS_LOG_FUNCTION(this << packet->GetSize() << from);

    if (from.IsInvalid())
    {
        NS_LOG_WARN(this << " discarding packet " << packet << " (" << packet->GetSize()
                         << " bytes)" << " from statistics collection because of"
                         << " invalid sender address");
    }
    else
    {
        // Determine the identifier associated with the sender address.
        std::map<const Address, uint32_t>::const_iterator it = m_identifierMap.find(from);

        if (it == m_identifierMap.end())
        {
            NS_LOG_WARN(this << " discarding packet " << packet << " (" << packet->GetSize()
                             << " bytes)" << " from statistics collection because of"
                             << " unknown sender address " << from);
        }
        else
        {
            // Find the first-level collector with the right identifier.
            Ptr<DataCollectionObject> collector = m_conversionCollectors.Get(it->second);
            NS_ASSERT_MSG(collector != nullptr,
                          "Unable to find collector with identifier " << it->second);
            Ptr<UnitConversionCollector> c = collector->GetObject<UnitConversionCollector>();
            NS_ASSERT(c != nullptr);

            // Pass the sample to the collector.
            c->TraceSinkUinteger32(0, packet->GetSize());
        }
    }

} // end of `void RxCallback (Ptr<const Packet>, const Address);`

// APPLICATION-LEVEL /////////////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED(StatsAppThroughputHelper);

StatsAppThroughputHelper::StatsAppThroughputHelper()
    : StatsThroughputHelper()
{
    NS_LOG_FUNCTION(this);
}

StatsAppThroughputHelper::~StatsAppThroughputHelper()
{
    NS_LOG_FUNCTION(this);
}

TypeId // static
StatsAppThroughputHelper::GetTypeId()
{
    static TypeId tid = TypeId("ns3::StatsAppThroughputHelper").SetParent<StatsThroughputHelper>();
    return tid;
}

void
StatsAppThroughputHelper::DoInstallProbes()
{
    NS_LOG_FUNCTION(this);

    NodeContainer nodes = GetNodes();
    for (NodeContainer::Iterator it = nodes.Begin(); it != nodes.End(); ++it)
    {
        const int32_t id = (*it)->GetId();
        uint32_t identifier = 0;
        if (GetIdentifierType() == StatsHelper::IDENTIFIER_NODE)
        {
            identifier = id;
        }

        for (uint32_t i = 0; i < (*it)->GetNApplications(); i++)
        {
            // Create the probe.
            std::ostringstream probeName;
            probeName << id << "-" << i;
            Ptr<ApplicationPacketProbe> probe = CreateObject<ApplicationPacketProbe>();
            probe->SetName(probeName.str());

            // Connect the object to the probe.
            if (probe->ConnectByObject("Rx", (*it)->GetApplication(i)))
            {
                // Connect the probe to the right collector.
                if (m_conversionCollectors.ConnectWithProbe(
                        probe->GetObject<Probe>(),
                        "OutputBytes",
                        identifier,
                        &UnitConversionCollector::TraceSinkUinteger32))
                {
                    NS_LOG_INFO(this << " created probe " << probeName.str()
                                     << ", connected to collector " << identifier);
                    m_probes.push_back(probe->GetObject<Probe>());
                }
                else
                {
                    NS_LOG_WARN(this << " unable to connect probe " << probeName.str()
                                     << " to collector " << identifier);
                }
            }
            else
            {
                /*
                 * We're being tolerant here by only logging a warning, because
                 * not every kind of Application is equipped with the expected
                 * Rx trace source.
                 */
                NS_LOG_WARN(this << " unable to connect probe " << probeName.str()
                                 << " with node ID " << (*it)->GetId() << " application #" << i);
            }

        } // end of `for (i = 0; i < (*it)->GetNApplications (); i++)`

    } // end of `for (it = nodes.Begin(); it != nodes.End (); ++it)`

} // end of `void DoInstallProbes ();`

} // end of namespace ns3
