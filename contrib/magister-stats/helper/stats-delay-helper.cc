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

#include "stats-delay-helper.h"

#include "ns3/address-time-probe.h"
#include "ns3/application.h"
#include "ns3/boolean.h"
#include "ns3/callback.h"
#include "ns3/data-collection-object.h"
#include "ns3/distribution-collector.h"
#include "ns3/enum.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4.h"
#include "ns3/log.h"
#include "ns3/mac48-address.h"
#include "ns3/magister-gnuplot-aggregator.h"
#include "ns3/multi-file-aggregator.h"
#include "ns3/net-device.h"
#include "ns3/node-container.h"
#include "ns3/nstime.h"
#include "ns3/packet.h"
#include "ns3/probe.h"
#include "ns3/scalar-collector.h"
#include "ns3/singleton.h"
#include "ns3/string.h"
#include "ns3/traffic-time-tag.h"
#include "ns3/unit-conversion-collector.h"

#include <sstream>
#include <string>

NS_LOG_COMPONENT_DEFINE("StatsDelayHelper");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(StatsDelayHelper);

StatsDelayHelper::StatsDelayHelper()
    : StatsHelper(),
      m_averagingMode(false)
{
    NS_LOG_FUNCTION(this);
}

StatsDelayHelper::~StatsDelayHelper()
{
    NS_LOG_FUNCTION(this);
}

TypeId // static
StatsDelayHelper::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::StatsDelayHelper")
            .SetParent<StatsHelper>()
            .AddAttribute("AveragingMode",
                          "If true, all samples will be averaged before passed to aggregator. "
                          "Only affects histogram, PDF, and CDF output types.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&StatsDelayHelper::SetAveragingMode,
                                              &StatsDelayHelper::GetAveragingMode),
                          MakeBooleanChecker());
    return tid;
}

void
StatsDelayHelper::SetAveragingMode(bool averagingMode)
{
    NS_LOG_FUNCTION(this << averagingMode);
    m_averagingMode = averagingMode;
}

bool
StatsDelayHelper::GetAveragingMode() const
{
    return m_averagingMode;
}

void
StatsDelayHelper::DoInstall()
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
                                        StringValue(GetIdentifierHeading("delay_sec")));

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

    case StatsHelper::OUTPUT_SCATTER_FILE: {
        // Setup aggregator.
        m_aggregator = CreateAggregator("ns3::MultiFileAggregator",
                                        "OutputFileName",
                                        StringValue(GetOutputFileName()),
                                        "GeneralHeading",
                                        StringValue(GetTimeHeading("delay_sec")));

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

    case StatsHelper::OUTPUT_HISTOGRAM_FILE:
    case StatsHelper::OUTPUT_PDF_FILE:
    case StatsHelper::OUTPUT_CDF_FILE: {
        if (m_averagingMode)
        {
            // Setup aggregator.
            m_aggregator = CreateAggregator("ns3::MultiFileAggregator",
                                            "OutputFileName",
                                            StringValue(GetOutputFileName()),
                                            "MultiFileMode",
                                            BooleanValue(false),
                                            "EnableContextPrinting",
                                            BooleanValue(false),
                                            "GeneralHeading",
                                            StringValue(GetDistributionHeading("delay_sec")));
            Ptr<MultiFileAggregator> fileAggregator =
                m_aggregator->GetObject<MultiFileAggregator>();
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

            // Setup collectors.
            m_terminalCollectors.SetType("ns3::ScalarCollector");
            m_terminalCollectors.SetAttribute("InputDataType",
                                              EnumValue(ScalarCollector::INPUT_DATA_TYPE_DOUBLE));
            m_terminalCollectors.SetAttribute(
                "OutputType",
                EnumValue(ScalarCollector::OUTPUT_TYPE_AVERAGE_PER_SAMPLE));
            CreateCollectorPerIdentifier(m_terminalCollectors);
            Callback<void, double> callback =
                MakeCallback(&DistributionCollector::TraceSinkDouble1, m_averagingCollector);
            for (CollectorMap::Iterator it = m_terminalCollectors.Begin();
                 it != m_terminalCollectors.End();
                 ++it)
            {
                it->second->TraceConnectWithoutContext("Output", callback);
            }
        }
        else
        {
            // Setup aggregator.
            m_aggregator = CreateAggregator("ns3::MultiFileAggregator",
                                            "OutputFileName",
                                            StringValue(GetOutputFileName()),
                                            "GeneralHeading",
                                            StringValue(GetDistributionHeading("delay_sec")));

            // Setup collectors.
            m_terminalCollectors.SetType("ns3::DistributionCollector");
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
        }

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
        plotAggregator->SetLegend("Time (in seconds)", "Packet delay (in seconds)");
        plotAggregator->Set2dDatasetDefaultStyle(Gnuplot2dDataset::LINES);

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
                                                 &MagisterGnuplotAggregator::Write2d);
        break;
    }

    case StatsHelper::OUTPUT_HISTOGRAM_PLOT:
    case StatsHelper::OUTPUT_PDF_PLOT:
    case StatsHelper::OUTPUT_CDF_PLOT: {
        if (m_averagingMode)
        {
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
            plotAggregator->SetLegend("Packet delay (in seconds)", "Frequency");
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

            // Setup collectors.
            m_terminalCollectors.SetType("ns3::ScalarCollector");
            m_terminalCollectors.SetAttribute("InputDataType",
                                              EnumValue(ScalarCollector::INPUT_DATA_TYPE_DOUBLE));
            m_terminalCollectors.SetAttribute(
                "OutputType",
                EnumValue(ScalarCollector::OUTPUT_TYPE_AVERAGE_PER_SAMPLE));
            CreateCollectorPerIdentifier(m_terminalCollectors);
            Callback<void, double> callback =
                MakeCallback(&DistributionCollector::TraceSinkDouble1, m_averagingCollector);
            for (CollectorMap::Iterator it = m_terminalCollectors.Begin();
                 it != m_terminalCollectors.End();
                 ++it)
            {
                it->second->TraceConnectWithoutContext("Output", callback);
            }
        }
        else
        {
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
            plotAggregator->SetLegend("Packet delay (in seconds)", "Frequency");
            plotAggregator->Set2dDatasetDefaultStyle(Gnuplot2dDataset::LINES);

            // Setup collectors.
            m_terminalCollectors.SetType("ns3::DistributionCollector");
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
                                                     &MagisterGnuplotAggregator::Write2d);
        }

        break;
    }

    default:
        NS_FATAL_ERROR("StatsDelayHelper - Invalid output type");
        break;
    }

    // Setup probes and connect them to the collectors.
    InstallProbes();

} // end of `void DoInstall ();`

void
StatsDelayHelper::InstallProbes()
{
    // The method below is supposed to be implemented by the child class.
    DoInstallProbes();
}

bool
StatsDelayHelper::ConnectProbeToCollector(Ptr<Probe> probe, uint32_t identifier)
{
    NS_LOG_FUNCTION(this << probe << probe->GetName() << identifier);

    bool ret = false;
    switch (GetOutputType())
    {
    case StatsHelper::OUTPUT_SCALAR_FILE:
    case StatsHelper::OUTPUT_SCALAR_PLOT:
        ret = m_terminalCollectors.ConnectWithProbe(probe,
                                                    "OutputSeconds",
                                                    identifier,
                                                    &ScalarCollector::TraceSinkDouble);
        break;

    case StatsHelper::OUTPUT_SCATTER_FILE:
    case StatsHelper::OUTPUT_SCATTER_PLOT:
        ret = m_terminalCollectors.ConnectWithProbe(probe,
                                                    "OutputSeconds",
                                                    identifier,
                                                    &UnitConversionCollector::TraceSinkDouble);
        break;

    case StatsHelper::OUTPUT_HISTOGRAM_FILE:
    case StatsHelper::OUTPUT_HISTOGRAM_PLOT:
    case StatsHelper::OUTPUT_PDF_FILE:
    case StatsHelper::OUTPUT_PDF_PLOT:
    case StatsHelper::OUTPUT_CDF_FILE:
    case StatsHelper::OUTPUT_CDF_PLOT:
        if (m_averagingMode)
        {
            ret = m_terminalCollectors.ConnectWithProbe(probe,
                                                        "OutputSeconds",
                                                        identifier,
                                                        &ScalarCollector::TraceSinkDouble);
        }
        else
        {
            ret = m_terminalCollectors.ConnectWithProbe(probe,
                                                        "OutputSeconds",
                                                        identifier,
                                                        &DistributionCollector::TraceSinkDouble);
        }
        break;

    default:
        NS_FATAL_ERROR(GetOutputTypeName(GetOutputType())
                       << " is not a valid output type for this statistics.");
        break;
    }

    if (ret)
    {
        NS_LOG_INFO(this << " created probe " << probe->GetName() << ", connected to collector "
                         << identifier);
    }
    else
    {
        NS_LOG_WARN(this << " unable to connect probe " << probe->GetName() << " to collector "
                         << identifier);
    }

    return ret;
}

void
StatsDelayHelper::PassSampleToCollector(Time delay, uint32_t identifier)
{
    // NS_LOG_FUNCTION (this << delay.GetSeconds () << identifier);

    Ptr<DataCollectionObject> collector = m_terminalCollectors.Get(identifier);
    NS_ASSERT_MSG(collector != nullptr, "Unable to find collector with identifier " << identifier);

    switch (GetOutputType())
    {
    case StatsHelper::OUTPUT_SCALAR_FILE:
    case StatsHelper::OUTPUT_SCALAR_PLOT: {
        Ptr<ScalarCollector> c = collector->GetObject<ScalarCollector>();
        NS_ASSERT(c != nullptr);
        c->TraceSinkDouble(0.0, delay.GetSeconds());
        break;
    }

    case StatsHelper::OUTPUT_SCATTER_FILE:
    case StatsHelper::OUTPUT_SCATTER_PLOT: {
        Ptr<UnitConversionCollector> c = collector->GetObject<UnitConversionCollector>();
        NS_ASSERT(c != nullptr);
        c->TraceSinkDouble(0.0, delay.GetSeconds());
        break;
    }

    case StatsHelper::OUTPUT_HISTOGRAM_FILE:
    case StatsHelper::OUTPUT_HISTOGRAM_PLOT:
    case StatsHelper::OUTPUT_PDF_FILE:
    case StatsHelper::OUTPUT_PDF_PLOT:
    case StatsHelper::OUTPUT_CDF_FILE:
    case StatsHelper::OUTPUT_CDF_PLOT:
        if (m_averagingMode)
        {
            Ptr<ScalarCollector> c = collector->GetObject<ScalarCollector>();
            NS_ASSERT(c != nullptr);
            c->TraceSinkDouble(0.0, delay.GetSeconds());
        }
        else
        {
            Ptr<DistributionCollector> c = collector->GetObject<DistributionCollector>();
            NS_ASSERT(c != nullptr);
            c->TraceSinkDouble(0.0, delay.GetSeconds());
        }
        break;

    default:
        NS_FATAL_ERROR(GetOutputTypeName(GetOutputType())
                       << " is not a valid output type for this statistics.");
        break;

    } // end of `switch (GetOutputType ())`

} // end of `void PassSampleToCollector (Time, uint32_t)`

// FORWARD LINK APPLICATION-LEVEL /////////////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED(StatsAppDelayHelper);

StatsAppDelayHelper::StatsAppDelayHelper()
    : StatsDelayHelper()
{
    NS_LOG_FUNCTION(this);
}

StatsAppDelayHelper::~StatsAppDelayHelper()
{
    NS_LOG_FUNCTION(this);
}

TypeId // static
StatsAppDelayHelper::GetTypeId()
{
    static TypeId tid = TypeId("ns3::StatsAppDelayHelper").SetParent<StatsDelayHelper>();
    return tid;
}

void
StatsAppDelayHelper::DoInstallProbes()
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
            Ptr<Application> app = (*it)->GetApplication(i);
            bool rxIsConnected = false;
            bool txIsConnected = false;

            /**
             * Connect to Rx trace sources of receiver applications in order to read
             * traffic time tags.
             */
            if (app->GetInstanceTypeId().LookupTraceSourceByName("Rx") != nullptr)
            {
                NS_LOG_INFO(this << " attempt to connect using Rx");
                Callback<void, Ptr<const Packet>, const Address&> rxCallback =
                    MakeBoundCallback(&StatsAppDelayHelper::RxCallback, this, identifier);
                rxIsConnected = app->TraceConnectWithoutContext("Rx", rxCallback);
            }

            /**
             * Connect to Tx trace sources of sender applications in order to attach traffic time
             * tags
             */
            if (app->GetInstanceTypeId().LookupTraceSourceByName("Tx") != nullptr)
            {
                NS_LOG_INFO(this << " attempt to connect using Tx");
                Callback<void, Ptr<const Packet>> txCallback =
                    MakeBoundCallback(&StatsAppDelayHelper::TxCallback, this);
                txIsConnected = app->TraceConnectWithoutContext("Tx", txCallback);
            }

            if (rxIsConnected)
            {
                NS_LOG_INFO(this << " successfully connected to Rx " << " with node ID "
                                 << (*it)->GetId() << " application #" << i);
            }
            if (txIsConnected)
            {
                NS_LOG_INFO(this << " successfully connected to Tx " << " with node ID "
                                 << (*it)->GetId() << " application #" << i);
            }
            if (!txIsConnected && !rxIsConnected)
            {
                /*
                 * We're being tolerant here by only logging a warning, because
                 * not every kind of Application is equipped with the expected
                 * RxDelay or Rx or Tx trace sources.
                 */
                NS_LOG_WARN(this << " unable to connect" << " with node ID " << (*it)->GetId()
                                 << " application #" << i);
            }

        } // end of `for (i = 0; i < (*it)->GetNApplications (); i++)`

    } // end of `for (it = nodes.Begin(); it != nodes.End (); ++it)`

} // end of `void DoInstallProbes ();`

void // static
StatsAppDelayHelper::RxCallback(Ptr<StatsAppDelayHelper> helper,
                                uint32_t identifier,
                                Ptr<const Packet> packet,
                                const Address& from)
{
    NS_LOG_FUNCTION(helper << identifier << packet << packet->GetSize() << from);

    bool isTagged = false;
    ByteTagIterator it = packet->GetByteTagIterator();

    while (!isTagged && it.HasNext())
    {
        ByteTagIterator::Item item = it.Next();

        if (item.GetTypeId() == TrafficTimeTag::GetTypeId())
        {
            NS_LOG_DEBUG("Contains a TrafficTimeTag tag:" << " start=" << item.GetStart()
                                                          << " end=" << item.GetEnd());
            TrafficTimeTag timeTag;
            item.GetTag(timeTag);
            const Time delay = Simulator::Now() - timeTag.GetSenderTimestamp();
            helper->PassSampleToCollector(delay, identifier);
            isTagged = true; // this will exit the while loop.
        }
    }

    if (!isTagged)
    {
        NS_LOG_WARN("Discarding a packet of " << packet->GetSize() << " from statistics collection"
                                              << " because it does not contain any TrafficTimeTag");
    }

    /*
    TrafficTimeTag timeTag;
    if (packet->PeekPacketTag (timeTag))
      {
        NS_LOG_DEBUG ("Contains a TrafficTimeTag tag");
        const Time delay = Simulator::Now () - timeTag.GetSenderTimestamp ();
        helper->PassSampleToCollector (delay, identifier);
      }
    else
      {
        NS_LOG_WARN ("Discarding a packet of " << packet->GetSize ()
                                               << " from statistics collection"
                                               << " because it does not contain any
    TrafficTimeTag");
      }
      */
}

void // static
StatsAppDelayHelper::TxCallback(Ptr<StatsAppDelayHelper> helper, Ptr<const Packet> packet)
{
    NS_LOG_FUNCTION(helper << packet << packet->GetSize());

    TrafficTimeTag timeTag(Simulator::Now());
    packet->AddByteTag(timeTag);
    NS_LOG_INFO(helper << " attached TrafficTimeTag to packet " << packet << " of size "
                       << packet->GetSize());
}

} // end of namespace ns3
