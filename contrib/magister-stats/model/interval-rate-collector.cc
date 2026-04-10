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

#include "interval-rate-collector.h"

#include "magister-stats.h"

#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <iostream>
#include <sstream>
#include <string>

NS_LOG_COMPONENT_DEFINE("IntervalRateCollector");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(IntervalRateCollector);

std::string // static
IntervalRateCollector::GetInputDataTypeName(IntervalRateCollector::InputDataType_t inputDataType)
{
    switch (inputDataType)
    {
    case IntervalRateCollector::INPUT_DATA_TYPE_DOUBLE:
        return "INPUT_DATA_TYPE_DOUBLE";
    case IntervalRateCollector::INPUT_DATA_TYPE_UINTEGER:
        return "INPUT_DATA_TYPE_UINTEGER";
    case IntervalRateCollector::INPUT_DATA_TYPE_BOOLEAN:
        return "INPUT_DATA_TYPE_BOOLEAN";
    default:
        return "";
    }
}

std::string // static
IntervalRateCollector::GetOutputTypeName(IntervalRateCollector::OutputType_t outputType)
{
    switch (outputType)
    {
    case IntervalRateCollector::OUTPUT_TYPE_SUM:
        return "OUTPUT_TYPE_SUM";
    case IntervalRateCollector::OUTPUT_TYPE_NUMBER_OF_SAMPLE:
        return "OUTPUT_TYPE_NUMBER_OF_SAMPLE";
    case IntervalRateCollector::OUTPUT_TYPE_AVERAGE_PER_SAMPLE:
        return "OUTPUT_TYPE_AVERAGE_PER_SAMPLE";
    default:
        return "";
    }
}

IntervalRateCollector::IntervalRateCollector()
    : m_intervalSumDouble(0.0),
      m_overallSumDouble(0.0),
      m_intervalSumUinteger(0),
      m_overallSumUinteger(0),
      m_intervalNumOfSamples(0),
      m_overallNumOfSamples(0),
      m_nextReset(),
      m_intervalLength(Seconds(1.0)),
      m_inputDataType(IntervalRateCollector::INPUT_DATA_TYPE_DOUBLE),
      m_outputType(IntervalRateCollector::OUTPUT_TYPE_SUM),
      m_timeUnit(Time::S)
{
    NS_LOG_FUNCTION(this << GetName());

    // Delayed start to ensure attributes are completely initialized.
    Simulator::ScheduleNow(&IntervalRateCollector::FirstInterval, this);
}

TypeId // static
IntervalRateCollector::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::IntervalRateCollector")
            .SetParent<DataCollectionObject>()
            .AddConstructor<IntervalRateCollector>()
            .AddAttribute("IntervalLength",
                          "Control the frequency of producing output. For example, "
                          "an interval length of one second (the default) causes "
                          "this collector instance to emit new output via the "
                          "`OutputWithTime` and `OutputWithoutTime` trace sources "
                          "after every one second. Accumulated values are reset to "
                          "zero after every output invocation, hence the values "
                          "emitted by these trace sources can be regarded as rate or "
                          "throughput. Note that an interval length of zero prevents "
                          "these trace sources from emitting any output.",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&IntervalRateCollector::SetIntervalLength,
                                           &IntervalRateCollector::GetIntervalLength),
                          MakeTimeChecker())
            .AddAttribute("InputDataType",
                          "The data type accepted as inputs. "
                          "The value `INPUT_DATA_TYPE_DOUBLE` (the default) will "
                          "activate the TraceSinkDouble() method. "
                          "The value `INPUT_DATA_TYPE_UINTEGER` will activate the "
                          "TraceSinkUinteger8(), TraceSinkUinteger16(), "
                          "TraceSinkUinteger32(), and TraceSinkUinteger64() methods. "
                          "Finally, the value `INPUT_DATA_TYPE_BOOLEAN` will activate"
                          "the TraceSinkBoolean() method. "
                          "The separation of input data type is useful for preserving "
                          "accuracy (e.g., unsigned integer has better accuracy "
                          "at handling packet sizes, but has the risk of overflow). "
                          "In spite of this separation, output data type from trace "
                          "sources are still fixed to `double` in any case.",
                          EnumValue(IntervalRateCollector::INPUT_DATA_TYPE_DOUBLE),
                          MakeEnumAccessor<IntervalRateCollector::InputDataType_t>(
                              &IntervalRateCollector::SetInputDataType,
                              &IntervalRateCollector::GetInputDataType),
                          MakeEnumChecker(IntervalRateCollector::INPUT_DATA_TYPE_DOUBLE,
                                          "DOUBLE",
                                          IntervalRateCollector::INPUT_DATA_TYPE_UINTEGER,
                                          "UINTEGER",
                                          IntervalRateCollector::INPUT_DATA_TYPE_BOOLEAN,
                                          "BOOLEAN"))
            .AddAttribute("OutputType",
                          "Determines the mechanism of processing the incoming samples.",
                          EnumValue(IntervalRateCollector::OUTPUT_TYPE_SUM),
                          MakeEnumAccessor<IntervalRateCollector::OutputType_t>(
                              &IntervalRateCollector::SetOutputType,
                              &IntervalRateCollector::GetOutputType),
                          MakeEnumChecker(IntervalRateCollector::OUTPUT_TYPE_SUM,
                                          "SUM",
                                          IntervalRateCollector::OUTPUT_TYPE_NUMBER_OF_SAMPLE,
                                          "NUMBER_OF_SAMPLE",
                                          IntervalRateCollector::OUTPUT_TYPE_AVERAGE_PER_SAMPLE,
                                          "AVERAGE_PER_SAMPLE"))
            .AddAttribute("TimeUnit",
                          "Determines the unit used for the time output (i.e., the "
                          "`OutputWithTime` trace source",
                          EnumValue(Time::S),
                          MakeEnumAccessor<Time::Unit>(&IntervalRateCollector::SetTimeUnit,
                                                       &IntervalRateCollector::GetTimeUnit),
                          MakeEnumChecker(Time::Y,
                                          "Y", // year, 365 days
                                          Time::D,
                                          "D", // day, 24 hours
                                          Time::H,
                                          "H", // hour, 60 minutes
                                          Time::MIN,
                                          "MIN", // minute, 60 seconds
                                          Time::S,
                                          "S", // second
                                          Time::MS,
                                          "MS", // millisecond
                                          Time::US,
                                          "US", // microsecond
                                          Time::NS,
                                          "NS", // nanosecond
                                          Time::PS,
                                          "PS", // picosecond
                                          Time::FS,
                                          "FS", // femtosecond
                                          Time::LAST,
                                          "LAST"))
            .AddTraceSource("OutputOverall",
                            "The accumulated sum, "
                            "fired when the collector instance is destroyed.",
                            MakeTraceSourceAccessor(&IntervalRateCollector::m_outputOverall),
                            "ns3::CollectorOutputCallback")
            .AddTraceSource("OutputWithTime",
                            "The recent interval's ending time "
                            "and the accumulated sum during the interval.",
                            MakeTraceSourceAccessor(&IntervalRateCollector::m_outputWithTime),
                            "ns3::CollectorTimedOutputCallback")
            .AddTraceSource("OutputWithoutTime",
                            "The accumulated sum during the recent interval.",
                            MakeTraceSourceAccessor(&IntervalRateCollector::m_outputWithoutTime),
                            "ns3::CollectorOutputCallback")
            .AddTraceSource("OutputString",
                            "Various setup and statistical information, "
                            "fired when the collector instance is destroyed.",
                            MakeTraceSourceAccessor(&IntervalRateCollector::m_outputString),
                            "ns3::CollectorInformationCallback");
    return tid;
}

void
IntervalRateCollector::DoDispose()
{
    NS_LOG_FUNCTION(this << GetName());

    if (IsEnabled())
    {
        double sum = 0.0;

        switch (m_inputDataType)
        {
        case IntervalRateCollector::INPUT_DATA_TYPE_DOUBLE:
            sum = m_overallSumDouble;
            break;

        case IntervalRateCollector::INPUT_DATA_TYPE_UINTEGER:
        case IntervalRateCollector::INPUT_DATA_TYPE_BOOLEAN:
            sum = static_cast<double>(m_overallSumUinteger);
            break;

        default:
            break;
        }

        switch (m_outputType)
        {
        case IntervalRateCollector::OUTPUT_TYPE_SUM:
            m_outputOverall(sum);
            break;

        case IntervalRateCollector::OUTPUT_TYPE_NUMBER_OF_SAMPLE:
            m_outputOverall(m_overallNumOfSamples);
            break;

        case IntervalRateCollector::OUTPUT_TYPE_AVERAGE_PER_SAMPLE:
            // This may produce -nan if number of sample is zero.
            m_outputOverall(sum / static_cast<double>(m_overallNumOfSamples));
            break;

        default:
            break;
        }

        // Compute output for `OutputString` trace source.
        std::ostringstream oss;
        oss << "% output_type: '" << GetOutputTypeName(m_outputType) << "'" << std::endl;
        oss << "% count: " << m_overallNumOfSamples << std::endl;
        oss << "% sum: " << sum << std::endl;
        m_outputString(oss.str());

    } // end of `if (IsEnabled ())`

} // end of `void DoDispose ();`

void
IntervalRateCollector::SetIntervalLength(Time intervalLength)
{
    NS_LOG_FUNCTION(this << GetName() << intervalLength.GetSeconds());
    m_intervalLength = intervalLength;
}

Time
IntervalRateCollector::GetIntervalLength() const
{
    return m_intervalLength;
}

void
IntervalRateCollector::SetInputDataType(IntervalRateCollector::InputDataType_t inputDataType)
{
    NS_LOG_FUNCTION(this << GetName() << GetInputDataTypeName(inputDataType));
    m_inputDataType = inputDataType;
}

IntervalRateCollector::InputDataType_t
IntervalRateCollector::GetInputDataType() const
{
    return m_inputDataType;
}

void
IntervalRateCollector::SetOutputType(IntervalRateCollector::OutputType_t outputType)
{
    NS_LOG_FUNCTION(this << GetName() << GetOutputTypeName(outputType));
    m_outputType = outputType;
}

IntervalRateCollector::OutputType_t
IntervalRateCollector::GetOutputType() const
{
    return m_outputType;
}

void
IntervalRateCollector::SetTimeUnit(Time::Unit unit)
{
    NS_LOG_FUNCTION(this << GetName() << unit);
    m_timeUnit = unit;
}

Time::Unit
IntervalRateCollector::GetTimeUnit() const
{
    return m_timeUnit;
}

void
IntervalRateCollector::FirstInterval()
{
    NS_LOG_FUNCTION(this << GetName());

    if (m_intervalLength > MilliSeconds(0))
    {
        // Schedule the next interval
        m_nextReset =
            Simulator::Schedule(m_intervalLength, &IntervalRateCollector::NewInterval, this);
    }
}

void
IntervalRateCollector::NewInterval()
{
    NS_LOG_FUNCTION(this << GetName());

    if (IsEnabled())
    {
        const double time = Simulator::Now().ToDouble(m_timeUnit);

        double sum = 0.0;

        switch (m_inputDataType)
        {
        case IntervalRateCollector::INPUT_DATA_TYPE_DOUBLE:
            sum = m_intervalSumDouble;
            break;

        case IntervalRateCollector::INPUT_DATA_TYPE_UINTEGER:
        case IntervalRateCollector::INPUT_DATA_TYPE_BOOLEAN:
            sum = static_cast<double>(m_intervalSumUinteger);
            break;

        default:
            break;
        }

        switch (m_outputType)
        {
        case IntervalRateCollector::OUTPUT_TYPE_SUM:
            m_outputWithTime(time, sum);
            m_outputWithoutTime(sum);
            break;

        case IntervalRateCollector::OUTPUT_TYPE_NUMBER_OF_SAMPLE:
            m_outputWithTime(time, m_intervalNumOfSamples);
            m_outputWithoutTime(m_intervalNumOfSamples);
            break;

        case IntervalRateCollector::OUTPUT_TYPE_AVERAGE_PER_SAMPLE: {
            // This may produce -nan if number of sample is zero.
            const double ratio = sum / static_cast<double>(m_intervalNumOfSamples);
            m_outputWithTime(time, ratio);
            m_outputWithoutTime(ratio);
            break;
        }

        default:
            break;
        }
    }

    // Reset the accumulated values.
    m_intervalSumDouble = 0.0;
    m_intervalSumUinteger = 0;
    m_intervalNumOfSamples = 0;

    if (m_intervalLength > MilliSeconds(0))
    {
        // Schedule the next interval
        m_nextReset =
            Simulator::Schedule(m_intervalLength, &IntervalRateCollector::NewInterval, this);
    }
}

void
IntervalRateCollector::TraceSinkDouble(double oldData, double newData)
{
    NS_LOG_FUNCTION(this << GetName() << oldData << newData);

    if (IsEnabled())
    {
        if (m_inputDataType == IntervalRateCollector::INPUT_DATA_TYPE_DOUBLE)
        {
            m_intervalSumDouble += newData;
            m_overallSumDouble += newData;
            m_intervalNumOfSamples++;
            m_overallNumOfSamples++;
        }
        else
        {
            NS_LOG_WARN(this << " ignoring the incoming sample " << newData
                             << " because of unexpected data type");
        }
    }
}

void
IntervalRateCollector::TraceSinkUinteger8(uint8_t oldData, uint8_t newData)
{
    TraceSinkUinteger64(0, static_cast<uint64_t>(newData));
    // Note: old data is discarded.
}

void
IntervalRateCollector::TraceSinkUinteger16(uint16_t oldData, uint16_t newData)
{
    TraceSinkUinteger64(0, static_cast<uint64_t>(newData));
    // Note: old data is discarded.
}

void
IntervalRateCollector::TraceSinkUinteger32(uint32_t oldData, uint32_t newData)
{
    TraceSinkUinteger64(0, static_cast<uint64_t>(newData));
    // Note: old data is discarded.
}

void
IntervalRateCollector::TraceSinkUinteger64(uint64_t oldData, uint64_t newData)
{
    NS_LOG_FUNCTION(this << GetName() << oldData << newData);

    if (IsEnabled())
    {
        if (m_inputDataType == IntervalRateCollector::INPUT_DATA_TYPE_UINTEGER)
        {
            m_intervalSumUinteger += newData;
            m_overallSumUinteger += newData;
            m_intervalNumOfSamples++;
            m_overallNumOfSamples++;
        }
        else
        {
            NS_LOG_WARN(this << " ignoring the incoming sample " << newData
                             << " because of unexpected data type");
        }
    }
}

void
IntervalRateCollector::TraceSinkBoolean(bool oldData, bool newData)
{
    NS_LOG_FUNCTION(this << GetName() << oldData << newData);

    if (IsEnabled())
    {
        if (m_inputDataType == IntervalRateCollector::INPUT_DATA_TYPE_BOOLEAN)
        {
            if (newData)
            {
                m_intervalSumUinteger++;
                m_overallSumUinteger++;
            }

            m_intervalNumOfSamples++;
            m_overallNumOfSamples++;
        }
        else
        {
            NS_LOG_WARN(this << " ignoring the incoming sample " << newData
                             << " because of unexpected data type");
        }
    }
}

} // end of namespace ns3
