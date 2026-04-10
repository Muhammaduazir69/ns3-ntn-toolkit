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

#include "scalar-collector.h"

#include "magister-stats.h"

#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <string>

NS_LOG_COMPONENT_DEFINE("ScalarCollector");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(ScalarCollector);

std::string // static
ScalarCollector::GetInputDataTypeName(ScalarCollector::InputDataType_t inputDataType)
{
    switch (inputDataType)
    {
    case ScalarCollector::INPUT_DATA_TYPE_DOUBLE:
        return "INPUT_DATA_TYPE_DOUBLE";
    case ScalarCollector::INPUT_DATA_TYPE_UINTEGER:
        return "INPUT_DATA_TYPE_UINTEGER";
    case ScalarCollector::INPUT_DATA_TYPE_BOOLEAN:
        return "INPUT_DATA_TYPE_BOOLEAN";
    default:
        return "";
    }
}

std::string // static
ScalarCollector::GetOutputTypeName(ScalarCollector::OutputType_t outputType)
{
    switch (outputType)
    {
    case ScalarCollector::OUTPUT_TYPE_SUM:
        return "OUTPUT_TYPE_SUM";
    case ScalarCollector::OUTPUT_TYPE_NUMBER_OF_SAMPLE:
        return "OUTPUT_TYPE_NUMBER_OF_SAMPLE";
    case ScalarCollector::OUTPUT_TYPE_AVERAGE_PER_SAMPLE:
        return "OUTPUT_TYPE_AVERAGE_PER_SAMPLE";
    case ScalarCollector::OUTPUT_TYPE_AVERAGE_PER_SECOND:
        return "OUTPUT_TYPE_AVERAGE_PER_SECOND";
    default:
        return "";
    }
}

ScalarCollector::ScalarCollector()
    : m_sumDouble(0.0),
      m_sumUinteger(0),
      m_numOfSamples(0),
      m_firstSample(MilliSeconds(0)),
      m_lastSample(MilliSeconds(0)),
      m_hasReceivedSample(false),
      m_inputDataType(ScalarCollector::INPUT_DATA_TYPE_DOUBLE),
      m_outputType(ScalarCollector::OUTPUT_TYPE_SUM)

{
    NS_LOG_FUNCTION(this << GetName());
}

TypeId // static
ScalarCollector::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ScalarCollector")
            .SetParent<DataCollectionObject>()
            .AddConstructor<ScalarCollector>()
            .AddAttribute("InputDataType",
                          "The data type accepted as inputs. "
                          "The value INPUT_DATA_TYPE_DOUBLE (the default) will "
                          "activate the TraceSinkDouble() method. "
                          "The value INPUT_DATA_TYPE_UINTEGER will activate the "
                          "TraceSinkUinteger8(), TraceSinkUinteger16(), "
                          "TraceSinkUinteger32(), and TraceSinkUinteger64() methods. "
                          "Finally, the value `INPUT_DATA_TYPE_BOOLEAN` will activate"
                          "the TraceSinkBoolean() method. "
                          "The separation of input data type is useful for preserving "
                          "accuracy (e.g., Uinteger has better accuracy at handling "
                          "packet sizes, but has the risk of overflow). In spite of "
                          "this separation, output data type from trace sources are "
                          "still fixed to double in any case.",
                          EnumValue(ScalarCollector::INPUT_DATA_TYPE_DOUBLE),
                          MakeEnumAccessor<ScalarCollector::InputDataType_t>(
                              &ScalarCollector::SetInputDataType,
                              &ScalarCollector::GetInputDataType),
                          MakeEnumChecker(ScalarCollector::INPUT_DATA_TYPE_DOUBLE,
                                          "DOUBLE",
                                          ScalarCollector::INPUT_DATA_TYPE_UINTEGER,
                                          "UINTEGER",
                                          ScalarCollector::INPUT_DATA_TYPE_BOOLEAN,
                                          "BOOLEAN"))
            .AddAttribute(
                "OutputType",
                "Determines the mechanism of processing the incoming samples.",
                EnumValue(ScalarCollector::OUTPUT_TYPE_SUM),
                MakeEnumAccessor<ScalarCollector::OutputType_t>(&ScalarCollector::SetOutputType,
                                                                &ScalarCollector::GetOutputType),
                MakeEnumChecker(ScalarCollector::OUTPUT_TYPE_SUM,
                                "SUM",
                                ScalarCollector::OUTPUT_TYPE_NUMBER_OF_SAMPLE,
                                "NUMBER_OF_SAMPLE",
                                ScalarCollector::OUTPUT_TYPE_AVERAGE_PER_SAMPLE,
                                "AVERAGE_PER_SAMPLE",
                                ScalarCollector::OUTPUT_TYPE_AVERAGE_PER_SECOND,
                                "AVERAGE_PER_SECOND"))
            .AddTraceSource("Output",
                            "Single scalar output, fired once the instance is destroyed.",
                            MakeTraceSourceAccessor(&ScalarCollector::m_output),
                            "ns3::CollectorOutputCallback");
    return tid;
}

void
ScalarCollector::DoDispose()
{
    NS_LOG_FUNCTION(this << GetName());

    if (IsEnabled())
    {
        double sum = 0.0;

        switch (m_inputDataType)
        {
        case ScalarCollector::INPUT_DATA_TYPE_DOUBLE:
            sum = m_sumDouble;
            break;

        case ScalarCollector::INPUT_DATA_TYPE_UINTEGER:
        case ScalarCollector::INPUT_DATA_TYPE_BOOLEAN:
            sum = static_cast<double>(m_sumUinteger);
            break;

        default:
            break;
        }

        double output = 0.0;

        switch (m_outputType)
        {
        case ScalarCollector::OUTPUT_TYPE_SUM:
            output = sum;
            break;

        case ScalarCollector::OUTPUT_TYPE_NUMBER_OF_SAMPLE:
            output = m_numOfSamples;
            break;

        case ScalarCollector::OUTPUT_TYPE_AVERAGE_PER_SAMPLE:
            // This may produce -nan if number of sample is zero.
            output = sum / static_cast<double>(m_numOfSamples);
            break;

        case ScalarCollector::OUTPUT_TYPE_AVERAGE_PER_SECOND:
            if (m_hasReceivedSample)
            {
                const Time duration = m_lastSample - m_firstSample;

                if (duration.IsZero())
                {
                    output = 0.0;
                }
                else
                {
                    NS_ASSERT(duration.IsStrictlyPositive());
                    output = sum / duration.GetSeconds();
                }
            }
            break;

        default:
            break;
        }

        m_output(output);

    } // end of if (IsEnabled ())

} // end of void DoDispose ();

void
ScalarCollector::SetInputDataType(ScalarCollector::InputDataType_t inputDataType)
{
    NS_LOG_FUNCTION(this << GetName() << GetInputDataTypeName(inputDataType));
    m_inputDataType = inputDataType;
}

ScalarCollector::InputDataType_t
ScalarCollector::GetInputDataType() const
{
    return m_inputDataType;
}

void
ScalarCollector::SetOutputType(ScalarCollector::OutputType_t outputType)
{
    NS_LOG_FUNCTION(this << GetName() << GetOutputTypeName(outputType));
    m_outputType = outputType;
}

ScalarCollector::OutputType_t
ScalarCollector::GetOutputType() const
{
    return m_outputType;
}

void
ScalarCollector::TraceSinkDouble(double oldData, double newData)
{
    // NS_LOG_FUNCTION (this << GetName () << newData);

    if (IsEnabled())
    {
        if (m_inputDataType == ScalarCollector::INPUT_DATA_TYPE_DOUBLE)
        {
            m_sumDouble += newData;
            m_numOfSamples++;
            m_lastSample = Simulator::Now();

            if (!m_hasReceivedSample)
            {
                m_firstSample = Simulator::Now();
                m_hasReceivedSample = true;
                NS_LOG_INFO(this << " first sample at " << m_firstSample.GetSeconds());
            }
        }
        else
        {
            NS_LOG_WARN(this << " ignoring the incoming sample " << newData
                             << " because of unexpected data type");
        }
    }
}

void
ScalarCollector::TraceSinkUinteger8(uint8_t oldData, uint8_t newData)
{
    TraceSinkUinteger64(0, static_cast<uint64_t>(newData));
    // Note: old data is discarded.
}

void
ScalarCollector::TraceSinkUinteger16(uint16_t oldData, uint16_t newData)
{
    TraceSinkUinteger64(0, static_cast<uint64_t>(newData));
    // Note: old data is discarded.
}

void
ScalarCollector::TraceSinkUinteger32(uint32_t oldData, uint32_t newData)
{
    TraceSinkUinteger64(0, static_cast<uint64_t>(newData));
    // Note: old data is discarded.
}

void
ScalarCollector::TraceSinkUinteger64(uint64_t oldData, uint64_t newData)
{
    // NS_LOG_FUNCTION (this << GetName () << newData);

    if (IsEnabled())
    {
        if (m_inputDataType == ScalarCollector::INPUT_DATA_TYPE_UINTEGER)
        {
            m_sumUinteger += newData;
            m_numOfSamples++;
            m_lastSample = Simulator::Now();

            if (!m_hasReceivedSample)
            {
                m_firstSample = Simulator::Now();
                m_hasReceivedSample = true;
                NS_LOG_INFO(this << " first sample at " << m_firstSample.GetSeconds());
            }
        }
        else
        {
            NS_LOG_WARN(this << " ignoring the incoming sample " << newData
                             << " because of unexpected data type");
        }
    }
}

void
ScalarCollector::TraceSinkBoolean(bool oldData, bool newData)
{
    // NS_LOG_FUNCTION (this << GetName () << newData);

    if (IsEnabled())
    {
        if (m_inputDataType == ScalarCollector::INPUT_DATA_TYPE_BOOLEAN)
        {
            if (newData)
            {
                m_sumUinteger++;
            }

            m_numOfSamples++;
            m_lastSample = Simulator::Now();

            if (!m_hasReceivedSample)
            {
                m_firstSample = Simulator::Now();
                m_hasReceivedSample = true;
                NS_LOG_INFO(this << " first sample at " << m_firstSample.GetSeconds());
            }
        }
        else
        {
            NS_LOG_WARN(this << " ignoring the incoming sample " << newData
                             << " because of unexpected data type");
        }
    }
}

} // end of namespace ns3
