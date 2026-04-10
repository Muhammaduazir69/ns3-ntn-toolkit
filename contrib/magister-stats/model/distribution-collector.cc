/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014, 2017 Magister Solutions
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
 * Modified: Frans Laakso <frans.laakso@magister.fi>
 *
 */

#include "distribution-collector.h"

#include "magister-stats.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <list>
#include <sstream>
#include <string>

NS_LOG_COMPONENT_DEFINE("DistributionCollector");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(DistributionCollector);

std::string // static
DistributionCollector::GetOutputTypeName(DistributionCollector::OutputType_t outputType)
{
    switch (outputType)
    {
    case DistributionCollector::OUTPUT_TYPE_HISTOGRAM:
        return "OUTPUT_TYPE_HISTOGRAM";
    case DistributionCollector::OUTPUT_TYPE_PROBABILITY:
        return "OUTPUT_TYPE_PROBABILITY";
    case DistributionCollector::OUTPUT_TYPE_CUMULATIVE:
        return "OUTPUT_TYPE_CUMULATIVE";
    default:
        return "";
    }
}

std::string // static
DistributionCollector::GetBinTypeName(DistributionCollector::DistributionBinType_t binType)
{
    switch (binType)
    {
    case DistributionCollector::BIN_TYPE_ADAPTIVE:
        return "BIN_TYPE_ADAPTIVE";
    case DistributionCollector::BIN_TYPE_STATIC:
        return "BIN_TYPE_STATIC";
    default:
        return "";
    }
}

DistributionCollector::DistributionCollector()
    : m_outputType(DistributionCollector::OUTPUT_TYPE_HISTOGRAM),
      m_numOfBins(500),
      m_outOfBoundLimit(0.10),
      m_isInitialized(false),
      m_bins(0),
      m_binType(DistributionCollector::BIN_TYPE_ADAPTIVE),
      m_smallestSettlingSamples(std::numeric_limits<double>::max()),
      m_largestSettlingSamples(-std::numeric_limits<double>::max()),
      m_allowOnlyPositiveValues(false)
{
    NS_LOG_FUNCTION(this << GetName());

    // Delay the initialization, giving the user the chance to set the attributes.
    Simulator::ScheduleNow(&DistributionCollector::InitializeBins, this);
}

TypeId // static
DistributionCollector::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::DistributionCollector")
            .SetParent<DataCollectionObject>()
            .AddConstructor<DistributionCollector>()
            .AddAttribute("Bins",
                          "The bins instance which stores and categorizes samples.",
                          PointerValue(),
                          MakePointerAccessor(&DistributionCollector::m_bins),
                          MakePointerChecker<DistributionBins>())
            .AddAttribute("NumOfBins",
                          "Determine the resolution of the categorization of samples; "
                          "higher values consume more memory but produce smoother "
                          "results.",
                          UintegerValue(500),
                          MakeUintegerAccessor(&DistributionCollector::SetNumOfBins),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("OutputType",
                          "Determines the mechanism of processing the incoming samples.",
                          EnumValue(DistributionCollector::OUTPUT_TYPE_HISTOGRAM),
                          MakeEnumAccessor<DistributionCollector::OutputType_t>(
                              &DistributionCollector::SetOutputType,
                              &DistributionCollector::GetOutputType),
                          MakeEnumChecker(DistributionCollector::OUTPUT_TYPE_HISTOGRAM,
                                          "HISTOGRAM",
                                          DistributionCollector::OUTPUT_TYPE_PROBABILITY,
                                          "PROBABILITY",
                                          DistributionCollector::OUTPUT_TYPE_CUMULATIVE,
                                          "CUMULATIVE"))
            .AddAttribute("DistributionBinType",
                          "Determine the distribution bin type.",
                          EnumValue(DistributionCollector::BIN_TYPE_ADAPTIVE),
                          MakeEnumAccessor<DistributionCollector::DistributionBinType_t>(
                              &DistributionCollector::SetBinType,
                              &DistributionCollector::GetBinType),
                          MakeEnumChecker(DistributionCollector::BIN_TYPE_ADAPTIVE,
                                          "ADAPTIVE",
                                          DistributionCollector::BIN_TYPE_STATIC,
                                          "STATIC"))
            .AddAttribute("AllowOnlyPositiveValues",
                          "Allow only positive values in the range.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&DistributionCollector::SetAllowOnlyPositiveValues,
                                              &DistributionCollector::GetAllowOnlyPositiveValues),
                          MakeBooleanChecker())
            .AddAttribute("SmallestSettlingValue",
                          "Proportion of the original range to be added to the lower "
                          "bound of the predicted range.",
                          DoubleValue(std::numeric_limits<double>::max()),
                          MakeDoubleAccessor(&DistributionCollector::SetSmallestSettlingValue,
                                             &DistributionCollector::GetSmallestSettlingValue),
                          MakeDoubleChecker<double>())
            .AddAttribute("LargestSettlingValue",
                          "Proportion of the original range to be added to the upper "
                          "bound of the predicted range.",
                          DoubleValue(-std::numeric_limits<double>::max()),
                          MakeDoubleAccessor(&DistributionCollector::SetLargestSettlingValue,
                                             &DistributionCollector::GetLargestSettlingValue),
                          MakeDoubleChecker<double>())
            .AddAttribute("OutOfBoundLimit",
                          "Warning is issued when this amount of samples (relative "
                          "to the total number of samples) lie outside the bins.",
                          DoubleValue(0.10),
                          MakeDoubleAccessor(&DistributionCollector::m_outOfBoundLimit),
                          MakeDoubleChecker<double>(0.0, 1.0))
            // MAIN TRACE SOURCE //////////////////////////////////////////////////////
            .AddTraceSource("Output",
                            "A bin identifier and the value corresponding to that bin. "
                            "Emitted upon the instance's destruction.",
                            MakeTraceSourceAccessor(&DistributionCollector::m_output),
                            "ns3::DistributionCollector::OutputCallback")
            .AddTraceSource("OutputString",
                            "Various setup and statistical information. "
                            "Emitted upon the instance's destruction.",
                            MakeTraceSourceAccessor(&DistributionCollector::m_outputString),
                            "ns3::CollectorInformationCallback")
            // PERCENTILE TRACE SOURCES FOR CUMULATIVE OUTPUT TYPE ////////////////////
            .AddTraceSource("Output5thPercentile",
                            "The 5th percentile of the received samples. "
                            "Only available for cumulative output type. "
                            "Emitted upon the instance's destruction.",
                            MakeTraceSourceAccessor(&DistributionCollector::m_output5thPercentile),
                            "ns3::DistributionCollector::Output5thPercentileCallback")
            .AddTraceSource("Output25thPercentile",
                            "The 25th percentile (first quartile) of the received samples. "
                            "Only available for cumulative output type. "
                            "Emitted upon the instance's destruction.",
                            MakeTraceSourceAccessor(&DistributionCollector::m_output25thPercentile),
                            "ns3::DistributionCollector::Output25thPercentileCallback")
            .AddTraceSource("Output50thPercentile",
                            "The 50th percentile (median) of the received samples. "
                            "Only available for cumulative output type. "
                            "Emitted upon the instance's destruction.",
                            MakeTraceSourceAccessor(&DistributionCollector::m_output50thPercentile),
                            "ns3::DistributionCollector::Output50thPercentileCallback")
            .AddTraceSource("Output75thPercentile",
                            "The 75th percentile (third quartile) of the received samples. "
                            "Only available for cumulative output type. "
                            "Emitted upon the instance's destruction.",
                            MakeTraceSourceAccessor(&DistributionCollector::m_output75thPercentile),
                            "ns3::DistributionCollector::Output75thPercentileCallback")
            .AddTraceSource("Output95thPercentile",
                            "The 95th percentile of the received samples. "
                            "Only available for cumulative output type. "
                            "Emitted upon the instance's destruction.",
                            MakeTraceSourceAccessor(&DistributionCollector::m_output95thPercentile),
                            "ns3::DistributionCollector::Output95thPercentileCallback")
            // OTHER BASIC STATISTICAL INFORMATION TRACE SOURCES //////////////////////
            .AddTraceSource("OutputCount",
                            "The number of received samples. "
                            "Emitted upon the instance's destruction.",
                            MakeTraceSourceAccessor(&DistributionCollector::m_outputCount),
                            "ns3::DistributionCollector::OutputCountCallback")
            .AddTraceSource("OutputSum",
                            "The sum of the received samples. "
                            "Emitted upon the instance's destruction.",
                            MakeTraceSourceAccessor(&DistributionCollector::m_outputSum),
                            "ns3::DistributionCollector::OutputSumCallback")
            .AddTraceSource("OutputMin",
                            "The minimum value from the received samples. "
                            "Emitted upon the instance's destruction.",
                            MakeTraceSourceAccessor(&DistributionCollector::m_outputMin),
                            "ns3::DistributionCollector::OutputMinCallback")
            .AddTraceSource("OutputMax",
                            "The maximum value from the received samples. "
                            "Emitted upon the instance's destruction.",
                            MakeTraceSourceAccessor(&DistributionCollector::m_outputMax),
                            "ns3::DistributionCollector::OutputMaxCallback")
            .AddTraceSource("OutputMean",
                            "The mean of the received samples. "
                            "Emitted upon the instance's destruction.",
                            MakeTraceSourceAccessor(&DistributionCollector::m_outputMean),
                            "ns3::DistributionCollector::OutputMeanCallback")
            .AddTraceSource("OutputStddev",
                            "The standard deviation of the received samples. "
                            "Emitted upon the instance's destruction.",
                            MakeTraceSourceAccessor(&DistributionCollector::m_outputStddev),
                            "ns3::DistributionCollector::OutputStddevCallback")
            .AddTraceSource("OutputVariance",
                            "The variance of the received samples. "
                            "Emitted upon the instance's destruction.",
                            MakeTraceSourceAccessor(&DistributionCollector::m_outputVariance),
                            "ns3::DistributionCollector::OutputVarianceCallback")
            .AddTraceSource("OutputSqrSum",
                            "The sum of squares of the received samples. "
                            "Emitted upon the instance's destruction.",
                            MakeTraceSourceAccessor(&DistributionCollector::m_outputSqrSum),
                            "ns3::DistributionCollector::OutputSqrSumCallback")
            // WARNING TRACE SOURCE ///////////////////////////////////////////////////
            .AddTraceSource("Warning",
                            "Emitted upon encountering a problem with the data "
                            "collection, e.g., lack of variability in the received "
                            "samples, or too many samples fall outside the bins.",
                            MakeTraceSourceAccessor(&DistributionCollector::m_warning),
                            "ns3::DistributionCollector::WarningCallback");
    return tid;
}

void
DistributionCollector::InitializeBins()
{
    NS_LOG_FUNCTION(this << GetName());

    if (!m_isInitialized)
    {
        switch (m_binType)
        {
        case BIN_TYPE_ADAPTIVE: {
            m_bins = CreateObject<AdaptiveBins>(m_numOfBins);
            break;
        }
        case BIN_TYPE_STATIC: {
            m_bins = CreateObject<StaticBins>(m_numOfBins);
            m_bins->SetSmallestSettlingValue(m_smallestSettlingSamples);
            m_bins->SetLargestSettlingValue(m_largestSettlingSamples);
            break;
        }
        default: {
            NS_FATAL_ERROR("Unknown bin type");
        }
        }
        m_bins->SetAllowOnlyPositiveValues(m_allowOnlyPositiveValues);
        m_bins->SetInaccuracyCallback(
            MakeCallback(&DistributionCollector::InaccuracyCallback, this));
        m_isInitialized = true;
    }
}

void
DistributionCollector::DoDispose()
{
    NS_LOG_FUNCTION(this << GetName());

    if (IsEnabled() && m_isInitialized && m_bins->GetNumOfSamples() > 1)
    {
        if (m_bins->IsSettled())
        {
            const double outOfBoundsRatio =
                m_bins->GetNumOfOutOfBounds() / static_cast<double>(m_bins->GetNumOfSamples());
            if (outOfBoundsRatio > m_outOfBoundLimit)
            {
                NS_LOG_WARN(this << " Collector " << GetName() << " has assigned too many samples"
                                 << " (" << m_bins->GetNumOfOutOfBounds() << " out of "
                                 << m_bins->GetNumOfSamples() << ")"
                                 << " outside the collector coverage.");
                m_warning();
            }
        }
        else
        {
            /*
             * Not enough samples received to automatically invoke SettleBins().
             * So we force its invocation here at the end of data collection.
             */
            m_bins->SettleBins();
        }

        // Variables related to cumulative distribution.
        double percentile5 = 0.0;
        double percentile25 = 0.0;
        double percentile50 = 0.0;
        double percentile75 = 0.0;
        double percentile95 = 0.0;

        // Compute output for `Output` trace source.

        switch (m_outputType)
        {
        case DistributionCollector::OUTPUT_TYPE_HISTOGRAM: {
            for (uint32_t i = 0; i < m_bins->GetNumOfBins(); i++)
            {
                m_output(m_bins->GetCenterOfBin(i), static_cast<double>(m_bins->GetCountOfBin(i)));
            }
            break;
        }

        case DistributionCollector::OUTPUT_TYPE_PROBABILITY: {
            const uint32_t n = m_calculator.getCount();

            if (n == 0)
            {
                NS_LOG_WARN(this << " skipping output computation"
                                 << " because of no input samples received");
            }
            else
            {
                double p = 0.0;
                for (uint32_t i = 0; i < m_bins->GetNumOfBins(); i++)
                {
                    p = static_cast<double>(m_bins->GetCountOfBin(i)) / n;
                    m_output(m_bins->GetCenterOfBin(i), p);
                }
            }

            break;
        }

        case DistributionCollector::OUTPUT_TYPE_CUMULATIVE: {
            const uint32_t n = m_calculator.getCount();

            if (n == 0)
            {
                NS_LOG_WARN(this << " skipping output computation"
                                 << " because of no input samples received");
            }
            else
            {
                double p = 0.0;
                double x0 = m_bins->GetMinValue();
                double y0 = 0.0;
                double x2 = 0.0; // will be computed in the loop below
                double y2 = 0.0; // will be computed in the loop below

                for (uint32_t i = 0; i < m_bins->GetNumOfBins(); i++)
                {
                    p = static_cast<double>(m_bins->GetCountOfBin(i)) / n;
                    y2 += p;
                    x2 = m_bins->GetCenterOfBin(i);
                    m_output(x2, y2);

                    if ((y0 < 0.05) && (y2 >= 0.05))
                    {
                        percentile5 = GetInterpolatedX1(x0, y0, 0.05, y2);
                        m_output5thPercentile(percentile5);
                    }

                    if ((y0 < 0.25) && (y2 >= 0.25))
                    {
                        percentile25 = GetInterpolatedX1(x0, y0, 0.25, y2);
                        m_output25thPercentile(percentile25);
                    }

                    if ((y0 < 0.50) && (y2 >= 0.50))
                    {
                        percentile50 = GetInterpolatedX1(x0, y0, 0.50, y2);
                        m_output50thPercentile(percentile50);
                    }

                    if ((y0 < 0.75) && (y2 >= 0.75))
                    {
                        percentile75 = GetInterpolatedX1(x0, y0, 0.75, y2);
                        m_output75thPercentile(percentile75);
                    }

                    if ((y0 < 0.95) && (y2 >= 0.95))
                    {
                        percentile95 = GetInterpolatedX1(x0, y0, 0.95, y2);
                        m_output95thPercentile(percentile95);
                    }

                    // Advance x0 and y0.
                    x0 = x2;
                    y0 = y2;

                } // end of  `for (i = 0; i < m_bins->GetNumOfBins (); i++)`

            } // end of else of `if (n == 0)`

            break;

        } // end of `case DistributionCollector::OUTPUT_TYPE_CUMULATIVE:`

        default:
            break;

        } // end of `switch (m_outputType)`

        // Other trace sources are taken from the online calculator.

        m_outputCount(m_calculator.getCount());
        m_outputSum(m_calculator.getSum());
        m_outputMin(m_calculator.getMin());
        m_outputMax(m_calculator.getMax());
        m_outputMean(m_calculator.getMean());
        m_outputStddev(m_calculator.getStddev());
        m_outputVariance(m_calculator.getVariance());
        m_outputSqrSum(m_calculator.getSqrSum());

        // Compute output for `OutputString` trace source.

        std::ostringstream oss;
        oss << "% min_value: " << m_bins->GetMinValue() << std::endl;
        oss << "% max_value: " << m_bins->GetMaxValue() << std::endl;
        oss << "% bin_length: " << m_bins->GetBinLength() << std::endl;
        oss << "% num_of_bins: " << m_bins->GetNumOfBins() << std::endl;
        oss << "% output_type: '" << GetOutputTypeName(m_outputType) << "'" << std::endl;
        oss << "% count: " << m_calculator.getCount() << std::endl;
        oss << "% sum: " << m_calculator.getSum() << std::endl;
        oss << "% min: " << m_calculator.getMin() << std::endl;
        oss << "% max: " << m_calculator.getMax() << std::endl;
        oss << "% mean: " << m_calculator.getMean() << std::endl;
        oss << "% stddev: " << m_calculator.getStddev() << std::endl;
        oss << "% variance: " << m_calculator.getVariance() << std::endl;
        oss << "% sqr_sum: " << m_calculator.getSqrSum() << std::endl;

        if (m_outputType == DistributionCollector::OUTPUT_TYPE_CUMULATIVE)
        {
            oss << "% percentile_5: " << percentile5 << std::endl;
            oss << "% percentile_25: " << percentile25 << std::endl;
            oss << "% percentile_50: " << percentile50 << std::endl;
            oss << "% percentile_75: " << percentile75 << std::endl;
            oss << "% percentile_95: " << percentile95 << std::endl;
        }

        m_outputString(oss.str());

    } // end of `if (IsEnabled ())`

} // end of `void DoDispose ()`

double
DistributionCollector::GetInterpolatedX1(double x0, double y0, double y1, double y2) const
{
    return x0 + (m_bins->GetBinLength() * (y1 - y0) / (y2 - y0));
}

void
DistributionCollector::InaccuracyCallback(double commonValue)
{
    NS_LOG_FUNCTION(this << commonValue);
    NS_LOG_WARN(this << " Collector " << GetName()
                     << " is unable to predict the sample distribution"
                     << " because each of the received samples" << " (" << m_bins->GetNumOfSamples()
                     << " samples)" << " holds the value " << commonValue);
    m_warning(); // propagate accordingly
}

// ATTRIBUTE SETTERS AND GETTERS //////////////////////////////////////////////

void
DistributionCollector::SetNumOfBins(uint32_t numOfBins)
{
    NS_LOG_FUNCTION(this << GetName() << numOfBins);
    m_numOfBins = numOfBins;
}

uint32_t
DistributionCollector::GetNumOfBins() const
{
    return m_numOfBins;
}

void
DistributionCollector::SetOutputType(OutputType_t outputType)
{
    NS_LOG_FUNCTION(this << GetName() << GetOutputTypeName(outputType));
    m_outputType = outputType;
}

DistributionCollector::OutputType_t
DistributionCollector::GetOutputType() const
{
    return m_outputType;
}

void
DistributionCollector::SetBinType(DistributionBinType_t binType)
{
    NS_LOG_FUNCTION(this << GetName() << GetBinTypeName(binType));
    m_binType = binType;
}

DistributionCollector::DistributionBinType_t
DistributionCollector::GetBinType() const
{
    return m_binType;
}

void
DistributionCollector::SetSmallestSettlingValue(double value)
{
    NS_LOG_FUNCTION(this << value);
    m_smallestSettlingSamples = value;
}

double
DistributionCollector::GetSmallestSettlingValue() const
{
    return m_smallestSettlingSamples;
}

void
DistributionCollector::SetLargestSettlingValue(double value)
{
    NS_LOG_FUNCTION(this << value);
    m_largestSettlingSamples = value;
}

double
DistributionCollector::GetLargestSettlingValue() const
{
    return m_largestSettlingSamples;
}

void
DistributionCollector::SetAllowOnlyPositiveValues(bool value)
{
    NS_LOG_FUNCTION(this << value);
    m_allowOnlyPositiveValues = value;
}

bool
DistributionCollector::GetAllowOnlyPositiveValues() const
{
    return m_allowOnlyPositiveValues;
}

// TRACE SINKS ////////////////////////////////////////////////////////////////

void
DistributionCollector::TraceSinkDouble1(double newData)
{
    // NS_LOG_FUNCTION (this << GetName () << newData);

    if (!m_isInitialized)
    {
        NS_FATAL_ERROR("This collector instance has not been initialized yet.");
    }

    if (IsEnabled())
    {
        m_bins->NewSample(newData);
        m_calculator.Update(newData);
    }
}

void
DistributionCollector::TraceSinkDouble(double oldData, double newData)
{
    TraceSinkDouble1(newData);
    // Note: old data is discarded.
}

void
DistributionCollector::TraceSinkInteger8(int8_t oldData, int8_t newData)
{
    TraceSinkDouble1(static_cast<double>(newData));
    // Note: old data is discarded.
}

void
DistributionCollector::TraceSinkInteger16(int16_t oldData, int16_t newData)
{
    TraceSinkDouble1(static_cast<double>(newData));
    // Note: old data is discarded.
}

void
DistributionCollector::TraceSinkInteger32(int32_t oldData, int32_t newData)
{
    TraceSinkDouble1(static_cast<double>(newData));
    // Note: old data is discarded.
}

void
DistributionCollector::TraceSinkInteger64(int64_t oldData, int64_t newData)
{
    TraceSinkDouble1(static_cast<double>(newData));
    // Note: old data is discarded.
}

void
DistributionCollector::TraceSinkUinteger8(uint8_t oldData, uint8_t newData)
{
    TraceSinkDouble1(static_cast<double>(newData));
    // Note: old data is discarded.
}

void
DistributionCollector::TraceSinkUinteger16(uint16_t oldData, uint16_t newData)
{
    TraceSinkDouble1(static_cast<double>(newData));
    // Note: old data is discarded.
}

void
DistributionCollector::TraceSinkUinteger32(uint32_t oldData, uint32_t newData)
{
    TraceSinkDouble1(static_cast<double>(newData));
    // Note: old data is discarded.
}

void
DistributionCollector::TraceSinkUinteger64(uint64_t oldData, uint64_t newData)
{
    TraceSinkDouble1(static_cast<double>(newData));
    // Note: old data is discarded.
}

// DistributionBins CLASS METHOD DEFINITION ///////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED(DistributionBins);

DistributionBins::DistributionBins()
    : m_lowerOffset(0.0),
      m_upperOffset(0.0),
      m_numOfSettlingSamples(0),
      m_smallestSettlingSamples(std::numeric_limits<double>::max()),
      m_largestSettlingSamples(-std::numeric_limits<double>::max()),
      m_numOfSamples(0),
      m_numOfOutOfBounds(0),
      m_binsMinValue(0.0),
      m_binsMaxValue(0.0),
      m_binLength(0.0),
      m_numOfBins(0),
      m_isSettled(false),
      m_allowOnlyPositiveValues(false),
      m_notifyInaccuracy()
{
    NS_LOG_FUNCTION(this);
    NS_FATAL_ERROR("This constructor should not be called");
}

DistributionBins::DistributionBins(uint32_t numOfBins)
    : m_lowerOffset(0.0),
      m_upperOffset(0.0),
      m_numOfSettlingSamples(0),
      m_smallestSettlingSamples(std::numeric_limits<double>::max()),
      m_largestSettlingSamples(-std::numeric_limits<double>::max()),
      m_numOfSamples(0),
      m_numOfOutOfBounds(0),
      m_binsMinValue(0.0),
      m_binsMaxValue(0.0),
      m_binLength(0.0),
      m_numOfBins(numOfBins),
      m_isSettled(false),
      m_allowOnlyPositiveValues(false),
      m_notifyInaccuracy()
{
    NS_LOG_FUNCTION(this);
}

TypeId // static
DistributionBins::GetTypeId()
{
    static TypeId tid = TypeId("ns3::DistributionBins").SetParent<Object>();
    return tid;
}

uint32_t
DistributionBins::GetNumOfSamples() const
{
    return m_numOfSamples;
}

void
DistributionBins::SettleBins(double minValue, double maxValue)
{
    NS_LOG_FUNCTION(this << minValue << maxValue);
    NS_ASSERT_MSG(!m_isSettled, "This function has been run before.");
    NS_ASSERT(minValue < maxValue);

    // Divide into bins, initialized to zero.
    m_binLength = (maxValue - minValue) / m_numOfBins;
    NS_LOG_DEBUG(this << " bin length=" << m_binLength);
    m_bins.resize(m_numOfBins, 0);
    m_isSettled = true;

    // Copy all the settling samples into the bins.
    for (std::list<double>::const_iterator it = m_settlingSamples.begin();
         it != m_settlingSamples.end();
         ++it)
    {
        NewSample(*it);
    }

    // Clear the settling samples.
    m_settlingSamples.clear();
}

bool
DistributionBins::IsSettled() const
{
    return m_isSettled;
}

double
DistributionBins::GetMinValue() const
{
    NS_ASSERT_MSG(m_isSettled, "More samples are needed before this function is available.");
    return m_binsMinValue;
}

double
DistributionBins::GetMaxValue() const
{
    NS_ASSERT_MSG(m_isSettled, "More samples are needed before this function is available.");
    return m_binsMaxValue;
}

double
DistributionBins::GetBinLength() const
{
    NS_ASSERT_MSG(m_isSettled, "More samples are needed before this function is available.");
    return m_binLength;
}

uint32_t
DistributionBins::GetNumOfBins() const
{
    return m_numOfBins;
}

void
DistributionBins::SetSmallestSettlingValue(double value)
{
    NS_LOG_FUNCTION(this << value);
    m_smallestSettlingSamples = value;
}

double
DistributionBins::GetSmallestSettlingValue() const
{
    return m_smallestSettlingSamples;
}

void
DistributionBins::SetLargestSettlingValue(double value)
{
    NS_LOG_FUNCTION(this << value);
    m_largestSettlingSamples = value;
}

double
DistributionBins::GetLargestSettlingValue() const
{
    return m_largestSettlingSamples;
}

void
DistributionBins::SetAllowOnlyPositiveValues(bool value)
{
    NS_LOG_FUNCTION(this << value);
    m_allowOnlyPositiveValues = value;
}

bool
DistributionBins::GetAllowOnlyPositiveValues() const
{
    return m_allowOnlyPositiveValues;
}

uint32_t
DistributionBins::GetCountOfBin(uint32_t binIndex) const
{
    NS_ASSERT_MSG(m_isSettled, "More samples are needed before this function is available.");
    NS_ASSERT_MSG(binIndex < m_numOfBins, "Out of bound bin index " << binIndex);
    return m_bins[binIndex];
}

double
DistributionBins::GetCenterOfBin(uint32_t binIndex) const
{
    NS_ASSERT_MSG(m_isSettled, "More samples are needed before this function is available.");
    NS_ASSERT_MSG(binIndex < m_numOfBins, "Out of bound bin index " << binIndex);
    const double binStart = m_binsMinValue + (binIndex * m_binLength);
    NS_ASSERT(binStart < m_binsMaxValue);
    const double binCenter = binStart + (m_binLength / 2.0);
    NS_ASSERT(binCenter < m_binsMaxValue);
    return binCenter;
}

uint32_t
DistributionBins::DetermineBin(double sample)
{
    NS_ASSERT_MSG(m_isSettled, "More samples are needed before this function is available.");

    if (sample < m_binsMinValue)
    {
        m_numOfOutOfBounds++;
        return 0; // Sample less than the minimum value goes to the first bin.
    }
    else if (sample < m_binsMaxValue)
    {
        const double binIndex = std::floor((sample - m_binsMinValue) / m_binLength);
        return static_cast<uint32_t>(binIndex);
    }
    else
    {
        // NS_ASSERT (sample >= m_maxValue);
        //  Sample equal or greater than the maximum value goes to the last bin.
        m_numOfOutOfBounds++;
        return m_numOfBins - 1;
    }
}

void
DistributionBins::SetInaccuracyCallback(Callback<void, double> callback)
{
    m_notifyInaccuracy = callback;
}

uint32_t
DistributionBins::GetNumOfOutOfBounds() const
{
    return m_numOfOutOfBounds;
}

void
DistributionBins::DoDispose()
{
    NS_LOG_FUNCTION(this);
}

// ADAPTIVEBINS CLASS METHOD DEFINITION ///////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED(AdaptiveBins);

AdaptiveBins::AdaptiveBins()
{
    NS_LOG_FUNCTION(this);
    NS_FATAL_ERROR("This constructor should not be called");
}

AdaptiveBins::AdaptiveBins(uint32_t numOfBins)
    : DistributionBins(numOfBins)
{
    NS_LOG_FUNCTION(this);
}

TypeId // static
AdaptiveBins::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::AdaptiveBins")
            .SetParent<DistributionBins>()
            .AddConstructor<AdaptiveBins>()
            .AddAttribute("LowerOffset",
                          "Proportion of the original range to be added to the lower "
                          "bound of the predicted range.",
                          DoubleValue(0.05), // i.e., five percents
                          MakeDoubleAccessor(&AdaptiveBins::m_lowerOffset),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("UpperOffset",
                          "Proportion of the original range to be added to the upper "
                          "bound of the predicted range.",
                          DoubleValue(0.05), // i.e., five percents
                          MakeDoubleAccessor(&AdaptiveBins::m_upperOffset),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("SettlingSamples",
                          "The number of samples to receive and store before the "
                          "bins' structure is fixed. A value of zero is considered "
                          "as infinite number of settling samples, i.e., the highest "
                          "possible accuracy in predicting bins' structure, but may "
                          "consume more memory.",
                          UintegerValue(1000),
                          MakeUintegerAccessor(&AdaptiveBins::m_numOfSettlingSamples),
                          MakeUintegerChecker<uint32_t>());
    return tid;
}

void
AdaptiveBins::DoDispose()
{
    NS_LOG_FUNCTION(this);
    DistributionBins::DoDispose();
}

void
AdaptiveBins::DoInitialize()
{
    NS_LOG_FUNCTION(this);
    DistributionBins::DoInitialize();
}

void
AdaptiveBins::SettleBins()
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(!m_isSettled, "This function has been run before.");
    NS_ASSERT_MSG(m_numOfSamples > 1, "More samples are needed before this function is available.");

    // Compute the overall coverage of the bins.
    NS_LOG_DEBUG(this << " settling samples:" << " smallest=" << m_smallestSettlingSamples
                      << " largest=" << m_largestSettlingSamples);
    double originalRange = m_largestSettlingSamples - m_smallestSettlingSamples;

    if (originalRange > 0.0)
    {
        m_binsMinValue = m_smallestSettlingSamples - (m_lowerOffset * originalRange);
        m_binsMaxValue = m_largestSettlingSamples + (m_upperOffset * originalRange);

        if (m_allowOnlyPositiveValues && m_binsMinValue < 0.0)
        {
            m_binsMinValue = 0.0;
        }
    }
    else
    {
        NS_ASSERT(m_smallestSettlingSamples == m_largestSettlingSamples);
        /*
         * All of the received samples are of equal value. It's impossible to
         * define a proper range from these samples, so we fallback to a
         * "default" (but ugly) bin length of 1 and disregarding the offsets.
         * We configure the range so that the sample values are categorized to
         * the center of the range.
         */
        const double halfRange = static_cast<double>(m_numOfBins) / 2.0;
        m_binsMinValue = m_smallestSettlingSamples - halfRange;
        m_binsMaxValue = m_smallestSettlingSamples + halfRange;

        if (m_allowOnlyPositiveValues && m_binsMinValue < 0.0)
        {
            m_binsMinValue = 0.0;
        }

        if (!m_notifyInaccuracy.IsNull())
        {
            m_notifyInaccuracy(m_smallestSettlingSamples);
        }
    }

    SettleBins(m_binsMinValue, m_binsMaxValue);
}

void
AdaptiveBins::NewSample(double newSample)
{
    // NS_LOG_FUNCTION (this << newSample);

    if (m_isSettled)
    {
        const uint32_t binIndex = DetermineBin(newSample);
        NS_ASSERT_MSG(binIndex < m_numOfBins, "Out of bound bin index " << binIndex);
        m_bins[binIndex] += 1;
    }
    else
    {
        // Store the sample as one of the settling samples.
        m_settlingSamples.push_back(newSample);
        m_numOfSamples++;

        if (m_smallestSettlingSamples > newSample)
        {
            m_smallestSettlingSamples = newSample;
        }

        if (m_largestSettlingSamples < newSample)
        {
            m_largestSettlingSamples = newSample;
        }

        if ((m_numOfSamples >= m_numOfSettlingSamples) && (m_numOfSettlingSamples > 1))
        {
            // We have received enough samples. Let's construct the bins.
            NS_LOG_INFO(this << " automatically settling the bins.");
            SettleBins();
        }
    }
}

// STATICBINS CLASS METHOD DEFINITION ///////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED(StaticBins);

StaticBins::StaticBins()
{
    NS_LOG_FUNCTION(this);
    NS_FATAL_ERROR("This constructor should not be called");
}

StaticBins::StaticBins(uint32_t numOfBins)
    : DistributionBins(numOfBins)
{
    NS_LOG_FUNCTION(this);
}

TypeId // static
StaticBins::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::StaticBins")
            .SetParent<DistributionBins>()
            .AddConstructor<StaticBins>()
            .AddAttribute("SmallestSettlingValue",
                          "Proportion of the original range to be added to the lower "
                          "bound of the predicted range.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&StaticBins::m_smallestSettlingSamples),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("LargestSettlingValue",
                          "Proportion of the original range to be added to the upper "
                          "bound of the predicted range.",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&StaticBins::m_largestSettlingSamples),
                          MakeDoubleChecker<double>(0.0));
    return tid;
}

void
StaticBins::DoDispose()
{
    NS_LOG_FUNCTION(this);
    DistributionBins::DoDispose();
}

void
StaticBins::DoInitialize()
{
    NS_LOG_FUNCTION(this);
    DistributionBins::DoInitialize();
}

void
StaticBins::SettleBins()
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(!m_isSettled, "This function has been run before.");
    NS_ASSERT_MSG(m_numOfSamples >= 1,
                  "More samples are needed before this function is available.");

    // Compute the overall coverage of the bins.
    NS_LOG_DEBUG(this << " settling samples:" << " smallest=" << m_smallestSettlingSamples
                      << " largest=" << m_largestSettlingSamples);
    double originalRange = m_largestSettlingSamples - m_smallestSettlingSamples;

    if (originalRange > 0.0)
    {
        m_binsMinValue = m_smallestSettlingSamples;
        m_binsMaxValue = m_largestSettlingSamples;

        if (m_allowOnlyPositiveValues && m_binsMinValue < 0.0)
        {
            m_binsMinValue = 0.0;
        }
    }
    else
    {
        NS_ASSERT(m_smallestSettlingSamples == m_largestSettlingSamples);
        /*
         * All of the received samples are of equal value. It's impossible to
         * define a proper range from these samples, so we fallback to a
         * "default" (but ugly) bin length of 1 and disregarding the offsets.
         * We configure the range so that the sample values are categorized to
         * the center of the range.
         */
        const double halfRange = static_cast<double>(m_numOfBins) / 2.0;
        m_binsMinValue = m_smallestSettlingSamples - halfRange;
        m_binsMaxValue = m_smallestSettlingSamples + halfRange;

        if (m_allowOnlyPositiveValues && m_binsMinValue < 0.0)
        {
            m_binsMinValue = 0.0;
        }

        if (!m_notifyInaccuracy.IsNull())
        {
            m_notifyInaccuracy(m_smallestSettlingSamples);
        }
    }

    SettleBins(m_binsMinValue, m_binsMaxValue);
}

void
StaticBins::NewSample(double newSample)
{
    // NS_LOG_FUNCTION (this << newSample);

    m_numOfSamples++;

    if (m_isSettled)
    {
        const uint32_t binIndex = DetermineBin(newSample);
        NS_ASSERT_MSG(binIndex < m_numOfBins, "Out of bound bin index " << binIndex);
        m_bins[binIndex] += 1;
    }
    else
    {
        // Store the sample as one of the settling samples.
        m_settlingSamples.push_back(newSample);

        // We have received enough samples. Let's construct the bins.
        NS_LOG_INFO(this << " automatically settling the bins.");
        SettleBins();
    }
}

} // end of namespace ns3
