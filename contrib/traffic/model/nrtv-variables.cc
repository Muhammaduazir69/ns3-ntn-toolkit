/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 Magister Solutions
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

#include "nrtv-variables.h"

#include "ns3/double.h"
#include "ns3/integer.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/rng-stream.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <cmath>
#include <limits>

NS_LOG_COMPONENT_DEFINE("NrtvVariables");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(NrtvVariables);

NrtvVariables::NrtvVariables()
    : m_numOfFramesRng(CreateObject<LogNormalRandomVariable>()),
      m_frameIntervalRng(CreateObject<ConstantRandomVariable>()),
      m_numOfSlicesRng(CreateObject<ConstantRandomVariable>()),
      m_sliceSizeRng(CreateObject<ParetoRandomVariable>()),
      m_sliceEncodingDelayRng(CreateObject<ParetoRandomVariable>()),
      m_dejitterBufferWindowSizeRng(CreateObject<ConstantRandomVariable>()),
      m_idleTimeRng(CreateObject<ExponentialRandomVariable>()),
      m_numberOfVideosRng(CreateObject<ConstantRandomVariable>()),
      m_connectionOpenDelayRng(CreateObject<UniformRandomVariable>()),
      m_numOfFramesMean(3000),
      m_numOfFramesStdDev(2400),
      m_numOfFramesMin(200),
      m_numOfFramesMax(36000)
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrtvVariables::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrtvVariables")
            .SetParent<Object>()
            .AddConstructor<NrtvVariables>()
            .AddAttribute("Stream",
                          "The stream number for the underlying random number generators stream. "
                          "-1 means \"allocate a stream automatically\".",
                          IntegerValue(-1),
                          MakeIntegerAccessor(&NrtvVariables::SetStream),
                          MakeIntegerChecker<int64_t>())

            // NUMBER OF FRAMES
            .AddAttribute("NumOfFramesMean",
                          "The mean of number of frames per video.",
                          UintegerValue(3000),
                          MakeUintegerAccessor(&NrtvVariables::SetNumOfFramesMean,
                                               &NrtvVariables::GetNumOfFramesMean),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("NumOfFramesStdDev",
                          "The standard deviation of number of frames per video.",
                          UintegerValue(2400),
                          MakeUintegerAccessor(&NrtvVariables::SetNumOfFramesStdDev),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("NumOfFramesMin",
                          "The minimum value of number of frames per video.",
                          UintegerValue(200),
                          MakeUintegerAccessor(&NrtvVariables::SetNumOfFramesMin),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("NumOfFramesMax",
                          "The maximum value of number of frames per video.",
                          UintegerValue(36000),
                          MakeUintegerAccessor(&NrtvVariables::SetNumOfFramesMax),
                          MakeUintegerChecker<uint32_t>())

            // FRAME INTERVAL
            .AddAttribute("FrameInterval",
                          "The constant length of time between frames. The default "
                          "value of 100 ms is equivalent with 10 frames per second",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&NrtvVariables::SetFrameInterval),
                          MakeTimeChecker())

            // NUMBER OF SLICES PER FRAME
            .AddAttribute("NumOfSlices",
                          "The constant number of slices (packets) per frame.",
                          UintegerValue(8),
                          MakeUintegerAccessor(&NrtvVariables::SetNumOfSlices),
                          MakeUintegerChecker<uint16_t>())

            // SLICE SIZE
            .AddAttribute("SliceSizeMax",
                          "The upper bound parameter of Pareto distribution for the "
                          "slice size.",
                          UintegerValue(250),
                          MakeUintegerAccessor(&NrtvVariables::SetSliceSizeMax,
                                               &NrtvVariables::GetSliceSizeMax),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("SliceSizeShape",
                          "The shape parameter of Pareto distribution for the slice size.",
                          DoubleValue(1.2),
                          MakeDoubleAccessor(&NrtvVariables::SetSliceSizeShape),
                          MakeDoubleChecker<double>())
            .AddAttribute("SliceSizeScale",
                          "The scale parameter of Pareto distribution for the slice size.",
                          DoubleValue(40.0),
                          MakeDoubleAccessor(&NrtvVariables::SetSliceSizeScale),
                          MakeDoubleChecker<double>())

            // SLICE ENCODING DELAY
            .AddAttribute("SliceEncodingDelayMax",
                          "The upper bound parameter of Pareto distribution for the "
                          "slice size.",
                          TimeValue(MilliSeconds(15)),
                          MakeTimeAccessor(&NrtvVariables::SetSliceEncodingDelayMax,
                                           &NrtvVariables::GetSliceEncodingDelayMax),
                          MakeTimeChecker())
            .AddAttribute("SliceEncodingDelayShape",
                          "The shape parameter of Pareto distribution for the slice size.",
                          DoubleValue(1.2),
                          MakeDoubleAccessor(&NrtvVariables::SetSliceEncodingDelayShape),
                          MakeDoubleChecker<double>())
            .AddAttribute("SliceEncodingDelayScale",
                          "The scale parameter of Pareto distribution for the slice size.",
                          DoubleValue(2.5),
                          MakeDoubleAccessor(&NrtvVariables::SetSliceEncodingDelayScale),
                          MakeDoubleChecker<double>())

            // DE-JITTER BUFFER WINDOW SIZE
            .AddAttribute("DejitterBufferWindowSize",
                          "The constant length of NRTV client's de-jitter buffer "
                          "window size.",
                          TimeValue(Seconds(5)),
                          MakeTimeAccessor(&NrtvVariables::SetDejitterBufferWindowSize),
                          MakeTimeChecker())

            // IDLE TIME
            .AddAttribute(
                "IdleTimeMean",
                "The mean of client's idle time.",
                TimeValue(Seconds(5)),
                MakeTimeAccessor(&NrtvVariables::SetIdleTimeMean, &NrtvVariables::GetIdleTimeMean),
                MakeTimeChecker())
            .AddAttribute("ConnectionOpeningDelay",
                          "The distribution for the delay between opening starting an application "
                          "and opening a connection (in seconds).",
                          StringValue("ns3::UniformRandomVariable[Min=0.001|Max=0.01]"),
                          MakePointerAccessor(&NrtvVariables::m_connectionOpenDelayRng),
                          MakePointerChecker<RandomVariableStream>())

            // (UDP) NUMBER OF VIDEOS STREAMED
            .AddAttribute("NumberOfVideos",
                          "The distribution for the amount of videos streamed to UDP clients.",
                          StringValue("ns3::ConstantRandomVariable[Constant=1]"),
                          MakePointerAccessor(&NrtvVariables::m_numberOfVideosRng),
                          MakePointerChecker<RandomVariableStream>());
    return tid;

} // end of `TypeId NrtvVariables::GetTypeId ()`

uint32_t
NrtvVariables::GetNumOfFrames()
{
    return GetBoundedInteger(m_numOfFramesRng, m_numOfFramesMin, m_numOfFramesMax);
}

Time
NrtvVariables::GetFrameInterval()
{
    return Seconds(m_frameIntervalRng->GetValue());
}

uint16_t
NrtvVariables::GetNumOfSlices()
{
    return m_numOfSlicesRng->GetInteger();
}

uint32_t
NrtvVariables::GetSliceSize()
{
    return m_sliceSizeRng->GetInteger();
}

Time
NrtvVariables::GetSliceEncodingDelay()
{
    return MilliSeconds(m_sliceEncodingDelayRng->GetInteger());
}

uint64_t
NrtvVariables::GetSliceEncodingDelayMilliSeconds()
{
    return m_sliceEncodingDelayRng->GetInteger();
}

Time
NrtvVariables::GetDejitterBufferWindowSize()
{
    return Seconds(m_dejitterBufferWindowSizeRng->GetValue());
}

Time
NrtvVariables::GetIdleTime()
{
    return Seconds(m_idleTimeRng->GetValue());
}

Time
NrtvVariables::GetConnectionOpenDelay()
{
    return Seconds(m_connectionOpenDelayRng->GetValue());
}

double
NrtvVariables::GetIdleTimeSeconds()
{
    return m_idleTimeRng->GetValue();
}

void
NrtvVariables::SetStream(int64_t stream)
{
    NS_LOG_FUNCTION(this << stream);

    m_numOfFramesRng->SetStream(stream);
    m_frameIntervalRng->SetStream(stream);
    m_numOfSlicesRng->SetStream(stream);
    m_sliceSizeRng->SetStream(stream);
    m_sliceEncodingDelayRng->SetStream(stream);
    m_dejitterBufferWindowSizeRng->SetStream(stream);
    m_idleTimeRng->SetStream(stream);
}

// NUMBER OF FRAMES PER VIDEO ATTRIBUTE SETTER AND GETTER METHODS /////////////

void
NrtvVariables::SetNumOfFramesMean(uint32_t mean)
{
    NS_LOG_FUNCTION(this << mean);
    m_numOfFramesMean = mean;
    RefreshLogNormalParameters(m_numOfFramesRng, m_numOfFramesMean, m_numOfFramesStdDev);
}

void
NrtvVariables::SetNumOfFramesStdDev(uint32_t stdDev)
{
    NS_LOG_FUNCTION(this << stdDev);
    m_numOfFramesStdDev = stdDev;
    RefreshLogNormalParameters(m_numOfFramesRng, m_numOfFramesMean, m_numOfFramesStdDev);
}

void
NrtvVariables::SetNumOfFramesMin(uint32_t min)
{
    NS_LOG_FUNCTION(this << min);
    m_numOfFramesMin = min;
}

void
NrtvVariables::SetNumOfFramesMax(uint32_t max)
{
    NS_LOG_FUNCTION(this << max);
    m_numOfFramesMax = max;
}

uint32_t
NrtvVariables::GetNumOfFramesMean() const
{
    return m_numOfFramesMean;
}

// FRAME INTERVAL ATTRIBUTE SETTER METHODS ////////////////////////////////////

void
NrtvVariables::SetFrameInterval(Time constant)
{
    NS_LOG_FUNCTION(this << constant.GetSeconds());
    m_frameIntervalRng->SetAttribute("Constant", DoubleValue(constant.GetSeconds()));
}

// NUMBER OF SLICES PER FRAME ATTRIBUTE SETTER METHODS ////////////////////////

void
NrtvVariables::SetNumOfSlices(uint16_t constant)
{
    NS_LOG_FUNCTION(this << constant);
    m_numOfSlicesRng->SetAttribute("Constant", DoubleValue(static_cast<double>(constant)));
}

// SLICE SIZE ATTRIBUTE SETTER AND GETTER METHODS /////////////////////////////

void
NrtvVariables::SetSliceSizeMax(uint32_t max)
{
    NS_LOG_FUNCTION(this << max);
    m_sliceSizeRng->SetAttribute("Bound", DoubleValue(static_cast<double>(max)));
}

void
NrtvVariables::SetSliceSizeShape(double shape)
{
    NS_LOG_FUNCTION(this << shape);
    m_sliceSizeRng->SetAttribute("Shape", DoubleValue(shape));
}

void
NrtvVariables::SetSliceSizeScale(double scale)
{
    NS_LOG_FUNCTION(this << scale);
    SetParetoScale(m_sliceSizeRng, scale);
}

double
NrtvVariables::GetSliceSizeMean() const
{
    // extract value from parent class
    double mean = std::numeric_limits<double>::infinity();

    double shape = m_sliceSizeRng->GetShape();
    double scale = m_sliceSizeRng->GetScale();
    if (shape > 1)
    {
        mean = shape * scale / (shape - 1);
    }

    return mean;
}

uint32_t
NrtvVariables::GetSliceSizeMax() const
{
    // extract value from parent class
    return static_cast<uint32_t>(m_sliceSizeRng->GetBound());
}

// SLICE ENCODING DELAY ATTRIBUTE SETTER AND GETTER METHODS ///////////////////

void
NrtvVariables::SetSliceEncodingDelayMax(Time max)
{
    NS_LOG_FUNCTION(this << max.GetSeconds());
    m_sliceEncodingDelayRng->SetAttribute("Bound",
                                          DoubleValue(static_cast<double>(max.GetMilliSeconds())));
}

void
NrtvVariables::SetSliceEncodingDelayShape(double shape)
{
    NS_LOG_FUNCTION(this << shape);

    if (std::abs(shape - 1.0) < 0.000001)
    {
        NS_FATAL_ERROR("Shape parameter of a Pareto distribution must not equal to 1.0"
                       << " (the current value is " << shape << ")");
    }

    m_sliceEncodingDelayRng->SetAttribute("Shape", DoubleValue(shape));
}

void
NrtvVariables::SetSliceEncodingDelayScale(double scale)
{
    NS_LOG_FUNCTION(this << scale);
    SetParetoScale(m_sliceEncodingDelayRng, scale);
}

Time
NrtvVariables::GetSliceEncodingDelayMean() const
{
    // extract value from parent class
    double mean = std::numeric_limits<double>::infinity();

    double shape = m_sliceEncodingDelayRng->GetShape();
    double scale = m_sliceEncodingDelayRng->GetScale();
    if (shape > 1)
    {
        mean = shape * scale / (shape - 1);
    }

    return MilliSeconds(mean);
}

Time
NrtvVariables::GetSliceEncodingDelayMax() const
{
    // extract value from parent class
    return MilliSeconds(m_sliceEncodingDelayRng->GetBound());
}

// DE-JITTER BUFFER WINDOW SIZE ATTRIBUTE SETTER METHODS //////////////////////

void
NrtvVariables::SetDejitterBufferWindowSize(Time constant)
{
    NS_LOG_FUNCTION(this << constant.GetSeconds());
    m_dejitterBufferWindowSizeRng->SetAttribute("Constant", DoubleValue(constant.GetSeconds()));
}

// PARSING TIME ATTRIBUTES SETTER AND GETTER METHODS //////////////////////////

void
NrtvVariables::SetIdleTimeMean(Time mean)
{
    NS_LOG_FUNCTION(this << mean.GetSeconds());
    m_idleTimeRng->SetAttribute("Mean", DoubleValue(mean.GetSeconds()));
}

Time
NrtvVariables::GetIdleTimeMean() const
{
    return Seconds(m_idleTimeRng->GetMean());
}

// GETTING NUMBER OF VIDEOS
uint32_t
NrtvVariables::GetNumOfVideos() const
{
    uint32_t nmbr = m_numberOfVideosRng->GetInteger();
    NS_ASSERT_MSG(nmbr > 0, "Number of videos must be positive!");
    return nmbr;
}

// OTHER HELPER METHODS ///////////////////////////////////////////////////////

uint64_t
NrtvVariables::GetBoundedInteger(Ptr<RandomVariableStream> random, double min, double max)
{
    uint64_t value;
    do
    {
        value = random->GetInteger();
    } while (value < min || value > max);
    return value;
}

void
NrtvVariables::SetParetoScale(Ptr<ParetoRandomVariable> random, double scale)
{
    NS_ASSERT_MSG(scale > 0.0, "Scale parameter must be greater than zero");

    random->SetAttribute("Scale", DoubleValue(scale));
}

void
NrtvVariables::RefreshLogNormalParameters(Ptr<LogNormalRandomVariable> random,
                                          double mean,
                                          double stddev)
{
    NS_LOG_FUNCTION(this);

    const double a1 = pow(stddev, 2);
    const double a2 = pow(mean, 2);
    const double a = log(1 + (a1 / a2));

    const double mu = log(mean) - (0.5 * a);
    const double sigma = sqrt(a);
    NS_LOG_INFO(this << " Mu= " << mu << " Sigma= " << sigma);

    // updating attributes of the log normal
    random->SetAttribute("Mu", DoubleValue(mu));
    random->SetAttribute("Sigma", DoubleValue(sigma));
}

} // namespace ns3
