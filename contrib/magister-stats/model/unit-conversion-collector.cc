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

#include "unit-conversion-collector.h"

#include "magister-stats.h"

#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <cmath>
#include <string>

NS_LOG_COMPONENT_DEFINE("UnitConversionCollector");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(UnitConversionCollector);

std::string // static
UnitConversionCollector::GetConversionTypeName(
    UnitConversionCollector::ConversionType_t conversionType)
{
    switch (conversionType)
    {
    case UnitConversionCollector::TRANSPARENT:
        return "TRANSPARENT";
    case UnitConversionCollector::FROM_BYTES_TO_BIT:
        return "FROM_BYTES_TO_BIT";
    case UnitConversionCollector::FROM_BYTES_TO_KBIT:
        return "FROM_BYTES_TO_KBIT";
    case UnitConversionCollector::FROM_BYTES_TO_MBIT:
        return "FROM_BYTES_TO_MBIT";
    case UnitConversionCollector::FROM_SECONDS_TO_MS:
        return "FROM_SECONDS_TO_MS";
    case UnitConversionCollector::FROM_LINEAR_TO_DB:
        return "FROM_LINEAR_TO_DB";
    case UnitConversionCollector::FROM_LINEAR_TO_DBM:
        return "FROM_LINEAR_TO_DBM";
    default:
        return "";
    }
}

UnitConversionCollector::UnitConversionCollector()
    : m_isFirstSample(true),
      m_conversionType(UnitConversionCollector::TRANSPARENT),
      m_timeUnit(Time::S)
{
    NS_LOG_FUNCTION(this << GetName());
}

TypeId // static
UnitConversionCollector::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::UnitConversionCollector")
            .SetParent<DataCollectionObject>()
            .AddConstructor<UnitConversionCollector>()
            .AddAttribute("ConversionType",
                          "Determines the unit conversion procedure utilized to "
                          "process the incoming samples.",
                          EnumValue(UnitConversionCollector::TRANSPARENT),
                          MakeEnumAccessor<UnitConversionCollector::ConversionType_t>(
                              &UnitConversionCollector::SetConversionType,
                              &UnitConversionCollector::GetConversionType),
                          MakeEnumChecker(UnitConversionCollector::TRANSPARENT,
                                          "TRANSPARENT",
                                          UnitConversionCollector::FROM_BYTES_TO_BIT,
                                          "FROM_BYTES_TO_BIT",
                                          UnitConversionCollector::FROM_BYTES_TO_KBIT,
                                          "FROM_BYTES_TO_KBIT",
                                          UnitConversionCollector::FROM_BYTES_TO_MBIT,
                                          "FROM_BYTES_TO_MBIT",
                                          UnitConversionCollector::FROM_SECONDS_TO_MS,
                                          "FROM_SECONDS_TO_MS",
                                          UnitConversionCollector::FROM_LINEAR_TO_DB,
                                          "FROM_LINEAR_TO_DB",
                                          UnitConversionCollector::FROM_LINEAR_TO_DBM,
                                          "FROM_LINEAR_TO_DBM"))
            .AddAttribute("TimeUnit",
                          "Determines the unit used for the timed output (i.e., the "
                          "`OutputTimeValue` trace source",
                          EnumValue(Time::S),
                          MakeEnumAccessor<Time::Unit>(&UnitConversionCollector::SetTimeUnit,
                                                       &UnitConversionCollector::GetTimeUnit),
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
            .AddTraceSource("Output",
                            "The result traced value (old and new values) of the "
                            "conversion of an input sample.",
                            MakeTraceSourceAccessor(&UnitConversionCollector::m_output),
                            "ns3::TracedValue::DoubleCallback")
            .AddTraceSource("OutputValue",
                            "The result of the conversion of an input sample.",
                            MakeTraceSourceAccessor(&UnitConversionCollector::m_outputValue),
                            "ns3::CollectorOutputCallback")
            .AddTraceSource("OutputTimeValue",
                            "The current simulation time "
                            "(using unit determined by `TimeUnit` attribute) "
                            "and the result of the conversion of an input sample.",
                            MakeTraceSourceAccessor(&UnitConversionCollector::m_outputTimeValue),
                            "ns3::CollectorTimedOutputCallback");
    return tid;
}

void
UnitConversionCollector::DoDispose()
{
    NS_LOG_FUNCTION(this << GetName());
}

void
UnitConversionCollector::SetConversionType(UnitConversionCollector::ConversionType_t conversionType)
{
    NS_LOG_FUNCTION(this << GetName() << GetConversionTypeName(conversionType));
    m_conversionType = conversionType;
}

UnitConversionCollector::ConversionType_t
UnitConversionCollector::GetConversionType() const
{
    return m_conversionType;
}

void
UnitConversionCollector::SetTimeUnit(Time::Unit unit)
{
    NS_LOG_FUNCTION(this << GetName() << unit);
    m_timeUnit = unit;
}

Time::Unit
UnitConversionCollector::GetTimeUnit() const
{
    return m_timeUnit;
}

void
UnitConversionCollector::TraceSinkDouble(double oldData, double newData)
{
    NS_LOG_FUNCTION(this << GetName() << newData);

    if (m_isFirstSample)
    {
        oldData = 0.0;
        m_isFirstSample = false;
    }

    if (IsEnabled())
    {
        const double convertedOldData = Convert(oldData);
        const double convertedNewData = Convert(newData);
        m_output(convertedOldData, convertedNewData);
        m_outputValue(convertedNewData);
        m_outputTimeValue(Simulator::Now().ToDouble(m_timeUnit), convertedNewData);
    }
}

void
UnitConversionCollector::TraceSinkInteger8(int8_t oldData, int8_t newData)
{
    if (m_isFirstSample)
    {
        oldData = 0;
        m_isFirstSample = false;
    }

    TraceSinkDouble(static_cast<double>(oldData), static_cast<double>(newData));
}

void
UnitConversionCollector::TraceSinkInteger16(int16_t oldData, int16_t newData)
{
    if (m_isFirstSample)
    {
        oldData = 0;
        m_isFirstSample = false;
    }

    TraceSinkDouble(static_cast<double>(oldData), static_cast<double>(newData));
}

void
UnitConversionCollector::TraceSinkInteger32(int32_t oldData, int32_t newData)
{
    if (m_isFirstSample)
    {
        oldData = 0;
        m_isFirstSample = false;
    }

    TraceSinkDouble(static_cast<double>(oldData), static_cast<double>(newData));
}

void
UnitConversionCollector::TraceSinkInteger64(int64_t oldData, int64_t newData)
{
    if (m_isFirstSample)
    {
        oldData = 0;
        m_isFirstSample = false;
    }

    TraceSinkDouble(static_cast<double>(oldData), static_cast<double>(newData));
}

void
UnitConversionCollector::TraceSinkUinteger8(uint8_t oldData, uint8_t newData)
{
    if (m_isFirstSample)
    {
        oldData = 0;
        m_isFirstSample = false;
    }

    TraceSinkDouble(static_cast<double>(oldData), static_cast<double>(newData));
}

void
UnitConversionCollector::TraceSinkUinteger16(uint16_t oldData, uint16_t newData)
{
    if (m_isFirstSample)
    {
        oldData = 0;
        m_isFirstSample = false;
    }

    TraceSinkDouble(static_cast<double>(oldData), static_cast<double>(newData));
}

void
UnitConversionCollector::TraceSinkUinteger32(uint32_t oldData, uint32_t newData)
{
    if (m_isFirstSample)
    {
        oldData = 0;
        m_isFirstSample = false;
    }

    TraceSinkDouble(static_cast<double>(oldData), static_cast<double>(newData));
}

void
UnitConversionCollector::TraceSinkUinteger64(uint64_t oldData, uint64_t newData)
{
    if (m_isFirstSample)
    {
        oldData = 0;
        m_isFirstSample = false;
    }

    TraceSinkDouble(static_cast<double>(oldData), static_cast<double>(newData));
}

double
UnitConversionCollector::Convert(double original) const
{
    switch (m_conversionType)
    {
    case UnitConversionCollector::TRANSPARENT:
        return original;
        break;

    case UnitConversionCollector::FROM_BYTES_TO_BIT:
        return 8.0 * original;
        break;

    case UnitConversionCollector::FROM_BYTES_TO_KBIT:
        return 8.0 * original * 1e-3;
        break;

    case UnitConversionCollector::FROM_BYTES_TO_MBIT:
        return 8.0 * original * 1e-6;
        break;

    case UnitConversionCollector::FROM_SECONDS_TO_MS:
        return 1000.0 * original;
        break;

    case UnitConversionCollector::FROM_LINEAR_TO_DB:
        NS_ASSERT_MSG(original > 0.0,
                      "Error converting non-positive value " << original << " to decibel unit");
        return 10.0 * std::log10(original);
        break;

    case UnitConversionCollector::FROM_LINEAR_TO_DBM:
        NS_ASSERT_MSG(original > 0.0,
                      "Error converting non-positive value " << original << " to decibel unit");
        return 10.0 * std::log10(1000.0 * original);
        break;

    default:
        NS_FATAL_ERROR("UnitConversionCollector - Invalid conversion type");
        break;
    }

    return original;
}

} // end of namespace ns3
