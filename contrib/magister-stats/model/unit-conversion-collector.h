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

#ifndef UNIT_CONVERSION_COLLECTOR_H
#define UNIT_CONVERSION_COLLECTOR_H

#include "ns3/data-collection-object.h"
#include "ns3/nstime.h"
#include "ns3/traced-callback.h"
#include "ns3/traced-value.h"

#include <string>

namespace ns3
{

/**
 * @ingroup aggregator
 * @brief Collector which converts input sample data to a different unit.
 *
 * ### Input ###
 * This class provides 9 trace sinks for receiving inputs. Each trace sink
 * is a function with a signature similar to the following:
 * @code
 *   void TraceSinkP (P oldData, P newData);
 * @endcode
 * where `P` is one of the 9 supported data types. This type of signature
 * follows the trace source signature types commonly exported by probes.
 * Although different data types are accepted, they are all internally
 * processed using `double` data type.
 *
 * ### Processing ###
 * This class provides 7 types of unit conversion procedure. It can be selected
 * by calling the SetConversionType() method or setting the `ConversionType`
 * attribute.
 * - `TRANSPARENT` (no conversion at all)
 * - `FROM_BYTES_TO_BIT`
 * - `FROM_BYTES_TO_KBIT`
 * - `FROM_BYTES_TO_MBIT`
 * - `FROM_SECONDS_TO_MS`
 * - `FROM_LINEAR_TO_DB`
 * - `FROM_LINEAR_TO_DBM`
 *
 * ### Output ###
 * This class utilizes 3 trace sources to export the converted data:
 * - `Output`: the converted old data and the converted new data (similar
 *   signature as probe's trace source, hence can be used to export to another
 *   collector).
 * - `OutputValue`: the converted new data.
 * - `OutputTimeValue`: the current simulation time and the converted new data.
 * All of the above information are exported using `double` data type in the
 * unit specified by the selected conversion type. An exception here is in the
 * trace source `OutputTimeValue` where the time information is exported in
 * unit of seconds by default, or as specified otherwise by calling the
 * SetTimeUnit() method or setting the `TimeUnit` attribute.
 */
class UnitConversionCollector : public DataCollectionObject
{
  public:
    /**
     * @enum ConversionType_t
     * @brief Types of unit conversion procedure supported by this class.
     *
     * @note Conversion to dB and dBm units expect strictly positive input value.
     */
    typedef enum
    {
        TRANSPARENT = 0,    ///< No conversion at all (the default).
        FROM_BYTES_TO_BIT,  ///< Input in bytes and output in bits.
        FROM_BYTES_TO_KBIT, ///< Input in bytes and output in kilobits.
        FROM_BYTES_TO_MBIT, ///< Input in bytes and output in megabits.
        FROM_SECONDS_TO_MS, ///< Input in seconds and output in milliseconds.
        FROM_LINEAR_TO_DB,  ///< Input in linear unit and output in decibel (dB).
        FROM_LINEAR_TO_DBM  ///< Input in linear unit and output in decibel (dBm).
    } ConversionType_t;

    /**
     * @param conversionType an arbitrary type of unit conversion procedure.
     * @return representation of the type in string.
     */
    static std::string GetConversionTypeName(ConversionType_t conversionType);

    /// Creates a new collector instance.
    UnitConversionCollector();

    // inherited from ObjectBase base class
    static TypeId GetTypeId();

    // ATTRIBUTE SETTERS AND GETTERS ////////////////////////////////////////////

    /**
     * @param conversionType the type of unit conversion procedure to be utilized
     *                       by this collector instance to process the incoming
     *                       samples.
     */
    void SetConversionType(ConversionType_t conversionType);

    /**
     * @return the type of unit conversion procedure to be utilized by this
     *         collector instance to process the incoming samples.
     */
    ConversionType_t GetConversionType() const;

    /**
     * @param unit the unit used for the time output.
     */
    void SetTimeUnit(Time::Unit unit);

    /**
     * @return the unit used for the time output.
     */
    Time::Unit GetTimeUnit() const;

    // TRACE SINKS //////////////////////////////////////////////////////////////

    /**
     * @brief Trace sink for receiving data from `double` valued trace sources.
     * @param oldData the original value.
     * @param newData the new value.
     *
     * This method serves as a trace sink to `double` valued trace sources.
     *
     * @warning In order to avoid processing uninitialized values, the `oldData`
     *          argument is regarded as zero at the very first sample.
     */
    void TraceSinkDouble(double oldData, double newData);

    /**
     * @brief Trace sink for receiving data from `int8_t` valued trace sources.
     * @param oldData the original value.
     * @param newData the new value.
     *
     * This method serves as a trace sink to `int8_t` valued trace sources.
     * The data will be converted to double and then simply passed to the
     * TraceSinkDouble() method.
     *
     * @warning In order to avoid processing uninitialized values, the `oldData`
     *          argument is regarded as zero at the very first sample.
     */
    void TraceSinkInteger8(int8_t oldData, int8_t newData);

    /**
     * @brief Trace sink for receiving data from `int16_t` valued trace sources.
     * @param oldData the original value.
     * @param newData the new value.
     *
     * This method serves as a trace sink to `int16_t` valued trace sources.
     * The data will be converted to double and then simply passed to the
     * TraceSinkDouble() method.
     *
     * @warning In order to avoid processing uninitialized values, the `oldData`
     *          argument is regarded as zero at the very first sample.
     */
    void TraceSinkInteger16(int16_t oldData, int16_t newData);

    /**
     * @brief Trace sink for receiving data from `int32_t` valued trace sources.
     * @param oldData the original value.
     * @param newData the new value.
     *
     * This method serves as a trace sink to `int32_t` valued trace sources.
     * The data will be converted to double and then simply passed to the
     * TraceSinkDouble() method.
     *
     * @warning In order to avoid processing uninitialized values, the `oldData`
     *          argument is regarded as zero at the very first sample.
     */
    void TraceSinkInteger32(int32_t oldData, int32_t newData);

    /**
     * @brief Trace sink for receiving data from `int64_t` valued trace sources.
     * @param oldData the original value.
     * @param newData the new value.
     *
     * This method serves as a trace sink to `int64_t` valued trace sources.
     * The data will be converted to double and then simply passed to the
     * TraceSinkDouble() method.
     *
     * @warning In order to avoid processing uninitialized values, the `oldData`
     *          argument is regarded as zero at the very first sample.
     */
    void TraceSinkInteger64(int64_t oldData, int64_t newData);

    /**
     * @brief Trace sink for receiving data from `uint8_t` valued trace sources.
     * @param oldData the original value.
     * @param newData the new value.
     *
     * This method serves as a trace sink to `uint8_t` valued trace sources.
     * The data will be converted to double and then simply passed to the
     * TraceSinkDouble() method.
     *
     * @warning In order to avoid processing uninitialized values, the `oldData`
     *          argument is regarded as zero at the very first sample.
     */
    void TraceSinkUinteger8(uint8_t oldData, uint8_t newData);

    /**
     * @brief Trace sink for receiving data from `uint16_t` valued trace sources.
     * @param oldData the original value.
     * @param newData the new value.
     *
     * This method serves as a trace sink to `uint16_t` valued trace sources.
     * The data will be converted to double and then simply passed to the
     * TraceSinkDouble() method.
     *
     * @warning In order to avoid processing uninitialized values, the `oldData`
     *          argument is regarded as zero at the very first sample.
     */
    void TraceSinkUinteger16(uint16_t oldData, uint16_t newData);

    /**
     * @brief Trace sink for receiving data from `uint32_t` valued trace sources.
     * @param oldData the original value.
     * @param newData the new value.
     *
     * This method serves as a trace sink to `uint32_t` valued trace sources.
     * The data will be converted to double and then simply passed to the
     * TraceSinkDouble() method.
     *
     * @warning In order to avoid processing uninitialized values, the `oldData`
     *          argument is regarded as zero at the very first sample.
     */
    void TraceSinkUinteger32(uint32_t oldData, uint32_t newData);

    /**
     * @brief Trace sink for receiving data from `uint64_t` valued trace sources.
     * @param oldData the original value.
     * @param newData the new value.
     *
     * This method serves as a trace sink to `uint64_t` valued trace sources.
     * The data will be converted to double and then simply passed to the
     * TraceSinkDouble() method.
     *
     * @warning In order to avoid processing uninitialized values, the `oldData`
     *          argument is regarded as zero at the very first sample.
     */
    void TraceSinkUinteger64(uint64_t oldData, uint64_t newData);

  protected:
    // Inherited from Object base class
    virtual void DoDispose();

    /**
     * @internal
     * @param original the new sample data received by trace sink.
     * @return the sample data converted by the selected unit conversion
     *         procedure.
     *
     * The unit conversion procedure can be selected by calling
     * the SetConversionType() method or setting the `ConversionType` attribute.
     */
    virtual double Convert(double original) const;

  private:
    /**
     * @brief Indicate that the next sample would be the first sample received.
     *
     * The first sample of data received from the probe usually contains
     * uninitialized old data values. This fact makes Valgrind unhappy. Because
     * of this, the trace sinks "force initialize" the first sample of this old
     * data value to zero.
     */
    bool m_isFirstSample;

    ConversionType_t m_conversionType; ///< `ConversionType` attribute.
    Time::Unit m_timeUnit;             ///< `TimeUnit` attribute.

    TracedCallback<double, double> m_output;          ///< `Output` trace source.
    TracedCallback<double> m_outputValue;             ///< `OutputValue` trace source.
    TracedCallback<double, double> m_outputTimeValue; ///< `OutputTimeValue` trace source.

}; // end of class UnitConversionCollector

} // end of namespace ns3

#endif /* UNIT_CONVERSION_COLLECTOR_H */
