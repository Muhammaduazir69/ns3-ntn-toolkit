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

#ifndef INTERVAL_RATE_COLLECTOR_H
#define INTERVAL_RATE_COLLECTOR_H

#include "ns3/data-collection-object.h"
#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/traced-callback.h"

#include <string>

namespace ns3
{

/**
 * @ingroup aggregator
 * @brief Collector which partitions the simulation into fixed length time
 *        intervals and produce the sum of input sample data during each
 *        interval as output.
 *
 * ### Input ###
 * This class provides 5 trace sinks for receiving inputs. Each trace sink
 * is a function with a signature similar to the following:
 * @code
 *   void TraceSinkP (P oldData, P newData);
 * @endcode
 * where `P` is one of the 5 supported data types. This type of signature
 * follows the trace source signature types commonly exported by probes. The
 * input data is processed using either `double` (the default) or `uint64_t`
 * data types, depending on the input data type selected by calling the
 * SetInputDataType() method or setting the `InputDataType` attribute.
 *
 * ### Processing ###
 * Upon created, this class instance begins an interval. It lasts for a fixed
 * time duration (one second by default) that can be specified by calling the
 * SetIntervalLength() method or setting the `IntervalLength` attribute. The
 * instance accumulates the received input during the interval into a summed
 * value. Then at the end of each interval, the summed value is emitted as
 * output and reset back to zero. For boolean data type, a `true` value is
 * regarded as 1, while a `false` value is regarded as 0.
 *
 * ### Output ###
 * Samples received are *consolidated* using one of 3 available ways (e.g., sum,
 * count, average). It can be selected by calling the SetOutputType() method or
 * setting the `OutputType` attribute.
 *
 * After that, this class utilizes 3 trace sources to export the output:
 * - `OutputWithoutTime`: the consolidated value from an interval, emitted at
 *   the end of every interval.
 * - `OutputWithTime`: the interval's ending time and its consolidated value,
 *   emitted at the end of every interval.
 * - `OutputOverall`: the consolidated value from all intervals, emitted when
 *   the instance is destroyed.
 * The consolidated values are exported in `double` data type in the same unit
 * as the inputs. The time information is exported in unit of seconds by
 * default, or as specified otherwise by calling the SetTimeUnit() method or
 * setting the `TimeUnit` attribute.
 *
 * In addition, the class also exports the total number of input samples
 * received during the simulation as the `OutputCount` trace source. The same
 * information is also available from the `OutputString` trace source, which
 * has an output similar to the following:
 * @code
 * % output_type: 'OUTPUT_TYPE_SUM'
 * % count: 57
 * % sum: 672.72
 * @endcode
 * These two trace sources are exported at the end of simulation.
 */
class IntervalRateCollector : public DataCollectionObject
{
  public:
    /**
     * @enum InputDataType_t
     * @brief Data types that can serve as inputs for this class.
     */
    typedef enum
    {
        INPUT_DATA_TYPE_DOUBLE = 0, ///< Accepts `double` data type as input.
        INPUT_DATA_TYPE_UINTEGER,   ///< Accepts unsigned integer data types as input.
        INPUT_DATA_TYPE_BOOLEAN     ///< Accepts boolean data type as input.
    } InputDataType_t;

    /**
     * @param inputDataType an arbitrary input data type.
     * @return representation of the input data type in string.
     */
    static std::string GetInputDataTypeName(InputDataType_t inputDataType);

    /**
     * @enum OutputType_t
     * @brief Type of output supported by this class.
     */
    typedef enum
    {
        /**
         * The sum of all the received inputs.
         */
        OUTPUT_TYPE_SUM = 0,
        /**
         * The number of received input samples.
         */
        OUTPUT_TYPE_NUMBER_OF_SAMPLE,
        /**
         * The sum of the received inputs, divided by the number of input samples.
         * Equals to `-nan` if there is no input sample received.
         */
        OUTPUT_TYPE_AVERAGE_PER_SAMPLE
    } OutputType_t;

    /**
     * @param outputType an arbitrary output type.
     * @return representation of the output type in string.
     */
    static std::string GetOutputTypeName(OutputType_t outputType);

    /// Creates a new collector instance.
    IntervalRateCollector();

    // inherited from ObjectBase base class
    static TypeId GetTypeId();

    // ATTRIBUTE SETTERS AND GETTERS ////////////////////////////////////////////

    /**
     * @param intervalLength the length of interval.
     * @warning Updating interval length after the simulation has started may
     *          produce unpredictable behaviour.
     */
    void SetIntervalLength(Time intervalLength);

    /**
     * @return the length of interval.
     */
    Time GetIntervalLength() const;

    /**
     * @param inputDataType the data type accepted as input.
     */
    void SetInputDataType(InputDataType_t inputDataType);

    /**
     * @return the data type accepted as input.
     */
    InputDataType_t GetInputDataType() const;

    /**
     * @param outputType the processing mechanism used by this instance.
     */
    void SetOutputType(OutputType_t outputType);

    /**
     * @return the processing mechanism used by this instance.
     */
    OutputType_t GetOutputType() const;

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
     * This trace sink is only operating when the current input data type is set
     * to `INPUT_DATA_TYPE_DOUBLE`. This can be set by calling the
     * SetInputDataType() method or setting the `InputDataType` attribute.
     */
    void TraceSinkDouble(double oldData, double newData);

    /**
     * @brief Trace sink for receiving data from `uint8_t` valued trace sources.
     * @param oldData the original value.
     * @param newData the new value.
     *
     * This method serves as a trace sink to `uint8_t` valued trace sources.
     * The data will be converted to `uint64_t` and then simply passed to the
     * TraceSinkuint64_t() method.
     *
     * This trace sink is only operating when the current input data type is set
     * to `INPUT_DATA_TYPE_UINTEGER`. This can be set by calling the
     * SetInputDataType() method or setting the `InputDataType` attribute.
     */
    void TraceSinkUinteger8(uint8_t oldData, uint8_t newData);

    /**
     * @brief Trace sink for receiving data from `uint16_t` valued trace sources.
     * @param oldData the original value.
     * @param newData the new value.
     *
     * This method serves as a trace sink to `uint16_t` valued trace sources.
     * The data will be converted to `uint64_t` and then simply passed to the
     * TraceSinkuint64_t() method.
     *
     * This trace sink is only operating when the current input data type is set
     * to `INPUT_DATA_TYPE_UINTEGER`. This can be set by calling the
     * SetInputDataType() method or setting the `InputDataType` attribute.
     */
    void TraceSinkUinteger16(uint16_t oldData, uint16_t newData);

    /**
     * @brief Trace sink for receiving data from `uint32_t` valued trace sources.
     * @param oldData the original value.
     * @param newData the new value.
     *
     * This method serves as a trace sink to `uint32_t` valued trace sources.
     * The data will be converted to `uint64_t` and then simply passed to the
     * TraceSinkuint64_t() method.
     *
     * This trace sink is only operating when the current input data type is set
     * to `INPUT_DATA_TYPE_UINTEGER`. This can be set by calling the
     * SetInputDataType() method or setting the `InputDataType` attribute.
     */
    void TraceSinkUinteger32(uint32_t oldData, uint32_t newData);

    /**
     * @brief Trace sink for receiving data from `uint64_t` valued trace sources.
     * @param oldData the original value.
     * @param newData the new value.
     *
     * This method serves as a trace sink to `uint64_t` valued trace sources.
     *
     * This trace sink is only operating when the current input data type is set
     * to `INPUT_DATA_TYPE_UINTEGER`. This can be set by calling the
     * SetInputDataType() method or setting the `InputDataType` attribute.
     */
    void TraceSinkUinteger64(uint64_t oldData, uint64_t newData);

    /**
     * @brief Trace sink for receiving data from `bool` valued trace sources.
     * @param oldData the original value.
     * @param newData the new value.
     *
     * This method serves as a trace sink to `bool` valued trace sources.
     *
     * This trace sink is only operating when the current input data type is set
     * to `INPUT_DATA_TYPE_BOOLEAN`. This can be set by calling the
     * SetInputDataType() method or setting the `InputDataType` attribute.
     */
    void TraceSinkBoolean(bool oldData, bool newData);

  protected:
    // Inherited from Object base class
    virtual void DoDispose();

  private:
    /**
     * @internal
     * Start the first interval.
     */
    void FirstInterval();

    /**
     * @internal
     * End the current interval, emit outputs related to this interval through
     * trace sources, reset the accumulated values, and finally starts the next
     * interval.
     */
    void NewInterval();

    /// Sum of all `DOUBLE` inputs received during the current interval.
    double m_intervalSumDouble;

    /// Sum of all `DOUBLE` inputs received from all the intervals so far.
    double m_overallSumDouble;

    /// Sum of all `UINTEGER` and `BOOLEAN` inputs received during the current interval.
    uint64_t m_intervalSumUinteger;

    /// Sum of all `UINTEGER` and `BOOLEAN` inputs received from all the intervals so far.
    uint64_t m_overallSumUinteger;

    /// Number of inputs received during the current interval.
    uint32_t m_intervalNumOfSamples;

    /// Number of inputs received from all the intervals so far.
    uint32_t m_overallNumOfSamples;

    /// The end time of the current interval and the start time of the next interval.
    EventId m_nextReset;

    Time m_intervalLength;           ///< `IntervalLength` attribute.
    InputDataType_t m_inputDataType; ///< `InputDataType` attribute.
    OutputType_t m_outputType;       ///< `OutputType` attribute.
    Time::Unit m_timeUnit;           ///< `TimeUnit` attribute.

    TracedCallback<double> m_outputOverall;          ///< `OutputOverall` trace source.
    TracedCallback<double, double> m_outputWithTime; ///< `OutputWithTime` trace source.
    TracedCallback<double> m_outputWithoutTime;      ///< `OutputWithoutTime` trace source.
    TracedCallback<std::string> m_outputString;      ///< `OutputString` trace source.

}; // end of class IntervalRateCollector

} // end of namespace ns3

#endif /* INTERVAL_RATE_COLLECTOR_H */
