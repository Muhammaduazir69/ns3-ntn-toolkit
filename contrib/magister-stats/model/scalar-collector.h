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

#ifndef SCALAR_COLLECTOR_H
#define SCALAR_COLLECTOR_H

#include "ns3/data-collection-object.h"
#include "ns3/nstime.h"
#include "ns3/traced-callback.h"

#include <string>

namespace ns3
{

/**
 * @ingroup aggregator
 * @brief Collector which sums all the input data and emits the sum as a single
 *        scalar output value.
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
 * This class sums all the received input values. The operation utilized to sum
 * those values is by default a simple addition operation. Additional operation,
 * such as averaging, may be specified by calling the the SetOutputType() method
 * or setting the `OutputType` attribute. For boolean data type, a `true` value
 * is regarded as 1, while a `false` value is regarded as 0.
 *
 * ### Output ###
 * At the end of the instance's life (e.g., when the simulation ends), the
 * `Output` trace source is fired to export the output. It contains a single
 * value in `double` type carrying the sum accumulated during the simulation.
 */
class ScalarCollector : public DataCollectionObject
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
        OUTPUT_TYPE_AVERAGE_PER_SAMPLE,
        /**
         * The sum of the received inputs, divided by the time difference between
         * the last received input sample and the first received input sample.
         */
        OUTPUT_TYPE_AVERAGE_PER_SECOND
    } OutputType_t;

    /**
     * @param outputType an arbitrary output type.
     * @return representation of the output type in string.
     */
    static std::string GetOutputTypeName(OutputType_t outputType);

    /// Creates a new collector instance.
    ScalarCollector();

    // inherited from ObjectBase base class
    static TypeId GetTypeId();

    // ATTRIBUTE SETTERS AND GETTERS ////////////////////////////////////////////

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
     * TraceSinkUinteger64() method.
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
     * TraceSinkUinteger64() method.
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
     * TraceSinkUinteger64() method.
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
    /// Sum of all `DOUBLE` input samples received.
    double m_sumDouble;

    /// Sum of all `UINTEGER` and `BOOLEAN` input samples received.
    uint64_t m_sumUinteger;

    /// Number of input samples that have been received.
    uint32_t m_numOfSamples;

    /// The time when the first input sample is received.
    Time m_firstSample;

    /// The time when the last input sample is received.
    Time m_lastSample;

    /// True if an input sample has been received.
    bool m_hasReceivedSample;

    InputDataType_t m_inputDataType; ///< `InputDataType` attribute.
    OutputType_t m_outputType;       ///< `OutputType` attribute.
    TracedCallback<double> m_output; ///< `Output` trace source.

}; // end of class ScalarCollector

} // end of namespace ns3

#endif /* SCALAR_COLLECTOR_H */
