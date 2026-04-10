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
 */

#ifndef DISTRIBUTION_COLLECTOR_H
#define DISTRIBUTION_COLLECTOR_H

#include "ns3/basic-data-calculators.h"
#include "ns3/callback.h"
#include "ns3/data-collection-object.h"
#include "ns3/traced-callback.h"

#include <list>
#include <string>
#include <vector>

namespace ns3
{

/**
 * @ingroup aggregator
 * @brief Parent class for the bins.
 */
class DistributionBins : public Object
{
  public:
    /**
     * @warning the default constructor should not be used
     */
    DistributionBins();

    /**
     * @brief Create an empty set of bins which will adapt its structure.
     * @param numOfBins a positive number indicating the number of bins.
     */
    DistributionBins(uint32_t numOfBins);

    // inherited from ObjectBase base class
    static TypeId GetTypeId();

    /**
     * @return the number of samples received so far.
     */
    uint32_t GetNumOfSamples() const;

    /**
     * @brief Instruct the class to construct the bins.
     */
    virtual void SettleBins() = 0;

    /**
     * @brief Instruct the class to construct the bins using the given parameters.
     * @param minValue the lower bound of the first bin
     * @param maxValue the upper bound of the last bin
     *
     * @warning Must have not been settled before.
     */
    void SettleBins(double minValue, double maxValue);

    /**
     * @return true if the bins' structure have been fixed.
     */
    bool IsSettled() const;

    /**
     * @return the lower bound of the first bin.
     * @warning Requires IsSettled() to be true.
     */
    double GetMinValue() const;

    /**
     * @return the upper bound of the last bin.
     * @warning Requires IsSettled() to be true.
     */
    double GetMaxValue() const;

    /**
     * @return the length of each bin.
     * @warning Requires IsSettled() to be true.
     */
    double GetBinLength() const;

    /**
     * @return the number of bins maintained in this instance.
     */
    uint32_t GetNumOfBins() const;

    /**
     * @param value the smallest settling value.
     */
    void SetSmallestSettlingValue(double value);

    /**
     * @return the smallest settling value.
     */
    double GetSmallestSettlingValue() const;

    /**
     * @param value the largest settling value.
     */
    void SetLargestSettlingValue(double value);

    /**
     * @return the largest settling value.
     */
    double GetLargestSettlingValue() const;

    /**
     * @param value allow only positive values setting.
     */
    void SetAllowOnlyPositiveValues(bool value);

    /**
     * @return the allow only positive values setting.
     */
    bool GetAllowOnlyPositiveValues() const;

    /**
     * @brief Receive the given sample.
     */
    virtual void NewSample(double newSample) = 0;

    /**
     * @return the current value of the counter of a certain bin.
     * @warning Requires IsSettled() to be true.
     */
    uint32_t GetCountOfBin(uint32_t binIndex) const;

    /**
     * @return the sum of lower bound and upper bound divided by two.
     * @warning Requires IsSettled() to be true.
     */
    double GetCenterOfBin(uint32_t binIndex) const;

    /**
     * @param sample a new sample.
     * @return the bin index where the given sample should belong to.
     * @warning Requires IsSettled() to be true.
     * @note Any calls with sample that fall outside the set of bins will
     *       increase the number of out of bounds.
     */
    uint32_t DetermineBin(double sample);

    /**
     * @param callback a simple function that will be invoked when inaccuracy
     *                 problem is encountered.
     *
     * The problem provoking the callback is described in the following. When the
     * first samples received (i.e., the settling samples) are all having the
     * same value, there is not enough information to predict the proper bin
     * structure. As a consequence, the class is forced to guess the bin
     * structure. It simply assume a bin length of 1 and put the known values at
     * the center.
     */
    void SetInaccuracyCallback(Callback<void, double> callback);

    /**
     * @return the number of samples determined to fall outside the bins.
     */
    uint32_t GetNumOfOutOfBounds() const;

    // Inherited from Object base class
    virtual void DoDispose();

  protected:
    double m_lowerOffset;            ///< `LowerOffset` attribute.
    double m_upperOffset;            ///< `UpperOffset` attribute.
    uint32_t m_numOfSettlingSamples; ///< `NumOfSettlingSamples` attribute.

    std::list<double> m_settlingSamples; ///< Temporary storage.
    double m_smallestSettlingSamples;    ///< Smallest value in the storage.
    double m_largestSettlingSamples;     ///< Largest value in the storage.
    uint32_t m_numOfSamples;             ///< Number of samples received so far.

    /// Number of samples which are determined to fall down outside the bins.
    uint32_t m_numOfOutOfBounds;

    std::vector<uint32_t> m_bins; ///< Internal bins representation.
    double m_binsMinValue;        ///< The lower bound of the first bin.
    double m_binsMaxValue;        ///< The upper bound of the last bin.
    double m_binLength;           ///< The length of each bin.
    uint32_t m_numOfBins;         ///< The number of bins.

    bool m_isSettled;               ///< True after SettleBins().
    bool m_allowOnlyPositiveValues; ///< Allow only positive values when settling the bins.

    /// Pointer to function which is invoked upon encountering inaccuracy problem.
    Callback<void, double> m_notifyInaccuracy;

  private:
}; // end of class DistributionBins

/**
 * @ingroup aggregator
 * @brief Bins which categorize and count samples, with the ability to predict
 *        its structure based on the received samples.
 *
 * After receiving a number of samples (the `NumOfSettlingSamples` attribute,
 * which is 1000 samples by default), the class will automatically analyze the
 * samples' distribution and create a fixed set of equal-length bins. For
 * example, the lowest value of the received samples determines the lower bound
 * of the bins. The `LowerOffset` attribute may be set to extend the lower
 * bound by a certain percentage to anticipate any "unexpected" outliers. The
 * `UpperOffset` attribute has the same function for the upper bound.
 */
class AdaptiveBins : public DistributionBins
{
  public:
    using DistributionBins::SettleBins;

    /**
     * @warning the default constructor should not be used
     */
    AdaptiveBins();

    /**
     * @brief Create an empty set of bins which will adapt its structure.
     * @param numOfBins a positive number indicating the number of bins.
     */
    AdaptiveBins(uint32_t numOfBins);

    // inherited from ObjectBase base class
    static TypeId GetTypeId();

    // Inherited from Object base class
    virtual void DoInitialize(void);
    virtual void DoDispose(void);

    /**
     * @brief Instruct the class to construct the bins, based on the samples
     *        received so far.
     *
     * Bins' structure will be automatically derived from the distribution of the
     * samples received and the offset attributes.
     *
     * @warning Must have not been settled before and must have received at least
     *          one sample.
     */
    virtual void SettleBins();

    /**
     * @brief Receive the given sample, thereby increasing the counter of the bin
     *        associated with the given sample by 1.
     * @param newSample a new sample to be received
     */
    virtual void NewSample(double newSample);

  protected:
  private:
}; // end of class AdaptiveBins

/**
 * @ingroup aggregator
 * @brief Bins which categorize and count samples.
 */
class StaticBins : public DistributionBins
{
  public:
    using DistributionBins::SettleBins;

    /**
     * @warning the default constructor should not be used
     */
    StaticBins();

    /**
     * @brief Create an empty set of bins.
     * @param numOfBins a positive number indicating the number of bins.
     */
    StaticBins(uint32_t numOfBins);

    // inherited from ObjectBase base class
    static TypeId GetTypeId();

    // Inherited from Object base class
    virtual void DoInitialize(void);
    virtual void DoDispose(void);

    /**
     * @brief Instruct the class to construct the bins.
     *
     * @warning Must have not been settled before.
     */
    virtual void SettleBins();

    /**
     * @brief Receive the given sample, thereby increasing the counter of the bin
     *        associated with the given sample by 1.
     * @param newSample a new sample to be received
     */
    virtual void NewSample(double newSample);

  protected:
  private:
}; // end of class StaticBins

/**
 * @ingroup aggregator
 * @brief Collector which computes the value distribution of the input samples.
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
 * This class begins by setting up a set of _bins_. Each bin covers an equal
 * length of input value range which does not overlap with the range of other
 * bins. Each received input sample is categorized into exactly one of these
 * bins. In this case, that bin's counter is increased by one. At the end of
 * the simulation, the bins would represent the distribution information of all
 * the received samples.
 *
 * The setup of the bins can be configured through the `NumOfBins` attribute.
 * The corresponding method SetNumOfBins() can also be used for the same
 * purpose.
 *
 * The class will compute a prediction of range of the bins based on the first
 * 1000 samples received (see ns3::AdaptiveBins). Any subsequent samples which
 * do not fit into the predicted range are handled as follows.
 * - Input values which are less than lower bound of the first bin are
 *   categorized into the first bin.
 * - Input values which are equal or greater than upper bound of the last bin
 *   are categorized into the last bin.
 *
 * ### Output ###
 * At the end of the instance's life (e.g., when the simulation ends), the
 * `Output` trace source is fired, typically several times in a row, to export
 * the output. Each time the trace source is fired, it contains the bin
 * identifier (i.e., the center value of the bin) and the corresponding value
 * of that bin. The bin value is determined by the selected output type, which
 * can be modified by calling the SetOutputType() method or setting the
 * `OutputType` attribute. The burst of output is guaranteed to be in order
 * from the first bin (the lowest identifier) until the last bin.
 *
 * In addition, the class also computes several statistical information and
 * export them as output trace sources.
 * - `OutputCount`
 * - `OutputSum`
 * - `OutputMin`
 * - `OutputMax`
 * - `OutputMean`
 * - `OutputStddev`
 * - `OutputVariance`
 * - `OutputSqrSum`
 *
 * Finally, when the OUTPUT_TYPE_CUMULATIVE is selected as the output type, the
 * class also includes percentile information in the following trace sources.
 * - `Output5thPercentile`
 * - `Output25thPercentile`
 * - `Output50thPercentile`
 * - `Output75thPercentile`
 * - `Output95thPercentile`
 * Note that linear interpolation is used to calculate these percentile
 * information, and thus might have some errors.
 *
 * All the additional statistical and percentile trace sources mentioned above
 * are also emitted in string format through the `OutputString` trace source.
 * The resulting string also includes the parameters used to collect the
 * samples. Example `OutputString` output:
 * @code
 * % min_value: 0
 * % max_value: 1
 * % bin_length: 0.02
 * % num_of_bins: 50
 * % output_type: 'OUTPUT_TYPE_CUMULATIVE'
 * % count: 9
 * % sum: 4.40882
 * % min: 0.258985
 * % max: 1.29714
 * % mean: 0.489869
 * % stddev: 0.457671
 * % variance: 0.209463
 * % sqr_sum: 3.83545
 * % percentile_5: 0.2315
 * % percentile_25: 0.2375
 * % percentile_50: 0.245
 * % percentile_75: 0.265
 * % percentile_95: 0.9855
 * @endcode
 */
class DistributionCollector : public DataCollectionObject
{
  public:
    /**
     * @enum OutputType_t
     * @brief Type of output supported by this class.
     */
    typedef enum
    {
        /**
         * Number of samples from each bin is presented as it is (i.e., absolute
         * value).
         */
        OUTPUT_TYPE_HISTOGRAM = 0,
        /**
         * Number of samples from each bin is presented as a value relative to
         * the total number of samples (i.e., ranging between 0.0 and 1.0). Thus,
         * producing a probability distribution function (PDF).
         */
        OUTPUT_TYPE_PROBABILITY,
        /**
         * The values associated with each bin is the sum of number of samples from
         * that bin and all the preceding bins, presented as a value relative to
         * the total number of samples (i.e., ranging between 0.0 and 1.0). Thus,
         * producing a cumulative distribution function (CDF).
         */
        OUTPUT_TYPE_CUMULATIVE
    } OutputType_t;

    /**
     * @enum DistributionBinType_t
     * @brief Type of bins supported by this class.
     */
    typedef enum
    {
        BIN_TYPE_ADAPTIVE = 0,
        BIN_TYPE_STATIC = 1
    } DistributionBinType_t;

    /**
     * @param outputType an arbitrary output type.
     * @return representation of the output type in string.
     */
    static std::string GetOutputTypeName(OutputType_t outputType);

    /**
     * @param binType an arbitrary bin type.
     * @return representation of the bin type in string.
     */
    static std::string GetBinTypeName(DistributionCollector::DistributionBinType_t binType);

    /// Creates a new collector instance.
    DistributionCollector();

    // inherited from ObjectBase base class
    static TypeId GetTypeId();

    /**
     * @brief Create internal bins for categorization purpose.
     *
     * Automatically invoked at the beginning of simulation, but can be safely
     * executed manually too.
     */
    void InitializeBins();

    // ATTRIBUTE SETTERS AND GETTERS ////////////////////////////////////////////

    /**
     * @param numOfBins a positive number indicating the resolution.
     */
    void SetNumOfBins(uint32_t numOfBins);

    /**
     * @return the resolution.
     */
    uint32_t GetNumOfBins() const;

    /**
     * @param outputType the processing mechanism used by this instance.
     */
    void SetOutputType(OutputType_t outputType);

    /**
     * @return the processing mechanism used by this instance.
     */
    OutputType_t GetOutputType() const;

    /**
     * @param binType the bin type used by this instance.
     */
    void SetBinType(DistributionBinType_t binType);

    /**
     * @return the bin type used by this instance.
     */
    DistributionBinType_t GetBinType() const;

    /**
     * @param value the smallest settling value.
     */
    void SetSmallestSettlingValue(double value);

    /**
     * @return the smallest settling value.
     */
    double GetSmallestSettlingValue() const;

    /**
     * @param value the largest settling value.
     */
    void SetLargestSettlingValue(double value);

    /**
     * @return the largest settling value.
     */
    double GetLargestSettlingValue() const;

    /**
     * @param value the allow only positive values setting.
     */
    void SetAllowOnlyPositiveValues(bool value);

    /**
     * @return the allow only positive values setting.
     */
    bool GetAllowOnlyPositiveValues() const;

    // TRACE SINKS //////////////////////////////////////////////////////////////

    /**
     * @brief Trace sink for receiving data from `double` valued trace sources.
     * @param newData the new value.
     *
     * This method serves as a trace sink to `double` valued trace sources.
     */
    void TraceSinkDouble1(double newData);

    /**
     * @brief Trace sink for receiving data from `double` valued trace sources.
     * @param oldData the original value.
     * @param newData the new value.
     *
     * This method serves as a trace sink to `double` valued trace sources.
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
     */
    void TraceSinkUinteger64(uint64_t oldData, uint64_t newData);

    /**
     * Callback signature for `Output` trace source.
     * @param centerOfBin the center value of a distribution bin, identifying the
     *                    bin.
     * @param value the value which corresponds to the bin; the type of the value
     *              is determined by the `OutputType` attribute of the collector.
     */
    typedef void (*OutputCallback)(double centerOfBin, double value);
    /**
     * Callback signature for `Output5thPercentile` trace source.
     * @param value the 5th percentile of samples received by the collector.
     */
    typedef void (*Output5thPercentileCallback)(double value);
    /**
     * Callback signature for `Output25thPercentile` trace source.
     * @param value the 25th percentile (i.e., first quartile) of samples
     *              received by the collector.
     */
    typedef void (*Output25thPercentileCallback)(double value);
    /**
     * Callback signature for `Output50thPercentile` trace source.
     * @param value the 50th percentile (i.e., median) of samples received by
     *              the collector.
     */
    typedef void (*Output50thPercentileCallback)(double value);
    /**
     * Callback signature for `Output75thPercentile` trace source.
     * @param value the 75th percentile (i.e., third quartile) of samples
     *              received by the collector.
     */
    typedef void (*Output75thPercentileCallback)(double value);
    /**
     * Callback signature for `Output95thPercentile` trace source.
     * @param value the 95th percentile of samples received by the collector.
     */
    typedef void (*Output95thPercentileCallback)(double value);
    /**
     * Callback signature for `OutputCount` trace source.
     * @param count the number of samples received by the collector.
     */
    typedef void (*OutputCountCallback)(double count);
    /**
     * Callback signature for `OutputSum` trace source.
     * @param sum the sum of the samples received by the collector.
     */
    typedef void (*OutputSumCallback)(double sum);
    /**
     * Callback signature for `OutputMin` trace source.
     * @param min the minimum value of the samples received by the collector.
     */
    typedef void (*OutputMinCallback)(double min);
    /**
     * Callback signature for `OutputMax` trace source.
     * @param max the maximum value of the samples received by the collector.
     */
    typedef void (*OutputMaxCallback)(double max);
    /**
     * Callback signature for `OutputMean` trace source.
     * @param mean the mean of the samples received by the collector.
     */
    typedef void (*OutputMeanCallback)(double mean);
    /**
     * Callback signature for `OutputStddev` trace source.
     * @param stddev the standard deviation of the samples received by the
     *               collector.
     */
    typedef void (*OutputStddevCallback)(double stddev);
    /**
     * Callback signature for `OutputVariance` trace source.
     * @param variance the variance of the samples received by the collector.
     */
    typedef void (*OutputVarianceCallback)(double variance);
    /**
     * Callback signature for `OutputSqrSum` trace source.
     * @param sqrSum the sum of squares of the samples received by the collector.
     */
    typedef void (*OutputSqrSumCallback)(double sqrSum);
    /**
     * Callback signature for `Warning` trace source.
     */
    typedef void (*WarningCallback)();

  protected:
    // Inherited from Object base class
    virtual void DoDispose();

  private:
    /**
     * @internal
     * @code
     *   Y
     *   ^        + (x2, y2)
     *   |       /
     *   |      + (x1, y1)
     *   |     /
     *   |    /
     *   |   + (x0, y0)
     *   |
     *   +-----------------> X
     * @endcode
     *
     * We assume \f$x_0\f$ and \f$x_2\f$ are the center of two adjacent bins.
     * Thus, it follows that \f$x_2 - x_0\f$ equals to bin length \f$b\f$. The
     * formula for computing \f$x_1\f$ is therefore:
     *
     *    \f[
     *    x_1 = x_0 + (\frac{y_1 - y_0}{y_2 - y_0} \times b)
     *    \f]
     */
    double GetInterpolatedX1(double x0, double y0, double y1, double y2) const;

    /**
     * @brief Receive notification of inaccuracy from the underlying bins.
     * @param commonValue the value which is shared by all the samples received
     *                    by the bins.
     */
    void InaccuracyCallback(double commonValue);

    OutputType_t m_outputType; ///< `OutputType` attribute.
    uint32_t m_numOfBins;      ///< `NumOfBins` attribute.
    double m_outOfBoundLimit;  ///< `OutOfBoundLimit` attribute.
    bool m_isInitialized;      ///< True after InitializeBins().

    TracedCallback<double, double> m_output;    ///< `Output` trace source.
    TracedCallback<std::string> m_outputString; ///< `OutputString` trace source.

    TracedCallback<double> m_output5thPercentile;  ///< `Output5thPercentile` trace source.
    TracedCallback<double> m_output25thPercentile; ///< `Output25thPercentile` trace source.
    TracedCallback<double> m_output50thPercentile; ///< `Output50thPercentile` trace source.
    TracedCallback<double> m_output75thPercentile; ///< `Output75thPercentile` trace source.
    TracedCallback<double> m_output95thPercentile; ///< `Output95thPercentile` trace source.

    TracedCallback<uint32_t> m_outputCount;  ///< `OutputCount` trace source.
    TracedCallback<double> m_outputSum;      ///< `OutputSum` trace source.
    TracedCallback<double> m_outputMin;      ///< `OutputMin` trace source.
    TracedCallback<double> m_outputMax;      ///< `OutputMax` trace source.
    TracedCallback<double> m_outputMean;     ///< `OutputMean` trace source.
    TracedCallback<double> m_outputStddev;   ///< `OutputStddev` trace source.
    TracedCallback<double> m_outputVariance; ///< `OutputVariance` trace source.
    TracedCallback<double> m_outputSqrSum;   ///< `OutputSqrSum` trace source.

    TracedCallback<> m_warning; ///< `Warning` trace source.

    /// Tools for online computing of most of the statistical information.
    MinMaxAvgTotalCalculator<double> m_calculator;

    /// The bin categories.
    Ptr<DistributionBins> m_bins;

    DistributionBinType_t m_binType;  ///< Bin type.
    double m_smallestSettlingSamples; ///< Smallest value in the storage.
    double m_largestSettlingSamples;  ///< Largest value in the storage.
    bool m_allowOnlyPositiveValues;   ///< Allow only positive values.

}; // end of class DistributionCollector

} // end of namespace ns3

#endif /* DISTRIBUTION_COLLECTOR_H */
