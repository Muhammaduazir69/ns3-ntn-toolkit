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

/**
 * @file distribution-collector-test-suite.cc
 * @ingroup stats
 * @brief Test cases for DistributionCollector.
 *
 * Usage example:
 * @code
 *    $ ./test.py --suite=distribution-collector
 * @endcode
 */

#include "ns3/core-module.h"
#include "ns3/distribution-collector.h"

#include <list>
#include <sstream>

NS_LOG_COMPONENT_DEFINE("DistributionCollectorTest");

namespace ns3
{

/**
 * @ingroup stats
 *
 * Part of the `distribution-collector` test suite. Creates a instance of
 * DistributionCollector using a specified configuration parameters, feed it
 * with a given set of inputs, and then verify the instance's output with the
 * given expected outputs as reference.
 */
class DistributionCollectorTestCase : public TestCase
{
  public:
    /**
     * @brief Construct a new test case.
     * @param name the test case name, which will be printed on the test report.
     * @param type output that will be produced by the collector.
     * @param minValue lower bound of the collector coverage.
     * @param maxValue upper bound of the collector coverage.
     * @param numOfBins number of internal bins to be used.
     * @param input a string of space-separated real numbers which will be fed
     *              to the collector as input samples.
     * @param expectedOutput a string of pairs of real numbers, with a space
     *                       after every number; each pair represents a single
     *                       output of the collector.
     */
    DistributionCollectorTestCase(std::string name,
                                  DistributionCollector::OutputType_t type,
                                  double minValue,
                                  double maxValue,
                                  uint32_t numOfBins,
                                  std::string input,
                                  std::string expectedOutput);

  private:
    // inherited from TestCase base class
    virtual void DoRun();
    virtual void DoTeardown();

    /// Scheduled to run at +0.001s, pushing all input samples to #m_collector.
    void FeedInput();

    /**
     * @brief Compare the order and values of the output emitted by the `Output`
     *        trace source of #m_collector with the output expected by this test
     *        case (i.e., #m_expectedSample and #m_expectedCount).
     */
    void CollectorOutputCallback(double sample, double count);

    /*
     * The following are trace sink functions for the other outputs of
     * #m_collector. Most of them are currently doing nothing, except for
     * CollectorOutputCountCallback().
     */
    void CollectorOutput5thPercentileCallback(double percentile5th);
    void CollectorOutput25thPercentileCallback(double percentile25th);
    void CollectorOutput50thPercentileCallback(double percentile50th);
    void CollectorOutput75thPercentileCallback(double percentile75th);
    void CollectorOutput95thPercentileCallback(double percentile95th);
    /// Verify the count reported by #m_collector with the number of input samples.
    void CollectorOutputCountCallback(uint32_t count);
    void CollectorOutputSumCallback(double sum);
    void CollectorOutputMinCallback(double min);
    void CollectorOutputMaxCallback(double max);
    void CollectorOutputMeanCallback(double mean);
    void CollectorOutputStddevCallback(double stddev);
    void CollectorOutputVarianceCallback(double variance);
    void CollectorOutputSqrSumCallback(double sqrSum);

    /// `OutputType` attribute for #m_collector.
    DistributionCollector::OutputType_t m_type;
    double m_minValue;    ///< Lower bound of the collector coverage.
    double m_maxValue;    ///< Upper bound of the collector coverage.
    uint32_t m_numOfBins; ///< Number of internal bins to be used.

    /// Input samples for the collector as space-separated numbers.
    std::string m_input;
    uint32_t m_inputSize; /// Number of samples found in #m_input.

    std::string m_expectedOutput;       /// The expected output.
    std::list<double> m_expectedSample; /// The expected output: the sample part.
    std::list<double> m_expectedCount;  /// The expected output: the count part.

    Ptr<DistributionCollector> m_collector; /// The subject of the test.

}; // end of `class DistributionCollectorTestCase`

DistributionCollectorTestCase::DistributionCollectorTestCase(
    std::string name,
    DistributionCollector::OutputType_t type,
    double minValue,
    double maxValue,
    uint32_t numOfBins,
    std::string input,
    std::string expectedOutput)
    : TestCase(name),
      m_type(type),
      m_minValue(minValue),
      m_maxValue(maxValue),
      m_numOfBins(numOfBins),
      m_input(input),
      m_inputSize(0),
      m_expectedOutput(expectedOutput)
{
    NS_LOG_FUNCTION(this << name << DistributionCollector::GetOutputTypeName(type) << minValue
                         << maxValue << numOfBins << input << expectedOutput);
}

void
DistributionCollectorTestCase::DoRun()
{
    NS_LOG_FUNCTION(this << GetName());

    // Convert expectedOutput string to list.
    double sample = 0.0;
    double count = 0;
    std::istringstream iss(m_expectedOutput);
    while (iss.good())
    {
        iss >> sample;
        m_expectedSample.push_back(sample);
        NS_ASSERT(iss.good());
        iss >> count;
        m_expectedCount.push_back(count);
    }
    NS_ASSERT(m_expectedSample.size() == m_expectedCount.size());

    // Create the collector to test.
    m_collector = CreateObject<DistributionCollector>();
    m_collector->SetOutputType(m_type);
    m_collector->SetNumOfBins(m_numOfBins);

    // Manually set the bins' structure.
    m_collector->InitializeBins();
    PointerValue bins;
    m_collector->GetAttribute("Bins", bins);
    bins.Get<AdaptiveBins>()->SettleBins(m_minValue, m_maxValue);

    // Connect the collector's outputs to a callback of this class.
    bool ret = false;
    ret = m_collector->TraceConnectWithoutContext(
        "Output",
        MakeCallback(&DistributionCollectorTestCase::CollectorOutputCallback, this));
    NS_ASSERT(ret);
    ret = m_collector->TraceConnectWithoutContext(
        "Output5thPercentile",
        MakeCallback(&DistributionCollectorTestCase::CollectorOutput5thPercentileCallback, this));
    NS_ASSERT(ret);
    ret = m_collector->TraceConnectWithoutContext(
        "Output25thPercentile",
        MakeCallback(&DistributionCollectorTestCase::CollectorOutput25thPercentileCallback, this));
    NS_ASSERT(ret);
    ret = m_collector->TraceConnectWithoutContext(
        "Output50thPercentile",
        MakeCallback(&DistributionCollectorTestCase::CollectorOutput50thPercentileCallback, this));
    NS_ASSERT(ret);
    ret = m_collector->TraceConnectWithoutContext(
        "Output75thPercentile",
        MakeCallback(&DistributionCollectorTestCase::CollectorOutput75thPercentileCallback, this));
    NS_ASSERT(ret);
    ret = m_collector->TraceConnectWithoutContext(
        "Output95thPercentile",
        MakeCallback(&DistributionCollectorTestCase::CollectorOutput95thPercentileCallback, this));
    NS_ASSERT(ret);
    ret = m_collector->TraceConnectWithoutContext(
        "OutputCount",
        MakeCallback(&DistributionCollectorTestCase::CollectorOutputCountCallback, this));
    NS_ASSERT(ret);
    ret = m_collector->TraceConnectWithoutContext(
        "OutputSum",
        MakeCallback(&DistributionCollectorTestCase::CollectorOutputSumCallback, this));
    NS_ASSERT(ret);
    ret = m_collector->TraceConnectWithoutContext(
        "OutputMin",
        MakeCallback(&DistributionCollectorTestCase::CollectorOutputMinCallback, this));
    NS_ASSERT(ret);
    ret = m_collector->TraceConnectWithoutContext(
        "OutputMax",
        MakeCallback(&DistributionCollectorTestCase::CollectorOutputMaxCallback, this));
    NS_ASSERT(ret);
    ret = m_collector->TraceConnectWithoutContext(
        "OutputMean",
        MakeCallback(&DistributionCollectorTestCase::CollectorOutputMeanCallback, this));
    NS_ASSERT(ret);
    ret = m_collector->TraceConnectWithoutContext(
        "OutputStddev",
        MakeCallback(&DistributionCollectorTestCase::CollectorOutputStddevCallback, this));
    NS_ASSERT(ret);
    ret = m_collector->TraceConnectWithoutContext(
        "OutputVariance",
        MakeCallback(&DistributionCollectorTestCase::CollectorOutputVarianceCallback, this));
    NS_ASSERT(ret);
    ret = m_collector->TraceConnectWithoutContext(
        "OutputSqrSum",
        MakeCallback(&DistributionCollectorTestCase::CollectorOutputSqrSumCallback, this));
    NS_ASSERT(ret);

    // Push inputs into the collector after 1 ms of simulation time.
    Simulator::Schedule(MilliSeconds(1), &DistributionCollectorTestCase::FeedInput, this);

    Simulator::Stop(MilliSeconds(2));
    Simulator::Run();
    Simulator::Destroy();
}

void
DistributionCollectorTestCase::DoTeardown()
{
    NS_LOG_FUNCTION(this << GetName());
    /*
     * We destroy the collector here, earlier than it's supposed to be, in order
     * to flush the output of the collector out.
     */
    m_collector->Dispose();
    m_collector = 0;
}

void
DistributionCollectorTestCase::FeedInput()
{
    NS_LOG_FUNCTION(this << GetName());

    double sample = 0.0;
    std::istringstream iss(m_input);
    while (iss.good())
    {
        iss >> sample;
        Simulator::ScheduleNow(&DistributionCollector::TraceSinkDouble,
                               m_collector,
                               sample,
                               sample);
        m_inputSize++;
    }
}

void
DistributionCollectorTestCase::CollectorOutputCallback(double sample, double count)
{
    NS_LOG_FUNCTION(this << GetName() << sample << count);

    NS_ASSERT(m_expectedSample.size() == m_expectedCount.size());
    NS_TEST_ASSERT_MSG_GT(m_expectedSample.size(), 0, "Received more samples than expected");

    if (m_expectedSample.size() > 0)
    {
        NS_TEST_ASSERT_MSG_EQ_TOL(m_expectedSample.front(),
                                  sample,
                                  0.001,
                                  "Sample values do not match");
        NS_TEST_ASSERT_MSG_EQ_TOL(m_expectedCount.front(),
                                  count,
                                  0.0001,
                                  "Count values for sample " << m_expectedSample.front()
                                                             << " do not match");
        m_expectedSample.pop_front();
        m_expectedCount.pop_front();
    }
}

void
DistributionCollectorTestCase::CollectorOutputCountCallback(uint32_t count)
{
    NS_LOG_FUNCTION(this << GetName() << count);
    NS_TEST_ASSERT_MSG_EQ(count, m_inputSize, "Inconsistent sample size");
}

void
DistributionCollectorTestCase::CollectorOutput5thPercentileCallback(double percentile5th)
{
    NS_LOG_FUNCTION(this << GetName() << percentile5th);
}

void
DistributionCollectorTestCase::CollectorOutput25thPercentileCallback(double percentile25th)
{
    NS_LOG_FUNCTION(this << GetName() << percentile25th);
}

void
DistributionCollectorTestCase::CollectorOutput50thPercentileCallback(double percentile50th)
{
    NS_LOG_FUNCTION(this << GetName() << percentile50th);
}

void
DistributionCollectorTestCase::CollectorOutput75thPercentileCallback(double percentile75th)
{
    NS_LOG_FUNCTION(this << GetName() << percentile75th);
}

void
DistributionCollectorTestCase::CollectorOutput95thPercentileCallback(double percentile95th)
{
    NS_LOG_FUNCTION(this << GetName() << percentile95th);
}

void
DistributionCollectorTestCase::CollectorOutputSumCallback(double sum)
{
    NS_LOG_FUNCTION(this << GetName() << sum);
}

void
DistributionCollectorTestCase::CollectorOutputMinCallback(double min)
{
    NS_LOG_FUNCTION(this << GetName() << min);
}

void
DistributionCollectorTestCase::CollectorOutputMaxCallback(double max)
{
    NS_LOG_FUNCTION(this << GetName() << max);
}

void
DistributionCollectorTestCase::CollectorOutputMeanCallback(double mean)
{
    NS_LOG_FUNCTION(this << GetName() << mean);
}

void
DistributionCollectorTestCase::CollectorOutputStddevCallback(double stddev)
{
    NS_LOG_FUNCTION(this << GetName() << stddev);
}

void
DistributionCollectorTestCase::CollectorOutputVarianceCallback(double variance)
{
    NS_LOG_FUNCTION(this << GetName() << variance);
}

void
DistributionCollectorTestCase::CollectorOutputSqrSumCallback(double sqrSum)
{
    NS_LOG_FUNCTION(this << GetName() << sqrSum);
}

/**
 * @brief Test suite `distribution-collector`, verifying the
 *        DistributionCollector class.
 */
class DistributionCollectorTestSuite : public TestSuite
{
  public:
    DistributionCollectorTestSuite();
};

DistributionCollectorTestSuite::DistributionCollectorTestSuite()
    : TestSuite("distribution-collector", Type::UNIT)
{
    // LogComponentEnable ("DistributionCollectorTest", LOG_LEVEL_ALL);
    // LogComponentEnable ("DistributionCollectorTest", LOG_PREFIX_ALL);
    // LogComponentEnable ("DistributionCollector", LOG_LEVEL_ALL);
    // LogComponentEnable ("DistributionCollector", LOG_PREFIX_ALL);

    AddTestCase(
        new DistributionCollectorTestCase("d-1-histogram",
                                          DistributionCollector::OUTPUT_TYPE_HISTOGRAM,
                                          0.0,
                                          100.0,
                                          10,
                                          "-10 10 30 50 70 90 110",
                                          "5 1 15 1 25 0 35 1 45 0 55 1 65 0 75 1 85 0 95 2"),
        TestCase::Duration::QUICK);

    AddTestCase(new DistributionCollectorTestCase("d-2-histogram",
                                                  DistributionCollector::OUTPUT_TYPE_HISTOGRAM,
                                                  -100.0,
                                                  0.0,
                                                  5,
                                                  "-30 -10 10 30 50 70 90",
                                                  "-90 0 -70 0 -50 0 -30 1 -10 6"),
                TestCase::Duration::QUICK);

    AddTestCase(new DistributionCollectorTestCase("d-3-histogram",
                                                  DistributionCollector::OUTPUT_TYPE_HISTOGRAM,
                                                  0.0,
                                                  9.0,
                                                  5,
                                                  "10 9 8 6 5 4 3 2 1 0",
                                                  "1 2 3 2 5 2 7 1 9 3"),
                TestCase::Duration::QUICK);

    AddTestCase(new DistributionCollectorTestCase("d-3-probability",
                                                  DistributionCollector::OUTPUT_TYPE_PROBABILITY,
                                                  0.0,
                                                  9.0,
                                                  5,
                                                  "10 9 8 6 5 4 3 2 1 0",
                                                  "1 0.2 3 0.2 5 0.2 7 0.1 9 0.3"),
                TestCase::Duration::QUICK);

    AddTestCase(new DistributionCollectorTestCase("d-3-cumulative",
                                                  DistributionCollector::OUTPUT_TYPE_CUMULATIVE,
                                                  0.0,
                                                  9.0,
                                                  5,
                                                  "10 9 8 6 5 4 3 2 1 0",
                                                  "1 0.2 3 0.4 5 0.6 7 0.7 9 1"),
                TestCase::Duration::QUICK);

    AddTestCase(new DistributionCollectorTestCase(
                    "d-4-histogram",
                    DistributionCollector::OUTPUT_TYPE_HISTOGRAM,
                    -100.0,
                    0.0,
                    10,
                    "-33 -32 -31 -77 -76 -75 -74 -73 -72 -71",
                    "-95 0 -85 0 -75 7 -65 0 -55 0 -45 0 -35 3 -25 0 -15 0 -5 0"),
                TestCase::Duration::QUICK);

    AddTestCase(new DistributionCollectorTestCase(
                    "d-4-probability",
                    DistributionCollector::OUTPUT_TYPE_PROBABILITY,
                    -100.0,
                    0.0,
                    10,
                    "-33 -32 -31 -77 -76 -75 -74 -73 -72 -71",
                    "-95 0 -85 0 -75 0.7 -65 0 -55 0 -45 0 -35 0.3 -25 0 -15 0 -5 0"),
                TestCase::Duration::QUICK);

    AddTestCase(new DistributionCollectorTestCase(
                    "d-4-cumulative",
                    DistributionCollector::OUTPUT_TYPE_CUMULATIVE,
                    -100.0,
                    0.0,
                    10,
                    "-33 -32 -31 -77 -76 -75 -74 -73 -72 -71",
                    "-95 0 -85 0 -75 0.7 -65 0.7 -55 0.7 -45 0.7 -35 1 -25 1 -15 1 -5 1"),
                TestCase::Duration::QUICK);

} // end of `DistributionCollectorTestSuite ()`

static DistributionCollectorTestSuite g_distributionCollectorTestSuiteInstance;

} // end of namespace ns3
