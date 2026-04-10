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

#ifndef NRTV_VARIABLES_H
#define NRTV_VARIABLES_H

#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/random-variable-stream.h"

#include <string>

namespace ns3
{

/**
 * @ingroup nrtv
 * @brief Container of various random variables for assisting the generation of
 *        streaming traffic pattern by the Near Real-Time Video (NRTV) traffic
 *        model.
 *
 * The default configuration of some of the underlying random distributions are
 * according to NGMN [1] and WiMAX [2] specifications.
 *
 * The available random values to be retrieved are:
 * - number of frames per video --- truncated LogNormal distribution with mean
 *   of 3000 frames (i.e., 5 minutes of 10 fps video);
 * - frame interval --- constant 100 ms (i.e., 10 fps);
 * - number of slices per frame --- constant 8 slices (packets);
 * - slice size --- truncated Pareto distribution with mean of approximately
 *   82.64 bytes;
 * - slice encoding delay --- truncated Pareto distribution with mean of
 *   approximately 5.31 ms;
 * - client's de-jitter buffer window size --- constant 5 seconds; and
 * - client's idle time --- unbounded exponential distribution with mean of
 *   5 seconds.
 *
 * Most parameters of the random distributions are configurable via attributes
 * and methods of this class.
 *
 * References:
 * [1] NGMN Alliance, "NGMN Radio Access Performance Evaluation Methodology",
 *     v1.0, January 2008.
 * [2] WiMAX Forum, "WiMAX (TM) System Evaluation Methodology", Version 2.1,
 *     July 2008.
 *
 */
class NrtvVariables : public Object
{
  public:
    /**
     * @brief Common signature used by callback to collector's trace source.
     * @param oldState Name of the old state.
     * @param newState Name of the new state.
     */
    typedef void (*StateTransitionCallback)(std::string, std::string);

    /// Create a new instance with default configuration of random distributions.
    NrtvVariables();

    // Inherited from ObjectBase base class
    static TypeId GetTypeId();

    // THE MORE USEFUL METHODS

    /**
     * @brief Get a random length of video (in number of frames) to be sent by an
     *        NRTV server.
     * @return the number of frames in the video
     *
     * Number of frames per video is determined by a truncated log-norma
     * distribution. The default distribution settings produces random integers
     * with a mean of 3000 frames and a standard deviation of 2400 frames, and
     * then truncated to fit between 200 frames and 36000 frames. These default
     * settings can be modified via attributes or class methods.
     */
    uint32_t GetNumOfFrames();

    /**
     * @brief Get a constant length of time between consecutive frames.
     * @return time interval between frames
     *
     * By default, frame interval is a constant value of 100 ms, which is
     * equivalent with a frame rate of 10 frames per second.
     *
     * Frame interval can be modified by setting the `FrameInterval` attribute or
     * by calling the SetFrameInterval() method. This value should apply to both
     * NRTV server/application and NRTV client/sink.
     *
     * Frame interval is also known in the standard specification as the
     * "inter-arrrival time between the beginning of each frame".
     */
    Time GetFrameInterval();

    /**
     * @brief Get a constant number of slices (packets) per frame.
     * @return the number of slices per frame
     *
     * By default, each frame consists of 8 slices, which can be modified by
     * setting the `NumOfSlices` attribute or calling the SetNumOfSlices()
     * method.
     */
    uint16_t GetNumOfSlices();

    /**
     * @brief Get a random integer indicating the size of a slice (in bytes).
     * @return slice size in bytes
     *
     * Slice size is determined by a truncated Pareto distribution. The default
     * distribution settings produces (after truncation) random integers between
     * 40 and 250 bytes, with an actual mean of approximately 82.64 bytes.
     */
    uint32_t GetSliceSize();

    /**
     * @brief Get a random length of delay which is introduced by a hypothetical
     *        video encoder at NRTV server/application before serving each slice.
     * @return the delay for encoding a video slice
     *
     * Slice encoding delay is determined by truncated Pareto distribution. The
     * default distribution settings produces (after truncation) random values
     * between 2.5 ms and 15 ms, with an actual mean of approximately 5.31 ms.
     */
    Time GetSliceEncodingDelay();

    /// Equivalent with GetSliceEncodingDelay(), but only for plotting purpose.
    uint64_t GetSliceEncodingDelayMilliSeconds();

    /**
     * @brief Get a constant length of NRTV client's de-jitter buffer window size.
     * @return length of de-jitter buffer window
     *
     * De-jitter buffer window is used to guarantee a continuous display of video
     * streaming data. Its default size is a constant value of 5 seconds. This
     * value can be modified by setting the `DejitterBufferWindowSize` attribute
     * or by calling the SetDejitterBufferWindowSize() method.
     */
    Time GetDejitterBufferWindowSize();

    /**
     * @brief Get a random length of time which is spent by a hypothetical human
     *        user (NRTV client) to "rest" after a video session (e.g., posting a
     *        comment about the video, selecting the next video) before
     *        transitioning to the next video session.
     * @return time interval between videos
     *
     * Client's idle time is determined by an exponential distribution. The
     * default distribution settings produces random values with a mean of
     * 5 seconds without any maximum bound. The mean can be modified by setting
     * the `IdleTimeMean` attribute or by calling the SetIdleTimeMean() method.
     */
    Time GetIdleTime();

    /**
     * @brief Get a random length of time which is spent between opening the
     *              application and connecting to the server/client.
     * @return time interval between starting application and opening connection.
     */
    Time GetConnectionOpenDelay();

    /// Equivalent with GetIdleTime(), but only for plotting purpose.
    double GetIdleTimeSeconds();

    /**
     * @brief Set a fixed random variable stream number to the random variables
     *        used by this model.
     * @param stream the stream index to use.
     *
     * Different random variable stream number makes random number generators to
     * produce different set of random values, thus may also produce different
     * simulation results. However, two identical simulations which use same
     * stream number should produce identical results as well (the repeatability
     * property of ns-3 simulation).
     */
    void SetStream(int64_t stream);

    // THE REST ARE THE NOT-SO-USEFUL METHODS

    // NUMBER OF FRAMES SETTER METHOD

    void SetNumOfFramesMean(uint32_t mean);
    void SetNumOfFramesStdDev(uint32_t stdDev);
    void SetNumOfFramesMin(uint32_t min);
    void SetNumOfFramesMax(uint32_t max);
    uint32_t GetNumOfFramesMean() const;

    // FRAME INTERVAL SETTER METHOD

    void SetFrameInterval(Time constant);

    // NUMBER OF SLICES SETTER METHOD

    void SetNumOfSlices(uint16_t constant);

    // SLICE SIZE SETTER METHODS

    void SetSliceSizeMax(uint32_t max);
    void SetSliceSizeShape(double shape);
    void SetSliceSizeScale(double scale);
    double GetSliceSizeMean() const;
    uint32_t GetSliceSizeMax() const;

    // SLICE ENCODING DELAY SETTER METHODS

    void SetSliceEncodingDelayMax(Time max);
    void SetSliceEncodingDelayShape(double shape);
    void SetSliceEncodingDelayScale(double scale);
    Time GetSliceEncodingDelayMean() const;
    Time GetSliceEncodingDelayMax() const;

    // CLIENT'S DE-JITTER BUFFER WINDOW SIZE SETTER METHOD

    void SetDejitterBufferWindowSize(Time constant);

    // CLIENT'S IDLE TIME SETTER METHODS

    void SetIdleTimeMean(Time mean);
    Time GetIdleTimeMean() const;

    // (UDP) NUMBER OF VIDEOS STREAMED TO CLIENT

    uint32_t GetNumOfVideos() const;

  private:
    // HELPER METHODS

    // Get a bounded integer from a random variable stream
    uint64_t GetBoundedInteger(Ptr<RandomVariableStream> random, double min, double max);

    // Set scale of a ParetoRandomVariable
    void SetParetoScale(Ptr<ParetoRandomVariable> random, double scale);

    // Refresh Log-normal distribution mu (location) and sigma (scale) according to mean and
    // standard deviation
    void RefreshLogNormalParameters(Ptr<LogNormalRandomVariable> random,
                                    double mean,
                                    double stddev);

    // RANDOM NUMBER VARIABLES

    Ptr<LogNormalRandomVariable> m_numOfFramesRng;
    Ptr<ConstantRandomVariable> m_frameIntervalRng;
    Ptr<ConstantRandomVariable> m_numOfSlicesRng;
    Ptr<ParetoRandomVariable> m_sliceSizeRng;
    Ptr<ParetoRandomVariable> m_sliceEncodingDelayRng;
    Ptr<ConstantRandomVariable> m_dejitterBufferWindowSizeRng;
    Ptr<ExponentialRandomVariable> m_idleTimeRng;
    Ptr<RandomVariableStream> m_numberOfVideosRng;
    Ptr<RandomVariableStream> m_connectionOpenDelayRng;

    // HELPER VARIABLES
    double m_numOfFramesMean;
    double m_numOfFramesStdDev;
    double m_numOfFramesMin;
    double m_numOfFramesMax;

}; // end of `class NrtvVariables`

} // namespace ns3

#endif /* NRTV_VARIABLES_H */
