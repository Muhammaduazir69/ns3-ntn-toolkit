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

/**
 * @file
 *
 * @ingroup nrtv
 * @brief Example script for plotting histograms from some of the random
 *        variable distributions used in NRTV traffic model.
 *
 * The script repeatedly draws random samples from the distributions and then
 * plot a histogram for each distribution. By default, 100 000 samples are
 * taken, which can be modified through a command line argument, for example:
 *
 *     $ ./waf --run="nrtv-variables-plot --numOfSamples=1000000"
 *
 * The script generates the following files in the ns-3 project root directory:
 * - `nrtv-slice-size.plt`
 * - `nrtv-slice-encoding-delay.plt`
 *
 * These files are Gnuplot files. Each of these files can be converted to a PNG
 * file, for example by this command:
 *
 *     $ gnuplot nrtv-slice-size.plt
 *
 * which will produce `slice-size.png` file in the same directory. To
 * convert all the Gnuplot files in the directory, the command below can be
 * used:
 *
 *     $ gnuplot *.plt
 *
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/stats-module.h"
#include "ns3/traffic-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrtvVariablesPlot");

int
main(int argc, char* argv[])
{
    uint32_t numOfSamples = 100000;

    // read command line arguments given by the user
    CommandLine cmd;
    cmd.AddValue("numOfSamples",
                 "Number of samples taken from each random number distribution",
                 numOfSamples);
    cmd.Parse(argc, argv);

    Ptr<NrtvVariables> nrtvVariables = CreateObject<NrtvVariables>();
    // nrtvVariables->SetStream (99);

    HistogramPlotHelper::Plot<uint32_t>(MakeCallback(&NrtvVariables::GetNumOfFrames, nrtvVariables),
                                        "nrtv-num-of-frames",
                                        "Histogram of number of frames in NRTV traffic model",
                                        "Number of frames",
                                        numOfSamples,
                                        100, // bin width = 100 frames
                                        static_cast<double>(nrtvVariables->GetNumOfFramesMean()));

    HistogramPlotHelper::Plot<uint32_t>(MakeCallback(&NrtvVariables::GetSliceSize, nrtvVariables),
                                        "nrtv-slice-size",
                                        "Histogram of slice size in NRTV traffic model",
                                        "Slice size (in bytes)",
                                        numOfSamples,
                                        5, // bin width = 5 bytes
                                        nrtvVariables->GetSliceSizeMean(),
                                        nrtvVariables->GetSliceSizeMax());

    HistogramPlotHelper::Plot<uint64_t>(
        MakeCallback(&NrtvVariables::GetSliceEncodingDelayMilliSeconds, nrtvVariables),
        "nrtv-slice-encoding-delay",
        "Histogram of slice encoding delay in NRTV traffic model",
        "Slice encoding delay (in milliseconds)",
        numOfSamples,
        1, // bin width = 1 ms
        nrtvVariables->GetSliceEncodingDelayMean().GetMilliSeconds(),
        nrtvVariables->GetSliceEncodingDelayMax().GetMilliSeconds());

    HistogramPlotHelper::Plot<double>(
        MakeCallback(&NrtvVariables::GetIdleTimeSeconds, nrtvVariables),
        "nrtv-idle-time",
        "Histogram of client idle time in NRTV traffic model",
        "Idle time (in seconds)",
        numOfSamples,
        1, // bar width = 1 second
        nrtvVariables->GetIdleTimeMean().GetSeconds());

    return 0;

} // end of `int main (int argc, char *argv[])`
