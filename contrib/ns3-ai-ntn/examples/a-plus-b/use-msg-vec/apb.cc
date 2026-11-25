/*
 * Copyright (c) 2023 Huazhong University of Science and Technology
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
 * Author: Muyuan Shen <muyuan_shen@hust.edu.cn>
 */

#include "../apb-leo-path.h"
#include "apb.h"

#include "ns3/core-module.h"
#include "ns3/ntn-dataset-writer.h"

#include <ns3/ns3-ai-ntn-module.h>

#include <iostream>

#define APB_SIZE 3

// NTN version of the a-plus-b bridge test. The C++ side samples a LEO bent pipe (Walker shell,
// SGP4, ntn-constellation) between a terminal in Toulouse and a feeder gateway in Madrid and sends
// the two one-way delays, terminal to satellite (a) and satellite to gateway (b), in microseconds;
// the Python agent returns a + b, the end-to-end one-way delay. Each exchange is an event of a
// simulation run as long as the pass, so the serving satellites and ground ends are recorded as a
// globe scene (ntn-observability). The run checks every answer and writes the delay statistics.

using namespace ns3;

namespace
{

/// One exchange: APB_SIZE consecutive samples of the pass go to the agent in one vector.
void
Exchange(Ns3AiMsgInterfaceImpl<EnvStruct, ActStruct>* msg,
         const std::vector<apbleo::Sample>* samples,
         uint32_t first,
         apbleo::PassStats* stats)
{
    msg->CppSendBegin();
    for (uint32_t j = 0; j < APB_SIZE; ++j)
    {
        const apbleo::Sample& s = samples->at(first + j);
        msg->GetCpp2PyVector()->at(j).env_a = s.upUs;
        msg->GetCpp2PyVector()->at(j).env_b = s.downUs;
    }
    msg->CppSendEnd();
    msg->CppRecvBegin();
    for (uint32_t j = 0; j < APB_SIZE; ++j)
    {
        stats->Add(samples->at(first + j), msg->GetPy2CppVector()->at(j).act_c);
    }
    msg->CppRecvEnd();
}

} // namespace

int
main(int argc, char* argv[])
{
    double altKm = 550.0;
    double inclinationDeg = 53.0;
    double minElevationDeg = 25.0;
    uint32_t steps = 10000;
    double dtS = 0.1;
    CommandLine cmd(__FILE__);
    cmd.AddValue("altKm", "Walker shell altitude (km)", altKm);
    cmd.AddValue("inclinationDeg", "Walker shell inclination (deg)", inclinationDeg);
    cmd.AddValue("minElevationDeg", "minimum elevation at both ground ends (deg)", minElevationDeg);
    cmd.AddValue("steps", "number of exchanges with the Python agent", steps);
    cmd.AddValue("dtS", "time between samples of the pass (s)", dtS);
    cmd.Parse(argc, argv);

    auto interface = Ns3AiMsgInterface::Get();
    interface->SetIsMemoryCreator(false);
    interface->SetUseVector(true);
    interface->SetHandleFinish(true);
    Ns3AiMsgInterfaceImpl<EnvStruct, ActStruct>* msgInterface =
        interface->GetInterface<EnvStruct, ActStruct>();
    // Should run after Python
    NS_ABORT_MSG_IF(msgInterface->GetCpp2PyVector()->size() != APB_SIZE,
                    "the Python side must create a vector of " << APB_SIZE);

    apbleo::LeoPath path;
    path.Build(altKm, inclinationDeg, minElevationDeg);
    const std::vector<apbleo::Sample> samples = path.Plan(steps * APB_SIZE, dtS);
    path.AddSceneNodes();
    ns3::ntnobs::StartSceneFromNodeList(".");

    apbleo::PassStats stats;
    for (uint32_t i = 0; i < steps; ++i)
    {
        Simulator::Schedule(Seconds(i * APB_SIZE * dtS), &Exchange, msgInterface, &samples,
                            i * APB_SIZE, &stats);
    }
    Simulator::Stop(Seconds(steps * APB_SIZE * dtS));
    Simulator::Run();
    stats.Print(path);
    ns3::ntnobs::WriteDatasetRecord(".", "ns3ai_apb_msg_vec", "analytic", argc, argv,
                                    stats.Metrics(path, dtS));
    Simulator::Destroy();
    return stats.GetWrong() == 0 ? 0 : 1;
}
