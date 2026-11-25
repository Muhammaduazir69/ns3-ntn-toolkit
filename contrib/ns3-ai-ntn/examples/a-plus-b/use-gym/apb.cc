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
 * Author:  Muyuan Shen <muyuan_shen@hust.edu.cn>
 */

#include "../apb-leo-path.h"

#include "ns3/ntn-dataset-writer.h"
#include <ns3/ns3-ai-ntn-module.h>
#include <ns3/core-module.h>

#include <iostream>

// NTN version of the a-plus-b bridge test. The C++ side samples a LEO bent pipe (Walker shell,
// SGP4, ntn-constellation) between a terminal in Toulouse and a feeder gateway in Madrid and sends
// the two one-way delays, terminal to satellite (a) and satellite to gateway (b), in microseconds;
// the Python agent returns a + b, the end-to-end one-way delay. Each exchange is an event of a
// simulation run as long as the pass, so the serving satellites and ground ends are recorded as a
// globe scene (ntn-observability). The run checks every answer and writes the delay statistics.

namespace ns3
{

class ApbEnv : public OpenGymEnv
{
  public:
    ApbEnv();
    ~ApbEnv() override;
    static TypeId GetTypeId();
    void DoDispose() override;

    uint32_t GetAPlusB();

    // OpenGym interfaces:
    Ptr<OpenGymSpace> GetActionSpace() override;
    Ptr<OpenGymSpace> GetObservationSpace() override;
    bool GetGameOver() override;
    Ptr<OpenGymDataContainer> GetObservation() override;
    float GetReward() override;
    std::string GetExtraInfo() override;
    bool ExecuteActions(Ptr<OpenGymDataContainer> action) override;

    uint32_t m_a;
    uint32_t m_b;

  private:
    uint32_t m_sum;
};

ApbEnv::ApbEnv()
{
    SetOpenGymInterface(OpenGymInterface::Get());
}

ApbEnv::~ApbEnv()
{
}

TypeId
ApbEnv::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ApbEnv").SetParent<OpenGymEnv>().SetGroupName("OpenGym");
    return tid;
}

void
ApbEnv::DoDispose()
{
}

uint32_t
ApbEnv::GetAPlusB()
{
    Notify();
    return m_sum;
}

Ptr<OpenGymSpace>
ApbEnv::GetActionSpace()
{
    std::vector<uint32_t> shape = {1};
    std::string dtype = TypeNameGet<uint32_t>();
    Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace>(0, 100000, shape, dtype);
    return box;
}

Ptr<OpenGymSpace>
ApbEnv::GetObservationSpace()
{
    std::vector<uint32_t> shape = {2};
    std::string dtype = TypeNameGet<uint32_t>();
    Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace>(0, 50000, shape, dtype);
    return box;
}

bool
ApbEnv::GetGameOver()
{
    return false;
}

Ptr<OpenGymDataContainer>
ApbEnv::GetObservation()
{
    std::vector<uint32_t> shape = {2};
    Ptr<OpenGymBoxContainer<uint32_t>> box = CreateObject<OpenGymBoxContainer<uint32_t>>(shape);

    box->AddValue(m_a);
    box->AddValue(m_b);

    return box;
}

float
ApbEnv::GetReward()
{
    return 0.0;
}

std::string
ApbEnv::GetExtraInfo()
{
    return "";
}

bool
ApbEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
    Ptr<OpenGymBoxContainer<uint32_t>> box = DynamicCast<OpenGymBoxContainer<uint32_t>>(action);
    m_sum = box->GetValue(0);
    return true;
}

} // namespace ns3

namespace
{

/// One Gym step: one sample of the pass as the observation, the agent's action as the answer.
void
Step(ns3::Ptr<ns3::ApbEnv> apb, const apbleo::Sample* s, apbleo::PassStats* stats)
{
    apb->m_a = s->upUs;
    apb->m_b = s->downUs;
    stats->Add(*s, apb->GetAPlusB());
}

} // namespace

int
main(int argc, char* argv[])
{
    using namespace ns3;

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

    Ptr<ApbEnv> apb = CreateObject<ApbEnv>();

    apbleo::LeoPath path;
    path.Build(altKm, inclinationDeg, minElevationDeg);
    const std::vector<apbleo::Sample> samples = path.Plan(steps, dtS);
    path.AddSceneNodes();
    ns3::ntnobs::StartSceneFromNodeList(".");

    apbleo::PassStats stats;
    for (uint32_t i = 0; i < steps; ++i)
    {
        Simulator::Schedule(Seconds(i * dtS), &Step, apb, &samples[i], &stats);
    }
    Simulator::Stop(Seconds(steps * dtS));
    Simulator::Run();
    apb->NotifySimulationEnd();
    stats.Print(path);
    ns3::ntnobs::WriteDatasetRecord(".", "ns3ai_apb_gym", "analytic", argc, argv,
                                    stats.Metrics(path, dtS));
    Simulator::Destroy();
    return stats.GetWrong() == 0 ? 0 : 1;
}
