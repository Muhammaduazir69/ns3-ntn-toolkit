#!/usr/bin/env python3
# Copyright (c) 2026 Muhammad Uzair
# SPDX-License-Identifier: GPL-2.0-only
"""Handover agent for oran-ntn-gym-handover-example (--gym=1) on the current
ns3-ai Gym API (ns3ai_gym_env.Ns3Env(targetName, ns3Path)).

Wave 2 M2 (P0-5). The C++ side (--kpm=measured, the default) observes the
MEASURED radio through the O-RAN KPM producer and actuates the chosen handover
through the RIC on the NR stack (NtnRealStackHelper::TriggerHandover); the
reward is the measured goodput after the action minus the handover cost and the
ping-pong penalty.

Observation (12 floats, OranNtnGymHandover):
  0 serving SINR (dB, PHY trace)            6 best-neighbour TTE (0 = not measured)
  1 serving TTE (0 = not measured)          7 best-neighbour elevation (deg)
  2 serving elevation (deg, SGP4)           8 time on the serving cell (s)
  3 serving Doppler (Hz, SGP4)              9 recent HO count
  4 SINR slope (dB per step)               10 PRB utilisation (measured)
  5 best neighbour RSRP - serving RSRP     11 active UEs on the cell
    (dB, RRC measResults; -200 = none)
Action: 0 stay, k >= 1 hand over to the k-th neighbour gNB (ascending id).

Policies:
  greedy  hand over when feature 5 exceeds --a3-offset (TS 38.331 A3 on
          measured RSRP), with a 2-step hold-off after a handover
  stay    never hand over (baseline)
  dqn     small online DQN (PyTorch, epsilon-greedy), greedy fallback when
          PyTorch is missing

Under the control center the ns3ai launcher shim intercepts the simulator start
(it prints NTN_CC_AGENT_READY and the runner starts the binary). Standalone,
ns3ai_utils starts `./ns3 run oran-ntn-gym-handover-example` in --ns3-path.

Output: one line per step and a final JSON line with steps, handover actions
and the episode reward.
"""

import argparse
import json
import os
import random
import sys

import numpy as np

try:
    import torch
    import torch.nn as nn

    TORCH = True
except ImportError:  # pragma: no cover - optional
    TORCH = False


def parse():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--policy", choices=["greedy", "stay", "dqn"], default="greedy")
    p.add_argument("--a3-offset", type=float, default=3.0, help="greedy RSRP margin (dB)")
    p.add_argument("--target", default="oran-ntn-gym-handover-example")
    p.add_argument("--ns3-path", default=os.environ.get("NS3_PATH", os.getcwd()))
    p.add_argument("--duration", type=float, default=60.0)
    p.add_argument("--num-sats", type=int, default=3)
    p.add_argument("--num-ues", type=int, default=2)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--out", default="agent_episode.json")
    return p.parse_args()


class Dqn:
    def __init__(self, n_obs, n_act, seed):
        torch.manual_seed(seed)
        self.n_act = n_act
        self.q = nn.Sequential(nn.Linear(n_obs, 32), nn.ReLU(), nn.Linear(32, n_act))
        self.opt = torch.optim.Adam(self.q.parameters(), lr=1e-3)
        self.eps = 0.3
        self.prev = None

    def act(self, obs):
        if random.random() < self.eps:
            return random.randrange(self.n_act)
        with torch.no_grad():
            return int(self.q(torch.tensor(obs, dtype=torch.float32)).argmax())

    def learn(self, obs, action, reward, nxt, gamma=0.9):
        q = self.q(torch.tensor(obs, dtype=torch.float32))[action]
        with torch.no_grad():
            target = reward + gamma * self.q(torch.tensor(nxt, dtype=torch.float32)).max()
        loss = (q - target) ** 2
        self.opt.zero_grad()
        loss.backward()
        self.opt.step()
        self.eps = max(0.05, self.eps * 0.97)


def main():
    a = parse()
    random.seed(a.seed)
    np.random.seed(a.seed)
    import ns3ai_gym_env  # noqa: F401  (registers the env)
    from ns3ai_gym_env.envs import Ns3Env

    settings = {
        "gym": 1,
        "duration": a.duration,
        "numSats": a.num_sats,
        "numUes": a.num_ues,
        "kpm": "measured",
    }
    env = Ns3Env(targetName=a.target, ns3Path=a.ns3_path, ns3Settings=settings)
    obs, _ = env.reset()
    n_act = int(env.action_space.n)
    agent = Dqn(len(obs), n_act, a.seed) if (a.policy == "dqn" and TORCH) else None
    steps, hos, total, hold = 0, 0, 0.0, 0
    done = False
    while not done:
        o = np.asarray(obs, dtype=np.float32)
        if a.policy == "stay":
            action = 0
        elif agent is not None:
            action = agent.act(o)
        else:
            action = 1 if (hold == 0 and o[5] > a.a3_offset and n_act > 1) else 0
        hold = 2 if action else max(0, hold - 1)
        nxt, reward, terminated, truncated, info = env.step(action)
        if agent is not None:
            agent.learn(o, action, float(reward), np.asarray(nxt, dtype=np.float32))
        steps += 1
        hos += 1 if action else 0
        total += float(reward)
        print(f"step={steps} sinr={o[0]:.2f} margin={o[5]:.1f} action={action} "
              f"reward={float(reward):.3f}", flush=True)
        obs = nxt
        done = terminated or truncated
    env.close()
    result = {"policy": a.policy, "steps": steps, "ho_actions": hos,
              "episode_reward": round(total, 4), "torch": TORCH}
    with open(a.out, "w") as f:
        json.dump(result, f)
    print(json.dumps(result), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
