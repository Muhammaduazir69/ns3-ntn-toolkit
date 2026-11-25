#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2024 Seoul National University (SNU)
# Copyright (c) 2024 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
#
# SPDX-License-Identifier: GPL-2.0-only

import argparse
import math
import time
from collections import deque
from types import MappingProxyType

import numpy as np

np.set_printoptions(precision=3, suppress=True)
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from ns3gym import ns3env


class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim):
        super(Actor, self).__init__()
        self.actor = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Softmax(dim=-1),  # 각 action의 확률 출력
        )

    def forward(self, state):
        action_probs = self.actor(state)  # Action 확률 분포
        return action_probs


class Critic(nn.Module):
    def __init__(self, state_dim, hidden_dim):
        super(Critic, self).__init__()
        self.critic = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1),  # State Value 출력
        )

    def forward(self, state):
        value = self.critic(state)  # 상태 가치 함수 V(s) 출력
        return value


class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=64):
        super(ActorCritic, self).__init__()
        self.actor = Actor(state_dim, action_dim, hidden_dim)
        self.critic = Critic(state_dim, hidden_dim)

    def forward(self, state):
        action_probs = self.actor(state)
        state_value = self.critic(state)
        return action_probs, state_value

    def act(self, state, mask):
        action_probs, _ = self.forward(state)
        action_probs = action_probs * mask
        action_probs /= action_probs.sum() + 1e-8
        distribution = torch.distributions.Categorical(action_probs)
        action = distribution.sample()
        return action, distribution.log_prob(action)


class A2CAgent:
    def __init__(self, state_dim, action_dim, actor_lr=1e-4, critic_lr=1e-3, gamma=0.99):
        self.gamma = gamma
        self.policy = ActorCritic(state_dim, action_dim)

        self.actor_optimizer = optim.Adam(self.policy.actor.parameters(), lr=actor_lr)
        self.critic_optimizer = optim.Adam(self.policy.critic.parameters(), lr=critic_lr)

        self.device = torch.device(
            "cuda" if torch.cuda.is_available() and args.enableCuda else "cpu"
        )  # GPU or CPU
        if args.enableCuda and not torch.cuda.is_available():
            print("CUDA is not available. Using CPU instead.")
        print(f"Using Device: {str(self.device).upper()}")

    def select_action(self, state, mask):
        state = torch.FloatTensor(state).to(self.device)
        mask = torch.FloatTensor(mask).to(self.device)
        action, log_prob = self.policy.act(state, mask)
        return action.item(), log_prob

    def evaluate(self, state, action, reward, next_state, done):
        state = torch.FloatTensor(state).to(self.device)
        next_state = torch.FloatTensor(next_state).to(self.device)
        action = torch.LongTensor([action]).to(self.device)
        reward = torch.FloatTensor([reward]).to(self.device)
        done = torch.FloatTensor([done]).to(self.device)

        action_probs, state_value = self.policy(state)
        _, next_state_value = self.policy(next_state)

        advantage = (
            reward + (1 - done) * self.gamma * next_state_value.detach() - state_value.detach()
        )

        distribution = torch.distributions.Categorical(action_probs)
        log_prob = distribution.log_prob(action)
        entropy = distribution.entropy().mean()

        actor_loss = -log_prob * advantage.detach() - 0.2 * entropy
        critic_loss = F.mse_loss(state_value, (reward + self.gamma * next_state_value).detach())

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()


def debug(msg, debug_flag):
    """!
    Prints debug messages if the debug flag is enabled.

    @param msg: The debug message to print.
    @param debug_flag: A boolean flag indicating whether to print debug information.
    @return None: This function prints debug messages when necessary.
    """
    if debug_flag:
        print(msg)


def create_flow_map(flow_order):
    """!
    Creates a flow order map for mapping flow order to indices.

    @param flow_order: A list of flow order tuples, where each tuple contains the flow ID and the UE ID.
    @return MappingProxyType: An immutable mapping of flow order tuples to indices.
    """
    mutable_flow_order = {tuple(row): i for i, row in enumerate(flow_order)}
    return MappingProxyType(mutable_flow_order)


def reorder_state(reshaped_obs, flow_order, num_features):
    """Reorders the state rows to match the obj_order and fills missing entries with zeros."""
    # Initialize the reordered state with zeros
    reordered_state = np.zeros((len(flow_order), num_features))
    reordered_state[:, 0] = np.array(flow_order)

    # Iterate over the flow_order and match it with the new state
    for row in reshaped_obs:
        # key = tuple(row[0])
        key = int(row[0])
        if key in flow_order:
            reordered_state[key - 1] = row
    # If no matching row is found, the initialized zeros will remain

    mask = [
        0 if np.array_equal(row[1:], np.zeros(num_features - 1)) else 1 for row in reordered_state
    ]

    return reordered_state[:, 1:].reshape(-1), np.array(mask)


def reorder_action(action, reshaped_obs, flow_order):
    reordered_action = []

    for row in reshaped_obs:
        key = int(row[0])
        if key in flow_order:
            reordered_action.append(action[key - 1])

    return reordered_action


def main(args):
    """!
    Main function for running the PPO-based reinforcement learning simulation using the ns-3 gym environment.

    @param args: Command-line arguments containing simulation parameters, such as the number of UEs,
                 simulation time, bandwidth, and other necessary configuration options.

    @return None: This function runs the PPO agent in the environment, selecting actions, and updating the model.
    """
    simArgs = {
        "--ueNum": args.ueNum,
        "--logging": args.logging,
        "--priorityTrafficScenario": args.priorityTrafficScenario,
        "--simTime": args.simTime,
        "--numerology": args.numerology,
        "--centralFrequency": args.centralFrequency,
        "--bandwidth": args.bandwidth,
        "--totalTxPower": args.totalTxPower,
        "--simTag": args.simTag,
        "--outputDir": args.outputDir,
        "--enableOfdma": args.enableOfdma,
        "--enableLcLevelQos": args.enableLcLevelQos,
        "--ueLevelSchedulerType": "Ai",  # Type of scheduler being used in the simulation (here, AI-based)
        "--enablePdcpDiscarding": args.enablePdcpDiscarding,
    }
    # Create the environment
    env = ns3env.Ns3Env(port=args.port, simSeed=args.simSeed, simArgs=simArgs, debug=args.debug)
    state_shape = env.observation_space.shape
    state_dim = state_shape[0] * (state_shape[1] - 1)
    action_dim = state_shape[0]
    a2c = A2CAgent(state_dim, action_dim)

    total_time = 0

    # Training the agent
    step_idx = 0
    step_interval = args.stepInterval
    try:
        obs = env.reset()
        reshaped_obs = obs.reshape(state_shape)
        sorted_obs = np.argsort(reshaped_obs[:, 0])
        flow_map = reshaped_obs[sorted_obs, 0]
        state, mask = reorder_state(reshaped_obs, flow_map, state_shape[1])
        while True:
            debug(f"\nStep: {step_idx}", args.debug)
            start_time = time.time()

            # debug(f"Observation: \n{reshaped_obs}", args.debug)
            debug(f"State: \n{state.reshape(state_shape[0], state_shape[1] - 1)}", args.debug)
            debug(f"Mask: {mask}", args.debug)
            action, _ = a2c.select_action(state, mask)
            debug(f"Selected action: {action}", args.debug)
            action_vec = np.zeros(action_dim)
            action_vec[action] = 1
            reordered_action = reorder_action(action_vec, reshaped_obs, flow_map)
            debug(f"Reordered action: {reordered_action}", args.debug)

            obs, reward, done, _ = env.step(reordered_action)
            normalized_reward = math.copysign(math.log1p(abs(reward)), reward)
            reshaped_obs = obs.reshape(int(len(obs) / state_shape[1]), state_shape[1])
            next_state, mask = reorder_state(reshaped_obs, flow_map, state_shape[1])
            debug(
                f"Next State: \n{next_state.reshape(state_shape[0], state_shape[1] - 1)}",
                args.debug,
            )
            debug(f"Reward: {reward}", args.debug)
            debug(f"Normalized Reward: {normalized_reward}", args.debug)

            a2c.evaluate(state, action, normalized_reward, next_state, done)
            state = next_state

            end_time = time.time()
            total_time += end_time - start_time

            if done:
                break

            step_idx += 1

    except KeyboardInterrupt:
        print("Ctrl-C -> Exit")
    finally:
        print(f"Average Processing Time: {total_time*1000 / step_idx:.4f} ms/step")
        env.close()
        print("Done")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser = argparse.ArgumentParser()
    # Arguments used for the gym script
    parser.add_argument("--port", type=int, default=5552, help="Port number")
    parser.add_argument("--simSeed", type=int, default=3002, help="Seed number")
    parser.add_argument("--debug", action="store_true", help="Debug mode")
    parser.add_argument("--enableCuda", action="store_true", help="Enable CUDA")
    # Arguments used for the ns3 simulation (simArgs)
    parser.add_argument(
        "--ueNum", type=int, default=2, help="Number of UEs (User Equipment) in the simulation"
    )
    parser.add_argument(
        "--logging",
        action="store_true",
        help="Enable or disable logging during the simulation",
    )
    parser.add_argument(
        "--priorityTrafficScenario",
        type=int,
        default=0,
        help="The traffic scenario for the case of priority. Can be 0: saturation or 1: medium-load",
    )
    parser.add_argument(
        "--simTime", type=float, default=1.0, help="Total simulation time in seconds"
    )
    parser.add_argument(
        "--numerology", type=int, default=0, help="Numerology parameter for the simulation"
    )
    parser.add_argument(
        "--centralFrequency",
        type=float,
        default=4e9,
        help="Central frequency for the simulation in Hz",
    )
    parser.add_argument(
        "--bandwidth", type=float, default=10e6, help="Bandwidth in Hz for the simulation"
    )
    parser.add_argument(
        "--totalTxPower", type=float, default=43, help="Total transmission power in dBm"
    )
    parser.add_argument(
        "--simTag", type=str, default="default", help="Tag to label the simulation for reference"
    )
    parser.add_argument(
        "--outputDir", type=str, default="./", help="Directory where output data will be stored"
    )
    parser.add_argument(
        "--enableOfdma", action="store_true", help="Whether to enable OFDMA in the simulation"
    )
    parser.add_argument(
        "--enableLcLevelQos",
        action="store_true",
        help="Whether to enable QoS LC scheduler",
    )
    parser.add_argument(
        "--enablePdcpDiscarding",
        action="store_true",
        help="Enable PDCP discarding",
    )
    # Update interval for the PPO algorithm
    parser.add_argument(
        "--stepInterval", type=int, default=100, help="Step interval for updating the PPO"
    )
    args = parser.parse_args()

    main(args)
