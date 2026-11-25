#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2024 Seoul National University (SNU)
# Copyright (c) 2024 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
#
# SPDX-License-Identifier: GPL-2.0-only

import argparse
import math
import random
import time
from collections import deque
from types import MappingProxyType

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from ns3gym import ns3env

np.set_printoptions(precision=3, suppress=True)


# Define Q-Network
class QNetwork(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(QNetwork, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),  # Q-values for each action
        )

    def forward(self, state):
        """!
        Forward pass through the network.
        @param state: The input state tensor.
        @return: (batch_size, action_dim)
        """
        return self.fc(state)


# DDQN Agent
class DDQNAgent:
    def __init__(
        self,
        state_dim,
        action_dim,
        lr=0.001,
        gamma=0.98,
        tau=0.005,
        epsilon=1.0,
        epsilon_min=0.05,
        epsilon_decay=0.992,
        buffer_size=10000,
        batch_size=64,
    ):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.gamma = gamma
        self.tau = tau  # Soft update parameter
        self.epsilon = epsilon
        self.epsilon_min = epsilon_min
        self.epsilon_decay = epsilon_decay

        # Device Setting
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() and args.enableCuda else "cpu"
        )  # GPU or CPU
        if args.enableCuda and not torch.cuda.is_available():
            print("CUDA is not available. Using CPU instead.")
        print(f"Using Device: {str(self.device).upper()}")

        # Initialize Q-Networks
        self.q_network = QNetwork(state_dim, action_dim).to(self.device)
        self.target_q_network = QNetwork(state_dim, action_dim).to(self.device)
        self.target_q_network.load_state_dict(self.q_network.state_dict())  # 초기 가중치 동기화
        self.target_q_network.eval()  # target 네트워크는 평가 모드

        self.optimizer = optim.Adam(self.q_network.parameters(), lr=lr)
        self.criterion = nn.MSELoss()

        # Replay buffer
        self.memory = deque(maxlen=buffer_size)
        self.batch_size = batch_size

    def select_action(self, state, mask, train=True):
        state = torch.FloatTensor(state).unsqueeze(0)  # (1, state_dim) 형태로 변환
        mask = torch.FloatTensor(mask).unsqueeze(0)  # (1, action_dim) 형태로 변환
        if train and random.random() < self.epsilon:  # ε-greedy exploration
            valid_actions = torch.where(mask.squeeze() == 1)[0]
            return random.choice(valid_actions.tolist())
        else:
            with torch.no_grad():
                q_values = self.q_network(state)
                q_values = q_values * mask
                return torch.argmax(q_values, dim=1).item()  # 가장 Q-value가 높은 action 선택

    def store_experience(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def train(self):
        if len(self.memory) < self.batch_size:
            return  # 데이터가 충분히 쌓일 때까지 대기

        # Mini-batch 샘플링
        batch = random.sample(self.memory, self.batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)

        states = torch.tensor(np.array(states).astype(np.float32))
        actions = torch.LongTensor(actions).unsqueeze(1)  # (batch_size, 1)
        rewards = torch.FloatTensor(rewards).unsqueeze(1)
        next_states = torch.FloatTensor(np.array(next_states).astype(np.float32))
        dones = torch.FloatTensor(dones).unsqueeze(1)

        # Current Q-value
        q_values = self.q_network(states).gather(1, actions)  # (batch_size, 1)

        # Double Q-learning 적용
        next_actions = self.q_network(next_states).argmax(
            dim=1, keepdim=True
        )  # Policy Network가 선택한 action
        next_q_values = self.target_q_network(next_states).gather(
            1, next_actions
        )  # Target Network에서 평가
        target_q_values = rewards + (1 - dones) * self.gamma * next_q_values

        # Loss 계산 및 업데이트
        loss = self.criterion(
            q_values, target_q_values.detach()
        )  # detach()로 target_q_values 업데이트 막음
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        debug(f"Loss: {loss.item()}", args.debug)

        # **Soft update로 Target Network 업데이트**
        for target_param, param in zip(
            self.target_q_network.parameters(), self.q_network.parameters()
        ):
            target_param.data.copy_(self.tau * param.data + (1.0 - self.tau) * target_param.data)

        # Epsilon decay
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)

    def save_model(self, path):
        torch.save(self.q_network.state_dict(), path)

    def load_model(self, path):
        self.q_network.load_state_dict(torch.load(path))
        self.target_q_network.load_state_dict(self.q_network.state_dict())


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
            index = np.where(flow_order == key)[0][0]
            reordered_state[index] = row
    # If no matching row is found, the initialized zeros will remain

    mask = [
        0 if np.array_equal(row[1:], np.zeros(num_features - 1)) else 1 for row in reordered_state
    ]

    debug(f"Reordered state: \n{reordered_state}", args.debug)
    debug(f"Mask: {mask}", args.debug)

    return reordered_state[:, 1:], np.array(mask)


def reorder_action(action, reshaped_obs, flow_order):
    reordered_action = []

    for row in reshaped_obs:
        key = int(row[0])
        if key in flow_order:
            index = np.where(key == flow_order)[0][0]
            reordered_action.append(action[index])

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
    ddqn = DDQNAgent(state_dim, action_dim)

    total_time = 0

    # Configure the statistics
    stats_data = pd.DataFrame(columns=["step", "process_time", "reward", "normalized_reward"])

    # Training the agent
    step_idx = 0
    step_interval = args.stepInterval
    try:
        obs = env.reset()
        reshaped_obs = obs.reshape(state_shape)
        print(f"Reshaped observation:\n{reshaped_obs}")
        print(f"Reshaped observation shape: {reshaped_obs.shape}")
        sorted_obs = np.argsort(reshaped_obs[:, 0])
        flow_map = reshaped_obs[sorted_obs, 0]
        print(f"Flow map:\n{flow_map}")
        print(f"Flow map shape: {flow_map.shape}")
        state, mask = reorder_state(reshaped_obs, flow_map, state_shape[1])
        while True:
            debug(f"\nStep: {step_idx}", args.debug)
            start_time = time.time()

            # debug(f"Observation: \n{reshaped_obs}", args.debug)
            debug(f"State: \n{state.reshape(state_shape[0], state_shape[1] - 1)}", args.debug)
            debug(f"Mask: {mask}", args.debug)
            action = ddqn.select_action(state.reshape(-1), mask)
            debug(f"Selected action: {action}", args.debug)
            action_vec = np.zeros(action_dim)
            action_vec[action] = 1
            reordered_action = reorder_action(action_vec, reshaped_obs, flow_map)
            debug(f"Reordered action: {reordered_action}", args.debug)

            obs, reward, done, _ = env.step(reordered_action)
            if done:
                break

            normalized_reward = math.copysign(math.log1p(abs(reward) + 1e-6), reward)
            # normalized_reward = math.tanh(reward/1000)
            reshaped_obs = obs.reshape(int(len(obs) / state_shape[1]), state_shape[1])
            next_state, mask = reorder_state(reshaped_obs, flow_map, state_shape[1])
            debug(
                f"Next State: \n{next_state.reshape(state_shape[0], state_shape[1] - 1)}",
                args.debug,
            )
            debug(f"Reward: {reward}", args.debug)
            debug(f"Normalized Reward: {normalized_reward}", args.debug)

            ddqn.store_experience(
                state.reshape(-1), action, normalized_reward, next_state.reshape(-1), done
            )
            state = next_state

            if step_idx > 0 and step_idx % step_interval == 0:
                ddqn.train()

            end_time = time.time()
            total_time += end_time - start_time

            stats_data.loc[len(stats_data)] = {
                "step": step_idx,
                "process_time": end_time - start_time,
                "reward": reward,
                "normalized_reward": normalized_reward,
            }
            step_idx += 1
    except KeyboardInterrupt:
        print("Ctrl-C -> Exit")
    finally:
        print(f"Average Processing Time: {total_time*1000 / step_idx:.4f} ms/step")
        stats_data.to_csv(f"{args.outputDir}/ddqn_stats_{args.simTag}.csv", index=False)
        env.close()
        print("Done")


if __name__ == "__main__":
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
        "--stepInterval", type=int, default=256, help="Step interval for updating the PPO"
    )
    args = parser.parse_args()

    main(args)
