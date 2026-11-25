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

np.set_printoptions(precision=3, suppress=True)
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from ns3gym import ns3env


class Actor(nn.Module):
    def __init__(
        self,
        state_size,
        action_size,
        hidden_size=256,
        temperature=0.5,
        temperature_max=1.5,
        temperature_growth=1.001,
    ):
        super(Actor, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(state_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, action_size),
            nn.Sigmoid(),
        )
        self.temperature = temperature
        self.temperature_max = temperature_max
        self.temperature_growth = temperature_growth

        self.init_weights()

    def init_weights(self):
        for layer in self.fc:
            if isinstance(layer, nn.Linear):
                torch.nn.init.xavier_uniform_(layer.weight)  # Initialize Xavier
                torch.nn.init.zeros_(layer.bias)  # Initialize bias to zero

    def forward(self, state):
        logits = self.fc(state) / self.temperature
        noise = torch.randn_like(logits) * 0.05
        action_probabilities = torch.sigmoid(logits + noise)

        self.temperature = max(self.temperature * self.temperature_growth, self.temperature_max)

        return action_probabilities


class Critic(nn.Module):
    def __init__(self, state_size, action_size, hidden_size=256):
        super(Critic, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(state_size + action_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, action_size),
        )

        self.init_weights()

    def init_weights(self):
        for layer in self.fc:
            if isinstance(layer, nn.Linear):
                torch.nn.init.kaiming_uniform_(
                    layer.weight, nonlinearity="relu"
                )  # Initialize Kaiming
                torch.nn.init.zeros_(layer.bias)  # Initialize bias to zero

    def forward(self, state, action):
        q_values = self.fc(torch.cat([state, action], dim=-1))
        expected_q_value = (q_values * action).sum(dim=-1, keepdim=True)
        return expected_q_value


class ReplayBuffer:
    def __init__(self, buffer_size=100000):
        self.buffer = deque(maxlen=buffer_size)

    def add(self, experience):
        self.buffer.append(experience)

    def sample(self, batch_size=64):
        return random.sample(self.buffer, batch_size)

    def size(self):
        return len(self.buffer)


class OUNoise:
    def __init__(self, action_size, mu=0, theta=0.2, sigma=0.2):
        self.action_size = action_size
        self.mu = mu
        self.theta = theta
        self.sigma = sigma
        self.sigma_min = 0.01
        self.sigma_decay = 0.999
        self.reset()

    def reset(self):
        self.state = np.ones(self.action_size) * self.mu

    def sample(self):
        dx = self.theta * (self.mu - self.state) + self.sigma * np.random.randn(self.action_size)
        self.state = self.state + dx
        self.update_sigma()
        return self.state

    def update_sigma(self):
        self.sigma = max(self.sigma * self.sigma_decay, self.sigma_min)


def soft_update(target, source, tau=0.01):
    for target_param, source_param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(tau * source_param.data + (1.0 - tau) * target_param.data)


class DDPG:
    """!
    Proximal Policy Optimization (PPO) implementation for training a reinforcement learning agent.

    @param state_shape: The shape of the state space, typically a tuple representing the observation dimensions.
    @param hidden_dim: The size of the hidden layers for the neural network (default is 64).
    @param lr: Learning rate for the optimizer (default is 0.0003).
    @param gamma: Discount factor for future rewards (default is 0.99).
    @param eps_clip: Clipping value for the surrogate loss to stabilize training (default is 0.2).
    @param k_epochs: Number of epochs for which PPO updates are applied (default is 4).

    @return PPO object for policy optimization during the RL simulation.
    """

    def __init__(
        self,
        state_size,
        action_size,
        actor_lr=1e-4,
        critic_lr=1e-4,
        gamma=0.99,
        tau=0.001,
    ):
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() and args.enableCuda else "cpu"
        )  # GPU or CPU
        if args.enableCuda and not torch.cuda.is_available():
            print("CUDA is not available. Using CPU instead.")
        print(f"Using Device: {str(self.device).upper()}")

        self.actor = Actor(state_size, action_size).to(self.device)
        self.critic = Critic(state_size, action_size).to(self.device)

        self.target_actor = Actor(state_size, action_size).to(self.device)
        self.target_critic = Critic(state_size, action_size).to(self.device)

        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=actor_lr)
        self.critic_optimizer = optim.Adam(
            self.critic.parameters(), lr=critic_lr, weight_decay=1e-4
        )
        self.tau = tau
        self.gamma = gamma
        self.lambda_fairness = 0.5
        self.max_lambda_fairness = 10.0

        # OU Noise
        self.ou_noise = OUNoise(action_size=action_size, sigma=0.5)

        # Initialize target networks
        self.target_actor.load_state_dict(self.actor.state_dict())
        self.target_critic.load_state_dict(self.critic.state_dict())

        self.replay_buffer = ReplayBuffer()

    def act(self, state, mask):
        state = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        self.actor.eval()
        with torch.no_grad():
            action_probabilities = self.actor(state).cpu().numpy().flatten()
        self.actor.train()

        # Apply OU noise to the action probabilities
        noise = self.ou_noise.sample()
        action_probabilities = np.clip(action_probabilities + noise, 1e-10, 1)

        # Mask the action probabilities
        action_probabilities *= mask

        # Prevent the action probabilities from summing to zero
        if action_probabilities.sum() == 0:
            action_probabilities = np.ones_like(action_probabilities) / len(action_probabilities)
        else:
            action_probabilities /= action_probabilities.sum()  # Normalize

        return action_probabilities

    def learn(self, fairness, batch_size=64):
        if self.replay_buffer.size() < batch_size:
            return

        batch = self.replay_buffer.sample(batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        states = torch.FloatTensor(np.array(states)).to(self.device)
        actions = torch.FloatTensor(np.array(actions)).to(self.device)
        rewards = torch.FloatTensor(rewards).unsqueeze(1).to(self.device)
        next_states = torch.FloatTensor(np.array(next_states)).to(self.device)
        dones = torch.FloatTensor(dones).unsqueeze(1).to(self.device)

        # Calculate Target Network
        next_actions = self.target_actor(next_states)
        next_q_values = self.target_critic(next_states, next_actions)

        # Current Q Value
        current_q_values = self.critic(states, actions)
        expected_current_q = (current_q_values * actions).sum(dim=-1, keepdim=True)
        debug(f"Critic Predicted Q-values Mean: {expected_current_q.mean().item()}", args.debug)

        # Target Q Value
        target_q_values = (
            rewards
            + (1 - dones) * self.gamma * (0.9 * next_q_values.detach() + 0.1 * expected_current_q)
            - self.lambda_fairness * (1.0 - fairness)
        )
        print(f"Target Q-values: {target_q_values.mean().item()}")
        target_q_values = torch.clamp(target_q_values, -50, 50)
        print(f"Clamped Target Q-values: {target_q_values.mean().item()}")

        # Critic Loss
        critic_loss = F.smooth_l1_loss(expected_current_q, target_q_values)
        fairness_penalty = self.lambda_fairness * (1.0 - fairness) ** 2
        critic_loss += fairness_penalty
        critic_params = torch.cat([p.view(-1) for p in self.critic.parameters()])
        critic_loss += 0.0001 * torch.norm(critic_params, p=2)  # Add L2 regularization

        debug(f"Critic Loss: {critic_loss}", args.debug)
        print(f"Critic Loss: {critic_loss.item()}")

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # Actor Loss
        predicted_actions = self.actor(states)
        action_distribution = torch.distributions.Normal(predicted_actions, 0.1)
        sampled_actions = action_distribution.rsample()

        critic_q_values = self.critic(states, predicted_actions)

        expected_actor_q = (critic_q_values * sampled_actions).sum(dim=-1, keepdim=True)
        actor_loss = -0.1 * expected_actor_q.mean()

        entropy = action_distribution.entropy().mean()
        actor_loss -= 0.05 * entropy

        actor_loss -= fairness_penalty

        debug(f"Actor Loss: {actor_loss}", args.debug)
        print(f"Actor Loss: {actor_loss}")

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        soft_update(self.target_actor, self.actor, self.tau)
        soft_update(self.target_critic, self.critic, self.tau)

        self.lambda_fairness = min(self.lambda_fairness * 1.05, self.max_lambda_fairness)


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

    # debug(f"Reordered state: \n{reordered_state}", args.debug)
    # debug(f"Mask: {mask}", args.debug)

    return reordered_state[:, 1:], np.array(mask)


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
    state_size = state_shape[0] * (state_shape[1] - 1)
    action_size = state_shape[0]
    ddpg = DDPG(state_size, action_size)

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

            debug(f"State: \n{state}", args.debug)
            debug(f"Mask: {mask}", args.debug)
            action = ddpg.act(state.reshape(-1), mask)
            debug(f"Action: {action}", args.debug)
            debug(f"Selected action: {np.argmax(action)}", args.debug)
            reordered_action = reorder_action(action, reshaped_obs, flow_map)
            # debug(f"Reordered action: {reordered_action}", args.debug)

            obs, reward, done, _ = env.step(reordered_action)
            if done:
                break

            reshaped_obs = obs.reshape(int(len(obs) / state_shape[1]), state_shape[1])
            next_state, mask = reorder_state(reshaped_obs, flow_map, state_shape[1])

            # normalized_reward = math.copysign(math.log1p(abs(reward)), reward)
            normalized_reward = math.tanh(reward)
            debug(f"Reward: {reward}", args.debug)
            debug(f"Normalized Reward: {normalized_reward}", args.debug)

            end_time = time.time()
            total_time += end_time - start_time

            ddpg.replay_buffer.add(
                (state.reshape(-1), action, normalized_reward, next_state.reshape(-1), done)
            )
            state = next_state

            if step_idx > 0 and step_idx % step_interval == 0:
                avg_tput = state[:, -1]
                print(f"Average Throughput: {avg_tput}")
                fairness = (np.sum(avg_tput) ** 2) / (
                    len(avg_tput) * np.sum(avg_tput**2) + 1e-6
                ) - 0.05 * np.var(avg_tput)
                debug(f"Fairness: {fairness}", args.debug)
                print(f"Fairness: {fairness}")
                ddpg.learn(fairness=fairness)

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
        "--stepInterval", type=int, default=256, help="Step interval for updating the PPO"
    )
    args = parser.parse_args()

    main(args)
