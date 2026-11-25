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
import torch.optim as optim
from ns3gym import ns3env


class PPO:
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
        state_dim,
        action_dim,
        hidden_dim=64,
        lr=0.0003,
        gamma=0.99,
        eps_clip=0.2,
        k_epochs=4,
    ):

        self.gamma = gamma
        self.eps_clip = eps_clip
        self.k_epochs = k_epochs

        self.device = torch.device(
            "cuda" if torch.cuda.is_available() and args.enableCuda else "cpu"
        )  # GPU or CPU
        if args.enableCuda and not torch.cuda.is_available():
            print("CUDA is not available. Using CPU instead.")
        print(f"Using Device: {str(self.device).upper()}")

        self.policy = self.ActorCritic(state_dim, action_dim, hidden_dim, self.device).to(
            self.device
        )
        self.optimizer = optim.Adam(self.policy.parameters(), lr=lr)
        self.policy_old = self.ActorCritic(state_dim, action_dim, hidden_dim, self.device).to(
            self.device
        )
        self.policy_old.load_state_dict(self.policy.state_dict())

        self.mse_loss = nn.MSELoss()

    class ActorCritic(nn.Module):
        """!
        A neural network module that contains both the actor and critic components for PPO.

        @param state_shape: Shape of the state space (e.g., for a 3D observation, it could be (batch_size, 3, 4)).
        @param action_dim: Number of possible actions the agent can take.
        @param hidden_dim: Dimension of hidden layers for both actor and critic networks.

        @return ActorCritic object with methods to compute actions and evaluate states.
        """

        def __init__(self, state_dim, action_dim, hidden_dim, device, temperature=1.5):
            super(PPO.ActorCritic, self).__init__()
            self.device = device
            self.temperature = temperature

            # Actor Network outputs mean and log_std for Normal distribution
            self.actor = nn.Sequential(
                nn.Linear(state_dim, hidden_dim),  # Expect input of shape (n, m)
                nn.ReLU(),
                nn.Linear(hidden_dim, hidden_dim),
                nn.ReLU(),
                nn.Linear(hidden_dim, action_dim),
            )

            for layer in self.actor:
                if isinstance(layer, nn.Linear):
                    nn.init.xavier_uniform_(layer.weight)
                    nn.init.zeros_(layer.bias)

            # Critic Network for value estimation
            self.critic = nn.Sequential(
                nn.Linear(state_dim, hidden_dim),
                nn.ReLU(),
                nn.Linear(hidden_dim, hidden_dim),
                nn.ReLU(),
                nn.BatchNorm1d(hidden_dim),
                nn.Linear(hidden_dim, 1),
            )

        def act(self, state, mask):
            """!
            Takes in the current state of the environment and outputs a continuous action based on the policy.

            @param state: The state of the environment, usually a multi-dimensional array (e.g., (3, 4)).
            @param mask: Boolean mask indicating which rows are active.
            @return Tuple containing the selected action, the log-probability of that action, and the
                    temperature used to sample it.
            """
            state = state.unsqueeze(0)
            mask = mask.unsqueeze(0)

            temperature = self.temperature
            logits = self.actor(state) / temperature
            action_probs = torch.softmax(logits, dim=-1) * mask
            action_probs = torch.clamp(action_probs, min=1e-8)
            action_probs = action_probs / (action_probs.sum(dim=-1, keepdim=True) + 1e-8)
            distribution = torch.distributions.Categorical(action_probs)
            action = distribution.sample()

            print("Action Probs:", action_probs)
            print("Sum of Probs:", action_probs.sum(dim=-1))
            print("Mask:", mask)

            self.temperature = max(0.1, self.temperature * 0.999)

            return action.item(), distribution.log_prob(action).squeeze(0), temperature

        def evaluate(self, state, action, mask, temperature):
            """!
            Evaluates the state and action to compute log-probabilities, state values, and entropy.

            @param state: The state from the environment, potentially batch-processed.
            @param action: The action taken by the agent.
            @param mask: Boolean mask indicating which rows are active.
            @param temperature: The temperature each action was sampled with, one row per state.
            @return Tuple containing the log-probabilities of the actions, the state values (critic), and the entropy.
            """
            logits = self.actor(state) / temperature
            action_probs = torch.softmax(logits, dim=-1) * mask
            action_probs = torch.clamp(action_probs, min=1e-8)
            action_probs = action_probs / (action_probs.sum(dim=-1, keepdim=True) + 1e-8)
            distribution = torch.distributions.Categorical(action_probs)

            log_probs = distribution.log_prob(action)
            entropy = distribution.entropy().mean() * 0.01
            state_value = self.critic(state)

            return log_probs, torch.squeeze(state_value), entropy

    def select_action(self, state, memory, mask):
        """!
        Selects an action based on the current policy and stores relevant information in memory.

        @param state: The current state of the environment.
        @param memory: An instance of the Memory class to store states, actions, and log probabilities for future updates.
        @param mask: Boolean mask indicating which rows are active.
        @return action: The action selected by the policy.
        """
        state = state.flatten()
        memory.states.append(state)
        memory.mask.append(mask)

        state = torch.tensor(state, dtype=torch.float32, device=self.device)
        mask = torch.tensor(mask, dtype=torch.float32, device=self.device)
        action, action_logprob, temperature = self.policy_old.act(state, mask)
        memory.actions.append(action)
        memory.logprobs.append(action_logprob.detach())
        memory.temperatures.append(temperature)

        return action

    def update(self, memory):
        """!
        Updates the policy using the memory of past actions and states based on the PPO update rule.

        @param memory: Memory instance containing past states, actions, log probabilities, and rewards.
        @return None: This function updates the policy parameters based on the experience stored in memory.
        """
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(memory.rewards), reversed(memory.is_terminals)):
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)

        rewards = torch.tensor(rewards, dtype=torch.float32).to(self.device)
        rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-5)

        old_states = torch.tensor(np.array(memory.states), dtype=torch.float32, device=self.device)
        old_masks = torch.tensor(np.array(memory.mask), dtype=torch.float32, device=self.device)
        old_actions = torch.tensor(memory.actions, dtype=torch.long, device=self.device)
        old_logprobs = torch.stack(list(memory.logprobs)).to(self.device)
        # The sampling temperature decays at every action and is not part of the state dict, so
        # evaluate each action with the temperature it was sampled with
        old_temperatures = torch.tensor(
            list(memory.temperatures), dtype=torch.float32, device=self.device
        ).unsqueeze(-1)

        for _ in range(self.k_epochs):
            # Get new log_probs, state_values, entropy from current policy
            logprobs, state_values, dist_entropy = self.policy.evaluate(
                old_states, old_actions, old_masks, old_temperatures
            )

            # Compute PPO objective
            ratios = torch.exp(logprobs - old_logprobs.detach())  # Compute probability ratios
            advantages = rewards - state_values.detach()  # Compute advantages

            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1 - self.eps_clip, 1 + self.eps_clip) * advantages

            loss = (
                -torch.min(surr1, surr2)  # PPO loss
                + 0.5 * self.mse_loss(state_values, rewards)  # Critic loss
                - 0.01 * dist_entropy.mean()  # Entropy loss (mean over batch)
            )

            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()

        # Update old policy weights
        self.policy_old.load_state_dict(self.policy.state_dict())


class Memory:
    """!
    Memory class for storing states, actions, log probabilities, rewards, and terminal flags during training.

    The PPO algorithm requires storage of past experiences (states, actions, rewards, etc.) so that they can be
    used during the policy update step. This class implements a memory buffer using dequeues with a fixed size to
    prevent excessive memory consumption.

    @param maxlen: Maximum length of the memory buffer. The default value is 1000, which means the memory will hold
                   up to 1000 samples before older samples start being discarded.

    @return Memory object for storing experiences during the RL simulation.
    """

    def __init__(self, maxlen=1000):
        self.states = deque(maxlen=maxlen)
        self.actions = deque(maxlen=maxlen)
        self.logprobs = deque(maxlen=maxlen)
        self.rewards = deque(maxlen=maxlen)
        self.is_terminals = deque(maxlen=maxlen)
        self.mask = deque(maxlen=maxlen)
        self.temperatures = deque(maxlen=maxlen)

    def clear_memory(self):
        """!
        Clears the memory of stored states, actions, log probabilities, rewards, and terminal flags after an update.

        @return None: This function resets the memory buffers.
        """
        self.states.clear()
        self.actions.clear()
        self.logprobs.clear()
        self.rewards.clear()
        self.is_terminals.clear()
        self.mask.clear()
        self.temperatures.clear()


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
    state_dim = state_shape[0] * (state_shape[1] - 1)
    action_dim = state_shape[0]
    ppo = PPO(state_dim, action_dim)
    memory = Memory(maxlen=args.stepInterval)

    total_time = 0

    # Training the agent
    step_idx = 0
    step_interval = args.stepInterval
    try:
        obs = env.reset()
        reshaped_obs = obs.reshape(state_shape)
        sorted_obs = np.argsort(reshaped_obs[:, 0])
        flow_map = reshaped_obs[sorted_obs, 0]
        while True:
            debug(f"\nStep: {step_idx}", args.debug)
            start_time = time.time()

            # debug(f"Observation: \n{reshaped_obs}", args.debug)
            state, mask = reorder_state(reshaped_obs, flow_map, state_shape[1])
            debug(f"State: \n{state.reshape(state_shape[0], state_shape[1] - 1)}", args.debug)
            debug(f"Mask: {mask}", args.debug)
            action = ppo.select_action(state, memory, mask)
            action_vec = np.zeros(action_dim)
            action_vec[action] = 1
            debug(f"Selected action: {action}", args.debug)
            reordered_action = reorder_action(action_vec, reshaped_obs, flow_map)
            debug(f"Reordered action: {reordered_action}", args.debug)

            obs, reward, done, _ = env.step(reordered_action)
            normalized_reward = math.copysign(math.log1p(abs(reward)), reward)
            reshaped_obs = obs.reshape(int(len(obs) / state_shape[1]), state_shape[1])
            debug(f"Reward: {reward}", args.debug)
            debug(f"Normalized Reward: {normalized_reward}", args.debug)

            memory.rewards.append(normalized_reward)
            memory.is_terminals.append(done)

            end_time = time.time()
            total_time += end_time - start_time

            if done:
                break

            if step_idx > 0 and step_idx % step_interval == 0:
                ppo.update(memory)
                memory.clear_memory()

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
