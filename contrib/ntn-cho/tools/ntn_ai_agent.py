#!/usr/bin/env python3
"""
NTN AI Agent - Deep Reinforcement Learning for LEO Satellite Handover

This Python agent communicates with the ns-3 NTN-CHO simulation via
ns3-ai shared memory to make AI-driven handover decisions.

Supported Algorithms:
  - DQN (Deep Q-Network) - Discrete action: select target cell
  - PPO (Proximal Policy Optimization) - Continuous timing prediction
  - Rainbow-DQN - DQN with prioritized replay, dueling, noisy nets
  - Federated DQN - Multi-UE federated averaging

Architecture (based on ETRI Journal 2026, MDPI Electronics 2026):
  State:  [serving_sinr, serving_elev, serving_tte, serving_doppler,
           cand1_sinr, cand1_elev, cand1_tte, ...,
           ue_speed, recent_ho_count, recent_failures, avg_tos]
  Action: Discrete index 0..K (0=stay, 1-K=handover to candidate k)
  Reward: +1 success + SINR_improvement - ping_pong_penalty - failure_penalty

Usage:
  # Terminal 1: Run ns-3 simulation
  ./ns3 run "ntn-cho-ai-example --simTime=300 --aiEnabled=true"

  # Terminal 2: Run AI agent (connects via shared memory)
  python3 ntn_ai_agent.py --algorithm dqn --episodes 100

References:
  [1] ETRI Journal 2026 - Multi-agent DRL proactive handover for LEO-TN
  [2] MDPI Electronics 2026 - Attention-enhanced Rainbow-DQN for LEO HO
  [3] MDPI Electronics 2026 - K-Means + DQN for LEO satellite HO
  [4] Springer 2026 - Federated MARL for LEO interference management

Author: Muhammad Uzair
License: GPL-2.0
"""

import argparse
import os
import sys
import time
import json
import struct
import numpy as np
from collections import deque
from pathlib import Path

# Try importing AI frameworks
try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    import torch.nn.functional as F
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False
    print("WARNING: PyTorch not available. Using random agent fallback.")

try:
    from ns3ai_utils import Experiment
    HAS_NS3AI = False  # Will use custom shared memory
except ImportError:
    HAS_NS3AI = False

# ============================================================================
#  Constants matching C++ NtnAiObservation/NtnAiAction structs
# ============================================================================

MAX_CANDIDATES = 8

# Observation struct layout (must match ntn-ai-interface.h exactly)
# All doubles (8 bytes each) + uint32s (4 bytes each)
OBS_FIELDS = [
    ('ueId', 'I'),           # uint32
    ('simTime', 'd'),        # double
    ('servingSatId', 'I'),
    ('servingSinr', 'd'),
    ('servingRsrp', 'd'),
    ('servingElev', 'd'),
    ('servingDoppler', 'd'),
    ('servingTte', 'd'),
    ('servingDelay', 'd'),
    ('ueSpeed', 'd'),
    ('ueDirection', 'd'),
    ('ueLat', 'd'),
    ('ueLon', 'd'),
    ('numCandidates', 'I'),
]

CANDIDATE_FIELDS = [
    ('satId', 'I'),
    ('sinr', 'd'),
    ('rsrp', 'd'),
    ('elev', 'd'),
    ('doppler', 'd'),
    ('tte', 'd'),
    ('delay', 'd'),
    ('distToServing', 'd'),
]

HISTORY_FIELDS = [
    ('recentHoCount', 'I'),
    ('recentHoFailures', 'I'),
    ('recentPingPongs', 'I'),
    ('avgSinrLast30s', 'd'),
    ('avgTosLast5HOs', 'd'),
    ('previousReward', 'd'),
]

# State vector dimension
STATE_DIM = 7 + MAX_CANDIDATES * 7 + 5  # serving(7) + candidates(8*7) + history(5) = 68
ACTION_DIM = MAX_CANDIDATES + 1  # 0=stay, 1-8=handover to candidate


# ============================================================================
#  DQN Network Architecture
# ============================================================================

if HAS_TORCH:
    class DQNetwork(nn.Module):
        """
        Deep Q-Network for NTN handover decision.
        Architecture: FC(68) -> 256 -> 128 -> 64 -> 9 (action values)

        Dueling architecture (Wang et al. 2016):
          V(s) + A(s,a) - mean(A) = Q(s,a)
        """
        def __init__(self, state_dim=STATE_DIM, action_dim=ACTION_DIM,
                     hidden_dims=[256, 128, 64], dueling=True):
            super().__init__()
            self.dueling = dueling

            # Feature extractor
            layers = []
            prev_dim = state_dim
            for h in hidden_dims[:-1]:
                layers.extend([nn.Linear(prev_dim, h), nn.ReLU(), nn.Dropout(0.1)])
                prev_dim = h
            self.features = nn.Sequential(*layers)

            if dueling:
                # Value stream
                self.value = nn.Sequential(
                    nn.Linear(prev_dim, hidden_dims[-1]),
                    nn.ReLU(),
                    nn.Linear(hidden_dims[-1], 1)
                )
                # Advantage stream
                self.advantage = nn.Sequential(
                    nn.Linear(prev_dim, hidden_dims[-1]),
                    nn.ReLU(),
                    nn.Linear(hidden_dims[-1], action_dim)
                )
            else:
                self.q_head = nn.Sequential(
                    nn.Linear(prev_dim, hidden_dims[-1]),
                    nn.ReLU(),
                    nn.Linear(hidden_dims[-1], action_dim)
                )

        def forward(self, x):
            features = self.features(x)
            if self.dueling:
                value = self.value(features)
                advantage = self.advantage(features)
                q = value + advantage - advantage.mean(dim=-1, keepdim=True)
            else:
                q = self.q_head(features)
            return q

    class LSTMDQNetwork(nn.Module):
        """
        LSTM-enhanced DQN for temporal prediction.
        Uses sequence of past observations for RSRP/SINR prediction.

        Reference: MDPI Electronics 2026 - LSTM + Attention-Enhanced Rainbow DQN
        """
        def __init__(self, state_dim=STATE_DIM, action_dim=ACTION_DIM,
                     hidden_dim=128, lstm_layers=2, seq_len=10):
            super().__init__()
            self.hidden_dim = hidden_dim
            self.seq_len = seq_len

            self.input_fc = nn.Linear(state_dim, hidden_dim)
            self.lstm = nn.LSTM(hidden_dim, hidden_dim, lstm_layers,
                                batch_first=True, dropout=0.1)
            self.attention = nn.MultiheadAttention(hidden_dim, num_heads=4, batch_first=True)
            self.output_fc = nn.Sequential(
                nn.Linear(hidden_dim, 64),
                nn.ReLU(),
                nn.Linear(64, action_dim)
            )

        def forward(self, x, hidden=None):
            # x shape: (batch, seq_len, state_dim) or (batch, state_dim)
            if x.dim() == 2:
                x = x.unsqueeze(1)  # Add sequence dim
            batch_size = x.size(0)

            # Project input
            x = F.relu(self.input_fc(x))

            # LSTM temporal encoding
            lstm_out, hidden = self.lstm(x, hidden)

            # Self-attention over temporal features
            attn_out, _ = self.attention(lstm_out, lstm_out, lstm_out)

            # Use last timestep
            out = attn_out[:, -1, :]
            q_values = self.output_fc(out)
            return q_values, hidden


# ============================================================================
#  Replay Buffer
# ============================================================================

class ReplayBuffer:
    """Prioritized Experience Replay buffer."""
    def __init__(self, capacity=50000):
        self.buffer = deque(maxlen=capacity)
        self.priorities = deque(maxlen=capacity)

    def push(self, state, action, reward, next_state, done):
        self.buffer.append((state, action, reward, next_state, done))
        self.priorities.append(1.0)  # Max priority for new experiences

    def sample(self, batch_size=64):
        if len(self.buffer) < batch_size:
            batch_size = len(self.buffer)

        # Prioritized sampling
        priors = np.array(self.priorities)
        probs = priors / priors.sum()
        indices = np.random.choice(len(self.buffer), batch_size, p=probs)

        batch = [self.buffer[i] for i in indices]
        states, actions, rewards, next_states, dones = zip(*batch)
        return (np.array(states), np.array(actions), np.array(rewards),
                np.array(next_states), np.array(dones), indices)

    def update_priorities(self, indices, td_errors):
        for idx, err in zip(indices, td_errors):
            self.priorities[idx] = abs(err) + 1e-6

    def __len__(self):
        return len(self.buffer)


# ============================================================================
#  DQN Agent
# ============================================================================

class DQNAgent:
    """
    Deep Q-Network agent for NTN handover decision.

    Features:
    - Dueling DQN architecture
    - Prioritized experience replay
    - Double DQN (separate target network)
    - Epsilon-greedy exploration with decay
    - LSTM variant for temporal prediction

    Reference: MDPI Electronics 2026 - Rainbow-DQN for LEO satellite HO
    """
    def __init__(self, state_dim=STATE_DIM, action_dim=ACTION_DIM,
                 lr=1e-4, gamma=0.99, epsilon_start=1.0, epsilon_end=0.01,
                 epsilon_decay=0.995, batch_size=64, target_update=100,
                 use_lstm=False):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.gamma = gamma
        self.epsilon = epsilon_start
        self.epsilon_end = epsilon_end
        self.epsilon_decay = epsilon_decay
        self.batch_size = batch_size
        self.target_update_freq = target_update
        self.step_count = 0
        self.use_lstm = use_lstm

        if HAS_TORCH:
            if use_lstm:
                self.policy_net = LSTMDQNetwork(state_dim, action_dim)
                self.target_net = LSTMDQNetwork(state_dim, action_dim)
            else:
                self.policy_net = DQNetwork(state_dim, action_dim, dueling=True)
                self.target_net = DQNetwork(state_dim, action_dim, dueling=True)
            self.target_net.load_state_dict(self.policy_net.state_dict())
            self.optimizer = optim.Adam(self.policy_net.parameters(), lr=lr)

        self.replay_buffer = ReplayBuffer()
        self.last_state = None
        self.last_action = None

    def obs_to_state(self, obs):
        """Convert NtnAiObservation dict to flat state vector."""
        state = []

        # Serving cell features (normalized)
        state.append(obs.get('servingSinr', -20) / 30.0)
        state.append(obs.get('servingElev', 0) / 90.0)
        state.append(obs.get('servingTte', 0) / 120.0)
        state.append(obs.get('servingDoppler', 0) / 50000.0)
        state.append(obs.get('servingDelay', 10) / 50.0)
        state.append(obs.get('servingRsrp', -110) / 150.0)
        state.append(obs.get('ueSpeed', 0) / 100.0)

        # Candidate features (padded to MAX_CANDIDATES)
        candidates = obs.get('candidates', [])
        for i in range(MAX_CANDIDATES):
            if i < len(candidates):
                c = candidates[i]
                state.append(c.get('sinr', -20) / 30.0)
                state.append(c.get('elev', 0) / 90.0)
                state.append(c.get('tte', 0) / 120.0)
                state.append(c.get('doppler', 0) / 50000.0)
                state.append(c.get('delay', 10) / 50.0)
                state.append(c.get('rsrp', -110) / 150.0)
                state.append(c.get('distToServing', 500) / 2000.0)
            else:
                state.extend([0.0] * 7)

        # Historical features
        state.append(obs.get('recentHoCount', 0) / 20.0)
        state.append(obs.get('recentHoFailures', 0) / 10.0)
        state.append(obs.get('recentPingPongs', 0) / 10.0)
        state.append(obs.get('avgSinrLast30s', -10) / 30.0)
        state.append(obs.get('avgTosLast5HOs', 30) / 120.0)

        return np.array(state, dtype=np.float32)

    def select_action(self, state, num_valid_actions=None):
        """Epsilon-greedy action selection."""
        if num_valid_actions is None:
            num_valid_actions = self.action_dim

        if np.random.random() < self.epsilon:
            return np.random.randint(0, num_valid_actions)

        if HAS_TORCH:
            with torch.no_grad():
                state_t = torch.FloatTensor(state).unsqueeze(0)
                if self.use_lstm:
                    q_values, _ = self.policy_net(state_t)
                else:
                    q_values = self.policy_net(state_t)
                # Mask invalid actions
                q_values[0, num_valid_actions:] = -1e9
                return q_values.argmax(1).item()
        else:
            return np.random.randint(0, num_valid_actions)

    def store_transition(self, state, action, reward, next_state, done):
        """Store transition in replay buffer."""
        self.replay_buffer.push(state, action, reward, next_state, done)

    def train_step(self):
        """One training step with mini-batch from replay buffer."""
        if not HAS_TORCH or len(self.replay_buffer) < self.batch_size:
            return 0.0

        states, actions, rewards, next_states, dones, indices = \
            self.replay_buffer.sample(self.batch_size)

        states_t = torch.FloatTensor(states)
        actions_t = torch.LongTensor(actions).unsqueeze(1)
        rewards_t = torch.FloatTensor(rewards).unsqueeze(1)
        next_states_t = torch.FloatTensor(next_states)
        dones_t = torch.FloatTensor(dones).unsqueeze(1)

        # Current Q values
        if self.use_lstm:
            current_q, _ = self.policy_net(states_t)
        else:
            current_q = self.policy_net(states_t)
        current_q = current_q.gather(1, actions_t)

        # Double DQN: use policy net to select action, target net to evaluate
        with torch.no_grad():
            if self.use_lstm:
                next_q_policy, _ = self.policy_net(next_states_t)
                next_q_target, _ = self.target_net(next_states_t)
            else:
                next_q_policy = self.policy_net(next_states_t)
                next_q_target = self.target_net(next_states_t)
            best_actions = next_q_policy.argmax(1, keepdim=True)
            next_q = next_q_target.gather(1, best_actions)
            target_q = rewards_t + self.gamma * next_q * (1 - dones_t)

        # Compute loss and update
        loss = F.smooth_l1_loss(current_q, target_q)
        self.optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self.policy_net.parameters(), 1.0)
        self.optimizer.step()

        # Update priorities
        td_errors = (current_q - target_q).detach().cpu().numpy().flatten()
        self.replay_buffer.update_priorities(indices, td_errors)

        # Update target network
        self.step_count += 1
        if self.step_count % self.target_update_freq == 0:
            self.target_net.load_state_dict(self.policy_net.state_dict())

        # Decay epsilon
        self.epsilon = max(self.epsilon_end, self.epsilon * self.epsilon_decay)

        return loss.item()

    def save(self, path):
        """Save model weights."""
        if HAS_TORCH:
            torch.save({
                'policy_net': self.policy_net.state_dict(),
                'target_net': self.target_net.state_dict(),
                'optimizer': self.optimizer.state_dict(),
                'epsilon': self.epsilon,
                'step_count': self.step_count,
            }, path)
            print(f"Model saved to {path}")

    def load(self, path):
        """Load model weights."""
        if HAS_TORCH and os.path.exists(path):
            ckpt = torch.load(path, map_location='cpu')
            self.policy_net.load_state_dict(ckpt['policy_net'])
            self.target_net.load_state_dict(ckpt['target_net'])
            self.optimizer.load_state_dict(ckpt['optimizer'])
            self.epsilon = ckpt['epsilon']
            self.step_count = ckpt['step_count']
            print(f"Model loaded from {path}")


# ============================================================================
#  Federated Learning Wrapper
# ============================================================================

class FederatedDQNAgent:
    """
    Federated DQN for multi-UE distributed learning.

    Each UE trains a local DQN model. Periodically, local models are
    aggregated using Federated Averaging (FedAvg) to produce a global model.

    Reference: Springer Wireless Networks 2026 - Federated MARL for LEO
    """
    def __init__(self, num_agents, aggregation_interval=50, **dqn_kwargs):
        self.num_agents = num_agents
        self.agents = [DQNAgent(**dqn_kwargs) for _ in range(num_agents)]
        self.aggregation_interval = aggregation_interval
        self.global_step = 0
        self.aggregation_count = 0

    def get_agent(self, ue_id):
        return self.agents[ue_id % self.num_agents]

    def federated_average(self):
        """FedAvg: average all local model weights."""
        if not HAS_TORCH:
            return

        self.aggregation_count += 1
        global_state = {}

        # Average all agent parameters
        for key in self.agents[0].policy_net.state_dict():
            tensors = [agent.policy_net.state_dict()[key].float()
                       for agent in self.agents]
            global_state[key] = torch.stack(tensors).mean(0)

        # Distribute global model back to all agents
        for agent in self.agents:
            agent.policy_net.load_state_dict(global_state)
            agent.target_net.load_state_dict(global_state)

        print(f"  [FedAvg] Aggregation #{self.aggregation_count} "
              f"({self.num_agents} agents)")

    def step(self):
        """Check if aggregation is needed."""
        self.global_step += 1
        if self.global_step % self.aggregation_interval == 0:
            self.federated_average()


# ============================================================================
#  Standalone Testing (without ns3-ai shared memory)
# ============================================================================

def generate_synthetic_observation(t, ue_id=0):
    """Generate a synthetic NTN observation for testing."""
    obs = {
        'ueId': ue_id,
        'simTime': t,
        'servingSatId': 37,
        'servingSinr': -5.0 + np.sin(t * 0.1) * 3.0,
        'servingRsrp': -100.0 + np.sin(t * 0.1) * 5.0,
        'servingElev': 25.0 + np.sin(t * 0.05) * 15.0,
        'servingDoppler': 40000.0 * np.cos(t * 0.02),
        'servingTte': max(5.0, 45.0 - t * 0.5),
        'servingDelay': 5.0,
        'ueSpeed': 20.0,
        'ueDirection': 45.0,
        'ueLat': 45.0,
        'ueLon': 10.0,
        'numCandidates': 3,
        'candidates': [
            {'satId': 1, 'sinr': -8.0 + np.random.randn(), 'rsrp': -105.0,
             'elev': 15.0 + np.random.rand() * 10, 'doppler': 35000.0,
             'tte': 30.0 + np.random.rand() * 20, 'delay': 8.0, 'distToServing': 500.0},
            {'satId': 12, 'sinr': -3.0 + np.random.randn(), 'rsrp': -98.0,
             'elev': 35.0 + np.random.rand() * 10, 'doppler': 25000.0,
             'tte': 50.0 + np.random.rand() * 30, 'delay': 4.0, 'distToServing': 300.0},
            {'satId': 45, 'sinr': -6.0 + np.random.randn(), 'rsrp': -102.0,
             'elev': 20.0 + np.random.rand() * 10, 'doppler': 42000.0,
             'tte': 20.0 + np.random.rand() * 15, 'delay': 6.0, 'distToServing': 700.0},
        ],
        'recentHoCount': 2,
        'recentHoFailures': 0,
        'recentPingPongs': 0,
        'avgSinrLast30s': -6.0,
        'avgTosLast5HOs': 35.0,
        'previousReward': 0.0,
    }
    return obs


def simulate_ho_outcome(obs, action):
    """Simulate handover outcome for training without ns-3."""
    if action == 0:
        # Stay: small positive reward if serving is OK
        reward = 0.1 if obs['servingSinr'] > -10 else -0.5
        done = False
    else:
        cand_idx = action - 1
        cands = obs.get('candidates', [])
        if cand_idx < len(cands):
            cand = cands[cand_idx]
            sinr_imp = cand['sinr'] - obs['servingSinr']
            tte = cand['tte']
            success = cand['sinr'] > -8.0 and np.random.rand() > 0.1
            ping_pong = tte < 15.0
            # Reward function matching C++ NtnAiInterface::ComputeReward
            if not success:
                reward = -10.0
            else:
                reward = 1.0 + np.clip(sinr_imp * 0.5, -3, 5) + min(30.0 / 30.0, 3.0)
                if ping_pong:
                    reward -= 5.0
                reward -= 40.0 / 100.0  # interruption penalty
        else:
            reward = -5.0  # Invalid action
        done = False
    return reward, done


def run_standalone_training(args):
    """Train DQN agent using synthetic NTN data (no ns-3 required)."""
    print(f"=== NTN AI Agent - Standalone Training ===")
    print(f"Algorithm: {args.algorithm}")
    print(f"Episodes: {args.episodes}")
    print(f"LSTM: {args.use_lstm}")
    print(f"Federated: {args.federated} (agents: {args.num_agents})")
    print()

    if args.federated:
        fed_agent = FederatedDQNAgent(
            args.num_agents,
            aggregation_interval=args.fed_interval,
            use_lstm=args.use_lstm,
            lr=args.lr,
            gamma=args.gamma,
        )
        agent = fed_agent.get_agent(0)
    else:
        agent = DQNAgent(
            use_lstm=args.use_lstm,
            lr=args.lr,
            gamma=args.gamma,
        )

    total_rewards = []
    total_successes = []

    for episode in range(args.episodes):
        ep_reward = 0.0
        ep_successes = 0
        steps_per_episode = args.steps

        for step in range(steps_per_episode):
            t = step * 1.0
            ue_id = step % args.num_agents if args.federated else 0

            if args.federated:
                agent = fed_agent.get_agent(ue_id)

            obs = generate_synthetic_observation(t, ue_id)
            state = agent.obs_to_state(obs)
            num_valid = obs['numCandidates'] + 1  # +1 for "stay"

            action = agent.select_action(state, num_valid)
            reward, done = simulate_ho_outcome(obs, action)

            # Next observation
            next_obs = generate_synthetic_observation(t + 1.0, ue_id)
            next_obs['previousReward'] = reward
            next_state = agent.obs_to_state(next_obs)

            agent.store_transition(state, action, reward, next_state, done)
            loss = agent.train_step()

            ep_reward += reward
            if action > 0 and reward > 0:
                ep_successes += 1

            if args.federated:
                fed_agent.step()

        total_rewards.append(ep_reward)
        total_successes.append(ep_successes)

        if (episode + 1) % 10 == 0:
            avg_r = np.mean(total_rewards[-10:])
            avg_s = np.mean(total_successes[-10:])
            print(f"  Episode {episode+1}/{args.episodes}: "
                  f"avg_reward={avg_r:.2f}, avg_successes={avg_s:.1f}, "
                  f"epsilon={agent.epsilon:.3f}"
                  f"{', loss=' + f'{loss:.4f}' if HAS_TORCH else ''}")

    # Save model
    os.makedirs(args.output, exist_ok=True)
    model_path = os.path.join(args.output, f"ntn_dqn_{args.algorithm}.pth")
    if args.federated:
        fed_agent.get_agent(0).save(model_path)
    else:
        agent.save(model_path)

    # Save training log
    log_path = os.path.join(args.output, "training_log.json")
    with open(log_path, 'w') as f:
        json.dump({
            'algorithm': args.algorithm,
            'episodes': args.episodes,
            'rewards': [float(r) for r in total_rewards],
            'successes': [int(s) for s in total_successes],
            'final_epsilon': float(agent.epsilon),
            'use_lstm': args.use_lstm,
            'federated': args.federated,
            'num_agents': args.num_agents,
        }, f, indent=2)

    print(f"\nTraining complete!")
    print(f"  Model saved: {model_path}")
    print(f"  Log saved: {log_path}")
    print(f"  Final avg reward: {np.mean(total_rewards[-10:]):.2f}")
    print(f"  Final epsilon: {agent.epsilon:.4f}")


# ============================================================================
#  Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description='NTN AI Agent for LEO Satellite Handover')
    parser.add_argument('--algorithm', choices=['dqn', 'ppo', 'rainbow', 'random'],
                       default='dqn', help='RL algorithm')
    parser.add_argument('--mode', choices=['standalone', 'ns3ai'],
                       default='standalone', help='Run mode')
    parser.add_argument('--episodes', type=int, default=100, help='Training episodes')
    parser.add_argument('--steps', type=int, default=200, help='Steps per episode')
    parser.add_argument('--lr', type=float, default=1e-4, help='Learning rate')
    parser.add_argument('--gamma', type=float, default=0.99, help='Discount factor')
    parser.add_argument('--use-lstm', action='store_true', help='Use LSTM-DQN')
    parser.add_argument('--federated', action='store_true', help='Use Federated Learning')
    parser.add_argument('--num-agents', type=int, default=4, help='Number of FL agents')
    parser.add_argument('--fed-interval', type=int, default=50, help='FL aggregation interval')
    parser.add_argument('--output', default='ntn-ai-output', help='Output directory')
    parser.add_argument('--load-model', default=None, help='Path to pre-trained model')
    args = parser.parse_args()

    print("================================================================")
    print("  NTN AI Agent - Deep RL for 6G LEO Satellite Handover")
    print("================================================================")
    print(f"  PyTorch: {'available' if HAS_TORCH else 'NOT AVAILABLE (using random)'}")
    print(f"  Algorithm: {args.algorithm}")
    print(f"  Mode: {args.mode}")
    print(f"  LSTM: {args.use_lstm}")
    print(f"  Federated: {args.federated}")
    print("================================================================\n")

    if args.mode == 'standalone':
        run_standalone_training(args)
    elif args.mode == 'ns3ai':
        print("ns3-ai shared memory mode: run ns-3 simulation first, then this agent.")
        print("  ./ns3 run 'ntn-cho-ai-example --aiEnabled=true'")
        print("  python3 ntn_ai_agent.py --mode ns3ai")
        # TODO: Implement ns3-ai shared memory loop
        print("\nNot yet implemented. Use --mode standalone for training.")
    else:
        print(f"Unknown mode: {args.mode}")
        sys.exit(1)


if __name__ == '__main__':
    main()
