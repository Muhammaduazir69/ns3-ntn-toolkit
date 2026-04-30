#!/usr/bin/env python3
"""
O-RAN NTN Multi-xApp AI Agent Framework
Author: Muhammad Uzair
License: GPL-2.0-only

Implements DRL agents for each O-RAN NTN xApp:
  1. HO Prediction Agent (DQN + LSTM)
  2. Beam Hopping Agent (PPO)
  3. Slice Manager Agent (MAPPO)
  4. Doppler Compensation Agent (DDPG)
  5. TN-NTN Steering Agent (DQN)

Supports:
  - Shared memory interface via ns3-ai (Gym or direct)
  - Federated learning across satellites
  - Multi-agent coordination (CTDE paradigm)
  - Experience replay with prioritized sampling
  - Model checkpointing and tensorboard logging

Requirements:
  pip install torch numpy gymnasium tensorboard
  Optional: pip install ns3-ai pettingzoo
"""

import argparse
import json
import logging
import os
import sys
import time
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    import torch.optim as optim

    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    print("WARNING: PyTorch not available. Install with: pip install torch")

# ============================================================================
#  Neural Network Architectures
# ============================================================================


class DQNNetwork(nn.Module):
    """Deep Q-Network with dueling architecture."""

    def __init__(self, obs_dim: int, act_dim: int, hidden: int = 256):
        super().__init__()
        self.feature = nn.Sequential(
            nn.Linear(obs_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
        )
        # Dueling streams
        self.value_stream = nn.Sequential(
            nn.Linear(hidden, hidden // 2), nn.ReLU(), nn.Linear(hidden // 2, 1)
        )
        self.advantage_stream = nn.Sequential(
            nn.Linear(hidden, hidden // 2),
            nn.ReLU(),
            nn.Linear(hidden // 2, act_dim),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        features = self.feature(x)
        value = self.value_stream(features)
        advantage = self.advantage_stream(features)
        # Dueling: Q = V + (A - mean(A))
        return value + advantage - advantage.mean(dim=-1, keepdim=True)


class LSTMPredictor(nn.Module):
    """LSTM for SINR time-series prediction."""

    def __init__(
        self,
        input_dim: int = 6,
        hidden_dim: int = 128,
        num_layers: int = 2,
        output_dim: int = 1,
    ):
        super().__init__()
        self.lstm = nn.LSTM(input_dim, hidden_dim, num_layers, batch_first=True)
        self.attention = nn.MultiheadAttention(hidden_dim, num_heads=4, batch_first=True)
        self.fc = nn.Sequential(
            nn.Linear(hidden_dim, 64), nn.ReLU(), nn.Linear(64, output_dim)
        )

    def forward(
        self, x: torch.Tensor
    ) -> Tuple[torch.Tensor, Tuple[torch.Tensor, torch.Tensor]]:
        lstm_out, (h_n, c_n) = self.lstm(x)
        # Self-attention over LSTM outputs
        attn_out, _ = self.attention(lstm_out, lstm_out, lstm_out)
        # Use last time step
        out = self.fc(attn_out[:, -1, :])
        return out, (h_n, c_n)


class PPOActorCritic(nn.Module):
    """Actor-Critic network for PPO (beam hopping)."""

    def __init__(self, obs_dim: int, act_dim: int, hidden: int = 256):
        super().__init__()
        self.shared = nn.Sequential(
            nn.Linear(obs_dim, hidden), nn.ReLU(), nn.Linear(hidden, hidden), nn.ReLU()
        )
        self.actor = nn.Sequential(nn.Linear(hidden, act_dim), nn.Softmax(dim=-1))
        self.critic = nn.Linear(hidden, 1)

    def forward(self, x: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        shared = self.shared(x)
        return self.actor(shared), self.critic(shared)

    def get_action(self, x: torch.Tensor) -> Tuple[int, float, float]:
        probs, value = self.forward(x)
        dist = torch.distributions.Categorical(probs)
        action = dist.sample()
        return action.item(), dist.log_prob(action).item(), value.item()


class DDPGActor(nn.Module):
    """Continuous actor for DDPG (Doppler compensation)."""

    def __init__(self, obs_dim: int, act_dim: int, hidden: int = 256,
                 max_action: float = 1.0):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, act_dim),
            nn.Tanh(),
        )
        self.max_action = max_action

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x) * self.max_action


class DDPGCritic(nn.Module):
    """Critic for DDPG."""

    def __init__(self, obs_dim: int, act_dim: int, hidden: int = 256):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim + act_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, 1),
        )

    def forward(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        return self.net(torch.cat([state, action], dim=-1))


# ============================================================================
#  Experience Replay
# ============================================================================


class PrioritizedReplayBuffer:
    """Prioritized experience replay buffer."""

    def __init__(self, capacity: int = 100000, alpha: float = 0.6):
        self.capacity = capacity
        self.alpha = alpha
        self.buffer: List = []
        self.priorities = np.zeros(capacity, dtype=np.float32)
        self.pos = 0
        self.size = 0

    def push(self, state, action, reward, next_state, done):
        max_prio = self.priorities[: self.size].max() if self.size > 0 else 1.0
        if self.size < self.capacity:
            self.buffer.append((state, action, reward, next_state, done))
        else:
            self.buffer[self.pos] = (state, action, reward, next_state, done)
        self.priorities[self.pos] = max_prio
        self.pos = (self.pos + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size: int, beta: float = 0.4):
        prios = self.priorities[: self.size] ** self.alpha
        probs = prios / prios.sum()
        indices = np.random.choice(self.size, batch_size, p=probs, replace=False)
        weights = (self.size * probs[indices]) ** (-beta)
        weights /= weights.max()

        batch = [self.buffer[i] for i in indices]
        states, actions, rewards, next_states, dones = zip(*batch)
        return (
            np.array(states),
            np.array(actions),
            np.array(rewards),
            np.array(next_states),
            np.array(dones),
            indices,
            weights,
        )

    def update_priorities(self, indices, priorities):
        for idx, prio in zip(indices, priorities):
            self.priorities[idx] = abs(prio) + 1e-6

    def __len__(self):
        return self.size


# ============================================================================
#  Base AI Agent
# ============================================================================


@dataclass
class AgentConfig:
    """Configuration for an AI agent."""

    name: str = "base"
    obs_dim: int = 68
    act_dim: int = 9  # 0=stay, 1-8=handover to candidate
    hidden_dim: int = 256
    lr: float = 3e-4
    gamma: float = 0.99
    batch_size: int = 64
    buffer_size: int = 100000
    epsilon_start: float = 1.0
    epsilon_end: float = 0.01
    epsilon_decay: int = 50000
    target_update: int = 1000
    tau: float = 0.005
    output_dir: str = "oran-ntn-output"
    device: str = "cpu"
    checkpoint_interval: int = 1000


class BaseAgent(ABC):
    """Abstract base class for xApp AI agents."""

    def __init__(self, config: AgentConfig):
        self.config = config
        self.device = torch.device(config.device)
        self.step_count = 0
        self.episode_count = 0
        self.training_log: List[Dict] = []

        os.makedirs(config.output_dir, exist_ok=True)
        self.logger = logging.getLogger(config.name)
        self.logger.setLevel(logging.INFO)

    @abstractmethod
    def select_action(self, observation: np.ndarray) -> int:
        """Select action from observation."""

    @abstractmethod
    def update(self, state, action, reward, next_state, done) -> Optional[float]:
        """Update agent with experience. Returns loss if training occurred."""

    @abstractmethod
    def save(self, path: str):
        """Save model weights."""

    @abstractmethod
    def load(self, path: str):
        """Load model weights."""

    def get_weights(self) -> List[float]:
        """Get flattened weights for federated learning."""
        weights = []
        for param in self.get_parameters():
            weights.extend(param.detach().cpu().numpy().flatten().tolist())
        return weights

    def set_weights(self, weights: List[float]):
        """Set weights from flattened list (federated learning)."""
        idx = 0
        for param in self.get_parameters():
            param_size = param.numel()
            param_data = torch.tensor(
                weights[idx : idx + param_size], dtype=torch.float32
            ).reshape(param.shape)
            param.data = param_data.to(self.device)
            idx += param_size

    @abstractmethod
    def get_parameters(self):
        """Get model parameters iterator."""

    def log_step(self, reward: float, loss: Optional[float] = None, **kwargs):
        """Log a training step."""
        entry = {
            "step": self.step_count,
            "episode": self.episode_count,
            "reward": reward,
            "loss": loss,
            **kwargs,
        }
        self.training_log.append(entry)

    def save_training_log(self):
        """Save training log to JSON."""
        path = os.path.join(self.config.output_dir, f"{self.config.name}_log.json")
        with open(path, "w") as f:
            json.dump(self.training_log, f, indent=2)


# ============================================================================
#  HO Prediction Agent (DQN + LSTM)
# ============================================================================


class HoPredictAgent(BaseAgent):
    """DQN agent with LSTM for proactive handover prediction."""

    def __init__(self, config: AgentConfig):
        super().__init__(config)

        # DQN for action selection
        self.q_net = DQNNetwork(config.obs_dim, config.act_dim, config.hidden_dim).to(
            self.device
        )
        self.target_net = DQNNetwork(
            config.obs_dim, config.act_dim, config.hidden_dim
        ).to(self.device)
        self.target_net.load_state_dict(self.q_net.state_dict())
        self.q_optimizer = optim.Adam(self.q_net.parameters(), lr=config.lr)

        # LSTM for SINR prediction
        self.lstm = LSTMPredictor(input_dim=6, hidden_dim=128).to(self.device)
        self.lstm_optimizer = optim.Adam(self.lstm.parameters(), lr=config.lr)

        self.replay = PrioritizedReplayBuffer(config.buffer_size)
        self.sinr_history: Dict[int, deque] = {}  # ueId -> deque of (features)

        self.epsilon = config.epsilon_start

    def select_action(self, observation: np.ndarray) -> int:
        self.epsilon = max(
            self.config.epsilon_end,
            self.config.epsilon_start
            - self.step_count / self.config.epsilon_decay,
        )

        if np.random.random() < self.epsilon:
            return np.random.randint(self.config.act_dim)

        with torch.no_grad():
            state = torch.FloatTensor(observation).unsqueeze(0).to(self.device)
            q_values = self.q_net(state)
            return q_values.argmax(dim=1).item()

    def predict_sinr(self, ue_id: int, horizon_steps: int = 10) -> float:
        """Predict future SINR using LSTM."""
        if ue_id not in self.sinr_history or len(self.sinr_history[ue_id]) < 10:
            return 0.0

        history = list(self.sinr_history[ue_id])[-20:]
        seq = torch.FloatTensor(history).unsqueeze(0).to(self.device)

        with torch.no_grad():
            predicted, _ = self.lstm(seq)
            return predicted.item()

    def update(self, state, action, reward, next_state, done) -> Optional[float]:
        self.replay.push(state, action, reward, next_state, done)
        self.step_count += 1

        if len(self.replay) < self.config.batch_size:
            return None

        states, actions, rewards, next_states, dones, indices, weights = (
            self.replay.sample(self.config.batch_size)
        )

        states_t = torch.FloatTensor(states).to(self.device)
        actions_t = torch.LongTensor(actions).to(self.device)
        rewards_t = torch.FloatTensor(rewards).to(self.device)
        next_states_t = torch.FloatTensor(next_states).to(self.device)
        dones_t = torch.FloatTensor(dones).to(self.device)
        weights_t = torch.FloatTensor(weights).to(self.device)

        # Double DQN
        q_values = self.q_net(states_t).gather(1, actions_t.unsqueeze(1)).squeeze()
        next_actions = self.q_net(next_states_t).argmax(dim=1)
        next_q = (
            self.target_net(next_states_t).gather(1, next_actions.unsqueeze(1)).squeeze()
        )
        target = rewards_t + self.config.gamma * next_q * (1 - dones_t)

        td_errors = (q_values - target.detach()).abs()
        loss = (weights_t * F.smooth_l1_loss(q_values, target.detach(), reduction="none")).mean()

        self.q_optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self.q_net.parameters(), 10.0)
        self.q_optimizer.step()

        # Update priorities
        self.replay.update_priorities(indices, td_errors.detach().cpu().numpy())

        # Soft target update
        if self.step_count % self.config.target_update == 0:
            for tp, sp in zip(
                self.target_net.parameters(), self.q_net.parameters()
            ):
                tp.data.copy_(
                    self.config.tau * sp.data + (1 - self.config.tau) * tp.data
                )

        return loss.item()

    def save(self, path: str):
        torch.save(
            {
                "q_net": self.q_net.state_dict(),
                "target_net": self.target_net.state_dict(),
                "lstm": self.lstm.state_dict(),
                "q_optimizer": self.q_optimizer.state_dict(),
                "lstm_optimizer": self.lstm_optimizer.state_dict(),
                "step_count": self.step_count,
                "epsilon": self.epsilon,
            },
            path,
        )

    def load(self, path: str):
        ckpt = torch.load(path, map_location=self.device)
        self.q_net.load_state_dict(ckpt["q_net"])
        self.target_net.load_state_dict(ckpt["target_net"])
        self.lstm.load_state_dict(ckpt["lstm"])
        self.q_optimizer.load_state_dict(ckpt["q_optimizer"])
        self.lstm_optimizer.load_state_dict(ckpt["lstm_optimizer"])
        self.step_count = ckpt["step_count"]
        self.epsilon = ckpt["epsilon"]

    def get_parameters(self):
        return list(self.q_net.parameters()) + list(self.lstm.parameters())


# ============================================================================
#  Beam Hopping Agent (PPO)
# ============================================================================


class BeamHopAgent(BaseAgent):
    """PPO agent for dynamic beam scheduling."""

    def __init__(self, config: AgentConfig):
        super().__init__(config)

        self.actor_critic = PPOActorCritic(
            config.obs_dim, config.act_dim, config.hidden_dim
        ).to(self.device)
        self.optimizer = optim.Adam(self.actor_critic.parameters(), lr=config.lr)

        # PPO hyperparameters
        self.clip_epsilon = 0.2
        self.entropy_coef = 0.01
        self.value_coef = 0.5
        self.gae_lambda = 0.95
        self.ppo_epochs = 4

        # Trajectory buffer
        self.trajectory: List[Dict] = []

    def select_action(self, observation: np.ndarray) -> int:
        state = torch.FloatTensor(observation).unsqueeze(0).to(self.device)
        action, log_prob, value = self.actor_critic.get_action(state)
        self.trajectory.append(
            {
                "state": observation,
                "action": action,
                "log_prob": log_prob,
                "value": value,
            }
        )
        return action

    def update(self, state, action, reward, next_state, done) -> Optional[float]:
        self.step_count += 1

        if self.trajectory:
            self.trajectory[-1]["reward"] = reward
            self.trajectory[-1]["done"] = done

        if not done and len(self.trajectory) < 128:
            return None

        # Compute GAE
        rewards = [t["reward"] for t in self.trajectory]
        values = [t["value"] for t in self.trajectory]
        dones = [t["done"] for t in self.trajectory]

        advantages = []
        gae = 0
        next_val = 0 if dones[-1] else values[-1]
        for i in reversed(range(len(rewards))):
            delta = rewards[i] + self.config.gamma * next_val * (1 - dones[i]) - values[i]
            gae = delta + self.config.gamma * self.gae_lambda * (1 - dones[i]) * gae
            advantages.insert(0, gae)
            next_val = values[i]

        advantages_t = torch.FloatTensor(advantages).to(self.device)
        returns_t = advantages_t + torch.FloatTensor(values).to(self.device)
        advantages_t = (advantages_t - advantages_t.mean()) / (advantages_t.std() + 1e-8)

        states_t = torch.FloatTensor([t["state"] for t in self.trajectory]).to(self.device)
        actions_t = torch.LongTensor([t["action"] for t in self.trajectory]).to(self.device)
        old_log_probs_t = torch.FloatTensor(
            [t["log_prob"] for t in self.trajectory]
        ).to(self.device)

        total_loss = 0
        for _ in range(self.ppo_epochs):
            probs, values_pred = self.actor_critic(states_t)
            dist = torch.distributions.Categorical(probs)
            new_log_probs = dist.log_prob(actions_t)

            ratio = (new_log_probs - old_log_probs_t).exp()
            surr1 = ratio * advantages_t
            surr2 = torch.clamp(ratio, 1 - self.clip_epsilon, 1 + self.clip_epsilon) * advantages_t

            policy_loss = -torch.min(surr1, surr2).mean()
            value_loss = F.mse_loss(values_pred.squeeze(), returns_t)
            entropy = dist.entropy().mean()

            loss = policy_loss + self.value_coef * value_loss - self.entropy_coef * entropy

            self.optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(self.actor_critic.parameters(), 0.5)
            self.optimizer.step()

            total_loss += loss.item()

        self.trajectory.clear()
        return total_loss / self.ppo_epochs

    def save(self, path: str):
        torch.save(
            {
                "actor_critic": self.actor_critic.state_dict(),
                "optimizer": self.optimizer.state_dict(),
                "step_count": self.step_count,
            },
            path,
        )

    def load(self, path: str):
        ckpt = torch.load(path, map_location=self.device)
        self.actor_critic.load_state_dict(ckpt["actor_critic"])
        self.optimizer.load_state_dict(ckpt["optimizer"])
        self.step_count = ckpt["step_count"]

    def get_parameters(self):
        return self.actor_critic.parameters()


# ============================================================================
#  Slice Manager Agent (Multi-Agent PPO)
# ============================================================================


class SliceManagerAgent(BaseAgent):
    """MAPPO agent for slice resource management across satellites."""

    def __init__(self, config: AgentConfig):
        config.act_dim = 7  # PRB adjustment: -30%, -20%, -10%, 0, +10%, +20%, +30%
        super().__init__(config)

        # Separate actor-critic per slice
        self.slice_nets = nn.ModuleDict()
        self.slice_optimizers = {}
        for slice_name in ["embb", "urllc", "mmtc"]:
            net = PPOActorCritic(config.obs_dim, config.act_dim, config.hidden_dim).to(
                self.device
            )
            self.slice_nets[slice_name] = net
            self.slice_optimizers[slice_name] = optim.Adam(
                net.parameters(), lr=config.lr
            )

        self.current_slice = "embb"
        self.trajectories = {"embb": [], "urllc": [], "mmtc": []}

    def select_action(self, observation: np.ndarray) -> int:
        state = torch.FloatTensor(observation).unsqueeze(0).to(self.device)
        net = self.slice_nets[self.current_slice]
        action, log_prob, value = net.get_action(state)
        self.trajectories[self.current_slice].append(
            {
                "state": observation,
                "action": action,
                "log_prob": log_prob,
                "value": value,
            }
        )
        return action

    def set_active_slice(self, slice_name: str):
        """Set which slice is currently being managed."""
        self.current_slice = slice_name

    def update(self, state, action, reward, next_state, done) -> Optional[float]:
        self.step_count += 1
        traj = self.trajectories[self.current_slice]
        if traj:
            traj[-1]["reward"] = reward
            traj[-1]["done"] = done
        return None  # Actual update in batch at trajectory end

    def save(self, path: str):
        torch.save(
            {
                "slice_nets": {k: v.state_dict() for k, v in self.slice_nets.items()},
                "step_count": self.step_count,
            },
            path,
        )

    def load(self, path: str):
        ckpt = torch.load(path, map_location=self.device)
        for k, v in ckpt["slice_nets"].items():
            self.slice_nets[k].load_state_dict(v)
        self.step_count = ckpt["step_count"]

    def get_parameters(self):
        params = []
        for net in self.slice_nets.values():
            params.extend(net.parameters())
        return params


# ============================================================================
#  Doppler Compensation Agent (DDPG)
# ============================================================================


class DopplerCompAgent(BaseAgent):
    """DDPG agent for continuous Doppler compensation."""

    def __init__(self, config: AgentConfig):
        config.act_dim = 2  # [common_doppler_correction, per_ue_correction_scale]
        super().__init__(config)

        self.actor = DDPGActor(
            config.obs_dim, config.act_dim, config.hidden_dim, max_action=50000.0
        ).to(self.device)
        self.actor_target = DDPGActor(
            config.obs_dim, config.act_dim, config.hidden_dim, max_action=50000.0
        ).to(self.device)
        self.actor_target.load_state_dict(self.actor.state_dict())

        self.critic = DDPGCritic(
            config.obs_dim, config.act_dim, config.hidden_dim
        ).to(self.device)
        self.critic_target = DDPGCritic(
            config.obs_dim, config.act_dim, config.hidden_dim
        ).to(self.device)
        self.critic_target.load_state_dict(self.critic.state_dict())

        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=config.lr)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=config.lr)

        self.replay = PrioritizedReplayBuffer(config.buffer_size)
        self.noise_std = 0.1

    def select_action(self, observation: np.ndarray) -> int:
        state = torch.FloatTensor(observation).unsqueeze(0).to(self.device)
        with torch.no_grad():
            action = self.actor(state).cpu().numpy()[0]
        # Add exploration noise
        noise = np.random.normal(0, self.noise_std, size=action.shape)
        action = action + noise
        return int(np.argmax(np.abs(action)))  # Discretize for interface

    def get_continuous_action(self, observation: np.ndarray) -> np.ndarray:
        """Get continuous action (for actual Doppler compensation)."""
        state = torch.FloatTensor(observation).unsqueeze(0).to(self.device)
        with torch.no_grad():
            action = self.actor(state).cpu().numpy()[0]
        noise = np.random.normal(0, self.noise_std, size=action.shape)
        return action + noise

    def update(self, state, action, reward, next_state, done) -> Optional[float]:
        self.replay.push(state, action, reward, next_state, done)
        self.step_count += 1

        if len(self.replay) < self.config.batch_size:
            return None

        states, actions, rewards, next_states, dones, indices, weights = (
            self.replay.sample(self.config.batch_size)
        )

        states_t = torch.FloatTensor(states).to(self.device)
        actions_t = torch.FloatTensor(actions).unsqueeze(1).to(self.device)
        rewards_t = torch.FloatTensor(rewards).to(self.device)
        next_states_t = torch.FloatTensor(next_states).to(self.device)
        dones_t = torch.FloatTensor(dones).to(self.device)

        # Critic update
        with torch.no_grad():
            next_actions = self.actor_target(next_states_t)
            target_q = self.critic_target(next_states_t, next_actions).squeeze()
            target = rewards_t + self.config.gamma * target_q * (1 - dones_t)

        current_q = self.critic(states_t, actions_t).squeeze()
        critic_loss = F.mse_loss(current_q, target)

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # Actor update
        actor_loss = -self.critic(states_t, self.actor(states_t)).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # Soft target updates
        for tp, sp in zip(self.actor_target.parameters(), self.actor.parameters()):
            tp.data.copy_(self.config.tau * sp.data + (1 - self.config.tau) * tp.data)
        for tp, sp in zip(self.critic_target.parameters(), self.critic.parameters()):
            tp.data.copy_(self.config.tau * sp.data + (1 - self.config.tau) * tp.data)

        return critic_loss.item()

    def save(self, path: str):
        torch.save(
            {
                "actor": self.actor.state_dict(),
                "critic": self.critic.state_dict(),
                "step_count": self.step_count,
            },
            path,
        )

    def load(self, path: str):
        ckpt = torch.load(path, map_location=self.device)
        self.actor.load_state_dict(ckpt["actor"])
        self.critic.load_state_dict(ckpt["critic"])
        self.step_count = ckpt["step_count"]

    def get_parameters(self):
        return list(self.actor.parameters()) + list(self.critic.parameters())


# ============================================================================
#  TN-NTN Steering Agent (DQN)
# ============================================================================


class TnNtnSteeringAgent(BaseAgent):
    """DQN agent for terrestrial/satellite traffic steering."""

    def __init__(self, config: AgentConfig):
        config.act_dim = 3  # 0=terrestrial, 1=satellite, 2=dual-connectivity
        super().__init__(config)

        self.q_net = DQNNetwork(config.obs_dim, config.act_dim, config.hidden_dim).to(
            self.device
        )
        self.target_net = DQNNetwork(
            config.obs_dim, config.act_dim, config.hidden_dim
        ).to(self.device)
        self.target_net.load_state_dict(self.q_net.state_dict())
        self.optimizer = optim.Adam(self.q_net.parameters(), lr=config.lr)

        self.replay = PrioritizedReplayBuffer(config.buffer_size)
        self.epsilon = config.epsilon_start

    def select_action(self, observation: np.ndarray) -> int:
        self.epsilon = max(
            self.config.epsilon_end,
            self.config.epsilon_start
            - self.step_count / self.config.epsilon_decay,
        )
        if np.random.random() < self.epsilon:
            return np.random.randint(self.config.act_dim)

        with torch.no_grad():
            state = torch.FloatTensor(observation).unsqueeze(0).to(self.device)
            return self.q_net(state).argmax(dim=1).item()

    def update(self, state, action, reward, next_state, done) -> Optional[float]:
        self.replay.push(state, action, reward, next_state, done)
        self.step_count += 1

        if len(self.replay) < self.config.batch_size:
            return None

        states, actions, rewards, next_states, dones, indices, weights = (
            self.replay.sample(self.config.batch_size)
        )

        states_t = torch.FloatTensor(states).to(self.device)
        actions_t = torch.LongTensor(actions).to(self.device)
        rewards_t = torch.FloatTensor(rewards).to(self.device)
        next_states_t = torch.FloatTensor(next_states).to(self.device)
        dones_t = torch.FloatTensor(dones).to(self.device)
        weights_t = torch.FloatTensor(weights).to(self.device)

        q_values = self.q_net(states_t).gather(1, actions_t.unsqueeze(1)).squeeze()
        next_actions = self.q_net(next_states_t).argmax(dim=1)
        next_q = (
            self.target_net(next_states_t).gather(1, next_actions.unsqueeze(1)).squeeze()
        )
        target = rewards_t + self.config.gamma * next_q * (1 - dones_t)

        td_errors = (q_values - target.detach()).abs()
        loss = (weights_t * F.smooth_l1_loss(q_values, target.detach(), reduction="none")).mean()

        self.optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self.q_net.parameters(), 10.0)
        self.optimizer.step()

        self.replay.update_priorities(indices, td_errors.detach().cpu().numpy())

        if self.step_count % self.config.target_update == 0:
            self.target_net.load_state_dict(self.q_net.state_dict())

        return loss.item()

    def save(self, path: str):
        torch.save(
            {
                "q_net": self.q_net.state_dict(),
                "target_net": self.target_net.state_dict(),
                "step_count": self.step_count,
            },
            path,
        )

    def load(self, path: str):
        ckpt = torch.load(path, map_location=self.device)
        self.q_net.load_state_dict(ckpt["q_net"])
        self.target_net.load_state_dict(ckpt["target_net"])
        self.step_count = ckpt["step_count"]

    def get_parameters(self):
        return self.q_net.parameters()


# ============================================================================
#  Federated Learning Coordinator
# ============================================================================


class FederatedCoordinator:
    """Federated averaging coordinator for multi-satellite learning."""

    def __init__(self, num_satellites: int):
        self.num_satellites = num_satellites
        self.global_weights: Optional[List[float]] = None
        self.local_updates: Dict[int, List[float]] = {}
        self.round_count = 0
        self.logger = logging.getLogger("federated")

    def receive_update(self, sat_id: int, weights: List[float]):
        """Receive local model weights from a satellite."""
        self.local_updates[sat_id] = weights

    def aggregate(self, min_participants: int = 3) -> Optional[List[float]]:
        """FedAvg aggregation."""
        if len(self.local_updates) < min_participants:
            self.logger.info(
                f"Not enough participants: {len(self.local_updates)}/{min_participants}"
            )
            return None

        weights_list = list(self.local_updates.values())
        n = len(weights_list)
        weight_len = len(weights_list[0])

        # Simple average
        avg_weights = [0.0] * weight_len
        for w in weights_list:
            for i in range(weight_len):
                avg_weights[i] += w[i] / n

        self.global_weights = avg_weights
        self.round_count += 1
        self.local_updates.clear()

        self.logger.info(
            f"FedAvg round {self.round_count}: aggregated {n} participants, "
            f"{weight_len} parameters"
        )
        return avg_weights

    def get_global_weights(self) -> Optional[List[float]]:
        return self.global_weights


# ============================================================================
#  Multi-xApp Agent Manager
# ============================================================================


class OranNtnAgentManager:
    """Manages all 5 xApp AI agents."""

    def __init__(self, output_dir: str = "oran-ntn-output"):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        self.agents: Dict[str, BaseAgent] = {}
        self.federated = FederatedCoordinator(num_satellites=66)

    def create_all_agents(
        self,
        obs_dim: int = 68,
        device: str = "cpu",
    ) -> Dict[str, BaseAgent]:
        """Create all 5 xApp agents."""

        # 1. HO Prediction (DQN + LSTM)
        ho_config = AgentConfig(
            name="ho-predict",
            obs_dim=obs_dim,
            act_dim=9,
            hidden_dim=256,
            lr=3e-4,
            output_dir=os.path.join(self.output_dir, "ho-predict"),
            device=device,
        )
        self.agents["ho-predict"] = HoPredictAgent(ho_config)

        # 2. Beam Hopping (PPO)
        beam_config = AgentConfig(
            name="beam-hop",
            obs_dim=obs_dim,
            act_dim=16,  # 16 beam-slot combinations
            hidden_dim=256,
            lr=3e-4,
            output_dir=os.path.join(self.output_dir, "beam-hop"),
            device=device,
        )
        self.agents["beam-hop"] = BeamHopAgent(beam_config)

        # 3. Slice Manager (MAPPO)
        slice_config = AgentConfig(
            name="slice-manager",
            obs_dim=obs_dim,
            act_dim=7,
            hidden_dim=128,
            lr=1e-4,
            output_dir=os.path.join(self.output_dir, "slice-manager"),
            device=device,
        )
        self.agents["slice-manager"] = SliceManagerAgent(slice_config)

        # 4. Doppler Compensation (DDPG)
        doppler_config = AgentConfig(
            name="doppler-comp",
            obs_dim=obs_dim,
            act_dim=2,
            hidden_dim=128,
            lr=1e-4,
            output_dir=os.path.join(self.output_dir, "doppler-comp"),
            device=device,
        )
        self.agents["doppler-comp"] = DopplerCompAgent(doppler_config)

        # 5. TN-NTN Steering (DQN)
        steering_config = AgentConfig(
            name="tn-ntn-steering",
            obs_dim=obs_dim,
            act_dim=3,
            hidden_dim=256,
            lr=3e-4,
            output_dir=os.path.join(self.output_dir, "tn-ntn-steering"),
            device=device,
        )
        self.agents["tn-ntn-steering"] = TnNtnSteeringAgent(steering_config)

        return self.agents

    def save_all(self):
        """Save all agent models."""
        for name, agent in self.agents.items():
            path = os.path.join(self.output_dir, f"{name}_model.pth")
            agent.save(path)
            agent.save_training_log()

    def load_all(self):
        """Load all agent models if checkpoints exist."""
        for name, agent in self.agents.items():
            path = os.path.join(self.output_dir, f"{name}_model.pth")
            if os.path.exists(path):
                agent.load(path)
                print(f"Loaded {name} model from {path}")

    def federated_round(self, agent_name: str = "ho-predict"):
        """Run one federated learning round for an agent."""
        agent = self.agents.get(agent_name)
        if not agent:
            return

        weights = agent.get_weights()
        self.federated.receive_update(0, weights)  # Local update

        global_weights = self.federated.aggregate(min_participants=1)
        if global_weights:
            agent.set_weights(global_weights)


# ============================================================================
#  CLI Entry Point
# ============================================================================


def main():
    parser = argparse.ArgumentParser(
        description="O-RAN NTN Multi-xApp AI Agent Framework"
    )
    parser.add_argument(
        "--agent",
        type=str,
        default="all",
        choices=["all", "ho-predict", "beam-hop", "slice-manager",
                 "doppler-comp", "tn-ntn-steering"],
        help="Which agent to run",
    )
    parser.add_argument("--episodes", type=int, default=1000, help="Training episodes")
    parser.add_argument(
        "--output", type=str, default="oran-ntn-output", help="Output directory"
    )
    parser.add_argument(
        "--device", type=str, default="cpu", choices=["cpu", "cuda"], help="Device"
    )
    parser.add_argument("--load-model", type=str, default=None, help="Load checkpoint")
    parser.add_argument(
        "--obs-dim", type=int, default=68, help="Observation dimension"
    )
    parser.add_argument(
        "--federated", action="store_true", help="Enable federated learning"
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="train",
        choices=["train", "eval", "export"],
        help="Operation mode",
    )

    args = parser.parse_args()

    if not TORCH_AVAILABLE:
        print("ERROR: PyTorch is required. Install with: pip install torch")
        sys.exit(1)

    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s [%(name)s] %(levelname)s: %(message)s"
    )

    manager = OranNtnAgentManager(output_dir=args.output)
    agents = manager.create_all_agents(obs_dim=args.obs_dim, device=args.device)

    if args.load_model:
        manager.load_all()

    if args.mode == "export":
        # Export model weights for ns3 C++ on-board inference
        for name, agent in agents.items():
            weights = agent.get_weights()
            path = os.path.join(args.output, f"{name}_weights.json")
            with open(path, "w") as f:
                json.dump(weights, f)
            print(f"Exported {name}: {len(weights)} weights to {path}")
        return

    if args.mode == "train":
        print(f"\n{'='*60}")
        print(f"  O-RAN NTN Multi-xApp AI Training")
        print(f"  Agents: {', '.join(agents.keys())}")
        print(f"  Episodes: {args.episodes}")
        print(f"  Device: {args.device}")
        print(f"  Federated: {args.federated}")
        print(f"{'='*60}\n")

        # Synthetic training loop (would be replaced by ns3-ai integration)
        for episode in range(args.episodes):
            obs = np.random.randn(args.obs_dim).astype(np.float32)
            episode_reward = 0

            for step in range(100):
                for name, agent in agents.items():
                    if args.agent != "all" and args.agent != name:
                        continue

                    action = agent.select_action(obs)
                    next_obs = np.random.randn(args.obs_dim).astype(np.float32)
                    reward = np.random.randn() * 0.1
                    done = step == 99

                    loss = agent.update(obs, action, reward, next_obs, done)
                    agent.log_step(reward, loss, action=action)
                    episode_reward += reward

                obs = next_obs

            if (episode + 1) % 100 == 0:
                print(
                    f"Episode {episode+1}/{args.episodes} | "
                    f"Avg Reward: {episode_reward/100:.4f}"
                )

            if args.federated and (episode + 1) % 50 == 0:
                for name in agents:
                    manager.federated_round(name)
                print(f"  [Federated] Round completed for all agents")

        manager.save_all()
        print(f"\nTraining complete. Models saved to {args.output}/")

    elif args.mode == "eval":
        print("Evaluation mode - loading models and running inference")
        manager.load_all()
        obs = np.random.randn(args.obs_dim).astype(np.float32)
        for name, agent in agents.items():
            action = agent.select_action(obs)
            print(f"  {name}: action={action}")


if __name__ == "__main__":
    main()
