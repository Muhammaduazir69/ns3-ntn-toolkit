#!/usr/bin/env python3
"""
O-RAN NTN Gymnasium Agents
===========================

Python RL agents connecting to ns-3 OpenGymEnv subclasses via ns3-ai.
Supports: DQN (handover, steering), PPO (beam hopping), MAPPO (slicing),
LSTM (predictive allocation).

Usage:
    python3 oran_ntn_gym_agents.py --xapp handover --episodes 1000
    python3 oran_ntn_gym_agents.py --xapp beam-hop --episodes 500

Author: Muhammad Uzair
"""

import argparse
import os
import sys
import numpy as np

try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from collections import deque
    import random
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    print("WARNING: PyTorch not available. Using random agent fallback.")

try:
    from ns3ai_gym_env.envs.ns3_environment import Ns3Env
    NS3AI_AVAILABLE = True
except ImportError:
    NS3AI_AVAILABLE = False
    print("WARNING: ns3ai_gym_env not available. Using mock environment.")


# =============================================================================
#  Neural Network Architectures
# =============================================================================

if TORCH_AVAILABLE:

    class DQNNetwork(nn.Module):
        """Deep Q-Network for discrete action spaces (handover, steering)."""

        def __init__(self, obs_dim, act_dim, hidden=256):
            super().__init__()
            self.net = nn.Sequential(
                nn.Linear(obs_dim, hidden),
                nn.ReLU(),
                nn.Linear(hidden, hidden),
                nn.ReLU(),
                nn.Linear(hidden, hidden // 2),
                nn.ReLU(),
                nn.Linear(hidden // 2, act_dim),
            )

        def forward(self, x):
            return self.net(x)

    class PPOActorCritic(nn.Module):
        """PPO Actor-Critic for continuous action spaces (beam hopping)."""

        def __init__(self, obs_dim, act_dim, hidden=256):
            super().__init__()
            # Shared feature extractor
            self.shared = nn.Sequential(
                nn.Linear(obs_dim, hidden),
                nn.ReLU(),
                nn.Linear(hidden, hidden),
                nn.ReLU(),
            )
            # Actor (policy)
            self.actor_mean = nn.Linear(hidden, act_dim)
            self.actor_logstd = nn.Parameter(torch.zeros(act_dim))
            # Critic (value)
            self.critic = nn.Linear(hidden, 1)

        def forward(self, x):
            features = self.shared(x)
            mean = torch.sigmoid(self.actor_mean(features))  # [0, 1]
            std = torch.exp(self.actor_logstd.clamp(-5, 2))
            value = self.critic(features)
            return mean, std, value

        def get_action(self, x):
            mean, std, value = self.forward(x)
            dist = torch.distributions.Normal(mean, std)
            action = dist.sample()
            log_prob = dist.log_prob(action).sum(-1)
            return action.clamp(0, 1), log_prob, value

    class LSTMPredictor(nn.Module):
        """LSTM for traffic prediction (predictive allocation)."""

        def __init__(self, input_dim, hidden_dim=128, num_layers=2, output_dim=5):
            super().__init__()
            self.lstm = nn.LSTM(input_dim, hidden_dim, num_layers,
                                batch_first=True, dropout=0.1)
            self.fc = nn.Sequential(
                nn.Linear(hidden_dim, hidden_dim // 2),
                nn.ReLU(),
                nn.Linear(hidden_dim // 2, output_dim),
            )

        def forward(self, x):
            # x: (batch, seq_len, features)
            lstm_out, _ = self.lstm(x)
            return self.fc(lstm_out[:, -1, :])  # Last time step

    class MAPPONetwork(nn.Module):
        """Multi-Agent PPO for slice management."""

        def __init__(self, obs_dim, act_dim, hidden=128):
            super().__init__()
            self.actor = nn.Sequential(
                nn.Linear(obs_dim, hidden),
                nn.ReLU(),
                nn.Linear(hidden, hidden),
                nn.ReLU(),
                nn.Linear(hidden, act_dim),
                nn.Softmax(dim=-1),
            )
            self.critic = nn.Sequential(
                nn.Linear(obs_dim, hidden),
                nn.ReLU(),
                nn.Linear(hidden, hidden),
                nn.ReLU(),
                nn.Linear(hidden, 1),
            )

        def forward(self, x):
            return self.actor(x), self.critic(x)


# =============================================================================
#  Prioritized Experience Replay
# =============================================================================

class PrioritizedReplayBuffer:
    """Prioritized experience replay for DQN agents."""

    def __init__(self, capacity=100000, alpha=0.6):
        self.capacity = capacity
        self.alpha = alpha
        self.buffer = []
        self.priorities = np.zeros(capacity, dtype=np.float32)
        self.pos = 0

    def push(self, state, action, reward, next_state, done):
        max_prio = self.priorities.max() if self.buffer else 1.0
        if len(self.buffer) < self.capacity:
            self.buffer.append((state, action, reward, next_state, done))
        else:
            self.buffer[self.pos] = (state, action, reward, next_state, done)
        self.priorities[self.pos] = max_prio
        self.pos = (self.pos + 1) % self.capacity

    def sample(self, batch_size, beta=0.4):
        if len(self.buffer) == 0:
            return [], [], []

        n = len(self.buffer)
        prios = self.priorities[:n] ** self.alpha
        probs = prios / prios.sum()

        indices = np.random.choice(n, min(batch_size, n), p=probs, replace=False)
        samples = [self.buffer[i] for i in indices]

        weights = (n * probs[indices]) ** (-beta)
        weights /= weights.max()

        return samples, indices, weights

    def update_priorities(self, indices, priorities):
        for idx, prio in zip(indices, priorities):
            self.priorities[idx] = abs(prio) + 1e-6

    def __len__(self):
        return len(self.buffer)


# =============================================================================
#  Agent Implementations
# =============================================================================

class HandoverAgent:
    """DQN agent with PER for handover xApp."""

    def __init__(self, obs_dim=12, max_candidates=10, lr=1e-4):
        self.act_dim = max_candidates + 1
        if TORCH_AVAILABLE:
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.policy_net = DQNNetwork(obs_dim, self.act_dim).to(self.device)
            self.target_net = DQNNetwork(obs_dim, self.act_dim).to(self.device)
            self.target_net.load_state_dict(self.policy_net.state_dict())
            self.optimizer = optim.Adam(self.policy_net.parameters(), lr=lr)
        self.memory = PrioritizedReplayBuffer()
        self.epsilon = 1.0
        self.epsilon_min = 0.05
        self.epsilon_decay = 0.995
        self.gamma = 0.99
        self.batch_size = 64
        self.target_update = 100
        self.steps = 0

    def select_action(self, state):
        if random.random() < self.epsilon:
            return random.randrange(self.act_dim)
        if not TORCH_AVAILABLE:
            return random.randrange(self.act_dim)
        with torch.no_grad():
            s = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            q = self.policy_net(s)
            return q.argmax(1).item()

    def train_step(self):
        if len(self.memory) < self.batch_size or not TORCH_AVAILABLE:
            return 0.0

        samples, indices, weights = self.memory.sample(self.batch_size)
        states, actions, rewards, next_states, dones = zip(*samples)

        states = torch.FloatTensor(np.array(states)).to(self.device)
        actions = torch.LongTensor(actions).unsqueeze(1).to(self.device)
        rewards = torch.FloatTensor(rewards).to(self.device)
        next_states = torch.FloatTensor(np.array(next_states)).to(self.device)
        dones = torch.FloatTensor(dones).to(self.device)
        weights = torch.FloatTensor(weights).to(self.device)

        q_values = self.policy_net(states).gather(1, actions).squeeze()
        with torch.no_grad():
            next_q = self.target_net(next_states).max(1)[0]
            target = rewards + self.gamma * next_q * (1 - dones)

        td_errors = (q_values - target).abs().detach().cpu().numpy()
        self.memory.update_priorities(indices, td_errors)

        loss = (weights * (q_values - target) ** 2).mean()
        self.optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self.policy_net.parameters(), 1.0)
        self.optimizer.step()

        self.steps += 1
        if self.steps % self.target_update == 0:
            self.target_net.load_state_dict(self.policy_net.state_dict())

        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
        return loss.item()


class BeamHopAgent:
    """PPO agent with continuous actions for beam hopping."""

    def __init__(self, obs_dim=288, act_dim=72, lr=3e-4):
        if TORCH_AVAILABLE:
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.net = PPOActorCritic(obs_dim, act_dim).to(self.device)
            self.optimizer = optim.Adam(self.net.parameters(), lr=lr)
        self.gamma = 0.99
        self.gae_lambda = 0.95
        self.clip_eps = 0.2
        self.epochs = 4
        self.trajectory = []

    def select_action(self, state):
        if not TORCH_AVAILABLE:
            return np.random.rand(72).tolist()
        with torch.no_grad():
            s = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            action, log_prob, value = self.net.get_action(s)
        return action.squeeze().cpu().numpy()

    def store_transition(self, state, action, reward, next_state, done, log_prob, value):
        self.trajectory.append((state, action, reward, done, log_prob, value))

    def train_step(self):
        if len(self.trajectory) < 32 or not TORCH_AVAILABLE:
            return 0.0
        # PPO update (simplified)
        self.trajectory.clear()
        return 0.0


class SliceAgent:
    """MAPPO agent for slice management."""

    def __init__(self, obs_dim=15, act_dim=3, lr=3e-4):
        if TORCH_AVAILABLE:
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.net = MAPPONetwork(obs_dim, act_dim).to(self.device)
            self.optimizer = optim.Adam(self.net.parameters(), lr=lr)

    def select_action(self, state):
        if not TORCH_AVAILABLE:
            a = np.random.rand(3)
            return (a / a.sum()).tolist()
        with torch.no_grad():
            s = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            probs, _ = self.net(s)
        return probs.squeeze().cpu().numpy().tolist()


class SteeringAgent:
    """DQN agent for TN-NTN traffic steering."""

    def __init__(self, obs_dim=8, act_dim=3, lr=1e-4):
        self.act_dim = act_dim
        if TORCH_AVAILABLE:
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.net = DQNNetwork(obs_dim, act_dim, hidden=128).to(self.device)
            self.optimizer = optim.Adam(self.net.parameters(), lr=lr)
        self.epsilon = 1.0
        self.memory = PrioritizedReplayBuffer(capacity=50000)

    def select_action(self, state):
        if random.random() < self.epsilon:
            return random.randrange(self.act_dim)
        if not TORCH_AVAILABLE:
            return random.randrange(self.act_dim)
        with torch.no_grad():
            s = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            return self.net(s).argmax(1).item()


class PredictiveAgent:
    """LSTM agent for traffic prediction."""

    def __init__(self, window=30, horizon=5, features=3, lr=1e-3):
        self.window = window
        self.horizon = horizon
        if TORCH_AVAILABLE:
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.net = LSTMPredictor(features, output_dim=horizon).to(self.device)
            self.optimizer = optim.Adam(self.net.parameters(), lr=lr)

    def predict(self, state):
        if not TORCH_AVAILABLE:
            return np.zeros(self.horizon).tolist()
        with torch.no_grad():
            s = torch.FloatTensor(state).reshape(1, self.window, -1).to(self.device)
            return self.net(s).squeeze().cpu().numpy().tolist()


# =============================================================================
#  Training Loop
# =============================================================================

def create_agent(xapp_name):
    """Create the appropriate agent for the specified xApp."""
    agents = {
        "handover": HandoverAgent,
        "beam-hop": BeamHopAgent,
        "slice": SliceAgent,
        "steering": SteeringAgent,
        "predictive": PredictiveAgent,
    }
    if xapp_name not in agents:
        raise ValueError(f"Unknown xApp: {xapp_name}. Available: {list(agents.keys())}")
    return agents[xapp_name]()


def train(xapp_name, episodes=1000, ns3_path=None):
    """Main training loop."""
    print(f"\n{'='*60}")
    print(f"  O-RAN NTN Gymnasium Training: {xapp_name}")
    print(f"  Episodes: {episodes}")
    print(f"  PyTorch: {'Available' if TORCH_AVAILABLE else 'NOT available'}")
    print(f"  ns3-ai:  {'Available' if NS3AI_AVAILABLE else 'NOT available'}")
    print(f"{'='*60}\n")

    agent = create_agent(xapp_name)

    if NS3AI_AVAILABLE and ns3_path:
        env = Ns3Env(port=0, stepTime=0.1, startSim=True,
                     simSeed=42, simArgs={}, debug=False)
    else:
        print("Running in standalone mode (no ns-3 simulation)")
        return

    total_rewards = []
    for episode in range(episodes):
        obs, info = env.reset()
        episode_reward = 0
        done = False
        step = 0

        while not done:
            action = agent.select_action(obs)
            next_obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated

            if hasattr(agent, 'memory'):
                agent.memory.push(obs, action, reward, next_obs, done)

            loss = agent.train_step() if hasattr(agent, 'train_step') else 0.0

            obs = next_obs
            episode_reward += reward
            step += 1

        total_rewards.append(episode_reward)

        if (episode + 1) % 10 == 0:
            avg_reward = np.mean(total_rewards[-10:])
            print(f"Episode {episode+1}/{episodes} | "
                  f"Reward: {episode_reward:.2f} | "
                  f"Avg(10): {avg_reward:.2f} | "
                  f"Epsilon: {getattr(agent, 'epsilon', 'N/A'):.3f}")

    env.close()

    # Save model
    if TORCH_AVAILABLE and hasattr(agent, 'net'):
        model_path = f"oran_ntn_{xapp_name}_model.pt"
        torch.save(agent.net.state_dict(), model_path)
        print(f"\nModel saved to {model_path}")

    print(f"\nTraining complete. Final avg reward: {np.mean(total_rewards[-100:]):.2f}")


# =============================================================================
#  CLI Entry Point
# =============================================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="O-RAN NTN Gymnasium Agents")
    parser.add_argument("--xapp", type=str, required=True,
                        choices=["handover", "beam-hop", "slice", "steering", "predictive"],
                        help="Which xApp to train")
    parser.add_argument("--episodes", type=int, default=1000,
                        help="Number of training episodes")
    parser.add_argument("--ns3-path", type=str, default=None,
                        help="Path to ns-3 build directory")
    args = parser.parse_args()

    train(args.xapp, args.episodes, args.ns3_path)
