#!/usr/bin/env python3
"""
Space RIC On-Board AI Agent
============================

Python agent for Space RIC inference via ns3-ai msg-interface.
Uses shared memory IPC for low-latency decision making.

Supports:
  - DQN for discrete beam/HO decisions
  - PPO for continuous power/allocation decisions
  - Multi-action support (simultaneous beam + power decisions)
  - Federated learning gradient computation

Author: Muhammad Uzair
"""

import argparse
import sys
import numpy as np

try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


# =============================================================================
#  Space RIC Neural Network
# =============================================================================

MAX_BEAMS = 72
MAX_ISL = 4

if TORCH_AVAILABLE:
    class SpaceRicDQN(nn.Module):
        """Multi-head DQN for Space RIC decisions."""

        def __init__(self, obs_dim=None, hidden=512):
            super().__init__()
            # Observation: beam states + orbital + ISL + system
            if obs_dim is None:
                obs_dim = MAX_BEAMS * 5 + 6 + MAX_ISL * 2 + 5  # 371

            # Shared feature extractor
            self.shared = nn.Sequential(
                nn.Linear(obs_dim, hidden),
                nn.ReLU(),
                nn.LayerNorm(hidden),
                nn.Linear(hidden, hidden),
                nn.ReLU(),
                nn.LayerNorm(hidden),
            )

            # Action type head (8 types)
            self.action_type_head = nn.Linear(hidden, 8)

            # Target beam head
            self.beam_head = nn.Linear(hidden, MAX_BEAMS)

            # Parameter head (continuous)
            self.param_head = nn.Sequential(
                nn.Linear(hidden, 64),
                nn.ReLU(),
                nn.Linear(64, 2),  # parameter1, parameter2
            )

            # Confidence head
            self.confidence_head = nn.Sequential(
                nn.Linear(hidden, 32),
                nn.ReLU(),
                nn.Linear(32, 1),
                nn.Sigmoid(),
            )

        def forward(self, x):
            features = self.shared(x)
            action_type = self.action_type_head(features)
            beam = self.beam_head(features)
            params = self.param_head(features)
            confidence = self.confidence_head(features)
            return action_type, beam, params, confidence


    class FederatedLearningClient:
        """Federated learning client for Space RIC models."""

        def __init__(self, model, lr=1e-3, mu=0.01):
            self.model = model
            self.optimizer = optim.SGD(model.parameters(), lr=lr)
            self.mu = mu  # FedProx proximal term
            self.global_params = None

        def set_global_model(self, weights):
            """Receive global model from ground RIC."""
            self.global_params = [p.clone() for p in self.model.parameters()]
            idx = 0
            for p in self.model.parameters():
                numel = p.numel()
                p.data = torch.FloatTensor(
                    weights[idx:idx + numel]).reshape(p.shape)
                idx += numel

        def compute_local_gradients(self, data_loader, epochs=1):
            """Compute local gradients for FL aggregation."""
            self.model.train()
            local_loss = 0.0
            samples = 0

            for epoch in range(epochs):
                for batch in data_loader:
                    states, targets = batch
                    self.optimizer.zero_grad()
                    outputs = self.model(states)
                    loss = nn.functional.mse_loss(outputs[0], targets)

                    # FedProx: add proximal term
                    if self.global_params is not None:
                        prox_term = 0.0
                        for p, gp in zip(self.model.parameters(),
                                         self.global_params):
                            prox_term += ((p - gp) ** 2).sum()
                        loss += (self.mu / 2) * prox_term

                    loss.backward()
                    self.optimizer.step()
                    local_loss += loss.item()
                    samples += states.size(0)

            # Extract gradient weights
            gradients = []
            for p in self.model.parameters():
                gradients.extend(p.data.flatten().tolist())

            return gradients, local_loss / max(samples, 1), samples


# =============================================================================
#  Observation/Action Parsing
# =============================================================================

def parse_observation(obs_flat):
    """Parse flat observation array into structured dict."""
    obs = {}
    idx = 0
    obs['beam_loads'] = obs_flat[idx:idx + MAX_BEAMS]; idx += MAX_BEAMS
    obs['beam_sinr'] = obs_flat[idx:idx + MAX_BEAMS]; idx += MAX_BEAMS
    obs['beam_ue_count'] = obs_flat[idx:idx + MAX_BEAMS]; idx += MAX_BEAMS
    obs['beam_interference'] = obs_flat[idx:idx + MAX_BEAMS]; idx += MAX_BEAMS
    obs['beam_throughput'] = obs_flat[idx:idx + MAX_BEAMS]; idx += MAX_BEAMS

    obs['latitude'] = obs_flat[idx]; idx += 1
    obs['longitude'] = obs_flat[idx]; idx += 1
    obs['altitude_km'] = obs_flat[idx]; idx += 1
    obs['velocity'] = obs_flat[idx:idx + 3]; idx += 3

    obs['isl_delays'] = obs_flat[idx:idx + MAX_ISL]; idx += MAX_ISL
    obs['isl_utilization'] = obs_flat[idx:idx + MAX_ISL]; idx += MAX_ISL

    obs['feeder_link_avail'] = obs_flat[idx]; idx += 1
    obs['battery_level'] = obs_flat[idx]; idx += 1
    obs['solar_power'] = obs_flat[idx]; idx += 1
    obs['compute_util'] = obs_flat[idx]; idx += 1
    obs['total_throughput'] = obs_flat[idx]; idx += 1

    return obs


def build_action(action_type, target_beam=0, target_ue=0,
                 param1=0.0, param2=0.0, confidence=0.5):
    """Build action dict for Space RIC."""
    return {
        'action_type': int(action_type),
        'target_beam': int(target_beam),
        'target_ue': int(target_ue),
        'param1': float(param1),
        'param2': float(param2),
        'confidence': float(confidence),
    }


# =============================================================================
#  Rule-Based Fallback Agent
# =============================================================================

class RuleBasedSpaceRicAgent:
    """Rule-based agent when PyTorch is not available."""

    def decide(self, obs):
        parsed = parse_observation(obs)
        loads = np.array(parsed['beam_loads'])
        sinrs = np.array(parsed['beam_sinr'])
        battery = parsed['battery_level']

        # Rule 1: Battery critical -> shutdown lowest beam
        if battery < 0.2:
            lowest_beam = int(np.argmin(loads))
            return build_action(5, target_beam=lowest_beam,
                                confidence=0.8)

        # Rule 2: Overloaded beam -> reallocate
        if np.max(loads) > 0.9:
            overloaded = int(np.argmax(loads))
            return build_action(1, target_beam=overloaded,
                                confidence=0.85)

        # Rule 3: Poor SINR -> boost power
        active_sinrs = sinrs[loads > 0.01]
        if len(active_sinrs) > 0 and np.min(active_sinrs) < -5.0:
            poor_beam = int(np.argmin(sinrs))
            return build_action(4, target_beam=poor_beam,
                                param1=3.0, confidence=0.7)

        # Default: no action
        return build_action(0, confidence=0.5)


# =============================================================================
#  Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="Space RIC AI Agent")
    parser.add_argument("--mode", choices=["train", "infer", "rule-based"],
                        default="rule-based")
    parser.add_argument("--model-path", type=str, default=None)
    parser.add_argument("--episodes", type=int, default=500)
    args = parser.parse_args()

    print(f"Space RIC Agent - Mode: {args.mode}")
    print(f"PyTorch: {'Available' if TORCH_AVAILABLE else 'NOT available'}")

    if args.mode == "rule-based" or not TORCH_AVAILABLE:
        agent = RuleBasedSpaceRicAgent()
        print("Using rule-based agent")
        # In actual deployment, this connects via ns3-ai msg-interface
    elif args.mode == "train":
        print("Training mode requires ns3-ai msg-interface runtime")
    elif args.mode == "infer":
        if args.model_path and TORCH_AVAILABLE:
            model = SpaceRicDQN()
            model.load_state_dict(torch.load(args.model_path))
            model.eval()
            print(f"Loaded model from {args.model_path}")


if __name__ == "__main__":
    main()
