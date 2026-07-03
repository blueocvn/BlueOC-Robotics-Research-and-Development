"""Environment subclass that bounds the action space to [-1, 1].

Isaac Lab's :class:`ManagerBasedRLEnv` always advertises an *unbounded* action space
(``Box(-inf, +inf)``; see ``_configure_gym_env_spaces``). That's fine for PPO, but it breaks SAC:
with an unbounded space, skrl's Gaussian policy cannot clip its (un-squashed) samples, so the
off-policy critic gets trained on arbitrarily large action values and the Q-estimates diverge to
NaN within a few thousand updates.

Our actions are already bounded in effect -- :class:`AckermannAction` clamps the raw action to
[-1, 1] before scaling -- so advertising a matching [-1, 1] Box is both honest and what lets the
SAC agent's ``clip_actions: True`` keep the critic's action inputs bounded and stable.
"""

from __future__ import annotations

import gymnasium as gym
import numpy as np

from isaaclab.envs import ManagerBasedRLEnv


class StagingDockEnv(ManagerBasedRLEnv):
    """ManagerBasedRLEnv with a bounded [-1, 1] action space (needed for stable SAC)."""

    def _configure_gym_env_spaces(self):
        # let the base class set up observation spaces and the (unbounded) action space first
        super()._configure_gym_env_spaces()
        # then replace the action space with a bounded one matching AckermannAction's clamp range
        action_dim = sum(self.action_manager.action_term_dim)
        self.single_action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(action_dim,))
        self.action_space = gym.vector.utils.batch_space(self.single_action_space, self.num_envs)
