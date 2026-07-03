"""Bounded-action-space env subclass, reused for the SAC config sanity check (see __init__.py).

Same pattern as ``docking_rl.tasks.staging_dock.staging_dock_env.StagingDockEnv`` -- Isaac Lab's
``ManagerBasedRLEnv`` always advertises an unbounded action space, which breaks SAC's
``clip_actions: True``. Cartpole's action is a single joint-effort command scaled to [-100, 100]
by ``JointEffortActionCfg(scale=100.0)``, so its natural raw range is [-1, 1] before that scaling
-- matching JetRacer's action-term convention closely enough for a fair, like-for-like comparison.
"""

from __future__ import annotations

import gymnasium as gym

from isaaclab.envs import ManagerBasedRLEnv


class BoundedActionCartpoleEnv(ManagerBasedRLEnv):
    """ManagerBasedRLEnv with a bounded [-1, 1] action space (required for stable SAC)."""

    def _configure_gym_env_spaces(self):
        super()._configure_gym_env_spaces()
        action_dim = sum(self.action_manager.action_term_dim)
        self.single_action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(action_dim,))
        self.action_space = gym.vector.utils.batch_space(self.single_action_space, self.num_envs)
