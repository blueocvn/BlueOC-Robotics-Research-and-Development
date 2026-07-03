"""SAC-config sanity check task -- NOT part of the docking task, a debugging tool.

Registers Isaac Lab's own, unmodified ``CartpoleEnvCfg`` (a simple, well-tested continuous-control
task) under a new gym id, paired with our exact ``skrl_sac_cfg.yaml`` (same learning rate, network
architecture, discount factor, grad_norm_clip, log_std bounds, rewards_shaper_scale). Purpose:
JetRacer-Dock-Staging-v0 showed a critic/Q-value divergence that survived being tested at reward
scales from -650 to -15 per episode (see the docking task's RewardsCfg docstring) -- proving it
wasn't about reward design. This isolates the remaining two suspects: is it our SAC *config*
(independent of environment), or something specific to the JetRacer environment/action term? If
this ALSO diverges on cartpole, the config itself is the problem. If it stays stable, the bug is
specific to the docking task's environment/action term.

Only the action-space-bounding wrapper (``BoundedActionCartpoleEnv``, needed for
``clip_actions: True`` to work at all -- same reason ``StagingDockEnv`` exists for the docking
task) differs from cartpole's own standard registration.
"""

import gymnasium as gym

from . import agents

gym.register(
    id="SacSanityCheck-Cartpole-v0",
    entry_point=f"{__name__}.sac_sanity_env:BoundedActionCartpoleEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "isaaclab_tasks.manager_based.classic.cartpole.cartpole_env_cfg:CartpoleEnvCfg",
        "skrl_sac_cfg_entry_point": f"{agents.__name__}:skrl_sac_cfg.yaml",
    },
)

# Same env, clip_actions: False instead of True -- isolates whether skrl's internal action-
# clipping step (reading bounds from our custom-overridden action space) is the specific broken
# link. See skrl_sac_cfg_noclip.yaml's header comment.
gym.register(
    id="SacSanityCheck-Cartpole-NoClip-v0",
    entry_point=f"{__name__}.sac_sanity_env:BoundedActionCartpoleEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "isaaclab_tasks.manager_based.classic.cartpole.cartpole_env_cfg:CartpoleEnvCfg",
        "skrl_sac_cfg_entry_point": f"{agents.__name__}:skrl_sac_cfg_noclip.yaml",
    },
)
