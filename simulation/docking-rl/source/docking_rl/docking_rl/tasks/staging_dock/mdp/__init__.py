"""MDP terms for the JetRacer staging-pose docking task.

Re-exports Isaac Lab's built-in mdp terms (observations, events, common rewards/terminations)
alongside the task-specific ones defined in this package, so config files can do a single
``import ... mdp as mdp`` and reference everything as ``mdp.<term>``, matching the convention
used throughout ``isaaclab_tasks``.
"""

from isaaclab.envs.mdp import *  # noqa: F401, F403

from .actions import AckermannAction, AckermannActionCfg  # noqa: F401
from .commands import StagingPoseCommand, StagingPoseCommandCfg  # noqa: F401
from .events import reset_root_state_in_approach_arc  # noqa: F401
from .observations import staging_pose_hold_progress  # noqa: F401
from .rewards import (  # noqa: F401
    gear_shift_penalty,
    heading_alignment,
    loiter_penalty,
    position_progress,
    spin_in_place_penalty,
    staging_pose_reached,
)
from .terminations import out_of_bounds, staging_pose_success  # noqa: F401
