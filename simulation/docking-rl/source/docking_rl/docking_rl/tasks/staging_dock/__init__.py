import gymnasium as gym

from . import agents

##
# Register Gym environment.
##

gym.register(
    id="JetRacer-Dock-Staging-v0",
    # custom env subclass that bounds the action space to [-1, 1] (required for stable SAC)
    entry_point=f"{__name__}.staging_dock_env:StagingDockEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.staging_dock_env_cfg:StagingDockEnvCfg",
        "skrl_sac_cfg_entry_point": f"{agents.__name__}:skrl_sac_cfg.yaml",
    },
)
