# docking-rl

Isaac Lab RL task: drive a JetRacer from an arbitrary start pose to a **staging pose** near a
docking bay, avoiding the obstacles that flank it. That's the whole job of this RL policy. Final,
sub-centimeter alignment onto the AprilTag mounted in the bay is a separate, classical PnP
visual-servo stage (not implemented here) that takes over once the policy reaches the staging
pose -- see `isaaclab_rl_parking_setup.md` (in the repo root's `Downloads`/attached guide),
Section 6.4, for that stage's design. This RL task has no observation, reward, or termination term
that references the tag; it only ever targets the staging pose.

## Layout

```
docking-rl/
├── source/docking_rl/          # Isaac Lab external task extension (pip install -e this)
│   └── docking_rl/
│       ├── assets/jetracer.py          # JETRACER_CFG articulation config
│       └── tasks/staging_dock/
│           ├── staging_dock_env_cfg.py # scene + actions/observations/rewards/terminations
│           ├── mdp/                    # task-specific mdp terms (actions, commands, rewards, ...)
│           └── agents/skrl_sac_cfg.yaml
├── scripts/skrl/{train,play}.py        # Isaac Lab's skrl scripts + `import docking_rl.tasks`
├── usd/jetracer_docking_scene.usda     # JetRacer articulation (chassis + steering + wheels)
└── ros2/nav2_rl_handoff_node.py        # deployment-side Nav2 -> RL -> docking-handoff skeleton
```

## The task

- **Action** (2D continuous): `[steer_cmd, drive_cmd]` in `[-1, 1]`, scaled to the steering-servo
  angle limit (±30°) and rear drive-wheel velocity limit. Both front steering joints get the same
  commanded angle (single-servo tie-rod steering -- matches the real JetRacer hardware, which
  doesn't have independent per-wheel Ackermann geometry either). Both rear wheels get the same
  commanded velocity.
- **Observation**: staging-pose error `(dx, dy, dheading)` in the vehicle frame, linear/angular
  velocity, front steering angle, last action.
- **Reward**: collision penalty (dominant) + progress/heading shaping toward the staging pose +
  sparse staging-pose-reached bonus + jerk/gear-shift/time shaping (secondary). See
  `RewardsCfg` in `staging_dock_env_cfg.py`.
- **Termination**: staging-pose success, chassis collision, out-of-bounds, timeout.
- **Domain randomization**: start pose, and small jitter on the two obstacle positions, every
  episode (`EventCfg`). The staging-pose *goal* itself is also resampled within a range every
  episode (`CommandsCfg.staging_pose`), which is what stands in for "randomize slot position."

Dock geometry constants (env-local frame, i.e. relative to each cloned env's origin) live at the
top of `staging_dock_env_cfg.py` and are cross-referenced in the USD file's prim positions --
if you move the bay, update both.

## Install

Requires an Isaac Lab installation with a working Python env (conda/venv) that already has
`isaaclab`, `isaaclab_assets`, `isaaclab_tasks` importable -- e.g. the `isaaclab` conda env used
elsewhere on this machine, or run everything through `IsaacLab/isaaclab.sh -p`.

```bash
# from this directory
python -m pip install -e source/docking_rl
```

## Train / play

Run the copies of Isaac Lab's skrl scripts under `scripts/skrl/` -- **not** the ones inside the
Isaac Lab repo. They are byte-for-byte identical except for one added line, `import
docking_rl.tasks`, at Isaac Lab's `# PLACEHOLDER: Extension template` marker. That import is what
registers our task with gymnasium; Isaac Lab's own scripts only import `isaaclab_tasks`, so running
them directly fails with `gymnasium.error.NameNotFound: Environment 'JetRacer-Dock-Staging'`. This
is the standard Isaac Lab external-project pattern (`isaaclab.sh -n` generates the same copies).

```bash
conda activate isaaclab                # so isaaclab.sh picks the right python ($CONDA_PREFIX)
cd /home/apc/IsaacLab
DOCKRL=/home/apc/robot-fulfillment/simulation/docking-rl

# train (SAC run length is set by trainer.timesteps in the yaml, currently 500k)
./isaaclab.sh -p $DOCKRL/scripts/skrl/train.py \
    --task JetRacer-Dock-Staging-v0 --num_envs 2048 --algorithm SAC

# quick smoke test: 16 envs, headless, short run via a Hydra override
./isaaclab.sh -p $DOCKRL/scripts/skrl/train.py \
    --task JetRacer-Dock-Staging-v0 --num_envs 16 --algorithm SAC --headless \
    agent.trainer.timesteps=200

# play back a checkpoint (renders the policy)
./isaaclab.sh -p $DOCKRL/scripts/skrl/play.py \
    --task JetRacer-Dock-Staging-v0 --num_envs 16 --algorithm SAC \
    --checkpoint <path/to/agent_*.pt>
```

Note: **do not pass `--max_iterations` with SAC.** That flag is PPO-only -- Isaac Lab's train.py
computes `timesteps = max_iterations * agent.rollouts`, and an off-policy SAC config has no
`rollouts` key, so it errors with `KeyError: 'rollouts'`. Bound a SAC run with the yaml's
`trainer.timesteps` (or the `agent.trainer.timesteps=<N>` Hydra override shown above) instead.

Checkpoints, TensorBoard events, and the resolved configs land in
`IsaacLab/logs/skrl/jetracer_dock_staging/<timestamp>/`.

## Known gaps / next steps

- `skrl_sac_cfg.yaml` targets the SAC agent API of the skrl version in this machine's `isaaclab`
  conda env (dataclass-based `AgentCfg`, explicit `target_critic_1/2` models in the `models:`
  block). If skrl gets upgraded, diff this file against a freshly generated
  `IsaacLab/isaaclab.sh -n` template.
- **Smoke-tested, not trained to convergence.** The full pipeline has been run end-to-end once
  (task registration -> env build from the USD articulation -> SAC loop -> checkpointing) with
  `--num_envs 16 --headless agent.trainer.timesteps=200`: 200 steps in ~5.6 s, checkpoints written
  to `logs/skrl/jetracer_dock_staging/.../checkpoints/`. No policy has actually *learned* to dock
  yet -- reward weights and episode length almost certainly need tuning, and behavior should be
  validated visually (watch for reward hacking / collision-detection gaps, per the guide's Sec 6.1).
- `ros2/nav2_rl_handoff_node.py` is a skeleton (Section 7.6 of the guide): the Nav2
  `NavigateToPose` goal-sending, the trained-policy inference call, and the docking-handoff
  trigger to whatever does the AprilTag approach are all left as `TODO`s. Its `/cmd_vel` output
  (`geometry_msgs/Twist`) matches the existing
  `jetracer_ws/src/ackermann_control/cmdvel_to_ackermann` node already in this repo, which
  converts `Twist` -> `AckermannDriveStamped` -- so no new conversion node should be needed on
  the robot side.
- No AprilTag prim, camera sensor, or visual-servo controller in this repo -- out of scope per the
  current plan (RL → staging pose only).
