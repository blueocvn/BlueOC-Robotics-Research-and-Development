"""Visualize the standalone bicycle-trained SB3 policy driving the ARTICULATED JetRacer in Isaac
Lab's 3D viewer -- a sim-to-sim transfer check (kinematic-trained policy -> full physics model).

Training stays standalone/fast (bicycle_env.py + SB3). This script ONLY loads the trained policy
and steps the real Isaac Lab docking env, translating between the two interfaces each step:

  Isaac obs (13-dim)  --extract-->  bicycle obs [dx, dy, dheading, delta, v_prev]
  bicycle action [steer_rate, velocity]  --integrate steer_rate->angle-->  Isaac action [steer, drive]

dt matches (both 1/30 s), and the staging-pose error is body-frame in both, so the policy is
frame-invariant to where the goal actually sits. Run headless-off to watch it in 3D.
"""

"""Launch Isaac Sim first."""
import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Play a bicycle-trained SB3 policy on the Isaac JetRacer.")
parser.add_argument("--task", type=str, default="JetRacer-Dock-Staging-v0")
parser.add_argument("--num_envs", type=int, default=9)
parser.add_argument("--model", type=str, default="runs/policy_ts.pt",
                    help="path to the TorchScript policy (export_policy.py) -- numpy-agnostic")
parser.add_argument("--arc", action="store_true", help="full 180deg arc starts (default: straight-in)")
parser.add_argument("--disable_fabric", action="store_true", default=False)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest follows."""
import math

import gymnasium as gym
import torch

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg

import docking_rl.tasks  # noqa: F401

# bicycle-env constants used for the interface translation (must match bicycle_env.py defaults)
BIKE_MAX_STEER = 0.349
BIKE_MAX_STEER_RATE = 2.0
BIKE_MAX_SPEED = 0.6
DT = 1.0 / 30.0
STEER_ANGLE_LIMIT = 0.349  # Isaac AckermannActionCfg.steer_angle_limit


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs,
                            use_fabric=not args_cli.disable_fabric)
    if not args_cli.arc:
        # straight-in starts (matches how we mostly trained/tested); drop --arc lines for full arc
        env_cfg.events.reset_car_pose.params["arc_half_angle"] = 0.0
        env_cfg.events.reset_car_pose.params["heading_noise"] = 0.0

    env = gym.make(args_cli.task, cfg=env_cfg)
    base = env.unwrapped
    device = base.device
    n = base.num_envs

    car = base.scene["car"]
    steer_idx, _ = car.find_joints(["steer_joint_L"])
    steer_idx = steer_idx[0]

    policy = torch.jit.load(args_cli.model, map_location=device).eval()
    print(f"[INFO] loaded TorchScript policy {args_cli.model}; driving {n} articulated JetRacers", flush=True)

    obs_dict, _ = env.reset()
    # integrated steering angle per env (the bicycle policy commands steer RATE)
    steer_angle = torch.zeros(n, device=device)
    v_prev = torch.zeros(n, device=device)

    while simulation_app.is_running():
        with torch.inference_mode():
            # --- build the 5-dim bicycle observation from the Isaac env state -------------------
            cmd = base.command_manager.get_command("staging_pose")  # (n,3): dx, dy, dheading (body frame)
            delta = car.data.joint_pos[:, steer_idx]                # current physical steering angle
            bike_obs = torch.stack([cmd[:, 0], cmd[:, 1], cmd[:, 2], delta, v_prev], dim=1)

            # --- policy: [steer_rate_norm, velocity_norm] in [-1,1] (TorchScript, on-device) -----
            action = policy(bike_obs.float()).clamp(-1.0, 1.0)
            steer_rate = action[:, 0] * BIKE_MAX_STEER_RATE
            vel = action[:, 1]  # normalized; bicycle max_speed ~ Isaac drive limit, use directly

            # integrate steer rate -> angle (what the physical steering servo is commanded to)
            steer_angle = torch.clamp(steer_angle + steer_rate * DT, -BIKE_MAX_STEER, BIKE_MAX_STEER)
            v_prev = vel * BIKE_MAX_SPEED

            # --- Isaac action: [steer_cmd, drive_cmd] both in [-1,1] -----------------------------
            isaac_action = torch.stack([steer_angle / STEER_ANGLE_LIMIT, vel], dim=1)
            obs_dict, _, terminated, truncated, _ = env.step(isaac_action)

            # reset integrators for envs that just reset
            done = terminated | truncated
            if done.any():
                steer_angle[done] = 0.0
                v_prev[done] = 0.0

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
