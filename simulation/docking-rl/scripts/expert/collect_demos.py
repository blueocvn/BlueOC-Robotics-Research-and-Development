"""Roll out the scripted Pure-Pursuit/PID expert (docking_rl.tasks.staging_dock.expert) in the
JetRacer staging-dock env and save the resulting transitions to disk, for later use seeding SAC's
replay buffer (see the docking-rl README's "Train / play" section for the equivalent train.py /
play.py invocations -- this script follows the same external-task pattern).

Not a training script itself: no gradients, no neural network. Just env rollout + a hand-written
controller + tensor bookkeeping.
"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Collect scripted-expert demonstrations for JetRacer-Dock-Staging.")
parser.add_argument("--num_envs", type=int, default=256, help="Number of parallel environments to roll out.")
parser.add_argument("--task", type=str, default="JetRacer-Dock-Staging-v0", help="Name of the task.")
parser.add_argument("--num_steps", type=int, default=20000, help="Number of env steps to collect (per env).")
parser.add_argument(
    "--calibration_steps", type=int, default=30, help="Steps used to auto-detect the steer-command sign."
)
parser.add_argument(
    "--output",
    type=str,
    default=None,
    help="Output .pt path. Defaults to IsaacLab/logs/skrl/jetracer_dock_staging/expert_demos/demos.pt",
)
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import os

import gymnasium as gym
import torch

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg

# Register this project's tasks with gymnasium -- must run after the Sim app launch above (env
# cfgs import isaaclab.sim). See docking-rl's README for why this import is required.
import docking_rl.tasks  # noqa: F401
from docking_rl.tasks.staging_dock.expert import PurePursuitExpert


@torch.no_grad()
def calibrate_steer_sign(env, expert: PurePursuitExpert, num_steps: int) -> float:
    """Drive a short forward+steer burst and measure the resulting yaw-rate sign, so the expert's
    steer commands are calibrated to this specific USD articulation's joint-axis convention rather
    than assumed (see PurePursuitExpert's docstring).
    """
    car = env.unwrapped.scene["car"]
    calib_steer, calib_drive = 0.5, 0.3
    actions = torch.zeros(env.unwrapped.num_envs, 2, device=env.unwrapped.device)
    actions[:, 0] = calib_steer
    actions[:, 1] = calib_drive
    for _ in range(num_steps):
        env.step(actions)
    yaw_rate = car.data.root_ang_vel_b[:, 2]
    sign = 1.0 if yaw_rate.mean().item() >= 0.0 else -1.0
    print(
        f"[INFO] Calibration: steer_cmd={calib_steer:+.2f} -> mean yaw_rate={yaw_rate.mean().item():+.4f} "
        f"rad/s => steer_sign={sign:+.0f}"
    )
    return sign


def main():
    env_cfg = parse_env_cfg(
        args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )
    env = gym.make(args_cli.task, cfg=env_cfg)
    num_envs = env.unwrapped.num_envs
    device = env.unwrapped.device

    obs, _ = env.reset()
    obs_dim = obs["policy"].shape[1]

    expert = PurePursuitExpert(num_envs=num_envs, device=device)
    expert.steer_sign = calibrate_steer_sign(env, expert, args_cli.calibration_steps)

    # calibration burst left the envs in an arbitrary state -- start the real collection fresh.
    obs, _ = env.reset()
    expert.reset()

    obs_buf = torch.zeros(args_cli.num_steps, num_envs, obs_dim, device=device)
    action_buf = torch.zeros(args_cli.num_steps, num_envs, 2, device=device)
    reward_buf = torch.zeros(args_cli.num_steps, num_envs, device=device)
    next_obs_buf = torch.zeros(args_cli.num_steps, num_envs, obs_dim, device=device)
    done_buf = torch.zeros(args_cli.num_steps, num_envs, dtype=torch.bool, device=device)

    num_episodes_finished = 0
    num_successes = 0
    # per-gear progress_delta accumulators, to sanity-check the reverse-pivot logic is actually
    # closing the distance rather than making it worse (see PurePursuitExpert's docstring on the
    # reverse-bearing formula being a geometric approximation, not exact bicycle-model inversion).
    fwd_progress_sum, fwd_progress_n = 0.0, 0
    rev_progress_sum, rev_progress_n = 0.0, 0

    command_term = env.unwrapped.command_manager.get_term("staging_pose")

    for step in range(args_cli.num_steps):
        command = command_term.command
        actions = expert.compute(command)
        in_reverse = expert.gear < 0

        next_obs, reward, terminated, truncated, _ = env.step(actions)
        done = terminated | truncated

        progress = command_term.progress_delta
        fwd_progress_sum += progress[~in_reverse].sum().item()
        fwd_progress_n += int((~in_reverse).sum().item())
        rev_progress_sum += progress[in_reverse].sum().item()
        rev_progress_n += int(in_reverse.sum().item())

        obs_buf[step] = obs["policy"]
        action_buf[step] = actions
        reward_buf[step] = reward
        next_obs_buf[step] = next_obs["policy"]
        done_buf[step] = done

        if done.any():
            done_ids = done.nonzero(as_tuple=False).squeeze(-1)
            expert.reset(done_ids)
            num_episodes_finished += int(done.sum().item())
            num_successes += int((command_term.success_held[done_ids]).sum().item())

        obs = next_obs

        if (step + 1) % 2000 == 0:
            print(
                f"[INFO] step {step + 1}/{args_cli.num_steps} | episodes finished: "
                f"{num_episodes_finished} | successes: {num_successes} "
                f"({100.0 * num_successes / max(num_episodes_finished, 1):.1f}%)"
            )

    avg_fwd_progress = fwd_progress_sum / max(fwd_progress_n, 1)
    avg_rev_progress = rev_progress_sum / max(rev_progress_n, 1)
    print(
        f"[INFO] Done. {num_episodes_finished} episodes, {num_successes} successes "
        f"({100.0 * num_successes / max(num_episodes_finished, 1):.1f}%)."
    )
    print(
        f"[INFO] Mean per-step progress_delta -- forward gear: {avg_fwd_progress:+.4f} m, "
        f"reverse gear: {avg_rev_progress:+.4f} m (both should be positive: distance shrinking)."
    )
    if avg_rev_progress <= 0.0 and rev_progress_n > 0:
        print(
            "[WARNING] Reverse-gear progress is non-positive -- the reverse-bearing steering "
            "convention in PurePursuitExpert is likely inverted for this articulation. Consider "
            "flipping the sign used for bearing_rev there and re-collecting."
        )

    output_path = args_cli.output
    if output_path is None:
        output_path = os.path.abspath(
            os.path.join("logs", "skrl", "jetracer_dock_staging", "expert_demos", "demos.pt")
        )
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    torch.save(
        {
            "obs": obs_buf.reshape(-1, obs_dim).cpu(),
            "actions": action_buf.reshape(-1, 2).cpu(),
            "rewards": reward_buf.reshape(-1).cpu(),
            "next_obs": next_obs_buf.reshape(-1, obs_dim).cpu(),
            "dones": done_buf.reshape(-1).cpu(),
        },
        output_path,
    )
    print(f"[INFO] Saved {args_cli.num_steps * num_envs} transitions to: {output_path}")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
