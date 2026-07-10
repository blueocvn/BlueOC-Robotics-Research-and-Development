"""Evaluate a trained bicycle-dock policy: report success rate over N episodes and optionally
save a GIF of a few rollouts.

Usage:
    python play.py --model runs/sac_bicycle --episodes 200
    python play.py --model runs/sac_bicycle --gif rollouts.gif      # visualize 6 episodes
    python play.py --model runs/sac_bicycle --obs_noise 0.02        # eval under odom noise
"""

import argparse

import numpy as np
from stable_baselines3 import SAC

from bicycle_env import BicycleDockEnv


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--model", type=str, default="runs/sac_bicycle")
    p.add_argument("--episodes", type=int, default=200)
    p.add_argument("--obs_noise", type=float, default=0.0)
    p.add_argument("--obs_bias", type=float, default=0.0)
    p.add_argument("--gif", type=str, default=None, help="save a GIF of the first 6 episodes")
    args = p.parse_args()

    model = SAC.load(args.model)
    render = args.gif is not None
    env = BicycleDockEnv(
        obs_noise_std=args.obs_noise, obs_bias_std=args.obs_bias,
        render_mode="rgb_array" if render else None,
    )

    successes, dists, heads, steps_to_dock = 0, [], [], []
    frames = []
    for ep in range(args.episodes):
        obs, _ = env.reset(seed=ep)
        done = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, _, term, trunc, info = env.step(action)
            done = term or trunc
            if render and ep < 6:
                frames.append(env.render())
        if info["docked"]:
            successes += 1
            steps_to_dock.append(env.steps)
        dists.append(info["dist"])
        heads.append(info["heading_err"])

    n = args.episodes
    print(f"\n=== {n} episodes (obs_noise={args.obs_noise}, obs_bias={args.obs_bias}) ===")
    print(f"  success rate     : {successes}/{n} = {100*successes/n:.1f}%")
    print(f"  final dist  (m)  : mean {np.mean(dists):.3f}, median {np.median(dists):.3f}")
    print(f"  final heading(deg): mean {np.degrees(np.mean(heads)):.1f}")
    if steps_to_dock:
        print(f"  steps to dock    : mean {np.mean(steps_to_dock):.0f}  (~{np.mean(steps_to_dock)*env.dt:.1f}s)")

    if render and frames:
        import imageio
        imageio.mimsave(args.gif, frames, fps=20)
        print(f"  saved GIF -> {args.gif}")


if __name__ == "__main__":
    main()
