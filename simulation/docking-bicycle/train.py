"""Train a SAC docking policy on the kinematic-bicycle env (Stable-Baselines3).

Fast: the env is pure math (no physics engine), so this trains in minutes on CPU/GPU. SAC is the
sample-efficient default; swap ALGO for PPO with a one-line change if you prefer on-policy.

Usage:
    python train.py                              # clean observations, 300k steps
    python train.py --timesteps 500000 --n_envs 16
    python train.py --obs_noise 0.02 --obs_bias 0.05   # drift-tolerance domain randomization
"""

import argparse
import os

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.env_util import make_vec_env

from bicycle_env import BicycleDockEnv


class SuccessRateCallback(BaseCallback):
    """Logs rolling docking success rate from episode infos."""

    def __init__(self, verbose=0):
        super().__init__(verbose)
        self._results = []

    def _on_step(self) -> bool:
        for info in self.locals.get("infos", []):
            if "episode" in info:  # episode just ended
                self._results.append(1.0 if info.get("docked", False) else 0.0)
                self._results = self._results[-500:]
        if self._results and self.num_timesteps % 5000 < self.training_env.num_envs:
            rate = sum(self._results) / len(self._results)
            self.logger.record("rollout/success_rate", rate)
            if self.verbose:
                print(f"[{self.num_timesteps:>7d}] success rate (last {len(self._results)}): {rate:.1%}", flush=True)
        return True


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--timesteps", type=int, default=300_000)
    p.add_argument("--n_envs", type=int, default=8)
    p.add_argument("--obs_noise", type=float, default=0.0, help="per-step obs noise std (drift-tolerance)")
    p.add_argument("--obs_bias", type=float, default=0.0, help="per-episode obs bias std (simulates drift)")
    p.add_argument("--device", type=str, default="auto")
    p.add_argument("--out", type=str, default="runs/sac_bicycle")
    p.add_argument("--load", type=str, default=None,
                   help="continue training from this SB3 .zip (e.g. runs/sac_bicycle) instead of fresh")
    p.add_argument("--save_freq", type=int, default=50_000,
                   help="checkpoint every N timesteps to runs/checkpoints/ (0 disables, e.g. for tune.py's short trials)")
    args = p.parse_args()

    out_dir = os.path.dirname(args.out) or "."
    os.makedirs(out_dir, exist_ok=True)

    env_kwargs = dict(obs_noise_std=args.obs_noise, obs_bias_std=args.obs_bias)
    vec_env = make_vec_env(BicycleDockEnv, n_envs=args.n_envs, env_kwargs=env_kwargs)

    if args.load:
        print(f"[INFO] continuing from {args.load}.zip", flush=True)
        model = SAC.load(args.load, env=vec_env, device=args.device, tensorboard_log=out_dir)
    else:
        action_dim = vec_env.action_space.shape[0]

        model = SAC(
            "MlpPolicy",
            vec_env,
            learning_rate=0.00039550981679237966,
            buffer_size=300_000,
            batch_size=1024,
            gamma=0.98,
            tau=0.019982064088650796,
            train_freq=1,
            gradient_steps=4,
            learning_starts=2000,
            target_entropy=-action_dim - 1.2884317026942653,  # if that's how it was tuned
            policy_kwargs=dict(net_arch=[256, 256]),
            device=args.device,
            verbose=0,
            tensorboard_log=out_dir,
        )
    print(f"[INFO] training SAC on bicycle-dock: {args.timesteps} steps, {args.n_envs} envs, "
          f"obs_noise={args.obs_noise}, obs_bias={args.obs_bias}", flush=True)

    callbacks = [SuccessRateCallback(verbose=1)]
    if args.save_freq > 0:
        # save_freq is in vectorized _on_step calls, not total timesteps -- each call advances
        # num_timesteps by n_envs, so divide to make --save_freq mean "every N total timesteps"
        # like --timesteps does. Names include step count so an interrupted run (e.g. killed before
        # the final model.save() below) still leaves a usable, loadable checkpoint on disk.
        callbacks.append(CheckpointCallback(
            save_freq=max(args.save_freq // args.n_envs, 1),
            save_path=os.path.join(out_dir, "checkpoints"),
            name_prefix=os.path.basename(args.out),
        ))
    model.learn(total_timesteps=args.timesteps, callback=callbacks,
                progress_bar=True, reset_num_timesteps=(args.load is None))
    model.save(args.out)
    print(f"[INFO] saved -> {args.out}.zip", flush=True)


if __name__ == "__main__":
    main()
