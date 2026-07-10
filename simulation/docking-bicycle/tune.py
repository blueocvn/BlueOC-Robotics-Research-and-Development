"""Optuna hyperparameter search for the SAC bicycle-dock policy.

Cheap to do properly here: the env is pure math (no physics engine), so each trial trains in
under a minute. Each trial runs a short proxy training (--trial_timesteps) and is scored on
rolling docking success rate; Optuna prunes bad trials early (median pruner) so the search spends
its budget on promising configs. Re-train the winning config to full length with train.py
afterwards -- this script is for finding hyperparameters, not for producing the final policy.

Usage:
    python tune.py                                # 50 trials, 150k steps each
    python tune.py --n_trials 100 --trial_timesteps 200000
    python tune.py --study_name carpet_v1 --storage sqlite:///tune.db   # resumable/parallel study
"""

import argparse
import json

import optuna
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.env_util import make_vec_env

from bicycle_env import BicycleDockEnv


class SuccessRatePruningCallback(BaseCallback):
    """Tracks rolling docking success rate and reports it to Optuna for pruning, same rolling
    window/cadence as SuccessRateCallback in train.py so proxy scores are comparable across runs."""

    def __init__(self, trial: optuna.Trial, verbose: int = 0):
        super().__init__(verbose)
        self.trial = trial
        self._results: list[float] = []
        self._last_rate = 0.0

    def _on_step(self) -> bool:
        for info in self.locals.get("infos", []):
            if "episode" in info:
                self._results.append(1.0 if info.get("docked", False) else 0.0)
                self._results = self._results[-500:]
        if self._results and self.num_timesteps % 5000 < self.training_env.num_envs:
            self._last_rate = sum(self._results) / len(self._results)
            self.trial.report(self._last_rate, step=self.num_timesteps)
            if self.trial.should_prune():
                raise optuna.TrialPruned()
        return True

    @property
    def final_rate(self) -> float:
        return self._last_rate


def objective(trial: optuna.Trial, args: argparse.Namespace) -> float:
    learning_rate = trial.suggest_float("learning_rate", 1e-5, 1e-3, log=True)
    tau = trial.suggest_float("tau", 0.003, 0.02, log=True)
    gradient_steps = trial.suggest_int("gradient_steps", 1, 4)
    train_freq = trial.suggest_int("train_freq", 1, 4)
    batch_size = trial.suggest_categorical("batch_size", [256, 512, 1024])
    net_arch_key = trial.suggest_categorical("net_arch", ["64_64", "128_128", "256_256"])
    net_arch = {"64_64": [64, 64], "128_128": [128, 128], "256_256": [256, 256]}[net_arch_key]
    # target_entropy offset from SB3's default (-action_dim); more negative -> less exploration
    # pressure, less negative -> more. Sweeping the offset (not the absolute value) keeps the
    # search space meaningful regardless of action dimensionality.
    target_entropy_offset = trial.suggest_float("target_entropy_offset", -2.0, 2.0)

    vec_env = make_vec_env(BicycleDockEnv, n_envs=args.n_envs)
    action_dim = vec_env.action_space.shape[0]
    model = SAC(
        "MlpPolicy",
        vec_env,
        learning_rate=learning_rate,
        buffer_size=300_000,
        batch_size=batch_size,
        gamma=0.98,
        tau=tau,
        train_freq=train_freq,
        gradient_steps=gradient_steps,
        learning_starts=2000,
        target_entropy=-action_dim + target_entropy_offset,
        policy_kwargs=dict(net_arch=net_arch),
        device=args.device,
        verbose=0,
    )
    callback = SuccessRatePruningCallback(trial)
    try:
        model.learn(total_timesteps=args.trial_timesteps, callback=callback, progress_bar=False)
    except optuna.TrialPruned:
        vec_env.close()
        raise
    vec_env.close()
    return callback.final_rate


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--n_trials", type=int, default=50, help="upper bound on trials; --timeout (if set) usually ends the study first")
    p.add_argument("--timeout", type=float, default=None, help="wall-clock budget in seconds -- study stops after the in-flight trial finishes")
    p.add_argument("--trial_timesteps", type=int, default=150_000)
    p.add_argument("--n_envs", type=int, default=8)
    p.add_argument("--device", type=str, default="auto")
    p.add_argument("--study_name", type=str, default="bicycle_sac")
    p.add_argument("--storage", type=str, default=None,
                    help="e.g. sqlite:///tune.db -- makes the study resumable and shareable across parallel workers")
    p.add_argument("--out", type=str, default="runs/best_hparams.json")
    args = p.parse_args()

    study = optuna.create_study(
        study_name=args.study_name,
        storage=args.storage,
        load_if_exists=args.storage is not None,
        direction="maximize",
        pruner=optuna.pruners.MedianPruner(n_startup_trials=5, n_warmup_steps=30_000),
    )
    study.optimize(lambda trial: objective(trial, args), n_trials=args.n_trials, timeout=args.timeout)

    print(f"\n[INFO] best success rate: {study.best_value:.1%}")
    print(f"[INFO] best params: {study.best_params}")
    with open(args.out, "w") as f:
        json.dump(study.best_params, f, indent=2)
    print(
        f"[INFO] saved -> {args.out}\n"
        "[INFO] none of these are train.py CLI flags -- hand-edit the SAC(...) call in "
        f"train.py with the values in {args.out}, then run train.py at full length to "
        "produce the final policy."
    )


if __name__ == "__main__":
    main()
