"""Kinematic-bicycle docking env for the JetRacer staging-pose task -- a lightweight Gymnasium
env with NO physics engine.

Why this exists: the Isaac Lab version fought a long series of *physics artifacts* (articulation
NaNs, critic divergence from unbounded actions, wheel-collapse from a chassis-scale bug, dense
reward farming) -- none of which are the docking task itself. This env integrates the kinematic
bicycle model directly (pure math), so those whole classes of problem cannot occur, and it trains
in minutes. The sim-to-real gap here is an odometry/localization *drift* problem, which no amount
of sim fidelity fixes (the policy drives whatever pose estimate it is given to zero) -- so the full
physics never bought real-world accuracy anyway. What DOES help transfer is observation
noise/bias injection (drift-tolerance) + IMU-yaw fusion on the robot; the former is built in here.

State (internal):  x, y, theta (world pose), delta (steering angle)
Observation:       [dx, dy, dheading, delta, v_prev] -- staging-pose error in the BODY frame
                   (x-forward, y-left), current steering, last commanded velocity. Matches what the
                   real robot can compute from (Nav2/AMCL pose - known staging pose).
Action (2D, [-1,1]): [steer_rate_cmd, velocity_cmd]
                   steer_rate scales to +-max_steer_rate (integrated into delta, clamped to
                   +-max_steer); velocity scales to +-max_speed (SIGNED -> reverse is free, so
                   multi-point turns emerge without any hand-written state machine).
Reward:            -(w_pos*dist + w_head*|dheading|)  (dense pose-error pull, always <= 0)
                   - w_head_near*|dheading|*near   (extra heading pull gated to near-goal -> alignment)
                   - w_steer*steer_rate^2 - w_speed_near*speed_near_goal   (smoothness / settle)
                   + success_bonus on reaching tolerance stopped (episode ends).
                   NB: the dense term is <=0 and the episode ends on success, so "hover near the
                   goal" is strictly worse than finishing -- this avoids the positive-reward
                   farming that bit the Isaac Lab dense reward.
"""

from __future__ import annotations

import math

import gymnasium as gym
import numpy as np
from gymnasium import spaces


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class BicycleDockEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array", "human"], "render_fps": 30}

    def __init__(
        self,
        wheelbase: float = 0.166,          # measured JetRacer front-to-rear axle distance
        max_steer: float = 0.349,          # +-20 deg steering limit
        max_steer_rate: float = 2.0,       # rad/s -- steering can't snap instantly (transferable)
        max_speed: float = 0.6,            # m/s (signed; negative = reverse)
        dt: float = 1.0 / 30.0,
        max_steps: int = 250,
        # spawn: staging pose at origin facing +x; robot starts in a REAR approach cone on the -x
        # side. Narrowed from the old +-90deg (a full 180deg arc, whose +-90deg extremes put the
        # robot directly beside the goal -> side entries) to +-30deg so every spawn is roughly
        # behind the bay and drives straight in. Match this to how Nav2 actually pre-aligns the
        # robot: if the real handoff pose can be more off-axis, widen this; tighter than ~30deg gets
        # brittle to handoff placement error. heading_noise stays wide so orientation is still
        # randomized even though position is constrained (cheap robustness at the handoff).
        radius_range: tuple[float, float] = (0.8, 1.8),
        arc_half_angle: float = math.pi / 6.0,   # +-30 deg rear cone (was +-90 deg)
        heading_noise: float = math.pi / 2.0,    # +-90 deg around "facing the staging pose"
        # success tolerances (tighter than the old 0.3m/15deg "match Isaac Lab" values: with the old
        # bar the policy learned to grab the success bonus right at the 0.3m edge -- eval showed it
        # docking at mean 0.287m, 9.5deg. Shrinking the goal forces it to actually centre.)
        # pos_tol is a RADIUS, so the acceptance zone diameter is 2*pos_tol. At 0.10m that's a 0.20m
        # circle -- about the JetRacer footprint itself (chassis box 0.19x0.13m, wheelbase 0.166m),
        # i.e. "the car must sit essentially on the spot", not just in a loose bay around it.
        pos_tol: float = 0.10,            # ~car-sized goal: 0.20m-diameter zone (was 0.3, then 0.15)
        heading_tol: float = 0.17,        # ~10 deg final-heading alignment (was ~15 deg)
        speed_tol: float = 0.08,          # must be nearly stopped to count as docked
        # reward weights
        w_pos: float = 1.0,
        w_head: float = 0.5,              # gentle GLOBAL heading pull (roughly face the staging heading)
        w_head_near: float = 1.5,         # STRONG heading pull gated to near-goal only -> final alignment
        w_steer: float = 0.02,
        w_speed_near: float = 0.3,        # penalize speed near the goal (encourages settling)
        success_bonus: float = 50.0,
        time_penalty: float = 0.01,
        # domain randomization for drift-tolerance (the real value of RL for hazy odom)
        obs_noise_std: float = 0.0,       # per-step Gaussian noise on observed (dx,dy,dheading)
        obs_bias_std: float = 0.0,        # per-EPISODE constant bias on observed pose (simulates drift)
        render_mode: str | None = None,
    ):
        super().__init__()
        self.L = wheelbase
        self.max_steer = max_steer
        self.max_steer_rate = max_steer_rate
        self.max_speed = max_speed
        self.dt = dt
        self.max_steps = max_steps
        self.radius_range = radius_range
        self.arc_half_angle = arc_half_angle
        self.heading_noise = heading_noise
        self.pos_tol = pos_tol
        self.heading_tol = heading_tol
        self.speed_tol = speed_tol
        self.w_pos = w_pos
        self.w_head = w_head
        self.w_head_near = w_head_near
        self.w_steer = w_steer
        self.w_speed_near = w_speed_near
        self.success_bonus = success_bonus
        self.time_penalty = time_penalty
        self.obs_noise_std = obs_noise_std
        self.obs_bias_std = obs_bias_std
        self.render_mode = render_mode

        # staging pose (goal): origin, facing +x
        self.goal = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        # obs: dx, dy, dheading, delta, v_prev
        high = np.array([5.0, 5.0, math.pi, max_steer, max_speed], dtype=np.float32)
        self.observation_space = spaces.Box(low=-high, high=high, dtype=np.float32)

        self._traj: list[tuple[float, float]] = []  # for rendering

    # ------------------------------------------------------------------ helpers
    def _obs(self) -> np.ndarray:
        gx, gy, gth = self.goal
        dxw, dyw = gx - self.x, gy - self.y
        c, s = math.cos(self.theta), math.sin(self.theta)
        dx = c * dxw + s * dyw
        dy = -s * dxw + c * dyw
        dheading = _wrap(gth - self.theta)
        obs = np.array([dx, dy, dheading, self.delta, self.v_prev], dtype=np.float64)
        # drift/noise injection on the *observed* pose error (not the true state)
        obs[:3] += self._bias
        if self.obs_noise_std > 0.0:
            obs[:3] += self.np_random.normal(0.0, self.obs_noise_std, size=3)
        return obs.astype(np.float32)

    def _dist_heading(self) -> tuple[float, float]:
        dxw, dyw = self.goal[0] - self.x, self.goal[1] - self.y
        return math.hypot(dxw, dyw), abs(_wrap(self.goal[2] - self.theta))

    # ------------------------------------------------------------------ gym API
    def reset(self, *, seed: int | None = None, options=None):
        super().reset(seed=seed)
        # spawn in the front 180deg approach arc on the -x side of the goal
        r = self.np_random.uniform(*self.radius_range)
        approach = math.pi  # arc opens toward -x
        phi = approach + self.np_random.uniform(-self.arc_half_angle, self.arc_half_angle)
        self.x = self.goal[0] + r * math.cos(phi)
        self.y = self.goal[1] + r * math.sin(phi)
        # heading roughly toward the goal, +- heading_noise
        to_goal = math.atan2(self.goal[1] - self.y, self.goal[0] - self.x)
        self.theta = _wrap(to_goal + self.np_random.uniform(-self.heading_noise, self.heading_noise))
        self.delta = 0.0
        self.v_prev = 0.0
        self.steps = 0
        # per-episode constant observation bias (simulates accumulated odom drift for this run)
        self._bias = (
            self.np_random.normal(0.0, self.obs_bias_std, size=3) if self.obs_bias_std > 0.0
            else np.zeros(3)
        )
        self._traj = [(self.x, self.y)]
        return self._obs(), {}

    def step(self, action: np.ndarray):
        a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        steer_rate = a[0] * self.max_steer_rate
        v = a[1] * self.max_speed

        # integrate kinematic bicycle
        self.delta = float(np.clip(self.delta + steer_rate * self.dt, -self.max_steer, self.max_steer))
        self.theta = _wrap(self.theta + (v / self.L) * math.tan(self.delta) * self.dt)
        self.x += v * math.cos(self.theta) * self.dt
        self.y += v * math.sin(self.theta) * self.dt
        self.v_prev = v
        self.steps += 1
        self._traj.append((self.x, self.y))

        dist, head_err = self._dist_heading()
        speed = abs(v)

        # reward: dense pose-error pull (<=0), smoothness, settle-near-goal, time
        near = math.exp(-((dist / 0.6) ** 2))  # proximity gate: ~1 at the goal, decays with distance
        reward = -(self.w_pos * dist + self.w_head * head_err)
        # Extra heading pull that only bites near the goal. Far away the robot should point where
        # it's driving (the gentle global w_head is enough); once it's on the spot, matching the
        # staging heading tightly is what matters -- gating by `near` stops this from fighting the
        # approach and targets exactly the final-alignment error the old reward left loose (~9.5deg).
        reward -= self.w_head_near * head_err * near
        reward -= self.w_steer * (steer_rate ** 2)
        reward -= self.w_speed_near * speed * near
        reward -= self.time_penalty

        docked = (dist < self.pos_tol) and (head_err < self.heading_tol) and (speed < self.speed_tol)
        terminated = False
        if docked:
            reward += self.success_bonus
            terminated = True

        truncated = self.steps >= self.max_steps
        # leaving the arena is a hard fail (keeps episodes on-task)
        if dist > (self.radius_range[1] + 1.0):
            truncated = True

        info = {"dist": dist, "heading_err": head_err, "docked": docked}
        return self._obs(), float(reward), terminated, truncated, info

    def render(self):
        # lightweight matplotlib rgb_array render (no physics viewer needed)
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(5, 5), dpi=80)
        ax.set_xlim(-2.2, 2.2)
        ax.set_ylim(-2.2, 2.2)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        # goal + tolerance circle + heading arrow
        ax.add_patch(plt.Circle((self.goal[0], self.goal[1]), self.pos_tol, color="gold", fill=False, lw=2))
        ax.arrow(self.goal[0], self.goal[1], 0.25 * math.cos(self.goal[2]), 0.25 * math.sin(self.goal[2]),
                 head_width=0.08, color="green")
        # trajectory
        if len(self._traj) > 1:
            tx, ty = zip(*self._traj)
            ax.plot(tx, ty, "-", color="tab:blue", lw=1, alpha=0.6)
        # robot as a triangle
        c, s = math.cos(self.theta), math.sin(self.theta)
        pts = np.array([[0.12, 0], [-0.06, 0.06], [-0.06, -0.06]])
        pts = pts @ np.array([[c, s], [-s, c]]) + np.array([self.x, self.y])
        ax.add_patch(plt.Polygon(pts, color="tab:red"))
        fig.canvas.draw()
        buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
        img = buf.reshape(fig.canvas.get_width_height()[::-1] + (4,))[..., :3].copy()
        plt.close(fig)
        return img
