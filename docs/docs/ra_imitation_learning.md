# RA Use Case — Imitation Learning (LeRobot)

An alternative to the scripted MTC pick-and-place pipeline: teach the SO-ARM 101
a **pick → hold → place** skill by *demonstration*. A human drives a **leader**
arm, the **follower** arm mirrors it, and every frame (joint states + camera
images) is recorded into a [LeRobot](https://github.com/huggingface/lerobot)
dataset. A policy (ACT / diffusion / etc.) is then trained by behaviour cloning
and replayed on the follower.

This is **separate** from the ROS 2 / MoveIt stack in `ra_ws` — it runs entirely
through LeRobot on the physical SO-101 leader+follower pair. Nothing here depends
on Isaac Sim or `move_group`.

!!! note "Handover status"
    A **trained ACT policy exists and is converged** — see [Trained
    model](#the-trained-model) below. The **recorded dataset** is the primary
    reusable asset (you can retrain any policy from it). Everything needed to
    re-record, retrain, deploy, or replay is on this page. The LeRobot library
    itself is an upstream install (`~/lerobot`, editable) and is **not** vendored
    into this repo.

## Environment

| Item | Value |
|------|-------|
| LeRobot version | **0.3.4** |
| Conda env | `lerobot` (`conda activate lerobot`) |
| Python | via `~/miniconda3/envs/lerobot` |
| Extras | `ffmpeg` (video encode/decode), CUDA-matched PyTorch for training |

!!! warning "GPU / PyTorch for training"
    Training needs a PyTorch build matching the GPU. The model here was trained on
    an **RTX 4060 Laptop (8 GB)** with **torch 2.7.1 + CUDA 12.6 (cu126)** — verify
    `torch.cuda.is_available()` is `True` before a long run, or it silently falls
    back to CPU. On a newer card (e.g. RTX 50-series) use the matching cu128
    wheels. `ffmpeg` must be on `PATH` for video decoding.

## Hardware

Two SO-101 arms on USB serial:

| Role | Device | LeRobot `id` | Type |
|------|--------|--------------|------|
| Follower (the robot that acts) | `/dev/ttyACM0` | `my_follower` | `so101_follower` |
| Leader (the one you hand-drive) | `/dev/ttyACM1` | `my_leader` | `so101_leader` |

Ports are not guaranteed stable across reboots — confirm with:

```bash
conda activate lerobot
lerobot-find-port          # unplug/replug to identify each arm
```

## Calibration

Per-arm motor calibration lives in the LeRobot cache (**not** in this repo):

```
~/.cache/huggingface/lerobot/calibration/
├── robots/so101_follower/my_follower.json
└── teleoperators/so101_leader/my_leader.json
```

Each JSON holds homing offset + range for the six joints
(`shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper`).
Recreate if lost:

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=my_follower
lerobot-calibrate --teleop.type=so101_leader  --teleop.port=/dev/ttyACM1 --teleop.id=my_leader
```

## The dataset

**`weeho/so101_pick_hold_place`** — the pick/hold/place demonstrations.

| Property | Value |
|----------|-------|
| Location | `~/.cache/huggingface/lerobot/weeho/so101_pick_hold_place` |
| Episodes | 48 |
| Frames | 28,817 |
| FPS | 30 |
| Robot type | `so101_follower` |
| Action / state | 6-DOF: `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper` (`.pos`) |
| Cameras | 2 (96 videos = 48 × 2) |
| Tasks | 1 |
| Dataset format | LeRobot codebase **v2.1** |

Layout: `meta/` (info/episodes/tasks jsonl) · `data/chunk-000/episode_*.parquet`
· `videos/chunk-000/<cam>/episode_*.mp4`. A `.bak` sibling copy exists.

!!! danger "The dataset is NOT in git"
    It's 1.6 GB of parquet + mp4 and lives only in the HF cache above. To hand it
    over, do **one** of:

    - **Push to the Hub:** `huggingface-cli login` then
      `lerobot-record … --dataset.push_to_hub=true` (or push after the fact) — the
      recipient pulls with `--dataset.repo_id=weeho/so101_pick_hold_place`.
    - **Copy the cache dir** `~/.cache/huggingface/lerobot/weeho/so101_pick_hold_place`
      (and the `calibration/` dir) to the new machine.
    - **Tarball:** `tar czf so101_pick_hold_place.tar.gz -C ~/.cache/huggingface/lerobot weeho/so101_pick_hold_place`.

    On the new machine the dataset **plus** a LeRobot install, GPU-matched
    PyTorch, and `ffmpeg` are all required before training will run.

## Workflow

All commands run inside `conda activate lerobot`. Exact flag names evolve between
LeRobot releases — check `--help` on the pinned 0.3.4 install if one is rejected.

### 1. Teleoperate (sanity check)

Follower should mirror the leader with no recording:

```bash
lerobot-teleoperate \
  --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=my_follower \
  --teleop.type=so101_leader  --teleop.port=/dev/ttyACM1 --teleop.id=my_leader
```

### 2. Record demonstrations

Adds cameras + a dataset sink to the same teleop loop:

```bash
lerobot-record \
  --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=my_follower \
  --teleop.type=so101_leader  --teleop.port=/dev/ttyACM1 --teleop.id=my_leader \
  --dataset.repo_id=weeho/so101_pick_hold_place \
  --dataset.single_task="Pick the cup, hold it, then place it" \
  --dataset.num_episodes=48 --dataset.fps=30
```

(Camera keys are declared via `--robot.cameras=…`; use `lerobot-find-cameras`
to enumerate connected devices.)

### 3. Train a policy

The command used for the shipped model (ACT, ~0.35 s/step on the RTX 4060):

```bash
lerobot-train \
  --dataset.repo_id=weeho/so101_pick_hold_place \
  --policy.type=act \
  --policy.device=cuda \
  --policy.push_to_hub=false \
  --output_dir=outputs/train/so101_pick_hold_place_act_100k \
  --steps=100000 --batch_size=8 --save_freq=50000
```

- `--policy.push_to_hub=false` is **required** unless you also pass
  `--policy.repo_id=…` — otherwise training aborts immediately at config validation.
- ACT here **converged early** (L1 loss flat at ~0.05 from ~step 50k on); the 50k
  checkpoint was as good as later ones.

!!! danger "Dataloader deadlock under disk pressure"
    A run once **hung** (process alive, GPU at 0%, no step progress) when the disk
    got near-full — a dataloader worker doing video decode stalled and the main
    loop blocked on it forever, and it does **not** recover on its own. Guard against it:

    - Keep several GB of disk headroom (checkpoints are ~0.6 GB each with optimizer state).
    - Data loading is not the bottleneck here (`data_s ≈ 0`), so **`--num_workers=0`**
      is a safe way to remove the multiprocessing deadlock at ~no speed cost.

## The trained model

A converged **ACT** policy is saved at:

```
outputs/train/so101_pick_hold_place_act_100k/checkpoints/last/pretrained_model/
├── model.safetensors     # ~198 MB — the policy weights
├── config.json           # policy + feature spec
└── train_config.json
```

| Property | Value |
|----------|-------|
| Policy | ACT |
| Trained steps | 50,000 (final L1 loss ~0.05) |
| **Inputs** | `observation.state` (6 joints) + **2 cameras**: `observation.images.wrist`, `observation.images.top` (each 640×480 @ 30 fps) |
| **Output** | `action` — 6 joint position targets |

> The optimizer/resume state (`training_state/`) was deleted to save disk — the
> model runs for inference without it, but you **cannot resume training** from this
> checkpoint. Retrain from the dataset if you need to continue.

### 4. Replay a recorded episode

Open-loop playback of a demo on the follower (sanity check, no policy):

```bash
lerobot-replay --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=my_follower \
  --dataset.repo_id=weeho/so101_pick_hold_place --dataset.episode=0
```

### 5. Deploy the trained policy on the real arm

Running a policy on real hardware is done with **`lerobot-record --policy.path=…`**
(the policy replaces the leader/teleop and drives the follower; `lerobot-eval` is
for *simulated* gym environments, not a physical arm). **No leader needed.**

```bash
lerobot-record \
  --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=my_follower \
  --robot.cameras='{ wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30},
                     top:   {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30} }' \
  --policy.path=outputs/train/so101_pick_hold_place_act_100k/checkpoints/last/pretrained_model \
  --dataset.repo_id=weeho/eval_so101_pick_hold_place \
  --dataset.single_task="Pick the cup, hold it, then place it" \
  --dataset.num_episodes=3 --dataset.episode_time_s=30 --dataset.push_to_hub=false
```

Replace the camera `index_or_path` values with the real ones from
`lerobot-find-cameras`.

!!! danger "What makes or breaks deployment"
    - **Camera names must be exactly `wrist` and `top`** — that's how the policy
      maps its two image inputs (`observation.images.wrist` / `.top`). Wrong names
      = it won't run.
    - **Viewpoints must match recording** — wrist cam on the gripper, top cam
      overhead. The policy learned those exact views; move a camera and it fails.
    - **Same calibration** (`my_follower.json`) and **30 fps** as training.
    - The arm moves **autonomously** on start — keep a hand on the e-stop, clear
      the workspace, and place the cup near where the demos had it.
    - **Generalization is narrow** — 48 demos of one cup/task; expect success only
      near the demonstrated conditions.

## Relationship to the ROS 2 stack

| | Scripted (MTC) | Imitation learning (LeRobot) |
|---|---|---|
| Where | `ra_ws` (ROS 2 Jazzy, MoveIt) | LeRobot, no ROS |
| Perception | YOLO + ray-plane, AprilTag | Raw camera frames into the policy |
| Control | IK + planned trajectories | Learned action = joint targets |
| Strength | Deterministic, inspectable | Handles variation without hand-tuning |

The two are independent skill implementations of the same **pick-and-place**
task and do not share a runtime. See the [Pick and Place](ra_pick_and_place.md)
page for the scripted pipeline.
